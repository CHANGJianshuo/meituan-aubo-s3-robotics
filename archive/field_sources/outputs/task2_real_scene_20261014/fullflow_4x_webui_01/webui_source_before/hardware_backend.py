"""Single-use, observed hardware trajectory. No Mock execution or motion retries."""
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import threading
import time

from mtc_webui.mock_backend import ApiError
from mtc_webui.ros_backend import RosBackend, ACTIVE
from mtc_webui.file_preview import FilePreviewBackend

STAGES = ['接近把手', '水平插入', '试提起', '垂直提起', '搬运到 T0',
          '预放置', '接触支撑面', '卸载', '退出把手', '撤离']


@dataclass(frozen=True)
class HardwareConfig:
    read_only: bool = False
    freshness_ms: int = 3000
    backend: str = 'guarded_aubo_trajectory'


class HardwareBackend(RosBackend):
    mode = 'hardware_trajectory'
    scope_note = 'AUBO 实机轨迹服务；仅执行已准备的红电池 → T0 单次任务，不自动恢复或重发'
    _read_preview = FilePreviewBackend._read_preview

    def __init__(self, settings, output_dir, **kwargs):
        self.settings = deepcopy(settings)
        self.package = Path(settings['package'])
        self.manifest = json.loads((self.package/'manifest.json').read_text())
        self.bundle = json.loads((self.package/'full_trajectory.json').read_text())
        self.preview_dir = Path(settings['preview_dir'])
        self.preview_stamps = {}
        self.claim = Path(settings['claim_file'])
        self.preflight = Path(settings['preflight_file'])
        self.last_state = None
        self.last_state_received = 0
        self.stationary_since = None
        self.finished_stages = []
        self.stage_count = len(self.bundle['segments'])
        self.stage_offset = settings.get('stage_offset',0)
        self.completed_received = None
        super().__init__(HardwareConfig(), output_dir, **kwargs)
        self.connection.update(reason='等待 NUC 实机只读连接')
        (self.output_dir/'hardware-config.json').write_text(json.dumps(settings, indent=2)+'\n')
        if settings.get('previous_task_record'):
            previous = json.loads(Path(settings['previous_task_record']).read_text())
            previous.update(_start=self.clock()-previous.get('elapsed_ms',0)/1000,_ended=self.clock(),owned=False)
            self.tasks[previous['task_id']] = previous
        if self.claim.exists():
            claim = json.loads(self.claim.read_text())
            old = Path(claim.get('output_dir','/nonexistent'))/(claim.get('task_id','missing')+'.json')
            if old.is_file():
                task = json.loads(old.read_text())
                elapsed = task.get('elapsed_ms', 0)/1000
                task.update(_start=self.clock()-elapsed, _ended=self.clock(), owned=False)
                if task['status'] in ACTIVE:
                    task.update(status='UNKNOWN', detail='上次服务退出时任务尚无可靠终态；不自动重试')
                self.tasks[task['task_id']] = task
                self.current_id = task['task_id']
                self._event('RECOVERED_RECORD', '恢复上次执行记录；未提交任何运动', task)

    def connection_update(self, **fields):
        with self.lock:
            old = (self.connection['connected'], self.connection['reason'])
            self.connection.update(fields)
            if old != (self.connection['connected'], self.connection['reason']):
                self._event('HARDWARE_CONNECTION', self.connection['reason'])

    def evidence(self):
        try:
            data = json.loads(self.preflight.read_text())
            if (data.get('plan_id') == self.manifest['plan_id'] and data.get('passed') is True
                    and data.get('expires_unix', 0) > self.wall_clock()
                    and data.get('manifest_sha256') == self.settings['manifest_sha256']):
                return data
        except (OSError, ValueError):
            pass
        return None

    def scene_matches(self, evidence):
        # Recheck the saved reviewed scene at submit time. No inference of grasp
        # success from colored pixels; this only detects a changed starting scene.
        try:
            import cv2
            import numpy as np
            reference = cv2.imread(evidence['reference_image'])
            image = cv2.imdecode(np.frombuffer(self.frames['color'], dtype=np.uint8), cv2.IMREAD_COLOR)
            if reference is None or image is None or reference.shape != image.shape:
                return False
            checks = evidence.get('scene_checks')
            if checks is None:
                checks = [dict(scene_roi=evidence['scene_roi'],
                    checked_color_indices=evidence.get('checked_color_indices',[0,1,2,3]))]
            if not isinstance(checks, list) or not checks:
                return False
            def masks(a, roi):
                x0,y0,x1,y1 = roi
                h = cv2.cvtColor(a[y0:y1,x0:x1],cv2.COLOR_BGR2HSV)
                common = (h[:,:,1]>100)&(h[:,:,2]>55)
                return [common&((h[:,:,0]<10)|(h[:,:,0]>170)),
                    common&(h[:,:,0]>=15)&(h[:,:,0]<=40),
                    common&(h[:,:,0]>40)&(h[:,:,0]<=80),
                    common&(h[:,:,0]>=85)&(h[:,:,0]<=120)]
            for check in checks:
                roi = check['scene_roi']
                if len(roi) != 4 or any(type(v) is not int for v in roi):
                    return False
                x0,y0,x1,y1 = roi
                if not (0 <= x0 < x1 <= image.shape[1] and 0 <= y0 < y1 <= image.shape[0]):
                    return False
                indices = check['checked_color_indices']
                if not indices or any(type(i) is not int or i not in range(4) for i in indices):
                    return False
                mask_pairs=list(zip(masks(reference,roi),masks(image,roi)))
                for index in indices:
                    a,b=mask_pairs[index]
                    if min(a.sum(),b.sum()) < 300 or (a&b).sum()/(a|b).sum() < .86:
                        return False
                    ya,xa=np.nonzero(a);yb,xb=np.nonzero(b)
                    if np.linalg.norm([xa.mean()-xb.mean(),ya.mean()-yb.mean()])>2.0:
                        return False
            return True
        except Exception:
            return False

    def _reasons(self, include_active=True):
        reasons = []
        if not self.connection['connected']:
            reasons.append(self.connection['reason'])
        sample = self._sample('robot')
        if not sample.get('fresh'):
            reasons.append('实机关节 / 停稳反馈缺失或过期')
        elif not sample['data']['robot_ready'] or not sample['data']['stop_confirmed']:
            reasons.append('机械臂未就绪或尚未停稳')
        if self.last_state and max(abs(a-b) for a,b in zip(self.last_state['q'], self.bundle['segments'][0]['points'][0]['positions'])) > math.radians(.05):
            reasons.append('当前姿态不在已验证轨迹起点 0.05° 容差内')
        evidence = self.evidence()
        if not evidence:
            reasons.append('等待本次电池、邻近障碍物、V1 安装和轨迹衔接核验')
        elif not self.scene_matches(evidence):
            reasons.append('电池区图像与本次核验画面不一致；需要重新核对')
        if not self._sample('color').get('fresh'):
            reasons.append('现场相机画面缺失或过期')
        if self.claim.exists():
            reasons.append('本次单次执行凭据已使用；不会自动重放')
        task = self.tasks.get(self.current_id)
        if include_active and task and task['status'] in ACTIVE:
            reasons.append('当前任务未结束或结果未知')
        return reasons

    @staticmethod
    def validate(payload):
        RosBackend.validate(payload)
        if payload['mode'] != 1 or payload['ordered_colors'] != [1]:
            raise ApiError(400, 'UNSUPPORTED_HARDWARE_TASK', '本实机入口仅支持已验证的红电池 → T0')
        if payload['timeout_ms'] < 120000:
            raise ApiError(400, 'TIMEOUT_TOO_SHORT', '此轨迹的任务超时不得少于 120 秒')

    def submit(self, payload):
        with self.lock:
            result = super().submit(payload)
            task = self.tasks[self.current_id]
            if task['status'] == 'SUBMITTING':
                task['detail'] = '请求已登记，正在核对 NUC 播放器与实机状态'
                self._persist(task)
            return result

    def observe_state(self, state, stamp, source_age_ms=None):
        with self.lock:
            age = (self.wall_clock()-stamp)*1000 if source_age_ms is None else source_age_ms
            if age > 2500 or age < -500:
                self.connection_update(connected=False, reason='NUC 反馈或往返心跳过期')
                return
            self.last_state = deepcopy(state)
            self.last_state_received = self.clock()
            still = state['exec_id'] == -1 and state['queue'] == 0 and max(map(abs, state['qd'])) <= math.radians(.02)
            self.stationary_since = (self.stationary_since or self.clock()) if still else None
            ready = state['robot_mode'] == 'Running' and state['safety_mode'] in ('Normal', 'ReducedMode') and state['power_on'] and state['within_limits'] and state['runtime'] == 'Stopped' and not any(state[k] for k in ['simulation','freedrive','servo','collision','force_control','soft_float','backdrive'])
            self.connection_update(connected=True, reason='NUC → AUBO SDK 实时只读反馈', action_ready=True)
            source = 'AUBO SDK / NUC 192.0.2.10 / rob1'
            self.observe('robot', dict(communication_ok=True, robot_ready=bool(ready),
                motion_active=not still, stop_confirmed=bool(still and self.clock()-self.stationary_since >= 1),
                safety_normal=state['safety_mode'] in ('Normal','ReducedMode'),
                controller_mode=state['robot_mode']+' / '+state['safety_mode'],
                simulation_mode=state['simulation'], q=state['q'], qd=state['qd'], tcp=state['tcp'],
                exec_id=state['exec_id'], queue=state['queue']), source, int(stamp*1e9), age)
            self.observe('joints', dict(name=['J1','J2','J3','J4','J5','J6'], position=state['q']), source, int(stamp*1e9), age)
            # This is inspection evidence for the passive tool's installation,
            # not a fabricated attachment sensor.
            self.observe('tool', dict(verified=bool(self.evidence()), state=0, evidence_level=2,
                detail='原 V1 44 mm，安装由本次现场图像核验；无附着传感器'),
                '本次 V1 安装核验（非抓取传感器）', int(stamp*1e9), age)
            task = self.tasks.get(self.current_id)
            self.observe('task_state', dict(task_id=task['task_id'] if task else '',
                state=task['status'] if task else 'WAIT_TASK', substate=task['substate'] if task else '',
                completed_count=task['completed_count'] if task else 0, total_count=1,
                last_error_code=task['last_error_code'] if task else 0,
                detail=task['detail'] if task else '等待红电池 → T0 单次任务'),
                '实机执行台账', int(stamp*1e9), age)

    def hardware_event(self, goal_id, event):
        with self.lock:
            task = self._owned(goal_id)
            if not task or task['status'] not in ACTIVE:
                return
            kind = event['event']
            if kind == 'attempt_reserved':
                task.update(status='ACCEPTED', detail='NUC 已取得机械臂独占锁并登记单次执行')
                self._event('GOAL_ACCEPTED', task['detail'], task)
            elif kind == 'move_intent':
                index = event['stage']
                task.update(status='EXECUTING', state='EXECUTING', substate=event['primitive'],
                    stage_index=index, current_slot='T0', current_object_id='red-battery',
                    detail=f'实机第 {index+self.stage_offset}/10 段：{STAGES[index+self.stage_offset-1]}')
                self._event('FEEDBACK', task['detail'], task)
            elif kind == 'stage_complete':
                if event['stage'] != len(self.finished_stages)+1:
                    self.uncertain(goal_id, '实机阶段回执次序异常；禁止重试')
                    self.transport.stop_owned()
                    return
                self.finished_stages.append(event['stage'])
                task['stages_completed'] = len(self.finished_stages)
                self._event('STAGE_COMPLETE', f'第 {event["stage"]+self.stage_offset}/10 段终点及停稳已确认', task)
            elif kind == 'completed':
                if event.get('plan_id') != self.manifest['plan_id']:
                    self.uncertain(goal_id, '完成回执 plan_id 不匹配')
                    return
                self.completed_received = self.clock()
                task.update(status='EXECUTING', state='VERIFYING', detail=f'本次 {self.stage_count} 段轨迹反馈已返回，正在独立复核末姿态与停稳')
                self._event('VERIFYING', task['detail'], task)
            elif kind in ('bridge_error', 'stop_response', 'stop_feedback', 'stop_unverified', 'owned_stop_requested'):
                if kind == 'bridge_error':
                    task['execution_error'] = event.get('error', '未知播放器异常')
                if kind != 'stop_feedback':
                    self._event(kind.upper(), json.dumps(event, ensure_ascii=False), task, 'warning')

    def cancel(self, task_id, goal_id):
        with self.lock:
            task = self._owned(goal_id)
            if not task or task['task_id'] != task_id or task['status'] not in ACTIVE or not self.transport.running:
                raise ApiError(409, 'NOT_CANCELABLE', '没有本会话正在运行的实机任务')
            if task['status'] == 'CANCELING':
                raise ApiError(409, 'ALREADY_CANCELING', '停止请求已经发出，不会重复发送')
            task.update(status='CANCELING', detail='已请求中断本次轨迹；等待控制柜停稳反馈')
            self._event('CANCEL_REQUESTED', task['detail'], task)
            delivered = self.transport.stop_owned()
            task['cancel_ack'] = dict(request_delivered=delivered, message='已写入本次 SSH 控制通道' if delivered else '发送结果未知，请使用现场停止装置')
            self._persist(task)
            return deepcopy(task['cancel_ack'])

    def request_stop(self):
        with self.lock:
            task = self.tasks.get(self.current_id)
            if not task:
                raise ApiError(409, 'STOP_UNAVAILABLE', '本入口没有活动实机轨迹')
            reply = self.cancel(task['task_id'], task['goal_id'])
            self.stop_request = dict(status='PENDING', **reply)
            return deepcopy(self.stop_request)

    def snapshot(self):
        with self.lock:
            self._read_preview('color', 'latest_color.jpg')
            self._read_preview('depth', 'latest_depth_preview.jpg')
            value = super().snapshot()
            value['config'] = dict(backend='AUBO SDK / hash-pinned guarded player',
                read_only=False, plan_id=self.manifest['plan_id'], trajectory='half_time_package' + (f' / 停稳后继续剩余 {self.stage_count} 段' if self.stage_offset else ''),
                source_sha256=self.settings['source_trajectory_sha256'], nuc='192.0.2.10',
                supported_task='红电池 → T0', automatic_retry=False, single_use=True)
            running = bool(self.transport and self.transport.running)
            value['operations'].update(cancel=running and not self.transport.stop_sent, stop=running and not self.transport.stop_sent)
            value['local_preview'] = dict(directory=str(self.preview_dir), role='现场 RGB / 深度预览')
            return value


class HardwareTransport:
    def __init__(self, backend, credentials):
        self.backend = backend
        backend.transport = self
        self.credentials = credentials
        self.quit = threading.Event()
        self.process_lock = threading.Lock()
        self.execution = None
        self.running = False
        self.stop_sent = False
        self.workers = []

    def _spawn(self, command):
        s = self.backend.settings
        remote = [s['remote_python'], s['remote_bridge'], command, '--root', s['remote_package'],
                  '--manifest-sha256', s['manifest_sha256']]
        child = subprocess.Popen(['ssh', '-T', '-o','BatchMode=yes','-o','ConnectTimeout=8',
            '-o','ServerAliveInterval=2','-o','ServerAliveCountMax=3',
            '-o','HostKeyAlias=192.0.2.11','-o','StrictHostKeyChecking=yes',
            'robot@192.0.2.10', shlex.join(remote)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True, bufsize=1)
        child.stdin.write(json.dumps(self.credentials)+'\n'); child.stdin.flush()
        def heartbeat():
            while child.poll() is None and not self.quit.wait(.75):
                try:
                    with self.process_lock:
                        child.stdin.write(json.dumps(dict(command='ping', token=time.monotonic()))+'\n'); child.stdin.flush()
                except (OSError, ValueError):
                    return
            if self.quit.is_set() and child.poll() is None:
                try:
                    with self.process_lock:
                        child.stdin.write('{"command":"stop"}\n'); child.stdin.flush()
                except (OSError, ValueError):
                    pass
        threading.Thread(target=heartbeat, daemon=True).start()
        return child

    def start(self):
        if not self.credentials:
            self.backend.connection_update(connected=False, reason='未加载 SDK 凭据；从终端显式启动认证会话，不会自动执行')
            return
        t = threading.Thread(target=self._state_loop, daemon=True)
        self.workers.append(t); t.start()

    def _state_loop(self):
        while not self.quit.is_set():
            try:
                child = self._spawn('state')
                for line in child.stdout:
                    if not line.startswith('MTC_HARDWARE '):
                        continue
                    event = json.loads(line[len('MTC_HARDWARE '):])
                    if event['event'] == 'state' and isinstance(event.get('heartbeat_token'), (int,float)):
                        self.backend.observe_state(event['state'], event['bridge_unix'],
                            (time.monotonic()-event['heartbeat_token'])*1000)
                child.wait(timeout=5)
            except Exception as error:
                self.backend.connection_update(connected=False, reason='实机只读连接异常：'+str(error))
            self.backend.connection_update(connected=False, reason='实机只读反馈已断开；等待重新读取，不会重发轨迹')
            self.quit.wait(2)

    def enqueue(self, kind, goal_id, payload):
        assert kind == 'submit'
        # The durable local claim survives HTTP retries and service restarts.
        try:
            with self.backend.claim.open('x') as file:
                json.dump(dict(plan_id=self.backend.manifest['plan_id'], goal_id=goal_id,
                    task_id=payload['task_id'], output_dir=str(self.backend.output_dir),
                    status='claimed_no_retry', created_unix=time.time()), file)
                file.flush(); os.fsync(file.fileno())
        except FileExistsError:
            self.backend.accepted(goal_id, False, '单次执行凭据已使用；禁止重放')
            return
        self.running = True
        self.stop_sent = False
        worker = threading.Thread(target=self._execute, args=(goal_id,payload), daemon=True)
        self.workers.append(worker); worker.start()

    def stop_owned(self):
        with self.process_lock:
            if self.stop_sent:
                return True
            self.stop_sent = True
            if not self.execution or self.execution.poll() is not None:
                return False
            try:
                self.execution.stdin.write('{"command":"stop"}\n'); self.execution.stdin.flush()
                return True
            except (OSError, ValueError):
                return False

    def _record_camera(self):
        import csv
        import cv2
        import numpy as np
        b=self.backend
        start=time.monotonic(); count=0; writer=None
        try:
            with (b.output_dir/'camera-times.csv').open('x') as f:
                table=csv.writer(f); table.writerow(['frame','elapsed_s','source_stamp_ns'])
                while self.running:
                    with b.lock:
                        b._read_preview('color','latest_color.jpg')
                        jpeg=b.frames.get('color'); sample=b._sample('color')
                    if jpeg and sample.get('fresh'):
                        frame=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
                        if writer is None:
                            writer=cv2.VideoWriter(str(b.output_dir/'physical_observation.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),5,(frame.shape[1],frame.shape[0]))
                            if not writer.isOpened(): raise RuntimeError('Camera recorder failed')
                            cv2.imwrite(str(b.output_dir/'physical_before.png'),frame)
                        writer.write(frame)
                        table.writerow([count,time.monotonic()-start,sample['source_stamp_ns']]);f.flush();count+=1
                    time.sleep(.2)
        except Exception as error:
            with b.lock: b._event('RECORDING_ERROR',str(error),level='warning')
        finally:
            if writer: writer.release()
            (b.output_dir/'recording.json').write_text(json.dumps(dict(frames=count,video_fps=5,source_preview_rate_hz=1,
                method='Existing preview held between source updates; exact source stamps in camera-times.csv')))

    def _verify_placement(self):
        import cv2
        import numpy as np
        b=self.backend
        reference=cv2.imread(b.settings['placement_reference'])
        def red_mask(image):
            h=cv2.cvtColor(image[440:565,460:585],cv2.COLOR_BGR2HSV)
            return ((h[:,:,0]<10)|(h[:,:,0]>170))&(h[:,:,1]>100)&(h[:,:,2]>55)
        expected=red_mask(reference)
        rows=[];seen=set();until=time.monotonic()+10
        while time.monotonic()<until:
            with b.lock:
                b._read_preview('color','latest_color.jpg');sample=b._sample('color');jpeg=b.frames.get('color')
            if jpeg and sample.get('fresh') and sample['source_stamp_ns'] not in seen:
                stamp=sample['source_stamp_ns'];seen.add(stamp)
                frame=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
                mask=red_mask(frame); y,x=np.nonzero(mask); yr,xr=np.nonzero(expected)
                if len(x)<2000: return dict(passed=False,reason='T0 内没有足够的红色电池轮廓',frames=rows)
                iou=float((mask&expected).sum()/(mask|expected).sum())
                center=np.array([x.mean(),y.mean()]);distance=float(np.linalg.norm(center-[xr.mean(),yr.mean()]))
                rows.append(dict(source_stamp_ns=stamp,center_px=center.tolist(),iou=iou,reference_delta_px=distance))
                if iou<.80 or distance>3.0: return dict(passed=False,reason='T0 轮廓与已成功放置参考不一致',frames=rows)
                if len(rows)>=4 and (stamp-rows[0]['source_stamp_ns'])/1e9>=3:
                    centers=np.array([r['center_px'] for r in rows]);drift=float(np.max(np.linalg.norm(centers-centers.mean(axis=0),axis=1)))
                    cv2.imwrite(str(b.output_dir/'physical_final.png'),frame)
                    return dict(passed=drift<.75,source='Existing physical color camera',reference=b.settings['placement_reference'],
                        stable_interval_s=(stamp-rows[0]['source_stamp_ns'])/1e9,max_centroid_deviation_px=drift,frames=rows,
                        interpretation='Observed upright red silhouette at the previously successful T0 location; no force or attachment sensor claim')
            time.sleep(.1)
        return dict(passed=False,reason='没有足够的新鲜相机帧确认放置稳定',frames=rows)

    def _execute(self, goal_id, payload):
        b = self.backend
        child = None
        recorder = threading.Thread(target=self._record_camera, daemon=True)
        recorder.start()
        try:
            # Final software checks happen immediately before starting the NUC
            # player. Its independent inspect/lock/ledger checks run again there.
            with b.lock:
                if self.stop_sent:
                    raise RuntimeError('任务在启动前已请求取消')
                reasons = [r for r in b._reasons(False) if not r.startswith('本次单次执行凭据')]
                if reasons:
                    raise RuntimeError('；'.join(reasons))
            child = self._spawn('execute')
            with self.process_lock:
                self.execution = child
                if self.stop_sent:
                    child.stdin.write('{"command":"stop"}\n'); child.stdin.flush()
            deadline = time.monotonic() + payload['timeout_ms']/1000
            def deadline_guard():
                while child.poll() is None and not self.quit.wait(.2):
                    if time.monotonic() > deadline:
                        self.stop_owned(); return
            threading.Thread(target=deadline_guard, daemon=True).start()
            with (b.output_dir/'hardware-events.jsonl').open('x') as file:
                for line in child.stdout:
                    if line.startswith('MTC_HARDWARE '):
                        event = json.loads(line[len('MTC_HARDWARE '):])
                        file.write(json.dumps(event,ensure_ascii=False)+'\n'); file.flush()
                        b.hardware_event(goal_id, event)
            returncode = child.wait(timeout=10)
            (b.output_dir/'player-exit.json').write_text(json.dumps(dict(returncode=returncode, automatic_retry=False)))
            visual = None
            if b.finished_stages == list(range(1,b.stage_count+1)) and b.completed_received is not None:
                b.feedback(goal_id, dict(state='VERIFYING', substate='placement_camera',
                    detail='实机轨迹已完成，正在核对 T0 红电池轮廓与至少 3 秒稳定图像'))
                visual = self._verify_placement()
                (b.output_dir/'placement-visual.json').write_text(json.dumps(visual,ensure_ascii=False,indent=2)+'\n')
            # Completion is based on all ten controller-confirmed endpoints and
            # NEW independent read-only feedback, never child exit code or a timer.
            until = time.monotonic()+10
            while time.monotonic() < until:
                with b.lock:
                    sample = b._sample('robot')
                    fresh = sample.get('fresh') and b.last_state_received > (b.completed_received or 0)
                    stopped = fresh and sample['data']['stop_confirmed']
                    if stopped:
                        state = deepcopy(b.last_state)
                        end = b.bundle['segments'][-1]['points'][-1]['positions']
                        error = math.degrees(max(abs(a-c) for a,c in zip(state['q'],end)))
                        complete = b.finished_stages == list(range(1,b.stage_count+1)) and b.completed_received is not None
                        (b.output_dir/'state-after.json').write_text(json.dumps(dict(state=state,
                            final_joint_error_deg=error, completed_stages=b.finished_stages,
                            independent_feedback=True), indent=2)+'\n')
                        if complete and error <= .05 and visual and visual['passed']:
                            b.result(goal_id, 4, dict(success=True, completed_count=1,completed_slots=['T0'],
                                error_code=0, message=f'本次实机 {b.stage_count}/{b.stage_count} 段完成；末姿态、停稳及 T0 红电池至少 3 秒稳定图像均已核验',
                                placement_visual_verified=True, final_joint_error_deg=error,
                                player_returncode=returncode))
                        elif complete:
                            b.uncertain(goal_id, '轨迹已返回完成回执，但末姿态或 T0 稳定图像核验未通过；禁止重试')
                        else:
                            task = b._owned(goal_id)
                            reason = (task or {}).get('execution_error')
                            message = ('轨迹已中断：'+reason+'；' if reason else '轨迹已中断，') + '独立反馈确认停稳；不会重试'
                            b.result(goal_id, 5 if self.stop_sent else 6, dict(success=False, completed_count=0,
                                completed_slots=[], error_code=1, message=message))
                        if b.stop_request:
                            b.stop_request.update(status='CONFIRMED', message='独立控制柜反馈已确认停稳')
                        return
                time.sleep(.1)
            b.uncertain(goal_id, '未获得新鲜的独立停稳反馈，执行结果未知；不会重发')
        except BaseException as error:
            b.uncertain(goal_id, '实机执行异常：'+str(error)+'；不会自动重试')
            if child and child.poll() is None:
                self.stop_owned()
        finally:
            self.running = False
            recorder.join(timeout=3)
            (b.output_dir/'webui-session.json').write_text(json.dumps(b.export_session(), ensure_ascii=False, indent=2)+'\n')

    def close(self):
        if self.running:
            self.stop_owned()
        self.quit.set()
        for worker in self.workers:
            worker.join(timeout=12)
        self.credentials.clear()
