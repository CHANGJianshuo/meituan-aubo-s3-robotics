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
from mtc_webui.hardware_tasks import color_masks, describe, stage_description, tracking_limits

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
        if (hashlib.sha256((self.package/'manifest.json').read_bytes()).hexdigest()!=settings['manifest_sha256']
                or hashlib.sha256((self.package/'full_trajectory.json').read_bytes()).hexdigest()!=self.manifest['sha256']['full_trajectory.json']
                or self.bundle['plan_id']!=self.manifest['plan_id']):
            raise ValueError('Prepared hardware package identity changed')
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
        self.profile = describe(self.bundle, self.stage_offset)
        self.tracking_limits = tracking_limits(self.manifest) if self.profile['mode']==2 else None
        self.motion_seconds = sum(segment['points'][-1]['t'] for segment in self.bundle['segments'])
        self.minimum_timeout_ms = (max(120000, math.ceil((self.motion_seconds+4*self.stage_count+30)*1000))
                                   if self.profile['mode'] == 2 else 120000)
        self.scope_note = 'AUBO 实机轨迹服务；仅执行已准备的 '+self.profile['title']+' 单次任务，不自动恢复或重发'
        self.player_returned = False
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

    def _sample(self, kind):
        sample=super()._sample(kind)
        # The existing laptop camera is independent of the robot SDK session.
        # Its image can remain visible while robot feedback and motion stay gated.
        if (kind in ('color','depth') and (sample.get('source') or '').startswith('local-preview:')
                and sample.get('age_ms') is not None):
            sample['fresh']=sample['age_ms']<self.config.freshness_ms
        return sample

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
                mask_pairs=list(zip(color_masks(reference,roi,self.profile['mode']==2),
                                    color_masks(image,roi,self.profile['mode']==2)))
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

    def validate(self, payload):
        RosBackend.validate(payload)
        if payload['mode'] != self.profile['mode'] or payload['ordered_colors'] != self.profile['ordered_colors']:
            raise ApiError(400, 'UNSUPPORTED_HARDWARE_TASK', '本次已准备的实机任务为 '+self.profile['title'])
        if payload['timeout_ms'] < self.minimum_timeout_ms:
            raise ApiError(400, 'TIMEOUT_TOO_SHORT', f'此轨迹的任务超时不得少于 {math.ceil(self.minimum_timeout_ms/1000)} 秒')

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
                completed_count=task['completed_count'] if task else 0, total_count=len(self.profile['assignments']),
                last_error_code=task['last_error_code'] if task else 0,
                detail=task['detail'] if task else '等待 '+self.profile['title']+' 单次任务'),
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
                if (type(index) is not int or index != len(self.finished_stages)+1
                        or not 1 <= index <= self.stage_count
                        or event['primitive'] != self.bundle['segments'][index-1]['primitive']):
                    self.uncertain(goal_id, '运动阶段与已准备轨迹不符；禁止重试')
                    self.transport.stop_owned()
                    return
                task.update(status='EXECUTING', state='EXECUTING', substate=event['primitive'],
                    stage_index=index, **stage_description(self.profile, self.bundle['segments'][index-1],
                                                          index, self.stage_offset))
                self._event('FEEDBACK', task['detail'], task)
            elif kind == 'stage_complete':
                if (type(event['stage']) is not int or not 1 <= event['stage'] <= self.stage_count
                        or event['stage'] != len(self.finished_stages)+1):
                    self.uncertain(goal_id, '实机阶段回执次序异常；禁止重试')
                    self.transport.stop_owned()
                    return
                self.finished_stages.append(event['stage'])
                task['stages_completed'] = len(self.finished_stages)
                self._event('STAGE_COMPLETE', f'第 {event["stage"]+self.stage_offset}/{self.profile["total_stages"]} 段终点及停稳已确认', task)
            elif kind == 'player_returned':
                self.player_returned = True
            elif kind == 'completed':
                if event.get('plan_id') != self.manifest['plan_id']:
                    self.uncertain(goal_id, '完成回执 plan_id 不匹配')
                    return
                self.completed_received = self.clock()
                task.update(status='EXECUTING', state='VERIFYING', detail=f'本次 {self.stage_count} 段轨迹反馈已返回，正在独立复核末姿态与停稳')
                self._event('VERIFYING', task['detail'], task)
            elif kind in ('bridge_error', 'stop_response', 'stop_feedback', 'stop_unverified', 'owned_stop_requested', 'tracking_guard_rejected'):
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
                read_only=False, plan_id=self.manifest['plan_id'], trajectory=self.settings.get('trajectory_label', '平滑轨迹') + (f' / 停稳后继续剩余 {self.stage_count} 段' if self.stage_offset else ''),
                source_sha256=self.settings['source_trajectory_sha256'], nuc='192.0.2.10',
                supported_task=self.profile['title'], supported_goal=dict(mode=self.profile['mode'], ordered_colors=self.profile['ordered_colors']),
                nominal_motion_seconds=self.motion_seconds, stage_count=self.stage_count,
                stage_offset=self.stage_offset, minimum_timeout_ms=self.minimum_timeout_ms,
                color_calibration='task2_scene_hsv_v1' if self.profile['mode']==2 else 'legacy_task1_hsv',
                tracking_guard_deg=self.tracking_limits,
                tracking_guard_action=self.manifest.get('tracking_guard_action','stop'),
                automatic_retry=False, single_use=True)
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
        if s.get('remote_bridge_sha256'):
            pin = ('from pathlib import Path; import hashlib,os; '
                   f'p=Path({s["remote_bridge"]!r}); '
                   f'assert hashlib.sha256(p.read_bytes()).hexdigest()=={s["remote_bridge_sha256"]!r}, "Bridge changed"; '
                   f'os.execv({s["remote_python"]!r},{remote!r})')
            remote = [s['remote_python'], '-c', pin]
        options = ['-o', 'IPQoS=none']
        ssh_host = s.get('ssh_host')
        if not isinstance(ssh_host, str) or not ssh_host or ssh_host.startswith('-') or any(c.isspace() for c in ssh_host):
            raise ValueError('Set ssh_host to the commissioned NUC user@host in the private configuration')
        if s.get('ssh_host_key_alias'):
            options += ['-o', 'HostKeyAlias='+s['ssh_host_key_alias']]
        if s.get('ssh_control_path'):
            options += ['-o', 'ControlPath='+s['ssh_control_path']]
        child = subprocess.Popen(['ssh', '-T', '-o','BatchMode=yes','-o','ConnectTimeout=8',
            '-o','ServerAliveInterval=2','-o','ServerAliveCountMax=3',
            '-o','StrictHostKeyChecking=yes',
            *options, ssh_host, shlex.join(remote)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
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
        reference_path=Path(b.settings['placement_reference'])
        reference=cv2.imread(str(reference_path))
        if reference is None:
            return dict(passed=False,reason='放置参考图像不可用',frames=[])
        sequence=b.profile['mode']==2
        checks=b.settings.get('placement_checks')
        if sequence:
            expected=[('A','P1',0),('B','P2',1),('C','P3',3),('D','source',2)]
            if (not isinstance(checks,list)
                    or [(c.get('object'),c.get('slot'),c.get('color_index')) for c in checks]!=expected
                    or hashlib.sha256(reference_path.read_bytes()).hexdigest()!=b.settings.get('placement_reference_sha256')):
                return dict(passed=False,reason='三块放置及绿色保留参考未完整固定',frames=[])
        elif checks is None:
            checks=[dict(object='A',slot='T0',color_index=0,roi=[460,440,585,565],minimum_pixels=2000)]
        def mask(image,check):
            roi=check['roi'];index=check['color_index']
            if (len(roi)!=4 or any(type(v) is not int for v in roi)
                    or type(index) is not int or index not in range(4)):
                raise ValueError('Invalid placement region')
            x0,y0,x1,y1=roi
            if not (0<=x0<x1<=image.shape[1] and 0<=y0<y1<=image.shape[0]):
                raise ValueError('Placement region is outside the image')
            return color_masks(image,roi,sequence)[index]
        try:
            expected_masks=[mask(reference,check) for check in checks]
            if any(m.sum()<max(300,c.get('minimum_pixels',2000)) for m,c in zip(expected_masks,checks)):
                return dict(passed=False,reason='参考图像缺少必要电池轮廓',frames=[])
        except (KeyError,ValueError,TypeError):
            return dict(passed=False,reason='放置参考区域无效',frames=[])
        rows=[];seen=set();until=time.monotonic()+10
        while time.monotonic()<until:
            with b.lock:
                b._read_preview('color','latest_color.jpg');sample=b._sample('color');jpeg=b.frames.get('color')
            if jpeg and sample.get('fresh') and sample['source_stamp_ns'] not in seen:
                stamp=sample['source_stamp_ns'];seen.add(stamp)
                frame=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
                if frame is None or frame.shape!=reference.shape:
                    return dict(passed=False,reason='当前相机图像不可比较',frames=rows)
                objects=[]
                for check,expected in zip(checks,expected_masks):
                    current=mask(frame,check);y,x=np.nonzero(current);yr,xr=np.nonzero(expected)
                    if len(x)<max(300,check.get('minimum_pixels',2000)):
                        return dict(passed=False,reason=check['slot']+' 缺少电池轮廓',frames=rows)
                    iou=float((current&expected).sum()/(current|expected).sum())
                    center=np.array([x.mean(),y.mean()]);distance=float(np.linalg.norm(center-[xr.mean(),yr.mean()]))
                    objects.append(dict(object=check['object'],slot=check['slot'],center_px=center.tolist(),iou=iou,reference_delta_px=distance))
                    if iou<.80 or distance>3.0:
                        return dict(passed=False,reason=check['slot']+' 轮廓与成功参考不一致',frames=rows,objects=objects)
                rows.append(dict(source_stamp_ns=stamp,objects=objects))
                if len(rows)>=4 and (stamp-rows[0]['source_stamp_ns'])/1e9>=3:
                    drifts=[]
                    for index in range(len(checks)):
                        centers=np.array([r['objects'][index]['center_px'] for r in rows])
                        drifts.append(float(np.linalg.norm(centers-centers.mean(axis=0),axis=1).max()))
                    cv2.imwrite(str(b.output_dir/'physical_final.png'),frame)
                    return dict(passed=max(drifts)<.75,source='Existing physical color camera',reference=str(reference_path),
                        stable_interval_s=(stamp-rows[0]['source_stamp_ns'])/1e9,max_centroid_deviation_px=max(drifts),frames=rows,
                        checked_objects=[c['object'] for c in checks],checked_slots=[c['slot'] for c in checks],
                        interpretation='Color silhouettes match reviewed upright placements and remain stable; no force, attachment or absolute calibration claim')
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
                    detail='实机轨迹已完成，正在核对 '+b.profile['title']+' 与至少 3 秒稳定图像'))
                visual = self._verify_placement()
                (b.output_dir/'placement-visual.json').write_text(json.dumps(visual,ensure_ascii=False,indent=2)+'\n')
            # Completion is based on every controller-confirmed endpoint and
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
                        if complete and error <= .05 and visual and visual['passed'] and returncode==0 and b.player_returned:
                            slots=[a['slot'] for a in b.profile['assignments']]
                            b.result(goal_id, 4, dict(success=True, completed_count=len(slots),completed_slots=slots,
                                error_code=0, message=f'本次实机 {b.stage_count}/{b.stage_count} 段完成；末姿态、停稳及 '+b.profile['title']+' 至少 3 秒稳定图像均已核验',
                                placement_visual_verified=True, final_joint_error_deg=error,
                                player_returncode=returncode))
                        elif complete:
                            b.uncertain(goal_id, '轨迹已返回完成回执，但播放器收尾、末姿态或放置图像核验未通过；禁止重试')
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
