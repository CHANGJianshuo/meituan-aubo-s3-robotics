"""One reviewed +90 degree J6 identification, stopped images between stages.

Uses inspected vendor SDK/RTDE guards. Does not call either historical mission.
Credentials and camera leases arrive over SSH stdin. No startup, automatic retry,
return, runtime start, payload mutation, or controller safety-setting changes.
"""
import json
import hashlib
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

sys.path.insert(0, '/home/robot/aubo_s3_nuc_smoke/hardware')
import smoke
import j6_45 as base
import j6_45_fast as fast

require = smoke.require
RAD = math.pi / 180


def stages_for(plan, lower, upper):
    require(plan['allow_field_j6_90'] is True and plan['automatic_return'] is False, 'Wrong authorization')
    require(plan['delta_deg'] == 90 and plan['speed_deg_s'] == 5 and plan['acceleration_deg_s2'] == 5,
            'Wrong reviewed motion profile')
    require(plan['cumulative_stops_deg'] == [90], 'Remaining stop changed')
    initial = smoke.vector(plan['initial_q'], 'initial_q')
    q = smoke.vector(plan['original_q'], 'original_q')
    require(max(abs(a-b) for a,b in zip(initial[:5], q[:5])) <= base.DRIFT_TOL, 'Other joints changed')
    require(58*RAD <= initial[5] - q[5] <= 59*RAD, 'Resume requires inspected 58-degree hold')
    lower = smoke.vector(lower, 'lower'); upper = smoke.vector(upper, 'upper')
    result = []
    for angle in [0] + plan['cumulative_stops_deg']:
        target = q.copy(); target[5] += angle * RAD
        for i, value in enumerate(target):
            require(lower[i] < upper[i] and lower[i] + 2*RAD <= value <= upper[i] - 2*RAD,
                    'Start/target violates controller position limit margin')
        if angle:
            result.append(dict(name=f'plus_{angle:02d}', target=target, speed_deg_s=5., acceleration_deg_s2=5.))
    return result


class Log:
    def __init__(self, path): self.path = path
    def event(self, event, **fields):
        with self.path.open('a') as f:
            f.write(json.dumps(dict(host_time_ns=time.time_ns(), event=event, **fields), allow_nan=False) + '\n')


class Lease:
    def __init__(self):
        self.last = time.monotonic(); self.dead = False; self.advance = None
        threading.Thread(target=self.receive, daemon=True).start()
    def receive(self):
        try:
            for line in sys.stdin:
                value = json.loads(line)
                require(value.get('camera_fresh') is True, 'Camera lease withdrawn')
                self.last = time.monotonic()
                if 'advance' in value: self.advance = value['advance']
        finally: self.dead = True
    def check(self):
        require(not self.dead and time.monotonic() - self.last <= 5., 'Camera or SSH lease lost')


class Metadata:
    """Independent read-only RPC client; cannot block the RTDE consumer."""
    def __init__(self, config, credentials):
        self.reader = base.Robot45(config, credentials['username'], credentials['password'])
        self.stop_event = threading.Event(); self.cache = None; self.error = None
        self.thread = threading.Thread(target=self.collect, daemon=True)
        self.thread.start()
    def collect(self):
        while not self.stop_event.is_set():
            try:
                start = time.monotonic(); s = self.reader.snapshot()
                self.cache = (start, s)
            except BaseException as e:
                self.error = str(e); return
            self.stop_event.wait(.15)
    def latest(self):
        require(self.error is None, 'Metadata RPC failure: ' + str(self.error))
        require(self.cache is not None, 'Metadata not ready')
        stamp, state = self.cache
        require(0 <= time.monotonic() - stamp <= .5, 'Full status older than 500 ms')
        return stamp, state
    def close(self):
        self.stop_event.set(); self.thread.join(timeout=2); self.reader.close()


def segment(robot, metadata, lease, plan, initial, stage, log):
    """Unchanged 100 Hz position/speed guard, independent full-status polling."""
    target = stage['target']; speed = stage['speed_deg_s']
    guard = fast.FeedbackGuard(initial, target, plan['original_q'], speed)
    end = time.monotonic()+3
    while metadata.cache is None:
        lease.check(); require(time.monotonic() < end, 'Metadata startup timeout'); time.sleep(.01)
    _, s = metadata.latest()
    base.validate_state(s, idle=True, fraction=plan['speed_fraction'], operation=plan['operational_mode'])
    require(max(abs(a-b) for a,b in zip(s['q'], initial)) <= base.DRIFT_TOL, 'Start changed before command')
    robot.begin_stage()
    start = time.monotonic(); end = start+max(30,20+2*(target[5]-initial[5])/RAD/(speed*plan['speed_fraction']))
    arrived_since = None; saw_motion = False; last_stamp = None; next_print = start
    log.event('command_intent_fast', stage=stage, initial_q=initial, metadata_profile='independent_rpc_500ms_rtde_100ms')
    code = robot.move_stage(target, speed, stage['acceleration_deg_s2'])
    log.event('move_response_fast', stage=stage['name'], result=code)
    require(code == 0, 'moveJoint rejected; no retry')
    while time.monotonic() < end:
        lease.check(); stamp, full = metadata.latest()
        base.validate_state(full, fraction=plan['speed_fraction'], operation=plan['operational_mode'])
        if stamp != last_stamp:
            log.event('status_fast', stage=stage['name'], metadata_age=time.monotonic()-stamp, **full); last_stamp=stamp
        samples = robot.fast_samples()
        if not samples:
            time.sleep(.002); continue
        for s in samples:
            log.event('feedback_fast',stage=stage['name'],**s); guard.check(s)
        saw_motion = saw_motion or abs(s['q'][5]-initial[5]) > .05*RAD
        arrived = (saw_motion and max(abs(a-b) for a,b in zip(s['q'],target)) <= base.POSITION_TOL
                   and max(map(abs,s['qd'])) <= base.STILL_SPEED and full['exec_id'] == -1 and full['queue'] == 0)
        if arrived:
            if arrived_since is None: arrived_since=time.monotonic()
            if time.monotonic()-arrived_since >= 1:
                stats=dict(max_actual_deg_s=guard.max_actual,max_target_deg_s=guard.max_target,
                           max_position_slope_deg_s=guard.max_slope,max_tracking_deg=guard.max_tracking,
                           elapsed_seconds=time.monotonic()-start)
                log.event('stage_verified_fast',stage=stage['name'],q=s['q'],**stats)
                print('VERIFIED '+stage['name']+' '+json.dumps(stats),flush=True); return s
        else: arrived_since=None
        if time.monotonic() >= next_print:
            print(f"{stage['name']}: J6={s['q'][5]/RAD:.4f}, target={target[5]/RAD:.4f}",flush=True)
            next_print=time.monotonic()+2
        time.sleep(.005)
    raise smoke.Refused('Segment timeout; no retry')


def main():
    request = json.loads(sys.stdin.readline()); plan = request['plan']
    require(0 <= (time.time_ns() - plan['created_ns'])/1e9 <= plan['expires_seconds'], 'Plan expired')
    lease = Lease()
    root = Path('/home/robot/mtc_field_calibration/20261012/j6_90_04')
    root.mkdir(parents=True, exist_ok=True)
    log = Log(root/'events.jsonl')
    config = smoke.load_config(Path('/home/robot/aubo_s3_nuc_smoke/hardware/commission.local.json'))
    source = Path('/home/robot/mtc_field_calibration/20261012/j6_90_02/plan.json').read_bytes()
    require(hashlib.sha256(source).hexdigest() == plan['source_plan_sha256'], 'Original plan changed')
    require(json.loads(source)['initial_q'] == plan['original_q'], 'Original endpoint reference mismatch')

    class Robot(fast.Robot):
        def snapshot(self):
            lease.check(); result = super().snapshot(); lease.check(); return result
        def stop(self): return self.motion.stopJoint(10 * RAD)

    robot = None; attempted = False; metadata = None
    with smoke.local_lock():
        try:
            credentials = request.pop('credentials')
            robot = Robot(config, credentials['username'], credentials['password'])
            base.validate_identity(robot.identity(), config)
            require(robot.identity() == plan['identity'], 'Plan identity mismatch')
            require(robot.runtime.isTaskAlive(0) is False, 'Existing runtime task')
            require(json.loads(json.dumps(robot.config.getPayload())) == plan['payload'], 'Payload changed')
            require(list(robot.config.getTcpOffset()) == plan['tcp_offset'], 'TCP offset changed')
            for name in ('getJointMaxSpeeds', 'getLimitJointMaxSpeeds'):
                require(smoke.vector(getattr(robot.config, name)(), name)[5] >= 5*RAD, 'Velocity limit too low')
            for name in ('getJointMaxAccelerations', 'getLimitJointMaxAccelerations'):
                require(smoke.vector(getattr(robot.config, name)(), name)[5] >= 10*RAD, 'Acceleration limit too low')
            stages = stages_for(plan, *robot.limits())
            s = fast.stationary(robot, log, fraction=plan['speed_fraction'], operation=plan['operational_mode'])
            require(max(abs(a-b) for a,b in zip(s['q'], plan['initial_q'])) <= base.DRIFT_TOL, 'Initial pose changed')
            log.event('preflight', state=s, plan=plan, stages=stages)
            with (root/'attempted').open('x') as f:
                f.write(plan['plan_id']); f.flush(); os.fsync(f.fileno())
            (root/'plan.json').write_text(json.dumps(plan, indent=2))
            for stage in stages:
                # Remain stationary until this exact stage's image has been reviewed.
                ready = dict(event='ready_for_stage', stage=stage['name'], state=s,
                             sdk_forward_kinematics=robot.robot.getRobotAlgorithm().forwardKinematics(s['q']))
                print(json.dumps(ready), flush=True); log.event('ready_for_stage', stage=stage['name'], state=s)
                lease.advance = None
                end = time.monotonic() + 180
                while lease.advance != stage['name']:
                    lease.check()
                    require(time.monotonic() < end, 'Observation gate expired; no further command')
                    now = robot.snapshot()
                    base.validate_state(now, idle=True, fraction=plan['speed_fraction'], operation=plan['operational_mode'])
                    require(max(abs(a-b) for a,b in zip(now['q'], s['q'])) <= base.DRIFT_TOL, 'Moved during observation hold')
                    robot.fast_samples()
                    time.sleep(.1)
                lease.check(); attempted = True
                metadata = Metadata(config, credentials)
                s = segment(robot, metadata, lease, plan, s['q'], stage, log)
                metadata.close(); metadata = None
                s = fast.stationary(robot, log, fraction=plan['speed_fraction'], operation=plan['operational_mode'])
                require(max(abs(a-b) for a,b in zip(s['q'], stage['target'])) <= base.POSITION_TOL, 'Final hold error')
                require(json.loads(json.dumps(robot.config.getPayload())) == plan['payload'], 'Payload changed')
                require(list(robot.config.getTcpOffset()) == plan['tcp_offset'], 'TCP changed')
                log.event('stage_result', stage=stage['name'], state=s,
                          sdk_forward_kinematics=robot.robot.getRobotAlgorithm().forwardKinematics(s['q']))
            report = dict(success=True, initial_q=plan['initial_q'], final_q=s['q'], final_state=s,
                          measured_delta_deg=[math.degrees(b-a) for a,b in zip(plan['initial_q'],s['q'])],
                          sdk_forward_kinematics=robot.robot.getRobotAlgorithm().forwardKinematics(s['q']),
                          automatic_return=False, commanded_speed_deg_s=5.)
            (root/'result.json').write_text(json.dumps(report, indent=2))
            log.event('complete', **report); print(json.dumps(dict(event='complete', **report)), flush=True)
        except BaseException as error:
            log.event('failed', error=type(error).__name__ + ': ' + str(error))
            if robot is not None and attempted:
                try: log.event('failure_stop', result=robot.stop())
                except BaseException: print('STOP DELIVERY UNKNOWN; check onsite physical stop', flush=True)
            raise
        finally:
            if metadata is not None: metadata.close()
            if robot is not None: robot.close()


if __name__ == '__main__':
    def interrupt(*_): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    main()
