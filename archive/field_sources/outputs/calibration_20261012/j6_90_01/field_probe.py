"""One reviewed +90 degree J6 identification, stopped images between stages.

Uses inspected vendor SDK/RTDE guards. Does not call either historical mission.
Credentials and camera leases arrive over SSH stdin. No startup, automatic retry,
return, runtime start, payload mutation, or controller safety-setting changes.
"""
import json
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
    require(plan['cumulative_stops_deg'] == [5, 30, 60, 90], 'Stops changed')
    q = smoke.vector(plan['initial_q'], 'initial_q')
    require(q == plan['original_q'], 'Original start changed')
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


def main():
    request = json.loads(sys.stdin.readline()); plan = request['plan']
    require(0 <= (time.time_ns() - plan['created_ns'])/1e9 <= plan['expires_seconds'], 'Plan expired')
    lease = Lease()
    root = Path('/home/robot/mtc_field_calibration/20261012/j6_90_01')
    root.mkdir(parents=True, exist_ok=True)
    log = Log(root/'events.jsonl')
    config = smoke.load_config(Path('/home/robot/aubo_s3_nuc_smoke/hardware/commission.local.json'))

    class Robot(fast.Robot):
        def snapshot(self):
            lease.check(); result = super().snapshot(); lease.check(); return result
        def stop(self): return self.motion.stopJoint(10 * RAD)

    robot = None; attempted = False
    with smoke.local_lock():
        try:
            robot = Robot(config, request['credentials']['username'], request['credentials']['password'])
            request.pop('credentials')
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
                s = fast.segment(robot, plan, s['q'], stage, log)
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
            if robot is not None: robot.close()


if __name__ == '__main__':
    def interrupt(*_): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    main()
