"""Single-use field identification wrapper around the inspected J6 monitor.

Reads credentials, then camera leases, only from encrypted SSH stdin. The
historical 45-degree mission and its enable flags are never executed/changed.
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
import j6_45 as guarded
import smoke

require = smoke.require
RAD = math.pi / 180


def preflight(s, plan):
    smoke.vector(s['q'], 'q'); smoke.vector(s['qd'], 'qd')
    require(s['power_on'] is True and s['robot_mode'] in ('Idle', 'Running'), 'Not powered and idle/running')
    require(s['safety_mode'] in ('Normal', 'ReducedMode'), 'Safety state changed')
    expected_joint = 'Idle' if s['robot_mode'] == 'Idle' else 'Running'
    require(s['joint_states'] == [expected_joint] * 6, 'Unexpected joint states')
    require(s['joint_servo_modes'] == ['Position'] * 6, 'Not six position servos')
    require(s['control_mode'] in ('Position', 'Unknown'), 'Unexpected aggregate control mode')
    for key in ('simulation', 'force_control', 'soft_float', 'backdrive', 'freedrive', 'servo', 'collision'):
        require(s[key] is False, key + ' active or unknown')
    require(s['within_limits'] is True, 'Outside controller limits')
    require(s['runtime'] == 'Stopped' and s['exec_id'] == -1 and s['queue'] == 0, 'Existing program/motion')
    require(s['operational_mode'] == plan['expected_operation'], 'Operation mode changed')
    require(math.isfinite(s['speed_fraction']) and .1 <= s['speed_fraction'] <= 1, 'Invalid fraction')
    require(abs(s['speed_fraction'] - plan['expected_speed_fraction']) < 1e-6, 'Speed fraction changed')
    require(math.isfinite(s['read_seconds']) and 0 <= s['read_seconds'] <= 1, 'Feedback slow')
    require(max(map(abs, s['qd'])) <= guarded.STILL_SPEED, 'Already moving')
    require(max(abs(a-b) for a,b in zip(s['q'], plan['expected_q'])) <= guarded.DRIFT_TOL, 'Initial pose changed')


def target_for(plan, lower, upper):
    require(plan['allow_single_j6_motion'] is True and plan['automatic_return'] is False, 'Wrong authorization')
    require(plan['delta_deg'] == 15.0 and plan['velocity_deg_s'] == .2 and plan['acceleration_deg_s2'] == .5,
            'This reviewed profile is exactly +15 degrees at 0.2 deg/s')
    q = smoke.vector(plan['expected_q'], 'planned start')
    target = q.copy(); target[5] += 15 * RAD
    for values in (q, target):
        for i, value in enumerate(values):
            require(lower[i] + 2 * RAD <= value <= upper[i] - 2 * RAD, 'Position limit margin')
    return target


class Log:
    def __init__(self, path): self.path = path
    def event(self, event, **fields):
        with self.path.open('a') as f:
            f.write(json.dumps(dict(host_time_ns=time.time_ns(), event=event, **fields), allow_nan=False) + '\n')


class Lease:
    def __init__(self):
        self.last = time.monotonic()
        self.dead = False
        threading.Thread(target=self.receive, daemon=True).start()
    def receive(self):
        try:
            for line in sys.stdin:
                value = json.loads(line)
                require(value == {'camera_fresh': True}, 'Camera lease rejected')
                self.last = time.monotonic()
        finally:
            self.dead = True
    def check(self):
        require(not self.dead and time.monotonic() - self.last <= 5.0, 'Camera/SSH lease lost')


def main():
    request = json.loads(sys.stdin.readline())
    plan = request['plan']
    require(0 <= (time.time_ns() - plan['created_ns']) / 1e9 <= plan['expires_seconds'], 'Expired field plan')
    lease = Lease()
    root = Path('/home/robot/mtc_field_calibration/20261012/visible_j6_01')
    root.mkdir(parents=True, exist_ok=True)
    log = Log(root / 'events.jsonl')
    config = smoke.load_config(Path('/home/robot/aubo_s3_nuc_smoke/hardware/commission.local.json'))

    class Robot(guarded.Robot45):
        def snapshot(self):
            lease.check()
            value = super().snapshot()
            lease.check()
            return value

    robot = None
    started = False
    with smoke.local_lock():
        try:
            robot = Robot(config, request['credentials']['username'], request['credentials']['password'])
            request.pop('credentials')
            identity = robot.identity()
            guarded.validate_identity(identity, config)
            require(identity == plan['expected_identity'], 'Plan identity mismatch')
            require(robot.runtime.isTaskAlive(0) is False, 'Main task exists')
            require(json.loads(json.dumps(robot.config.getPayload())) == plan['expected_payload'], 'Payload configuration changed')
            require(list(robot.config.getTcpOffset()) == plan['expected_tcp_offset'], 'TCP configuration changed')
            target = target_for(plan, *robot.limits())
            before = robot.snapshot(); preflight(before, plan)
            log.event('preflight', state=before, plan=plan, target_q=target)
            # Reserve BEFORE startup or position command; cannot replay this run.
            with (root / 'attempted').open('x') as f:
                f.write(plan['plan_id']); f.flush(); os.fsync(f.fileno())
            (root / 'plan.json').write_text(json.dumps(plan, indent=2))
            if before['robot_mode'] == 'Idle':
                require(plan['allow_startup'] is True, 'Startup not authorized')
                lease.check()
                started = True
                log.event('startup_intent')
                code = robot.manage.startup()
                log.event('startup_response', result=code)
                require(code == 0, 'Startup rejected; no retry')
                deadline = time.monotonic() + 30
                while time.monotonic() < deadline:
                    s = robot.snapshot(); log.event('startup_sample', **s)
                    require(s['safety_mode'] in ('Normal', 'ReducedMode'), 'Safety changed during startup')
                    require(s['collision'] is False and s['runtime'] == 'Stopped' and s['queue'] == 0,
                            'Unexpected startup state')
                    require(max(abs(a-b) for a,b in zip(s['q'], before['q'])) <= .2*RAD, 'Pose changed during startup')
                    if s['robot_mode'] == 'Running' and s['joint_states'] == ['Running']*6:
                        break
                    time.sleep(.1)
                else:
                    raise smoke.Refused('Startup timeout; no motion')
            s = guarded.stationary(robot, log, fraction=plan['expected_speed_fraction'], operation=plan['expected_operation'])
            preflight(s, plan)
            log.event('before_motion_fk', value=robot.robot.getRobotAlgorithm().forwardKinematics(s['q']))
            active_plan = dict(initial_q=s['q'], speed_fraction=s['speed_fraction'], operational_mode=s['operational_mode'])
            started = True
            final = guarded.execute_segment(robot, s['q'], target, active_plan, log, 'field_j6_plus_15')
            final = guarded.stationary(robot, log, fraction=s['speed_fraction'], operation=s['operational_mode'])
            report = dict(success=True, automatic_return=False, initial_q=s['q'], final_q=final['q'],
                          measured_delta_deg=[math.degrees(b-a) for a,b in zip(s['q'], final['q'])],
                          final_state=final, sdk_forward_kinematics=robot.robot.getRobotAlgorithm().forwardKinematics(final['q']))
            (root / 'result.json').write_text(json.dumps(report, indent=2))
            log.event('complete', **report)
            print(json.dumps(report), flush=True)
        except BaseException as error:
            log.event('failed', error=type(error).__name__ + ': ' + str(error))
            if robot is not None and started:
                try: log.event('failure_stop', result=robot.stop())
                except BaseException: print('STOP DELIVERY UNKNOWN; onsite physical stop required', flush=True)
            raise
        finally:
            if robot is not None: robot.close()


if __name__ == '__main__':
    def interrupt(*_): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    main()
