#!/usr/bin/env python3
"""Captured-start task-one playback. Default offline; inspect is read-only.

Only execute sends path buffers. It never changes controller configuration,
power, operating mode, runtime, safety settings or I/O. No automatic retries.
The source succeeded in Gazebo; real collision clearance remains unverified.
"""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import getpass
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import uuid

from trajectory import RAD, dense, load_package, require, validate, vector

ROOT = Path(__file__).resolve().parent
HARDWARE = Path('/home/robot/aubo_s3_nuc_smoke/hardware')
START_TOL = .05 * RAD
STILL_SPEED = .02 * RAD
SETTLE_TIMEOUT = 5.0
TRACKING_POLICY = 'task2_j4_0p6_v1'
TRACKING_LIMITS_DEG = (.5, .5, .5, .6, .5, .5)
ENVELOPE_TOL = .2 * RAD
STOP_ACCELERATION = 1.0 * RAD


class Log:
    def __init__(self, root, command):
        folder = root / 'logs'; folder.mkdir(exist_ok=True)
        self.path = folder / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '.jsonl')
        self.file = self.path.open('x', buffering=1)
        self.event('begin', command=command)

    def event(self, event, **fields):
        self.file.write(json.dumps(dict(utc=datetime.now(timezone.utc).isoformat(), event=event, **fields),
                                   ensure_ascii=False, allow_nan=False) + '\n')

    def close(self):
        self.file.close()


def normalized(value):
    return json.loads(json.dumps(value, allow_nan=False))


def assert_start(actual, expected):
    vector(actual, 'actual start'); vector(expected, 'planned start')
    delta = max(abs(a-b) for a,b in zip(actual, expected))
    require(delta <= START_TOL,
            f'Start differs by {delta/RAD:.4f} deg (> 0.05 deg). Re-read and rebuild; no automatic repositioning.')


def assert_state(s, idle=False, fraction=None, operation=None):
    vector(s['q'], 'actual q'); vector(s['qd'], 'actual qd')
    require(s['robot_mode'] == 'Running' and s['power_on'] is True, 'Robot must already be enabled onsite')
    require(s['safety_mode'] in ('Normal', 'ReducedMode'), 'Controller safety state rejects motion')
    require(s['joint_states'] == ['Running']*6 and s['joint_servo_modes'] == ['Position']*6,
            'All six joints must be running in position mode')
    require(s['control_mode'] in ('Position', 'Unknown'), 'Incompatible control mode')
    for key in ('simulation', 'freedrive', 'servo', 'collision', 'force_control', 'soft_float', 'backdrive'):
        require(s[key] is False, f'{key} active or unknown')
    require(s['within_limits'] is True and s['runtime'] == 'Stopped', 'Limits invalid or another runtime active')
    require(s['operational_mode'] in ('Manual', 'Automatic'), 'Unknown operational mode')
    require(math.isfinite(s['read_seconds']) and 0 <= s['read_seconds'] <= 1., 'RPC feedback too slow')
    require(math.isfinite(s['speed_fraction']) and .1 <= s['speed_fraction'] <= 1., 'Invalid speed fraction')
    if fraction is not None:
        require(abs(s['speed_fraction']-fraction) < 1e-6, 'Speed fraction changed; do not raise it during playback')
    if operation is not None:
        require(s['operational_mode'] == operation, 'Operational mode changed since capture')
    if idle:
        require(s['exec_id'] == -1 and s['queue'] == 0, 'Existing motion or queued command')
        require(max(map(abs, s['qd'])) <= STILL_SPEED, 'Robot is not stationary')


def stationary(robot, expected, manifest, log, seconds=1.0, fraction=None):
    """Wait for continuous stillness, without relaxing state/position guards.

    A transient velocity sample resets the dwell; it does not send motion or
    retry a segment. Active queues, unsafe state and position drift still fail
    immediately. Record the sample before checking it so failures are auditable.
    """
    require(math.isfinite(seconds) and 0 < seconds < SETTLE_TIMEOUT,
            'Invalid stationary dwell')
    started = time.monotonic()
    stable_since = None
    while True:
        s = robot.snapshot()
        now = time.monotonic()
        log.event('stationary_sample', elapsed=now-started, **s)
        try:
            assert_state(s, fraction=fraction, operation=manifest['operational_mode'])
            require(s['exec_id'] == -1 and s['queue'] == 0, 'Existing motion or queued command')
            assert_start(s['q'], expected)
        except Exception as error:
            log.event('stationary_rejected', error=type(error).__name__+': '+str(error))
            raise
        speed = max(map(abs, s['qd']))
        if now-started >= SETTLE_TIMEOUT:
            log.event('stationary_timeout', elapsed=now-started,
                      max_speed_deg_s=speed/RAD, required_stable_seconds=seconds)
            raise ValueError(f'Robot did not remain stationary for {seconds:g} s within {SETTLE_TIMEOUT:g} s')
        if speed <= STILL_SPEED:
            if stable_since is None:
                stable_since = now
            log.event('stationary', **s)
            if now-stable_since >= seconds:
                log.event('stationary_confirmed', elapsed=now-started,
                          stable_seconds=now-stable_since)
                return s
        else:
            stable_since = None
            log.event('stationary_wait', elapsed=now-started,
                      max_speed_deg_s=speed/RAD, threshold_deg_s=STILL_SPEED/RAD)
        time.sleep(.1)


def verify_tracking_policy(manifest):
    require(manifest.get('tracking_guard_policy') == TRACKING_POLICY
            and manifest.get('tracking_guard_deg') == list(TRACKING_LIMITS_DEG)
            and manifest.get('uniform_tracking_guard_deg') is None,
            'Tracking policy does not match this J4 0.6 degree player')


def configuration_check(robot, manifest):
    verify_tracking_policy(manifest)
    require(robot.identity() == manifest['expected_identity'], 'Robot identity or controller version changed')
    require(normalized(robot.config.getTcpOffset()) == manifest['expected_tcp_offset'], 'Configured TCP changed')
    require(normalized(robot.config.getPayload()) == manifest['expected_payload'], 'Configured payload changed')
    require(robot.runtime.isTaskAlive(0) is False, 'A runtime task is active')


def inspect(robot, bundle, manifest, log, scale):
    """Getters only: do not allocate/evaluate buffers, set modes or move."""
    configuration_check(robot, manifest)
    limits = [list(x) for x in robot.limits()]
    stats = validate(bundle, limits)
    hw_v = vector(list(robot.config.getJointMaxSpeeds()), 'joint speed limits')
    cfg_v = vector(list(robot.config.getLimitJointMaxSpeeds()), 'configured speed limits')
    hw_a = vector(list(robot.config.getJointMaxAccelerations()), 'joint acceleration limits')
    cfg_a = vector(list(robot.config.getLimitJointMaxAccelerations()), 'configured acceleration limits')
    for i in range(6):
        require(stats['peak_velocity_deg_s'][i]*RAD/scale < min(hw_v[i], cfg_v[i]), f'J{i+1} speed limit')
        require(stats['peak_acceleration_deg_s2'][i]*RAD/scale**2 < min(hw_a[i], cfg_a[i]), f'J{i+1} acceleration limit')
    s = stationary(robot, bundle['segments'][0]['points'][0]['positions'], manifest, log, seconds=1.5)
    report = dict(read_only=True, controller_checks_passed=True, physical_clearance_verified=False,
                  actual_q_deg=[x/RAD for x in s['q']], selection=bundle['selection'],
                  speed_fraction=s['speed_fraction'], operational_mode=s['operational_mode'],
                  planned_motion_seconds=stats['nominal_duration_s']*scale/s['speed_fraction'],
                  time_scale=scale, limits=limits, expected_identity=manifest['expected_identity'])
    log.event('inspected', report=report)
    return s, report


@contextmanager
def motion_lock(path):
    with path.open('a') as f:
        try:
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError('Another local motion script holds the robot lock') from None
        try:
            yield
        finally:
            fcntl.flock(f, fcntl.LOCK_UN)


def reserve_attempt(root, manifest, selection):
    folder = root/'attempts'; folder.mkdir(exist_ok=True)
    # The shared NUC ledger prevents a copied package from replaying the same plan.
    path = folder/(manifest['plan_id'] + '_' + selection + '.json')
    with path.open('x') as f:
        json.dump(dict(plan_id=manifest['plan_id'], selection=selection,
                       utc=datetime.now(timezone.utc).isoformat(), status='reserved_no_automatic_retry'), f)
        f.flush(); os.fsync(f.fileno())
    return path


def prepare_buffer(motion, name, positions, interval, log):
    require(motion.pathBufferAlloc(name, 2, len(positions)) == 0, 'Path buffer allocation rejected')
    # Each call is made once, including the final partial batch.
    for start in range(0, len(positions), 100):
        require(motion.pathBufferAppend(name, positions[start:start+100]) == 0, 'Path buffer append rejected')
    require(motion.pathBufferEval(name, [], [], interval) == 0, 'Path buffer evaluation rejected')
    until = time.monotonic()+30
    while not motion.pathBufferValid(name):
        require(time.monotonic() < until, 'Path buffer validation timed out')
        time.sleep(.05)
    log.event('buffer_ready', name=name, points=len(positions), interval=interval)


def position_bounds_for_samples(samples):
    lows = [min(q[i] for q,_,_ in samples) for i in range(6)]
    highs = [max(q[i] for q,_,_ in samples) for i in range(6)]
    return lows, highs


def check_stream(samples, bounds, log, stage=None):
    lo, hi = bounds
    for s in samples:
        require(s['robot_mode'] == 'Running' and s['safety_mode'] in ('Normal', 'ReducedMode')
                and s['runtime'] == 'Stopped', 'RTDE reports incompatible controller state')
        for key in ('q', 'qd', 'target_q', 'target_qd', 'target_qdd'):
            vector(s[key], 'RTDE '+key)
        # Preserve the synchronized position pair even when a guard rejects it.
        # These are observations, not assertions that a sample was accepted.
        log.event('rtde', **s)
        for i in range(6):
            require(lo[i]-ENVELOPE_TOL <= s['q'][i] <= hi[i]+ENVELOPE_TOL, f'J{i+1} outside stage envelope')
            require(lo[i]-ENVELOPE_TOL <= s['target_q'][i] <= hi[i]+ENVELOPE_TOL, f'J{i+1} target outside envelope')
            error_deg = abs(s['q'][i]-s['target_q'][i])/RAD
            if error_deg > TRACKING_LIMITS_DEG[i]:
                log.event('tracking_guard_rejected', stage=stage, joint=i+1,
                          error_deg=error_deg, limit_deg=TRACKING_LIMITS_DEG[i], sample=s)
                raise ValueError(f'J{i+1} tracking error {error_deg:.6f} deg > {TRACKING_LIMITS_DEG[i]:g} deg')


def stop_after_failure(robot, log):
    """One best-effort stop; a lost connection cannot guarantee physical stopping."""
    try:
        result = robot.motion.stopJoint(STOP_ACCELERATION)
        log.event('stop_response', result=result)
        require(result == 0, 'Controller rejected stopJoint')
        until = time.monotonic()+10
        while time.monotonic() < until:
            s = robot.snapshot()
            log.event('stop_feedback', **s)
            if s['exec_id'] == -1 and s['queue'] == 0 and max(map(abs, s['qd'])) <= STILL_SPEED:
                print('停止后的反馈已静止；请在现场核实。', flush=True)
                return True
            time.sleep(.1)
    except BaseException as exc:
        try:
            log.event('stop_unverified', error=type(exc).__name__+': '+str(exc))
        except BaseException:
            pass
    print('未能验证停止！请使用现场物理停止装置。断开 SSH 不保证停止。', file=sys.stderr, flush=True)
    return False


def run_segment(robot, stream, segment, manifest, scale, fraction, log, index):
    interval, sampled = dense(segment, .01)
    positions = [q for q,_,_ in sampled]
    bounds = position_bounds_for_samples(sampled)
    name = 'task1_' + uuid.uuid4().hex[:16]
    attempted = False; completed = False
    try:
        stationary(robot, positions[0], manifest, log, fraction=fraction)
        prepare_buffer(robot.motion, name, positions, interval*scale, log)
        # Upload/evaluation may take seconds; recheck configuration and start afterwards.
        configuration_check(robot, manifest)
        stationary(robot, positions[0], manifest, log, fraction=fraction)
        check_stream(stream.drain(), bounds, log, stage=index)
        start = time.monotonic()
        planned = segment['points'][-1]['t']*scale/fraction
        deadline = start + planned*1.5 + 30
        log.event('move_intent', stage=index, primitive=segment['primitive'], buffer=name)
        attempted = True  # A timeout may mean the controller accepted the command.
        result = robot.motion.movePathBuffer(name)
        log.event('move_response', stage=index, result=result)
        require(result == 0, f'movePathBuffer returned {result}; no retry')
        arrived_since = None; saw_active = False; next_print = start
        while time.monotonic() < deadline:
            check_stream(stream.drain(), bounds, log, stage=index)
            s = robot.snapshot()
            assert_state(s, fraction=fraction, operation=manifest['operational_mode'])
            log.event('rpc_motion', stage=index, elapsed=time.monotonic()-start, **s)
            if s['exec_id'] != -1 or max(abs(a-b) for a,b in zip(s['q'], positions[0])) > START_TOL:
                saw_active = True
            arrived = (saw_active and s['exec_id'] == -1 and s['queue'] == 0
                       and max(map(abs, s['qd'])) <= STILL_SPEED
                       and max(abs(a-b) for a,b in zip(s['q'], positions[-1])) <= START_TOL)
            if arrived:
                if arrived_since is None:
                    arrived_since = time.monotonic()
                if time.monotonic()-arrived_since >= 1.:
                    completed = True
                    log.event('stage_complete', stage=index, final_q=s['q'])
                    return s
            else:
                arrived_since = None
            if not saw_active and time.monotonic()-start > 10:
                raise ValueError('Command accepted but no execution observed; no retry')
            if time.monotonic() >= next_print:
                print(f'{index:02d} {segment["primitive"]}: {time.monotonic()-start:.1f}s / ~{planned:.1f}s', flush=True)
                next_print = time.monotonic()+5
            time.sleep(.02)
        raise ValueError('Motion deadline exceeded; no retry')
    except BaseException:
        if attempted:
            stop_after_failure(robot, log)
        raise
    finally:
        # Never free an active/uncertain path. Leave its name in the log for diagnosis.
        if not attempted or completed:
            try:
                result = robot.motion.pathBufferFree(name)
                log.event('buffer_free', name=name, result=result)
            except Exception as exc:
                log.event('buffer_cleanup_failed', name=name, error=str(exc))


def execute(robot, stream_factory, bundle, manifest, scale, log, ledger_root=HARDWARE):
    with motion_lock(HARDWARE/'.motion.lock'):
        initial, report = inspect(robot, bundle, manifest, log, scale)
        print(json.dumps(report, indent=2, ensure_ascii=False))
        print(f'状态检查通过，按时间倍率 {scale:g} 执行 {bundle["selection"]} 轨迹。', flush=True)
        configuration_check(robot, manifest)
        stationary(robot, bundle['segments'][0]['points'][0]['positions'], manifest, log,
                   fraction=initial['speed_fraction'])
        # Start the bounded RTDE queue after the read-only preflight.
        stream = stream_factory()
        try:
            ledger = reserve_attempt(ledger_root, manifest, bundle['selection'])
            log.event('attempt_reserved', path=str(ledger), plan_id=manifest['plan_id'])
            for i, segment in enumerate(bundle['segments'], 1):
                run_segment(robot, stream, segment, manifest, scale, initial['speed_fraction'], log, i)
            log.event('completed', selection=bundle['selection'], plan_id=manifest['plan_id'])
            print('已通过各段终点反馈检查；未执行自动返回。', flush=True)
        finally:
            stream.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', nargs='?', choices=['offline', 'inspect', 'execute'], default='offline')
    parser.add_argument('--trajectory', choices=['full', 'task'], default='full')
    parser.add_argument('--time-scale', type=float, default=1., help='1-10; default 1 preserves nominal trajectory timing')
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--auth-stdin', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    require(math.isfinite(args.time_scale) and 1 <= args.time_scale <= 10, 'time-scale must be between 1 and 10')
    require(not args.auth_stdin or args.command == 'inspect', '--auth-stdin is available only for read-only inspect')
    bundle, manifest = load_package(args.root, args.trajectory)
    verify_tracking_policy(manifest)
    stats = validate(bundle, manifest['captured_limits'])
    print(json.dumps(dict(selection=args.trajectory, plan_id=manifest['plan_id'],
                          nominal_duration_s=stats['nominal_duration_s'], time_scale=args.time_scale,
                          motion_seconds_at_fraction_1=stats['nominal_duration_s']*args.time_scale,
                          peak_speed_deg_s=max(stats['peak_velocity_deg_s'])/args.time_scale,
                          application_speed_thresholds=False,
                          tracking_guard_deg=list(TRACKING_LIMITS_DEG),
                          physical_execution_validated=False), indent=2))
    if args.command == 'offline':
        print('离线校验通过；未连接机械臂。')
        return 0
    # Hardware imports occur only after the explicit inspect/execute selection.
    sys.path.insert(0, str(HARDWARE))
    from j6_45 import Robot45
    from telemetry import Telemetry, quiet_native_auth
    if args.auth_stdin:
        credentials = json.loads(sys.stdin.readline())
        username, password = credentials['username'], credentials['password']
        del credentials
    else:
        username = input('SDK username [aubo]: ').strip() or 'aubo'
        password = getpass.getpass('SDK password (not saved): ')
    robot = None
    log = Log(args.root, args.command)
    try:
        with quiet_native_auth():
            robot = Robot45(manifest, username, password)
        if args.command == 'inspect':
            _, report = inspect(robot, bundle, manifest, log, args.time_scale)
            path = log.path.with_suffix('.summary.json')
            path.write_text(json.dumps(report, indent=2)+'\n')
            print(json.dumps(report, indent=2, ensure_ascii=False))
        else:
            execute(robot, lambda: Telemetry(manifest['host'], username, password),
                    bundle, manifest, args.time_scale, log)
        return 0
    except BaseException as exc:
        log.event('failed', error=type(exc).__name__+': '+str(exc))
        raise
    finally:
        password = None
        if robot is not None:
            robot.close()
        log.close()
        print('Log:', log.path, flush=True)


if __name__ == '__main__':
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('已中断；请在现场核实机械臂状态。', file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(type(exc).__name__+': '+str(exc), file=sys.stderr)
        raise SystemExit(1)
