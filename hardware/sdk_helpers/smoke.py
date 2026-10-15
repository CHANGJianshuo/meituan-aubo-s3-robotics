#!/usr/bin/env python3
"""Read-only by default. One bounded J6 move, only after explicit local arming.

This is a commissioning aid, NOT a safety controller or collision planner.
No power/brake release, fault reset, mode change, runtime start, or auto return.
"""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import getpass
import importlib.metadata
import ipaddress
import json
import math
import os
from pathlib import Path
import signal
import socket
import sys
import time

ROOT = Path(__file__).resolve().parent
RAD = math.pi / 180
DELTA_DEG = 0.3
MAX_DELTA_DEG = 0.5
VISIBLE_MAX_DEG = 3.0
SPEED = 0.2 * RAD
ACCEL = 0.5 * RAD
STOP_ACCEL = 1.0 * RAD
POSITION_TOL = 0.03 * RAD
STILL_SPEED = 0.02 * RAD
START_DRIFT = 0.02 * RAD
MONITOR_SPEED = 0.6 * RAD
MAX_SNAPSHOT_SECONDS = 1.0
MOTION_TIMEOUT = 30.0


class Refused(RuntimeError):
    pass


def require(ok, message):
    if not ok:
        raise Refused(message)


def vector(value, label):
    require(isinstance(value, (list, tuple)) and len(value) == 6, label + ': expected six values')
    require(all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
                for x in value), label + ': invalid / non-finite values')
    return list(value)


def checked_delta(value, visible=False):
    limit = VISIBLE_MAX_DEG if visible else MAX_DELTA_DEG
    require(math.isfinite(value) and 0.1 <= abs(value) <= limit,
            f'J6 delta must be between 0.1 and {limit} degrees in magnitude')
    return value


def target_for(q, delta_deg, low, high, visible=False):
    checked_delta(delta_deg, visible)
    q, low, high = vector(q, 'q'), vector(low, 'lower limits'), vector(high, 'upper limits')
    target = q.copy()
    target[5] += delta_deg * RAD
    for i in range(6):
        require(low[i] < high[i], f'Invalid J{i+1} limits')
        require(low[i] + 2 * RAD <= q[i] <= high[i] - 2 * RAD,
                f'J{i+1} start is within 2 degrees of a controller limit')
        require(low[i] + 2 * RAD <= target[i] <= high[i] - 2 * RAD,
                f'J{i+1} target is within 2 degrees of a controller limit')
    return target


def norm_type(value):
    return str(value).lower().removeprefix('aubo_')


def enum_name(value):
    return str(value).split('.')[-1]


class Log:
    def __init__(self, command):
        directory = ROOT / 'logs'
        directory.mkdir(exist_ok=True)
        self.path = directory / (datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.jsonl')
        self.event('begin', command=command, sdk=importlib.metadata.version('pyaubo-sdk'))

    def event(self, event, **fields):
        row = dict(time_utc=datetime.now(timezone.utc).isoformat(), event=event, **fields)
        with self.path.open('a', encoding='utf-8') as f:
            f.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')


class Robot:
    def __init__(self, config, username, password):
        import pyaubo_sdk as sdk
        self.rpc = sdk.RpcClient()
        # SDK 0.24.1 may include authentication frames in native diagnostics.
        # Keep severity/source visible without persisting RPC bodies or secrets.
        self.rpc.setLogHandler(lambda level, source, line, message: print(
            f'[SDK diagnostic level={level} {Path(source).name}:{line}; RPC body omitted]',
            file=sys.stderr, flush=True))
        self.rpc.setRequestTimeout(1000)
        try:
            self.rpc.connect(config['host'], config['port'])
            require(self.rpc.hasConnected(), 'SDK connection failed')
            try:
                self.rpc.login(username, password)
            except Exception:
                raise Refused('SDK login failed; check local credentials') from None
            require(self.rpc.hasLogined(), 'SDK login failed')
            names = list(self.rpc.getRobotNames())
            name = config.get('robot_name')
            if not name:
                require(len(names) == 1, 'Multiple/no robots: explicitly configure robot_name')
                name = names[0]
            require(name in names, 'Configured robot_name is not present')
            self.name = name
            self.robot = self.rpc.getRobotInterface(name)
            self.state = self.robot.getRobotState()
            self.motion = self.robot.getMotionControl()
            self.config = self.robot.getRobotConfig()
            self.manage = self.robot.getRobotManage()
            self.runtime = self.rpc.getRuntimeMachine()
        except BaseException:
            self.close()
            raise

    def close(self):
        # No robot motion/power/runtime commands are issued during disconnect.
        try:
            self.rpc.disconnect()
        except Exception:
            pass

    def identity(self):
        return {
            'robot_name': self.name,
            'robot_type': self.config.getRobotType(),
            'robot_subtype': self.config.getRobotSubType(),
            'controller_version': self.rpc.getSystemInfo().getControlSoftwareFullVersion(),
            'joint_ids': list(self.state.getJointUniqueIds()),
            'dof': self.config.getDof(),
        }

    def limits(self):
        # Intersect hardware and active configured position limits.
        lows = [vector(self.config.getJointMinPositions(), 'hardware lower'),
                vector(self.config.getLimitJointMinPositions(), 'configured lower')]
        highs = [vector(self.config.getJointMaxPositions(), 'hardware upper'),
                 vector(self.config.getLimitJointMaxPositions(), 'configured upper')]
        return [max(v) for v in zip(*lows)], [min(v) for v in zip(*highs)]

    def snapshot(self):
        start = time.monotonic()
        s = {
            'q': vector(self.state.getJointPositions(), 'q'),
            'qd': vector(self.state.getJointSpeeds(), 'qd'),
            'tcp': vector(self.state.getTcpPose(), 'tcp'),
            'robot_mode': enum_name(self.state.getRobotModeType()),
            'safety_mode': enum_name(self.state.getSafetyModeType()),
            'control_mode': enum_name(self.manage.getRobotControlMode()),
            'joint_servo_modes': [enum_name(v) for v in self.state.getJointServoMode()],
            'joint_states': [enum_name(v) for v in self.state.getJointState()],
            'operational_mode': enum_name(self.manage.getOperationalMode()),
            'simulation': self.manage.isSimulationEnabled(),
            'freedrive': self.manage.isFreedriveEnabled(),
            'servo': self.motion.isServoModeEnabled(),
            'runtime': enum_name(self.runtime.getStatus()),
            'exec_id': self.motion.getExecId(),
            'queue': self.motion.getQueueSize(),
            'speed_fraction': self.motion.getSpeedFraction(),
            'within_limits': self.state.isWithinSafetyLimits(),
            'collision': self.state.isCollisionOccurred(),
        }
        s['read_seconds'] = time.monotonic() - start
        return s

    def move(self, target):
        # SDK order: positions [rad], acceleration [rad/s²], speed [rad/s], blend, duration.
        return self.motion.moveJoint(target, ACCEL, SPEED, 0.0, 0.0)

    def stop(self):
        return self.motion.stopJoint(STOP_ACCEL)


def check_identity(identity, config):
    require(config.get('allow_motion') is True, 'allow_motion is false; read-only inspection is available')
    require(identity['dof'] == 6 and norm_type(identity['robot_type']) == 's3',
            'Only the six-axis S3 is supported; do not relabel another robot')
    require(config.get('robot_name') == identity['robot_name'], 'robot_name must be explicitly pinned')
    require(bool(config.get('expected_controller_version')) and
            config['expected_controller_version'] == identity['controller_version'],
            'Controller version not pinned / changed; compatibility must be checked onsite')
    ids = config.get('expected_joint_ids')
    require(isinstance(ids, list) and len(ids) == 6 and ids == identity['joint_ids'],
            'Joint hardware identity not pinned / changed')


def check_state(s, idle):
    vector(s['q'], 'q'); vector(s['qd'], 'qd')
    require(math.isfinite(s['read_seconds']) and 0 <= s['read_seconds'] <= MAX_SNAPSHOT_SECONDS,
            'Slow state read: motion refused / aborted')
    require(s['robot_mode'] == 'Running', 'Robot must already be enabled by the onsite operator')
    require(s['safety_mode'] in ('Normal', 'ReducedMode'), 'Safety state does not permit this test')
    require(s['control_mode'] == 'Position',
            f"Aggregate control mode is {s['control_mode']}; joint servo modes are "
            f"{s.get('joint_servo_modes', 'not sampled')}. Aggregate Unknown requires investigation; "
            'it does not establish that the individual drives are outside position mode.')
    require(s['simulation'] is False and s['freedrive'] is False and s['servo'] is False,
            'Simulation/freedrive/servo state is incompatible')
    require(s['within_limits'] is True and s['collision'] is False, 'Controller safety check failed')
    require(s['runtime'] == 'Running',
            'SDK runtime must be prepared by onsite engineer; this script never starts a saved program/runtime')
    require(math.isfinite(s['speed_fraction']) and 0 < s['speed_fraction'] <= 1,
            'Speed fraction must be finite and within (0,1]')
    if idle:
        require(s['exec_id'] == -1 and s['queue'] == 0, 'Existing motion/queue: refuse to interfere')
        require(max(map(abs, s['qd'])) <= STILL_SPEED, 'Robot is not stationary')


def stationary(robot, log, seconds=2.0):
    start = time.monotonic()
    anchor = None
    latest = None
    while time.monotonic() - start < seconds:
        latest = robot.snapshot()
        check_state(latest, idle=True)
        if anchor is None:
            anchor = latest['q']
        require(max(abs(a-b) for a,b in zip(anchor, latest['q'])) <= START_DRIFT,
                'Robot drifted while checking stationary state')
        log.event('stationary_sample', **latest)
        time.sleep(0.1)
    require(latest is not None, 'No stationary samples')
    return latest


def execute_once(robot, initial, target, log, timeout=MOTION_TIMEOUT):
    """No retries: even a lost moveJoint response can mean the robot accepted it."""
    attempted = False
    started = time.monotonic()
    completed_since = None
    try:
        attempted = True
        result = robot.move(target)
        log.event('move_response', result=result)
        require(result == 0, f'moveJoint returned nonzero code {result}; no retry')
        while time.monotonic() - started < timeout:
            s = robot.snapshot()
            check_state(s, idle=False)
            log.event('motion_sample', **s)
            for i, q in enumerate(s['q']):
                low, high = sorted((initial[i], target[i]))
                require(low - 0.05*RAD <= q <= high + 0.05*RAD,
                        f'J{i+1} left the bounded motion envelope')
            require(max(map(abs, s['qd'])) <= MONITOR_SPEED, 'Measured joint speed exceeded monitor limit')
            arrived = (max(abs(a-b) for a,b in zip(s['q'], target)) <= POSITION_TOL
                       and max(map(abs, s['qd'])) <= STILL_SPEED
                       and s['queue'] == 0 and s['exec_id'] == -1)
            if arrived:
                if completed_since is None:
                    completed_since = time.monotonic()
                if time.monotonic() - completed_since >= 0.5:
                    log.event('success', final_q=s['q'], automatic_return=False)
                    return s
            else:
                completed_since = None
            time.sleep(0.1)
        raise Refused('Motion completion timed out; do not resend or automatically return')
    except BaseException:
        if attempted:
            # Best effort only. Disconnection/WSL pause can prevent this from reaching the controller.
            try:
                code = robot.stop()
                print(f'Best-effort stopJoint returned {code}; verify physically at the pendant.', file=sys.stderr)
                log.event('stop_attempt', result=code)
            except BaseException:
                print('STOP DELIVERY UNKNOWN: use the physical stop / emergency stop as needed.', file=sys.stderr)
        raise


@contextmanager
def local_lock():
    import fcntl
    with (ROOT / '.motion.lock').open('w') as f:
        try:
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise Refused('Another smoke command is running in this workspace') from None
        yield


def load_config(path):
    config = json.loads(path.read_text())
    host = config.get('host', '')
    try:
        address = ipaddress.ip_address(host)
    except ValueError:
        raise Refused('Set the actual controller IP in robot.local.json; no default IP is guessed') from None
    require(address.version == 4 and not address.is_multicast and not address.is_unspecified
            and not address.is_loopback, 'Use the actual unicast IPv4 controller address')
    require(type(config.get('port')) is int and 1 <= config['port'] <= 65535, 'Invalid TCP port')
    return config


def check_baseline(path, identity, config, q):
    """Local commissioning evidence, not authenticated evidence or a safety certificate."""
    require(path is not None, '--visible requires --baseline-log from a recent successful small hardware move')
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    identities = [r for r in rows if r.get('event') == 'identity']
    plans = [r for r in rows if r.get('event') == 'motion_plan']
    replies = [r for r in rows if r.get('event') == 'move_response']
    require(len(identities) == len(plans) == len(replies) == 1 and rows[-1].get('event') == 'success',
            'Baseline must contain exactly one successful hardware move')
    require(all(identities[0].get(k) == v for k,v in identity.items()), 'Baseline robot identity differs')
    plan = plans[0]
    require(plan.get('host') == config['host'] and plan.get('port') == config['port'], 'Baseline endpoint differs')
    require(plan.get('visible') is False and replies[0].get('result') == 0, 'Baseline must be a small move')
    checked_delta(plan['delta_deg'])
    elapsed = (datetime.now(timezone.utc) - datetime.fromisoformat(rows[-1]['time_utc'])).total_seconds()
    require(0 <= elapsed <= 1800, 'Baseline is older than 30 minutes; repeat inspection and small test')
    final = vector(rows[-1]['final_q'], 'baseline final position')
    require(max(abs(a-b) for a,b in zip(final,q)) <= 0.1*RAD,
            'Robot moved since baseline; repeat small test in current setup')


def motion_timeout(delta, fraction):
    require(math.isfinite(fraction) and 0 < fraction <= 1, 'Invalid speed fraction')
    timeout = max(MOTION_TIMEOUT, 10 + 2*abs(delta)/(SPEED/RAD*fraction))
    require(timeout <= 180, 'Estimated motion is too slow for this test; inspect controller settings, do not bypass')
    return timeout


def dry_run(delta, visible=False):
    q = [0.0, -0.4, 1.0, 0.0, 1.0, 0.0]  # Synthetic example, NEVER sent to hardware.
    target = target_for(q, delta, [-math.pi]*6, [math.pi]*6, visible)
    print(json.dumps({'mode': 'OFFLINE_ONLY', 'synthetic_q_deg': [x/RAD for x in q],
                      'synthetic_target_deg': [x/RAD for x in target], 'delta_J6_deg': delta,
                      'speed_deg_s': SPEED/RAD, 'accel_deg_s2': ACCEL/RAD,
                      'note': 'Real command derives all six targets from a fresh physical state; no homing/return.'},
                     ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['dry-run', 'network', 'inspect', 'move'])
    parser.add_argument('--config', type=Path, default=ROOT/'robot.local.json')
    parser.add_argument('--delta-deg', type=float, default=DELTA_DEG)
    parser.add_argument('--visible', action='store_true', help='Reviewed J6 motion up to 3 degrees; hardware baseline required')
    parser.add_argument('--baseline-log', type=Path, help='Successful small hardware move log from the last 30 minutes')
    args = parser.parse_args()
    checked_delta(args.delta_deg, args.visible)
    if args.command == 'dry-run':
        dry_run(args.delta_deg, args.visible)
        return 0
    config = load_config(args.config)
    if args.command == 'network':
        started = time.monotonic()
        with socket.create_connection((config['host'], config['port']), timeout=2):
            print(f'TCP reachable in {time.monotonic()-started:.3f}s; this does not verify robot readiness.')
        return 0
    if args.command == 'move':
        require(config.get('allow_motion') is True, 'Motion disabled in configuration')
        if args.visible:
            require(config.get('visible_motion_reviewed') is True, 'Visible motion requires onsite tool/cable/clearance review')
            require(args.baseline_log is not None, 'Run and verify a small hardware move first; provide --baseline-log')
        require(sys.stdin.isatty(), 'Motion requires an interactive onsite terminal; no --yes option')
    username = os.environ.get('AUBO_USER') or input('SDK username: ').strip()
    password = os.environ.get('AUBO_PASSWORD') or getpass.getpass('SDK password (not logged): ')
    log = Log(args.command)
    robot = None
    with local_lock():
        try:
            robot = Robot(config, username, password)
            identity = robot.identity()
            print(json.dumps(identity, ensure_ascii=False, indent=2))
            log.event('identity', **identity)
            if args.command == 'inspect':
                for _ in range(5):
                    s = robot.snapshot()
                    print(json.dumps({**s, 'q_deg': [q/RAD for q in s['q']]}, ensure_ascii=False))
                    log.event('read_only_sample', **s)
                    time.sleep(0.2)
                print('Read-only check completed. No power/mode/motion commands were sent.')
                return 0
            check_identity(identity, config)
            before = stationary(robot, log)
            if args.visible:
                check_baseline(args.baseline_log, identity, config, before['q'])
            low, high = robot.limits()
            target = target_for(before['q'], args.delta_deg, low, high, args.visible)
            print('Current joint degrees:', [round(x/RAD, 4) for x in before['q']])
            print('Target joint degrees: ', [round(x/RAD, 4) for x in target])
            print('Confirm physically: fixed base, clear swept volume/cables, correct payload/TCP,')
            print('working physical stop, onsite operator, no other program/client commanding this arm.')
            print('Ctrl+C is NOT an emergency stop; network loss may let the one submitted move finish.')
            phrase = f'MOVE J6 {args.delta_deg:+.3f} DEG'
            if args.visible:
                require(input('After onsite clearance/stop/tool review, type ONSITE CLEAR: ').strip() == 'ONSITE CLEAR',
                        'Onsite review not confirmed')
            require(input(f'Type exactly {phrase!r} to perform ONE move: ').strip() == phrase, 'Not confirmed')
            # The confirmation may take minutes: resample and refuse any moved start or changed identity.
            check_identity(robot.identity(), config)
            fresh = stationary(robot, log)
            require(max(abs(a-b) for a,b in zip(fresh['q'], before['q'])) <= START_DRIFT,
                    'Start changed during confirmation; rerun inspection')
            low, high = robot.limits()
            target = target_for(fresh['q'], args.delta_deg, low, high, args.visible)
            if args.visible:
                check_baseline(args.baseline_log, identity, config, fresh['q'])
            timeout = motion_timeout(args.delta_deg, fresh['speed_fraction'])
            log.event('motion_plan', host=config['host'], port=config['port'], visible=args.visible,
                      delta_deg=args.delta_deg, initial_q=fresh['q'], target_q=target, timeout_seconds=timeout)
            execute_once(robot, fresh['q'], target, log, timeout=timeout)
            print('ONE move verified complete. No automatic return. Inspect physically before any next command.')
            return 0
        finally:
            if robot is not None:
                robot.close()
            print('Log:', log.path)


if __name__ == '__main__':
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted. Check physical robot state; terminal interruption is not a safety stop.', file=sys.stderr)
        raise SystemExit(130)
    except Exception as error:
        print(f'REFUSED / FAILED: {error}', file=sys.stderr)
        raise SystemExit(1)
