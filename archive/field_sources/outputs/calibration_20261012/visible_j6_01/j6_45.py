#!/usr/bin/env python3
"""A pinned, staged +45 degree commissioning plan. Default: read-only prepare.

No homing, power, brake, safety, simulation, operational-mode, runtime or payload
mutations. Two zero-blend moveJoint calls at most: +0.3 then cumulative +45.
No automatic retries or return. Software monitoring is not a safety controller.
"""
import argparse
from datetime import datetime, timezone
import getpass
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import uuid

import smoke

ROOT = Path(__file__).resolve().parent
RAD = math.pi / 180
VERSION = '0.25.6-alpha.1+c24e317c'
TOTAL_DEG = 45.0
PROBE_DEG = 0.3
VELOCITY = 0.2 * RAD
ACCELERATION = 0.5 * RAD
STOP_ACCELERATION = 1.0 * RAD
POSITION_TOL = 0.03 * RAD
DRIFT_TOL = 0.02 * RAD
ENVELOPE_TOL = 0.05 * RAD
SPEED_MONITOR = 0.4 * RAD
STILL_SPEED = 0.02 * RAD
PLAN_TTL = 600
require = smoke.require


class Robot45(smoke.Robot):
    def snapshot(self):
        start = time.monotonic()
        s = super().snapshot()
        fc = self.robot.getForceControl()
        s.update(force_control=fc.isFcEnabled(), soft_float=fc.isSoftFloatEnabled(),
                 backdrive=self.manage.isBackdriveEnabled(), power_on=self.state.isPowerOn())
        s['read_seconds'] = time.monotonic() - start
        return s

    def move(self, target):
        return self.motion.moveJoint(target, ACCELERATION, VELOCITY, 0.0, 0.0)

    def stop(self):
        return self.motion.stopJoint(STOP_ACCELERATION)


def validate_identity(identity, config):
    require(identity == config['expected_identity'], 'Robot identity changed; inspect, do not overwrite blindly')
    require(identity['dof'] == 6 and identity['robot_type'] == 'aubo_S3', 'Only this six-axis S3 is supported')
    require(identity['controller_version'] == VERSION, 'This profile is pinned to the inspected ARCS version')


def validate_state(s, idle=False, fraction=None, operation=None):
    smoke.vector(s['q'], 'q'); smoke.vector(s['qd'], 'qd')
    require(s['robot_mode'] == 'Running' and s['power_on'] is True, 'Robot is not already started')
    require(s['safety_mode'] in ('Normal', 'ReducedMode'), 'Unsafe or undefined controller safety state')
    require(s['simulation'] is False, 'Simulation is enabled; cannot count as a physical test')
    require(s['joint_states'] == ['Running'] * 6, 'At least one joint is not running')
    require(s['joint_servo_modes'] == ['Position'] * 6, 'All six drives must report position servo mode')
    # This controller reports aggregate Unknown while idle in real mode. Verify
    # individual drives AND the separate force/freedrive/backdrive interfaces.
    require(s['control_mode'] in ('Position', 'Unknown'), 'Incompatible aggregate control mode')
    for key in ('force_control', 'soft_float', 'backdrive', 'freedrive', 'servo', 'collision'):
        require(s[key] is False, f'{key} is active or unknown')
    require(s['within_limits'] is True, 'Controller limits check failed')
    # Official JSON-RPC example 7.2 is unblended moveJoint without runtime start.
    # Require Stopped here to exclude concurrently running saved programs.
    require(s['runtime'] == 'Stopped', 'A script runtime is active; refuse concurrent control')
    require(s['operational_mode'] in ('Manual', 'Automatic'), 'Unsupported operational mode')
    require(math.isfinite(s['read_seconds']) and 0 <= s['read_seconds'] <= 1, 'Slow feedback snapshot')
    require(math.isfinite(s['speed_fraction']) and .1 <= s['speed_fraction'] <= 1,
            'Speed fraction must be 10%-100%; do not increase it just to bypass this check')
    if fraction is not None:
        require(abs(s['speed_fraction'] - fraction) < 1e-6, 'Speed fraction changed during the test')
    if operation is not None:
        require(s['operational_mode'] == operation, 'Operational mode changed during the test')
    if idle:
        require(s['exec_id'] == -1 and s['queue'] == 0, 'Motion/queue already exists')
        require(max(map(abs, s['qd'])) <= STILL_SPEED, 'Robot is not stationary')


def validate_bare_flange(payload, tcp):
    require(isinstance(payload, (tuple, list)) and len(payload) >= 1,
            'Invalid payload response')
    require(isinstance(payload[0], (int, float)) and not isinstance(payload[0], bool)
            and math.isfinite(payload[0]) and abs(payload[0]) < 1e-6,
            'Bare-flange profile requires configured external payload 0 kg')
    tcp = smoke.vector(tcp, 'TCP offset')
    require(max(map(abs, tcp)) < 1e-6, 'Bare-flange profile requires zero TCP offset')


def targets(q, lower, upper):
    q = smoke.vector(q, 'initial q')
    lower = smoke.vector(lower, 'lower'); upper = smoke.vector(upper, 'upper')
    first = q.copy(); first[5] += PROBE_DEG * RAD
    final = q.copy(); final[5] += TOTAL_DEG * RAD
    for point in (q, first, final):
        for i, value in enumerate(point):
            require(lower[i] < upper[i] and lower[i] + 2 * RAD <= value <= upper[i] - 2 * RAD,
                    f'J{i+1} start/target violates two-degree position-limit margin')
    return first, final


def deadline_seconds(delta_deg, fraction):
    # At 10% this can take ~38 minutes: lower speed is not overridden.
    return max(30, 20 + 1.8 * abs(delta_deg) / (VELOCITY / RAD * fraction))


def stationary(robot, log, seconds=2.0, fraction=None, operation=None):
    end = time.monotonic() + seconds
    anchor = None
    while time.monotonic() < end:
        s = robot.snapshot()
        validate_state(s, idle=True, fraction=fraction, operation=operation)
        if anchor is None: anchor = s['q'].copy()
        require(max(abs(a-b) for a,b in zip(anchor,s['q'])) <= DRIFT_TOL, 'Start drifted during check')
        log.event('stationary', **s)
        time.sleep(.1)
    return s


def prepare(robot, config, log):
    identity = robot.identity(); validate_identity(identity, config)
    payload, tcp = robot.config.getPayload(), robot.config.getTcpOffset()
    validate_bare_flange(payload, tcp)
    require(robot.runtime.isTaskAlive(0) is False, 'Runtime main task is alive')
    s = stationary(robot, log)
    lo, hi = robot.limits(); first, final = targets(s['q'], lo, hi)
    plan = dict(schema=1, plan_id=uuid.uuid4().hex, created_utc=datetime.now(timezone.utc).isoformat(),
                host=config['host'], port=config['port'], identity=identity,
                initial_q=s['q'], probe_q=first, target_q=final,
                delta_deg=TOTAL_DEG, speed_deg_s=VELOCITY/RAD, acceleration_deg_s2=ACCELERATION/RAD,
                speed_fraction=s['speed_fraction'], operational_mode=s['operational_mode'],
                payload=payload, tcp_offset=tcp, simulation=False, automatic_return=False,
                nominal_motion_seconds=TOTAL_DEG/(VELOCITY/RAD*s['speed_fraction']))
    log.event('prepared', plan=plan)
    return plan


def validate_plan(plan, robot, config, log):
    require(plan['schema'] == 1 and plan['delta_deg'] == TOTAL_DEG, 'Wrong plan profile')
    require(len(plan['plan_id']) == 32 and all(c in '0123456789abcdef' for c in plan['plan_id']), 'Bad plan ID')
    age = (datetime.now(timezone.utc) - datetime.fromisoformat(plan['created_utc'])).total_seconds()
    require(0 <= age <= PLAN_TTL, 'Plan expired; generate a fresh read-only plan')
    require(plan['host'] == config['host'] and plan['port'] == config['port'], 'Endpoint changed')
    identity = robot.identity(); validate_identity(identity, config)
    require(identity == plan['identity'], 'Plan identity mismatch')
    require(plan['simulation'] is False and plan['automatic_return'] is False, 'Wrong plan semantics')
    require(plan['speed_deg_s'] == VELOCITY/RAD and plan['acceleration_deg_s2'] == ACCELERATION/RAD,
            'Plan speed/acceleration changed')
    validate_bare_flange(robot.config.getPayload(), robot.config.getTcpOffset())
    require(robot.runtime.isTaskAlive(0) is False, 'Runtime main task is alive')
    s = stationary(robot, log, fraction=plan['speed_fraction'], operation=plan['operational_mode'])
    initial = smoke.vector(plan['initial_q'], 'plan start')
    require(max(abs(a-b) for a,b in zip(s['q'], initial)) <= DRIFT_TOL, 'Start changed since planning')
    first, final = targets(initial, *robot.limits())
    require(smoke.vector(plan['probe_q'], 'probe') == first and
            smoke.vector(plan['target_q'], 'target') == final, 'Plan targets modified')
    return s


def execute_segment(robot, initial, target, plan, log, stage):
    """One command attempt, then bounded feedback monitoring. No retries."""
    start = time.monotonic()
    delta = math.degrees(target[5] - initial[5])
    deadline = start + deadline_seconds(delta, plan['speed_fraction'])
    arrived_since = None; next_progress = start; saw_motion = False; attempted = False
    try:
        log.event('command_intent', stage=stage, initial_q=initial, target_q=target)
        attempted = True
        code = robot.move(target)
        log.event('move_response', stage=stage, result=code)
        require(code == 0, f'moveJoint returned {code}; no retry')
        while time.monotonic() < deadline:
            s = robot.snapshot()
            validate_state(s, fraction=plan['speed_fraction'], operation=plan['operational_mode'])
            log.event('motion_sample', stage=stage, elapsed=time.monotonic()-start, **s)
            for i,value in enumerate(s['q']):
                lo,hi = sorted((initial[i], target[i]))
                require(lo-ENVELOPE_TOL <= value <= hi+ENVELOPE_TOL, f'J{i+1} escaped stage envelope')
                if i < 5:
                    require(abs(value-plan['initial_q'][i]) <= ENVELOPE_TOL, f'J{i+1} drifted from original start')
            require(max(map(abs,s['qd'])) <= SPEED_MONITOR, 'Measured speed exceeded monitoring bound')
            if abs(s['q'][5]-initial[5]) > .05*RAD: saw_motion=True
            arrived = (saw_motion and max(abs(a-b) for a,b in zip(s['q'],target)) <= POSITION_TOL
                       and max(map(abs,s['qd'])) <= STILL_SPEED and s['exec_id'] == -1 and s['queue'] == 0)
            if arrived:
                if arrived_since is None: arrived_since=time.monotonic()
                if time.monotonic()-arrived_since >= 1:
                    log.event('stage_verified', stage=stage, final_q=s['q'])
                    return s
            else: arrived_since=None
            if time.monotonic() >= next_progress:
                print(f'{stage}: actual J6={math.degrees(s["q"][5]):.4f} deg, target={math.degrees(target[5]):.4f}',flush=True)
                next_progress=time.monotonic()+5
            time.sleep(.1)
        raise smoke.Refused('Stage timed out; no retry or return')
    except BaseException:
        if attempted:
            try:
                result=robot.stop(); log.event('stop_attempt',result=result)
                print('Best-effort stopJoint sent. Check physical stop state.',flush=True)
            except BaseException:
                print('STOP DELIVERY UNKNOWN. Use onsite physical stop.',file=sys.stderr,flush=True)
        raise


def run_plan(robot, config, plan, log, ledger):
    require(config.get('allow_motion') is True, 'Execution is disabled in commission.local.json')
    require(config.get('bare_flange_confirmed') is True and config.get('onsite_safety_confirmed') is True,
            'This profile requires the reviewed bare-flange setup')
    start = validate_plan(plan, robot, config, log)
    ledger.mkdir(exist_ok=True)
    # Reserve before any send. A crash/lost reply must not allow a repeated +45.
    with (ledger/(plan['plan_id']+'.attempted')).open('x') as f:
        f.write(str(log.path)+'\n'); f.flush(); os.fsync(f.fileno())
    try:
        first = execute_segment(robot, start['q'], plan['probe_q'], plan, log, 'probe_0.3_deg')
        between = stationary(robot, log, seconds=2, fraction=plan['speed_fraction'], operation=plan['operational_mode'])
        require(max(abs(a-b) for a,b in zip(between['q'],first['q'])) <= DRIFT_TOL, 'Robot moved between stages')
        validate_identity(robot.identity(), config)
        validate_bare_flange(robot.config.getPayload(), robot.config.getTcpOffset())
        targets(plan['initial_q'], *robot.limits())
        execute_segment(robot, between['q'], plan['target_q'], plan, log, 'cumulative_45_deg')
        final = stationary(robot, log, seconds=2, fraction=plan['speed_fraction'], operation=plan['operational_mode'])
        require(max(abs(a-b) for a,b in zip(final['q'],plan['target_q'])) <= POSITION_TOL, 'Final hold differs from target')
    except BaseException:
        # Also cover errors between stages and during final hold. stopJoint is a
        # best-effort bounded stop, never a resend of the position command.
        try:log.event('sequence_stop_attempt',result=robot.stop())
        except BaseException:print('STOP DELIVERY UNKNOWN; check onsite.',file=sys.stderr,flush=True)
        raise
    report=dict(success=True, physical_robot=True, plan_id=plan['plan_id'],
                initial_q_deg=[math.degrees(x) for x in plan['initial_q']],
                final_q_deg=[math.degrees(x) for x in final['q']],
                measured_delta_deg=[math.degrees(b-a) for a,b in zip(plan['initial_q'], final['q'])],
                automatic_return=False)
    log.event('complete', **report)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=('prepare','execute'),nargs='?',default='prepare')
    p.add_argument('--config',type=Path,default=ROOT/'commission.local.json')
    p.add_argument('--plan',type=Path)
    args=p.parse_args()
    config=smoke.load_config(args.config)
    if args.command == 'execute':
        require(config.get('allow_motion') is True,'Execution is disabled; preparation remains read-only')
        require(args.plan is not None and sys.stdin.isatty(),'Execution needs a plan and an onsite interactive terminal')
    username=input('SDK username [aubo]: ').strip() or 'aubo'
    password=getpass.getpass('SDK password (not saved): ')
    log=smoke.Log('j6_45_'+args.command)
    robot=None
    with smoke.local_lock():
        try:
            robot=Robot45(config,username,password)
            if args.command == 'prepare':
                plan=prepare(robot,config,log)
                directory=ROOT/'plans';directory.mkdir(exist_ok=True)
                path=directory/('j6-45-'+plan['plan_id']+'.json')
                path.write_text(json.dumps(plan,indent=2))
                print('READ-ONLY PLAN:',path)
                print('Current degrees:',[round(math.degrees(x),4) for x in plan['initial_q']])
                print('Target degrees: ',[round(math.degrees(x),4) for x in plan['target_q']])
                print('Nominal move time:',round(plan['nominal_motion_seconds'],1),'seconds; no motion sent')
            else:
                plan=json.loads(args.plan.read_text());validate_plan(plan,robot,config,log)
                print('BARE FLANGE. J6 +0.3 then cumulative +45 degrees. No return.')
                print('Target degrees:',[round(math.degrees(x),4) for x in plan['target_q']])
                print('Keep all people clear; onsite physical stop ready. Ctrl+C is not a safety stop.')
                require(input('Type MOVE J6 +45 DEG BARE FLANGE: ').strip()=='MOVE J6 +45 DEG BARE FLANGE','Not confirmed')
                report=run_plan(robot,config,plan,log,ROOT/'attempts')
                report_path=log.path.with_suffix('.summary.json');report_path.write_text(json.dumps(report,indent=2))
                print(json.dumps(report,indent=2));print('Summary:',report_path)
            return 0
        except BaseException as e:
            log.event('failed',error=type(e).__name__+': '+str(e));raise
        finally:
            if robot is not None:robot.close()
            print('Log:',log.path)


if __name__ == '__main__':
    def interrupt(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt)
    try:raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted; inspect physical state.',file=sys.stderr);raise SystemExit(130)
    except Exception as e:
        print('REFUSED / FAILED:',e,file=sys.stderr);raise SystemExit(1)
