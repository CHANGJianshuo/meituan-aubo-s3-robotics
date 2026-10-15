#!/usr/bin/env python3
"""Read-only NUC state capture. Credentials are read from stdin and never saved."""
import argparse
from datetime import datetime, timezone
import json
import math
import random
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.0.2.20')
    parser.add_argument('--port', type=int, default=30004)
    parser.add_argument('--seconds', type=float, default=8.0)
    parser.add_argument('--kinematics', action='store_true',
                        help='Also query controller FK at virtual joint vectors; no motion')
    parser.add_argument('--adapter', type=Path,
                        default=Path('/home/robot/aubo_s3_nuc_smoke/hardware'))
    args = parser.parse_args()
    sys.path.insert(0, str(args.adapter))
    from j6_45 import Robot45
    from telemetry import quiet_native_auth
    credentials = json.loads(sys.stdin.readline())
    supplied_probes = credentials.pop('fk_queries', None)
    endpoint = {'host': args.host, 'port': args.port}
    robot = None
    try:
        with quiet_native_auth():
            robot = Robot45(endpoint, credentials['username'], credentials['password'])
        credentials.clear()
        result = dict(schema=1, read_only=True, captured_unix=time.time(),
                      created_utc=datetime.now(timezone.utc).isoformat(),
                      identity=robot.identity(), limits=robot.limits(),
                      config={**endpoint, 'robot_name': robot.name},
                      robot_configuration={}, samples=[])
        for name in ('getTcpOffset', 'getPayload', 'getJointMaxSpeeds',
                     'getLimitJointMaxSpeeds', 'getJointMaxAccelerations',
                     'getLimitJointMaxAccelerations'):
            result['robot_configuration'][name] = getattr(robot.config, name)()
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            start = time.time()
            state = robot.snapshot()
            state.update(read_started_unix=start, read_finished_unix=time.time())
            result['samples'].append(state)
            time.sleep(.05)
        qs = [s['q'] for s in result['samples']]
        result['max_joint_drift_deg'] = math.degrees(max(
            abs(a-b) for q in qs for a, b in zip(q, qs[-1])))
        result['max_joint_speed_deg_s'] = math.degrees(max(
            abs(v) for s in result['samples'] for v in s['qd']))
        result['ended_unix'] = time.time()
        if args.kinematics:
            result['kinematics'] = {
                'parameters_false': robot.config.getKinematicsParam(False),
                'parameters_true': robot.config.getKinematicsParam(True),
                'compensation_at_20C': robot.config.getKinematicsCompensate(20.0),
                'is_steady': robot.state.isSteady(),
                'virtual_fk_samples': [],
            }
            algorithm = robot.robot.getRobotAlgorithm()
            reference = qs[-1]
            probes = [reference, [0.0]*6]
            for index in range(6):
                for delta in [-.35, .35]:
                    q = reference.copy()
                    q[index] += delta
                    probes.append(q)
            rng = random.Random(714)
            probes.extend([[rng.uniform(-2, 2) for _ in range(6)] for _ in range(16)])
            if supplied_probes is not None:
                probes = supplied_probes
            for q in probes:
                pose, status = algorithm.forwardKinematics(q)
                result['kinematics']['virtual_fk_samples'].append(
                    dict(q=q, pose=pose, status=status))
        print(json.dumps(result, allow_nan=False))
    finally:
        credentials.clear()
        if robot is not None:
            robot.close()


if __name__ == '__main__':
    main()
