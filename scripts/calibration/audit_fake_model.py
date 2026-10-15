#!/usr/bin/env python3
"""Audit the archived S3 fake model against its TF logs, offline and without ROS.

Independent 4x4 matrix arithmetic deliberately does not import calibration.geometry.
Checks this capture's zero-joint hypothesis, not live robot states or calibration.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_EVIDENCE = ROOT / 'docs/planning/evidence/20261011_075828'
FILES = ('aubo_S3_fake_expanded.urdf', 'aubo_robot.srdf',
         'tf2_echo_world_to_base_link.txt', 'tf2_echo_base_link_to_ee_link.txt',
         'tf2_echo_world_to_ee_link.txt')
JOINTS = ('shoulder_joint', 'upperArm_joint', 'foreArm_joint',
          'wrist1_joint', 'wrist2_joint', 'wrist3_joint')


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def origin_matrix(joint):
    origin = joint.find('origin')
    xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0.0] * 3
    rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0.0] * 3
    if len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(v) for v in xyz + rpy):
        raise ValueError('invalid joint origin')
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    # URDF fixed-axis RPY: Rz(yaw) Ry(pitch) Rx(roll).
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, xyz[0]],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, xyz[1]],
            [-sp, cp*sr, cp*cr, xyz[2]], [0.0, 0.0, 0.0, 1.0]]


def zero_joint_chain(robot, source, target):
    by_child = {j.find('child').get('link'): j for j in robot.findall('joint')}
    chain, visited = [], set()
    while target != source:
        if target in visited or target not in by_child:
            raise ValueError('missing or cyclic model chain')
        visited.add(target)
        joint = by_child[target]
        if joint.get('type') not in ('fixed', 'revolute', 'continuous'):
            raise ValueError('unsupported joint type for this S3 audit')
        chain.append(joint)
        target = joint.find('parent').get('link')
    result = identity()
    for joint in reversed(chain):
        result = multiply(result, origin_matrix(joint))
    return result


def quaternion_matrix(q):
    x, y, z, w = q
    if not all(math.isfinite(v) for v in q) or abs(sum(v*v for v in q) - 1) > 0.003:
        raise ValueError('invalid rounded log quaternion')
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def log_comparison(text, expected):
    pattern = (r'At time ([0-9.]+)\s*- Translation: \[([^\]]+)\]\s*'
               r'- Rotation: in Quaternion \(xyzw\) \[([^\]]+)\]')
    samples = re.findall(pattern, text)
    passed, position_error, rotation_error = bool(samples), 0.0, 0.0
    for stamp, translation, quaternion in samples:
        xyz = [float(v) for v in translation.split(',')]
        q = [float(v) for v in quaternion.split(',')]
        if len(xyz) != 3 or len(q) != 4 or not all(math.isfinite(v) for v in xyz):
            raise ValueError('invalid TF log sample')
        rotation = quaternion_matrix(q)
        ep = max(abs(xyz[i] - expected[i][3]) for i in range(3))
        er = max(abs(rotation[i][j] - expected[i][j]) for i in range(3) for j in range(3))
        position_error, rotation_error = max(position_error, ep), max(rotation_error, er)
        # Translation and quaternion are printed to 3 decimals; matrix error is not angular accuracy.
        passed = passed and ep <= 0.000500001 and er <= 0.003
    return {'passed': passed, 'samples': len(samples),
            'first_stamp_text': samples[0][0] if samples else None,
            'last_stamp_text': samples[-1][0] if samples else None,
            'max_translation_component_difference_m': position_error,
            'max_rotation_matrix_component_difference': rotation_error}


def audit(directory):
    directory = Path(directory)
    contents = {name: (directory / name).read_bytes() for name in FILES}
    robot = ET.fromstring(contents[FILES[0]])
    srdf = ET.fromstring(contents[FILES[1]])
    checks = {}
    hardware = robot.find('ros2_control/hardware/plugin')
    initial = {j.get('name'): j.find("state_interface[@name='position']/param[@name='initial_value']")
               for j in robot.findall('ros2_control/joint')}
    checks['fake_zero_initial_state'] = bool(
        hardware is not None and hardware.text.strip() == 'mock_components/GenericSystem'
        and set(initial) == set(JOINTS)
        and all(v is not None and float(v.text) == 0.0 for v in initial.values()))
    movable = {j.get('name') for j in robot.findall('joint') if j.get('type') != 'fixed'}
    checks['s3_joint_names'] = movable == set(JOINTS)
    group = srdf.find("group[@name='manipulator']/chain")
    checks['srdf_base_and_tip'] = bool(group is not None and group.get('base_link') == 'base_link'
                                      and group.get('tip_link') == 'ee_link')
    world_base = zero_joint_chain(robot, 'world', 'base_link')
    base_ee = zero_joint_chain(robot, 'base_link', 'ee_link')
    wrist_ee = zero_joint_chain(robot, 'wrist3_Link', 'ee_link')
    expected_wrist_ee = [[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    checks['world_base_identity'] = max(abs(world_base[i][j] - identity()[i][j])
                                         for i in range(4) for j in range(4)) < 1e-12
    checks['wrist_ee_positive_quarter_turn'] = max(abs(wrist_ee[i][j] - expected_wrist_ee[i][j])
                                                   for i in range(4) for j in range(4)) < 1e-12
    logs = {FILES[2]: log_comparison(contents[FILES[2]].decode('utf-8'), world_base),
            FILES[3]: log_comparison(contents[FILES[3]].decode('utf-8'), base_ee),
            FILES[4]: log_comparison(contents[FILES[4]].decode('utf-8'), multiply(world_base, base_ee))}
    return {'all_passed': all(checks.values()) and all(v['passed'] for v in logs.values()),
            'evidence_kind': 'FAKE_RUNTIME_CAPTURE', 'real_calibration_verified': False,
            'motion_authorized': False, 'checks': checks, 'logs': logs,
            'sha256_lf_normalized': {k: hashlib.sha256(v.replace(b'\r\n', b'\n')).hexdigest()
                                     for k, v in contents.items()},
            'T_base_ee_q0': {'translation_m': [base_ee[i][3] for i in range(3)], 'matrix': base_ee},
            'T_wrist3_ee': {'matrix': wrist_ee},
            'scope': 'Archived fake capture only. No time-synchronized real joint states, camera/TCP or physical flange proof.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-dir', type=Path, default=DEFAULT_EVIDENCE)
    args = parser.parse_args()
    try:
        result = audit(args.evidence_dir)
    except (OSError, ValueError, ET.ParseError, AttributeError) as error:
        result = {'all_passed': False, 'error': str(error), 'motion_authorized': False,
                  'real_calibration_verified': False}
    print(json.dumps(result, ensure_ascii=True, indent=2, allow_nan=False))
    return 0 if result['all_passed'] else 2


if __name__ == '__main__':
    sys.exit(main())
