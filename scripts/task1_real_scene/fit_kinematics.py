#!/usr/bin/env python3
"""Match a URDF to read-only controller FK queries, retaining its mesh frames.

This matches the controller's mathematical model; it is not a new physical
calibration. Reserved virtual configurations are never used by the optimizer.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

JOINTS = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint',
          'wrist1_joint', 'wrist2_joint', 'wrist3_joint']


def origins(root):
    result = []
    for name in JOINTS:
        joint = root.find(f"joint[@name='{name}']")
        if joint.find('axis').get('xyz') != '0 0 1':
            raise ValueError('Expected S3 revolute Z axes')
        origin = joint.find('origin')
        T = np.eye(4)
        T[:3, :3] = Rotation.from_euler('xyz', list(map(float, origin.get('rpy', '0 0 0').split()))).as_matrix()
        T[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
        result.append(T)
    return np.array(result)


def forward(origins_, qs):
    qs = np.atleast_2d(qs)
    T = np.tile(np.eye(4), (len(qs), 1, 1))
    for index, O in enumerate(origins_):
        Q = np.tile(np.eye(4), (len(qs), 1, 1))
        Q[:, :3, :3] = Rotation.from_euler('z', qs[:, index]).as_matrix()
        T = T @ O @ Q
    return T


def poses_to_matrices(poses):
    poses = np.asarray(poses)
    T = np.tile(np.eye(4), (len(poses), 1, 1))
    T[:, :3, :3] = Rotation.from_euler('xyz', poses[:, 3:]).as_matrix()
    T[:, :3, 3] = poses[:, :3]
    return T


def errors(predicted, expected):
    p = np.linalg.norm(predicted[:, :3, 3]-expected[:, :3, 3], axis=1)
    r = Rotation.from_matrix(np.transpose(expected[:, :3, :3], (0, 2, 1)) @ predicted[:, :3, :3]).magnitude()
    return dict(max_position_mm=float(p.max()*1000), rms_position_mm=float(np.sqrt(np.mean(p*p))*1000),
                max_rotation_deg=float(np.rad2deg(r.max())))


def fit(urdf, reference, output):
    root = ET.parse(urdf).getroot()
    original = origins(root)
    state = json.loads(reference.read_text())
    samples = state['kinematics']['virtual_fk_samples']
    if len(samples) < 30 or any(s['status'] != 0 for s in samples):
        raise ValueError('Need at least 30 successful controller FK samples')
    qs = np.array([s['q'] for s in samples])
    expected = poses_to_matrices([s['pose'] for s in samples])
    train = np.arange(len(samples)-8)
    holdout = np.arange(len(samples)-8, len(samples))

    def corrected(x):
        delta = np.tile(np.eye(4), (6, 1, 1))
        values = x.reshape(6, 6)
        delta[:, :3, :3] = Rotation.from_rotvec(values[:, :3]).as_matrix()
        delta[:, :3, 3] = values[:, 3:]
        return original @ delta

    def residual(x):
        predicted = forward(corrected(x), qs[train])
        rot = Rotation.from_matrix(np.transpose(expected[train, :3, :3], (0, 2, 1)) @ predicted[:, :3, :3]).as_rotvec()
        return np.r_[((predicted[:, :3, 3]-expected[train, :3, 3])).ravel(),
                     (rot*.2).ravel(), x*1e-8]

    bounds = np.tile([.05, .05, .05, .02, .02, .02], 6)
    result = least_squares(residual, np.zeros(36), bounds=(-bounds, bounds),
                           x_scale=np.tile([.01]*3+[.002]*3, 6),
                           ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=250)
    fitted = corrected(result.x)
    report = dict(kind='CONTROLLER_MODEL_MATCH_NOT_PHYSICAL_CALIBRATION',
                  reference_sha256=hashlib.sha256(reference.read_bytes()).hexdigest(),
                  original_urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
                  train_indices=train.tolist(), holdout_indices=holdout.tolist(),
                  nominal=errors(forward(original, qs), expected),
                  train=errors(forward(fitted, qs[train]), expected[train]),
                  holdout=errors(forward(fitted, qs[holdout]), expected[holdout]),
                  optimizer_success=bool(result.success), origin_corrections=result.x.reshape(6, 6).tolist())
    report['passed'] = report['holdout']['max_position_mm'] < .05 and report['holdout']['max_rotation_deg'] < .01
    output.mkdir(parents=True, exist_ok=True)
    (output/'kinematics_validation.json').write_text(json.dumps(report, indent=2)+'\n')
    if not report['passed']:
        raise ValueError('Controller-model holdout validation failed')
    for name, T in zip(JOINTS, fitted):
        origin = root.find(f"joint[@name='{name}']/origin")
        origin.set('xyz', ' '.join(format(x, '.17g') for x in T[:3, 3]))
        origin.set('rpy', ' '.join(format(x, '.17g') for x in Rotation.from_matrix(T[:3, :3]).as_euler('xyz')))
    ET.indent(root)
    ET.ElementTree(root).write(output/'controller_matched_arm.urdf', encoding='utf-8', xml_declaration=True)
    reread = origins(ET.parse(output/'controller_matched_arm.urdf').getroot())
    report['serialized_model'] = errors(forward(reread, qs), expected)
    (output/'kinematics_validation.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--urdf', type=Path, required=True)
    p.add_argument('--reference', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    print(json.dumps(fit(args.urdf, args.reference, args.output), indent=2))
