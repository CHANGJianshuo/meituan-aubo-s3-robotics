#!/usr/bin/env python3
"""Compose an unloaded return from accepted approach paths; no new planning."""
import argparse
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import uuid
import xml.etree.ElementTree as ET

import numpy as np
from scipy.interpolate import BPoly
from scipy.spatial.transform import Rotation


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def slowed(segment, reverse=False, factor=2.):
    value = deepcopy(segment)
    duration = segment['points'][-1]['t']
    points = reversed(segment['points']) if reverse else segment['points']
    value['points'] = [dict(p, t=(duration-p['t'] if reverse else p['t'])*factor,
        velocities=[v*(-1 if reverse else 1)/factor for v in p['velocities']],
        accelerations=[a/factor**2 for a in p['accelerations']]) for p in points]
    return value


def prepare(current_source, original_source, math_dir, urdf, output):
    output.mkdir(parents=True, exist_ok=False)
    sources = []
    for folder in (current_source, original_source):
        accepted = json.loads((folder/'acceptance.json').read_text())
        validation = json.loads((folder/'validation.json').read_text())
        assert accepted['passed'] and validation['identical_command_replay_passed']
        path = folder/'task_two_trajectory.json'
        assert digest(path) == validation['sha256'][path.name]
        data = json.loads(path.read_text())
        assert data['order'] == 'ABC'
        assert data['segments'][0]['object'] == 'battery_A'
        assert data['segments'][0]['primitive'] == 'side_insert'
        sources.append(data)
    spec = importlib.util.spec_from_file_location('return_math', math_dir/'trajectory.py')
    tm = importlib.util.module_from_spec(spec);spec.loader.exec_module(tm)
    first = slowed(sources[0]['segments'][0])
    second = slowed(sources[1]['segments'][0], reverse=True)
    first.update(object='robot_unloaded', primitive='return_via_red_approach')
    second.update(object='robot_unloaded', primitive='return_to_original_start')
    bundle = dict(schema=1, joint_names=sources[0]['joint_names'], units=sources[0]['units'],
                  task='return_to_start', selection='full', plan_id=uuid.uuid4().hex,
                  segments=[first, second], simulation_only=True)
    stats = tm.validate(bundle)
    assert stats['maximum_join_difference_deg'] < .001
    assert second['points'][-1]['positions'] == sources[1]['segments'][0]['points'][0]['positions']
    # Validate the actual time-reversal polynomial, not only its endpoints.
    reversal_error = 0.
    bounds = []
    joint_samples = []
    for index, segment in enumerate(bundle['segments']):
        original = sources[index]['segments'][0]
        duration = original['points'][-1]['t']
        for t in np.linspace(0, duration, 1001):
            actual = tm.sample(segment, (duration-t if index else t)*2.)
            expected = tm.sample(original, t)
            for derivative, (a, b) in enumerate(zip(actual, expected)):
                sign = -1 if index and derivative == 1 else 1
                reversal_error = max(reversal_error, float(np.max(np.abs(np.array(a)-np.array(b)*sign/2.**derivative))))
        p = segment['points']
        poly = BPoly.from_derivatives([r['t'] for r in p], [[r[k] for k in ('positions','velocities','accelerations')] for r in p])
        bounds.append([float(np.rad2deg(np.abs(poly.derivative(n).c).max())) for n in (1,2,3)])
        joint_samples.extend(q for q, _, _ in tm.dense(segment)[1])
        for p in (segment['points'][0], segment['points'][-1]):
            assert p['velocities'] == [0.]*6 and p['accelerations'] == [0.]*6
    assert reversal_error < 1e-9
    assert np.all(np.array(bounds) <= [6.5, 6.25, 25.])
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'task1_real_scene'))
    from fit_kinematics import origins, forward
    fk = forward(origins(ET.parse(urdf).getroot()), joint_samples)
    angle = float(np.rad2deg(Rotation.from_matrix(fk[:, :3, :3] @ fk[0, :3, :3].T).magnitude().max()))
    assert angle < .1
    candidate = output/'return_trajectory.json'
    candidate.write_text(json.dumps(bundle, indent=2)+'\n')
    report = dict(passed=True, simulation_pending=True, no_new_path_planning=True,
        route='accepted current-to-red approach followed by reversed original red approach; unloaded throughout',
        time_scale=2., sources=[dict(path=str(p.resolve()),sha256=digest(p)) for p in
            [current_source/'task_two_trajectory.json',original_source/'task_two_trajectory.json']],
        source_stage=1, candidate_sha256=digest(candidate), statistics=stats,
        reversal_polynomial_error=reversal_error, bernstein_derivative_bounds_deg=bounds,
        fixed_orientation_max_deg=angle, original_task_unchanged=True)
    (output/'composition_validation.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('current-source','original-source','math-dir','urdf','output'):
        p.add_argument('--'+key, type=Path, required=True)
    prepare(**vars(p.parse_args()))
