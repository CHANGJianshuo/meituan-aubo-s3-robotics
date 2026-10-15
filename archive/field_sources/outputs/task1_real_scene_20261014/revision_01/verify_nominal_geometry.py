"""Offline model consistency only; does not measure physical clearances."""
import hashlib
import importlib.util
import itertools
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
REV = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'scripts/task1_real_scene'))
from fit_kinematics import origins, forward, poses_to_matrices, errors, JOINTS

run = REV/'replay_red_01'
scenario = json.loads((run/'scenario.json').read_text())
model = ET.parse(run/'robot.urdf').getroot()
commands = [json.loads(line) for line in (run/'commands.jsonl').read_text().splitlines()]
spec = importlib.util.spec_from_file_location('trajectory_math',
    ROOT.parent/'meituan_challenge_ws_with_gazebo_simulation/scripts/task_one_hardware/trajectory.py')
math_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(math_module)


def fixed(name):
    item = model.find(f"joint[@name='{name}']/origin")
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', np.fromstring(item.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    T[:3, 3] = np.fromstring(item.get('xyz', '0 0 0'), sep=' ')
    return T


mount = fixed('ee_fixed')@fixed('v1_mount')
base = fixed('world_mount')
assert np.allclose(base, np.eye(4), atol=1e-12)
assert set(scenario['initial_objects']) == {'A'}
world = ET.parse(run/'world.sdf').getroot()
batteries = [m.get('name') for m in world.iter('model') if m.get('name').startswith('battery_')]
assert batteries == ['battery_A']
source = np.array(scenario['initial_objects']['A'])
anchor = np.array(scenario['source_anchor']['base_xyz_m'])
assert np.allclose(anchor-source, [0, 0, .080], atol=1e-12)
assert abs(source[2]-scenario['targets']['T0'][2]) < 1e-12
R_body = Rotation.from_euler('z', scenario['object_yaws']['A']).as_matrix()
o = origins(model)
captured = json.loads((REV/'inputs_red_only/captured_state.json').read_text())['samples'][-1]
q0 = np.array([scenario['initial_joints'][j] for j in JOINTS])
start_error = np.rad2deg(np.max(np.abs(q0-np.array(commands[0]['points'][0]['positions']))))
assert start_error < .01
flange = forward(o, q0)[0]
tool = flange@mount
under = tool[:3, 3]+tool[:3, :3]@np.array([.0325, 0, .113])
n = tool[:3, 2]
gap = float(n@(under-anchor)/n[2]*1000)


def vertices(part):
    col = model.find(f"link[@name='v1_hook']/collision[@name='v1_{part}_collision']")
    center = np.fromstring(col.find('origin').get('xyz'), sep=' ')
    half = np.fromstring(col.find('geometry/box').get('size'), sep=' ')/2
    return center+np.array(list(itertools.product([-1, 1], repeat=3)))*half


segment = next(c for c in commands if c['primitive'] == 'hook_engage')
dt, samples = math_module.dense(segment, .01)
tools = forward(o, [s[0] for s in samples])@mount
clearances = {}
for part, half_xy, mode in [('lip', [.035, .013], 'above'), ('tongue', [.035, .040], 'below')]:
    v = vertices(part)
    w = np.einsum('nij,kj->nki', tools[:, :3, :3], v)+tools[:, None, :3, 3]
    b = (w-source)@R_body
    lo, hi = b.min(axis=1), b.max(axis=1)
    overlap = ((lo[:, :2] <= half_xy)&(hi[:, :2] >= -np.array(half_xy))).all(axis=1)
    separation = (.068-hi[:, 2]) if mode == 'above' else (lo[:, 2]-.050)
    distance = separation[overlap]*1000
    assert len(distance) and distance.min() > 3.5
    clearances[part] = dict(minimum_nominal_vertical_clearance_mm=float(distance.min()),
                           maximum_nominal_vertical_clearance_mm=float(distance.max()),
                           overlapping_samples=int(overlap.sum()), sample_interval_s=dt)

provenance = {path: hashlib.sha256(Path(path).read_bytes()).hexdigest() == sha
              for path, sha in scenario['provenance'].items()}
assert all(provenance.values())
assert (REV/'plan_red_01/commands.jsonl').read_bytes() == (run/'commands.jsonl').read_bytes()
assert (REV/'saved_trajectory/original_commands.jsonl').read_bytes() == (run/'commands.jsonl').read_bytes()
report = dict(model_checks_passed=True, physical_coordinates_verified=False,
              method='URDF FK at saved joint samples; conservative box-vertex Z gaps where XY bounding boxes overlap. This is model-only geometry.',
              world_equals_base_link=True, battery_models=batteries, supports_share_plane=True,
              initial_joint_error_deg=float(start_error), initial_gap_constraint_mm=gap,
              initial_controller_FK_match=errors(flange[None], poses_to_matrices([captured['tcp']])),
              insertion=clearances, immutable_input_hashes_match=provenance,
              commands_identical_across_plan_replay_export=True,
              commands_sha256=hashlib.sha256((run/'commands.jsonl').read_bytes()).hexdigest())
(REV/'nominal_geometry_validation.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps({k: v for k, v in report.items() if k != 'immutable_input_hashes_match'}, indent=2))
