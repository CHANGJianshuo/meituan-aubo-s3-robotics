"""Add a validated approach to the exact nine stages proven on the real robot."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[4]
sys.path.insert(0,str(ROOT/'scripts/task1_real_scene'))
sys.path.insert(0,str(ROOT/'outputs/task1_real_scene_20261014/hardware_package'))
from fit_kinematics import forward,origins
from trajectory import validate

status=json.loads((HERE/'plan/run.json').read_text());assert status['success'] and status['shutdown_clean']
assert json.loads((HERE/'plan/acceptance.json').read_text())['passed']
source=HERE.parent/'continuation_aligned_01/saved_trajectory/task_one_trajectory.json'
proven=json.loads(source.read_text())
first=json.loads((HERE/'plan/commands.jsonl').read_text().splitlines()[0])
original=copy.deepcopy(first)
target=proven['segments'][1]['points'][0]['positions']
delta=np.array(target)-first['points'][-1]['positions']
assert np.max(np.abs(delta))<np.deg2rad(.01)
assert max(map(abs,first['points'][-1]['velocities']))<1e-12
# Retiming supplies a nonzero terminal acceleration despite zero terminal
# velocity. Preserve both derivative arrays and every timestamp exactly.
first['points'][-1]['positions']=target
cfg=json.loads((HERE/'scenario.json').read_text())
F=forward(origins(ET.parse(cfg['arm_urdf']).getroot()),[original['points'][-1]['positions'],target])
position_change=float(np.linalg.norm(F[1,:3,3]-F[0,:3,3])*1000)
assert position_change<.1
commands=[first]+proven['segments'][1:]
bundle={**proven,'segments':commands}
stats=validate(bundle,cfg['controller_joint_limits_rad'])
q0=[cfg['initial_joints'][j] for j in bundle['joint_names']]
assert np.max(np.abs(np.array(first['points'][0]['positions'])-q0))<1e-8
(HERE/'combined_commands_input.jsonl').write_text(''.join(json.dumps(s,sort_keys=True,separators=(',',':'))+'\n' for s in commands))
report=dict(passed=True,proven_source=str(source),proven_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
 approach_source=str(HERE/'plan/commands.jsonl'),
 approach_endpoint_change_deg=np.rad2deg(delta).tolist(),approach_endpoint_translation_change_mm=position_change,
 approach_derivatives_and_timestamps_unchanged=True,
 approach_endpoint_reason='End directly at exact onsite-approved insertion-start joints; final waypoint only, below 0.01 deg and 0.1 mm; complete combined path must pass replay before use',
 all_nine_proven_stages_reused_without_changes=True,stats=stats,combined_replay_pending=True)
(HERE/'composition.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
