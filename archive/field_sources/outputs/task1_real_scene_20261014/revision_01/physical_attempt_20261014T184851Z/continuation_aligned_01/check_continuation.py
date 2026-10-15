"""Check saved continuation against the onsite-approved start and directions."""
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[4]
sys.path.insert(0,str(ROOT/'scripts/task1_real_scene'))
sys.path.insert(0,str(ROOT/'outputs/task1_real_scene_20261014/hardware_package'))
from fit_kinematics import origins, forward
from trajectory import validate, dense

candidate=HERE/'saved_trajectory/task_one_trajectory.json'
bundle=json.loads(candidate.read_text())
cfg=json.loads((HERE/'scenario.json').read_text())
state=json.loads((HERE.parent/'state_after_y_translation.json').read_text())
q0=np.array(state['samples'][-1]['q'])
model=origins(ET.parse(cfg['arm_urdf']).getroot())
validation=json.loads((HERE/'saved_trajectory/validation.json').read_text())
assert validation['simulation_success'] and validation['identical_command_replay_passed']
stats=validate(bundle,state['limits'])
hold,insert,trial=bundle['segments'][:3]
assert [s['primitive'] for s in [hold,insert,trial]]==['side_insert','hook_engage','test_lift']
assert all(np.max(np.abs(np.array(p['positions'])-q0))<1e-8 and
           max(map(abs,p['velocities']))==0 and max(map(abs,p['accelerations']))==0 for p in hold['points'])
direction=np.r_[cfg['acquire_directions']['A'],0.]
results=[]
for segment,requested in [(insert,direction*.060),(trial,np.array([0,0,.0285]))]:
 _,path=dense(segment,.01)
 F=forward(model,[row[0] for row in path])
 delta=F[:,:3,3]-F[0,:3,3]
 progress=delta@requested/np.dot(requested,requested)
 deviation=delta-np.outer(progress,requested)
 rotation=Rotation.from_matrix(np.transpose(F[:,:3,:3],(0,2,1))@F[0,:3,:3]).magnitude()
 assert np.linalg.norm(delta[-1]-requested)<.0001
 assert np.max(np.linalg.norm(deviation,axis=1))<.0001
 assert np.min(np.diff(progress))>-.0001 and progress.min()>-.0001 and progress.max()<1.001
 assert float(np.max(rotation))<np.deg2rad(.01)
 results.append(dict(primitive=segment['primitive'],samples=len(path),duration_s=segment['points'][-1]['t'],
  requested_delta_base_mm=(requested*1000).tolist(),actual_delta_base_mm=(delta[-1]*1000).tolist(),
  maximum_line_deviation_mm=float(np.max(np.linalg.norm(deviation,axis=1))*1000),
  maximum_orientation_change_deg=float(np.rad2deg(np.max(rotation)))))
original_cfg=json.loads((HERE.parent.parent/'inputs_red_only/scenario.json').read_text())
assert cfg['targets']==original_cfg['targets'] and cfg['table_z']==original_cfg['table_z']
result=dict(passed=True,candidate_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),
 user_confirmed_alignment=True,approach_is_stationary_hold=True,source_start_unchanged=True,
 target_and_support_unchanged=True,segments=results,stats=stats,
 measured_physical_clearances=False,physical_attachment_check_required_after_test_lift=True)
(HERE/'continuation_checks.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
