"""Generate only the user's base -Y 3 mm adjustment, without hardware I/O."""
import hashlib
import json
from pathlib import Path
import sys
import uuid
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts/task1_real_scene'))
sys.path.insert(0, str(ROOT/'outputs/task1_real_scene_20261014/hardware_package'))
from fit_kinematics import origins, forward, poses_to_matrices, errors
from trajectory import dense, validate

state_path = HERE/'state_before_y_translation.json'
state = json.loads(state_path.read_text())
assert all(s['robot_mode']=='Running' and s['safety_mode']=='Normal' and
           s['exec_id']==-1 and s['queue']==0 and
           max(abs(v) for v in s['qd']) < np.deg2rad(.02) for s in state['samples'])
scenario = json.loads((HERE/'j6_adjustment_01/scenario.json').read_text())
model = origins(ET.parse(scenario['arm_urdf']).getroot())
q0 = np.array(state['samples'][-1]['q'])
F0 = forward(model, [q0])[0]
match = errors(np.array([F0]), poses_to_matrices([state['samples'][-1]['tcp']]))
assert match['max_position_mm'] < .05 and match['max_rotation_deg'] < .01
target = F0.copy()
target[1, 3] -= .003

def residual(q):
    actual = forward(model, [q])[0]
    return np.r_[actual[:3, 3]-target[:3, 3],
                 .2*Rotation.from_matrix(target[:3, :3].T@actual[:3, :3]).as_rotvec()]

solution = least_squares(residual, q0, bounds=(q0-np.deg2rad(1), q0+np.deg2rad(1)),
                         xtol=1e-13, ftol=1e-13, gtol=1e-13, max_nfev=100)
assert solution.success and np.max(np.abs(residual(solution.x))) < 1e-9
candidate = json.loads((HERE/'j6_adjustment_01/candidate.json').read_text())
candidate.update(plan_id=uuid.uuid4().hex,
                 source='User-requested translation: base -Y 3 mm, current orientation and height held',
                 source_sha256=hashlib.sha256(state_path.read_bytes()).hexdigest())
segment = candidate['segments'][0]
segment['primitive'] = 'cartesian_minus_y_3mm'
segment['points'] = [dict(t=t, positions=q.tolist(), velocities=[0.]*6, accelerations=[0.]*6)
                     for t,q in [(0.,q0),(4.,solution.x)]]
stats = validate(candidate, state['limits'])
_, samples = dense(segment, .01)
fk = forward(model, [row[0] for row in samples])
delta = fk[-1,:3,3]-fk[0,:3,3]
xz = float(np.max(np.abs(fk[:,[0,2],3]-fk[0,[0,2],3]))*1000)
angles = Rotation.from_matrix(np.transpose(fk[:,:3,:3],(0,2,1))@fk[0,:3,:3]).magnitude()
assert np.max(np.abs(delta-[0,-.003,0])) < 1e-6
assert xz < .05 and np.max(angles) < np.deg2rad(.01)
assert np.all(np.diff(fk[:,1,3]) <= 1e-8)
scenario['initial_joints'] = dict(zip(candidate['joint_names'],q0.tolist()))
report = dict(user_request='再向y轴负方向平移3mm左右', frame='physical robot base',
              target_delta_mm=[0,-3,0], duration_s=4,
              preserve_current_orientation=True, preserve_current_height=True,
              controller_model_agreement=match,
              commanded_delta_mm=(delta*1000).tolist(), maximum_xz_deviation_mm=xz,
              maximum_orientation_change_deg=float(np.rad2deg(np.max(angles))),
              joint_change_deg=np.rad2deg(solution.x-q0).tolist(),
              source_state=str(state_path),source_state_sha256=hashlib.sha256(state_path.read_bytes()).hexdigest(),
              physical_execution_status='PENDING_SIMULATION_VALIDATION',
              source_scene_geometry_status='Approximate; no claim of independent physical alignment validation',
              stats=stats)
out = HERE/'y_translation_01'
out.mkdir()
for name,value in [('candidate.json',candidate),('scenario.json',scenario),('requested_translation.json',report)]:
    (out/name).write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n')
print(json.dumps(report,indent=2,ensure_ascii=False))
