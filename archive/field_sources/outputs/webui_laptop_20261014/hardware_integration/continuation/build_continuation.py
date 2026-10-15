"""Explicit user-authorized continuation after a confirmed mid-transport stop.

The physical suffix begins at the freshly captured stationary q. Only the
remaining transport is a new zero-speed/acceleration quintic (4 seconds).
Stages 6-10 retain the successful half_time positions and timing exactly.
The full simulation includes a synthetic safe prefix to reproduce this stop.
"""
from pathlib import Path
import json,sys,copy,uuid,hashlib,shutil,subprocess
import numpy as np
import xml.etree.ElementTree as ET
HERE=Path(__file__).resolve().parent;INTEG=HERE.parent;WS=HERE.parents[3]
sys.path.insert(0,str(INTEG/'execution_package'));sys.path.insert(0,str(WS/'scripts/task1_real_scene'))
from trajectory import validate,dense
from fit_kinematics import forward,origins
source=json.loads((INTEG/'execution_package/full_trajectory.json').read_text())
state=json.loads((HERE/'stopped_state.json').read_text());q=state['samples'][-1]['q'];transport=source['segments'][4]
assert state['max_joint_speed_deg_s']<.02 and state['max_joint_drift_deg']<.01
assert state['samples'][-1]['exec_id']==-1 and state['samples'][-1]['queue']==0
qend=transport['points'][-1]['positions'];q0=transport['points'][0]['positions']
def pt(t,q):return dict(t=float(t),positions=q,velocities=[0.]*6,accelerations=[0.]*6)
resume={**transport,'points':[pt(0,q),pt(4,qend)]}
full=copy.deepcopy(source);full['segments'][4]['points']=[pt(0,q0),pt(7,q),pt(8,q),pt(12,qend)]
continuation={**source,'plan_id':uuid.uuid4().hex,'parent_plan_id':source['plan_id'],
 'segments':[resume]+copy.deepcopy(source['segments'][5:]),'commissioning_scope':__doc__}
stats=validate(continuation,state['limits']);validate(full,state['limits'])
assert max(stats['peak_velocity_deg_s'])<13 and max(stats['peak_acceleration_deg_s2'])<25
scenario=json.loads((INTEG/'obstacle_scenario.json').read_text())
_,samples=dense(resume,.002);F=forward(origins(ET.parse(scenario['arm_urdf']).getroot()),[s[0] for s in samples]);local=np.array([0,.045,.1105])
tcp=F[:,:3,3]+np.einsum('nij,j->ni',F[:,:3,:3],local)
assert min(tcp[:,2])>.30
(HERE/'replay_commands_input.jsonl').write_text(''.join(json.dumps(s,sort_keys=True,separators=(',',':'))+'\n' for s in full['segments']))
canonical=WS/'outputs/task1_real_scene_20261014/revision_01/physical_attempt_20261014T184851Z/one_shot_01/canonical_jsonl.cpp'
subprocess.run(['g++','-O2','-o',str(HERE/'canonical_jsonl'),str(canonical)],check=True)
with (HERE/'replay_commands_input.jsonl').open('rb') as i,(HERE/'replay_commands.jsonl').open('xb') as o:subprocess.run([str(HERE/'canonical_jsonl')],stdin=i,stdout=o,check=True)
scenario['replay_commands']=str(HERE/'replay_commands.jsonl');(HERE/'scenario.json').write_text(json.dumps(scenario,indent=2))
(HERE/'continuation_trajectory.json').write_text(json.dumps(continuation,indent=2)+'\n')
manifest=json.loads((INTEG/'execution_package/manifest.json').read_text());manifest.update(plan_id=continuation['plan_id'],parent_plan_id=source['plan_id'],captured_reference_q=q,task_start_q=q,commissioning_scope=__doc__,task_validation=stats,full_validation=stats)
out=HERE/'execution_package';out.mkdir()
for name in ['run_hardware.py','trajectory.py','test_runner.py','run.sh']:shutil.copyfile(INTEG/'execution_package'/name,out/name)
for name,sel in [('full_trajectory.json','full'),('task_one_trajectory.json','task')]:
 (out/name).write_text(json.dumps({**continuation,'selection':sel},indent=2)+'\n')
manifest['sha256']={n:hashlib.sha256((out/n).read_bytes()).hexdigest() for n in ['full_trajectory.json','task_one_trajectory.json']}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
report=dict(captured_start_q=q,remaining_stage_count=6,new_transport_duration_s=4.,nominal_tcp_height_range_m=[float(tcp[:,2].min()),float(tcp[:,2].max())],following_five_stages_exactly_unchanged=True,stats=stats,requires_full_simulation_replay=True,new_plan_id=continuation['plan_id'])
(HERE/'construction.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
