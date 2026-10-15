"""Offline construction: smooth rest-to-rest pre-seat suffix; no real motion."""
from pathlib import Path
import copy,hashlib,json,math,shutil,subprocess,sys,uuid
import xml.etree.ElementTree as ET
import numpy as np
import cv2
HERE=Path(__file__).resolve().parent;INTEG=HERE.parent;WS=HERE.parents[3]
PLAYER=INTEG/'player_tracking_v3'
SOURCE=INTEG/'settle_fix_20261014T212353Z/repeat_package'
sys.path.insert(0,str(PLAYER));sys.path.insert(0,str(WS/'scripts/task1_real_scene'))
from trajectory import validate,dense
from fit_kinematics import forward,origins

def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def point(t,q):return dict(t=float(t),positions=q,velocities=[0.]*6,accelerations=[0.]*6)

state=json.loads((HERE/'robot_before.json').read_text());q=state['samples'][-1]['q']
assert state['max_joint_speed_deg_s']<=.02 and state['max_joint_drift_deg']<.01
source=json.loads((SOURCE/'full_trajectory.json').read_text());manifest=json.loads((SOURCE/'manifest.json').read_text())
original=source['segments'][5];q0=original['points'][0]['positions'];qend=original['points'][-1]['positions']
resume={**original,'points':[point(0,q),point(9,qend)]}
full_sim=copy.deepcopy(source)
full_sim['segments'][5]['points']=[point(0,q0),point(6,q),point(7,q),point(16,qend)]
validate(full_sim,manifest['captured_limits'])
scenario=json.loads((INTEG/'obstacle_scenario.json').read_text())
links=origins(ET.parse(scenario['arm_urdf']).getroot())
dt,samples=dense(resume,.002);qs=np.array([x[0] for x in samples])
F=forward(links,qs);tcp=F[:,:3,3]+np.einsum('nij,j->ni',F[:,:3,:3],[0,.045,.1105])
assert tcp[:,2].min()>=.0472
assert np.max(np.linalg.norm(tcp[:,:2]-tcp[-1,:2],axis=1))<.05
delta_rotation=np.einsum('nij,njk->nik',np.transpose(F[:,:3,:3],(0,2,1)),np.repeat(F[-1:,:3,:3],len(F),axis=0))
angles=np.arccos(np.clip((np.trace(delta_rotation,axis1=1,axis2=2)-1)/2,-1,1))
assert np.degrees(angles).max()<.05

# Evaluate the isolated J3 lag allowance against the nominal pre-seat poses.
# This is a mathematical kinematic sensitivity check, not measured dynamics.
_,base_samples=dense(original,.01);base_q=np.array([x[0] for x in base_samples])
nominal=forward(links,base_q);lag_q=base_q.copy();lag_q[:,2]-=math.radians(.8)
lag=forward(links,lag_q)
nom_tcp=nominal[:,:3,3]+np.einsum('nij,j->ni',nominal[:,:3,:3],[0,.045,.1105])
lag_tcp=lag[:,:3,3]+np.einsum('nij,j->ni',lag[:,:3,:3],[0,.045,.1105])
lag_delta=lag_tcp-nom_tcp
assert lag_delta[:,2].min()>0
geometry=dict(resume_duration_s=9.,tcp_min_height_m=float(tcp[:,2].min()),
    tcp_max_xy_excursion_m=float(np.max(np.linalg.norm(tcp[:,:2]-tcp[-1,:2],axis=1))),
    tool_orientation_change_deg=float(np.degrees(angles).max()),
    isolated_j3_lag_08_deg_tcp_z_delta_mm_range=[float(lag_delta[:,2].min()*1000),float(lag_delta[:,2].max()*1000)],
    isolated_j3_lag_08_deg_max_tcp_displacement_mm=float(np.max(np.linalg.norm(lag_delta,axis=1))*1000),
    interpretation='Only positive-direction J3 lag during pre_seat is relaxed; modeled TCP stays above nominal, other guards unchanged')
save(HERE/'kinematic_checks.json',geometry)

capture=Path((HERE/'capture_path.txt').read_text());meta=json.loads((capture/'capture.json').read_text());frames=np.load(capture/'frames.npz')['color_rgb']
assert meta['complete'] and len(frames)==60
centers=[];areas=[]
for rgb in frames:
    h=cv2.cvtColor(rgb[300:425,465:565],cv2.COLOR_RGB2HSV)
    m=((h[:,:,0]<10)|(h[:,:,0]>170))&(h[:,:,1]>100)&(h[:,:,2]>55)
    y,x=np.nonzero(m);assert len(x)>1500
    centers.append([float(x.mean()+465),float(y.mean()+300)]);areas.append(len(x))
mean=np.mean(centers,axis=0);drift=float(np.max(np.linalg.norm(np.array(centers)-mean,axis=1)))
assert drift<.3
scene=dict(passed=True,capture=str(capture),frames=len(frames),
    stable_interval_s=(meta['frames'][-1]['host_received_ns']-meta['frames'][0]['host_received_ns'])/1e9,
    red_center_px=mean.tolist(),max_red_drift_px=drift,red_area_range=[min(areas),max(areas)],
    scene_roi=[460,300,820,565],checked_color_indices=[0,1,2,3],
    reviewed='Red upright and suspended on original V1 above T0; neighboring three batteries retained; no attachment sensor claim')
save(HERE/'scene_verification.json',scene)

(HERE/'replay_commands_input.jsonl').write_text(''.join(json.dumps(s,sort_keys=True,separators=(',',':'))+'\n' for s in full_sim['segments']))
with (HERE/'replay_commands_input.jsonl').open('rb') as src,(HERE/'replay_commands.jsonl').open('xb') as dst:
    subprocess.run([str(INTEG/'continuation/canonical_jsonl')],stdin=src,stdout=dst,check=True)
scenario['replay_commands']=str(HERE/'replay_commands.jsonl');save(HERE/'scenario.json',scenario)
packages={}
for label,segments,offset in [('resume',[resume]+copy.deepcopy(source['segments'][6:]),5),('repeat',copy.deepcopy(source['segments']),0)]:
    out=HERE/(label+'_package');out.mkdir()
    b={**source,'plan_id':uuid.uuid4().hex,'parent_plan_id':source['plan_id'],'segments':segments}
    stats=validate(b,manifest['captured_limits'])
    assert max(stats['peak_velocity_deg_s'])<13 and max(stats['peak_acceleration_deg_s2'])<25
    for name in ('run_hardware.py','trajectory.py','test_runner.py','run.sh'):shutil.copyfile(PLAYER/name,out/name)
    for name,selection in [('full_trajectory.json','full'),('task_one_trajectory.json','task')]:save(out/name,{**b,'selection':selection})
    m=copy.deepcopy(manifest)
    m.update(plan_id=b['plan_id'],parent_plan_id=source['plan_id'],task_start_q=segments[0]['points'][0]['positions'],
        captured_reference_q=q if offset else segments[0]['points'][0]['positions'],
        task_validation=stats,full_validation=stats,player_revision='tracking_v3',
        commissioning_scope='Manually submit remaining stages 6-10 after verified 9 s rest-to-rest bridge' if offset else 'Future full run; requires new start-scene verification',
        unchanged_motion_and_safety_thresholds=False,controller_safety_settings_unchanged=True,
        tracking_policy=dict(default_deg=.5,pre_seat_positive_direction_j3_lag_deg=.8),
        geometry_validation_pending=bool(offset),geometry_evidence=str(HERE/'replay/acceptance.json') if offset else str(INTEG/'obstacle_replay/acceptance.json'))
    m['sha256']={n:sha(out/n) for n in ('full_trajectory.json','task_one_trajectory.json')}
    m['adapter_sha256']={n:sha(out/n) for n in ('run_hardware.py','trajectory.py','test_runner.py','run.sh')}
    save(out/'manifest.json',m)
    packages[label]=dict(package=str(out),plan_id=b['plan_id'],manifest_sha256=sha(out/'manifest.json'),stage_offset=offset,stage_count=len(segments),stats=stats)
save(HERE/'construction.json',dict(packages=packages,geometry=geometry,scene=scene,
    final_four_stages_exactly_unchanged=True,simulation_pending=True,
    stopped_q=q,source_plan_id=source['plan_id']))
print(json.dumps(dict(geometry=geometry,scene=scene,packages={k:{a:b for a,b in v.items() if a!='stats'} for k,v in packages.items()}),ensure_ascii=False,indent=2))
