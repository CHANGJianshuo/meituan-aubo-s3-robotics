"""Prepare exact trajectory suffix and a separate manual repeat; never move."""
from pathlib import Path
import copy, hashlib, json, math, shutil, sys, uuid
import cv2
import numpy as np

HERE=Path(__file__).resolve().parent
INTEG=HERE.parent
SOURCE=INTEG/'user_submit_20261014T211352Z/execution_package'
PLAYER=INTEG/'player_settle_v2'
CAPTURE=Path('/home/operator/meituan/eye_to_hand_calib/data/measurements/red_handle_20261014_104532/webui_settle_fix_20261014T2123')
sys.path.insert(0,str(PLAYER))
from trajectory import validate

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')

source=json.loads((SOURCE/'full_trajectory.json').read_text())
original_manifest=json.loads((SOURCE/'manifest.json').read_text())
state=json.loads((HERE/'robot_before.json').read_text())
assert state['start_error_deg'] < .05 and state['max_speed_deg_s'] <= .02 and state['max_drift_deg'] < .01
assert json.loads((INTEG/'obstacle_replay/acceptance.json').read_text())['passed']
baseline=json.loads((INTEG/'execution_package/full_trajectory.json').read_text())
assert source['segments'] == baseline['segments']

metadata=json.loads((CAPTURE/'capture.json').read_text())
frames=np.load(CAPTURE/'frames.npz')['color_rgb']
assert metadata['complete'] and len(frames)==60
assert metadata['camera']['device']['serial_number']=='348122071230'
roi=[660,430,820,565]
def masks(image):
    h=cv2.cvtColor(image[430:565,660:820],cv2.COLOR_RGB2HSV)
    common=(h[:,:,1]>100)&(h[:,:,2]>55)
    return [common&((h[:,:,0]<10)|(h[:,:,0]>170)),
            common&(h[:,:,0]>=15)&(h[:,:,0]<=40),
            common&(h[:,:,0]>40)&(h[:,:,0]<=80),
            common&(h[:,:,0]>=85)&(h[:,:,0]<=120)]
centers=[]
for frame in frames:
    y,x=np.nonzero(masks(frame)[0]);assert len(x)>2000
    centers.append([float(x.mean()+660),float(y.mean()+430)])
center=np.mean(centers,axis=0)
drift=float(np.max(np.linalg.norm(np.array(centers)-center,axis=1)))
assert drift<.2
reference=cv2.cvtColor(cv2.imread(str(HERE/'failed_run_end.png')),cv2.COLOR_BGR2RGB)
comparison={}
for name,a,b in zip(('red','yellow','green','blue'),masks(reference),masks(frames[-1])):
    ya,xa=np.nonzero(a);yb,xb=np.nonzero(b)
    iou=float((a&b).sum()/(a|b).sum())
    shift=float(np.linalg.norm([xa.mean()-xb.mean(),ya.mean()-yb.mean()]))
    assert min(a.sum(),b.sum())>300 and iou>.86 and shift<2.,(name,iou,shift)
    comparison[name]=dict(iou=iou,centroid_shift_px=shift)
scene=dict(passed=True,capture=str(CAPTURE),frames=60,
    stable_interval_s=(metadata['frames'][-1]['host_received_ns']-metadata['frames'][0]['host_received_ns'])/1e9,
    red_center_px=center.tolist(),red_max_centroid_deviation_px=drift,
    comparison_to_stopped_execution_frame=comparison,
    visually_reviewed='V1 remains inserted; red battery upright at trial-lift endpoint; neighboring batteries retained and unchanged',
    attachment_sensor_present=False,scene_roi=roi)
save(HERE/'scene_verification.json',scene)

packages={}
for label,offset in [('resume',3),('repeat',0)]:
    out=HERE/(label+'_package');out.mkdir()
    b=copy.deepcopy(source)
    b.update(plan_id=uuid.uuid4().hex,parent_plan_id=source['plan_id'])
    b['segments']=copy.deepcopy(source['segments'][offset:])
    stats=validate(b,original_manifest['captured_limits'])
    for name in ('run_hardware.py','trajectory.py','test_runner.py','run.sh'):
        shutil.copyfile(PLAYER/name,out/name)
    for name,selection in [('full_trajectory.json','full'),('task_one_trajectory.json','task')]:
        save(out/name,{**b,'selection':selection})
    m=copy.deepcopy(original_manifest)
    m.update(plan_id=b['plan_id'],parent_plan_id=source['plan_id'],
        captured_reference_q=state['samples'][-1]['q'] if offset else source['segments'][0]['points'][0]['positions'],
        task_start_q=b['segments'][0]['points'][0]['positions'],full_validation=stats,task_validation=stats,
        commissioning_scope='User-authorized exact remaining stages 4-10' if offset else 'Next full run; operator submission only after battery reset verification',
        player_revision='settle_v2',unchanged_player_checks=False,
        unchanged_motion_and_safety_thresholds=True,settle_timeout_seconds=5,
        physical_execution_validated=False,geometry_validation_pending=False,
        geometry_evidence=str(INTEG/'obstacle_replay/acceptance.json'))
    m['sha256']={n:sha(out/n) for n in ('full_trajectory.json','task_one_trajectory.json')}
    m['adapter_sha256']={n:sha(out/n) for n in ('run_hardware.py','trajectory.py','test_runner.py','run.sh')}
    save(out/'manifest.json',m)
    packages[label]=dict(package=str(out),plan_id=b['plan_id'],manifest_sha256=sha(out/'manifest.json'),
        stage_offset=offset,stage_count=len(b['segments']),nominal_duration_s=stats['nominal_duration_s'],
        trajectory_segments_exactly_unchanged=b['segments']==source['segments'][offset:])
save(HERE/'construction.json',dict(packages=packages,scene=scene,start_error_deg=state['start_error_deg'],
    original_motion_sha256=sha(SOURCE/'full_trajectory.json'),
    player_original_sha256=sha(SOURCE/'run_hardware.py'),player_patched_sha256=sha(PLAYER/'run_hardware.py'),
    additional_alignment_motion_required=False,simulation_reused_for_identical_segments=True))
print(json.dumps(dict(packages=packages,start_error_deg=state['start_error_deg'],scene=scene),ensure_ascii=False,indent=2))
