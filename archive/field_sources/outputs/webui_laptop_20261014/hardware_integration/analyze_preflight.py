import json,hashlib,math
from pathlib import Path
import numpy as np,cv2
HERE=Path(__file__).resolve().parent
p=Path((HERE/'final_before_capture.txt').read_text());meta=json.loads((p/'capture.json').read_text());a=np.load(p/'frames.npz')['color_rgb']
centers=[];angles=[];areas=[]
for rgb in a:
 hsv=cv2.cvtColor(rgb[430:565,660:820],cv2.COLOR_RGB2HSV)
 mask=((hsv[:,:,0]<10)|(hsv[:,:,0]>170))&(hsv[:,:,1]>100)&(hsv[:,:,2]>55)
 y,x=np.nonzero(mask);centers.append([x.mean()+660,y.mean()+430]);areas.append(int(mask.sum()))
 xs=np.arange(90,125);ys=np.array([np.flatnonzero(mask[:,xx])[0] for xx in xs]);angles.append(math.degrees(math.atan(np.polyfit(xs,ys,1)[0])))
center=np.mean(centers,axis=0);drift=float(np.max(np.linalg.norm(centers-center,axis=1)))
state=json.loads((HERE/'final_before_robot_state.json').read_text());bundle=json.loads((HERE/'execution_package/full_trajectory.json').read_text())
error=math.degrees(max(abs(x-y) for x,y in zip(state['samples'][-1]['q'],bundle['segments'][0]['points'][0]['positions'])))
sim=HERE/'obstacle_replay';result=json.loads((sim/'result.json').read_text());run=json.loads((sim/'run.json').read_text());accept=json.loads((sim/'acceptance.json').read_text())
source=HERE.parents[2]/'outputs/task1_real_scene_20261014/revision_01/physical_attempt_20261014T184851Z/half_time_01/candidate_commands.jsonl'
assert (sim/'commands.jsonl').read_bytes()==source.read_bytes()
assert run['success'] and run['shutdown_clean'] and accept['passed']
assert not any('neighbor_' in name for name in result['contacts']['total'])
assert error<.05 and drift<.2
report=dict(passed=True,capture=str(p),frames=len(a),stable_interval_s=(meta['frames'][-1]['host_received_ns']-meta['frames'][0]['host_received_ns'])/1e9,red_center_px=center.tolist(),max_centroid_deviation_px=drift,red_top_edge_angle_deg=float(np.mean(angles)),red_area_range=[min(areas),max(areas)],start_max_joint_error_deg=error,alignment_motion_required=False,robot_mode=state['samples'][-1]['robot_mode'],tool='Original V1 44 mm, visually unchanged at the same robot posture; TCP/payload checked by original player',simulation_acceptance=accept['passed'],neighbors_retained=True,neighbor_contact_records=0,exact_source_command_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),note='Red source restored within the prior proven image range; no absolute camera recalibration claim')
(HERE/'preflight-validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
