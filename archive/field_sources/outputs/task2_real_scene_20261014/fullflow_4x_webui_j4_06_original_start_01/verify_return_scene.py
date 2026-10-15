"""Verify the reset board and reuse geometry with the newly observed arm start."""
from pathlib import Path
import hashlib,json,shutil,sys
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parent
OLD=ROOT.parent/'fullflow_4x_webui_01'
import argparse,math
p=argparse.ArgumentParser();p.add_argument('--phase',choices=['before','after'],required=True);args=p.parse_args()
phase=args.phase
sys.path.insert(0,str(ROOT.parents[2]/'outputs/webui_laptop_20261014/source/src/mtc_cpp_mock_bringup'))
from mtc_webui.hardware_tasks import color_masks
record=json.loads((ROOT/'return_to_start'/('capture_'+phase)/'capture_record.json').read_text())
source=Path(record['capture_directory']);meta=json.loads((source/'capture.json').read_text())
frames=np.load(source/'frames.npz')['color_rgb'];reference=cv2.imread(str(OLD/'capture_03_ready/physical_initial.png'))
checks=[]
# The stationary blue V1 tool overlaps the right side of the blue battery.
# Compare its exposed top and outer left edge, not the same-color tool pixels.
for name,index,roi in [('A',0,[680,420,870,570]),('B',1,[680,420,870,570]),
                       ('D',2,[680,420,870,570]),('C_exposed_outer_edge',3,[740,430,786,490]) if phase=='before' else ('C',3,[680,420,870,570])]:
 a=color_masks(reference,roi,True)[index];ya,xa=np.nonzero(a);values=[]
 for rgb in frames:
  b=color_masks(cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),roi,True)[index];yb,xb=np.nonzero(b)
  assert min(len(xa),len(xb))>=300,(name,len(xa),len(xb))
  iou=float((a&b).sum()/(a|b).sum());shift=float(np.linalg.norm([xa.mean()-xb.mean(),ya.mean()-yb.mean()]))
  assert iou>=.86 and shift<=2.,(name,iou,shift)
  values.append(dict(iou=iou,shift_px=shift))
 checks.append(dict(object=name,roi=roi,minimum_iou=min(v['iou'] for v in values),maximum_shift_px=max(v['shift_px'] for v in values)))
frame=cv2.cvtColor(frames[-1],cv2.COLOR_RGB2BGR)
mask=np.zeros(frame.shape[:2],np.uint8);mask[460:635,350:980]=255;mask[420:575,680:870]=0
sift=cv2.SIFT_create(nfeatures=3000,contrastThreshold=.02)
k0,d0=sift.detectAndCompute(cv2.cvtColor(reference,cv2.COLOR_BGR2GRAY),mask)
k1,d1=sift.detectAndCompute(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),mask)
pairs=[m for m,n in cv2.BFMatcher().knnMatch(d0,d1,k=2) if m.distance<.7*n.distance]
assert len(pairs)>=15
H,inliers=cv2.findHomography(np.array([k0[m.queryIdx].pt for m in pairs]),np.array([k1[m.trainIdx].pt for m in pairs]),cv2.RANSAC,1.)
assert H is not None and inliers.sum()>=15
points=json.loads((OLD/'provisional_points.json').read_text())
centers=np.array([p['center_pixel'] for p in points['targets'].values()],float)
board_shift=float(np.linalg.norm(cv2.perspectiveTransform(centers[None,:,:],H)[0]-centers,axis=1).max())
assert board_shift<=2.
duration=(meta['frames'][-1]['host_received_ns']-meta['frames'][0]['host_received_ns'])/1e9
assert duration>=3 and len(frames)==30
q=record['telemetry'][-1]['robot']['data']['q'];drift=np.degrees(max(abs(a-b) for t in record['telemetry'] for a,b in zip(q,t['robot']['data']['q'])))
assert drift<=.05
report=dict(passed=True,motion_commands_sent=0,frame_count=len(frames),capture_duration_s=duration,
    color_checks=checks,blue_visibility='Unoccluded top and outer left edge; right side occluded by stationary V1 tool; user confirms blue unchanged',
    board_target_shift_max_px=board_shift,board_inliers=int(inliers.sum()),stationary_drift_deg=float(drift),start_q=q,
    user_reset_and_clearance_confirmed=True,source_capture_sha256=hashlib.sha256((ROOT/'return_to_start'/('capture_'+phase)/'capture_record.json').read_bytes()).hexdigest())
expected=(json.loads((ROOT/'return_to_start/return_trajectory.json').read_text())['segments'][0]['points'][0]['positions'] if phase=='before' else json.loads((Path(json.loads((ROOT/'hardware_config.json').read_text())['package'])/'manifest.json').read_text())['task_start_q'])
error=math.degrees(max(abs(a-b) for t in record['telemetry'] for a,b in zip(expected,t['robot']['data']['q'])))
assert error<=.05
report.update(phase=phase,start_reference_error_deg=error,original_task_unchanged=True)
if phase=='after':report['blue_visibility']='Entire battery; the tool has returned to the original start'
(ROOT/('scene_'+phase+'_check.json')).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
