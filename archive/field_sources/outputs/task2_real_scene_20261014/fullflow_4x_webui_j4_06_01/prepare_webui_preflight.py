"""Verify fresh scene/robot evidence and enable only manual Web UI submission."""
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import urllib.request
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.parents[2]/'outputs/webui_laptop_20261014/source/src/mtc_cpp_mock_bringup'
sys.path.insert(0,str(SOURCE))
from mtc_webui.hardware_tasks import color_masks

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
settings=json.loads((ROOT/'hardware_config.json').read_text())
manifest_path=Path(settings['package'])/'manifest.json'
assert sha(manifest_path)==settings['manifest_sha256']
manifest=json.loads(manifest_path.read_text())
assert manifest['tracking_guard_deg']==[.5,.5,.5,.6,.5,.5]
assert json.loads((ROOT/'reset_scene_check.json').read_text())['passed']
assert json.loads((ROOT/'visual_review.json').read_text())['passed']
request=json.loads((ROOT/'request.json').read_text())
assert not request.get('status','').startswith('superseded'), 'User requested original trajectory; this revision is superseded'
assert request['user_submission_only'] and not request['automatic_submission']
assert request['user_reset_and_clearance_confirmed']
assert not Path(settings['claim_file']).exists()
reference=cv2.imread(str(ROOT/'capture_01_reset/physical_initial.png'))
preview=Path(settings['preview_dir'])/'latest_color.jpg'
checks=[dict(name='four_batteries_and_tool',scene_roi=[680,420,870,570],checked_color_indices=[0,1,2,3]),
        dict(name='blue_exposed_outer_edge',scene_roi=[740,430,786,490],checked_color_indices=[3]),
        dict(name='fixed_v1_tool',scene_roi=[788,398,828,474],checked_color_indices=[3])]
rows=[];seen=set();until=time.monotonic()+22
while time.monotonic()<until:
    before=preview.stat();raw=preview.read_bytes();after=preview.stat()
    if before.st_mtime_ns!=after.st_mtime_ns or before.st_size!=after.st_size:continue
    assert 0<=time.time()-after.st_mtime<2. and raw[:2]==b'\xff\xd8' and raw[-2:]==b'\xff\xd9'
    stamp=after.st_mtime_ns
    if stamp not in seen:
        seen.add(stamp);frame=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
        assert frame is not None and frame.shape==reference.shape
        values=[]
        for check in checks:
            pairs=list(zip(color_masks(reference,check['scene_roi'],True),color_masks(frame,check['scene_roi'],True)))
            for index in check['checked_color_indices']:
                a,b=pairs[index];ya,xa=np.nonzero(a);yb,xb=np.nonzero(b)
                assert min(len(xa),len(xb))>=300,(check['name'],index)
                value=dict(region=check['name'],color_index=index,iou=float((a&b).sum()/(a|b).sum()),
                    centroid_shift_px=float(np.linalg.norm([xa.mean()-xb.mean(),ya.mean()-yb.mean()])))
                assert value['iou']>=.86 and value['centroid_shift_px']<=2.,value
                values.append(value)
        rows.append(dict(source_stamp_ns=stamp,checks=values))
        if len(rows)>=10 and (stamp-rows[0]['source_stamp_ns'])/1e9>=8:break
    time.sleep(.1)
assert len(rows)>=10 and (rows[-1]['source_stamp_ns']-rows[0]['source_stamp_ns'])/1e9>=8

original=cv2.imread(str(ROOT.parent/'fullflow_4x_webui_01/capture_03_ready/physical_initial.png'))
board_mask=np.zeros(original.shape[:2],np.uint8);board_mask[460:635,350:980]=255;board_mask[420:575,680:870]=0
sift=cv2.SIFT_create(nfeatures=3000,contrastThreshold=.02)
k0,d0=sift.detectAndCompute(cv2.cvtColor(original,cv2.COLOR_BGR2GRAY),board_mask)
k1,d1=sift.detectAndCompute(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),board_mask)
pairs=[m for m,n in cv2.BFMatcher().knnMatch(d0,d1,k=2) if m.distance<.7*n.distance]
assert len(pairs)>=15
H,inliers=cv2.findHomography(np.array([k0[m.queryIdx].pt for m in pairs]),np.array([k1[m.trainIdx].pt for m in pairs]),cv2.RANSAC,1.)
assert H is not None and inliers.sum()>=15
geometry=json.loads((ROOT/'provisional_points.json').read_text())
centers=np.array([p['center_pixel'] for p in geometry['targets'].values()],dtype=float)
board_shift=float(np.linalg.norm(cv2.perspectiveTransform(centers[None,:,:],H)[0]-centers,axis=1).max())
assert board_shift<=2.
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
state=json.load(opener.open('http://127.0.0.1:8767/api/snapshot',timeout=5))
assert state['task'] is None and state['robot']['fresh'] and state['robot']['data']['stop_confirmed']
assert state['config']['plan_id']==manifest['plan_id'] and state['config']['tracking_guard_deg']==manifest['tracking_guard_deg']
assert state['config']['color_calibration']=='task2_scene_hsv_v1'
q=state['robot']['data']['q'];start_error=math.degrees(max(abs(a-b) for a,b in zip(q,manifest['task_start_q'])))
assert start_error<=.05
proof=dict(passed=True,created_utc=datetime.now(timezone.utc).isoformat(),color_calibration='task2_scene_hsv_v1',
    source_sha256=sha(SOURCE/'mtc_webui/hardware_tasks.py'),frames=rows,board_target_shift_max_px=board_shift,
    board_inliers=int(inliers.sum()),start_error_deg=start_error,geometry_thresholds_unchanged=True,motion_commands_sent=0)
with (ROOT/'ui_preflight_validation.json').open('x') as f:json.dump(proof,f,indent=2);f.write('\n')
evidence=dict(passed=True,plan_id=manifest['plan_id'],manifest_sha256=settings['manifest_sha256'],
    created_unix=time.time(),expires_unix=time.time()+600,reference_image=str((ROOT/'capture_01_reset/physical_initial.png').resolve()),
    reference_image_sha256=sha(ROOT/'capture_01_reset/physical_initial.png'),scene_checks=checks,
    reviewed_scene_sha256=sha(ROOT/'reset_scene_check.json'),visual_review_sha256=sha(ROOT/'visual_review.json'),
    preview_reference_validation_sha256=sha(ROOT/'ui_preflight_validation.json'),tracking_guard_deg=manifest['tracking_guard_deg'],
    automatic_retry=False,user_submission_only=True)
with Path(settings['preflight_file']).open('x') as f:json.dump(evidence,f,indent=2);f.write('\n')
print(json.dumps(dict(passed=True,frames=len(rows),board_target_shift_max_px=board_shift,start_error_deg=start_error),indent=2))
