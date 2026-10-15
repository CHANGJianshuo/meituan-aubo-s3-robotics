"""Bind like-for-like live JPEG preview to the reviewed RGB-D capture."""
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import time
import sys
import urllib.request
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.parents[2]/'outputs/webui_laptop_20261014/source/src/mtc_cpp_mock_bringup'
sys.path.insert(0,str(SOURCE))
from mtc_webui.hardware_tasks import color_masks
request=json.loads((ROOT/'request.json').read_text())
assert request['task_user_submission_only'] and request['automatic_task_submission'] is False
assert request['user_reports_manual_reset']
assert json.loads((ROOT/'previous_success_check.json').read_text())['passed']
assert json.loads((ROOT/'visual_review.json').read_text())['passed']
assert not (ROOT/'webui_claim.json').exists()
settings=json.loads((ROOT/'hardware_config.json').read_text())
review=json.loads((ROOT/'reset_scene_check.json').read_text())
assert review['passed'] and review['start_reference_error_deg']<=.05
manifest=json.loads((Path(settings['package'])/'manifest.json').read_text())
assert manifest['tracking_guard_deg']==[.5,.5,.5,.6,.5,.5]
assert hashlib.sha256((Path(settings['package'])/'manifest.json').read_bytes()).hexdigest()==settings['manifest_sha256']
preview=Path(settings['preview_dir'])/'latest_color.jpg'
png=cv2.imread(str(ROOT/'capture_reset/physical_initial.png'))
checks=[dict(name='all_four_batteries',scene_roi=[680,420,870,570],checked_color_indices=[0,1,2,3]),
        dict(name='fixed_v1_tool',scene_roi=[380,305,465,415],checked_color_indices=[3])]

def read_preview():
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        before=preview.stat();raw=preview.read_bytes();after=preview.stat()
        valid=(before.st_mtime_ns==after.st_mtime_ns and before.st_size==after.st_size
            and 0<=time.time()-after.st_mtime<2. and raw[:2]==b'\xff\xd8' and raw[-2:]==b'\xff\xd9')
        if valid:
            image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
            assert image is not None and image.shape==png.shape
            return after.st_mtime_ns,raw,image
        time.sleep(.1)
    raise RuntimeError('No fresh complete preview frame within five seconds')

def calibrated(image):
    h,s,v=cv2.split(cv2.cvtColor(image[420:570,680:870],cv2.COLOR_BGR2HSV))
    return [((h<=10)|(h>170))&(s>95)&(v>65),
            (h>=16)&(h<=38)&(s>95)&(v>65),
            (h>=85)&(h<=130)&(s>95)&(v>65),
            (h>=39)&(h<=84)&(s>50)&(v>65)]

def ui_masks(image,roi):
    return color_masks(image,roi,sequence=True)

def compare(a,b):
    ya,xa=np.nonzero(a);yb,xb=np.nonzero(b)
    assert min(len(xa),len(xb))>=300
    row=dict(iou=float((a&b).sum()/(a|b).sum()),
             shift_px=float(np.linalg.norm([xa.mean()-xb.mean(),ya.mean()-yb.mean()])),
             min_pixels=min(len(xa),len(xb)))
    assert row['iou']>=.86 and row['shift_px']<=2.,row
    return row

stamp,raw,reference=read_preview()
rgbd_checks=[compare(a,b) for a,b in zip(calibrated(png),calibrated(reference))]
geometry=json.loads((ROOT.parent/'points_02/measured_scene.json').read_text())
board_mask=np.zeros(png.shape[:2],np.uint8)
board_mask[460:635,350:980]=255;board_mask[420:575,680:870]=0
sift=cv2.SIFT_create(nfeatures=3000,contrastThreshold=.02)
k0,d0=sift.detectAndCompute(cv2.cvtColor(png,cv2.COLOR_BGR2GRAY),board_mask)
k1,d1=sift.detectAndCompute(cv2.cvtColor(reference,cv2.COLOR_BGR2GRAY),board_mask)
pairs=[m for m,n in cv2.BFMatcher().knnMatch(d0,d1,k=2) if m.distance<.7*n.distance]
assert len(pairs)>=15
H,inliers=cv2.findHomography(np.array([k0[m.queryIdx].pt for m in pairs]),
    np.array([k1[m.trainIdx].pt for m in pairs]),cv2.RANSAC,1.)
assert H is not None and inliers.sum()>=15
centers=np.array([p['center_pixel'] for p in geometry['targets'].values()],dtype=float)
board_shift=float(np.linalg.norm(cv2.perspectiveTransform(centers[None,:,:],H)[0]-centers,axis=1).max())
assert board_shift<=2.,board_shift
reference_path=ROOT/'ui_ready_reference_04.jpg'
with reference_path.open('xb') as file:file.write(raw)
rows=[];seen=set();deadline=time.monotonic()+20
while time.monotonic()<deadline:
    stamp,_,image=read_preview()
    if stamp not in seen:
        seen.add(stamp);values=[]
        for check in checks:
            pairs=list(zip(ui_masks(reference,check['scene_roi']),ui_masks(image,check['scene_roi'])))
            for index in check['checked_color_indices']:
                values.append(dict(region=check['name'],color_index=index,**compare(*pairs[index])))
        rows.append(dict(stamp_ns=stamp,checks=values))
        if len(rows)>=10 and (rows[-1]['stamp_ns']-rows[0]['stamp_ns'])/1e9>=8:break
    time.sleep(.1)
assert len(rows)>=10 and (rows[-1]['stamp_ns']-rows[0]['stamp_ns'])/1e9>=8
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
state=json.load(opener.open('http://127.0.0.1:8767/api/snapshot',timeout=5))
assert state['config']['color_calibration']=='task2_scene_hsv_v1'
assert state['task'] is None and state['robot']['fresh'] and state['robot']['data']['stop_confirmed']
assert state['config']['plan_id']==manifest['plan_id']
assert state['config']['tracking_guard_deg']==manifest['tracking_guard_deg']
import math
assert max(abs(a-b) for a,b in zip(state['robot']['data']['q'],manifest['task_start_q']))<=math.radians(.05)
proof=dict(passed=True,created_utc=datetime.now(timezone.utc).isoformat(),reference=str(reference_path),
    reference_sha256=hashlib.sha256(raw).hexdigest(),rgbd_to_preview_checks=rgbd_checks,
    board_check=dict(inliers=int(inliers.sum()),target_shift_max_px=board_shift,threshold_px=2.),
    live_jpeg_stability_checks=rows,thresholds_unchanged=True,motion_commands_sent=0,color_calibration='task2_scene_hsv_v1',
    source_sha256=hashlib.sha256((SOURCE/'mtc_webui/hardware_tasks.py').read_bytes()).hexdigest())
with (ROOT/'ui_preview_reference_validation_04.json').open('x') as file:json.dump(proof,file,indent=2);file.write('\n')
evidence=dict(passed=True,plan_id=state['config']['plan_id'],manifest_sha256=settings['manifest_sha256'],
    created_unix=time.time(),expires_unix=time.time()+600,reference_image=str(reference_path),
    reference_image_sha256=proof['reference_sha256'],scene_checks=checks,
    reviewed_scene_sha256=hashlib.sha256((ROOT/'reset_scene_check.json').read_bytes()).hexdigest(),
    visual_review_sha256=hashlib.sha256((ROOT/'visual_review.json').read_bytes()).hexdigest(),
    preview_reference_validation_sha256=hashlib.sha256((ROOT/'ui_preview_reference_validation_04.json').read_bytes()).hexdigest(),
    tracking_guard_deg=manifest['tracking_guard_deg'],automatic_retry=False,user_submission_only=True,
    color_calibration='task2_scene_hsv_v1')
with (ROOT/'webui_preflight.json').open('x') as file:json.dump(evidence,file,indent=2);file.write('\n')
print(json.dumps(dict(passed=True,frames=len(rows),stable_seconds=(rows[-1]['stamp_ns']-rows[0]['stamp_ns'])/1e9,
    rgbd_to_preview_checks=rgbd_checks,thresholds_unchanged=True),indent=2))
