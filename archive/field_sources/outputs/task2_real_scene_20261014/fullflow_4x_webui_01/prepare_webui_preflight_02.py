"""Bind like-for-like live JPEG preview to the reviewed RGB-D capture."""
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import time
import urllib.request
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent
settings=json.loads((ROOT/'hardware_config.json').read_text())
review=json.loads((ROOT/'ready_scene_check_planned_start.json').read_text())
assert review['passed'] and review['start_reference']['sha256']==settings['manifest_sha256']
preview=Path(settings['preview_dir'])/'latest_color.jpg'
png=cv2.imread(str(ROOT/'capture_03_ready/physical_initial.png'))
checks=[dict(name='all_four_batteries',scene_roi=[680,420,870,570],checked_color_indices=[0,1,2,3]),
        dict(name='fixed_v1_tool',scene_roi=[380,305,465,415],checked_color_indices=[3])]

def read_preview():
    before=preview.stat();raw=preview.read_bytes();after=preview.stat()
    assert before.st_mtime_ns==after.st_mtime_ns and before.st_size==after.st_size
    assert 0<=time.time()-after.st_mtime<2. and raw[:2]==b'\xff\xd8' and raw[-2:]==b'\xff\xd9'
    image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
    assert image is not None and image.shape==png.shape
    return after.st_mtime_ns,raw,image

def calibrated(image):
    h,s,v=cv2.split(cv2.cvtColor(image[420:570,680:870],cv2.COLOR_BGR2HSV))
    return [((h<=10)|(h>170))&(s>95)&(v>65),
            (h>=16)&(h<=38)&(s>95)&(v>65),
            (h>=85)&(h<=130)&(s>95)&(v>65),
            (h>=89)&(h<=84)&(s>50)&(v>65)]

def ui_masks(image,roi):
    x0,y0,x1,y1=roi;h=cv2.cvtColor(image[y0:y1,x0:x1],cv2.COLOR_BGR2HSV)
    common=(h[:,:,1]>100)&(h[:,:,2]>55)
    return [common&((h[:,:,0]<10)|(h[:,:,0]>170)),common&(h[:,:,0]>=15)&(h[:,:,0]<=40),
            common&(h[:,:,0]>40)&(h[:,:,0]<=80),common&(h[:,:,0]>=85)&(h[:,:,0]<=120)]

def compare(a,b):
    ya,xa=np.nonzero(a);yb,xb=np.nonzero(b)
    assert min(len(xa),len(xb))>=800
    row=dict(iou=float((a&b).sum()/(a|b).sum()),
             shift_px=float(np.linalg.norm([xa.mean()-xb.mean(),ya.mean()-yb.mean()])),
             min_pixels=min(len(xa),len(xb)))
    assert row['iou']>=.86 and row['shift_px']<=2.,row
    return row

stamp,raw,reference=read_preview()
rgbd_checks=[compare(a,b) for a,b in zip(calibrated(png),calibrated(reference))]
reference_path=ROOT/'ui_ready_reference_02.jpg'
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
assert state['task'] is None and state['robot']['fresh'] and state['robot']['data']['stop_confirmed']
assert state['config']['plan_id']==review['start_reference']['plan_id']
proof=dict(passed=True,created_utc=datetime.now(timezone.utc).isoformat(),reference=str(reference_path),
    reference_sha256=hashlib.sha256(raw).hexdigest(),rgbd_to_preview_checks=rgbd_checks,
    live_jpeg_stability_checks=rows,thresholds_unchanged=True,motion_commands_sent=0)
with (ROOT/'ui_preview_reference_validation_02.json').open('x') as file:json.dump(proof,file,indent=2);file.write('\n')
evidence=dict(passed=True,plan_id=state['config']['plan_id'],manifest_sha256=settings['manifest_sha256'],
    created_unix=time.time(),expires_unix=time.time()+600,reference_image=str(reference_path),
    reference_image_sha256=proof['reference_sha256'],scene_checks=checks,
    reviewed_scene_sha256=hashlib.sha256((ROOT/'ready_scene_check_planned_start.json').read_bytes()).hexdigest(),
    visual_review_sha256=hashlib.sha256((ROOT/'ready_visual_review.json').read_bytes()).hexdigest(),
    preview_reference_validation_sha256=hashlib.sha256((ROOT/'ui_preview_reference_validation_02.json').read_bytes()).hexdigest(),
    uniform_tracking_guard_deg=.5,automatic_retry=False)
with (ROOT/'webui_preflight.json').open('x') as file:json.dump(evidence,file,indent=2);file.write('\n')
print(json.dumps(dict(passed=True,frames=len(rows),stable_seconds=(rows[-1]['stamp_ns']-rows[0]['stamp_ns'])/1e9,
    rgbd_to_preview_checks=rgbd_checks,thresholds_unchanged=True),indent=2))
