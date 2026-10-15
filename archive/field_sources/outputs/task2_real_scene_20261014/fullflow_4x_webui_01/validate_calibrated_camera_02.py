"""Read-only camera comparison; never enable the UI or submit motion."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.parents[2]/'outputs/webui_laptop_20261014/source/src/mtc_cpp_mock_bringup'
sys.path.insert(0,str(SOURCE))
from mtc_webui.hardware_tasks import color_masks

settings=json.loads((ROOT/'hardware_config.json').read_text())
reference=cv2.imread(str(ROOT/'capture_03_ready/physical_initial.png'))
preview=Path(settings['preview_dir'])/'latest_color.jpg'
checks=[dict(name='all_four_batteries',scene_roi=[680,420,870,570],checked_color_indices=[0,1,2,3]),
        dict(name='fixed_v1_tool',scene_roi=[380,305,465,415],checked_color_indices=[3])]

def compare(a,b):
    ya,xa=np.nonzero(a);yb,xb=np.nonzero(b)
    assert min(len(xa),len(xb))>=300
    result=dict(iou=float((a&b).sum()/(a|b).sum()),
                centroid_shift_px=float(np.linalg.norm([xa.mean()-xb.mean(),ya.mean()-yb.mean()])),
                minimum_pixels=min(len(xa),len(xb)))
    assert result['iou']>=.86 and result['centroid_shift_px']<=2.,result
    return result

capture_out=ROOT/'color_validation_02';capture_out.mkdir()
rows=[];seen=set();until=time.monotonic()+35
while time.monotonic()<until:
    before=preview.stat();raw=preview.read_bytes();after=preview.stat()
    if before.st_mtime_ns!=after.st_mtime_ns or before.st_size!=after.st_size:
        continue
    assert 0<=time.time()-after.st_mtime<2.
    stamp=after.st_mtime_ns
    if stamp not in seen:
        seen.add(stamp);(capture_out/(str(stamp)+'.jpg')).write_bytes(raw);frame=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
        assert frame is not None and frame.shape==reference.shape
        values=[]
        for check in checks:
            roi=check['scene_roi']
            pairs=list(zip(color_masks(reference,roi,True),color_masks(frame,roi,True)))
            for index in check['checked_color_indices']:
                try:
                    values.append(dict(region=check['name'],color_index=index,**compare(*pairs[index])))
                except AssertionError as error:
                    (capture_out/'failure.json').write_text(json.dumps(dict(stamp=stamp,region=check['name'],color_index=index,pixels=[int(m.sum()) for m in pairs[index]],error=str(error),previous_frames=rows),indent=2))
                    raise
        rows.append(dict(source_stamp_ns=stamp,checks=values))
        if len(rows)>=20 and (stamp-rows[0]['source_stamp_ns'])/1e9>=18:break
    time.sleep(.1)
assert len(rows)>=20 and (rows[-1]['source_stamp_ns']-rows[0]['source_stamp_ns'])/1e9>=18

# The independent prior final burst checks the same final-placement color
# calibration against an actual successful outcome, including exposed green.
prior=ROOT.parent/'fullflow_2x_01/capture_04_final'
capture=Path(json.loads((prior/'capture_record.json').read_text())['capture_directory'])
frames=np.load(capture/'frames.npz')['color_rgb']
final=cv2.imread(settings['placement_reference']);placement=[]
for check in settings['placement_checks']:
    expected=color_masks(final,check['roi'],True)[check['color_index']]
    values=[];centers=[]
    for rgb in frames:
        current=color_masks(cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),check['roi'],True)[check['color_index']]
        values.append(compare(expected,current))
        y,x=np.nonzero(current);centers.append([x.mean(),y.mean()])
    centers=np.array(centers);drift=float(np.linalg.norm(centers-centers.mean(axis=0),axis=1).max())
    assert drift<.75
    placement.append(dict(object=check['object'],min_iou=min(r['iou'] for r in values),
        max_centroid_shift_px=max(r['centroid_shift_px'] for r in values),max_drift_px=drift))
report=dict(passed=True,created_utc=datetime.now(timezone.utc).isoformat(),read_only=True,
    motion_commands_sent=0,color_calibration='task2_scene_hsv_v1',thresholds_unchanged=True,
    source_sha256=hashlib.sha256((SOURCE/'mtc_webui/hardware_tasks.py').read_bytes()).hexdigest(),
    live_frames=rows,live_duration_s=(rows[-1]['source_stamp_ns']-rows[0]['source_stamp_ns'])/1e9,
    prior_successful_final_frames=len(frames),prior_final_checks=placement)
with (ROOT/'calibrated_camera_validation_02.json').open('x') as f:json.dump(report,f,indent=2);f.write('\n')
print(json.dumps(dict(passed=True,live_frames=len(rows),live_duration_s=report['live_duration_s'],
    prior_final_checks=placement),indent=2))
