"""Read-only camera and robot checks for the manually submitted recovery."""
import json
import math
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
CAM = Path('/home/operator/meituan/eye_to_hand_calib/data/measurements/red_handle_20261014_104532')
CAPTURE = CAM / 'tracking_fix_resume_ready_20261014T2146'
CHECKS = [
    dict(name='carried_red_and_v1', scene_roi=[465,300,565,425], checked_color_indices=[0,3]),
    dict(name='neighbor_batteries_and_board', scene_roi=[660,430,820,565], checked_color_indices=[0,1,2,3]),
    dict(name='v1_stem', scene_roi=[470,270,540,360], checked_color_indices=[3]),
]


def masks(rgb, roi):
    x0,y0,x1,y1 = roi
    h = cv2.cvtColor(rgb[y0:y1,x0:x1], cv2.COLOR_RGB2HSV)
    common = (h[:,:,1] > 100) & (h[:,:,2] > 55)
    return [common & ((h[:,:,0] < 10) | (h[:,:,0] > 170)),
            common & (h[:,:,0] >= 15) & (h[:,:,0] <= 40),
            common & (h[:,:,0] > 40) & (h[:,:,0] <= 80),
            common & (h[:,:,0] >= 85) & (h[:,:,0] <= 120)]


meta = json.loads((CAPTURE / 'capture.json').read_text())
frames = np.load(CAPTURE / 'frames.npz')['color_rgb']
assert meta['complete'] and len(frames) == 60
assert 0 <= time.time() - meta['frames'][-1]['host_received_ns'] / 1e9 < 600
reference = cv2.cvtColor(cv2.imread(str(CAM / 'tracking_fix_20261014T213725Z_before/color.png')), cv2.COLOR_BGR2RGB)
comparisons = []
centers = []
for rgb in frames:
    for check in CHECKS:
        before = masks(reference, check['scene_roi'])
        after = masks(rgb, check['scene_roi'])
        for index in check['checked_color_indices']:
            a,b = before[index], after[index]
            ya,xa = np.nonzero(a)
            yb,xb = np.nonzero(b)
            row = dict(region=check['name'], color_index=index,
                       min_pixels=int(min(a.sum(),b.sum())),
                       iou=float((a&b).sum()/(a|b).sum()),
                       centroid_delta_px=float(np.linalg.norm([xa.mean()-xb.mean(),ya.mean()-yb.mean()])))
            assert row['min_pixels'] >= 300 and row['iou'] >= .86 and row['centroid_delta_px'] <= 2., row
            comparisons.append(row)
    y,x = np.nonzero(masks(rgb, CHECKS[0]['scene_roi'])[0])
    centers.append([x.mean()+465, y.mean()+300])
centers = np.array(centers)
drift = float(np.linalg.norm(centers-centers.mean(0),axis=1).max())
assert drift < .75

opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
start = json.loads((HERE / 'resume_package/full_trajectory.json').read_text())['segments'][0]['points'][0]['positions']
samples = []
for _ in range(12):
    snapshot = json.load(opener.open('http://127.0.0.1:8767/api/snapshot', timeout=4))
    robot = snapshot['robot']
    data = robot['data']
    assert snapshot['task'] is None and robot['fresh'] and data['robot_ready'] and data['stop_confirmed'] and data['safety_normal']
    assert data['queue'] == 0 and data['exec_id'] == -1 and max(map(abs,data['qd'])) <= math.radians(.02)
    error = math.degrees(max(abs(a-b) for a,b in zip(data['q'],start)))
    assert error <= .05
    samples.append(dict(at=snapshot['generated_at'],q=data['q'],qd=data['qd'],start_error_deg=error))
    time.sleep(.25)

report = dict(passed=True,capture=str(CAPTURE),frame_count=len(frames),scene_checks=CHECKS,
              stable_interval_s=(meta['frames'][-1]['host_received_ns']-meta['frames'][0]['host_received_ns'])/1e9,
              max_red_drift_px=drift,min_scene_iou=min(r['iou'] for r in comparisons),
              max_scene_center_delta_px=max(r['centroid_delta_px'] for r in comparisons),
              start_max_joint_error_deg=max(r['start_error_deg'] for r in samples),robot_samples=samples,
              original_v1_visual_review=True,neighbors_retained=True,manual_submission_only=True)
(HERE / 'resume_ready_verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k != 'robot_samples'},ensure_ascii=False,indent=2))
