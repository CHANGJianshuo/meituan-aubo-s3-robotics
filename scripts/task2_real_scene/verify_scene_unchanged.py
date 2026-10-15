#!/usr/bin/env python3
"""Compare a fresh frozen capture to the scene used for task-two planning."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np


def verify(reference_record, current_record, points, output, start_manifest=None):
    reference = json.loads(reference_record.read_text())
    current = json.loads(current_record.read_text())
    geometry = json.loads(points.read_text())
    old = Path(reference['capture_directory'])
    new = Path(current['capture_directory'])
    im0 = cv2.imread(str(old/'color.png'))
    im1 = cv2.imread(str(new/'color.png'))
    if im0 is None or im1 is None or im0.shape != im1.shape:
        raise ValueError('Comparable complete camera frames are required')
    roi = geometry['source_roi']; x0,y0,x1,y1 = roi
    def masks(image):
        hsv = cv2.cvtColor(image[y0:y1,x0:x1],cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(hsv)
        return {'A':(((h<=10)|(h>170))&(s>95)&(v>65)),
                'B':((h>=16)&(h<=38)&(s>95)&(v>65)),
                'C':((h>=85)&(h<=130)&(s>95)&(v>65)),
                'D':((h>=39)&(h<=84)&(s>50)&(v>65))}
    original = masks(im0); colors = {}; frames = np.load(new/'frames.npz')['color_rgb']
    for letter, first in original.items():
        ya,xa = np.nonzero(first)
        if len(xa)<300:raise ValueError('Missing reference battery '+letter)
        ious=[]; shifts=[]
        for rgb in frames:
            other = masks(cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))[letter]
            yb,xb = np.nonzero(other)
            if len(xb)<300:raise ValueError('Missing current battery '+letter)
            ious.append(float((first&other).sum()/(first|other).sum()))
            shifts.append(float(np.linalg.norm([xa.mean()-xb.mean(),ya.mean()-yb.mean()])))
        colors[letter]=dict(min_iou=min(ious),max_centroid_shift_px=max(shifts),
                            passed=min(ious)>=.86 and max(shifts)<=2.)
    # Check the printed board separately: stationary batteries cannot prove
    # that P1/P2/P3 have not moved beneath them.
    mask = np.zeros(im0.shape[:2],np.uint8)
    mask[460:635,350:980]=255; mask[420:575,680:870]=0
    sift=cv2.SIFT_create(nfeatures=3000,contrastThreshold=.02)
    k0,d0=sift.detectAndCompute(cv2.cvtColor(im0,cv2.COLOR_BGR2GRAY),mask)
    k1,d1=sift.detectAndCompute(cv2.cvtColor(im1,cv2.COLOR_BGR2GRAY),mask)
    if d0 is None or d1 is None:raise ValueError('No board features')
    pairs=[m for m,n in cv2.BFMatcher().knnMatch(d0,d1,k=2) if m.distance<.7*n.distance]
    if len(pairs)<15:raise ValueError('Insufficient board correspondences')
    a=np.array([k0[m.queryIdx].pt for m in pairs]);b=np.array([k1[m.trainIdx].pt for m in pairs])
    H,inliers=cv2.findHomography(a,b,cv2.RANSAC,1.)
    if H is None or inliers.sum()<15:raise ValueError('No consistent board registration')
    centers=np.array([p['center_pixel'] for p in geometry['targets'].values()],dtype=float)
    moved=cv2.perspectiveTransform(centers[None,:,:],H)[0]
    displacement=float(np.linalg.norm(moved-centers,axis=1).max())
    q0=reference['telemetry'][-1]['robot']['data']['q']
    camera_reference_joint_error=math.degrees(max(abs(a-b) for r in current['telemetry']
        for a,b in zip(q0,r['robot']['data']['q'])))
    start_reference=dict(kind='camera_reference_telemetry')
    if start_manifest is not None:
        # Reuse an unchanged board/objects image even when the new trajectory
        # begins at the previous run's final pose. Compare against its pinned
        # start, never by widening the existing 0.05 degree tolerance.
        manifest=json.loads(start_manifest.read_text())
        path=start_manifest.parent/'full_trajectory.json'
        if hashlib.sha256(path.read_bytes()).hexdigest()!=manifest['sha256']['full_trajectory.json']:
            raise ValueError('Start trajectory hash mismatch')
        trajectory=json.loads(path.read_text());q0=manifest['task_start_q']
        if (trajectory['plan_id']!=manifest['plan_id'] or len(q0)!=6
                or not all(math.isfinite(x) for x in q0)
                or q0!=trajectory['segments'][0]['points'][0]['positions']):
            raise ValueError('Inconsistent pinned trajectory start')
        start_reference=dict(kind='prepared_trajectory_manifest',path=str(start_manifest.resolve()),
            sha256=hashlib.sha256(start_manifest.read_bytes()).hexdigest(),plan_id=manifest['plan_id'])
    error=math.degrees(max(abs(a-b) for r in current['telemetry'] for a,b in zip(q0,r['robot']['data']['q'])))
    meta=json.loads((new/'capture.json').read_text())
    duration=(meta['frames'][-1]['host_received_ns']-meta['frames'][0]['host_received_ns'])/1e9
    report=dict(passed=all(v['passed'] for v in colors.values()) and displacement<=2. and error<=.05 and duration>=3.,
        color_checks=colors, board_inliers=int(inliers.sum()), board_target_shift_max_px=displacement,
        start_max_joint_error_deg=error, capture_duration_s=duration, frames=len(frames),
        start_reference=start_reference,camera_reference_joint_error_deg=camera_reference_joint_error,
        reference_capture=str(old), current_capture=str(new),
        reference_record_sha256=hashlib.sha256(reference_record.read_bytes()).hexdigest(),
        current_record_sha256=hashlib.sha256(current_record.read_bytes()).hexdigest(),
        physical_alignment_verified=False, motion_commands_sent=0)
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return report


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['reference-record','current-record','points','output']:
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--start-manifest',type=Path,
                   help='Compare current joints to this pinned new trajectory, while retaining the original scene image')
    raise SystemExit(0 if verify(**vars(p.parse_args()))['passed'] else 1)
