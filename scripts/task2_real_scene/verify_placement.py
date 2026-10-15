#!/usr/bin/env python3
"""Check placed colors and >=3 s image stability against the captured slot map.

XY errors use the existing candidate camera geometry; they are not a new
absolute calibration or competition score. Uprightness still needs image review.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from estimate_scene import ray_plane


def verify(capture_record, points, placed, output):
    record = json.loads(capture_record.read_text())
    geometry = json.loads(points.read_text())
    if not placed or len(set(placed)) != len(placed) or any(x not in geometry['order'] for x in placed):
        raise ValueError('Distinct placed colors from the requested sequence are required')
    capture = Path(record['capture_directory'])
    for name in ['capture.json', 'frames.npz']:
        if hashlib.sha256((capture/name).read_bytes()).hexdigest() != record['sha256'][name]:
            raise ValueError('Captured evidence changed: '+name)
    meta = json.loads((capture / 'capture.json').read_text())
    frames = np.load(capture / 'frames.npz')['color_rgb']
    i = meta['camera']['color_intrinsics']
    K = np.array([[i['fx'], 0, i['ppx']], [0, i['fy'], i['ppy']], [0, 0, 1.]])
    X = np.array(geometry['T_base_color_optical_candidate'])
    duration = (meta['frames'][-1]['host_received_ns'] - meta['frames'][0]['host_received_ns']) / 1e9
    reports = []
    for letter in placed:
        slot = 'P' + str(geometry['order'].index(letter) + 1)
        target = np.array(geometry['targets'][slot]['base_xyz_m'])
        top = target + [0, 0, .080]
        camera_point = (top - X[:3, 3]) @ X[:3, :3]
        uv = (K @ camera_point)[:2] / camera_point[2]
        bottom_v = geometry['targets'][slot]['center_pixel'][1]
        x0, x1 = max(0, int(uv[0]-55)), min(frames.shape[2], int(uv[0]+55))
        y0, y1 = max(0, int(uv[1]-15)), min(frames.shape[1], int(bottom_v+25))
        centers, positions, areas = [], [], []
        for frame in frames:
            h, s, v = cv2.split(cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_RGB2HSV))
            if letter == 'A': mask = ((h < 10) | (h > 170)) & (s > 95) & (v > 65)
            elif letter == 'B': mask = (h >= 16) & (h <= 38) & (s > 95) & (v > 65)
            elif letter == 'C': mask = (h >= 85) & (h <= 130) & (s > 95) & (v > 65)
            elif letter == 'D': mask = (h >= 39) & (h <= 84) & (s > 50) & (v > 65)
            else: raise ValueError('Unknown battery')
            n, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype('uint8'))
            if n < 2 or stats[1:, cv2.CC_STAT_AREA].max() < 300:
                raise ValueError(f'Missing {letter} at {slot}')
            label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
            y, x = np.nonzero(labels == label)
            near = y <= y.min()+7
            bar = [x[near].mean()+x0, y[near].mean()+y0]
            positions.append(ray_plane(bar, X, K, top[2])[0])
            centers.append([x.mean()+x0, y.mean()+y0])
            areas.append(len(x))
        centers = np.array(centers)
        drift = float(np.linalg.norm(centers-centers.mean(0), axis=1).max())
        error = float(np.linalg.norm((np.array(positions)-target)[:, :2], axis=1).max())
        reports.append(dict(object=letter, slot=slot, roi=[x0,y0,x1,y1],
            max_centroid_deviation_px=drift, estimated_max_xy_error_m=error,
            candidate_mean_xy_m=np.mean(positions, axis=0)[:2].tolist(),
            color_area_range=[int(min(areas)),int(max(areas))],
            passed=drift < .75 and error <= .015 and min(areas)/max(areas) >= .9))
    result = dict(passed=duration >= 3 and len(frames) >= 30 and all(x['passed'] for x in reports),
        objects=reports, frames=len(frames), stable_interval_s=duration,
        capture=str(capture), capture_record_sha256=hashlib.sha256(capture_record.read_bytes()).hexdigest(),
        upright_image_review_required=True, absolute_calibration_claimed=False,
        competition_score_claimed=False, motion_commands_sent_by_this_check=0)
    if output.exists(): raise ValueError('Use a new verification output')
    output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['capture-record', 'points', 'output']:
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--placed', required=True)
    args = parser.parse_args()
    raise SystemExit(0 if verify(**vars(args))['passed'] else 1)
