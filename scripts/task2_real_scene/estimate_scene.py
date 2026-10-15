#!/usr/bin/env python3
"""Estimate the placed battery scene from frozen pixels and task-one geometry.

The task-one red alignment transfers a constant XY correction to the registered
view. Current batteries are measured again; their historical positions are not
reused. All results remain candidates until physical alignment is checked.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np


def ray_plane(pixels, transform, intrinsics, height):
    uv = np.atleast_2d(pixels)
    rays = np.column_stack([uv, np.ones(len(uv))]) @ np.linalg.inv(intrinsics).T
    directions = rays @ transform[:3, :3].T
    scale = (height-transform[2, 3])/directions[:, 2]
    if not np.all(np.isfinite(scale)) or np.any(scale <= 0):
        raise ValueError('Pixel rays do not intersect the support in front of the camera')
    return transform[:3, 3] + scale[:, None]*directions


def circle(points):
    xy = points[:, :2]
    center0 = xy.mean(axis=0)
    local = xy-center0
    values = np.linalg.lstsq(np.c_[2*local, np.ones(len(local))],
                             (local*local).sum(axis=1), rcond=None)[0]
    center = center0+values[:2]
    radius = math.sqrt(values[2]+np.dot(values[:2], values[:2]))
    residual = np.linalg.norm(xy-center, axis=1)-radius
    return center, radius, float(np.sqrt(np.mean(residual**2)))


def estimate(capture_record, registration, task1_scenario, anchor_report, anchor_view, output, order='ABC'):
    if len(order) != 3 or len(set(order)) != 3 or any(c not in 'ABCD' for c in order):
        raise ValueError('Three distinct battery letters are required')
    output.mkdir(parents=True, exist_ok=False)
    capture = json.loads(capture_record.read_text())
    source = Path(capture['capture_directory'])
    meta = json.loads((source/'capture.json').read_text())
    i = meta['camera']['color_intrinsics']
    K = np.array([[i['fx'], 0, i['ppx']], [0, i['fy'], i['ppy']], [0, 0, 1.]])
    cfg = json.loads(task1_scenario.read_text())
    reg = json.loads(registration.read_text())
    if reg['train_inliers'] < 50 or reg['holdout_all_reprojection_px']['median'] > 1.5:
        raise ValueError('View registration has insufficient held-out support')
    X = np.array(reg['T_base_current_camera_candidate'])
    anchor = json.loads(anchor_report.read_text())
    old_view = np.array(json.loads(anchor_view.read_text())['T_base_current_camera_candidate'])
    z = cfg['table_z']
    correction = np.array(anchor['base_top_points_m']['red']) - ray_plane(
        anchor['bar_top_pixels']['red'], old_view, K, z+.080)[0]
    X[:3, 3] += correction
    image = cv2.imread(str(source/'color.png'))
    marked = image.copy()
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Regions identify the current source area and printed slot labels. They
    # are retained as evidence, and cannot be reused on a different camera view.
    source_roi = [680, 420, 870, 570]
    rois = {'P1': [520, 520, 645, 601], 'P2': [375, 528, 510, 612],
            'P3': [400, 468, 520, 530]}
    objects = {}
    for letter, name, lo, hi in [('A', 'red', 0, 10), ('B', 'yellow', 16, 38),
                                 ('C', 'blue', 85, 130), ('D', 'green', 39, 84)]:
        hue, sat, val = cv2.split(hsv)
        # The pale green top face is less saturated than its vertical wall.
        # Retain it so the bar-top measurement is not biased down the wall.
        min_saturation = 50 if name == 'green' else 95
        mask = (hue >= lo) & (hue <= hi) & (sat > min_saturation) & (val > 65)
        if name == 'red':
            mask |= (hue > 170) & (sat > 95) & (val > 65)
        keep = np.zeros(mask.shape, bool)
        x0, y0, x1, y1 = source_roi
        keep[y0:y1, x0:x1] = True
        count, labels, stats, _ = cv2.connectedComponentsWithStats((mask & keep).astype('uint8'))
        if count < 2:
            raise ValueError('Missing current battery '+name)
        label = 1+np.argmax(stats[1:, cv2.CC_STAT_AREA])
        y, x = np.nonzero(labels == label)
        if len(x) < 300:
            raise ValueError('Insufficient color support for '+name)
        top = y.min()
        near = y <= top+7
        uv = np.array([x[near].mean(), y[near].mean()])
        xyz = ray_plane(uv, X, K, z+.080)[0]
        # Fit the visible bar-top edge, excluding the rounded end corners.
        columns = np.arange(int(np.percentile(x[near], 15)), int(np.percentile(x[near], 85)))
        edge = [(u, np.min(y[x == u])) for u in columns if np.any(x == u)]
        edge = np.array(edge)
        fit = np.polyfit(edge[:, 0], edge[:, 1], 1)
        ends = np.array([[columns[0], np.polyval(fit, columns[0])],
                         [columns[-1], np.polyval(fit, columns[-1])]])
        p = ray_plane(ends, X, K, z+.080)
        yaw = math.atan2(*(p[1]-p[0])[[1, 0]])
        objects[letter] = dict(color=name, handle_top_pixel=uv.tolist(),
                              handle_top_base_m=xyz.tolist(), base_xyz_m=[*xyz[:2], z],
                              yaw_rad=yaw, edge_pixels=ends.tolist(), color_pixel_count=len(x))
        cv2.circle(marked, tuple(np.rint(uv).astype(int)), 5, (255, 0, 255), 2)
        cv2.putText(marked, letter, tuple(np.rint(uv+[10, -9]).astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 0, 255), 2)
    gray = cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (3, 3), 0)
    edges = cv2.Canny(gray, 15, 40)
    targets = {}
    for slot, (x0, y0, x1, y1) in rois.items():
        mask = np.zeros_like(edges); mask[y0:y1, x0:x1] = edges[y0:y1, x0:x1]
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        candidates = []
        for contour in contours:
            if len(contour) < 100 or cv2.contourArea(contour) < 2500:
                continue
            points = ray_plane(contour.reshape(-1, 2), X, K, z)
            center, radius, rms = circle(points)
            if .055 < radius < .095 and rms < .004:
                candidates.append((abs(radius-.075)+rms, center, radius, rms, contour))
        if not candidates:
            raise ValueError('No valid outer target circle for '+slot)
        _, center, radius, rms, contour = min(candidates, key=lambda c:c[0])
        xyz = np.array([*center, z])
        camera_point = (xyz-X[:3, 3]) @ X[:3, :3]
        uv = (K@camera_point)[:2]/camera_point[2]
        targets[slot] = dict(base_xyz_m=xyz.tolist(), center_pixel=uv.tolist(),
                             fitted_radius_m=radius, circle_rms_m=rms, roi=[x0,y0,x1,y1])
        cv2.drawContours(marked, [contour], -1, (0, 150, 0), 1)
        cv2.drawMarker(marked, tuple(np.rint(uv).astype(int)), (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
        cv2.putText(marked, slot, tuple(np.rint(uv+[10, -10]).astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 200), 2)
    report = dict(schema=1, task='sequence', order=order, frame='base_link', units='m/rad',
                  status='CANDIDATE_REQUIRES_PHYSICAL_ALIGNMENT', physical_coordinates_verified=False,
                  capture_directory=str(source), support_z_m=z, objects=objects, targets=targets,
                  T_base_color_optical_candidate=X.tolist(), task1_xy_correction_m=correction.tolist(),
                  source_roi=source_roi, initial_joints=capture['telemetry'][-1]['robot']['data']['q'],
                  assumptions=['Same support height as successful task one; no new table survey.',
                      'XY correction transferred from physically proven task-one red insertion.',
                      'New source poses and target circles estimated from current pixels.',
                      'RGB-D view registration and circle residuals do not prove absolute accuracy.'],
                  provenance={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in [capture_record, registration, task1_scenario, anchor_report, anchor_view]})
    (output/'measured_scene.json').write_text(json.dumps(report, indent=2)+'\n')
    cv2.imwrite(str(output/'annotated_scene.png'), marked)
    print(json.dumps({'objects':objects, 'targets':targets, 'task1_xy_correction_m':correction.tolist()}, indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['capture-record', 'registration', 'task1-scenario', 'anchor-report', 'anchor-view', 'output']:
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--order', default='ABC')
    args = parser.parse_args()
    estimate(**vars(args))
