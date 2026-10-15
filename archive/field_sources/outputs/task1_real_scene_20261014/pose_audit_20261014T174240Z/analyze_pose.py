"""Read-only RGB-D / CAD comparison of the original and stopped poses.

All depths refer to visible surfaces, not an inferred tool TCP. A matching
projection is not an independent physical TCP or clearance calibration.
"""
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts/task1_real_scene'))
from fit_kinematics import forward, origins
from inspect_mount import box

OUT = Path(__file__).resolve().parent
RUN = OUT.parent
POINT = json.loads((RUN / 'hardware_package/metadata/red_handle_original_point.json').read_text())
X = np.array(POINT['T_base_color_optical'])
C = np.linalg.inv(X)
ANGLES = np.linspace(0, 2*np.pi, 96)
BOXES = [([0, 0, .058], [.020, .030, .100]),
         ([.0325, 0, .1105], [.085, .044, .005]),
         ([.0725, 0, .1055], [.005, .044, .005])]
BODIES = [np.array([[.0375*np.cos(a), .0375*np.sin(a), z]
                    for z in [-.007, .008] for a in ANGLES])]
BODIES += [box(center, size) for center, size in BOXES]
NAMES = ['disc', 'stem', 'tongue', 'lip']


def raycast(o, d):
    """Nearest positive ray intersection with the union of nominal V1 solids."""
    candidates = []
    a = np.sum(d[:, :2]**2, axis=1)
    b = 2*(d[:, :2] @ o[:2])
    c = np.dot(o[:2], o[:2])-.0375**2
    delta = b*b-4*a*c
    hits = []
    for sign in [-1, 1]:
        t = (-b+sign*np.sqrt(np.maximum(delta, 0)))/(2*a)
        z = o[2]+d[:, 2]*t
        hits.append(np.where((delta >= 0) & (t > 0) & (z >= -.007) & (z <= .008), t, np.inf))
    for z in [-.007, .008]:
        t = (z-o[2])/d[:, 2]
        xy = o[:2]+d[:, :2]*t[:, None]
        hits.append(np.where((t > 0) & (np.sum(xy**2, axis=1) <= .0375**2), t, np.inf))
    candidates.append(np.min(hits, axis=0))
    for center, size in BOXES:
        lo = np.array(center)-np.array(size)/2
        hi = np.array(center)+np.array(size)/2
        a, b = (lo-o)/d, (hi-o)/d
        near, far = np.max(np.minimum(a, b), axis=1), np.min(np.maximum(a, b), axis=1)
        candidates.append(np.where((near <= far) & (near > 0), near, np.inf))
    candidates = np.array(candidates)
    return candidates.min(axis=0), candidates.argmin(axis=0)


def analyze(label, state_path):
    state = json.loads(state_path.read_text())
    capture = Path(state['camera_capture_dir'])
    meta = json.loads((capture/'capture.json').read_text())
    rgb = cv2.imread(str(capture/'color.png'))
    intr = meta['camera']['color_intrinsics']
    K = np.array([[intr['fx'], 0, intr['ppx']], [0, intr['fy'], intr['ppy']], [0, 0, 1.]])
    F = forward(origins(ET.parse(RUN/'inputs/controller_matched_arm.urdf').getroot()), state['samples'][-1]['q'])[0]

    def project(points):
        camera = points @ C[:3, :3].T+C[:3, 3]
        uv = camera @ K.T
        return uv[:, :2]/uv[:, 2:]

    cx, cy = np.rint(project(F[None, :3, 3])[0]).astype(int)
    x1, y1, x2, y2 = cx-105, cy-70, cx+75, cy+110
    blue = cv2.inRange(cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV), (85, 80, 45), (120, 255, 255))
    mask = np.zeros_like(blue)
    mask[y1:y2, x1:x2] = blue[y1:y2, x1:x2]
    rows, panels = [], []
    for yaw in [0, 90, 180, 270]:
        T = F.copy()
        T[:3, :3] = F[:3, :3] @ Rotation.from_euler('z', yaw, degrees=True).as_matrix()
        pred = np.zeros_like(blue)
        overlay = rgb.copy()
        for body in BODIES:
            uv = project(body @ T[:3, :3].T+T[:3, 3])
            hull = cv2.convexHull(np.rint(uv).astype(np.int32))
            cv2.fillConvexPoly(pred, hull, 255)
            cv2.polylines(overlay, [hull], True, (0, 0, 255), 1, cv2.LINE_AA)
        iou = np.count_nonzero((pred > 0) & (mask > 0))/np.count_nonzero((pred > 0) | (mask > 0))
        rows.append({'flange_yaw_deg': yaw, 'silhouette_iou': iou})
        tile = cv2.resize(overlay[y1:y2, x1:x2], (540, 540))
        cv2.rectangle(tile, (0, 0), (540, 45), (25, 25, 25), -1)
        cv2.putText(tile, f'{label}: flange {yaw}, IoU {iou:.3f}', (8, 30), 0, .65, (255, 255, 255), 2)
        panels.append(tile)
        if yaw == 90:
            model, mounted, annotated = pred.copy(), T.copy(), overlay.copy()
    cv2.imwrite(str(OUT/f'{label}_mount_candidates.png'), np.vstack([np.hstack(panels[:2]), np.hstack(panels[2:])]))
    cv2.imwrite(str(OUT/f'{label}_cad_projection.png'), annotated)

    # Exclude color silhouette edges. Expanded depth pixels are correlated; the
    # count below is image pixels, not independent sensor samples.
    inside = cv2.erode(mask, np.ones((7, 7), np.uint8)) > 0
    inside &= cv2.erode(model, np.ones((7, 7), np.uint8)) > 0
    v, u = np.nonzero(inside)
    rays = np.linalg.solve(K, np.vstack([u, v, np.ones(len(u))])).T
    tool_from_camera = np.linalg.inv(mounted) @ X
    z_model, parts = raycast(tool_from_camera[:3, 3], rays @ tool_from_camera[:3, :3].T)
    raw = np.load(capture/'frames.npz')['aligned_depth_u16'][:, v, u].astype(float)
    raw[raw == 0] = np.nan
    Rcd = np.array(meta['camera']['T_color_depth']['rotation_column_major']).reshape(3, 3).T
    tcd = np.array(meta['camera']['T_color_depth']['translation_m'])
    z = (raw*meta['camera']['depth_unit_m']+(Rcd.T @ tcd)[2])/(rays @ Rcd[:, 2])[None, :]
    valid_count = np.isfinite(z).sum(axis=0)
    have_depth = valid_count > 0
    z_observed = np.full(len(u), np.nan)
    temporal_mad = np.full(len(u), np.nan)
    z_observed[have_depth] = np.nanmedian(z[:, have_depth], axis=0)
    temporal_mad[have_depth] = np.nanmedian(np.abs(z[:, have_depth]-z_observed[have_depth]), axis=0)*1000
    valid = np.isfinite(z_model) & (valid_count >= 20) & np.isfinite(z_observed)
    stats = {}
    for i, name in enumerate(NAMES):
        keep = valid & (parts == i)
        residual = (z_observed[keep]-z_model[keep])*1000
        stats[name] = {'image_pixels': int(keep.sum())}
        if len(residual):
            stats[name].update(color_z_residual_mm_p05_median_p95=np.percentile(residual, [5, 50, 95]).tolist(),
                               temporal_depth_mad_mm_median=float(np.median(temporal_mad[keep])))
    order = sorted(rows, key=lambda row: -row['silhouette_iou'])
    return dict(state_file=str(state_path), capture=str(capture), camera_intrinsics=intr,
                candidates=order, original_mount_criterion_passed=order[0]['silhouette_iou'] >= .65 and order[0]['silhouette_iou']-order[1]['silhouette_iou'] >= .15,
                silhouette_caveat='Original ROI-based blue segmentation also includes unrelated blue pixels; the predicted silhouette does not hide areas occluded by the robot. A low score alone does not establish a changed mount.',
                depth_method='30-frame per-pixel median; observed and predicted silhouettes eroded 3 px; raycast visible CAD surfaces; at least 20 valid frames; no depth-residual filtering',
                depth_residuals=stats,
                caveat='Residuals mix hand-eye error, camera depth error, physical mounting and CAD dimensions; they are not physical TCP errors or clearance certification.')


if __name__ == '__main__':
    report = {'initial': analyze('initial', RUN/'inputs/captured_state.json'),
              'stopped': analyze('stopped', RUN/'readback_20261014T173727Z/robot_state.json')}
    current = json.loads((RUN/'readback_20261014T173727Z/robot_state.json').read_text())['samples'][-1]['q']
    segment = json.loads((RUN/'hardware_package/controller_position_samples.json').read_text())[0]
    q = np.array(segment['positions_rad'])
    errors = np.max(np.abs(q-current), axis=1)
    index = int(errors.argmin())
    report['trajectory_match'] = {'segment': segment['primitive'], 'sample_index': index,
                                'elapsed_s': index*segment['interval_s'],
                                'max_joint_error_deg': float(np.degrees(errors[index])),
                                'current_q_rad': current, 'saved_sample_q_rad': q[index].tolist()}
    (OUT/'pose_comparison.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
