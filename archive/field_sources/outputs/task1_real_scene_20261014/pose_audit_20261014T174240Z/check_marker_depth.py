"""Compare RGB square PnP and depth in the original static measurement.

This is an offline camera-frame diagnostic. It does not update calibration,
reference points or motion commands. The marker length is an approximate user
measurement; temporal repeatability is not absolute measurement accuracy.
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

CALIB = Path('/home/operator/meituan/eye_to_hand_calib')
sys.path.insert(0, str(CALIB))
from e2h.edgerefine import refine
from e2h.pnp import solve_square

OUT = Path(__file__).resolve().parent
CAPTURE = CALIB/'data/measurements/red_handle_20261014_104532/red_handle_stable'
LENGTH = .0767


def stats(values):
    x = np.asarray(values)
    x = x[np.isfinite(x)]
    return None if not x.size else dict(n=int(x.size), p05=float(np.percentile(x, 5)),
                                       median=float(np.median(x)), p95=float(np.percentile(x, 95)))


def main():
    meta = json.loads((CAPTURE/'capture.json').read_text())
    point = json.loads((CAPTURE.parent/'point_result.json').read_text())
    data = np.load(CAPTURE/'frames.npz')
    rgb = data['color_rgb']
    indices = point['used_frame_indices']
    intr = meta['camera']['color_intrinsics']
    K = np.array([[intr['fx'], 0, intr['ppx']], [0, intr['fy'], intr['ppy']], [0, 0, 1.]])
    D = np.array(intr['coeffs'])
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detections = []
    for i in indices:
        gray = cv2.cvtColor(rgb[i], cv2.COLOR_RGB2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
        if ids is None or 0 not in ids.ravel():
            continue
        c = corners[list(ids.ravel()).index(0)].reshape(4, 2)
        c, ok = refine(gray, c)
        detections.append(dict(frame=i, corners=c, edge_refinement_ok=bool(ok)))
    if len(detections) < len(indices)*.6:
        raise RuntimeError('Insufficient consistent marker detections')
    all_corners = np.array([row['corners'] for row in detections])
    deviations = np.linalg.norm(all_corners-np.median(all_corners, axis=0), axis=2).max(axis=1)
    used = deviations <= 1.
    if used.mean() < .6:
        raise RuntimeError('Excessive corner variation')
    corners = all_corners[used].mean(axis=0)
    pose = solve_square(corners, K, D, LENGTH)
    mask = np.zeros(rgb.shape[1:3], np.uint8)
    cv2.fillConvexPoly(mask, np.rint(corners).astype(np.int32), 255)
    mask = cv2.erode(mask, np.ones((17, 17), np.uint8))
    v, u = np.where(mask > 0)
    rays = np.column_stack([u, v, np.ones(len(u))]) @ np.linalg.inv(K).T
    Rcd = np.array(meta['camera']['T_color_depth']['rotation_column_major']).reshape(3, 3).T
    tcd = np.array(meta['camera']['T_color_depth']['translation_m'])
    depth = data['aligned_depth_u16'][indices][:, v, u].astype(float)*meta['camera']['depth_unit_m']
    depth[depth == 0] = np.nan
    color_z = (depth+(Rcd.T@tcd)[2])/(rays@Rcd[:, 2])
    median_z = np.nanmedian(color_z, axis=0)
    comparisons = []
    for name, T, err in [('best', pose['T'], pose['err']), ('alternate', pose['T_alt'], pose['err_alt'])]:
        n = T[:3, 2]
        predicted = np.dot(n, T[:3, 3])/(rays@n)
        residual = (median_z-predicted)*1000
        comparisons.append(dict(solution=name, reprojection_rms_px=err,
                                T_color_marker=T.tolist(),
                                predicted_color_z_m=stats(predicted),
                                residual_depth_minus_rgb_plane_mm=stats(residual),
                                relative_z_error_percent=stats((median_z/predicted-1)*100),
                                per_frame_median_residual_mm=stats(np.nanmedian((color_z-predicted)*1000, axis=1))))
    frame_ranges = [solve_square(c, K, D, LENGTH)['T'][2, 3] for c in all_corners[used]]
    report = dict(status='OFFLINE_CAMERA_DIAGNOSTIC_NOT_PHYSICAL_CALIBRATION',
                  capture_directory=str(CAPTURE), marker_id=0, marker_length_m=LENGTH,
                  marker_length_source='User approximate measurement; uncertainty unknown',
                  time_matched_frame_indices=indices, detected_frames=len(detections),
                  used_corner_frames=[d['frame'] for d, yes in zip(detections, used) if yes],
                  corner_uv=corners.tolist(), corner_jitter_max_std_px=float(all_corners[used].std(axis=0).max()),
                  per_frame_pnp_centre_z_m_repeatability_only=stats(frame_ranges),
                  pnp_alternate_to_best_error_ratio=pose['ratio'],
                  interior_erosion_kernel_px=17, interior_color_pixels=int(len(u)),
                  observed_color_z_m=stats(median_z),
                  valid_depth_frames_per_pixel=stats(np.sum(np.isfinite(color_z), axis=0)),
                  comparisons=comparisons,
                  caveats=['Uses camera coordinates only; no assumption that the marker and battery support are coplanar.',
                           'Expanded aligned-depth pixels and neighboring stereo estimates are correlated.',
                           'PnP corner, flatness, intrinsics and approximate marker length errors remain.',
                           'This comparison does not validate the robot base transform or establish tool clearance.'])
    (OUT/'marker_depth_diagnostic.json').write_text(json.dumps(report, indent=2)+'\n')
    view = cv2.cvtColor(rgb[indices[-1]], cv2.COLOR_RGB2BGR)
    cv2.polylines(view, [np.rint(corners).astype(np.int32)], True, (0, 255, 255), 1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(view, contours, -1, (255, 255, 0), 1)
    crop = cv2.resize(view[440:535, 305:425], (720, 570))
    crop = cv2.copyMakeBorder(crop, 65, 0, 0, 0, cv2.BORDER_CONSTANT, value=(245, 245, 245))
    cv2.putText(crop, '76.7 mm square: yellow outline; cyan depth ROI', (10, 25), 0, .65, (30, 30, 30), 1)
    cv2.putText(crop, 'Approximate scale / offline diagnostic only', (10, 50), 0, .65, (30, 30, 30), 1)
    cv2.imwrite(str(OUT/'marker_depth_diagnostic.png'), crop)
    print(json.dumps({k:v for k,v in report.items() if k not in ['time_matched_frame_indices', 'used_corner_frames', 'caveats']}, indent=2))


if __name__ == '__main__':
    main()
