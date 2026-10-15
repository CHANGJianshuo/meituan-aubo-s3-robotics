"""Offline RGB-D view registration, with held-out image correspondences.

Estimates an effective change of view. This cannot identify the physical cause
or certify a replacement hand-eye transform. No original data are modified.
"""
import json
from pathlib import Path

import cv2
import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = Path('/home/operator/meituan/eye_to_hand_calib/data/measurements/red_handle_20261014_104532')
REFERENCE = ROOT/'current_state_20261014T173727Z'


def stats(x):
    x = np.asarray(x)
    x = x[np.isfinite(x)]
    return dict(n=int(x.size), p05=float(np.percentile(x, 5)), median=float(np.median(x)),
                p95=float(np.percentile(x, 95))) if x.size else None


def intrinsics(meta):
    i = meta['camera']['color_intrinsics']
    return np.array([[i['fx'], 0, i['ppx']], [0, i['fy'], i['ppy']], [0, 0, 1.]])


def depth_at(capture, pixels):
    meta = json.loads((capture/'capture.json').read_text())
    K = intrinsics(meta)
    indices = []
    seen = set()
    for i, frame in enumerate(meta['frames']):
        if abs(frame['color_timestamp_ms']-frame['depth_timestamp_ms']) <= 10 and frame['depth_number'] not in seen:
            indices.append(i)
            seen.add(frame['depth_number'])
    uv = np.rint(pixels).astype(int)
    raw = np.load(capture/'frames.npz')['aligned_depth_u16'][indices][:, uv[:, 1], uv[:, 0]].astype(float)
    raw[raw == 0] = np.nan
    valid = np.isfinite(raw).sum(axis=0) >= max(10, .8*len(indices))
    z = np.full(len(uv), np.nan)
    z[valid] = np.nanmedian(raw[:, valid], axis=0)*meta['camera']['depth_unit_m']
    rays = np.column_stack([pixels, np.ones(len(pixels))]) @ np.linalg.inv(K).T
    ext = meta['camera']['T_color_depth']
    Rcd = np.array(ext['rotation_column_major']).reshape(3, 3).T
    tcd = np.array(ext['translation_m'])
    z = (z+(Rcd.T@tcd)[2])/(rays@Rcd[:, 2])
    return rays*z[:, None], valid, len(indices)


def main():
    request = json.loads((OUT/'current_placement_capture_request.json').read_text())
    current = Path(request['capture_directory'])
    im0 = cv2.imread(str(REFERENCE/'color.png'))
    im1 = cv2.imread(str(current/'color.png'))
    meta = json.loads((current/'capture.json').read_text())
    K = intrinsics(meta)
    assert np.allclose(K, intrinsics(json.loads((REFERENCE/'capture.json').read_text())))
    # Robot is stationary according to controller readback. Exclude batteries,
    # desktop objects and lower table, which the user may have handled.
    mask = np.zeros(im0.shape[:2], np.uint8)
    mask[:435, :] = 255
    mask[260:435, :430] = 0  # laptop screen/nearby handled items
    mask[:220, 1130:] = 0  # people at the far right of the background
    sift = cv2.SIFT_create(nfeatures=7000, contrastThreshold=.025)
    k0, d0 = sift.detectAndCompute(cv2.cvtColor(im0, cv2.COLOR_BGR2GRAY), mask)
    k1, d1 = sift.detectAndCompute(cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY), mask)
    bf = cv2.BFMatcher()
    fwd = [m for m, n in bf.knnMatch(d0, d1, k=2) if m.distance < .7*n.distance]
    reverse = {m.queryIdx:m.trainIdx for m, n in bf.knnMatch(d1, d0, k=2) if m.distance < .7*n.distance}
    matched = [m for m in fwd if reverse.get(m.trainIdx) == m.queryIdx]
    # SIFT can produce several descriptors at one pixel. Retain one correspondence
    # per 8 px source cell, avoiding duplicate orientations masquerading as samples.
    unique = {}
    for m in sorted(matched, key=lambda m:m.distance):
        cell = tuple((np.array(k0[m.queryIdx].pt)/8).astype(int))
        unique.setdefault(cell, m)
    matched = list(unique.values())
    uv0 = np.array([k0[m.queryIdx].pt for m in matched])
    uv1 = np.array([k1[m.trainIdx].pt for m in matched])
    pts0, good0, frames0 = depth_at(REFERENCE, uv0)
    pts1, good1, frames1 = depth_at(current, uv1)
    valid = good0 & np.all(np.isfinite(pts0), axis=1) & (pts0[:, 2] > .3) & (pts0[:, 2] < 4)
    uv0, uv1, pts0, pts1 = uv0[valid], uv1[valid], pts0[valid], pts1[valid]
    # Spatial checkerboard holdout, assigned before any fit/residual screening.
    cells = (uv0/80).astype(int)
    held = (cells[:, 0]+2*cells[:, 1]) % 4 == 0
    train = ~held
    cv2.setRNGSeed(714)
    ok, rv, tv, inliers = cv2.solvePnPRansac(pts0[train], uv1[train], K, np.zeros(5),
                                          iterationsCount=2000, reprojectionError=2., confidence=.999,
                                          flags=cv2.SOLVEPNP_EPNP)
    if not ok or len(inliers) < 50:
        raise RuntimeError('Insufficient robust RGB-D registration support')
    inlier_indices = np.flatnonzero(train)[inliers.ravel()]
    rv, tv = cv2.solvePnPRefineLM(pts0[inlier_indices], uv1[inlier_indices], K, np.zeros(5), rv, tv)
    R = cv2.Rodrigues(rv)[0]
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, tv.ravel()
    predicted = cv2.projectPoints(pts0, rv, tv, K, np.zeros(5))[0].reshape(-1, 2)
    err = np.linalg.norm(predicted-uv1, axis=1)
    mapped = pts0@R.T+tv.ravel()
    old_x = np.array(json.loads((OUT.parent/'hardware_package/metadata/red_handle_original_point.json').read_text())['T_base_color_optical'])
    new_x = old_x@np.linalg.inv(T)
    report = dict(status='DIAGNOSTIC_VIEW_REGISTRATION_NOT_MOTION_CALIBRATION',
                  accepted_for_motion=False, reference_capture=str(REFERENCE), current_capture=str(current),
                  same_camera_intrinsics=True, unique_rgb_matches=len(matched), valid_3d_matches=len(pts0),
                  matched_depth_frame_counts=[frames0, frames1], train_count=int(train.sum()), holdout_count=int(held.sum()),
                  train_inliers=len(inliers), T_current_camera_from_reference_camera=T.tolist(),
                  equivalent_rotation_deg=float(np.linalg.norm(rv)*180/np.pi),
                  equivalent_translation_mm=(tv.ravel()*1000).tolist(),
                  unchanged_view_pixel_displacement=stats(np.linalg.norm(uv1-uv0, axis=1)),
                  training_inlier_reprojection_px=stats(err[inlier_indices]),
                  holdout_all_reprojection_px=stats(err[held]),
                  holdout_within_2px_ratio=float(np.mean(err[held] <= 2)),
                  holdout_depth_z_residual_mm=stats((pts1[held, 2]-mapped[held, 2])*1000),
                  T_base_current_camera_candidate=new_x.tolist(),
                  physical_cause='Undetermined; user reports camera not moved. Registration estimates effective image-frame change only.',
                  caveats=['Source feature 3D uses low-resolution stereo depth with possible systematic errors.',
                           'Temporal/feature repeatability and reprojection fit do not establish physical absolute accuracy.',
                           'Candidate is not published as TF and does not replace saved hand-eye calibration.'])
    (OUT/'camera_registration_candidate.json').write_text(json.dumps(report, indent=2)+'\n')
    np.savez_compressed(OUT/'camera_registration_correspondences.npz', uv_reference=uv0, uv_current=uv1,
                        xyz_reference=pts0, xyz_current_depth=pts1, held=held, training_inliers=inlier_indices,
                        predicted_uv_current=predicted, error_px=err)
    view = np.hstack([im0, im1])
    for i in np.flatnonzero(held)[::2]:
        color = (0, 200, 0) if err[i] <= 2 else (0, 0, 255)
        a = tuple(np.rint(uv0[i]).astype(int)); b = tuple(np.rint(uv1[i]+[1280, 0]).astype(int))
        cv2.line(view, a, b, color, 1)
        cv2.circle(view, b, 3, color, 1)
    cv2.imwrite(str(OUT/'camera_registration_holdout.png'), view)
    print(json.dumps({k:v for k,v in report.items() if k not in ['caveats', 'T_base_current_camera_candidate', 'T_current_camera_from_reference_camera']}, indent=2))


if __name__ == '__main__':
    main()
