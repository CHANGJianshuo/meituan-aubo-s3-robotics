#!/usr/bin/env python3
"""Compare the four V1 mounting orientations against a time-matched RGB-D capture."""
import argparse
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from fit_kinematics import forward, origins, poses_to_matrices


def box(center, size):
    return np.array(list(itertools.product(*[[-s/2, s/2] for s in size]))) + center


def inspect(inputs, point_file):
    state = json.loads((inputs/'captured_state.json').read_text())
    point = json.loads(point_file.read_text())
    capture = Path(state['camera_capture_dir'])
    meta = json.loads((capture/'capture.json').read_text())
    rgb = cv2.imread(str(capture/'color.png'))
    intr = meta['camera']['color_intrinsics']
    K = np.array([[intr['fx'], 0, intr['ppx']], [0, intr['fy'], intr['ppy']], [0, 0, 1.]])
    X = np.array(point['T_base_color_optical'])
    C = np.linalg.inv(X)
    F = forward(origins(ET.parse(inputs/'controller_matched_arm.urdf').getroot()), state['samples'][-1]['q'])[0]
    if np.max(np.abs(np.array(state['robot_configuration']['getTcpOffset']))) > 1e-12:
        raise ValueError('Controller TCP must be removed before treating its pose as flange pose')
    direct = poses_to_matrices([state['samples'][-1]['tcp']])[0]
    if np.max(np.abs(F-direct)) > 1e-6:
        raise ValueError('Controller FK and model disagree')
    if state['max_joint_drift_deg'] > .02:
        raise ValueError('Robot moved during capture')
    times = [f['host_received_ns']/1e9 for f in meta['frames']]
    overlap = min(max(times), state['ended_unix'])-max(min(times), state['captured_unix'])
    if overlap < 2.0:
        raise ValueError('No sufficient timestamp overlap between camera and controller captures')

    def project(points):
        camera = (C[:3, :3] @ points.T).T + C[:3, 3]
        pixels = camera @ K.T
        return pixels[:, :2]/pixels[:, 2:], camera[:, 2]

    flange_pixel = project(F[None, :3, 3])[0][0]
    cx, cy = np.rint(flange_pixel).astype(int)
    roi = list(map(int, [max(cx-105, 0), max(cy-70, 0), min(cx+75, rgb.shape[1]), min(cy+110, rgb.shape[0])]))
    x1, y1, x2, y2 = roi
    mask = cv2.inRange(cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV), (85, 80, 45), (120, 255, 255))
    clipped = np.zeros_like(mask)
    clipped[y1:y2, x1:x2] = mask[y1:y2, x1:x2]
    mask = clipped
    if np.count_nonzero(mask) < 500:
        raise ValueError('Blue V1 tool is not sufficiently visible')
    angles = np.linspace(0, 2*np.pi, 96)
    bodies = [np.array([[.0375*np.cos(a), .0375*np.sin(a), z] for z in [-.007, .008] for a in angles]),
              box([0, 0, .058], [.02, .03, .1]),
              box([.0325, 0, .1105], [.085, .044, .005]),
              box([.0725, 0, .1055], [.005, .044, .005])]
    rows, panels = [], []
    for yaw in [0, 90, 180, 270]:
        T = F.copy()
        T[:3, :3] = F[:3, :3] @ Rotation.from_euler('z', yaw, degrees=True).as_matrix()
        predicted = np.zeros_like(mask)
        overlay = rgb.copy()
        for body in bodies:
            uv, _ = project((T[:3, :3] @ body.T).T+T[:3, 3])
            hull = cv2.convexHull(np.rint(uv).astype(np.int32))
            cv2.fillConvexPoly(predicted, hull, 255)
            cv2.polylines(overlay, [hull], True, (0, 0, 255), 1, cv2.LINE_AA)
        iou = np.count_nonzero((mask > 0) & (predicted > 0))/np.count_nonzero((mask > 0) | (predicted > 0))
        tcp = T[:3, :3] @ np.array([.045, 0, .1105])+T[:3, 3]
        uv, z = project(tcp[None])
        rows.append(dict(flange_yaw_deg=yaw, ee_link_yaw_deg=(yaw-90) % 360,
                         silhouette_iou=float(iou), tcp_base_m=tcp.tolist(),
                         tcp_pixel=uv[0].tolist(), tcp_color_z_m=float(z[0])))
        cv2.drawMarker(overlay, tuple(np.rint(uv[0]).astype(int)), (0, 255, 255), cv2.MARKER_CROSS, 12, 1)
        tile = cv2.resize(overlay[y1:y2, x1:x2], (540, 540))
        cv2.rectangle(tile, (0, 0), (540, 55), (25, 25, 25), -1)
        cv2.putText(tile, f'flange {yaw} / ee {(yaw-90)%360}: IoU {iou:.3f}',
                    (10, 35), cv2.FONT_HERSHEY_SIMPLEX, .72, (255, 255, 255), 2)
        panels.append(tile)
    ranking = sorted(rows, key=lambda r: r['silhouette_iou'], reverse=True)
    best = ranking[0]
    if best['silhouette_iou'] < .65 or best['silhouette_iou']-ranking[1]['silhouette_iou'] < .15:
        raise ValueError('Mounting orientation is ambiguous')
    R = Rotation.from_euler('z', best['flange_yaw_deg'], degrees=True).as_matrix()
    T_flange_tool = np.eye(4)
    T_flange_tool[:3, :3] = R
    T_flange_tcp = T_flange_tool.copy()
    T_flange_tcp[:3, 3] = R @ np.array([.045, 0, .1105])
    T_flange_tcp[:3, :3] = R @ Rotation.from_euler('y', 90, degrees=True).as_matrix()
    u, v = np.rint(best['tcp_pixel']).astype(int)
    depth = np.load(capture/'frames.npz')['aligned_depth_u16'][:, v, u].astype(float)
    depth = depth[depth > 0]*meta['camera']['depth_unit_m']
    ext = meta['camera']['T_color_depth']
    Rcd = np.array(ext['rotation_column_major']).reshape(3, 3).T
    tcd = np.array(ext['translation_m'])
    ray = np.linalg.solve(K, [u, v, 1])
    color_z = (np.median(depth)+(Rcd.T @ tcd)[2])/(Rcd.T @ ray)[2]
    report = dict(method='CAD silhouette projection with controller FK and existing hand-eye calibration',
                  conclusion='DISCRETE_MOUNT_ORIENTATION_IDENTIFIED; TCP uses nominal original V1 CAD geometry',
                  camera_capture=str(capture), robot_capture=str(inputs/'captured_state.json'),
                  camera_robot_time_overlap_s=overlap, roi_xyxy=roi,
                  calibration_sha256=point['calibration_sha256'], candidates=ranking,
                  tool_mount_yaw_deg=best['ee_link_yaw_deg'],
                  flange_mount_yaw_deg=best['flange_yaw_deg'],
                  T_flange_tool=T_flange_tool.tolist(), T_flange_tcp=T_flange_tcp.tolist(),
                  depth_check=dict(pixel_uv=[int(u), int(v)], valid_frames=len(depth),
                                   observed_color_z_m=float(color_z), model_tcp_color_z_m=best['tcp_color_z_m'],
                                   difference_mm=float((color_z-best['tcp_color_z_m'])*1000),
                                   note='Visible material surface versus nominal internal TCP; corroboration only'),
                  independent_tcp_calibration=False,
                  controller_state=dict(robot_mode=state['samples'][-1]['robot_mode'],
                                        safety_mode=state['samples'][-1]['safety_mode'],
                                        reported_max_speed_deg_s=state['max_joint_speed_deg_s'],
                                        measured_joint_drift_deg=state['max_joint_drift_deg']))
    cv2.imwrite(str(inputs/'mount_comparison.png'), np.vstack([np.hstack(panels[:2]), np.hstack(panels[2:])]))
    cv2.imwrite(str(inputs/'observed_tool_mask.png'), mask)
    (inputs/'mount_identification.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs', type=Path, required=True)
    p.add_argument('--point', type=Path, required=True)
    args = p.parse_args()
    print(json.dumps(inspect(args.inputs, args.point), indent=2))
