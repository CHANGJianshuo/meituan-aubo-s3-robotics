"""Current battery position candidates and model-origin consistency checks.

Diagnostic only: measured approximate vertical gap + current colour ray +
effective view registration. None of these candidates authorizes motion.
"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

OUT = Path(__file__).resolve().parent
RUN = OUT.parent
WS = RUN.parents[1]
PIXEL = np.array([808., 490.])
GAP = .016


def main():
    cfg = json.loads((RUN/'inputs/scenario.json').read_text())
    original = json.loads((RUN/'hardware_package/metadata/red_handle_original_point.json').read_text())
    request = json.loads((OUT/'current_placement_capture_request.json').read_text())
    capture = Path(request['capture_directory'])
    meta = json.loads((capture/'capture.json').read_text())
    intr = meta['camera']['color_intrinsics']
    K = np.array([[intr['fx'], 0, intr['ppx']], [0, intr['fy'], intr['ppy']], [0, 0, 1.]])
    X = np.array(json.loads((OUT/'camera_registration_candidate.json').read_text())['T_base_current_camera_candidate'])
    C = np.linalg.inv(X)
    ray = np.linalg.solve(K, [*PIXEL, 1.])
    state = json.loads((OUT/'object_placement_current_state.json').read_text())
    assert np.allclose(state['robot_configuration']['getTcpOffset'], 0)
    tcp = np.array(state['samples'][-1]['tcp'])
    F = np.eye(4)
    F[:3, :3], F[:3, 3] = Rotation.from_euler('xyz', tcp[3:]).as_matrix(), tcp[:3]
    tool = F.copy()
    tool[:3, :3] = F[:3, :3]@Rotation.from_euler('z', 90, degrees=True).as_matrix()
    normal = tool[:3, 2]
    under = tool[:3, 3]+tool[:3, :3]@np.array([.0325, 0, .113])

    def from_gap(uv, gap):
        ray = np.linalg.solve(K, [*uv, 1.])
        direction = X[:3, :3]@ray
        z = (normal@(under-X[:3, 3])-gap*normal[2])/(normal@direction)
        return X[:3, 3]+z*direction, z

    anchor, gap_z = from_gap(PIXEL, GAP)
    body_origin = anchor-[0, 0, .08]
    old = np.array(original['base_xyz_m'])
    old_gap = (normal@(under-old))/normal[2]
    face_point = tool[:3, :3].T@(anchor+[0, 0, GAP]-tool[:3, 3])
    assert -.010 <= face_point[0] <= .075 and abs(face_point[1]) <= .022
    assert abs(face_point[2]-.113) < 1e-10

    path = WS/'src/mtc_simulation/assets/meshes/battery.stl'
    blob = path.read_bytes()
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    vertices = np.frombuffer(blob, dtype=dtype, count=int.from_bytes(blob[80:84], 'little'), offset=84)['vertices'].astype(float)
    vmin, vmax = vertices.min(axis=(0, 1)), vertices.max(axis=(0, 1))
    assert np.allclose(vmin, [-.035, -.04, 0], atol=1e-8)
    assert np.allclose(vmax, [.035, .04, .08], atol=1e-8)
    world = ET.parse(RUN/'inputs/measured_world.sdf').getroot()
    sdf_pose = np.fromstring(world.findtext("world/model[@name='battery_A']/pose"), sep=' ')
    assert np.allclose(sdf_pose[:3], cfg['initial_objects']['A'])
    assert np.allclose(sdf_pose[:3]+[0, 0, .08], old)
    physics = json.loads((RUN/'replay_02/physics.jsonl').open().readline())['objects']['battery_A']['xyz']
    rgb = cv2.imread(str(capture/'color.png'))
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, (0, 110, 50), (10, 255, 255)) | cv2.inRange(hsv, (170, 110, 50), (179, 255, 255))
    blue = cv2.inRange(hsv, (85, 80, 45), (120, 255, 255))
    # Tool occludes part of the crossbar. Exclude observed blue pixels from the
    # RGB battery score; this does not certify physical occlusion/clearance.
    x1, y1, x2, y2 = 755, 470, 865, 575
    observed = red[y1:y2, x1:x2] > 0
    usable = blue[y1:y2, x1:x2] == 0
    R = C[:3, :3]@Rotation.from_euler('z', cfg['yaw']).as_matrix()
    local = (vertices-[0, 0, .08])@R.T

    def render(camera_anchor):
        xyz = local+camera_anchor
        uv = xyz@K.T
        uv = np.rint(uv[..., :2]/uv[..., 2:]-[x1, y1]).astype(np.int32)
        mask = np.zeros(observed.shape, np.uint8)
        for triangle in uv:
            cv2.fillConvexPoly(mask, triangle, 255)
        return mask

    def score(mask):
        p = mask > 0
        return float(np.count_nonzero(p & observed & usable)/np.count_nonzero((p | observed) & usable))

    scan = [{'color_z_m': float(z), 'iou': score(render(ray*z))} for z in np.arange(1.08, 1.322, .002)]
    best = max(scan, key=lambda row:row['iou'])
    old_camera_anchor = C[:3, :3]@old+C[:3, 3]
    panels = []
    candidates = [('Original saved position', old_camera_anchor),
                  ('Current RGB-size candidate', ray*best['color_z_m']),
                  ('Current 16 mm gap candidate', ray*gap_z)]
    scores = {}
    for label, candidate in candidates:
        mask = render(candidate)
        scores[label] = score(mask)
        view = rgb[y1:y2, x1:x2].copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(view, contours, -1, (255, 255, 0), 1)
        if label != candidates[0][0]:
            cv2.drawMarker(view, tuple((PIXEL-[x1, y1]).astype(int)), (0, 255, 0), cv2.MARKER_CROSS, 9, 1)
        view = cv2.resize(view, (440, 420), interpolation=cv2.INTER_NEAREST)
        view = cv2.copyMakeBorder(view, 65, 0, 0, 0, cv2.BORDER_CONSTANT, value=(245, 245, 245))
        cv2.putText(view, label, (8, 26), 0, .55, (25, 25, 25), 1)
        cv2.putText(view, f'Diagnostic only | IoU {scores[label]:.3f}', (8, 51), 0, .52, (25, 25, 25), 1)
        panels.append(view)
    cv2.imwrite(str(OUT/'current_object_placement_comparison.png'), np.hstack(panels))
    sensitivity = []
    for label, uv, gap in [('pixel_v_minus_1', PIXEL-[0, 1], GAP),
                           ('pixel_v_plus_1', PIXEL+[0, 1], GAP),
                           ('gap_minus_1mm', PIXEL, GAP-.001), ('gap_plus_1mm', PIXEL, GAP+.001)]:
        pt, _ = from_gap(uv, gap)
        sensitivity.append(dict(case=label, displacement_mm=((pt-anchor)*1000).tolist()))
    report = dict(status='ESTIMATED_CURRENT_PLACEMENT_PENDING_SUPPORT_HEIGHT_VALIDATION', accepted_for_motion=False,
                  reference_pixel_current_uv=PIXEL.tolist(), pixel_selection='Approximate geometric top-bar centre inferred from contour; centre lies at the tool occlusion boundary',
                  capture_directory=str(capture), gap_mm=GAP*1000,
                  gap_source='User approximate 16 mm; subsequent clarification: translated horizontally, vertical height should be unchanged',
                  T_base_current_camera_source=str(OUT/'camera_registration_candidate.json'),
                  old_model_vertical_gap_mm=old_gap*1000, candidate_color_z_m=gap_z,
                  candidate_handle_top_base_mm=(anchor*1000).tolist(),
                  candidate_battery_origin_base_mm=(body_origin*1000).tolist(),
                  delta_from_original_handle_mm=((anchor-old)*1000).tolist(),
                  underside_intersection_tool_xyz_mm=(face_point*1000).tolist(),
                  support_height_mismatch_mm=(body_origin[2]-cfg['table_z'])*1000,
                  T0_minus_candidate_source_support_mm=(cfg['targets']['T0'][2]-body_origin[2])*1000,
                  original_T0_base_mm=(np.array(cfg['targets']['T0'])*1000).tolist(),
                  source_chain_checks=dict(mesh_min_m=vmin.tolist(), mesh_max_m=vmax.tolist(),
                                           source_sdf_origin_m=sdf_pose[:3].tolist(),
                                           simulated_initial_origin_m=physics,
                                           sdf_to_initial_sim_difference_mm=((np.array(physics)-sdf_pose[:3])*1000).tolist(),
                                           handle_top_equals_origin_plus_80mm=True),
                  rgb_size_fit=best, silhouette_scores=scores, pixel_gap_sensitivity_only=sensitivity,
                  limitations=['Approximate gap, nominal tool geometry, controller absolute pose and view registration errors remain.',
                               'The selected pixel is at a blue/red mixed edge; its raw depth must not be treated as the battery surface depth.',
                               'RGB size fit additionally assumes nominal 70x80x80 mm battery and original upright yaw.',
                               'Do not add an invented support block to hide the support-height mismatch.',
                               'No map XY, T0 coordinates, original reference records or physical trajectory have been changed.'])
    (OUT/'current_object_placement.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ['limitations', 'source_chain_checks', 'pixel_gap_sensitivity_only']}, indent=2))


if __name__ == '__main__':
    main()
