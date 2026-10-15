"""Diagnostic RGB size/range consistency; never changes the fixed anchor.

Uses the nominal battery CAD size and old camera orientation, so this is not an
independent physical calibration. Range is varied along the already selected
handle-centre image ray, not fitted to make the tool collision disappear.
"""
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

OUT = Path(__file__).resolve().parent
WS = OUT.parents[2]
POINT = json.loads((OUT.parent/'hardware_package/metadata/red_handle_original_point.json').read_text())
CFG = json.loads((OUT.parent/'inputs/scenario.json').read_text())


def main():
    capture = Path(POINT['capture_directory'])
    meta = json.loads((capture/'capture.json').read_text())
    intr = meta['camera']['color_intrinsics']
    K = np.array([[intr['fx'], 0, intr['ppx']], [0, intr['fy'], intr['ppy']], [0, 0, 1.]])
    X = np.array(POINT['T_base_color_optical'])
    ray = np.linalg.solve(K, [*POINT['pixel_uv_zero_based'], 1.])
    path = WS/'src/mtc_simulation/assets/meshes/battery.stl'
    blob = path.read_bytes()
    count = int.from_bytes(blob[80:84], 'little')
    assert len(blob) == 84+count*50
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    vertices = np.frombuffer(blob, dtype=dtype, count=count, offset=84)['vertices'].astype(float)
    assert np.allclose(vertices.max(axis=(0, 1))-vertices.min(axis=(0, 1)), [.07, .08, .08], atol=1e-5)
    rgb = cv2.imread(POINT['reference_image'])
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    observed = cv2.inRange(hsv, (0, 110, 50), (10, 255, 255)) | cv2.inRange(hsv, (170, 110, 50), (179, 255, 255))
    # A fixed ROI includes only the source battery and excludes red printed squares.
    roi = [840, 470, 945, 580]
    x1, y1, x2, y2 = roi
    obs = observed[y1:y2, x1:x2] > 0
    yaw = CFG['yaw']
    R = X[:3, :3].T @ Rotation.from_euler('z', yaw).as_matrix()
    local = (vertices-[0, 0, .08]) @ R.T

    # fillPoly applies parity between polygons; fill each triangle independently
    # for an opaque projected mesh union, avoiding cancellation of shared faces.
    def render_union(z):
        points = local+ray*z
        pixels = points @ K.T
        uv = np.rint(pixels[..., :2]/pixels[..., 2:]-[x1, y1]).astype(np.int32)
        mask = np.zeros(obs.shape, dtype=np.uint8)
        for triangle in uv:
            cv2.fillConvexPoly(mask, triangle, 255)
        return mask

    def score(mask):
        return float(np.count_nonzero((mask > 0) & obs)/np.count_nonzero((mask > 0) | obs))

    source_z = POINT['color_optical_z_median_m']
    original_mask = render_union(source_z)
    rows = []
    for z in np.arange(1.10, 1.352, .002):
        rows.append({'color_z_m': float(z), 'silhouette_iou': score(render_union(z))})
    best = max(rows, key=lambda r: r['silhouette_iou'])
    near = [r['color_z_m'] for r in rows if r['silhouette_iou'] >= best['silhouette_iou']-.01]
    frames = []
    for label, mask in [('Saved depth + nominal CAD', original_mask), ('Best RGB range + nominal CAD', render_union(best['color_z_m']))]:
        overlay = rgb[y1:y2, x1:x2].copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(overlay, contours, -1, (255, 255, 0), 1)
        panel = cv2.resize(overlay, (525, 550))
        panel = cv2.copyMakeBorder(panel, 50, 0, 0, 0, cv2.BORDER_CONSTANT, value=(245, 245, 245))
        cv2.putText(panel, label, (8, 31), 0, .65, (20, 20, 20), 2)
        frames.append(panel)
    cv2.imwrite(str(OUT/'battery_range_diagnostic.png'), np.hstack(frames))
    report = dict(status='DIAGNOSTIC_ONLY_NO_ANCHOR_CHANGE', nominal_battery_size_mm=[70, 80, 80],
                  observed_component_bbox_xywh=cv2.boundingRect(observed[y1:y2, x1:x2]),
                  original_color_z_m=source_z, original_iou=score(original_mask), best=best,
                  range_within_0p01_iou_of_best_m=[min(near), max(near)], samples=rows,
                  caveat='Conditional on nominal battery CAD dimensions, orientation and colour segmentation; the score plateau is not a confidence interval. It does not establish physical clearance.')
    (OUT/'battery_range_diagnostic.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'samples'}, indent=2))


if __name__ == '__main__':
    main()
