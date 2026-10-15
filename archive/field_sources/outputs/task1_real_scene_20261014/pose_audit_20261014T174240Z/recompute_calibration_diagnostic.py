"""Offline sensitivity audit. Never publishes or replaces a calibration.

First run the existing solver with its unchanged acceptance checks. Separately
refine the original inlier set to isolate the effect of consistent controller
FK. Rejected candidates remain diagnostic and must not be used for motion.
"""
import copy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

OUT = Path(__file__).resolve().parent
WS = OUT.parents[2]
CALIB = Path('/home/operator/meituan/eye_to_hand_calib')
SESSION = CALIB/'data/sessions/20261014_094346'
sys.path[:0] = [str(CALIB), str(WS/'scripts/task1_real_scene')]
from e2h.pipeline import samples_from_records
from e2h.pnp import marker_points
from e2h.solver import CalibrationError, evaluate, refine, solve
from fit_kinematics import forward, origins, JOINTS


def main():
    original = yaml.safe_load((SESSION/'samples.yaml').read_text())
    old = yaml.safe_load((SESSION/'offline/result.yaml').read_text())
    records = original['samples']
    corrected = copy.deepcopy(original)
    model = OUT.parent/'inputs/controller_matched_arm.urdf'
    q = [[r['joints'][j] for j in JOINTS] for r in records]
    poses = forward(origins(ET.parse(model).getroot()), q)
    for record, pose in zip(corrected['samples'], poses):
        record['T_B_E_original'] = record['T_B_E']
        record['T_B_E'] = pose.tolist()
        record['T_B_E_source'] = 'controller_matched_fk_offline; not a new physical observation'
    (OUT/'samples_consistent_fk_diagnostic.yaml').write_text(yaml.safe_dump(corrected, sort_keys=False))
    K, D = np.array(original['intrinsics']['K']), np.array(original['intrinsics']['D'])
    length = original['marker']['length_m']
    X0 = np.array(old['X_base_to_optical']['matrix'])
    Y0 = np.array(old['Y_flange_to_marker']['matrix'])
    excluded = {sid for sid, _ in old['quality']['outliers']}
    rows = {}
    for label, data in [('original_mixed_fk', original), ('consistent_controller_fk', corrected)]:
        samples = samples_from_records(data['samples'], 'solve')
        holdout = samples_from_records(data['samples'], 'holdout')
        try:
            result = solve(samples, K, D, length)
            strict = {'accepted': True, 'rms_px': result.rms_px}
        except CalibrationError as error:
            strict = {'accepted': False, 'reason': str(error)}
        # Diagnostic optimization does not alter the strict acceptance outcome.
        selected = [s for s in samples if s.sid not in excluded]
        X, Y, scale, fitted = refine(X0, Y0, selected, K, D, marker_points(length),
                                    huber_px=1., scale0=old['marker_length_estimated_m']/length,
                                    estimate_scale=True)
        point = json.loads((OUT.parent/'hardware_package/metadata/red_handle_original_point.json').read_text())
        candidate_point = X[:3, :3] @ np.array(point['camera_xyz_m'])+X[:3, 3]
        rows[label] = dict(strict_default_solver=strict, diagnostic_only=True,
                           original_inlier_ids=[s.sid for s in selected],
                           rms_px=float(np.sqrt(np.mean(fitted.fun**2))),
                           holdout=evaluate(X, Y, holdout, K, D, length*scale, length),
                           estimated_marker_length_mm=float(length*scale*1000),
                           camera_translation_change_mm=((X[:3, 3]-X0[:3, 3])*1000).tolist(),
                           camera_rotation_change_deg=float(np.degrees(Rotation.from_matrix(X0[:3, :3].T@X[:3, :3]).magnitude())),
                           red_anchor_candidate_base_mm=(candidate_point*1000).tolist(),
                           red_anchor_candidate_change_mm=((candidate_point-np.array(point['base_xyz_m']))*1000).tolist(),
                           candidate_X_base_color=X.tolist(), candidate_Y_flange_marker=Y.tolist())
    report = dict(status='DIAGNOSTIC_ONLY_NOT_ACCEPTED_FOR_MOTION',
                  original_calibration_unchanged=True, user_fixed_anchor_unchanged=True,
                  source_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                 for path in [SESSION/'samples.yaml', SESSION/'offline/result.yaml', model]},
                  cases=rows,
                  caveat='Joint-model consistency is only one input defect. No independent physical TCP/hand-eye check was performed. The diagnostic refinement does not bypass or change solver acceptance checks.')
    (OUT/'calibration_recompute_diagnostic.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps({k:{a:b for a,b in v.items() if not a.startswith('candidate_')} for k,v in rows.items()}, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
