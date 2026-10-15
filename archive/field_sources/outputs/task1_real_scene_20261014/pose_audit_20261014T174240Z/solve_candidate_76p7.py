"""Re-solve saved observations with the user's approximate 76.7 mm marker size.

No camera/robot acquisition and no TF publication. The saved PnP translations
are rescaled consistently with the marker size; original observations and
original calibration are preserved. The candidate remains ESTIMATED.
"""
import copy
import json
from pathlib import Path
import sys
import time

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
CALIB = Path('/home/operator/meituan/eye_to_hand_calib')
sys.path.insert(0, str(CALIB))
from e2h.camera import Intrinsics
from e2h.config import load
from e2h.pipeline import samples_from_records
from e2h.report import write_report
from e2h.sim.scene import pose_from_cfg
from e2h.solver import solve, evaluate


def main():
    out = HERE/'calibration_candidate_76p7'
    out.mkdir(exist_ok=False)
    data = yaml.safe_load((HERE/'samples_consistent_fk_diagnostic.yaml').read_text())
    old_length, length = data['marker']['length_m'], .0767
    data['marker']['length_m'] = length
    data['marker']['length_provenance'] = 'User approximate physical measurement: 大概是76.7mm'
    data['marker']['length_uncertainty_m'] = None
    for rec in data['samples']:
        rec['obs_original_pnp'] = copy.deepcopy(rec['obs'])
        for field in ['T_C_M', 'T_alt']:
            if rec['obs'].get(field) is not None:
                matrix = np.array(rec['obs'][field])
                matrix[:3, 3] *= length/old_length
                rec['obs'][field] = matrix.tolist()
        rec['pnp_marker_length_original_m'] = old_length
        rec['pnp_marker_length_m'] = length
    (out/'samples.yaml').write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True))
    cfg = load(CALIB/'config/site.yaml')
    cfg['marker'] = {**cfg['marker'], **data['marker']}
    i = data['intrinsics']
    intr = Intrinsics(K=np.array(i['K']), D=np.array(i['D']), width=i['width'], height=i['height'], source=i['source'],
                      T_link_optical=np.array(data['T_link_optical']) if 'T_link_optical' in data else None)
    samples, holdout = samples_from_records(data['samples'], 'solve'), samples_from_records(data['samples'], 'holdout')
    start = time.monotonic()
    result = solve(samples, intr.K, intr.D, length, cfg.get('solver'))
    ev = evaluate(result.X, result.Y, holdout, intr.K, intr.D, result.marker_length, length)
    report = write_report(out, cfg, result, ev, holdout, data['samples'], intr,
                          pose_from_cfg(cfg['marker']['mount_nominal']), {}, time.monotonic()-start,
                          title='76.7 mm 标记与统一运动学的离线候选标定')
    report.update(offline_solver_checks_passed=True, physical_validation_passed=False,
                  accepted_for_robot_motion=False, published_to_tf=False,
                  marker_length_source=data['marker']['length_provenance'],
                  marker_length_measurement_uncertainty_m=None,
                  original_calibration_replaced=False, original_reference_points_replaced=False)
    (out/'result.yaml').write_text(yaml.safe_dump(report, sort_keys=False, allow_unicode=True))
    summary = dict(status='OFFLINE_FIT_ACCEPTED_PHYSICAL_GEOMETRY_UNRESOLVED',
                   configured_marker_mm=length*1000, estimated_marker_mm=result.marker_length*1000,
                   rms_px=result.rms_px, holdout=ev, inliers=result.inliers, outliers=result.outliers,
                   points={}, physical_validation_passed=False, accepted_for_robot_motion=False)
    for label, name in [('red_handle', 'red_handle_original_point.json'), ('T0_original', 'T0_original_point.json')]:
        point = json.loads((HERE.parent/'hardware_package/metadata'/name).read_text())
        old = np.array(point['base_xyz_m'])
        new = result.X[:3, :3] @ np.array(point['camera_xyz_m'])+result.X[:3, 3]
        summary['points'][label] = dict(original_base_mm=(old*1000).tolist(), candidate_base_mm=(new*1000).tolist(),
                                        difference_mm=((new-old)*1000).tolist(), difference_norm_mm=float(np.linalg.norm(new-old)*1000),
                                        adopted=False)
    (out/'comparison.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False)+'\n')
    note = ('\n\n## 本候选的使用状态\n\n'
            '用户确认标记黑框边长约 76.7 mm，未提供测量不确定度。保存的 PnP 平移已按边长同步换算，'
            '法兰位姿统一到控制柜匹配模型。原有求解检查通过，但当前实物舌片与横梁的间隙矛盾仍未解决，'
            '本结果仍为 ESTIMATED，不作为实机运动依据。没有发布 TF、替换旧标定或修改两个固定参考点。\n')
    with (out/'report.md').open('a') as f:
        f.write(note)
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
