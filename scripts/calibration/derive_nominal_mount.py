#!/usr/bin/env python3
"""D435-family layout estimate in physical design frame F; does not publish TF.

Datums, RealSense D400 datasheet 337029-017 (2024-10), printed page numbers:
- p94 Table 4-16: depth start point (depth = 0 plane) behind the front cover glass.
- p146 Figure 10-9: "(17.5) FROM CENTERLINE OF 1/4-20 TO LEFT IMAGER", a reference dimension.
  The drawing does not dimension the 1/4-20 axis, the rear M3 holes or the lens centreline
  against the 90 x 25 mm housing, so placing them relative to the housing centre H is an
  assumption that the output reports explicitly.
"""
import argparse
import json
import math
from pathlib import Path

from geometry import CalibrationError, Transform, finite_vector

ZERO_DEPTH_BEHIND_GLASS_M = {'D435': 0.0042, 'D435i': 0.0042, 'D435f': 0.0047, 'D435if': 0.0047}
LEFT_IMAGER_FROM_SCREW_AXIS_M = 0.0175   # towards optical -X, viewed from behind the camera
# camera_link <- depth optical: realsense2_description depth_optical_joint rpy(-pi/2, 0, -pi/2).
CAMERA_LINK_TO_DEPTH_OPTICAL_XYZW = (-0.5, 0.5, -0.5, 0.5)
MAX_GLASS_OFFSET_M = 1.0         # a flange bracket longer than this is most likely mm typed as m
MAX_HOUSING_CORRECTION_M = 0.02  # datum corrections must stay inside the 90 x 25 mm housing


def as_dict(t):
    return {'translation': list(t.translation), 'rotation_xyzw': list(t.rotation)}


def rejection(code, message):
    return {'status': 'REJECTED', 'code': code, 'message': message, 'motion_authorized': False}


def housing_correction(document, key, assumed_basis):
    """Optional measured offset of a datum from H along H axes; absent means assumed zero."""
    if key not in document:
        return 0.0, {'item': key, 'value': 0.0, 'status': 'ASSUMED', 'basis': assumed_basis}
    value = finite_vector([document[key]], 1, key)[0]
    if abs(value) > MAX_HOUSING_CORRECTION_M:
        raise CalibrationError('IMPLAUSIBLE_DESIGN', f'{key} must be within +-{MAX_HOUSING_CORRECTION_M} m')
    return value, {'item': key, 'value': value, 'status': 'PROVIDED',
                   'basis': 'caller-supplied correction; keep its measurement record'}


def derive(document):
    if not isinstance(document, dict) or document.get('data_kind') != 'MOUNT_DESIGN':
        raise CalibrationError('INVALID_DESIGN', 'require MOUNT_DESIGN; no measured calibration is inferred')
    if document.get('length_unit') != 'm':
        raise CalibrationError('UNIT_MISMATCH', 'design positions must be m')
    model = document.get('camera_model')
    if not isinstance(model, str) or model not in ZERO_DEPTH_BEHIND_GLASS_M:
        raise CalibrationError('INVALID_DESIGN', 'camera_model must be one of ' + ', '.join(ZERO_DEPTH_BEHIND_GLASS_M))
    # H: housing centre on the front cover glass, axes parallel to the optical axes
    # (X right, Y down, Z forward, viewed from behind the camera).
    t_f_h = Transform(document.get('front_glass_center_in_F'), document.get('optical_axes_in_F_xyzw'))
    if math.sqrt(sum(v * v for v in t_f_h.translation)) > MAX_GLASS_OFFSET_M:
        raise CalibrationError('IMPLAUSIBLE_DESIGN',
                               f'front glass more than {MAX_GLASS_OFFSET_M} m from the flange; mm entered as m?')
    screw_x, screw_note = housing_correction(
        document, 'screw_axis_x_in_H_m',
        '1/4-20 centreline assumed on the housing width centre; not dimensioned in Fig.10-9 '
        '(the official ROS description makes the same approximation)')
    imager_y, imager_note = housing_correction(
        document, 'imager_axis_y_in_H_m',
        'lens centreline (-C-) assumed at the housing height centre; not dimensioned in Fig.10-9')
    zero_depth = ZERO_DEPTH_BEHIND_GLASS_M[model]
    t_h_d = Transform([screw_x - LEFT_IMAGER_FROM_SCREW_AXIS_M, imager_y, -zero_depth], [0, 0, 0, 1])
    t_f_d = t_f_h.compose(t_h_d)
    t_l_d = Transform([0, 0, 0], CAMERA_LINK_TO_DEPTH_OPTICAL_XYZW)
    return {'status': 'ESTIMATED', 'data_kind': 'MOUNT_DESIGN', 'camera_model': model,
            'reference_frame': 'flange_design_F', 'length_unit': 'm',
            'quaternion_order': 'xyzw', 'T_F_front_glass_center': as_dict(t_f_h),
            'T_H_depth_optical_nominal': as_dict(t_h_d),
            'T_F_depth_optical': as_dict(t_f_d),
            'T_F_camera_link_nominal': as_dict(t_f_d.compose(t_l_d.inverse())),
            'assumptions': [
                screw_note, imager_note,
                {'item': 'R_H_depth_optical', 'value': 'identity', 'status': 'ASSUMED',
                 'basis': 'housing axes parallel to the calibrated optical axes; no datasheet angular tolerance'},
                {'item': 'left_imager_from_screw_axis_m', 'value': LEFT_IMAGER_FROM_SCREW_AXIS_M,
                 'status': 'DATASHEET_REFERENCE',
                 'basis': 'p146 Fig.10-9 reference dimension, from the 1/4-20 centreline towards optical -X'},
                {'item': 'zero_depth_behind_glass_m', 'value': zero_depth, 'status': 'DATASHEET',
                 'basis': f'p94 Table 4-16 for {model}; H lies on the front cover glass, '
                          'about 0.1 mm behind the front aluminium plate'},
                {'item': 'camera_link_origin', 'value': 'depth optical origin', 'status': 'WRAPPER_CONVENTION',
                 'basis': 'realsense-ros README: camera_link = left IR = depth origin; verify installed wrapper TF'}],
            'deployment_ready': False, 'motion_authorized': False,
            'pending': ['verify flange_design_F relative to deployed robot frame',
                        'check V1 tool and bracket geometry, holes, occlusion and cable space',
                        'measure rear M3 holes against the 1/4-20 axis or left imager if the bracket uses them',
                        'confirm the camera model label (f variants have a different zero-depth offset)',
                        'read actual depth/color/internal TF from this D435i',
                        'verify installation and calibration errors against task tolerance']}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('design', type=Path)
    args = parser.parse_args()
    try:
        result = derive(json.loads(args.design.read_text(encoding='utf-8-sig')))
    except (CalibrationError, OSError, ValueError, TypeError) as error:
        print(json.dumps(rejection(getattr(error, 'code', 'INVALID_INPUT'), str(error)), ensure_ascii=True))
        return 2
    except RecursionError:
        print(json.dumps(rejection('INVALID_INPUT', 'JSON nesting exceeds the parser recursion limit'), ensure_ascii=True))
        return 2
    except Exception as error:  # an unexpected failure must still be a rejection, never exit 0/1
        print(json.dumps(rejection('INTERNAL_ERROR', f'{type(error).__name__}: {error}'), ensure_ascii=True))
        return 2
    print(json.dumps(result, indent=2, ensure_ascii=True, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
