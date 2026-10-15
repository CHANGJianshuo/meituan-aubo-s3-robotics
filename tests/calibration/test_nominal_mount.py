"""Nominal mounting geometry: optical origin, axes, datums and estimation status."""
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/calibration'))
AVAILABLE = importlib.util.find_spec('derive_nominal_mount') is not None
if AVAILABLE:
    from derive_nominal_mount import derive
    from geometry import Transform, CalibrationError


def design():
    return {'data_kind': 'MOUNT_DESIGN', 'length_unit': 'm', 'camera_model': 'D435i',
            'front_glass_center_in_F': [0.100, 0, 0.040],
            'optical_axes_in_F_xyzw': [0, 0, 0, 1]}


class MountFeatureGate(unittest.TestCase):
    def test_nominal_mount_feature_exists(self):
        self.assertTrue(AVAILABLE, 'nominal mounting derivation not implemented')


@unittest.skipUnless(AVAILABLE, 'feature not implemented')
class MountDesignTests(unittest.TestCase):
    def assertVector(self, actual, expected):
        for a, e in zip(actual, expected):
            self.assertAlmostEqual(a, e, places=12)

    def test_optical_origin_is_left_imager_behind_front_glass(self):
        result = derive(design())
        self.assertVector(result['T_F_depth_optical']['translation'], [0.0825, 0, 0.0358])
        self.assertEqual(result['status'], 'ESTIMATED')
        self.assertFalse(result['deployment_ready'])
        self.assertFalse(result['motion_authorized'])

    def test_sensor_offset_rotates_with_camera_housing(self):
        d = design()
        z = math.sqrt(0.5)
        d['optical_axes_in_F_xyzw'] = [0, 0, z, z]
        self.assertVector(derive(d)['T_F_depth_optical']['translation'], [0.100, -0.0175, 0.0358])

    def test_ros_camera_axes_recover_designed_optical_axes(self):
        result = derive(design())
        d = result['T_F_camera_link_nominal']
        t = Transform(d['translation'], d['rotation_xyzw'])
        optical = t.compose(Transform([0, 0, 0], [-0.5, 0.5, -0.5, 0.5]))
        self.assertAlmostEqual(abs(optical.rotation[3]), 1)
        self.assertEqual(tuple(optical.translation), tuple(result['T_F_depth_optical']['translation']))

    def test_camera_link_axes_follow_ros_convention_independently(self):
        # REP-103 camera_link: X forward, Y left, Z up. With optical axes = F axes
        # (X right, Y down, Z forward seen from behind) that is +Z_F, -X_F, -Y_F.
        q = derive(design())['T_F_camera_link_nominal']['rotation_xyzw']
        rotation_only = Transform([0, 0, 0], q)
        self.assertVector(rotation_only.apply([1, 0, 0]), [0, 0, 1])
        self.assertVector(rotation_only.apply([0, 1, 0]), [-1, 0, 0])
        self.assertVector(rotation_only.apply([0, 0, 1]), [0, -1, 0])

    def test_real_data_and_mm_are_not_accepted_as_mount_design(self):
        for key, value in [('data_kind', 'REAL'), ('length_unit', 'mm')]:
            d = design()
            d[key] = value
            with self.subTest(key=key), self.assertRaises(CalibrationError):
                derive(d)

    def test_camera_model_selects_datasheet_zero_depth_offset(self):
        for model, z in [('D435', 0.0358), ('D435i', 0.0358), ('D435f', 0.0353), ('D435if', 0.0353)]:
            with self.subTest(model=model):
                d = design()
                d['camera_model'] = model
                self.assertVector(derive(d)['T_F_depth_optical']['translation'], [0.0825, 0, z])

    def test_missing_or_unknown_camera_model_rejected(self):
        for model in [None, 'D455', 'd435i', True]:
            with self.subTest(model=model):
                d = design()
                d['camera_model'] = model
                with self.assertRaisesRegex(CalibrationError, 'INVALID_DESIGN'):
                    derive(d)
        d = design()
        del d['camera_model']
        with self.assertRaisesRegex(CalibrationError, 'INVALID_DESIGN'):
            derive(d)

    def test_datum_assumptions_are_reported_and_corrections_shift_origin(self):
        result = derive(design())
        status = {a['item']: a['status'] for a in result['assumptions']}
        self.assertEqual(status['screw_axis_x_in_H_m'], 'ASSUMED')
        self.assertEqual(status['imager_axis_y_in_H_m'], 'ASSUMED')
        self.assertEqual(status['R_H_depth_optical'], 'ASSUMED')
        self.assertEqual(status['left_imager_from_screw_axis_m'], 'DATASHEET_REFERENCE')
        d = design()
        d.update(screw_axis_x_in_H_m=0.001, imager_axis_y_in_H_m=-0.0005)
        corrected = derive(d)
        self.assertVector(corrected['T_F_depth_optical']['translation'], [0.0835, -0.0005, 0.0358])
        status = {a['item']: a['status'] for a in corrected['assumptions']}
        self.assertEqual(status['screw_axis_x_in_H_m'], 'PROVIDED')
        self.assertEqual(corrected['status'], 'ESTIMATED')

    def test_implausible_scale_is_rejected(self):
        d = design()
        d['front_glass_center_in_F'] = [100, 0, 40]   # mm typed under a metre label
        with self.assertRaisesRegex(CalibrationError, 'IMPLAUSIBLE_DESIGN'):
            derive(d)
        d = design()
        d['screw_axis_x_in_H_m'] = 0.05               # outside a 90 x 25 mm housing
        with self.assertRaisesRegex(CalibrationError, 'IMPLAUSIBLE_DESIGN'):
            derive(d)

    def test_cli_rejections_exit_2_and_never_authorize_motion(self):
        command = [sys.executable, str(ROOT / 'scripts/calibration/derive_nominal_mount.py')]
        overflow = design()
        overflow['front_glass_center_in_F'][0] = 10**400
        cases = [(json.dumps(overflow), 'INVALID_VECTOR'),
                 ('[' * 200000 + ']' * 200000, 'INVALID_INPUT'),
                 ('{', 'INVALID_INPUT')]
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'design.json'
            for text, code in cases:
                with self.subTest(code=code, size=len(text)):
                    path.write_text(text, encoding='utf-8')
                    result = subprocess.run(command + [str(path)], capture_output=True, text=True)
                    self.assertEqual(result.returncode, 2, result.stderr[-300:])
                    payload = json.loads(result.stdout)
                    self.assertEqual((payload['status'], payload['code']), ('REJECTED', code))
                    self.assertIs(payload['motion_authorized'], False)
