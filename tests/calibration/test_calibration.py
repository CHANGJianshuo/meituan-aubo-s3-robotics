"""Calibration contract behavior; runs without ROS, numpy or hardware."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts' / 'calibration'))
AVAILABLE = importlib.util.find_spec('geometry') is not None and importlib.util.find_spec('contract') is not None
if AVAILABLE:
    from geometry import Transform, CalibrationError, ee_goal_from_tcp
    from contract import FrameGraph, Pose, transform_pose


def case():
    z = math.sqrt(0.5)
    return {
        'schema_version': 1, 'data_kind': 'SYNTHETIC', 'length_unit': 'm',
        'quaternion_order': 'xyzw', 'time_domain': 'synthetic',
        'transforms': [
            {'parent': 'base', 'child': 'wrist', 'status': 'ESTIMATED', 'static': False,
             'stamp_ns': 10_000_000_000, 'translation': [1, 0, 0], 'rotation_xyzw': [0, 0, z, z]},
            {'parent': 'wrist', 'child': 'camera', 'status': 'ESTIMATED', 'static': True,
             'translation': [0.1, 0, 0.05], 'rotation_xyzw': [0, 0, 0, 1]},
            {'parent': 'camera', 'child': 'optical', 'status': 'ESTIMATED', 'static': True,
             'translation': [0, 0, 0], 'rotation_xyzw': [-0.5, 0.5, -0.5, 0.5]},
        ],
        'pose': {'frame_id': 'optical', 'stamp_ns': 10_000_000_000,
                 'time_domain': 'synthetic', 'length_unit': 'm', 'position': [0, 0, 1],
                 'orientation_xyzw': [0, 0, 0, 1], 'valid': True, 'ring_pose_valid': True},
        'target_frame': 'base', 'now_ns': 10_100_000_000, 'max_age_ns': 300_000_000,
    }


class FeatureGate(unittest.TestCase):
    def test_calibration_feature_exists(self):
        self.assertTrue(AVAILABLE, 'SE(3) and calibration contract not implemented')


@unittest.skipUnless(AVAILABLE, 'feature not implemented')
class GeometryTests(unittest.TestCase):
    def assertVector(self, actual, expected):
        for a, e in zip(actual, expected):
            self.assertAlmostEqual(a, e, places=10)

    def test_rotation_and_translation_transform_position_and_orientation(self):
        z = math.sqrt(0.5)
        result = Transform([1, 2, 3], [0, 0, z, z]).compose(Transform([2, 0, 0], [0, 0, z, z]))
        self.assertVector(result.translation, [1, 4, 3])
        self.assertVector(result.rotation, [0, 0, 1, 0])

    def test_compose_matches_hand_computed_matrix_for_noncoaxial_rotations(self):
        # Expected values worked out by hand, independent of the quaternion code:
        # A = [Rx(+90) | 1,2,3], B = [Rz(+90) | 0,1,0]
        # Rx(90)*Rz(90) = [[0,-1,0],[0,0,-1],[1,0,0]], t_AB = Rx(90)*[0,1,0] + [1,2,3] = [1,2,4]
        z = math.sqrt(0.5)
        ab = Transform([1, 2, 3], [z, 0, 0, z]).compose(Transform([0, 1, 0], [0, 0, z, z]))
        self.assertVector(ab.apply([0, 0, 0]), [1, 2, 4])
        self.assertVector(ab.apply([1, 0, 0]), [1, 2, 5])
        self.assertVector(ab.apply([0, 1, 0]), [0, 2, 4])
        self.assertVector(ab.apply([0, 0, 1]), [1, 1, 4])
        # The reversed product Rz(90)*Rx(90) would send x to (0,1,0) and fail the second check.

    def test_inverse_recovers_position_for_noncommuting_rotations(self):
        z = math.sqrt(0.5)
        t = Transform([1, 2, 3], [z, 0, 0, z]).compose(Transform([-2, 1, 4], [0, 0, z, z]))
        self.assertVector(t.inverse().apply(t.apply([0.2, -0.4, 0.8])), [0.2, -0.4, 0.8])
        identity = t.inverse().compose(t)
        self.assertVector(identity.translation, [0, 0, 0])
        self.assertAlmostEqual(abs(identity.rotation[3]), 1)

    def test_tcp_compensation_is_inverse_and_includes_rotation(self):
        z = math.sqrt(0.5)
        wanted = Transform([1, 2, 3], [0, 0, z, z])
        offset = Transform([0.1, 0, 0.2], [z, 0, 0, z])
        ee = ee_goal_from_tcp(wanted, offset)
        recovered = ee.compose(offset)
        self.assertVector(recovered.translation, wanted.translation)
        self.assertVector(recovered.rotation, wanted.rotation)

    def test_zero_quaternion_rejected(self):
        with self.assertRaisesRegex(CalibrationError, 'QUATERNION'):
            Transform([0, 0, 0], [0, 0, 0, 0])

    def test_nonunit_quaternion_is_not_silently_normalized(self):
        with self.assertRaisesRegex(CalibrationError, 'QUATERNION'):
            Transform([0, 0, 0], [0, 0, 0, 2])

    def test_nan_and_infinity_rejected(self):
        for value in [float('nan'), float('inf')]:
            with self.subTest(value=value), self.assertRaises(CalibrationError):
                Transform([value, 0, 0], [0, 0, 0, 1])

    def test_dimensions_and_non_numeric_values_rejected(self):
        for vector in [[0, 0], [True, 0, 0], ['0', 0, 0], [10**400, 0, 0]]:
            with self.subTest(vector=vector), self.assertRaises(CalibrationError):
                Transform(vector, [0, 0, 0, 1])


@unittest.skipUnless(AVAILABLE, 'feature not implemented')
class ContractTests(unittest.TestCase):
    def convert(self, d=None, **overrides):
        d = d or case()
        args = dict(now_ns=d['now_ns'], max_age_ns=d['max_age_ns'], time_domain=d['time_domain'])
        args.update(overrides)
        return transform_pose(FrameGraph(d), Pose.from_dict(d['pose']), d['target_frame'], **args)

    def rejected(self, document, code, **overrides):
        with self.assertRaisesRegex(CalibrationError, code):
            self.convert(document, **overrides)

    def test_multihop_eye_in_hand_at_observation_time(self):
        result = self.convert()
        for a, e in zip(result['position'], [1, 1.1, 0.05]):
            self.assertAlmostEqual(a, e)
        for a, e in zip(result['orientation_xyzw'], [-math.sqrt(0.5), 0, 0, math.sqrt(0.5)]):
            self.assertAlmostEqual(a, e)
        self.assertEqual(result['frame_id'], 'base')
        self.assertEqual(result['stamp_ns'], 10_000_000_000)
        self.assertTrue(result['simulation_only'])
        self.assertEqual(result['calibration_status'], 'SYNTHETIC_ONLY')

    def test_reverse_graph_lookup_recovers_original_pose(self):
        d = case()
        p = self.convert(d)
        p.update(valid=True, ring_pose_valid=True)
        q = transform_pose(FrameGraph(d), Pose.from_dict(p), 'optical',
                           now_ns=d['now_ns'], max_age_ns=d['max_age_ns'], time_domain='synthetic')
        for a, e in zip(q['position'], [0, 0, 1]):
            self.assertAlmostEqual(a, e)
        self.assertAlmostEqual(abs(q['orientation_xyzw'][3]), 1)

    def test_camera_moves_with_wrist_instead_of_world_static(self):
        d = case()
        d['transforms'][0]['translation'] = [2, 0, 0]
        self.assertAlmostEqual(self.convert(d)['position'][0], 2)

    def test_unknown_extrinsic_is_rejected_not_identity(self):
        d = case()
        d['transforms'][1].update(status='UNKNOWN', translation=None, rotation_xyzw=None)
        self.rejected(d, 'UNKNOWN_CALIBRATION')

    def test_unknown_frame_and_disconnected_path_rejected(self):
        d = case()
        d['target_frame'] = 'unseen'
        self.rejected(d, 'MISSING_TF')
        d['transforms'].append({'parent': 'isolated', 'child': 'other', 'status': 'ESTIMATED',
                                'static': True, 'translation': [0, 0, 0], 'rotation_xyzw': [0, 0, 0, 1]})
        d['target_frame'] = 'other'
        self.rejected(d, 'MISSING_TF')

    def test_real_document_rejects_estimated_calibration(self):
        d = case()
        d.update(data_kind='REAL', time_domain='ros')
        d['pose']['time_domain'] = 'ros'
        self.rejected(d, 'UNMEASURED_CALIBRATION')

    def test_measured_snapshot_is_diagnostic_only(self):
        d = case()
        d.update(data_kind='REAL', time_domain='ros')
        d['pose']['time_domain'] = 'ros'
        for t in d['transforms']:
            t['status'] = 'MEASURED'
            t['evidence'] = 'unit-test fixture, not actual field evidence'
        p = self.convert(d)
        self.assertFalse(p['simulation_only'])
        self.assertFalse(p['motion_authorized'])

    def test_measured_edge_without_evidence_rejected(self):
        d = case()
        d['transforms'][1]['status'] = 'MEASURED'
        self.rejected(d, 'MISSING_EVIDENCE')

    def test_stale_future_and_zero_time_rejected(self):
        d = case()
        self.rejected(d, 'STALE_POSE', now_ns=11_000_000_000)
        self.rejected(d, 'FUTURE_POSE', now_ns=9_000_000_000)
        d['pose']['stamp_ns'] = 0
        self.rejected(d, 'INVALID_TIME')

    def test_equal_max_age_is_allowed(self):
        self.assertEqual(self.convert(now_ns=10_300_000_000)['age_ns'], 300_000_000)

    def test_dynamic_snapshot_at_wrong_time_rejected(self):
        d = case()
        d['transforms'][0]['stamp_ns'] += 1
        self.rejected(d, 'TF_TIME_MISMATCH')

    def test_static_edge_does_not_expire_like_dynamic_snapshot(self):
        d = case()
        d['transforms'][1]['stamp_ns'] = 1
        self.assertEqual(self.convert(d)['frame_id'], 'base')

    def test_wrong_clock_and_invalid_age_threshold_rejected(self):
        self.rejected(case(), 'CLOCK_MISMATCH', time_domain='ros')
        for age in [0, -1, True, 1.2]:
            with self.subTest(age=age):
                self.rejected(case(), 'INVALID_TIME', max_age_ns=age)

    def test_mm_is_rejected_instead_of_assumed_m(self):
        d = case()
        d['pose']['length_unit'] = 'mm'
        self.rejected(d, 'UNIT_MISMATCH')
        d = case()
        d['length_unit'] = 'mm'
        self.rejected(d, 'UNIT_MISMATCH')

    def test_false_ring_valid_and_missing_pose_fields_rejected(self):
        d = case()
        d['pose']['ring_pose_valid'] = False
        self.rejected(d, 'INVALID_POSE')
        for key in ['orientation_xyzw', 'frame_id', 'stamp_ns']:
            d = case()
            del d['pose'][key]
            self.rejected(d, 'INVALID_POSE')

    def test_missing_dynamic_time_and_zero_dynamic_time_rejected(self):
        for stamp in [None, 0]:
            d = case()
            d['transforms'][0]['stamp_ns'] = stamp
            self.rejected(d, 'INVALID_TIME')

    def test_duplicates_multiple_parents_and_cycles_rejected(self):
        for child, parent in [('camera', 'wrist'), ('camera', 'base'), ('base', 'optical')]:
            d = case()
            e = copy.deepcopy(d['transforms'][1])
            e.update(child=child, parent=parent)
            d['transforms'].append(e)
            self.rejected(d, 'INVALID_TF_TREE')

    def test_unknown_numeric_identity_not_allowed(self):
        d = case()
        d['transforms'][1]['status'] = 'UNKNOWN'
        self.rejected(d, 'UNKNOWN_HAS_NUMBERS')

    def test_unknown_data_kind_and_invalid_flags_rejected(self):
        d = case()
        d['data_kind'] = 'assume_real'
        self.rejected(d, 'INVALID_CONFIG')
        d = case()
        d['pose']['valid'] = 'true'
        self.rejected(d, 'INVALID_POSE')

    def test_schema_version_must_be_integer_one(self):
        for version in [True, 1.0, '1', 2]:
            with self.subTest(version=version):
                d = case()
                d['schema_version'] = version
                self.rejected(d, 'INVALID_CONFIG')

    def test_invalid_frame_names_rejected(self):
        for name in ['/base', 'base link', '', ' base', 7]:
            with self.subTest(name=name):
                d = case()
                d['target_frame'] = name
                self.rejected(d, 'INVALID_FRAME')
        d = case()
        d['transforms'][1]['child'] = 'camera '
        self.rejected(d, 'INVALID_FRAME')

    def test_whitespace_only_evidence_is_missing_evidence(self):
        d = case()
        d['transforms'][1].update(status='MEASURED', evidence='   ')
        self.rejected(d, 'MISSING_EVIDENCE')

    def test_invalid_now_rejected(self):
        for now in [0, -1, True, 10_100_000_000.0, None]:
            with self.subTest(now=now):
                self.rejected(case(), 'INVALID_TIME', now_ns=now)

    def test_stamp_beyond_int64_nanoseconds_rejected(self):
        d = case()
        d['pose']['stamp_ns'] = 2**63
        self.rejected(d, 'INVALID_TIME', now_ns=2**63 + 1)
        d = case()
        d['transforms'][0]['stamp_ns'] = 2**63
        self.rejected(d, 'INVALID_TIME')


@unittest.skipUnless(AVAILABLE, 'feature not implemented')
class CliTests(unittest.TestCase):
    def test_cli_success_and_unknown_extrinsic_exit_codes(self):
        command = [sys.executable, str(ROOT / 'scripts/calibration/check_calibration.py')]
        import tempfile
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'case.json'
            path.write_text(json.dumps(case()), encoding='utf-8')
            passed = subprocess.run(command + [str(path)], capture_output=True, text=True)
            self.assertEqual(passed.returncode, 0, passed.stderr)
            self.assertTrue(json.loads(passed.stdout)['simulation_only'])
            d = case()
            d['transforms'][1].update(status='UNKNOWN', translation=None, rotation_xyzw=None)
            path.write_text(json.dumps(d), encoding='utf-8')
            rejected = subprocess.run(command + [str(path)], capture_output=True, text=True)
            self.assertEqual(rejected.returncode, 2)
            self.assertEqual(json.loads(rejected.stdout)['code'], 'UNKNOWN_CALIBRATION')

    def test_cli_bad_input_never_prints_success(self):
        import tempfile
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'case.json'
            path.write_text('{', encoding='utf-8')
            result = subprocess.run([sys.executable, str(ROOT / 'scripts/calibration/check_calibration.py'), str(path)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertEqual(json.loads(result.stdout)['status'], 'REJECTED')

    def test_cli_deeply_nested_json_is_a_parse_rejection(self):
        import tempfile
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'nested.json'
            path.write_text('[' * 200000 + ']' * 200000, encoding='utf-8')
            result = subprocess.run([sys.executable, str(ROOT / 'scripts/calibration/check_calibration.py'), str(path)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2, result.stderr[-300:])
            payload = json.loads(result.stdout)
            self.assertEqual((payload['status'], payload['code']), ('REJECTED', 'INVALID_INPUT'))
            self.assertIs(payload['motion_authorized'], False)

    def test_cli_output_error_keeps_motion_unauthorized(self):
        import tempfile
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'case.json'
            path.write_text(json.dumps(case()), encoding='utf-8')
            blocked = path / 'cannot_be_a_directory' / 'out.json'
            result = subprocess.run([sys.executable, str(ROOT / 'scripts/calibration/check_calibration.py'),
                                     str(path), '--output', str(blocked)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            payload = json.loads(result.stdout)
            self.assertEqual(payload['code'], 'OUTPUT_ERROR')
            self.assertIs(payload['motion_authorized'], False)


if __name__ == '__main__':
    unittest.main()
