"""Detect model/TF disagreements without treating a fake capture as real calibration."""
import importlib.util
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/calibration'))
AVAILABLE = importlib.util.find_spec('audit_fake_model') is not None
if AVAILABLE:
    from audit_fake_model import audit
EVIDENCE = ROOT / 'docs/planning/evidence/20261011_075828'


class FakeAuditGate(unittest.TestCase):
    def test_audit_available(self):
        self.assertTrue(AVAILABLE)


@unittest.skipUnless(AVAILABLE, 'audit not implemented')
class FakeAuditTests(unittest.TestCase):
    def changed_capture(self, filename, before, after):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        destination = Path(temp.name)
        for source in EVIDENCE.iterdir():
            if source.suffix in ('.txt', '.urdf', '.srdf'):
                shutil.copyfile(source, destination / source.name)
        target = destination / filename
        content = target.read_text(encoding='utf-8')
        self.assertIn(before, content)
        target.write_text(content.replace(before, after), encoding='utf-8')
        return destination

    def test_saved_fake_chain_matches_fk_with_rounding_and_no_motion_permission(self):
        result = audit(EVIDENCE)
        self.assertTrue(result['all_passed'], result)
        for actual, expected in zip(result['T_base_ee_q0']['translation_m'], [0, -0.2562, 0.8072]):
            self.assertAlmostEqual(actual, expected, places=10)
        self.assertFalse(result['motion_authorized'])
        self.assertEqual(result['evidence_kind'], 'FAKE_RUNTIME_CAPTURE')
        self.assertFalse(result['real_calibration_verified'])

    def test_omitting_ee_quarter_turn_disagrees_with_saved_orientation(self):
        directory = self.changed_capture('aubo_S3_fake_expanded.urdf',
                                        '1.5707963267948966', '0.0')
        self.assertFalse(audit(directory)['all_passed'])

    def test_wrong_log_translation_is_detected(self):
        directory = self.changed_capture('tf2_echo_base_link_to_ee_link.txt',
                                        '-0.256', '0.256')
        self.assertFalse(audit(directory)['all_passed'])

    def test_missing_valid_samples_cannot_pass(self):
        directory = self.changed_capture('tf2_echo_world_to_ee_link.txt', 'At time', 'No time')
        self.assertFalse(audit(directory)['all_passed'])

    def test_wrong_moveit_tip_is_detected(self):
        directory = self.changed_capture('aubo_robot.srdf', 'tip_link="ee_link"',
                                        'tip_link="wrist3_Link"')
        self.assertFalse(audit(directory)['all_passed'])


if __name__ == '__main__':
    unittest.main()
