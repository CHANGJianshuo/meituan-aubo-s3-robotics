"""Regression: historical segfault logs must not count as successful shutdown."""
import importlib.util
from pathlib import Path
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
spec = importlib.util.spec_from_file_location(
    "check_launch_shutdown", PACKAGE / "scripts/check_launch_shutdown.py")
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


class ShutdownEvidenceTest(unittest.TestCase):
    def test_historical_move_group_crash_is_rejected(self):
        log = WORKSPACE / "docs/planning/evidence/20261012_032828/move_group_launch.log"
        errors = checker.shutdown_errors(log.read_text())
        self.assertTrue(any("no clean-exit" in error for error in errors))
        self.assertTrue(any("failed to terminate" in error for error in errors))

    def test_historical_control_shutdown_is_accepted(self):
        log = WORKSPACE / "docs/planning/evidence/20261012_032828/control_launch.log"
        self.assertEqual(checker.shutdown_errors(log.read_text()), [])

    def test_missing_evidence_is_rejected(self):
        for text in ("", "PASS all nodes started by this script exited\n"):
            with self.subTest(text=text):
                self.assertTrue(checker.shutdown_errors(text))

    def test_matching_pid_is_required(self):
        text = ("[INFO] [move_group-1]: process started with pid [100]\n"
                "[INFO] [move_group-1]: process has finished cleanly [pid 101]\n")
        self.assertTrue(checker.shutdown_errors(text))

    def test_signal_escalation_is_rejected_even_if_exit_is_clean(self):
        text = ("[INFO] [move_group-1]: process started with pid [100]\n"
                "[INFO] [move_group-1]: sending signal 'SIGTERM' to process[move_group-1]\n"
                "[INFO] [move_group-1]: process has finished cleanly [pid 100]\n")
        self.assertTrue(checker.shutdown_errors(text))


if __name__ == "__main__":
    unittest.main()
