"""Behavior checks for the UI fixture contract, independent of ROS and wall sleeps."""

import json
from pathlib import Path
import tempfile
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from mtc_webui.mock_backend import ApiError, MockBackend
from mtc_webui.server import Server


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def goal(task_id="test-1", mode=1, colors=None, scenario="success", timeout=120000):
    return {"task_id": task_id, "mode": mode, "ordered_colors": colors or ([3] if mode == 1 else [2, 1, 3]),
            "timeout_ms": timeout, "placement_stability_sec": 3, "scenario": scenario}


class BackendTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.backend = MockBackend(clock=self.clock, wall_clock=lambda: 1791700000 + self.clock.now)

    def tick(self, seconds):
        self.clock.now += seconds
        return self.backend.snapshot()

    def test_empty_snapshot_has_no_fabricated_task_or_camera(self):
        data = self.backend.snapshot()
        self.assertIsNone(data["task"])
        self.assertFalse(data["camera"]["available"])
        self.assertEqual(data["mode"], "mock")

    def test_basic_success_requires_full_observation_and_exports_result(self):
        self.backend.submit(goal())
        self.assertEqual(self.tick(9)["task"]["completed_count"], 0)
        data = self.tick(1.5)
        self.assertEqual(data["task"]["status"], "SUCCEEDED")
        self.assertEqual(data["task"]["result"]["completed_slots"], ["T0"])
        self.assertEqual(data["task"]["elapsed_ms"], 10500)

    def test_sequence_catches_up_after_disconnection_without_resubmission(self):
        self.backend.submit(goal(mode=2))
        self.assertEqual(self.tick(11)["task"]["completed_count"], 1)
        data = self.tick(30)
        self.assertEqual(data["task"]["result"]["completed_slots"], ["P1", "P2", "P3"])
        self.assertEqual(len(data["history"]), 1)
        self.assertEqual(data["task"]["elapsed_ms"], 30500)

    def test_goal_validation_rejects_ambiguous_colors_and_bad_types(self):
        bad = [goal(mode=2, colors=[1,1,2]), goal(mode=1, colors=[1,2]), goal(colors=[True]),
               goal(colors=[0]), goal(colors=[5]), goal(timeout=0), goal(task_id="../escape"),
               {**goal(), "placement_stability_sec": 2}, {**goal(), "mode": True},
               {**goal(), "scenario": []}, {**goal(), "extra": 1}]
        for payload in bad:
            with self.subTest(payload=payload), self.assertRaises(ApiError) as caught:
                self.backend.submit(payload)
            self.assertEqual(caught.exception.status, 400)
        self.assertEqual(self.backend.snapshot()["history"], [])

    def test_duplicate_and_concurrent_submits_are_rejected(self):
        self.backend.submit(goal())
        for payload, code in [(goal(), "DUPLICATE_TASK"), (goal(task_id="other"), "TASK_BUSY")]:
            with self.assertRaises(ApiError) as caught:
                self.backend.submit(payload)
            self.assertEqual(caught.exception.code, code)
        self.tick(20)
        with self.assertRaises(ApiError):
            self.backend.submit(goal())

    def test_simultaneous_requests_accept_only_one_goal(self):
        gate = threading.Barrier(3)
        results = []
        def submit(task_id):
            gate.wait()
            try:
                results.append(self.backend.submit(goal(task_id=task_id))["status"])
            except ApiError as error:
                results.append(error.code)
        threads = [threading.Thread(target=submit, args=(name,)) for name in ("first", "second")]
        for thread in threads:
            thread.start()
        gate.wait()
        for thread in threads:
            thread.join()
        self.assertCountEqual(results, ["ACCEPTED", "TASK_BUSY"])
        self.assertEqual(len(self.backend.snapshot()["history"]), 1)

    def test_cancel_result_does_not_imply_stop_confirmed(self):
        ack = self.backend.submit(goal())
        self.tick(3)
        reply = self.backend.cancel(ack["task_id"], ack["goal_id"])
        self.assertTrue(reply["request_delivered"])
        data = self.tick(1)
        self.assertEqual(data["task"]["status"], "CANCELED")
        self.assertFalse(data["robot"]["data"]["stop_confirmed"])
        self.assertFalse(data["operations"]["submit"])
        self.assertTrue(self.tick(1)["operations"]["submit"])

    def test_wrong_or_old_goal_cannot_cancel_current_task(self):
        ack = self.backend.submit(goal())
        with self.assertRaises(ApiError):
            self.backend.cancel(ack["task_id"], "wrong")
        self.tick(20)
        self.backend.submit(goal(task_id="next"))
        with self.assertRaises(ApiError):
            self.backend.cancel(ack["task_id"], ack["goal_id"])
        self.assertEqual(self.backend.snapshot()["task"]["status"], "ACCEPTED")

    def test_unconfirmed_cancel_blocks_next_task_until_separate_feedback(self):
        ack = self.backend.submit(goal(scenario="unconfirmed_stop"))
        self.tick(3)
        self.backend.cancel(ack["task_id"], ack["goal_id"])
        data = self.tick(4)
        self.assertEqual(data["task"]["result"]["error_code"], 520)
        self.assertFalse(data["operations"]["submit"])
        self.backend.confirm_stop()
        self.assertTrue(self.backend.snapshot()["operations"]["submit"])
        self.assertEqual(self.backend.task_record(ack["task_id"])["result"]["error_code"], 520)

    def test_timeout_and_failure_are_preserved_with_goal_ids(self):
        self.backend.submit(goal(scenario="timeout", timeout=2000))
        data = self.tick(5)
        self.assertEqual(data["task"]["result"]["error_code"], 220)
        self.tick(1)
        self.backend.submit(goal(task_id="failure", scenario="perception_failure"))
        data = self.tick(5)
        self.assertEqual(data["task"]["result"]["error_code"], 110)
        self.assertEqual(len(data["history"]), 2)
        for event in data["task"]["events"]:
            self.assertEqual(event["goal_id"], data["task"]["goal_id"])

    def test_stale_and_missing_samples_cannot_advertise_submit(self):
        for condition in ("stale", "missing"):
            self.backend.configure(condition)
            data = self.backend.snapshot()
            self.assertFalse(data["operations"]["submit"])
            with self.assertRaises(ApiError):
                self.backend.submit(goal())
        self.assertIsNone(data["robot"]["data"])
        self.backend.configure("normal")
        self.assertTrue(self.backend.snapshot()["operations"]["submit"])

    def test_snapshots_are_detached_and_old_result_cannot_replace_current(self):
        self.backend.submit(goal())
        data = self.tick(20)
        data["task"]["result"]["success"] = False
        self.backend.submit(goal(task_id="next"))
        old = self.backend.task_record("test-1")
        self.assertTrue(old["result"]["success"])
        self.assertEqual(self.backend.snapshot()["task"]["task_id"], "next")

    def test_raw_records_are_persisted_and_existing_output_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "session"
            backend = MockBackend(output, clock=self.clock)
            backend.submit(goal())
            self.clock.now = 20
            backend.snapshot()
            record = json.loads((output / "test-1.json").read_text())
            self.assertEqual(record["result"]["completed_count"], 1)
            self.assertTrue((output / "events.jsonl").exists())
            with self.assertRaises(FileExistsError):
                MockBackend(output)


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.server = Server(("127.0.0.1", 0), MockBackend())
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def request(self, path, body=None, headers=None):
        headers = headers or {}
        if body is not None:
            headers.setdefault("Content-Type", "application/json")
        data = None if body is None else json.dumps(body).encode()
        return urlopen(Request(self.url + path, data=data, headers=headers), timeout=3)

    def test_assets_contract_submit_export_and_cancel(self):
        with self.request("/") as response:
            self.assertIn("Mock 演练", response.read().decode())
            self.assertIn("default-src 'self'", response.headers["Content-Security-Policy"])
        with self.request("/api/tasks", goal()) as response:
            self.assertEqual(response.status, 202)
            ack = json.load(response)
        with self.request("/api/tasks/test-1/cancel", {"goal_id": ack["goal_id"]}) as response:
            self.assertTrue(json.load(response)["request_delivered"])
        with self.request("/api/export") as response:
            self.assertIn("attachment", response.headers["Content-Disposition"])
            self.assertEqual(json.load(response)["tasks"][0]["task_id"], "test-1")

    def test_cross_origin_and_unknown_host_are_rejected(self):
        for headers in ({"Origin": "https://example.invalid"}, {"Host": "example.invalid"}, {"Origin": "null"}):
            with self.subTest(headers=headers), self.assertRaises(HTTPError) as caught:
                self.request("/api/tasks", goal(), headers)
            self.assertEqual(caught.exception.code, 403)

    def test_no_filesystem_or_shell_routes(self):
        for path in ("/../materials/manifest.json", "/api/term", "/api/tasks/not-present"):
            with self.subTest(path=path), self.assertRaises(HTTPError) as caught:
                self.request(path)
            self.assertEqual(caught.exception.code, 404)


if __name__ == "__main__":
    unittest.main()
