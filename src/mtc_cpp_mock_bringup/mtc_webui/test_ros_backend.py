"""Offline checks of ROS ownership, stale feedback and hardware command boundaries."""

from dataclasses import replace
import json
from pathlib import Path
import tempfile
import threading
import unittest
from urllib.request import Request, urlopen
from urllib.error import HTTPError

from mtc_webui.config import RosConfig
from mtc_webui.mock_backend import ApiError
from mtc_webui.ros_backend import RosBackend
from mtc_webui.ros_transport import RosTransport, encode_image
from mtc_webui.server import Server
from mtc_webui.test_mock_backend import Clock, goal


class Transport:
    def __init__(self):
        self.calls = []

    def enqueue(self, *args):
        self.calls.append(args)


class RealBackendTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.clock.now = 1
        self.backend = RosBackend(RosConfig(domain_id=92), clock=self.clock)
        self.backend.transport = Transport()
        self.stamp = 1

    def ready(self, **robot_overrides):
        self.backend.connection_update(connected=True, reason="test fixture", action_ready=True, stop_ready=True)
        self.stamp += 1
        robot = dict(communication_ok=True, robot_ready=True, motion_active=False, safety_normal=True,
                     stop_confirmed=True, simulation_mode=False, **{})
        robot.update(robot_overrides)
        samples = dict(robot=robot, tool=dict(state=1, evidence_level=3, verified=True),
                       task_state=dict(task_id="", state="WAIT_TASK", substate="", completed_count=0,
                                       total_count=0, last_error_code=0, detail="ready"))
        for kind, data in samples.items():
            self.backend.observe(kind, data, "/test-publisher", self.stamp, 0)

    def submit(self, task_id="real-1"):
        payload = goal(task_id)
        del payload["scenario"]
        return self.backend.submit(payload)

    def result(self, identity, status=4):
        result = dict(success=status == 4, completed_count=1 if status == 4 else 0,
                      completed_slots=["T0"] if status == 4 else [], error_code=0, message="fixture result")
        self.backend.result(identity, status, result)

    def test_default_is_offline_without_fabricated_devices(self):
        data = self.backend.snapshot()
        self.assertEqual(data["mode"], "ros")
        self.assertIsNone(data["task"])
        self.assertIsNone(data["robot"]["data"])
        self.assertFalse(data["operations"]["submit"])
        with self.assertRaises(ApiError):
            self.submit()

    def test_ack_is_not_acceptance_and_time_never_creates_result(self):
        self.ready()
        ack = self.submit()
        self.assertEqual(ack["status"], "SUBMITTING")
        self.assertEqual(self.backend.transport.calls[0][0], "submit")
        self.clock.now += 20
        data = self.backend.snapshot()
        self.assertEqual(data["task"]["status"], "UNKNOWN")
        self.assertIsNone(data["task"]["result"])
        self.ready()
        with self.assertRaises(ApiError):
            self.submit("other")
        self.backend.accepted(ack["goal_id"], True)
        self.assertEqual(self.backend.snapshot()["task"]["status"], "ACCEPTED")

    def test_result_and_stop_and_post_result_state_are_independent(self):
        self.ready()
        identity = self.submit()["goal_id"]
        self.backend.accepted(identity, True)
        self.clock.now += 0.5
        self.result(identity)
        self.assertFalse(self.backend.snapshot()["operations"]["submit"])
        self.clock.now += 0.01
        self.ready(stop_confirmed=False)
        self.assertFalse(self.backend.snapshot()["operations"]["submit"])
        self.ready()
        self.assertTrue(self.backend.snapshot()["operations"]["submit"])
        self.assertEqual(self.backend.task_record("real-1")["elapsed_ms"], 500)

    def test_mismatched_result_is_unknown_and_not_retryable(self):
        self.ready()
        identity = self.submit()["goal_id"]
        self.backend.result(identity, 4, dict(success=False, completed_count=0, completed_slots=[], error_code=210, message="bad"))
        self.assertEqual(self.backend.snapshot()["task"]["status"], "UNKNOWN")
        self.assertFalse(self.backend.snapshot()["operations"]["submit"])

    def test_cancel_requires_owned_goal_and_real_ack_does_not_confirm_stop(self):
        self.ready()
        identity = self.submit()["goal_id"]
        with self.assertRaises(ApiError):
            self.backend.cancel("real-1", identity)
        self.backend.accepted(identity, True)
        with self.assertRaises(ApiError):
            self.backend.cancel("real-1", "foreign")
        ack = self.backend.cancel("real-1", identity)
        self.assertIsNone(ack["request_delivered"])
        self.backend.cancel_reply(identity, True, "fixture accepted")
        self.ready(stop_confirmed=False)
        self.result(identity, 5)
        data = self.backend.snapshot()
        self.assertEqual(data["task"]["status"], "CANCELED")
        self.assertFalse(data["robot"]["data"]["stop_confirmed"])
        self.assertFalse(data["operations"]["submit"])

    def test_late_and_foreign_feedback_do_not_corrupt_result(self):
        self.ready()
        identity = self.submit()["goal_id"]
        self.backend.feedback("other-goal", dict(state="WRONG"))
        self.assertEqual(self.backend.snapshot()["task"]["state"], "")
        self.result(identity)
        self.backend.feedback(identity, dict(state="WRONG"))
        self.backend.accepted(identity, True)
        self.assertEqual(self.backend.snapshot()["task"]["status"], "SUCCEEDED")

    def test_feedback_index_does_not_invent_completed_count(self):
        self.ready()
        identity = self.submit()["goal_id"]
        self.backend.feedback(identity, dict(current_index=1, state="EXECUTE_ITEM", substate="TRANSFERRING"))
        self.assertEqual(self.backend.snapshot()["task"]["completed_count"], 0)

    def test_old_repeated_zero_and_future_header_stamps_are_not_fresh(self):
        for stamp, source_age in [(0, 0), (1, 4000), (1, -2000)]:
            with self.subTest(stamp=stamp, age=source_age):
                self.backend.invalidate("robot")
                self.backend.observe("robot", {}, "robot", stamp, source_age)
                self.assertFalse(self.backend.snapshot()["robot"]["fresh"])
        self.ready()
        old = self.backend.samples["robot"]
        self.clock.now += 4
        self.backend.observe("robot", old["data"], old["source"], old["stamp_ns"], 0)
        self.assertFalse(self.backend.snapshot()["robot"]["fresh"])
        self.assertFalse(self.backend.snapshot()["operations"]["submit"])

    def test_simulation_faults_stale_data_and_missing_action_block_hardware_goals(self):
        for overrides in [dict(simulation_mode=True), dict(safety_normal=False), dict(robot_ready=False), dict(motion_active=True)]:
            self.ready(**overrides)
            with self.assertRaises(ApiError):
                self.submit()
        self.ready()
        self.backend.connection_update(action_ready=False)
        with self.assertRaises(ApiError):
            self.submit()
        self.assertEqual(self.backend.transport.calls, [])

    def test_external_task_is_observed_without_claiming_goal_ownership(self):
        self.ready()
        data = dict(task_id="external", state="EXECUTE_ITEM", substate="", completed_count=1, total_count=3, last_error_code=0, detail="external")
        self.backend.observe("task_state", data, "/test-publisher", self.stamp + 1, 0)
        snapshot = self.backend.snapshot()
        self.assertIsNone(snapshot["task"])
        self.assertEqual(snapshot["external_task"]["task_id"], "external")
        self.assertFalse(snapshot["operations"]["submit"])
        self.assertFalse(snapshot["operations"]["cancel"])

    def test_concurrent_commands_only_queue_one_goal(self):
        self.ready()
        gate = threading.Barrier(3)
        results = []
        def submit(name):
            gate.wait()
            try:
                results.append(self.submit(name)["status"])
            except ApiError:
                results.append("REJECTED")
        threads = [threading.Thread(target=submit, args=(name,)) for name in ("a", "b")]
        for thread in threads:
            thread.start()
        gate.wait()
        for thread in threads:
            thread.join()
        self.assertCountEqual(results, ["SUBMITTING", "REJECTED"])
        self.assertEqual(len(self.backend.transport.calls), 1)

    def test_software_stop_has_separate_delivery_and_no_stop_state_mutation(self):
        self.ready(stop_confirmed=False)
        request = self.backend.request_stop()
        self.backend.stop_reply(request["request_id"], True, "delivered")
        data = self.backend.snapshot()
        self.assertTrue(data["stop_request"]["request_delivered"])
        self.assertFalse(data["robot"]["data"]["stop_confirmed"])

    def test_readonly_blocks_all_commands_and_real_mode_rejects_demo_fields(self):
        self.ready()
        with self.assertRaises(ApiError):
            self.backend.submit(goal())
        self.backend.config = replace(self.backend.config, read_only=True)
        with self.assertRaises(ApiError):
            self.submit()
        with self.assertRaises(ApiError):
            self.backend.request_stop()
        self.assertFalse(self.backend.snapshot()["operations"]["stop"])

    def test_config_and_lazy_start_do_not_fall_back_to_mock(self):
        with self.assertRaises(ValueError):
            RosConfig(domain_id=True)
        with self.assertRaises(ValueError):
            RosConfig(robot_publisher="")
        backend = RosBackend()
        RosTransport(backend).start()
        self.assertIn("ROS_DOMAIN_ID", backend.snapshot()["connection"]["reason"])
        self.assertEqual(backend.snapshot()["mode"], "ros")

    def test_task_persistence_and_expired_camera(self):
        with tempfile.TemporaryDirectory() as directory:
            self.backend = RosBackend(RosConfig(domain_id=92), Path(directory) / "session", clock=self.clock)
            self.backend.transport = Transport()
            self.ready()
            identity = self.submit()["goal_id"]
            self.clock.now += 0.5
            self.result(identity)
            record = json.loads((self.backend.output_dir / "real-1.json").read_text())
            self.assertTrue(record["result"]["success"])
            self.assertEqual(record["elapsed_ms"], 500)
            self.backend.frame("color", {}, b"jpeg", "camera", 10, 0)
            self.assertEqual(self.backend.camera_frame("color"), b"jpeg")
            self.clock.now += 4
            with self.assertRaises(ApiError):
                self.backend.camera_frame("color")


class ImageTests(unittest.TestCase):
    def test_rgb_padding_and_big_endian_depth_are_decoded(self):
        from types import SimpleNamespace
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.skipTest("OpenCV/NumPy are optional outside camera deployment")
        message = SimpleNamespace(width=2, height=2, step=8, is_bigendian=False, encoding="rgb8",
                                  data=bytes([255, 0, 0] * 2 + [0, 0]) * 2)
        jpeg, meta = encode_image(message, "color")
        pixels = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.assertGreater(int(pixels[0, 0, 2]), 240)
        self.assertLess(int(pixels[0, 0, 0]), 10)
        message.encoding, message.step, message.is_bigendian = "16UC1", 4, True
        message.data = np.array([[0, 1000], [2000, 500]], dtype=">u2").tobytes()
        jpeg, meta = encode_image(message, "depth")
        self.assertIn("0–2 m", meta["preview"])
        self.assertTrue(jpeg.startswith(b"\xff\xd8"))
        message.step = 1
        with self.assertRaises(ValueError):
            encode_image(message, "depth")


class RealHttpTests(unittest.TestCase):
    def test_ros_routes_reject_demo_mutations_and_tunnel_host_is_explicit(self):
        backend = RosBackend()
        server = Server(("127.0.0.1", 0), backend, tunnel_ports=[8768])
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_port}"
        try:
            with urlopen(Request(base + "/api/snapshot", headers={"Host": "localhost:8768", "Origin": "http://localhost:8768"})) as response:
                self.assertEqual(json.load(response)["mode"], "ros")
            for path, body, code in [("/api/demo/confirm-stop", {}, 404), ("/api/demo/condition", {"condition":"normal"}, 404), ("/api/stop", {}, 409)]:
                with self.subTest(path=path), self.assertRaises(HTTPError) as caught:
                    urlopen(Request(base + path, data=json.dumps(body).encode(), headers={"Content-Type":"application/json"}))
                self.assertEqual(caught.exception.code, code)
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == "__main__":
    unittest.main()
