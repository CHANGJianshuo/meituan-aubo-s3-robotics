"""Isolated Jazzy ROS fixture, NEVER a robot driver or hardware acceptance test.

Run only with --isolated-fixture in a network-isolated test container. The fixture
sets simulation_mode=false solely to test the real-mode UI's command gate.
No FSM, MoveIt, SDK or hardware is launched. Not included in offline test discovery.
"""

import argparse
from dataclasses import replace
import json
from pathlib import Path
import threading
import time
import unittest

from mtc_webui.config import RosConfig
from mtc_webui.mock_backend import ApiError
from mtc_webui.ros_backend import RosBackend
from mtc_webui.ros_transport import RosTransport


def wait_for(predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError("Timed out waiting for ROS fixture condition")


class Fixture:
    def __init__(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.action import ActionServer, GoalResponse, CancelResponse
        from mtc_interfaces.action import ExecuteTask
        from mtc_interfaces.msg import TaskState, RobotState, ToolState
        from mtc_interfaces.srv import RequestStop
        from sensor_msgs.msg import JointState, Image
        self.types = dict(task_state=TaskState, robot=RobotState, tool=ToolState, joints=JointState, color=Image, depth=Image)
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=94)
        self.node = Node("webui_isolated_fixture", context=self.context)
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.executor.add_node(self.node)
        self.group = ReentrantCallbackGroup()
        self.running = True
        self.emit = True
        self.state = "WAIT_TASK"
        self.task_id = ""
        self.stop_confirmed = True
        self.simulation_mode = False  # Deliberate test stimulus, NEVER hardware evidence.
        self.behavior = "success"
        self.goal_count = 0
        self.total_count = 0
        self.publishers = {}
        topics = dict(task_state="/mtc/task/state", robot="/mtc/robot/state", tool="/mtc/tool/state",
                      joints="/joint_states", color="/camera/camera/color/image_raw", depth="/camera/camera/aligned_depth_to_color/image_raw")
        for kind, msg_type in self.types.items():
            self.publishers[kind] = self.node.create_publisher(msg_type, topics[kind], 10)
        self.timer = self.node.create_timer(0.1, self.publish, callback_group=self.group)
        def goal_callback(request):
            self.goal_count += 1
            return GoalResponse.REJECT if self.behavior == "reject" else GoalResponse.ACCEPT
        self.server = ActionServer(self.node, ExecuteTask, "/mtc/task/execute", self.execute,
                                   goal_callback=goal_callback, cancel_callback=lambda handle: CancelResponse.ACCEPT,
                                   callback_group=self.group)
        def stop(request, response):
            response.request_delivered = True
            response.message = "isolated fixture delivery only; robot stop is independent"
            return response
        self.stop = self.node.create_service(RequestStop, "/mtc/bridge/request_stop", stop, callback_group=self.group)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def publish(self):
        if not self.emit:
            return
        import numpy as np
        import cv2
        for kind, msg_type in self.types.items():
            message = msg_type()
            message.header.stamp = self.node.get_clock().now().to_msg()
            if kind == "task_state":
                message.task_id, message.state = self.task_id, self.state
                message.total_count = self.total_count if self.task_id else 0
                message.detail = "ISOLATED ROS FIXTURE — no physical robot"
            elif kind == "robot":
                message.communication_ok = message.robot_ready = message.safety_normal = True
                message.stop_confirmed = self.stop_confirmed
                message.motion_active = not self.stop_confirmed
                message.simulation_mode = self.simulation_mode
                message.controller_mode = "ISOLATED_FIXTURE"
                message.detail = "Test stimulus only; not a hardware observation"
            elif kind == "tool":
                message.state, message.evidence_level, message.verified = 1, 3, True
                message.tool_type = "fixture"
            elif kind == "joints":
                message.name = [f"shoulder_joint_{n}" for n in range(1, 7)]
                message.position = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
            else:
                width, height = 640, 360
                message.width, message.height = width, height
                message.header.frame_id = "isolated_fixture_camera"
                if kind == "color":
                    frame = np.full((height, width, 3), (40, 55, 65), np.uint8)
                    cv2.putText(frame, "ISOLATED ROS FIXTURE", (55, 165), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (80, 215, 255), 2)
                    cv2.putText(frame, "NO PHYSICAL ROBOT", (105, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (225, 225, 225), 1)
                    message.encoding, message.step = "bgr8", width * 3
                else:
                    frame = np.tile(np.linspace(200, 1800, width, dtype=np.uint16), (height, 1))
                    message.encoding, message.step = "16UC1", width * 2
                message.data = frame.tobytes()
            self.publishers[kind].publish(message)

    def execute(self, handle):
        from mtc_interfaces.action import ExecuteTask
        self.task_id, self.state, self.stop_confirmed = handle.request.task_id, "EXECUTE_ITEM", False
        self.total_count = len(handle.request.ordered_colors)
        feedback = ExecuteTask.Feedback()
        feedback.state, feedback.substate, feedback.current_slot = "EXECUTE_ITEM", "FIXTURE_FEEDBACK", "T0"
        feedback.total_count = len(handle.request.ordered_colors)
        handle.publish_feedback(feedback)
        deadline = time.monotonic() + (5 if self.behavior == "cancel" else 1)
        while time.monotonic() < deadline and self.running:
            if handle.is_cancel_requested:
                handle.canceled()
                self.state, self.task_id = "WAIT_TASK", ""
                result = ExecuteTask.Result()
                result.message = "fixture canceled; stop deliberately unconfirmed"
                return result
            time.sleep(0.05)
        result = ExecuteTask.Result()
        result.success = self.behavior != "abort"
        result.completed_count = len(handle.request.ordered_colors) if result.success else 0
        result.completed_slots = (["T0"] if handle.request.mode == 1 else ["P1", "P2", "P3"]) if result.success else []
        result.error_code = 0 if result.success else 110
        result.message = "isolated ROS fixture result, not hardware evidence"
        handle.succeed() if result.success else handle.abort()
        self.state, self.task_id, self.stop_confirmed = "WAIT_TASK", "", True
        return result

    def close(self):
        self.running = False
        self.executor.shutdown(timeout_sec=2)
        self.server.destroy()
        self.node.destroy_node()
        self.context.shutdown()
        self.thread.join(timeout=2)


class RosIntegrationTests(unittest.TestCase):
    fixture = backend = None

    @classmethod
    def setUpClass(cls):
        cls.fixture = Fixture()
        cls.backend = RosBackend(RosConfig(domain_id=94, robot_publisher="/webui_isolated_fixture", task_publisher="/webui_isolated_fixture"))
        RosTransport(cls.backend).start()
        try:
            wait_for(lambda: cls.backend.snapshot()["operations"]["submit"], 15)
        except Exception:
            print(json.dumps(cls.backend.snapshot(), ensure_ascii=False, indent=2))
            cls.backend.close()
            cls.fixture.close()
            raise

    @classmethod
    def tearDownClass(cls):
        cls.backend.close()
        cls.fixture.close()

    def setUp(self):
        self.fixture.behavior, self.fixture.emit = "success", True
        self.fixture.stop_confirmed, self.fixture.simulation_mode = True, False
        wait_for(lambda: self.backend.snapshot()["operations"]["submit"])

    def submit(self):
        return self.backend.submit(dict(task_id=self._testMethodName, mode=1, ordered_colors=[1], timeout_ms=120000, placement_stability_sec=3))

    def test_01_topics_joints_camera_and_authority(self):
        wait_for(lambda: self.backend.snapshot()["camera"]["depth"].get("fresh"))
        self.fixture.node.get_logger().get_child("mtc_integration").warning("webui_jazzy_log_probe")
        wait_for(lambda: any(e["kind"] == "ROS_LOG" and "webui_jazzy_log_probe" in e["message"] for e in self.backend.snapshot()["events"]))
        snapshot = self.backend.snapshot()
        self.assertEqual(snapshot["robot"]["data"]["controller_mode"], "ISOLATED_FIXTURE")
        self.assertEqual(len(snapshot["joints"]["data"]["position"]), 6)
        self.assertIn("webui_isolated_fixture", snapshot["robot"]["source"])
        self.assertTrue(self.backend.camera_frame("color").startswith(b"\xff\xd8"))
        Path("/ui-evidence/connected-snapshot.json").write_text(json.dumps(snapshot, ensure_ascii=False, indent=2))

    def test_02_action_goal_feedback_result(self):
        count = self.fixture.goal_count
        self.submit()
        wait_for(lambda: self.backend.snapshot()["task"]["status"] == "EXECUTING")
        wait_for(lambda: self.backend.snapshot()["task"]["status"] == "SUCCEEDED")
        self.assertEqual(self.fixture.goal_count, count + 1)
        self.assertEqual(self.backend.snapshot()["task"]["result"]["completed_slots"], ["T0"])

    def test_03_rejected_goal(self):
        self.fixture.behavior = "reject"
        self.submit()
        wait_for(lambda: self.backend.snapshot()["task"]["status"] == "REJECTED")
        self.assertIsNone(self.backend.snapshot()["task"]["result"])

    def test_04_aborted_result(self):
        self.fixture.behavior = "abort"
        self.submit()
        wait_for(lambda: self.backend.snapshot()["task"]["status"] == "ABORTED")
        self.assertEqual(self.backend.snapshot()["task"]["result"]["error_code"], 110)

    def test_05_cancel_and_stop_are_independent(self):
        self.fixture.behavior = "cancel"
        ack = self.submit()
        wait_for(lambda: self.backend.snapshot()["task"]["status"] == "EXECUTING")
        self.backend.cancel(ack["task_id"], ack["goal_id"])
        wait_for(lambda: self.backend.snapshot()["task"]["status"] == "CANCELED")
        wait_for(lambda: self.backend.snapshot()["task"]["cancel_ack"] is not None)
        snapshot = self.backend.snapshot()
        self.assertTrue(snapshot["task"]["cancel_ack"]["request_delivered"])
        self.assertFalse(snapshot["robot"]["data"]["stop_confirmed"])
        self.backend.request_stop()
        wait_for(lambda: self.backend.snapshot()["stop_request"]["request_delivered"] is True)
        self.assertFalse(self.backend.snapshot()["robot"]["data"]["stop_confirmed"])
        self.assertFalse(self.backend.snapshot()["operations"]["submit"])

    def test_06_stale_messages_block_commands_and_camera(self):
        self.fixture.emit = False
        wait_for(lambda: not self.backend.snapshot()["robot"]["fresh"], 5)
        self.assertFalse(self.backend.snapshot()["operations"]["submit"])
        with self.assertRaises(ApiError):
            self.backend.camera_frame("color")
        self.fixture.emit = True
        wait_for(lambda: self.backend.snapshot()["operations"]["submit"])

    def test_07_simulation_flag_blocks_real_goals(self):
        self.fixture.simulation_mode = True
        wait_for(lambda: self.backend.snapshot()["robot"]["data"]["simulation_mode"])
        self.assertFalse(self.backend.snapshot()["operations"]["submit"])
        with self.assertRaises(ApiError):
            self.submit()
        self.fixture.simulation_mode = False

    def test_08_any_second_robot_publisher_blocks_ambiguous_data(self):
        from rclpy.node import Node
        from mtc_interfaces.msg import RobotState
        node = Node("untrusted_other_fixture", context=self.fixture.context)
        publisher = node.create_publisher(RobotState, "/mtc/robot/state", 10)
        try:
            wait_for(lambda: len(self.backend.snapshot()["connection"]["publishers"]["robot"]["candidates"]) == 2)
            message = RobotState()
            message.header.stamp = node.get_clock().now().to_msg()
            message.controller_mode = "WRONG_SOURCE"
            for _ in range(5):
                publisher.publish(message)
                time.sleep(0.1)
            self.assertIsNone(self.backend.snapshot()["robot"]["data"])
            self.assertFalse(self.backend.snapshot()["operations"]["submit"])
        finally:
            node.destroy_node()

    def test_09_duplicate_authority_is_unavailable(self):
        from rclpy.node import Node
        from mtc_interfaces.msg import RobotState
        node = Node("webui_isolated_fixture", context=self.fixture.context)
        node.create_publisher(RobotState, "/mtc/robot/state", 10)
        try:
            wait_for(lambda: not self.backend.snapshot()["robot"]["available"])
            self.assertFalse(self.backend.snapshot()["operations"]["submit"])
        finally:
            node.destroy_node()
        # This Jazzy middleware may lose the surviving node's name after a
        # duplicate-name node is destroyed. Do not relax authority to recover:
        # recreate the fixture so a fresh, identifiable endpoint is discovered.
        self.fixture.close()
        type(self).fixture = Fixture()
        try:
            wait_for(lambda: self.backend.snapshot()["operations"]["submit"])
        except Exception:
            print(json.dumps(self.backend.snapshot(), ensure_ascii=False, indent=2))
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isolated-fixture", action="store_true", required=True)
    parser.add_argument("--serve", action="store_true", help="Keep isolated fixture HTTP on loopback 8769 for browser QA")
    args = parser.parse_args()
    if args.serve:
        from mtc_webui.server import Server
        fixture = Fixture()
        fixture.behavior = "cancel"  # Five-second task gives browser QA time to cancel.
        backend = RosBackend(RosConfig(domain_id=94, robot_publisher="/webui_isolated_fixture", task_publisher="/webui_isolated_fixture"), "/ui-evidence/browser-session")
        RosTransport(backend).start()
        try:
            with Server(("127.0.0.1", 8769), backend) as server:
                server.serve_forever()
        finally:
            backend.close()
            fixture.close()
    else:
        Path("/ui-evidence").mkdir(exist_ok=True)
        unittest.main(argv=["ros_integration_check"], verbosity=2)


if __name__ == "__main__":
    main()
