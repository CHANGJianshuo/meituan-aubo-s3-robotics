"""ROS wiring regressions; run explicitly after sourcing a built workspace.

Uses real rclpy nodes, generated interfaces and DDS, with only Mock tool IO.
The config test stops after real parameter/config construction. The remaining
ActionServer/ActionClient bindings are separate issues, not a passing E2E claim.
OpenAI Codex assisted implementation, 2026-10-11.
"""

import threading

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("mtc_interfaces.action")

from rclpy.clock import Clock
from rclpy.executors import SingleThreadedExecutor

from mtc_interfaces.srv import TriggerUnlock
from mtc_manipulation import pick_place_server
from mtc_task.task_executor import TaskExecutorNode
from mtc_tool import codes
from mtc_tool.io_port import MockUnlockIo
from mtc_tool.strategy import PoseTarget
from mtc_tool.tool_manager_node import ToolManagerNode


@pytest.mark.parametrize("injected", [False, True], ids=["ros-clock", "injected-clock"])
def test_task_node_keeps_ros_clock_and_fsm_clock_independent(injected):
    rclpy.init(args=[])
    node = None
    fake_time = [123.0]
    try:
        node = TaskExecutorNode(clock=(lambda: fake_time[0]) if injected else None)
        assert isinstance(node.get_clock(), Clock)
        assert node.get_clock().now().nanoseconds > 0
        if injected:
            fake_time[0] += 2.0
            assert node.fsm.elapsed_in_state() == pytest.approx(2.0)
        # Exercise a real timer as well as the initial stamped state publish.
        rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize("max_age", [None, 0.75], ids=["default", "ros-override"])
def test_pickplace_entrypoint_maps_existing_ros_parameter(monkeypatch, max_age):
    nodes = []
    captured = {}
    create_node = rclpy.create_node
    config_type = pick_place_server.PickPlaceConfig

    def track_node(*args, **kwargs):
        node = create_node(*args, **kwargs)
        nodes.append(node)
        return node

    class ConfigCaptured(Exception):
        pass

    def capture_config(**kwargs):
        captured["config"] = config_type(**kwargs)
        # Stop before the separate, unresolved ROS Action adapter bindings.
        raise ConfigCaptured

    monkeypatch.setattr(rclpy, "create_node", track_node)
    monkeypatch.setattr(pick_place_server, "PickPlaceConfig", capture_config)
    args = [] if max_age is None else [
        "--ros-args", "-p", "perception_max_age_sec:=" + str(max_age)
    ]
    try:
        with pytest.raises(ConfigCaptured):
            pick_place_server.main(args)
        assert captured["config"].perception_max_age_s == pytest.approx(
            0.3 if max_age is None else max_age
        )
    finally:
        # The original B02 exits before main's cleanup block.
        if rclpy.ok():
            for node in nodes:
                node.destroy_node()
            rclpy.try_shutdown()


@pytest.mark.parametrize("gates_open", [False, True], ids=["interlock", "mock-accepted"])
def test_default_unlock_client_reaches_tool_manager_without_release_claim(gates_open):
    rclpy.init(args=[
        "--ros-args",
        "-p", "tool_type:=magnetic_latch_v2",
        "-p", "simulation_mode:=true",
        "-p", "unlock_io_backend:=mock",
        "-p", "seated_verified:=" + str(gates_open).lower(),
        "-p", "unloaded:=" + str(gates_open).lower(),
    ])
    manager = client_node = executor = thread = None
    try:
        manager = ToolManagerNode()
        assert isinstance(manager._io, MockUnlockIo)
        # Synthetic test evidence only; prepare the public tool lifecycle so
        # the service response tests the seat/unload gates, not DETACHED.
        pose = PoseTarget("base_link", (0.3, 0.0, 0.1))
        assert manager._manager.begin_acquire("wiring-test-battery", pose).ok
        manager._manager.notify_engage_complete()
        assert manager._manager.verify_attach(
            codes.EVIDENCE_GEOMETRY, "synthetic wiring test"
        ).verified
        assert manager._manager.begin_release(pose).ok
        executor = SingleThreadedExecutor()
        executor.add_node(manager)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        client_node = rclpy.create_node("unlock_wiring_probe")
        port = pick_place_server.RosToolPort(
            client_node, tool_type="magnetic_latch_v2", unlock_srv_type=TriggerUnlock
        )
        evidence = port.unlock(pulse_ms=175)
        assert "服务不可用" not in evidence.message
        assert "调用超时" not in evidence.message
        assert not evidence.ok
        assert not evidence.verified
        if gates_open:
            assert manager._io.history == (175,)
            assert evidence.released is None
        else:
            assert manager._io.history == ()
            assert "SAFETY_INTERLOCK" in evidence.message
            assert evidence.released is False
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if thread is not None:
            thread.join(timeout=3.0)
        for node in (client_node, manager):
            if node is not None:
                node.destroy_node()
        rclpy.try_shutdown()
