"""PickPlace Action Server 外壳的离线测试（不依赖 ROS 运行时）。

本机只有 ROS 2 Humble 且任务禁止 colcon build，无法生成 mtc_interfaces 消息/动作类型。
因此这里用**替身模块**（rclpy / rclpy.action / rclpy.qos / mtc_interfaces.*）注入 sys.path，
在不安装 ROS 的前提下验证：
    1. pick_place_server 模块可导入且无 rclpy 相关语法/结构错误；
    2. 真实 ROS 适配器类可实例化（订阅/客户端走替身）；
    3. PickPlaceServer 用 Fake 端口 + 替身 Action 类型可端到端跑通一次抓放闭环；
    4. 取消路径不会因为收到 cancel 就直接返回成功。

注意：这些替身只验证**代码结构**，不能替代在 Jazzy 环境下的 colcon build 与真实 DDS 验证。
"""

from __future__ import annotations

import sys
import types
from pathlib import Path
from typing import Any, List, Optional

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from mtc_manipulation.fakes import FakeClock, FakeMotion, FakePerception, FakePlanner, FakeTool  # noqa: E402
from mtc_manipulation.pick_place_fsm import (  # noqa: E402
    PickPlaceConfig,
    PickPlaceState,
    TOOL_MAGNETIC_LATCH_V2,
)
from mtc_manipulation.tool_strategy import MagneticLatchV2Strategy, PassiveHookV1Strategy  # noqa: E402


# ---------------------------------------------------------------------------
# 替身模块安装
# ---------------------------------------------------------------------------


class _FakeFuture:
    def __init__(self, value: Any = None) -> None:
        self._value = value

    def done(self) -> bool:
        return True

    def result(self) -> Any:
        return self._value


class _FakeQoSProfile:
    def __init__(self, **kwargs: Any) -> None:
        self.kwargs = kwargs


class _FakeNode:
    """最小节点替身：记录发布/订阅，并提供一个假时钟。"""

    def __init__(self) -> None:
        self.published: List[Any] = []
        self.subscriptions: List[Any] = []
        self.clients: List[Any] = []
        self.logger = _FakeLogger()

    # -- ROS 接口 --------------------------------------------------------
    def create_publisher(self, msg_type: Any, topic: str, depth: int = 10) -> Any:
        node = self

        class _Pub:
            def __init__(self) -> None:
                self.msg_type = msg_type
                self.topic = topic

            def publish(self, msg: Any) -> None:
                node.published.append((topic, msg))

        return _Pub()

    def create_subscription(self, msg_type: Any, topic: str, callback: Any, qos: Any = None) -> Any:
        self.subscriptions.append((topic, msg_type, callback, qos))
        return object()

    def create_client(self, srv_type: Any, name: str) -> Any:
        self.clients.append((name, srv_type))
        return object()

    def get_clock(self) -> Any:
        class _Clock:
            @staticmethod
            def now() -> Any:
                class _Time:
                    nanoseconds = 0

                    @staticmethod
                    def to_msg() -> Any:
                        return object()

                return _Time()

        return _Clock()

    def get_logger(self) -> Any:
        return self.logger

    def declare_parameter(self, name: str, value: Any) -> None:
        return None


class _FakeLogger:
    def __init__(self) -> None:
        self.records: List[str] = []

    def _log(self, level: str, message: str) -> None:
        self.records.append("%s: %s" % (level, message))

    def info(self, message: str) -> None:
        self._log("info", message)

    def warning(self, message: str) -> None:
        self._log("warning", message)

    def error(self, message: str) -> None:
        self._log("error", message)

    def fatal(self, message: str) -> None:
        self._log("fatal", message)

    def debug(self, message: str) -> None:
        self._log("debug", message)


def _install_stub_modules() -> None:
    if "rclpy" in sys.modules:
        return

    rclpy = types.ModuleType("rclpy")
    rclpy.spin = lambda node: None
    rclpy.spin_once = lambda node, timeout_sec=0.0: None
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None
    rclpy.create_node = lambda name: _FakeNode()

    action_mod = types.ModuleType("rclpy.action")

    class _ActionClient:
        def __init__(self, node: Any, action_type: Any, name: str) -> None:
            self.node = node
            self.action_type = action_type
            self.name = name

        def wait_for_server(self, timeout_sec: float = 0.0) -> bool:
            return False

    action_mod.ActionClient = _ActionClient
    rclpy.action = action_mod

    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.QoSProfile = _FakeQoSProfile
    qos_mod.QoSReliabilityPolicy = types.SimpleNamespace(RELIABLE="RELIABLE")
    qos_mod.QoSHistoryPolicy = types.SimpleNamespace(KEEP_LAST="KEEP_LAST")
    qos_mod.QoSDurabilityPolicy = types.SimpleNamespace(VOLATILE="VOLATILE")

    # mtc_interfaces 替身（只需存在，供 pick_place_server.main 的延迟导入路径使用）
    pkg = types.ModuleType("mtc_interfaces")
    action_pkg = types.ModuleType("mtc_interfaces.action")
    msg_pkg = types.ModuleType("mtc_interfaces.msg")
    srv_pkg = types.ModuleType("mtc_interfaces.srv")
    action_pkg.PickPlace = type("PickPlace", (), {})
    action_pkg.ExecuteJointMove = type("ExecuteJointMove", (), {})
    msg_pkg.BatteryDetectionArray = type("BatteryDetectionArray", (), {})
    msg_pkg.ToolState = type("ToolState", (), {})
    srv_pkg.TriggerUnlock = type("TriggerUnlock", (), {})
    pkg.action = action_pkg
    pkg.msg = msg_pkg
    pkg.srv = srv_pkg

    sys.modules.setdefault("rclpy", rclpy)
    sys.modules.setdefault("rclpy.action", action_mod)
    sys.modules.setdefault("rclpy.qos", qos_mod)
    sys.modules.setdefault("mtc_interfaces", pkg)
    sys.modules.setdefault("mtc_interfaces.action", action_pkg)
    sys.modules.setdefault("mtc_interfaces.msg", msg_pkg)
    sys.modules.setdefault("mtc_interfaces.srv", srv_pkg)


class _GoalHandle:
    def __init__(self, goal: Any) -> None:
        self.request = goal
        self.is_cancel_requested = False
        self.feedback: List[Any] = []
        self.outcome: Optional[str] = None

    def publish_feedback(self, feedback: Any) -> None:
        self.feedback.append(feedback)

    def succeed(self) -> None:
        self.outcome = "succeeded"

    def abort(self) -> None:
        self.outcome = "aborted"

    def canceled(self) -> None:
        self.outcome = "canceled"


class _ActionTypeStub:
    """PickPlace 动作类型替身：只提供字段容器与 Server 注册点。"""

    class Goal:
        def __init__(self) -> None:
            self.request_id = ""
            self.object_id = ""
            self.expected_color = 0
            self.target_slot = "T0"
            self.placement_stability_sec = 3.0

    class Result:
        def __init__(self) -> None:
            self.success = False
            self.placement_verified = False
            self.error_code = 0
            self.message = ""

    class Feedback:
        def __init__(self) -> None:
            self.substate = ""
            self.progress = 0.0
            self.object_attached_estimated = False
            self.detail = ""

    class CancelResponse:
        pass

    class Server:
        def __init__(self, node: Any, name: str, execute_callback: Any, cancel_callback: Any) -> None:
            self.node = node
            self.name = name
            self.execute_callback = execute_callback
            self.cancel_callback = cancel_callback
            self.destroyed = False

        def destroy(self) -> None:
            self.destroyed = True


class _ToolStateStub:
    def __init__(self) -> None:
        self.header = types.SimpleNamespace(stamp=None)
        self.state = 0
        self.evidence_level = 0
        self.verified = False
        self.tool_type = ""
        self.object_id = ""
        self.detail = ""


def _make_server(tool_type: str = "passive_hook_v1", **kwargs: Any) -> Any:
    _install_stub_modules()
    from mtc_manipulation import pick_place_server as server_mod

    strategy = (
        MagneticLatchV2Strategy() if tool_type == TOOL_MAGNETIC_LATCH_V2 else PassiveHookV1Strategy()
    )
    node = _FakeNode()
    server = server_mod.PickPlaceServer(
        node=node,
        action_type=_ActionTypeStub,
        perception=kwargs.get("perception", FakePerception()),
        planner=FakePlanner(),
        motion=kwargs.get("motion", FakeMotion()),
        tool=kwargs.get("tool", FakeTool(tool_type=tool_type)),
        # 时钟必须自动前进：server 循环没有外部时钟推进者，
        # 否则 FSM 看门狗永远不触发（这正是 FakeClock.auto_advance 的用途）。
        clock=kwargs.get("clock", FakeClock(auto_advance=0.05)),
        strategy=strategy,
        config=PickPlaceConfig(tool_type=tool_type),
        tool_state_type=_ToolStateStub,
        max_execution_steps=kwargs.get("max_execution_steps", 20000),
    )
    return server, node


# ---------------------------------------------------------------------------
# 测试
# ---------------------------------------------------------------------------


def test_server_module_imports_and_constructs() -> None:
    server, node = _make_server()
    assert server._server.name == "/mtc/manipulation/pick_place"
    # 工具状态发布器已创建，且话题名与设计一致
    assert node.published == []  # 构造阶段不应发布任何状态
    server.stop()
    assert server._server.destroyed is True


def test_server_executes_full_pick_place_cycle() -> None:
    server, node = _make_server()
    goal = _ActionTypeStub.Goal()
    goal.request_id = "req-1"
    goal.object_id = "battery_1"
    goal.expected_color = 1
    goal.target_slot = "P1"
    handle = _GoalHandle(goal)

    result = server._server.execute_callback(handle)

    assert result.success is True
    assert result.placement_verified is True
    assert result.error_code == 0
    assert handle.outcome == "succeeded"
    # 反馈包含状态机子状态名
    assert handle.feedback
    assert handle.feedback[-1].substate == PickPlaceState.SUCCESS.value
    assert handle.feedback[-1].progress == 1.0
    # 工具状态已发布
    assert node.published
    topic, message = node.published[-1]
    assert topic == "/mtc/tool/state"
    assert message.tool_type == "passive_hook_v1"
    assert "SUCCESS" in message.detail


def test_server_cancel_path_never_claims_success() -> None:
    server, node = _make_server()
    goal = _ActionTypeStub.Goal()
    goal.object_id = "battery_1"
    goal.expected_color = 1
    goal.target_slot = "P1"
    handle = _GoalHandle(goal)
    handle.is_cancel_requested = True  # 一进入就请求取消

    result = server._server.execute_callback(handle)

    assert result.success is False
    assert result.placement_verified is False
    assert handle.outcome == "canceled"


def test_server_invalid_goal_is_aborted() -> None:
    server, _ = _make_server()
    goal = _ActionTypeStub.Goal()
    goal.object_id = "battery_1"
    goal.target_slot = "ZZZ"
    handle = _GoalHandle(goal)

    result = server._server.execute_callback(handle)

    assert result.success is False
    assert result.error_code == 100  # INVALID_TASK
    assert handle.outcome == "aborted"


def test_ros_adapters_are_instantiable_with_stubs() -> None:
    _install_stub_modules()
    from mtc_manipulation import pick_place_server as server_mod
    from mtc_manipulation.pick_place_fsm import MotionCommand

    node = _FakeNode()
    perception = server_mod.RosPerceptionPort(node, type("M", (), {}))
    assert node.subscriptions and node.subscriptions[0][0] == "/mtc/perception/batteries"
    detection = perception.get_detection("battery_1", 1, 0.3)
    assert detection.found is False  # 无缓存帧时保守返回未找到

    planner = server_mod.RosPlannerPort(node)
    assert planner.plan("ACQUIRE", {}).ok is True

    class _ActionTypeStubForMotion:
        @staticmethod
        def create_client(node: Any, name: str) -> Any:
            from rclpy.action import ActionClient

            return ActionClient(node, _ActionTypeStubForMotion, name)

    motion = server_mod.RosMotionPort(node, _ActionTypeStubForMotion)
    # 替身 ActionClient.wait_for_server 返回 False -> 必须判定为“状态未知”而不是成功
    result = motion.execute_joint_move(
        MotionCommand(command_id="c1", joint_names=("J1",), target_rad=(0.0,))
    )
    assert result.ok is False
    assert result.status_unknown is True

    tool = server_mod.RosToolPort(node, tool_type="magnetic_latch_v2")
    evidence = tool.engage("ENGAGE", ())
    assert evidence.ok is False  # 未接真实 tool_manager：保守失败
    assert tool.holding_state() == "unknown"


# ---------------------------------------------------------------------------
# 标准库兼容层：本文件用 pytest 风格的模块级测试函数编写；
# 下面的 test_suite() 让 `python3 -m unittest discover -s test` 也能收集并运行全部用例，
# 从而在“没有 pytest”的机器上依然可离线验收。
# ---------------------------------------------------------------------------


def load_tests(loader: Any, tests: Any, pattern: Any) -> Any:
    """unittest 协议钩子：让 `python3 -m unittest discover` 收集本文件的全部用例。"""
    return _module_test_suite()


def _module_test_suite() -> Any:
    import unittest

    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    for name, function in sorted(globals().items()):
        if name.startswith("test_") and callable(function):
            suite.addTest(loader.loadTestsFromTestCase(_make_case(name, function)))
    return suite


def _make_case(name: str, function: Any) -> Any:
    import unittest

    def _run(self: Any) -> None:
        function()

    return type("UT_" + name, (unittest.TestCase,), {name: _run})
