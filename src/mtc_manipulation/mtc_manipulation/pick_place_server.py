"""PickPlace Action Server 外壳（rclpy）。

边界与设计：
    - 本文件是**唯一**允许导入 rclpy 的模块；FSM 核心（pick_place_fsm.py）保持纯 Python。
    - 所有外部能力通过注入端口实现，因此本文件可以在离线测试中用 Fake 端口 + 替身
      Action 类型（见 test/test_server_shell.py）构造与驱动，无需 ROS 运行时。
    - 真实 ROS 适配器（RosMotionPort / RosPerceptionPort / RosPlannerPort / RosToolPort）
      只做协议翻译，不含任何安全判决逻辑；安全判决全部在 FSM 内。
    - 尚未编译验证：mtc_interfaces 的 Action/Message 需在 Jazzy 环境 colcon build 后生成，
      当前机器只有 Humble，且本任务禁止编译。见 README.md 的“未实现/未验证”章节。

话题/动作：
    Action  /mtc/manipulation/pick_place   (mtc_interfaces/action/PickPlace)
    Topic   /mtc/tool/state                (mtc_interfaces/msg/ToolState, RELIABLE/KEEP_LAST(10))
    Topic   /mtc/perception/batteries      (mtc_interfaces/msg/BatteryDetectionArray, 只读订阅)
"""

from __future__ import annotations

import logging
import time
from typing import Any, Callable, Dict, Optional, Sequence, Tuple

from .pick_place_fsm import (
    ClockPort,
    DetectionResult,
    ErrorCode,
    EvidenceLevel,
    MotionCommand,
    MotionPort,
    MotionResult,
    PerceptionPort,
    PickPlaceConfig,
    PickPlaceFSM,
    PickPlaceState,
    PlannerPort,
    PlanResult,
    PoseEstimate,
    StopResult,
    ToolEvidence,
    ToolPort,
    ToolStateCode,
)
from .tool_strategy import ToolStrategy

LOGGER = logging.getLogger("mtc_manipulation.pick_place_server")

ACTION_NAME = "/mtc/manipulation/pick_place"
TOOL_STATE_TOPIC = "/mtc/tool/state"
PERCEPTION_TOPIC = "/mtc/perception/batteries"
UNLOCK_SERVICE = "/mtc/tool/trigger_unlock"


# ---------------------------------------------------------------------------
# 真实 ROS 适配器
# ---------------------------------------------------------------------------


class RosClock(ClockPort):
    """基于节点时钟（默认使用系统时间，便于 FakeClock 之外的真机运行）。"""

    def __init__(self, node: Any) -> None:
        self._node = node

    def now(self) -> float:
        return time.monotonic()


class RosPerceptionPort(PerceptionPort):
    """订阅电池识别话题并缓存最新一帧，供 FSM 同步读取。

    注意：这是“最后一次观测快照”，新鲜度（stale）由 FSM 用 ClockPort 判定，
    适配器本身不做安全判决。
    """

    def __init__(self, node: Any, message_type: Any, topic: str = PERCEPTION_TOPIC, qos_depth: int = 10) -> None:
        self._node = node
        self._message_type = message_type
        self._latest: Dict[str, Any] = {}
        self._lock = None
        try:  # threading 仅为保护缓存，缺失也不影响功能
            import threading

            self._lock = threading.Lock()
        except Exception:  # pragma: no cover
            self._lock = None
        from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

        qos = QoSProfile(
            depth=qos_depth,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        node.create_subscription(message_type, topic, self._on_message, qos)

    def _on_message(self, msg: Any) -> None:
        if self._lock is not None:
            with self._lock:
                self._latest = {}
                for det in getattr(msg, "detections", []) or []:
                    self._latest[str(getattr(det, "object_id", ""))] = det
        else:  # pragma: no cover
            self._latest = {
                str(getattr(det, "object_id", "")): det
                for det in getattr(msg, "detections", []) or []
            }

    def get_detection(
        self, object_id: str, expected_color: Optional[int], max_age_s: float
    ) -> DetectionResult:
        det = self._latest.get(object_id)
        if det is None:
            return DetectionResult(found=False, message="感知缓存中没有 object_id=%s" % object_id)
        ring_pose_valid = bool(getattr(det, "ring_pose_valid", False))
        color = int(getattr(det, "color", 0))
        if expected_color is not None and color != int(expected_color):
            return DetectionResult(
                found=True,
                unique=True,
                pose=self._to_pose(object_id, det),
                message="颜色不符：期望 %s，实际 %s" % (expected_color, color),
            )
        stamp = getattr(getattr(det, "body_pose", None), "header", None)
        stamp_s = 0.0
        if stamp is not None and getattr(stamp, "stamp", None) is not None:
            stamp_s = float(stamp.stamp.sec) + float(stamp.stamp.nanosec) * 1e-9
        now_s = self._node.get_clock().now().nanoseconds * 1e-9 if self._node is not None else stamp_s
        stale = bool(max_age_s > 0 and stamp_s > 0 and (now_s - stamp_s) > max_age_s)
        return DetectionResult(
            found=True,
            unique=True,
            stale=stale,
            pose=self._to_pose(object_id, det, stamp_s=stamp_s, ring_pose_valid=ring_pose_valid),
            message="ROS 感知帧" + ("（过期）" if stale else ""),
        )

    @staticmethod
    def _to_pose(object_id: str, det: Any, stamp_s: float = 0.0, ring_pose_valid: bool = True) -> PoseEstimate:
        def _xyz(msg: Any) -> Optional[Tuple[float, float, float]]:
            if msg is None:
                return None
            pose_stamped = getattr(msg, "pose", None)
            position = getattr(pose_stamped, "position", pose_stamped)
            if position is None:
                return None
            return (float(position.x), float(position.y), float(position.z))

        return PoseEstimate(
            object_id=object_id,
            color=int(getattr(det, "color", 0)),
            ring_pose=_xyz(getattr(det, "ring_pose", None)),
            body_pose=_xyz(getattr(det, "body_pose", None)),
            confidence=float(getattr(det, "confidence", 0.0)),
            stamp_s=stamp_s,
            ring_pose_valid=ring_pose_valid,
        )


class RosPlannerPort(PlannerPort):
    """规划适配器。

    当前工作空间的 mtc_interfaces 只提供 PickPlace / ExecuteTask / ExecuteJointMove，
    **不存在 PlanMotion.action**（见 README 的偏差说明）。因此本适配器只做输入校验与透传：
        - 规划失败 => PLANNING_FAILED=200
        - 规划成功 => 返回空原语列表（原语由 ToolStrategy 提供）
    真实运动规划接入前不得声称具备碰撞检查能力。
    """

    def __init__(self, node: Any, planner_client: Any = None) -> None:
        self._node = node
        self._client = planner_client

    def plan(self, phase: str, target: Any) -> PlanResult:
        if self._client is None:
            self._node.get_logger().debug(
                "规划阶段 %s 使用透传实现（未接入 MoveIt/PlanMotion）" % phase
            )
            return PlanResult(ok=True, error_code=ErrorCode.OK, message="passthrough planner")
        try:
            return self._client.plan(phase, target)
        except Exception as exc:  # 规划异常必须显式失败，不得默认放行
            return PlanResult(
                ok=False,
                error_code=ErrorCode.PLANNING_FAILED,
                message="规划调用异常: %s" % exc,
            )


class RosMotionPort(MotionPort):
    """ExecuteJointMove Action 客户端适配器。

    - execute_joint_move 是**同步阻塞**实现（内部 spin 等待结果），符合 FSM 单步语义；
    - 结果不确定（未收到 result / 取消后未确认）一律返回 status_unknown，
      由 FSM 判定为 MOTION_STATUS_UNKNOWN=230 并禁止自动重发；
    - 不把“发令成功”当作“运动到位”。
    """

    def __init__(
        self,
        node: Any,
        action_type: Any,
        action_name: str = "/mtc/motion/execute_joint_move",
        server_wait_s: float = 2.0,
        result_timeout_s: float = 60.0,
    ) -> None:
        self._node = node
        self._action_type = action_type
        self._action_name = action_name
        self._server_wait_s = server_wait_s
        self._result_timeout_s = result_timeout_s
        self._client = action_type.create_client(node, action_name)
        self._inflight_goal: Any = None

    # -- MotionPort -----------------------------------------------------
    def execute_joint_move(self, command: MotionCommand) -> Optional[MotionResult]:
        if not self._client.wait_for_server(timeout_sec=self._server_wait_s):
            return MotionResult(
                success=False,
                arrived=False,
                status_unknown=True,
                error_code=ErrorCode.MOTION_STATUS_UNKNOWN,
                message="运动执行 Action 服务不可用：%s" % self._action_name,
            )

        goal = self._action_type.Goal()
        goal.command_id = command.command_id
        goal.joint_names = list(command.joint_names)
        goal.target_rad = [float(v) for v in command.target_rad]
        goal.velocity_scaling = float(command.velocity_scaling)
        goal.acceleration_scaling = float(command.acceleration_scaling)
        goal.timeout_ms = int(command.timeout_ms)

        send_future = self._client.send_goal_async(goal)
        self._node.get_logger().info("已发令 %s（%s）" % (command.command_id, command.primitive_name))
        if not self._spin_until(send_future, self._server_wait_s):
            return self._unknown("运动目标未被受理（结果不确定）")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return MotionResult(
                success=False,
                arrived=False,
                stop_confirmed=True,
                error_code=ErrorCode.EXECUTION_REJECTED,
                message="运动目标被拒绝",
            )
        self._inflight_goal = goal_handle

        result_future = goal_handle.get_result_async()
        if not self._spin_until(result_future, self._result_timeout_s):
            return self._unknown("等待 %s 的运动结果超时" % command.command_id)
        wrapped = result_future.result()
        result = wrapped.result if wrapped is not None else None
        if result is None:
            return self._unknown("运动结果为空（结果不确定）")
        self._inflight_goal = None
        success = bool(getattr(result, "success", False))
        return MotionResult(
            success=success,
            arrived=success,
            stop_confirmed=bool(getattr(result, "stop_confirmed", False)),
            final_position_rad=tuple(float(v) for v in getattr(result, "final_position_rad", ()) or ()),
            error_code=int(getattr(result, "error_code", ErrorCode.OK)),
            message=str(getattr(result, "message", "")),
        )

    def request_stop(self, reason: str) -> Optional[StopResult]:
        if self._inflight_goal is None:
            return StopResult(
                request_delivered=True,
                stop_confirmed=True,
                message="当前无在途运动目标；不以此作为安全保证",
            )
        cancel_future = self._inflight_goal.cancel_goal_async()
        if not self._spin_until(cancel_future, self._server_wait_s):
            return StopResult(
                request_delivered=True,
                stop_confirmed=False,
                message="取消请求已发出但未获响应（%s）" % reason,
            )
        result_future = self._inflight_goal.get_result_async()
        if not self._spin_until(result_future, self._server_wait_s):
            return StopResult(
                request_delivered=True,
                stop_confirmed=False,
                message="取消后未在 %.1fs 内确认停稳" % self._server_wait_s,
            )
        wrapped = result_future.result()
        result = wrapped.result if wrapped is not None else None
        stop_confirmed = bool(getattr(result, "stop_confirmed", False))
        return StopResult(
            request_delivered=True,
            stop_confirmed=stop_confirmed,
            message="stop_confirmed=%s" % stop_confirmed,
        )

    # -- 内部 -----------------------------------------------------------
    def _spin_until(self, future: Any, timeout_s: float) -> bool:
        deadline = time.monotonic() + float(timeout_s)
        while not future.done():
            if time.monotonic() > deadline:
                return False
            rclpy_spin_once(self._node, 0.05)
        return future.done()

    @staticmethod
    def _unknown(message: str) -> MotionResult:
        return MotionResult(
            success=False,
            arrived=False,
            status_unknown=True,
            error_code=ErrorCode.MOTION_STATUS_UNKNOWN,
            message=message,
        )


class RosToolPort(ToolPort):
    """工具端口 ROS 适配器骨架。

    现状（如实声明，不臆造实现）：
      - mtc_interfaces 没有 TriggerUnlock 之外的 V1/V2 工具接口；
      - TriggerUnlock.srv 的注释明确“仅允许 tool_manager 在 VERIFY_SEATED + UNLOAD 之后调用”，
        因此本适配器只做“解锁脉冲请求”的翻译，其余机构证据必须由真实 tool_manager 提供。
      - 在未接入真实 tool_manager 前，除解锁外的机构证据一律返回未验证，
        使 FSM 保守地停在 FAULT / FAILED，而不是伪装成功。
    """

    def __init__(
        self,
        node: Any,
        tool_type: str,
        unlock_srv_type: Any = None,
        unlock_service: str = UNLOCK_SERVICE,
        unlock_pulse_ms: int = 200,
        evidence_source: Any = None,
    ) -> None:
        self._node = node
        self._tool_type = tool_type
        self._unlock_pulse_ms = int(unlock_pulse_ms)
        self._evidence_source = evidence_source
        self._unlock_client = None
        if unlock_srv_type is not None:
            self._unlock_client = node.create_client(unlock_srv_type, unlock_service)

    # -- 未接通的机构证据：一律保守返回未验证 ----------------------------
    def _not_wired(self, what: str) -> ToolEvidence:
        self._node.get_logger().error(
            "工具端口未接通真实 tool_manager：%s 无法提供机构证据" % what
        )
        return ToolEvidence(
            ok=False,
            state=ToolStateCode.STATE_UNKNOWN,
            evidence_level=EvidenceLevel.NONE,
            verified=False,
            error_code=ErrorCode.HARDWARE_FAULT,
            message="%s：未接入真实工具管理器（Mock/未编译验证）" % what,
        )

    def engage(self, phase: str, primitives: Sequence[Any]) -> Optional[ToolEvidence]:
        source = self._evidence_source
        if source is not None:
            return source.engage(phase, primitives)
        return self._not_wired("engage(%s)" % phase)

    def test_lift(self, primitives: Sequence[Any]) -> Optional[ToolEvidence]:
        source = self._evidence_source
        if source is not None:
            return source.test_lift(primitives)
        return self._not_wired("test_lift")

    def seat_verify(self) -> ToolEvidence:
        source = self._evidence_source
        if source is not None:
            return source.seat_verify()
        return self._not_wired("seat_verify")

    def unload(self, primitives: Sequence[Any]) -> Optional[ToolEvidence]:
        source = self._evidence_source
        if source is not None:
            return source.unload(primitives)
        return self._not_wired("unload")

    def unlock(self, pulse_ms: int = 200) -> Optional[ToolEvidence]:
        if self._unlock_client is None:
            return self._not_wired("unlock")
        request = self._unlock_client.srv_type.Request()
        request.request_id = getattr(self, "_current_request_id", "")
        request.pulse_ms = int(pulse_ms or self._unlock_pulse_ms)
        if not self._unlock_client.wait_for_service(timeout_sec=1.0):
            return ToolEvidence(
                ok=False,
                state=ToolStateCode.STATE_ATTACHED,
                verified=False,
                locked=True,
                error_code=ErrorCode.UNLOCK_FAILED,
                message="解锁服务不可用",
            )
        future = self._unlock_client.call_async(request)
        deadline = time.monotonic() + 2.0
        while not future.done():
            if time.monotonic() > deadline:
                return ToolEvidence(
                    ok=False,
                    state=ToolStateCode.STATE_ATTACHED,
                    verified=False,
                    locked=None,
                    error_code=ErrorCode.UNLOCK_FAILED,
                    message="解锁服务调用超时，锁止状态未知",
                )
            rclpy_spin_once(self._node, 0.05)
        response = future.result()
        accepted = bool(getattr(response, "accepted", False))
        # accepted=true 只表示脉冲请求被受理，不代表电池已释放
        return ToolEvidence(
            ok=False,
            state=ToolStateCode.STATE_ATTACHED,
            verified=False,
            locked=None,
            released=None if accepted else False,
            error_code=ErrorCode.UNLOCK_FAILED,
            message=(
                "解锁脉冲已受理但缺少锁止解除反馈，按设计禁止撤离（%s）"
                % getattr(response, "message", "")
            ),
        )

    def disengage(self, primitives: Sequence[Any]) -> Optional[ToolEvidence]:
        source = self._evidence_source
        if source is not None:
            return source.disengage(primitives)
        return self._not_wired("disengage")

    def holding_state(self) -> str:
        source = self._evidence_source
        if source is not None:
            return source.holding_state()
        return "unknown"


def rclpy_spin_once(node: Any, timeout_s: float = 0.0) -> None:
    """统一的 spin_once 包装（便于单元测试替身拦截）。"""
    import rclpy

    rclpy.spin_once(node, timeout_sec=timeout_s)


# ---------------------------------------------------------------------------
# Action Server
# ---------------------------------------------------------------------------


class PickPlaceServer:
    """PickPlace Action Server 外壳。

    所有端口与 Action 类型均可注入，因此可离线构造（见 test/test_server_shell.py）。
    """

    #: 单次 Goal 的最大推进步数（防止端口长时间返回“未完成”而卡死 Action 线程）
    MAX_EXECUTION_STEPS = 20000

    def __init__(
        self,
        node: Any,
        action_type: Any,
        perception: PerceptionPort,
        planner: PlannerPort,
        motion: MotionPort,
        tool: ToolPort,
        clock: ClockPort,
        strategy: ToolStrategy,
        config: Optional[PickPlaceConfig] = None,
        tool_state_type: Any = None,
        action_name: str = ACTION_NAME,
        tool_state_topic: str = TOOL_STATE_TOPIC,
        max_execution_steps: int = MAX_EXECUTION_STEPS,
    ) -> None:
        self.node = node
        self.action_type = action_type
        self.perception = perception
        self.planner = planner
        self.motion = motion
        self.tool = tool
        self.clock = clock
        self.strategy = strategy
        self.config = config or PickPlaceConfig()
        self.max_execution_steps = int(max_execution_steps)
        self._fsm: Optional[PickPlaceFSM] = None
        self.tool_state_type = tool_state_type
        self._publisher = None
        if tool_state_type is not None:
            self._publisher = node.create_publisher(tool_state_type, tool_state_topic, 10)
        self._server = action_type.Server(node, action_name, self._execute_callback, self._cancel_callback)

    # -- Action 回调 -----------------------------------------------------
    def _execute_callback(self, goal_handle: Any) -> Any:
        goal = goal_handle.request
        fsm = PickPlaceFSM(
            self.perception, self.planner, self.motion, self.tool, self.clock, self.strategy, self.config
        )
        self._fsm = fsm
        fsm.start(
            object_id=str(getattr(goal, "object_id", "")),
            expected_color=int(getattr(goal, "expected_color", 0)),
            target_slot=str(getattr(goal, "target_slot", "")),
            request_id=str(getattr(goal, "request_id", "")),
            placement_stability_sec=float(
                getattr(goal, "placement_stability_sec", self.config.placement_stability_sec)
            ),
        )

        # 步数硬上限：防止端口长时间返回“未完成”而把 Action 线程卡死。
        # 达到上限即按运动状态未知处理（MOTION_STATUS_UNKNOWN=230），锁定动作、禁止重发。
        max_steps = self.max_execution_steps
        steps = 0
        while fsm.state not in (
            PickPlaceState.SUCCESS,
            PickPlaceState.FAILED,
            PickPlaceState.FAULT,
            PickPlaceState.CANCELLED,
        ):
            if getattr(goal_handle, "is_cancel_requested", False):
                fsm.request_cancel("Action Cancel")
            fsm.step()
            self._publish_feedback(goal_handle, fsm)
            steps += 1
            if steps > max_steps:
                self.node.get_logger().error(
                    "PickPlace 超过 %d 步仍未结束，判定运动状态未知并锁定动作" % max_steps
                )
                fsm.force_fault(
                    ErrorCode.MOTION_STATUS_UNKNOWN,
                    "超过 %d 步仍未结束：运动状态未知，禁止自动重发" % max_steps,
                )
                break

        result_state = fsm.result()
        self._publish_tool_state(fsm, result_state)

        result = self.action_type.Result()
        result.success = bool(result_state.success)
        result.placement_verified = bool(result_state.placement_verified)
        result.error_code = int(result_state.error_code)
        result.message = str(result_state.message or result_state.final_state.value)

        if result_state.final_state is PickPlaceState.SUCCESS:
            goal_handle.succeed()
        elif result_state.final_state is PickPlaceState.CANCELLED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _cancel_callback(self, goal_handle: Any) -> Any:
        # 只登记取消意图；真正的安全停止由 FSM 在 CANCEL_PENDING 中向后端确认
        if self._fsm is not None:
            self._fsm.request_cancel("Action Cancel")
        return self.action_type.CancelResponse()

    # -- 反馈与状态发布 ---------------------------------------------------
    def _publish_feedback(self, goal_handle: Any, fsm: PickPlaceFSM) -> None:
        feedback = self.action_type.Feedback()
        snapshot = fsm.feedback()
        feedback.substate = snapshot.substate
        feedback.progress = float(snapshot.progress)
        feedback.object_attached_estimated = bool(snapshot.object_attached_estimated)
        feedback.detail = snapshot.detail
        try:
            goal_handle.publish_feedback(feedback)
        except Exception:  # pragma: no cover - 替身/无订阅者
            LOGGER.debug("publish_feedback 失败", exc_info=True)

    def _publish_tool_state(self, fsm: PickPlaceFSM, result_state: Any) -> None:
        if self._publisher is None:
            return
        if self.tool_state_type is None:
            return
        message = self.tool_state_type()
        header = getattr(message, "header", None)
        if header is not None and hasattr(header, "stamp"):
            header.stamp = self.node.get_clock().now().to_msg()
        message.state = self._tool_state_code(fsm)
        message.evidence_level = int(self._last_evidence_level(fsm))
        message.verified = bool(
            getattr(getattr(fsm, "_disengage_evidence", None), "released", False)
        )
        message.tool_type = self.strategy.type()
        message.object_id = fsm.object_id
        message.detail = "final_state=%s error_code=%s" % (
            result_state.final_state.value,
            result_state.error_code,
        )
        self._publisher.publish(message)

    def stop(self) -> None:
        """销毁 Action Server（幂等）。"""
        destroy = getattr(self._server, "destroy", None)
        if callable(destroy):
            destroy()

    @staticmethod
    def _last_evidence_level(fsm: PickPlaceFSM) -> int:
        for attr in ("_disengage_evidence", "_unload_evidence", "_seat_evidence", "_test_lift_evidence"):
            evidence = getattr(fsm, attr, None)
            if evidence is not None:
                return int(evidence.evidence_level)
        return EvidenceLevel.NONE

    @staticmethod
    def _tool_state_code(fsm: PickPlaceFSM) -> int:
        evidence = getattr(fsm, "_disengage_evidence", None)
        if evidence is not None and evidence.released:
            return ToolStateCode.STATE_DETACHED
        evidence = getattr(fsm, "_unload_evidence", None)
        if evidence is not None:
            return ToolStateCode.STATE_RELEASING
        evidence = getattr(fsm, "_engage_evidence", None)
        if evidence is not None and evidence.ok:
            return ToolStateCode.STATE_ATTACHED
        if fsm.state is PickPlaceState.FAULT:
            return ToolStateCode.STATE_FAULT
        return ToolStateCode.STATE_UNKNOWN


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------


def main(argv: Optional[Sequence[str]] = None) -> int:
    """可执行入口（ROS 2 ament_python console_scripts）。"""
    import rclpy

    from mtc_interfaces.action import PickPlace
    from mtc_interfaces.msg import BatteryDetectionArray, ToolState
    from mtc_interfaces.srv import TriggerUnlock

    from .tool_strategy import make_strategy

    rclpy.init(args=list(argv) if argv is not None else None)
    node = rclpy.create_node("pick_place_server")
    node.declare_parameter("tool_type", "passive_hook_v1")
    node.declare_parameter("placement_stability_sec", 3.0)
    node.declare_parameter("perception_max_age_sec", 0.3)
    node.declare_parameter("recovery_max_attempts", 1)
    node.declare_parameter("verify_test_lift", True)
    node.declare_parameter("verify_placement", True)
    node.declare_parameter("motion_action_name", "/mtc/motion/execute_joint_move")

    tool_type = str(node.get_parameter("tool_type").value)
    strategy = make_strategy(tool_type)
    config = PickPlaceConfig(
        tool_type=tool_type,
        placement_stability_sec=float(node.get_parameter("placement_stability_sec").value),
        perception_max_age_s=float(node.get_parameter("perception_max_age_sec").value),
        recovery_max_attempts=int(node.get_parameter("recovery_max_attempts").value),
        verify_test_lift=bool(node.get_parameter("verify_test_lift").value),
        verify_placement=bool(node.get_parameter("verify_placement").value),
    )

    try:
        from mtc_interfaces.action import ExecuteJointMove
    except ImportError as exc:  # pragma: no cover
        node.get_logger().fatal("缺少 mtc_interfaces/ExecuteJointMove：请先 colcon build（本机禁止）: %s" % exc)
        rclpy.shutdown()
        return 2

    server = PickPlaceServer(
        node=node,
        action_type=PickPlace,
        perception=RosPerceptionPort(node, BatteryDetectionArray),
        planner=RosPlannerPort(node),
        motion=RosMotionPort(
            node, ExecuteJointMove, action_name=str(node.get_parameter("motion_action_name").value)
        ),
        tool=RosToolPort(node, tool_type=tool_type, unlock_srv_type=TriggerUnlock),
        clock=RosClock(node),
        strategy=strategy,
        config=config,
        tool_state_type=ToolState,
    )
    node.get_logger().info(
        "pick_place_server 就绪：%s（tool_type=%s，Mock/未编译验证）" % (ACTION_NAME, tool_type)
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        node.get_logger().info("收到中断，停止服务")
    finally:
        server.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


# ---------------------------------------------------------------------------
# 测试/集成用的可选辅助
# ---------------------------------------------------------------------------


class CallbackActionType:
    """极简 Action 类型替身：仅供离线测试构造 PickPlaceServer。

    不实现任何 DDS 行为，只提供 Goal/Result/Feedback/CancelResponse/Server 的构造能力。
    """

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
        def __init__(
            self,
            node: Any,
            name: str,
            execute_callback: Callable[..., Any],
            cancel_callback: Callable[..., Any],
        ) -> None:
            self.node = node
            self.name = name
            self.execute_callback = execute_callback
            self.cancel_callback = cancel_callback
