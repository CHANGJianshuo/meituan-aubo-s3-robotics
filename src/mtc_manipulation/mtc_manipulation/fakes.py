"""离线 Fake 端口集合：供单元测试与上层 Mock 集成使用。

设计原则：
    - Fake 必须能复现“危险/不确定”场景，而不只是“happy path”；
      否则离线测试无法证明安全语义被实现。
    - 所有 Fake 都是纯 Python，不导入 rclpy；可被 pytest / unittest 直接使用。
    - FakeMotion 支持：正常到位、超时、运动状态未知、取消后确认停稳、取消后不确认。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from .pick_place_fsm import (
    DetectionResult,
    ErrorCode,
    EvidenceLevel,
    MotionCommand,
    MotionPrimitive,
    MotionResult,
    PerceptionPort,
    PlannerPort,
    PlanResult,
    PoseEstimate,
    ClockPort,
    MotionPort,
    StopResult,
    ToolEvidence,
    ToolPort,
    ToolStateCode,
)
from .tool_strategy import PassiveHookV1Strategy, ToolStrategy


# ---------------------------------------------------------------------------
# 时钟
# ---------------------------------------------------------------------------


class FakeClock(ClockPort):
    """可手动推进的假时钟；用于加速超时与 3 秒稳定性判定。"""

    def __init__(self, start: float = 0.0, auto_advance: float = 0.0) -> None:
        self._now = float(start)
        self.auto_advance = float(auto_advance)

    def now(self) -> float:
        value = self._now
        self._now += self.auto_advance
        return value

    def advance(self, seconds: float) -> float:
        self._now += float(seconds)
        return self._now

    def set(self, seconds: float) -> None:
        self._now = float(seconds)


# ---------------------------------------------------------------------------
# 感知
# ---------------------------------------------------------------------------


@dataclass
class FakePerception(PerceptionPort):
    """假感知。

    found / stale / unique 由 scenario 直接设定；detection 可整体替换。
    placement_at_target / placement_settled 控制 VERIFY_PLACED 的观测。
    """

    detection: Optional[DetectionResult] = None
    found: bool = True
    stale: bool = False
    unique: bool = True
    color: int = 1
    ring_pose: Tuple[float, float, float] = (0.3, 0.0, 0.15)
    body_pose: Tuple[float, float, float] = (0.3, 0.0, 0.1)
    confidence: float = 0.95
    ring_pose_valid: bool = True
    followed: bool = True
    placement_at_target: bool = True
    placement_settled: bool = True
    placement_found: bool = True
    placement_stale: bool = False
    placement_pose: Optional[Tuple[float, float, float]] = None
    calls: List[Tuple[str, Optional[int], float]] = field(default_factory=list)

    def get_detection(
        self, object_id: str, expected_color: Optional[int], max_age_s: float
    ) -> DetectionResult:
        self.calls.append((object_id, expected_color, max_age_s))
        if self.detection is not None:
            return self.detection

        if expected_color is None:
            # 放置后核验：不做颜色校验，只看是否在目标区域且稳定
            pose = PoseEstimate(
                object_id=object_id,
                color=self.color,
                ring_pose=self.placement_pose or (0.5, 0.2, 0.05),
                body_pose=self.placement_pose or (0.5, 0.2, 0.05),
                confidence=self.confidence,
                stamp_s=0.0,
                at_target=self.placement_at_target,
                settled=self.placement_settled,
                followed=False,
            )
            return DetectionResult(
                found=self.placement_found,
                unique=True,
                stale=self.placement_stale,
                pose=pose,
                message="placement check",
            )

        pose = PoseEstimate(
            object_id=object_id,
            color=self.color,
            ring_pose=self.ring_pose,
            body_pose=self.body_pose,
            confidence=self.confidence,
            stamp_s=0.0,
            ring_pose_valid=self.ring_pose_valid,
            followed=self.followed,
        )
        # 防抓错颜色的双重校验（设计 V0.1 §4.4）：期望颜色与实际颜色不符时
        # 视同“未找到该目标”，避免上层把它当作可抓对象。
        color_mismatch = bool(
            expected_color and int(self.color or 0) != int(expected_color))
        if color_mismatch:
            return DetectionResult(
                found=False,
                unique=True,
                stale=False,
                pose=None,
                message=("目标颜色不匹配：期望 %s，实际 %s；拒绝抓错颜色"
                         % (expected_color, self.color)),
            )
        return DetectionResult(
            found=self.found,
            unique=self.unique,
            stale=self.stale,
            pose=pose,
            message="fake detection",
        )


# ---------------------------------------------------------------------------
# 规划
# ---------------------------------------------------------------------------


@dataclass
class FakePlanner(PlannerPort):
    """假规划器；ok=False 时返回 PLANNING_FAILED=200。"""

    ok: bool = True
    fail_phases: Tuple[str, ...] = ()
    error_code: int = ErrorCode.PLANNING_FAILED
    message: str = "fake planner"
    calls: List[Tuple[str, Any]] = field(default_factory=list)

    def plan(self, phase: str, target: Any) -> PlanResult:
        self.calls.append((phase, target))
        if not self.ok or phase in self.fail_phases:
            return PlanResult(
                ok=False, error_code=self.error_code, message=self.message or "规划失败(%s)" % phase
            )
        return PlanResult(ok=True, error_code=ErrorCode.OK, message="ok")


# ---------------------------------------------------------------------------
# 运动执行
# ---------------------------------------------------------------------------


@dataclass
class FakeMotion(MotionPort):
    """假运动执行层。

    可模拟：
        - 正常到位（arrived=True）
        - 运动超时（timeout_primitives 或 timeout_after_n_commands）
        - 运动状态未知（unknown_primitives / unknown_after_n_commands）
        - 执行被拒绝（rejected_primitives）
        - 取消后确认停稳（stop_confirm_after_ticks 内确认）
        - 取消后不确认（stop_confirm_after_ticks=None 且 request_stop 后始终未确认）
    """

    delay_ticks: int = 0
    stop_confirm_after_ticks: Optional[int] = 0
    timeout_primitives: Tuple[str, ...] = ()
    unknown_primitives: Tuple[str, ...] = ()
    rejected_primitives: Tuple[str, ...] = ()
    timeout_after_n_commands: Optional[int] = None
    unknown_after_n_commands: Optional[int] = None

    calls: List[MotionCommand] = field(default_factory=list)
    arrive_calls: List[MotionCommand] = field(default_factory=list)
    stop_calls: List[str] = field(default_factory=list)
    _inflight: Dict[str, int] = field(default_factory=dict)
    _stop_requested: bool = False
    _stop_ticks: int = 0
    _stop_confirmed: bool = False
    _not_arrived: bool = False

    # -- 运动 -----------------------------------------------------------
    def execute_joint_move(self, command: MotionCommand) -> Optional[MotionResult]:
        key = command.command_id
        if key not in self._inflight:
            self._inflight[key] = 0
            self.calls.append(command)
        self._inflight[key] += 1
        elapsed = self._inflight[key]

        if elapsed <= self.delay_ticks:
            return None  # 仍在执行

        self.arrive_calls.append(command)
        index = len(self.calls)

        if command.primitive_name in self.timeout_primitives:
            return MotionResult(
                success=False,
                arrived=False,
                timed_out=True,
                error_code=ErrorCode.MOTION_TIMEOUT,
                message="fake 超时: %s" % command.primitive_name,
            )
        if command.primitive_name in self.unknown_primitives:
            return MotionResult(
                success=False,
                arrived=False,
                status_unknown=True,
                error_code=ErrorCode.MOTION_STATUS_UNKNOWN,
                message="fake 状态未知: %s" % command.primitive_name,
            )
        if command.primitive_name in self.rejected_primitives:
            return MotionResult(
                success=False,
                arrived=False,
                stop_confirmed=True,
                error_code=ErrorCode.EXECUTION_REJECTED,
                message="fake 被拒绝: %s" % command.primitive_name,
            )
        if self.timeout_after_n_commands is not None and index >= self.timeout_after_n_commands:
            return MotionResult(
                success=False,
                arrived=False,
                timed_out=True,
                error_code=ErrorCode.MOTION_TIMEOUT,
                message="fake 第 %d 条指令超时" % index,
            )
        if self.unknown_after_n_commands is not None and index >= self.unknown_after_n_commands:
            return MotionResult(
                success=False,
                arrived=False,
                status_unknown=True,
                error_code=ErrorCode.MOTION_STATUS_UNKNOWN,
                message="fake 第 %d 条指令状态未知" % index,
            )
        return MotionResult(
            success=True,
            arrived=True,
            stop_confirmed=True,
            final_position_rad=command.target_rad,
            error_code=ErrorCode.OK,
            message="fake 到位",
        )

    # -- 停止 -----------------------------------------------------------
    def request_stop(self, reason: str) -> Optional[StopResult]:
        self.stop_calls.append(reason)
        if not self._stop_requested:
            self._stop_requested = True
            self._stop_ticks = 0
        self._stop_ticks += 1

        if self.stop_confirm_after_ticks is None:
            # 永远不确认停稳：等价于“停止结果不明”
            return StopResult(
                request_delivered=True,
                stop_confirmed=False,
                message="fake: 停止请求已送达，但未确认停稳",
            )
        if self._stop_ticks > self.stop_confirm_after_ticks:
            self._stop_confirmed = True
            return StopResult(
                request_delivered=True, stop_confirmed=True, message="fake: 已确认停稳"
            )
        return None  # 等待后端确认

    @property
    def stop_confirmed(self) -> bool:
        return self._stop_confirmed

    @property
    def stopped(self) -> bool:
        return self._stop_requested


# ---------------------------------------------------------------------------
# 工具
# ---------------------------------------------------------------------------


@dataclass
class FakeTool(ToolPort):
    """假工具（机构证据源）。

    与 ToolStrategy 配对使用：
        - tool_type="passive_hook_v1"：engage 期望 hook_engage，不涉及解锁 IO
        - tool_type="magnetic_latch_v2"：engage 期望 auto_lock，解锁计入 unlock_calls
    """

    tool_type: str = "passive_hook_v1"
    object_id: str = "battery_1"
    engage_ok: bool = True
    engage_evidence: int = EvidenceLevel.GEOMETRY
    engage_locked: bool = True
    test_lift_ok: bool = True
    test_lift_evidence: int = EvidenceLevel.GEOMETRY
    seated: bool = True
    unloaded: bool = True
    unlock_ok: bool = True
    unlock_released: Optional[bool] = True
    disengage_ok: bool = True
    disengage_released: bool = True
    delay_ticks: int = 0

    object_present: bool = False
    engage_calls: int = 0
    test_lift_calls: int = 0
    seat_verify_calls: int = 0
    unload_calls: int = 0
    unlock_calls: int = 0
    disengage_calls: int = 0
    io_events: List[str] = field(default_factory=list)
    received_primitives: List[str] = field(default_factory=list)
    _inflight: Dict[str, int] = field(default_factory=dict)

    # -- 内部 -----------------------------------------------------------
    def _gate(self, key: str) -> bool:
        """返回 True 表示“仍需等待”（本次返回 None）。"""
        if self.delay_ticks <= 0:
            return False
        count = self._inflight.get(key, 0) + 1
        self._inflight[key] = count
        return count <= self.delay_ticks

    def _names(self, primitives: Sequence[MotionPrimitive]) -> Tuple[str, ...]:
        names = tuple(item.name for item in primitives)
        self.received_primitives.extend(names)
        return names

    # -- 端口实现 --------------------------------------------------------
    def engage(self, phase: str, primitives: Sequence[MotionPrimitive]) -> Optional[ToolEvidence]:
        if self._gate("engage"):
            return None
        names = self._names(primitives)
        self.engage_calls += 1
        expected = "auto_lock" if self.tool_type == "magnetic_latch_v2" else "hook_engage"
        self.io_events.append("%s:%s" % (phase, ",".join(names)))
        if expected not in names:
            return ToolEvidence(
                ok=False,
                state=ToolStateCode.STATE_FAULT,
                evidence_level=EvidenceLevel.NONE,
                verified=False,
                error_code=ErrorCode.ENGAGE_FAILED,
                message="engage 原语不匹配工具类型（期望 %s，收到 %s）" % (expected, names),
            )
        if not self.engage_ok:
            self.object_present = False
            return ToolEvidence(
                ok=False,
                state=ToolStateCode.STATE_ENGAGING,
                evidence_level=EvidenceLevel.NONE,
                verified=False,
                locked=False,
                error_code=ErrorCode.ENGAGE_FAILED,
                message="fake: 接合未完成",
            )
        self.object_present = True
        return ToolEvidence(
            ok=True,
            state=ToolStateCode.STATE_ATTACHED,
            evidence_level=self.engage_evidence,
            verified=self.engage_evidence >= EvidenceLevel.GEOMETRY,
            locked=self.engage_locked,
            object_present=True,
            message="fake: 已接合",
        )

    def test_lift(self, primitives: Sequence[MotionPrimitive]) -> Optional[ToolEvidence]:
        if self._gate("test_lift"):
            return None
        self._names(primitives)
        self.test_lift_calls += 1
        if not self.test_lift_ok:
            # 试提失败意味着“是否仍挂载”不确定：保持 object_present=True（保守），
            # 从而禁止自动重试（安全语义 3/§7.1 重复策略）。
            self.object_present = True
            return ToolEvidence(
                ok=False,
                state=ToolStateCode.STATE_ATTACHED,
                evidence_level=self.test_lift_evidence,
                verified=False,
                object_present=True,
                error_code=ErrorCode.ATTACH_NOT_VERIFIED,
                message="fake: 试提时电池未随动",
            )
        return ToolEvidence(
            ok=True,
            state=ToolStateCode.STATE_ATTACHED,
            evidence_level=self.test_lift_evidence,
            verified=self.test_lift_evidence >= EvidenceLevel.GEOMETRY,
            locked=self.engage_locked,
            object_present=True,
            message="fake: 随动证据>=GEOMETRY",
        )

    def seat_verify(self) -> ToolEvidence:
        self.seat_verify_calls += 1
        return ToolEvidence(
            ok=self.seated,
            state=ToolStateCode.STATE_ATTACHED if not self.seated else ToolStateCode.STATE_RELEASING,
            evidence_level=EvidenceLevel.SENSOR_OR_VISION if self.seated else EvidenceLevel.NONE,
            verified=self.seated,
            seated=self.seated,
            object_present=True,
            error_code=ErrorCode.OK if self.seated else ErrorCode.SEAT_NOT_VERIFIED,
            message="fake: 桌面承托已确认" if self.seated else "fake: 未确认桌面承托",
        )

    def unload(self, primitives: Sequence[MotionPrimitive]) -> Optional[ToolEvidence]:
        if self._gate("unload"):
            return None
        self._names(primitives)
        self.unload_calls += 1
        return ToolEvidence(
            ok=self.unloaded,
            state=ToolStateCode.STATE_RELEASING,
            evidence_level=EvidenceLevel.GEOMETRY if self.unloaded else EvidenceLevel.NONE,
            verified=self.unloaded,
            unloaded=self.unloaded,
            object_present=True,
            error_code=ErrorCode.OK if self.unloaded else ErrorCode.RELEASE_NOT_VERIFIED,
            message="fake: 卸载完成" if self.unloaded else "fake: 卸载未确认",
        )

    def unlock(self, pulse_ms: int = 200) -> Optional[ToolEvidence]:
        if self._gate("unlock"):
            return None
        self.unlock_calls += 1
        self.io_events.append("electromagnetic_unlock:%dms" % pulse_ms)
        if not self.unlock_ok:
            return ToolEvidence(
                ok=False,
                state=ToolStateCode.STATE_ATTACHED,
                evidence_level=EvidenceLevel.NONE,
                verified=False,
                locked=True,
                released=False,
                object_present=True,
                error_code=ErrorCode.UNLOCK_FAILED,
                message="fake: 电磁解锁失败",
            )
        return ToolEvidence(
            ok=True,
            state=ToolStateCode.STATE_RELEASING,
            evidence_level=EvidenceLevel.SENSOR_OR_VISION,
            verified=True,
            locked=False,
            released=self.unlock_released,
            object_present=True,
            error_code=ErrorCode.OK,
            message="fake: 解锁脉冲已发出",
        )

    def disengage(self, primitives: Sequence[MotionPrimitive]) -> Optional[ToolEvidence]:
        if self._gate("disengage"):
            return None
        names = self._names(primitives)
        self.disengage_calls += 1
        self.io_events.append("disengage:%s" % ",".join(names))
        if self.disengage_ok and self.disengage_released:
            self.object_present = False
        return ToolEvidence(
            ok=self.disengage_ok,
            state=ToolStateCode.STATE_DETACHED if self.disengage_ok else ToolStateCode.STATE_ATTACHED,
            evidence_level=EvidenceLevel.GEOMETRY if self.disengage_ok else EvidenceLevel.NONE,
            verified=self.disengage_ok and self.disengage_released,
            released=self.disengage_released,
            object_present=False,
            error_code=ErrorCode.OK if self.disengage_ok else ErrorCode.RELEASE_NOT_VERIFIED,
            message="fake: 已分离" if self.disengage_ok else "fake: 分离结果未知",
        )

    def holding_state(self) -> str:
        """能否确认“无载荷”，决定是否允许自动重试。

        object_present 默认 False，仅在 engage 成功后才置 True；
        detach 成功后清空。这样 ENGAGE 失败（尚未接合）时处于“确认无载荷”，
        而 ATTACH_NOT_VERIFIED 等“可能已挂载”场景保持 unknown/holding，禁止自动重试。
        """
        if self.object_present:
            return "holding"
        return "none"


def make_fake_tool_for(strategy: ToolStrategy, **kwargs: Any) -> FakeTool:
    """按策略类型构造配套 FakeTool（保证 engage 原语匹配）。"""
    return FakeTool(tool_type=strategy.type(), **kwargs)


def default_strategy(tool_type: str) -> ToolStrategy:
    if tool_type == "magnetic_latch_v2":
        from .tool_strategy import MagneticLatchV2Strategy

        return MagneticLatchV2Strategy()
    return PassiveHookV1Strategy()
