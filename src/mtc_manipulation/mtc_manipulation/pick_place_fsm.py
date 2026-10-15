"""单块电池抓放状态机（第二层 PickPlace FSM）核心实现。

设计依据：
    docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md
    §3.2（PickPlace 状态表）、§5（工具策略与几何）、§6.4（停止与不确定状态）、
    §7（故障分类与安全联锁）、§10.1（必测故障用例）

本模块**刻意不导入 rclpy / ROS 消息类型**：
    - 便于在只有 Python 3 的环境（含 CI、本机 Humble 环境）中离线单元测试；
    - 便于把 FSM 逻辑与 ROS 节点外壳严格分离。
所有对外能力通过“端口（ports）”依赖注入，测试中可替换为 fakes.Fake*。

安全语义（均有 test/test_pick_place_fsm.py 覆盖）：
    1. placement_verified 只允许由 VERIFY_PLACED 成功路径写入，且仅在该处写；
    2. VERIFY_ATTACHED 必须有几何级（或更高）挂载证据，否则 ATTACH_NOT_VERIFIED=310，
       禁止进入 LIFT / TRANSPORT；
    3. ENGAGE 失败 -> ENGAGE_FAILED=300，且只有确认“无载荷”时才允许退回重试；
    4. SEAT 后未确认桌面承托 -> SEAT_NOT_VERIFIED=400，禁止进入 UNLOAD / DISENGAGE；
    5. DISENGAGE 解锁失败或解锁结果未知 -> UNLOCK_FAILED=410 / RELEASE_NOT_VERIFIED=420
       -> FAULT，禁止强行抽出 / 上抬；
    6. 取消 -> CANCEL_PENDING，等待 MotionPort.request_stop() 返回 stop_confirmed；
       未确认 -> FAULT(CANCEL_NOT_CONFIRMED=520)，绝不因收到 cancel 就声称已安全停止；
    7. 运动状态未知 MOTION_STATUS_UNKNOWN=230 -> FAULT，并禁止自动重发同一条运动命令
       （FSM 进入 FAULT 后 _motion_resend_allowed=False，即使人工调用 step() 也不会重发）；
    8. VERIFY_PLACED 必须在目标区域内连续稳定 placement_stability_sec（默认 3.0 s）
       才允许 SUCCESS，否则 PLACEMENT_FAILED=430 -> FAILED。

V1 / V2 差异只体现在 ENGAGE / VERIFY_ENGAGEMENT / DISENGAGE 三个阶段内部，
由注入的 ToolPort + ToolStrategy 实现差异承载，其余状态完全共用。
"""

from __future__ import annotations

import logging
from math import isfinite
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

LOGGER = logging.getLogger("mtc_manipulation.pick_place_fsm")

# ---------------------------------------------------------------------------
# 工具类型
# ---------------------------------------------------------------------------

TOOL_PASSIVE_HOOK_V1 = "passive_hook_v1"
TOOL_MAGNETIC_LATCH_V2 = "magnetic_latch_v2"

# 允许的目标工位（与 PickPlace.action / 设计 §3.1 一致）
ALLOWED_TARGET_SLOTS = ("T0", "P1", "P2", "P3")

# ---------------------------------------------------------------------------
# 消息常量镜像（取自 mtc_interfaces/msg/ToolState.msg 与 ErrorCodes.msg）
# 这里用纯 Python 常量镜像，避免核心模块依赖 ROS 生成的消息包。
# 若 .msg 变更，必须同步本文件与 test 断言。
# ---------------------------------------------------------------------------


class ToolStateCode:
    """镜像 ToolState.msg 的 state 常量。"""

    STATE_UNKNOWN = 0
    STATE_DETACHED = 1
    STATE_ENGAGING = 2
    STATE_ATTACHED = 3
    STATE_RELEASING = 4
    STATE_FAULT = 5


class EvidenceLevel:
    """镜像 ToolState.msg 的 evidence_level 常量（有序，可比较大小）。"""

    NONE = 0
    COMMAND_ONLY = 1
    GEOMETRY = 2
    SENSOR_OR_VISION = 3

    ORDER = (NONE, COMMAND_ONLY, GEOMETRY, SENSOR_OR_VISION)


class ErrorCode:
    """镜像 mtc_interfaces/msg/ErrorCodes.msg（设计 §7.1，建议冻结）。"""

    OK = 0
    INVALID_TASK = 100
    TARGET_NOT_FOUND = 110
    POSE_STALE = 120
    PLANNING_FAILED = 200
    EXECUTION_REJECTED = 210
    MOTION_TIMEOUT = 220
    MOTION_STATUS_UNKNOWN = 230
    ENGAGE_FAILED = 300
    ATTACH_NOT_VERIFIED = 310
    OBJECT_DROPPED = 320
    SEAT_NOT_VERIFIED = 400
    UNLOCK_FAILED = 410
    RELEASE_NOT_VERIFIED = 420
    PLACEMENT_FAILED = 430
    SAFETY_INTERLOCK = 500
    HARDWARE_FAULT = 510
    CANCEL_NOT_CONFIRMED = 520


class ToolErrorCode:
    """端口返回的错误码：优先使用 ErrorCode.*，本类提供少量端口内部约定。"""

    OK = ErrorCode.OK
    NOT_ALLOWED = ErrorCode.UNLOCK_FAILED
    NOT_SEATED = ErrorCode.SEAT_NOT_VERIFIED
    NOT_UNLOADED = ErrorCode.RELEASE_NOT_VERIFIED
    FAILED = ErrorCode.UNLOCK_FAILED
    UNKNOWN = ErrorCode.MOTION_STATUS_UNKNOWN


# 安全语义 1：只有 VERIFY_PLACED 成功路径允许写入 placement_verified。
# 该常量用作文档与静态检查锚点；运行时断言见 PickPlaceFSM._set_placement_verified()。
PLACEMENT_VERIFIED_OWNER_STATE = "VERIFY_PLACED"


# ---------------------------------------------------------------------------
# 状态枚举（顺序即设计 §3.2 的正常流水线顺序）
# ---------------------------------------------------------------------------


class PickPlaceState(Enum):
    RESOLVE_TARGET = "RESOLVE_TARGET"
    PLAN_ACQUIRE = "PLAN_ACQUIRE"
    MOVE_PRE_ALIGN = "MOVE_PRE_ALIGN"
    ENGAGE = "ENGAGE"
    VERIFY_ENGAGEMENT = "VERIFY_ENGAGEMENT"
    TEST_LIFT = "TEST_LIFT"
    VERIFY_ATTACHED = "VERIFY_ATTACHED"
    LIFT = "LIFT"
    TRANSPORT = "TRANSPORT"
    MOVE_PRE_SEAT = "MOVE_PRE_SEAT"
    SEAT = "SEAT"
    VERIFY_SEATED = "VERIFY_SEATED"
    UNLOAD = "UNLOAD"
    DISENGAGE = "DISENGAGE"
    RETREAT = "RETREAT"
    VERIFY_PLACED = "VERIFY_PLACED"
    SUCCESS = "SUCCESS"
    RECOVERY = "RECOVERY"
    FAILED = "FAILED"
    FAULT = "FAULT"
    CANCEL_PENDING = "CANCEL_PENDING"
    CANCELLED = "CANCELLED"

    def __str__(self) -> str:  # 便于日志与 feedback.substate
        return self.value


#: 终态：step() 到达后不再变化
TERMINAL_STATES = frozenset(
    {
        PickPlaceState.SUCCESS,
        PickPlaceState.FAILED,
        PickPlaceState.FAULT,
        PickPlaceState.CANCELLED,
    }
)

#: 运动在途状态（这些状态超时说明“机器人是否还在动”不确定 -> FAULT，禁止重发）
MOTION_IN_FLIGHT_STATES = frozenset(
    {
        PickPlaceState.MOVE_PRE_ALIGN,
        PickPlaceState.ENGAGE,
        PickPlaceState.TEST_LIFT,
        PickPlaceState.LIFT,
        PickPlaceState.TRANSPORT,
        PickPlaceState.MOVE_PRE_SEAT,
        PickPlaceState.SEAT,
        PickPlaceState.UNLOAD,
        PickPlaceState.DISENGAGE,
        PickPlaceState.RETREAT,
    }
)


# ---------------------------------------------------------------------------
# 数据类：端口契约
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PoseEstimate:
    """一次目标位姿观测（纯数据，替代 geometry_msgs/PoseStamped）。"""

    object_id: str
    color: int = 0
    ring_pose: Optional[Tuple[float, float, float]] = None
    body_pose: Optional[Tuple[float, float, float]] = None
    confidence: float = 0.0
    stamp_s: float = 0.0
    ring_pose_valid: bool = True
    at_target: bool = False
    settled: bool = True
    #: 视觉是否确认“物体随工具一起运动”（VERIFY_ATTACHED 的独立证据之一）
    followed: bool = False

    @property
    def has_ring_pose(self) -> bool:
        return self.ring_pose is not None and self.ring_pose_valid


@dataclass(frozen=True)
class DetectionResult:
    """PerceptionPort.get_detection 的返回。"""

    found: bool = False
    unique: bool = True
    stale: bool = False
    pose: Optional[PoseEstimate] = None
    error_code: int = ErrorCode.OK
    message: str = ""

    @property
    def ok(self) -> bool:
        return self.found and self.unique and not self.stale and self.pose is not None


@dataclass(frozen=True)
class MotionPrimitive:
    """工具策略产生的运动原语（设计 §5.1）。

    name            原语名（如 side_insert / hook_engage / electromagnetic_unlock / withdraw）
    target_pose     目标位姿占位（本包不做 TF 求解，仅透传标签）
    max_linear_speed / max_angular_speed  速度上限，由运动执行层强制
    requires_contact 是否接触敏感动作（接触阶段不得用自由空间高速轨迹）
    """

    name: str
    target_pose: Any = None
    max_linear_speed: float = 0.05
    max_angular_speed: float = 0.2
    requires_contact: bool = False


@dataclass(frozen=True)
class MotionCommand:
    """等价于 ExecuteJointMove.action Goal 的字段集合。"""

    command_id: str
    joint_names: Tuple[str, ...]
    target_rad: Tuple[float, ...]
    velocity_scaling: float = 0.2
    acceleration_scaling: float = 0.2
    timeout_ms: int = 5000
    primitive_name: str = ""
    requires_contact: bool = False


@dataclass(frozen=True)
class MotionResult:
    """等价于 ExecuteJointMove.action Result/Feedback 的可观测结果。"""

    success: bool = False
    arrived: bool = False
    stop_confirmed: bool = False
    final_position_rad: Tuple[float, ...] = ()
    error_code: int = ErrorCode.OK
    message: str = ""
    status_unknown: bool = False
    timed_out: bool = False

    @property
    def ok(self) -> bool:
        return self.success and self.arrived


@dataclass(frozen=True)
class StopResult:
    """取消 / 停止请求的结果；stop_confirmed 必须由运动后端独立确认。"""

    request_delivered: bool = False
    stop_confirmed: bool = False
    message: str = ""


@dataclass(frozen=True)
class PlanResult:
    """PlannerPort.plan 的返回。"""

    ok: bool = False
    error_code: int = ErrorCode.OK
    message: str = ""
    primitives: Tuple[MotionPrimitive, ...] = ()


@dataclass(frozen=True)
class ToolEvidence:
    """ToolPort 各方法返回的机构证据（对应 ToolState.msg）。"""

    ok: bool = False
    state: int = ToolStateCode.STATE_UNKNOWN
    evidence_level: int = EvidenceLevel.NONE
    verified: bool = False
    locked: Optional[bool] = None
    released: Optional[bool] = None
    seated: bool = False
    unloaded: bool = False
    object_present: bool = False
    error_code: int = ErrorCode.OK
    message: str = ""

    def has_geometry_evidence(self) -> bool:
        return self.evidence_level >= EvidenceLevel.GEOMETRY


@dataclass
class PickPlaceConfig:
    """抓放参数（数值为保守默认值，未做实机标定，见设计 §8）。"""

    tool_type: str = TOOL_PASSIVE_HOOK_V1
    placement_stability_sec: float = 3.0
    perception_max_age_s: float = 0.3
    recovery_max_attempts: int = 1
    verify_test_lift: bool = True
    verify_placement: bool = True
    joint_names: Tuple[str, ...] = ("J1", "J2", "J3", "J4", "J5", "J6")
    default_velocity_scaling: float = 0.2
    default_acceleration_scaling: float = 0.2
    default_timeout_ms: int = 5000
    # 每状态看门狗超时（秒）。运动在途状态超时 = 状态不确定 -> FAULT
    state_timeouts: Dict[str, float] = field(
        default_factory=lambda: {
            "RESOLVE_TARGET": 2.0,
            "PLAN_ACQUIRE": 2.0,
            "MOVE_PRE_ALIGN": 30.0,
            "ENGAGE": 15.0,
            "VERIFY_ENGAGEMENT": 5.0,
            "TEST_LIFT": 15.0,
            "VERIFY_ATTACHED": 5.0,
            "LIFT": 30.0,
            "TRANSPORT": 60.0,
            "MOVE_PRE_SEAT": 30.0,
            "SEAT": 20.0,
            "VERIFY_SEATED": 5.0,
            "UNLOAD": 15.0,
            "DISENGAGE": 20.0,
            "RETREAT": 30.0,
            "VERIFY_PLACED": 30.0,
        }
    )


@dataclass
class StepResult:
    """单次 step() 的返回：状态 + 错误码 + 是否可恢复。"""

    state: PickPlaceState
    error_code: int = ErrorCode.OK
    resumable: bool = False
    message: str = ""

    @property
    def terminal(self) -> bool:
        return self.state in TERMINAL_STATES


@dataclass
class PickPlaceResult:
    """等价于 PickPlace.action Result。"""

    success: bool = False
    placement_verified: bool = False
    error_code: int = ErrorCode.OK
    message: str = ""
    final_state: PickPlaceState = PickPlaceState.RESOLVE_TARGET
    states_history: Tuple[PickPlaceState, ...] = ()


@dataclass
class PickPlaceFeedback:
    """等价于 PickPlace.action Feedback。"""

    substate: str = ""
    progress: float = 0.0
    object_attached_estimated: bool = False
    detail: str = ""


# ---------------------------------------------------------------------------
# 端口（Ports）：全部为鸭子类型协议，核心只调用下列方法
# ---------------------------------------------------------------------------


class PerceptionPort:
    """感知端口。实现方见 fakes.FakePerception / pick_place_server.RosPerceptionPort。"""

    def get_detection(
        self, object_id: str, expected_color: Optional[int], max_age_s: float
    ) -> DetectionResult:
        """expected_color 为 None 时跳过颜色校验（用于放置后核验）。"""
        raise NotImplementedError


class PlannerPort:
    """规划端口。只做“规划可行性 + 运动原语返回”，不掌控关节控制权。"""

    def plan(self, phase: str, target: Any) -> PlanResult:
        raise NotImplementedError


class MotionPort:
    """运动执行端口（对应 ExecuteJointMove 的能力边界）。"""

    def execute_joint_move(self, command: MotionCommand) -> Optional[MotionResult]:
        """返回 None 表示“仍在执行”（由调用方按状态看门狗等待）。"""
        raise NotImplementedError

    def request_stop(self, reason: str) -> StopResult:
        """软件停止请求；stop_confirmed 必须由后端独立确认。"""
        raise NotImplementedError


class ToolPort:
    """工具端口：产生机构证据，不发出关节命令。"""

    def engage(self, phase: str, primitives: Sequence[MotionPrimitive]) -> Optional[ToolEvidence]:
        raise NotImplementedError

    def test_lift(self, primitives: Sequence[MotionPrimitive]) -> Optional[ToolEvidence]:
        raise NotImplementedError

    def seat_verify(self) -> ToolEvidence:
        raise NotImplementedError

    def unload(self, primitives: Sequence[MotionPrimitive]) -> Optional[ToolEvidence]:
        raise NotImplementedError

    def unlock(self, pulse_ms: int = 200) -> Optional[ToolEvidence]:
        raise NotImplementedError

    def disengage(self, primitives: Sequence[MotionPrimitive]) -> Optional[ToolEvidence]:
        raise NotImplementedError

    def holding_state(self) -> str:
        """返回 'none' / 'holding' / 'unknown'：决定失败后是否允许自动重试。"""
        raise NotImplementedError


class ClockPort:
    """时间端口；测试用假时钟加速。"""

    def now(self) -> float:
        raise NotImplementedError


# ---------------------------------------------------------------------------
# 内部辅助
# ---------------------------------------------------------------------------

_RUN = object()
"""哨兵：端口返回该值表示调用未完成，需要后续 step() 继续等待。"""


def _find_primitive(
    primitives: Iterable[MotionPrimitive], name: str
) -> Optional[MotionPrimitive]:
    for item in primitives:
        if item.name == name:
            return item
    return None


#: 确定性状态转移表（from -> to）。用于 README 描述与自检，实际转移由 handler 返回。
TRANSITION_TABLE: Dict[str, Tuple[str, ...]] = {
    "RESOLVE_TARGET": ("PLAN_ACQUIRE", "RECOVERY", "FAILED", "CANCEL_PENDING", "FAULT"),
    "PLAN_ACQUIRE": ("MOVE_PRE_ALIGN", "RECOVERY", "FAILED", "CANCEL_PENDING", "FAULT"),
    "MOVE_PRE_ALIGN": ("ENGAGE", "RECOVERY", "FAILED", "CANCEL_PENDING", "FAULT"),
    "ENGAGE": ("VERIFY_ENGAGEMENT", "RECOVERY", "FAILED", "CANCEL_PENDING", "FAULT"),
    "VERIFY_ENGAGEMENT": ("TEST_LIFT", "FAULT", "FAILED", "CANCEL_PENDING"),
    "TEST_LIFT": ("VERIFY_ATTACHED", "FAULT", "FAILED", "CANCEL_PENDING"),
    "VERIFY_ATTACHED": ("LIFT", "FAULT", "FAILED", "CANCEL_PENDING"),
    "LIFT": ("TRANSPORT", "FAULT", "FAILED", "CANCEL_PENDING"),
    "TRANSPORT": ("MOVE_PRE_SEAT", "FAULT", "FAILED", "CANCEL_PENDING"),
    "MOVE_PRE_SEAT": ("SEAT", "FAULT", "FAILED", "CANCEL_PENDING"),
    "SEAT": ("VERIFY_SEATED", "FAULT", "FAILED", "CANCEL_PENDING"),
    "VERIFY_SEATED": ("UNLOAD", "FAULT", "FAILED", "CANCEL_PENDING"),
    "UNLOAD": ("DISENGAGE", "FAULT", "FAILED", "CANCEL_PENDING"),
    "DISENGAGE": ("RETREAT", "FAULT", "FAILED", "CANCEL_PENDING"),
    "RETREAT": ("VERIFY_PLACED", "FAULT", "FAILED", "CANCEL_PENDING"),
    "VERIFY_PLACED": ("SUCCESS", "FAILED", "CANCEL_PENDING", "FAULT"),
    "RECOVERY": ("RESOLVE_TARGET", "FAILED", "FAULT", "CANCEL_PENDING"),
    "CANCEL_PENDING": ("CANCELLED", "FAULT"),
    "SUCCESS": (),
    "FAILED": (),
    "FAULT": (),
    "CANCELLED": (),
}

#: 进入这些状态后需要运动后端确认停稳；未确认一律 CANCEL_NOT_CONFIRMED
CANCEL_SAFE_TERMINAL = frozenset(
    {PickPlaceState.SUCCESS, PickPlaceState.FAILED, PickPlaceState.FAULT, PickPlaceState.CANCELLED}
)


# ---------------------------------------------------------------------------
# 状态机主体
# ---------------------------------------------------------------------------


class PickPlaceFSM:
    """单块电池抓放状态机（纯 Python，可离线单测）。

    典型用法::

        fsm = PickPlaceFSM(perception, planner, motion, tool, clock, strategy, config)
        fsm.start("battery_1", expected_color=1, target_slot="P1", request_id="req-1",
                  placement_stability_sec=3.0)
        result = fsm.run(max_ticks=400, tick_seconds=0.05)
    """

    MAX_COMMAND_ID = 10 ** 9

    def __init__(
        self,
        perception: PerceptionPort,
        planner: PlannerPort,
        motion: MotionPort,
        tool: ToolPort,
        clock: ClockPort,
        strategy: Any,
        config: Optional[PickPlaceConfig] = None,
    ) -> None:
        self.perception = perception
        self.planner = planner
        self.motion = motion
        self.tool = tool
        self.clock = clock
        self.strategy = strategy
        self.config = config or PickPlaceConfig()
        if strategy is not None:
            # 工具策略与实际工具类型必须一致，否则证据语义不可信
            declared = strategy.type()
            if declared != self.config.tool_type:
                raise ValueError(
                    "tool_type 配置(%s)与策略(%s)不一致" % (self.config.tool_type, declared)
                )

        # --- 请求上下文 ---
        self.request_id = ""
        self.object_id = ""
        self.expected_color = 0
        self.target_slot = ""
        self.placement_stability_sec = self.config.placement_stability_sec

        # --- 运行状态 ---
        self.state = PickPlaceState.RESOLVE_TARGET
        self.error_code = ErrorCode.OK
        self.message = ""
        self.states_history: List[PickPlaceState] = []
        # 安全语义 1：该字段只允许 _set_placement_verified() 从 VERIFY_PLACED 写入
        self.placement_verified = False
        self.object_attached_estimated = False

        # --- 内部簿记 ---
        self._state_entered_at = 0.0
        self._started_at = 0.0
        self._started = False
        self._step_count = 0
        self._recovery_attempts = 0
        self._resumable_error = False
        self._last_resumable_state: Optional[PickPlaceState] = None
        self._cancel_pending = False
        self._cancel_reason = ""
        self._stop_confirmed = False
        self._motion_resend_allowed = True
        self._command_seq = 0
        self._command_ids: List[str] = []
        self._primitive_calls: List[str] = []
        self._port_results: Dict[Tuple[str, str], Any] = {}
        self._stability_started_at: Optional[float] = None
        #: 声明 SUCCESS 时的实际累计稳定时间（秒），用于日志/回放与测试断言
        self.stability_verified_for_s: float = 0.0
        self._placement_last_pose: Optional[Tuple[float, float, float]] = None
        self.on_event: Optional[Callable[[str, PickPlaceState], None]] = None
        self.logger = LOGGER

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def start(
        self,
        object_id: str,
        expected_color: int,
        target_slot: str,
        request_id: str = "",
        placement_stability_sec: Optional[float] = None,
    ) -> None:
        """校验并受理一次抓放请求。非法目标工位立即 INVALID_TASK。"""
        self.request_id = request_id
        self.object_id = object_id
        self.expected_color = int(expected_color)
        self.target_slot = str(target_slot)
        # 优先级：Goal 显式传入 > 运行时配置（PickPlaceConfig.placement_stability_sec）
        if placement_stability_sec is not None:
            self.placement_stability_sec = float(placement_stability_sec)
        else:
            self.placement_stability_sec = float(self.config.placement_stability_sec)

        self._started = True
        self._started_at = self.clock.now()
        self._step_count = 0
        self._recovery_attempts = 0
        self._cancel_pending = False
        self._stop_confirmed = False
        self._command_seq = 0
        self._command_ids = []
        self._primitive_calls = []
        self._port_results = {}
        self._stability_started_at = None
        self.stability_verified_for_s = 0.0
        self._placement_last_pose = None
        self._pending_command = None
        self._pending_command_label = ""
        self.object_attached_estimated = False
        self.error_code = ErrorCode.OK
        self.message = ""

        if not isfinite(self.placement_stability_sec) or self.placement_stability_sec <= 0.0:
            self.error_code = ErrorCode.INVALID_TASK
            self.message = "placement_stability_sec must be finite and strictly positive"
            self._enter_state(PickPlaceState.FAILED)
            return

        if self.target_slot not in ALLOWED_TARGET_SLOTS:
            self.error_code = ErrorCode.INVALID_TASK
            self.message = "非法目标工位: %r（仅允许 %s）" % (
                self.target_slot,
                "/".join(ALLOWED_TARGET_SLOTS),
            )
            self._enter_state(PickPlaceState.FAILED)
            return

        self._enter_state(PickPlaceState.RESOLVE_TARGET)
        self._emit("goal_accepted", self.state)

    def request_cancel(self, reason: str = "") -> None:
        """记录取消请求。真正的安全停止必须在 CANCEL_PENDING 中由后端确认。"""
        if self.state in TERMINAL_STATES:
            return
        if self.state is PickPlaceState.CANCEL_PENDING:
            return
        self._cancel_pending = True
        self._cancel_reason = reason or "cancel requested"
        self._emit("cancel_requested", self.state)

    # ------------------------------------------------------------------
    # 单步推进
    # ------------------------------------------------------------------

    def step(self) -> StepResult:
        """推进一步；可被反复调用直到终态。"""
        self._step_count += 1

        if self.state in TERMINAL_STATES:
            return StepResult(state=self.state, error_code=self.error_code, message=self.message)

        # 取消优先级最高：但绝不在此直接声称已停稳
        if self._cancel_pending and self.state is not PickPlaceState.CANCEL_PENDING:
            self.logger.info("收到取消请求，进入 CANCEL_PENDING：%s", self._cancel_reason)
            self._enter_state(PickPlaceState.CANCEL_PENDING)

        handler = _STATE_ACTIONS.get(self.state)
        if handler is None:  # 防御：未知状态 -> FAULT
            return self._fault(ErrorCode.HARDWARE_FAULT, "未实现的状态处理: %s" % self.state)

        outcome = handler(self)

        if outcome is _RUN:
            # 端口未完成：等待，并做看门狗超时判定
            timeout = self.config.state_timeouts.get(self.state.value)
            elapsed = self.clock.now() - self._state_entered_at
            if timeout is not None and elapsed > timeout:
                if self.state in MOTION_IN_FLIGHT_STATES:
                    # 超时后机器人是否还在动未知：禁止自动重发
                    return self._fault(
                        ErrorCode.MOTION_STATUS_UNKNOWN,
                        "状态 %s 超过 %.1fs 未返回，运动状态未知，禁止重发" % (self.state, timeout),
                        resend_allowed=False,
                    )
                previous = self.state
                outcome = self._error(
                    ErrorCode.MOTION_TIMEOUT,
                    "状态 %s 超时 %.1fs" % (self.state, timeout),
                    resumable=self._is_resumable_context(),
                )
                self._resumable_error = outcome.resumable
                self._last_resumable_state = previous
                self._enter_state(outcome.state)
                return outcome
            return StepResult(state=self.state, error_code=self.error_code, message=self.message)

        if isinstance(outcome, StepResult):
            if outcome.state is not self.state:
                self.error_code = outcome.error_code
                self.message = outcome.message
                self._resumable_error = outcome.resumable
                self._last_resumable_state = self.state
                self._enter_state(outcome.state)
                return StepResult(
                    state=outcome.state,
                    error_code=outcome.error_code,
                    resumable=outcome.resumable,
                    message=outcome.message,
                )
            return outcome

        raise TypeError("handler 返回类型非法: %r" % (outcome,))

    def step_until_terminal(self, max_ticks: int = 500) -> PickPlaceState:
        for _ in range(max_ticks):
            self.step()
            if self.state in TERMINAL_STATES:
                break
        return self.state

    def run(self, max_ticks: int = 500, tick_seconds: float = 0.0) -> PickPlaceResult:
        """驱动到终态。tick_seconds>0 时会推进假时钟（仅适用于 FakeClock）。"""
        for _ in range(max_ticks):
            self.step()
            if self.state in TERMINAL_STATES:
                break
            if tick_seconds > 0:
                advance = getattr(self.clock, "advance", None)
                if callable(advance):
                    advance(tick_seconds)
        return self.result()

    def result(self) -> PickPlaceResult:
        return PickPlaceResult(
            success=self.state is PickPlaceState.SUCCESS,
            placement_verified=self.placement_verified,
            error_code=self.error_code,
            message=self.message,
            final_state=self.state,
            states_history=tuple(self.states_history),
        )

    def feedback(self) -> PickPlaceFeedback:
        return PickPlaceFeedback(
            substate=self.state.value,
            progress=self._progress(),
            object_attached_estimated=self.object_attached_estimated,
            detail=self.message,
        )

    # ------------------------------------------------------------------
    # 内部：状态与错误处理
    # ------------------------------------------------------------------

    def _progress(self) -> float:
        if self.state is PickPlaceState.SUCCESS:
            return 1.0
        total = len(_PIPELINE_STATES) - 1
        try:
            index = _PIPELINE_STATES.index(self.state)
        except ValueError:
            return 0.0
        return min(1.0, max(0.0, index / float(total)))

    def _enter_state(self, state: PickPlaceState) -> None:
        self.state = state
        self._state_entered_at = self.clock.now()
        # 离开旧状态即释放“在途命令”引用：再次进入同一状态必须生成新的 command_id，
        # 而在同一次状态访问内（端口未返回时）必须复用同一条命令，绝不重发。
        self._pending_command = None
        self._pending_command_label = ""
        self.states_history.append(state)
        self._emit("state_entered", state)

    def _emit(self, event: str, state: PickPlaceState) -> None:
        if self.on_event is not None:
            try:
                self.on_event(event, state)
            except Exception:  # 回调异常不得破坏状态机时序
                self.logger.exception("on_event 回调异常")

    def _next_command_id(self, label: str) -> str:
        self._command_seq += 1
        command_id = "%s#%s-%03d" % (self.request_id or "req", label, self._command_seq)
        self._command_ids.append(command_id)
        return command_id

    @property
    def command_ids(self) -> Tuple[str, ...]:
        return tuple(self._command_ids)

    @property
    def motion_resend_allowed(self) -> bool:
        return self._motion_resend_allowed

    @property
    def stop_confirmed(self) -> bool:
        return self._stop_confirmed

    @property
    def recovery_attempts(self) -> int:
        return self._recovery_attempts

    def _set_placement_verified(self) -> None:
        """安全语义 1 的唯一写入口。"""
        assert (
            self.state is PickPlaceState.VERIFY_PLACED
            or self.state.value == PLACEMENT_VERIFIED_OWNER_STATE
        ), "placement_verified 只允许由 VERIFY_PLACED 设置"
        self.placement_verified = True

    def _error(
        self, code: int, message: str, resumable: bool = False, next_state: Optional[PickPlaceState] = None
    ) -> StepResult:
        if next_state is None:
            next_state = PickPlaceState.RECOVERY if resumable else PickPlaceState.FAILED
        self.error_code = code
        self.message = message
        self.logger.warning("[%s] 错误 %s: %s", self.state, code, message)
        self._emit("error", self.state)
        return StepResult(state=next_state, error_code=code, resumable=resumable, message=message)

    def _fault(self, code: int, message: str, resend_allowed: bool = True) -> StepResult:
        """进入 FAULT。resend_allowed=False 用于运动状态未知，禁止自动重发。"""
        if not resend_allowed:
            self._motion_resend_allowed = False
        self.error_code = code
        self.message = message
        self.logger.error("[%s] FAULT %s: %s", self.state, code, message)
        self._emit("fault", self.state)
        # 关键：必须真正切换到 FAULT。否则 handler 直接返回 _fault(...) 时，
        # step() 会认为“状态未变化”而继续推进，导致锁定失效。
        self._enter_state(PickPlaceState.FAULT)
        return StepResult(state=PickPlaceState.FAULT, error_code=code, resumable=False, message=message)

    def force_fault(self, code: int, message: str, resend_allowed: bool = False) -> StepResult:
        """外部监护（如 Action 执行步数上限）判定危险/不确定时，强制进入 FAULT。

        默认 resend_allowed=False：一旦判定运动状态不确定，禁止自动重发任何指令。
        """
        if self.state in TERMINAL_STATES:
            return StepResult(
                state=self.state, error_code=self.error_code, resumable=False, message=self.message
            )
        return self._fault(code, message, resend_allowed=resend_allowed)

    def _is_resumable_context(self) -> bool:
        """是否允许退回重试：必须确认当前无载荷且运动状态已知。"""
        if not self._motion_resend_allowed:
            return False
        try:
            holding = self.tool.holding_state()
        except Exception:
            self.logger.exception("holding_state 查询失败，保守判定为不明确")
            return False
        if holding != "none":
            self.logger.warning("holding_state=%s，禁止自动重试", holding)
            return False
        return True

    def _do(self, key: str, func: Callable[[], Any]) -> Any:
        """确保带副作用端口调用每状态只执行一次。

        返回 _RUN 表示调用尚未完成（结果为 None），调用方返回 _RUN 等待。
        注意：返回 _RUN 时**不缓存**结果，因此 func 会被再次调用；
        所有“必须在多次调用间保持一致”的可变对象（如 MotionCommand）必须通过
        _cached_command() 复用，绝不允许在等待期间生成新的 command_id 重发指令。
        """
        cache_key = (self.state.value, key)
        if cache_key in self._port_results:
            return self._port_results[cache_key]
        value = func()
        if value is None:
            return _RUN
        self._port_results[cache_key] = value
        return value

    def _clear_port_results(self) -> None:
        self._port_results.clear()
        self._pending_command = None
        self._pending_command_label = ""

    def _cached_command(
        self, label: str, primitive: MotionPrimitive, joint_names: Optional[Sequence[str]] = None
    ) -> MotionCommand:
        """构造并缓存运动命令：同一状态内的多次 step 必须复用同一条命令。

        安全语义：运动在途时（端口返回 None）绝不允许重新发令。
        """
        key = "%s:%s" % (self.state.value, label)
        if self._pending_command is not None and self._pending_command_label == key:
            return self._pending_command
        command = self._make_command(label, primitive, joint_names)
        self._pending_command = command
        self._pending_command_label = key
        self.logger.info("生成运动命令 %s（%s）", command.command_id, primitive.name)
        return command

    def _make_command(
        self, label: str, primitive: MotionPrimitive, joint_names: Optional[Sequence[str]] = None
    ) -> MotionCommand:
        names = tuple(joint_names or self.config.joint_names)
        target = tuple(0.0 for _ in names)
        return MotionCommand(
            command_id=self._next_command_id(label),
            joint_names=names,
            target_rad=target,
            velocity_scaling=self.config.default_velocity_scaling,
            acceleration_scaling=self.config.default_acceleration_scaling,
            timeout_ms=self.config.default_timeout_ms,
            primitive_name=primitive.name,
            requires_contact=primitive.requires_contact,
        )

    # ------------------------------------------------------------------
    # 状态处理器
    # ------------------------------------------------------------------

    def _h_resolve_target(self) -> StepResult:
        # 安全语义：感知不可靠（颜色不符 / 过期 / 不唯一）时绝不运动
        detection = self._do(
            "detection",
            lambda: self.perception.get_detection(
                self.object_id, self.expected_color, self.config.perception_max_age_s
            ),
        )
        if detection is _RUN:
            return _RUN

        if not detection.found:
            return self._error(
                ErrorCode.TARGET_NOT_FOUND,
                detection.message or "未找到目标对象 %s" % self.object_id,
                resumable=True,
            )
        if not detection.unique:
            return self._error(
                ErrorCode.TARGET_NOT_FOUND,
                detection.message or "目标颜色/ID 不唯一",
                resumable=True,
            )
        if detection.stale:
            return self._error(
                ErrorCode.POSE_STALE,
                detection.message or "感知/TF 数据过期",
                resumable=True,
            )
        pose = detection.pose
        if pose is None or not pose.has_ring_pose:
            return self._error(
                ErrorCode.TARGET_NOT_FOUND,
                "提环位姿不可用（ring_pose_valid=False）",
                resumable=True,
            )
        # 安全语义（设计 V0.1 §4.4）：expected_color 是防抓错的**双重校验**。
        # FSM 必须独立比对感知颜色，不能只依赖端口内部校验；
        # 颜色不匹配或颜色未知（COLOR_UNKNOWN=0）一律拒绝运动。
        if self.expected_color:
            observed_color = int(getattr(pose, "color", 0) or 0)
            if observed_color == 0:
                return self._error(
                    ErrorCode.TARGET_NOT_FOUND,
                    "目标颜色未知（COLOR_UNKNOWN），拒绝在颜色未确认时运动；"
                    "object_id=%s" % self.object_id,
                    resumable=True,
                )
            if observed_color != int(self.expected_color):
                return self._error(
                    ErrorCode.TARGET_NOT_FOUND,
                    "目标颜色不匹配：期望 color=%d，感知 color=%d（拒绝抓错颜色）；"
                    "object_id=%s" % (int(self.expected_color), observed_color, self.object_id),
                    resumable=True,
                )
        self._target_pose = pose
        return StepResult(state=PickPlaceState.PLAN_ACQUIRE)

    def _h_plan_acquire(self) -> StepResult:
        """规划接近段 + 接合段 + 试提段原语。"""
        primitives = self._primitives("acquire", lambda: self.strategy.make_acquire_plan(self._target_pose))
        if primitives is _RUN:
            return _RUN
        engage_primitives = self._primitives(
            "engage", lambda: self.strategy.make_engage_plan(self._target_pose)
        )
        if engage_primitives is _RUN:
            return _RUN
        test_lift_primitives = self._primitives(
            "test_lift", lambda: self.strategy.make_test_lift_plan(self._target_pose)
        )
        if test_lift_primitives is _RUN:
            return _RUN

        plan = self._do(
            "plan_acquire",
            lambda: self.planner.plan(
                "ACQUIRE",
                {
                    "object_id": self.object_id,
                    "slot": self.target_slot,
                    "ring_pose": getattr(self._target_pose, "ring_pose", None),
                    "primitives": tuple(p.name for p in primitives),
                },
            ),
        )
        if plan is _RUN:
            return _RUN
        if not plan.ok:
            return self._error(
                plan.error_code or ErrorCode.PLANNING_FAILED,
                plan.message or "接近段规划失败",
                resumable=True,
            )
        return StepResult(state=PickPlaceState.MOVE_PRE_ALIGN)

    def _h_move_pre_align(self) -> StepResult:
        primitives = self._primitives("acquire", lambda: self.strategy.make_acquire_plan(self._target_pose))
        if primitives is _RUN:
            return _RUN
        if not primitives:
            return self._error(ErrorCode.PLANNING_FAILED, "接近段无运动原语", resumable=True)
        command = self._cached_command("pre_align", primitives[0])
        self._primitive_calls.append(primitives[0].name)
        result = self._do("motion", lambda: self.motion.execute_joint_move(command))
        if result is _RUN:
            return _RUN
        failure = self._motion_failure(result)
        if failure is not None:
            return failure
        return StepResult(state=PickPlaceState.ENGAGE)

    # ---- V1/V2 差异区（一）：ENGAGE 与 VERIFY_ENGAGEMENT -------------------

    def _h_engage(self) -> StepResult:
        primitives = self._primitives(
            "engage", lambda: self.strategy.make_engage_plan(self._target_pose)
        )
        if primitives is _RUN:
            return _RUN
        for prim in primitives:
            self._primitive_calls.append(prim.name)
        evidence = self._do("engage", lambda: self.tool.engage("ENGAGE", primitives))
        if evidence is _RUN:
            return _RUN
        self._engage_evidence = evidence
        return StepResult(
            state=PickPlaceState.VERIFY_ENGAGEMENT,
            error_code=evidence.error_code,
            message=evidence.message,
        )

    def _h_verify_engagement(self) -> StepResult:
        evidence = getattr(self, "_engage_evidence", None)
        if not evidence.ok:
            # 安全语义 3：ENGAGE 失败 -> 300，只有确认无载荷才允许退回重试
            resumable = self._is_resumable_context()
            return self._error(
                evidence.error_code or ErrorCode.ENGAGE_FAILED,
                evidence.message or "接合（穿环/锁止）未完成",
                resumable=resumable,
            )
        if evidence.evidence_level < EvidenceLevel.GEOMETRY:
            # 仅有指令级证据：不得据此认定接合成立
            return self._error(
                ErrorCode.ENGAGE_FAILED,
                "接合证据等级不足（evidence_level=%d）" % evidence.evidence_level,
                resumable=self._is_resumable_context(),
            )
        if evidence.state != ToolStateCode.STATE_ATTACHED:
            return self._fault(
                ErrorCode.HARDWARE_FAULT,
                "接合后工具状态异常: %d" % evidence.state,
            )
        return StepResult(state=PickPlaceState.TEST_LIFT)

    def _h_test_lift(self) -> StepResult:
        primitives = self._primitives(
            "test_lift", lambda: self.strategy.make_test_lift_plan(self._target_pose)
        )
        if primitives is _RUN:
            return _RUN
        if not primitives:
            return self._error(ErrorCode.PLANNING_FAILED, "试提段无运动原语", resumable=False)

        if self.config.verify_test_lift:
            evidence = self._do("test_lift", lambda: self.tool.test_lift(primitives))
            if evidence is _RUN:
                return _RUN
            self._test_lift_evidence = evidence
            if not evidence.ok:
                return self._error(
                    evidence.error_code or ErrorCode.ATTACH_NOT_VERIFIED,
                    evidence.message or "试提失败",
                    resumable=self._is_resumable_context(),
                )

        command = self._cached_command("test_lift", primitives[0])
        self._primitive_calls.append(primitives[0].name)
        motion = self._do("motion", lambda: self.motion.execute_joint_move(command))
        if motion is _RUN:
            return _RUN
        failure = self._motion_failure(motion)
        if failure is not None:
            return failure
        self._test_lift_motion = motion
        return StepResult(state=PickPlaceState.VERIFY_ATTACHED)

    def _h_verify_attached(self) -> StepResult:
        """安全语义 2：必须有几何级（或更高）挂载证据，否则 310，禁止运输。"""
        evidence = getattr(self, "_test_lift_evidence", None)
        if evidence is None:
            primitives = self._primitives(
                "test_lift", lambda: self.strategy.make_test_lift_plan(self._target_pose)
            )
            if primitives is _RUN:
                return _RUN
            evidence = self._do("test_lift", lambda: self.tool.test_lift(primitives))
            if evidence is _RUN:
                return _RUN
            self._test_lift_evidence = evidence

        vision_follow = False
        if evidence.evidence_level < EvidenceLevel.GEOMETRY:
            # 退一步：尝试用“视觉随动确认”补足独立证据
            detection = self._do(
                "follow_detection",
                lambda: self.perception.get_detection(
                    self.object_id, self.expected_color, self.config.perception_max_age_s
                ),
            )
            if detection is _RUN:
                return _RUN
            pose = detection.pose if detection is not None else None
            vision_follow = bool(
                detection is not None
                and detection.ok
                and pose is not None
                and getattr(pose, "followed", False)
            )

        if not (evidence.has_geometry_evidence() or vision_follow):
            self.object_attached_estimated = False
            return self._error(
                ErrorCode.ATTACH_NOT_VERIFIED,
                "缺少挂载证据（evidence_level=%d, vision_follow=%s），禁止进入 LIFT/TRANSPORT"
                % (evidence.evidence_level, vision_follow),
                resumable=self._is_resumable_context(),
            )

        self.object_attached_estimated = True
        self._emit("object_attached_verified", self.state)
        return StepResult(state=PickPlaceState.LIFT)

    def _h_lift(self) -> StepResult:
        primitives = self._primitives(
            "lift", lambda: self.strategy.make_lift_plan(self._target_pose)
        )
        if primitives is _RUN:
            return _RUN
        plan = self._do(
            "plan_transport",
            lambda: self.planner.plan(
                "TRANSPORT",
                {
                    "object_id": self.object_id,
                    "slot": self.target_slot,
                    "primitives": tuple(p.name for p in primitives),
                },
            ),
        )
        if plan is _RUN:
            return _RUN
        if not plan.ok:
            return self._fault(
                plan.error_code or ErrorCode.PLANNING_FAILED,
                plan.message or "已挂载物体后的搬运规划失败",
            )
        if not primitives:
            return self._fault(ErrorCode.PLANNING_FAILED, "提升段无运动原语")
        command = self._cached_command("lift", primitives[0])
        self._primitive_calls.append(primitives[0].name)
        motion = self._do("motion", lambda: self.motion.execute_joint_move(command))
        if motion is _RUN:
            return _RUN
        failure = self._motion_failure(motion)
        if failure is not None:
            return failure
        return StepResult(state=PickPlaceState.TRANSPORT)

    def _h_transport(self) -> StepResult:
        primitives = self._primitives(
            "transport", lambda: self.strategy.make_transport_plan(self._target_pose, self.target_slot)
        )
        if primitives is _RUN:
            return _RUN
        if not primitives:
            return self._fault(ErrorCode.PLANNING_FAILED, "搬运段无运动原语")
        command = self._cached_command("transport", primitives[0])
        self._primitive_calls.append(primitives[0].name)
        motion = self._do("motion", lambda: self.motion.execute_joint_move(command))
        if motion is _RUN:
            return _RUN
        failure = self._motion_failure(motion)
        if failure is not None:
            return failure
        return StepResult(state=PickPlaceState.MOVE_PRE_SEAT)

    def _h_move_pre_seat(self) -> StepResult:
        primitives = self._primitives("seat", lambda: self.strategy.make_seat_plan(self.target_slot))
        if primitives is _RUN:
            return _RUN
        if not primitives:
            return self._fault(ErrorCode.PLANNING_FAILED, "落座段无运动原语")
        command = self._cached_command("pre_seat", primitives[0])
        self._primitive_calls.append(primitives[0].name)
        motion = self._do("motion", lambda: self.motion.execute_joint_move(command))
        if motion is _RUN:
            return _RUN
        failure = self._motion_failure(motion)
        if failure is not None:
            return failure
        return StepResult(state=PickPlaceState.SEAT)

    def _h_seat(self) -> StepResult:
        primitives = self._primitives(
            "seat_contact", lambda: self.strategy.make_seat_contact_plan(self.target_slot)
        )
        if primitives is _RUN:
            return _RUN
        if not primitives:
            return self._fault(ErrorCode.PLANNING_FAILED, "接触下降段无运动原语")
        command = self._cached_command("seat", primitives[0])
        self._primitive_calls.append(primitives[0].name)
        motion = self._do("motion", lambda: self.motion.execute_joint_move(command))
        if motion is _RUN:
            return _RUN
        failure = self._motion_failure(motion)
        if failure is not None:
            return failure
        return StepResult(state=PickPlaceState.VERIFY_SEATED)

    def _h_verify_seated(self) -> StepResult:
        """安全语义 4：未确认桌面承托 -> 400，禁止进入 UNLOAD/DISENGAGE。"""
        evidence = self._do("seat_verify", lambda: self.tool.seat_verify())
        if evidence is _RUN:
            return _RUN
        self._seat_evidence = evidence
        if not (evidence.ok and evidence.seated):
            return self._fault(
                evidence.error_code or ErrorCode.SEAT_NOT_VERIFIED,
                evidence.message
                or "未确认电池由桌面承载（seated=%s），禁止释放/解锁" % evidence.seated,
            )
        return StepResult(state=PickPlaceState.UNLOAD)

    def _h_unload(self) -> StepResult:
        primitives = self._primitives(
            "release", lambda: self.strategy.make_release_plan(self.target_slot)
        )
        if primitives is _RUN:
            return _RUN
        for prim in primitives:
            self._primitive_calls.append(prim.name)
        evidence = self._do("unload", lambda: self.tool.unload(primitives))
        if evidence is _RUN:
            return _RUN
        self._unload_evidence = evidence
        if not (evidence.ok and evidence.unloaded):
            code = evidence.error_code or ErrorCode.RELEASE_NOT_VERIFIED
            if code in (ErrorCode.RELEASE_NOT_VERIFIED, ErrorCode.UNLOCK_FAILED):
                return self._fault(code, evidence.message or "卸载（释放主要重量）未确认")
            return self._error(
                code,
                evidence.message or "卸载未确认",
                resumable=self._is_resumable_context(),
            )
        return StepResult(state=PickPlaceState.DISENGAGE)

    # ---- V1/V2 差异区（二）：DISENGAGE ------------------------------------

    def _h_disengage(self) -> StepResult:
        """安全语义 5：V2 解锁失败/结果未知 -> 410/420 -> FAULT，禁止硬拉上抬。"""
        # 防御性联锁：到达 DISENGAGE 时“已确认落座 + 已卸载”的证据必须仍然成立。
        # 该检查不依赖状态流转顺序，确保未来改动无法绕过设计 §7.2 的禁止行为。
        seated = bool(getattr(self, "_seat_evidence", None) and self._seat_evidence.seated)
        unloaded = bool(getattr(self, "_unload_evidence", None) and self._unload_evidence.unloaded)
        if self.strategy.requires_unlock_pulse():
            if not seated:
                return self._fault(
                    ErrorCode.SEAT_NOT_VERIFIED,
                    "DISENGAGE 前置联锁失败：未确认落座，禁止解锁/退出",
                )
            if not unloaded:
                return self._fault(
                    ErrorCode.RELEASE_NOT_VERIFIED,
                    "DISENGAGE 前置联锁失败：未确认卸载，禁止解锁/退出",
                )

        primitives = self._primitives(
            "disengage", lambda: self.strategy.make_disengage_plan(self.target_slot)
        )
        if primitives is _RUN:
            return _RUN
        for prim in primitives:
            self._primitive_calls.append(prim.name)

        if self.strategy.requires_unlock_pulse():
            decision = self.strategy.evaluate_unlock(
                seated_verified=seated, unloaded_verified=unloaded
            )
            if not decision.allowed:
                # 前置条件不满足：不得调用解锁 IO，直接保守故障
                return self._fault(
                    decision.error_code, "拒绝解锁：" + decision.reason
                )
            unlock = self._do("unlock", lambda: self.tool.unlock(decision.pulse_ms))
            if unlock is _RUN:
                return _RUN
            self._unlock_evidence = unlock
            if not unlock.ok:
                return self._fault(
                    unlock.error_code or ErrorCode.UNLOCK_FAILED,
                    unlock.message or "电磁解锁失败",
                )
            if not unlock.released:
                # 解锁脉冲被受理但锁止未解除：不得撤离（设计 §10.1）
                return self._fault(
                    ErrorCode.UNLOCK_FAILED,
                    unlock.message or "解锁请求已受理但锁止状态未解除",
                )

        evidence = self._do("disengage", lambda: self.tool.disengage(primitives))
        if evidence is _RUN:
            return _RUN
        self._disengage_evidence = evidence
        if not (evidence.ok and evidence.released):
            return self._fault(
                evidence.error_code or ErrorCode.RELEASE_NOT_VERIFIED,
                evidence.message or "机构分离（退钩/退出）结果未知，禁止撤离与上抬",
            )
        return StepResult(state=PickPlaceState.RETREAT)

    def _h_retreat(self) -> StepResult:
        primitives = self._primitives(
            "retreat", lambda: self.strategy.make_retreat_plan(self.target_slot)
        )
        if primitives is _RUN:
            return _RUN
        if not primitives:
            return self._fault(ErrorCode.PLANNING_FAILED, "撤离段无运动原语")
        command = self._cached_command("retreat", primitives[0])
        self._primitive_calls.append(primitives[0].name)
        motion = self._do("motion", lambda: self.motion.execute_joint_move(command))
        if motion is _RUN:
            return _RUN
        failure = self._motion_failure(motion)
        if failure is not None:
            return failure
        return StepResult(state=PickPlaceState.VERIFY_PLACED)

    def _h_verify_placed(self) -> StepResult:
        """安全语义 8：目标区域 + 连续稳定 placement_stability_sec 才 SUCCESS。"""
        if not self.config.verify_placement:
            return self._error(
                ErrorCode.PLACEMENT_FAILED, "placement 验证被配置关闭，不得声明放置成功"
            )
        detection = self._do(
            "placement_detection",
            lambda: self.perception.get_detection(
                self.object_id, None, self.config.perception_max_age_s
            ),
        )
        if detection is _RUN:
            return _RUN

        pose = detection.pose if detection is not None else None
        usable = bool(detection is not None and detection.found and not detection.stale and pose is not None)
        in_target = bool(usable and getattr(pose, "at_target", False))
        settled = bool(usable and getattr(pose, "settled", True))

        # 稳定性采样：每个 step 视为一次独立观察；
        # 只要持续满足“在目标区域 + 已静止 + 数据新鲜”，就累计稳定时间。
        # 一旦出现反例样本（数据过期/不在区域/仍振动），累计清零重新计时。
        if usable and in_target and settled:
            if self._stability_started_at is None:
                self._stability_started_at = self.clock.now()
            stable_for = self.clock.now() - self._stability_started_at
            self.stability_verified_for_s = stable_for  # 日志/回放字段（设计 §10.2）
            time_tolerance = min(1e-9, self.placement_stability_sec * 1e-9)
            if stable_for > 0.0 and stable_for + time_tolerance >= self.placement_stability_sec:
                self._set_placement_verified()
                return StepResult(state=PickPlaceState.SUCCESS)
            return self._placement_wait_or_fail(
                "放置后稳定性累计 %.2fs < %.2fs" % (stable_for, self.placement_stability_sec)
            )
        # 观察到不满足条件的样本：累计稳定性清零，继续观察
        self._stability_started_at = None
        return self._placement_wait_or_fail(
            "放置核验未通过：found=%s stale=%s in_target=%s settled=%s"
            % (
                bool(detection is not None and detection.found),
                bool(detection is not None and detection.stale),
                in_target,
                settled,
            )
        )

    def _placement_wait_or_fail(self, detail: str) -> StepResult:
        """放置核验窗口内继续观察；窗口耗尽仍未稳定 -> PLACEMENT_FAILED=430 -> FAILED。"""
        timeout = self.config.state_timeouts.get(PickPlaceState.VERIFY_PLACED.value, 30.0)
        if self.clock.now() - self._state_entered_at > timeout:
            return self._error(ErrorCode.PLACEMENT_FAILED, detail + "（核验窗口耗尽）")
        return _RUN

    def _placement_pose_changed(self, pose: Optional[PoseEstimate]) -> bool:
        """辅助：观察放置后位姿是否发生跳变（可用于日志/回放字段，不参与计时）。"""
        if pose is None:
            return False
        current = pose.ring_pose or pose.body_pose
        if current is None:
            return False
        changed = self._placement_last_pose is not None and self._placement_last_pose != current
        self._placement_last_pose = current
        return changed

    def _h_recovery(self) -> StepResult:
        """仅对“确认无载荷且运动状态已知”的可恢复错误重试。"""
        self._recovery_attempts += 1
        can_retry = (
            self._recovery_attempts <= self.config.recovery_max_attempts
            and self._resumable_error
            and self._is_resumable_context()
        )
        if not can_retry:
            return self._error(
                self.error_code,
                "恢复失败（attempt=%d/%d, resumable=%s）：%s"
                % (
                    self._recovery_attempts,
                    self.config.recovery_max_attempts,
                    self._resumable_error,
                    self.message,
                ),
                resumable=False,
            )
        self.logger.info("第 %d 次恢复重试", self._recovery_attempts)
        self._clear_port_results()
        return StepResult(state=PickPlaceState.RESOLVE_TARGET)

    def _h_success(self) -> StepResult:
        return StepResult(state=PickPlaceState.SUCCESS, error_code=self.error_code)

    def _h_failed(self) -> StepResult:
        return StepResult(state=PickPlaceState.FAILED, error_code=self.error_code)

    def _h_fault(self) -> StepResult:
        return StepResult(state=PickPlaceState.FAULT, error_code=self.error_code)

    def _h_cancelled(self) -> StepResult:
        return StepResult(state=PickPlaceState.CANCELLED, error_code=self.error_code)

    def _h_cancel_pending(self) -> StepResult:
        """安全语义 6：必须等 MotionPort 确认停稳；未确认一律 FAULT(520)。"""
        stop = self._do(
            "request_stop", lambda: self.motion.request_stop(self._cancel_reason or "cancel requested")
        )
        if stop is _RUN:
            return _RUN
        self._stop_confirmed = bool(stop.stop_confirmed)
        self.object_attached_estimated = self.object_attached_estimated  # 保留负载估计，不做自动解锁
        if not stop.stop_confirmed:
            return self._fault(
                ErrorCode.CANCEL_NOT_CONFIRMED,
                "取消后未确认停稳（request_delivered=%s）：%s"
                % (stop.request_delivered, stop.message),
                resend_allowed=False,
            )
        self.error_code = ErrorCode.OK
        self.message = "取消已确认停稳（request_delivered=%s）" % stop.request_delivered
        self._emit("stop_confirmed", self.state)
        return StepResult(state=PickPlaceState.CANCELLED)

    # ------------------------------------------------------------------
    # 内部辅助（续）
    # ------------------------------------------------------------------

    def _motion_failure(self, result: MotionResult) -> Optional[StepResult]:
        """把运动后端结果映射为 FSM 转移；返回 None 表示成功到位。"""
        if result.status_unknown:
            # 安全语义 7：运动状态未知 -> FAULT 且禁止自动重发
            self._motion_resend_allowed = False
            return self._fault(
                ErrorCode.MOTION_STATUS_UNKNOWN,
                result.message or "运动状态未知（反馈断流/结果不确定），禁止自动重发",
                resend_allowed=False,
            )
        if result.timed_out:
            self._motion_resend_allowed = False
            return self._fault(
                ErrorCode.MOTION_TIMEOUT,
                result.message or "运动指令超时，机器人可能仍在运动，禁止自动重发",
                resend_allowed=False,
            )
        if not self._motion_resend_allowed and not result.ok:
            return self._fault(
                result.error_code or ErrorCode.MOTION_STATUS_UNKNOWN,
                "运动结果不可用且状态未知，禁止重发：" + result.message,
                resend_allowed=False,
            )
        if result.ok:
            return None
        return self._error(
            result.error_code or ErrorCode.MOTION_TIMEOUT,
            result.message or "运动未到位",
            resumable=self._is_resumable_context(),
        )

    def _primitives(self, key: str, factory: Callable[[], Sequence[MotionPrimitive]]) -> Any:
        cached = self._port_results.get(("__primitives__", key))
        if cached is not None:
            return cached
        value = factory()
        if value is None:
            return _RUN
        value = tuple(value)
        self._port_results[("__primitives__", key)] = value
        return value


#: 正常流水线（用于 progress 计算）
_PIPELINE_STATES: Tuple[PickPlaceState, ...] = (
    PickPlaceState.RESOLVE_TARGET,
    PickPlaceState.PLAN_ACQUIRE,
    PickPlaceState.MOVE_PRE_ALIGN,
    PickPlaceState.ENGAGE,
    PickPlaceState.VERIFY_ENGAGEMENT,
    PickPlaceState.TEST_LIFT,
    PickPlaceState.VERIFY_ATTACHED,
    PickPlaceState.LIFT,
    PickPlaceState.TRANSPORT,
    PickPlaceState.MOVE_PRE_SEAT,
    PickPlaceState.SEAT,
    PickPlaceState.VERIFY_SEATED,
    PickPlaceState.UNLOAD,
    PickPlaceState.DISENGAGE,
    PickPlaceState.RETREAT,
    PickPlaceState.VERIFY_PLACED,
    PickPlaceState.SUCCESS,
)


def _bind(name: str) -> Callable[[PickPlaceFSM], Any]:
    def _call(fsm: PickPlaceFSM) -> Any:
        return getattr(fsm, name)()

    return _call


#: 集中转移表：状态 -> 处理器（设计 §3.2 的确定性实现）
_STATE_ACTIONS: Dict[PickPlaceState, Callable[[PickPlaceFSM], Any]] = {
    PickPlaceState.RESOLVE_TARGET: _bind("_h_resolve_target"),
    PickPlaceState.PLAN_ACQUIRE: _bind("_h_plan_acquire"),
    PickPlaceState.MOVE_PRE_ALIGN: _bind("_h_move_pre_align"),
    PickPlaceState.ENGAGE: _bind("_h_engage"),
    PickPlaceState.VERIFY_ENGAGEMENT: _bind("_h_verify_engagement"),
    PickPlaceState.TEST_LIFT: _bind("_h_test_lift"),
    PickPlaceState.VERIFY_ATTACHED: _bind("_h_verify_attached"),
    PickPlaceState.LIFT: _bind("_h_lift"),
    PickPlaceState.TRANSPORT: _bind("_h_transport"),
    PickPlaceState.MOVE_PRE_SEAT: _bind("_h_move_pre_seat"),
    PickPlaceState.SEAT: _bind("_h_seat"),
    PickPlaceState.VERIFY_SEATED: _bind("_h_verify_seated"),
    PickPlaceState.UNLOAD: _bind("_h_unload"),
    PickPlaceState.DISENGAGE: _bind("_h_disengage"),
    PickPlaceState.RETREAT: _bind("_h_retreat"),
    PickPlaceState.VERIFY_PLACED: _bind("_h_verify_placed"),
    PickPlaceState.RECOVERY: _bind("_h_recovery"),
    PickPlaceState.CANCEL_PENDING: _bind("_h_cancel_pending"),
    PickPlaceState.SUCCESS: _bind("_h_success"),
    PickPlaceState.FAILED: _bind("_h_failed"),
    PickPlaceState.FAULT: _bind("_h_fault"),
    PickPlaceState.CANCELLED: _bind("_h_cancelled"),
}
