"""整轮比赛任务状态机（Task FSM，第一层）——纯 Python 核心。

设计依据
--------
``docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md``：

* §3.1 第一层 Task FSM 状态表与状态图
* §4.3 ``ExecuteTask.action`` 校验规则
* §7.1 错误码表 / §7.2 关键联锁
* §9 蓝—红—黄时序示例

本模块 **不导入 rclpy**，也不做任何真实 IO：所有外部副作用（调用 PickPlace、
取消子任务、发布状态、请求停止）都只以 :class:`ActionRequest` 形式返回给调用方。
ROS 2 外壳见 :mod:`mtc_task.task_executor`，可注入 fake 组件。

结构约定
--------
* 状态转移集中在 :data:`TRANSITIONS`：``(state, event) -> (next_state, guard, fallback)``；
* 进入状态时的数据更新与动作产出集中在 :data:`ENTRY_ACTIONS`：
  ``state -> handler_name``，处理函数命名以 ``_enter_`` 开头；
* 超时不散落 if：每个状态经 ``_guard_state_timed_out`` 校验后以 ``timeout`` 事件重新查表，
  "状态超时去哪个状态"同样由转移表描述。
"""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from .task_validation import (
    ALLOWED_SLOTS,
    BASIC_SLOT,
    MODE_BASIC,
    MODE_SEQUENCE,
    SEQUENCE_SLOTS,
    TASK_MODE_NAMES,
    expected_slot_for_index,
    normalize_task_request,
    validate_task_request,
)

__all__ = [
    "TaskStateName",
    "Event",
    "ActionKind",
    "ActionRequest",
    "TaskRequest",
    "TaskStateSnapshot",
    "TaskOutcome",
    "StepResult",
    "FsmsConfig",
    "ErrorCodes",
    "Transition",
    "TRANSITIONS",
    "ENTRY_ACTIONS",
    "TaskFsm",
    "MODE_BASIC",
    "MODE_SEQUENCE",
    "ALLOWED_SLOTS",
]


# ---------------------------------------------------------------------------
# 错误码：与 src/mtc_interfaces/msg/ErrorCodes.msg 逐项对齐
# ---------------------------------------------------------------------------


class ErrorCodes:
    """``mtc_interfaces/msg/ErrorCodes.msg`` 的纯 Python 镜像（不依赖 ROS）。"""

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

    NAMES: Dict[int, str] = {
        0: "OK",
        100: "INVALID_TASK",
        110: "TARGET_NOT_FOUND",
        120: "POSE_STALE",
        200: "PLANNING_FAILED",
        210: "EXECUTION_REJECTED",
        220: "MOTION_TIMEOUT",
        230: "MOTION_STATUS_UNKNOWN",
        300: "ENGAGE_FAILED",
        310: "ATTACH_NOT_VERIFIED",
        320: "OBJECT_DROPPED",
        400: "SEAT_NOT_VERIFIED",
        410: "UNLOCK_FAILED",
        420: "RELEASE_NOT_VERIFIED",
        430: "PLACEMENT_FAILED",
        500: "SAFETY_INTERLOCK",
        510: "HARDWARE_FAULT",
        520: "CANCEL_NOT_CONFIRMED",
    }

    @classmethod
    def name_of(cls, code: Any) -> str:
        try:
            return cls.NAMES.get(int(code), "UNKNOWN_%d" % int(code))
        except (TypeError, ValueError):
            return "UNKNOWN"


# ---------------------------------------------------------------------------
# 状态与事件
# ---------------------------------------------------------------------------


class TaskStateName(enum.Enum):
    """§3.1 定义的 13 个 Task FSM 状态名。"""

    INIT = "INIT"
    SELF_CHECK = "SELF_CHECK"
    WAIT_TASK = "WAIT_TASK"
    VALIDATE_TASK = "VALIDATE_TASK"
    SCAN_SCENE = "SCAN_SCENE"
    EXECUTE_ITEM = "EXECUTE_ITEM"
    RECORD_RESULT = "RECORD_RESULT"
    RECOVERY = "RECOVERY"
    CANCEL_PENDING = "CANCEL_PENDING"
    FINISHED = "FINISHED"
    TASK_FAILED = "TASK_FAILED"
    FAULT = "FAULT"
    CANCELLED = "CANCELLED"


ALL_STATES: Tuple[TaskStateName, ...] = tuple(TaskStateName)

#: 处于这些状态时视为"有进行中任务"（§7.2 单 Goal 并发拒绝判定依据）
ACTIVE_TASK_STATES: Tuple[TaskStateName, ...] = (
    TaskStateName.VALIDATE_TASK,
    TaskStateName.SCAN_SCENE,
    TaskStateName.EXECUTE_ITEM,
    TaskStateName.RECORD_RESULT,
    TaskStateName.RECOVERY,
    TaskStateName.CANCEL_PENDING,
)

#: 终态：任务已有结论，只等 reset 事件回到 WAIT_TASK
TERMINAL_STATES: Tuple[TaskStateName, ...] = (
    TaskStateName.FINISHED,
    TaskStateName.TASK_FAILED,
    TaskStateName.CANCELLED,
)

#: 仅这些错误允许自动重试：均为"明确安全可重试"且不涉及在途运动不确定性。
#: * 110/120/200 纯感知与纯规划失败；
#: * 430 PLACEMENT_FAILED 表示未落到判定区或稳定性不足，电池已由台面承托、
#:   工具已分离（否则会报 400/410/420），因此允许限次重试；
#: * 210 EXECUTION_REJECTED 表示目标不合法/后端不支持，未产生运动，可重新规划一次。
DEFAULT_RECOVERABLE_ERRORS: Tuple[int, ...] = (
    ErrorCodes.TARGET_NOT_FOUND,
    ErrorCodes.POSE_STALE,
    ErrorCodes.PLANNING_FAILED,
    ErrorCodes.PLACEMENT_FAILED,
    ErrorCodes.EXECUTION_REJECTED,
)

#: 明确禁止自动重试的错误（保留常量供审计与外壳判断）
NON_RECOVERABLE_ERRORS: Tuple[int, ...] = (
    ErrorCodes.MOTION_STATUS_UNKNOWN,
    ErrorCodes.MOTION_TIMEOUT,
    ErrorCodes.SAFETY_INTERLOCK,
    ErrorCodes.HARDWARE_FAULT,
    ErrorCodes.UNLOCK_FAILED,
    ErrorCodes.RELEASE_NOT_VERIFIED,
    ErrorCodes.OBJECT_DROPPED,
    ErrorCodes.CANCEL_NOT_CONFIRMED,
)

#: 每状态超时（秒）。``None`` = 不设超时（等待态 / 终态 / 主动取消中）。
DEFAULT_STATE_TIMEOUTS: Dict[TaskStateName, Optional[float]] = {
    TaskStateName.INIT: 5.0,
    TaskStateName.SELF_CHECK: 10.0,
    TaskStateName.WAIT_TASK: None,
    TaskStateName.VALIDATE_TASK: 2.0,
    TaskStateName.SCAN_SCENE: 5.0,
    TaskStateName.EXECUTE_ITEM: 60.0,
    TaskStateName.RECORD_RESULT: 2.0,
    TaskStateName.RECOVERY: 10.0,
    TaskStateName.CANCEL_PENDING: 5.0,
    TaskStateName.FINISHED: None,
    TaskStateName.TASK_FAILED: None,
    TaskStateName.FAULT: None,
    TaskStateName.CANCELLED: None,
}


class Event:
    """事件名常量。"""

    SELF_CHECK_OK = "self_check_ok"
    SELF_CHECK_FAIL = "self_check_fail"

    GOAL_ACCEPTED = "goal_accepted"
    GOAL_REJECTED = "goal_rejected"

    VALIDATED = "task_validated"
    VALIDATION_FAILED = "task_validation_failed"

    TARGET_FOUND = "target_found"
    TARGET_NOT_FOUND = "target_not_found"
    SCAN_FAILED = "scan_failed"

    ITEM_SUCCEEDED = "item_succeeded"
    ITEM_FAILED = "item_failed"
    ITEM_UNSAFE = "item_unsafe"

    RETRY_APPROVED = "retry_approved"
    RETRY_DENIED = "retry_denied"

    RECORD_OK = "record_ok"
    RECORD_FAILED = "record_failed"

    CANCEL_REQUESTED = "cancel_requested"
    CHILD_STOPPED_CONFIRMED = "child_stopped_confirmed"

    RESET = "reset"
    TIMEOUT = "timeout"


# ---------------------------------------------------------------------------
# 动作请求（对外的副作用意图，不含真实 IO）
# ---------------------------------------------------------------------------


class ActionKind(enum.Enum):
    CALL_PICK_PLACE = "call_pick_place"
    CANCEL_CHILD = "cancel_child"
    REQUEST_STOP = "request_stop"
    PUBLISH_STATE = "publish_state"
    LOCK_ACTIONS = "lock_actions"


@dataclass(frozen=True)
class ActionRequest:
    """FSM 产出的动作意图；由 ROS 外壳翻译成真实调用。"""

    kind: ActionKind
    payload: Dict[str, Any] = field(default_factory=dict)


def _act(kind: ActionKind, **payload: Any) -> ActionRequest:
    return ActionRequest(kind=kind, payload=payload)


# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------


@dataclass
class TaskRequest:
    """已规范化的任务请求（对应 ``ExecuteTask.action`` 的 Goal 字段）。"""

    task_id: str
    mode: int
    ordered_colors: List[int]
    timeout_ms: int
    target_slot: Optional[str] = None
    placement_stability_sec: float = 3.0

    @classmethod
    def from_goal(cls, goal: Any, **overrides: Any) -> "TaskRequest":
        """从类 Goal 对象（ROS 消息 / dict / SimpleNamespace）构造。

        只读取 ExecuteTask.action 中真实存在的字段，不臆造字段名。
        """
        data = normalize_task_request(goal)
        allowed = set(cls.__dataclass_fields__)  # type: ignore[attr-defined]
        unknown = set(overrides) - allowed
        if unknown:
            raise ValueError("unknown TaskRequest override(s): %s" % sorted(unknown))
        data.update(overrides)
        if data.get("target_slot") is None:
            data.pop("target_slot", None)
        return cls(**data)

    @property
    def mode_name(self) -> str:
        return TASK_MODE_NAMES.get(int(self.mode), "UNKNOWN_MODE_%s" % self.mode)

    @property
    def total_count(self) -> int:
        return len(self.ordered_colors)

    def slot_for_index(self, index: int) -> Optional[str]:
        return expected_slot_for_index(int(self.mode), index, self.target_slot)

    def slots(self) -> List[str]:
        return [self.slot_for_index(i) or "" for i in range(self.total_count)]


@dataclass
class TaskStateSnapshot:
    """``mtc_interfaces/msg/TaskState.msg`` 的内容（不含 Header）。"""

    task_id: str
    state: str
    substate: str
    completed_count: int
    total_count: int
    last_error_code: int
    detail: str


@dataclass
class TaskOutcome:
    """``ExecuteTask.action`` 的 Result 内容。"""

    success: bool
    completed_count: int
    completed_slots: List[str]
    error_code: int
    message: str


@dataclass
class StepResult:
    """``step()`` 返回值：新状态 + 需要调用的方执行的动作列表。"""

    state: TaskStateName
    actions: List[ActionRequest] = field(default_factory=list)
    reason: str = ""
    goal_rejected: bool = False

    @property
    def state_name(self) -> str:
        return self.state.value

    def action_kinds(self) -> List[str]:
        return [a.kind.value for a in self.actions]

    def find(self, kind: ActionKind) -> Optional[ActionRequest]:
        for action in self.actions:
            if action.kind is kind:
                return action
        return None

    def has(self, kind: ActionKind) -> bool:
        return self.find(kind) is not None


@dataclass
class FsmsConfig:
    """可注入配置：每状态超时 + 恢复策略。"""

    recovery_max_attempts: int = 1
    default_timeout_s: float = 10.0
    state_timeouts: Dict[TaskStateName, Optional[float]] = field(
        default_factory=lambda: dict(DEFAULT_STATE_TIMEOUTS)
    )
    recoverable_errors: Tuple[int, ...] = DEFAULT_RECOVERABLE_ERRORS

    def timeout_for(self, state: TaskStateName) -> Optional[float]:
        if state in self.state_timeouts:
            return self.state_timeouts[state]
        return self.default_timeout_s

    def is_recoverable(self, error_code: Any) -> bool:
        """只有显式列入 ``recoverable_errors`` 的错误才允许自动重试。

        采用白名单而非黑名单：未知错误码默认不可重试（保守）。
        """
        try:
            code = int(error_code)
        except (TypeError, ValueError):
            return False
        return code in tuple(int(c) for c in self.recoverable_errors)


# ---------------------------------------------------------------------------
# 集中的转移表
#
#   (state, event) -> (next_state, guard_name, fallback_state)
#   guard_name 以 "_guard_" 开头，None 表示无条件转移；
#   fallback_state 仅在守卫拒绝时使用（用于"必须离开当前状态"的分支，如不可重试错误 → FAULT）。
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Transition:
    """一条状态转移。

    * ``guard``      为 None 表示无条件转移；
    * ``fallback``   守卫拒绝且**必须离开当前状态**时使用（如不可重试错误 → FAULT）；
    * ``else_state`` 守卫拒绝且仍要转移、但去向不同时使用（如"还有任务项"二选一）。
      这样可避免在转移表中出现重复的 ``(state, event)`` 键。
    """

    state: TaskStateName
    event: str
    next_state: TaskStateName
    guard: Optional[str] = None
    fallback: Optional[TaskStateName] = None
    else_state: Optional[TaskStateName] = None
    note: str = ""


def _t(
    state: TaskStateName,
    event: str,
    next_state: TaskStateName,
    guard: Optional[str] = None,
    fallback: Optional[TaskStateName] = None,
    else_state: Optional[TaskStateName] = None,
    note: str = "",
) -> Transition:
    return Transition(state, event, next_state, guard, fallback, else_state, note)


TRANSITIONS: Tuple[Transition, ...] = (
    # --- 启动与自检 -------------------------------------------------------
    _t(TaskStateName.INIT, Event.SELF_CHECK_OK, TaskStateName.SELF_CHECK, note="参数/依赖就绪"),
    _t(TaskStateName.INIT, Event.SELF_CHECK_FAIL, TaskStateName.FAULT, note="初始化失败"),
    _t(TaskStateName.SELF_CHECK, Event.SELF_CHECK_OK, TaskStateName.WAIT_TASK, note="设备正常/通信就绪"),
    _t(TaskStateName.SELF_CHECK, Event.SELF_CHECK_FAIL, TaskStateName.FAULT, note="自检失败"),
    # --- 任务受理（§7.2：同一时刻只允许一个进行中 Goal）------------------
    _t(TaskStateName.WAIT_TASK, Event.GOAL_ACCEPTED, TaskStateName.VALIDATE_TASK, "_guard_goal_admissible"),
    # 以下显式登记"任务进行中再次收到 Goal"：必须明确拒绝，且不得排队。
    # 若缺少这些条目，step() 会把它当作"无转移事件"静默忽略，等于没有拒绝。
    _t(TaskStateName.VALIDATE_TASK, Event.GOAL_ACCEPTED, TaskStateName.VALIDATE_TASK, "_guard_goal_denied"),
    _t(TaskStateName.SCAN_SCENE, Event.GOAL_ACCEPTED, TaskStateName.SCAN_SCENE, "_guard_goal_denied"),
    _t(TaskStateName.EXECUTE_ITEM, Event.GOAL_ACCEPTED, TaskStateName.EXECUTE_ITEM, "_guard_goal_denied"),
    _t(TaskStateName.RECORD_RESULT, Event.GOAL_ACCEPTED, TaskStateName.RECORD_RESULT, "_guard_goal_denied"),
    _t(TaskStateName.RECOVERY, Event.GOAL_ACCEPTED, TaskStateName.RECOVERY, "_guard_goal_denied"),
    _t(TaskStateName.CANCEL_PENDING, Event.GOAL_ACCEPTED, TaskStateName.CANCEL_PENDING, "_guard_goal_denied"),
    # --- 任务校验（§4.3）--------------------------------------------------
    _t(TaskStateName.VALIDATE_TASK, Event.VALIDATED, TaskStateName.SCAN_SCENE),
    _t(TaskStateName.VALIDATE_TASK, Event.VALIDATION_FAILED, TaskStateName.TASK_FAILED, note="非法任务不得猜测补全"),
    _t(TaskStateName.VALIDATE_TASK, Event.GOAL_REJECTED, TaskStateName.TASK_FAILED, note="受理后被撤回"),
    # --- 场景扫描（§3.1）--------------------------------------------------
    _t(TaskStateName.SCAN_SCENE, Event.TARGET_FOUND, TaskStateName.EXECUTE_ITEM, "_guard_target_usable"),
    _t(TaskStateName.SCAN_SCENE, Event.TARGET_NOT_FOUND, TaskStateName.RECOVERY, note="重新感知；限次"),
    _t(TaskStateName.SCAN_SCENE, Event.SCAN_FAILED, TaskStateName.TASK_FAILED, note="重试预算已耗尽"),
    _t(TaskStateName.SCAN_SCENE, Event.CANCEL_REQUESTED, TaskStateName.CANCEL_PENDING),
    # --- 单项执行（§3.1 / §7.2）------------------------------------------
    _t(TaskStateName.EXECUTE_ITEM, Event.ITEM_SUCCEEDED, TaskStateName.RECORD_RESULT),
    _t(
        TaskStateName.EXECUTE_ITEM,
        Event.ITEM_FAILED,
        TaskStateName.RECOVERY,
        "_guard_error_recoverable",
        fallback=TaskStateName.FAULT,
        note="不可重试（如 230）→ FAULT",
    ),
    _t(TaskStateName.EXECUTE_ITEM, Event.ITEM_UNSAFE, TaskStateName.FAULT, note="危险/不确定状态"),
    _t(TaskStateName.EXECUTE_ITEM, Event.CANCEL_REQUESTED, TaskStateName.CANCEL_PENDING),
    # --- 记录结果（不依赖发令返回码）--------------------------------------
    # 单条转移表达二选一：还有项 → SCAN_SCENE；全部完成 → FINISHED。
    _t(
        TaskStateName.RECORD_RESULT,
        Event.RECORD_OK,
        TaskStateName.FINISHED,
        "_guard_all_items_done",
        else_state=TaskStateName.SCAN_SCENE,
        note="all_items_done ? FINISHED : SCAN_SCENE",
    ),
    _t(TaskStateName.RECORD_RESULT, Event.RECORD_FAILED, TaskStateName.TASK_FAILED),
    _t(TaskStateName.RECORD_RESULT, Event.CANCEL_REQUESTED, TaskStateName.CANCEL_PENDING),
    # --- 恢复（仅安全可重试错误）------------------------------------------
    # 预算不足时经 fallback 直接 TASK_FAILED，避免"守卫拒绝→原地不动"导致重试死循环。
    _t(
        TaskStateName.RECOVERY,
        Event.RETRY_APPROVED,
        TaskStateName.SCAN_SCENE,
        "_guard_retry_budget_available",
        fallback=TaskStateName.TASK_FAILED,
        note="预算耗尽 → TASK_FAILED",
    ),
    _t(TaskStateName.RECOVERY, Event.RETRY_DENIED, TaskStateName.TASK_FAILED, note="重试耗尽/不可重试"),
    _t(TaskStateName.RECOVERY, Event.CANCEL_REQUESTED, TaskStateName.CANCEL_PENDING),
    # --- 取消（§4.3：不可无等待地返回 CANCELED）--------------------------
    _t(TaskStateName.CANCEL_PENDING, Event.CHILD_STOPPED_CONFIRMED, TaskStateName.CANCELLED, note="停稳+负载已确认"),
    _t(TaskStateName.CANCEL_PENDING, Event.TIMEOUT, TaskStateName.FAULT, "_guard_cancel_timeout", note="520"),
    _t(TaskStateName.CANCEL_PENDING, Event.ITEM_UNSAFE, TaskStateName.FAULT),
    # --- 终态复位 ---------------------------------------------------------
    _t(TaskStateName.FINISHED, Event.RESET, TaskStateName.WAIT_TASK),
    _t(TaskStateName.TASK_FAILED, Event.RESET, TaskStateName.WAIT_TASK, note="复位并确认可运行"),
    _t(TaskStateName.CANCELLED, Event.RESET, TaskStateName.WAIT_TASK, note="复位并确认可运行"),
    # --- 每状态超时去向（守卫仅在真的超时后放行）--------------------------
    _t(TaskStateName.INIT, Event.TIMEOUT, TaskStateName.FAULT, "_guard_state_timed_out"),
    _t(TaskStateName.SELF_CHECK, Event.TIMEOUT, TaskStateName.FAULT, "_guard_state_timed_out"),
    _t(TaskStateName.VALIDATE_TASK, Event.TIMEOUT, TaskStateName.TASK_FAILED, "_guard_state_timed_out"),
    _t(TaskStateName.SCAN_SCENE, Event.TIMEOUT, TaskStateName.RECOVERY, "_guard_state_timed_out", note="扫描超时→限次"),
    _t(
        TaskStateName.EXECUTE_ITEM,
        Event.TIMEOUT,
        TaskStateName.FAULT,
        "_guard_state_timed_out",
        note="220：运动状态不确定，不自动重发",
    ),
    _t(TaskStateName.RECORD_RESULT, Event.TIMEOUT, TaskStateName.TASK_FAILED, "_guard_state_timed_out"),
    _t(TaskStateName.RECOVERY, Event.TIMEOUT, TaskStateName.TASK_FAILED, "_guard_state_timed_out"),
)

#: 便于查询与自检：(state, event) -> Transition
TRANSITION_INDEX: Dict[Tuple[TaskStateName, str], Transition] = {
    (tr.state, tr.event): tr for tr in TRANSITIONS
}


# ---------------------------------------------------------------------------
# 进入状态时的处理（集中表：state -> handler 名，handler 以 "_enter_" 开头）
# ---------------------------------------------------------------------------

ENTRY_ACTIONS: Dict[TaskStateName, Optional[str]] = {
    TaskStateName.INIT: None,
    TaskStateName.SELF_CHECK: "_enter_self_check",
    TaskStateName.WAIT_TASK: "_enter_wait_task",
    TaskStateName.VALIDATE_TASK: "_enter_validate_task",
    TaskStateName.SCAN_SCENE: "_enter_scan_scene",
    TaskStateName.EXECUTE_ITEM: "_enter_execute_item",
    TaskStateName.RECORD_RESULT: None,
    TaskStateName.RECOVERY: "_enter_recovery",
    TaskStateName.CANCEL_PENDING: "_enter_cancel_pending",
    TaskStateName.FINISHED: "_enter_terminal_ok",
    TaskStateName.TASK_FAILED: "_enter_task_failed",
    TaskStateName.FAULT: "_enter_fault",
    TaskStateName.CANCELLED: "_enter_cancelled",
}


def _self_check_tables() -> None:
    """导入期自检：保证两张表与状态枚举保持一致（避免静默漏配）。"""
    missing_entries = set(ALL_STATES) - set(ENTRY_ACTIONS)
    if missing_entries:
        raise AssertionError("ENTRY_ACTIONS missing states: %s" % sorted(s.value for s in missing_entries))
    for state in ALL_STATES:
        if state not in DEFAULT_STATE_TIMEOUTS:
            raise AssertionError("DEFAULT_STATE_TIMEOUTS missing state: %s" % state.value)
    for transition in TRANSITIONS:
        if transition.guard is not None and not transition.guard.startswith("_guard_"):
            raise AssertionError("guard naming convention violated: %s" % transition.guard)
        if transition.guard is None and (transition.fallback is not None or transition.else_state is not None):
            raise AssertionError(
                "fallback/else_state requires a guard: %s/%s" % (transition.state.value, transition.event)
            )
    for handler in ENTRY_ACTIONS.values():
        if handler is not None and not handler.startswith("_enter_"):
            raise AssertionError("entry handler naming convention violated: %s" % handler)
    duplicated = len(TRANSITIONS) - len(TRANSITION_INDEX)
    if duplicated:
        raise AssertionError("TRANSITIONS contains %d duplicated (state, event) key(s)" % duplicated)


_self_check_tables()


# ---------------------------------------------------------------------------
# FSM 主体
# ---------------------------------------------------------------------------


class TaskFsm:
    """整轮比赛任务状态机（第一层）。

    典型用法（ROS 外壳）::

        fsm = TaskFsm(clock=lambda: node.get_clock().now().nanoseconds / 1e9)
        result = fsm.step("goal_accepted", {"goal": goal})
        for action in result.actions:
            dispatch(action)

    离线测试可注入 ``clock`` / ``config``，完全不接触 ROS。
    """

    def __init__(
        self,
        clock: Optional[Callable[[], float]] = None,
        config: Optional[FsmsConfig] = None,
        transitions: Sequence[Transition] = TRANSITIONS,
        initial_state: TaskStateName = TaskStateName.INIT,
    ) -> None:
        self._clock: Callable[[], float] = clock or time.monotonic
        self.config = config or FsmsConfig()
        self._transitions: Dict[Tuple[TaskStateName, str], Transition] = {
            (tr.state, tr.event): tr for tr in transitions
        }

        self.state: TaskStateName = initial_state
        self.previous_state: Optional[TaskStateName] = None
        self.state_entered_at: float = self._clock()

        self.current_task: Optional[TaskRequest] = None
        self.current_index: int = 0
        self.current_slot: str = ""
        self.current_object_id: str = ""
        self.substate: str = ""
        self.completed_slots: List[str] = []
        self.completed_count: int = 0
        self.recovery_attempts: int = 0
        self.last_error_code: int = ErrorCodes.OK
        self.last_error_message: str = ""
        self.last_validation_error: str = ""
        self.last_rejection: Dict[str, Any] = {}
        self.last_outcome: Optional[TaskOutcome] = None
        self.task_started_at: Optional[float] = None
        self.transition_log: List[Tuple[str, str, str, str]] = []
        self.ignored_events: List[Tuple[str, str]] = []

    # -- 只读视图 ---------------------------------------------------------

    @property
    def state_name(self) -> str:
        return self.state.value

    @property
    def has_active_task(self) -> bool:
        """是否已有进行中的 Goal（§7.2 并发拒绝判定依据）。"""
        return self.state in ACTIVE_TASK_STATES

    @property
    def total_count(self) -> int:
        return self.current_task.total_count if self.current_task else 0

    def elapsed_in_state(self) -> float:
        """自进入当前状态以来的停留时间（秒）。"""
        return self._clock() - self.state_entered_at

    def snapshot(self, detail: str = "") -> TaskStateSnapshot:
        return TaskStateSnapshot(
            task_id=self.current_task.task_id if self.current_task else "",
            state=self.state.value,
            substate=self.substate,
            completed_count=self.completed_count,
            total_count=self.total_count,
            last_error_code=self.last_error_code,
            detail=detail or self.last_error_message,
        )

    def build_outcome(self) -> TaskOutcome:
        """按当前状态生成 ExecuteTask.Result 内容。"""
        if self.state is TaskStateName.FINISHED:
            return TaskOutcome(
                success=True,
                completed_count=self.completed_count,
                completed_slots=list(self.completed_slots),
                error_code=ErrorCodes.OK,
                message="task finished: %s" % ",".join(self.completed_slots),
            )
        if self.state is TaskStateName.CANCELLED:
            return TaskOutcome(
                success=False,
                completed_count=self.completed_count,
                completed_slots=list(self.completed_slots),
                error_code=ErrorCodes.OK,
                message="cancelled after stop confirmation (completed %d)" % self.completed_count,
            )
        return TaskOutcome(
            success=False,
            completed_count=self.completed_count,
            completed_slots=list(self.completed_slots),
            error_code=self.last_error_code,
            message=self.last_error_message
            or "%s (%s)" % (self.state.value, ErrorCodes.name_of(self.last_error_code)),
        )

    # -- 超时检查（外壳周期调用，或测试显式调用）--------------------------

    def check_timeouts(self) -> Optional[StepResult]:
        """当前状态已超时则返回一次 ``timeout`` 驱动结果；否则返回 None。

        计时基准是**进入该状态的时刻**：超时表示"在该状态停留过久"。
        本方法不做任何心跳刷新，否则周期性调用会无限推迟超时。
        """
        if self._state_timeout_expired():
            return self.step(Event.TIMEOUT, {"source": "watchdog"})
        return None

    def elapsed_task_time(self) -> Optional[float]:
        """当前任务已耗时（秒）；无进行中任务时返回 None。"""
        if self.task_started_at is None:
            return None
        return self._clock() - self.task_started_at

    def check_task_deadline(self) -> Optional[StepResult]:
        """任务级截止时间（``timeout_ms``）看门狗。

        超时说明可能仍有在途运动，按 §7.1 处置进入 FAULT（220），不自动重发。
        """
        if self.current_task is None or self.state not in ACTIVE_TASK_STATES:
            return None
        elapsed = self.elapsed_task_time()
        if elapsed is None:
            return None
        limit_s = float(self.current_task.timeout_ms) / 1000.0
        if elapsed < limit_s:
            return None
        self.last_error_code = ErrorCodes.MOTION_TIMEOUT
        self.last_error_message = "task %s exceeded timeout_ms=%d (elapsed %.3fs)" % (
            self.current_task.task_id,
            self.current_task.timeout_ms,
            elapsed,
        )
        return self._enter(TaskStateName.FAULT, "task_deadline_exceeded", {})

    # -- 核心入口 ---------------------------------------------------------

    def step(self, event: Any, payload: Optional[Dict[str, Any]] = None) -> StepResult:
        """推进状态机，返回新状态与动作请求列表。"""
        event_name = event.value if isinstance(event, enum.Enum) else str(event)
        payload = dict(payload or {})

        transition = self._transitions.get((self.state, event_name))
        if transition is None:
            self.ignored_events.append((self.state.value, event_name))
            return StepResult(
                state=self.state,
                actions=[],
                reason="no transition for (%s, %s)" % (self.state.value, event_name),
            )

        if transition.guard is not None:
            guard = getattr(self, transition.guard)
            if not guard(event_name, payload):
                if transition.guard in ("_guard_goal_admissible", "_guard_goal_denied"):
                    # §7.2 并发拒绝；两种情况都保留守卫写入的具体原因
                    if self.has_active_task:
                        self._register_second_goal_rejection(payload)
                    return StepResult(
                        state=self.state,
                        actions=[],
                        reason=self.last_rejection.get("message", "goal rejected by guard"),
                        goal_rejected=True,
                    )
                if transition.fallback is not None:
                    return self._enter(
                        transition.fallback,
                        event_name,
                        payload,
                        note="guard %s denied -> fallback" % transition.guard,
                    )
                if transition.else_state is not None:
                    return self._enter(
                        transition.else_state,
                        event_name,
                        payload,
                        note="guard %s denied -> else_state" % transition.guard,
                    )
                self.ignored_events.append((self.state.value, event_name))
                return StepResult(
                    state=self.state,
                    actions=[],
                    reason="guard %s rejected %s" % (transition.guard, event_name),
                )
            # 转移被批准：状态可能变化不大（自定向下游决策），在这里重置计时基准，
            # 保证后续超时判定从"本次有效进展"开始，而不是上一次进入状态的时刻。
            self._touch()

        return self._enter(transition.next_state, event_name, payload)

    # -- 状态进入与动作产出 ------------------------------------------------

    def _enter(
        self,
        next_state: TaskStateName,
        event_name: str,
        payload: Dict[str, Any],
        note: str = "",
    ) -> StepResult:
        previous = self.state
        self.previous_state = previous
        self.state = next_state
        self._touch()
        self.transition_log.append((previous.value, event_name, next_state.value, note))

        handler_name = ENTRY_ACTIONS.get(next_state)
        actions: List[ActionRequest] = []
        if handler_name is not None:
            actions.extend(getattr(self, handler_name)(payload))

        actions.append(_act(ActionKind.PUBLISH_STATE, snapshot=self.snapshot()))
        return StepResult(state=self.state, actions=actions, reason=event_name)

    # -- 进入处理函数（全部以 _enter_ 开头，供 ENTRY_ACTIONS 引用）--------

    def _enter_self_check(self, payload: Dict[str, Any]) -> List[ActionRequest]:
        self.substate = "SELF_CHECK"
        return []

    def _enter_wait_task(self, payload: Dict[str, Any]) -> List[ActionRequest]:
        """终态复位：清空上一轮任务上下文，恢复可受理状态。"""
        self.substate = "WAIT_TASK"
        self.current_task = None
        self.current_index = 0
        self.current_slot = ""
        self.current_object_id = ""
        self.completed_slots = []
        self.completed_count = 0
        self.recovery_attempts = 0
        self.last_error_code = ErrorCodes.OK
        self.last_error_message = ""
        self.last_validation_error = ""
        self.last_outcome = None
        self.task_started_at = None
        return []

    def _enter_validate_task(self, payload: Dict[str, Any]) -> List[ActionRequest]:
        """受理 Goal 后立刻做全量校验（§4.3），并把结论翻译成事件。"""
        self.substate = "VALIDATE_TASK"
        result = validate_task_request(self.current_task)
        if result.ok:
            self._pending_validation_event = Event.VALIDATED
        else:
            self.last_error_code = result.error_code
            self.last_error_message = result.message
            self.last_validation_error = result.reason
            self._pending_validation_event = Event.VALIDATION_FAILED
        return []

    def _enter_scan_scene(self, payload: Dict[str, Any]) -> List[ActionRequest]:
        self.current_object_id = ""
        self.substate = "SCAN_SCENE"
        return []

    def _enter_execute_item(self, payload: Dict[str, Any]) -> List[ActionRequest]:
        if payload.get("object_id"):
            self.current_object_id = str(payload["object_id"])
        self.substate = "EXECUTE_ITEM"
        self.current_slot = self._slot_for_current_index()
        return [_act(ActionKind.CALL_PICK_PLACE, **self._build_pick_place_request(self.current_slot))]

    def _enter_recovery(self, payload: Dict[str, Any]) -> List[ActionRequest]:
        error_code = self._extract_error_code(payload, default=ErrorCodes.TARGET_NOT_FOUND)
        self.last_error_code = error_code
        self.last_error_message = str(payload.get("message") or ErrorCodes.name_of(error_code))
        self.substate = "RECOVERY"
        # 注意：recovery_attempts 统计"已批准的重试次数"，进入 RECOVERY 本身不计数，
        # 因此首次尝试不占用预算（recovery_max_attempts=1 恰好允许一次重试）。
        return []

    def _enter_cancel_pending(self, payload: Dict[str, Any]) -> List[ActionRequest]:
        self.substate = "CANCEL_PENDING"
        return [_act(ActionKind.CANCEL_CHILD, request_id=self._current_request_id())]

    def _enter_terminal_ok(self, payload: Dict[str, Any]) -> List[ActionRequest]:
        self.substate = "FINISHED"
        self.current_slot = ""
        self.current_object_id = ""
        self.last_error_code = ErrorCodes.OK
        self.last_error_message = ""
        self.last_outcome = self.build_outcome()
        return []

    def _enter_task_failed(self, payload: Dict[str, Any]) -> List[ActionRequest]:
        self.substate = "TASK_FAILED"
        if self.last_error_code == ErrorCodes.OK:
            self.last_error_code = ErrorCodes.INVALID_TASK
            self.last_error_message = "task failed"
        self.last_outcome = self.build_outcome()
        return []

    def _enter_cancelled(self, payload: Dict[str, Any]) -> List[ActionRequest]:
        self.substate = "CANCELLED"
        self.last_outcome = self.build_outcome()
        return []

    def _enter_fault(self, payload: Dict[str, Any]) -> List[ActionRequest]:
        self.substate = "FAULT"
        # 保留已由守卫/超时翻译好的具体错误码；仅在没有更具体原因时才退化。
        if self.last_error_code == ErrorCodes.OK:
            self.last_error_code = ErrorCodes.HARDWARE_FAULT
            self.last_error_message = "entered FAULT without a specific error code"
        self.last_outcome = self.build_outcome()
        # §3.1：FAULT 需停止请求并锁定新的动作，等待人工排查
        return [_act(ActionKind.REQUEST_STOP), _act(ActionKind.LOCK_ACTIONS)]

    # -- 守卫函数（全部以 _guard_ 开头，供转移表引用）----------------------

    def _guard_goal_denied(self, event_name: str, payload: Dict[str, Any]) -> bool:
        """§7.2：已有进行中 Goal 时，第二个 Goal 一律拒绝，不得排队执行。

        守卫总是返回 False，由 ``step()`` 依据 ``has_active_task`` 登记拒绝原因。
        """
        return False

    def _guard_goal_admissible(self, event_name: str, payload: Dict[str, Any]) -> bool:
        """WAIT_TASK 下受理 Goal：先做最基本的 task_id 检查，再进入 VALIDATE_TASK。"""
        if self.has_active_task:
            return False
        goal = payload.get("goal")
        if goal is None:
            self.last_rejection = {
                "reason": "missing_goal",
                "error_code": ErrorCodes.INVALID_TASK,
                "message": "goal payload missing",
            }
            return False
        try:
            task = goal if isinstance(goal, TaskRequest) else TaskRequest.from_goal(goal)
        except (TypeError, ValueError) as exc:
            self.last_rejection = {
                "reason": "unparsable_goal",
                "error_code": ErrorCodes.INVALID_TASK,
                "message": "cannot parse goal: %s" % exc,
            }
            return False
        if not str(task.task_id).strip():
            self.last_rejection = {
                "reason": "empty_task_id",
                "error_code": ErrorCodes.INVALID_TASK,
                "message": "task_id must not be empty",
            }
            return False

        self.current_task = task
        self.current_index = 0
        self.completed_slots = []
        self.completed_count = 0
        self.recovery_attempts = 0
        self.last_error_code = ErrorCodes.OK
        self.last_error_message = ""
        self.last_validation_error = ""
        self.last_rejection = {}
        self.last_outcome = None
        self.task_started_at = self._clock()
        return True

    def _guard_target_usable(self, event_name: str, payload: Dict[str, Any]) -> bool:
        """object_id 必须在任务执行前由感知选定，执行中不得静默切换。"""
        object_id = str(payload.get("object_id", ""))
        if not object_id:
            self.last_error_code = ErrorCodes.TARGET_NOT_FOUND
            self.last_error_message = "target_found without object_id"
            return False
        expected = payload.get("expected_color")
        if expected not in (None, self._current_color()):
            self.last_error_code = ErrorCodes.INVALID_TASK
            self.last_error_message = "expected_color mismatch on target_found"
            return False
        self.current_object_id = object_id
        return True

    def _guard_error_recoverable(self, event_name: str, payload: Dict[str, Any]) -> bool:
        """只有"明确安全可重试"的错误才允许进入 RECOVERY。

        MOTION_STATUS_UNKNOWN(230) 等不确定在途运动错误返回 False，
        由转移表的 fallback 直接进入 FAULT（不自动重试）。
        """
        error_code = self._extract_error_code(payload, default=ErrorCodes.HARDWARE_FAULT)
        self.last_error_code = error_code
        self.last_error_message = str(payload.get("message") or ErrorCodes.name_of(error_code))
        return self.config.is_recoverable(error_code)

    def _guard_all_items_done(self, event_name: str, payload: Dict[str, Any]) -> bool:
        """全部完成 → FINISHED；否则由转移表的 ``else_state`` 回到 SCAN_SCENE。"""
        return self.total_count > 0 and self.completed_count >= self.total_count

    def _guard_retry_budget_available(self, event_name: str, payload: Dict[str, Any]) -> bool:
        """重试预算：``recovery_max_attempts`` 指**额外重试**次数（不含首次尝试）。

        ``recovery_attempts`` 只在重试被批准时 +1；预算耗尽则由转移表的
        ``fallback`` 转入 TASK_FAILED，不会出现"守卫拒绝后原地不动"的死循环。
        """
        if not self.config.is_recoverable(self.last_error_code):
            return False
        if self.recovery_attempts >= max(0, int(self.config.recovery_max_attempts)):
            return False
        self.recovery_attempts += 1
        return True

    def _guard_cancel_timeout(self, event_name: str, payload: Dict[str, Any]) -> bool:
        """CANCEL_PENDING 超时 → FAULT(520)；仅真超时才放行该转移。"""
        if not self._state_timeout_expired():
            return False
        self.last_error_code = ErrorCodes.CANCEL_NOT_CONFIRMED
        self.last_error_message = "cancel not confirmed before timeout (child never reported stopped)"
        return True

    def _guard_state_timed_out(self, event_name: str, payload: Dict[str, Any]) -> bool:
        if not self._state_timeout_expired():
            return False
        self._apply_timeout_error()
        return True

    # -- 超时辅助 ---------------------------------------------------------

    def _state_timeout_expired(self, now: Optional[float] = None) -> bool:
        """当前状态自进入以来的停留时间是否已达到其 ``timeout_s``。"""
        timeout = self.config.timeout_for(self.state)
        if timeout is None:
            return False
        if now is None:
            now = self._clock()
        return (now - self.state_entered_at) >= float(timeout)

    def _apply_timeout_error(self) -> None:
        """把超时翻译成本层错误码（见设计文档 §7.1 默认处置）。"""
        mapping = {
            TaskStateName.INIT: ErrorCodes.HARDWARE_FAULT,
            TaskStateName.SELF_CHECK: ErrorCodes.HARDWARE_FAULT,
            TaskStateName.VALIDATE_TASK: ErrorCodes.INVALID_TASK,
            TaskStateName.SCAN_SCENE: ErrorCodes.TARGET_NOT_FOUND,
            TaskStateName.EXECUTE_ITEM: ErrorCodes.MOTION_TIMEOUT,
            TaskStateName.RECORD_RESULT: ErrorCodes.PLACEMENT_FAILED,
            TaskStateName.RECOVERY: ErrorCodes.TARGET_NOT_FOUND,
            TaskStateName.CANCEL_PENDING: ErrorCodes.CANCEL_NOT_CONFIRMED,
        }
        code = mapping.get(self.state, ErrorCodes.HARDWARE_FAULT)
        self.last_error_code = code
        self.last_error_message = "state %s timed out after %.3fs -> %s" % (
            self.state.value,
            self.elapsed_in_state(),
            ErrorCodes.name_of(code),
        )

    # -- 内部工具 ---------------------------------------------------------

    def _extract_error_code(self, payload: Dict[str, Any], default: int) -> int:
        try:
            return int(payload.get("error_code", default))
        except (TypeError, ValueError):
            return int(default)

    def _touch(self) -> None:
        """重置超时计时基准（进入新状态或发生有效进展时调用）。"""
        self.state_entered_at = self._clock()

    def _current_color(self) -> Optional[int]:
        if self.current_task is None:
            return None
        if self.current_index >= len(self.current_task.ordered_colors):
            return None
        return int(self.current_task.ordered_colors[self.current_index])

    def _slot_for_current_index(self) -> str:
        if self.current_task is None:
            return ""
        return self.current_task.slot_for_index(self.current_index) or ""

    def _current_request_id(self) -> str:
        if self.current_task is None:
            return ""
        return "%s#%d" % (self.current_task.task_id, self.current_index)

    def _build_pick_place_request(self, slot: str) -> Dict[str, Any]:
        """构造 PickPlace.action Goal 内容（字段与 .action 完全一致）。"""
        task = self.current_task
        return {
            "request_id": self._current_request_id(),
            "object_id": self.current_object_id,
            "expected_color": self._current_color(),
            "target_slot": slot,
            "placement_stability_sec": float(task.placement_stability_sec) if task else 0.0,
        }

    def _register_second_goal_rejection(self, payload: Dict[str, Any]) -> None:
        """并发拒绝：记录原因、返回拒绝结果，不排队、不切换任务。"""
        requested_id = ""
        goal = payload.get("goal")
        if goal is not None:
            try:
                requested_id = str(getattr(goal, "task_id", "") or "")
                if not requested_id and isinstance(goal, dict):
                    requested_id = str(goal.get("task_id", "") or "")
            except Exception:  # pragma: no cover - 防御式
                requested_id = ""
        active_id = self.current_task.task_id if self.current_task else ""
        self.last_rejection = {
            "reason": "task_already_in_progress",
            "error_code": ErrorCodes.EXECUTION_REJECTED,
            "message": (
                "rejected: task '%s' already in progress in state %s; "
                "a second concurrent goal is not queued" % (active_id, self.state.value)
            ),
            "active_task_id": active_id,
            "rejected_task_id": requested_id,
            "active_state": self.state.value,
        }

    # -- 便捷驱动接口（ROS 外壳与测试共用）--------------------------------

    def start(self) -> StepResult:
        """INIT → SELF_CHECK。"""
        return self.step(Event.SELF_CHECK_OK, {"source": "start"})

    def submit_goal(self, goal: Any) -> StepResult:
        """受理一个 ExecuteTask Goal（等价于 ROS Action Server 的 goal callback）。"""
        return self.step(Event.GOAL_ACCEPTED, {"goal": goal})

    def take_pending_validation_event(self) -> Optional[str]:
        """取出 ``_enter_validate_task`` 产生的待发事件（由外壳/测试消费一次）。"""
        event = getattr(self, "_pending_validation_event", None)
        self._pending_validation_event = None
        return event

    def advance_validation(self) -> StepResult:
        """消费 VALIDATE_TASK 的校验结论：合法 → SCAN_SCENE，非法 → TASK_FAILED。"""
        if self.state is not TaskStateName.VALIDATE_TASK:
            return StepResult(state=self.state, actions=[], reason="not in VALIDATE_TASK")
        event = self.take_pending_validation_event()
        if event is None:
            # 没有缓存结论时现场重算一次，保证接口自洽
            result = validate_task_request(self.current_task)
            event = Event.VALIDATED if result.ok else Event.VALIDATION_FAILED
            if not result.ok:
                self.last_error_code = result.error_code
                self.last_error_message = result.message
                self.last_validation_error = result.reason
        return self.step(event, {})

    def report_target(self, object_id: str, expected_color: Optional[int] = None) -> StepResult:
        """感知回调：找到唯一候选目标。"""
        payload: Dict[str, Any] = {"object_id": object_id}
        if expected_color is not None:
            payload["expected_color"] = int(expected_color)
        return self.step(Event.TARGET_FOUND, payload)

    def report_target_missing(self, reason: str = "no unique candidate for current color") -> StepResult:
        """感知回调：未找到唯一候选。"""
        return self.step(
            Event.TARGET_NOT_FOUND,
            {"error_code": ErrorCodes.TARGET_NOT_FOUND, "message": reason},
        )

    def report_item_result(self, payload: Dict[str, Any]) -> StepResult:
        """把 PickPlace.Result 翻译成 item_succeeded / item_failed / item_unsafe。"""
        if self.state is not TaskStateName.EXECUTE_ITEM:
            return StepResult(
                state=self.state,
                actions=[],
                reason="item result ignored: state is %s, not EXECUTE_ITEM" % self.state.value,
            )
        error_code = self._extract_error_code(payload, default=ErrorCodes.OK)
        if bool(payload.get("success")):
            if not bool(payload.get("placement_verified")):
                # §4.4：placement_verified 必须在 VERIFY_PLACED 通过后才为 true；
                # 反向约束——未验证放置不得记为完成。
                return self.step(
                    Event.ITEM_FAILED,
                    {
                        "error_code": ErrorCodes.PLACEMENT_FAILED,
                        "message": "PickPlace reported success without placement_verified",
                    },
                )
            return self.step(Event.ITEM_SUCCEEDED, payload)
        if error_code in NON_RECOVERABLE_ERRORS:
            # 先登记具体错误码，再走 ITEM_UNSAFE → FAULT，
            # 使 FAULT 快照保留真实原因（如 230），而不是退化为 HARDWARE_FAULT。
            self.last_error_code = error_code
            self.last_error_message = str(payload.get("message") or ErrorCodes.name_of(error_code))
            return self.step(
                Event.ITEM_UNSAFE,
                {"error_code": error_code, "message": self.last_error_message},
            )
        return self.step(
            Event.ITEM_FAILED,
            {
                "error_code": error_code or ErrorCodes.PLACEMENT_FAILED,
                "message": payload.get("message", ""),
            },
        )

    def record_completed_item(self, slot: Optional[str] = None) -> StepResult:
        """在 RECORD_RESULT 中登记真实完成的一项（不依赖发令返回码）。"""
        if self.state is not TaskStateName.RECORD_RESULT:
            return StepResult(state=self.state, actions=[], reason="record ignored: not RECORD_RESULT")
        resolved = str(slot if slot is not None else self.current_slot)
        if resolved and resolved not in self.completed_slots:
            self.completed_slots.append(resolved)
        self.completed_count = len(self.completed_slots)
        self.recovery_attempts = 0
        self.current_index += 1
        return self.step(Event.RECORD_OK, {})

    def request_retry(self) -> StepResult:
        """在 RECOVERY 中请求重试（守卫决定是否放行）。"""
        return self.step(Event.RETRY_APPROVED, {})

    def deny_retry(self) -> StepResult:
        return self.step(Event.RETRY_DENIED, {})

    def request_cancel(self) -> StepResult:
        return self.step(Event.CANCEL_REQUESTED, {})

    def confirm_child_stopped(self) -> StepResult:
        return self.step(Event.CHILD_STOPPED_CONFIRMED, {})

    def reset(self) -> StepResult:
        return self.step(Event.RESET, {})
