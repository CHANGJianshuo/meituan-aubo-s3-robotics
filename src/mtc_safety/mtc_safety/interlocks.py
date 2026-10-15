"""mtc_safety.interlocks —— 软件安全联锁与故障锁存核心逻辑。

本模块 **纯 Python 3，绝不 import rclpy / ROS 消息包**，可在无 ROS 的机器上
直接用 pytest / unittest 运行（见 ``test/test_interlocks.py``）。所有 ROS 相关的
订阅、发布、服务调用都封装在 ``mtc_safety.safety_supervisor`` 外壳里。

设计依据
--------
- 设计文档 V0.1 §7.1 错误码表（与 ``mtc_interfaces/msg/ErrorCodes.msg`` 一致）
- 设计文档 V0.1 §7.2 关键联锁表
- 设计文档 V0.1 §3.3 设备运行状态与任务状态分离
- 设计文档 V0.1 §6.4 停止与不确定状态处理
- ``mtc_motion_execution/executor_core.py`` 的 ``stop_confirmed`` /
  ``motion_state_unknown`` 语义（本模块不重复实现运动执行）

核心安全不变量（任何修改都不得破坏）
------------------------------------
1. **不确定就绝不重发**：运动状态未知、或上一条指令「可能已经执行过运动」时，
   ``allow_resend_motion()`` 必须拒绝（MOTION_STATUS_UNKNOWN=230）。只有当执行层
   给出「该指令在开始运动前即被拒绝」的明确证据，且停稳已确认时，才允许重发。
2. **送达不等于停稳**：停止服务的 ``request_delivered`` 只是请求送达；
   ``stop_confirmed`` 只能来自执行层/机器人状态的独立确认。
3. **载荷未知即最保守**：``payload == unknown`` 时禁止自动解锁、禁止无条件 Home、
   禁止进入下一任务项。
4. **FAULT 只能人工解除**：``FaultLatch`` 没有任何自动清除路径，
   只有显式的人工 ``clear_by_human(note)`` 才能解锁，并记录 note 与时间戳。

检查顺序与错误码选择（保持全局一致）
------------------------------------
- FAULT 锁定或设备处于 FAULT：除 ``allow_resend_motion`` 外一律优先返回
  ``SAFETY_INTERLOCK=500``。
- ``allow_resend_motion`` 最优先判定 230（重发是最高风险路径，不允许被其他
  状态码掩盖）。
- ``allow_unlock``：载荷未知 / 运动状态未知 / 运动中 / 载荷为 none → 500；
  **未落座 → SEAT_NOT_VERIFIED=400**（本包统一选择 400，不使用 500 表示未落座）。
- ``allow_next_task_item``：运动状态未知 → 230；未确认停稳 → 520。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, List, NamedTuple, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# 错误码（对应 mtc_interfaces/msg/ErrorCodes.msg；此处为纯 Python 副本，
# 避免在无 ROS 环境下 import 消息包。数值必须与 .msg 文件保持一致。）
# ---------------------------------------------------------------------------
ERR_OK = 0
ERR_INVALID_TASK = 100
ERR_TARGET_NOT_FOUND = 110
ERR_POSE_STALE = 120
ERR_PLANNING_FAILED = 200
ERR_EXECUTION_REJECTED = 210
ERR_MOTION_TIMEOUT = 220
ERR_MOTION_STATUS_UNKNOWN = 230
ERR_ENGAGE_FAILED = 300
ERR_ATTACH_NOT_VERIFIED = 310
ERR_OBJECT_DROPPED = 320
ERR_SEAT_NOT_VERIFIED = 400
ERR_UNLOCK_FAILED = 410
ERR_RELEASE_NOT_VERIFIED = 420
ERR_PLACEMENT_FAILED = 430
ERR_SAFETY_INTERLOCK = 500
ERR_HARDWARE_FAULT = 510
ERR_CANCEL_NOT_CONFIRMED = 520

ERROR_CODE_NAMES: Dict[int, str] = {
    ERR_OK: 'OK',
    ERR_INVALID_TASK: 'INVALID_TASK',
    ERR_TARGET_NOT_FOUND: 'TARGET_NOT_FOUND',
    ERR_POSE_STALE: 'POSE_STALE',
    ERR_PLANNING_FAILED: 'PLANNING_FAILED',
    ERR_EXECUTION_REJECTED: 'EXECUTION_REJECTED',
    ERR_MOTION_TIMEOUT: 'MOTION_TIMEOUT',
    ERR_MOTION_STATUS_UNKNOWN: 'MOTION_STATUS_UNKNOWN',
    ERR_ENGAGE_FAILED: 'ENGAGE_FAILED',
    ERR_ATTACH_NOT_VERIFIED: 'ATTACH_NOT_VERIFIED',
    ERR_OBJECT_DROPPED: 'OBJECT_DROPPED',
    ERR_SEAT_NOT_VERIFIED: 'SEAT_NOT_VERIFIED',
    ERR_UNLOCK_FAILED: 'UNLOCK_FAILED',
    ERR_RELEASE_NOT_VERIFIED: 'RELEASE_NOT_VERIFIED',
    ERR_PLACEMENT_FAILED: 'PLACEMENT_FAILED',
    ERR_SAFETY_INTERLOCK: 'SAFETY_INTERLOCK',
    ERR_HARDWARE_FAULT: 'HARDWARE_FAULT',
    ERR_CANCEL_NOT_CONFIRMED: 'CANCEL_NOT_CONFIRMED',
}

# ToolState.msg 常量副本（uint8 state / evidence_level）
TOOL_STATE_UNKNOWN = 0
TOOL_STATE_DETACHED = 1
TOOL_STATE_ENGAGING = 2
TOOL_STATE_ATTACHED = 3
TOOL_STATE_RELEASING = 4
TOOL_STATE_FAULT = 5

TOOL_EVIDENCE_NONE = 0
TOOL_EVIDENCE_COMMAND_ONLY = 1
TOOL_EVIDENCE_GEOMETRY = 2
TOOL_EVIDENCE_SENSOR_OR_VISION = 3


def error_code_name(code: int) -> str:
    """把错误码转成可读常量名，便于日志与断言失败信息。"""
    return ERROR_CODE_NAMES.get(int(code), f'UNKNOWN_{int(code)}')


class GuardResult(NamedTuple):
    """守卫检查结果；本身是 tuple，可直接 ``allowed, code, reason = ...`` 解包。"""

    allowed: bool
    error_code: int
    reason: str

    @property
    def blocked(self) -> bool:
        return not self.allowed


# ---------------------------------------------------------------------------
# 设备运行状态（§3.3）
# ---------------------------------------------------------------------------
class DeviceState(Enum):
    """设备运行状态，与任务状态严格分离（§3.3）。

    设备 READY 不代表 PickPlace 成功；设备 STOPPED 也不代表任务可安全重试。
    """

    DISCONNECTED = 'DISCONNECTED'
    NOT_READY = 'NOT_READY'
    READY = 'READY'
    MOVING = 'MOVING'
    STOP_REQUESTED = 'STOP_REQUESTED'
    STOPPED = 'STOPPED'
    FAULT = 'FAULT'


# ---------------------------------------------------------------------------
# 载荷状态机（unknown / none / carrying）
# ---------------------------------------------------------------------------
class PayloadState(Enum):
    """载荷（电池）状态。

    - ``UNKNOWN``：未验证，或事件后失效。**最保守状态**：禁止自动解锁、
      禁止无条件 Home、禁止进入下一任务项（§6.4）。
    - ``NONE``：有独立证据表明工具未携带电池。
    - ``CARRYING``：有独立证据表明工具携带电池。
    """

    UNKNOWN = 'unknown'
    NONE = 'none'
    CARRYING = 'carrying'


class PayloadTracker:
    """载荷状态机：任意状态都可以回到 ``unknown``；正向确认必须给出证据 note。"""

    def __init__(self, state: PayloadState = PayloadState.UNKNOWN,
                 clock: Optional[Callable[[], float]] = None) -> None:
        self._clock = clock or time.monotonic
        self._state = state
        self.history: List[Tuple[float, str, str]] = []

    @property
    def state(self) -> PayloadState:
        return self._state

    def _record(self, new_state: PayloadState, note: str) -> None:
        self.history.append((self._clock(), new_state.value, note))

    def mark_unknown(self, reason: str = '') -> PayloadState:
        """进入 unknown：任何时候都允许（保守方向），不需要证据。"""
        self._state = PayloadState.UNKNOWN
        self._record(PayloadState.UNKNOWN, reason or 'invalidated')
        return self._state

    def _confirm(self, state: PayloadState, note: str) -> Tuple[bool, str]:
        if not note or not str(note).strip():
            return False, f'确认载荷状态 {state.value} 必须提供证据 note，已拒绝'
        self._state = state
        self._record(state, str(note))
        return True, f'载荷状态已确认为 {state.value}（证据：{note}）'

    def mark_none(self, note: str) -> Tuple[bool, str]:
        """确认未携带载荷（例如 ToolState=DETACHED 且 verified）。"""
        return self._confirm(PayloadState.NONE, note)

    def mark_carrying(self, note: str) -> Tuple[bool, str]:
        """确认携带载荷（例如 ToolState=ATTACHED 且 verified）。"""
        return self._confirm(PayloadState.CARRYING, note)

    # -- 载荷相关的安全询问（载荷未知时全部为 False）------------------------
    def automatic_unlock_allowed(self) -> bool:
        """只有确认携带载荷（且已确认落座，由 InterlockGuard 另行判定）才允许自动解锁脉冲。"""
        return self._state is PayloadState.CARRYING

    def unconditional_home_allowed(self) -> bool:
        """只有明确证明未携带载荷时，才允许无条件 Home / 原路返回（§6.4）。"""
        return self._state is PayloadState.NONE

    def next_task_item_allowed(self) -> bool:
        """载荷未知时禁止进入下一任务项（§6.4）。"""
        return self._state is not PayloadState.UNKNOWN


def payload_from_tool_state(state: int, verified: bool,
                            evidence_level: int = TOOL_EVIDENCE_NONE) -> PayloadState:
    """由 ToolState 推断载荷状态。

    ``state`` 是当前估计，``verified`` 是独立维度：两者不得混为一谈——
    只有 ``verified=True`` 的 ATTACHED / DETACHED 才能给出 carrying / none，
    其余（含 EVIDENCE_COMMAND_ONLY）一律保守回落到 unknown。
    """
    if not verified:
        return PayloadState.UNKNOWN
    if int(evidence_level) <= TOOL_EVIDENCE_COMMAND_ONLY:
        return PayloadState.UNKNOWN
    if int(state) == TOOL_STATE_ATTACHED:
        return PayloadState.CARRYING
    if int(state) == TOOL_STATE_DETACHED:
        return PayloadState.NONE
    return PayloadState.UNKNOWN


def device_state_from_robot_state(communication_ok: bool,
                                  robot_ready: bool,
                                  motion_active: bool,
                                  safety_normal: bool,
                                  stop_confirmed: bool,
                                  stop_requested: bool = False,
                                  fault_latched: bool = False) -> DeviceState:
    """由 RobotState 字段推导 §3.3 设备状态（纯函数，便于离线测试）。"""
    if not communication_ok:
        return DeviceState.DISCONNECTED
    if fault_latched or not safety_normal:
        return DeviceState.FAULT
    if stop_requested and not stop_confirmed:
        return DeviceState.STOP_REQUESTED
    if motion_active:
        return DeviceState.MOVING
    if stop_requested:
        return DeviceState.STOPPED
    return DeviceState.READY if robot_ready else DeviceState.NOT_READY


# ---------------------------------------------------------------------------
# FAULT 锁存：只能人工解除
# ---------------------------------------------------------------------------
@dataclass
class FaultRecord:
    error_code: int
    reason: str
    timestamp: float
    context: Dict[str, object] = field(default_factory=dict)

    def as_dict(self) -> Dict[str, object]:
        return {
            'error_code': self.error_code,
            'error_name': error_code_name(self.error_code),
            'reason': self.reason,
            'timestamp': self.timestamp,
            'context': dict(self.context),
        }


class FaultLatch:
    """故障锁存器：进入 FAULT 后保持锁定，必须人工显式解除。

    - ``latch()``：任何检测到故障的路径都可以锁存（自动路径只允许**加锁**）。
    - ``try_auto_clear()``：提供给「自动恢复」代码的入口，**永远返回失败**，
      并记录一次被拒绝的尝试，用于证明自动路径无法清除。
    - ``clear_by_human(note)``：唯一解除路径，必须带非空 note，记录 note 与时间戳。
    """

    def __init__(self, clock: Optional[Callable[[], float]] = None) -> None:
        self._clock = clock or time.monotonic
        self._record: Optional[FaultRecord] = None
        self.history: List[FaultRecord] = []
        self.clear_attempts: List[Dict[str, object]] = []
        self.clear_history: List[Dict[str, object]] = []

    @property
    def latched(self) -> bool:
        return self._record is not None

    @property
    def record(self) -> Optional[FaultRecord]:
        return self._record

    def latch(self, error_code: int, reason: str,
              context: Optional[Dict[str, object]] = None,
              now: Optional[float] = None) -> FaultRecord:
        """锁存故障。已锁存时保留**首次**故障（首因优先，便于现场诊断）。"""
        timestamp = self._clock() if now is None else float(now)
        record = FaultRecord(int(error_code), str(reason), timestamp,
                             dict(context or {}))
        self.history.append(record)
        if self._record is None:
            self._record = record
        return record

    def try_auto_clear(self, origin: str, note: str = '',
                       now: Optional[float] = None) -> Tuple[bool, str]:
        """自动路径清除尝试：永远失败并留痕（安全不变量 4）。"""
        timestamp = self._clock() if now is None else float(now)
        self.clear_attempts.append({
            'origin': str(origin),
            'note': str(note),
            'timestamp': timestamp,
            'accepted': False,
        })
        return False, (f'FAULT 锁定禁止由自动路径（{origin}）解除；'
                       '必须由人工调用 clear_by_human(note)')

    def clear_by_human(self, note: str,
                       now: Optional[float] = None) -> Tuple[bool, str]:
        """唯一解除路径：必须由人工调用且给出非空说明。"""
        if not note or not str(note).strip():
            timestamp = self._clock() if now is None else float(now)
            self.clear_attempts.append({
                'origin': 'human', 'note': '', 'timestamp': timestamp,
                'accepted': False,
            })
            return False, 'clear_by_human 必须提供非空 note（记录现场核验结论），已拒绝'
        timestamp = self._clock() if now is None else float(now)
        record = self._record
        self._record = None
        self.clear_history.append({
            'note': str(note),
            'timestamp': timestamp,
            'cleared_fault': record.as_dict() if record else None,
        })
        return True, f'FAULT 锁定已由人工解除（note={note}）'


# ---------------------------------------------------------------------------
# 联锁守卫
# ---------------------------------------------------------------------------
class InterlockGuard:
    """软件安全联锁守卫（§7.2）。

    每个 ``allow_*`` 方法返回 ``GuardResult(allowed, error_code, reason)``。
    所有拒绝都是**纯函数式判断**，不做任何 I/O，也不调用 ROS。
    """

    def __init__(self, *,
                 device_state: DeviceState = DeviceState.DISCONNECTED,
                 controller_mode: str = '',
                 expected_modes: Sequence[str] = (),
                 simulation_mode: bool = False,
                 expect_simulation: Optional[bool] = None,
                 state_ttl_sec: float = 0.5,
                 clock: Optional[Callable[[], float]] = None,
                 payload: Optional[PayloadTracker] = None,
                 fault_latch: Optional[FaultLatch] = None,
                 link_ok: bool = False,
                 attach_verified: bool = False,
                 seat_verified: bool = False,
                 release_verified: bool = False,
                 motion_state_unknown: bool = False,
                 may_have_executed_motion: bool = False,
                 stop_confirmed: bool = True,
                 retry_evidence: bool = False,
                 motion_in_flight: Optional[str] = None,
                 last_state_ts: Optional[float] = None) -> None:
        self._clock = clock or time.monotonic
        self.fault_latch = fault_latch if fault_latch is not None else FaultLatch(self._clock)
        self.payload_tracker = payload if payload is not None else PayloadTracker(
            PayloadState.UNKNOWN, self._clock)

        self.device_state = device_state
        self.controller_mode = controller_mode
        self.expected_modes: Tuple[str, ...] = tuple(expected_modes)
        self.simulation_mode = bool(simulation_mode)
        self.expect_simulation = expect_simulation
        self.state_ttl_sec = float(state_ttl_sec)

        self.link_ok = bool(link_ok)
        self.attach_verified = bool(attach_verified)
        self.seat_verified = bool(seat_verified)
        self.release_verified = bool(release_verified)
        self.motion_state_unknown = bool(motion_state_unknown)
        self.may_have_executed_motion = bool(may_have_executed_motion)
        self.stop_confirmed = bool(stop_confirmed)
        self.retry_evidence = bool(retry_evidence)
        self.motion_in_flight = motion_in_flight

        self._last_state_ts = last_state_ts
        self.notes: List[str] = []

    # -- 内部工具 ----------------------------------------------------------
    def _now(self, now: Optional[float] = None) -> float:
        return self._clock() if now is None else float(now)

    def _note(self, text: str) -> None:
        self.notes.append(f'{self._now():.3f} {text}')

    @property
    def payload(self) -> PayloadState:
        return self.payload_tracker.state

    @property
    def fault_latched(self) -> bool:
        return self.fault_latch.latched

    @property
    def motion_active(self) -> bool:
        """是否正在运动中：有在途指令或设备状态为 MOVING。"""
        return self.motion_in_flight is not None or self.device_state is DeviceState.MOVING

    def mode_matches(self) -> bool:
        """控制器模式 / 真机-仿真模式是否与期望一致（§7.2 第二行）。"""
        if self.expected_modes and self.controller_mode not in self.expected_modes:
            return False
        if self.expect_simulation is not None and \
                bool(self.simulation_mode) != bool(self.expect_simulation):
            return False
        return True

    def state_expired(self, now: Optional[float] = None) -> bool:
        """设备状态是否过期：从未收到过状态也视为过期（保守）。"""
        if self._last_state_ts is None:
            return True
        return (self._now(now) - self._last_state_ts) > self.state_ttl_sec

    # -- 状态输入接口 ------------------------------------------------------
    def mark_state(self, now: Optional[float] = None) -> None:
        """收到一次新的设备状态观测（刷新时间戳）。"""
        self._last_state_ts = self._now(now)

    def set_device_state(self, state: DeviceState,
                         now: Optional[float] = None) -> None:
        self.device_state = state
        self.mark_state(now)

    def set_controller_mode(self, controller_mode: str,
                            simulation_mode: Optional[bool] = None,
                            now: Optional[float] = None) -> None:
        self.controller_mode = controller_mode
        if simulation_mode is not None:
            self.simulation_mode = bool(simulation_mode)
        self.mark_state(now)

    def note_link_lost(self, detail: str = '') -> None:
        self.link_ok = False
        self.device_state = DeviceState.DISCONNECTED
        self.note_motion_state_unknown(detail or 'link_lost')

    def note_attach_verified(self, verified: bool, note: str = '') -> None:
        self.attach_verified = bool(verified)
        if note:
            self._note(f'attach_verified={verified} {note}')

    def note_seat_verified(self, verified: bool, note: str = '') -> None:
        self.seat_verified = bool(verified)
        if note:
            self._note(f'seat_verified={verified} {note}')

    def note_release_verified(self, verified: bool, note: str = '') -> None:
        self.release_verified = bool(verified)
        if note:
            self._note(f'release_verified={verified} {note}')

    def note_motion_started(self, command_id: str) -> None:
        """登记一条已发令的运动（发令后即视为「可能已经执行」）。"""
        self.motion_in_flight = command_id
        self.may_have_executed_motion = True
        self.retry_evidence = False
        self.stop_confirmed = False
        self._note(f'motion_started command_id={command_id}')

    def note_motion_finished(self, command_id: Optional[str] = None,
                             stop_confirmed: bool = True) -> None:
        """执行层回报运动结束；只有确认到位停稳才置 stop_confirmed。"""
        if command_id is None or command_id == self.motion_in_flight:
            self.motion_in_flight = None
        self.stop_confirmed = bool(stop_confirmed)
        if stop_confirmed:
            self.may_have_executed_motion = False
            self.retry_evidence = False
        self._note(f'motion_finished stop_confirmed={stop_confirmed}')

    def note_motion_state_unknown(self, reason: str = '') -> None:
        """反馈断流 / 请求结果不确定：锁定重发，禁止自动补发（§6.4）。"""
        self.motion_state_unknown = True
        self.may_have_executed_motion = True
        self.retry_evidence = False
        self.stop_confirmed = False
        self._note(f'motion_state_unknown {reason}')

    def note_command_rejected_before_motion(self, reason: str = '') -> None:
        """执行层明确回报「从未开始运动」（如后端 REJECTED）：唯一允许重发的证据。

        这是**唯一**能置位 ``retry_evidence`` 的入口，必须由执行层给出。
        """
        self.motion_state_unknown = False
        self.may_have_executed_motion = False
        self.retry_evidence = True
        self.stop_confirmed = True
        self.motion_in_flight = None
        self._note(f'command_rejected_before_motion {reason}')

    def note_may_have_executed_motion(self, reason: str = '') -> None:
        self.may_have_executed_motion = True
        self.retry_evidence = False
        self._note(f'may_have_executed_motion {reason}')

    def note_stop_confirmed(self, confirmed: bool, reason: str = '') -> None:
        """停止确认只能来自执行层/机器人状态的独立证据。"""
        self.stop_confirmed = bool(confirmed)
        self._note(f'stop_confirmed={confirmed} {reason}')

    def clear_unknown_after_manual_confirmation(self, note: str) -> Tuple[bool, str]:
        """人工现场核实后解除「运动状态未知」锁定（自动流程禁止调用）。"""
        if not note or not str(note).strip():
            return False, '解除运动状态未知锁定必须提供非空 note'
        self.motion_state_unknown = False
        self.may_have_executed_motion = False
        self.retry_evidence = False
        self.stop_confirmed = False  # 仍需执行层确认停稳后才能重发
        self._note(f'manual_unknown_clear {note}')
        return True, '人工已确认现场状态；重发仍需 stop_confirmed 证据'

    # -- 守卫检查 ----------------------------------------------------------
    def _fault_block(self) -> Optional[GuardResult]:
        if self.fault_latch.latched:
            record = self.fault_latch.record
            assert record is not None  # latched 保证非 None
            return GuardResult(
                False, ERR_SAFETY_INTERLOCK,
                f'FAULT 已锁存（{error_code_name(record.error_code)}：{record.reason}），'
                '必须人工 clear_by_human 后才能继续')
        if self.device_state is DeviceState.FAULT:
            return GuardResult(False, ERR_SAFETY_INTERLOCK, '设备处于 FAULT 状态')
        return None

    def allow_motion(self, now: Optional[float] = None) -> GuardResult:
        """是否允许发出实机运动（§7.2：未 READY / 模式不符 / 状态过期 → 禁止）。"""
        block = self._fault_block()
        if block is not None:
            return block
        if self.motion_state_unknown:
            return GuardResult(False, ERR_MOTION_STATUS_UNKNOWN,
                               '运动状态未知且未确认：禁止任何实机运动')
        if not self.link_ok or self.device_state is DeviceState.DISCONNECTED:
            return GuardResult(False, ERR_SAFETY_INTERLOCK, '通信断开：禁止实机运动')
        if self.state_expired(now):
            return GuardResult(False, ERR_SAFETY_INTERLOCK,
                               f'设备状态已过期（ttl={self.state_ttl_sec}s）：禁止实机运动')
        if not self.mode_matches():
            return GuardResult(
                False, ERR_SAFETY_INTERLOCK,
                f'控制器模式不符（controller_mode={self.controller_mode!r}, '
                f'expected={self.expected_modes}, simulation={self.simulation_mode}, '
                f'expect_simulation={self.expect_simulation}）：禁止实机运动')
        if self.device_state is not DeviceState.READY:
            return GuardResult(False, ERR_SAFETY_INTERLOCK,
                               f'设备状态 {self.device_state.value} 非 READY：禁止实机运动')
        return GuardResult(True, ERR_OK, '设备 READY、模式一致且状态新鲜：允许运动')

    def allow_transport(self, now: Optional[float] = None) -> GuardResult:
        """是否允许进入正常运输阶段（§7.2：未验证挂载 → 禁止运输）。"""
        block = self._fault_block()
        if block is not None:
            return block
        if not self.attach_verified:
            return GuardResult(False, ERR_ATTACH_NOT_VERIFIED,
                               '挂载未验证（缺少试提/传感证据）：禁止进入运输阶段')
        if self.payload is not PayloadState.CARRYING:
            return GuardResult(False, ERR_ATTACH_NOT_VERIFIED,
                               f'载荷状态为 {self.payload.value}，不足以证明电池已挂稳：禁止运输')
        return GuardResult(True, ERR_OK, '挂载已验证且载荷确认为 carrying：允许运输')

    def allow_unlock(self, now: Optional[float] = None) -> GuardResult:
        """是否允许发送 V2 电磁解锁脉冲（§7.2：电池尚未安全落座 → 禁止解锁）。

        本包统一约定：**未落座返回 SEAT_NOT_VERIFIED=400**（不使用 500）；
        载荷未知 / 运动状态未知 / 运动中 / 载荷为 none 一律返回 SAFETY_INTERLOCK=500。
        """
        block = self._fault_block()
        if block is not None:
            return block
        if self.payload is PayloadState.UNKNOWN:
            return GuardResult(False, ERR_SAFETY_INTERLOCK,
                               '载荷状态未知：禁止自动解锁（§6.4）')
        if self.payload is not PayloadState.CARRYING:
            return GuardResult(False, ERR_SAFETY_INTERLOCK,
                               f'载荷状态为 {self.payload.value}：无待卸载电池，禁止解锁脉冲')
        if self.motion_state_unknown:
            return GuardResult(False, ERR_SAFETY_INTERLOCK,
                               '运动状态未知：禁止解锁脉冲')
        if self.motion_active:
            return GuardResult(False, ERR_SAFETY_INTERLOCK, '设备正在运动：禁止解锁脉冲')
        if not self.seat_verified:
            return GuardResult(False, ERR_SEAT_NOT_VERIFIED,
                               '未确认电池已由桌面/工位承载：禁止解锁')
        return GuardResult(True, ERR_OK, '载荷 carrying 且落座已验证：允许解锁脉冲')

    def allow_force_withdraw(self, now: Optional[float] = None) -> GuardResult:
        """是否允许水平硬拉 / 直接上抬撤离（§7.2：解锁/脱离状态不确定 → 禁止）。"""
        block = self._fault_block()
        if block is not None:
            return block
        if self.motion_state_unknown:
            return GuardResult(False, ERR_MOTION_STATUS_UNKNOWN,
                               '运动状态未知：禁止撤离动作')
        if not self.release_verified:
            return GuardResult(False, ERR_RELEASE_NOT_VERIFIED,
                               '解锁/脱离状态不确定：禁止水平硬拉、直接上抬与撤离')
        return GuardResult(True, ERR_OK, '解锁/脱离已独立确认：允许撤离')

    def allow_second_motion_goal(self, now: Optional[float] = None) -> GuardResult:
        """是否允许第二个运动 Goal（§7.2：正在执行运动 → 拒绝并发/抢占）。"""
        if self.motion_active or self.motion_in_flight is not None:
            return GuardResult(False, ERR_EXECUTION_REJECTED,
                               f'已有在途运动命令 {self.motion_in_flight!r}：拒绝第二个运动 Goal')
        if self.motion_state_unknown:
            return GuardResult(False, ERR_MOTION_STATUS_UNKNOWN,
                               '运动状态未知：无法判定是否空闲，拒绝新的运动 Goal')
        block = self._fault_block()
        if block is not None:
            return block
        return GuardResult(True, ERR_OK, '无在途运动命令：允许接受运动 Goal')

    def allow_next_task_item(self, now: Optional[float] = None) -> GuardResult:
        """是否允许启动下一任务项 / 宣告本项成功 / 自动 Home（§7.2 最后一行）。"""
        block = self._fault_block()
        if block is not None:
            return block
        if self.motion_state_unknown:
            return GuardResult(False, ERR_MOTION_STATUS_UNKNOWN,
                               '运动状态未知：禁止启动下一任务项')
        if not self.stop_confirmed:
            return GuardResult(False, ERR_CANCEL_NOT_CONFIRMED,
                               '取消或断线后未确认停稳：禁止返回成功、启动下一目标或自动 Home')
        if self.payload is PayloadState.UNKNOWN:
            return GuardResult(False, ERR_SAFETY_INTERLOCK,
                               '载荷状态未知：禁止进入下一任务项（§6.4）')
        if self.payload is PayloadState.CARRYING:
            return GuardResult(False, ERR_SAFETY_INTERLOCK,
                               '仍携带电池：禁止进入下一任务项，需先由故障处理确定安全停放策略')
        return GuardResult(True, ERR_OK, '已确认停稳且载荷为空：允许进入下一任务项')

    def allow_unconditional_home(self, now: Optional[float] = None) -> GuardResult:
        """是否允许无条件 Home / 原路返回（§6.4；补充守卫，非 §7.2 原文）。"""
        block = self._fault_block()
        if block is not None:
            return block
        if self.payload is PayloadState.UNKNOWN:
            return GuardResult(False, ERR_SAFETY_INTERLOCK,
                               '载荷状态未知：禁止无条件 Home / 原路返回')
        if self.payload is PayloadState.CARRYING:
            return GuardResult(False, ERR_SAFETY_INTERLOCK,
                               '正携带电池：禁止无条件 Home，需由故障处理确定安全停放策略')
        if self.motion_state_unknown:
            return GuardResult(False, ERR_MOTION_STATUS_UNKNOWN,
                               '运动状态未知：禁止无条件 Home')
        if not self.stop_confirmed:
            return GuardResult(False, ERR_CANCEL_NOT_CONFIRMED,
                               '取消或断线后未确认停稳：禁止自动 Home')
        return GuardResult(True, ERR_OK, '载荷确认为 none 且已确认停稳：允许无条件 Home')

    def allow_resend_motion(self, now: Optional[float] = None) -> GuardResult:
        """是否允许重发上一条运动指令（**最关键的一条**）。

        规则：运动状态未知、或曾可能已执行运动 → **必须拒绝**（230）。
        只有执行层给出「上一条指令在开始运动前即被拒绝」的明确证据
        （``retry_evidence``）且停稳已确认时，才允许重发。
        """
        if self.motion_state_unknown:
            return GuardResult(False, ERR_MOTION_STATUS_UNKNOWN,
                               '运动状态未知：禁止自动重发原目标（§6.4）')
        if self.may_have_executed_motion:
            return GuardResult(False, ERR_MOTION_STATUS_UNKNOWN,
                               '上一条指令可能已经执行过运动：禁止自动重发')
        if self.fault_latch.latched:
            return GuardResult(False, ERR_SAFETY_INTERLOCK,
                               'FAULT 已锁存：禁止重发，须人工核验')
        if not self.retry_evidence:
            return GuardResult(False, ERR_EXECUTION_REJECTED,
                               '缺少「上次指令从未开始运动」的执行层证据：拒绝重发并诊断')
        if not self.stop_confirmed:
            return GuardResult(False, ERR_CANCEL_NOT_CONFIRMED,
                               '未确认停稳：禁止重发')
        if self.motion_active:
            return GuardResult(False, ERR_EXECUTION_REJECTED,
                               f'仍有在途运动命令 {self.motion_in_flight!r}：禁止重发')
        return GuardResult(True, ERR_OK,
                           '执行层已确认上次指令在运动前即被拒绝，且当前停稳：允许重发')


# ---------------------------------------------------------------------------
# 看门狗核心（纯逻辑）：超时/失联 → 生成停止请求并记录日志字段
# ---------------------------------------------------------------------------
@dataclass
class WatchdogConfig:
    """超时参数（秒）。全部通过参数注入，便于离线测试与现场标定。"""

    robot_state_timeout_sec: float = 0.5
    tool_state_timeout_sec: float = 1.0
    stop_confirm_timeout_sec: float = 2.0

    def as_dict(self) -> Dict[str, float]:
        return {
            'robot_state_timeout_sec': self.robot_state_timeout_sec,
            'tool_state_timeout_sec': self.tool_state_timeout_sec,
            'stop_confirm_timeout_sec': self.stop_confirm_timeout_sec,
        }


def adapt_stop_client(client, request_factory=None) -> Optional[Callable[[str], Tuple[Optional[bool], str]]]:
    """把「停止请求客户端」适配成 ``requester(reason) -> (delivered|None, message)``。

    支持注入的形态（离线 fake client 与真实 rclpy client 通用）：

    - ``callable``：直接 ``client(reason)``，返回 ``(delivered, message)`` 或带
      ``request_delivered`` / ``message`` 属性的对象；
    - 带 ``call_async`` 的对象（rclpy ServiceClient）：由 ``request_factory`` 造请求，
      异步发出，返回 ``(None, ...)`` 表示**送达结果未知**（稍后由回调更新）；
    - 带 ``call(reason)`` / ``request_stop(reason)`` / ``stop(reason)`` 的同步 fake。
    """
    if client is None:
        return None

    def _normalize(result) -> Tuple[Optional[bool], str]:
        if isinstance(result, tuple):
            delivered = result[0]
            message = str(result[1]) if len(result) > 1 else ''
            return (None if delivered is None else bool(delivered)), message
        if result is None:
            return None, 'requester 返回 None：送达结果未知'
        if hasattr(result, 'request_delivered'):
            return bool(result.request_delivered), str(getattr(result, 'message', ''))
        return None, 'requester 返回值无法解释：送达结果未知'

    if callable(client):
        def _call_direct(reason: str) -> Tuple[Optional[bool], str]:
            return _normalize(client(reason))
        return _call_direct

    if hasattr(client, 'call_async'):
        def _call_async(reason: str) -> Tuple[Optional[bool], str]:
            request = request_factory(reason) if request_factory is not None else reason
            client.call_async(request)
            return None, 'stop request dispatched asynchronously (delivery unknown yet)'
        return _call_async

    for attribute in ('call', 'request_stop', 'stop'):
        method = getattr(client, attribute, None)
        if callable(method):
            def _call_sync(reason: str, _method=method) -> Tuple[Optional[bool], str]:
                try:
                    return _normalize(_method(reason))
                except TypeError:
                    return _normalize(_method())
            return _call_sync

    raise TypeError(f'无法适配停止客户端：{client!r}')


_STOP_REASON_ERROR_CODES: Dict[str, int] = {
    'robot_state_timeout': ERR_MOTION_STATUS_UNKNOWN,
    'robot_state_never_received': ERR_MOTION_STATUS_UNKNOWN,
    'tool_state_timeout': ERR_MOTION_STATUS_UNKNOWN,
    'tool_state_never_received': ERR_MOTION_STATUS_UNKNOWN,
    'robot_communication_lost': ERR_MOTION_STATUS_UNKNOWN,
    'motion_status_unknown': ERR_MOTION_STATUS_UNKNOWN,
    'safety_normal_false': ERR_SAFETY_INTERLOCK,
    'stop_confirm_timeout': ERR_CANCEL_NOT_CONFIRMED,
}


class WatchdogCore:
    """状态看门狗（纯 Python）：超时 / 失联 / 状态未知 → 调用停止请求并留痕。

    安全边界：本类只负责发出**停止请求**并记录 ``stop_requested`` /
    ``stop_confirmed`` / 所用超时参数；``stop_confirmed`` 只能由外部传入的
    执行层证据置位（``on_robot_state`` 的 ``stop_confirmed`` 字段等），
    **绝不因为停止请求送达就宣告停稳**。
    """

    def __init__(self, config: Optional[WatchdogConfig] = None,
                 stop_requester: Optional[Callable[[str], Tuple[Optional[bool], str]]] = None,
                 clock: Optional[Callable[[], float]] = None,
                 guard: Optional[InterlockGuard] = None) -> None:
        self.config = config or WatchdogConfig()
        self._requester = stop_requester
        self._clock = clock or time.monotonic
        self.guard = guard

        self._last_robot_ts: Optional[float] = None
        self._last_tool_ts: Optional[float] = None
        self._robot_ok = False
        self._safety_normal = True
        self._motion_state_unknown = False
        self._payload = PayloadState.UNKNOWN

        self.stop_requested = False
        self.stop_confirmed = False
        self.stop_confirm_timed_out = False
        self._stop_requested_at: Optional[float] = None
        # 停稳确认截止时间从**第一次**停止请求起算：后续重发不刷新该时刻，
        # 否则持续超时的状态会让「停稳超时」永不触发（安全侧不接受这种掩盖）。
        self._stop_first_requested_at: Optional[float] = None
        self._requested_reasons: List[str] = []
        self.last_delivery: Optional[bool] = None
        self.events: List[Dict[str, object]] = []

    # -- 工具 -------------------------------------------------------------
    def _now(self, now: Optional[float] = None) -> float:
        return self._clock() if now is None else float(now)

    def _event(self, name: str, timestamp: float, *, reason: str = '',
               error_code: int = ERR_OK, message: str = '') -> Dict[str, object]:
        """构造结构化日志事件，字段名与 §10.2 对齐。"""
        return {
            'event': name,
            'timestamp': timestamp,
            'reason': reason,
            'error_code': int(error_code),
            'error_name': error_code_name(error_code),
            'stop_requested': bool(self.stop_requested),
            'stop_confirmed': bool(self.stop_confirmed),
            'request_delivered': self.last_delivery,
            'timeouts': self.config.as_dict(),
            'message': message,
        }

    def log_fields(self) -> Dict[str, object]:
        """§10.2 要求的最小日志字段集合。"""
        return {
            'stop_requested': self.stop_requested,
            'stop_confirmed': self.stop_confirmed,
            'timeouts': self.config.as_dict(),
        }

    # -- 状态输入 ---------------------------------------------------------
    def on_robot_state(self, msg, now: Optional[float] = None) -> Dict[str, object]:
        """处理 RobotState（duck typing，离线可用任意带同名字段的对象）。"""
        timestamp = self._now(now)
        self._last_robot_ts = timestamp
        communication_ok = bool(getattr(msg, 'communication_ok', False))
        safety_normal = bool(getattr(msg, 'safety_normal', True))
        robot_ready = bool(getattr(msg, 'robot_ready', False))
        motion_active = bool(getattr(msg, 'motion_active', False))
        stop_confirmed = bool(getattr(msg, 'stop_confirmed', False))
        controller_mode = str(getattr(msg, 'controller_mode', ''))
        simulation_mode = bool(getattr(msg, 'simulation_mode', False))
        detail = str(getattr(msg, 'detail', ''))
        self._robot_ok = communication_ok
        self._safety_normal = safety_normal

        # 只有执行层的独立证据可以置位 stop_confirmed。
        self.stop_confirmed = stop_confirmed
        if stop_confirmed:
            self.stop_confirm_timed_out = False
            self._stop_first_requested_at = None

        if self.guard is not None:
            self.guard.link_ok = communication_ok
            self.guard.set_controller_mode(controller_mode, simulation_mode, now=timestamp)
            derived = device_state_from_robot_state(
                communication_ok, robot_ready, motion_active, safety_normal,
                stop_confirmed, stop_requested=self.stop_requested,
                fault_latched=self.guard.fault_latched)
            self.guard.set_device_state(derived, now=timestamp)
            self.guard.note_stop_confirmed(stop_confirmed, 'robot_state')
        return self._event('robot_state', timestamp, message=detail)

    def on_tool_state(self, msg, now: Optional[float] = None) -> Dict[str, object]:
        """处理 ToolState：state 是估计，verified 是独立维度，不得混为一谈。"""
        timestamp = self._now(now)
        self._last_tool_ts = timestamp
        state = int(getattr(msg, 'state', TOOL_STATE_UNKNOWN))
        verified = bool(getattr(msg, 'verified', False))
        evidence_level = int(getattr(msg, 'evidence_level', TOOL_EVIDENCE_NONE))
        detail = str(getattr(msg, 'detail', ''))
        payload = payload_from_tool_state(state, verified, evidence_level)
        self._payload = payload

        if self.guard is not None:
            tracker = self.guard.payload_tracker
            if payload is PayloadState.CARRYING:
                tracker.mark_carrying(f'tool_state=ATTACHED, verified, evidence={evidence_level}')
            elif payload is PayloadState.NONE:
                tracker.mark_none(f'tool_state=DETACHED, verified, evidence={evidence_level}')
            else:
                tracker.mark_unknown(f'tool_state={state}, verified={verified}, '
                                     f'evidence={evidence_level}')
            if state == TOOL_STATE_FAULT:
                self.guard.fault_latch.latch(ERR_HARDWARE_FAULT,
                                             f'ToolState=FAULT：{detail}',
                                             context={'tool_state': state}, now=timestamp)
        return self._event('tool_state', timestamp, message=detail)

    def note_motion_state_unknown(self, unknown: bool = True,
                                  reason: str = '') -> Dict[str, object]:
        timestamp = self._now()
        self._motion_state_unknown = bool(unknown)
        if self.guard is not None and unknown:
            self.guard.note_motion_state_unknown(reason)
        return self._event('motion_state_unknown', timestamp,
                           reason=reason or 'motion_status_unknown',
                           error_code=ERR_MOTION_STATUS_UNKNOWN)

    def note_stop_delivery(self, delivered: Optional[bool], message: str = '',
                           now: Optional[float] = None) -> Dict[str, object]:
        """异步停止请求的回执到达（update only；不改变 stop_confirmed）。"""
        timestamp = self._now(now)
        self.last_delivery = None if delivered is None else bool(delivered)
        event = self._event('stop_delivery', timestamp, reason='stop_request_delivery',
                            message=message)
        self.events.append(event)
        return event

    # -- 停止请求与巡检 ---------------------------------------------------
    def request_stop(self, reason: str, now: Optional[float] = None,
                     force: bool = False) -> Optional[Dict[str, object]]:
        """发出停止请求。送达 != 停稳：本方法不会置位 stop_confirmed。"""
        timestamp = self._now(now)
        if reason in self._requested_reasons and not force:
            return None  # 同一原因持续存在时幂等，避免刷屏
        self._requested_reasons.append(reason)
        self.stop_requested = True
        self._stop_requested_at = timestamp
        if self._stop_first_requested_at is None:
            self._stop_first_requested_at = timestamp
        self.stop_confirmed = False  # 新的停止请求等价于「尚未确认停稳」
        delivered: Optional[bool] = None
        message = ''
        if self._requester is not None:
            try:
                delivered, message = self._requester(reason)
            except Exception as exc:  # noqa: BLE001 - 任何异常都按「送达未知」处理
                delivered, message = None, f'停止请求异常：{type(exc).__name__}: {exc}'
        self.last_delivery = delivered
        error_code = _STOP_REASON_ERROR_CODES.get(reason, ERR_SAFETY_INTERLOCK)
        event = self._event('stop_requested_event', timestamp, reason=reason,
                            error_code=error_code, message=message)
        self.events.append(event)
        if self.guard is not None and error_code == ERR_MOTION_STATUS_UNKNOWN:
            self.guard.note_motion_state_unknown(reason)
        return event

    def stale_reasons(self, now: Optional[float] = None) -> List[str]:
        """当前所有需要停止的原因（按固定顺序，便于测试与日志比对）。"""
        timestamp = self._now(now)
        reasons: List[str] = []
        if self._last_robot_ts is None:
            reasons.append('robot_state_never_received')
        elif timestamp - self._last_robot_ts > self.config.robot_state_timeout_sec:
            reasons.append('robot_state_timeout')
        if self._last_tool_ts is None:
            reasons.append('tool_state_never_received')
        elif timestamp - self._last_tool_ts > self.config.tool_state_timeout_sec:
            reasons.append('tool_state_timeout')
        if self._last_robot_ts is not None and not self._robot_ok:
            reasons.append('robot_communication_lost')
        if self._last_robot_ts is not None and not self._safety_normal:
            reasons.append('safety_normal_false')
        if self._motion_state_unknown:
            reasons.append('motion_status_unknown')
        return reasons

    def tick(self, now: Optional[float] = None) -> List[Dict[str, object]]:
        """周期性巡检：返回本轮新产生的事件（含停止请求与升级告警）。"""
        timestamp = self._now(now)
        events: List[Dict[str, object]] = []
        for reason in self.stale_reasons(timestamp):
            event = self.request_stop(reason, now=timestamp)
            if event is not None:
                events.append(event)

        if self.stop_requested and not self.stop_confirmed and \
                self._stop_first_requested_at is not None and not self.stop_confirm_timed_out:
            if timestamp - self._stop_first_requested_at > self.config.stop_confirm_timeout_sec:
                self.stop_confirm_timed_out = True
                event = self._event(
                    'stop_confirm_timeout', timestamp, reason='stop_confirm_timeout',
                    error_code=ERR_CANCEL_NOT_CONFIRMED,
                    message='停止请求已发出但超时仍未确认停稳：按 §7.1/§6.4 故障锁定')
                self.events.append(event)
                events.append(event)
                if self.guard is not None:
                    self.guard.fault_latch.latch(
                        ERR_CANCEL_NOT_CONFIRMED,
                        '停止请求送达后超时仍未确认停稳',
                        context={'timeouts': self.config.as_dict()}, now=timestamp)
        return events
