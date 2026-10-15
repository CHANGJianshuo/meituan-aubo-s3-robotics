"""运动执行核心逻辑（**不依赖 rclpy**）。

它把 V0.1 §6.3 的七步握手落成可测试的状态转换：
PREPARE → ACCEPT → VALIDATE → SEND → MONITOR → CONFIRM → STOP/FAULT。

关键安全语义：
- 只有同时满足到位、停稳、队列清空才返回 success=True；
- 后端拒绝（REJECTED，从未开始运动）才允许立即重发；
- 状态未知（STOP_UNKNOWN）时置位 unknown 标志，上层禁止自动重发；
- 取消不等于已停稳，只有 stop_confirmed 才敢声称安全停止。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from .backends import (
    Clock,
    ExecState,
    JointMoveRequest,
    JointMoveResult,
    MotionBackend,
    ValidationConfig,
    make_backend,
    validate_joint_move,
    ERR_MOTION_STATUS_UNKNOWN,
    ERR_OK,
)


@dataclass
class HandshakeRecord:
    """每一步握手的可追溯记录（V0.1 §10.2 要求的日志字段）。"""

    command_id: str
    steps: List[str] = field(default_factory=list)

    def mark(self, step: str, detail: str = '') -> None:
        self.steps.append(f'{step}:{detail}' if detail else step)


@dataclass
class ExecutorSnapshot:
    exec_state: ExecState
    active_command_id: Optional[str]
    last_error_code: int
    motion_state_unknown: bool
    stop_confirmed: bool


class MotionExecutorCore:
    """运动执行核心：唯一执行权 + 能力校验 + 到位确认。"""

    def __init__(self,
                 backend_name: str = 'mock',
                 config: Optional[ValidationConfig] = None,
                 clock: Optional[Clock] = None,
                 backend: Optional[MotionBackend] = None) -> None:
        self._clock = clock or Clock()
        self._config = config or ValidationConfig()
        self._backend = backend if backend is not None else make_backend(
            backend_name, config=self._config, clock=self._clock)
        self._exec_state = ExecState.IDLE
        self._active_command_id: Optional[str] = None
        self._last_error_code = ERR_OK
        self._motion_state_unknown = False
        self._stop_confirmed = True  # 初始静止
        self._handshakes: Dict[str, HandshakeRecord] = {}

    # -- 只读查询 ----------------------------------------------------------
    @property
    def backend(self) -> MotionBackend:
        return self._backend

    @property
    def exec_state(self) -> ExecState:
        return self._exec_state

    @property
    def motion_state_unknown(self) -> bool:
        return self._motion_state_unknown

    def capabilities(self) -> Dict[str, object]:
        return self._backend.capabilities().as_dict()

    def snapshot(self) -> ExecutorSnapshot:
        return ExecutorSnapshot(
            exec_state=self._exec_state,
            active_command_id=self._active_command_id,
            last_error_code=self._last_error_code,
            motion_state_unknown=self._motion_state_unknown,
            stop_confirmed=self._stop_confirmed,
        )

    def handshake(self, command_id: str) -> Optional[HandshakeRecord]:
        return self._handshakes.get(command_id)

    # -- 主流程 ------------------------------------------------------------
    def execute_joint_move(self,
                           command_id: str,
                           joint_names: Sequence[str],
                           target_rad: Sequence[float],
                           velocity_scaling: float,
                           acceleration_scaling: float,
                           timeout_ms: int) -> JointMoveResult:
        record = HandshakeRecord(command_id=command_id)
        self._handshakes[command_id] = record
        record.mark('PREPARE')

        # 状态未知时优先锁定：未确认状态前一律拒绝新动作。
        if self._motion_state_unknown:
            self._exec_state = ExecState.STOP_UNKNOWN
            self._last_error_code = ERR_MOTION_STATUS_UNKNOWN
            record.mark('PREPARE_REJECTED', 'motion_state_unknown')
            return JointMoveResult(
                success=False, exec_state=ExecState.STOP_UNKNOWN,
                error_code=ERR_MOTION_STATUS_UNKNOWN,
                message='上一次运动状态未知且未确认，已锁住新的运动请求；禁止自动重发',
                motion_state_unknown=True)

        request = JointMoveRequest(
            command_id=command_id,
            joint_names=tuple(joint_names),
            target_rad=tuple(float(v) for v in target_rad),
            velocity_scaling=float(velocity_scaling),
            acceleration_scaling=float(acceleration_scaling),
            timeout_ms=int(timeout_ms),
        )

        # ACCEPT：唯一执行权由后端 guard 与 core 双重把关。
        if self._active_command_id is not None:
            self._exec_state = ExecState.REJECTED
            self._last_error_code = 210
            record.mark('ACCEPT_REJECTED', f'in_flight={self._active_command_id}')
            return JointMoveResult(
                success=False, exec_state=ExecState.REJECTED, error_code=210,
                message=f'已有在途命令 {self._active_command_id}，拒绝并发运动请求 {command_id}')
        self._active_command_id = command_id
        record.mark('ACCEPT')

        # VALIDATE：能力与目标校验，能力不足直接拒绝而不是变形执行。
        ok, code, message = validate_joint_move(
            request, self._config, self._backend.capabilities())
        if not ok:
            self._active_command_id = None
            self._exec_state = ExecState.REJECTED
            self._last_error_code = code
            record.mark('VALIDATE_REJECTED', message)
            return JointMoveResult(success=False, exec_state=ExecState.REJECTED,
                                   error_code=code, message=message)
        record.mark('VALIDATE')

        # SEND + MONITOR + CONFIRM：交给后端实现，核心只解释结果。
        self._exec_state = ExecState.MOVING
        record.mark('SEND')
        record.mark('MONITOR_REQUEST', ExecState.MOVING.value)
        try:
            result = self._backend.execute_joint_move(request)
        except Exception as exc:  # 后端异常一律按状态未知处理，禁止重发
            self._active_command_id = None
            self._exec_state = ExecState.STOP_UNKNOWN
            self._last_error_code = ERR_MOTION_STATUS_UNKNOWN
            self._motion_state_unknown = True
            record.mark('MONITOR_EXCEPTION', type(exc).__name__)
            return JointMoveResult(
                success=False, exec_state=ExecState.STOP_UNKNOWN,
                error_code=ERR_MOTION_STATUS_UNKNOWN,
                message=f'后端执行异常，运动状态不确定：{type(exc).__name__}；禁止自动重发',
                motion_state_unknown=True)

        self._exec_state = result.exec_state
        self._last_error_code = result.error_code
        self._motion_state_unknown = bool(result.motion_state_unknown)
        self._stop_confirmed = bool(result.stop_confirmed)
        self._active_command_id = None
        record.mark('MONITOR', result.exec_state.value)
        if result.success:
            record.mark('CONFIRM', 'settled')
        else:
            record.mark('STOP_OR_FAULT', result.message)
        return result

    def request_stop(self, reason: str, in_flight_command_id: Optional[str] = None) -> Tuple[bool, str, bool]:
        """软件停止请求。

        返回 (请求是否送达, 说明, 是否已确认停稳)。
        **送达不等于停稳**：调用方不得据 delivered 直接宣告安全停止。

        `in_flight_command_id` 用于显式声明当前在途命令；不给时使用内部记录。
        """
        if in_flight_command_id is not None:
            self._active_command_id = in_flight_command_id
        delivered, message = self._backend.request_stop(reason)
        capabilities = self._backend.capabilities()
        if not capabilities.cancel_supported:
            # 后端不支持取消时，不能伪造 stop_confirmed。
            self._stop_confirmed = False
            self._motion_state_unknown = True
            self._exec_state = ExecState.STOP_UNKNOWN
            return False, f'后端 {capabilities.backend_name} 未声明 cancel_supported：{message}', False

        # 送达 != 停稳。显式标志优先；无标志可用时退回保守判断。
        flag_reader = getattr(self._backend, 'stop_confirmed_flag', None)
        if self._motion_state_unknown:
            # 运动状态未知时，任何停止请求都不能被宣告为“已确认停稳”。
            self._stop_confirmed = False
        elif callable(flag_reader):
            self._stop_confirmed = bool(delivered) and bool(flag_reader())
        elif self._active_command_id is None:
            self._stop_confirmed = bool(delivered)
        else:
            self._stop_confirmed = False
        if self._stop_confirmed:
            self._exec_state = ExecState.STOPPED
        else:
            self._exec_state = ExecState.CANCELLING
        return bool(delivered), message, self._stop_confirmed

    def clear_unknown_after_manual_confirmation(self, note: str) -> None:
        """仅在人工现场核实后调用：解除“状态未知”锁定。

        这是给人用的显式出口，任何自动流程都不得调用它。
        """
        self._motion_state_unknown = False
        self._exec_state = ExecState.IDLE
        self._stop_confirmed = True
        self._last_error_code = ERR_OK
        self._handshakes.setdefault('manual', HandshakeRecord('manual')).mark(
            'MANUAL_CONFIRMATION', note)
