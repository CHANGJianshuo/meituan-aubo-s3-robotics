"""AUBO S3 SDK 桥接核心（**不依赖 rclpy，不导入 pyaubo**）。

设计依据：`docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md` §6。

本模块只定义**契约与安全语义**，不实现真实 SDK 调用。真实通路必须由
独立的 Python 3.10 SDK Worker 提供（路线 B），或经实测验证的原生 ROS 2 驱动（路线 A）。

安全默认值：
- `AuboBridge` 的默认实现是 `DisabledSdkWorker`，任何业务调用都会返回明确的拒绝；
- 不存在“看起来可用但静默无操作”的中间状态——拒绝必须有明确错误码与原因。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

# 与 mtc_interfaces/msg/ErrorCodes.msg 保持一致（本模块需可脱离 ROS 2 测试）
ERR_OK = 0
ERR_EXECUTION_REJECTED = 210
ERR_MOTION_STATUS_UNKNOWN = 230
ERR_HARDWARE_FAULT = 510
ERR_SAFETY_INTERLOCK = 500


# ---------------------------------------------------------------------------
# 身份与能力
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class RobotIdentity:
    """机器人身份。用于 PREPARE 步的“是不是同一台设备”校验。"""

    controller_model: str = ''
    serial_number: str = ''
    arcs_version: str = ''
    sdk_version: str = ''
    interface_version: str = ''
    verified: bool = False

    def matches(self, expected: 'RobotIdentity') -> bool:
        """白名单校验：序列号为空或未核实时一律视为不匹配。"""
        if not self.verified or not expected.verified:
            return False
        if not expected.serial_number or not self.serial_number:
            return False
        return (self.serial_number == expected.serial_number
                and self.controller_model == expected.controller_model)

    def as_dict(self) -> Dict[str, object]:
        return {
            'controller_model': self.controller_model,
            'serial_number': self.serial_number,
            'arcs_version': self.arcs_version,
            'sdk_version': self.sdk_version,
            'interface_version': self.interface_version,
            'verified': self.verified,
        }


@dataclass(frozen=True)
class BridgeCapabilities:
    """Bridge 对外声明的能力。

    依据 V0.1 §4.9/§6.1：能力只代表**当前已连接且实测启用**的通路：
    - 历史已实测：只读身份/状态查询、单组六关节目标移动（仅 J6 实测）、RTDE 反馈订阅；
    - **未**实测：完整时间参数化轨迹跟踪、受约束笛卡尔运动、V2 末端 I/O 与锁止反馈。
    """

    read_only_state_supported: bool = False
    joint_goal_supported: bool = False
    timed_trajectory_supported: bool = False
    cartesian_motion_supported: bool = False
    cancel_supported: bool = False
    tool_io_supported: bool = False
    worker_name: str = 'disabled'
    detail: str = ''

    def as_dict(self) -> Dict[str, object]:
        return {
            'worker_name': self.worker_name,
            'read_only_state_supported': self.read_only_state_supported,
            'joint_goal_supported': self.joint_goal_supported,
            'timed_trajectory_supported': self.timed_trajectory_supported,
            'cartesian_motion_supported': self.cartesian_motion_supported,
            'cancel_supported': self.cancel_supported,
            'tool_io_supported': self.tool_io_supported,
            'detail': self.detail,
        }


@dataclass
class HealthStatus:
    """RPC/RTDE 健康度与数据新鲜度。"""

    rpc_ok: bool = False
    rtde_ok: bool = False
    last_feedback_age_s: float = float('inf')
    feedback_freshness_limit_s: float = 0.5
    detail: str = ''

    @property
    def feedback_is_fresh(self) -> bool:
        return self.rtde_ok and self.last_feedback_age_s <= self.feedback_freshness_limit_s

    @property
    def usable_for_motion(self) -> bool:
        return self.rpc_ok and self.rtde_ok and self.feedback_is_fresh


@dataclass
class JointStateSnapshot:
    position_rad: Tuple[float, ...] = ()
    velocity_rad_s: Tuple[float, ...] = ()
    stamp_s: float = 0.0
    stale: bool = True


@dataclass
class WorkerCommandResult:
    accepted: bool
    error_code: int
    message: str
    command_id: str = ''
    motion_state_unknown: bool = False
    stop_confirmed: bool = False
    # Worker 是否已把“状态未知”在设备侧对账清楚（例如重新读到有效关节反馈、
    # 确认执行队列为空）。仅 stop_confirmed=True 不足以清除不确定状态锁定。
    state_reconciled: bool = False


# ---------------------------------------------------------------------------
# SDK Worker 契约
# ---------------------------------------------------------------------------
class SdkWorker:
    """SDK Worker 契约。

    真实实现应运行在**独立 Python 3.10 进程**中，通过本机 IPC（Unix Domain Socket）
    与 ROS 2 Jazzy（Python 3.12）侧通信；两侧**不得**混装不兼容的 Python 二进制模块。
    """

    def identity(self) -> RobotIdentity:  # pragma: no cover - 抽象
        raise NotImplementedError

    def capabilities(self) -> BridgeCapabilities:  # pragma: no cover - 抽象
        raise NotImplementedError

    def health(self) -> HealthStatus:  # pragma: no cover - 抽象
        raise NotImplementedError

    def joint_state(self) -> JointStateSnapshot:  # pragma: no cover - 抽象
        raise NotImplementedError

    def send_joint_move(self, command_id: str, joint_names: Sequence[str],
                        target_rad: Sequence[float], velocity_scaling: float,
                        acceleration_scaling: float, timeout_ms: int) -> WorkerCommandResult:
        raise NotImplementedError  # pragma: no cover - 抽象

    def request_stop(self, reason: str) -> WorkerCommandResult:  # pragma: no cover - 抽象
        raise NotImplementedError

    def send_tool_unlock_pulse(self, request_id: str, pulse_ms: int) -> WorkerCommandResult:
        raise NotImplementedError  # pragma: no cover - 抽象


class DisabledSdkWorker(SdkWorker):
    """默认 Worker：**不连接任何设备**，一切业务调用都明确拒绝。"""

    def __init__(self, reason: str = '真实 AUBO S3 通路默认禁用：未获现场明确授权与实测验证') -> None:
        self._reason = reason
        self.call_log: List[str] = []

    def identity(self) -> RobotIdentity:
        self.call_log.append('identity')
        return RobotIdentity(verified=False)

    def capabilities(self) -> BridgeCapabilities:
        return BridgeCapabilities(
            worker_name='disabled',
            read_only_state_supported=False,
            joint_goal_supported=False,
            timed_trajectory_supported=False,
            cartesian_motion_supported=False,
            cancel_supported=False,
            tool_io_supported=False,
            detail=self._reason,
        )

    def health(self) -> HealthStatus:
        return HealthStatus(rpc_ok=False, rtde_ok=False, detail=self._reason)

    def joint_state(self) -> JointStateSnapshot:
        return JointStateSnapshot(stale=True)

    def send_joint_move(self, command_id, joint_names, target_rad,
                        velocity_scaling, acceleration_scaling, timeout_ms):
        self.call_log.append(f'send_joint_move:{command_id}')
        return WorkerCommandResult(accepted=False, error_code=ERR_EXECUTION_REJECTED,
                                   message=self._reason, command_id=command_id)

    def request_stop(self, reason: str) -> WorkerCommandResult:
        self.call_log.append(f'request_stop:{reason}')
        return WorkerCommandResult(
            accepted=False, error_code=ERR_EXECUTION_REJECTED,
            message=f'{self._reason}；未向任何设备发送停止请求',
            stop_confirmed=False)

    def send_tool_unlock_pulse(self, request_id: str, pulse_ms: int) -> WorkerCommandResult:
        self.call_log.append(f'unlock_pulse:{request_id}')
        return WorkerCommandResult(
            accepted=False, error_code=ERR_SAFETY_INTERLOCK,
            message=f'{self._reason}；未触发任何电磁铁输出', command_id=request_id)


class FakeSdkWorker(SdkWorker):
    """离线 Fake Worker：用内存状态模拟，供集成测试使用。

    **它不连接任何设备。** 所有能力声明都必须由测试显式开启，
    默认全部关闭，以此保证“默认不可运动”。
    """

    def __init__(self,
                 identity: Optional[RobotIdentity] = None,
                 capabilities: Optional[BridgeCapabilities] = None,
                 health: Optional[HealthStatus] = None,
                 fail_motion: bool = False,
                 motion_state_unknown: bool = False,
                 stop_confirms: bool = True) -> None:
        self._identity = identity or RobotIdentity(
            controller_model='AUBO-S3', serial_number='FAKE-SN',
            arcs_version='0.0.0-fake', sdk_version='0.0.0-fake',
            interface_version='0.0.0-fake', verified=True)
        self._capabilities = capabilities or BridgeCapabilities(
            worker_name='fake',
            read_only_state_supported=True,
            joint_goal_supported=True,
            timed_trajectory_supported=False,
            cartesian_motion_supported=False,
            cancel_supported=True,
            tool_io_supported=False,
            detail='离线 Fake Worker，仅用于测试')
        self._health = health or HealthStatus(
            rpc_ok=True, rtde_ok=True, last_feedback_age_s=0.05)
        self._fail_motion = fail_motion
        self._unknown = motion_state_unknown
        self._stop_confirms = stop_confirms
        self._in_flight: Optional[str] = None
        self._state_unknown = False
        self.call_log: List[str] = []

    def identity(self) -> RobotIdentity:
        self.call_log.append('identity')
        return self._identity

    def capabilities(self) -> BridgeCapabilities:
        return self._capabilities

    def health(self) -> HealthStatus:
        return self._health

    def joint_state(self) -> JointStateSnapshot:
        return JointStateSnapshot(position_rad=(0.0,) * 6,
                                  velocity_rad_s=(0.0,) * 6,
                                  stamp_s=0.0, stale=not self._health.rtde_ok)

    def send_joint_move(self, command_id, joint_names, target_rad,
                        velocity_scaling, acceleration_scaling, timeout_ms):
        self.call_log.append(f'send_joint_move:{command_id}')
        if self._in_flight is not None:
            return WorkerCommandResult(
                accepted=False, error_code=ERR_EXECUTION_REJECTED,
                message=f'Worker 已有在途命令 {self._in_flight}',
                command_id=command_id)
        self._in_flight = command_id
        try:
            if self._unknown:
                self._state_unknown = True
                return WorkerCommandResult(
                    accepted=True, error_code=ERR_MOTION_STATUS_UNKNOWN,
                    message='RTDE 反馈断流，运动状态不确定',
                    command_id=command_id, motion_state_unknown=True)
            if self._fail_motion:
                return WorkerCommandResult(
                    accepted=True, error_code=ERR_HARDWARE_FAULT,
                    message='Fake Worker 模拟运动失败', command_id=command_id)
            return WorkerCommandResult(
                accepted=True, error_code=ERR_OK,
                message='已到位', command_id=command_id, stop_confirmed=True)
        finally:
            self._in_flight = None

    def request_stop(self, reason: str) -> WorkerCommandResult:
        self.call_log.append(f'request_stop:{reason}')
        if self._in_flight is None:
            # 无在途命令时也要尊重本 Worker 的“是否确认停稳”配置，
            # 以及是否仍处于未对账的“状态未知”。
            confirmed = bool(self._stop_confirms)
            reconciled = confirmed and not self._state_unknown
            if reconciled:
                self._state_unknown = False
            return WorkerCommandResult(
                accepted=True, error_code=ERR_OK,
                message=('无在途命令，已确认停稳' if confirmed
                         else '无在途命令，但未确认停稳（模拟未确认场景）'),
                stop_confirmed=confirmed,
                state_reconciled=reconciled)
        # 处于“状态未知”时，Worker 不能仅凭一次停止请求宣告已对账：
        # 必须先重新取得有效反馈并确认执行队列为空。
        reconciled = bool(self._stop_confirms) and not self._state_unknown
        if reconciled:
            self._state_unknown = False
        return WorkerCommandResult(
            accepted=True, error_code=ERR_OK,
            message=('停止请求已送达并确认停稳' if self._stop_confirms
                     else '停止请求已送达，但尚未确认停稳'),
            stop_confirmed=bool(self._stop_confirms),
            state_reconciled=reconciled)

    def send_tool_unlock_pulse(self, request_id: str, pulse_ms: int) -> WorkerCommandResult:
        self.call_log.append(f'unlock_pulse:{request_id}')
        if not self._capabilities.tool_io_supported:
            return WorkerCommandResult(
                accepted=False, error_code=ERR_SAFETY_INTERLOCK,
                message='未声明 tool_io_supported，拒绝触发解锁脉冲',
                command_id=request_id)
        return WorkerCommandResult(accepted=True, error_code=ERR_OK,
                                   message='解锁脉冲请求已受理（不代表已解锁）',
                                   command_id=request_id)


# ---------------------------------------------------------------------------
# Bridge：对上层提供受控、可审计的门面
# ---------------------------------------------------------------------------
class AuboBridge:
    """Bridge 门面：身份校验、健康检查、单在途、命令 ID 唯一性、不确定状态锁定。

    这些约束与 `mtc_motion_execution` 的语义一致，属于**防御性重复**：
    即使上层出错，Bridge 也必须拒绝危险请求。
    """

    def __init__(self,
                 worker: Optional[SdkWorker] = None,
                 expected_identity: Optional[RobotIdentity] = None,
                 command_id_retention: int = 64) -> None:
        self._worker = worker if worker is not None else DisabledSdkWorker()
        self._expected_identity = expected_identity or RobotIdentity(
            controller_model='AUBO-S3', serial_number='', verified=False)
        self._seen_command_ids: List[str] = []
        self._retention = command_id_retention
        self._in_flight: Optional[str] = None
        self._motion_state_unknown = False
        self._last_stop_confirmed = True
        self._audit: List[Dict[str, object]] = []

    # -- 只读 -------------------------------------------------------------
    @property
    def worker(self) -> SdkWorker:
        return self._worker

    @property
    def motion_state_unknown(self) -> bool:
        return self._motion_state_unknown

    @property
    def stop_confirmed(self) -> bool:
        return self._last_stop_confirmed

    def audit_trail(self) -> List[Dict[str, object]]:
        return list(self._audit)

    def capabilities(self) -> BridgeCapabilities:
        return self._worker.capabilities()

    def identity_check(self) -> Tuple[bool, str]:
        """身份白名单校验。未配置期望序列号时视为**不通过**（安全默认）。"""
        actual = self._worker.identity()
        if not self._expected_identity.verified or not self._expected_identity.serial_number:
            return False, ('未配置经核实的期望设备身份（序列号白名单为空），'
                           '按安全默认拒绝建立运动通路')
        if not actual.matches(self._expected_identity):
            return False, (f'设备身份不匹配：实际 serial={actual.serial_number or "<空>"} '
                           f'model={actual.controller_model or "<空>"}')
        return True, '身份校验通过'

    def health_check(self) -> Tuple[bool, str]:
        health = self._worker.health()
        if not health.rpc_ok:
            return False, 'RPC 不可用'
        if not health.rtde_ok:
            return False, 'RTDE 不可用'
        if not health.feedback_is_fresh:
            return False, (f'反馈不新鲜：age={health.last_feedback_age_s}s > '
                           f'limit={health.feedback_freshness_limit_s}s')
        return True, '健康检查通过'

    # -- 命令 -------------------------------------------------------------
    def send_joint_move(self, command_id: str, joint_names: Sequence[str],
                        target_rad: Sequence[float], velocity_scaling: float,
                        acceleration_scaling: float, timeout_ms: int) -> WorkerCommandResult:
        self._audit.append({'event': 'command_intent', 'command_id': command_id})

        if not command_id:
            return self._reject(command_id, 'command_id 不能为空')
        if command_id in self._seen_command_ids:
            return self._reject(command_id, f'command_id 重复：{command_id}（禁止重复执行同一命令）')
        if self._motion_state_unknown:
            return self._reject(command_id, '上一运动状态未知且未确认，Bridge 锁定新的运动请求')
        if self._in_flight is not None:
            return self._reject(command_id, f'Bridge 已有在途命令 {self._in_flight}')

        ok, why = self.identity_check()
        if not ok:
            return self._reject(command_id, why)
        ok, why = self.health_check()
        if not ok:
            return self._reject(command_id, why)

        capabilities = self._worker.capabilities()
        if not capabilities.joint_goal_supported:
            return self._reject(command_id, 'Worker 未声明 joint_goal_supported')

        self._remember(command_id)
        self._in_flight = command_id
        try:
            result = self._worker.send_joint_move(
                command_id, list(joint_names), list(target_rad),
                velocity_scaling, acceleration_scaling, timeout_ms)
        finally:
            self._in_flight = None

        if result.motion_state_unknown:
            self._motion_state_unknown = True
            self._last_stop_confirmed = False
        elif result.accepted and result.error_code == ERR_OK:
            self._last_stop_confirmed = True
        self._audit.append({'event': 'sdk_response', 'command_id': command_id,
                            'accepted': result.accepted,
                            'error_code': result.error_code,
                            'motion_state_unknown': result.motion_state_unknown})
        self._audit.append({'event': 'result', 'command_id': command_id,
                            'message': result.message})
        return result

    def request_stop(self, reason: str) -> WorkerCommandResult:
        self._audit.append({'event': 'stop_requested', 'reason': reason})
        result = self._worker.request_stop(reason)
        self._last_stop_confirmed = bool(result.stop_confirmed)
        # 只有在途运动已停稳 **且** 设备侧状态已对账，才允许自动解除不确定锁定；
        # 否则必须保持锁定，等待人工现场确认（clear_unknown_after_manual_confirmation）。
        if self._last_stop_confirmed and bool(result.state_reconciled):
            self._motion_state_unknown = False
        self._audit.append({'event': 'stop_confirmed' if self._last_stop_confirmed
                            else 'stop_not_confirmed', 'message': result.message})
        return result

    def send_tool_unlock_pulse(self, request_id: str, pulse_ms: int,
                               seated_verified: bool, unloaded: bool) -> WorkerCommandResult:
        """V2 解锁脉冲。前置条件必须在 Bridge 层**再校验一次**。

        `accepted=True` **不代表**电池已释放；释放必须另有独立证据。
        """
        self._audit.append({'event': 'unlock_intent', 'request_id': request_id})
        if not seated_verified:
            return WorkerCommandResult(
                accepted=False, error_code=ERR_SAFETY_INTERLOCK,
                message='拒绝解锁：未确认电池已由台面承托（落座未验证）',
                command_id=request_id)
        if not unloaded:
            return WorkerCommandResult(
                accepted=False, error_code=ERR_SAFETY_INTERLOCK,
                message='拒绝解锁：载荷尚未卸除', command_id=request_id)
        if pulse_ms <= 0:
            return WorkerCommandResult(False, ERR_EXECUTION_REJECTED,
                                       'pulse_ms 必须大于 0', command_id=request_id)
        result = self._worker.send_tool_unlock_pulse(request_id, pulse_ms)
        self._audit.append({'event': 'unlock_response', 'request_id': request_id,
                            'accepted': result.accepted, 'message': result.message})
        return result

    def clear_unknown_after_manual_confirmation(self, note: str) -> None:
        """仅限现场人工核实后调用：解除不确定状态锁定。"""
        self._motion_state_unknown = False
        self._last_stop_confirmed = True
        self._audit.append({'event': 'manual_confirmation', 'note': note})

    # -- 内部 -------------------------------------------------------------
    def _reject(self, command_id: str, message: str) -> WorkerCommandResult:
        self._audit.append({'event': 'command_rejected', 'command_id': command_id,
                            'message': message})
        return WorkerCommandResult(accepted=False, error_code=ERR_EXECUTION_REJECTED,
                                   message=message, command_id=command_id)

    def _remember(self, command_id: str) -> None:
        self._seen_command_ids.append(command_id)
        if len(self._seen_command_ids) > self._retention:
            self._seen_command_ids = self._seen_command_ids[-self._retention:]


def make_bridge(mode: str = 'disabled', **kwargs) -> AuboBridge:
    """按模式构造 Bridge。

    - `disabled`（默认）：不连接任何设备，一切运动/IO 请求被拒绝；
    - `fake`：离线 Fake Worker，供测试；
    - 其他取值：一律落到 `disabled`（**绝不**因为拼写或扩展名而启用真实通路）。
    """
    normalized = (mode or '').strip().lower()
    if normalized == 'fake':
        return AuboBridge(worker=FakeSdkWorker(**kwargs))
    return AuboBridge(worker=DisabledSdkWorker())
