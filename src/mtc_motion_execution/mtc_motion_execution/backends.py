"""运动后端抽象与 Mock/Disabled 实现。

设计依据：docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md §6。

本模块**不依赖 rclpy**，因此可以在没有 ROS 2 环境的机器上直接做单元测试。
本模块不实现任何运动规划算法；规划结果由 mtc_motion_planning 提供。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, List, Optional, Sequence, Tuple


# ---------------------------------------------------------------------------
# 执行状态：严格区分“已接受 / 正在运动 / 已到位 / 取消中 / 停止确认 / 状态未知”
# ---------------------------------------------------------------------------
class ExecState(str, Enum):
    IDLE = 'IDLE'
    ACCEPTED = 'ACCEPTED'
    MOVING = 'MOVING'
    SETTLED = 'SETTLED'
    CANCELLING = 'CANCELLING'
    STOPPED = 'STOPPED'
    STOP_UNKNOWN = 'STOP_UNKNOWN'
    TIMEOUT = 'TIMEOUT'
    REJECTED = 'REJECTED'
    FAULT = 'FAULT'


# 错误码与 mtc_interfaces/msg/ErrorCodes.msg 保持一致。
# 这里重复定义是为了让本模块可以在没有 ROS 2 的环境下被测试；
# 若有 ROS 2 环境，motion_executor_node 会断言两者一致。
ERR_OK = 0
ERR_EXECUTION_REJECTED = 210
ERR_MOTION_TIMEOUT = 220
ERR_MOTION_STATUS_UNKNOWN = 230
ERR_SAFETY_INTERLOCK = 500
ERR_HARDWARE_FAULT = 510
ERR_CANCEL_NOT_CONFIRMED = 520


@dataclass(frozen=True)
class MotionCapabilities:
    """后端能力声明。

    设计约束（V0.1 §4.9）：capability 只表示**当前已连接且经实测启用**的通路能力，
    不能仅因为 SDK 宣称存在某方法就置 True。
    """

    backend_name: str = 'unknown'
    joint_goal_supported: bool = False
    timed_trajectory_supported: bool = False
    cartesian_motion_supported: bool = False
    cancel_supported: bool = False
    detail: str = ''

    def as_dict(self) -> Dict[str, object]:
        return {
            'backend_name': self.backend_name,
            'joint_goal_supported': self.joint_goal_supported,
            'timed_trajectory_supported': self.timed_trajectory_supported,
            'cartesian_motion_supported': self.cartesian_motion_supported,
            'cancel_supported': self.cancel_supported,
            'detail': self.detail,
        }


@dataclass
class JointMoveRequest:
    command_id: str
    joint_names: Sequence[str]
    target_rad: Sequence[float]
    velocity_scaling: float = 0.1
    acceleration_scaling: float = 0.1
    timeout_ms: int = 30_000


@dataclass
class JointMoveResult:
    success: bool
    exec_state: ExecState
    error_code: int
    message: str
    stop_confirmed: bool = False
    final_position_rad: Tuple[float, ...] = ()
    # 运动是否可能仍在进行 / 状态不确定。为 True 时上层**禁止**自动重发。
    motion_state_unknown: bool = False

    @property
    def retry_allowed_without_reobservation(self) -> bool:
        """只有明确被拒绝、且从未开始运动的请求才允许立即重发。"""
        return (not self.success) and self.exec_state is ExecState.REJECTED


@dataclass
class JointFeedback:
    position_rad: Tuple[float, ...] = ()
    velocity_rad_s: Tuple[float, ...] = ()
    stamp_s: float = 0.0


class MotionBackend:
    """运动后端抽象基类。

    实现者必须保证：
    - 单在途命令：同一时刻只接受一个未结束的 command_id；
    - 到位判定同时满足位置误差、持续静止与执行队列清空；
    - 取消/停止只表示“请求已送达”，必须另有 stop_confirmed 依据。
    """

    def capabilities(self) -> MotionCapabilities:  # pragma: no cover - 抽象
        raise NotImplementedError

    def execute_joint_move(self, request: JointMoveRequest) -> JointMoveResult:  # pragma: no cover
        raise NotImplementedError

    def request_stop(self, reason: str) -> Tuple[bool, str]:  # pragma: no cover - 抽象
        raise NotImplementedError

    def joint_feedback(self) -> JointFeedback:  # pragma: no cover - 抽象
        raise NotImplementedError


class Clock:
    """可注入时钟，便于离线测试压缩时间。"""

    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        if seconds > 0:
            time.sleep(seconds)


# ---------------------------------------------------------------------------
# 唯一执行权（single in-flight command）
# ---------------------------------------------------------------------------
class InFlightGuard:
    """保证同一时刻只有一个未结束的运动命令。

    设计依据：V0.1 §2“控制命令所有权”与 §7.2“正在执行运动时禁止第二个 Goal”。
    """

    def __init__(self) -> None:
        self._in_flight: Optional[str] = None

    @property
    def in_flight(self) -> Optional[str]:
        return self._in_flight

    def acquire(self, command_id: str) -> Tuple[bool, str]:
        if not command_id:
            return False, 'command_id 不能为空'
        if self._in_flight is not None:
            return False, f'已有在途命令 {self._in_flight}，拒绝并发运动请求 {command_id}'
        self._in_flight = command_id
        return True, 'acquired'

    def release(self, command_id: str) -> None:
        if self._in_flight == command_id:
            self._in_flight = None


# ---------------------------------------------------------------------------
# 目标校验
# ---------------------------------------------------------------------------
@dataclass
class ValidationConfig:
    expected_joint_names: Tuple[str, ...] = ()
    max_velocity_scaling: float = 0.2
    max_acceleration_scaling: float = 0.2
    joint_limit_rad: Tuple[Tuple[float, float], ...] = ()
    settle_position_tolerance_rad: float = 0.001
    settle_velocity_tolerance_rad_s: float = 0.001
    settle_duration_s: float = 0.2


def validate_joint_move(request: JointMoveRequest,
                        config: ValidationConfig,
                        capabilities: MotionCapabilities) -> Tuple[bool, int, str]:
    """执行前校验（V0.1 §6.3 的 PREPARE/VALIDATE 步）。

    返回 (是否通过, 错误码, 说明)。任何不通过都必须**拒绝**，
    不允许把不支持的运动表示悄悄“变形”成另一种表示去执行。
    """
    if not capabilities.joint_goal_supported:
        return False, ERR_EXECUTION_REJECTED, (
            f'后端 {capabilities.backend_name} 未声明 joint_goal_supported，拒绝执行关节目标运动')
    if len(request.joint_names) != len(request.target_rad):
        return False, ERR_EXECUTION_REJECTED, (
            'joint_names 与 target_rad 长度不一致：必须按索引一一对应')
    if len(request.joint_names) == 0:
        return False, ERR_EXECUTION_REJECTED, 'joint_names 不能为空'
    if len(set(request.joint_names)) != len(request.joint_names):
        return False, ERR_EXECUTION_REJECTED, 'joint_names 存在重复项'
    if config.expected_joint_names:
        if tuple(request.joint_names) != tuple(config.expected_joint_names):
            return False, ERR_EXECUTION_REJECTED, (
                'joint_names 与配置的关节顺序不一致，禁止假定固定 J1..J6 顺序：'
                f'得到 {tuple(request.joint_names)}，期望 {tuple(config.expected_joint_names)}')
    if request.timeout_ms <= 0:
        return False, ERR_EXECUTION_REJECTED, 'timeout_ms 必须大于 0'
    if not (0.0 < request.velocity_scaling <= config.max_velocity_scaling):
        return False, ERR_SAFETY_INTERLOCK, (
            f'velocity_scaling={request.velocity_scaling} 超出已审核上限 '
            f'{config.max_velocity_scaling}（未验收的高速动作禁止执行）')
    if not (0.0 < request.acceleration_scaling <= config.max_acceleration_scaling):
        return False, ERR_SAFETY_INTERLOCK, (
            f'acceleration_scaling={request.acceleration_scaling} 超出已审核上限 '
            f'{config.max_acceleration_scaling}')
    if config.joint_limit_rad:
        if len(config.joint_limit_rad) != len(request.target_rad):
            return False, ERR_EXECUTION_REJECTED, 'joint_limit_rad 维度与目标不一致'
        for name, value, (low, high) in zip(request.joint_names,
                                            request.target_rad,
                                            config.joint_limit_rad):
            if not (low <= value <= high):
                return False, ERR_SAFETY_INTERLOCK, (
                    f'关节 {name} 目标 {value} 超出限位 [{low}, {high}]')
    return True, ERR_OK, 'validated'


# ---------------------------------------------------------------------------
# Mock 后端
# ---------------------------------------------------------------------------
class MockMotionBackend(MotionBackend):
    """离线 Mock 运动后端。

    行为可配置，用于覆盖“到位 / 超时 / 取消已确认 / 取消未确认 / 状态未知”等路径。
    **它永远不会连接任何真实设备。**
    """

    def __init__(self,
                 config: Optional[ValidationConfig] = None,
                 clock: Optional[Clock] = None,
                 travel_time_s: float = 0.5,
                 timeout: bool = False,
                 status_unknown: bool = False,
                 cancel_confirms: bool = True,
                 fail_at: Optional[str] = None,
                 backend_name: str = 'mock') -> None:
        self._config = config or ValidationConfig(
            expected_joint_names=('shoulder_joint', 'upperArm_joint', 'foreArm_joint',
                                  'wrist1_joint', 'wrist2_joint', 'wrist3_joint'))
        self._clock = clock or Clock()
        self._guard = InFlightGuard()
        self._travel_time_s = travel_time_s
        self._timeout = timeout
        self._status_unknown = status_unknown
        self._cancel_confirms = cancel_confirms
        self._fail_at = fail_at
        self._backend_name = backend_name
        self._position = tuple(0.0 for _ in self._config.expected_joint_names)
        self._velocity = tuple(0.0 for _ in self._config.expected_joint_names)
        self._capabilities = MotionCapabilities(
            backend_name=backend_name,
            joint_goal_supported=True,
            timed_trajectory_supported=False,
            cartesian_motion_supported=False,
            cancel_supported=True,
            detail='离线 Mock 后端；不具备轨迹跟踪与笛卡尔运动能力',
        )
        self._cancel_requested = False
        self._stop_confirmed = False
        self._active_command: Optional[str] = None
        # 可观测的执行轨迹记录，便于测试断言“没有把轨迹拆成多次调用”。
        self.call_log: List[JointMoveRequest] = []

    # -- MotionBackend 接口 -------------------------------------------------
    def capabilities(self) -> MotionCapabilities:
        return self._capabilities

    def joint_feedback(self) -> JointFeedback:
        return JointFeedback(position_rad=tuple(self._position),
                             velocity_rad_s=tuple(self._velocity),
                             stamp_s=self._clock.now())

    def request_stop(self, reason: str) -> Tuple[bool, str]:
        """软件停止请求。送达本身不等于已停稳。

        返回的 `stop_confirmed` 由 `stop_confirmed_flag()` 单独暴露，
        调用方**不得**依赖消息文本判断是否停稳。
        """
        if self._active_command is None:
            self._stop_confirmed = True
            return True, f'无在途命令，已确认停稳（原因：{reason}）'
        self._cancel_requested = True
        self._stop_confirmed = bool(self._cancel_confirms)
        if self._stop_confirmed:
            self._velocity = tuple(0.0 for _ in self._velocity)
            self._guard.release(self._active_command)
            return True, f'停止请求已送达并确认停稳（原因：{reason}）'
        return True, f'停止请求已送达，但尚未确认停稳（原因：{reason}）'

    def stop_confirmed_flag(self) -> bool:
        """最近一次停止请求是否已被确认停稳（显式语义，供上层判断）。"""
        return bool(self._stop_confirmed)

    def execute_joint_move(self, request: JointMoveRequest) -> JointMoveResult:
        self.call_log.append(request)
        ok, code, message = validate_joint_move(request, self._config, self._capabilities)
        if not ok:
            return JointMoveResult(success=False, exec_state=ExecState.REJECTED,
                                   error_code=code, message=message,
                                   final_position_rad=tuple(self._position))

        acquired, why = self._guard.acquire(request.command_id)
        if not acquired:
            return JointMoveResult(success=False, exec_state=ExecState.REJECTED,
                                   error_code=ERR_EXECUTION_REJECTED, message=why,
                                   final_position_rad=tuple(self._position))

        self._active_command = request.command_id
        self._cancel_requested = False
        self._stop_confirmed = False

        if self._fail_at == 'status_unknown' or self._status_unknown:
            # 反馈断流：状态不确定，必须锁定动作，禁止自动重发。
            self._guard.release(request.command_id)
            self._active_command = None
            return JointMoveResult(
                success=False, exec_state=ExecState.STOP_UNKNOWN,
                error_code=ERR_MOTION_STATUS_UNKNOWN,
                message='反馈断流，运动状态不确定；已锁定新的运动请求，禁止自动重发',
                stop_confirmed=False, final_position_rad=tuple(self._position),
                motion_state_unknown=True)

        budget_s = request.timeout_ms / 1000.0
        simulated_s = min(self._travel_time_s, budget_s)
        if self._timeout or self._travel_time_s > budget_s:
            elapsed = 0.0
            while elapsed < budget_s:
                step = min(0.05, budget_s - elapsed)
                self._clock.sleep(step)
                elapsed += step
                if self._cancel_requested:
                    break
            if self._cancel_requested:
                self._guard.release(request.command_id)
                self._active_command = None
                return JointMoveResult(
                    success=False,
                    exec_state=ExecState.STOPPED if self._stop_confirmed else ExecState.STOP_UNKNOWN,
                    error_code=ERR_OK if self._stop_confirmed else ERR_CANCEL_NOT_CONFIRMED,
                    message=('已取消并确认停稳' if self._stop_confirmed
                             else '已取消，但后端未确认停稳'),
                    stop_confirmed=self._stop_confirmed,
                    final_position_rad=tuple(self._position),
                    motion_state_unknown=not self._stop_confirmed)
            self._guard.release(request.command_id)
            self._active_command = None
            return JointMoveResult(
                success=False, exec_state=ExecState.TIMEOUT, error_code=ERR_MOTION_TIMEOUT,
                message=f'运动超时（预算 {request.timeout_ms} ms）',
                stop_confirmed=False, final_position_rad=tuple(self._position),
                motion_state_unknown=True)

        # 逐步“运动”：中途允许被取消。
        steps = max(1, int(simulated_s / 0.05))
        for _ in range(steps):
            if self._cancel_requested:
                break
            self._clock.sleep(simulated_s / steps)
            self._velocity = tuple(
                (t - p) / max(simulated_s / steps, 1e-9)
                for p, t in zip(self._position, request.target_rad))

        if self._cancel_requested:
            self._guard.release(request.command_id)
            self._active_command = None
            return JointMoveResult(
                success=False,
                exec_state=ExecState.STOPPED if self._stop_confirmed else ExecState.STOP_UNKNOWN,
                error_code=ERR_OK if self._stop_confirmed else ERR_CANCEL_NOT_CONFIRMED,
                message=('已取消并确认停稳' if self._stop_confirmed
                         else '已取消，但后端未确认停稳'),
                stop_confirmed=self._stop_confirmed,
                final_position_rad=tuple(self._position),
                motion_state_unknown=not self._stop_confirmed)

        self._position = tuple(float(v) for v in request.target_rad)
        self._velocity = tuple(0.0 for _ in self._position)
        self._guard.release(request.command_id)
        self._active_command = None
        return JointMoveResult(
            success=True, exec_state=ExecState.SETTLED, error_code=ERR_OK,
            message='已到位并停稳', stop_confirmed=True,
            final_position_rad=tuple(self._position))


class DisabledMotionBackend(MotionBackend):
    """真实后端占位：始终拒绝，绝不产生任何真实运动。

    任何试图在未授权情况下驱动真实设备的调用都会在这里被明确拒绝，
    而不是静默成功或抛出难以定位的异常。
    """

    def __init__(self, reason: str = '真实运动后端默认禁用，需现场明确授权与实测验证') -> None:
        self._reason = reason

    def capabilities(self) -> MotionCapabilities:
        return MotionCapabilities(
            backend_name='disabled',
            joint_goal_supported=False,
            timed_trajectory_supported=False,
            cartesian_motion_supported=False,
            cancel_supported=False,
            detail=self._reason,
        )

    def execute_joint_move(self, request: JointMoveRequest) -> JointMoveResult:
        return JointMoveResult(success=False, exec_state=ExecState.REJECTED,
                               error_code=ERR_EXECUTION_REJECTED,
                               message=f'{self._reason}（command_id={request.command_id}）')

    def request_stop(self, reason: str) -> Tuple[bool, str]:
        return False, f'{self._reason}；未送达任何真实停止请求（原因：{reason}）'

    def joint_feedback(self) -> JointFeedback:
        return JointFeedback()


def make_backend(name: str,
                 config: Optional[ValidationConfig] = None,
                 clock: Optional[Clock] = None) -> MotionBackend:
    """按名称构造后端。未知名称一律回到 disabled，绝不默认连真实设备。"""
    normalized = (name or '').strip().lower()
    if normalized == 'mock':
        return MockMotionBackend(config=config, clock=clock)
    return DisabledMotionBackend(
        reason=f'不支持或未启用的运动后端 "{name}"；仅 mock 可用于离线运行')
