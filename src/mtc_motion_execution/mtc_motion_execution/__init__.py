"""mtc_motion_execution —— 运动执行抽象层（离线 Mock 默认）。

本包只负责“执行已规划的受控运动请求”，不实现运动规划算法。
真实 AUBO 后端在获得现场明确授权与实测验证前始终 disabled。
"""

from .backends import (  # noqa: F401
    Clock,
    DisabledMotionBackend,
    ExecState,
    InFlightGuard,
    JointFeedback,
    JointMoveRequest,
    JointMoveResult,
    MockMotionBackend,
    MotionBackend,
    MotionCapabilities,
    ValidationConfig,
    make_backend,
    validate_joint_move,
)
from .executor_core import MotionExecutorCore  # noqa: F401

__all__ = [
    'Clock',
    'DisabledMotionBackend',
    'ExecState',
    'InFlightGuard',
    'JointFeedback',
    'JointMoveRequest',
    'JointMoveResult',
    'MockMotionBackend',
    'MotionBackend',
    'MotionCapabilities',
    'MotionExecutorCore',
    'ValidationConfig',
    'make_backend',
    'validate_joint_move',
]
