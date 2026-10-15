"""mtc_task —— 整轮比赛任务状态机（Task FSM，第一层）。

包内模块职责
------------
* :mod:`mtc_task.task_validation` —— 纯 Python 任务口令规范化与合法性校验；
* :mod:`mtc_task.task_fsm`        —— 纯 Python 状态机核心（**不导入 rclpy**）；
* :mod:`mtc_task.task_executor`   —— rclpy Action Server 外壳（导入 rclpy，
  仅在真正运行 ROS 节点时需要）。

顶层只导出不依赖 ROS 的核心符号；``TaskExecutorNode`` 通过 ``__getattr__``
延迟导入，因此 ``import mtc_task`` 在离线环境下不会触碰 rclpy。
"""

from .task_fsm import (
    ActionKind,
    ActionRequest,
    ErrorCodes,
    Event,
    FsmsConfig,
    StepResult,
    TaskFsm,
    TaskOutcome,
    TaskRequest,
    TaskStateName,
    TaskStateSnapshot,
)
from .task_validation import (
    ALLOWED_SLOTS,
    MODE_BASIC,
    MODE_SEQUENCE,
    VALID_COLORS,
    ValidationResult,
    validate_task_request,
)

__all__ = [
    "TaskFsm",
    "TaskStateName",
    "Event",
    "ActionKind",
    "ActionRequest",
    "StepResult",
    "FsmsConfig",
    "TaskRequest",
    "TaskStateSnapshot",
    "TaskOutcome",
    "ErrorCodes",
    "ValidationResult",
    "validate_task_request",
    "MODE_BASIC",
    "MODE_SEQUENCE",
    "VALID_COLORS",
    "ALLOWED_SLOTS",
    "TaskExecutorNode",
]

__version__ = "0.1.0"

#: 需要 ROS 运行时才能导入的符号（延迟导入，避免离线测试触碰 rclpy）
_LAZY_ROS_SYMBOLS = frozenset({"TaskExecutorNode"})


def __getattr__(name: str):
    if name in _LAZY_ROS_SYMBOLS:
        from .task_executor import TaskExecutorNode

        return TaskExecutorNode
    raise AttributeError("module %r has no attribute %r" % (__name__, name))


def __dir__():
    return sorted(list(globals().keys()) + list(_LAZY_ROS_SYMBOLS))
