"""mtc_tool —— V1 舌规 / V2 电磁锁止末端策略的统一工具层。

包结构与职责::

    mtc_tool.codes        # ToolState.msg / ErrorCodes.msg 的纯 Python 数值镜像
    mtc_tool.strategy     # ToolStrategy 抽象 + MotionPrimitive + V1/V2 实现（无 ROS、无 IO）
    mtc_tool.io_port      # UnlockIoPort 抽象 + MockUnlockIo + DisabledUnlockIo（默认禁用真实 IO）
    mtc_tool.manager      # ToolManager：状态机 + 证据等级 + 解锁联锁 + 错误码
    mtc_tool.tool_manager_node  # rclpy 外壳（仅 ROS 环境需要；核心模块不依赖 ROS）

安全默认值：真实电磁 IO **禁用**；工具层只输出 ``MotionPrimitive``，
不产生关节命令、不调用任何机器人 SDK。
"""

from .codes import (
    ERROR_ATTACH_NOT_VERIFIED,
    ERROR_ENGAGE_FAILED,
    ERROR_EXECUTION_REJECTED,
    ERROR_OK,
    ERROR_RELEASE_NOT_VERIFIED,
    ERROR_SAFETY_INTERLOCK,
    ERROR_SEAT_NOT_VERIFIED,
    ERROR_UNLOCK_FAILED,
    EVIDENCE_COMMAND_ONLY,
    EVIDENCE_GEOMETRY,
    EVIDENCE_NONE,
    EVIDENCE_SENSOR_OR_VISION,
    MIN_INDEPENDENT_EVIDENCE,
    STATE_ATTACHED,
    STATE_DETACHED,
    STATE_ENGAGING,
    STATE_FAULT,
    STATE_RELEASING,
    STATE_UNKNOWN,
    TOOL_TYPE_MAGNETIC_LATCH_V2,
    TOOL_TYPE_PASSIVE_HOOK_V1,
)
from .io_port import (
    DisabledUnlockIo,
    MockUnlockIo,
    UnlockIoDisabledError,
    UnlockIoPort,
)
from .manager import (
    PlanResult,
    ToolManager,
    ToolStatus,
    UnlockOutcome,
    WithdrawDecision,
    is_primitive_sequence,
)
from .strategy import (
    MagneticLatchParams,
    MagneticLatchV2,
    MotionPrimitive,
    PassiveHookParams,
    PassiveHookV1,
    PoseTarget,
    ToolStrategy,
)

__all__ = [
    # codes
    "STATE_UNKNOWN",
    "STATE_DETACHED",
    "STATE_ENGAGING",
    "STATE_ATTACHED",
    "STATE_RELEASING",
    "STATE_FAULT",
    "EVIDENCE_NONE",
    "EVIDENCE_COMMAND_ONLY",
    "EVIDENCE_GEOMETRY",
    "EVIDENCE_SENSOR_OR_VISION",
    "MIN_INDEPENDENT_EVIDENCE",
    "ERROR_OK",
    "ERROR_ENGAGE_FAILED",
    "ERROR_ATTACH_NOT_VERIFIED",
    "ERROR_SEAT_NOT_VERIFIED",
    "ERROR_UNLOCK_FAILED",
    "ERROR_RELEASE_NOT_VERIFIED",
    "ERROR_SAFETY_INTERLOCK",
    "ERROR_EXECUTION_REJECTED",
    "TOOL_TYPE_PASSIVE_HOOK_V1",
    "TOOL_TYPE_MAGNETIC_LATCH_V2",
    # strategy
    "PoseTarget",
    "MotionPrimitive",
    "ToolStrategy",
    "PassiveHookParams",
    "PassiveHookV1",
    "MagneticLatchParams",
    "MagneticLatchV2",
    # io
    "UnlockIoPort",
    "MockUnlockIo",
    "DisabledUnlockIo",
    "UnlockIoDisabledError",
    # manager
    "ToolManager",
    "ToolStatus",
    "PlanResult",
    "UnlockOutcome",
    "WithdrawDecision",
    "is_primitive_sequence",
]

__version__ = "0.1.0"
