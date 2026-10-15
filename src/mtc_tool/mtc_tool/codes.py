"""mtc_tool 纯 Python 常量层。

本模块是 ROSIDL 定义的**数值镜像**，不 import 任何 ROS 2 / SDK 依赖，
以便工具核心逻辑可以在没有 rclpy 的环境中进行离线单元测试：

- ``ToolState.msg``  → ``STATE_*`` / ``EVIDENCE_*`` / ``TOOL_TYPE_*``
- ``ErrorCodes.msg`` → ``ERROR_*``（并额外保留文档 §7.1 的错误码语义）

权威来源（以文件为准，禁止在本模块发明新语义）::

    /home/robot/meituan_challenge_ws/src/mtc_interfaces/msg/ToolState.msg
    /home/robot/meituan_challenge_ws/src/mtc_interfaces/msg/ErrorCodes.msg

``test/test_tool_manager.py`` 会解析上述 ``.msg`` 文件并逐项断言本模块数值与之一致；
若 IDL 变更而本文件未同步，测试必须失败。
"""

from __future__ import annotations

from typing import Tuple

# ---------------------------------------------------------------------------
# ToolState.msg: state
# ---------------------------------------------------------------------------
STATE_UNKNOWN = 0
STATE_DETACHED = 1
STATE_ENGAGING = 2
STATE_ATTACHED = 3
STATE_RELEASING = 4
STATE_FAULT = 5

STATE_NAMES = {
    STATE_UNKNOWN: "STATE_UNKNOWN",
    STATE_DETACHED: "STATE_DETACHED",
    STATE_ENGAGING: "STATE_ENGAGING",
    STATE_ATTACHED: "STATE_ATTACHED",
    STATE_RELEASING: "STATE_RELEASING",
    STATE_FAULT: "STATE_FAULT",
}

# ---------------------------------------------------------------------------
# ToolState.msg: evidence_level
# ---------------------------------------------------------------------------
EVIDENCE_NONE = 0
EVIDENCE_COMMAND_ONLY = 1
EVIDENCE_GEOMETRY = 2
EVIDENCE_SENSOR_OR_VISION = 3

EVIDENCE_NAMES = {
    EVIDENCE_NONE: "EVIDENCE_NONE",
    EVIDENCE_COMMAND_ONLY: "EVIDENCE_COMMAND_ONLY",
    EVIDENCE_GEOMETRY: "EVIDENCE_GEOMETRY",
    EVIDENCE_SENSOR_OR_VISION: "EVIDENCE_SENSOR_OR_VISION",
}

#: 证据强度的全序（仅用于比较，不改变 msg 数值语义）。
EVIDENCE_ORDER: Tuple[int, ...] = (
    EVIDENCE_NONE,
    EVIDENCE_COMMAND_ONLY,
    EVIDENCE_GEOMETRY,
    EVIDENCE_SENSOR_OR_VISION,
)

#: 判定 ``verified=True`` 与「解锁已确认」所需的最低证据等级。
#: 设计文档 §4.6：运动命令结束（COMMAND_ONLY）与电磁铁输出信号都不是独立证据。
MIN_INDEPENDENT_EVIDENCE = EVIDENCE_GEOMETRY


def evidence_at_least(level: int, required: int = MIN_INDEPENDENT_EVIDENCE) -> bool:
    """判断证据等级 ``level`` 是否达到 ``required``。"""

    return int(level) >= int(required)


# ---------------------------------------------------------------------------
# ToolState.msg: tool_type
# ---------------------------------------------------------------------------
TOOL_TYPE_PASSIVE_HOOK_V1 = "passive_hook_v1"
TOOL_TYPE_MAGNETIC_LATCH_V2 = "magnetic_latch_v2"

# ---------------------------------------------------------------------------
# ErrorCodes.msg（数值与 mtc_interfaces/msg/ErrorCodes.msg 逐项一致）
# ---------------------------------------------------------------------------
ERROR_OK = 0
ERROR_INVALID_TASK = 100
ERROR_TARGET_NOT_FOUND = 110
ERROR_POSE_STALE = 120
ERROR_PLANNING_FAILED = 200
ERROR_EXECUTION_REJECTED = 210
ERROR_MOTION_TIMEOUT = 220
ERROR_MOTION_STATUS_UNKNOWN = 230
ERROR_ENGAGE_FAILED = 300
ERROR_ATTACH_NOT_VERIFIED = 310
ERROR_OBJECT_DROPPED = 320
ERROR_SEAT_NOT_VERIFIED = 400
ERROR_UNLOCK_FAILED = 410
ERROR_RELEASE_NOT_VERIFIED = 420
ERROR_PLACEMENT_FAILED = 430
ERROR_SAFETY_INTERLOCK = 500
ERROR_HARDWARE_FAULT = 510
ERROR_CANCEL_NOT_CONFIRMED = 520

ERROR_NAMES = {
    ERROR_OK: "OK",
    ERROR_INVALID_TASK: "INVALID_TASK",
    ERROR_TARGET_NOT_FOUND: "TARGET_NOT_FOUND",
    ERROR_POSE_STALE: "POSE_STALE",
    ERROR_PLANNING_FAILED: "PLANNING_FAILED",
    ERROR_EXECUTION_REJECTED: "EXECUTION_REJECTED",
    ERROR_MOTION_TIMEOUT: "MOTION_TIMEOUT",
    ERROR_MOTION_STATUS_UNKNOWN: "MOTION_STATUS_UNKNOWN",
    ERROR_ENGAGE_FAILED: "ENGAGE_FAILED",
    ERROR_ATTACH_NOT_VERIFIED: "ATTACH_NOT_VERIFIED",
    ERROR_OBJECT_DROPPED: "OBJECT_DROPPED",
    ERROR_SEAT_NOT_VERIFIED: "SEAT_NOT_VERIFIED",
    ERROR_UNLOCK_FAILED: "UNLOCK_FAILED",
    ERROR_RELEASE_NOT_VERIFIED: "RELEASE_NOT_VERIFIED",
    ERROR_PLACEMENT_FAILED: "PLACEMENT_FAILED",
    ERROR_SAFETY_INTERLOCK: "SAFETY_INTERLOCK",
    ERROR_HARDWARE_FAULT: "HARDWARE_FAULT",
    ERROR_CANCEL_NOT_CONFIRMED: "CANCEL_NOT_CONFIRMED",
}


def error_name(code: int) -> str:
    """错误码 → 可读常量名；未知码返回 ``UNKNOWN_<code>``。"""

    return ERROR_NAMES.get(int(code), "UNKNOWN_%d" % int(code))


# ---------------------------------------------------------------------------
# 统一阶段名（设计文档 §5.2 动作矩阵的左侧列；跨 V1/V2 对齐）
# ---------------------------------------------------------------------------
STAGE_PRE_ALIGN = "PRE_ALIGN"
STAGE_ENGAGE = "ENGAGE"
STAGE_TEST_LIFT = "TEST_LIFT"
STAGE_TRANSPORT = "TRANSPORT"
STAGE_SEAT = "SEAT"
STAGE_UNLOAD = "UNLOAD"
STAGE_DISENGAGE = "DISENGAGE"
STAGE_RETREAT = "RETREAT"

#: 分离结果三元组：明确成功 / 明确失败 / 结果未知（未知必须按不安全处理）。
DISENGAGE_CONFIRMED = "CONFIRMED"
DISENGAGE_FAILED = "FAILED"
DISENGAGE_UNKNOWN = "UNKNOWN"
DISENGAGE_OUTCOMES = (DISENGAGE_CONFIRMED, DISENGAGE_FAILED, DISENGAGE_UNKNOWN)
