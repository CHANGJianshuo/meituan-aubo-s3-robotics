"""mtc_safety —— 软件安全联锁与故障监控（ROS 2 功能包）。

分层约定（硬性）：

- ``mtc_safety.interlocks``：**纯 Python**，不 import rclpy，可在无 ROS 环境离线测试；
- ``mtc_safety.safety_supervisor``：rclpy 外壳，只做订阅/发布/服务调用，把状态喂给
  ``interlocks`` 的核心逻辑。

安全边界：本包只发出软件停止**请求**，真实停稳必须由 ``mtc_motion_execution``
确认；ROS 软件停止请求不替代实体急停。
"""

from .interlocks import (  # noqa: F401
    DeviceState,
    FaultLatch,
    FaultRecord,
    GuardResult,
    InterlockGuard,
    PayloadState,
    PayloadTracker,
    WatchdogConfig,
    WatchdogCore,
    adapt_stop_client,
    device_state_from_robot_state,
    error_code_name,
    payload_from_tool_state,
)

__all__ = [
    'DeviceState',
    'FaultLatch',
    'FaultRecord',
    'GuardResult',
    'InterlockGuard',
    'PayloadState',
    'PayloadTracker',
    'WatchdogConfig',
    'WatchdogCore',
    'adapt_stop_client',
    'device_state_from_robot_state',
    'error_code_name',
    'payload_from_tool_state',
]
