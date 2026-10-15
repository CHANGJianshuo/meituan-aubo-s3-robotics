"""mtc_aubo_bridge —— AUBO S3 SDK 适配层（默认 disabled）。

本包**不导入 pyaubo**、**不打开任何网络连接**、**不触发任何 IO**。
真实通路需现场明确授权，并完成独立 Python 3.10 SDK Worker 的实测验证。
"""

from .bridge_core import (  # noqa: F401
    AuboBridge,
    BridgeCapabilities,
    DisabledSdkWorker,
    FakeSdkWorker,
    HealthStatus,
    JointStateSnapshot,
    RobotIdentity,
    SdkWorker,
    WorkerCommandResult,
    make_bridge,
)

__all__ = [
    'AuboBridge',
    'BridgeCapabilities',
    'DisabledSdkWorker',
    'FakeSdkWorker',
    'HealthStatus',
    'JointStateSnapshot',
    'RobotIdentity',
    'SdkWorker',
    'WorkerCommandResult',
    'make_bridge',
]
