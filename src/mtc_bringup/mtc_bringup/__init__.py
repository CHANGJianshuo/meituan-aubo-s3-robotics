"""mtc_bringup —— 启动、参数组合与配置校验。

本包的核心模块（``config_loader``）是**纯 Python 3、不 import rclpy**，
因此可以在没有 ROS 2 环境的机器上直接做配置校验与离线测试::

    PYTHONPATH=src/mtc_bringup python3 -m mtc_bringup.validate_config

ROS 2 相关模块（``bringup_mock``）只在真正需要启动节点时导入。
"""

from .config_loader import (  # noqa: F401
    COLOR_NAMES,
    DEFAULT_CONFIG_DIR,
    MODES,
    TOOL_TYPES,
    Finding,
    LoadedConfig,
    ValidationReport,
    check_interlock_invariants,
    load_configuration,
    parse_yaml,
    validate_all,
    yaml_backend,
)

__all__ = [
    "COLOR_NAMES",
    "DEFAULT_CONFIG_DIR",
    "MODES",
    "TOOL_TYPES",
    "Finding",
    "LoadedConfig",
    "ValidationReport",
    "check_interlock_invariants",
    "load_configuration",
    "parse_yaml",
    "validate_all",
    "yaml_backend",
]

__version__ = "0.1.0"
