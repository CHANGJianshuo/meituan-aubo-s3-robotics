#!/usr/bin/env python3
"""mock_bringup.launch.py —— 离线 Mock 组合启动（默认且唯一可用于本机的组合）。

启动内容（全部为 Mock / 离线形态）
----------------------------------
* ``mtc_motion_execution/motion_executor``  后端 ``backend:=mock``（唯一允许的取值）
* ``mtc_task/task_executor``               ``/mtc/task/execute``
* ``mtc_manipulation/pick_place_server``    ``/mtc/manipulation/pick_place``
* ``mtc_tool/tool_manager``                解锁 IO 为 ``mock``/``disabled``
* ``mtc_safety/safety_supervisor``         只发软件停止**请求**，不替代实体急停

**绝对不启动任何真实设备节点**：不启动 AUBO Bridge 真实 SDK Worker、不启用
``real``/``aubo`` 运动后端、不触发电磁铁、不启动 Gazebo。

已就绪性说明（如实登记，不掩盖）
--------------------------------
节点外壳就绪情况（2026-10-09 实测核对，非推测）：

* 已落地并默认启动：``motion_executor``、``task_executor``、``pick_place_server``、
  ``tool_manager``、``safety_supervisor``
* ``mtc_aubo_bridge`` 的 ``bridge_node`` 亦已落地，但它面向真实 SDK 通路
  （默认 ``mode=disabled``），**不属于 Mock 组合**，因此本 launch 不启动它；
  真机桥接需现场授权后使用独立启动流程。

任何 ``start_*`` 参数被关闭或因故未启动的节点都会打印明确日志，而不是静默失败。

用法::

    ros2 launch mtc_bringup mock_bringup.launch.py
    ros2 launch mtc_bringup mock_bringup.launch.py backend:=mock tool_type:=magnetic_latch_v2
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: 醒目的 Mock 告警（模块级常量，便于被测试脚本或 CI 直接检查文本）
MOCK_WARNING = (
    "=" * 78 + "\n"
    "!! 警告：Mock 模式，未连接真实 AUBO S3。\n"
    "!! 本次启动不会建立任何真实控制通路：不连接控制柜、不发送真实运动指令、\n"
    "!! 不触发电磁铁、不启动 Gazebo；软件停止请求也不等价于实体急停。\n"
    "!! Mock 通过 != 实机抓放成功。\n" + "=" * 78
)

#: 明确禁止的后端取值（出现即拒绝启动）
FORBIDDEN_BACKENDS = ("real", "aubo", "hardware", "sdk", "pyaubo")

#: 组合清单：(launch 参数名, 功能包, 可执行文件, 配置文件, 是否默认启动)
COMPOSITION = (
    ("start_motion_executor", "mtc_motion_execution", "motion_executor",
     "motion_profiles.yaml", True),
    ("start_task_executor", "mtc_task", "task_executor", "manipulation.yaml", True),
    ("start_pick_place_server", "mtc_manipulation", "pick_place_server",
     "manipulation.yaml", True),
    ("start_tool_manager", "mtc_tool", "tool_manager", None, True),
    ("start_safety_supervisor", "mtc_safety", "safety_supervisor",
     "safety.yaml", True),
)


def _load_params(package_share: str, config_name: str, config_dir: str) -> dict:
    """读取 ``config/<name>``；优先使用显式 ``config_dir``，否则退回包内 share。"""
    try:
        import yaml  # PyYAML 由 exec_depend python3-yaml 保证
    except Exception:  # pragma: no cover
        return {}
    candidates = []
    if config_dir:
        candidates.append(os.path.join(config_dir, config_name))
    candidates.append(os.path.join(package_share, "config", config_name))
    for path in candidates:
        if os.path.isfile(path):
            with open(path, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            if isinstance(data, dict):
                return data
    return {}


def _build_nodes(context, *args, **kwargs):
    backend = LaunchConfiguration("backend").perform(context)
    tool_type = LaunchConfiguration("tool_type").perform(context)
    config_dir = LaunchConfiguration("config_dir").perform(context)
    placement_stability_sec = LaunchConfiguration("placement_stability_sec").perform(context)

    actions = [LogInfo(msg=MOCK_WARNING)]

    if str(backend).strip().lower() in FORBIDDEN_BACKENDS:
        return [LogInfo(msg=(
            "!! 拒绝启动：backend=%r 属于真实/硬件后端。mock_bringup 只允许 mock；"
            "真实通路需现场明确授权并改用经过安全评审的启动流程。" % backend))]

    if str(backend).strip().lower() != "mock":
        actions.append(LogInfo(msg=(
            "提示：backend=%r 非 mock；执行层会把未知取值一律落到 disabled（绝不连真实设备）。"
            % backend)))

    actions.append(LogInfo(msg=(
        "组合参数：backend=%s tool_type=%s placement_stability_sec=%s config_dir=%s"
        % (backend, tool_type, placement_stability_sec, config_dir or "(包内 share)"))))

    for arg_name, package, executable, config_name, default_on in COMPOSITION:
        enabled = str(LaunchConfiguration(arg_name).perform(context)).strip().lower() in (
            "true", "1", "yes", "on")
        if not enabled:
            actions.append(LogInfo(msg="SKIPPED(已显式关闭该节点)：%s/%s"
                                       % (package, executable)))
            continue

        try:
            share = get_package_share_directory(package)
        except Exception:
            actions.append(LogInfo(msg=(
                "SKIPPED(依赖未就绪)：找不到功能包 %s（未 build 或未安装）；"
                "节点 %s 未启动。" % (package, executable))))
            continue

        params = _load_params(share, config_name, config_dir) if config_name else {}
        parameter_sets = []
        if params:
            parameter_sets.append(params)
        if package in ("mtc_tool", "mtc_manipulation"):
            # 工具类型以 launch 参数为准（与 manipulation.yaml 的 tool_type 互斥校验一致）
            parameter_sets.append({"tool_type": tool_type})
        if package == "mtc_motion_execution":
            parameter_sets.append({"backend": "mock", "motion_backend": "mock"})

        actions.append(Node(
            package=package,
            executable=executable,
            name=executable,
            output="screen",
            parameters=parameter_sets,
            emulate_tty=True,
        ))

    actions.append(LogInfo(msg=(
        "Mock 组合启动完成。再次提醒：未连接真实 AUBO S3；"
        "任何实机操作都必须获得单独的现场授权。")))
    return actions


def generate_launch_description() -> LaunchDescription:
    default_config_dir = os.environ.get("MTC_CONFIG_DIR", "")

    declared = [
        DeclareLaunchArgument("backend", default_value="mock",
                              description="运动后端；本 launch 只允许 mock"),
        DeclareLaunchArgument("motion_backend", default_value="mock",
                              description="兼容别名（与 backend 同义）"),
        DeclareLaunchArgument("tool_type", default_value="passive_hook_v1",
                              description="passive_hook_v1 或 magnetic_latch_v2"),
        DeclareLaunchArgument("config_dir", default_value=default_config_dir,
                              description="工作空间 config/ 目录（留空则用包内 share）"),
        DeclareLaunchArgument("placement_stability_sec", default_value="3.0",
                              description="放置稳定性观察时长（秒）；待实物标定"),
    ]
    for arg_name, _package, _executable, _config, default_on in COMPOSITION:
        declared.append(DeclareLaunchArgument(
            arg_name, default_value="true" if default_on else "false",
            description="是否启动该节点（默认 True；可按需显式关闭）"))

    return LaunchDescription(declared + [OpaqueFunction(function=_build_nodes)])
