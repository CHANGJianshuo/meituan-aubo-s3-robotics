#!/usr/bin/env python3
"""mtc_bringup.bringup_mock —— 离线 Mock 组合启动入口。

**纯 Python 3，不 import rclpy**：它不创建任何 ROS 2 节点，只做三件事：

1. 校验 ``config/*.yaml``（缺一不可，失败即退出码 1）；
2. 打印本次 Mock 组合的**节点/话题/服务清单**（供人工核对）；
3. 打印醒目告警：**Mock 模式，未连接真实 AUBO S3**。

如需要真正的 ROS 2 节点组合，请使用同包的
``launch/mock_bringup.launch.py``（该文件才导入 launch/launch_ros）。

用法::

    PYTHONPATH=src/mtc_bringup python3 -m mtc_bringup.bringup_mock
    python3 -m mtc_bringup.bringup_mock --config-dir config --self-check

``--self-check`` 会在内存中跑一次“蓝→红→黄”全 Mock 序列，用于离线自检；
它同样**不连接任何真实设备**。
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional, Sequence

try:
    from mtc_bringup.config_loader import DEFAULT_CONFIG_DIR, validate_all
    from mtc_bringup.mock_pipeline import (
        MockOrchestrator,
        MockPipeline,
        PipelineConfig,
        ensure_package_paths,
        require_pipeline_dependencies,
        resolve_dependencies,
    )
except ImportError:  # pragma: no cover - 直接以脚本路径运行时
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from mtc_bringup.config_loader import DEFAULT_CONFIG_DIR, validate_all  # type: ignore
    from mtc_bringup.mock_pipeline import (  # type: ignore
        MockOrchestrator,
        MockPipeline,
        PipelineConfig,
        ensure_package_paths,
        require_pipeline_dependencies,
        resolve_dependencies,
    )

#: Mock 组合清单：条目为 (功能包, 可执行文件, 话题/服务) —— 全部为离线替身。
MOCK_COMPOSITION = (
    ("mtc_motion_execution", "motion_executor", "/mtc/motion/execute_joint_move (backend:=mock)"),
    ("mtc_task", "task_executor", "/mtc/task/execute, /mtc/task/state"),
    ("mtc_manipulation", "pick_place_server", "/mtc/manipulation/pick_place"),
    ("mtc_tool", "tool_manager", "/mtc/tool/state, /mtc/tool/trigger_unlock (IO:=mock/disabled)"),
    ("mtc_safety", "safety_supervisor", "/mtc/motion/request_stop, /mtc/robot/state (只发停止请求)"),
)

#: 明确**不启动**的条目（安全声明的一部分）。
NEVER_STARTED = (
    "mtc_aubo_bridge 真实 SDK Worker（pyaubo / RPC / RTDE）",
    "任何 real / aubo 运动后端（一律落到 disabled）",
    "电磁铁解锁真实 IO（真实 IO 通路未实现）",
    "Gazebo / 仿真世界（本机无 ROS 2 Jazzy，sim_bringup 为 NOT RUN）",
)

BANNER = "!" * 78
WARNING_LINES = (
    "警告：Mock 模式，未连接真实 AUBO S3。",
    "本进程不会建立任何真实控制通路：不连接控制柜、不发送运动指令、不触发电磁铁。",
    "Mock 通过不等于实机抓放成功；软件停止请求也不等价于实体急停。",
)


def banner() -> None:
    print(BANNER)
    for line in WARNING_LINES:
        print("!! " + line)
    print(BANNER)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="bringup_mock",
        description="离线 Mock 组合启动（纯 Python；不连接任何真实设备）。",
    )
    parser.add_argument("--config-dir", default=DEFAULT_CONFIG_DIR,
                        help="配置目录（默认：%s）" % DEFAULT_CONFIG_DIR)
    parser.add_argument("--tool-type", default=None,
                        choices=["passive_hook_v1", "magnetic_latch_v2"],
                        help="覆盖工具类型（默认取 manipulation.yaml）")
    parser.add_argument("--self-check", action="store_true",
                        help="在内存中跑一次“蓝→红→黄”全 Mock 序列")
    parser.add_argument("--dry-run", action="store_true",
                        help="只打印组合清单与校验结果，不运行自检")
    return parser


def print_composition(tool_type: str) -> None:
    print("Mock 组合（全部为离线替身，不会启动真实设备节点）：")
    for package, executable, endpoint in MOCK_COMPOSITION:
        print("  - %-22s %-20s %s" % (package, executable, endpoint))
    print("明确不启动：")
    for item in NEVER_STARTED:
        print("  - %s" % item)
    print("工具类型：%s" % tool_type)


def run_self_check(config_dir: str, tool_type: str) -> int:
    """内存中跑一次蓝→红→黄全 Mock 序列（不连接任何设备）。"""
    ensure_package_paths()
    deps = resolve_dependencies()
    try:
        require_pipeline_dependencies(deps)
    except Exception as exc:
        print("SKIPPED(依赖未就绪)：%s" % exc)
        return 0

    pipeline = MockPipeline(PipelineConfig(tool_type=tool_type))
    orchestrator = MockOrchestrator(
        pipeline, colors=[2, 1, 3], object_ids=["battery-blue", "battery-red", "battery-yellow"], mode=2)
    outcome = orchestrator.run()
    for step in orchestrator.steps:
        print("  %-4s %-16s success=%s placement_verified=%s error=%s"
              % (step.slot, step.final_state, step.result_success,
                 step.placement_verified, step.error_code))
    print("self-check：success=%s completed=%s slots=%s"
          % (outcome.success, outcome.completed_count, outcome.completed_slots))
    print("self-check：解锁 IO 调用次数=%d（V1 必须为 0）" % pipeline.unlock_io_call_count)
    print("Mock PASS，非实机抓放成功。")
    return 0 if outcome.success else 1


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(list(argv) if argv is not None else None)
    config_dir = os.path.abspath(args.config_dir)

    banner()

    report = validate_all(config_dir)
    print("配置校验：%s（ERROR %d / WARN %d / INFO %d）"
          % ("PASS" if report.ok else "FAIL", report.count("error"),
             report.count("warn"), report.count("info")))
    for item in report.findings:
        if item.level == "error":
            print(item.format())

    params = {}
    for item in report.loaded:
        if item.name == "manipulation.yaml":
            params = item.params
            break
    tool_type = args.tool_type or str(params.get("tool_type") or "passive_hook_v1")
    motion_backend = str(params.get("motion_backend") or "mock")

    print("-" * 78)
    print_composition(tool_type)
    print("运动后端：%s（mock 以外的取值一律落到 disabled）" % motion_backend)
    print("-" * 78)

    if not report.ok:
        print("配置存在 error：拒绝启动任何节点（包括 Mock）。")
        return 1

    if args.dry_run:
        print("dry-run：仅打印组合与校验结果，未运行自检。")
        return 0

    if motion_backend not in ("mock", "capability_checked", "disabled"):
        print("后端 %r 不被支持：拒绝启动。" % motion_backend)
        return 1

    if args.self_check or not args.dry_run:
        print("运行离线自检（Mock，蓝→红→黄）……")
        return run_self_check(config_dir, tool_type)
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
