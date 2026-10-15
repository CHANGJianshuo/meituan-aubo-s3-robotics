#!/usr/bin/env python3
"""mtc_bringup.validate_config —— 配置校验 CLI（纯 Python，不依赖 ROS 2）。

用法::

    # 校验工作空间默认 config/ 目录
    PYTHONPATH=src/mtc_bringup python3 -m mtc_bringup.validate_config

    # 指定目录 / 只看错误 / 机器可读
    python3 -m mtc_bringup.validate_config --config-dir config
    python3 -m mtc_bringup.validate_config --quiet
    python3 -m mtc_bringup.validate_config --json

退出码
------
* ``0`` —— 全部通过（可能有 WARN/INFO）
* ``1`` —— 存在至少一条 ``error``（配置不合法，禁止据此启动任何节点）
* ``2`` —— 用法错误（由 argparse 产生）

安全说明
--------
校验通过**只**表示“配置自洽且不含明显危险取值”，**不代表**获得驱动真实
AUBO S3 的授权。真实通路仍需现场授权、身份白名单核实与实测验证。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import List, Optional, Sequence

try:  # 支持 `python3 -m mtc_bringup.validate_config` 与直接脚本运行两种方式
    from mtc_bringup.config_loader import (
        DEFAULT_CONFIG_DIR,
        ValidationReport,
        validate_all,
        yaml_backend,
    )
except ImportError:  # pragma: no cover - 直接以脚本路径运行时
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from mtc_bringup.config_loader import (  # type: ignore
        DEFAULT_CONFIG_DIR,
        ValidationReport,
        validate_all,
        yaml_backend,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="validate_config",
        description="校验 meituan_challenge_ws 的 config/*.yaml（纯 Python，离线，不连接任何设备）。",
    )
    parser.add_argument(
        "--config-dir",
        default=DEFAULT_CONFIG_DIR,
        help="配置目录（默认：%s）" % DEFAULT_CONFIG_DIR,
    )
    parser.add_argument("--quiet", action="store_true", help="只打印汇总，不逐条列出发现")
    parser.add_argument("--json", action="store_true", help="以 JSON 输出完整报告（便于 CI 解析）")
    return parser


def report_as_dict(report: ValidationReport) -> dict:
    return {
        "ok": report.ok,
        "config_dir": report.config_dir,
        "yaml_backend": report.backend,
        "counts": {
            "error": report.count("error"),
            "warn": report.count("warn"),
            "info": report.count("info"),
        },
        "files": [item.name for item in report.loaded],
        "findings": [
            {
                "level": item.level,
                "file": item.file,
                "path": item.path,
                "message": item.message,
            }
            for item in report.findings
        ],
    }


def print_report(report: ValidationReport, quiet: bool = False) -> None:
    backend = report.backend or yaml_backend()
    print("=" * 78)
    print("mtc_bringup 配置校验（纯 Python，离线 Mock；不连接任何真实设备）")
    print("=" * 78)
    print("配置目录    : %s" % report.config_dir)
    print("YAML 解析器 : %s%s" % (backend, "" if backend == "pyyaml" else "（PyYAML 不可用，使用内置最小解析器）"))
    print("已加载文件  : %s" % (", ".join(item.name for item in report.loaded) or "(无)"))
    print("-" * 78)

    if not quiet:
        for item in report.findings:
            print(item.format())
        if not report.findings:
            print("(无任何 WARN/INFO)")

    print("-" * 78)
    print(
        "RESULT: %s / ERROR %d / WARN %d / INFO %d"
        % ("PASS" if report.ok else "FAIL", report.count("error"), report.count("warn"), report.count("info"))
    )
    if report.ok:
        print("提示：校验通过只表示配置自洽；真实 AUBO S3 通路仍需现场授权与实测验证。")
    else:
        print("提示：存在 error，禁止据此启动任何节点；请逐条修正后重跑。")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    config_dir = os.path.abspath(args.config_dir)
    report = validate_all(config_dir)

    if args.json:
        print(json.dumps(report_as_dict(report), ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print_report(report, quiet=args.quiet)

    return 0 if report.ok else 1


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
