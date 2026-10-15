#!/usr/bin/env python3
"""mtc_bringup.config_loader —— 纯 Python 配置加载与校验器。

设计依据
--------
* ``docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md`` §8（参数与配置约定）
* §7.2（关键联锁）、§10.1（必测故障用例）
* ``docs/migration/INTERFACE_CHANGELOG.md`` §4（单位/枚举/命名空间策略）

硬性约束（本模块刻意遵守）
--------------------------
1. **不 import rclpy**，也不 import 任何 ROS 2 包；纯 Python 3 可直接运行。
2. **只允许依赖 PyYAML**。先 ``import yaml``；不可用时回退到本文件内的
   最小 YAML 子集解析器 ``parse_yaml_minimal``（不引入任何第三方库）。
3. **不读取、不写入任何凭据**；设备身份白名单留空即按安全默认拒绝运动通路。
4. 校验失败必须给出**具体文件 + 具体字段路径 + 原因**，不得静默通过。

安全语义提醒
------------
本模块只做**配置层**校验：通过校验只说明“配置自洽且不含明显危险取值”，
**不等于**获得了驱动真实机械臂的授权。真实通路在 `AUBO S3` 侧仍需现场授权、
身份白名单核实与实测验证。
"""

from __future__ import annotations

import io
import os
import re
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

__all__ = [
    "COLOR_NAMES",
    "CONFIG_FILES",
    "DEFAULT_CONFIG_DIR",
    "MODES",
    "TOOL_TYPES",
    "Finding",
    "LoadedConfig",
    "ValidationReport",
    "check_interlock_invariants",
    "detect_yaml_backend",
    "load_configuration",
    "parse_yaml",
    "parse_yaml_minimal",
    "validate_all",
    "yaml_backend",
]

# ---------------------------------------------------------------------------
# 常量：颜色枚举、模式、工具类型（与 mtc_interfaces 保持一致）
# ---------------------------------------------------------------------------

#: ``BatteryDetection.msg`` 的颜色枚举。0（UNKNOWN）不得作为任务目标颜色。
COLOR_NAMES: Dict[int, str] = {
    0: "COLOR_UNKNOWN",
    1: "COLOR_RED",
    2: "COLOR_BLUE",
    3: "COLOR_YELLOW",
    4: "COLOR_GREEN",
}
VALID_COLOR_VALUES: Tuple[int, ...] = (1, 2, 3, 4)

#: 任务模式（``ExecuteTask.action`` 的 ``mode``）。
MODES: Dict[str, int] = {"basic": 1, "sequence": 2}

#: 工具类型（``ToolState.msg`` 的 ``tool_type``）。
TOOL_TYPES: Tuple[str, ...] = ("passive_hook_v1", "magnetic_latch_v2")

#: V1 工具类型：**禁止**依赖任何电磁解锁 IO。
V1_TOOL_TYPE = "passive_hook_v1"
V2_TOOL_TYPE = "magnetic_latch_v2"

#: 已实现的运动后端取值。其他取值（含 real/aubo）一律落到 disabled。
KNOWN_BACKENDS: Tuple[str, ...] = ("mock", "capability_checked", "disabled")

#: 已实现的解锁 IO 后端取值。``real`` 未实现，必须判失败。
KNOWN_UNLOCK_IO_BACKENDS: Tuple[str, ...] = ("mock", "disabled")

#: 允许的目标工位。
ALLOWED_SLOTS: Tuple[str, ...] = ("T0", "P1", "P2", "P3")
SEQUENCE_SLOTS: Tuple[str, ...] = ("P1", "P2", "P3")

#: 必须存在的配置文件（相对于 config 目录）。
CONFIG_FILES: Tuple[str, ...] = (
    "manipulation.yaml",
    "tool_v1.yaml",
    "tool_v2.yaml",
    "motion_profiles.yaml",
    "safety.yaml",
)

_HERE = os.path.dirname(os.path.abspath(__file__))
#: 默认 config 目录：从本文件向上找到工作空间根的 ``config/``。
DEFAULT_CONFIG_DIR = os.path.normpath(os.path.join(_HERE, "..", "..", "..", "config"))

# 凭据类字段名黑名单（配置里出现即判失败，避免把密码/Token 写进仓库）。
_FORBIDDEN_KEY_PATTERN = re.compile(
    r"(password|passwd|secret|token|api[_-]?key|credential|private[_-]?key"
    r"|access[_-]?key|bearer|ssh[_-]?key)",
    re.IGNORECASE,
)

# 真实网络地址（IPv4 字面量）黑名单：配置中不得出现。
_IPV4_PATTERN = re.compile(r"\b(?:\d{1,3}\.){3}\d{1,3}\b")

# 明显非法的槽位前缀（用于拒绝 P4/T1 之类）。
_UNSUPPORTED_SLOT_HINT = re.compile(r"^(?:P|T)\d+$")


# ---------------------------------------------------------------------------
# YAML 后端探测与回退解析器
# ---------------------------------------------------------------------------
_YAML_BACKEND: Optional[str] = None
_YAML_MODULE: Any = None


def detect_yaml_backend() -> str:
    """返回 ``'pyyaml'`` 或 ``'minimal'``（本模块自带的最小解析器）。"""
    global _YAML_BACKEND, _YAML_MODULE
    if _YAML_BACKEND is not None:
        return _YAML_BACKEND
    try:  # pragma: no cover - 取决于运行环境
        import yaml  # type: ignore
    except Exception:
        _YAML_MODULE = None
        _YAML_BACKEND = "minimal"
    else:
        _YAML_MODULE = yaml
        _YAML_BACKEND = "pyyaml"
    return _YAML_BACKEND


def yaml_backend() -> str:
    """``detect_yaml_backend`` 的公开别名。"""
    return detect_yaml_backend()


# --------------------------- 最小 YAML 子集解析器 ---------------------------
# 支持：注释、空行、嵌套映射（缩进）、块序列（``- item``）、行内 ``[a, b]`` /
#       ``{a: b}``、标量（str/int/float/bool/null）、空序列 ``[]``、引号字符串。
# 不支持：锚点/别名、多行折叠标量（``|`` / ``>``）、复杂键、制表符缩进。
# 覆盖本工程 ``config/*.yaml`` 的全部结构；不支持的结构会抛出 ValueError，
# 绝不静默猜测（避免把危险数值误读成默认值）。


def _strip_comment(line: str) -> str:
    """去掉行尾注释，但保留引号内的 ``#``。"""
    out: List[str] = []
    quote: Optional[str] = None
    for ch in line:
        if quote:
            out.append(ch)
            if ch == quote:
                quote = None
            continue
        if ch in ("'", '"'):
            quote = ch
            out.append(ch)
            continue
        if ch == "#":
            break
        out.append(ch)
    return "".join(out).rstrip()


def _scalar(token: str) -> Any:
    """把标量 token 转成 Python 值（保守实现，不做隐式类型猜测之外的事）。"""
    text = token.strip()
    if text == "":
        return None
    if len(text) >= 2 and text[0] == text[-1] and text[0] in ("'", '"'):
        return text[1:-1]
    lowered = text.lower()
    if lowered in ("null", "~", "none"):
        return None
    if lowered in ("true", "yes", "on"):
        return True
    if lowered in ("false", "no", "off"):
        return False
    if text.startswith("[") and text.endswith("]"):
        inner = text[1:-1].strip()
        if inner == "":
            return []
        return [_scalar(part) for part in _split_top_level(inner)]
    if text.startswith("{") and text.endswith("}"):
        inner = text[1:-1].strip()
        if inner == "":
            return {}
        mapping: Dict[str, Any] = {}
        for part in _split_top_level(inner):
            key, _, value = part.partition(":")
            mapping[str(_scalar(key))] = _scalar(value)
        return mapping
    try:
        return int(text, 10)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        pass
    return text


def _flow_unbalanced(text: str) -> bool:
    """判断流式集合（``[...]`` / ``{...}``）的括号是否尚未闭合。"""
    depth = 0
    quote: Optional[str] = None
    for ch in text:
        if quote:
            if ch == quote:
                quote = None
            continue
        if ch in ("'", '"'):
            quote = ch
            continue
        if ch in "[{":
            depth += 1
        elif ch in "]}":
            depth -= 1
    return depth > 0


def _split_top_level(text: str) -> List[str]:
    """按逗号切分，忽略括号/引号内部的逗号。"""
    parts: List[str] = []
    depth = 0
    quote: Optional[str] = None
    current: List[str] = []
    for ch in text:
        if quote:
            current.append(ch)
            if ch == quote:
                quote = None
            continue
        if ch in ("'", '"'):
            quote = ch
            current.append(ch)
            continue
        if ch in "[{":
            depth += 1
        elif ch in "]}":
            depth -= 1
        if ch == "," and depth == 0:
            parts.append("".join(current).strip())
            current = []
            continue
        current.append(ch)
    tail = "".join(current).strip()
    if tail or parts:
        parts.append(tail)
    return [p for p in parts if p != ""]


def parse_yaml_minimal(text: str) -> Any:
    """解析 YAML 的一个**受限子集**。不支持的结构抛 :class:`ValueError`。"""
    lines: List[Tuple[int, str]] = []
    for raw in text.splitlines():
        if "\t" in raw[: len(raw) - len(raw.lstrip())]:
            raise ValueError("最小 YAML 解析器不支持制表符缩进")
        cleaned = _strip_comment(raw)
        if not cleaned.strip():
            continue
        if cleaned.strip() in ("---", "..."):
            continue
        indent = len(cleaned) - len(cleaned.lstrip(" "))
        lines.append((indent, cleaned.strip()))
    if not lines:
        return {}

    pos = 0

    def parse_block(indent: int) -> Any:
        nonlocal pos
        if pos >= len(lines):
            return None
        first_indent, first_text = lines[pos]
        if first_indent < indent:
            return None
        if first_text.startswith("- "):
            return parse_sequence(first_indent)
        return parse_mapping(first_indent)

    def parse_sequence(indent: int) -> List[Any]:
        nonlocal pos
        items: List[Any] = []
        while pos < len(lines):
            line_indent, line_text = lines[pos]
            if line_indent < indent:
                break
            if line_indent > indent:
                raise ValueError("序列项缩进异常: %r" % line_text)
            if not line_text.startswith("- "):
                break
            body = line_text[2:].strip()
            if not body:
                pos += 1
                nested = parse_block(indent + 1) if pos < len(lines) and lines[pos][0] > indent else None
                items.append(nested)
                continue
            key, sep, rest = _split_key(body)
            if sep:
                # 序列项是映射（形如 ``- name: value``）：把该行当作映射首行处理。
                items.append(parse_inline_mapping_entry(indent + 2, key, rest))
                continue
            pos += 1
            items.append(_scalar(body))
        return items

    def _split_key(text: str) -> Tuple[str, bool, str]:
        quote: Optional[str] = None
        depth = 0
        for index, ch in enumerate(text):
            if quote:
                if ch == quote:
                    quote = None
                continue
            if ch in ("'", '"'):
                quote = ch
                continue
            if ch in "[{":
                depth += 1
                continue
            if ch in "]}":
                depth -= 1
                continue
            if ch == ":" and depth == 0:
                return text[:index], True, text[index + 1:].strip()
        return text, False, ""

    def parse_inline_mapping_entry(child_indent: int, key: str, rest: str) -> Dict[str, Any]:
        """处理 ``- key: value`` 及其后续同缩进的兄弟键。"""
        nonlocal pos
        mapping: Dict[str, Any] = {}
        key_text = str(_scalar(key.strip()))
        if rest:
            mapping[key_text] = _scalar(rest)
            pos += 1
        else:
            pos += 1
            if pos < len(lines) and lines[pos][0] > child_indent - 2:
                mapping[key_text] = parse_block(lines[pos][0])
            else:
                mapping[key_text] = None
        while pos < len(lines):
            line_indent, line_text = lines[pos]
            if line_indent != child_indent:
                break
            next_key, sep, next_rest = _split_key(line_text)
            if not sep:
                break
            next_key_text = str(_scalar(next_key.strip()))
            if next_rest:
                mapping[next_key_text] = _scalar(next_rest)
                pos += 1
            else:
                pos += 1
                if pos < len(lines) and lines[pos][0] > child_indent:
                    mapping[next_key_text] = parse_block(lines[pos][0])
                else:
                    mapping[next_key_text] = None
        return mapping

    def _accumulate_flow(start: str, start_index: int, base_indent: int) -> Tuple[str, int]:
        """把跨越多行的 ``[...]`` / ``{...}`` 流式集合拼成一行。

        约定：续行的缩进必须 **大于等于** 起始键所在行的缩进（本工程 config/*.yaml
        就是在键的同一缩进层级上继续写列表元素）。若出现更小缩进则视为语法错误，
        绝不猜测或静默截断。
        """
        buffer = start
        cursor = start_index
        while _flow_unbalanced(buffer):
            if cursor >= len(lines):
                raise ValueError("流式集合括号未闭合：%r" % buffer)
            next_indent, next_text = lines[cursor]
            if next_indent < base_indent:
                raise ValueError("流式集合跨行但缩进回退：%r / %r" % (buffer, next_text))
            buffer = "%s %s" % (buffer, next_text)
            cursor += 1
        return buffer, cursor

    def parse_mapping(indent: int) -> Dict[str, Any]:
        nonlocal pos
        mapping: Dict[str, Any] = {}
        while pos < len(lines):
            line_indent, line_text = lines[pos]
            if line_indent < indent:
                break
            if line_indent > indent:
                raise ValueError("映射缩进异常: %r" % line_text)
            if line_text.startswith("- "):
                break
            key, sep, rest = _split_key(line_text)
            if not sep:
                raise ValueError("映射行缺少 ':'：%r" % line_text)
            key_text = str(_scalar(key.strip()))
            pos += 1
            if rest:
                if _flow_unbalanced(rest):
                    rest, pos = _accumulate_flow(rest, pos, indent)
                mapping[key_text] = _scalar(rest)
                continue
            if pos < len(lines) and lines[pos][0] > indent:
                mapping[key_text] = parse_block(lines[pos][0])
            elif (
                pos < len(lines)
                and lines[pos][0] == indent
                and lines[pos][1][:1] in ("[", "{")
            ):
                # 允许 ``key:`` 后接同缩进的跨行流式集合（本工程 config/*.yaml 用到）。
                flow, pos = _accumulate_flow(lines[pos][1], pos + 1, indent)
                mapping[key_text] = _scalar(flow)
            else:
                mapping[key_text] = None
        return mapping

    return parse_block(lines[0][0])


def parse_yaml(text: str) -> Any:
    """解析 YAML 文本；优先 PyYAML，不可用时回退最小解析器。"""
    backend = detect_yaml_backend()
    if backend == "pyyaml":  # pragma: no cover - 取决于运行环境
        assert _YAML_MODULE is not None
        return _YAML_MODULE.safe_load(text)
    return parse_yaml_minimal(text)


# ---------------------------------------------------------------------------
# 结果数据结构
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Finding:
    """一条校验发现。``level`` ∈ {``error``, ``warn``, ``info``}。"""

    level: str
    file: str
    path: str
    message: str

    def format(self) -> str:
        prefix = {"error": "FAIL", "warn": "WARN", "info": "INFO"}.get(self.level, self.level.upper())
        location = self.file
        if self.path:
            location = "%s::%s" % (self.file, self.path)
        return "[%s] %s — %s" % (prefix, location, self.message)


@dataclass
class LoadedConfig:
    """一份已解析的配置文件。"""

    name: str
    path: str
    data: Dict[str, Any]
    backend: str

    @property
    def params(self) -> Dict[str, Any]:
        """返回 ``<node>: {ros__parameters: {...}}`` 中的参数字典。

        支持两种合法形态：
        * ``{node_name: {ros__parameters: {...}}}``（ROS 2 标准）
        * ``{ros__parameters: {...}}``（简化）
        """
        if not isinstance(self.data, Mapping):
            return {}
        if "ros__parameters" in self.data:
            inner = self.data.get("ros__parameters")
            return dict(inner) if isinstance(inner, Mapping) else {}
        for value in self.data.values():
            if isinstance(value, Mapping) and "ros__parameters" in value:
                inner = value.get("ros__parameters")
                return dict(inner) if isinstance(inner, Mapping) else {}
        return {}


@dataclass
class ValidationReport:
    """全部配置的校验结论。"""

    findings: List[Finding] = field(default_factory=list)
    loaded: List[LoadedConfig] = field(default_factory=list)
    config_dir: str = ""
    backend: str = ""

    def add(self, level: str, file: str, path: str, message: str) -> Finding:
        finding = Finding(level=level, file=file, path=path, message=message)
        self.findings.append(finding)
        return finding

    def error(self, file: str, path: str, message: str) -> Finding:
        return self.add("error", file, path, message)

    def warn(self, file: str, path: str, message: str) -> Finding:
        return self.add("warn", file, path, message)

    def info(self, file: str, path: str, message: str) -> Finding:
        return self.add("info", file, path, message)

    def count(self, level: str) -> int:
        return sum(1 for item in self.findings if item.level == level)

    @property
    def errors(self) -> List[Finding]:
        return [item for item in self.findings if item.level == "error"]

    @property
    def warnings(self) -> List[Finding]:
        return [item for item in self.findings if item.level == "warn"]

    @property
    def infos(self) -> List[Finding]:
        return [item for item in self.findings if item.level == "info"]

    @property
    def ok(self) -> bool:
        return not self.errors


# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------
def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _has_key(params: Mapping[str, Any], *names: str) -> bool:
    return any(name in params for name in names)


def _first_present(params: Mapping[str, Any], *names: str, default: Any = None) -> Any:
    for name in names:
        if name in params:
            return params[name]
    return default


def _scan_forbidden(node: Any, path: str, report: ValidationReport, filename: str) -> None:
    """递归检查凭据字段名与 IPv4 字面量。"""
    if isinstance(node, Mapping):
        for key, value in node.items():
            key_text = str(key)
            child = "%s.%s" % (path, key_text) if path else key_text
            if _FORBIDDEN_KEY_PATTERN.search(key_text):
                report.error(
                    filename,
                    child,
                    "配置中出现疑似凭据字段名 %r；本工程禁止把密码/Token/密钥写入配置" % key_text,
                )
            _scan_forbidden(value, child, report, filename)
        return
    if isinstance(node, (list, tuple)):
        for index, value in enumerate(node):
            _scan_forbidden(value, "%s[%d]" % (path, index), report, filename)
        return
    if isinstance(node, str) and _IPV4_PATTERN.search(node):
        report.error(
            filename,
            path,
            "配置中出现 IPv4 字面量 %r；本工程禁止写入真实设备地址" % node,
        )


# ---------------------------------------------------------------------------
# 各文件校验
# ---------------------------------------------------------------------------
def _validate_manipulation(params: Mapping[str, Any], report: ValidationReport, filename: str) -> None:
    pre = ""  # 路径前缀由调用方传入的报告上下文决定，这里直接写字段名

    tool_type = params.get("tool_type")
    if tool_type is None:
        report.error(filename, "tool_type", "缺少必填字段 tool_type")
    elif tool_type not in TOOL_TYPES:
        report.error(
            filename,
            "tool_type",
            "非法工具类型 %r；必须是 %s 之一" % (tool_type, " 或 ".join(TOOL_TYPES)),
        )

    slots = params.get("target_slots")
    if slots is None:
        report.error(filename, "target_slots", "缺少必填字段 target_slots")
    elif not isinstance(slots, (list, tuple)) or not slots:
        report.error(filename, "target_slots", "target_slots 必须是非空序列")
    else:
        unknown = [s for s in slots if s not in ALLOWED_SLOTS]
        if unknown:
            report.error(
                filename,
                "target_slots",
                "含非法工位 %s；仅允许 %s" % (unknown, list(ALLOWED_SLOTS)),
            )
        if "P4" in slots or any(
            isinstance(s, str) and _UNSUPPORTED_SLOT_HINT.match(s) and s not in ALLOWED_SLOTS
            for s in slots
        ):
            report.error(filename, "target_slots", "本工程仅支持 3 个序列槽位（P1/P2/P3）")

    seq_slots = params.get("sequence_slots")
    if seq_slots is not None:
        if list(seq_slots) != list(SEQUENCE_SLOTS):
            report.error(
                filename,
                "sequence_slots",
                "序列槽位必须严格为 %s，得到 %s（槽位映射不得随意更改）"
                % (list(SEQUENCE_SLOTS), list(seq_slots)),
            )

    basic_slot = params.get("basic_slot")
    if basic_slot is not None and basic_slot != "T0":
        report.error(filename, "basic_slot", "基础任务槽位必须是 T0，得到 %r" % (basic_slot,))

    stability = _first_present(params, "placement_stability_sec")
    if stability is None:
        report.error(filename, "placement_stability_sec", "缺少必填字段 placement_stability_sec")
    elif not _is_number(stability) or float(stability) <= 0.0:
        report.error(
            filename,
            "placement_stability_sec",
            "必须为 > 0 的数值，得到 %r（0 或负值会导致“未稳定即判成功”）" % (stability,),
        )

    max_age = _first_present(params, "perception_max_age_sec", "perception_max_age_s")
    if max_age is None:
        report.error(
            filename,
            "perception_max_age_sec",
            "缺少必填字段 perception_max_age_sec",
        )
    elif not _is_number(max_age) or float(max_age) <= 0.0:
        report.error(
            filename,
            "perception_max_age_sec",
            "必须为 > 0 的数值，得到 %r（0 或负值会使所有观测都被判过期）" % (max_age,),
        )

    recovery = _first_present(params, "recovery_max_attempts")
    if recovery is None:
        report.error(filename, "recovery_max_attempts", "缺少必填字段 recovery_max_attempts")
    elif not isinstance(recovery, int) or isinstance(recovery, bool) or recovery < 0:
        report.error(
            filename,
            "recovery_max_attempts",
            "必须为 >= 0 的整数，得到 %r" % (recovery,),
        )

    for key in ("task_timeout_ms", "item_timeout_ms"):
        value = params.get(key)
        if value is None:
            report.error(filename, key, "缺少必填字段 %s" % key)
        elif not isinstance(value, int) or isinstance(value, bool) or value <= 0:
            report.error(filename, key, "必须为 > 0 的整数（毫秒），得到 %r" % (value,))

    for key in ("verify_test_lift", "verify_placement"):
        value = params.get(key)
        if value is None:
            report.warn(
                filename,
                key,
                "未配置 %s；默认按 true 处理（关闭验证即等于放弃成功判定）" % key,
            )
        elif not isinstance(value, bool):
            report.error(filename, key, "必须是布尔值 true/false，得到 %r" % (value,))
        elif value is False:
            report.warn(
                filename,
                key,
                "%s=false：将无法证明抓取/放置成功，禁止用于比赛与真机" % key,
            )

    backend = _first_present(params, "motion_backend")
    if backend is None:
        report.error(filename, "motion_backend", "缺少必填字段 motion_backend")
    elif str(backend).lower() not in KNOWN_BACKENDS:
        report.error(
            filename,
            "motion_backend",
            "未知后端 %r；可选 %s（真实后端在获得现场授权前一律 disabled）"
            % (backend, list(KNOWN_BACKENDS)),
        )

    colors = params.get("valid_colors")
    if colors is None:
        report.error(filename, "valid_colors", "缺少必填字段 valid_colors")
    else:
        if not isinstance(colors, (list, tuple)) or not colors:
            report.error(filename, "valid_colors", "valid_colors 必须是非空序列")
        else:
            illegal = [c for c in colors if c not in VALID_COLOR_VALUES]
            if illegal:
                report.error(
                    filename,
                    "valid_colors",
                    "含非法颜色值 %s；仅允许 %s（0=UNKNOWN 不得作为目标颜色）"
                    % (illegal, list(VALID_COLOR_VALUES)),
                )
            if 0 in colors:
                report.error(filename, "valid_colors", "COLOR_UNKNOWN(0) 不得作为任务目标颜色")

    for key in ("default_velocity_scaling", "default_acceleration_scaling"):
        value = params.get(key)
        if value is None:
            report.warn(filename, key, "未配置 %s" % key)
        elif not _is_number(value) or not (0.0 < float(value) <= 0.2):
            report.error(
                filename,
                key,
                "必须落在 (0, 0.2] 已审核区间内，得到 %r（执行层会以 500 拒绝超限速度）"
                % (value,),
            )

    confidence = params.get("min_detection_confidence")
    if confidence is not None:
        if not _is_number(confidence) or not (0.0 <= float(confidence) <= 1.0):
            report.error(
                filename,
                "min_detection_confidence",
                "必须落在 [0, 1]，得到 %r" % (confidence,),
            )

    frame_id = params.get("slot_pose_frame_id")
    if frame_id is not None and not str(frame_id).strip():
        report.error(filename, "slot_pose_frame_id", "frame_id 不得为空（必须可转换到 base_link）")


def _validate_tool_v1(params: Mapping[str, Any], report: ValidationReport, filename: str) -> None:
    tool_type = params.get("tool_type")
    if tool_type != V1_TOOL_TYPE:
        report.error(
            filename,
            "tool_type",
            "必须是 %r，得到 %r" % (V1_TOOL_TYPE, tool_type),
        )

    uses_io = params.get("uses_unlock_io")
    if uses_io is None:
        report.error(filename, "uses_unlock_io", "缺少必填字段 uses_unlock_io")
    elif bool(uses_io):
        report.error(
            filename,
            "uses_unlock_io",
            "V1 被动舌规必须为 false：不得依赖电磁解锁 IO（§10.1 要求解锁 IO 调用次数为 0）",
        )

    channels = params.get("io_channels")
    if channels is None:
        report.warn(filename, "io_channels", "未配置 io_channels；V1 必须为空列表")
    elif list(channels):
        report.error(
            filename,
            "io_channels",
            "V1 的 io_channels 必须为空，得到 %s" % (list(channels),),
        )

    pulse = params.get("unlock_pulse_ms")
    if pulse is None:
        report.warn(filename, "unlock_pulse_ms", "未配置 unlock_pulse_ms；V1 必须为 0")
    elif not isinstance(pulse, int) or isinstance(pulse, bool) or pulse != 0:
        report.error(
            filename,
            "unlock_pulse_ms",
            "V1 的 unlock_pulse_ms 必须为 0（不发任何脉冲），得到 %r" % (pulse,),
        )

    _validate_positive_numbers(
        params,
        report,
        filename,
        keys=(
            "side_approach_offset_m",
            "insert_depth_m",
            "hook_lift_m",
            "unload_drop_m",
            "retreat_offset_m",
            "contact_linear_speed_m_s",
            "contact_angular_speed_rad_s",
            "free_linear_speed_m_s",
            "free_angular_speed_rad_s",
        ),
        allow_negative=True,
        require_positive=(
            "insert_depth_m",
            "hook_lift_m",
            "contact_linear_speed_m_s",
            "contact_angular_speed_rad_s",
            "free_linear_speed_m_s",
            "free_angular_speed_rad_s",
        ),
    )

    min_attach = params.get("min_attach_evidence_level")
    if min_attach is None:
        report.error(filename, "min_attach_evidence_level", "缺少必填字段 min_attach_evidence_level")
    elif not isinstance(min_attach, int) or isinstance(min_attach, bool) or not (2 <= min_attach <= 3):
        report.error(
            filename,
            "min_attach_evidence_level",
            "必须 >= 2（GEOMETRY）且 <= 3，得到 %r：仅命令级证据不得宣告已挂载"
            % (min_attach,),
        )

    min_release = params.get("min_release_evidence_level")
    if min_release is not None and (
        not isinstance(min_release, int) or isinstance(min_release, bool) or min_release < 2
    ):
        report.error(
            filename,
            "min_release_evidence_level",
            "必须 >= 2（GEOMETRY），得到 %r" % (min_release,),
        )

    if params.get("unlock_io_backend") is not None:
        report.error(
            filename,
            "unlock_io_backend",
            "V1 不得配置解锁 IO 后端（V1 无电磁通道）",
        )


def _validate_tool_v2(params: Mapping[str, Any], report: ValidationReport, filename: str) -> None:
    tool_type = params.get("tool_type")
    if tool_type != V2_TOOL_TYPE:
        report.error(
            filename,
            "tool_type",
            "必须是 %r，得到 %r" % (V2_TOOL_TYPE, tool_type),
        )

    uses_io = params.get("uses_unlock_io")
    if uses_io is None:
        report.error(filename, "uses_unlock_io", "缺少必填字段 uses_unlock_io")
    elif not bool(uses_io):
        report.error(
            filename,
            "uses_unlock_io",
            "V2 使用电磁锁止，uses_unlock_io 必须为 true（false 会造成“锁止却以为已解锁”）",
        )

    pulse = params.get("unlock_pulse_ms")
    if pulse is None:
        report.error(filename, "unlock_pulse_ms", "缺少必填字段 unlock_pulse_ms")
    elif not isinstance(pulse, int) or isinstance(pulse, bool) or pulse <= 0:
        report.error(
            filename,
            "unlock_pulse_ms",
            "必须为 > 0 的整数（毫秒），得到 %r" % (pulse,),
        )
    elif pulse > 5000:
        report.error(
            filename,
            "unlock_pulse_ms",
            "脉冲宽度 %r ms 超出保守上限 5000 ms（未标定前禁止长时间通电）" % (pulse,),
        )

    backend = params.get("unlock_io_backend")
    if backend is None:
        report.error(filename, "unlock_io_backend", "缺少必填字段 unlock_io_backend")
    elif str(backend) not in KNOWN_UNLOCK_IO_BACKENDS:
        extra = ""
        if str(backend).lower() == "real":
            extra = "；真实 IO 通路未实现，必须保持 mock/disabled"
        report.error(
            filename,
            "unlock_io_backend",
            "非法或未实现的取值 %r；可选 %s%s"
            % (backend, list(KNOWN_UNLOCK_IO_BACKENDS), extra),
        )

    for key in ("require_seated_before_unlock", "require_unloaded_before_unlock"):
        value = params.get(key)
        if value is None:
            report.error(filename, key, "缺少必填联锁开关 %s（不得省略）" % key)
        elif value is not True:
            report.error(
                filename,
                key,
                "%s 必须为 true：放宽后将允许“未落座/未卸载即解锁”，违反 §7.2" % key,
            )

    evidence = params.get("require_independent_unlock_evidence")
    if evidence is not None and evidence is not True:
        report.error(
            filename,
            "require_independent_unlock_evidence",
            "必须为 true：解锁请求被受理 != 锁止已解除，必须另有独立证据",
        )

    min_unlock = params.get("min_unlock_evidence_level")
    if min_unlock is not None and (
        not isinstance(min_unlock, int) or isinstance(min_unlock, bool) or min_unlock < 2
    ):
        report.error(
            filename,
            "min_unlock_evidence_level",
            "必须 >= 2（GEOMETRY），得到 %r" % (min_unlock,),
        )

    _validate_positive_numbers(
        params,
        report,
        filename,
        keys=(
            "top_approach_height_m",
            "insert_depth_m",
            "lock_seat_depth_m",
            "unload_drop_m",
            "withdraw_offset_m",
            "contact_linear_speed_m_s",
            "contact_angular_speed_rad_s",
            "free_linear_speed_m_s",
            "free_angular_speed_rad_s",
        ),
        allow_negative=True,
        require_positive=(
            "top_approach_height_m",
            "insert_depth_m",
            "contact_linear_speed_m_s",
            "contact_angular_speed_rad_s",
            "free_linear_speed_m_s",
            "free_angular_speed_rad_s",
        ),
    )

    direction = params.get("withdraw_direction")
    if direction is not None and str(direction).strip().lower() in ("", "calibrate_me", "unknown"):
        report.warn(
            filename,
            "withdraw_direction",
            "退出方向待标定（%r）：不得假定解锁后必定垂直上退，必须由实物测试确定" % (direction,),
        )


def _validate_motion_profiles(params: Mapping[str, Any], report: ValidationReport, filename: str) -> None:
    backend = params.get("backend")
    if backend is None:
        report.error(filename, "backend", "缺少必填字段 backend")
    elif str(backend).lower() not in KNOWN_BACKENDS:
        report.error(
            filename,
            "backend",
            "未知运动后端 %r；可选 %s（real/aubo 等取值一律落到 disabled）"
            % (backend, list(KNOWN_BACKENDS)),
        )

    for key in ("max_velocity_scaling", "max_acceleration_scaling"):
        value = params.get(key)
        if value is None:
            report.error(filename, key, "缺少必填字段 %s" % key)
        elif not _is_number(value) or not (0.0 < float(value) <= 1.0):
            report.error(
                filename,
                key,
                "必须落在 (0, 1]，得到 %r（执行层以此为硬上限，超限请求返回 500）" % (value,),
            )

    for key in (
        "settle_position_tolerance_rad",
        "settle_velocity_tolerance_rad_s",
        "settle_duration_s",
    ):
        value = params.get(key)
        if value is None:
            report.warn(filename, key, "未配置 %s；执行层将使用其保守默认值" % key)
        elif not _is_number(value) or float(value) <= 0.0:
            report.error(
                filename,
                key,
                "必须为 > 0 的数值，得到 %r（<=0 会退化为“无停稳判据”）" % (value,),
            )

    names = params.get("joint_names")
    lower = params.get("joint_lower_limit_rad")
    upper = params.get("joint_upper_limit_rad")
    if names is None:
        report.error(filename, "joint_names", "缺少必填字段 joint_names")
    elif not isinstance(names, (list, tuple)) or len(names) != 6:
        report.error(
            filename,
            "joint_names",
            "必须是 6 个关节名，得到 %r" % (names,),
        )
    elif len(set(names)) != len(names):
        report.error(filename, "joint_names", "存在重复关节名：%s" % (list(names),))

    if lower is None or upper is None:
        report.error(filename, "joint_lower_limit_rad", "缺少关节限位（lower/upper 都必须给出）")
    elif len(lower) != len(upper):
        report.error(filename, "joint_lower_limit_rad", "上下限维度不一致")
    elif names is not None and len(lower) != len(names):
        report.error(filename, "joint_lower_limit_rad", "关节限位维度与 joint_names 不一致")
    else:
        for index, (low, high) in enumerate(zip(lower, upper)):
            if not (_is_number(low) and _is_number(high)):
                report.error(
                    filename,
                    "joint_lower_limit_rad[%d]" % index,
                    "限位必须是数值，得到 %r/%r" % (low, high),
                )
            elif float(low) >= float(high):
                report.error(
                    filename,
                    "joint_lower_limit_rad[%d]" % index,
                    "下限必须严格小于上限，得到 [%r, %r]" % (low, high),
                )

    profiles = params.get("profiles")
    if profiles is None:
        report.error(filename, "profiles", "缺少必填字段 profiles")
    elif not isinstance(profiles, Mapping) or not profiles:
        report.error(filename, "profiles", "profiles 必须是非空映射（阶段名 -> 参数）")
    else:
        for stage, stage_params in profiles.items():
            base = "profiles.%s" % stage
            if not isinstance(stage_params, Mapping):
                report.error(filename, base, "阶段参数必须是映射，得到 %r" % (stage_params,))
                continue
            timeout = stage_params.get("timeout_ms")
            if timeout is None:
                report.error(filename, base + ".timeout_ms", "缺少必填字段 timeout_ms")
            elif not isinstance(timeout, int) or isinstance(timeout, bool) or timeout <= 0:
                report.error(
                    filename,
                    base + ".timeout_ms",
                    "必须为 > 0 的整数（毫秒），得到 %r" % (timeout,),
                )
            for key in ("max_linear_speed_m_s", "max_angular_speed_rad_s"):
                value = stage_params.get(key)
                if value is None:
                    report.error(filename, base + "." + key, "缺少必填字段 %s" % key)
                elif not _is_number(value) or float(value) <= 0.0:
                    report.error(
                        filename,
                        base + "." + key,
                        "必须为 > 0 的数值，得到 %r" % (value,),
                    )
            contact = stage_params.get("requires_contact")
            if contact is None:
                report.warn(filename, base + ".requires_contact", "未标注是否接触敏感动作")

    declared_traj = params.get("declare_timed_trajectory_supported")
    if declared_traj is True:
        report.error(
            filename,
            "declare_timed_trajectory_supported",
            "不得声明时间参数化轨迹能力：从未实测，且会造成“把轨迹拆成多次 moveJoint”的风险",
        )
    declared_cart = params.get("declare_cartesian_motion_supported")
    if declared_cart is True:
        report.error(
            filename,
            "declare_cartesian_motion_supported",
            "不得声明笛卡尔运动能力：从未实测（INTERFACE_CHANGELOG §3）",
        )
    if params.get("forbid_trajectory_split") is False:
        report.error(
            filename,
            "forbid_trajectory_split",
            "禁止改为 false：拆解轨迹为多次调用会伪装成轨迹控制，违反 §4.2",
        )


def _validate_safety(params: Mapping[str, Any], report: ValidationReport, filename: str) -> None:
    identity = params.get("device_identity")
    if identity is None:
        report.error(
            filename,
            "device_identity",
            "缺少设备身份白名单段；缺失时无法表达“留空即拒绝”的安全默认",
        )
    elif not isinstance(identity, Mapping):
        report.error(filename, "device_identity", "必须是映射")
    else:
        serials = _first_present(identity, "serial_number_whitelist", default=None)
        models = _first_present(identity, "controller_model_whitelist", default=None)
        verified = _first_present(identity, "identity_verified", default=None)
        if serials is None or models is None:
            report.error(
                filename,
                "device_identity.serial_number_whitelist",
                "白名单字段必须显式存在（即使留空）；缺失会在实现上退化为未定义行为",
            )
        elif not list(serials) and not list(models):
            report.info(
                filename,
                "device_identity",
                "设备身份白名单留空：按安全默认拒绝建立运动通路（身份校验必然失败）",
            )
        if verified is True and not list(serials):
            report.error(
                filename,
                "device_identity.identity_verified",
                "白名单为空却声明 identity_verified=true：会造成“身份未核实却放行”",
            )
        if verified is False:
            report.info(
                filename,
                "device_identity.identity_verified",
                "identity_verified=false：真实运动通路保持拒绝（安全默认）",
            )
        allow_motion = _first_present(identity, "allow_motion_after_identity_check", default=None)
        if allow_motion is True and not list(serials):
            report.error(
                filename,
                "device_identity.allow_motion_after_identity_check",
                "白名单为空时不得置 true：未核实身份即放行运动通路",
            )

    for key in (
        "robot_state_timeout_sec",
        "tool_state_timeout_sec",
        "stop_confirm_timeout_sec",
        "feedback_freshness_limit_s",
        "state_ttl_sec",
    ):
        value = params.get(key)
        if value is None:
            report.error(filename, key, "缺少必填超时字段 %s" % key)
        elif not _is_number(value) or float(value) <= 0.0:
            report.error(
                filename,
                key,
                "必须为 > 0 的数值（秒），得到 %r（<=0 会使所有观测立即过期或永不超时）" % (value,),
            )

    for key in (
        "require_attach_verified_before_transport",
        "require_seat_verified_before_unlock",
        "require_release_verified_before_withdraw",
        "reject_second_motion_goal",
        "forbid_auto_resend_after_timeout",
        "require_stop_confirmed_after_cancel",
        "fault_latch_human_clear_only",
    ):
        value = params.get(key)
        if value is None:
            report.error(filename, key, "缺少必填联锁开关 %s（不得省略）" % key)
        elif value is not True:
            report.error(
                filename,
                key,
                "%s 必须为 true：该开关对应 §7.2 的关键联锁，不得放宽" % key,
            )

    attempts = params.get("recovery_max_attempts")
    if attempts is not None and (
        not isinstance(attempts, int) or isinstance(attempts, bool) or attempts < 0
    ):
        report.error(
            filename,
            "recovery_max_attempts",
            "必须为 >= 0 的整数，得到 %r" % (attempts,),
        )

    codes = params.get("recoverable_error_codes")
    non_retryable = {
        220: "MOTION_TIMEOUT",
        230: "MOTION_STATUS_UNKNOWN",
        310: "ATTACH_NOT_VERIFIED",
        320: "OBJECT_DROPPED",
        400: "SEAT_NOT_VERIFIED",
        410: "UNLOCK_FAILED",
        420: "RELEASE_NOT_VERIFIED",
        500: "SAFETY_INTERLOCK",
        510: "HARDWARE_FAULT",
        520: "CANCEL_NOT_CONFIRMED",
    }
    if codes is not None:
        if not isinstance(codes, (list, tuple)) or not codes:
            report.error(filename, "recoverable_error_codes", "必须是非空序列")
        else:
            bad = [c for c in codes if c in non_retryable]
            if bad:
                report.error(
                    filename,
                    "recoverable_error_codes",
                    "含禁止自动重试的错误码 %s（%s）：存在“是否已运动/是否仍携带电池”不确定时禁止自动重发"
                    % (bad, [non_retryable[c] for c in bad]),
                )

    expect_sim = params.get("expect_simulation", "missing")
    if expect_sim != "missing" and expect_sim is not None and not isinstance(expect_sim, bool):
        report.error(
            filename,
            "expect_simulation",
            "必须是 true / false / null，得到 %r" % (expect_sim,),
        )

    if params.get("software_stop_is_not_estop") is False:
        report.error(
            filename,
            "software_stop_is_not_estop",
            "禁止声明软件停止等价于实体急停",
        )


def _validate_positive_numbers(
    params: Mapping[str, Any],
    report: ValidationReport,
    filename: str,
    *,
    keys: Iterable[str],
    allow_negative: bool = False,
    require_positive: Sequence[str] = (),
) -> None:
    for key in keys:
        if key not in params:
            report.error(filename, key, "缺少必填数值字段 %s" % key)
            continue
        value = params[key]
        if not _is_number(value):
            report.error(filename, key, "必须是数值，得到 %r" % (value,))
            continue
        number = float(value)
        if key in require_positive and number <= 0.0:
            report.error(filename, key, "必须 > 0，得到 %r" % (value,))
        elif not allow_negative and number < 0.0:
            report.error(filename, key, "不得为负值，得到 %r" % (value,))


# ---------------------------------------------------------------------------
# 跨文件互斥与联锁不变量
# ---------------------------------------------------------------------------
def check_interlock_invariants(loaded: Sequence[LoadedConfig], report: ValidationReport) -> None:
    """跨文件一致性检查（工具类型、颜色、槽位映射、解锁通道等）。"""
    by_name = {item.name: item for item in loaded}
    manipulation = by_name.get("manipulation.yaml")
    tool_v1 = by_name.get("tool_v1.yaml")
    tool_v2 = by_name.get("tool_v2.yaml")
    safety = by_name.get("safety.yaml")

    if manipulation is not None:
        params = manipulation.params
        tool_type = params.get("tool_type")
        chosen = tool_v1 if tool_type == V1_TOOL_TYPE else tool_v2 if tool_type == V2_TOOL_TYPE else None
        if chosen is None and tool_type is not None:
            report.error(
                "manipulation.yaml",
                "tool_type",
                "工具类型 %r 没有对应的工具配置文件（必须与 tool_v1/tool_v2 一致）" % (tool_type,),
            )
        elif chosen is not None:
            report.info(
                "manipulation.yaml",
                "tool_type",
                "活动工具配置为 %s（另一份配置文件仍会被校验，但其参数不生效）" % chosen.name,
            )
            chosen_params = chosen.params
            if chosen.name == "tool_v1.yaml" and chosen_params.get("uses_unlock_io"):
                report.error(
                    "manipulation.yaml",
                    "tool_type",
                    "工具类型与 tool_v1.yaml 的 uses_unlock_io 冲突：V1 不得使用电磁 IO",
                )

        slots = params.get("target_slots")
        if slots is not None:
            missing_seq = [slot for slot in SEQUENCE_SLOTS if slot not in list(slots)]
            if missing_seq:
                report.error(
                    "manipulation.yaml",
                    "target_slots",
                    "序列任务需要 P1/P2/P3，缺少 %s" % (missing_seq,),
                )

        if params.get("require_unique_detection") is False:
            report.warn(
                "manipulation.yaml",
                "require_unique_detection",
                "关闭唯一性判定后可能抓错电池（重复 ID/颜色不唯一时必须有明确拒绝路径）",
            )

    if tool_v1 is not None and tool_v2 is not None:
        v1 = tool_v1.params
        v2 = tool_v2.params
        if v1.get("tool_type") == v2.get("tool_type"):
            report.error(
                "tool_v1.yaml",
                "tool_type",
                "tool_v1.yaml 与 tool_v2.yaml 的 tool_type 相同（必须分别为 %s 与 %s）"
                % (V1_TOOL_TYPE, V2_TOOL_TYPE),
            )

    if safety is not None:
        params = safety.params
        for key in (
            "require_seat_verified_before_unlock",
            "require_attach_verified_before_transport",
            "forbid_auto_resend_after_timeout",
        ):
            if params.get(key) is False:
                report.error(
                    "safety.yaml",
                    key,
                    "该联锁与 tool/motion 配置存在耦合，关闭后会使 §7.2 保护失效",
                )


# ---------------------------------------------------------------------------
# 顶层加载与校验
# ---------------------------------------------------------------------------
def load_configuration(config_dir: str) -> Tuple[List[LoadedConfig], ValidationReport]:
    """加载并解析 ``config_dir`` 下的全部配置文件。"""
    backend = detect_yaml_backend()
    report = ValidationReport(config_dir=config_dir, backend=backend)
    loaded: List[LoadedConfig] = []

    if not os.path.isdir(config_dir):
        report.error("(workspace)", "config_dir", "配置目录不存在：%s" % config_dir)
        return loaded, report

    for name in CONFIG_FILES:
        path = os.path.join(config_dir, name)
        if not os.path.isfile(path):
            report.error(name, "", "配置文件缺失：%s" % path)
            continue
        try:
            with io.open(path, "r", encoding="utf-8") as handle:
                text = handle.read()
        except OSError as exc:
            report.error(name, "", "无法读取文件：%s" % exc)
            continue
        try:
            data = parse_yaml(text)
        except Exception as exc:  # YAML 语法错误 / 最小解析器不支持的结构
            report.error(name, "", "YAML 解析失败：%s: %s" % (type(exc).__name__, exc))
            continue
        if data is None:
            data = {}
        if not isinstance(data, Mapping):
            report.error(name, "", "配置根节点必须是映射，得到 %s" % type(data).__name__)
            continue
        item = LoadedConfig(name=name, path=path, data=dict(data), backend=backend)
        loaded.append(item)
        if not item.params:
            report.error(name, "", "缺少 <node>: ros__parameters 结构或参数字典为空")
        _scan_forbidden(item.data, "", report, name)

    return loaded, report


def validate_all(config_dir: Optional[str] = None) -> ValidationReport:
    """加载 + 逐文件校验 + 跨文件互斥校验，返回完整报告。"""
    directory = os.path.abspath(config_dir or DEFAULT_CONFIG_DIR)
    loaded, report = load_configuration(directory)

    validators = {
        "manipulation.yaml": _validate_manipulation,
        "tool_v1.yaml": _validate_tool_v1,
        "tool_v2.yaml": _validate_tool_v2,
        "motion_profiles.yaml": _validate_motion_profiles,
        "safety.yaml": _validate_safety,
    }
    for item in loaded:
        validator = validators.get(item.name)
        if validator is None:
            continue
        params = item.params
        if not params:
            continue
        validator(params, report, item.name)

    check_interlock_invariants(loaded, report)
    report.loaded = loaded
    return report
