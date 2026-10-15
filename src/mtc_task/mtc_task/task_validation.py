"""任务请求的规范化与合法性校验——纯 Python，不依赖 rclpy。

字段与规则来源：
* ``src/mtc_interfaces/action/ExecuteTask.action``
* ``docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md`` §3.1 / §4.3

规则（§3.1 任务顺序约定 + §4.3 校验）：
* ``task_id`` 非空；
* ``timeout_ms > 0``；
* ``MODE_BASIC``：``ordered_colors`` 恰好 1 项，目标 ``T0``；
* ``MODE_SEQUENCE``：``ordered_colors`` 恰好 3 项且互不重复，目标按数组次序固定映射 ``P1/P2/P3``；
* 所有颜色必须属于 ``{1,2,3,4}``（RED/BLUE/YELLOW/GREEN，见 BatteryDetection.msg）；
* ``target_slot``（若由上层显式给出）只允许 ``T0/P1/P2/P3``，且必须与模式/次序推导出的工位一致。

任何一条不满足都返回 :data:`ErrorCodes.INVALID_TASK`（100），FSM 侧转 ``TASK_FAILED``，
不猜测、不补全。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Any, Dict, List, Mapping, Optional, Tuple

MODE_BASIC = 1
MODE_SEQUENCE = 2

TASK_MODE_NAMES: Dict[int, str] = {MODE_BASIC: "MODE_BASIC", MODE_SEQUENCE: "MODE_SEQUENCE"}

#: BatteryDetection.msg 中的合法颜色集合（COLOR_UNKNOWN=0 不是合法口令）
VALID_COLORS: Tuple[int, ...] = (1, 2, 3, 4)
COLOR_NAMES: Dict[int, str] = {0: "COLOR_UNKNOWN", 1: "COLOR_RED", 2: "COLOR_BLUE", 3: "COLOR_YELLOW", 4: "COLOR_GREEN"}

#: PickPlace.action：target_slot 仅允许 T0 / P1 / P2 / P3
ALLOWED_SLOTS: Tuple[str, ...] = ("T0", "P1", "P2", "P3")
BASIC_SLOT = "T0"
SEQUENCE_SLOTS: Tuple[str, ...] = ("P1", "P2", "P3")

INVALID_TASK = 100

#: ExecuteTask.action Goal 的真实字段（不臆造字段名）
GOAL_FIELDS: Tuple[str, ...] = (
    "task_id",
    "mode",
    "ordered_colors",
    "timeout_ms",
    "placement_stability_sec",
)

#: 上位机可选附加的工位约束（ExecuteTask.action 本身没有该字段，
#: 但允许调用方显式指定以便交叉校验；缺省时按模式/次序推导）。
OPTIONAL_GOAL_FIELDS: Tuple[str, ...] = ("target_slot",)


@dataclass
class ValidationResult:
    ok: bool
    error_code: int = 0
    reason: str = ""
    message: str = ""
    normalized_slots: List[str] = field(default_factory=list)


def _fail(reason: str, message: str) -> ValidationResult:
    return ValidationResult(ok=False, error_code=INVALID_TASK, reason=reason, message=message)


def _coerce_int(value: Any, default: Optional[int] = None) -> Optional[int]:
    if value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _goal_field(goal: Any, name: str) -> Any:
    if isinstance(goal, Mapping):
        return goal.get(name)
    return getattr(goal, name, None)


def normalize_task_request(goal: Any) -> Dict[str, Any]:
    """从任意"类 Goal 对象"提取 ExecuteTask.action 的 Goal 字段。

    支持 ROS 消息对象、``dict``、``SimpleNamespace``。若调用方额外提供了
    ``target_slot``（可选约束字段），也会一并提取，供交叉校验使用。
    """
    if goal is None:
        raise ValueError("goal is None")
    data: Dict[str, Any] = {name: _goal_field(goal, name) for name in GOAL_FIELDS}
    if data["task_id"] is None:
        raise ValueError("missing field: task_id")
    stability = data.get("placement_stability_sec")
    data["placement_stability_sec"] = 3.0 if stability is None else float(stability)
    for name in OPTIONAL_GOAL_FIELDS:
        value = _goal_field(goal, name)
        if value is not None:
            data[name] = value
    return data


def expected_slot_for_index(mode: int, index: int, explicit_slot: Optional[str] = None) -> Optional[str]:
    """按设计约定推导第 ``index`` 项的目标工位；非法情形返回 None。"""
    mode_int = _coerce_int(mode, -1)
    if mode_int == MODE_BASIC:
        return BASIC_SLOT
    if mode_int == MODE_SEQUENCE:
        if 0 <= int(index) < len(SEQUENCE_SLOTS):
            return SEQUENCE_SLOTS[int(index)]
        return None
    return None


def validate_task_request(goal_or_task: Any) -> ValidationResult:
    """全量校验一个 ExecuteTask Goal。返回 :class:`ValidationResult`。"""
    try:
        raw = normalize_task_request(goal_or_task)
    except (TypeError, ValueError) as exc:
        return _fail("unparsable_goal", "cannot read goal fields: %s" % exc)

    task_id = raw.get("task_id")
    task_id = "" if task_id is None else str(task_id).strip()
    if not task_id:
        return _fail("empty_task_id", "task_id must not be empty")

    mode = _coerce_int(raw.get("mode"))
    if mode is None:
        return _fail("bad_mode", "mode must be an integer (MODE_BASIC=1 / MODE_SEQUENCE=2)")
    if mode not in (MODE_BASIC, MODE_SEQUENCE):
        return _fail("unknown_mode", "unsupported mode=%s; expected MODE_BASIC(1) or MODE_SEQUENCE(2)" % mode)

    colors_raw = raw.get("ordered_colors")
    if colors_raw is None:
        return _fail("missing_colors", "ordered_colors is required")
    try:
        colors = [int(c) for c in colors_raw]
    except (TypeError, ValueError):
        return _fail("bad_color_type", "ordered_colors must contain integers")

    if len(colors) == 0:
        return _fail("empty_colors", "ordered_colors must not be empty")

    if mode == MODE_BASIC and len(colors) != 1:
        return _fail(
            "basic_count",
            "MODE_BASIC requires exactly 1 color, got %d" % len(colors),
        )
    if mode == MODE_SEQUENCE and len(colors) != 3:
        return _fail(
            "sequence_count",
            "MODE_SEQUENCE requires exactly 3 colors, got %d" % len(colors),
        )

    for color in colors:
        if color not in VALID_COLORS:
            return _fail(
                "illegal_color",
                "illegal color %s; allowed %s" % (color, list(VALID_COLORS)),
            )

    if len(set(colors)) != len(colors):
        return _fail("duplicate_color", "ordered_colors must be mutually distinct, got %s" % colors)

    timeout_ms = _coerce_int(raw.get("timeout_ms"))
    if timeout_ms is None:
        return _fail("bad_timeout", "timeout_ms must be an integer")
    if timeout_ms <= 0:
        return _fail("timeout_not_positive", "timeout_ms must be > 0, got %s" % timeout_ms)

    stability = raw.get("placement_stability_sec")
    try:
        stability_value = float(stability) if stability is not None else 3.0
    except (TypeError, ValueError):
        return _fail("bad_stability", "placement_stability_sec must be a number")
    if not isfinite(stability_value) or stability_value <= 0.0:
        return _fail("stability_not_positive", "placement_stability_sec must be finite and > 0")

    explicit_slot = raw.get("target_slot")
    derived = [expected_slot_for_index(mode, i) for i in range(len(colors))]
    if any(slot is None for slot in derived):
        return _fail("slot_mapping", "cannot derive target slots for mode=%s count=%d" % (mode, len(colors)))

    if explicit_slot not in (None, ""):
        explicit_slot = str(explicit_slot)
        if explicit_slot not in ALLOWED_SLOTS:
            return _fail(
                "illegal_target_slot",
                "target_slot '%s' not allowed; allowed %s" % (explicit_slot, list(ALLOWED_SLOTS)),
            )
        if explicit_slot != derived[0]:
            return _fail(
                "target_slot_mismatch",
                "target_slot '%s' conflicts with mode=%s derived slot '%s'"
                % (explicit_slot, TASK_MODE_NAMES.get(mode, mode), derived[0]),
            )

    return ValidationResult(ok=True, error_code=0, reason="ok", message="task accepted", normalized_slots=list(derived))


def validate_slot_name(slot: str) -> ValidationResult:
    """单独校验一个工位名（供上层核对 PickPlace 目标）。"""
    text = "" if slot is None else str(slot)
    if text not in ALLOWED_SLOTS:
        return _fail("illegal_target_slot", "target_slot '%s' not allowed; allowed %s" % (text, list(ALLOWED_SLOTS)))
    return ValidationResult(ok=True, error_code=0, reason="ok", message="slot accepted", normalized_slots=[text])
