#!/usr/bin/env python3
"""mtc_bringup.mock_pipeline —— 离线 Mock 组合编排（**纯 Python，不 import rclpy**）。

职责
----
把各功能包的**核心纯 Python 模块**装配成一条可离线运行的端到端链路：

    TaskFsm (mtc_task.task_fsm)
        └── PickPlaceFSM (mtc_manipulation.pick_place_fsm)
                ├── PlannerPort      -> MockPlanner（mtc_motion_planning 属迁移占位包，
                │                       缺失时显式标注 FAKE，绝不伪装成“已迁移完成”）
                ├── MotionPort       -> MockMotionPort -> MotionExecutorCore
                │                                      -> MockMotionBackend (mtc_motion_execution)
                ├── ToolPort         -> MockToolPort   -> ToolManager (mtc_tool.manager)
                └── PerceptionPort   -> MockPerception（视觉节点尚未实现，显式标注 FAKE）
                        （安全联锁参考 mtc_safety.interlocks；AUBO 通路参考
                          mtc_aubo_bridge.bridge_core，本链路只用其 disabled/fake 形态）

装配出的链路**永远不连接真实设备**：运动后端为 ``mock``，解锁 IO 为
``mock``/``disabled``，AUBO Bridge 只允许 ``fake``/``disabled``。

本模块不含任何凭据，也不做任何网络/硬件访问。
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# 依赖探测：把各功能包的包目录加入 sys.path 后导入其核心模块
# ---------------------------------------------------------------------------
_PKG_NAMES = (
    "mtc_interfaces",
    "mtc_description",
    "mtc_motion_planning",
    "mtc_simulation",
    "mtc_task",
    "mtc_manipulation",
    "mtc_motion_execution",
    "mtc_tool",
    "mtc_safety",
    "mtc_aubo_bridge",
)


def workspace_root() -> str:
    """从本文件位置推断工作空间根目录（``src/mtc_bringup/mtc_bringup/`` 向上三级）。"""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "..", ".."))


def ensure_package_paths(root: Optional[str] = None) -> List[str]:
    """把 ``src/<pkg>`` 加入 ``sys.path``，返回实际加入的目录列表。"""
    base = os.path.join(root or workspace_root(), "src")
    added: List[str] = []
    for name in _PKG_NAMES:
        candidate = os.path.join(base, name)
        if os.path.isdir(candidate) and candidate not in sys.path:
            sys.path.insert(0, candidate)
            added.append(candidate)
    package_parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if package_parent not in sys.path:
        sys.path.insert(0, package_parent)
        added.append(package_parent)
    return added


@dataclass
class DependencyStatus:
    """单个依赖模块的可用性。"""

    module: str
    available: bool
    detail: str = ""

    def format(self) -> str:
        mark = "OK" if self.available else "SKIPPED(依赖未就绪)"
        return "%-42s %s%s" % (self.module, mark, (" — " + self.detail) if self.detail else "")


def _try_import(module_name: str) -> Tuple[Any, DependencyStatus]:
    try:
        module = __import__(module_name, fromlist=["*"])
    except Exception as exc:  # ImportError 及模块内部错误都视为未就绪
        return None, DependencyStatus(module_name, False, "%s: %s" % (type(exc).__name__, exc))
    return module, DependencyStatus(module_name, True, "")


@dataclass
class Dependencies:
    """已解析的依赖模块集合（缺失项为 ``None``，调用方必须优雅降级）。"""

    statuses: List[DependencyStatus] = field(default_factory=list)
    modules: Dict[str, Any] = field(default_factory=dict)

    def get(self, name: str) -> Any:
        return self.modules.get(name)

    @property
    def all_ready(self) -> bool:
        return all(item.available for item in self.statuses)

    def missing(self) -> List[str]:
        return [item.module for item in self.statuses if not item.available]


MODULE_REQUESTS: Sequence[str] = (
    "mtc_task.task_fsm",
    "mtc_task.task_validation",
    "mtc_manipulation.pick_place_fsm",
    "mtc_manipulation.tool_strategy",
    "mtc_motion_execution.backends",
    "mtc_motion_execution.executor_core",
    "mtc_tool.manager",
    "mtc_tool.io_port",
    "mtc_tool.strategy",
    "mtc_tool.codes",
    "mtc_safety.interlocks",
    "mtc_aubo_bridge.bridge_core",
    # 迁移占位包：当前应为不可用（源缺失），用于验证“优雅降级”而非假 PASS
    "mtc_motion_planning",
)


def resolve_dependencies() -> Dependencies:
    """尝试导入全部核心模块，返回带状态说明的集合。"""
    deps = Dependencies()
    for name in MODULE_REQUESTS:
        module, status = _try_import(name)
        deps.statuses.append(status)
        if module is not None:
            deps.modules[name] = module
    return deps


class DependencyNotReady(RuntimeError):
    """某个必需依赖模块尚未就绪，调用方必须降级为 SKIPPED 而不是假 PASS。"""


_REQUIRED_FOR_PIPELINE = (
    "mtc_task.task_fsm",
    "mtc_manipulation.pick_place_fsm",
    "mtc_motion_execution.backends",
    "mtc_motion_execution.executor_core",
    "mtc_tool.manager",
    "mtc_tool.io_port",
)


def require_pipeline_dependencies(deps: Dependencies) -> None:
    """缺少链路必需模块时抛 :class:`DependencyNotReady`（调用方据此输出 SKIPPED）。"""
    missing = [name for name in _REQUIRED_FOR_PIPELINE if name not in deps.modules]
    if missing:
        raise DependencyNotReady(
            "以下依赖模块未就绪，链路降级为 SKIPPED：%s" % ", ".join(missing)
        )


# ---------------------------------------------------------------------------
# 时钟
# ---------------------------------------------------------------------------
class ManualClock:
    """可手动推进的时钟（同时实现 manipulation 的 ClockPort 与 backends 的 Clock）。

    ``advance`` 由测试显式调用，因此超时/稳定性判定是确定性的（不依赖墙钟）。
    """

    def __init__(self, start: float = 0.0) -> None:
        self._now = float(start)

    def now(self) -> float:
        return self._now

    def sleep(self, seconds: float) -> None:  # backends.Clock 接口
        if seconds > 0:
            self._now += float(seconds)

    def advance(self, seconds: float) -> float:
        self._now += float(seconds)
        return self._now

    def set(self, seconds: float) -> None:
        self._now = float(seconds)


# ---------------------------------------------------------------------------
# 规划端口（mtc_motion_planning 未就绪时的替身）
# ---------------------------------------------------------------------------
@dataclass
class CapabilityToken:
    """规划上下文指纹：机械臂模型 / TCP / 负载 / 控制器模式 任一变化即作废旧规划。"""

    model: str = "AUBO-S3"
    tcp_frame: str = "tool0"
    payload_kg: float = 0.0
    controller_mode: str = "position"
    version: int = 1

    def as_tuple(self) -> Tuple[str, str, float, str]:
        return (self.model, self.tcp_frame, round(float(self.payload_kg), 6), self.controller_mode)


class MockPlanner:
    """规划端口替身（FAKE，因为 ``mtc_motion_planning`` 是迁移占位包）。

    安全行为：
    * 扫掠区域存在非目标障碍物（例如另一块电池）-> 直接拒绝规划；
    * 规划上下文指纹变化（模型/TCP/负载/控制器模式）-> 旧规划作废，必须重新规划；
    * 不做任何几何求解，只做契约与安全判定。
    """

    def __init__(self, capability: Optional[CapabilityToken] = None,
                 clock: Optional[Callable[[], float]] = None) -> None:
        self.capability = capability or CapabilityToken()
        self._clock = clock or (lambda: 0.0)
        self.calls: List[Dict[str, Any]] = []
        self.rejected: List[str] = []
        #: 扫掠区域内的非目标物体（安全边界：非空即拒绝）
        self.sweep_zone_obstacles: List[str] = []
        self.target_object_id: str = ""
        #: 上一次成功规划时的能力指纹（用于“旧规划作废”判定）
        self.last_plan_capability: Optional[Tuple[str, str, float, str]] = None
        self.plans: List["MockPlan"] = []

    def set_capability(self, **changes: Any) -> None:
        for key, value in changes.items():
            if not hasattr(self.capability, key):
                raise AttributeError("未知能力字段：%s" % key)
            setattr(self.capability, key, value)
        self.capability.version += 1

    def plan(self, phase: str, target: Any) -> Any:
        from mtc_manipulation.pick_place_fsm import ErrorCode, PlanResult

        record = {
            "phase": phase,
            "target": target,
            "capability": self.capability.as_tuple(),
            "invocation": len(self.calls),
        }
        self.calls.append(record)

        obstacles = [item for item in self.sweep_zone_obstacles if item != self.target_object_id]
        if obstacles:
            self.rejected.append("sweep_zone_occupied:%s" % ",".join(obstacles))
            return PlanResult(
                ok=False,
                error_code=ErrorCode.SAFETY_INTERLOCK,
                message="工具扫掠区域内存在非目标物体 %s：拒绝规划（§10.1）" % obstacles,
                primitives=(),
            )

        current = self.capability.as_tuple()
        if self.last_plan_capability is not None and self.last_plan_capability != current:
            self.rejected.append("capability_changed")
            return PlanResult(
                ok=False,
                error_code=ErrorCode.PLANNING_FAILED,
                message=(
                    "机械臂模型/TCP/负载/控制器模式已变化（%s -> %s）：旧规划作废，"
                    "必须重新标定与规划" % (self.last_plan_capability, current)
                ),
                primitives=(),
            )

        self.last_plan_capability = current
        plan = MockPlan(phase=phase, capability=current, created_at=self._clock())
        self.plans.append(plan)
        return PlanResult(ok=True, error_code=ErrorCode.OK, message="mock 规划成功", primitives=())

    def invalidate_old_plan(self) -> bool:
        """显式使旧规划作废（能力变化时由外部调用）。"""
        if self.last_plan_capability is None:
            return False
        self.last_plan_capability = None
        return True


@dataclass
class MockPlan:
    """一次成功规划的最小记录（含生成时的能力指纹与时间戳）。"""

    phase: str
    capability: Tuple[str, str, float, str]
    created_at: float
    invalidated: bool = False


# ---------------------------------------------------------------------------
# 感知端口（视觉节点未实现时的替身）
# ---------------------------------------------------------------------------
class MockPerception:
    """感知端口替身（FAKE：``/mtc/perception/batteries`` 尚无实现节点）。

    可复现 §10.1 要求的负面场景：目标颜色错误、重复 ID、提环位姿过期、
    提环位姿不可用、非目标电池进入扫掠区域、放置未落座/未稳定。
    """

    #: 颜色枚举（BatteryDetection.msg）
    COLOR_UNKNOWN, COLOR_RED, COLOR_BLUE, COLOR_YELLOW, COLOR_GREEN = 0, 1, 2, 3, 4

    def __init__(self) -> None:
        self.clock: Optional[Any] = None
        self.targets: Dict[str, Dict[str, Any]] = {}
        self.placements: Dict[str, Dict[str, Any]] = {}
        self.detections: Dict[str, int] = {}
        self.calls: List[Tuple[str, Optional[int], float]] = []

    # -- 配置接口 -------------------------------------------------------
    def set_target(self, object_id: str, color: int, *, ring_pose: Tuple[float, float, float] = (0.3, 0.0, 0.15),
                   ring_pose_valid: bool = True, stale: bool = False, unique: bool = True,
                   found: bool = True, followed: bool = True, age_s: float = 0.0,
                   duplicates: int = 1) -> None:
        self.targets[object_id] = {
            "color": int(color),
            "ring_pose": ring_pose,
            "ring_pose_valid": bool(ring_pose_valid),
            "stale": bool(stale),
            "unique": bool(unique),
            "found": bool(found),
            "followed": bool(followed),
            "age_s": float(age_s),
            "duplicates": int(duplicates),
        }
        self.detections[object_id] = int(duplicates)

    def set_placement(self, object_id: str, *, at_target: bool = True, settled: bool = True,
                      found: bool = True, stale: bool = False,
                      pose: Tuple[float, float, float] = (0.5, 0.2, 0.05)) -> None:
        self.placements[object_id] = {
            "at_target": bool(at_target),
            "settled": bool(settled),
            "found": bool(found),
            "stale": bool(stale),
            "pose": pose,
        }

    def _age(self, object_id: str) -> float:
        spec = self.targets.get(object_id, {})
        base = float(spec.get("age_s", 0.0))
        if self.clock is not None and spec.get("stale"):
            # 过期场景：让数据年龄明显超过调用方给出的 max_age_s
            return max(base, 999.0)
        return base

    # -- 端口实现 -------------------------------------------------------
    def get_detection(self, object_id: str, expected_color: Optional[int], max_age_s: float) -> Any:
        from mtc_manipulation.pick_place_fsm import DetectionResult, ErrorCode, PoseEstimate

        self.calls.append((object_id, expected_color, max_age_s))

        if expected_color is None:
            spec = self.placements.get(object_id)
            if spec is None:
                return DetectionResult(
                    found=False, unique=True, stale=False, pose=None,
                    error_code=ErrorCode.TARGET_NOT_FOUND,
                    message="放置核验：无该对象观测",
                )
            pose = PoseEstimate(
                object_id=object_id,
                color=self.targets.get(object_id, {}).get("color", 0),
                ring_pose=spec["pose"],
                body_pose=spec["pose"],
                confidence=0.9,
                stamp_s=self.clock.now() if self.clock else 0.0,
                at_target=spec["at_target"],
                settled=spec["settled"],
                followed=False,
            )
            return DetectionResult(
                found=spec["found"], unique=True, stale=spec["stale"], pose=pose,
                message="mock 放置核验",
            )

        spec = self.targets.get(object_id)
        if spec is None or not spec.get("found", True):
            return DetectionResult(
                found=False, unique=True, stale=False, pose=None,
                error_code=ErrorCode.TARGET_NOT_FOUND,
                message="mock：未找到目标 %s" % object_id,
            )

        if int(spec.get("duplicates", 1)) > 1:
            pose = PoseEstimate(object_id=object_id, color=int(spec["color"]),
                                ring_pose=spec["ring_pose"], confidence=0.9)
            return DetectionResult(
                found=True, unique=False, stale=False, pose=pose,
                error_code=ErrorCode.TARGET_NOT_FOUND,
                message="mock：同一 object_id 重复出现 %d 次（ID 不唯一）" % spec["duplicates"],
            )

        age_s = self._age(object_id)
        if bool(spec.get("stale")) or age_s > float(max_age_s):
            pose = PoseEstimate(object_id=object_id, color=int(spec["color"]),
                                ring_pose=spec["ring_pose"], confidence=0.4)
            return DetectionResult(
                found=True, unique=bool(spec.get("unique", True)), stale=True, pose=pose,
                error_code=ErrorCode.POSE_STALE,
                message="mock：提环位姿过期（age=%.3fs > max_age=%.3fs）" % (age_s, max_age_s),
            )

        if int(spec["color"]) != int(expected_color):
            pose = PoseEstimate(object_id=object_id, color=int(spec["color"]),
                                ring_pose=spec["ring_pose"], confidence=0.9)
            return DetectionResult(
                found=True, unique=bool(spec.get("unique", True)), stale=False, pose=pose,
                error_code=ErrorCode.INVALID_TASK,
                message="mock：目标颜色不匹配（期望 %d，实际 %d）" % (expected_color, spec["color"]),
            )

        if not bool(spec.get("ring_pose_valid", True)):
            return DetectionResult(
                found=True, unique=True, stale=False, pose=None,
                error_code=ErrorCode.TARGET_NOT_FOUND,
                message="mock：ring_pose_valid=false，提环位姿不可用",
            )

        pose = PoseEstimate(
            object_id=object_id,
            color=int(spec["color"]),
            ring_pose=spec["ring_pose"],
            body_pose=spec["ring_pose"],
            confidence=0.95,
            stamp_s=self.clock.now() if self.clock else 0.0,
            ring_pose_valid=True,
            followed=bool(spec.get("followed", True)),
        )
        return DetectionResult(found=True, unique=True, stale=False, pose=pose, message="mock 感知正常")


# ---------------------------------------------------------------------------
# 运动端口 -> MotionExecutorCore -> MockMotionBackend
# ---------------------------------------------------------------------------
class MockMotionPort:
    """运动端口：把 PickPlace 的 MotionCommand 交给真实执行核心。

    关键安全语义（由 ``MotionExecutorCore`` 强制，本适配器不放松任何一条）：
    * 唯一执行权：并发命令由 core 拒绝（210）；
    * 状态未知（230）与超时（220）**禁止自动重发**；
    * 停止请求送达不等于停稳（``stop_confirmed`` 必须显式确认）。
    """

    def __init__(self, core: Any, backend: Any, clock: Any) -> None:
        self.core = core
        self.backend = backend
        self.clock = clock
        #: 已发送的运动命令（用于断言“未自动重发”）
        self.issued: List[Any] = []
        self.stop_calls: List[str] = []

    @property
    def backend_call_count(self) -> int:
        """底层后端实际收到的命令次数（``MockMotionBackend.call_log`` 长度）。"""
        log = getattr(self.backend, "call_log", None)
        return len(log) if log is not None else len(self.issued)

    def execute_joint_move(self, command: Any) -> Optional[Any]:
        from mtc_manipulation.pick_place_fsm import ErrorCode, MotionResult
        from mtc_motion_execution.backends import ExecState

        self.issued.append(command)
        result = self.core.execute_joint_move(
            command_id=command.command_id,
            joint_names=tuple(command.joint_names),
            target_rad=tuple(command.target_rad),
            velocity_scaling=float(command.velocity_scaling),
            acceleration_scaling=float(command.acceleration_scaling),
            timeout_ms=int(command.timeout_ms),
        )
        # 语义映射顺序很重要：**先判超时**，否则后端的“超时且状态不确定”
        # （ExecState.TIMEOUT + motion_state_unknown=True）会被误报成 230，
        # 掩盖“指令后超时（220）”这一更精确、更可诊断的结论。
        timed_out = (
            result.exec_state is ExecState.TIMEOUT
            or result.error_code == ErrorCode.MOTION_TIMEOUT
        )
        status_unknown = (
            not timed_out
            and (bool(result.motion_state_unknown)
                 or result.error_code == ErrorCode.MOTION_STATUS_UNKNOWN)
        )
        return MotionResult(
            success=bool(result.success),
            arrived=bool(result.success),
            stop_confirmed=bool(result.stop_confirmed),
            final_position_rad=tuple(result.final_position_rad or ()),
            error_code=int(result.error_code),
            message=str(result.message),
            status_unknown=status_unknown,
            timed_out=timed_out,
        )

    def request_stop(self, reason: str) -> Any:
        from mtc_manipulation.pick_place_fsm import StopResult

        self.stop_calls.append(reason)
        delivered, message, confirmed = self.core.request_stop(reason)
        return StopResult(
            request_delivered=bool(delivered),
            stop_confirmed=bool(confirmed),
            message=str(message),
        )


# ---------------------------------------------------------------------------
# 工具端口 -> ToolManager（mtc_tool）
# ---------------------------------------------------------------------------
class MockToolPort:
    """工具端口：机构证据一律来自 ``mtc_tool.ToolManager``。

    * **V1**：``requires_unlock_pulse()`` 为 False，``unlock()`` 也绝不会触达 IO；
    * **V2**：解锁必须经过 ``ToolManager.trigger_unlock`` 的联锁（落座 + 卸载），
      且“受理 != 已解锁”，必须有独立证据才允许撤离。
    """

    #: 工具管理器状态（与 ToolState.msg 对齐）
    STATE_DETACHED = 1
    STATE_ENGAGING = 2
    STATE_ATTACHED = 3
    STATE_RELEASING = 4
    STATE_FAULT = 5

    def __init__(self, manager: Any, strategy: Any, object_id: str = "") -> None:
        self.manager = manager
        self.strategy = strategy
        self.object_id = object_id
        self.engage_calls = 0
        self.test_lift_calls = 0
        self.seat_verify_calls = 0
        self.unload_calls = 0
        self.unlock_calls = 0
        self.disengage_calls = 0
        #: 场景开关
        self.engage_evidence_level = 2
        self.test_lift_evidence_level = 2
        self.test_lift_followed = True
        self.release_evidence_level = 2
        #: 场景开关（全部为“保守默认”，只有显式设置才会放宽）
        self.placement_seated = True
        self.unload_ok = True
        self.disengage_ok = True
        self.unlock_confirm_evidence_level = 2
        #: 记录从工具管理器读到的“已卸载证据”（用于 DISENGAGE 前置联锁）
        self._last_unloaded = False
        self._last_seated = False
        self.stages: List[str] = []

    # -- 内部 -----------------------------------------------------------
    def _to_evidence(self, status: Any, *, unloaded: bool) -> Any:
        from mtc_manipulation.pick_place_fsm import ErrorCode, ToolEvidence

        state = int(getattr(status, "state", 0) or 0)
        return ToolEvidence(
            ok=state in (self.STATE_ATTACHED, self.STATE_RELEASING, self.STATE_DETACHED),
            state=state,
            evidence_level=int(getattr(status, "evidence_level", 0) or 0),
            verified=bool(getattr(status, "verified", False)),
            locked=None,
            released=bool(getattr(status, "withdraw_allowed", False)) or state == self.STATE_DETACHED,
            seated=bool(getattr(status, "seated_verified", False)),
            unloaded=bool(unloaded or getattr(status, "unloaded", False)),
            object_present=state in (self.STATE_ATTACHED, self.STATE_RELEASING),
            error_code=int(getattr(status, "error_code", ErrorCode.OK) or 0),
            message=str(getattr(status, "detail", "") or ""),
        )

    def _fault_evidence(self, code: int, message: str) -> Any:
        from mtc_manipulation.pick_place_fsm import ToolEvidence

        return ToolEvidence(ok=False, state=self.STATE_FAULT, evidence_level=0,
                            verified=False, error_code=int(code), message=message)

    # -- 端口实现 -------------------------------------------------------
    def engage(self, phase: str, primitives: Sequence[Any]) -> Optional[Any]:
        from mtc_tool.codes import EVIDENCE_GEOMETRY, EVIDENCE_NONE
        from mtc_tool.strategy import PoseTarget

        self.engage_calls += 1
        self.stages.append("ENGAGE")
        ring_pose = getattr(primitives[0], "target", None) if primitives else None
        if ring_pose is None:
            ring_pose = PoseTarget("base_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        issued = self.manager.begin_acquire(self.object_id, ring_pose)
        if not issued.ok:
            return self._fault_evidence(issued.error_code, issued.message)
        status = self.manager.notify_engage_complete()
        level = int(self.engage_evidence_level)
        if level < int(EVIDENCE_GEOMETRY):
            status = self.manager.verify_attach(EVIDENCE_NONE, source="mock:仅命令级证据")
        else:
            status = self.manager.verify_attach(level, source="mock:几何证据")
        self._last_unloaded = False
        return self._to_evidence(status, unloaded=False)

    def test_lift(self, primitives: Sequence[Any]) -> Optional[Any]:
        from mtc_manipulation.pick_place_fsm import ErrorCode, ToolEvidence
        from mtc_tool.codes import EVIDENCE_GEOMETRY

        self.test_lift_calls += 1
        self.stages.append("TEST_LIFT")
        level = int(self.test_lift_evidence_level)
        if not self.test_lift_followed:
            # 电池未随动：不得据此进入运输（310）
            return ToolEvidence(
                ok=False, state=self.STATE_ATTACHED, evidence_level=level, verified=False,
                locked=True, released=False, seated=False, unloaded=False,
                object_present=True, error_code=ErrorCode.ATTACH_NOT_VERIFIED,
                message="mock：试提时电池未随动，缺少挂载证据",
            )
        if level < int(EVIDENCE_GEOMETRY):
            return ToolEvidence(
                ok=False, state=self.STATE_ATTACHED, evidence_level=level, verified=False,
                object_present=True, error_code=ErrorCode.ATTACH_NOT_VERIFIED,
                message="mock：试提证据等级不足（level=%d）" % level,
            )
        status = self.manager.verify_attach(level, source="mock:试提随动")
        return self._to_evidence(status, unloaded=False)

    def seat_verify(self) -> Any:
        from mtc_tool.codes import EVIDENCE_NONE, EVIDENCE_SENSOR_OR_VISION

        self.seat_verify_calls += 1
        self.stages.append("VERIFY_SEATED")
        seated = bool(self.placement_seated)
        status = self.manager.mark_seated(
            seated, EVIDENCE_SENSOR_OR_VISION if seated else EVIDENCE_NONE, source="mock:落座判定"
        )
        self._last_seated = seated
        return self._to_evidence(status, unloaded=False)

    def unload(self, primitives: Sequence[Any]) -> Optional[Any]:
        from mtc_manipulation.pick_place_fsm import ErrorCode
        from mtc_tool.codes import (
            EVIDENCE_GEOMETRY,
            EVIDENCE_NONE,
            EVIDENCE_SENSOR_OR_VISION,
        )
        from mtc_tool.strategy import PoseTarget

        self.unload_calls += 1
        self.stages.append("UNLOAD")
        seat_pose = getattr(primitives[0], "target", None) if primitives else None
        if seat_pose is None:
            seat_pose = PoseTarget("base_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        issued = self.manager.begin_release(seat_pose)
        if not issued.ok:
            return self._fault_evidence(issued.error_code, issued.message)
        if not self.placement_seated:
            # 未落座却走到 UNLOAD：这是实现缺陷，明确报错而不是继续
            return self._fault_evidence(ErrorCode.SEAT_NOT_VERIFIED, "mock：未落座即尝试卸载")
        unloaded = bool(self.unload_ok)
        self.manager.mark_seated(True, EVIDENCE_SENSOR_OR_VISION, source="mock:落座复核")
        status = self.manager.mark_unloaded(
            unloaded, EVIDENCE_GEOMETRY if unloaded else EVIDENCE_NONE, source="mock:卸载判定"
        )
        self._last_unloaded = unloaded
        return self._to_evidence(status, unloaded=unloaded)

    def unlock(self, pulse_ms: int = 200) -> Optional[Any]:
        """V2 专属：必须经 ToolManager 联锁；V1 永远不会走到这里（且不得触达 IO）。

        语义严格区分三层：
        1. ``trigger_unlock`` 返回 ``accepted=True`` —— 仅表示脉冲请求被受理；
        2. ``confirm_unlock`` 需要独立证据（锁止开关反馈/视觉），否则 ``unlock_confirmed=False``；
        3. 只有 ``unlock_confirmed=True`` 才返回 ``released=True``，调用方方可撤离。
        """
        from mtc_tool.codes import EVIDENCE_GEOMETRY, EVIDENCE_NONE

        self.unlock_calls += 1
        self.stages.append("UNLOCK")
        requested = self.manager.trigger_unlock(
            request_id="%s#unlock" % self.object_id,
            pulse_ms=int(pulse_ms),
            seated_verified=bool(self._last_seated),
            unloaded=bool(self._last_unloaded),
        )
        if not requested.accepted:
            # 联锁拒绝或 IO 未受理：不得声称已解锁
            return self._to_unlock_evidence(requested, released=False)

        level = int(self.unlock_confirm_evidence_level)
        if level >= int(EVIDENCE_GEOMETRY):
            confirmed = self.manager.confirm_unlock(level, source="mock:锁止开关反馈")
        else:
            confirmed = self.manager.confirm_unlock(level, source="mock:仅命令级证据")
        if not bool(getattr(confirmed, "unlock_confirmed", False)):
            # 受理但未确认锁止解除：返回 ok=False / released=False，调用方必须 FAULT 且不得撤离
            return self._to_unlock_evidence(requested, released=False,
                                            message_suffix="；受理但未确认锁止解除")
        return self._to_unlock_evidence(confirmed, released=True)

    def _to_unlock_evidence(self, outcome: Any, *, released: bool,
                            message_suffix: str = "") -> Any:
        from mtc_manipulation.pick_place_fsm import ToolEvidence

        status = self.manager.status()
        return ToolEvidence(
            ok=bool(released),
            state=int(status.state),
            evidence_level=int(status.evidence_level),
            verified=bool(status.verified),
            locked=not released,
            released=bool(released),
            seated=bool(status.seated_verified),
            unloaded=bool(status.unloaded),
            object_present=True,
            error_code=int(outcome.error_code),
            message=str(outcome.message) + message_suffix,
        )

    def disengage(self, primitives: Sequence[Any]) -> Optional[Any]:
        from mtc_tool.codes import DISENGAGE_CONFIRMED, EVIDENCE_GEOMETRY, EVIDENCE_NONE

        self.disengage_calls += 1
        self.stages.append("DISENGAGE")
        level = int(self.release_evidence_level)
        outcome = DISENGAGE_CONFIRMED if self.disengage_ok else "FAILED"
        decision = self.manager.resolve_disengage(
            outcome,
            EVIDENCE_GEOMETRY if level >= 2 else EVIDENCE_NONE,
            source="mock:分离判定",
        )
        status = self.manager.status()
        return self._to_evidence(status, unloaded=bool(status.unloaded)) if decision.allowed else self._fault_evidence(
            decision.error_code, decision.detail
        )

    def holding_state(self) -> str:
        """能否确认“无载荷”：只有在卸载/分离已确认时才返回 ``none``。

        保守原则：只要还有“可能仍挂载”的迹象，就返回 ``holding``，从而禁止自动重试。
        """
        if self.disengage_calls and self.disengage_ok and self._last_unloaded:
            return "none"
        if self._last_unloaded and self._last_seated:
            return "none"
        return "holding"


# ---------------------------------------------------------------------------
# 组合装配：把上述端口替身与真实模块接成一条可运行的 Mock 链路
# ---------------------------------------------------------------------------
@dataclass
class PipelineConfig:
    """Mock 链路配置（默认值与 ``config/*.yaml`` 的保守占位值一致）。"""

    tool_type: str = "passive_hook_v1"
    motion_backend: str = "mock"
    placement_stability_sec: float = 3.0
    perception_max_age_sec: float = 0.3
    recovery_max_attempts: int = 1
    joint_names: Tuple[str, ...] = (
        "shoulder_joint", "upperArm_joint", "foreArm_joint",
        "wrist1_joint", "wrist2_joint", "wrist3_joint",
    )
    #: 运动后端行为开关（用于覆盖 §10.1 的故障路径）
    backend_timeout: bool = False
    backend_status_unknown: bool = False
    backend_cancel_confirms: bool = True
    #: 解锁 IO 行为（V2）：accept / reject / timeout / raise
    unlock_io_mode: str = "accept"


def make_tool_manager(tool_type: str, unlock_io_mode: str = "accept") -> Any:
    """按工具类型构造 ``ToolManager``。

    * V1 -> ``PassiveHookV1`` + ``DisabledUnlockIo(strict=False)``：即使被误调用也**不会**产生 IO；
    * V2 -> ``MagneticLatchV2`` + ``MockUnlockIo``：解锁计入 ``call_count``，便于断言联锁。
    """
    from mtc_tool.io_port import DisabledUnlockIo, MockUnlockIo
    from mtc_tool.manager import ToolManager
    from mtc_tool.strategy import MagneticLatchV2, PassiveHookV1

    if tool_type == "magnetic_latch_v2":
        return ToolManager(strategy=MagneticLatchV2(), unlock_io=MockUnlockIo(mode=unlock_io_mode))
    return ToolManager(strategy=PassiveHookV1(), unlock_io=DisabledUnlockIo(strict=False))


class MockPipeline:
    """一条完整的离线 Mock 链路（Task FSM + PickPlace FSM + 执行层 + 工具层）。"""

    def __init__(self, config: Optional[PipelineConfig] = None,
                 tool_type: Optional[str] = None, clock: Optional[ManualClock] = None) -> None:
        from mtc_manipulation.pick_place_fsm import PickPlaceConfig, PickPlaceFSM
        from mtc_manipulation.tool_strategy import make_strategy
        from mtc_motion_execution.backends import MockMotionBackend, ValidationConfig
        from mtc_motion_execution.executor_core import MotionExecutorCore
        from mtc_task.task_fsm import Event, FsmsConfig, TaskFsm

        self.config = config or PipelineConfig()
        if tool_type is not None:
            self.config.tool_type = tool_type
        self.clock = clock or ManualClock()

        # --- 运动执行层（真实模块） -----------------------------------
        validation = ValidationConfig(
            expected_joint_names=tuple(self.config.joint_names),
            max_velocity_scaling=0.2,
            max_acceleration_scaling=0.2,
            joint_limit_rad=tuple((-3.0, 3.0) for _ in self.config.joint_names),
            settle_position_tolerance_rad=0.001,
            settle_velocity_tolerance_rad_s=0.001,
            settle_duration_s=0.2,
        )
        self.validation = validation
        self.backend = MockMotionBackend(
            config=validation,
            clock=self.clock,
            travel_time_s=0.0,               # 时长由 ManualClock 显式推进（确定性）
            timeout=self.config.backend_timeout,
            status_unknown=self.config.backend_status_unknown,
            cancel_confirms=self.config.backend_cancel_confirms,
        )
        self.executor = MotionExecutorCore(
            backend_name=self.config.motion_backend, config=validation, clock=self.clock,
            backend=self.backend,
        )

        # --- 工具层（真实模块） ---------------------------------------
        self.tool_manager = make_tool_manager(self.config.tool_type, self.config.unlock_io_mode)
        self.strategy = make_strategy(self.config.tool_type)

        # --- 端口替身 --------------------------------------------------
        self.perception = MockPerception()
        self.perception.clock = self.clock
        self.planner = MockPlanner(clock=self.clock.now)
        self.motion = MockMotionPort(self.executor, self.backend, self.clock)
        self.tool = MockToolPort(self.tool_manager, self.strategy)

        # --- 抓放 FSM（真实模块） -------------------------------------
        self.pick_place_config = PickPlaceConfig(
            tool_type=self.config.tool_type,
            placement_stability_sec=self.config.placement_stability_sec,
            perception_max_age_s=self.config.perception_max_age_sec,
            recovery_max_attempts=self.config.recovery_max_attempts,
            verify_test_lift=True,
            verify_placement=True,
            joint_names=tuple(self.config.joint_names),
        )

        # --- 任务 FSM（真实模块） -------------------------------------
        self.task = TaskFsm(
            clock=self.clock.now,
            config=FsmsConfig(recovery_max_attempts=self.config.recovery_max_attempts),
        )
        self.task.start()
        self.task.step(Event.SELF_CHECK_OK)

    # -- 工厂 -----------------------------------------------------------
    def new_pick_place(self) -> Any:
        """为一项抓放构造新的 ``PickPlaceFSM``（复用同一执行层与工具层）。"""
        from mtc_manipulation.pick_place_fsm import PickPlaceFSM

        return PickPlaceFSM(
            perception=self.perception, planner=self.planner, motion=self.motion,
            tool=self.tool, clock=self.clock, strategy=self.strategy,
            config=self.pick_place_config,
        )

    # -- 观测 -----------------------------------------------------------
    @property
    def unlock_io_call_count(self) -> int:
        """解锁 IO 端口被实际调用的次数（V1 必须始终为 0）。"""
        return int(self.tool_manager.unlock_io.call_count)

    @property
    def unlock_io_history(self) -> List[int]:
        return list(self.tool_manager.unlock_io.history)

    @property
    def backend_call_count(self) -> int:
        return self.motion.backend_call_count


# ---------------------------------------------------------------------------
# 任务级编排：把 TaskFsm 的 ActionRequest 翻译成真实 PickPlace 调用
# ---------------------------------------------------------------------------
@dataclass
class OrchestrationStep:
    """一次抓放推进的观测记录。"""

    slot: str
    color: int
    object_id: str
    result_success: bool
    placement_verified: bool
    final_state: str
    error_code: int
    message: str = ""


class MockOrchestrator:
    """把 ``TaskFsm``（第一层）与 ``PickPlaceFSM``（第二层）串成完整链路。

    它只消费 FSM 产出的 ``ActionRequest``（``call_pick_place``），不自己发明调用
    顺序——因此端到端测试验证的是**真实状态机契约**，而不是测试写死的流程。
    """

    def __init__(self, pipeline: MockPipeline, colors: Sequence[int],
                 object_ids: Optional[Sequence[str]] = None,
                 task_id: str = "mock-task-1", mode: int = 2, timeout_ms: int = 120000,
                 max_ticks_per_item: int = 600) -> None:
        self.pipeline = pipeline
        self.colors = [int(c) for c in colors]
        self.object_ids = list(object_ids) if object_ids else [
            "battery-%d" % index for index in range(len(self.colors))]
        self.task_id = task_id
        self.mode = int(mode)
        self.timeout_ms = int(timeout_ms)
        self.max_ticks_per_item = int(max_ticks_per_item)
        self.steps: List[OrchestrationStep] = []
        self.pick_place_requests: List[Dict[str, Any]] = []
        self.detail: str = ""

    # -- 感知准备 -------------------------------------------------------
    def register_targets(self, *, followed: bool = True) -> None:
        for color, object_id in zip(self.colors, self.object_ids):
            self.pipeline.perception.set_target(object_id, color, followed=followed)
            self.pipeline.perception.set_placement(object_id, at_target=True, settled=True)

    def goal(self) -> Any:
        from types import SimpleNamespace

        return SimpleNamespace(
            task_id=self.task_id,
            mode=self.mode,
            ordered_colors=list(self.colors),
            timeout_ms=self.timeout_ms,
            placement_stability_sec=self.pipeline.config.placement_stability_sec,
        )

    # -- 单项执行 -------------------------------------------------------
    def run_pick_place(self, payload: Dict[str, Any]) -> OrchestrationStep:
        fsm = self.pipeline.new_pick_place()
        object_id = str(payload.get("object_id", ""))
        self.pipeline.tool.object_id = object_id
        fsm.start(
            object_id=object_id,
            expected_color=int(payload.get("expected_color", 0)),
            target_slot=str(payload.get("target_slot", "")),
            request_id=str(payload.get("request_id", "")),
            placement_stability_sec=float(payload.get("placement_stability_sec") or 0.0),
        )
        terminal = ("SUCCESS", "FAILED", "FAULT", "CANCELLED")
        for _ in range(self.max_ticks_per_item):
            if fsm.state.value in terminal:
                break
            fsm.step()
            self.pipeline.clock.advance(1.0)   # 每 tick = 1s 虚拟时间（确定性）
        result = fsm.result()
        return OrchestrationStep(
            slot=str(payload.get("target_slot", "")),
            color=int(payload.get("expected_color", 0)),
            object_id=object_id,
            result_success=bool(result.success),
            placement_verified=bool(result.placement_verified),
            final_state=fsm.state.value,
            error_code=int(result.error_code),
            message=str(result.message),
        )

    # -- 任务级驱动 -----------------------------------------------------
    def run(self, *, register: bool = True) -> Any:
        """执行整轮任务，返回真实 ``TaskOutcome``。

        驱动方式与 ROS 外壳一致：**只应答 FSM 产出的 ActionRequest**。
        ``call_pick_place`` 的 payload 直接取自 FSM（字段与 PickPlace.action 一致），
        结果再经 ``report_item_result`` 送回，因此不存在“测试自己编流程”。
        """
        from mtc_task.task_fsm import ActionKind, TaskStateName

        if register:
            self.register_targets()
        fsm = self.pipeline.task

        accepted = fsm.submit_goal(self.goal())
        if accepted.goal_rejected:
            self.detail = "goal rejected: %s" % accepted.reason
            return fsm.build_outcome()
        fsm.advance_validation()

        terminal = {TaskStateName.FINISHED, TaskStateName.TASK_FAILED,
                    TaskStateName.CANCELLED, TaskStateName.FAULT}
        pending_payload: Optional[Dict[str, Any]] = None

        for _ in range(64):
            if fsm.state in terminal:
                break
            state = fsm.state

            if state is TaskStateName.SCAN_SCENE:
                index = fsm.current_index
                if index >= len(self.colors):
                    self.detail = "编排器越界：index=%d" % index
                    break
                step = fsm.report_target(self.object_ids[index], expected_color=self.colors[index])

            elif state is TaskStateName.EXECUTE_ITEM:
                if pending_payload is None:
                    # 防御：EXECUTE_ITEM 必须由 call_pick_place 进入；缺失则不得运动
                    self.detail = "EXECUTE_ITEM 缺少 call_pick_place 请求，拒绝运动"
                    step = fsm.report_item_result(
                        {"success": False, "error_code": 430, "message": self.detail})
                else:
                    payload = pending_payload
                    pending_payload = None
                    self.pick_place_requests.append(payload)
                    item = self.run_pick_place(payload)
                    self.steps.append(item)
                    step = fsm.report_item_result({
                        "success": item.result_success,
                        "placement_verified": item.placement_verified,
                        "error_code": item.error_code,
                        "message": item.message,
                    })

            elif state is TaskStateName.RECORD_RESULT:
                step = fsm.record_completed_item(fsm.current_slot)

            elif state is TaskStateName.RECOVERY:
                step = fsm.request_retry()

            else:
                self.detail = "编排器无法处理的状态：%s" % state.value
                break

            request = step.find(ActionKind.CALL_PICK_PLACE)
            if request is not None:
                pending_payload = dict(request.payload)
            self.pipeline.clock.advance(1.0)

        return fsm.build_outcome()
