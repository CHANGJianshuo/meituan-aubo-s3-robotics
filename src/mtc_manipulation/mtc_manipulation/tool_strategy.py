"""工具策略（末端执行器）抽象与 V1/V2 两代具体实现。

设计依据：docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md §5.1 / §5.2

职责边界（重要）：
    ToolStrategy **只产生运动原语（MotionPrimitive）**，即“有约束的局部运动意图”，
    不直接调用任何 SDK、不发送关节命令、不掌控运动执行层。
    真正执行由 FSM 通过 PlannerPort / MotionPort / ToolPort 完成。

    V1 与 V2 的差异必须被封装在本文件（以及测试用的 ToolPort 实现）中，
    FSM 的 RESOLVE_TARGET → ... → VERIFY_PLACED 主流水线与工具型号无关。

V1 差异：side_insert / hook_engage / unload / side_withdraw，无任何电磁或夹爪 IO 依赖。
V2 差异：top_insert / auto_lock / unload / electromagnetic_unlock / withdraw；
        解锁前置条件必须是“已确认落座（seated verified）且载荷已卸除（unloaded）”。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List, Sequence, Tuple

from .pick_place_fsm import (
    ErrorCode,
    MotionPrimitive,
    TOOL_MAGNETIC_LATCH_V2,
    TOOL_PASSIVE_HOOK_V1,
)


@dataclass(frozen=True)
class UnlockDecision:
    """解锁前置条件判定结果。

    allowed   是否允许发出解锁脉冲
    error_code 拒绝原因对应的错误码
    reason    人类可读原因（写入日志，不写任何凭据）
    pulse_ms  允许解锁时的建议脉冲宽度
    """

    allowed: bool
    error_code: int = ErrorCode.OK
    reason: str = ""
    pulse_ms: int = 200


class ToolStrategy:
    """工具策略抽象基类。

    子类至少实现：type()、make_acquire_plan()、make_engage_plan()、
    make_release_plan()、make_disengage_plan()、requires_unlock_pulse()。
    默认实现给出保守的通用原语，避免子类漏实现导致“无运动原语”故障。
    """

    #: 该类工具会使用到的机构 IO 名称集合（用于离线断言“V1 不触碰电磁 IO”）
    IO_CHANNELS: Tuple[str, ...] = ()

    DEFAULT_LINEAR_SPEED = 0.02
    DEFAULT_ANGULAR_SPEED = 0.1
    CONTACT_LINEAR_SPEED = 0.005

    # ------------------------------------------------------------------
    # 必须实现
    # ------------------------------------------------------------------

    def type(self) -> str:
        """返回工具类型字符串，必须与 PickPlaceConfig.tool_type 一致。"""
        raise NotImplementedError

    def make_acquire_plan(self, ring_pose: Any) -> Sequence[MotionPrimitive]:
        raise NotImplementedError

    def make_engage_plan(self, ring_pose: Any) -> Sequence[MotionPrimitive]:
        raise NotImplementedError

    def make_release_plan(self, seat_pose: Any) -> Sequence[MotionPrimitive]:
        raise NotImplementedError

    def make_disengage_plan(self, seat_pose: Any) -> Sequence[MotionPrimitive]:
        raise NotImplementedError

    def requires_unlock_pulse(self) -> bool:
        raise NotImplementedError

    # ------------------------------------------------------------------
    # 通用实现（各代共用）
    # ------------------------------------------------------------------

    def make_test_lift_plan(self, ring_pose: Any) -> Sequence[MotionPrimitive]:
        """低速短距试提：所有工具都做，用于获取挂载证据。"""
        return (
            MotionPrimitive(
                name="test_lift",
                target_pose=ring_pose,
                max_linear_speed=0.01,
                max_angular_speed=0.05,
                requires_contact=False,
            ),
        )

    def make_lift_plan(self, ring_pose: Any) -> Sequence[MotionPrimitive]:
        return (
            MotionPrimitive(
                name="lift",
                target_pose=ring_pose,
                max_linear_speed=0.03,
                max_angular_speed=0.1,
                requires_contact=False,
            ),
        )

    def make_transport_plan(self, ring_pose: Any, target_slot: str) -> Sequence[MotionPrimitive]:
        return (
            MotionPrimitive(
                name="transport",
                target_pose=target_slot,
                max_linear_speed=0.15,
                max_angular_speed=0.3,
                requires_contact=False,
            ),
        )

    def make_seat_plan(self, seat_pose: Any) -> Sequence[MotionPrimitive]:
        """落座前的预放置（自由空间下降）。"""
        return (
            MotionPrimitive(
                name="pre_seat",
                target_pose=seat_pose,
                max_linear_speed=0.03,
                max_angular_speed=0.1,
                requires_contact=False,
            ),
        )

    def make_seat_contact_plan(self, seat_pose: Any) -> Sequence[MotionPrimitive]:
        """受约束低速下降至获得台面承托；不得继续硬顶。"""
        return (
            MotionPrimitive(
                name="seat_contact",
                target_pose=seat_pose,
                max_linear_speed=self.CONTACT_LINEAR_SPEED,
                max_angular_speed=0.05,
                requires_contact=True,
            ),
        )

    def make_retreat_plan(self, seat_pose: Any) -> Sequence[MotionPrimitive]:
        return (
            MotionPrimitive(
                name="retreat",
                target_pose=seat_pose,
                max_linear_speed=0.03,
                max_angular_speed=0.1,
                requires_contact=False,
            ),
        )

    def evaluate_unlock(self, seated_verified: bool, unloaded_verified: bool) -> UnlockDecision:
        """解锁前置条件判定（默认拒绝；只有需要解锁的工具才可能放行）。"""
        raise NotImplementedError

    def uses_io(self, channel: str) -> bool:
        return channel in self.IO_CHANNELS


class PassiveHookV1Strategy(ToolStrategy):
    """V1：纯机械舌规。侧向穿环 -> 挂钩 -> 放下卸载 -> 退钩。

    该策略**不存在解锁动作**，IO_CHANNELS 为空；任何解锁请求都必须被拒绝。
    """

    IO_CHANNELS: Tuple[str, ...] = ()

    def type(self) -> str:
        return TOOL_PASSIVE_HOOK_V1

    def make_acquire_plan(self, ring_pose: Any) -> Sequence[MotionPrimitive]:
        return (
            MotionPrimitive(
                name="side_insert",
                target_pose=ring_pose,
                max_linear_speed=self.DEFAULT_LINEAR_SPEED,
                max_angular_speed=self.DEFAULT_ANGULAR_SPEED,
                requires_contact=False,
            ),
        )

    def make_engage_plan(self, ring_pose: Any) -> Sequence[MotionPrimitive]:
        # 横向穿环 + 适当上提完成挂钩：接触敏感，低速
        return (
            MotionPrimitive(
                name="hook_engage",
                target_pose=ring_pose,
                max_linear_speed=self.CONTACT_LINEAR_SPEED,
                max_angular_speed=0.05,
                requires_contact=True,
            ),
        )

    def make_release_plan(self, seat_pose: Any) -> Sequence[MotionPrimitive]:
        return (
            MotionPrimitive(
                name="unload",
                target_pose=seat_pose,
                max_linear_speed=self.CONTACT_LINEAR_SPEED,
                max_angular_speed=0.05,
                requires_contact=True,
            ),
        )

    def make_disengage_plan(self, seat_pose: Any) -> Sequence[MotionPrimitive]:
        # 沿标定的反向路径退出，倒钩不再承重后才能退钩
        return (
            MotionPrimitive(
                name="side_withdraw",
                target_pose=seat_pose,
                max_linear_speed=self.DEFAULT_LINEAR_SPEED,
                max_angular_speed=self.DEFAULT_ANGULAR_SPEED,
                requires_contact=True,
            ),
        )

    def requires_unlock_pulse(self) -> bool:
        return False

    def evaluate_unlock(self, seated_verified: bool, unloaded_verified: bool) -> UnlockDecision:
        return UnlockDecision(
            allowed=False,
            error_code=ErrorCode.UNLOCK_FAILED,
            reason="V1 被动舌规无解锁机构，禁止任何解锁动作",
        )


class MagneticLatchV2Strategy(ToolStrategy):
    """V2：上方插入 + 机械锁止 + 电磁解锁后退出。

    解锁前置条件（硬约束）：seated_verified AND unloaded_verified，
    否则拒绝解锁并返回明确错误（未落座 -> 400；已落座但未卸载 -> 420）。
    """

    IO_CHANNELS: Tuple[str, ...] = ("electromagnetic_unlock",)
    DEFAULT_PULSE_MS = 200

    def __init__(self, pulse_ms: int = DEFAULT_PULSE_MS) -> None:
        self._pulse_ms = int(pulse_ms)

    def type(self) -> str:
        return TOOL_MAGNETIC_LATCH_V2

    def make_acquire_plan(self, ring_pose: Any) -> Sequence[MotionPrimitive]:
        return (
            MotionPrimitive(
                name="top_insert",
                target_pose=ring_pose,
                max_linear_speed=self.DEFAULT_LINEAR_SPEED,
                max_angular_speed=self.DEFAULT_ANGULAR_SPEED,
                requires_contact=False,
            ),
        )

    def make_engage_plan(self, ring_pose: Any) -> Sequence[MotionPrimitive]:
        # 从上方下降插入，触发机械锁止
        return (
            MotionPrimitive(
                name="auto_lock",
                target_pose=ring_pose,
                max_linear_speed=self.CONTACT_LINEAR_SPEED,
                max_angular_speed=0.05,
                requires_contact=True,
            ),
        )

    def make_release_plan(self, seat_pose: Any) -> Sequence[MotionPrimitive]:
        # 小幅调整让锁扣不承重（不涉及电磁 IO）
        return (
            MotionPrimitive(
                name="unload",
                target_pose=seat_pose,
                max_linear_speed=self.CONTACT_LINEAR_SPEED,
                max_angular_speed=0.05,
                requires_contact=True,
            ),
        )

    def make_disengage_plan(self, seat_pose: Any) -> Sequence[MotionPrimitive]:
        # 注意：不得假设一定能垂直向上退出，退出方向由 V2 CAD/实测确定
        return (
            MotionPrimitive(
                name="withdraw",
                target_pose=seat_pose,
                max_linear_speed=self.DEFAULT_LINEAR_SPEED,
                max_angular_speed=self.DEFAULT_ANGULAR_SPEED,
                requires_contact=True,
            ),
        )

    def requires_unlock_pulse(self) -> bool:
        return True

    def unlock_pulse_ms(self) -> int:
        return self._pulse_ms

    def evaluate_unlock(self, seated_verified: bool, unloaded_verified: bool) -> UnlockDecision:
        if not seated_verified:
            return UnlockDecision(
                allowed=False,
                error_code=ErrorCode.SEAT_NOT_VERIFIED,
                reason="未确认电池由桌面承载，禁止发出电磁解锁脉冲",
            )
        if not unloaded_verified:
            return UnlockDecision(
                allowed=False,
                error_code=ErrorCode.RELEASE_NOT_VERIFIED,
                reason="锁扣仍承担主要重量（未确认卸载），禁止解锁",
            )
        return UnlockDecision(
            allowed=True,
            error_code=ErrorCode.OK,
            reason="已确认落座且已卸载，允许解锁",
            pulse_ms=self._pulse_ms,
        )


#: 名称 -> 策略工厂（供参数化装配）
STRATEGY_REGISTRY = {
    TOOL_PASSIVE_HOOK_V1: PassiveHookV1Strategy,
    TOOL_MAGNETIC_LATCH_V2: MagneticLatchV2Strategy,
}


def make_strategy(tool_type: str, **kwargs: Any) -> ToolStrategy:
    try:
        factory = STRATEGY_REGISTRY[tool_type]
    except KeyError:
        raise ValueError("未知工具类型: %r（可选: %s）" % (tool_type, sorted(STRATEGY_REGISTRY)))
    return factory(**kwargs)


def primitive_names(primitives: Sequence[MotionPrimitive]) -> List[str]:
    return [item.name for item in primitives]
