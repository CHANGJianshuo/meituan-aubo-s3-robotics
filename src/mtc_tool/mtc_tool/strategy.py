"""工具策略接口与 V1/V2 两代实现（设计文档 §5.1 / §5.2 的落地）。

设计契约（务必遵守）：

1. ``ToolStrategy`` 只把「提环位姿 / 落座位姿」翻译为**有约束的运动原语列表**
   （``MotionPrimitive``）。它不掌握关节控制权，不产生关节角命令，不调用任何 SDK。
2. V1 ``PassiveHookV1`` 是纯机械舌规：**任何阶段都不涉及电磁/夹爪 IO**。
3. V2 ``MagneticLatchV2`` 需要解锁脉冲，但脉冲的实际发送由 ``ToolManager`` 通过
   ``UnlockIoPort`` 完成；策略本身只声明「需要解锁」并给出保持位姿的原语。
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

from .codes import (
    STAGE_DISENGAGE,
    STAGE_ENGAGE,
    STAGE_PRE_ALIGN,
    STAGE_RETREAT,
    STAGE_SEAT,
    STAGE_UNLOAD,
    TOOL_TYPE_MAGNETIC_LATCH_V2,
    TOOL_TYPE_PASSIVE_HOOK_V1,
)


@dataclass(frozen=True)
class PoseTarget:
    """目标位姿（``geometry_msgs/PoseStamped`` 的纯 Python 等价物）。

    仅承载数据；实际规划/执行由 ``motion_executor`` 负责。
    """

    frame_id: str
    position_xyz: Tuple[float, float, float]
    orientation_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def translated(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> "PoseTarget":
        x, y, z = self.position_xyz
        return PoseTarget(self.frame_id, (x + dx, y + dy, z + dz), self.orientation_xyzw)


@dataclass(frozen=True)
class MotionPrimitive:
    """一个运动原语（对应设计文档 §5.1 ``struct MotionPrimitive``）。

    字段与文档一致：``name`` / ``target`` / ``max_linear_speed_m_s`` /
    ``max_angular_speed_rad_s`` / ``requires_contact``。

    本实现的扩展字段（文档未列出，已在包 README 记为偏差）：

    - ``stage``: 统一阶段名（§5.2 动作矩阵左列），供上层 FSM 关联日志与联锁。
    - ``note``: 人类可读备注，仅用于日志。

    该结构**没有**任何关节、力矩或 SDK 句柄字段，也不可能产生关节级命令。
    """

    name: str
    target: PoseTarget
    max_linear_speed_m_s: float
    max_angular_speed_rad_s: float
    requires_contact: bool
    stage: str = ""
    note: str = ""

    def is_motion_primitive(self) -> bool:
        """恒定返回 ``True``：本类型是工具层唯一允许的输出类型。"""

        return True


class ToolStrategy(abc.ABC):
    """工具策略抽象基类（设计文档 §5.1 的 Python 落地）。

    子类必须实现 :meth:`make_acquire_plan` / :meth:`make_release_plan` /
    :meth:`requires_unlock_pulse`。基类不持有任何 IO 或执行器引用。
    """

    #: ``ToolState.msg`` 的 ``tool_type`` 取值。
    TOOL_TYPE: str = "unknown"

    #: 该策略是否使用电磁解锁 IO。V1 必须为 ``False``。
    USES_UNLOCK_IO: bool = False

    @property
    def tool_type(self) -> str:
        return self.TOOL_TYPE

    @property
    def uses_unlock_io(self) -> bool:
        return self.USES_UNLOCK_IO

    @property
    def acquire_stages(self) -> Sequence[str]:
        """抓取阶段顺序（用于日志与上层 FSM 对齐）。"""

        return (STAGE_PRE_ALIGN, STAGE_ENGAGE)

    @property
    def release_stages(self) -> Sequence[str]:
        """释放阶段顺序（用于日志与上层 FSM 对齐）。"""

        return (STAGE_UNLOAD, STAGE_DISENGAGE, STAGE_RETREAT)

    @abc.abstractmethod
    def make_acquire_plan(self, ring_pose: PoseTarget) -> List[MotionPrimitive]:
        """由提环位姿生成接合（穿环/插入 + 锁止/挂钩）运动原语。"""

        raise NotImplementedError

    @abc.abstractmethod
    def make_release_plan(self, seat_pose: PoseTarget) -> List[MotionPrimitive]:
        """由落座位姿生成卸载 + 分离 + 撤离运动原语。"""

        raise NotImplementedError

    @abc.abstractmethod
    def requires_unlock_pulse(self) -> bool:
        """本策略的 DISENGAGE 是否需要电磁解锁脉冲。"""

        raise NotImplementedError

    def describe(self) -> str:
        return "%s(uses_unlock_io=%s)" % (self.tool_type, self.uses_unlock_io)


# ---------------------------------------------------------------------------
# V1：被动舌规（side_insert / hook_engage / unload / side_withdraw）
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class PassiveHookParams:
    """V1 几何/速度参数。

    **所有数值均为未标定的示意占位值**，注释见设计文档 §8；上实机前必须由
    标定结果替换，且不得以本文件的默认值驱动真实机械臂。
    """

    side_approach_offset_m: float = -0.06   # 未标定占位值
    insert_depth_m: float = 0.03            # 未标定占位值
    hook_lift_m: float = 0.01               # 未标定占位值
    unload_drop_m: float = -0.005           # 未标定占位值
    retreat_offset_m: float = -0.08         # 未标定占位值
    contact_linear_speed_m_s: float = 0.01  # 未标定占位值
    contact_angular_speed_rad_s: float = 0.05
    free_linear_speed_m_s: float = 0.05
    free_angular_speed_rad_s: float = 0.2


class PassiveHookV1(ToolStrategy):
    """V1 被动舌规：侧向穿环 → 挂钩 → 卸载 → 反向退钩。

    **零电磁 / 零夹爪 IO**：本类不接收、不保存、也不可能调用 ``UnlockIoPort``。
    """

    TOOL_TYPE = TOOL_TYPE_PASSIVE_HOOK_V1
    USES_UNLOCK_IO = False

    def __init__(self, params: PassiveHookParams | None = None) -> None:
        self._params = params or PassiveHookParams()

    @property
    def params(self) -> PassiveHookParams:
        return self._params

    def make_acquire_plan(self, ring_pose: PoseTarget) -> List[MotionPrimitive]:
        p = self._params
        side = ring_pose.translated(dx=p.side_approach_offset_m)
        engaged = ring_pose.translated(dx=p.insert_depth_m)
        hooked = engaged.translated(dz=p.hook_lift_m)
        return [
            MotionPrimitive(
                name="side_insert_pre_align",
                target=side,
                max_linear_speed_m_s=p.free_linear_speed_m_s,
                max_angular_speed_rad_s=p.free_angular_speed_rad_s,
                requires_contact=False,
                stage=STAGE_PRE_ALIGN,
                note="提环开口侧面进环准备位（未标定占位几何）",
            ),
            MotionPrimitive(
                name="side_insert",
                target=engaged,
                max_linear_speed_m_s=p.contact_linear_speed_m_s,
                max_angular_speed_rad_s=p.contact_angular_speed_rad_s,
                requires_contact=True,
                stage=STAGE_ENGAGE,
                note="横向穿环：接触敏感，需低速受监护执行",
            ),
            MotionPrimitive(
                name="hook_engage",
                target=hooked,
                max_linear_speed_m_s=p.contact_linear_speed_m_s,
                max_angular_speed_rad_s=p.contact_angular_speed_rad_s,
                requires_contact=True,
                stage=STAGE_ENGAGE,
                note="适当上提完成挂钩；仍需试提取得独立证据",
            ),
        ]

    def make_release_plan(self, seat_pose: PoseTarget) -> List[MotionPrimitive]:
        p = self._params
        unloaded = seat_pose.translated(dz=p.unload_drop_m)
        withdrawn = seat_pose.translated(dx=p.retreat_offset_m)
        return [
            MotionPrimitive(
                name="seat_unload",
                target=unloaded,
                max_linear_speed_m_s=p.contact_linear_speed_m_s,
                max_angular_speed_rad_s=p.contact_angular_speed_rad_s,
                requires_contact=True,
                stage=STAGE_UNLOAD,
                note="小幅调整使倒钩不承重",
            ),
            MotionPrimitive(
                name="side_withdraw",
                target=withdrawn,
                max_linear_speed_m_s=p.contact_linear_speed_m_s,
                max_angular_speed_rad_s=p.contact_angular_speed_rad_s,
                requires_contact=True,
                stage=STAGE_DISENGAGE,
                note="沿标定反向路径退钩；结果未知时禁止强行抽出",
            ),
            MotionPrimitive(
                name="retreat",
                target=withdrawn.translated(dx=p.retreat_offset_m),
                max_linear_speed_m_s=p.free_linear_speed_m_s,
                max_angular_speed_rad_s=p.free_angular_speed_rad_s,
                requires_contact=False,
                stage=STAGE_RETREAT,
                note="净空后离开",
            ),
        ]

    def requires_unlock_pulse(self) -> bool:
        return False


# ---------------------------------------------------------------------------
# V2：机械锁止 + 电磁解锁（top_insert / auto_lock / unload /
#                            electromagnetic_unlock / withdraw）
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class MagneticLatchParams:
    """V2 几何/速度参数（同为未标定示意占位值）。"""

    top_approach_height_m: float = 0.06     # 未标定占位值
    insert_depth_m: float = 0.015           # 未标定占位值
    lock_seat_depth_m: float = 0.003        # 未标定占位值
    unload_drop_m: float = -0.004           # 未标定占位值
    withdraw_offset_m: float = 0.05         # 未标定占位值
    contact_linear_speed_m_s: float = 0.01  # 未标定占位值
    contact_angular_speed_rad_s: float = 0.05
    free_linear_speed_m_s: float = 0.05
    free_angular_speed_rad_s: float = 0.2


class MagneticLatchV2(ToolStrategy):
    """V2 上方插入 + 机械锁止 + 电磁解锁。

    解锁脉冲不由本类发送；:meth:`make_release_plan` 在 ``DISENGAGE`` 阶段给出一个
    **零位移保持原语**（``electromagnetic_unlock``），执行层在此期间保持位姿不动，
    由 ``ToolManager`` 在联锁通过后通过 ``UnlockIoPort`` 发送脉冲并等待独立证据。

    设计文档 §5.2 明确：不得假设 V2 解锁后必定能垂直向上退出，退出方向必须由
    CAD 装配与实物测试确定（见 :attr:`MagneticLatchParams.withdraw_offset_m` 注释）。
    """

    TOOL_TYPE = TOOL_TYPE_MAGNETIC_LATCH_V2
    USES_UNLOCK_IO = True

    def __init__(self, params: MagneticLatchParams | None = None) -> None:
        self._params = params or MagneticLatchParams()

    @property
    def params(self) -> MagneticLatchParams:
        return self._params

    def make_acquire_plan(self, ring_pose: PoseTarget) -> List[MotionPrimitive]:
        p = self._params
        above = ring_pose.translated(dz=p.top_approach_height_m)
        inserted = ring_pose.translated(dz=p.insert_depth_m)
        locked = ring_pose.translated(dz=p.lock_seat_depth_m)
        return [
            MotionPrimitive(
                name="top_insert_pre_align",
                target=above,
                max_linear_speed_m_s=p.free_linear_speed_m_s,
                max_angular_speed_rad_s=p.free_angular_speed_rad_s,
                requires_contact=False,
                stage=STAGE_PRE_ALIGN,
                note="提环上方对中位（未标定占位几何）",
            ),
            MotionPrimitive(
                name="top_insert",
                target=inserted,
                max_linear_speed_m_s=p.contact_linear_speed_m_s,
                max_angular_speed_rad_s=p.contact_angular_speed_rad_s,
                requires_contact=True,
                stage=STAGE_ENGAGE,
                note="从上方下降插入提环",
            ),
            MotionPrimitive(
                name="auto_lock",
                target=locked,
                max_linear_speed_m_s=p.contact_linear_speed_m_s,
                max_angular_speed_rad_s=p.contact_angular_speed_rad_s,
                requires_contact=True,
                stage=STAGE_ENGAGE,
                note="触发机械锁止；锁止开关反馈为独立证据",
            ),
        ]

    def make_release_plan(self, seat_pose: PoseTarget) -> List[MotionPrimitive]:
        p = self._params
        unloaded = seat_pose.translated(dz=p.unload_drop_m)
        return [
            MotionPrimitive(
                name="seat_unload",
                target=unloaded,
                max_linear_speed_m_s=p.contact_linear_speed_m_s,
                max_angular_speed_rad_s=p.contact_angular_speed_rad_s,
                requires_contact=True,
                stage=STAGE_UNLOAD,
                note="小幅调整让锁扣不承重",
            ),
            MotionPrimitive(
                name="electromagnetic_unlock",
                target=unloaded,
                max_linear_speed_m_s=0.0,
                max_angular_speed_rad_s=0.0,
                requires_contact=True,
                stage=STAGE_DISENGAGE,
                note="零位移保持位姿：脉冲由 ToolManager 经 UnlockIoPort 发送，策略不碰 IO",
            ),
            MotionPrimitive(
                name="withdraw",
                target=unloaded.translated(dz=p.withdraw_offset_m),
                max_linear_speed_m_s=p.contact_linear_speed_m_s,
                max_angular_speed_rad_s=p.contact_angular_speed_rad_s,
                requires_contact=True,
                stage=STAGE_DISENGAGE,
                note="仅在解锁已由独立证据确认后允许执行；退出方向待 CAD/实测确定",
            ),
            MotionPrimitive(
                name="retreat",
                target=unloaded.translated(dz=2.0 * p.withdraw_offset_m),
                max_linear_speed_m_s=p.free_linear_speed_m_s,
                max_angular_speed_rad_s=p.free_angular_speed_rad_s,
                requires_contact=False,
                stage=STAGE_RETREAT,
                note="净空后离开",
            ),
        ]

    def requires_unlock_pulse(self) -> bool:
        return True
