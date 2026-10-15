"""mtc_tool 离线单元测试（纯 Python 3，**不 import rclpy**）。

运行方式（无需编译、无需 source ROS）::

    cd /home/robot/meituan_challenge_ws/src/mtc_tool
    python3 -m pytest test/ -v
    # 或者
    python3 -m unittest discover -s test -v

覆盖内容：任务书 6 条强约束 + V1/V2 正常流程 + 「脉冲受理但无证据不得宣告解锁成功」
+ ``ToolState.msg`` / ``ErrorCodes.msg`` / ``TriggerUnlock.srv`` 数值与字段一致性。
"""

from __future__ import annotations

import os
import re
import sys
import unittest
from pathlib import Path

_PKG_DIR = Path(__file__).resolve().parents[1]
if str(_PKG_DIR) not in sys.path:
    sys.path.insert(0, str(_PKG_DIR))

import mtc_tool  # noqa: E402
from mtc_tool import codes  # noqa: E402
from mtc_tool.io_port import (  # noqa: E402
    DisabledUnlockIo,
    MockUnlockIo,
    UnlockIoDisabledError,
)
from mtc_tool.manager import ToolManager, is_primitive_sequence  # noqa: E402
from mtc_tool.strategy import (  # noqa: E402
    MagneticLatchV2,
    MotionPrimitive,
    PassiveHookV1,
    PoseTarget,
)

# ---------------------------------------------------------------------------
# 目录定位（只读引用 mtc_interfaces，绝不修改）
# ---------------------------------------------------------------------------
WS_ROOT = _PKG_DIR.parents[1]  # /home/robot/meituan_challenge_ws
MSG_DIR = WS_ROOT / "src" / "mtc_interfaces" / "msg"
SRV_DIR = WS_ROOT / "src" / "mtc_interfaces" / "srv"
CORE_MODULES = ("codes.py", "strategy.py", "manager.py", "io_port.py", "__init__.py")

RING = PoseTarget("ring_frame", (0.30, 0.10, 0.20))
SEAT = PoseTarget("P1_frame", (0.45, -0.10, 0.05))

#: 工具层绝不出现的关键字（关节级控制 / SDK / 网络）。
FORBIDDEN_SOURCE_TOKENS = (
    "moveJoint",
    "move_joint",
    "send_joint",
    "set_joint",
    "joint_targets",
    "target_rad",
    "FollowJointTrajectory",
    "pyaubo",
    "aubo_sdk",
    "import rclpy",
    "rtde",
    "socket",
    "serial",
    "urllib",
    "requests",
)

FORBIDDEN_API_PATTERN = re.compile(
    r"(move_?joint|send_?joint|set_?joint|servo_?j|follow_?joint|move_?linear)",
    re.IGNORECASE,
)


def parse_msg_constants(path: Path):
    """解析 ``.msg`` 中的 ``<type> NAME=VALUE`` 常量行。"""

    constants = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        match = re.match(r"^(?:uint8|uint16|int32)\s+([A-Z0-9_]+)\s*=\s*(-?\d+)$", line)
        if match:
            constants[match.group(1)] = int(match.group(2))
    return constants


def parse_idl_fields(path: Path, section: int = 0):
    """解析 ``.msg`` / ``.srv`` 的字段名（``section`` 用于 ``.srv`` 的请求段/响应段）。"""

    text = path.read_text(encoding="utf-8")
    if path.suffix == ".srv":
        text = text.split("---")[section]
    fields = []
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or "=" in line:
            continue
        parts = line.split()
        if len(parts) >= 2:
            fields.append(parts[-1])
    return fields


class _UnlockIoSentinel(MockUnlockIo):
    """带调用计数的替身：用于断言「IO 端口一次都没被调用」。"""

    def __init__(self, mode: str = MockUnlockIo.MODE_ACCEPT) -> None:
        super().__init__(mode=mode, name="sentinel_unlock_io")
        self.observed_pulses = []

    def send_unlock_pulse(self, pulse_ms: int) -> bool:
        self.observed_pulses.append(int(pulse_ms))
        return super().send_unlock_pulse(pulse_ms)


def make_manager(strategy, io_mode: str = MockUnlockIo.MODE_ACCEPT):
    io = _UnlockIoSentinel(mode=io_mode)
    return ToolManager(strategy=strategy, unlock_io=io), io


def v2_ready_for_unlock(io_mode: str = MockUnlockIo.MODE_ACCEPT):
    """把 V2 推进到「已落座 + 已卸载，等待解锁脉冲」的状态。"""

    manager, io = make_manager(MagneticLatchV2(), io_mode)
    assert manager.begin_acquire("battery_blue", RING).ok
    manager.notify_engage_complete()
    assert manager.verify_attach(codes.EVIDENCE_GEOMETRY, "test_lift").verified
    assert manager.begin_release(SEAT).ok
    manager.mark_seated(True, codes.EVIDENCE_GEOMETRY, "table_contact")
    manager.mark_unloaded(True, codes.EVIDENCE_GEOMETRY, "load_cell")
    return manager, io


def v1_with_verified_attach():
    manager, io = make_manager(PassiveHookV1())
    assert manager.begin_acquire("battery_red", RING).ok
    manager.notify_engage_complete()
    assert manager.verify_attach(codes.EVIDENCE_GEOMETRY, "test_lift").verified
    return manager, io


# ===========================================================================
# 约束 1：V2 解锁前置条件（已落座 + 已卸载），否则 500 且不调用 IO
# ===========================================================================
class TestUnlockPreconditions(unittest.TestCase):
    def test_seated_and_unloaded_required(self):
        cases = (
            ("seated_only", True, False),
            ("unloaded_only", False, True),
            ("neither", False, False),
        )
        for name, seated, unloaded in cases:
            with self.subTest(case=name):
                manager, io = v2_ready_for_unlock()
                manager.mark_seated(seated, codes.EVIDENCE_GEOMETRY)
                manager.mark_unloaded(unloaded, codes.EVIDENCE_GEOMETRY)
                outcome = manager.trigger_unlock("req-%s" % name, 300)
                self.assertFalse(outcome.accepted)
                self.assertEqual(outcome.error_code, codes.ERROR_SAFETY_INTERLOCK)
                self.assertEqual(outcome.error_code, 500)
                self.assertFalse(outcome.unlock_confirmed)
                self.assertFalse(manager.status().verified)
                self.assertEqual(io.call_count, 0, "联锁未通过时不得调用 IO 端口")
                self.assertEqual(io.observed_pulses, [])

    def test_seated_evidence_flag_must_be_explicitly_true(self):
        manager, io = v2_ready_for_unlock()
        # 重新进入释放流程：标志复位后必须先确认落座/卸载
        manager.mark_seated(False)
        manager.mark_unloaded(False)
        self.assertEqual(manager.status().seated_verified, False)
        outcome = manager.trigger_unlock("req-reset", 300)
        self.assertEqual(outcome.error_code, codes.ERROR_SAFETY_INTERLOCK)
        self.assertEqual(io.call_count, 0)

    def test_unlock_requires_releasing_stage(self):
        manager, io = make_manager(MagneticLatchV2())
        manager.begin_acquire("battery_blue", RING)
        manager.notify_engage_complete()
        manager.verify_attach(codes.EVIDENCE_GEOMETRY, "test_lift")
        # 还在 ATTACHED（未 begin_release）：即使置位两个标志也必须拒绝
        manager.mark_seated(True, codes.EVIDENCE_GEOMETRY)
        manager.mark_unloaded(True, codes.EVIDENCE_GEOMETRY)
        outcome = manager.trigger_unlock("req-stage", 300)
        self.assertEqual(outcome.error_code, codes.ERROR_SAFETY_INTERLOCK)
        self.assertEqual(io.call_count, 0)

    def test_invalid_pulse_ms_rejected_without_io(self):
        manager, io = v2_ready_for_unlock()
        for bad in (0, -1, 1.5, "300", None):
            with self.subTest(pulse_ms=bad):
                outcome = manager.trigger_unlock("req-bad", bad)
                self.assertFalse(outcome.accepted)
                self.assertEqual(outcome.error_code, codes.ERROR_EXECUTION_REJECTED)
        self.assertEqual(io.call_count, 0)


# ===========================================================================
# 约束 2：accepted=True 不等于解锁成功；需独立证据
# ===========================================================================
class TestAcceptedIsNotUnlock(unittest.TestCase):
    def test_accepted_true_does_not_confirm_unlock(self):
        manager, io = v2_ready_for_unlock()
        outcome = manager.trigger_unlock("req-1", 300)
        self.assertTrue(outcome.accepted, "脉冲请求应被受理")
        self.assertEqual(io.call_count, 1)
        self.assertFalse(outcome.unlock_confirmed)
        self.assertFalse(outcome.verified)
        status = manager.status()
        self.assertEqual(status.state, codes.STATE_RELEASING)
        self.assertEqual(status.evidence_level, codes.EVIDENCE_COMMAND_ONLY)
        self.assertFalse(status.unlock_confirmed)
        self.assertFalse(status.withdraw_allowed)

    def test_command_only_evidence_cannot_confirm_unlock(self):
        manager, io = v2_ready_for_unlock()
        manager.trigger_unlock("req-2", 300)
        confirmation = manager.confirm_unlock(codes.EVIDENCE_COMMAND_ONLY, "do_output_signal")
        self.assertEqual(confirmation.error_code, codes.ERROR_UNLOCK_FAILED)
        self.assertEqual(confirmation.error_code, 410)
        self.assertFalse(confirmation.unlock_confirmed)
        status = manager.status()
        self.assertEqual(status.state, codes.STATE_FAULT, "无独立证据必须保持保守 FAULT")
        self.assertFalse(status.verified)
        # 之后仍不得抽出
        decision = manager.request_withdraw()
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.error_code, codes.ERROR_RELEASE_NOT_VERIFIED)

    def test_geometry_evidence_confirms_unlock(self):
        manager, io = v2_ready_for_unlock()
        self.assertTrue(manager.trigger_unlock("req-3", 250).accepted)
        confirmation = manager.confirm_unlock(codes.EVIDENCE_GEOMETRY, "latch_switch")
        self.assertEqual(confirmation.error_code, codes.ERROR_OK)
        self.assertTrue(confirmation.unlock_confirmed)
        self.assertTrue(confirmation.verified)
        self.assertFalse(manager.status().withdraw_allowed, "解锁已确认但分离尚未验证")
        decision = manager.resolve_disengage(
            codes.DISENGAGE_CONFIRMED, codes.EVIDENCE_GEOMETRY, "latch_switch"
        )
        self.assertTrue(decision.allowed)
        self.assertEqual(manager.status().state, codes.STATE_DETACHED)

    def test_confirm_unlock_without_accepted_pulse_is_unlock_failed(self):
        manager, io = v2_ready_for_unlock()
        confirmation = manager.confirm_unlock(codes.EVIDENCE_SENSOR_OR_VISION, "vision")
        self.assertEqual(confirmation.error_code, codes.ERROR_UNLOCK_FAILED)
        self.assertEqual(manager.status().state, codes.STATE_FAULT)
        self.assertFalse(manager.status().unlock_confirmed)

    def test_service_contract_uses_accepted_not_success(self):
        srv = SRV_DIR / "TriggerUnlock.srv"
        if not srv.exists():
            self.skipTest("TriggerUnlock.srv 不存在")
        response_fields = parse_idl_fields(srv, section=1)
        self.assertEqual(response_fields, ["accepted", "message"])
        self.assertNotIn("success", response_fields)
        self.assertNotIn("unlocked", response_fields)
        self.assertNotIn("released", response_fields)


# ===========================================================================
# 约束 3：V1 不得依赖电磁 IO
# ===========================================================================
class TestV1NeverUsesUnlockIo(unittest.TestCase):
    def test_full_v1_cycle_touches_no_io(self):
        manager, io = v1_with_verified_attach()
        plan = manager.begin_release(SEAT)
        self.assertTrue(plan.ok)
        manager.mark_seated(True, codes.EVIDENCE_GEOMETRY, "table_contact")
        manager.mark_unloaded(True, codes.EVIDENCE_GEOMETRY, "load_cell")
        decision = manager.resolve_disengage(
            codes.DISENGAGE_CONFIRMED, codes.EVIDENCE_GEOMETRY, "hook_follow"
        )
        self.assertTrue(decision.allowed)
        self.assertEqual(manager.status().state, codes.STATE_DETACHED)
        self.assertEqual(io.call_count, 0, "V1 全流程不得调用解锁 IO")
        self.assertEqual(io.observed_pulses, [])

    def test_v1_unlock_request_rejected_by_interlock(self):
        manager, io = v1_with_verified_attach()
        manager.begin_release(SEAT)
        manager.mark_seated(True, codes.EVIDENCE_GEOMETRY)
        manager.mark_unloaded(True, codes.EVIDENCE_GEOMETRY)
        outcome = manager.trigger_unlock("req-v1", 300)
        self.assertFalse(outcome.accepted)
        self.assertEqual(outcome.error_code, codes.ERROR_SAFETY_INTERLOCK)
        self.assertEqual(io.call_count, 0, "V1 被拒绝时也不得调用 IO")

    def test_v1_strategy_declares_no_unlock_io(self):
        strategy = PassiveHookV1()
        self.assertFalse(strategy.requires_unlock_pulse())
        self.assertFalse(strategy.uses_unlock_io)
        self.assertEqual(strategy.tool_type, codes.TOOL_TYPE_PASSIVE_HOOK_V1)


# ===========================================================================
# 约束 4：DISENGAGE 失败/未知不得强行抽出
# ===========================================================================
class TestDisengageFailureBlocksWithdraw(unittest.TestCase):
    def _v2_unlock_confirmed(self):
        manager, io = v2_ready_for_unlock()
        manager.trigger_unlock("req-dis", 300)
        manager.confirm_unlock(codes.EVIDENCE_GEOMETRY, "latch_switch")
        return manager, io

    def test_withdraw_before_disengage_resolution_is_blocked(self):
        manager, io = self._v2_unlock_confirmed()
        decision = manager.request_withdraw()
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.error_code, codes.ERROR_RELEASE_NOT_VERIFIED)
        self.assertEqual(decision.error_code, 420)
        self.assertEqual(manager.status().state, codes.STATE_FAULT)
        self.assertFalse(manager.status().withdraw_allowed)

    def test_disengage_failed_blocks_withdraw(self):
        manager, io = self._v2_unlock_confirmed()
        decision = manager.resolve_disengage(codes.DISENGAGE_FAILED, codes.EVIDENCE_GEOMETRY, "attempt")
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.error_code, codes.ERROR_RELEASE_NOT_VERIFIED)
        self.assertEqual(manager.status().state, codes.STATE_FAULT)
        self.assertFalse(manager.status().withdraw_allowed)

    def test_disengage_unknown_blocks_withdraw(self):
        manager, io = self._v2_unlock_confirmed()
        decision = manager.resolve_disengage(codes.DISENGAGE_UNKNOWN, codes.EVIDENCE_NONE, "feedback_lost")
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.error_code, codes.ERROR_RELEASE_NOT_VERIFIED)
        self.assertEqual(manager.status().state, codes.STATE_FAULT)
        self.assertFalse(manager.status().withdraw_allowed)

    def test_disengage_confirmed_without_evidence_is_rejected(self):
        manager, io = self._v2_unlock_confirmed()
        decision = manager.resolve_disengage(codes.DISENGAGE_CONFIRMED, codes.EVIDENCE_COMMAND_ONLY)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.error_code, codes.ERROR_RELEASE_NOT_VERIFIED)
        self.assertEqual(manager.status().state, codes.STATE_FAULT)

    def test_v2_cannot_claim_release_without_confirmed_unlock(self):
        manager, io = v2_ready_for_unlock()
        manager.trigger_unlock("req-noev", 300)
        decision = manager.resolve_disengage(codes.DISENGAGE_CONFIRMED, codes.EVIDENCE_SENSOR_OR_VISION)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.error_code, codes.ERROR_UNLOCK_FAILED)
        self.assertEqual(manager.status().state, codes.STATE_FAULT)

    def test_fault_requires_explicit_operator_clear(self):
        manager, io = self._v2_unlock_confirmed()
        manager.resolve_disengage(codes.DISENGAGE_UNKNOWN, codes.EVIDENCE_NONE)
        self.assertFalse(manager.begin_acquire("battery_blue", RING).ok)
        with self.assertRaises(ValueError):
            manager.clear_fault("")
        self.assertTrue(manager.clear_fault("现场人工核验通过"))
        self.assertEqual(manager.status().state, codes.STATE_UNKNOWN)


# ===========================================================================
# 约束 5：state 与 verified 相互独立
# ===========================================================================
class TestVerifiedIsIndependentDimension(unittest.TestCase):
    def test_attached_with_command_only_is_not_verified(self):
        for strategy in (PassiveHookV1(), MagneticLatchV2()):
            with self.subTest(tool_type=strategy.tool_type):
                manager, io = make_manager(strategy)
                manager.begin_acquire("battery_blue", RING)
                status = manager.notify_engage_complete()
                self.assertEqual(status.state, codes.STATE_ATTACHED)
                self.assertEqual(status.evidence_level, codes.EVIDENCE_COMMAND_ONLY)
                self.assertFalse(status.verified, "命令级证据不得置 verified=True")

    def test_geometry_evidence_sets_verified(self):
        manager, io = make_manager(PassiveHookV1())
        manager.begin_acquire("battery_blue", RING)
        manager.notify_engage_complete()
        status = manager.verify_attach(codes.EVIDENCE_GEOMETRY, "lift_follow")
        self.assertEqual(status.state, codes.STATE_ATTACHED)
        self.assertTrue(status.verified)
        self.assertEqual(status.evidence_level, codes.EVIDENCE_GEOMETRY)

    def test_verified_and_state_are_separate_fields(self):
        status = codes.STATE_ATTACHED
        manager, io = make_manager(MagneticLatchV2())
        manager.begin_acquire("battery_blue", RING)
        manager.notify_engage_complete()
        snapshot = manager.status()
        # 同一个 state 对应两种 verified，证明二者独立
        self.assertEqual(snapshot.state, status)
        self.assertFalse(snapshot.verified)
        manager.verify_attach(codes.EVIDENCE_GEOMETRY, "latch_switch")
        updated = manager.status()
        self.assertEqual(updated.state, status)
        self.assertTrue(updated.verified)

    def test_manual_attach_evidence_below_threshold_is_310(self):
        manager, io = make_manager(PassiveHookV1())
        manager.begin_acquire("battery_blue", RING)
        manager.notify_engage_complete()
        status = manager.verify_attach(codes.EVIDENCE_COMMAND_ONLY, "motion_done")
        self.assertFalse(status.verified)
        self.assertEqual(status.error_code, codes.ERROR_ATTACH_NOT_VERIFIED)
        self.assertFalse(manager.begin_release(SEAT).ok, "未验证挂载禁止进入释放/运输")


# ===========================================================================
# 约束 6：只产生 MotionPrimitive，无关节命令 / 无 SDK
# ===========================================================================
class TestNoJointLevelOutput(unittest.TestCase):
    def test_plans_contain_only_motion_primitives(self):
        for strategy in (PassiveHookV1(), MagneticLatchV2()):
            with self.subTest(tool_type=strategy.tool_type):
                manager, io = make_manager(strategy)
                acquire = manager.begin_acquire("battery_blue", RING)
                self.assertTrue(acquire.ok)
                self.assertTrue(is_primitive_sequence(acquire.primitives))
                manager.notify_engage_complete()
                manager.verify_attach(codes.EVIDENCE_GEOMETRY, "lift_follow")
                release = manager.begin_release(SEAT)
                self.assertTrue(release.ok)
                self.assertTrue(is_primitive_sequence(release.primitives))
                for primitive in list(acquire.primitives) + list(release.primitives):
                    self.assertIsInstance(primitive, MotionPrimitive)
                    for banned in ("joints", "joint_names", "target_rad", "torque", "sdk_handle"):
                        self.assertFalse(
                            hasattr(primitive, banned),
                            "运动原语不得携带关节级字段：%s" % banned,
                        )

    def test_manager_and_strategy_expose_no_joint_api(self):
        manager, io = make_manager(MagneticLatchV2())
        for obj in (manager, MagneticLatchV2(), PassiveHookV1()):
            for name in dir(obj):
                if name.startswith("__"):
                    continue
                self.assertIsNone(
                    FORBIDDEN_API_PATTERN.search(name),
                    "工具层不得暴露关节级接口：%s" % name,
                )

    def test_core_modules_have_no_joint_or_sdk_tokens(self):
        for filename in CORE_MODULES:
            path = _PKG_DIR / "mtc_tool" / filename
            self.assertTrue(path.exists(), "核心模块缺失：%s" % path)
            text = path.read_text(encoding="utf-8")
            for token in FORBIDDEN_SOURCE_TOKENS:
                self.assertNotIn(
                    token,
                    text,
                    "%s 中出现了禁止的关节/SDK/网络关键字：%s" % (filename, token),
                )

    def test_plans_generated_from_pose_only(self):
        strategy = MagneticLatchV2()
        pose = PoseTarget("ring_frame", (0.1, 0.2, 0.3))
        primitives = strategy.make_acquire_plan(pose)
        names = [p.name for p in primitives]
        self.assertEqual(names, ["top_insert_pre_align", "top_insert", "auto_lock"])
        stages = [p.stage for p in primitives]
        self.assertEqual(stages, [codes.STAGE_PRE_ALIGN, codes.STAGE_ENGAGE, codes.STAGE_ENGAGE])
        release_names = [p.name for p in strategy.make_release_plan(SEAT)]
        self.assertEqual(
            release_names,
            ["seat_unload", "electromagnetic_unlock", "withdraw", "retreat"],
        )
        unlock_hold = strategy.make_release_plan(SEAT)[1]
        self.assertEqual(unlock_hold.max_linear_speed_m_s, 0.0)
        self.assertTrue(unlock_hold.requires_contact)


# ===========================================================================
# 正常流程
# ===========================================================================
class TestNormalFlows(unittest.TestCase):
    def test_v1_normal_cycle(self):
        manager, io = make_manager(PassiveHookV1())
        acquire = manager.begin_acquire("battery_red", RING)
        self.assertTrue(acquire.ok)
        self.assertEqual([p.name for p in acquire.primitives][:2], ["side_insert_pre_align", "side_insert"])
        manager.notify_engage_complete()
        self.assertTrue(manager.verify_attach(codes.EVIDENCE_GEOMETRY, "lift_follow").verified)
        release = manager.begin_release(SEAT)
        self.assertTrue(release.ok)
        manager.mark_seated(True, codes.EVIDENCE_GEOMETRY, "table_contact")
        manager.mark_unloaded(True, codes.EVIDENCE_GEOMETRY, "load_cell")
        decision = manager.resolve_disengage(
            codes.DISENGAGE_CONFIRMED, codes.EVIDENCE_SENSOR_OR_VISION, "hook_follow"
        )
        self.assertTrue(decision.allowed)
        status = manager.status()
        self.assertEqual(status.state, codes.STATE_DETACHED)
        self.assertTrue(status.verified)
        self.assertEqual(status.error_code, codes.ERROR_OK)
        self.assertEqual(io.call_count, 0)

    def test_v2_normal_cycle(self):
        manager, io = v2_ready_for_unlock()
        outcome = manager.trigger_unlock("req-normal", 300)
        self.assertTrue(outcome.accepted)
        self.assertFalse(outcome.unlock_confirmed, "受理不等于解锁")
        self.assertTrue(manager.confirm_unlock(codes.EVIDENCE_GEOMETRY, "latch_switch").unlock_confirmed)
        decision = manager.resolve_disengage(
            codes.DISENGAGE_CONFIRMED, codes.EVIDENCE_GEOMETRY, "latch_switch"
        )
        self.assertTrue(decision.allowed)
        status = manager.status()
        self.assertEqual(status.state, codes.STATE_DETACHED)
        self.assertTrue(status.verified)
        self.assertTrue(status.unlock_confirmed)
        self.assertTrue(status.withdraw_allowed)
        self.assertEqual(status.error_code, codes.ERROR_OK)
        self.assertEqual(io.call_count, 1)
        self.assertEqual(io.history, (300,))

    def test_state_machine_sequence_is_monotonic(self):
        manager, io = v2_ready_for_unlock()
        self.assertEqual(manager.status().state, codes.STATE_RELEASING)
        manager.trigger_unlock("req-seq", 300)
        manager.confirm_unlock(codes.EVIDENCE_GEOMETRY, "latch_switch")
        manager.resolve_disengage(codes.DISENGAGE_CONFIRMED, codes.EVIDENCE_GEOMETRY)
        self.assertEqual(manager.status().state, codes.STATE_DETACHED)
        self.assertEqual(manager.stage, "DISENGAGE")


# ===========================================================================
# 端口安全与失败模式
# ===========================================================================
class TestUnlockPortSafety(unittest.TestCase):
    def test_mock_reject_and_timeout_are_not_acceptance(self):
        for mode in (MockUnlockIo.MODE_REJECT, MockUnlockIo.MODE_TIMEOUT):
            with self.subTest(mode=mode):
                manager, io = v2_ready_for_unlock(io_mode=mode)
                outcome = manager.trigger_unlock("req-%s" % mode, 300)
                self.assertFalse(outcome.accepted)
                self.assertEqual(outcome.error_code, codes.ERROR_UNLOCK_FAILED)
                self.assertEqual(outcome.error_code, 410)
                self.assertEqual(manager.status().state, codes.STATE_FAULT)
                self.assertEqual(io.call_count, 1)

    def test_mock_raise_is_mapped_to_unlock_failed(self):
        manager, io = v2_ready_for_unlock(io_mode=MockUnlockIo.MODE_RAISE)
        outcome = manager.trigger_unlock("req-raise", 300)
        self.assertFalse(outcome.accepted)
        self.assertEqual(outcome.error_code, codes.ERROR_UNLOCK_FAILED)
        self.assertEqual(manager.status().state, codes.STATE_FAULT)

    def test_disabled_io_strict_never_touches_hardware(self):
        io = DisabledUnlockIo(strict=True)
        manager = ToolManager(strategy=MagneticLatchV2(), unlock_io=io)
        manager.begin_acquire("battery_blue", RING)
        manager.notify_engage_complete()
        manager.verify_attach(codes.EVIDENCE_GEOMETRY, "lift_follow")
        manager.begin_release(SEAT)
        manager.mark_seated(True, codes.EVIDENCE_GEOMETRY)
        manager.mark_unloaded(True, codes.EVIDENCE_GEOMETRY)
        outcome = manager.trigger_unlock("req-disabled", 300)
        self.assertFalse(outcome.accepted)
        self.assertEqual(outcome.error_code, codes.ERROR_UNLOCK_FAILED)
        self.assertEqual(outcome.io_status, "DISABLED")
        self.assertEqual(manager.status().state, codes.STATE_FAULT)
        with self.assertRaises(UnlockIoDisabledError):
            DisabledUnlockIo(strict=True).send_unlock_pulse(100)

    def test_disabled_io_lenient_returns_explicit_refusal(self):
        io = DisabledUnlockIo(strict=False)
        self.assertFalse(io.send_unlock_pulse(100))
        self.assertEqual(io.last_status, io.STATUS_DISABLED)
        self.assertEqual(io.call_count, 1)

    def test_default_manager_uses_disabled_io(self):
        manager = ToolManager(strategy=MagneticLatchV2())
        self.assertIsInstance(manager.unlock_io, DisabledUnlockIo)
        self.assertTrue(manager.unlock_io.strict)


# ===========================================================================
# 与 ROSIDL 定义的一致性（只读解析 .msg / .srv）
# ===========================================================================
class TestIdlFidelity(unittest.TestCase):
    def test_tool_state_constants_match(self):
        path = MSG_DIR / "ToolState.msg"
        if not path.exists():
            self.skipTest("ToolState.msg 不存在")
        constants = parse_msg_constants(path)
        expected = {name: value for name, value in constants.items() if name.startswith("STATE_")}
        self.assertTrue(expected)
        for name, value in expected.items():
            self.assertEqual(getattr(codes, name), value, "state 常量不一致：%s" % name)
        for name, value in constants.items():
            if name.startswith("EVIDENCE_"):
                self.assertEqual(getattr(codes, name), value, "evidence 常量不一致：%s" % name)
        self.assertEqual(
            len(expected), len([n for n in dir(codes) if n.startswith("STATE_") and not n.endswith("_NAMES")])
        )

    def test_error_codes_match(self):
        path = MSG_DIR / "ErrorCodes.msg"
        if not path.exists():
            self.skipTest("ErrorCodes.msg 不存在")
        constants = parse_msg_constants(path)
        self.assertTrue(constants)
        for name, value in constants.items():
            self.assertEqual(
                getattr(codes, "ERROR_" + name), value, "错误码不一致：%s" % name
            )

    def test_tool_state_fields_are_covered(self):
        path = MSG_DIR / "ToolState.msg"
        if not path.exists():
            self.skipTest("ToolState.msg 不存在")
        fields = set(parse_idl_fields(path))
        snapshot = ToolManager(strategy=PassiveHookV1()).status().to_tool_state_fields()
        self.assertTrue(set(snapshot).issubset(fields))
        manager, io = make_manager(PassiveHookV1())
        manager.begin_acquire("battery_blue", RING)
        manager.notify_engage_complete()
        snapshot = manager.status().to_tool_state_fields()
        self.assertEqual(snapshot["state"], codes.STATE_ATTACHED)
        self.assertFalse(snapshot["verified"])
        self.assertEqual(snapshot["evidence_level"], codes.EVIDENCE_COMMAND_ONLY)
        self.assertEqual(snapshot["tool_type"], codes.TOOL_TYPE_PASSIVE_HOOK_V1)
        self.assertEqual(snapshot["object_id"], "battery_blue")

    def test_package_metadata_exists(self):
        for name in ("package.xml", "setup.py", "setup.cfg", "README.md", "resource/mtc_tool"):
            self.assertTrue((_PKG_DIR / name).exists(), "缺少交付文件：%s" % name)

    def test_node_shell_fields_and_safety_defaults(self):
        """静态检查 rclpy 外壳：字段名与 IDL 一致、默认禁用真实 IO。

        本机 rclpy 不可导入（Humble 仅有 python3.10 扩展），因此这里只做源码级
        静态核对，不执行节点。
        """

        path = _PKG_DIR / "mtc_tool" / "tool_manager_node.py"
        text = path.read_text(encoding="utf-8")

        tool_state_fields = set(parse_idl_fields(MSG_DIR / "ToolState.msg")) | {"header"}
        for assigned in re.findall(r"\bmsg\.(\w+)\s*=", text):
            self.assertIn(assigned, tool_state_fields, "ToolState 字段名不一致：%s" % assigned)

        srv = SRV_DIR / "TriggerUnlock.srv"
        if srv.exists():
            request_fields = set(parse_idl_fields(srv, section=0))
            response_fields = set(parse_idl_fields(srv, section=1))
            for name in re.findall(r"\brequest\.(\w+)", text):
                self.assertIn(name, request_fields, "TriggerUnlock 请求字段不一致：%s" % name)
            for name in re.findall(r"\bresponse\.(\w+)\s*=", text):
                self.assertIn(name, response_fields, "TriggerUnlock 响应字段不一致：%s" % name)

        self.assertIn("DisabledUnlockIo", text, "节点必须默认使用 DisabledUnlockIo")
        self.assertIn("真实电磁解锁 IO 已禁用", text, "启动日志必须明确告警真实 IO 已禁用")
        self.assertIn("seated_verified", text)
        self.assertIn("unloaded", text)
        # 只允许两种端口实现，且不得出现其它 IO 后端名
        for forbidden in ("GpioUnlockIo", "SerialUnlockIo", "CanUnlockIo", "real_unlock_io"):
            self.assertNotIn(forbidden, text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
