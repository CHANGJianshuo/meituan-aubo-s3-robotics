"""mtc_safety 联锁单元测试 —— **纯 Python，不 import rclpy**。

运行方式（在包根目录，即 ``/home/robot/meituan_challenge_ws/src/mtc_safety``）::

    python3 -m pytest test/ -v
    python3 -m unittest discover -s test -v   # 备用（无 pytest 时）

覆盖设计文档 §7.2 关键联锁表与 §6.4 不确定状态处理，重点验证
「运动状态未知时绝不自动重发」与「FAULT 只能人工解除」。
"""

import os
import sys
import types
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from mtc_safety.interlocks import (  # noqa: E402
    DeviceState,
    FaultLatch,
    InterlockGuard,
    PayloadState,
    PayloadTracker,
    WatchdogConfig,
    WatchdogCore,
    adapt_stop_client,
    device_state_from_robot_state,
    payload_from_tool_state,
    ERR_ATTACH_NOT_VERIFIED,
    ERR_CANCEL_NOT_CONFIRMED,
    ERR_EXECUTION_REJECTED,
    ERR_HARDWARE_FAULT,
    ERR_MOTION_STATUS_UNKNOWN,
    ERR_OK,
    ERR_RELEASE_NOT_VERIFIED,
    ERR_SAFETY_INTERLOCK,
    ERR_SEAT_NOT_VERIFIED,
    TOOL_EVIDENCE_COMMAND_ONLY,
    TOOL_EVIDENCE_SENSOR_OR_VISION,
    TOOL_STATE_ATTACHED,
    TOOL_STATE_DETACHED,
    TOOL_STATE_FAULT,
)

NOW = 100.0


def make_guard(**overrides):
    """构造一个默认「一切就绪」的守卫，再按需覆盖以触发单条联锁。"""
    kwargs = dict(
        device_state=DeviceState.READY,
        controller_mode='AUTO',
        expected_modes=('AUTO',),
        simulation_mode=False,
        expect_simulation=False,
        state_ttl_sec=1.0,
        clock=lambda: NOW,
        link_ok=True,
        attach_verified=True,
        seat_verified=True,
        release_verified=True,
        motion_state_unknown=False,
        may_have_executed_motion=False,
        stop_confirmed=True,
        last_state_ts=NOW,
    )
    payload_state = overrides.pop('payload_state', PayloadState.NONE)
    payload_tracker = overrides.pop('payload_tracker', None)
    kwargs.update(overrides)
    tracker = payload_tracker or PayloadTracker(payload_state, clock=lambda: NOW)
    return InterlockGuard(payload=tracker, **kwargs)


class TestAllowMotion(unittest.TestCase):
    """§7.2 第一行：未 READY / 模式不符 / 状态过期 → 禁止任何实机运动。"""

    def test_not_ready_rejected(self):
        result = make_guard(device_state=DeviceState.NOT_READY).allow_motion()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)
        self.assertEqual(ERR_SAFETY_INTERLOCK, 500)

    def test_disconnected_rejected(self):
        result = make_guard(device_state=DeviceState.DISCONNECTED,
                            link_ok=False).allow_motion()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)

    def test_stop_requested_and_stopped_rejected(self):
        for state in (DeviceState.STOP_REQUESTED, DeviceState.STOPPED):
            with self.subTest(state=state):
                result = make_guard(device_state=state).allow_motion()
                self.assertFalse(result.allowed)
                self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)

    def test_controller_mode_mismatch_rejected(self):
        result = make_guard(controller_mode='MANUAL', expect_simulation=False).allow_motion()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)
        self.assertIn('模式不符', result.reason)

    def test_simulation_mode_mismatch_rejected(self):
        result = make_guard(simulation_mode=True, expect_simulation=False).allow_motion()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)

    def test_state_expired_rejected(self):
        result = make_guard(last_state_ts=NOW - 5.0,
                            state_ttl_sec=0.5).allow_motion()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)
        self.assertIn('过期', result.reason)

    def test_state_never_received_rejected(self):
        result = make_guard(last_state_ts=None).allow_motion()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)

    def test_motion_status_unknown_rejected(self):
        result = make_guard(motion_state_unknown=True).allow_motion()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_MOTION_STATUS_UNKNOWN)
        self.assertEqual(ERR_MOTION_STATUS_UNKNOWN, 230)

    def test_fault_device_rejected(self):
        result = make_guard(device_state=DeviceState.FAULT).allow_motion()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)

    def test_ready_allows(self):
        result = make_guard().allow_motion()
        self.assertTrue(result.allowed)
        self.assertEqual(result.error_code, ERR_OK)

    def test_expired_state_invalidates_ready(self):
        """READY 不代表安全：状态过期后 READY 也必须被拒。"""
        guard = make_guard()
        self.assertTrue(guard.allow_motion().allowed)
        guard.mark_state(now=NOW)          # 刷新
        self.assertTrue(guard.allow_motion().allowed)
        guard.state_ttl_sec = 0.0          # 任何时刻都视为过期
        guard.mark_state(now=NOW - 1.0)
        self.assertFalse(guard.allow_motion().allowed)


class TestTransport(unittest.TestCase):
    """§7.2 第二行：未验证挂载 → 禁止进入正常运输阶段。"""

    def test_attach_not_verified_rejected(self):
        result = make_guard(attach_verified=False,
                            payload_state=PayloadState.UNKNOWN).allow_transport()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_ATTACH_NOT_VERIFIED)
        self.assertEqual(ERR_ATTACH_NOT_VERIFIED, 310)

    def test_verified_but_payload_unknown_rejected(self):
        result = make_guard(attach_verified=True,
                            payload_state=PayloadState.UNKNOWN).allow_transport()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_ATTACH_NOT_VERIFIED)

    def test_verified_carrying_allows(self):
        result = make_guard(attach_verified=True,
                            payload_state=PayloadState.CARRYING).allow_transport()
        self.assertTrue(result.allowed)


class TestUnlock(unittest.TestCase):
    """§7.2 第三行：电池尚未安全落座 → 禁止 V2 解锁脉冲。

    本包统一约定：未落座 → 400 SEAT_NOT_VERIFIED；载荷未知等 → 500 SAFETY_INTERLOCK。
    """

    def test_payload_unknown_rejected_with_500(self):
        result = make_guard(payload_state=PayloadState.UNKNOWN,
                            seat_verified=True).allow_unlock()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)

    def test_seat_not_verified_rejected_with_400(self):
        result = make_guard(payload_state=PayloadState.CARRYING,
                            seat_verified=False).allow_unlock()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_SEAT_NOT_VERIFIED)
        self.assertEqual(ERR_SEAT_NOT_VERIFIED, 400)

    def test_unlock_without_payload_rejected(self):
        result = make_guard(payload_state=PayloadState.NONE,
                            seat_verified=True).allow_unlock()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)

    def test_unlock_while_moving_rejected(self):
        result = make_guard(payload_state=PayloadState.CARRYING,
                            seat_verified=True,
                            motion_in_flight='cmd-7').allow_unlock()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)

    def test_unlock_with_unknown_motion_rejected(self):
        result = make_guard(payload_state=PayloadState.CARRYING,
                            seat_verified=True,
                            motion_state_unknown=True).allow_unlock()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)

    def test_seated_carrying_allows(self):
        result = make_guard(payload_state=PayloadState.CARRYING,
                            seat_verified=True).allow_unlock()
        self.assertTrue(result.allowed)


class TestForceWithdraw(unittest.TestCase):
    """§7.2 第四行：解锁/脱离状态不确定 → 禁止水平硬拉、直接上抬、下一任务项。"""

    def test_release_not_verified_rejected(self):
        result = make_guard(release_verified=False).allow_force_withdraw()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_RELEASE_NOT_VERIFIED)
        self.assertEqual(ERR_RELEASE_NOT_VERIFIED, 420)

    def test_motion_unknown_rejected(self):
        result = make_guard(release_verified=True,
                            motion_state_unknown=True).allow_force_withdraw()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_MOTION_STATUS_UNKNOWN)

    def test_release_verified_allows(self):
        result = make_guard(release_verified=True).allow_force_withdraw()
        self.assertTrue(result.allowed)


class TestSecondMotionGoal(unittest.TestCase):
    """§7.2 第五行：正在执行运动 → 拒绝第二个运动 Goal / 抢占。"""

    def test_in_flight_goal_rejected(self):
        result = make_guard(motion_in_flight='cmd-1').allow_second_motion_goal()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_EXECUTION_REJECTED)
        self.assertEqual(ERR_EXECUTION_REJECTED, 210)

    def test_device_moving_rejected(self):
        result = make_guard(device_state=DeviceState.MOVING).allow_second_motion_goal()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_EXECUTION_REJECTED)

    def test_unknown_motion_state_rejected(self):
        result = make_guard(motion_state_unknown=True).allow_second_motion_goal()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_MOTION_STATUS_UNKNOWN)

    def test_idle_allows(self):
        result = make_guard(device_state=DeviceState.READY).allow_second_motion_goal()
        self.assertTrue(result.allowed)


class TestNextTaskItem(unittest.TestCase):
    """§7.2 最后一行：取消/断线后未确认停稳 → 禁止返回成功、下一目标、自动 Home。"""

    def test_cancel_not_confirmed_rejected(self):
        result = make_guard(stop_confirmed=False,
                            payload_state=PayloadState.NONE).allow_next_task_item()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_CANCEL_NOT_CONFIRMED)
        self.assertEqual(ERR_CANCEL_NOT_CONFIRMED, 520)

    def test_unknown_motion_state_rejected(self):
        result = make_guard(motion_state_unknown=True).allow_next_task_item()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_MOTION_STATUS_UNKNOWN)

    def test_unknown_payload_rejected(self):
        result = make_guard(payload_state=PayloadState.UNKNOWN,
                            stop_confirmed=True).allow_next_task_item()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)

    def test_still_carrying_rejected(self):
        result = make_guard(payload_state=PayloadState.CARRYING,
                            stop_confirmed=True).allow_next_task_item()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)

    def test_stopped_and_empty_allows(self):
        result = make_guard(payload_state=PayloadState.NONE,
                            stop_confirmed=True).allow_next_task_item()
        self.assertTrue(result.allowed)

    def test_unconditional_home_blocked_after_cancel(self):
        guard = make_guard(payload_state=PayloadState.NONE, stop_confirmed=False)
        result = guard.allow_unconditional_home()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_CANCEL_NOT_CONFIRMED)


class TestResendMotion(unittest.TestCase):
    """最关键：运动状态未知 / 可能已执行运动 → 绝不自动重发（230）。"""

    def test_no_evidence_rejected_by_default(self):
        result = make_guard().allow_resend_motion()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_EXECUTION_REJECTED)

    def test_motion_status_unknown_rejected(self):
        result = make_guard(motion_state_unknown=True,
                            stop_confirmed=True).allow_resend_motion()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_MOTION_STATUS_UNKNOWN)

    def test_may_have_executed_rejected(self):
        result = make_guard(may_have_executed_motion=True,
                            stop_confirmed=True).allow_resend_motion()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_MOTION_STATUS_UNKNOWN)

    def test_unknown_takes_precedence_over_other_evidence(self):
        """即使手工伪造其它「安全」标志，只要状态未知就必须 230 拒绝。"""
        guard = make_guard()
        guard.note_motion_state_unknown('sdk_link_lost')
        guard.retry_evidence = True
        guard.may_have_executed_motion = False
        guard.stop_confirmed = True
        result = guard.allow_resend_motion()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_MOTION_STATUS_UNKNOWN)

    def test_stop_not_confirmed_rejected(self):
        guard = make_guard()
        guard.note_command_rejected_before_motion('backend REJECTED before SEND')
        guard.note_stop_confirmed(False, 'cancel pending')
        result = guard.allow_resend_motion()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_CANCEL_NOT_CONFIRMED)

    def test_explicit_reject_before_motion_allows(self):
        guard = make_guard()
        guard.note_motion_started('cmd-9')
        self.assertFalse(guard.allow_resend_motion().allowed)
        guard.note_command_rejected_before_motion('后端在 SEND 前拒绝，从未开始运动')
        result = guard.allow_resend_motion()
        self.assertTrue(result.allowed, msg=result.reason)
        self.assertEqual(result.error_code, ERR_OK)

    def test_exception_path_locks_resend(self):
        guard = make_guard()
        guard.note_motion_state_unknown('backend_exception')
        result = guard.allow_resend_motion()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_MOTION_STATUS_UNKNOWN)
        # 人工核验解除未知后，仍不允许重发（缺少「从未执行」证据）。
        ok, _ = guard.clear_unknown_after_manual_confirmation('现场目视确认机械臂静止')
        self.assertTrue(ok)
        self.assertFalse(guard.allow_resend_motion().allowed)


class TestFaultLatch(unittest.TestCase):
    """FAULT 锁存：自动路径不得清除，人工 clear_by_human 才能解除。"""

    def test_latch_and_block(self):
        guard = make_guard()
        guard.fault_latch.latch(ERR_HARDWARE_FAULT, '控制柜报错')
        self.assertTrue(guard.fault_latched)
        result = guard.allow_motion()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)

    def test_first_cause_retained(self):
        latch = FaultLatch(clock=lambda: NOW)
        latch.latch(ERR_HARDWARE_FAULT, '首因')
        latch.latch(ERR_MOTION_STATUS_UNKNOWN, '次因')
        self.assertEqual(latch.record.reason, '首因')
        self.assertEqual(len(latch.history), 2)

    def test_auto_path_cannot_clear(self):
        latch = FaultLatch(clock=lambda: NOW)
        latch.latch(ERR_SAFETY_INTERLOCK, 'state unknown too long')
        ok, message = latch.try_auto_clear('auto_recovery_timer')
        self.assertFalse(ok)
        self.assertTrue(latch.latched)
        self.assertEqual(len(latch.clear_attempts), 1)
        self.assertFalse(latch.clear_attempts[0]['accepted'])

    def test_human_clear_requires_note(self):
        latch = FaultLatch(clock=lambda: NOW)
        latch.latch(ERR_HARDWARE_FAULT, '关节过流')
        ok, _ = latch.clear_by_human('   ')
        self.assertFalse(ok)
        self.assertTrue(latch.latched)

    def test_human_clear_records_note_and_timestamp(self):
        latch = FaultLatch(clock=lambda: NOW + 7.0)
        latch.latch(ERR_HARDWARE_FAULT, '关节过流')
        ok, message = latch.clear_by_human('已断电检查，无机械干涉，确认可继续')
        self.assertTrue(ok)
        self.assertFalse(latch.latched)
        self.assertEqual(len(latch.clear_history), 1)
        entry = latch.clear_history[0]
        self.assertEqual(entry['note'], '已断电检查，无机械干涉，确认可继续')
        self.assertEqual(entry['timestamp'], NOW + 7.0)
        self.assertEqual(entry['cleared_fault']['error_code'], ERR_HARDWARE_FAULT)

    def test_guard_allows_after_human_clear(self):
        guard = make_guard()
        guard.fault_latch.latch(ERR_CANCEL_NOT_CONFIRMED, '停止未确认', now=NOW)
        self.assertFalse(guard.allow_motion().allowed)
        ok, _ = guard.fault_latch.clear_by_human('人工核验：机械臂已静止')
        self.assertTrue(ok)
        self.assertTrue(guard.allow_motion().allowed)

    def test_all_guards_blocked_while_latched(self):
        guard = make_guard(payload_state=PayloadState.CARRYING)
        guard.fault_latch.latch(ERR_SAFETY_INTERLOCK, '测试锁存')
        for name in ('allow_motion', 'allow_transport', 'allow_unlock',
                     'allow_force_withdraw', 'allow_second_motion_goal',
                     'allow_next_task_item', 'allow_unconditional_home'):
            with self.subTest(guard_method=name):
                result = getattr(guard, name)()
                self.assertFalse(result.allowed)
                self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)

    def test_resend_blocked_while_latched(self):
        guard = make_guard()
        guard.retry_evidence = True
        guard.fault_latch.latch(ERR_HARDWARE_FAULT, '工具故障')
        result = guard.allow_resend_motion()
        self.assertFalse(result.allowed)
        self.assertEqual(result.error_code, ERR_SAFETY_INTERLOCK)


class TestPayloadStateMachine(unittest.TestCase):
    """载荷状态机：unknown / none / carrying，未知时禁止解锁、无条件 Home、下一项。"""

    def test_unknown_is_most_conservative(self):
        tracker = PayloadTracker(PayloadState.UNKNOWN, clock=lambda: NOW)
        self.assertFalse(tracker.automatic_unlock_allowed())
        self.assertFalse(tracker.unconditional_home_allowed())
        self.assertFalse(tracker.next_task_item_allowed())

    def test_confirm_requires_evidence(self):
        tracker = PayloadTracker(clock=lambda: NOW)
        ok, _ = tracker.mark_carrying('')
        self.assertFalse(ok)
        self.assertEqual(tracker.state, PayloadState.UNKNOWN)
        ok, _ = tracker.mark_none('   ')
        self.assertFalse(ok)
        self.assertEqual(tracker.state, PayloadState.UNKNOWN)

    def test_transitions(self):
        tracker = PayloadTracker(clock=lambda: NOW)
        self.assertTrue(tracker.mark_carrying('试提后传感确认')[0])
        self.assertEqual(tracker.state, PayloadState.CARRYING)
        self.assertTrue(tracker.automatic_unlock_allowed())
        self.assertFalse(tracker.unconditional_home_allowed())
        self.assertTrue(tracker.next_task_item_allowed())

        self.assertTrue(tracker.mark_none('视觉确认电池已落座且工具已脱离')[0])
        self.assertEqual(tracker.state, PayloadState.NONE)
        self.assertFalse(tracker.automatic_unlock_allowed())
        self.assertTrue(tracker.unconditional_home_allowed())

        tracker.mark_unknown('通信中断，无法确认')
        self.assertEqual(tracker.state, PayloadState.UNKNOWN)
        self.assertFalse(tracker.automatic_unlock_allowed())
        self.assertFalse(tracker.unconditional_home_allowed())
        self.assertFalse(tracker.next_task_item_allowed())

    def test_unknown_payload_blocks_unlock_and_home_on_guard(self):
        guard = make_guard(payload_state=PayloadState.UNKNOWN)
        self.assertFalse(guard.allow_unlock().allowed)
        self.assertEqual(guard.allow_unlock().error_code, ERR_SAFETY_INTERLOCK)
        home = guard.allow_unconditional_home()
        self.assertFalse(home.allowed)
        self.assertEqual(home.error_code, ERR_SAFETY_INTERLOCK)


class TestPureMappings(unittest.TestCase):
    """由 RobotState / ToolState 推导设备与载荷状态的纯函数。"""

    def test_device_state_precedence(self):
        self.assertEqual(
            device_state_from_robot_state(False, True, True, True, True),
            DeviceState.DISCONNECTED)
        self.assertEqual(
            device_state_from_robot_state(True, True, True, False, True),
            DeviceState.FAULT)
        self.assertEqual(
            device_state_from_robot_state(True, True, True, True, False,
                                          stop_requested=True),
            DeviceState.STOP_REQUESTED)
        self.assertEqual(
            device_state_from_robot_state(True, True, True, True, False),
            DeviceState.MOVING)
        self.assertEqual(
            device_state_from_robot_state(True, True, False, True, True,
                                          stop_requested=True),
            DeviceState.STOPPED)
        self.assertEqual(
            device_state_from_robot_state(True, True, False, True, True),
            DeviceState.READY)
        self.assertEqual(
            device_state_from_robot_state(True, False, False, True, True),
            DeviceState.NOT_READY)

    def test_payload_from_tool_state(self):
        self.assertEqual(
            payload_from_tool_state(TOOL_STATE_ATTACHED, True,
                                    TOOL_EVIDENCE_SENSOR_OR_VISION),
            PayloadState.CARRYING)
        self.assertEqual(
            payload_from_tool_state(TOOL_STATE_DETACHED, True,
                                    TOOL_EVIDENCE_SENSOR_OR_VISION),
            PayloadState.NONE)
        # state 是估计，verified 是独立维度：未验证一律 unknown
        self.assertEqual(
            payload_from_tool_state(TOOL_STATE_ATTACHED, False,
                                    TOOL_EVIDENCE_SENSOR_OR_VISION),
            PayloadState.UNKNOWN)
        # 仅有命令回执（COMMAND_ONLY）不算证据
        self.assertEqual(
            payload_from_tool_state(TOOL_STATE_ATTACHED, True,
                                    TOOL_EVIDENCE_COMMAND_ONLY),
            PayloadState.UNKNOWN)
        self.assertEqual(
            payload_from_tool_state(TOOL_STATE_FAULT, True,
                                    TOOL_EVIDENCE_SENSOR_OR_VISION),
            PayloadState.UNKNOWN)


def _robot_state(**overrides):
    fields = dict(communication_ok=True, robot_ready=True, motion_active=False,
                  safety_normal=True, stop_confirmed=False,
                  simulation_mode=False, controller_mode='AUTO', detail='')
    fields.update(overrides)
    return types.SimpleNamespace(**fields)


def _tool_state(**overrides):
    fields = dict(state=TOOL_STATE_DETACHED, verified=True,
                  evidence_level=TOOL_EVIDENCE_SENSOR_OR_VISION,
                  detail='')
    fields.update(overrides)
    return types.SimpleNamespace(**fields)


class TestWatchdogCore(unittest.TestCase):
    """看门狗：超时/失联/未知 → 停止请求；送达不等于停稳；超时升级为故障锁定。"""

    def _make(self, now_box, guard=None, requester=None):
        recorded = []

        def _requester(reason):
            recorded.append(reason)
            return True, f'delivered:{reason}'

        watchdog = WatchdogCore(
            config=WatchdogConfig(robot_state_timeout_sec=0.5,
                                  tool_state_timeout_sec=1.0,
                                  stop_confirm_timeout_sec=2.0),
            stop_requester=requester or _requester,
            clock=lambda: now_box[0],
            guard=guard,
        )
        return watchdog, recorded

    def test_missing_states_trigger_stop_requests(self):
        now_box = [0.0]
        watchdog, recorded = self._make(now_box)
        events = watchdog.tick()
        self.assertEqual(recorded, ['robot_state_never_received',
                                    'tool_state_never_received'])
        self.assertEqual(len(events), 2)
        self.assertTrue(watchdog.stop_requested)
        # 送达 true 也绝不等于停稳
        self.assertFalse(watchdog.stop_confirmed)
        self.assertTrue(watchdog.last_delivery)
        for event in events:
            self.assertTrue(event['stop_requested'])
            self.assertFalse(event['stop_confirmed'])
            self.assertEqual(event['timeouts'],
                             {'robot_state_timeout_sec': 0.5,
                              'tool_state_timeout_sec': 1.0,
                              'stop_confirm_timeout_sec': 2.0})

    def test_repeated_reason_is_idempotent(self):
        now_box = [0.0]
        watchdog, recorded = self._make(now_box)
        watchdog.tick()
        before = len(recorded)
        now_box[0] = 0.1
        self.assertEqual(watchdog.tick(), [])
        self.assertEqual(len(recorded), before)

    def test_stop_confirmed_only_from_evidence(self):
        now_box = [0.0]
        watchdog, _ = self._make(now_box)
        watchdog.tick()
        self.assertFalse(watchdog.stop_confirmed)
        now_box[0] = 0.1
        watchdog.on_robot_state(_robot_state(stop_confirmed=True))
        watchdog.on_tool_state(_tool_state())
        self.assertTrue(watchdog.stop_confirmed)
        self.assertEqual(watchdog.tick(), [])

    def test_stale_state_resets_stop_confirmed(self):
        now_box = [0.0]
        watchdog, recorded = self._make(now_box)
        watchdog.on_robot_state(_robot_state(stop_confirmed=True))
        watchdog.on_tool_state(_tool_state())
        self.assertTrue(watchdog.stop_confirmed)
        now_box[0] = 5.0  # 超时
        events = watchdog.tick()
        self.assertIn('robot_state_timeout', recorded)
        self.assertIn('tool_state_timeout', recorded)
        self.assertFalse(watchdog.stop_confirmed)
        self.assertTrue(any(e['reason'] == 'robot_state_timeout' for e in events))

    def test_communication_lost_and_safety_false(self):
        now_box = [0.0]
        watchdog, recorded = self._make(now_box)
        watchdog.on_robot_state(_robot_state(communication_ok=False,
                                             safety_normal=False))
        watchdog.on_tool_state(_tool_state())
        watchdog.tick()
        self.assertIn('robot_communication_lost', recorded)
        self.assertIn('safety_normal_false', recorded)

    def test_stop_confirm_timeout_escalates_to_fault_latch(self):
        now_box = [0.0]
        guard = make_guard(payload_state=PayloadState.CARRYING)
        watchdog, _ = self._make(now_box, guard=guard)
        watchdog.on_robot_state(_robot_state(stop_confirmed=False))
        watchdog.on_tool_state(_tool_state(state=TOOL_STATE_ATTACHED))
        watchdog.request_stop('manual_test')
        self.assertFalse(guard.fault_latched)
        now_box[0] = 3.0
        events = watchdog.tick()
        escalated = [e for e in events if e['event'] == 'stop_confirm_timeout']
        self.assertEqual(len(escalated), 1)
        self.assertEqual(escalated[0]['error_code'], ERR_CANCEL_NOT_CONFIRMED)
        self.assertTrue(watchdog.stop_confirm_timed_out)
        self.assertTrue(guard.fault_latched)
        self.assertEqual(guard.fault_latch.record.error_code,
                         ERR_CANCEL_NOT_CONFIRMED)
        # 锁存后所有守卫拒绝
        self.assertFalse(guard.allow_motion().allowed)

    def test_tool_fault_latches_guard(self):
        guard = make_guard()
        now_box = [0.0]
        watchdog, _ = self._make(now_box, guard=guard)
        watchdog.on_tool_state(_tool_state(state=TOOL_STATE_FAULT))
        self.assertTrue(guard.fault_latched)
        self.assertEqual(guard.fault_latch.record.error_code, ERR_HARDWARE_FAULT)

    def test_motion_unknown_reason_and_guard_sync(self):
        now_box = [0.0]
        guard = make_guard()
        watchdog, recorded = self._make(now_box, guard=guard)
        watchdog.on_robot_state(_robot_state())
        watchdog.on_tool_state(_tool_state())
        watchdog.note_motion_state_unknown(True, 'sdk_timeout')
        watchdog.tick()
        self.assertIn('motion_status_unknown', recorded)
        self.assertTrue(guard.motion_state_unknown)
        self.assertFalse(guard.allow_resend_motion().allowed)
        self.assertEqual(guard.allow_resend_motion().error_code,
                         ERR_MOTION_STATUS_UNKNOWN)

    def test_log_fields_contain_required_keys(self):
        now_box = [0.0]
        watchdog, _ = self._make(now_box)
        watchdog.tick()
        fields = watchdog.log_fields()
        self.assertIn('stop_requested', fields)
        self.assertIn('stop_confirmed', fields)
        self.assertIn('timeouts', fields)
        self.assertEqual(sorted(fields['timeouts']),
                         ['robot_state_timeout_sec',
                          'stop_confirm_timeout_sec',
                          'tool_state_timeout_sec'])

    def test_note_stop_delivery_does_not_confirm_stop(self):
        now_box = [0.0]
        watchdog, _ = self._make(now_box)
        watchdog.request_stop('manual')
        watchdog.note_stop_delivery(True, 'service returned True')
        self.assertTrue(watchdog.last_delivery)
        self.assertFalse(watchdog.stop_confirmed)


class _AsyncFuture:
    def __init__(self):
        self.called = []

    def add_done_callback(self, callback):
        self.called.append(callback)


class _AsyncStopClient:
    """模拟 rclpy ServiceClient：call_async 立即返回，送达结果稍后才有。"""

    def __init__(self):
        self.requests = []

    def call_async(self, request):
        self.requests.append(request)
        return _AsyncFuture()


class _SyncStopClient:
    def __init__(self, delivered=True):
        self.delivered = delivered
        self.reasons = []

    def call(self, reason):
        self.reasons.append(reason)
        return types.SimpleNamespace(request_delivered=self.delivered,
                                     message='ok')


class TestStopClientAdapter(unittest.TestCase):
    """fake client 注入适配：真实 rclpy ServiceClient 与离线 fake 都支持。"""

    def test_plain_callable(self):
        requester = adapt_stop_client(lambda reason: (True, f'ok:{reason}'))
        self.assertEqual(requester('timeout'), (True, 'ok:timeout'))

    def test_callable_returning_object(self):
        requester = adapt_stop_client(
            lambda reason: types.SimpleNamespace(request_delivered=False,
                                                 message='nack'))
        self.assertEqual(requester('x'), (False, 'nack'))

    def test_object_with_sync_call(self):
        client = _SyncStopClient(delivered=True)
        requester = adapt_stop_client(client)
        self.assertEqual(requester('r1'), (True, 'ok'))
        self.assertEqual(client.reasons, ['r1'])

    def test_object_with_call_async_is_not_treated_as_confirmed(self):
        client = _AsyncStopClient()
        requester = adapt_stop_client(client, request_factory=lambda reason: f'req:{reason}')
        delivered, message = requester('timeout')
        self.assertIsNone(delivered)          # 送达未知
        self.assertEqual(client.requests, ['req:timeout'])
        self.assertIn('unknown', message)

    def test_none_client(self):
        self.assertIsNone(adapt_stop_client(None))

    def test_watchdog_with_fake_client_end_to_end(self):
        """注入 fake client 的看门狗端到端：请求送达 ≠ 停稳。"""
        client = _SyncStopClient(delivered=True)
        now_box = [0.0]
        guard = make_guard()
        watchdog = WatchdogCore(
            config=WatchdogConfig(robot_state_timeout_sec=0.5,
                                  tool_state_timeout_sec=1.0,
                                  stop_confirm_timeout_sec=2.0),
            stop_requester=adapt_stop_client(client),
            clock=lambda: now_box[0],
            guard=guard,
        )
        watchdog.tick()
        self.assertTrue(client.reasons)
        self.assertTrue(watchdog.stop_requested)
        self.assertFalse(watchdog.stop_confirmed)


if __name__ == '__main__':
    unittest.main(verbosity=2)
