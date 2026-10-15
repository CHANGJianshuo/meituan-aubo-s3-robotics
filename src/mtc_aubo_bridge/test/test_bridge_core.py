"""mtc_aubo_bridge 离线单元测试（纯 Python，不 import rclpy，不连接任何设备）。

运行：
    cd /home/robot/meituan_challenge_ws/src/mtc_aubo_bridge
    python3 test/test_bridge_core.py
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mtc_aubo_bridge import (  # noqa: E402
    BridgeCapabilities,
    DisabledSdkWorker,
    FakeSdkWorker,
    HealthStatus,
    RobotIdentity,
    make_bridge,
)

VERIFIED_ID = RobotIdentity(controller_model='AUBO-S3', serial_number='SN-0001',
                            arcs_version='x', sdk_version='y',
                            interface_version='z', verified=True)


def fake_bridge(**worker_kwargs):
    return make_bridge('fake', identity=VERIFIED_ID, **worker_kwargs)


def with_whitelist(bridge, serial='SN-0001'):
    bridge._expected_identity = RobotIdentity(  # noqa: SLF001 - 测试内显式配置白名单
        controller_model='AUBO-S3', serial_number=serial, verified=True)
    return bridge


class TestDefaultDisabled(unittest.TestCase):
    def test_default_mode_is_disabled(self):
        for mode in ('', 'disabled', 'real', 'aubo', 'REAL', 'sdk', 'gazebo'):
            bridge = make_bridge(mode)
            self.assertIsInstance(bridge.worker, DisabledSdkWorker,
                                  f'{mode!r} 不得启用真实通路')

    def test_disabled_worker_declares_no_capability(self):
        caps = DisabledSdkWorker().capabilities()
        self.assertFalse(caps.joint_goal_supported)
        self.assertFalse(caps.timed_trajectory_supported)
        self.assertFalse(caps.cartesian_motion_supported)
        self.assertFalse(caps.tool_io_supported)

    def test_disabled_bridge_refuses_motion(self):
        bridge = make_bridge('disabled')
        result = bridge.send_joint_move('c1', ['j'] * 6, [0.0] * 6, 0.1, 0.1, 1000)
        self.assertFalse(result.accepted)
        self.assertEqual(result.error_code, 210)

    def test_disabled_bridge_refuses_unlock(self):
        bridge = make_bridge('disabled')
        result = bridge.send_tool_unlock_pulse('r1', 100, seated_verified=True, unloaded=True)
        self.assertFalse(result.accepted)
        self.assertEqual(result.error_code, 500)
        self.assertIn('电磁铁', result.message)

    def test_disabled_worker_never_reports_healthy(self):
        health = DisabledSdkWorker().health()
        self.assertFalse(health.rpc_ok)
        self.assertFalse(health.rtde_ok)
        self.assertFalse(health.usable_for_motion)


class TestIdentityAndHealth(unittest.TestCase):
    def test_identity_requires_verified_whitelist(self):
        bridge = fake_bridge()
        ok, why = bridge.identity_check()
        self.assertFalse(ok, '未配置白名单时必须拒绝')
        self.assertIn('白名单', why)

    def test_identity_mismatch_rejected(self):
        bridge = with_whitelist(fake_bridge(), serial='OTHER-SN')
        ok, why = bridge.identity_check()
        self.assertFalse(ok)
        self.assertIn('不匹配', why)

    def test_identity_match_passes(self):
        bridge = with_whitelist(fake_bridge())
        ok, why = bridge.identity_check()
        self.assertTrue(ok, why)

    def test_unverified_actual_identity_rejected(self):
        unverified = RobotIdentity(controller_model='AUBO-S3', serial_number='SN-0001',
                                   verified=False)
        bridge = with_whitelist(make_bridge('fake', identity=unverified))
        ok, _ = bridge.identity_check()
        self.assertFalse(ok, '未核实(unverified)的身份不得通过')

    def test_stale_feedback_blocks_motion(self):
        stale = HealthStatus(rpc_ok=True, rtde_ok=True, last_feedback_age_s=5.0)
        bridge = with_whitelist(fake_bridge(health=stale))
        ok, why = bridge.health_check()
        self.assertFalse(ok)
        self.assertIn('反馈不新鲜', why)

    def test_rtde_down_blocks_motion(self):
        down = HealthStatus(rpc_ok=True, rtde_ok=False, last_feedback_age_s=0.01)
        bridge = with_whitelist(fake_bridge(health=down))
        ok, why = bridge.health_check()
        self.assertFalse(ok)
        self.assertIn('RTDE', why)

    def test_motion_blocked_when_feedback_stale(self):
        stale = HealthStatus(rpc_ok=True, rtde_ok=True, last_feedback_age_s=10.0)
        bridge = with_whitelist(fake_bridge(health=stale))
        result = bridge.send_joint_move('c1', ['j'] * 6, [0.0] * 6, 0.1, 0.1, 1000)
        self.assertFalse(result.accepted)
        self.assertIn('不新鲜', result.message)


class TestCommandSemantics(unittest.TestCase):
    def test_successful_move(self):
        bridge = with_whitelist(fake_bridge())
        result = bridge.send_joint_move('c1', ['j'] * 6, [0.1] * 6, 0.1, 0.1, 1000)
        self.assertTrue(result.accepted)
        self.assertEqual(result.error_code, 0)

    def test_duplicate_command_id_rejected(self):
        bridge = with_whitelist(fake_bridge())
        self.assertTrue(bridge.send_joint_move('dup', ['j'] * 6, [0.1] * 6, 0.1, 0.1, 1000).accepted)
        second = bridge.send_joint_move('dup', ['j'] * 6, [0.1] * 6, 0.1, 0.1, 1000)
        self.assertFalse(second.accepted)
        self.assertIn('重复', second.message)

    def test_empty_command_id_rejected(self):
        bridge = with_whitelist(fake_bridge())
        result = bridge.send_joint_move('', ['j'] * 6, [0.0] * 6, 0.1, 0.1, 1000)
        self.assertFalse(result.accepted)

    def test_unsupported_capability_rejected(self):
        caps = BridgeCapabilities(worker_name='fake', read_only_state_supported=True,
                                  joint_goal_supported=False)
        bridge = with_whitelist(make_bridge('fake', identity=VERIFIED_ID, capabilities=caps))
        result = bridge.send_joint_move('c1', ['j'] * 6, [0.0] * 6, 0.1, 0.1, 1000)
        self.assertFalse(result.accepted)
        self.assertIn('joint_goal_supported', result.message)

    def test_unknown_state_locks_further_commands(self):
        bridge = with_whitelist(fake_bridge(motion_state_unknown=True))
        first = bridge.send_joint_move('u1', ['j'] * 6, [0.0] * 6, 0.1, 0.1, 1000)
        self.assertTrue(first.motion_state_unknown)
        self.assertTrue(bridge.motion_state_unknown)
        second = bridge.send_joint_move('u2', ['j'] * 6, [0.0] * 6, 0.1, 0.1, 1000)
        self.assertFalse(second.accepted)
        self.assertIn('未确认', second.message)

    def test_stop_not_confirmed_keeps_lock(self):
        bridge = with_whitelist(fake_bridge(motion_state_unknown=True, stop_confirms=False))
        bridge.send_joint_move('u1', ['j'] * 6, [0.0] * 6, 0.1, 0.1, 1000)
        result = bridge.request_stop('cancel')
        self.assertFalse(result.stop_confirmed)
        self.assertTrue(bridge.motion_state_unknown,
                        '未确认停稳时不得解除不确定状态锁定')

    def test_stop_confirmed_alone_cannot_clear_lock(self):
        """关键安全语义：状态未知后，即使停止请求被确认，也不得自动解除锁定。"""
        bridge = with_whitelist(fake_bridge(motion_state_unknown=True, stop_confirms=True))
        bridge.send_joint_move('u1', ['j'] * 6, [0.0] * 6, 0.1, 0.1, 1000)
        result = bridge.request_stop('cancel')
        self.assertTrue(result.stop_confirmed, '停止请求本身被确认停稳')
        self.assertTrue(bridge.motion_state_unknown,
                        '未完成设备侧状态对账时，禁止自动解除不确定状态锁定')
        self.assertFalse(result.state_reconciled)

    def test_stop_clears_lock_only_when_reconciled(self):
        bridge = with_whitelist(fake_bridge(stop_confirms=True))
        result = bridge.request_stop('idle')
        self.assertTrue(result.stop_confirmed)
        self.assertTrue(result.state_reconciled,
                        '无状态未知且确认停稳时应视为已对账')

    def test_manual_confirmation_is_the_only_automatic_escape(self):
        bridge = with_whitelist(fake_bridge(motion_state_unknown=True))
        bridge.send_joint_move('u1', ['j'] * 6, [0.0] * 6, 0.1, 0.1, 1000)
        self.assertTrue(bridge.motion_state_unknown)
        bridge.clear_unknown_after_manual_confirmation('现场确认静止')
        self.assertFalse(bridge.motion_state_unknown)
        events = [event['event'] for event in bridge.audit_trail()]
        self.assertIn('manual_confirmation', events)


class TestUnlockSemantics(unittest.TestCase):
    def test_unlock_requires_seated_and_unloaded(self):
        io_capable = BridgeCapabilities(worker_name='fake', read_only_state_supported=True,
                                        joint_goal_supported=True, tool_io_supported=True)
        bridge = with_whitelist(make_bridge('fake', identity=VERIFIED_ID,
                                            capabilities=io_capable))
        not_seated = bridge.send_tool_unlock_pulse('r1', 100, seated_verified=False, unloaded=True)
        self.assertFalse(not_seated.accepted)
        self.assertIn('落座', not_seated.message)

        not_unloaded = bridge.send_tool_unlock_pulse('r2', 100, seated_verified=True, unloaded=False)
        self.assertFalse(not_unloaded.accepted)
        self.assertIn('载荷', not_unloaded.message)

        ok = bridge.send_tool_unlock_pulse('r3', 100, seated_verified=True, unloaded=True)
        self.assertTrue(ok.accepted)
        self.assertIn('不代表已解锁', ok.message)

    def test_unlock_rejected_without_tool_io_capability(self):
        bridge = with_whitelist(fake_bridge())  # tool_io_supported=False
        result = bridge.send_tool_unlock_pulse('r1', 100, seated_verified=True, unloaded=True)
        self.assertFalse(result.accepted)
        self.assertEqual(result.error_code, 500)

    def test_zero_pulse_rejected(self):
        io_capable = BridgeCapabilities(worker_name='fake', joint_goal_supported=True,
                                       tool_io_supported=True)
        bridge = with_whitelist(make_bridge('fake', identity=VERIFIED_ID,
                                            capabilities=io_capable))
        result = bridge.send_tool_unlock_pulse('r1', 0, seated_verified=True, unloaded=True)
        self.assertFalse(result.accepted)


class TestAuditTrail(unittest.TestCase):
    def test_audit_records_linkable_ids(self):
        bridge = with_whitelist(fake_bridge())
        bridge.send_joint_move('cmd-42', ['j'] * 6, [0.0] * 6, 0.1, 0.1, 1000)
        trail = bridge.audit_trail()
        events = [entry['event'] for entry in trail]
        self.assertIn('command_intent', events)
        self.assertIn('sdk_response', events)
        self.assertIn('result', events)
        ids = [entry.get('command_id') for entry in trail if 'command_id' in entry]
        self.assertIn('cmd-42', ids)


if __name__ == '__main__':
    unittest.main(verbosity=2)
