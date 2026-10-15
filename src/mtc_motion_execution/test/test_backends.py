"""mtc_motion_execution 离线单元测试（纯 Python，不 import rclpy）。

运行方式：
    cd /home/robot/meituan_challenge_ws/src/mtc_motion_execution
    python3 test/test_backends.py
"""

import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mtc_motion_execution import (  # noqa: E402
    Clock,
    DisabledMotionBackend,
    ExecState,
    InFlightGuard,
    MockMotionBackend,
    MotionExecutorCore,
    ValidationConfig,
    make_backend,
)

JOINTS = ('shoulder_joint', 'upperArm_joint', 'foreArm_joint',
          'wrist1_joint', 'wrist2_joint', 'wrist3_joint')


class FakeClock(Clock):
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, seconds):
        self.t += max(seconds, 0.0)


def cfg():
    return ValidationConfig(
        expected_joint_names=JOINTS,
        max_velocity_scaling=0.2,
        max_acceleration_scaling=0.2,
        joint_limit_rad=tuple((-3.0, 3.0) for _ in JOINTS),
    )


def core(**kwargs):
    return MotionExecutorCore(backend_name='mock', config=cfg(),
                              clock=FakeClock(), **kwargs)


def move(c, cid='cmd-1', target=None, velocity=0.1, accel=0.1, timeout=30_000):
    return c.execute_joint_move(
        command_id=cid,
        joint_names=JOINTS,
        target_rad=target or (0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
        velocity_scaling=velocity,
        acceleration_scaling=accel,
        timeout_ms=timeout,
    )


class TestValidation(unittest.TestCase):
    def test_joint_order_mismatch_rejected(self):
        c = core()
        r = c.execute_joint_move('cmd-x', tuple(reversed(JOINTS)),
                                 (0.1, 0, 0, 0, 0, 0), 0.1, 0.1, 30_000)
        self.assertFalse(r.success)
        self.assertIs(r.exec_state, ExecState.REJECTED)
        self.assertEqual(r.error_code, 210)
        self.assertIn('顺序', r.message)

    def test_length_mismatch_rejected(self):
        c = core()
        r = c.execute_joint_move('cmd-y', JOINTS[:5], (0.1, 0, 0, 0, 0), 0.1, 0.1, 30_000)
        self.assertFalse(r.success)
        self.assertEqual(r.error_code, 210)

    def test_velocity_scaling_over_limit_is_interlock(self):
        c = core()
        r = move(c, velocity=0.9)
        self.assertFalse(r.success)
        self.assertEqual(r.error_code, 500)  # SAFETY_INTERLOCK
        self.assertIn('上限', r.message)

    def test_joint_limit_violation_is_interlock(self):
        c = core()
        r = move(c, target=(99.0, 0, 0, 0, 0, 0))
        self.assertFalse(r.success)
        self.assertEqual(r.error_code, 500)

    def test_zero_timeout_rejected(self):
        c = core()
        r = move(c, timeout=0)
        self.assertFalse(r.success)
        self.assertEqual(r.error_code, 210)


class TestExecution(unittest.TestCase):
    def test_success_path_settles(self):
        c = core()
        r = move(c)
        self.assertTrue(r.success)
        self.assertIs(r.exec_state, ExecState.SETTLED)
        self.assertTrue(r.stop_confirmed)
        self.assertEqual(r.final_position_rad, (0.1, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertEqual(r.error_code, 0)

    def test_concurrent_command_rejected(self):
        guard = InFlightGuard()
        self.assertTrue(guard.acquire('a')[0])
        ok, why = guard.acquire('b')
        self.assertFalse(ok)
        self.assertIn('在途命令', why)

    def test_second_command_rejected_while_in_flight(self):
        """核心层唯一执行权：并发第二个请求必须被拒绝。"""
        c = core()
        c._active_command_id = 'occupier'  # 模拟在途
        r = move(c, cid='cmd-2')
        self.assertFalse(r.success)
        self.assertIs(r.exec_state, ExecState.REJECTED)
        self.assertIn('并发', r.message)

    def test_trajectory_not_split_into_multiple_calls(self):
        """一个请求必须对应恰好一次后端调用，禁止把轨迹拆成高频调用。"""
        backend = MockMotionBackend(config=cfg(), clock=FakeClock())
        c = MotionExecutorCore(config=cfg(), clock=FakeClock(), backend=backend)
        move(c, cid='single')
        self.assertEqual(len(backend.call_log), 1)


class TestStopSemantics(unittest.TestCase):
    def test_stop_when_idle_confirms(self):
        c = core()
        delivered, message, confirmed = c.request_stop('test idle')
        self.assertTrue(delivered)
        self.assertTrue(confirmed)

    def _make_in_flight(self, cancel_confirms):
        """制造真实在途运动：后台线程执行长时运动，主线程在其运动期间请求停止。"""
        backend = MockMotionBackend(config=cfg(), clock=Clock(),
                                    cancel_confirms=cancel_confirms, travel_time_s=5.0)
        c = MotionExecutorCore(config=cfg(), clock=Clock(), backend=backend)
        result_box = {}

        def run():
            result_box['r'] = move(c, cid='in-flight', timeout=30_000)

        t = threading.Thread(target=run, daemon=True)
        t.start()
        time.sleep(0.15)  # 令后端处于 MOVING 且已记录在途命令
        return c, backend, t, result_box

    def test_cancel_without_backend_confirmation_is_not_confirmed(self):
        c, backend, t, box = self._make_in_flight(cancel_confirms=False)
        delivered, message, confirmed = c.request_stop('cancel', in_flight_command_id='in-flight')
        t.join(timeout=10)
        self.assertTrue(delivered)          # 请求送达
        self.assertFalse(confirmed)         # 但未确认停稳
        self.assertIn('未', message)
        self.assertIn('r', box)
        self.assertFalse(box['r'].success)
        self.assertTrue(box['r'].motion_state_unknown,
                        '取消未确认停稳时必须标记运动状态未知')

    def test_cancel_with_backend_confirmation(self):
        c, backend, t, box = self._make_in_flight(cancel_confirms=True)
        delivered, message, confirmed = c.request_stop('cancel', in_flight_command_id='in-flight')
        t.join(timeout=10)
        self.assertTrue(delivered)
        self.assertTrue(confirmed)
        self.assertIn('r', box)
        self.assertTrue(box['r'].stop_confirmed)


class TestUnknownState(unittest.TestCase):
    def test_status_unknown_locks_new_commands(self):
        backend = MockMotionBackend(config=cfg(), clock=FakeClock(), status_unknown=True)
        c = MotionExecutorCore(config=cfg(), clock=FakeClock(), backend=backend)
        r = move(c, cid='u1')
        self.assertFalse(r.success)
        self.assertEqual(r.error_code, 230)
        self.assertTrue(r.motion_state_unknown)
        self.assertFalse(r.retry_allowed_without_reobservation)

        # 之后任何新命令都必须被拒绝，且不得自动重发。
        r2 = move(c, cid='u2')
        self.assertFalse(r2.success)
        self.assertEqual(r2.error_code, 230)
        self.assertEqual(len(backend.call_log), 1)  # 后端只被调用一次

    def test_timeout_is_unknown_and_not_retryable(self):
        backend = MockMotionBackend(config=cfg(), clock=FakeClock(),
                                    travel_time_s=10.0)
        c = MotionExecutorCore(config=cfg(), clock=FakeClock(), backend=backend)
        r = move(c, timeout=1000)
        self.assertFalse(r.success)
        self.assertIs(r.exec_state, ExecState.TIMEOUT)
        self.assertEqual(r.error_code, 220)
        self.assertTrue(r.motion_state_unknown)
        self.assertFalse(r.retry_allowed_without_reobservation)

    def test_manual_clear_unlocks(self):
        backend = MockMotionBackend(config=cfg(), clock=FakeClock(), status_unknown=True)
        c = MotionExecutorCore(config=cfg(), clock=FakeClock(), backend=backend)
        move(c, cid='u1')
        c.clear_unknown_after_manual_confirmation('现场确认机械臂静止')
        # 解除后可以再次请求（mock 仍会返回 unknown，但锁已解除，说明出口生效）
        r = move(c, cid='u3')
        self.assertFalse(r.success)
        self.assertEqual(len(backend.call_log), 2)

    def test_backend_exception_becomes_unknown(self):
        class Boom(MockMotionBackend):
            def execute_joint_move(self, request):
                raise RuntimeError('rtde stream lost')

        c = MotionExecutorCore(config=cfg(), clock=FakeClock(),
                               backend=Boom(config=cfg(), clock=FakeClock()))
        r = move(c, cid='boom')
        self.assertFalse(r.success)
        self.assertEqual(r.error_code, 230)
        self.assertTrue(r.motion_state_unknown)
        self.assertIn('RuntimeError', r.message)


class TestDisabledBackend(unittest.TestCase):
    def test_disabled_backend_refuses_everything(self):
        backend = DisabledMotionBackend()
        caps = backend.capabilities().as_dict()
        self.assertFalse(caps['joint_goal_supported'])
        self.assertFalse(caps['timed_trajectory_supported'])
        self.assertFalse(caps['cartesian_motion_supported'])

    def test_unknown_backend_name_falls_back_to_disabled(self):
        for name in ('real', 'aubo', '', 'aubo_sdk', 'gazebo'):
            backend = make_backend(name)
            self.assertIsInstance(backend, DisabledMotionBackend,
                                  f'{name!r} 不应默认启用真实后端')

    def test_mock_backend_declares_no_trajectory_capability(self):
        caps = make_backend('mock', config=cfg(), clock=FakeClock()).capabilities()
        self.assertTrue(caps.joint_goal_supported)
        self.assertFalse(caps.timed_trajectory_supported,
                         'mock 不得声称支持时间参数化轨迹')
        self.assertFalse(caps.cartesian_motion_supported)


class TestHandshakeLog(unittest.TestCase):
    def test_handshake_records_steps(self):
        c = core()
        move(c, cid='h1')
        steps = c.handshake('h1').steps
        self.assertIn('PREPARE', steps)
        self.assertIn('ACCEPT', steps)
        self.assertIn('VALIDATE', steps)
        self.assertIn('SEND', steps)
        self.assertIn('MONITOR:SETTLED', steps)
        self.assertIn('CONFIRM:settled', steps)


if __name__ == '__main__':
    unittest.main(verbosity=2)
