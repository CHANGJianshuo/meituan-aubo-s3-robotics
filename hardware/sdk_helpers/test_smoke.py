"""Offline fault injection: never creates an SDK connection."""
import math
import unittest
from unittest.mock import patch
import smoke


def state(q=None):
    return dict(q=q or [0.0]*6, qd=[0.0]*6, read_seconds=0.01, robot_mode='Running',
                safety_mode='ReducedMode', control_mode='Position', operational_mode='Automatic',
                simulation=False, freedrive=False, servo=False, within_limits=True, collision=False,
                runtime='Running', exec_id=-1, queue=0, speed_fraction=0.1)


class FakeLog:
    def __init__(self): self.events = []
    def event(self, name, **kwargs): self.events.append((name, kwargs))


class Clock:
    def __init__(self): self.t = 0
    def monotonic(self): self.t += 0.02; return self.t
    def sleep(self, seconds): self.t += seconds


class FakeRobot:
    def __init__(self, snapshots=None, result=0, lost_reply=False):
        self.calls = []
        self.snapshots = snapshots
        self.result = result
        self.lost_reply = lost_reply
    def move(self, target):
        self.calls.append(('move', target.copy()))
        self.target = target
        if self.lost_reply: raise TimeoutError('response lost after send')
        return self.result
    def snapshot(self):
        if isinstance(self.snapshots, Exception): raise self.snapshots
        return self.snapshots or state(self.target.copy())
    def stop(self): self.calls.append(('stop',)); return 0


class Tests(unittest.TestCase):
    def test_degree_conversion_and_only_j6(self):
        q = [0.2]*6
        target = smoke.target_for(q, 0.3, [-3]*6, [3]*6)
        self.assertEqual(target[:5], q[:5])
        self.assertAlmostEqual(target[5]-q[5], math.pi/600)
        self.assertEqual(q, [0.2]*6)

    def test_invalid_delta(self):
        for x in [0, 0.01, 0.501, -0.501, 30, float('nan'), float('inf')]:
            with self.subTest(x=x), self.assertRaises(smoke.Refused): smoke.checked_delta(x)

    def test_negative_delta(self):
        target = smoke.target_for([0]*6, -0.3, [-3]*6, [3]*6)
        self.assertLess(target[5], 0)

    def test_malformed_joint_data(self):
        for q in [[0]*5, [float('nan')]*6, [True]*6, ['0']*6]:
            with self.subTest(q=q), self.assertRaises(smoke.Refused):
                smoke.target_for(q, 0.3, [-3]*6, [3]*6)

    def test_limit_margin_start_and_target(self):
        for value in [3, 3-1*smoke.RAD, 3-2.1*smoke.RAD]:
            q = [0]*5+[value]
            with self.subTest(value=value), self.assertRaises(smoke.Refused):
                smoke.target_for(q, 0.3, [-3]*6, [3]*6)

    def test_other_joint_near_limit(self):
        with self.assertRaises(smoke.Refused):
            smoke.target_for([3,0,0,0,0,0], 0.3, [-3]*6, [3]*6)

    def test_identity_requires_explicit_arming(self):
        identity = dict(dof=6, robot_type='S3', robot_name='rob1', controller_version='checked',
                        joint_ids=list(range(6)))
        cfg = dict(allow_motion=True, robot_name='rob1', expected_controller_version='checked',
                   expected_joint_ids=list(range(6)))
        smoke.check_identity(identity, cfg)
        for key,value in [('allow_motion',False),('robot_name','other'),
                          ('expected_controller_version',''),('expected_joint_ids',[])]:
            bad = {**cfg,key:value}
            with self.subTest(key=key), self.assertRaises(smoke.Refused): smoke.check_identity(identity,bad)
        with self.assertRaises(smoke.Refused): smoke.check_identity({**identity,'robot_type':'i5'},cfg)

    def test_nonready_states_are_rejected(self):
        cases = [('robot_mode','PowerOff'),('safety_mode','ProtectiveStop'),('safety_mode','Recovery'),
                 ('control_mode','Force'),('simulation',True),('freedrive',True),('servo',True),
                 ('within_limits',False),('collision',True),('runtime','Stopped'),('queue',1),
                 ('exec_id',2),('speed_fraction',0),('speed_fraction',float('nan')),
                 ('read_seconds',1.1),('qd',[0.01]*6)]
        for key,value in cases:
            with self.subTest(key=key), self.assertRaises(smoke.Refused):
                smoke.check_state({**state(),key:value},idle=True)

    def run_fake(self, robot, target=None):
        clock=Clock()
        with patch.object(smoke.time,'monotonic',clock.monotonic), patch.object(smoke.time,'sleep',clock.sleep):
            return smoke.execute_once(robot,[0]*6,target or [0]*5+[0.3*smoke.RAD],FakeLog(),timeout=2)

    def test_exactly_one_move_and_no_automatic_return(self):
        robot=FakeRobot();self.run_fake(robot)
        self.assertEqual([x[0] for x in robot.calls],['move'])

    def test_lost_reply_stops_without_resend(self):
        robot=FakeRobot(lost_reply=True)
        with self.assertRaises(TimeoutError):self.run_fake(robot)
        self.assertEqual([x[0] for x in robot.calls],['move','stop'])

    def test_rejected_command_stops_without_retry(self):
        robot=FakeRobot(result=3)
        with self.assertRaises(smoke.Refused):self.run_fake(robot)
        self.assertEqual([x[0] for x in robot.calls],['move','stop'])

    def test_disconnect_attempts_stop(self):
        robot=FakeRobot(snapshots=ConnectionError('lost link'))
        with self.assertRaises(ConnectionError):self.run_fake(robot)
        self.assertEqual(robot.calls[-1],('stop',))

    def test_uncommanded_joint_motion_aborts(self):
        robot=FakeRobot(snapshots=state([0.001,0,0,0,0,0]))
        with self.assertRaises(smoke.Refused):self.run_fake(robot)
        self.assertEqual(robot.calls[-1],('stop',))

    def test_excess_speed_aborts(self):
        robot=FakeRobot(snapshots={**state(),'qd':[0.02]*6})
        with self.assertRaises(smoke.Refused):self.run_fake(robot)
        self.assertEqual(robot.calls[-1],('stop',))

    def test_never_started_motion_times_out(self):
        robot=FakeRobot(snapshots=state())
        with self.assertRaises(smoke.Refused):self.run_fake(robot)
        self.assertEqual(robot.calls[-1],('stop',))

    def test_arrived_but_queue_not_empty_is_not_success(self):
        robot=FakeRobot(snapshots={**state([0]*5+[0.3*smoke.RAD]),'queue':1})
        with self.assertRaises(smoke.Refused):self.run_fake(robot)

    def test_interrupt_attempts_stop(self):
        robot=FakeRobot()
        robot.snapshot=lambda: (_ for _ in ()).throw(KeyboardInterrupt())
        with self.assertRaises(KeyboardInterrupt):self.run_fake(robot)
        self.assertEqual(robot.calls[-1],('stop',))

    def test_stop_failure_does_not_mask_original_error(self):
        robot=FakeRobot(lost_reply=True)
        robot.stop=lambda: (_ for _ in ()).throw(ConnectionError('stop lost'))
        with self.assertRaises(TimeoutError):self.run_fake(robot)

    def test_sdk_adapter_move_signature_and_units(self):
        from unittest.mock import Mock
        robot=object.__new__(smoke.Robot);robot.motion=Mock()
        robot.move([0]*6)
        robot.motion.moveJoint.assert_called_once_with([0]*6,0.5*smoke.RAD,0.2*smoke.RAD,0.0,0.0)

    def test_disabled_move_never_connects(self):
        with patch.object(smoke,'load_config',return_value={'allow_motion':False}), \
             patch.object(smoke,'Robot') as adapter, \
             patch.object(smoke.sys,'argv',['smoke.py','move']):
            with self.assertRaises(smoke.Refused):smoke.main()
            adapter.assert_not_called()

    def test_motion_refuses_noninteractive_input(self):
        with patch.object(smoke,'load_config',return_value={'allow_motion':True}), \
             patch.object(smoke,'Robot') as adapter,patch.object(smoke.sys.stdin,'isatty',return_value=False), \
             patch.object(smoke.sys,'argv',['smoke.py','move']):
            with self.assertRaises(smoke.Refused):smoke.main()
            adapter.assert_not_called()

    def test_stationary_position_drift_rejected(self):
        from unittest.mock import Mock
        robot=Mock();robot.snapshot.side_effect=[state(),state([0,0,0,0,0,0.03*smoke.RAD])]
        clock=Clock()
        with patch.object(smoke.time,'monotonic',clock.monotonic),patch.object(smoke.time,'sleep',clock.sleep):
            with self.assertRaises(smoke.Refused):smoke.stationary(robot,FakeLog())
        robot.move.assert_not_called()

    def test_read_only_path_has_no_mutating_calls(self):
        # At the CLI boundary a fake SDK adapter forbids movement and records the called methods.
        from tempfile import TemporaryDirectory
        class Reader:
            def __init__(self,*args): self.called=[]
            def identity(self):self.called.append('identity');return {'robot_name':'rob1'}
            def snapshot(self):self.called.append('snapshot');return state()
            def close(self):self.called.append('close')
            def move(self,*args):raise AssertionError('inspect sent motion')
        reader=Reader()
        with TemporaryDirectory() as folder, patch.object(smoke,'ROOT',smoke.Path(folder)), \
             patch.object(smoke,'load_config',return_value={'host':'192.0.2.1','port':30004}), \
             patch.object(smoke,'Robot',return_value=reader),patch.object(smoke,'Log',return_value=FakeLog()) as log, \
             patch.dict(smoke.os.environ,{'AUBO_USER':'test','AUBO_PASSWORD':'not-real'}), \
             patch.object(smoke.sys,'argv',['smoke.py','inspect']),patch.object(smoke.time,'sleep'):
            log.return_value.path='offline-test'
            self.assertEqual(smoke.main(),0)
        self.assertEqual(reader.called,['identity']+['snapshot']*5+['close'])


if __name__ == '__main__': unittest.main()
