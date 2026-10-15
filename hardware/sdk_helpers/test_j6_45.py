"""Offline fake-device fault injection. Never instantiates an SDK connection."""
from datetime import datetime, timezone, timedelta
import copy
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch, Mock
import j6_45 as m


IDENTITY=dict(robot_name='rob1',robot_type='aubo_S3',robot_subtype='',dof=6,
              controller_version=m.VERSION,joint_ids=['1','2','3','4','5','6'])
CONFIG=dict(host='192.0.2.1',port=30004,expected_identity=IDENTITY,allow_motion=True,
            bare_flange_confirmed=True,onsite_safety_confirmed=True)


def state(q):
    return dict(q=q.copy(),qd=[0.]*6,power_on=True,robot_mode='Running',safety_mode='Normal',
                simulation=False,joint_states=['Running']*6,joint_servo_modes=['Position']*6,
                control_mode='Unknown',force_control=False,soft_float=False,backdrive=False,
                freedrive=False,servo=False,collision=False,within_limits=True,runtime='Stopped',
                operational_mode='Automatic',read_seconds=.02,speed_fraction=1.,exec_id=-1,queue=0)


class Clock:
    def __init__(self):self.t=0
    def monotonic(self):self.t+=.01;return self.t
    def sleep(self,s):self.t+=s


class Log:
    path=Path('/tmp/FAKE-OFFLINE-ONLY.jsonl')
    def __init__(self):self.events=[]
    def event(self,name,**fields):self.events.append((name,fields))


class Robot:
    def __init__(self):
        self.q=[.2,-.3,.4,-.5,.6,.01];self.calls=[];self.fault=None;self.result=0;self.lost=False
        self.config=Mock();self.config.getPayload.return_value=(0.,[0]*3,[0]*3,[0]*9)
        self.config.getTcpOffset.return_value=[0]*6
        self.runtime=Mock();self.runtime.isTaskAlive.return_value=False
    def identity(self):return copy.deepcopy(IDENTITY)
    def limits(self):return [-3.]*6,[3.]*6
    def snapshot(self):
        s=state(self.q)
        if self.fault and self.calls:s.update(self.fault)
        return s
    def move(self,target):
        self.calls.append(('move',target.copy()))
        if self.lost:raise TimeoutError('reply lost')
        if self.result==0:self.q=target.copy()
        return self.result
    def stop(self):self.calls.append(('stop',));return 0


class Tests(unittest.TestCase):
    def setUp(self):
        clock=Clock()
        for name,fn in [('monotonic',clock.monotonic),('sleep',clock.sleep)]:
            p=patch.object(m.time,name,fn);p.start();self.addCleanup(p.stop)
        self.r=Robot();self.log=Log();self.plan=m.prepare(self.r,CONFIG,self.log)

    def test_prepare_never_moves(self):
        self.assertEqual(self.r.calls,[])
        self.assertAlmostEqual(self.plan['nominal_motion_seconds'],225)

    def test_two_stages_are_total_45_not_45_point_3(self):
        with TemporaryDirectory() as d:
            result=m.run_plan(self.r,CONFIG,self.plan,self.log,Path(d))
        moves=[x[1] for x in self.r.calls if x[0]=='move']
        self.assertEqual(len(moves),2)
        self.assertAlmostEqual((moves[0][5]-self.plan['initial_q'][5])/m.RAD,.3)
        self.assertAlmostEqual(result['measured_delta_deg'][5],45)
        self.assertEqual(result['measured_delta_deg'][:5],[0]*5)
        self.assertFalse(any(c[0]=='stop' for c in self.r.calls))

    def test_readiness_faults(self):
        faults=[{'simulation':True},{'runtime':'Running'},{'joint_servo_modes':['Position']*5+['Torque']},
                {'joint_states':['Running']*5+['Fault']},{'control_mode':'Force'},{'power_on':False},
                {'force_control':True},{'soft_float':True},{'backdrive':True},{'freedrive':True},
                {'servo':True},{'safety_mode':'Undefined'},{'collision':True},{'within_limits':False},
                {'read_seconds':1.1},{'q':[float('nan')]*6},{'speed_fraction':0},
                {'speed_fraction':float('nan')},{'queue':1},{'qd':[.1]*6}]
        for fault in faults:
            with self.subTest(fault=fault),self.assertRaises(m.smoke.Refused):
                m.validate_state({**state(self.r.q),**fault},idle=True)

    def test_aggregate_unknown_requires_all_drive_and_force_checks(self):
        m.validate_state(state(self.r.q),idle=True)
        for fault in ({'joint_servo_modes':[]},{'force_control':True},{'soft_float':True}):
            with self.assertRaises(m.smoke.Refused):m.validate_state({**state(self.r.q),**fault})

    def test_all_motion_faults_stop_and_do_not_send_second_stage(self):
        for fault in ({'simulation':True},{'qd':[0]*5+[m.RAD]},
                      {'q':[.2+1,-.3,.4,-.5,.6,.01]},{'runtime':'Running'},
                      {'speed_fraction':.5},{'operational_mode':'Manual'}):
            r=Robot();r.fault=fault
            with TemporaryDirectory() as d,self.subTest(fault=fault),self.assertRaises(m.smoke.Refused):
                m.run_plan(r,CONFIG,self.plan,self.log,Path(d))
            self.assertEqual(len([c for c in r.calls if c[0]=='move']),1)
            self.assertTrue(any(c[0]=='stop' for c in r.calls))

    def test_lost_response_does_not_repeat_move(self):
        self.r.lost=True
        with TemporaryDirectory() as d,self.assertRaises(TimeoutError):
            m.run_plan(self.r,CONFIG,self.plan,self.log,Path(d))
        self.assertEqual(len([c for c in self.r.calls if c[0]=='move']),1)
        self.assertTrue(any(c[0]=='stop' for c in self.r.calls))

    def test_nonzero_response_does_not_continue(self):
        self.r.result=3
        with TemporaryDirectory() as d,self.assertRaises(m.smoke.Refused):
            m.run_plan(self.r,CONFIG,self.plan,self.log,Path(d))
        self.assertEqual(len([c for c in self.r.calls if c[0]=='move']),1)

    def test_plan_replay_blocked(self):
        with TemporaryDirectory() as d:
            m.run_plan(self.r,CONFIG,self.plan,self.log,Path(d))
            other=Robot()
            with self.assertRaises(FileExistsError):m.run_plan(other,CONFIG,self.plan,self.log,Path(d))
            self.assertEqual(other.calls,[])

    def test_plan_tamper_expiry_moved_start_identity(self):
        for key,value in [('delta_deg',90),('target_q',[0]*6),('probe_q',[0]*6),('host','192.0.2.2'),
                          ('speed_deg_s',1),('identity',{**IDENTITY,'robot_name':'other'}),
                          ('created_utc',(datetime.now(timezone.utc)-timedelta(seconds=601)).isoformat()),
                          ('created_utc',(datetime.now(timezone.utc)+timedelta(seconds=60)).isoformat())]:
            with self.subTest(key=key),self.assertRaises(m.smoke.Refused):
                m.validate_plan({**self.plan,key:value},self.r,CONFIG,self.log)
        self.r.q[5]+=.04*m.RAD
        with self.assertRaises(m.smoke.Refused):m.validate_plan(self.plan,self.r,CONFIG,self.log)
        self.assertEqual(self.r.calls,[])

    def test_disabled_execution_and_cli_never_connect(self):
        with TemporaryDirectory() as d,self.assertRaises(m.smoke.Refused):
            m.run_plan(self.r,{**CONFIG,'allow_motion':False},self.plan,self.log,Path(d))
        with patch.object(m.smoke,'load_config',return_value={**CONFIG,'allow_motion':False}), \
             patch.object(m,'Robot45') as r,patch.object(m.sys,'argv',['j6_45.py','execute']):
            with self.assertRaises(m.smoke.Refused):m.main()
            r.assert_not_called()

    def test_payload_tcp_and_position_limits(self):
        for payload,tcp in [((.1,),[0]*6),((float('nan'),),[0]*6),((0,),[.01]*6)]:
            with self.assertRaises(m.smoke.Refused):m.validate_bare_flange(payload,tcp)
        with self.assertRaises(m.smoke.Refused):m.targets([0]*5+[2.5],[-3]*6,[3]*6)

    def test_adapter_parameters(self):
        r=object.__new__(m.Robot45);r.motion=Mock();r.move([0]*6);r.stop()
        r.motion.moveJoint.assert_called_once_with([0]*6,.5*m.RAD,.2*m.RAD,0.,0.)
        r.motion.stopJoint.assert_called_once_with(1.*m.RAD)

    def test_timeout_and_stop_failure_keep_original_error(self):
        self.r.lost=True;self.r.stop=lambda: (_ for _ in ()).throw(ConnectionError('lost'))
        with self.assertRaises(TimeoutError):
            m.execute_segment(self.r,self.r.q,self.plan['probe_q'],self.plan,self.log,'test')
        self.assertEqual(len(self.r.calls),1)

    def test_no_physical_progress_times_out_without_second_stage(self):
        initial=self.r.q.copy()
        self.r.snapshot=lambda: state(initial)
        with TemporaryDirectory() as d,self.assertRaisesRegex(m.smoke.Refused,'timed out'):
            m.run_plan(self.r,CONFIG,self.plan,self.log,Path(d))
        self.assertEqual(len([c for c in self.r.calls if c[0]=='move']),1)
        self.assertTrue(any(c[0]=='stop' for c in self.r.calls))

    def test_feedback_loss_after_send_stops_without_retry(self):
        snapshot=self.r.snapshot
        def lose_feedback():
            if self.r.calls:raise ConnectionError('feedback lost')
            return snapshot()
        self.r.snapshot=lose_feedback
        with TemporaryDirectory() as d,self.assertRaises(ConnectionError):
            m.run_plan(self.r,CONFIG,self.plan,self.log,Path(d))
        self.assertEqual(len([c for c in self.r.calls if c[0]=='move']),1)
        self.assertTrue(any(c[0]=='stop' for c in self.r.calls))

    def test_payload_changed_after_probe_blocks_second_stage(self):
        self.r.config.getPayload.side_effect=[(0.,),(1.,)]
        with TemporaryDirectory() as d,self.assertRaises(m.smoke.Refused):
            m.run_plan(self.r,CONFIG,self.plan,self.log,Path(d))
        self.assertEqual(len([c for c in self.r.calls if c[0]=='move']),1)
        self.assertTrue(any(c[0]=='stop' for c in self.r.calls))


if __name__=='__main__':unittest.main()
