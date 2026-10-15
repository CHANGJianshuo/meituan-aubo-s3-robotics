"""Offline fault tests for the one-degree/second RTDE commissioning profile."""
import copy
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
import j6_45_fast as m
import test_j6_45 as prior


class Robot(prior.Robot):
    def __init__(self):
        super().__init__()
        self.config.getJointMaxSpeeds.return_value=[1.]*6
        self.config.getLimitJointMaxSpeeds.return_value=[1.]*6
        self.config.getJointMaxAccelerations.return_value=[1.]*6
        self.config.getLimitJointMaxAccelerations.return_value=[1.]*6
        self.fast_fault=None
    def fast(self):
        s=dict(q=self.q.copy(),qd=[0]*6,target_q=self.q.copy(),target_qd=[0]*6,
               t=m.time.monotonic(),read_seconds=.01)
        if self.calls and self.fast_fault:s.update(self.fast_fault)
        return s
    def move_stage(self,target,speed,acceleration):return self.move(target)
    def begin_stage(self):pass
    def fast_samples(self):return [self.fast()]


class Tests(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        p=patch.object(m,'ROOT',self.root);p.start();self.addCleanup(p.stop)
        clock=prior.Clock()
        for name,fn in [('monotonic',clock.monotonic),('sleep',clock.sleep)]:
            p=patch.object(m.time,name,fn);p.start();self.addCleanup(p.stop)
        self.r=Robot();self.log=prior.Log()
        source=m.base.prepare(self.r,prior.CONFIG,self.log)
        data=json.dumps(source).encode();(self.root/'source.json').write_bytes(data)
        self.config={**prior.CONFIG,'allow_fast_motion':True,
                     'mission_45':{'source_plan':'source.json','sha256':hashlib.sha256(data).hexdigest()}}
        self.original=self.r.q.copy();self.r.q[5]+=.7090576171875*m.RAD
        self.plan=m.prepare(self.r,self.config,self.log)

    def test_prepare_is_readonly_and_preserves_original_45_endpoint(self):
        self.assertEqual(self.r.calls,[])
        self.assertAlmostEqual((self.plan['final_q'][5]-self.original[5])/m.RAD,45)
        self.assertLess((self.plan['final_q'][5]-self.plan['initial_q'][5])/m.RAD,45)

    def test_exactly_two_forward_stages_and_verified_original_endpoint(self):
        result=m.run_plan(self.r,self.config,self.plan,self.log,self.root/'attempts')
        self.assertEqual(len([c for c in self.r.calls if c[0]=='move']),2)
        self.assertAlmostEqual(result['cumulative_delta_deg'][5],45)
        self.assertFalse(result['automatic_return'])

    def test_limits_and_mission_tamper_block_before_send(self):
        self.r.config.getLimitJointMaxSpeeds.return_value=[.01]*6
        with self.assertRaises(m.smoke.Refused):m.validate_plan(self.plan,self.r,self.config,self.log)
        self.r.config.getLimitJointMaxSpeeds.return_value=[1.]*6
        (self.root/'source.json').write_text('{}')
        with self.assertRaises(m.smoke.Refused):m.reference(self.config)
        self.assertEqual(self.r.calls,[])

    def test_modified_speed_endpoint_or_start_refused(self):
        for key,value in [('final_q',[0]*6),('host','192.0.2.2')]:
            with self.assertRaises(m.smoke.Refused):m.validate_plan({**self.plan,key:value},self.r,self.config,self.log)
        p=copy.deepcopy(self.plan);p['stages'][1]['speed_deg_s']=6
        with self.assertRaises(m.smoke.Refused):m.validate_plan(p,self.r,self.config,self.log)
        self.r.q[0]+=.1*m.RAD
        with self.assertRaises(m.smoke.Refused):m.validate_plan(self.plan,self.r,self.config,self.log)
        self.assertEqual(self.r.calls,[])

    def test_failed_probe_stops_and_never_sends_fast_stage(self):
        self.r.fast_fault={'qd':[0]*5+[3*m.RAD]}
        with self.assertRaises(m.smoke.Refused):m.run_plan(self.r,self.config,self.plan,self.log,self.root/'attempts')
        self.assertEqual(len([c for c in self.r.calls if c[0]=='move']),1)
        self.assertEqual(self.r.calls[-1],('stop',))

    def test_lost_reply_no_repeat(self):
        self.r.lost=True
        with self.assertRaises(TimeoutError):m.run_plan(self.r,self.config,self.plan,self.log,self.root/'attempts')
        self.assertEqual([c[0] for c in self.r.calls],['move','stop'])

    def test_no_progress_times_out_and_stops(self):
        self.r.fast_fault={'q':self.r.q.copy(),'target_q':self.r.q.copy()}
        with self.assertRaisesRegex(m.smoke.Refused,'deadline'):
            m.run_plan(self.r,self.config,self.plan,self.log,self.root/'attempts')
        self.assertEqual([c[0] for c in self.r.calls],['move','stop'])

    def test_mode_change_mid_move_blocks_following_stages(self):
        self.r.fault={'simulation':True}
        with self.assertRaises(m.smoke.Refused):m.run_plan(self.r,self.config,self.plan,self.log,self.root/'attempts')
        self.assertEqual([c[0] for c in self.r.calls],['move','stop'])

    def test_execution_lock_prevents_any_command(self):
        with self.assertRaises(m.smoke.Refused):
            m.run_plan(self.r,{**self.config,'allow_fast_motion':False},self.plan,self.log,self.root/'attempts')
        self.assertEqual(self.r.calls,[])

    def test_original_mission_cannot_be_replayed(self):
        m.run_plan(self.r,self.config,self.plan,self.log,self.root/'attempts')
        self.r.q=self.plan['initial_q'].copy();self.r.calls=[]
        with self.assertRaises(FileExistsError):m.run_plan(self.r,self.config,self.plan,self.log,self.root/'attempts')
        self.assertEqual(self.r.calls,[])

    def test_feedback_guards_reject_bad_data_and_disagreement(self):
        start=[0]*6;target=[0]*5+[m.RAD]
        valid=dict(q=start.copy(),qd=[0]*6,target_q=start.copy(),target_qd=[0]*6,t=1.,read_seconds=.01)
        faults=[{'qd':[0]*5+[1.6*m.RAD]},{'target_qd':[0]*5+[1.1*m.RAD]},
                {'target_q':[0]*5+[.2*m.RAD]},
                {'read_seconds':.2},{'q':[float('nan')]*6},{'q':[.1*m.RAD]*6},
                {'qd':[.5*m.RAD]*5+[0]}]
        for f in faults:
            with self.subTest(fault=f),self.assertRaises(m.smoke.Refused):
                m.FeedbackGuard(start,target,start,1.).check({**valid,**f})
        g=m.FeedbackGuard(start,target,start,1.)
        g.check(valid)
        with self.assertRaises(m.smoke.Refused):g.check({**valid,'t':1.3})
        g=m.FeedbackGuard(start,target,start,1.);g.check(valid)
        fast={**valid,'t':1.16,'q':[0]*5+[.3*m.RAD],'target_q':[0]*5+[.3*m.RAD]}
        with self.assertRaises(m.smoke.Refused):g.check(fast)

    def test_profile_parameters_are_radians_and_stop_is_bounded(self):
        from unittest.mock import Mock
        r=object.__new__(m.Robot);r.motion=Mock()
        r.move_stage([0]*6,1.,1.);r.stop()
        r.motion.moveJoint.assert_called_once_with([0]*6,1*m.RAD,1*m.RAD,0.,0.)
        r.motion.stopJoint.assert_called_once_with(1*m.RAD)

    def test_sustained_velocity_disagreement_still_stops(self):
        start=[0]*6;target=[0]*5+[m.RAD]
        g=m.FeedbackGuard(start,target,start,1.)
        def sample(t):return dict(q=start.copy(),qd=[0]*5+[m.RAD],target_q=start.copy(),
                                  target_qd=[0]*6,t=t,read_seconds=.001)
        g.check(sample(1.));g.check(sample(1.08))
        with self.assertRaises(m.smoke.Refused):g.check(sample(1.16))

    def test_brief_velocity_spike_below_peak_cap_does_not_mimic_sustained_fault(self):
        start=[0]*6;target=[0]*5+[m.RAD];g=m.FeedbackGuard(start,target,start,1.)
        for i in range(21):
            q=[0]*5+[i*.002*m.RAD]
            g.check(dict(q=q,qd=[0]*5+[(1.13 if i==10 else .2)*m.RAD],target_q=q,
                         target_qd=[0]*5+[.2*m.RAD],t=1+i*.01,read_seconds=.001))

    def test_following_delay_bound_still_rejects_stall(self):
        start=[0]*6;target=[0]*5+[5*m.RAD]
        sample=dict(q=start.copy(),qd=[0]*5+[5*m.RAD],
                    target_q=[0]*5+[.24*m.RAD],target_qd=[0]*5+[5*m.RAD],t=1.,read_seconds=.004)
        m.FeedbackGuard(start,target,start,5.).check(sample)
        with self.assertRaises(m.smoke.Refused):
            m.FeedbackGuard(start,target,start,5.).check({**sample,'target_q':[0]*5+[.3*m.RAD]})


if __name__=='__main__':unittest.main()
