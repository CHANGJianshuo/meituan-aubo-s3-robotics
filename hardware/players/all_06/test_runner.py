"""Failure-focused offline tests. Fakes only; never connects to a robot."""
import copy
from contextlib import nullcontext, redirect_stdout
import hashlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import run_hardware as runner
from trajectory import JOINTS, RAD, dense, interpolate, load_package, validate


def point(t, q, v=None, a=None):
    return dict(t=t, positions=list(q), velocities=v or [0.]*6, accelerations=a or [0.]*6)


def segment():
    return dict(primitive='test', points=[point(0., [0.]*6), point(1., [.01]*6)])


def bundle():
    return dict(schema=1, plan_id='a'*32, joint_names=JOINTS,
                units={'position':'rad', 'velocity':'rad/s', 'acceleration':'rad/s^2', 'time':'s'},
                selection='full', segments=[segment()])


def state(q=None):
    return dict(q=q or [0.]*6, qd=[0.]*6, robot_mode='Running', power_on=True,
                safety_mode='Normal', joint_states=['Running']*6, joint_servo_modes=['Position']*6,
                control_mode='Unknown', simulation=False, freedrive=False, servo=False, collision=False,
                force_control=False, soft_float=False, backdrive=False, within_limits=True,
                runtime='Stopped', operational_mode='Automatic', read_seconds=.02,
                speed_fraction=1., exec_id=-1, queue=0)


def policy_manifest():
    return dict(tracking_guard_policy='task2_all_0p6_v1',tracking_guard_deg=[.6]*6)


class Clock:
    def __init__(self): self.now=0.
    def monotonic(self): return self.now
    def sleep(self, seconds): self.now+=seconds


class Log:
    def __init__(self): self.events=[]
    def event(self, event, **kw): self.events.append((event,kw))


class Motion:
    def __init__(self, fail=None): self.calls=[]; self.fail=fail
    def pathBufferAlloc(self, name, kind, count):
        self.calls.append(('alloc', name, kind, count)); return 0
    def pathBufferAppend(self, name, positions):
        self.calls.append(('append', positions)); return 0
    def pathBufferEval(self, name, a, v, interval):
        self.calls.append(('eval', interval)); return 0
    def pathBufferValid(self, name): return True
    def pathBufferFree(self, name): self.calls.append(('free', name)); return 0
    def movePathBuffer(self, name):
        self.calls.append(('move', name))
        if self.fail: raise self.fail
        return 0
    def stopJoint(self, acceleration): self.calls.append(('stop', acceleration)); return 0


class Robot:
    def __init__(self, motion=None, q=None): self.motion=motion or Motion(); self.q=q
    def snapshot(self): return state(self.q)


class Stream:
    def __init__(self, fail=False): self.calls=0; self.fail=fail
    def drain(self):
        self.calls+=1
        if self.fail and self.calls>1: raise ValueError('RTDE feedback stale')
        return []


class Tests(unittest.TestCase):
    def test_all_six_joints_accept_point_five_five_in_every_task_stage(self):
        for joint in range(6):
            sample=dict(q=[0.]*6,qd=[0.]*6,target_q=[0.]*6,target_qd=[0.]*6,target_qdd=[0.]*6,
                        robot_mode='Running',safety_mode='Normal',runtime='Stopped')
            sample['target_q'][joint]=.55*RAD;log=Log()
            for stage in range(1,31):
                runner.check_stream([sample],([-1.]*6,[1.]*6),log,stage=stage)
            self.assertEqual([e for e,_ in log.events],['rtde']*30)

    def test_every_joint_rejects_point_six_one_and_stops_once_with_exact_sample(self):
        for joint in range(6):
            sample=dict(q=[0.]*6,qd=[0.]*6,target_q=[0.]*6,target_qd=[0.]*6,target_qdd=[0.]*6,
                        robot_mode='Running',safety_mode='Normal',runtime='Stopped')
            sample['target_q'][joint]=.61*RAD
            robot=Robot();clock=Clock();log=Log();stream=Mock();stream.drain.side_effect=[[],[sample]]
            with patch.object(runner,'time',clock),patch.object(runner,'configuration_check'):
                with self.assertRaisesRegex(ValueError,f'J{joint+1} tracking error .* > 0.6 deg'):
                    runner.run_segment(robot,stream,segment(),{'operational_mode':'Automatic'},1.,1.,log,11)
            self.assertEqual(sum(c[0]=='move' for c in robot.motion.calls),1)
            self.assertEqual(sum(c[0]=='stop' for c in robot.motion.calls),1)
            rejection=next(d for e,d in log.events if e=='tracking_guard_rejected')
            self.assertAlmostEqual(rejection['error_deg'],.61);self.assertEqual(rejection['limit_deg'],.6)
            self.assertEqual(rejection['sample'],sample);self.assertEqual(rejection['joint'],joint+1)
            self.assertEqual(rejection['stage'],11)

    def test_policy_must_match_manifest_before_any_controller_access(self):
        runner.verify_tracking_policy(policy_manifest())
        for bad in [{},dict(uniform_tracking_guard_deg=.5),
                    {**policy_manifest(),'uniform_tracking_guard_deg':.5},
                    {**policy_manifest(),'tracking_guard_deg':[.5,.5,.5,.6,.5,.5]},
                    {**policy_manifest(),'tracking_guard_policy':'other'}]:
            robot=Mock()
            with self.assertRaisesRegex(ValueError,'Tracking policy'):
                runner.configuration_check(robot,bad)
            self.assertFalse(robot.mock_calls)

    def test_transient_speed_resets_continuous_stillness_without_relaxing_threshold(self):
        robot=Robot(); clock=Clock(); log=Log()
        def feedback():
            s=state()
            if .79 < clock.now < 1.01:
                s['qd'][5]=.03*RAD
            return s
        with patch.object(runner,'time',clock), patch.object(robot,'snapshot',side_effect=feedback):
            runner.stationary(robot,[0.]*6,{'operational_mode':'Automatic'},log)
        self.assertGreaterEqual(clock.now,2.09)
        self.assertLess(clock.now,2.3)
        self.assertEqual(runner.STILL_SPEED,.02*RAD)
        waits=[d for e,d in log.events if e=='stationary_wait']
        self.assertTrue(waits)
        self.assertAlmostEqual(waits[0]['max_speed_deg_s'],.03)
        self.assertFalse(robot.motion.calls)

    def test_continuing_motion_times_out_before_upload_or_move_and_records_speed(self):
        robot=Robot(); clock=Clock(); log=Log(); moving=state()
        moving['qd'][2]=.03*RAD
        with patch.object(runner,'time',clock), patch.object(robot,'snapshot',return_value=moving):
            with self.assertRaisesRegex(ValueError,'within 5 s'):
                runner.run_segment(robot,Stream(),segment(),{'operational_mode':'Automatic'},1.,1.,log,1)
        self.assertGreaterEqual(clock.now,5.)
        self.assertLess(clock.now,5.11)
        self.assertFalse(any(c[0] in ('alloc','move','stop') for c in robot.motion.calls))
        samples=[d for e,d in log.events if e=='stationary_sample']
        self.assertEqual(samples[-1]['qd'],moving['qd'])
        self.assertTrue(any(e=='stationary_timeout' for e,_ in log.events))

    def test_settle_wait_still_rejects_unsafe_state_queue_or_position_immediately(self):
        cases=[('queue',1),('exec_id',3),('collision',True),('runtime','Running'),
               ('robot_mode','Idle'),('q',[.06*RAD]*6),('speed_fraction',.5)]
        for key,value in cases:
            with self.subTest(key=key):
                robot=Robot(); clock=Clock(); log=Log(); bad=state()
                bad[key]=value; bad['qd'][0]=.03*RAD
                with patch.object(runner,'time',clock), patch.object(robot,'snapshot',return_value=bad):
                    with self.assertRaises(ValueError):
                        runner.stationary(robot,[0.]*6,{'operational_mode':'Automatic'},log,fraction=1.)
                self.assertEqual(clock.now,0.)
                self.assertEqual(log.events[0][0],'stationary_sample')
                self.assertEqual(log.events[0][1][key],value)
                self.assertEqual(log.events[-1][0],'stationary_rejected')
                self.assertFalse(robot.motion.calls)

    def test_cancellation_during_settle_wait_propagates(self):
        robot=Robot(); clock=Clock(); moving=state(); moving['qd'][0]=.03*RAD
        with patch.object(runner,'time',clock), patch.object(robot,'snapshot',side_effect=[moving,KeyboardInterrupt()]):
            with self.assertRaises(KeyboardInterrupt):
                runner.stationary(robot,[0.]*6,{'operational_mode':'Automatic'},Log())
        self.assertFalse(robot.motion.calls)

    def test_feedback_speed_does_not_stop_playback(self):
        for field in ('qd', 'target_qd'):
            with self.subTest(field=field):
                robot=Robot(); clock=Clock(); log=Log(); stream=Mock()
                sample=dict(q=[0.]*6,qd=[0.]*6,target_q=[0.]*6,
                            target_qd=[0.]*6,target_qdd=[0.]*6,
                            robot_mode='Running',safety_mode='Normal',runtime='Stopped')
                sample[field][1]=3.*RAD
                stream.drain.return_value=[sample]
                def feedback():
                    moved=any(c[0]=='move' for c in robot.motion.calls)
                    return state([.01]*6 if moved else [0.]*6)
                with patch.object(runner,'time',clock), patch.object(runner,'configuration_check'), \
                        patch.object(robot,'snapshot',side_effect=feedback):
                    runner.run_segment(robot,stream,segment(),{'operational_mode':'Automatic'},2.,1.,log,1)
                self.assertEqual(sum(c[0]=='move' for c in robot.motion.calls),1)
                self.assertFalse(any(c[0]=='stop' for c in robot.motion.calls))
                self.assertTrue(any(event=='stage_complete' for event,_ in log.events))
                feedback_samples=[data for event,data in log.events if event=='rtde']
                self.assertTrue(feedback_samples)
                self.assertEqual(feedback_samples[0],sample)

    def test_tracking_failure_still_stops_without_speed_checks(self):
        robot=Robot(); clock=Clock(); log=Log(); stream=Mock()
        sample=dict(q=[0.]*6,qd=[3.*RAD]*6,target_q=[.012]*6,
                    target_qd=[0.]*6,target_qdd=[0.]*6,
                    robot_mode='Running',safety_mode='Normal',runtime='Stopped')
        stream.drain.side_effect=[[], [sample]]
        with patch.object(runner,'time',clock), patch.object(runner,'configuration_check'):
            with self.assertRaisesRegex(ValueError,'tracking error'):
                runner.run_segment(robot,stream,segment(),{'operational_mode':'Automatic'},2.,1.,log,1)
        self.assertEqual(sum(c[0]=='move' for c in robot.motion.calls),1)
        self.assertEqual(sum(c[0]=='stop' for c in robot.motion.calls),1)
        self.assertFalse(any(c[0]=='free' for c in robot.motion.calls))

    def test_quintic_rest_endpoints_and_monotone_bridge(self):
        start=point(0., [3.3,.1,-2.1,-2.5,4.3,-.1]); end=point(123., [0.,.35,1.3,-.35,1.57,0.])
        for t,p in [(0,start),(123,end)]:
            q,v,a=interpolate(start,end,t)
            for got,want in zip(q,p['positions']): self.assertAlmostEqual(got,want,places=12)
            self.assertLess(max(map(abs,v)),1e-12); self.assertLess(max(map(abs,a)),1e-12)
        for step in range(1001):
            q,_,_=interpolate(start,end,123*step/1000)
            for i in range(6):
                lo,hi=sorted((start['positions'][i],end['positions'][i]))
                self.assertTrue(lo-1e-12<=q[i]<=hi+1e-12)

    def test_nonfinite_time_wrong_units_and_discontinuous_join_rejected(self):
        for mutate in [lambda b:b['segments'][0]['points'][1].update(t=math.nan),
                       lambda b:b['units'].update(position='deg'),
                       lambda b:b['segments'].append(segment())]:
            b=bundle(); mutate(b)
            with self.assertRaises(ValueError): validate(b)

    def test_interpolation_overshoot_rejected_even_when_waypoints_in_limits(self):
        b=bundle(); b['segments'][0]['points'][0]['velocities']=[20.]*6
        with self.assertRaises(ValueError): validate(b,[[-1.]*6,[1.]*6])

    def test_checksum_change_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); b=bundle(); data=json.dumps(b).encode()
            (root/'full_trajectory.json').write_bytes(data)
            m=dict(plan_id=b['plan_id'],captured_limits=[[-3.]*6,[3.]*6],sha256={'full_trajectory.json':hashlib.sha256(data).hexdigest()})
            (root/'manifest.json').write_text(json.dumps(m))
            load_package(root,'full')
            (root/'full_trajectory.json').write_bytes(data+b' ')
            with self.assertRaisesRegex(ValueError,'checksum'): load_package(root,'full')

    def test_dense_contains_exact_endpoints_and_uploads_last_partial_batch(self):
        dt,values=dense(segment(),.013)
        self.assertAlmostEqual(dt*(len(values)-1),1.)
        self.assertEqual(values[0][0],[0.]*6); self.assertEqual(values[-1][0],[.01]*6)
        positions=[[float(i)]*6 for i in range(253)]
        motion=Motion(); runner.prepare_buffer(motion,'test',positions,.03,Log())
        flat=[q for c in motion.calls if c[0]=='append' for q in c[1]]
        self.assertEqual(flat,positions)
        self.assertEqual([len(c[1]) for c in motion.calls if c[0]=='append'],[100,100,53])

    def test_start_drift_rejected_before_buffer_or_motion(self):
        robot=Robot(q=[1.*RAD]*6)
        with self.assertRaisesRegex(ValueError,'Start differs'):
            runner.run_segment(robot,Stream(),segment(),{'operational_mode':'Automatic'},3.,1.,Log(),1)
        self.assertFalse(any(c[0] in ('move','alloc','stop') for c in robot.motion.calls))

    def test_uncertain_move_response_stops_once_without_retry_or_free(self):
        motion=Motion(fail=TimeoutError('lost reply')); robot=Robot(motion); clock=Clock()
        with patch.object(runner,'time',clock), patch.object(runner,'configuration_check'):
            with self.assertRaises(TimeoutError):
                runner.run_segment(robot,Stream(),segment(),{'operational_mode':'Automatic'},3.,1.,Log(),1)
        self.assertEqual(sum(c[0]=='move' for c in motion.calls),1)
        self.assertEqual(sum(c[0]=='stop' for c in motion.calls),1)
        self.assertFalse(any(c[0]=='free' for c in motion.calls))

    def test_feedback_failure_after_send_stops_and_raises(self):
        robot=Robot(); clock=Clock()
        with patch.object(runner,'time',clock), patch.object(runner,'configuration_check'):
            with self.assertRaisesRegex(ValueError,'stale'):
                runner.run_segment(robot,Stream(fail=True),segment(),{'operational_mode':'Automatic'},3.,1.,Log(),1)
        self.assertEqual(sum(c[0]=='move' for c in robot.motion.calls),1)
        self.assertEqual(sum(c[0]=='stop' for c in robot.motion.calls),1)

    def test_verified_completion_frees_buffer_without_stop(self):
        robot=Robot(); clock=Clock(); log=Log()
        def feedback():
            moved=any(c[0]=='move' for c in robot.motion.calls)
            return state([.01]*6 if moved else [0.]*6)
        with patch.object(runner,'time',clock), patch.object(runner,'configuration_check'), patch.object(robot,'snapshot',side_effect=feedback):
            runner.run_segment(robot,Stream(),segment(),{'operational_mode':'Automatic'},3.,1.,log,1)
        self.assertEqual(sum(c[0]=='move' for c in robot.motion.calls),1)
        self.assertEqual(sum(c[0]=='free' for c in robot.motion.calls),1)
        self.assertFalse(any(c[0]=='stop' for c in robot.motion.calls))
        self.assertTrue(any(e=='stage_complete' for e,_ in log.events))

    def test_cancellation_during_cleanup_is_not_swallowed(self):
        robot=Robot(q=[1.*RAD]*6)
        with patch.object(robot.motion,'pathBufferFree',side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                runner.run_segment(robot,Stream(),segment(),{'operational_mode':'Automatic'},3.,1.,Log(),1)
        self.assertFalse(any(c[0]=='move' for c in robot.motion.calls))

    def test_ledger_duplicate_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); manifest={'plan_id':'a'*32}
            runner.reserve_attempt(root,manifest,'full')
            with self.assertRaises(FileExistsError): runner.reserve_attempt(root,manifest,'full')

    def test_controller_and_rtde_faults_rejected(self):
        for key,value in [('runtime','Running'),('simulation',True),('queue',1),('force_control',True),('qd',[.1]*6)]:
            s=state(); s[key]=value
            with self.assertRaises(ValueError): runner.assert_state(s,idle=True)
        sample=dict(q=[0.]*6,qd=[0.]*6,target_q=[2.*RAD]*6,target_qd=[0.]*6,target_qdd=[0.]*6,
                    robot_mode='Running',safety_mode='Normal',runtime='Stopped')
        with self.assertRaisesRegex(ValueError,'tracking'):
            runner.check_stream([sample],([-1.]*6,[1.]*6),Log())

    def test_offline_does_not_import_or_connect_sdk(self):
        b=bundle();m={'captured_limits':[[-3.]*6,[3.]*6],'plan_id':b['plan_id'],**policy_manifest()}
        import builtins
        original=builtins.__import__
        def no_sdk(name,*args,**kw):
            if name in ('j6_45','telemetry','pyaubo_sdk'): raise AssertionError('Hardware imported in offline mode')
            return original(name,*args,**kw)
        with patch.object(runner,'load_package',return_value=(b,m)), patch('builtins.__import__',side_effect=no_sdk):
            self.assertEqual(runner.main(['offline']),0)

    def test_default_timing_preserves_original_duration_and_accepts_original_speed(self):
        b=bundle(); b['segments'][0]['points'][-1]['positions']=[.07]*6
        # Nominal peak ~7.52 deg/s exercises the former 3 deg/s refusal.
        m={'captured_limits':[[-3.]*6,[3.]*6],'plan_id':b['plan_id'],**policy_manifest()}; out=io.StringIO()
        with patch.object(runner,'load_package',return_value=(b,m)), redirect_stdout(out):
            self.assertEqual(runner.main(['offline']),0)
        report,_=json.JSONDecoder().raw_decode(out.getvalue())
        self.assertEqual(report['time_scale'],1.)
        self.assertEqual(report['motion_seconds_at_fraction_1'],report['nominal_duration_s'])
        self.assertGreater(report['peak_speed_deg_s'],3.)

    def test_faster_than_original_or_invalid_time_scales_rejected(self):
        for value in ['0','0.99','-1','nan','inf','10.1']:
            with self.subTest(value=value), patch.object(runner,'load_package') as load:
                with self.assertRaisesRegex(ValueError,'time-scale'):
                    runner.main(['offline','--time-scale',value])
                load.assert_not_called()

    def test_planned_speed_has_no_fixed_application_cutoff(self):
        b=bundle(); b['segments'][0]['points'][-1]['positions']=[.2]*6
        m={'captured_limits':[[-3.]*6,[3.]*6],'plan_id':b['plan_id'],**policy_manifest()}; out=io.StringIO()
        with patch.object(runner,'load_package',return_value=(b,m)), redirect_stdout(out):
            self.assertEqual(runner.main(['offline']),0)
        report,_=json.JSONDecoder().raw_decode(out.getvalue())
        self.assertGreater(report['peak_speed_deg_s'],9.)
        self.assertFalse(report['application_speed_thresholds'])

    def test_execute_has_no_text_confirmation_and_keeps_preflight(self):
        b=bundle();m={'plan_id':b['plan_id']};stream=Mock();log=Log()
        with patch.object(runner,'motion_lock',return_value=nullcontext()), \
                patch.object(runner,'inspect',return_value=(state(),{})) as inspect, \
                patch.object(runner,'configuration_check') as config, \
                patch.object(runner,'stationary') as stationary, \
                patch.object(runner,'reserve_attempt',return_value=Path('/fake/attempt')) as reserve, \
                patch.object(runner,'run_segment') as run, \
                patch('builtins.input',side_effect=AssertionError('Unexpected confirmation prompt')):
            runner.execute(Robot(),lambda:stream,b,m,1.,log)
            inspect.assert_called_once();config.assert_called_once();stationary.assert_called_once()
            reserve.assert_called_once();run.assert_called_once()
            self.assertEqual(run.call_args.args[4],1.)
        stream.close.assert_called_once()

    def test_original_timing_sent_to_path_buffer(self):
        robot=Robot();clock=Clock();log=Log()
        def feedback():
            moved=any(c[0]=='move' for c in robot.motion.calls)
            return state([.01]*6 if moved else [0.]*6)
        with patch.object(runner,'time',clock), patch.object(runner,'configuration_check'), patch.object(robot,'snapshot',side_effect=feedback):
            runner.run_segment(robot,Stream(),segment(),{'operational_mode':'Automatic'},1.,1.,log,1)
        count=next(c[3] for c in robot.motion.calls if c[0]=='alloc')
        dt=next(c[1] for c in robot.motion.calls if c[0]=='eval')
        self.assertAlmostEqual((count-1)*dt,segment()['points'][-1]['t'])


if __name__=='__main__': unittest.main()
