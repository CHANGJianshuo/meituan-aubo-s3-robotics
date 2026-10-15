"""Task-two UI tests using synthetic data; never connect to hardware."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from mtc_webui.hardware_backend import HardwareBackend, HardwareTransport
from mtc_webui.hardware_tasks import PHASES, describe
from mtc_webui.mock_backend import ApiError


class Transport:
    running=False
    stop_sent=False
    def __init__(self):self.calls=[]
    def enqueue(self,*args):self.calls.append(args)
    def stop_owned(self):self.calls.append('stop');return True
    def close(self):pass


class SequenceTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.package=self.root/'package';self.package.mkdir()
        points=[dict(t=t,positions=[0.]*6,velocities=[0.]*6,accelerations=[0.]*6) for t in [0,1]]
        self.bundle=dict(task='sequence',order='ABC',plan_id='sequence-fixture',
            assignments=[dict(object='battery_'+c,slot='P'+str(i+1)) for i,c in enumerate('ABC')],
            segments=[dict(object='battery_'+c,primitive=p,points=deepcopy(points)) for c in 'ABC' for p in PHASES])
        (self.package/'full_trajectory.json').write_text(json.dumps(self.bundle))
        self.manifest=dict(plan_id=self.bundle['plan_id'],uniform_tracking_guard_deg=.5,
            sha256={'full_trajectory.json':self.sha(self.package/'full_trajectory.json')})
        (self.package/'manifest.json').write_text(json.dumps(self.manifest))
        self.settings=dict(package=str(self.package),preview_dir=str(self.root/'preview'),
            claim_file=str(self.root/'claim.json'),preflight_file=str(self.root/'preflight.json'),
            manifest_sha256=self.sha(self.package/'manifest.json'),source_trajectory_sha256='fixture',
            placement_reference=str(self.root/'placement.png'))
        self.b=HardwareBackend(self.settings,self.root/'run');self.b.transport=Transport()
        self.b.scene_matches=lambda _:True
        self.b.preflight.write_text(json.dumps(dict(plan_id=self.bundle['plan_id'],passed=True,
            expires_unix=time.time()+60,manifest_sha256=self.settings['manifest_sha256'])))
        self.state=dict(q=[0.]*6,qd=[0.]*6,tcp=[0.]*6,exec_id=-1,queue=0,
            robot_mode='Running',safety_mode='Normal',power_on=True,within_limits=True,runtime='Stopped',
            **{name:False for name in ['simulation','freedrive','servo','collision','force_control','soft_float','backdrive']})
        self.b.stationary_since=self.b.clock()-2;self.b.observe_state(self.state,time.time())
        self.b.observe('color',{'fixture':True},'fixture',time.time_ns(),0)
        self.goal=dict(task_id='sequence-test',mode=2,ordered_colors=[1,3,2],timeout_ms=600000,placement_stability_sec=3)

    def tearDown(self):self.tmp.cleanup()
    @staticmethod
    def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

    def test_payload_is_bound_to_prepared_abc(self):
        for changes in [dict(mode=1,ordered_colors=[1]),dict(ordered_colors=[2,1,3]),dict(timeout_ms=120000)]:
            with self.assertRaises(ApiError):self.b.submit({**self.goal,**changes})
        self.assertFalse(self.b.transport.calls)
        config=self.b.snapshot()['config']
        self.assertEqual(config['supported_goal'],dict(mode=2,ordered_colors=[1,3,2]))
        self.assertEqual(config['stage_count'],30)

    def test_local_camera_remains_visible_without_robot_login_but_cannot_enable_motion(self):
        self.b.observe('color',{'fixture':True},'local-preview:/fixture',time.time_ns(),0)
        self.b.connection_update(connected=False,reason='SDK session unavailable')
        self.assertTrue(self.b._sample('color')['fresh'])
        self.assertFalse(self.b._sample('robot')['fresh'])
        self.assertIn('SDK session unavailable',self.b._reasons())
        self.b.observe('color',{'fixture':True},'local-preview:/fixture',time.time_ns(),4000)
        self.assertFalse(self.b._sample('color')['fresh'])

    def test_real_stage_progress_and_assignments_do_not_fabricate_placement(self):
        goal=self.b.submit(self.goal)['goal_id']
        for index,segment in enumerate(self.bundle['segments'],1):
            self.b.hardware_event(goal,dict(event='move_intent',stage=index,primitive=segment['primitive']))
            task=self.b.tasks[self.goal['task_id']]
            self.assertEqual(task['current_slot'],'P'+str((index-1)//10+1))
            self.assertEqual(task['current_color'],[1,3,2][(index-1)//10])
            self.b.hardware_event(goal,dict(event='stage_complete',stage=index))
        self.assertEqual(task['stages_completed'],30)
        self.assertEqual(task['completed_count'],0)
        self.b.hardware_event(goal,dict(event='completed',plan_id=self.bundle['plan_id']))
        self.b.hardware_event(goal,dict(event='player_returned'))
        self.assertTrue(self.b.player_returned)
        self.assertIsNone(task['result']);self.assertNotEqual(task['status'],'SUCCEEDED')

    def test_invalid_move_index_stops_owned_task(self):
        goal=self.b.submit(self.goal)['goal_id']
        self.b.hardware_event(goal,dict(event='move_intent',stage=31,primitive='retreat'))
        self.assertEqual(self.b.tasks[self.goal['task_id']]['status'],'UNKNOWN')
        self.assertEqual(self.b.transport.calls[-1],'stop')

    def test_wrong_primitive_stops_owned_task(self):
        goal=self.b.submit(self.goal)['goal_id']
        self.b.hardware_event(goal,dict(event='move_intent',stage=1,primitive='transport'))
        self.assertEqual(self.b.transport.calls[-1],'stop')

    def test_duplicate_stage_receipt_is_not_counted_twice(self):
        goal=self.b.submit(self.goal)['goal_id']
        self.b.hardware_event(goal,dict(event='stage_complete',stage=1))
        self.b.hardware_event(goal,dict(event='stage_complete',stage=1))
        self.assertEqual(self.b.finished_stages,[1]);self.assertEqual(self.b.transport.calls[-1],'stop')

    def test_partial_reordered_and_wrong_assignment_trajectories_rejected(self):
        for fault in ['partial','order','assignment']:
            bundle=deepcopy(self.bundle)
            if fault=='partial':bundle['segments'].pop()
            elif fault=='order':bundle['segments'][0],bundle['segments'][10]=bundle['segments'][10],bundle['segments'][0]
            else:bundle['assignments'][0]['slot']='P2'
            with self.assertRaises(ValueError):describe(bundle)

    def test_changed_package_is_rejected_before_ui_ready(self):
        (self.package/'full_trajectory.json').write_text('{}')
        with self.assertRaises(ValueError):HardwareBackend(self.settings,self.root/'changed')

    def test_task_two_rejects_relaxed_tracking_manifest(self):
        self.manifest['uniform_tracking_guard_deg']=.8
        (self.package/'manifest.json').write_text(json.dumps(self.manifest))
        self.settings['manifest_sha256']=self.sha(self.package/'manifest.json')
        with self.assertRaises(ValueError):HardwareBackend(self.settings,self.root/'relaxed')

    def test_requested_j4_variant_is_pinned_and_other_joint_changes_rejected(self):
        request=self.root/'tracking-request.json';request.write_text('{"user_requested":true}')
        self.manifest.pop('uniform_tracking_guard_deg')
        self.manifest.update(tracking_guard_policy='task2_j4_0p6_v1',tracking_guard_deg=[.5,.5,.5,.6,.5,.5],
            tracking_guard_request=dict(path=str(request),sha256=self.sha(request)))
        def backend(name):
            (self.package/'manifest.json').write_text(json.dumps(self.manifest))
            self.settings['manifest_sha256']=self.sha(self.package/'manifest.json')
            return HardwareBackend(self.settings,self.root/name)
        b=backend('j4');self.assertEqual(b.snapshot()['config']['tracking_guard_deg'],[.5,.5,.5,.6,.5,.5])
        self.manifest['tracking_guard_deg'][0]=.6
        with self.assertRaises(ValueError):backend('j1-change')
        self.manifest['tracking_guard_deg'][0]=.5
        request.write_text('changed')
        with self.assertRaises(ValueError):backend('changed-request')

    def test_requested_all_joint_policy_requires_exact_vector_and_unchanged_request(self):
        request=self.root/'tracking-request.json'
        request.write_text(json.dumps(dict(tracking_guard_deg=[.6]*6,user_submission_only=True)))
        self.manifest.pop('uniform_tracking_guard_deg')
        self.manifest.update(tracking_guard_policy='task2_all_0p6_v1',tracking_guard_deg=[.6]*6,
            tracking_guard_request=dict(path=str(request),sha256=self.sha(request)))
        def backend(name):
            (self.package/'manifest.json').write_text(json.dumps(self.manifest))
            self.settings['manifest_sha256']=self.sha(self.package/'manifest.json')
            return HardwareBackend(self.settings,self.root/name)
        self.assertEqual(backend('all-six').snapshot()['config']['tracking_guard_deg'],[.6]*6)
        for joint in range(6):
            self.manifest['tracking_guard_deg'][joint]=.7
            with self.assertRaises(ValueError):backend('changed-joint-'+str(joint))
            self.manifest['tracking_guard_deg'][joint]=.6
        request.write_text('changed')
        with self.assertRaises(ValueError):backend('changed-request')

    def test_monitor_only_requires_exact_request_and_reports_disabled_magnitude_stop(self):
        request=self.root/'monitor-request.json'
        request.write_text(json.dumps(dict(tracking_guard_deg=None,tracking_guard_action='record_only',user_submission_only=True)))
        self.manifest.pop('uniform_tracking_guard_deg')
        self.manifest.update(tracking_guard_policy='task2_tracking_monitor_v1',tracking_guard_deg=None,
            tracking_guard_action='record_only',tracking_guard_request=dict(path=str(request),sha256=self.sha(request)))
        def backend(name):
            (self.package/'manifest.json').write_text(json.dumps(self.manifest))
            self.settings['manifest_sha256']=self.sha(self.package/'manifest.json')
            return HardwareBackend(self.settings,self.root/name)
        config=backend('monitor').snapshot()['config']
        self.assertIsNone(config['tracking_guard_deg']);self.assertEqual(config['tracking_guard_action'],'record_only')
        self.manifest['tracking_guard_action']='stop'
        with self.assertRaises(ValueError):backend('wrong-action')
        self.manifest['tracking_guard_action']='record_only'
        request.write_text(json.dumps(dict(user_submission_only=True)))
        self.manifest['tracking_guard_request']['sha256']=self.sha(request)
        with self.assertRaises(ValueError):backend('missing-policy')

    def lime_green_scene(self, hue=40, saturation=70, value=190, shift=0):
        import cv2
        import numpy as np
        hsv=np.zeros((120,160,3),np.uint8)
        hsv[30:90,40+shift:100+shift]=(hue,saturation,value)
        return cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)

    def test_task_two_scene_accepts_lime_green_exposure_and_jpeg_variation(self):
        import cv2
        reference=self.root/'scene.png';cv2.imwrite(str(reference),self.lime_green_scene())
        evidence=dict(reference_image=str(reference),scene_roi=[0,0,160,120],checked_color_indices=[2])
        for hue,saturation,value in [(39,60,150),(41,90,210),(43,130,170)]:
            frame=self.lime_green_scene(hue,saturation,value)
            self.b.frames['color']=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,90])[1].tobytes()
            self.assertTrue(HardwareBackend.scene_matches(self.b,evidence))
        # Task-one thresholds are preserved, including their narrower green mask.
        self.b.profile['mode']=1
        self.assertFalse(HardwareBackend.scene_matches(self.b,evidence))

    def test_calibrated_green_still_rejects_translation_and_wrong_color(self):
        import cv2
        reference=self.root/'scene.png';cv2.imwrite(str(reference),self.lime_green_scene())
        evidence=dict(reference_image=str(reference),scene_roi=[0,0,160,120],checked_color_indices=[2])
        for frame in [self.lime_green_scene(shift=3),self.lime_green_scene(hue=30,saturation=190)]:
            self.b.frames['color']=cv2.imencode('.png',frame)[1].tobytes()
            self.assertFalse(HardwareBackend.scene_matches(self.b,evidence))

    def placement_fixture(self):
        import cv2
        import numpy as np
        frame=np.zeros((240,640,3),np.uint8);checks=[]
        for i,(letter,slot,index,color) in enumerate([
                ('A','P1',0,(0,0,255)),('B','P2',1,(0,255,255)),
                ('C','P3',3,(255,0,0)),('D','source',2,(0,255,0))]):
            x=40+i*140;frame[60:140,x:x+80]=color
            checks.append(dict(object=letter,slot=slot,color_index=index,roi=[x-10,50,x+100,150],minimum_pixels=2000))
        reference=Path(self.b.settings['placement_reference']);cv2.imwrite(str(reference),frame)
        self.b.settings.update(placement_checks=checks,placement_reference_sha256=self.sha(reference))
        self.b.frames['color']=cv2.imencode('.png',frame)[1].tobytes()
        self.b._read_preview=lambda *args:None
        stamp=[time.time_ns()]
        def sample(_):
            stamp[0]+=1000000000;return dict(fresh=True,source_stamp_ns=stamp[0])
        self.b._sample=sample
        return frame,HardwareTransport(self.b,{})

    def test_all_three_placements_and_green_require_stable_new_frames(self):
        _,transport=self.placement_fixture()
        with patch('time.sleep'):result=transport._verify_placement()
        self.assertTrue(result['passed']);self.assertGreaterEqual(result['stable_interval_s'],3)
        self.assertEqual(result['checked_objects'],list('ABCD'))

    def test_placement_uses_same_lime_green_calibration(self):
        import cv2
        import numpy as np
        frame,transport=self.placement_fixture()
        green=cv2.cvtColor(np.uint8([[[40,70,190]]]),cv2.COLOR_HSV2BGR)[0,0]
        frame[60:140,460:540]=green
        reference=Path(self.b.settings['placement_reference']);cv2.imwrite(str(reference),frame)
        self.b.settings['placement_reference_sha256']=self.sha(reference)
        frame[60:140,460:540]=cv2.cvtColor(np.uint8([[[39,65,160]]]),cv2.COLOR_HSV2BGR)[0,0]
        self.b.frames['color']=cv2.imencode('.png',frame)[1].tobytes()
        with patch('time.sleep'):self.assertTrue(transport._verify_placement()['passed'])

    def test_moved_green_prevents_success(self):
        import cv2
        frame,transport=self.placement_fixture();frame[60:140,460:540]=0;frame[60:140,464:544]=(0,255,0)
        self.b.frames['color']=cv2.imencode('.png',frame)[1].tobytes()
        with patch('time.sleep'):result=transport._verify_placement()
        self.assertFalse(result['passed']);self.assertIn('source',result['reason'])

    def test_missing_blue_prevents_success(self):
        import cv2
        frame,transport=self.placement_fixture();frame[60:140,320:400]=0
        self.b.frames['color']=cv2.imencode('.png',frame)[1].tobytes()
        with patch('time.sleep'):self.assertFalse(transport._verify_placement()['passed'])

    def test_missing_green_check_or_unpinned_reference_prevents_success(self):
        _,transport=self.placement_fixture()
        self.b.settings['placement_checks'].pop()
        self.assertFalse(transport._verify_placement()['passed'])
        self.placement_fixture();self.b.settings['placement_reference_sha256']='wrong'
        self.assertFalse(transport._verify_placement()['passed'])

    def test_repeated_camera_timestamp_does_not_prove_stability(self):
        _,transport=self.placement_fixture();stamp=time.time_ns()
        self.b._sample=lambda _:dict(fresh=True,source_stamp_ns=stamp)
        with patch('time.sleep'),patch('time.monotonic',side_effect=[0,1,2,11]):
            self.assertFalse(transport._verify_placement()['passed'])

    def test_repeated_submit_dispatches_once(self):
        self.b.submit(self.goal)
        with self.assertRaises(ApiError):self.b.submit(self.goal)
        self.assertEqual(len(self.b.transport.calls),1)


if __name__=='__main__':unittest.main()
