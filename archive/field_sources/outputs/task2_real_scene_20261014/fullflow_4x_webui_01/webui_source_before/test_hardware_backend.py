"""Offline tests only: no SSH, controller, motion, or camera USB."""
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
from mtc_webui.hardware_backend import HardwareBackend, HardwareTransport
from mtc_webui.mock_backend import ApiError

ROOT=Path(__file__).resolve().parents[4]/'hardware_integration'

class FakeTransport:
    running=False
    def __init__(self):self.calls=[]
    def enqueue(self,*args):self.calls.append(args)
    def close(self):pass
    def stop_owned(self):self.calls.append('stop');return True

class HardwareTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        settings=json.loads((ROOT/'hardware_config.json').read_text())
        settings.update(claim_file=str(self.root/'claim.json'),preflight_file=str(self.root/'preflight.json'))
        self.b=HardwareBackend(settings,self.root/'run');self.b.transport=FakeTransport()
        self.b.scene_matches=lambda evidence: True
        self.goal=dict(task_id='test-red',mode=1,ordered_colors=[1],timeout_ms=180000,placement_stability_sec=3)
        self.evidence=dict(plan_id=self.b.manifest['plan_id'],passed=True,expires_unix=time.time()+100,
            manifest_sha256=settings['manifest_sha256'])
        self.b.preflight.write_text(json.dumps(self.evidence))
        self.state=json.loads((ROOT/'initial_robot_state.json').read_text())['samples'][-1]
        self.state['q']=self.b.bundle['segments'][0]['points'][0]['positions'].copy()
        self.b.stationary_since=self.b.clock()-2
        self.b.observe_state(self.state,time.time())
        self.b._read_preview('color','latest_color.jpg')
        # Deliberately use synthetic time-stamped image bytes in tests; no actual
        # scene-validation bypass in production.
        self.b.observe('color',{'test':True},'fixture',time.time_ns(),0)
    def tearDown(self):self.tmp.cleanup()
    def test_only_red_basic(self):
        for changes in [dict(ordered_colors=[2]),dict(mode=2,ordered_colors=[1,2,3]),dict(timeout_ms=1000)]:
            with self.assertRaises(ApiError):self.b.submit({**self.goal,**changes})
        self.assertFalse(self.b.transport.calls)
    def test_preflight_required_and_bound_to_plan(self):
        self.evidence['plan_id']='wrong';self.b.preflight.write_text(json.dumps(self.evidence))
        with self.assertRaises(ApiError):self.b.submit(self.goal)
    def test_changed_scene_blocks(self):
        self.b.scene_matches=lambda evidence: False
        with self.assertRaises(ApiError):self.b.submit(self.goal)
    def scene_fixture(self):
        import cv2
        import numpy as np
        frame=np.zeros((180,300,3),dtype=np.uint8)
        frame[20:50,20:50]=(0,0,255)
        frame[60:90,20:50]=(255,0,0)
        frame[30:60,160:190]=(0,255,255)
        frame[30:60,205:235]=(0,255,0)
        reference=self.root/'scene.png';cv2.imwrite(str(reference),frame)
        evidence=dict(reference_image=str(reference),scene_checks=[
            dict(scene_roi=[10,10,80,100],checked_color_indices=[0,3]),
            dict(scene_roi=[150,20,250,100],checked_color_indices=[1,2])])
        return frame,evidence
    def scene_result(self,frame,evidence):
        import cv2
        self.b.frames['color']=cv2.imencode('.png',frame)[1].tobytes()
        return HardwareBackend.scene_matches(self.b,evidence)
    def test_scene_regions_ignore_background_light_but_check_every_object(self):
        frame,evidence=self.scene_fixture()
        frame[130:140,:]=(0,0,255)
        self.assertTrue(self.scene_result(frame,evidence))
        for y0,y1,x0,x1 in [(20,50,20,50),(60,90,20,50),(30,60,160,190),(30,60,205,235)]:
            moved=frame.copy();color=frame[y0,x0].copy()
            moved[y0:y1,x0:x1]=0;moved[y0:y1,x0+4:x1+4]=color
            self.assertFalse(self.scene_result(moved,evidence))
    def test_scene_regions_missing_object_is_rejected(self):
        frame,evidence=self.scene_fixture();frame[30:60,205:235]=0
        self.assertFalse(self.scene_result(frame,evidence))
    def test_scene_regions_empty_or_invalid_is_rejected(self):
        frame,evidence=self.scene_fixture()
        for checks in [[],[dict(scene_roi=[-1,0,80,100],checked_color_indices=[0])],
                       [dict(scene_roi=[10,10,80,100],checked_color_indices=[])],
                       [dict(scene_roi=[10,10,80,100],checked_color_indices=[4])]]:
            self.assertFalse(self.scene_result(frame,{**evidence,'scene_checks':checks}))
    def test_legacy_single_scene_region_still_checks_all_colors(self):
        frame,evidence=self.scene_fixture()
        evidence=dict(reference_image=evidence['reference_image'],scene_roi=[0,0,300,180])
        self.assertTrue(self.scene_result(frame,evidence))
        frame[30:60,205:235]=0
        self.assertFalse(self.scene_result(frame,evidence))
    def test_start_mismatch_blocks(self):
        self.state['q'][0]+=.01;self.b.observe_state(self.state,time.time())
        with self.assertRaises(ApiError):self.b.submit(self.goal)
    def test_actual_lifecycle_no_success_from_completed_alone(self):
        response=self.b.submit(self.goal);g=response['goal_id']
        self.assertEqual(self.b.tasks['test-red']['status'],'SUBMITTING')
        self.b.hardware_event(g,dict(event='attempt_reserved'))
        self.assertEqual(self.b.tasks['test-red']['status'],'ACCEPTED')
        self.b.hardware_event(g,dict(event='move_intent',stage=1,primitive='side_insert'))
        self.assertEqual(self.b.tasks['test-red']['status'],'EXECUTING')
        self.b.hardware_event(g,dict(event='completed',plan_id=self.b.manifest['plan_id']))
        self.assertNotEqual(self.b.tasks['test-red']['status'],'SUCCEEDED')
        self.assertIsNone(self.b.tasks['test-red']['result'])
    def test_foreign_feedback_ignored(self):
        self.b.submit(self.goal)
        self.b.hardware_event('foreign',dict(event='stage_complete',stage=1))
        self.assertEqual(self.b.finished_stages,[])
    def test_player_error_is_preserved_for_the_real_task(self):
        response=self.b.submit(self.goal)
        error='ValueError: Robot did not remain stationary for 1 s within 5 s'
        self.b.hardware_event('foreign',dict(event='bridge_error',error='foreign error'))
        self.assertNotIn('execution_error',self.b.tasks['test-red'])
        self.b.hardware_event(response['goal_id'],dict(event='bridge_error',error=error))
        self.assertEqual(self.b.tasks['test-red']['execution_error'],error)
        self.assertIsNone(self.b.tasks['test-red']['result'])
    def test_out_of_order_stops_and_unknown(self):
        r=self.b.submit(self.goal)
        self.b.hardware_event(r['goal_id'],dict(event='stage_complete',stage=2))
        self.assertEqual(self.b.tasks['test-red']['status'],'UNKNOWN')
        self.assertIn('stop',self.b.transport.calls)
    def test_persistent_claim_survives_new_backend(self):
        self.b.claim.write_text('{}')
        b2=HardwareBackend(self.b.settings,self.root/'run2')
        self.assertIn('本次单次执行凭据已使用；不会自动重放',b2._reasons())
    def test_duplicate_submit_never_dispatches_twice(self):
        self.b.submit(self.goal)
        with self.assertRaises(ApiError):self.b.submit(self.goal)
        self.assertEqual(len(self.b.transport.calls),1)
    def test_cancel_foreign_goal_denied(self):
        self.b.submit(self.goal);self.b.transport.running=True
        with self.assertRaises(ApiError):self.b.cancel('test-red','foreign')
    def test_expired_telemetry_blocks(self):
        self.b.samples['robot']['_received']-=10
        with self.assertRaises(ApiError):self.b.submit(self.goal)
    def test_placement_requires_new_frames_stability_and_reference(self):
        import cv2
        frame=cv2.imread(self.b.settings['placement_reference'])
        jpeg=cv2.imencode('.jpg',frame)[1].tobytes()
        self.b.frames['color']=jpeg
        self.b._read_preview=lambda *args: None
        stamp=[time.time_ns()]
        def sample(kind):
            stamp[0]+=1000000000
            return dict(fresh=True,source_stamp_ns=stamp[0])
        self.b._sample=sample
        t=HardwareTransport(self.b,{})
        with patch('time.sleep'):
            result=t._verify_placement()
        self.assertTrue(result['passed']);self.assertGreaterEqual(result['stable_interval_s'],3)
        self.b.frames['color']=cv2.imencode('.jpg',frame*0)[1].tobytes()
        self.assertFalse(t._verify_placement()['passed'])
    def test_monotonic_heartbeat_handles_remote_clock_offset(self):
        self.b.observe_state(self.state,time.time()+120,source_age_ms=300)
        self.assertTrue(self.b.connection['connected'])
        self.b.observe_state(self.state,time.time()+121,source_age_ms=4000)
        self.assertFalse(self.b.connection['connected'])
    def test_no_credentials_starts_no_process(self):
        t=HardwareTransport(self.b,{})
        with patch('subprocess.Popen',side_effect=AssertionError('must not spawn')):
            t.start()
        self.assertFalse(self.b.connection['connected'])

if __name__=='__main__':unittest.main()
