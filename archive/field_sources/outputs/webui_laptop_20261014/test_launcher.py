"""Offline evidence tests; no processes, SSH or hardware are started."""
import copy,hashlib,json,tempfile,unittest
from pathlib import Path
from start import completed_motion_is_stopped

class CompletedMotionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        def write(name,data):
            raw=json.dumps(data).encode();(self.root/name).write_bytes(raw);return hashlib.sha256(raw).hexdigest()
        self.write=write
        digest=write('full_trajectory.json',dict(plan_id='p',segments=[dict(points=[dict(positions=[0.]*6)])]))
        digest=write('manifest.json',dict(plan_id='p',sha256={'full_trajectory.json':digest}))
        write('hardware-config.json',dict(package=str(self.root),manifest_sha256=digest))
        events=[dict(event='stage_complete',stage=1),dict(event='completed',plan_id='p'),dict(event='player_returned')]
        (self.root/'hardware-events.jsonl').write_text('\n'.join(json.dumps(e) for e in events))
        write('player-exit.json',dict(returncode=0))
        write('state-after.json',dict(independent_feedback=True,completed_stages=[1]))
        write('placement-visual.json',dict(passed=False))
        self.record=dict(run_dir=str(self.root))
        self.current=dict(task=dict(status='UNKNOWN',substate='placement_camera',stages_completed=1),
            config=dict(plan_id='p'),operations=dict(stop=False,cancel=False),
            robot=dict(fresh=True,data=dict(stop_confirmed=True,robot_ready=True,motion_active=False,exec_id=-1,
                queue=0,q=[0.]*6,qd=[0.]*6)))
    def tearDown(self):self.tmp.cleanup()
    def test_only_verified_motion_completion_can_be_archived_without_result_mutation(self):
        before=copy.deepcopy(self.current)
        self.assertTrue(completed_motion_is_stopped(self.record,self.current))
        self.assertEqual(before,self.current)
    def test_stale_or_active_or_displaced_robot_rejected(self):
        for field,value in [('fresh',False),('stop_confirmed',False),('motion_active',True),
                            ('exec_id',3),('queue',1),('q',[.01]*6),('qd',[.01]*6),('q',[float('nan')]*6)]:
            c=copy.deepcopy(self.current)
            (c['robot'] if field=='fresh' else c['robot']['data'])[field]=value
            self.assertFalse(completed_motion_is_stopped(self.record,c))
    def test_active_task_and_owned_execution_rejected(self):
        for status in ('EXECUTING','CANCELING','SUBMITTING'):
            c=copy.deepcopy(self.current);c['task']['status']=status
            self.assertFalse(completed_motion_is_stopped(self.record,c))
        c=copy.deepcopy(self.current);c['operations']['stop']=True
        self.assertFalse(completed_motion_is_stopped(self.record,c))
    def test_incomplete_receipts_and_failed_player_rejected(self):
        self.write('player-exit.json',dict(returncode=1))
        self.assertFalse(completed_motion_is_stopped(self.record,self.current))
        self.write('player-exit.json',dict(returncode=0))
        (self.root/'hardware-events.jsonl').write_text(json.dumps(dict(event='stage_complete',stage=1)))
        self.assertFalse(completed_motion_is_stopped(self.record,self.current))
    def test_wrong_plan_or_changed_package_rejected(self):
        c=copy.deepcopy(self.current);c['config']['plan_id']='other'
        self.assertFalse(completed_motion_is_stopped(self.record,c))
        (self.root/'full_trajectory.json').write_text('{}')
        self.assertFalse(completed_motion_is_stopped(self.record,self.current))

if __name__=='__main__':unittest.main()
