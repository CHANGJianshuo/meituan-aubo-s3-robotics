"""Isolated C++ Mock → ROS → HTTP acceptance. Starts no hardware or fake Action server.

Run after building/sourcing, in the documented constrained container only.
Retains every case's logs, parent/child status and feedback, and cleanup evidence.
"""
import argparse
from datetime import datetime, timezone
from http.client import HTTPConnection, HTTPException
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid
import rclpy
from rclpy.action import ActionClient
from unique_identifier_msgs.msg import UUID
from mtc_interfaces.action import ExecuteTask
from mtc_cpp_mock_bringup.client import Recorder
from mtc_cpp_mock_bringup.check import await_ready


def http(port, method, path, body=None):
    conn = HTTPConnection('127.0.0.1', port, timeout=3)
    try:
        conn.request(method, path, body=json.dumps(body) if body is not None else None,
                     headers={'Content-Type':'application/json'} if body is not None else {})
        response = conn.getresponse()
        return response.status, json.loads(response.read())
    finally:
        conn.close()


def until(function, timeout=25):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = function()
        if result:
            return result
        time.sleep(.03)
    raise TimeoutError('condition not reached within ' + str(timeout) + 's')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--cases', nargs='+', help='Run only named cases, preserving earlier evidence')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=[])
    results = []
    cases = [('basic','success',False,False,4,0,1),
             ('sequence','success',False,True,4,0,3),
             ('cancel','success',True,False,5,0,0),
             ('planning_failure','planning_failure',False,False,6,200,0),
             ('cancel_unconfirmed','cancel_unconfirmed',True,False,6,520,0),
             ('status_unknown','status_unknown',False,False,6,230,0)]
    if args.cases:
        cases = [case for case in cases if case[0] in args.cases]
        if len(cases) != len(set(args.cases)):
            parser.error('unknown case name')
    for label, profile, cancel, sequence, expected_status, expected_error, count in cases:
        folder = args.output_dir / label
        folder.mkdir()
        ns = '/cpp_accept_' + label
        case = dict(case=label, profile=profile, started_at=datetime.now(timezone.utc).isoformat())
        logs = (folder/'supervisor.log').open('wb')
        argv = [sys.executable, '-m', 'mtc_cpp_mock_bringup.bringup', '--namespace', ns,
                '--output-dir', str(folder/'stack'), '--port',str(args.port),
                '--profile', profile, '--result-delay-ms','350']
        proc = subprocess.Popen(argv, stdout=logs, stderr=subprocess.STDOUT, start_new_session=True)
        recorder = Recorder(ns, folder/'observations')
        done = threading.Event()
        def spin():
            while not done.is_set():
                rclpy.spin_once(recorder, timeout_sec=.02)
        thread = threading.Thread(target=spin)
        thread.start()
        direct = ActionClient(recorder, ExecuteTask, ns+'/task/execute')
        try:
            def ready():
                if proc.poll() is not None:
                    raise RuntimeError('stack exited ' + str(proc.returncode))
                try:
                    code, value = http(args.port,'GET','/api/snapshot')
                    return value if code == 200 and value['config']['cpp_namespace'] == ns and value['operations']['submit'] else None
                except (OSError, HTTPException):
                    return None
            snapshot = until(ready,30)
            (folder/'ready-http.json').write_text(json.dumps(snapshot,indent=2)+'\n')
            until(recorder.ready,10)
            recorder.event('observer_endpoints_matched')
            assert snapshot['mode']=='cpp_mock' and not snapshot['robot']['available'] and not snapshot['tool']['available']
            graph = snapshot['connection']['graph']
            assert graph['valid']
            if label=='basic':
                assert direct.wait_for_server(timeout_sec=5), 'Diagnostic client not discovered'
                # ROS rejects invalid Goals at the actual C++ server, not only UI validation.
                invalid = ExecuteTask.Goal(task_id='invalid_color',mode=1,ordered_colors=[0],timeout_ms=30000,placement_stability_sec=3.)
                ident=uuid.uuid4()
                recorder.event('invalid_goal_sent',goal_id=ident.hex)
                future=direct.send_goal_async(invalid,goal_uuid=UUID(uuid=list(ident.bytes)))
                until(future.done,5)
                case['invalid_ros_goal_rejected']=not future.result().accepted
                assert case['invalid_ros_goal_rejected']
                # A second supervisor must refuse the same ownership lock.
                duplicate=subprocess.run([sys.executable,'-m','mtc_cpp_mock_bringup.bringup','--namespace',ns,
                                          '--output-dir',str(folder/'duplicate-stack'),'--no-web'],capture_output=True,text=True,timeout=5)
                (folder/'duplicate-launch.log').write_text(duplicate.stdout+duplicate.stderr)
                case['duplicate_launch_rejected']=duplicate.returncode!=0
                assert case['duplicate_launch_rejected']
            taskid='http_'+label+'_'+uuid.uuid4().hex[:8]
            payload=dict(task_id=taskid,mode=2 if sequence else 1,ordered_colors=[2,1,3] if sequence else [1],
                         timeout_ms=30000,placement_stability_sec=3)
            code,reply=http(args.port,'POST','/api/tasks',payload)
            recorder.event('http_submit',http_status=code,reply=reply,goal=payload)
            assert code==202,reply
            goalid=reply['goal_id']; case['goal_id']=goalid
            if label=='basic':
                until(lambda: http(args.port,'GET','/api/tasks/'+taskid)[1]['status'] in ('ACCEPTED','EXECUTING'), 5)
                # Check HTTP ownership and independent ROS admission for a concurrent task.
                code,conflict=http(args.port,'POST','/api/tasks',{**payload,'task_id':taskid+'_second'})
                recorder.event('http_concurrent_goal',http_status=code,reply=conflict)
                assert code==409
                invalid.task_id='concurrent_task'; invalid.ordered_colors=[2]
                future=direct.send_goal_async(invalid)
                until(future.done,5)
                case['concurrent_ros_goal_rejected']=not future.result().accepted
                assert case['concurrent_ros_goal_rejected']
            if cancel:
                def moving():
                    value=http(args.port,'GET','/api/snapshot')[1]
                    task=value.get('task') or {}
                    return value if task.get('substate')=='MOVE_PRE_ALIGN' and value['operations']['cancel'] else None
                before=until(moving,10)
                (folder/'before-cancel.json').write_text(json.dumps(before,indent=2)+'\n')
                code,reply=http(args.port,'POST','/api/tasks/'+taskid+'/cancel',dict(goal_id=goalid))
                recorder.event('http_cancel',http_status=code,reply=reply)
                assert code==202
            def finished():
                record=http(args.port,'GET','/api/tasks/'+taskid)[1]
                return record if record['status'] in ('SUCCEEDED','CANCELED','ABORTED','REJECTED') else None
            task=until(finished,40)
            (folder/'http-result.json').write_text(json.dumps(task,ensure_ascii=False,indent=2)+'\n')
            status_map={'SUCCEEDED':4,'CANCELED':5,'ABORTED':6,'REJECTED':0}
            assert status_map[task['status']]==expected_status,task
            assert task['result']['error_code']==expected_error and task['completed_count']==count,task
            assert task['result']['success']==(expected_status==4),task
            assert any(e['kind']=='FEEDBACK' for e in task['events']),task
            if cancel:
                assert task['cancel_ack']['request_delivered'] is True,task
                case['cancel_ack']=task['cancel_ack']
            time.sleep(.3)
            case.update(status=task['status'],error_code=task['result']['error_code'],completed_count=count,
                        feedback_events=sum(e['kind']=='FEEDBACK' for e in task['events']))
            child_goals={e['goal_id'] for e in recorder.events if e['kind']=='status' and e['endpoint'].endswith('/manipulation/pick_place')}
            assert child_goals,'No actual downstream C++ Goal'
            case['child_goal_ids']=sorted(child_goals)
            statuses={gid: sorted({e['status'] for e in recorder.events if e['kind']=='status' and e['goal_id']==gid}) for gid in child_goals}
            case['child_statuses']=statuses
            if cancel:
                assert all(expected_status in status for status in statuses.values()),statuses
                case['cancel_not_treated_as_result']=True
            if expected_error in (230,520):
                after=http(args.port,'GET','/api/snapshot')[1]
                assert not after['operations']['submit'],after
                case['fault_blocks_next_goal']=True
            case['ok']=True
        except Exception as error:
            case.update(ok=False,error=repr(error))
        finally:
            direct.destroy()
            done.set();thread.join(timeout=3)
            recorder.finish()
            proc.send_signal(signal.SIGINT) if proc.poll() is None else None
            try:
                proc.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);proc.wait();case['forced_supervisor_kill']=True
                case['ok']=False
            logs.close()
            case['stack_exit_code']=proc.returncode
            if proc.returncode!=0:case['ok']=False
            cleanup=folder/'stack/cleanup.json'
            if cleanup.is_file():
                case['cleanup']=json.loads(cleanup.read_text())
                if not case['cleanup']['cleanup_ok']:case['ok']=False
            else:case['ok']=False
            case['ended_at']=datetime.now(timezone.utc).isoformat()
            (folder/'case.json').write_text(json.dumps(case,ensure_ascii=False,indent=2)+'\n')
            results.append(case)
            (args.output_dir/'summary.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
            print(label,case.get('ok'),case.get('status'),case.get('error'),flush=True)
            if case.get('forced_supervisor_kill') or not case.get('cleanup',{}).get('cleanup_ok'):
                break  # Never overlap a failed cleanup with the next case.
    rclpy.shutdown()
    raise SystemExit(0 if len(results)==len(cases) and all(c['ok'] for c in results) else 1)

if __name__=='__main__':main()
