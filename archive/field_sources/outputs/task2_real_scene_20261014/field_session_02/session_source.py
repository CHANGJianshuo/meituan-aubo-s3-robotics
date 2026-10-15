#!/usr/bin/env python3
"""One-login field session for individually requested, hash-pinned stages.

The unchanged task-one remote bridge owns motion and its heartbeat watchdog.
This terminal holds SDK credentials in memory. It never chains stages or retries.
Create stop.request in the private session directory to stop the owned stage.
"""
import argparse
from datetime import datetime, timezone
import getpass
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import threading
import time


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_request(request, entry, root, now=None):
    now = time.time() if now is None else now
    if request.get('command') != 'execute' or request.get('package') != entry['directory']:
        raise ValueError('Only the next single package may be requested')
    age = now - request.get('created_unix', 0)
    if not 0 <= age <= 30 or not request.get('request_id'):
        raise ValueError('A fresh uniquely identified request is required')
    evidence = request.get('reviewed_evidence', {})
    if not evidence:
        raise ValueError('Reviewed field evidence is required for every stage')
    for name, digest in evidence.items():
        path = (root / name).resolve()
        if not path.is_relative_to(root.resolve()) or not path.is_file() or sha(path) != digest:
            raise ValueError('Evidence is missing, outside this task, or changed')
    package = root / 'commissioning' / entry['directory']
    if sha(package / 'manifest.json') != entry['manifest_sha256']:
        raise ValueError('Reviewed package manifest changed')


def verified_checkpoint(root, packages, path):
    """Resume only between completed stages, without replacing any plan ID."""
    data = json.loads(path.read_text())
    if data.get('status') != 'VERIFIED_STOPPED_BETWEEN_STAGES':
        raise ValueError('A separately verified stationary checkpoint is required')

    def evidence(item):
        p = (root/item['path']).resolve()
        if not p.is_relative_to(root.resolve()) or sha(p) != item['sha256']:
            raise ValueError('Checkpoint evidence changed or is outside this task')
        return p

    completed = data['completed_packages']
    if not 0 < len(completed) < len(packages):
        raise ValueError('Checkpoint must identify a completed prefix and an unused next package')
    checked = []
    for index, item in enumerate(completed):
        entry = packages[index]
        if item['package'] != entry['directory']:
            raise ValueError('Checkpoint skips or reorders a stage')
        log = evidence(item['receipt'])
        rows = [json.loads(s) for s in log.read_text().splitlines()]
        expected = entry['source_stages'][1]-entry['source_stages'][0]+1
        endings = [r for r in rows if r['event'] == 'stage_complete']
        responses = [r for r in rows if r['event'] == 'move_response']
        if ([r['stage'] for r in endings] != list(range(1,expected+1))
                or [r['stage'] for r in responses] != list(range(1,expected+1))
                or any(r['result'] != 0 for r in responses)
                or not any(r['event'] == 'completed' and r.get('plan_id') == entry['plan_id'] for r in rows)):
            raise ValueError('Checkpoint lacks all successful segment and package receipts')
        final_q = endings[-1]['final_q']
        if len(final_q) != 6 or not all(math.isfinite(q) for q in final_q):
            raise ValueError('Invalid completed joint feedback')
        error = max(abs(a-b) for a,b in zip(final_q,entry['end_q']))
        if error > math.radians(.05):
            raise ValueError('Completed receipt is not at the saved endpoint')
        checked.append(dict(package=entry['directory'],log=str(log),checkpoint_verified=True))
    if data['next_package'] != packages[len(completed)]['directory']:
        raise ValueError('Checkpoint next package is not the unused successor')
    record = json.loads(evidence(data['stationary_capture_record']).read_text())
    if not record.get('read_only') or not record.get('telemetry'):
        raise ValueError('No independent stationary capture')
    expected = packages[len(completed)]['start_q']
    for sample in record['telemetry']:
        robot = sample['robot']; state = robot['data']
        if (not robot['fresh'] or not state['stop_confirmed'] or state['motion_active']
                or state['queue'] != 0 or state['exec_id'] != -1
                or len(state['q']) != 6 or len(state['qd']) != 6
                or not all(math.isfinite(x) for x in state['q']+state['qd'])
                or max(map(abs,state['qd'])) > math.radians(.02)
                or max(abs(a-b) for a,b in zip(state['q'],expected)) > math.radians(.05)):
            raise ValueError('Checkpoint robot is not stationary at the next saved start')
    for item in data['reviewed_evidence']:
        evidence(item)
    if not data['reviewed_evidence']:
        raise ValueError('Checkpoint needs field observation and remote reconciliation evidence')
    return checked


def run_stage(root, folder, deployment, entry, credentials, stop, status):
    remote = deployment['remote_directory']
    bridge = root / 'commissioning/remote_bridge.py'
    bridge_digest = deployment['sha256']['remote_bridge.py']
    if sha(bridge) != bridge_digest:
        raise ValueError('Remote bridge source changed')
    python = '/home/robot/miniforge3/envs/aubo-s3-hardware/bin/python'
    command = [python, remote + '/remote_bridge.py', 'execute', '--root',
               remote + '/' + entry['directory'], '--manifest-sha256', entry['manifest_sha256']]
    pinned = ('from pathlib import Path; import hashlib,os; '
              f'p=Path({remote + "/remote_bridge.py"!r}); '
              f'assert hashlib.sha256(p.read_bytes()).hexdigest()=={bridge_digest!r}, "Bridge changed"; '
              f'os.execv({python!r},{command!r})')
    argv = ['ssh', '-T', '-o', 'ControlPath=/tmp/mtc_task2_ssh_20261014_2242/control',
            *deployment['transport_options'], deployment['ssh_host'], shlex.join([python, '-c', pinned])]
    path = folder / (entry['directory'] + '.events.jsonl')
    finished = threading.Event()
    child = None
    completed = False
    returned = False
    with path.open('x') as log, (folder / (entry['directory'] + '.stderr.txt')).open('x') as errors:
        try:
            child = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=errors, text=True, bufsize=1)
            child.stdin.write(json.dumps(credentials) + '\n')
            child.stdin.flush()

            def heartbeat():
                while not finished.wait(.5):
                    message = dict(command='ping', token=time.monotonic())
                    if stop.is_set() or (folder / 'stop.request').exists():
                        stop.set()
                        message = dict(command='stop')
                    try:
                        child.stdin.write(json.dumps(message) + '\n')
                        child.stdin.flush()
                    except (BrokenPipeError, OSError, ValueError):
                        return
                    if message['command'] == 'stop':
                        return

            worker = threading.Thread(target=heartbeat, daemon=True)
            worker.start()
            for line in child.stdout:
                if not line.startswith('MTC_HARDWARE '):
                    continue
                event = json.loads(line[len('MTC_HARDWARE '):])
                log.write(json.dumps(event, ensure_ascii=False) + '\n')
                log.flush()
                kind = event['event']
                if kind in ('attempt_reserved', 'buffer_ready', 'segment_begin', 'segment_end',
                            'completed', 'bridge_error', 'owned_stop_requested'):
                    status('executing', package=entry['directory'], last_event=event)
                    print(entry['directory'], kind, flush=True)
                completed |= kind == 'completed' and event.get('plan_id') == entry['plan_id']
                returned |= kind == 'player_returned'
            code = child.wait(timeout=10)
            if code != 0 or not completed or not returned or stop.is_set():
                raise RuntimeError(f'Stage did not complete; returncode={code}, completed={completed}. No retry.')
        finally:
            finished.set()
            if child is not None:
                # Closing the owned input triggers the unchanged remote stop watchdog.
                child.stdin.close()
                if child.poll() is None:
                    try:
                        child.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        child.terminate()
                        child.wait(timeout=10)
    return str(path)


def session(root, session_name='field_session', resume_checkpoint=None):
    os.umask(0o077)
    deployment = json.loads((root / 'deployment.json').read_text())
    if deployment['status'] != 'deployed_and_offline_verified':
        raise ValueError('Deployment has not passed offline checks')
    packages = json.loads((root / 'commissioning/commissioning.json').read_text())['packages']
    if '/' in session_name or session_name in ('', '.', '..'):
        raise ValueError('Use a simple new session directory name')
    prior = verified_checkpoint(root, packages, resume_checkpoint) if resume_checkpoint else []
    folder = root / session_name
    folder.mkdir(mode=0o700, exist_ok=False)
    state = dict(pid=os.getpid(), completed_packages=prior, automatic_sequencing=False,
                 started_utc=datetime.now(timezone.utc).isoformat())
    if resume_checkpoint:
        state['resume_checkpoint'] = dict(path=str(resume_checkpoint),sha256=sha(resume_checkpoint))

    def status(value, **fields):
        state.update(status=value, updated_utc=datetime.now(timezone.utc).isoformat(), **fields)
        temporary = folder / 'status.tmp'
        temporary.write_text(json.dumps(state, indent=2) + '\n')
        temporary.replace(folder / 'status.json')

    stop = threading.Event()
    credentials = None
    try:
        status('awaiting_login')
        print('任务二分段调试：红→P1，黄→P2，蓝→P3；舌规朝向固定。', flush=True)
        print('每次仅执行一个已核对的分段请求，结束后停稳；不会自动串联。', flush=True)
        print('Ctrl+C 停止本会话拥有的动作。SDK 密码仅保存在本进程内存。', flush=True)
        username = input('SDK username [aubo]: ').strip() or 'aubo'
        credentials = dict(username=username, password=getpass.getpass('SDK password (not saved): '))
        signal.signal(signal.SIGINT, lambda *_: stop.set())
        signal.signal(signal.SIGTERM, lambda *_: stop.set())
        for entry in packages[len(prior):]:
            status('awaiting_stage_request', package=entry['directory'])
            print('等待核对并启动：', entry['directory'], flush=True)
            request_path = folder / 'request.json'
            while not request_path.exists() and not stop.wait(.2):
                if (folder / 'stop.request').exists():
                    stop.set()
            if stop.is_set():
                status('stopped')
                return
            request = json.loads(request_path.read_text())
            request_path.unlink()
            validate_request(request, entry, root)
            (folder / (entry['directory'] + '.request.json')).write_text(json.dumps(request, indent=2) + '\n')
            status('starting_stage', package=entry['directory'])
            log = run_stage(root, folder, deployment, entry, credentials, stop, status)
            state['completed_packages'].append(dict(package=entry['directory'], log=log))
            print('此段完成并停稳；等待现场核对。', flush=True)
        status('all_packages_completed_pending_placement_verification')
    except BaseException as exc:
        status('failed_no_retry', error=type(exc).__name__ + ': ' + str(exc))
        raise
    finally:
        if credentials is not None:
            credentials.clear()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--session-name', default='field_session')
    parser.add_argument('--resume-checkpoint', type=Path)
    args = parser.parse_args()
    try:
        session(args.root.resolve(), args.session_name, args.resume_checkpoint)
    except Exception as error:
        print(type(error).__name__ + ': ' + str(error), flush=True)
    input('按 Enter 关闭此终端。')
