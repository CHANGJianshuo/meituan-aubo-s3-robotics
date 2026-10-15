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


def session(root):
    os.umask(0o077)
    deployment = json.loads((root / 'deployment.json').read_text())
    if deployment['status'] != 'deployed_and_offline_verified':
        raise ValueError('Deployment has not passed offline checks')
    packages = json.loads((root / 'commissioning/commissioning.json').read_text())['packages']
    folder = root / 'field_session'
    folder.mkdir(mode=0o700, exist_ok=False)
    state = dict(pid=os.getpid(), completed_packages=[], automatic_sequencing=False,
                 started_utc=datetime.now(timezone.utc).isoformat())

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
        for entry in packages:
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
    args = parser.parse_args()
    try:
        session(args.root.resolve())
    except Exception as error:
        print(type(error).__name__ + ': ' + str(error), flush=True)
    input('按 Enter 关闭此终端。')
