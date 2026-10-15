"""Own exactly two C++ Mock processes and an optional loopback Web UI.

No driver, MoveIt, device, Python Task or Python PickPlace is launched.
"""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import http.client
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from ament_index_python.packages import get_package_prefix
from mtc_webui.cpp_mock import normalized_namespace
from mtc_cpp_mock_bringup.check import await_ready


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='/mtc_cpp_mock')
    parser.add_argument('--output-dir', required=True, type=Path, help='New output directory, never overwritten')
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--no-web', action='store_true')
    parser.add_argument('--readonly', action='store_true')
    parser.add_argument('--preview-dir', type=Path, help='Read existing camera preview files for display only')
    parser.add_argument('--profile', choices=['success', 'planning_failure', 'cancel_unconfirmed', 'status_unknown'], default='success')
    parser.add_argument('--result-delay-ms', type=int, default=250, help='Existing software Mock future delay')
    parser.add_argument('--ready-timeout', type=float, default=20)
    args = parser.parse_args()
    ns = normalized_namespace(args.namespace)
    if os.environ.get('ROS_DISTRO') != 'jazzy' or not os.environ.get('ROS_DOMAIN_ID', '').isdigit():
        parser.error('source Jazzy/install and set explicit ROS_DOMAIN_ID (0..232)')
    domain = int(os.environ['ROS_DOMAIN_ID'])
    if not 0 <= domain <= 232 or not 1 <= args.port <= 65535 or not 0 <= args.result_delay_ms <= 5000:
        parser.error('invalid domain, port or Mock result delay')
    lock_path = Path('/tmp') / ('mtc-cpp-mock-' + str(domain) + '-' + hashlib.sha256(ns.encode()).hexdigest()[:16] + '.lock')
    lock = lock_path.open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('A stack already owns this domain/namespace')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    output = args.output_dir.resolve()
    children = []
    logs = []
    stopping = False
    result = dict(supervisor_pid=os.getpid(), started_at=datetime.now(timezone.utc).isoformat(), namespace=ns,
                  domain_id=domain, profile=args.profile, result_delay_ms=args.result_delay_ms,
                  scope='cpp_mock; real ROS Actions, software-only ports; no hardware/MoveIt')
    def write(name, data):
        (output / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    def stop_signal(_signum, _frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    code = 1
    try:
        pre = await_ready(ns, domain, 3, empty=True)
        write('preflight.json', pre)
        graph = pre['snapshot']['connection'].get('graph', {})
        if not pre['ok'] or any(s['action'] in ['/mtc/task/execute', '/mtc/manipulation/pick_place'] for s in graph.get('servers', [])):
            raise RuntimeError('Namespace or legacy Action graph is not empty; inspect preflight.json')
        binary_dir = Path(get_package_prefix('mtc_ros2_action_cpp')) / 'lib/mtc_ros2_action_cpp'
        common = ['--ros-args', '-r', '__ns:=' + ns,
                  '-r', '/mtc/task/execute:=' + ns + '/task/execute',
                  '-r', '/mtc/manipulation/pick_place:=' + ns + '/manipulation/pick_place']
        options = ['-p', 'mock.result_delay_ms:=' + str(args.result_delay_ms),
                   '-p', 'mock.plan_available:=' + ('false' if args.profile == 'planning_failure' else 'true'),
                   '-p', 'mock.stop_evidence_available:=' + ('false' if args.profile == 'cancel_unconfirmed' else 'true'),
                   '-p', 'mock.status_unknown:=' + ('true' if args.profile == 'status_unknown' else 'false')]
        commands = [('pick_place', [str(binary_dir / 'mock_pick_place_server'), *common, *options]),
                    ('task', [str(binary_dir / 'mock_task_server'), *common])]
        def launch(label, argv):
            stream = (output / (label + '.log')).open('wb')
            logs.append(stream)
            child = subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            children.append((label, child))
            result.setdefault('processes', []).append(dict(label=label, pid=child.pid, argv=argv,
                                                          executable=os.readlink(f'/proc/{child.pid}/exe')))
            write('launch.json', result)
        for label, argv in commands:
            launch(label, argv)
        ready = await_ready(ns, domain, args.ready_timeout)
        write('ready.json', ready)
        if not ready['ok']:
            raise RuntimeError('C++ stack readiness failed; inspect ready.json and node logs')
        if not args.no_web:
            argv = [sys.executable, '-m', 'mtc_webui.server', '--cpp-mock', '--cpp-namespace', ns,
                    '--ros-domain-id', str(domain), '--port', str(args.port), '--output-dir', str(output / 'web-session')]
            if args.readonly:
                argv.append('--readonly')
            if args.preview_dir:
                argv.extend(['--preview-dir', str(args.preview_dir.resolve())])
            launch('web', argv)
            web_deadline = time.monotonic() + args.ready_timeout
            while time.monotonic() < web_deadline:
                if any(child.poll() is not None for _, child in children):
                    raise RuntimeError('Process exited before Web readiness')
                connection = http.client.HTTPConnection('127.0.0.1', args.port, timeout=1)
                try:
                    connection.request('GET', '/api/snapshot')
                    response = connection.getresponse()
                    snapshot = json.loads(response.read())
                    if response.status == 200 and snapshot.get('mode') == 'cpp_mock' and snapshot['config']['cpp_namespace'] == ns and snapshot['connection'].get('action_ready'):
                        write('web-ready.json', snapshot)
                        break
                except (OSError, ValueError, http.client.HTTPException):
                    pass
                finally:
                    connection.close()
                time.sleep(.1)
            else:
                raise RuntimeError('Web UI did not become ready for this C++ namespace')
        print('READY ' + json.dumps(dict(namespace=ns, domain=domain, logs=str(output),
                                         url=None if args.no_web else f'http://127.0.0.1:{args.port}/')), flush=True)
        code = 0
        while not stopping:
            exited = [(label, child.returncode) for label, child in children if child.poll() is not None]
            if exited:
                result['unexpected_exit'] = exited
                code = 1
                break
            time.sleep(.1)
    except Exception as error:
        result['error'] = str(error)
        print(str(error), file=sys.stderr, flush=True)
        code = 1
    finally:
        # Stop only children owned by this process, never a global pkill/ROS daemon.
        for label, child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        deadline = time.monotonic() + 8
        for label, child in reversed(children):
            try:
                child.wait(timeout=max(.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                result.setdefault('forced_cleanup', []).append(label)
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=3)
                code = 1
        result['returncodes'] = {label: child.returncode for label, child in children}
        for stream in logs:
            stream.close()
        post = await_ready(ns, domain, 8, empty=True)
        write('cleanup-graph.json', post)
        if not post['ok']:
            code = 1
        result.update(ended_at=datetime.now(timezone.utc).isoformat(), cleanup_ok=post['ok'], exit_code=code)
        write('cleanup.json', result)
        lock.close()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
