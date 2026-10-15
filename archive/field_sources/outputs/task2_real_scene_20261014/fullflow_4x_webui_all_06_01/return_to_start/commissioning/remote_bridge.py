#!/usr/bin/env python3
"""NUC adapter. Runs the unchanged, hash-pinned guarded player exactly once.

Credentials and watchdog messages arrive over the owned SSH stdin, never argv/files.
The bridge adds observability/owned cancellation; it does not weaken player checks.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time

PREFIX = 'MTC_HARDWARE '
write_lock = threading.Lock()


def emit(event, **fields):
    with write_lock:
        print(PREFIX + json.dumps(dict(event=event, bridge_unix=time.time(), **fields),
                                 ensure_ascii=False, allow_nan=False), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=['state', 'execute'])
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--manifest-sha256', required=True)
    args = parser.parse_args()
    assert hashlib.sha256((args.root/'manifest.json').read_bytes()).hexdigest() == args.manifest_sha256
    manifest = json.loads((args.root/'manifest.json').read_text())
    for name, digest in manifest['adapter_sha256'].items():
        assert hashlib.sha256((args.root/name).read_bytes()).hexdigest() == digest, name
    sys.path.insert(0, str(args.root))
    import run_hardware as player
    from trajectory import load_package, validate
    bundle, manifest = load_package(args.root, 'full')
    validate(bundle, manifest['captured_limits'])
    sys.path.insert(0, str(player.HARDWARE))
    from j6_45 import Robot45
    from telemetry import Telemetry, quiet_native_auth
    credentials = json.loads(sys.stdin.readline())
    username, password = credentials.pop('username'), credentials.pop('password')
    credentials.clear()
    quit_event = threading.Event()
    last_ping = [time.monotonic()]
    heartbeat_token = [None]
    interrupt_sent = threading.Event()
    active = threading.Event()

    def stop_owned(reason):
        quit_event.set()
        if args.command == 'execute' and active.is_set() and not interrupt_sent.is_set():
            interrupt_sent.set()
            emit('owned_stop_requested', reason=reason)
            os.kill(os.getpid(), signal.SIGTERM)

    def commands():
        try:
            for line in sys.stdin:
                message = json.loads(line)
                command = message.get('command')
                if command == 'ping':
                    last_ping[0] = time.monotonic()
                    heartbeat_token[0] = message.get("token")
                elif command == 'stop':
                    stop_owned('Explicit stop for this owned execution')
                    return
                else:
                    stop_owned('Invalid bridge command')
                    return
        finally:
            stop_owned('SSH control input closed')

    def watchdog():
        while not quit_event.wait(.2):
            if time.monotonic() - last_ping[0] > 5:
                stop_owned('No UI supervisor heartbeat for 5 seconds')

    def interrupt(*_):
        raise KeyboardInterrupt('Owned execution interrupted')

    signal.signal(signal.SIGTERM, interrupt)
    threading.Thread(target=commands, daemon=True).start()
    threading.Thread(target=watchdog, daemon=True).start()
    robot = None
    log = None
    try:
        with quiet_native_auth():
            robot = Robot45(manifest, username, password)
        player.configuration_check(robot, manifest)
        emit('identity', identity=robot.identity(), tcp_offset=robot.config.getTcpOffset(),
             payload=robot.config.getPayload(), plan_id=manifest['plan_id'])
        if args.command == 'state':
            while not quit_event.is_set():
                emit('state', state=robot.snapshot(), heartbeat_token=heartbeat_token[0])
                quit_event.wait(.2)
            return

        class StreamingLog(player.Log):
            def event(self, event, **fields):
                super().event(event, **fields)
                emit(event, **fields)

        log = StreamingLog(args.root, 'webui_execute')
        emit('log_path', path=str(log.path), pid=os.getpid())
        if quit_event.is_set():
            raise RuntimeError('Supervisor already disconnected; execution prohibited')
        active.set()
        # These are the original functions, including flock, atomic attempt ledger,
        # read-only inspect, start checks, RTDE guards and stop-on-failure.
        player.execute(robot, lambda: Telemetry(manifest['host'], username, password),
                       bundle, manifest, 1.0, log)
        emit('player_returned')
    except BaseException as error:
        emit('bridge_error', error=type(error).__name__ + ': ' + str(error))
        raise
    finally:
        active.clear()
        quit_event.set()
        password = None
        if robot is not None:
            robot.close()
        if log is not None:
            log.close()


if __name__ == '__main__':
    main()
