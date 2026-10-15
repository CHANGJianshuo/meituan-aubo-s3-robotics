#!/usr/bin/env python3
"""Freeze RGB-D and read-only UI telemetry without submitting robot motion."""
import argparse
from datetime import datetime, timezone
import hashlib
import http.client
import json
import os
from pathlib import Path
import time


def snapshot(port):
    client = http.client.HTTPConnection('127.0.0.1', port, timeout=3)
    try:
        client.request('GET', '/api/snapshot')
        response = client.getresponse()
        if response.status != 200:
            raise RuntimeError('Read-only state endpoint unavailable')
        data = json.loads(response.read())
        robot = data['robot']
        if not robot['fresh'] or not robot['data']['stop_confirmed']:
            raise RuntimeError('Fresh stationary controller feedback is required')
        if robot['data']['motion_active'] or robot['data']['queue'] != 0:
            raise RuntimeError('Robot has an active motion or queued command')
        return dict(read_only=True, captured_utc=datetime.now(timezone.utc).isoformat(),
                    robot=robot, camera=data['camera'],
                    previous_task={k: (data.get('task') or {}).get(k)
                                   for k in ('task_id', 'status', 'finished_at')})
    finally:
        client.close()


def capture(preview, output, port=8767, frames=30):
    output.mkdir(parents=True, exist_ok=False)
    name = 'task2_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    request = preview / 'request_capture.json'
    if request.exists():
        raise RuntimeError('Another camera capture is pending')
    samples = [snapshot(port)]
    # Publish a complete JSON atomically, without replacing another request.
    # The existing camera process consumes it; this script never opens USB.
    temporary = preview / (name+'_request.tmp')
    try:
        with temporary.open('x') as stream:
            json.dump(dict(name=name, frames=frames), stream)
            stream.flush(); os.fsync(stream.fileno())
        os.link(temporary, request)
    finally:
        temporary.unlink(missing_ok=True)
    target = preview / name
    deadline = time.monotonic() + 90
    while not (target / 'capture.json').exists():
        if time.monotonic() > deadline:
            raise TimeoutError('RGB-D capture did not finish; inspect the existing camera process')
        time.sleep(.5)
        samples.append(snapshot(port))
    samples.append(snapshot(port))
    metadata = json.loads((target / 'capture.json').read_text())
    if not metadata['complete'] or len(metadata['frames']) != frames:
        raise RuntimeError('Incomplete RGB-D evidence')
    q0 = samples[0]['robot']['data']['q']
    drift = max(abs(x-y) for s in samples for x, y in zip(q0, s['robot']['data']['q']))
    if drift > .00035:
        raise RuntimeError('Robot moved during capture')
    record = dict(capture_directory=str(target.resolve()), read_only=True,
                  motion_commands_sent=0, telemetry=samples, max_joint_drift_rad=drift,
                  timestamp_note='Robot feedback and camera are not hardware synchronized',
                  sha256={n: hashlib.sha256((target/n).read_bytes()).hexdigest()
                          for n in ('color.png', 'aligned_depth.png', 'capture.json', 'frames.npz')})
    (output / 'capture_record.json').write_text(json.dumps(record, indent=2)+'\n')
    (output / 'physical_initial.png').write_bytes((target / 'color.png').read_bytes())
    print(json.dumps({k: v for k, v in record.items() if k != 'telemetry'}, indent=2))
    return record


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--frames', type=int, default=30)
    args = parser.parse_args()
    if not 1 <= args.frames <= 90:
        parser.error('frames must be 1..90')
    capture(args.preview, args.output, args.port, args.frames)
