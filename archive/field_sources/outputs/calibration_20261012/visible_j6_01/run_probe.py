"""Local one-shot launcher; camera heartbeat and logs stay with this evidence."""
import json
from pathlib import Path
import shlex
import subprocess
import sys
import threading
import time

sys.path.insert(0, '/tmp/mtc_nuc_ssh_54nxfz40')
from query_robot import credentials

ROOT = Path(__file__).resolve().parent
CAMERA = Path('/mnt/c/Users/chang/AppData/Local/mtc_camera_runtime/captures/session_20261012')


def camera_fresh():
    s = json.loads((CAMERA / 'status.json').read_text())
    return s['state'] == 'streaming' and 0 <= time.time_ns() - s['host_received_ns'] < 4_000_000_000


def main():
    assert camera_fresh(), 'No fresh camera'
    plan = json.loads((ROOT / 'plan.json').read_text())
    assert not (ROOT / 'launched').exists(), 'Single use; inspect existing run'
    remote = (ROOT / 'field_probe.py').read_text()
    cmd = shlex.join(['env', '-u', 'PYTHONPATH', '-u', 'PYTHONHOME', '-u', 'LD_LIBRARY_PATH',
                      '/home/robot/miniforge3/envs/aubo-s3-hardware/bin/python', '-u', '-c', remote])
    with (ROOT / 'launched').open('x') as f: f.write(str(time.time_ns()))
    p = subprocess.Popen(['ssh', '-F', '/dev/null', '-S', '/tmp/mtc_nuc_ssh_54nxfz40/control',
                          'robot@10.11.79.239', cmd], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    p.stdin.write(json.dumps({'plan': plan, 'credentials': credentials()}) + '\n'); p.stdin.flush()

    def send_lease():
        try:
            while p.poll() is None:
                if not camera_fresh():
                    print('Camera stale: withdrawing motion lease', flush=True)
                    break
                p.stdin.write(json.dumps({'camera_fresh': True}) + '\n'); p.stdin.flush()
                time.sleep(.5)
        finally:
            try: p.stdin.close()
            except (BrokenPipeError, OSError): pass

    threading.Thread(target=send_lease, daemon=True).start()
    with (ROOT / 'console.log').open('a') as log:
        for line in p.stdout:
            log.write(line); log.flush(); print(line, end='', flush=True)
    raise SystemExit(p.wait())


if __name__ == '__main__': main()
