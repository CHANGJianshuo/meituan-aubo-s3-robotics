#!/usr/bin/env python3
"""Start the hardware UI with a private, commissioned config and in-memory login.

Connecting only reads state. The task is submitted by the operator in the UI.
Credentials are passed through stdin; no password appears in argv or files.
"""
import argparse
import getpass
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8767)
    args = parser.parse_args()
    settings = json.loads(args.config.read_text())
    if not settings.get('ssh_host') or settings['ssh_host'].endswith('.invalid'):
        parser.error('Complete the private hardware configuration before connecting')
    if args.output_dir.exists():
        parser.error('Use a new output directory for each UI session')
    credentials = dict(username=input('SDK username: ').strip(),
                       password=getpass.getpass('SDK password (not saved): '))
    env = os.environ.copy()
    module_dir = Path(__file__).resolve().parents[1]/'src/mtc_cpp_mock_bringup'
    env['PYTHONPATH'] = str(module_dir) + (os.pathsep+env['PYTHONPATH'] if env.get('PYTHONPATH') else '')
    child = subprocess.Popen([sys.executable, '-m', 'mtc_webui.server',
        '--hardware-config', str(args.config.resolve()), '--auth-stdin',
        '--output-dir', str(args.output_dir.resolve()), '--port', str(args.port)],
        stdin=subprocess.PIPE, text=True, env=env)
    try:
        child.stdin.write(json.dumps(credentials)+'\n')
        child.stdin.flush()
        child.stdin.close()
        credentials.clear()
        return child.wait()
    except KeyboardInterrupt:
        # The child owns its shutdown / stop protocol. Do not send motion here.
        return child.wait()
    finally:
        credentials.clear()


if __name__ == '__main__':
    raise SystemExit(main())
