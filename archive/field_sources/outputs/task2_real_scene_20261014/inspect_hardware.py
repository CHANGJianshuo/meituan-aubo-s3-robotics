#!/usr/bin/env python3
"""Open the deployed task-two package's read-only controller inspection.

SDK credentials go directly to the remote TTY. This launcher neither reads
them nor invokes execute. It saves only inspection results after SSH exits.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys


ROOT = Path(__file__).resolve().parent


def main():
    deployment = json.loads((ROOT / 'deployment.json').read_text())
    if deployment['status'] != 'deployed_and_offline_verified':
        raise ValueError('The NUC deployment has not passed offline verification')
    index = json.loads((ROOT / 'commissioning/commissioning.json').read_text())
    first = index['packages'][0]
    if first['directory'] != '01_A_P1_approach':
        raise ValueError('Expected the first red-to-P1 approach package')
    manifest = ROOT / 'commissioning' / first['directory'] / 'manifest.json'
    if hashlib.sha256(manifest.read_bytes()).hexdigest() != first['manifest_sha256']:
        raise ValueError('Local inspection manifest changed')
    remote = deployment['remote_directory'] + '/' + first['directory']
    python = '/home/robot/miniforge3/envs/aubo-s3-hardware/bin/python'
    options = ['-o', 'ControlPath=/tmp/mtc_task2_ssh_20261014_2242/control',
               *deployment['transport_options']]
    host = deployment['ssh_host']
    # Pin the reviewed manifest before the player's own trajectory/adapter checks.
    program = (
        'from pathlib import Path; import hashlib,os; '
        f'p=Path({remote!r}); '
        f'assert hashlib.sha256((p/"manifest.json").read_bytes()).hexdigest()=={first["manifest_sha256"]!r}, "Manifest changed"; '
        f'os.execv({python!r},[{python!r},str(p/"run_hardware.py"),"inspect","--root",str(p)])'
    )
    print('任务二 · 只读核对 · 红→P1 / 黄→P2 / 蓝→P3', flush=True)
    print('下面在 NUC 终端输入 SDK 账号与密码。密码隐藏、不保存。此入口不会发送运动。', flush=True)
    result = dict(started_utc=datetime.now(timezone.utc).isoformat(),
                  command='inspect', package=first['directory'],
                  plan_id=first['plan_id'], remote_directory=remote,
                  motion_commands_sent=0)
    completed = subprocess.run(
        ['ssh', '-tt', *options, host, shlex.join([python, '-c', program])], check=False)
    result['ssh_returncode'] = completed.returncode
    if completed.returncode == 0:
        collect = (
            'from pathlib import Path; import json; '
            f'p=Path({remote!r})/"logs"; '
            's=max(p.glob("*.summary.json"),key=lambda x:x.stat().st_mtime_ns); '
            'print(json.dumps(dict(path=str(s),summary=json.loads(s.read_text()))))'
        )
        fetched = subprocess.run(
            ['ssh', '-T', *options, host, shlex.join([python, '-c', collect])],
            capture_output=True, text=True, check=True, timeout=15)
        result['inspection'] = json.loads(fetched.stdout)
        report = result['inspection']['summary']
        if not (report.get('read_only') and report.get('controller_checks_passed')):
            raise ValueError('No successful read-only controller inspection report')
        result['status'] = 'read_only_controller_checks_passed'
    else:
        result['status'] = 'inspection_incomplete'
    result['finished_utc'] = datetime.now(timezone.utc).isoformat()
    (ROOT / 'hardware_inspect.json').write_text(json.dumps(result, indent=2) + '\n')
    print('核对结果：' + result['status'], flush=True)
    print('记录：' + str(ROOT / 'hardware_inspect.json'), flush=True)
    print('机械臂未执行任务二动作；候选点位仍待现场对中验证。', flush=True)
    return completed.returncode


if __name__ == '__main__':
    try:
        code = main()
    except KeyboardInterrupt:
        code = 130
        print('\n只读核对已中断。')
    except Exception as exc:
        code = 1
        print(type(exc).__name__ + ': ' + str(exc), file=sys.stderr)
    try:
        input('按 Enter 关闭此终端。')
    except (EOFError, KeyboardInterrupt):
        pass
    raise SystemExit(code)
