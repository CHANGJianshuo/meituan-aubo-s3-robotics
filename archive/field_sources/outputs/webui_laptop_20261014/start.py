#!/usr/bin/env python3
"""Start/stop the verified hardware UI. Opening the UI never submits motion."""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import http.client
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import sys

ROOT = Path(__file__).resolve().parent
STATE = ROOT / "current_run.json"
URL = "http://localhost:8767/"


def completed_motion_is_stopped(record, current):
    """Allow archiving a placement-only unknown after proven motion completion.

    This never changes the task result or clears its single-use claim. Missing
    completion receipts, an active process, stale feedback or a displaced arm
    still prohibit shutting down the execution supervisor.
    """
    try:
        task=current['task']; sample=current['robot']; robot=sample['data']
        if (task['status']!='UNKNOWN' or task['substate']!='placement_camera'
                or not sample['fresh'] or not robot['stop_confirmed'] or not robot['robot_ready']
                or robot['motion_active'] or robot['exec_id']!=-1 or robot['queue']!=0
                or current['operations']['stop'] or current['operations']['cancel']):
            return False
        if len(robot['q'])!=6 or len(robot['qd'])!=6 or not all(math.isfinite(v) for v in robot['q']+robot['qd']):
            return False
        if max(map(abs,robot['qd']))>math.radians(.02):return False
        root=Path(record['run_dir'])
        config=json.loads((root/'hardware-config.json').read_text()); package=Path(config['package'])
        manifest_bytes=(package/'manifest.json').read_bytes()
        if hashlib.sha256(manifest_bytes).hexdigest()!=config['manifest_sha256']:return False
        manifest=json.loads(manifest_bytes)
        raw=(package/'full_trajectory.json').read_bytes()
        if hashlib.sha256(raw).hexdigest()!=manifest['sha256']['full_trajectory.json']:return False
        bundle=json.loads(raw); count=len(bundle['segments'])
        if current['config']['plan_id']!=manifest['plan_id'] or bundle['plan_id']!=manifest['plan_id']:return False
        if task.get('stages_completed')!=count:return False
        if max(abs(a-b) for a,b in zip(robot['q'],bundle['segments'][-1]['points'][-1]['positions']))>math.radians(.05):return False
        events=[json.loads(line) for line in (root/'hardware-events.jsonl').read_text().splitlines()]
        if [e['stage'] for e in events if e['event']=='stage_complete']!=list(range(1,count+1)):return False
        if not any(e['event']=='completed' and e.get('plan_id')==manifest['plan_id'] for e in events):return False
        if not any(e['event']=='player_returned' for e in events):return False
        if json.loads((root/'player-exit.json').read_text())['returncode']!=0:return False
        after=json.loads((root/'state-after.json').read_text())
        if not after['independent_feedback'] or after['completed_stages']!=list(range(1,count+1)):return False
        visual=json.loads((root/'placement-visual.json').read_text())
        return visual['passed'] is False
    except (OSError,ValueError,KeyError,TypeError,IndexError):
        return False


def snapshot():
    connection = http.client.HTTPConnection("127.0.0.1", 8767, timeout=1)
    try:
        connection.request("GET", "/api/snapshot")
        response = connection.getresponse()
        if response.status != 200:
            raise RuntimeError("端口 8767 已被其他服务占用")
        result = json.loads(response.read())
        if result.get("mode") != "hardware_trajectory" or result.get("config", {}).get("nuc") != "192.0.2.10":
            raise RuntimeError("端口 8767 上不是本入口的实机轨迹 UI")
        return result
    except ConnectionRefusedError:
        return None
    except TimeoutError:
        # Some WSL networking modes drop connects to an unused loopback port.
        # A bound but unresponsive service must never trigger another launch.
        with socket.socket() as probe:
            try:
                probe.bind(("127.0.0.1", 8767))
            except OSError as error:
                raise RuntimeError("端口 8767 已占用但没有响应，请检查现有服务") from error
        return None
    finally:
        connection.close()


def stop():
    if not STATE.exists():
        print("没有本入口保存的运行记录。")
        return
    record = json.loads(STATE.read_text())
    pid = record["pid"]
    proc = Path(f"/proc/{pid}/cmdline")
    if not proc.exists():
        print("本次操作台已退出。")
        return
    argv = proc.read_bytes().split(b"\0")
    if b"mtc_webui.server" not in argv or record["run_dir"].encode() not in argv:
        raise RuntimeError("PID 已不属于本入口，未向任何进程发送信号")
    current = snapshot()
    if current and (current.get("task") or {}).get("status") in ("SUBMITTING", "ACCEPTED", "EXECUTING", "CANCELING", "UNKNOWN"):
        if not completed_motion_is_stopped(record,current):
            raise RuntimeError("当前实机任务尚未结束或结果未知；先在页面停止并核实现场，不直接关闭执行监督进程")
        evidence=dict(archived_at=datetime.now(timezone.utc).isoformat(),
            outcome='TRAJECTORY_COMPLETED_PLACEMENT_UNVERIFIED',task_id=current['task']['task_id'],
            task_result_unchanged=True,single_use_claim_preserved=True,robot=current['robot'],
            reason='All stage receipts, normal player exit and fresh independent final-pose stillness verified; placement-only UNKNOWN retained')
        (Path(record['run_dir'])/'supervisor-close-verification.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n')
    os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        if not proc.exists() or not proc.read_bytes():
            print("本机实机操作台已退出，执行记录已保留。")
            return
        time.sleep(.2)
    raise RuntimeError("服务尚未正常退出；未强制终止，请检查日志")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stop", action="store_true")
    parser.add_argument("--open-browser", action="store_true")
    parser.add_argument("--auth-stdin", action="store_true", help="从标准输入读取 SDK 凭据，只保存在本次服务内存")
    args = parser.parse_args()
    with (ROOT / "launcher.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if args.stop:
            stop()
            return
        current = snapshot()
        if current is None:
            credentials_line = sys.stdin.readline() if args.auth_stdin else None
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
            (ROOT / "runs").mkdir(exist_ok=True)
            run_dir = ROOT / "runs" / stamp
            log_path = ROOT / "runs" / (stamp + ".startup.log")
            with log_path.open("xb") as log:
                child = subprocess.Popen(["/bin/bash", str(ROOT / "run.sh"), str(run_dir), *(["--auth-stdin"] if args.auth_stdin else [])],
                    cwd=ROOT, stdin=subprocess.PIPE if args.auth_stdin else subprocess.DEVNULL, stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True)
            if credentials_line is not None:
                child.stdin.write(credentials_line.encode()); child.stdin.close()
                credentials_line = None
            STATE.write_text(json.dumps(dict(pid=child.pid, run_dir=str(run_dir),
                log=str(log_path), url=URL), ensure_ascii=False, indent=2) + "\n")
            deadline = time.monotonic() + 50
            while time.monotonic() < deadline:
                if child.poll() is not None:
                    raise RuntimeError(f"启动失败，请查看 {log_path}")
                current = snapshot()
                if current:
                    break
                time.sleep(.25)
            else:
                raise RuntimeError(f"尚未就绪；未自动重启。请查看 {log_path}")
        print("美团 Web UI：" + URL)
        print("运行模式：AUBO 实机已验证轨迹；打开页面不会自动执行。")
        if STATE.exists():
            print("本次日志：" + json.loads(STATE.read_text())["run_dir"])
        if args.open_browser:
            powershell = "/mnt/c/windows/System32/WindowsPowerShell/v1.0/powershell.exe"
            subprocess.run([powershell, "-NoProfile", "-NonInteractive", "-Command",
                            "Start-Process 'http://localhost:8767/'"], check=True)


if __name__ == "__main__":
    main()
