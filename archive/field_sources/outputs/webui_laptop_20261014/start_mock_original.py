#!/usr/bin/env python3
"""Start/stop this laptop's isolated C++ Mock Web UI; never contact hardware."""
import argparse
from datetime import datetime, timezone
import fcntl
import http.client
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

ROOT = Path(__file__).resolve().parent
STATE = ROOT / "current_run.json"
URL = "http://localhost:8767/"


def snapshot():
    connection = http.client.HTTPConnection("127.0.0.1", 8767, timeout=1)
    try:
        connection.request("GET", "/api/snapshot")
        response = connection.getresponse()
        if response.status != 200:
            raise RuntimeError("端口 8767 已被其他服务占用")
        result = json.loads(response.read())
        if result.get("mode") != "cpp_mock" or result.get("config", {}).get("cpp_namespace") != "/mtc_laptop_mock":
            raise RuntimeError("端口 8767 上不是本机配置的 C++ Mock UI")
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
    if b"mtc_cpp_mock_bringup.bringup" not in argv or record["run_dir"].encode() not in argv:
        raise RuntimeError("PID 已不属于本入口，未向任何进程发送信号")
    os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + 25
    cleanup = Path(record["run_dir"]) / "cleanup.json"
    while time.monotonic() < deadline:
        if cleanup.exists():
            data = json.loads(cleanup.read_text())
            if not data.get("cleanup_ok") or data.get("forced_cleanup") or data.get("exit_code") != 0:
                raise RuntimeError(f"退出检查未通过，请查看 {cleanup}")
            print("本机 Web UI 与所属 C++ Mock 节点已正常退出。")
            return
        time.sleep(.2)
    raise RuntimeError(f"未收到退出检查结果，请查看 {record['run_dir']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stop", action="store_true")
    parser.add_argument("--open-browser", action="store_true")
    args = parser.parse_args()
    with (ROOT / "launcher.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if args.stop:
            stop()
            return
        current = snapshot()
        if current is None:
            if not (ROOT / "install/setup.bash").is_file():
                raise RuntimeError("本机 Web UI 依赖尚未构建")
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
            (ROOT / "runs").mkdir(exist_ok=True)
            run_dir = ROOT / "runs" / stamp
            log_path = ROOT / "runs" / (stamp + ".startup.log")
            with log_path.open("xb") as log:
                child = subprocess.Popen(["/bin/bash", str(ROOT / "run.sh"), str(run_dir)],
                    cwd=ROOT, stdin=subprocess.DEVNULL, stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True)
            STATE.write_text(json.dumps(dict(pid=child.pid, run_dir=str(run_dir),
                log=str(log_path), url=URL), ensure_ascii=False, indent=2) + "\n")
            deadline = time.monotonic() + 50
            while time.monotonic() < deadline:
                if child.poll() is not None:
                    raise RuntimeError(f"启动失败，请查看 {log_path}")
                current = snapshot()
                if current and current.get("operations", {}).get("submit"):
                    break
                time.sleep(.25)
            else:
                raise RuntimeError(f"尚未就绪；未自动重启。请查看 {log_path}")
        print("美团 Web UI：" + URL)
        print("运行模式：C++ Mock · 本机 ROS 2，现场相机仅供观察。")
        if STATE.exists():
            print("本次日志：" + json.loads(STATE.read_text())["run_dir"])
        if args.open_browser:
            powershell = "/mnt/c/windows/System32/WindowsPowerShell/v1.0/powershell.exe"
            subprocess.run([powershell, "-NoProfile", "-NonInteractive", "-Command",
                            "Start-Process 'http://localhost:8767/'"], check=True)


if __name__ == "__main__":
    main()
