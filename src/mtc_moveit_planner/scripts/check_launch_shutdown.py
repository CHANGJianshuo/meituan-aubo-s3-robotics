#!/usr/bin/env python3
"""Require a clean exit record for every process started by a ROS launch.

Launch itself can return zero after a child segfault. A disappeared ROS node is
also insufficient evidence of a clean shutdown. This checks child lifecycle
records, including PID, rather than generic ROS [ERROR] messages.
"""

import argparse
import re
from pathlib import Path


def shutdown_errors(text):
    started = re.findall(
        r"^\[INFO\] \[([^]]+)\]: process started with pid \[(\d+)\]", text, re.M)
    finished = set(re.findall(
        r"^\[INFO\] \[([^]]+)\]: process has finished cleanly \[pid (\d+)\]", text, re.M))
    errors = []
    if not started:
        errors.append("no process-start record")
    for name, pid in started:
        if (name, pid) not in finished:
            errors.append(f"{name} pid={pid}: no clean-exit record")
    for line in text.splitlines():
        if re.match(r"^\[(?:ERROR|INFO)\] \[[^]]+\]: ", line) and (
            "process has died" in line or "failed to terminate" in line or
            re.search(r"sending signal 'SIG(?:TERM|KILL)'", line)
        ):
            errors.append(line)
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    args = parser.parse_args()
    try:
        errors = shutdown_errors(args.log.read_text(encoding="utf-8"))
    except (OSError, UnicodeError) as exc:
        errors = [str(exc)]
    if errors:
        print("FAIL clean shutdown: " + "; ".join(errors))
        return 1
    print("PASS clean shutdown: every launched process exited cleanly")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
