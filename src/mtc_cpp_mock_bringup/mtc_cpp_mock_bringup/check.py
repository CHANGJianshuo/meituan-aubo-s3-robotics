"""Read-only readiness and post-stop checks; bounded wait, JSON evidence."""
import argparse
import json
from pathlib import Path
import time
from mtc_webui.cpp_mock import CppMockBackend, CppMockTransport, cpp_config


def await_ready(namespace, domain_id=None, timeout=15, empty=False):
    backend = CppMockBackend(cpp_config(namespace, domain_id))
    transport = CppMockTransport(backend)
    transport.start()
    deadline = time.monotonic() + timeout
    ok = False
    snapshot = backend.snapshot()
    try:
        while time.monotonic() < deadline:
            snapshot = backend.snapshot()
            if empty:
                graph = snapshot['connection'].get('graph')
                # DDS discovery settles for at least two seconds, even on an empty graph.
                elapsed = timeout - max(0, deadline - time.monotonic())
                ok = graph is not None and elapsed >= 2 and not any(
                    n.startswith(namespace.rstrip('/') + '/') and '/mtc_web_console_' not in n
                    for n in graph['nodes']) and not any(
                    s['action'].startswith(namespace.rstrip('/') + '/') for s in graph['servers'])
            else:
                ok = snapshot['operations']['submit']
            if ok:
                break
            time.sleep(0.1)
    finally:
        backend.close()
    return dict(ok=bool(ok), expectation='empty' if empty else 'ready', snapshot=snapshot)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='/mtc_cpp_mock')
    parser.add_argument('--ros-domain-id', type=int)
    parser.add_argument('--timeout', type=float, default=15)
    parser.add_argument('--empty', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = await_ready(args.namespace, args.ros_domain_id, args.timeout, args.empty)
    text = json.dumps(report, ensure_ascii=False, indent=2) + '\n'
    if args.output:
        with args.output.open('x') as stream:
            stream.write(text)
    print(text)
    raise SystemExit(0 if report['ok'] else 1)


if __name__ == "__main__":
    main()
