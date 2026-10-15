"""Bounded ExecuteTask client with raw parent/child Action observations.

No synthetic status and no cancel-all. Cancel ACK and final Result are distinct.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatusArray
from mtc_interfaces.action import ExecuteTask, PickPlace
from rosidl_runtime_py.convert import message_to_ordereddict
from unique_identifier_msgs.msg import UUID
from mtc_webui.cpp_mock import normalized_namespace


class Recorder(Node):
    def __init__(self, namespace, output):
        super().__init__('mtc_chain_recorder', namespace='/mtc_cpp_mock_observers')
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.stream = (self.output / 'events.jsonl').open('x')
        self.events = []
        self.status_seen = set()
        self.observers = []
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for action, cls in [('task/execute', ExecuteTask), ('manipulation/pick_place', PickPlace)]:
            endpoint = namespace + '/' + action
            def status_callback(topic):
                def callback(msg):
                    self.status(topic, msg)
                return callback
            def feedback_callback(topic):
                def callback(msg):
                    self.event('feedback', endpoint=topic, goal_id=bytes(msg.goal_id.uuid).hex(),
                               feedback=dict(message_to_ordereddict(msg.feedback)))
                return callback
            self.observers.append(self.create_subscription(GoalStatusArray, endpoint + '/_action/status', status_callback(endpoint), qos))
            self.observers.append(self.create_subscription(cls.Impl.FeedbackMessage, endpoint + '/_action/feedback', feedback_callback(endpoint), 20))

    def ready(self):
        return all(sub.get_publisher_count() == 1 for sub in self.observers)

    def event(self, kind, **data):
        value = dict(kind=kind, at=datetime.now(timezone.utc).isoformat(), monotonic_ns=time.monotonic_ns(), **data)
        self.events.append(value)
        self.stream.write(json.dumps(value, ensure_ascii=False) + '\n')
        self.stream.flush()
        return value

    def status(self, endpoint, msg):
        for item in msg.status_list:
            identity = bytes(item.goal_info.goal_id.uuid).hex()
            key = (endpoint, identity, item.status)
            if key not in self.status_seen:
                self.status_seen.add(key)
                self.event('status', endpoint=endpoint, goal_id=identity, status=item.status,
                           goal_stamp=dict(message_to_ordereddict(item.goal_info.stamp)))

    def finish(self):
        self.stream.close()
        self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='/mtc_cpp_mock')
    parser.add_argument('--goal', required=True, help='ExecuteTask Goal JSON')
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--cancel-at', help='Cancel only after this actual Task feedback substate')
    parser.add_argument('--cancel-delay-ms', type=int, default=0, help='Diagnostic delay after observing --cancel-at; does not alter the servers')
    parser.add_argument('--wait', type=float, default=40)
    args = parser.parse_args()
    namespace = normalized_namespace(args.namespace)
    payload = json.loads(args.goal)
    rclpy.init(args=[])
    node = Recorder(namespace, args.output_dir)
    client = ActionClient(node, ExecuteTask, namespace + '/task/execute')
    handle = result_future = cancel_future = None
    feedback = []
    cancel_seen_at = None
    final = dict(status='UNKNOWN')
    try:
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('Task Action server unavailable')
        ready_deadline = time.monotonic() + 5
        while not node.ready() and time.monotonic() < ready_deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not node.ready():
            raise RuntimeError('Parent/child observation endpoints not matched')
        goal = ExecuteTask.Goal()
        for key, val in payload.items():
            setattr(goal, key, float(val) if key == 'placement_stability_sec' else val)
        identity = uuid.uuid4()
        node.event('goal_sent', goal_id=identity.hex, goal=payload)
        future = client.send_goal_async(goal, goal_uuid=UUID(uuid=list(identity.bytes)),
                                         feedback_callback=lambda msg: feedback.append(msg.feedback))
        deadline = time.monotonic() + args.wait
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if handle is None and future.done():
                handle = future.result()
                node.event('goal_response', goal_id=identity.hex, accepted=handle.accepted)
                if not handle.accepted:
                    final = dict(status='REJECTED', goal_id=identity.hex)
                    break
                result_future = handle.get_result_async()
            if handle is not None and handle.accepted and args.cancel_at and cancel_future is None:
                if any(f.substate == args.cancel_at for f in feedback):
                    if cancel_seen_at is None:
                        cancel_seen_at = time.monotonic()
                        node.event('cancel_trigger_seen', substate=args.cancel_at, delay_ms=args.cancel_delay_ms)
                    if (time.monotonic() - cancel_seen_at) * 1000 >= args.cancel_delay_ms:
                        node.event('cancel_requested', goal_id=identity.hex)
                        cancel_future = handle.cancel_goal_async()
            if cancel_future is not None and cancel_future.done() and not any(e['kind'] == 'cancel_reply' for e in node.events):
                node.event('cancel_reply', goal_id=identity.hex, response=dict(message_to_ordereddict(cancel_future.result())))
            if result_future is not None and result_future.done():
                response = result_future.result()
                final = dict(goal_id=identity.hex, status=response.status, result=dict(message_to_ordereddict(response.result)))
                node.event('result', **final)
                # Drain final child / parent status messages, without deriving a fake result from ACK.
                until = time.monotonic() + .3
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.02)
                break
        else:
            node.event('timeout', message='No terminal Result; final status unknown')
    except Exception as error:
        final['error'] = str(error)
        node.event('error', message=str(error))
    finally:
        (args.output_dir / 'result.json').write_text(json.dumps(final, ensure_ascii=False, indent=2) + '\n')
        client.destroy()
        node.finish()
        rclpy.shutdown()
    print(json.dumps(final, ensure_ascii=False))
    raise SystemExit(2 if final['status'] == 'UNKNOWN' else 0)


if __name__ == "__main__":
    main()
