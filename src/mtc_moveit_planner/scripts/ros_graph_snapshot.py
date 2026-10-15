#!/usr/bin/env python3
"""只读 ROS 图快照（不依赖 ros2 CLI daemon）：节点、话题、Action、控制器、硬件组件、一帧 JointState。

WSL2 mirrored 网络下 ros2 CLI 连接本机 daemon 端口会超时，故直接用 rclpy 查询。
只调用 list_* 查询服务，不加载/切换控制器，不发送任何运动目标。
"""

import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from sensor_msgs.msg import JointState


def call(node, srv_type, name, timeout=10.0):
    client = node.create_client(srv_type, name)
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    future = client.call_async(srv_type.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result()


def main():
    rclpy.init()
    node = rclpy.create_node('mtc_ros_graph_snapshot')
    end = time.time() + 3.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    print('## nodes')
    for name, ns in sorted(node.get_node_names_and_namespaces()):
        print(f"{ns.rstrip('/')}/{name}")

    topics = sorted(node.get_topic_names_and_types())
    print('## topics')
    for name, types in topics:
        if '/_action/' not in name:
            print(f'{name} [{", ".join(types)}]')

    print('## actions')
    for name, types in topics:
        if name.endswith('/_action/status'):
            action = name[: -len('/_action/status')]
            feedback = dict(topics).get(action + '/_action/feedback', [''])[0]
            print(f"{action} [{feedback.replace('_FeedbackMessage', '')}]")

    print('## controllers (/controller_manager/list_controllers)')
    result = call(node, ListControllers, '/controller_manager/list_controllers')
    if result is None:
        print('UNAVAILABLE')
    else:
        for c in result.controller:
            print(f'{c.name} type={c.type} state={c.state} '
                  f'claimed={list(c.claimed_interfaces)}')

    print('## hardware components (/controller_manager/list_hardware_components)')
    result = call(node, ListHardwareComponents, '/controller_manager/list_hardware_components')
    if result is None:
        print('UNAVAILABLE')
    else:
        for hw in result.component:
            print(f'{hw.name} type={hw.type} plugin={hw.plugin_name} '
                  f'state={hw.state.label}')

    print('## joint_states (one message)')
    msgs = []
    sub = node.create_subscription(JointState, '/joint_states', msgs.append, 10)
    end = time.time() + 5.0
    while time.time() < end and not msgs:
        rclpy.spin_once(node, timeout_sec=0.1)
    if msgs:
        m = msgs[0]
        print(f'stamp={m.header.stamp.sec}.{m.header.stamp.nanosec:09d}')
        for n, p in zip(m.name, m.position):
            print(f'{n}: {p:.6f}')
    else:
        print('NO MESSAGE')
    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
