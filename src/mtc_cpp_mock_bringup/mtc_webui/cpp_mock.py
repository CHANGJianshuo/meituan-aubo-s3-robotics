"""Explicit C++ software Mock ROS bridge; never manufactures robot/tool feedback.

Only observed C++ heartbeat admission and ROS Action results control the UI.
Authored with OpenAI Codex, 2026-10-12. See ../provenance.json.
"""
from dataclasses import dataclass
import json
import os
import re

from mtc_webui.config import RosConfig
from mtc_webui.mock_backend import ApiError
from mtc_webui.ros_backend import RosBackend, ACTIVE
from mtc_webui.ros_transport import RosTransport


def normalized_namespace(value):
    value = '/' + value.strip('/')
    if not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', value):
        raise ValueError('namespace 必须为非根 ROS 命名空间')
    return value


@dataclass(frozen=True)
class CppConfig(RosConfig):
    cpp_namespace: str = '/mtc_cpp_mock'


def cpp_config(namespace='/mtc_cpp_mock', domain_id=None, read_only=False):
    ns = normalized_namespace(namespace)
    if domain_id is None and 'ROS_DOMAIN_ID' in os.environ:
        domain_id = int(os.environ['ROS_DOMAIN_ID'])
    return CppConfig(domain_id=domain_id, read_only=read_only, cpp_namespace=ns,
                     task_action=ns + '/task/execute', task_topic=ns + '/task/state',
                     task_publisher=ns + '/mtc_mock_task')


class CppMockBackend(RosBackend):
    mode = 'cpp_mock'
    scope_note = 'C++ Mock ROS 操作台；软件 Mock，无硬件、MoveIt、独立停稳传感器'

    def _reasons(self, include_active=True):
        reasons = []
        if self.config.read_only:
            reasons.append('当前为只读模式')
        if not self.connection['connected']:
            reasons.append(self.connection['reason'])
        if not self.connection.get('action_ready') or not self.connection.get('child_ready'):
            reasons.append('C++ Task/PickPlace 服务未就绪、来源不符或图中存在冲突')
        for kind, label in [('cpp_task', 'C++ Task'), ('cpp_pick', 'C++ PickPlace')]:
            sample = self._sample(kind)
            if not sample.get('fresh'):
                reasons.append(label + ' 就绪状态缺失或过期')
            elif sample['data'].get('scope') != 'cpp_mock' or sample['data'].get('admission_open') is not True:
                reasons.append(label + ' 正忙或被内核锁定')
        sample = self._sample('cpp_task')
        if sample.get('fresh') and sample['data'].get('initialized') is not True:
            reasons.append('C++ Task 软件自检尚未完成')
        task = self.tasks.get(self.current_id)
        if include_active and task and task['status'] in ACTIVE:
            reasons.append('当前 Goal 尚未结束或结果未知')
        if task and task.get('_ended') is not None:
            if any(self.samples.get(k, {}).get('_received', -1) <= task['_ended'] for k in ('cpp_task', 'cpp_pick')):
                reasons.append('等待 Result 之后的新 C++ 就绪状态')
        return reasons

    def snapshot(self):
        with self.lock:
            data = super().snapshot()
            data.update(cpp_task=self._sample('cpp_task'), cpp_pick=self._sample('cpp_pick'),
                        cpp_pick_action=self.config.cpp_namespace + '/manipulation/pick_place')
            data['operations']['stop'] = False
            return data

    def request_stop(self):
        raise ApiError(409, 'STOP_UNAVAILABLE', 'C++ Mock 没有实机 RequestStop；只能定向取消本会话 Goal')


class CppMockTransport(RosTransport):
    def _run(self):
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.action import ActionClient
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from mtc_interfaces.action import ExecuteTask
            from std_msgs.msg import String
            from rcl_interfaces.msg import Log
            from rclpy.qos import qos_profile_sensor_data
            self.ExecuteTask = ExecuteTask
            self.context = Context()
            rclpy.init(args=[], context=self.context, domain_id=self.config.domain_id)
            self.node = Node('mtc_web_console_' + self.backend.session_id[:8],
                             namespace=self.config.cpp_namespace, context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.action = ActionClient(self.node, ExecuteTask, self.config.task_action)
            self.qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
            for kind, name in [('cpp_task', 'mtc_mock_task'), ('cpp_pick', 'mtc_mock_pick_place')]:
                full = self.config.cpp_namespace + '/' + name
                self._subscribe(kind, String, full + '/integration_state', full)
            self.node.create_subscription(Log, '/rosout', self._log, qos_profile_sensor_data)
            self.node.create_timer(0.02, self._drain)
            self.node.create_timer(0.2, self._graph)
            while not self.done.is_set():
                self.executor.spin_once(timeout_sec=0.1)
        except Exception as error:
            self.backend.connection_update(connected=False, action_ready=False, child_ready=False,
                                           reason=f'C++ Mock ROS 接入失败：{type(error).__name__}: {error}')
        finally:
            if self.executor:
                self.executor.shutdown(timeout_sec=1)
            if hasattr(self, 'action'):
                self.action.destroy()
            if self.node:
                self.node.destroy_node()
            if self.context and self.context.ok():
                self.context.shutdown()

    def _graph(self):
        from mtc_cpp_mock_bringup.graph import graph_snapshot
        graph = graph_snapshot(self.node, self.config.cpp_namespace)
        publishers = {kind: self._authority(kind) for kind in self.topics}
        ready = graph['valid'] and self.action.server_is_ready()
        self.backend.connection_update(connected=True, action_ready=ready, child_ready=ready,
                                       stop_ready=False, action_servers=graph['task_servers'],
                                       publishers=publishers, graph=graph,
                                       reason='C++ Mock ROS 已连接；仅使用 C++ 节点就绪状态与 Action 结果')

    def _receive(self, kind, message, info):
        self._authority(kind)
        source = self.sources.get(kind)
        if not source:
            return
        try:
            data = json.loads(message.data)
            if data.get('scope') != 'cpp_mock' or data.get('node') != self.topics[kind][1]:
                raise ValueError('unexpected C++ source')
            stamp = data['stamp_ns']
            if type(stamp) is not int:
                raise ValueError('invalid stamp')
            age = (self.node.get_clock().now().nanoseconds - stamp) / 1e6
            self.backend.observe(kind, data, source[1], stamp, age)
            # UI task observation uses only actual core/slot fields; no RobotState is synthesized.
            if kind == 'cpp_task':
                self.backend.observe('task_state', dict(task_id=data.get('task_id', ''),
                    state=data['state'], substate=data.get('substate', ''),
                    completed_count=data['completed_count'], total_count=data['total_count'],
                    last_error_code=data['last_error_code'], detail='C++ Task 只读状态'), source[1], stamp, age)
        except (ValueError, TypeError, KeyError):
            self.backend.invalidate(kind)
