"""mtc_aubo_bridge 的 rclpy 节点外壳。

安全默认值：`mode=disabled`。
- 该模式下**不会**导入 pyaubo、**不会**打开任何网络连接、**不会**触发任何 IO；
- 一切运动与 IO 请求都返回明确的拒绝，而不是静默成功；
- 只有显式设置 `mode=fake` 才会使用内存 Fake Worker（仅用于离线测试）。

本进程**不存在**真实 SDK 调用路径：真实通路必须由独立的 Python 3.10
SDK Worker 提供（设计文档 §6.2 路线 B），并通过本机 IPC 与本节点通信。
"""

from __future__ import annotations

import json

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mtc_interfaces.msg import RobotState
from mtc_interfaces.srv import GetMotionCapabilities, RequestStop

from .bridge_core import AuboBridge, RobotIdentity, make_bridge

ROBOT_STATE_TOPIC = '/mtc/robot/state'
CAPABILITIES_SRV = '/mtc/bridge/capabilities'
IDENTITY_SRV = '/mtc/bridge/identity'
REQUEST_STOP_SRV = '/mtc/bridge/request_stop'


class AuboBridgeNode(Node):
    """Bridge 节点：只做契约与安全语义，不实现真实 SDK 调用。"""

    def __init__(self) -> None:
        super().__init__('mtc_aubo_bridge')

        self.declare_parameter('mode', 'disabled')
        self.declare_parameter('expected_serial_number', '')
        self.declare_parameter('expected_controller_model', 'AUBO-S3')
        self.declare_parameter('identity_verified', False)
        self.declare_parameter('simulation_mode', True)

        mode = self.get_parameter('mode').get_parameter_value().string_value
        serial = self.get_parameter(
            'expected_serial_number').get_parameter_value().string_value
        model = self.get_parameter(
            'expected_controller_model').get_parameter_value().string_value
        verified = bool(self.get_parameter(
            'identity_verified').get_parameter_value().bool_value)

        self._bridge: AuboBridge = make_bridge(mode)
        # 身份白名单：留空或未核实 -> identity_check() 必然失败（安全默认）。
        self._bridge._expected_identity = RobotIdentity(  # noqa: SLF001 - 显式注入白名单
            controller_model=model, serial_number=serial, verified=verified)
        self._simulation_mode = bool(self.get_parameter(
            'simulation_mode').get_parameter_value().bool_value)
        self._mode = mode

        capabilities = self._bridge.capabilities()
        if capabilities.worker_name == 'disabled':
            self.get_logger().warning(
                f'Bridge 以 mode="{mode}" 启动：真实 AUBO S3 通路已禁用。'
                '不会连接控制柜、不会登录 SDK、不会发送运动命令、不会触发末端 IO。')
        else:
            self.get_logger().warning(
                f'Bridge 以 mode="{mode}" 启动：使用离线 Fake Worker（内存模拟），'
                '不连接任何真实设备。')
        if not (serial and verified):
            self.get_logger().warning(
                '设备身份白名单为空或未核实 -> identity_check() 将拒绝建立运动通路'
                '（符合安全默认）。')

        group = ReentrantCallbackGroup()
        self._state_pub = self.create_publisher(RobotState, ROBOT_STATE_TOPIC, 10)
        self._cap_srv = self.create_service(
            GetMotionCapabilities, CAPABILITIES_SRV,
            self._on_capabilities, callback_group=group)
        self._identity_srv = self.create_service(
            GetMotionCapabilities, IDENTITY_SRV,
            self._on_identity, callback_group=group)
        self._stop_srv = self.create_service(
            RequestStop, REQUEST_STOP_SRV, self._on_request_stop, callback_group=group)
        self.create_timer(1.0, self._publish_state, callback_group=group)

    # -- Service 回调 ------------------------------------------------------
    def _on_capabilities(self, _request, response) -> GetMotionCapabilities.Response:
        caps = self._bridge.capabilities()
        response.joint_goal_supported = bool(caps.joint_goal_supported)
        response.timed_trajectory_supported = bool(caps.timed_trajectory_supported)
        response.cartesian_motion_supported = bool(caps.cartesian_motion_supported)
        response.cancel_supported = bool(caps.cancel_supported)
        response.backend_name = str(caps.worker_name)
        response.detail = str(caps.detail)
        return response

    def _on_identity(self, _request, response) -> GetMotionCapabilities.Response:
        ok, why = self._bridge.identity_check()
        caps = self._bridge.capabilities()
        response.joint_goal_supported = bool(caps.joint_goal_supported and ok)
        response.timed_trajectory_supported = False
        response.cartesian_motion_supported = False
        response.cancel_supported = bool(caps.cancel_supported)
        response.backend_name = str(caps.worker_name)
        response.detail = json.dumps(
            {'identity_ok': ok, 'reason': why,
             'identity': self._bridge.worker.identity().as_dict()},
            ensure_ascii=False)
        return response

    def _on_request_stop(self, request, response) -> RequestStop.Response:
        result = self._bridge.request_stop(request.reason)
        response.request_delivered = bool(result.accepted)
        response.message = result.message
        self.get_logger().warning(
            f'收到软件停止请求 reason="{request.reason}"；delivered={result.accepted}，'
            f'stop_confirmed={result.stop_confirmed}。'
            '注意：送达不等于已停稳，且软件停止请求不能替代实体急停。')
        return response

    # -- 状态发布 ----------------------------------------------------------
    def _publish_state(self) -> None:
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        caps = self._bridge.capabilities()
        identity_ok, identity_reason = self._bridge.identity_check()
        msg.communication_ok = bool(caps.worker_name != 'disabled')
        msg.robot_ready = bool(identity_ok and not self._bridge.motion_state_unknown)
        msg.motion_active = False
        msg.safety_normal = not self._bridge.motion_state_unknown
        msg.stop_confirmed = bool(self._bridge.stop_confirmed)
        msg.simulation_mode = bool(self._simulation_mode)
        msg.controller_mode = str(caps.worker_name)
        msg.detail = json.dumps(
            {'mode': self._mode, 'identity_ok': identity_ok,
             'identity_reason': identity_reason,
             'motion_state_unknown': self._bridge.motion_state_unknown},
            ensure_ascii=False)
        self._state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AuboBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':  # pragma: no cover
    main()
