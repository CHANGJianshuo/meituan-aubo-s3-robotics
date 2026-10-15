"""mtc_motion_execution 的 rclpy 节点外壳。

所有决策逻辑都在 `mtc_motion_execution.executor_core`（不依赖 rclpy），
本文件只负责 ROS 2 消息/服务/Action 的编解码与线程安全调用。

安全默认值：`backend` 参数默认 `mock`；任何非 `mock` 取值都会落到
`DisabledMotionBackend`，绝不静默启用真实通路。
"""

from __future__ import annotations

import math
import threading
from typing import List

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from mtc_interfaces.action import ExecuteJointMove
from mtc_interfaces.msg import ErrorCodes, RobotState
from mtc_interfaces.srv import GetMotionCapabilities, RequestStop

from .backends import Clock, ValidationConfig
from .executor_core import MotionExecutorCore

MOTION_EXECUTE_ACTION = '/mtc/motion/execute_joint_move'
MOTION_CAPABILITIES_SRV = '/mtc/motion/capabilities'
MOTION_REQUEST_STOP_SRV = '/mtc/motion/request_stop'
ROBOT_STATE_TOPIC = '/mtc/robot/state'


def _state_qos() -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


class MotionExecutorNode(Node):
    """运动执行节点（默认 Mock）。"""

    def __init__(self) -> None:
        super().__init__('motion_executor')

        self.declare_parameter('backend', 'mock')
        self.declare_parameter('expected_joint_names', [
            'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
            'wrist1_joint', 'wrist2_joint', 'wrist3_joint'])
        self.declare_parameter('max_velocity_scaling', 0.2)
        self.declare_parameter('max_acceleration_scaling', 0.2)
        self.declare_parameter('joint_limit_rad', [-3.0, 3.0] * 6)
        self.declare_parameter('robot_ready', True)
        self.declare_parameter('simulation_mode', True)

        backend_name = self.get_parameter('backend').get_parameter_value().string_value
        joint_names = list(self.get_parameter(
            'expected_joint_names').get_parameter_value().string_array_value)
        raw_limits = list(self.get_parameter(
            'joint_limit_rad').get_parameter_value().double_array_value)
        limits = []
        for index in range(0, max(0, len(raw_limits) - 1), 2):
            limits.append((float(raw_limits[index]), float(raw_limits[index + 1])))

        config = ValidationConfig(
            expected_joint_names=tuple(joint_names),
            max_velocity_scaling=float(self.get_parameter(
                'max_velocity_scaling').get_parameter_value().double_value),
            max_acceleration_scaling=float(self.get_parameter(
                'max_acceleration_scaling').get_parameter_value().double_value),
            joint_limit_rad=tuple(limits),
        )
        self._core = MotionExecutorCore(
            backend_name=backend_name, config=config, clock=Clock())
        self._lock = threading.Lock()

        capabilities = self._core.capabilities()
        self.get_logger().warning(
            f"运动后端 = {capabilities['backend_name']}；"
            f"joint_goal_supported={capabilities['joint_goal_supported']}；"
            f"timed_trajectory_supported={capabilities['timed_trajectory_supported']}；"
            f"cartesian_motion_supported={capabilities['cartesian_motion_supported']}。"
            '真实 AUBO S3 通路默认禁用，未经现场授权与实测不得启用。')

        group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            ExecuteJointMove,
            MOTION_EXECUTE_ACTION,
            execute_callback=self._on_execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=group,
        )
        self._cap_srv = self.create_service(
            GetMotionCapabilities, MOTION_CAPABILITIES_SRV,
            self._on_get_capabilities, callback_group=group)
        self._stop_srv = self.create_service(
            RequestStop, MOTION_REQUEST_STOP_SRV, self._on_request_stop, callback_group=group)
        self._robot_state_pub = self.create_publisher(RobotState, ROBOT_STATE_TOPIC, _state_qos())
        self._robot_ready = bool(self.get_parameter(
            'robot_ready').get_parameter_value().bool_value)
        self._simulation_mode = bool(self.get_parameter(
            'simulation_mode').get_parameter_value().bool_value)
        self.create_timer(0.5, self._publish_robot_state, callback_group=group)

    # -- Action 回调 -------------------------------------------------------
    def _on_goal(self, goal_request) -> GoalResponse:
        with self._lock:
            if self._core.motion_state_unknown:
                self.get_logger().error(
                    '拒绝 Goal：上一运动状态未知且未确认，禁止自动重发')
                return GoalResponse.REJECT
            if self._core.snapshot().active_command_id is not None:
                self.get_logger().error('拒绝 Goal：已有在途运动命令')
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        self.get_logger().warning('收到取消请求：进入取消流程，等待停稳确认后才算安全停止')
        return CancelResponse.ACCEPT

    def _on_execute(self, goal_handle) -> ExecuteJointMove.Result:
        goal = goal_handle.request
        result = ExecuteJointMove.Result()

        with self._lock:
            core_result = self._core.execute_joint_move(
                command_id=goal.command_id,
                joint_names=list(goal.joint_names),
                target_rad=list(goal.target_rad),
                velocity_scaling=float(goal.velocity_scaling),
                acceleration_scaling=float(goal.acceleration_scaling),
                timeout_ms=int(goal.timeout_ms),
            )

        result.success = bool(core_result.success)
        result.stop_confirmed = bool(core_result.stop_confirmed)
        result.error_code = int(core_result.error_code)
        result.message = core_result.message
        final = list(core_result.final_position_rad)
        padded: List[float] = [float(v) for v in final[:6]]
        padded += [math.nan] * (6 - len(padded))
        result.final_position_rad = padded

        feedback = ExecuteJointMove.Feedback()
        fb = self._core.backend.joint_feedback()
        pos = list(fb.position_rad)
        vel = list(fb.velocity_rad_s)
        feedback.actual_position_rad = [float(v) for v in pos[:6]] + [math.nan] * (6 - len(pos[:6]))
        feedback.actual_velocity_rad_s = [float(v) for v in vel[:6]] + [math.nan] * (6 - len(vel[:6]))
        feedback.progress = 1.0 if core_result.success else 0.0
        feedback.execution_state = core_result.exec_state.value
        try:
            goal_handle.publish_feedback(feedback)
        except Exception:  # pragma: no cover - 反馈失败不应掩盖主结果
            self.get_logger().warning('发布 Action 反馈失败')

        if core_result.success:
            goal_handle.succeed()
        elif core_result.motion_state_unknown:
            # 状态未知：中止并保持故障锁定，不做自动恢复。
            goal_handle.abort()
        else:
            goal_handle.abort()
        return result

    # -- Service 回调 ------------------------------------------------------
    def _on_get_capabilities(self, _request, response) -> GetMotionCapabilities.Response:
        caps = self._core.capabilities()
        response.joint_goal_supported = bool(caps['joint_goal_supported'])
        response.timed_trajectory_supported = bool(caps['timed_trajectory_supported'])
        response.cartesian_motion_supported = bool(caps['cartesian_motion_supported'])
        response.cancel_supported = bool(caps['cancel_supported'])
        response.backend_name = str(caps['backend_name'])
        response.detail = str(caps['detail'])
        return response

    def _on_request_stop(self, request, response) -> RequestStop.Response:
        with self._lock:
            delivered, message, _confirmed = self._core.request_stop(request.reason)
        response.request_delivered = bool(delivered)
        response.message = message
        self.get_logger().warning(
            f'收到软件停止请求，reason="{request.reason}"；'
            f'delivered={delivered}。注意：停止请求送达不等于已确认停稳，'
            '且软件停止请求不能替代实体急停。')
        return response

    # -- 状态发布 ----------------------------------------------------------
    def _publish_robot_state(self) -> None:
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        snapshot = self._core.snapshot()
        caps = self._core.capabilities()
        msg.communication_ok = bool(caps['backend_name'] != 'disabled')
        msg.robot_ready = bool(self._robot_ready and not snapshot.motion_state_unknown)
        msg.motion_active = snapshot.active_command_id is not None
        msg.safety_normal = not snapshot.motion_state_unknown
        msg.stop_confirmed = bool(snapshot.stop_confirmed)
        msg.simulation_mode = bool(self._simulation_mode)
        msg.controller_mode = str(caps['backend_name'])
        msg.detail = (
            f"exec_state={snapshot.exec_state.value}; "
            f"active_command={snapshot.active_command_id or '-'}; "
            f"last_error={snapshot.last_error_code}; "
            f"motion_state_unknown={snapshot.motion_state_unknown}")
        self._robot_state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionExecutorNode()
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
