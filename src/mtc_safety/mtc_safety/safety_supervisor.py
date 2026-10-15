"""mtc_safety.safety_supervisor —— 安全联锁与故障监控的 rclpy 外壳。

职责边界（务必与 ``mtc_motion_execution`` 区分）：

- 本节点**只发软件停止请求**（``/mtc/motion/request_stop``，``RequestStop.srv``）；
  ``request_delivered=true`` 只表示请求送达执行适配层，**不代表已停稳**。
- 真实停稳由执行层确认，并通过 ``RobotState.stop_confirmed`` 回报；
  本节点只采信该字段（以及工具状态等独立证据），绝不因为服务返回成功就宣告安全。
- 远程进程结束、拔线、``Ctrl+C``、以及本节点的任何软件停止请求，
  **都不等价于实体急停**。

离线可测性：联锁判定逻辑全部在 ``mtc_safety.interlocks``（纯 Python）。
本模块 import rclpy，因此**不得**被离线测试 import；但本节点支持注入
``stop_client``（任何 ``adapt_stop_client`` 可适配的 fake 对象），
在真机以外的集成测试中无需真实 ROS 服务。
"""

from __future__ import annotations

import json
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from mtc_interfaces.msg import RobotState, ToolState
from mtc_interfaces.srv import RequestStop

from .interlocks import (
    DeviceState,
    InterlockGuard,
    WatchdogConfig,
    WatchdogCore,
    adapt_stop_client,
)

DEFAULT_ROBOT_STATE_TIMEOUT_SEC = 0.5
DEFAULT_TOOL_STATE_TIMEOUT_SEC = 1.0
DEFAULT_STOP_CONFIRM_TIMEOUT_SEC = 2.0
DEFAULT_WATCHDOG_PERIOD_SEC = 0.1
DEFAULT_STATE_TTL_SEC = 0.5

ROBOT_STATE_TOPIC = '/mtc/robot/state'
TOOL_STATE_TOPIC = '/mtc/tool/state'
INTERLOCK_STATE_TOPIC = '/mtc/safety/interlock_state'
REQUEST_STOP_SERVICE = '/mtc/motion/request_stop'


def make_stop_request_factory():
    """返回构造 ``RequestStop.Request`` 的工厂（延迟到运行时才 import 类型）。"""

    def _factory(reason: str):
        request = RequestStop.Request()
        request.reason = str(reason)
        return request

    return _factory


class SafetySupervisor(Node):
    """安全监控节点：状态采集 → 联锁判定 → 停止请求 → 结构化日志。"""

    def __init__(self,
                 node_name: str = 'safety_supervisor',
                 stop_client=None,
                 clock=None,
                 watchdog_config: Optional[WatchdogConfig] = None,
                 expected_controller_modes: Sequence[str] = (),
                 expect_simulation_mode: Optional[bool] = None,
                 state_ttl_sec: float = DEFAULT_STATE_TTL_SEC,
                 watchdog_period_sec: float = DEFAULT_WATCHDOG_PERIOD_SEC) -> None:
        super().__init__(node_name)

        self.declare_parameter('robot_state_timeout_sec', DEFAULT_ROBOT_STATE_TIMEOUT_SEC)
        self.declare_parameter('tool_state_timeout_sec', DEFAULT_TOOL_STATE_TIMEOUT_SEC)
        self.declare_parameter('stop_confirm_timeout_sec', DEFAULT_STOP_CONFIRM_TIMEOUT_SEC)
        self.declare_parameter('state_ttl_sec', state_ttl_sec)
        self.declare_parameter('watchdog_period_sec', watchdog_period_sec)
        self.declare_parameter('expected_controller_modes',
                               list(expected_controller_modes))
        self.declare_parameter('expect_simulation_mode',
                               -1 if expect_simulation_mode is None
                               else int(bool(expect_simulation_mode)))

        config = watchdog_config or WatchdogConfig(
            robot_state_timeout_sec=float(self.get_parameter(
                'robot_state_timeout_sec').value),
            tool_state_timeout_sec=float(self.get_parameter(
                'tool_state_timeout_sec').value),
            stop_confirm_timeout_sec=float(self.get_parameter(
                'stop_confirm_timeout_sec').value),
        )

        self.guard = InterlockGuard(
            device_state=DeviceState.DISCONNECTED,
            controller_mode='',
            expected_modes=tuple(self.get_parameter(
                'expected_controller_modes').value or ()),
            expect_simulation=None if int(self.get_parameter(
                'expect_simulation_mode').value) < 0 else bool(
                    int(self.get_parameter('expect_simulation_mode').value)),
            state_ttl_sec=float(self.get_parameter('state_ttl_sec').value),
            clock=clock,
            link_ok=False,
            stop_confirmed=False,
        )

        # 停止请求客户端：可为真实 rclpy ServiceClient，也可为注入的 fake。
        if stop_client is None:
            self._stop_client = self.create_client(RequestStop, REQUEST_STOP_SERVICE)
        else:
            self._stop_client = stop_client
        requester = adapt_stop_client(self._stop_client,
                                      request_factory=make_stop_request_factory())

        self.watchdog = WatchdogCore(config=config, stop_requester=requester,
                                     clock=clock, guard=self.guard)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(RobotState, ROBOT_STATE_TOPIC, self._on_robot_state, qos)
        self.create_subscription(ToolState, TOOL_STATE_TOPIC, self._on_tool_state, qos)
        self._interlock_pub = self.create_publisher(RobotState, INTERLOCK_STATE_TOPIC, qos)

        period = float(self.get_parameter('watchdog_period_sec').value)
        self.create_timer(max(period, 0.01), self._on_watchdog_timer)
        self.get_logger().info(
            f'mtc_safety 已启动：{ROBOT_STATE_TOPIC} / {TOOL_STATE_TOPIC} → '
            f'{REQUEST_STOP_SERVICE}；超时参数={json.dumps(config.as_dict())}')

    # -- 回调 --------------------------------------------------------------
    def _on_robot_state(self, msg: RobotState) -> None:
        self.watchdog.on_robot_state(msg)
        self._publish_interlock_state(msg)

    def _on_tool_state(self, msg: ToolState) -> None:
        self.watchdog.on_tool_state(msg)
        if int(msg.state) == ToolState.STATE_FAULT:
            self.get_logger().error(
                f'工具故障：state=FAULT detail={msg.detail}；FAULT 锁定只能人工解除')

    def _on_watchdog_timer(self) -> None:
        for event in self.watchdog.tick():
            self._log_event(event)
        self._publish_interlock_state(None)

    # -- 停止请求 ----------------------------------------------------------
    def request_stop(self, reason: str, force: bool = False) -> Optional[Dict[str, object]]:
        """发出软件停止请求（送达 != 停稳）。返回结构化日志事件。"""
        event = self.watchdog.request_stop(reason, force=force)
        if event is not None:
            self._log_event(event)
        return event

    def note_stop_delivered(self, delivered: Optional[bool], message: str = '') -> None:
        """异步服务回执：只更新送达状态，绝不置位 stop_confirmed。"""
        self._log_event(self.watchdog.note_stop_delivery(delivered, message))

    # -- 人工入口 ----------------------------------------------------------
    def clear_fault_by_human(self, note: str) -> Tuple[bool, str]:
        """FAULT 锁定的唯一解除入口（必须由人工显式调用并给出说明）。

        刻意**不**暴露为 ROS 服务：远程调用不能替代现场人工核验。
        """
        ok, message = self.guard.fault_latch.clear_by_human(note)
        if ok:
            self.get_logger().warn(f'安全监控：{message}')
        else:
            self.get_logger().error(f'安全监控：{message}')
        return ok, message

    def clear_motion_unknown_by_human(self, note: str) -> Tuple[bool, str]:
        """人工确认现场状态后解除「运动状态未知」锁定（重发仍需 stop_confirmed）。"""
        result = self.guard.clear_unknown_after_manual_confirmation(note)
        if result[0]:
            self.watchdog.note_motion_state_unknown(False, note)
        return result

    # -- 日志与发布 --------------------------------------------------------
    def _log_event(self, event: Dict[str, object]) -> None:
        payload = dict(event)
        payload['stop_requested'] = self.watchdog.stop_requested
        payload['stop_confirmed'] = self.watchdog.stop_confirmed
        payload['timeouts'] = self.watchdog.config.as_dict()
        self.get_logger().warn(json.dumps(payload, ensure_ascii=False, sort_keys=True))

    def _publish_interlock_state(self, robot_msg: Optional[RobotState]) -> None:
        out = RobotState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.communication_ok = bool(robot_msg.communication_ok) if robot_msg else False
        out.robot_ready = bool(robot_msg.robot_ready) if robot_msg else False
        out.motion_active = bool(robot_msg.motion_active) if robot_msg else False
        out.safety_normal = (not self.guard.fault_latched
                             and self.guard.device_state is not DeviceState.FAULT)
        out.stop_confirmed = bool(self.watchdog.stop_confirmed)
        out.simulation_mode = bool(robot_msg.simulation_mode) if robot_msg else False
        out.controller_mode = str(robot_msg.controller_mode) if robot_msg else ''
        out.detail = json.dumps({
            'device_state': self.guard.device_state.value,
            'payload': self.guard.payload.value,
            'fault_latched': self.guard.fault_latched,
            'motion_state_unknown': self.guard.motion_state_unknown,
            'stop_requested': self.watchdog.stop_requested,
            'stop_confirmed': self.watchdog.stop_confirmed,
            'stop_delivered': self.watchdog.last_delivery,
            'timeouts': self.watchdog.config.as_dict(),
            'allow_motion': self.guard.allow_motion().allowed,
        }, ensure_ascii=False, sort_keys=True)
        self._interlock_pub.publish(out)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[SafetySupervisor] = None
    try:
        node = SafetySupervisor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 注意：Ctrl+C 停止本进程 **不等于** 实体急停，也不保证机械臂停下。
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
