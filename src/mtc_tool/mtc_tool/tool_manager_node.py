"""``tool_manager`` 节点：``ToolManager`` 的 rclpy 外壳（薄适配层）。

- 发布：``/mtc/tool/state``（``mtc_interfaces/msg/ToolState``）
- 服务：``/mtc/tool/trigger_unlock``（``mtc_interfaces/srv/TriggerUnlock``）

安全默认值：

- 解锁 IO 默认使用 :class:`~mtc_tool.io_port.DisabledUnlockIo`，启动日志明确告警
  「真实 IO 已禁用」；``unlock_io_backend: mock`` 仅在 ``simulation_mode: true``
  时生效，纯离线使用。
- ``allow_real_io`` 参数保留但**无用**：即使被置为 true 也只记录错误并继续使用
  禁用端口（真实 IO 驱动尚未实现，也未被授权）。

注意：本节点不负责获取落座/卸载证据。真实联调时须由操作层把经过验证的证据写入
``ToolManager``（当前由 ``seated_verified`` / ``unloaded`` 参数充当离线占位输入，
见包 README「未实现点」）。
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from mtc_interfaces.msg import ErrorCodes as ErrorCodesMsg
from mtc_interfaces.msg import ToolState
from mtc_interfaces.srv import TriggerUnlock

from . import codes
from .io_port import DisabledUnlockIo, MockUnlockIo, UnlockIoPort
from .manager import ToolManager
from .strategy import MagneticLatchV2, PassiveHookV1, ToolStrategy

NODE_NAME = "mtc_tool_manager"
STATE_TOPIC = "/mtc/tool/state"
UNLOCK_SERVICE = "/mtc/tool/trigger_unlock"


def _error_constant(name: str, fallback: int) -> int:
    """优先使用 ``mtc_interfaces/msg/ErrorCodes`` 提供的常量。"""

    return int(getattr(ErrorCodesMsg, name, fallback))


def build_strategy(tool_type: str) -> ToolStrategy:
    """按 ``tool_type`` 构造策略；未知类型抛 ``ValueError``。"""

    if tool_type == codes.TOOL_TYPE_PASSIVE_HOOK_V1:
        return PassiveHookV1()
    if tool_type == codes.TOOL_TYPE_MAGNETIC_LATCH_V2:
        return MagneticLatchV2()
    raise ValueError("未知 tool_type: %r" % (tool_type,))


class ToolManagerNode(Node):
    """工具层统一管理节点。"""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        self.declare_parameter("tool_type", codes.TOOL_TYPE_PASSIVE_HOOK_V1)
        self.declare_parameter("unlock_pulse_ms", 300)
        self.declare_parameter("unlock_io_backend", "disabled")
        self.declare_parameter("simulation_mode", False)
        self.declare_parameter("allow_real_io", False)
        self.declare_parameter("state_publish_period_s", 0.2)
        # 离线占位证据输入：真实联调必须替换为经感知/传感器验证的来源。
        self.declare_parameter("seated_verified", False)
        self.declare_parameter("unloaded", False)

        tool_type = str(self.get_parameter("tool_type").value)
        self._strategy = build_strategy(tool_type)
        self._io = self._make_unlock_io()
        self._manager = ToolManager(strategy=self._strategy, unlock_io=self._io)
        self._manager.mark_seated(False)
        self._manager.mark_unloaded(False)

        self._pub = self.create_publisher(ToolState, STATE_TOPIC, 10)
        self._srv = self.create_service(TriggerUnlock, UNLOCK_SERVICE, self._on_trigger_unlock)

        period = float(self.get_parameter("state_publish_period_s").value)
        if period <= 0.0:
            self.get_logger().warn("state_publish_period_s<=0，回退到 0.2s")
            period = 0.2
        self._timer = self.create_timer(period, self._publish_state)

        self.get_logger().warn(
            "真实电磁解锁 IO 已禁用：unlock_io=%s；tools=%s；所有解锁请求都不会驱动真实硬件。"
            % (self._io.name, self._strategy.tool_type)
        )
        if self._strategy.requires_unlock_pulse():
            self.get_logger().warn(
                "当前工具 %s 需要解锁脉冲，联锁要求 seated_verified=True 且 unloaded=True。"
                % self._strategy.tool_type
            )
        else:
            self.get_logger().info(
                "当前工具 %s 为纯机械舌规：不使用电磁解锁 IO。" % self._strategy.tool_type
            )
        self._publish_state()

    # ------------------------------------------------------------------
    def _make_unlock_io(self) -> UnlockIoPort:
        allow_real_io = bool(self.get_parameter("allow_real_io").value)
        if allow_real_io:
            self.get_logger().error(
                "allow_real_io=true 被忽略：真实 IO 驱动未实现且未获授权，继续使用 DisabledUnlockIo。"
            )
        backend = str(self.get_parameter("unlock_io_backend").value).lower()
        simulation_mode = bool(self.get_parameter("simulation_mode").value)
        if backend == "mock" and simulation_mode:
            self.get_logger().warn(
                "unlock_io_backend=mock 且 simulation_mode=true：使用 MockUnlockIo（仅离线，不含真实 IO）。"
            )
            return MockUnlockIo(mode=MockUnlockIo.MODE_ACCEPT)
        if backend == "mock":
            self.get_logger().error(
                "请求 mock 解锁后端但 simulation_mode=false：拒绝，回退到 DisabledUnlockIo。"
            )
        return DisabledUnlockIo(strict=True)

    # ------------------------------------------------------------------
    def _publish_state(self) -> None:
        status = self._manager.status()
        msg = ToolState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tool"
        msg.state = int(status.state)
        msg.evidence_level = int(status.evidence_level)
        msg.verified = bool(status.verified)
        msg.tool_type = str(status.tool_type)
        msg.object_id = str(status.object_id)
        msg.detail = str(status.detail)
        self._pub.publish(msg)

    def _current_gates(self) -> tuple:
        """读取离线占位证据输入（真实联调须替换为已验证来源）。"""

        return (
            bool(self.get_parameter("seated_verified").value),
            bool(self.get_parameter("unloaded").value),
        )

    def _on_trigger_unlock(
        self, request: TriggerUnlock.Request, response: TriggerUnlock.Response
    ) -> TriggerUnlock.Response:
        pulse_ms = int(request.pulse_ms)
        if pulse_ms <= 0:
            pulse_ms = int(self.get_parameter("unlock_pulse_ms").value)
            self.get_logger().warn("请求未给出有效 pulse_ms，使用默认值 %d ms。" % pulse_ms)

        seated_verified, unloaded = self._current_gates()
        outcome = self._manager.trigger_unlock(
            request_id=str(request.request_id),
            pulse_ms=pulse_ms,
            seated_verified=seated_verified,
            unloaded=unloaded,
        )
        response.accepted = bool(outcome.accepted)
        response.message = "[%s=%d] %s" % (
            codes.error_name(outcome.error_code),
            outcome.error_code,
            outcome.message,
        )
        if outcome.accepted:
            self.get_logger().info(
                "解锁脉冲请求已受理（request_id=%s, io_status=%s）：受理不等于解锁成功。"
                % (request.request_id, outcome.io_status)
            )
        else:
            interlock = _error_constant("SAFETY_INTERLOCK", codes.ERROR_SAFETY_INTERLOCK)
            unlock_failed = _error_constant("UNLOCK_FAILED", codes.ERROR_UNLOCK_FAILED)
            if outcome.error_code == interlock:
                self.get_logger().warn(
                    "解锁请求被安全联锁拒绝（request_id=%s, SAFETY_INTERLOCK=%d）：%s"
                    % (request.request_id, interlock, outcome.message)
                )
            elif outcome.error_code == unlock_failed:
                self.get_logger().error(
                    "解锁失败（request_id=%s, UNLOCK_FAILED=%d）：%s"
                    % (request.request_id, unlock_failed, outcome.message)
                )
            else:
                self.get_logger().warn(
                    "解锁请求被拒绝（request_id=%s, code=%s=%d）：%s"
                    % (
                        request.request_id,
                        codes.error_name(outcome.error_code),
                        outcome.error_code,
                        outcome.message,
                    )
                )
        self._publish_state()
        return response


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node: Optional[ToolManagerNode] = None
    try:
        node = ToolManagerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
