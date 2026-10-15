"""Task FSM 的 ROS 2 Action Server 外壳（rclpy / ROS 2 Humble 语法）。

职责
----
* 提供 Action ``/mtc/task/execute``（类型 ``mtc_interfaces/action/ExecuteTask``）；
* 发布 ``/mtc/task/state``（``mtc_interfaces/msg/TaskState``），
  QoS = RELIABLE + KEEP_LAST(10)；
* 把 ROS 消息翻译成 :mod:`mtc_task.task_fsm` 的事件，并执行 FSM 返回的动作意图
  （调用 ``PickPlace`` Action Client ``/mtc/manipulation/pick_place``、
  取消子目标、发布状态、请求停止、锁定动作）；
* **本文件不做任何真实机械臂操作**，仅通过 Action 接口交互。

依赖注入
--------
:class:`TaskExecutorNode` 的构造函数允许注入：

* ``clock``：返回秒级浮点的可调用对象（默认 ``node.get_clock().now()``）；
* ``pickplace_client``：具有 ``send_goal_async`` / ``cancel_all`` 的对象；
* ``perception_provider``：``(color) -> [(object_id, ring_pose_valid, confidence), ...]``；
* ``state_publisher``：替换状态话题发布；
* ``fsm_config``：:class:`~mtc_task.task_fsm.FsmsConfig`。

注入后即可在**不启动 ROS 图**的情况下做离线联调与回归测试。
注意：``import rclpy`` 只在运行节点时才需要；FSM 核心逻辑全在
:mod:`mtc_task.task_fsm`，不依赖 ROS。

运行（真实 ROS 2 环境，当前机器仅有 Humble 且未编译验证）::

    ros2 run mtc_task task_executor --ros-args -p task_timeout_ms:=120000

安全声明
--------
本包默认 Mock/离线；编译或运行本节点**不被视为**已授权连接或驱动真实 AUBO S3。
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from mtc_interfaces.action import ExecuteTask, PickPlace
from mtc_interfaces.msg import TaskState

from .task_fsm import (
    ActionKind,
    ActionRequest,
    ErrorCodes,
    Event,
    FsmsConfig,
    TaskFsm,
    TaskStateName,
)

#: Action 名（设计文档 §4.2 接口总表约定）
TASK_ACTION_NAME = "/mtc/task/execute"
PICKPLACE_ACTION_NAME = "/mtc/manipulation/pick_place"
TASK_STATE_TOPIC = "/mtc/task/state"

#: §4.11：任务状态话题 QoS = RELIABLE + KEEP_LAST(10)
TASK_STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class PickPlaceRosClient:
    """``/mtc/manipulation/pick_place`` 的薄封装（Action Client）。

    与 :class:`FakePickPlaceClient` 保持相同的鸭子类型接口，便于注入替换。
    """

    def __init__(self, node: Node, action_name: str = PICKPLACE_ACTION_NAME) -> None:
        self._node = node
        self._client = ActionClient(node, PickPlace, action_name)
        self._goal_handles: List[Any] = []

    def wait_for_server(self, timeout_sec: float = 1.0) -> bool:
        return bool(self._client.wait_for_server(timeout_sec=timeout_sec))

    def send_goal_async(
        self,
        payload: Dict[str, Any],
        on_feedback: Optional[Callable[[Any], None]] = None,
        on_result: Optional[Callable[[Any], None]] = None,
    ) -> Any:
        goal = PickPlace.Goal()
        goal.request_id = str(payload.get("request_id", ""))
        goal.object_id = str(payload.get("object_id", ""))
        goal.expected_color = int(payload.get("expected_color") or 0)
        goal.target_slot = str(payload.get("target_slot", ""))
        goal.placement_stability_sec = float(payload.get("placement_stability_sec") or 0.0)

        future = self._client.send_goal_async(
            goal,
            feedback_callback=(lambda msg: on_feedback(msg)) if on_feedback else None,
        )
        future.add_done_callback(lambda fut: self._on_goal_response(fut, on_result))
        return future

    def _on_goal_response(self, future: Any, on_result: Optional[Callable[[Any], None]]) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().warn("PickPlace goal rejected by server")
            if on_result is not None:
                on_result({"success": False, "placement_verified": False,
                           "error_code": ErrorCodes.EXECUTION_REJECTED,
                           "message": "pick_place goal rejected"})
            return
        self._goal_handles.append(goal_handle)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_result(fut, on_result))

    def _on_result(self, future: Any, on_result: Optional[Callable[[Any], None]]) -> None:
        wrapped = future.result()
        result = getattr(wrapped, "result", None)
        if on_result is not None and result is not None:
            on_result(
                {
                    "success": bool(result.success),
                    "placement_verified": bool(result.placement_verified),
                    "error_code": int(result.error_code),
                    "message": str(result.message),
                }
            )

    def cancel_all(self) -> None:
        for goal_handle in list(self._goal_handles):
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:  # pragma: no cover - 运行时防御
                self._node.get_logger().warn("cancel_goal_async failed: %s" % exc)
        self._goal_handles.clear()


class TaskExecutorNode(Node):
    """整轮任务 FSM 的 ROS 2 节点（薄外壳）。"""

    def __init__(
        self,
        *,
        clock: Optional[Callable[[], float]] = None,
        pickplace_client: Optional[Any] = None,
        perception_provider: Optional[Callable[[int], Sequence[Any]]] = None,
        state_publisher: Optional[Callable[[Any], None]] = None,
        fsm_config: Optional[FsmsConfig] = None,
        node_name: str = "task_executor",
        **node_kwargs: Any,
    ) -> None:
        super().__init__(node_name, **node_kwargs)

        # --- 参数（含安全默认值；不含任何凭据/地址）-----------------------
        self.declare_parameter("recovery_max_attempts", 1)
        self.declare_parameter("state_scan_timeout_s", 5.0)
        self.declare_parameter("state_execute_item_timeout_s", 60.0)
        self.declare_parameter("state_record_result_timeout_s", 2.0)
        self.declare_parameter("state_validate_task_timeout_s", 2.0)
        self.declare_parameter("state_cancel_pending_timeout_s", 5.0)
        self.declare_parameter("state_recovery_timeout_s", 10.0)
        self.declare_parameter("state_self_check_timeout_s", 10.0)
        self.declare_parameter("watchdog_period_s", 0.05)
        self.declare_parameter("placement_stability_sec_default", 3.0)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("task_action_name", TASK_ACTION_NAME)
        self.declare_parameter("state_topic", TASK_STATE_TOPIC)
        self.declare_parameter("pickplace_action_name", PICKPLACE_ACTION_NAME)

        # --- 依赖注入 -----------------------------------------------------
        # Node._clock belongs to rclpy; keep the FSM's callable separate.
        self._fsm_clock: Callable[[], float] = clock or self._ros_clock
        self._perception_provider = perception_provider
        self._external_state_publisher = state_publisher

        config = fsm_config or self._config_from_parameters()
        self.fsm = TaskFsm(clock=self._fsm_clock, config=config)

        if pickplace_client is not None:
            self._pickplace = pickplace_client
            self._owns_pickplace = False
        else:
            self._pickplace = PickPlaceRosClient(
                self, str(self.get_parameter("pickplace_action_name").value)
            )
            self._owns_pickplace = True

        # --- ROS 接口 -----------------------------------------------------
        self._lock = threading.RLock()
        self._active_goal_handle: Optional[Any] = None
        self._goal_payload: Dict[str, Any] = {}

        state_qos = TASK_STATE_QOS
        self._state_pub = self.create_publisher(
            TaskState, str(self.get_parameter("state_topic").value), state_qos
        )
        self._action_server = ActionServer(
            self,
            ExecuteTask,
            str(self.get_parameter("task_action_name").value),
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        period = float(self.get_parameter("watchdog_period_s").value)
        self._watchdog = self.create_timer(
            period, self._on_watchdog, callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.get_logger().info(
            "mtc_task task_executor started (dry_run=%s); core FSM is pure-Python, "
            "no real robot I/O is performed by this node"
            % bool(self.get_parameter("dry_run").value)
        )
        self._publish_snapshot(self.fsm.snapshot(detail="node started"))

    # -- 参数 → 配置 ------------------------------------------------------

    def _config_from_parameters(self) -> FsmsConfig:
        from .task_fsm import DEFAULT_STATE_TIMEOUTS

        timeouts = dict(DEFAULT_STATE_TIMEOUTS)
        timeouts[TaskStateName.SELF_CHECK] = float(self.get_parameter("state_self_check_timeout_s").value)
        timeouts[TaskStateName.VALIDATE_TASK] = float(self.get_parameter("state_validate_task_timeout_s").value)
        timeouts[TaskStateName.SCAN_SCENE] = float(self.get_parameter("state_scan_timeout_s").value)
        timeouts[TaskStateName.EXECUTE_ITEM] = float(self.get_parameter("state_execute_item_timeout_s").value)
        timeouts[TaskStateName.RECORD_RESULT] = float(self.get_parameter("state_record_result_timeout_s").value)
        timeouts[TaskStateName.CANCEL_PENDING] = float(self.get_parameter("state_cancel_pending_timeout_s").value)
        timeouts[TaskStateName.RECOVERY] = float(self.get_parameter("state_recovery_timeout_s").value)
        return FsmsConfig(
            recovery_max_attempts=int(self.get_parameter("recovery_max_attempts").value),
            state_timeouts=timeouts,
        )

    def _ros_clock(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # -- Action Server 回调 ----------------------------------------------

    def _goal_callback(self, goal_request: Any) -> GoalResponse:
        """§7.2：同一时刻只允许一个进行中 Goal，第二个必须明确拒绝。"""
        with self._lock:
            if self.fsm.has_active_task:
                self.get_logger().warn(
                    "rejecting goal '%s': task '%s' already in progress in state %s"
                    % (
                        getattr(goal_request, "task_id", "?"),
                        self.fsm.current_task.task_id if self.fsm.current_task else "?",
                        self.fsm.state.value,
                    )
                )
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: Any) -> CancelResponse:
        """接受取消请求，但真正的取消结论由 FSM 的 CANCEL_PENDING 决定。"""
        with self._lock:
            self.get_logger().info("cancel requested; entering CANCEL_PENDING（等停稳确认）")
            return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: Any) -> Any:
        goal = goal_handle.request
        with self._lock:
            self._active_goal_handle = goal_handle
            self._goal_payload = {"goal": goal}

            if self.fsm.state is TaskStateName.INIT:
                self._dispatch(self.fsm.start())
            if self.fsm.state is TaskStateName.SELF_CHECK:
                self._dispatch(self.fsm.step(Event.SELF_CHECK_OK, {"source": "execute_request"}))

            # 再次确认可受理：避免 goal_callback 与 execute_callback 之间出现竞态
            if self.fsm.has_active_task:
                result = ExecuteTask.Result()
                result.success = False
                result.completed_count = self.fsm.completed_count
                result.completed_slots = list(self.fsm.completed_slots)
                result.error_code = ErrorCodes.EXECUTION_REJECTED
                result.message = "rejected: another task is already in progress"
                goal_handle.abort()
                return result

            accepted = self.fsm.submit_goal(goal)
            if accepted.goal_rejected:
                self._dispatch(accepted)
                result = ExecuteTask.Result()
                result.success = False
                result.completed_count = 0
                result.completed_slots = []
                result.error_code = int(
                    self.fsm.last_rejection.get("error_code", ErrorCodes.EXECUTION_REJECTED)
                )
                result.message = str(self.fsm.last_rejection.get("message", "goal rejected"))
                goal_handle.abort()
                self._active_goal_handle = None
                return result
            self._dispatch(accepted)

            validation = self.fsm.advance_validation()
            self._dispatch(validation)
            self._publish_feedback(goal_handle)

        # 任务级超时预算：FSM 的 task deadline 由 watchdog 处理；
        # 这里阻塞等待 FSM 走出终态或取消分支，由事件驱动。
        return self._await_terminal(goal_handle)

    def _await_terminal(self, goal_handle: Any, poll_sec: float = 0.02) -> Any:
        while rclpy.ok():
            with self._lock:
                state = self.fsm.state
                if state in (
                    TaskStateName.FINISHED,
                    TaskStateName.TASK_FAILED,
                    TaskStateName.CANCELLED,
                    TaskStateName.FAULT,
                ):
                    return self._finalize(goal_handle)
                if goal_handle.is_cancel_requested and state is not TaskStateName.CANCEL_PENDING:
                    self._dispatch(self.fsm.request_cancel())
                    self._publish_feedback(goal_handle)
            time.sleep(poll_sec)
        with self._lock:
            return self._finalize(goal_handle)

    def _finalize(self, goal_handle: Any) -> Any:
        outcome = self.fsm.build_outcome()
        result = ExecuteTask.Result()
        result.success = bool(outcome.success)
        result.completed_count = int(outcome.completed_count)
        result.completed_slots = list(outcome.completed_slots)
        result.error_code = int(outcome.error_code)
        result.message = str(outcome.message)

        if self.fsm.state is TaskStateName.FINISHED:
            goal_handle.succeed()
        elif self.fsm.state is TaskStateName.CANCELLED:
            goal_handle.canceled()
        else:
            goal_handle.abort()

        self._publish_snapshot(self.fsm.snapshot(detail=result.message))
        self._active_goal_handle = None
        self._goal_payload = {}
        return result

    # -- PickPlace 结果 / 感知回调 ---------------------------------------

    def on_pickplace_result(self, payload: Dict[str, Any]) -> None:
        """PickPlace Result 回调 → FSM 事件。"""
        with self._lock:
            self._dispatch(self.fsm.report_item_result(payload))
            if self.fsm.state is TaskStateName.RECORD_RESULT:
                if bool(payload.get("placement_verified")):
                    self._dispatch(self.fsm.record_completed_item())
                else:
                    self._dispatch(self.fsm.step(Event.RECORD_FAILED, {}))
            self._publish_feedback(self._active_goal_handle)

    def on_pickplace_feedback(self, payload: Dict[str, Any]) -> None:
        with self._lock:
            self.fsm.substate = str(payload.get("substate", self.fsm.substate))
            self._publish_feedback(self._active_goal_handle)

    def on_scan_tick(self) -> None:
        """周期扫描：在 SCAN_SCENE 中用注入的感知回调选择唯一候选。"""
        with self._lock:
            if self.fsm.state is not TaskStateName.SCAN_SCENE or self.fsm.current_task is None:
                return
            color = self.fsm.current_task.ordered_colors[self.fsm.current_index]
            candidates = list(self._perception_provider(color)) if self._perception_provider else []
            usable = [c for c in candidates if self._is_usable_candidate(c)]
            if len(usable) == 1:
                self._dispatch(self.fsm.report_target(self._candidate_id(usable[0]), expected_color=color))
            elif len(usable) == 0:
                self._dispatch(self.fsm.report_target_missing("no usable candidate for color %s" % color))
            else:
                self._dispatch(
                    self.fsm.report_target_missing(
                        "color %s is not unique (%d candidates)" % (color, len(usable))
                    )
                )

    @staticmethod
    def _candidate_id(candidate: Any) -> str:
        if isinstance(candidate, dict):
            return str(candidate.get("object_id", ""))
        return str(getattr(candidate, "object_id", ""))

    @staticmethod
    def _is_usable_candidate(candidate: Any) -> bool:
        """§4.5：ring_pose_valid 为 false 时禁止使用该候选生成运动目标。"""
        if isinstance(candidate, dict):
            return bool(candidate.get("ring_pose_valid")) and float(candidate.get("confidence", 0.0)) > 0.0
        return bool(getattr(candidate, "ring_pose_valid", False)) and float(
            getattr(candidate, "confidence", 0.0)
        ) > 0.0

    # -- 看门狗 -----------------------------------------------------------

    def _on_watchdog(self) -> None:
        with self._lock:
            if self.fsm.state is TaskStateName.SCAN_SCENE:
                self.on_scan_tick()
            result = self.fsm.check_timeouts()
            if result is not None:
                self._dispatch(result)
            deadline = self.fsm.check_task_deadline()
            if deadline is not None:
                self._dispatch(deadline)
            if self._active_goal_handle is not None and self._active_goal_handle.is_cancel_requested:
                if self.fsm.state is not TaskStateName.CANCEL_PENDING:
                    self._dispatch(self.fsm.request_cancel())

    # -- 分发 FSM 动作意图 ------------------------------------------------

    def _dispatch(self, result: Any) -> None:
        """执行 FSM 返回的动作意图（真实 IO 只发生在这里，且都是 ROS Action/Topic）。"""
        if result is None:
            return
        for action in result.actions:
            self._perform(action)

    def _perform(self, action: ActionRequest) -> None:
        if action.kind is ActionKind.PUBLISH_STATE:
            snapshot = action.payload.get("snapshot")
            self._publish_snapshot(snapshot if snapshot is not None else self.fsm.snapshot())
        elif action.kind is ActionKind.CALL_PICK_PLACE:
            self._call_pick_place(action.payload)
        elif action.kind is ActionKind.CANCEL_CHILD:
            self._cancel_child(action.payload)
        elif action.kind is ActionKind.REQUEST_STOP:
            self.get_logger().error("FSM requested stop: %s" % self.fsm.last_error_message)
        elif action.kind is ActionKind.LOCK_ACTIONS:
            self.get_logger().error("FSM locked further actions; human intervention required")

    def _call_pick_place(self, payload: Dict[str, Any]) -> None:
        if bool(self.get_parameter("dry_run").value):
            self.get_logger().info(
                "dry_run: would call PickPlace(request_id=%s, object_id=%s, color=%s, slot=%s)"
                % (
                    payload.get("request_id"),
                    payload.get("object_id"),
                    payload.get("expected_color"),
                    payload.get("target_slot"),
                )
            )
            return
        self._pickplace.send_goal_async(
            payload,
            on_feedback=lambda msg: self.on_pickplace_feedback(_pickplace_feedback_to_dict(msg)),
            on_result=self.on_pickplace_result,
        )

    def _cancel_child(self, payload: Dict[str, Any]) -> None:
        self.get_logger().warn("cancelling PickPlace child: %s" % payload.get("request_id"))
        cancel = getattr(self._pickplace, "cancel_all", None)
        if callable(cancel):
            cancel()
        # 真实停稳确认应来自运动执行层/工具反馈；此处不自行判定，交由
        # confirm_child_stopped() 在外壳收到证据时调用。

    def confirm_child_stopped(self, evidence: str = "") -> None:
        """外部（运动执行层/安全监控）确认子任务已停稳并核对负载状态后调用。"""
        with self._lock:
            self.get_logger().info("child stopped confirmed: %s" % (evidence or "no detail"))
            self._dispatch(self.fsm.confirm_child_stopped())

    # -- 发布 -------------------------------------------------------------

    def _publish_snapshot(self, snapshot: Any) -> None:
        if self._external_state_publisher is not None:
            self._external_state_publisher(snapshot)
            return
        msg = TaskState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = str(getattr(snapshot, "task_id", ""))
        msg.state = str(getattr(snapshot, "state", ""))
        msg.substate = str(getattr(snapshot, "substate", ""))
        msg.completed_count = int(getattr(snapshot, "completed_count", 0))
        msg.total_count = int(getattr(snapshot, "total_count", 0))
        msg.last_error_code = int(getattr(snapshot, "last_error_code", 0))
        msg.detail = str(getattr(snapshot, "detail", ""))
        self._state_pub.publish(msg)

    def _publish_feedback(self, goal_handle: Optional[Any]) -> None:
        if goal_handle is None:
            return
        feedback = ExecuteTask.Feedback()
        feedback.current_index = int(self.fsm.current_index)
        feedback.total_count = int(self.fsm.total_count)
        feedback.state = self.fsm.state.value
        feedback.current_slot = self.fsm.current_slot
        feedback.current_object_id = self.fsm.current_object_id
        feedback.substate = self.fsm.substate
        try:
            goal_handle.publish_feedback(feedback)
        except Exception as exc:  # pragma: no cover - 目标已终结时会抛错
            self.get_logger().debug("publish_feedback skipped: %s" % exc)


def _pickplace_feedback_to_dict(msg: Any) -> Dict[str, Any]:
    feedback = getattr(msg, "feedback", msg)
    return {
        "substate": str(getattr(feedback, "substate", "")),
        "progress": float(getattr(feedback, "progress", 0.0)),
        "object_attached_estimated": bool(getattr(feedback, "object_attached_estimated", False)),
        "detail": str(getattr(feedback, "detail", "")),
    }


def main(args: Optional[List[str]] = None) -> None:
    """``task_executor`` 可执行文件入口。"""
    rclpy.init(args=args)
    node = TaskExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:  # pragma: no cover - 交互式中断
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
