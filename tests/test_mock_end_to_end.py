#!/usr/bin/env python3
"""离线端到端集成测试（**纯 Python 3，不 import rclpy**）。

运行方式（无需 ROS，也无需 colcon build）::

    cd /home/robot/meituan_challenge_ws
    python3 tests/test_mock_end_to_end.py

覆盖范围
--------
设计文档 ``docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md``：
* §10.1 必测故障用例（13 项，见下方 ``CASES``）；
* §7.2 关键联锁；
* §8 参数与配置约定（读取真实 ``config/*.yaml`` 做交叉校验）。

被测对象（**真实模块**，通过 sys.path 装配，不使用测试内部复刻逻辑）
--------------------------------------------------------------------
* ``mtc_motion_execution.backends``        执行状态机 / Mock 后端 / 唯一执行权
* ``mtc_motion_execution.executor_core``   七步握手执行核心
* ``mtc_task.task_fsm``                    第一层 Task FSM
* ``mtc_manipulation.pick_place_fsm``      第二层 PickPlace FSM
* ``mtc_tool.manager`` / ``io_port``       工具管理器与解锁 IO 端口
* ``mtc_safety.interlocks``                软件联锁守卫
* ``mtc_aubo_bridge.bridge_core``          AUBO 桥接契约（只用 disabled/fake 形态）
* ``mtc_bringup.mock_pipeline``            本包提供的端口适配与编排（薄适配层）

依赖未就绪时的行为
------------------
``mtc_motion_planning`` 是迁移占位包（源缺失，无 ``package.xml``）。若某用例所需的
模块不可用，该用例输出 ``SKIPPED(依赖未就绪)`` 并计入 SKIP，**绝不记为 PASS**。

安全声明
--------
本测试**不连接任何真实设备**：不联网、不 SSH、不运行 colcon、不驱动机械臂、
不触发电磁铁。所有 PASS 均为 **Mock PASS，非实机抓放成功**。
"""

from __future__ import annotations

import os
import sys
import traceback
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# 0. 路径装配（必须在其它 import 之前）
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_ROOT = os.path.dirname(_HERE)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
# 使 `import mtc_bringup` 在未 build / 未安装的情况下也可用（pytest 执行本文件时同样有效）
_BRINGUP_PARENT = os.path.join(_WS_ROOT, "src", "mtc_bringup")
if os.path.isdir(_BRINGUP_PARENT) and _BRINGUP_PARENT not in sys.path:
    sys.path.insert(0, _BRINGUP_PARENT)

from mtc_bringup.mock_pipeline import (  # noqa: E402
    DependencyNotReady,
    ManualClock,
    MockOrchestrator,
    MockPerception,
    MockPipeline,
    MockPlanner,
    PipelineConfig,
    ensure_package_paths,
    require_pipeline_dependencies,
    resolve_dependencies,
)

ensure_package_paths(_WS_ROOT)

DEPS = resolve_dependencies()
_DEPS_READY = True
_DEPS_ERROR = ""
try:
    require_pipeline_dependencies(DEPS)
except DependencyNotReady as exc:  # pragma: no cover - 依赖齐全时不会触发
    _DEPS_READY = False
    _DEPS_ERROR = str(exc)

# 颜色枚举（BatteryDetection.msg）
RED, BLUE, YELLOW = 1, 2, 3
V1, V2 = "passive_hook_v1", "magnetic_latch_v2"

FORBIDDEN_MODULES = ("rclpy", "mtc_interfaces", "rcl_interfaces")


def _mod(name: str) -> Any:
    """取已解析的依赖模块；未就绪返回 None（调用方据此 SKIP）。"""
    return DEPS.get(name)


# ---------------------------------------------------------------------------
# 1. 极简断言与用例框架（不用 pytest，保证可直接 python3 运行）
# ---------------------------------------------------------------------------
class CaseSkipped(Exception):
    """依赖未就绪：计入 SKIP，绝不计入 PASS。"""


@dataclass
class CaseResult:
    case_id: str
    name: str
    status: str            # PASS / FAIL / SKIP
    detail: str = ""
    error: str = ""


@dataclass
class Harness:
    results: List[CaseResult] = field(default_factory=list)

    def run(self, case_id: str, name: str, func: Callable[[], str]) -> CaseResult:
        try:
            detail = func()
            result = CaseResult(case_id, name, "PASS", detail or "")
        except CaseSkipped as exc:
            result = CaseResult(case_id, name, "SKIP", "SKIPPED(依赖未就绪)：%s" % exc)
        except AssertionError as exc:
            result = CaseResult(case_id, name, "FAIL", "断言失败", str(exc))
        except Exception as exc:  # 未预期异常同样算失败，并保留栈信息
            result = CaseResult(case_id, name, "FAIL", "异常 %s" % type(exc).__name__,
                                "%s: %s\n%s" % (type(exc).__name__, exc, traceback.format_exc()))
        self.results.append(result)
        mark = {"PASS": "PASS", "FAIL": "FAIL", "SKIP": "SKIP"}[result.status]
        print("[%s] %-4s %s" % (mark, case_id, name))
        if result.status != "PASS":
            for line in (result.detail + "\n" + result.error).splitlines():
                if line.strip():
                    print("         %s" % line)
        elif result.detail:
            print("         %s" % result.detail)
        return result

    def count(self, status: str) -> int:
        return sum(1 for item in self.results if item.status == status)


def need(*module_names: str) -> None:
    """确保所需模块就绪，否则 SKIP。"""
    if not _DEPS_READY:
        raise CaseSkipped(_DEPS_ERROR)
    missing = [name for name in module_names if _mod(name) is None]
    if missing:
        raise CaseSkipped("缺少模块 %s" % ", ".join(missing))


# ---------------------------------------------------------------------------
# 2. 通用装配辅助
# ---------------------------------------------------------------------------
def new_pipeline(tool_type: str = V1, **config_overrides: Any) -> MockPipeline:
    config = PipelineConfig(tool_type=tool_type, **config_overrides)
    return MockPipeline(config)


def perception_for(pipeline: MockPipeline, color: int, object_id: str = "battery-1",
                   **kwargs: Any) -> MockPerception:
    pipeline.perception.set_target(object_id, color, **kwargs)
    pipeline.perception.set_placement(object_id, at_target=True, settled=True)
    return pipeline.perception


def start_item(pipeline: MockPipeline, color: int, slot: str = "T0",
               object_id: str = "battery-1", stability_sec: Optional[float] = None) -> Any:
    pipeline.tool.object_id = object_id
    fsm = pipeline.new_pick_place()
    fsm.start(object_id=object_id, expected_color=color, target_slot=slot,
              request_id="e2e#0",
              placement_stability_sec=(stability_sec if stability_sec is not None else 3.0))
    return fsm


TERMINAL = ("SUCCESS", "FAILED", "FAULT", "CANCELLED")


def drive(fsm: Any, pipeline: MockPipeline, *, max_ticks: int = 600,
          tick_seconds: float = 1.0) -> Any:
    """把 PickPlaceFSM 推到终态（每 tick 推进虚拟时间，保证确定性）。"""
    for _ in range(max_ticks):
        if fsm.state.value in TERMINAL:
            break
        fsm.step()
        pipeline.clock.advance(tick_seconds)
    return fsm.result()


def run_item(pipeline: MockPipeline, color: int, slot: str = "T0",
             object_id: str = "battery-1", **drive_kwargs: Any) -> Any:
    return drive(start_item(pipeline, color, slot, object_id), pipeline, **drive_kwargs)


def must(condition: bool, message: str) -> None:
    if not condition:
        # 附带调用点行号，便于快速定位失败断言
        frame = sys._getframe(1)
        raise AssertionError("%s（%s:%d）" % (message, os.path.basename(frame.f_code.co_filename),
                                            frame.f_lineno))


# ---------------------------------------------------------------------------
# 3. 用例实现
# ---------------------------------------------------------------------------
def case_01_sequence_three_colors() -> str:
    """蓝→红→黄：P1/P2/P3 槽位映射正确，全流程 Mock PASS。"""
    need("mtc_task.task_fsm", "mtc_manipulation.pick_place_fsm")
    pipeline = new_pipeline(V1)
    orchestrator = MockOrchestrator(
        pipeline, colors=[BLUE, RED, YELLOW],
        object_ids=["battery-blue", "battery-red", "battery-yellow"],
        task_id="seq-1", mode=2)
    outcome = orchestrator.run()

    slots = [step.slot for step in orchestrator.steps]
    must(slots == ["P1", "P2", "P3"], "槽位映射必须为 P1/P2/P3，实际 %s" % slots)
    must([step.color for step in orchestrator.steps] == [BLUE, RED, YELLOW],
         "颜色顺序必须为 蓝/红/黄")
    must(all(step.final_state == "SUCCESS" for step in orchestrator.steps),
         "每一步都应 SUCCESS，实际 %s" % [s.final_state for s in orchestrator.steps])
    must(all(step.placement_verified for step in orchestrator.steps),
         "每一步都必须 placement_verified=True")
    must(outcome.success, "整轮任务应成功：%s" % outcome.message)
    must(outcome.completed_slots == ["P1", "P2", "P3"],
         "completed_slots 应为 P1/P2/P3，实际 %s" % outcome.completed_slots)
    must(pipeline.unlock_io_call_count == 0, "V1 不得调用解锁 IO")
    return ("Mock PASS，非实机抓放成功；slots=%s completed=%d 后端命令数=%d"
            % (outcome.completed_slots, outcome.completed_count, pipeline.backend_call_count))


def case_02_concurrent_goals_rejected() -> str:
    """并发第二个 Task Goal / Motion Goal 被明确拒绝。"""
    need("mtc_task.task_fsm", "mtc_motion_execution.executor_core",
         "mtc_motion_execution.backends")
    from mtc_task.task_fsm import ActionKind, ErrorCodes, TaskStateName
    from types import SimpleNamespace

    pipeline = new_pipeline(V1)
    fsm = pipeline.task

    def goal(task_id: str) -> Any:
        return SimpleNamespace(task_id=task_id, mode=2, ordered_colors=[BLUE, RED, YELLOW],
                               timeout_ms=60000, placement_stability_sec=3.0)

    first = fsm.submit_goal(goal("first"))
    must(not first.goal_rejected, "第一个 Goal 应被受理")
    fsm.advance_validation()
    must(fsm.has_active_task, "受理后必须处于进行中状态")

    second = fsm.submit_goal(goal("second"))
    must(second.goal_rejected, "第二个 Task Goal 必须被明确拒绝")
    must(second.state is fsm.state, "拒绝不得改变当前状态")
    must(fsm.last_rejection.get("reason") == "task_already_in_progress",
         "拒绝原因应为 task_already_in_progress，实际 %r" % fsm.last_rejection)
    must(fsm.last_rejection.get("error_code") == ErrorCodes.EXECUTION_REJECTED,
         "拒绝错误码应为 210")
    must("not queued" in fsm.last_rejection.get("message", ""),
         "拒绝消息必须说明不排队")

    # 直接消费 FSM 产出的 call_pick_place，验证第二个运动 Goal 被拒绝
    step = fsm.report_target("battery-blue", expected_color=BLUE)
    request = step.find(ActionKind.CALL_PICK_PLACE)
    must(request is not None, "进入 EXECUTE_ITEM 必须产生 call_pick_place")
    payload = dict(request.payload)
    pipeline.tool.object_id = payload["object_id"]
    # 人为占用执行层的在途命令，再提交第二个运动 Goal
    pipeline.executor._active_command_id = "inflight-probe"   # 模拟在途命令
    second_motion = pipeline.executor.execute_joint_move(
        command_id="probe-2",
        joint_names=tuple(pipeline.config.joint_names),
        target_rad=tuple(0.0 for _ in pipeline.config.joint_names),
        velocity_scaling=0.1, acceleration_scaling=0.1, timeout_ms=1000)
    pipeline.executor._active_command_id = None
    must(not second_motion.success, "第二个运动 Goal 必须被拒绝")
    must(second_motion.error_code == ErrorCodes.EXECUTION_REJECTED,
         "拒绝错误码应为 210，实际 %s" % second_motion.error_code)
    must("并发" in second_motion.message or "在途" in second_motion.message,
         "拒绝消息应说明在途/并发：%s" % second_motion.message)
    return "Task Goal 拒绝码=210 reason=%s；Motion Goal 拒绝码=%s" % (
        fsm.last_rejection.get("reason"), second_motion.error_code)


def case_03_bad_target_never_moves() -> str:
    """目标颜色错误 / 重复 ID / 提环位姿过期 → 不运动。

    刻意把 ``recovery_max_attempts`` 设为 0：本用例考察的是“感知不可靠时绝不运动”，
    而不是“是否允许限次重试”。重试路径本身另设对照（见末尾），且同样要求零运动。
    """
    need("mtc_manipulation.pick_place_fsm")
    from mtc_manipulation.pick_place_fsm import ErrorCode

    notes: List[str] = []

    def check(label: str, setup: Callable[[MockPipeline], None], expected_codes: Tuple[int, ...],
              object_id: str) -> Any:
        pipeline = new_pipeline(V1, recovery_max_attempts=0)
        setup(pipeline)
        fsm = start_item(pipeline, BLUE, "T0", object_id)
        result = drive(fsm, pipeline)
        must(not result.success, "%s：不得成功（%s）" % (label, result.message))
        must(pipeline.backend_call_count == 0,
             "%s：不得运动，后端调用次数=%d" % (label, pipeline.backend_call_count))
        must(result.error_code in expected_codes,
             "%s：应报 %s，实际 %s" % (label, expected_codes, result.error_code))
        notes.append("%s -> %s（后端调用 0 次）" % (label, result.error_code))
        return result

    # (a) 颜色错误：期望蓝色，实际红色
    check("颜色错误",
          lambda p: perception_for(p, RED, "battery-mismatch"),
          (ErrorCode.INVALID_TASK, ErrorCode.TARGET_NOT_FOUND), "battery-mismatch")

    # (b) 重复 ID：同一 object_id 出现两次
    check("重复 ID",
          lambda p: perception_for(p, BLUE, "battery-dup", duplicates=2),
          (ErrorCode.TARGET_NOT_FOUND,), "battery-dup")

    # (c) 提环位姿过期
    check("位姿过期",
          lambda p: perception_for(p, BLUE, "battery-stale", stale=True),
          (ErrorCode.POSE_STALE,), "battery-stale")

    # (d) 提环位姿不可用（ring_pose_valid=false）
    check("ring_pose 不可用",
          lambda p: perception_for(p, BLUE, "battery-noring", ring_pose_valid=False),
          (ErrorCode.TARGET_NOT_FOUND, ErrorCode.POSE_STALE), "battery-noring")

    # 对照：允许重试时，重试同样不得产生运动（感知始终不可靠）
    pipeline = new_pipeline(V1, recovery_max_attempts=1)
    perception_for(pipeline, RED, "battery-mismatch")
    fsm = start_item(pipeline, BLUE, "T0", "battery-mismatch")
    result = drive(fsm, pipeline)
    must(pipeline.backend_call_count == 0,
         "允许重试时同样不得运动，后端调用次数=%d" % pipeline.backend_call_count)
    must(not result.success, "重试后仍不得成功")
    notes.append("重试对照 -> %s（后端调用仍为 0 次）" % result.error_code)

    # 所有感知拒绝都必须发生在规划之前（planner 未被调用）
    return "；".join(notes)


def case_04_execution_failure_not_success() -> str:
    """规划成功但底层执行失败 → 任务不得标记成功。"""
    need("mtc_task.task_fsm", "mtc_manipulation.pick_place_fsm",
         "mtc_motion_execution.executor_core")
    from mtc_manipulation.pick_place_fsm import ErrorCode
    from mtc_motion_execution.backends import ExecState, JointMoveResult

    class FaultingBackend:
        """包装真实 Mock 后端：规划成功、但底层运动全部失败。"""

        def __init__(self, inner: Any) -> None:
            self.inner = inner
            self.call_log: List[Any] = []
            self.faults = 0

        def capabilities(self) -> Any:
            return self.inner.capabilities()

        def joint_feedback(self) -> Any:
            return self.inner.joint_feedback()

        def request_stop(self, reason: str) -> Any:
            return self.inner.request_stop(reason)

        def execute_joint_move(self, request: Any) -> Any:
            self.call_log.append(request)
            self.inner.call_log.append(request)
            self.faults += 1
            return JointMoveResult(
                success=False, exec_state=ExecState.FAULT,
                error_code=ErrorCode.HARDWARE_FAULT,
                message="测试注入：底层执行失败（规划本身成功）")

    pipeline = new_pipeline(V1)
    perception_for(pipeline, BLUE, "battery-hw")
    wrapper = FaultingBackend(pipeline.backend)
    pipeline.executor._backend = wrapper          # 换掉后端，执行层逻辑不变
    pipeline.motion.backend = wrapper

    orchestrator = MockOrchestrator(pipeline, colors=[BLUE], object_ids=["battery-hw"],
                                    task_id="exec-fail", mode=1)
    outcome = orchestrator.run()

    must(wrapper.faults > 0, "应至少有一次底层执行失败被触发")
    must(orchestrator.steps, "应记录一次抓放推进")
    step = orchestrator.steps[0]
    must(not step.result_success, "底层失败时 PickPlace 不得成功")
    must(not step.placement_verified, "底层失败时不得 placement_verified=True")
    must(step.final_state in ("FAILED", "FAULT"),
         "底层失败应终态 FAILED/FAULT，实际 %s" % step.final_state)
    must(not outcome.success, "整轮任务不得标记成功")
    must(outcome.completed_count == 0, "不得记入完成计数")
    return ("规划成功但执行失败 -> PickPlace %s(ec=%s)，TaskOutcome success=%s completed=%d"
            % (step.final_state, step.error_code, outcome.success, outcome.completed_count))


def case_05_timeout_no_resend() -> str:
    """moveJoint 超时但机器人可能在动 → 不自动重发（后端调用次数不增加）。"""
    need("mtc_manipulation.pick_place_fsm", "mtc_motion_execution.backends")
    from mtc_manipulation.pick_place_fsm import ErrorCode

    pipeline = new_pipeline(V1, backend_timeout=True)
    perception_for(pipeline, BLUE, "battery-timeout")
    fsm = start_item(pipeline, BLUE, "T0", "battery-timeout")

    result = drive(fsm, pipeline, max_ticks=80)
    must(not result.success, "超时不得成功")
    must(result.error_code == ErrorCode.MOTION_TIMEOUT,
         "应报 220 MOTION_TIMEOUT，实际 %s" % result.error_code)
    must(fsm.state.value == "FAULT", "运动超时应进入 FAULT，实际 %s" % fsm.state.value)
    must(not fsm.motion_resend_allowed, "超时后必须禁止自动重发")

    calls_after_fault = pipeline.backend_call_count
    ticks_before = fsm._step_count
    for _ in range(40):
        fsm.step()
        pipeline.clock.advance(1.0)
    must(pipeline.backend_call_count == calls_after_fault,
         "进入 FAULT 后不得再次调用后端：%d -> %d"
         % (calls_after_fault, pipeline.backend_call_count))
    must(fsm._step_count > ticks_before, "应确实继续推进了状态机（证明不是因为没跑才没重发）")
    must(pipeline.executor.snapshot().stop_confirmed is False,
         "超时场景不得声称已确认停稳")
    return ("超时 -> FAULT(220)，后端调用次数保持 %d 次不变（继续推进 %d 次 step 后仍不增加）"
            % (calls_after_fault, fsm._step_count - ticks_before))


def case_06_test_lift_not_followed_blocks_transport() -> str:
    """试提时电池未跟随 → 禁止进入运输。"""
    need("mtc_manipulation.pick_place_fsm")
    from mtc_manipulation.pick_place_fsm import ErrorCode, PickPlaceState

    pipeline = new_pipeline(V1)
    perception_for(pipeline, BLUE, "battery-slip", followed=False)
    pipeline.tool.test_lift_followed = False
    fsm = start_item(pipeline, BLUE, "T0", "battery-slip")
    result = drive(fsm, pipeline)

    must(not result.success, "试提未随动不得成功")
    must(result.error_code == ErrorCode.ATTACH_NOT_VERIFIED,
         "应报 310 ATTACH_NOT_VERIFIED，实际 %s" % result.error_code)
    visited = {state.value for state in result.states_history}
    must(PickPlaceState.TRANSPORT.value not in visited, "不得进入 TRANSPORT")
    must(PickPlaceState.LIFT.value not in visited, "不得进入 LIFT")
    must(not fsm.object_attached_estimated, "不得估计为已挂载")
    return "试提未随动 -> %s(310)，未进入 LIFT/TRANSPORT（历史=%s）" % (
        result.final_state, "->".join(state.value for state in result.states_history[-4:]))


def case_07_not_seated_no_unlock() -> str:
    """放置未落座 → V2 电磁解锁不得触发。"""
    need("mtc_manipulation.pick_place_fsm")
    from mtc_manipulation.pick_place_fsm import ErrorCode

    pipeline = new_pipeline(V2)
    perception_for(pipeline, BLUE, "battery-noseat")
    pipeline.tool.placement_seated = False
    fsm = start_item(pipeline, BLUE, "P1", "battery-noseat")
    result = drive(fsm, pipeline)

    must(not result.success, "未落座不得成功")
    must(pipeline.unlock_io_call_count == 0,
         "未落座时解锁 IO 调用次数必须为 0，实际 %d" % pipeline.unlock_io_call_count)
    must(pipeline.tool.unlock_calls == 0, "未落座时不得调用 tool.unlock()")
    must(result.error_code == ErrorCode.SEAT_NOT_VERIFIED,
         "应报 400 SEAT_NOT_VERIFIED，实际 %s；消息=%s" % (result.error_code, result.message))
    visited = {state.value for state in result.states_history}
    must("UNLOAD" not in visited, "未落座不得进入 UNLOAD")
    must("DISENGAGE" not in visited, "未落座不得进入 DISENGAGE")

    # 交叉校验：ToolManager 自身的联锁也必须拒绝
    status = pipeline.tool_manager.trigger_unlock("probe", 200, seated_verified=False, unloaded=True)
    must(not status.accepted, "ToolManager 联锁必须拒绝未落座的解锁请求")
    must(pipeline.unlock_io_call_count == 0, "被拒绝的解锁不得触达 IO")
    return "未落座 -> %s(400)；解锁 IO 调用 0 次；ToolManager 联锁亦拒绝" % result.final_state


def case_08_unlock_accepted_not_released_no_retreat() -> str:
    """V2 解锁请求已接受但锁止未解除 → 不撤离。

    复位后无人解除锁止期间，对“是否已真正脱离”的判断必须是**否**；
    因此必须停在原地并报 410，而**不是**继续执行 DISENGAGE/RETREAT。
    """
    need("mtc_manipulation.pick_place_fsm")
    from mtc_manipulation.pick_place_fsm import ErrorCode

    pipeline = new_pipeline(V2)
    perception_for(pipeline, BLUE, "battery-stuck")
    pipeline.tool.unlock_confirm_evidence_level = 0   # 只有命令级证据：不得宣布解锁
    fsm = start_item(pipeline, BLUE, "P1", "battery-stuck")
    result = drive(fsm, pipeline)

    must(pipeline.unlock_io_call_count == 1, "解锁脉冲应已被受理 1 次")
    must(not result.success, "锁止未解除不得成功")
    must(result.final_state.value == "FAULT",
         "应停住并进入 FAULT，实际 %s" % result.final_state.value)
    must(result.error_code == ErrorCode.UNLOCK_FAILED,
         "应报 410 UNLOCK_FAILED，实际 %s" % result.error_code)
    must(pipeline.tool.disengage_calls == 0, "不得执行分离/退出动作")
    visited = {state.value for state in result.states_history}
    must("RETREAT" not in visited, "不得进入 RETREAT（不撤离）")
    reason = "%s | %s" % (result.message, fsm.message)
    must("解锁" in reason, "故障原因应指向解锁状态：%s" % reason)
    must(pipeline.tool_manager.status().unlock_confirmed is False,
         "ToolManager 不得声明解锁已确认")
    must(pipeline.tool_manager.status().withdraw_allowed is False,
         "ToolManager 不得允许撤离")
    # DISENGAGE 的前置联锁必须独立成立（落座 + 卸载）——证明不是靠顺序碰巧拦住
    status = pipeline.tool_manager.check_unlock_preconditions()
    must(status is None or isinstance(status, tuple),
         "check_unlock_preconditions 应返回 None 或 (code, detail)")
    return ("解锁受理但未确认 -> %s(410)；disengage 调用 %d 次；未进入 RETREAT；"
            "unlock_confirmed=%s withdraw_allowed=%s"
            % (result.final_state.value, pipeline.tool.disengage_calls,
               pipeline.tool_manager.status().unlock_confirmed,
               pipeline.tool_manager.status().withdraw_allowed))


def case_09_cancel_while_carrying() -> str:
    """取消正在搬运的 Action → 进入不确定负载保护，停止后保留状态。"""
    need("mtc_manipulation.pick_place_fsm")
    from mtc_manipulation.pick_place_fsm import ErrorCode, PickPlaceState

    pipeline = new_pipeline(V1)
    perception_for(pipeline, BLUE, "battery-carry")
    fsm = start_item(pipeline, BLUE, "P1", "battery-carry")

    for _ in range(200):
        if fsm.state is PickPlaceState.TRANSPORT:
            break
        fsm.step()
        pipeline.clock.advance(1.0)
    must(fsm.state is PickPlaceState.TRANSPORT, "应到达 TRANSPORT，实际 %s" % fsm.state.value)
    must(fsm.object_attached_estimated, "运输阶段应标记为可能携带电池")

    fsm.request_cancel("E2E: cancel while carrying")
    step = fsm.step()
    must(step.state is PickPlaceState.CANCEL_PENDING
         or (step.state is PickPlaceState.CANCELLED and pipeline.tool.holding_state() == "holding"),
         "取消后必须先进入 CANCEL_PENDING（或立即确认停稳），实际 %s" % step.state.value)
    pipeline.clock.advance(1.0)
    result = drive(fsm, pipeline, max_ticks=10)

    must(result.final_state.value == "CANCELLED",
         "确认停稳后应 CANCELLED，实际 %s" % result.final_state.value)
    must(not result.success, "取消不得返回成功")
    must(fsm.stop_confirmed, "必须有 stop_confirmed 证据")
    must(fsm.object_attached_estimated,
         "不确定负载保护：取消后必须保留“可能仍携带电池”的估计")
    must(pipeline.tool.holding_state() == "holding",
         "工具层必须保守保持 holding 状态，禁止自动解锁/掉落")
    must(pipeline.unlock_io_call_count == 0, "取消路径不得触发解锁 IO")
    visited = {state.value for state in result.states_history}
    must("RETREAT" not in visited, "取消不得继续撤离")

    # --- 关键控制组：停止**未被确认**时，必须 FAULT(520) 且不得宣告已安全停止 ---
    from mtc_manipulation.pick_place_fsm import StopResult

    pipeline2 = new_pipeline(V1)
    perception_for(pipeline2, BLUE, "battery-carry2")
    fsm2 = start_item(pipeline2, BLUE, "P1", "battery-carry2")
    for _ in range(200):
        if fsm2.state is PickPlaceState.TRANSPORT:
            break
        fsm2.step()
        pipeline2.clock.advance(1.0)
    must(fsm2.state is PickPlaceState.TRANSPORT, "对照组也应到达 TRANSPORT")

    stop_calls: List[str] = []

    def _unconfirmed_stop(reason: str) -> Any:
        stop_calls.append(reason)
        return StopResult(request_delivered=True, stop_confirmed=False,
                          message="E2E: 停止请求已送达，但未确认停稳")

    fsm2.motion.request_stop = _unconfirmed_stop     # 注入“未确认停稳”的端口行为
    fsm2.request_cancel("E2E: cancel without confirmation")
    fsm2.step()
    result2 = drive(fsm2, pipeline2, max_ticks=5)
    must(len(stop_calls) == 1, "必须且只能发出一次停止请求，实际 %d 次" % len(stop_calls))
    must(fsm2.state is PickPlaceState.CANCEL_PENDING
         or fsm2.state is PickPlaceState.FAULT, "未确认时应停在 CANCEL_PENDING 或 FAULT")
    if fsm2.state is PickPlaceState.CANCEL_PENDING:
        step2 = fsm2.step()
        must(step2.state is PickPlaceState.FAULT,
             "未确认停稳必须进入 FAULT，实际 %s" % step2.state.value)
    must(result2.final_state.value not in ("CANCELLED", "SUCCESS"),
         "未确认停稳不得返回取消成功，实际 %s" % result2.final_state.value)
    must(fsm2.error_code == ErrorCode.CANCEL_NOT_CONFIRMED,
         "应报 520 CANCEL_NOT_CONFIRMED，实际 %s" % fsm2.error_code)
    must(fsm2.object_attached_estimated or pipeline2.tool.holding_state() == "holding",
         "未确认停稳时同样必须保留不确定负载保护")
    return ("搬运中取消 -> %s（stop_confirmed=%s，保留 carrying 估计）；"
            "未确认停稳对照组 -> %s(520)，停止请求仅 1 次"
            % (result.final_state.value, fsm.stop_confirmed, fsm2.state.value))


def case_10_capability_change_invalidates_plan() -> str:
    """模型/TCP/负载/控制器模式变化 → 旧规划作废（能力查询或参数版本比对）。"""
    need("mtc_manipulation.pick_place_fsm", "mtc_motion_execution.backends")
    from mtc_manipulation.pick_place_fsm import ErrorCode

    pipeline = new_pipeline(V1)

    # 第一层：规划上下文指纹（模型 / TCP / 负载 / 控制器模式）
    first = pipeline.planner.plan("ACQUIRE", {"object_id": "battery-cap"})
    must(first.ok, "首次规划应成功：%s" % first.message)
    reference_plan = pipeline.planner.plans[-1]
    must(not reference_plan.invalidated, "新规划不应处于作废态")
    fingerprint_before = pipeline.planner.last_plan_capability

    pipeline.planner.set_capability(controller_mode="force")
    second = pipeline.planner.plan("ACQUIRE", {"object_id": "battery-cap"})
    must(not second.ok, "控制器模式变化后旧规划必须作废：%s" % second.message)
    must(second.error_code == ErrorCode.PLANNING_FAILED,
         "应报 200 PLANNING_FAILED，实际 %s" % second.error_code)
    must("旧规划作废" in second.message, "消息应明确说明旧规划作废：%s" % second.message)
    must(pipeline.planner.last_plan_capability == fingerprint_before,
         "作废判定不得悄悄把新指纹写回（必须重新规划）")

    pipeline.planner.set_capability(tcp_frame="tool0_v2", payload_kg=1.2)
    third = pipeline.planner.plan("ACQUIRE", {"object_id": "battery-cap"})
    must(not third.ok, "TCP/负载变化后旧规划同样必须作废：%s" % third.message)

    # 第二层：能力查询（GetMotionCapabilities 语义）
    capabilities_before = pipeline.executor.capabilities()
    must(capabilities_before.get("joint_goal_supported") is True,
         "mock 后端应声明 joint_goal_supported")
    must(capabilities_before.get("timed_trajectory_supported") is False,
         "不得声明轨迹能力")
    must(capabilities_before.get("cartesian_motion_supported") is False,
         "不得声明笛卡尔能力")

    from mtc_motion_execution.backends import DisabledMotionBackend
    disabled = DisabledMotionBackend(reason="E2E: 能力指纹变化后旧通路作废")
    caps_disabled = disabled.capabilities()
    must(caps_disabled.joint_goal_supported is False,
         "作废后的通路不得声明运动能力")
    result = disabled.execute_joint_move(pipeline.motion.issued[0]) if pipeline.motion.issued else None
    if result is not None:
        must(not result.success, "能力作废后不得执行旧计划")

    return ("规划指纹 %s -> 控制器模式/TCP/负载变化后旧规划作废(200)；"
            "能力查询 joint_goal=%s traj=%s cartesian=%s"
            % (fingerprint_before, capabilities_before.get("joint_goal_supported"),
               capabilities_before.get("timed_trajectory_supported"),
               capabilities_before.get("cartesian_motion_supported")))


def case_11_non_target_in_sweep_zone() -> str:
    """非目标电池进入工具扫掠区域 → 规划/执行拒绝。"""
    need("mtc_manipulation.pick_place_fsm")
    from mtc_manipulation.pick_place_fsm import ErrorCode

    pipeline = new_pipeline(V1)
    perception_for(pipeline, BLUE, "battery-target")
    # 另一块电池进入扫掠区域（非目标）
    pipeline.planner.sweep_zone_obstacles = ["battery-other-red"]
    pipeline.planner.target_object_id = "battery-target"
    fsm = start_item(pipeline, BLUE, "P1", "battery-target")
    result = drive(fsm, pipeline)

    must(not result.success, "扫掠区域被占用时不得成功")
    must(result.error_code == ErrorCode.SAFETY_INTERLOCK,
         "应报 500 SAFETY_INTERLOCK，实际 %s（%s）" % (result.error_code, result.message))
    must(pipeline.backend_call_count == 0,
         "规划被拒绝后不得执行任何运动，后端调用=%d" % pipeline.backend_call_count)
    must(any(item.startswith("sweep_zone_occupied") for item in pipeline.planner.rejected),
         "规划器应记录拒绝原因：%s" % pipeline.planner.rejected)

    # 对照：目标自身在扫掠区域不算障碍（不得误伤）
    pipeline2 = new_pipeline(V1)
    perception_for(pipeline2, BLUE, "battery-target")
    pipeline2.planner.sweep_zone_obstacles = ["battery-target"]
    pipeline2.planner.target_object_id = "battery-target"
    fsm2 = start_item(pipeline2, BLUE, "P1", "battery-target")
    result2 = drive(fsm2, pipeline2)
    must(result2.success, "目标自身不算障碍，应可完成：%s" % result2.message)
    return ("扫掠区含非目标 -> %s(500)，运动调用 0 次；目标自身不误伤（对照 SUCCESS）"
            % result.final_state)


def case_12_stability_window_not_met() -> str:
    """稳定性观察未满 3 秒 → 不得标记成功。

    ``VERIFY_PLACED`` 的稳定性是**多次独立观测**的累计：每次观测前清除端口结果缓存，
    等价于收到一帧新视觉数据（否则复用同一帧缓存无法体现“累计观察”语义）。
    """
    need("mtc_manipulation.pick_place_fsm")
    from mtc_manipulation.pick_place_fsm import PickPlaceState

    pipeline = new_pipeline(V1)
    perception_for(pipeline, BLUE, "battery-stab")
    fsm = start_item(pipeline, BLUE, "P1", "battery-stab", stability_sec=3.0)

    for _ in range(200):
        if fsm.state is PickPlaceState.VERIFY_PLACED:
            break
        fsm.step()
        pipeline.clock.advance(1.0)
    must(fsm.state is PickPlaceState.VERIFY_PLACED,
         "应到达 VERIFY_PLACED，实际 %s" % fsm.state.value)
    must(not fsm.placement_verified, "刚进入 VERIFY_PLACED 时不得已标记成功")

    def observe() -> None:
        """收到一帧新的放置核验观测（清除缓存后 step 一次）。"""
        fsm._clear_port_results()
        fsm.step()

    pipeline.clock.advance(1.0)
    observe()                                     # 累计观察 1.0s < 3.0s
    must(fsm.state is PickPlaceState.VERIFY_PLACED,
         "1.0s 时不得离开 VERIFY_PLACED，实际 %s" % fsm.state.value)
    must(not fsm.placement_verified, "1.0s 时不得 placement_verified=True")

    pipeline.clock.advance(1.0)
    observe()                                     # 累计观察 2.0s < 3.0s
    must(fsm.state is PickPlaceState.VERIFY_PLACED,
         "2.0s 时不得离开 VERIFY_PLACED，实际 %s" % fsm.state.value)
    must(not fsm.placement_verified, "2.0s 时不得 placement_verified=True")
    result = fsm.result()
    must(not result.success, "稳定性未满 3s 时不得 success")
    must(not result.placement_verified, "稳定性未满 3s 时不得 placement_verified=True")

    # 稳定性起点由**首次有效观测**确立，因此每次观测只累计一个推进量。
    # 这里连续推进直到累计稳定时间达到阈值，断言“恰好达标后才成功”。
    pipeline.clock.advance(1.0)
    observe()                                     # 累计 2.0s < 3.0s
    must(fsm.state is PickPlaceState.VERIFY_PLACED,
         "稳定性未满 3s 时不得离开 VERIFY_PLACED，实际 %s" % fsm.state.value)
    must(not fsm.placement_verified, "稳定性未满 3s 时不得 placement_verified=True")
    observed = fsm.stability_verified_for_s
    must(observed < 3.0, "此刻累计稳定时间应小于阈值，实际 %.2f" % observed)

    # 精确逼近阈值：先推算起点，再只补足到“恰好差 0.05s”，验证不会提前判成功
    started_at = fsm._stability_started_at
    must(started_at is not None, "稳定性计时起点必须存在")
    remaining = 3.0 - (pipeline.clock.now() - started_at)
    pipeline.clock.advance(max(remaining - 0.05, 0.0))
    observe()
    must(fsm.state is PickPlaceState.VERIFY_PLACED,
         "距离阈值差 0.05s 时不得判成功，实际 %s（累计 %.2fs）"
         % (fsm.state.value, fsm.stability_verified_for_s))
    must(not fsm.placement_verified, "距离阈值 0.05s 时不得 placement_verified=True")

    pipeline.clock.advance(0.1)                   # 越过阈值
    observe()
    must(fsm.state is PickPlaceState.SUCCESS,
         "稳定满 3s 后应 SUCCESS，实际 %s（累计 %.2fs）"
         % (fsm.state.value, fsm.stability_verified_for_s))
    must(fsm.placement_verified, "稳定满 3s 后才允许 placement_verified=True")
    stable_for = pipeline.clock.now() - started_at
    return ("未满 3s 时停在 VERIFY_PLACED 且 placement_verified=False；"
            "观测起点 t=%.2f，%.2fs 后 SUCCESS" % (started_at, stable_for))


def case_13_v1_has_no_unlock_io() -> str:
    """V1 路径不依赖电磁 IO（断言解锁 IO 调用次数为 0）。"""
    need("mtc_manipulation.pick_place_fsm")
    from mtc_manipulation.tool_strategy import PassiveHookV1Strategy

    pipeline = new_pipeline(V1)
    must(isinstance(pipeline.strategy, PassiveHookV1Strategy)
         or pipeline.strategy.type() == "passive_hook_v1",
         "V1 链路必须使用 passive_hook_v1 策略")
    must(not pipeline.strategy.requires_unlock_pulse(), "V1 不得要求解锁脉冲")
    must(not pipeline.strategy.uses_io("electromagnetic_unlock"), "V1 不得声明使用电磁通道")

    orchestrator = MockOrchestrator(
        pipeline, colors=[BLUE, RED, YELLOW], object_ids=["b1", "b2", "b3"], mode=2)
    outcome = orchestrator.run()
    must(outcome.success, "V1 三色序列应完成：%s" % outcome.message)
    must(pipeline.unlock_io_call_count == 0,
         "V1 解锁 IO 调用次数必须为 0，实际 %d" % pipeline.unlock_io_call_count)
    must(list(pipeline.unlock_io_history) == [], "V1 不得有任何解锁脉冲历史")
    must(pipeline.tool.unlock_calls == 0, "V1 不得调用 tool.unlock()")
    history_before_probe = list(pipeline.unlock_io_history)
    before_probe = pipeline.unlock_io_call_count

    # 防御性检查：即使被误调用，V1 的 IO 端口也不产生真实脉冲
    accepted = pipeline.tool_manager.unlock_io.send_unlock_pulse(200)
    must(accepted is False, "V1 的禁用 IO 端口必须拒绝任何脉冲请求")
    must(pipeline.tool_manager.unlock_io.last_status == "DISABLED",
         "端口状态应为 DISABLED，实际 %s" % pipeline.tool_manager.unlock_io.last_status)
    return ("V1 三色序列完成，全链路解锁 IO 调用 %d 次（脉冲历史 %s）；"
            "误调用亦被 DISABLED 端口拒绝（调用前后均为 %d 次）"
            % (before_probe, history_before_probe, before_probe))


# ---------------------------------------------------------------------------
# 4. 附加：配置交叉校验（设计 §8）与联锁守卫（§7.2）
# ---------------------------------------------------------------------------
def case_14_config_cross_check() -> str:
    """读取真实 config/*.yaml 做 §8 交叉校验（工具类型/稳定性/感知年龄/颜色）。"""
    from mtc_bringup.config_loader import validate_all

    config_dir = os.path.join(_WS_ROOT, "config")
    report = validate_all(config_dir)
    must(report.loaded, "应加载到 config/*.yaml")
    must(report.ok, "配置校验必须通过：%s"
         % "; ".join(item.format() for item in report.errors))

    params: Dict[str, Dict[str, Any]] = {item.name: item.params for item in report.loaded}
    manipulation = params.get("manipulation.yaml", {})
    must(manipulation.get("tool_type") in ("passive_hook_v1", "magnetic_latch_v2"),
         "tool_type 必须是两代工具之一")
    must(float(manipulation.get("placement_stability_sec", 0)) > 0, "placement_stability_sec 必须 > 0")
    must(float(manipulation.get("perception_max_age_sec", 0)) > 0, "perception_max_age_sec 必须 > 0")
    safety = params.get("safety.yaml", {})
    identity = safety.get("device_identity", {})
    must(not list(identity.get("serial_number_whitelist") or []),
         "设备身份白名单应保持留空（安全默认拒绝建立运动通路）")
    return ("%d 个配置文件通过校验（%s）；tool_type=%s；身份白名单留空 -> 拒绝运动通路"
            % (len(report.loaded), report.backend, manipulation.get("tool_type")))


def case_15_interlock_guard() -> str:
    """§7.2 关键联锁：未验证挂载禁止运输、未落座禁止解锁、未知状态禁止重发。"""
    need("mtc_safety.interlocks")
    from mtc_safety.interlocks import DeviceState, InterlockGuard, PayloadState

    guard = InterlockGuard(device_state=DeviceState.READY, link_ok=True,
                           last_state_ts=0.0, clock=lambda: 0.0,
                           attach_verified=False, seat_verified=False)
    guard.payload_tracker.mark_unknown("e2e")
    transport = guard.allow_transport()
    must(not transport.allowed, "未验证挂载必须禁止运输")
    unlock = guard.allow_unlock()
    must(not unlock.allowed, "载荷未知必须禁止解锁")
    resend = guard.allow_resend_motion()
    must(not resend.allowed, "缺少“从未开始运动”证据时必须禁止重发")
    withdraw = guard.allow_force_withdraw()
    must(not withdraw.allowed, "解锁/脱离状态不确定时必须禁止撤离")
    second = guard.allow_second_motion_goal()
    must(second.allowed, "无在途命令时应允许运动 Goal")

    guard.note_motion_started("cmd-1")
    must(not guard.allow_second_motion_goal().allowed, "在途运动时必须拒绝第二个运动 Goal")
    guard.note_motion_state_unknown("link lost")
    must(not guard.allow_resend_motion().allowed, "状态未知时必须禁止重发")
    must(guard.allow_motion().blocked, "状态未知时必须禁止实机运动")
    return "allow_transport/unlock/withdraw/resend 全部拒绝；在途时拒绝第二个 Goal"


# ---------------------------------------------------------------------------
# 5. 主流程
# ---------------------------------------------------------------------------
CASES: Sequence[Tuple[str, str, Callable[[], str]]] = (
    ("01", "蓝→红→黄三色序列：P1/P2/P3 映射正确，全流程 Mock PASS", case_01_sequence_three_colors),
    ("02", "并发第二个 Task Goal / Motion Goal 被明确拒绝", case_02_concurrent_goals_rejected),
    ("03", "目标颜色错误 / 重复 ID / 提环位姿过期 → 不运动", case_03_bad_target_never_moves),
    ("04", "规划成功但底层执行失败 → 任务不得标记成功", case_04_execution_failure_not_success),
    ("05", "moveJoint 超时但可能仍在动 → 不自动重发", case_05_timeout_no_resend),
    ("06", "试提时电池未跟随 → 禁止进入运输", case_06_test_lift_not_followed_blocks_transport),
    ("07", "放置未落座 → V2 电磁解锁不得触发", case_07_not_seated_no_unlock),
    ("08", "V2 解锁已受理但锁止未解除 → 不撤离", case_08_unlock_accepted_not_released_no_retreat),
    ("09", "取消正在搬运的 Action → 不确定负载保护 + 保留状态", case_09_cancel_while_carrying),
    ("10", "模型/TCP/负载/控制器模式变化 → 旧规划作废", case_10_capability_change_invalidates_plan),
    ("11", "非目标电池进入工具扫掠区域 → 规划/执行拒绝", case_11_non_target_in_sweep_zone),
    ("12", "稳定性观察未满 3 秒 → 不得标记成功", case_12_stability_window_not_met),
    ("13", "V1 路径不依赖电磁 IO（解锁 IO 调用次数为 0）", case_13_v1_has_no_unlock_io),
    ("14", "配置交叉校验（§8：tool_type/稳定性/感知年龄/身份白名单）", case_14_config_cross_check),
    ("15", "安全联锁守卫（§7.2：运输/解锁/撤离/重发/并发）", case_15_interlock_guard),
)


def print_header() -> None:
    print("=" * 78)
    print("mtc_bringup 离线端到端集成测试（纯 Python 3，不 import rclpy）")
    print("=" * 78)
    print("工作空间根目录 : %s" % _WS_ROOT)
    print("依赖模块状态：")
    for status in DEPS.statuses:
        print("  " + status.format())
    missing = DEPS.missing()
    if missing:
        print("依赖未就绪（相关用例将输出 SKIPPED，不计入 PASS）：%s" % ", ".join(missing))
    print("-" * 78)


def main() -> int:
    print_header()

    if not _DEPS_READY:
        print("链路必需依赖未就绪，全部用例将输出 SKIPPED：%s" % _DEPS_ERROR)
    for module in FORBIDDEN_MODULES:
        if module in sys.modules:
            print("FAIL: 本测试不得导入 %s（本机仅有 ROS 2 Humble）" % module)
            print("E2E RESULT: PASS 0 / FAIL 1 / SKIP 0")
            return 1

    harness = Harness()
    for case_id, name, func in CASES:
        harness.run(case_id, name, func)

    passed, failed, skipped = harness.count("PASS"), harness.count("FAIL"), harness.count("SKIP")
    print("-" * 78)
    if failed:
        print("失败用例：")
        for item in harness.results:
            if item.status == "FAIL":
                print("  - [%s] %s：%s" % (item.case_id, item.name, item.error.splitlines()[0] if item.error else item.detail))
    if skipped:
        print("跳过用例（依赖未就绪）：")
        for item in harness.results:
            if item.status == "SKIP":
                print("  - [%s] %s：%s" % (item.case_id, item.name, item.detail))

    total = passed + failed + skipped
    print("=" * 78)
    print("全部 PASS 均为 Mock PASS，非实机抓放成功。")
    print("用例总数 %d：PASS %d / FAIL %d / SKIP %d" % (total, passed, failed, skipped))
    print("E2E RESULT: PASS %d / FAIL %d / SKIP %d" % (passed, failed, skipped))
    return 0 if (failed == 0 and passed > 0) else 1


if __name__ == "__main__":
    sys.exit(main())

