#!/usr/bin/env python3
"""mtc_task 单元测试——**纯 Python 3，不导入 rclpy**。

可运行方式（两种均可）::

    cd /home/robot/meituan_challenge_ws/src/mtc_task
    python3 -m pytest test/ -v
    python3 test/test_task_fsm.py
    python3 -m unittest discover -s test -v

覆盖范围（对应设计文档 §3.1 / §4.3 / §7.1 / §7.2）：
1. 基础任务（单色 → T0）成功全流程
2. 序列任务三色成功流程，slot 固定映射 P1/P2/P3
3. 非法任务 7 类：空颜色、4 个颜色、重复颜色、非法颜色值、timeout_ms=0、
   非法 target_slot、空 task_id → TASK_FAILED + INVALID_TASK(100)
4. 并发第二个 Goal 被明确拒绝（不排队）
5. 取消：CANCEL_PENDING → 确认停稳 → CANCELLED；未确认 → FAULT(520)
6. SCAN_SCENE 找不到目标 → 限次重试后 TASK_FAILED
7. 单阶段超时进入预期状态
8. 运动状态未知(230) → FAULT，且不自动重试
9. 转移表 / 超时表 / 进入处理表的完整性自检
"""

from __future__ import annotations

import os
import sys
import unittest

# 允许 `python3 test/test_task_fsm.py` 直接运行（把包目录加入 sys.path）
_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from mtc_task.task_fsm import (  # noqa: E402  (import 位置受 sys.path 影响)
    ALL_STATES,
    DEFAULT_STATE_TIMEOUTS,
    ENTRY_ACTIONS,
    TRANSITIONS,
    ActionKind,
    ErrorCodes,
    Event,
    FsmsConfig,
    TaskFsm,
    TaskRequest,
    TaskStateName,
)
from mtc_task.task_validation import (  # noqa: E402
    ALLOWED_SLOTS,
    MODE_BASIC,
    MODE_SEQUENCE,
    validate_task_request,
)

# 颜色常量（BatteryDetection.msg）
RED, BLUE, YELLOW, GREEN = 1, 2, 3, 4

#: 确认本测试不依赖 ROS
FORBIDDEN_MODULES = ("rclpy", "mtc_interfaces", "rcl_interfaces")


class TestStabilityFinalGate(unittest.TestCase):
    def task(self, stability: float) -> TaskRequest:
        return TaskRequest("stability-gate", MODE_BASIC, [RED], 60000,
                           placement_stability_sec=stability)

    def check_invalid(self, value: float) -> None:
        result = validate_task_request(self.task(value))
        self.assertFalse(result.ok)
        self.assertEqual(result.error_code, ErrorCodes.INVALID_TASK)
        self.assertEqual(result.reason, "stability_not_positive")

    def test_zero_stability_rejected(self) -> None:
        self.check_invalid(0.0)

    def test_negative_stability_rejected(self) -> None:
        self.check_invalid(-0.1)

    def test_nan_stability_rejected(self) -> None:
        self.check_invalid(float("nan"))

    def test_infinite_stability_rejected(self) -> None:
        self.check_invalid(float("inf"))
        self.check_invalid(float("-inf"))

    def test_positive_stability_accepted(self) -> None:
        for value in (0.1, 1.5, 3.0, 5.0):
            self.assertTrue(validate_task_request(self.task(value)).ok)

    def test_default_stability_is_three_seconds(self) -> None:
        task = TaskRequest("default", MODE_BASIC, [RED], 60000)
        self.assertEqual(task.placement_stability_sec, 3.0)
        self.assertTrue(validate_task_request(task).ok)
        missing_field = dict(task_id="missing", mode=MODE_BASIC, ordered_colors=[RED], timeout_ms=60000)
        self.assertEqual(TaskRequest.from_goal(missing_field).placement_stability_sec, 3.0)
        self.assertTrue(validate_task_request(missing_field).ok)

    def test_explicit_zero_is_not_replaced_with_default(self) -> None:
        goal = dict(task_id="zero", mode=MODE_BASIC, ordered_colors=[RED], timeout_ms=60000,
                    placement_stability_sec=0.0)
        self.assertEqual(TaskRequest.from_goal(goal).placement_stability_sec, 0.0)
        self.assertFalse(validate_task_request(goal).ok)


class FakeClock:
    """可注入的假时钟（秒）。"""

    def __init__(self, start: float = 1000.0) -> None:
        self.now = float(start)

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> float:
        self.now += float(seconds)
        return self.now


class FakeGoal:
    """模拟 ExecuteTask.action 的 Goal 字段。"""

    def __init__(
        self,
        task_id="task-1",
        mode=MODE_BASIC,
        ordered_colors=(RED,),
        timeout_ms=120000,
        placement_stability_sec=3.0,
        target_slot=None,
    ):
        self.task_id = task_id
        self.mode = mode
        self.ordered_colors = list(ordered_colors)
        self.timeout_ms = timeout_ms
        self.placement_stability_sec = placement_stability_sec
        if target_slot is not None:
            # 仅在显式给出时才作为额外属性存在（ExecuteTask.action 本身无此字段）
            self.target_slot = target_slot


def make_fsm(**timeout_overrides):
    """构造处于 WAIT_TASK 的 FSM（注入假时钟）。

    ``timeout_overrides`` 形如 ``state_scan_timeout_s=5.0``；状态名部分会与
    :data:`DEFAULT_STATE_TIMEOUTS` 做宽松匹配（``scan`` → ``SCAN_SCENE``）。
    """
    clock = FakeClock()
    by_prefix = {}
    for state in DEFAULT_STATE_TIMEOUTS:
        by_prefix.setdefault(state.name.split("_")[0], state)

    overrides = {}
    for key, value in timeout_overrides.items():
        if not key.startswith("state_") or not key.endswith("_timeout_s"):
            raise ValueError("bad timeout override key: %s" % key)
        token = key[len("state_"):-len("_timeout_s")].upper()
        if token in TaskStateName.__members__:
            state = TaskStateName[token]
        elif token in by_prefix:
            state = by_prefix[token]
        else:
            raise ValueError("unknown state in timeout override: %s" % key)
        overrides[state] = value

    fsm = TaskFsm(clock=clock, config=FsmsConfig(state_timeouts={**DEFAULT_STATE_TIMEOUTS, **overrides}))
    fsm.start()
    fsm.step(Event.SELF_CHECK_OK)
    assert fsm.state is TaskStateName.WAIT_TASK, fsm.state
    return fsm, clock


def complete_one_item(fsm, slot, color, object_id="obj-1"):
    """走完一项：SCAN_SCENE → EXECUTE_ITEM → RECORD_RESULT。

    若调用方已手动推进到 EXECUTE_ITEM，则从该处继续。
    """
    if fsm.state is TaskStateName.SCAN_SCENE:
        fsm.report_target(object_id, expected_color=color)
    assert fsm.state is TaskStateName.EXECUTE_ITEM, fsm.state
    result = fsm.report_item_result(
        {"success": True, "placement_verified": True, "error_code": ErrorCodes.OK, "message": "placed"}
    )
    assert fsm.state is TaskStateName.RECORD_RESULT, fsm.state
    return fsm.record_completed_item(slot)


class TestNoRosDependency(unittest.TestCase):
    """保证测试是纯 Python 的（本机只有 Humble，刻意不编译）。"""

    def test_ros_modules_not_imported(self):
        for module in FORBIDDEN_MODULES:
            self.assertNotIn(module, sys.modules, "%s 不应被导入" % module)

    def test_core_import_does_not_need_ros(self):
        self.assertTrue(hasattr(TaskFsm, "step"))
        # 核心模块自身不得引用 ROS 客户端库
        core = sys.modules["mtc_task.task_fsm"]
        source_names = set(dir(core))
        for forbidden in ("rclpy", "Node", "ActionServer"):
            self.assertNotIn(forbidden, source_names, "核心模块不应引用 %s" % forbidden)


class TestTableIntegrity(unittest.TestCase):
    """转移表 / 超时表 / 进入处理表的完整性与集中性。"""

    def test_every_state_has_entry_handler_and_timeout(self):
        for state in ALL_STATES:
            self.assertIn(state, ENTRY_ACTIONS, state)
            self.assertIn(state, DEFAULT_STATE_TIMEOUTS, state)

    def test_no_duplicate_transition_keys(self):
        keys = [(t.state, t.event) for t in TRANSITIONS]
        self.assertEqual(len(keys), len(set(keys)), "转移表存在重复 (state, event)")

    def test_guard_names_are_centralised(self):
        for transition in TRANSITIONS:
            if transition.guard is not None:
                self.assertTrue(transition.guard.startswith("_guard_"), transition)
                self.assertTrue(hasattr(TaskFsm, transition.guard), transition.guard)

    def test_all_guards_referenced_by_table(self):
        table_guards = {t.guard for t in TRANSITIONS if t.guard}
        impl_guards = {n for n in dir(TaskFsm) if n.startswith("_guard_")}
        self.assertEqual(table_guards, impl_guards, "存在未被转移表引用的守卫（或缺失）")

    def test_required_transitions_present(self):
        required = {
            (TaskStateName.WAIT_TASK, Event.GOAL_ACCEPTED): TaskStateName.VALIDATE_TASK,
            (TaskStateName.VALIDATE_TASK, Event.VALIDATION_FAILED): TaskStateName.TASK_FAILED,
            (TaskStateName.SCAN_SCENE, Event.TARGET_FOUND): TaskStateName.EXECUTE_ITEM,
            (TaskStateName.SCAN_SCENE, Event.TARGET_NOT_FOUND): TaskStateName.RECOVERY,
            (TaskStateName.EXECUTE_ITEM, Event.ITEM_SUCCEEDED): TaskStateName.RECORD_RESULT,
            (TaskStateName.RECORD_RESULT, Event.RECORD_OK): TaskStateName.FINISHED,
            (TaskStateName.CANCEL_PENDING, Event.CHILD_STOPPED_CONFIRMED): TaskStateName.CANCELLED,
            (TaskStateName.CANCEL_PENDING, Event.TIMEOUT): TaskStateName.FAULT,
            (TaskStateName.RECOVERY, Event.RETRY_DENIED): TaskStateName.TASK_FAILED,
        }
        for key, expected in required.items():
            matches = [t for t in TRANSITIONS if (t.state, t.event) == key]
            self.assertTrue(matches, "缺少转移 %s" % (key,))
            self.assertIn(expected, [t.next_state for t in matches], key)

    def test_lifecycle_init_to_wait_task(self):
        clock = FakeClock()
        fsm = TaskFsm(clock=clock)
        self.assertIs(fsm.state, TaskStateName.INIT)
        fsm.start()
        self.assertIs(fsm.state, TaskStateName.SELF_CHECK)
        fsm.step(Event.SELF_CHECK_OK)
        self.assertIs(fsm.state, TaskStateName.WAIT_TASK)

    def test_self_check_failure_goes_fault(self):
        fsm = TaskFsm(clock=FakeClock())
        fsm.start()
        result = fsm.step(Event.SELF_CHECK_FAIL)
        self.assertIs(result.state, TaskStateName.FAULT)
        self.assertTrue(result.has(ActionKind.REQUEST_STOP), "FAULT 必须请求停止")
        self.assertTrue(result.has(ActionKind.LOCK_ACTIONS), "FAULT 必须锁定动作")


class TestBasicTaskSuccess(unittest.TestCase):
    """1. 基础任务（单色 → T0）成功全流程。"""

    def test_basic_single_color_to_t0(self):
        fsm, _clock = make_fsm()
        goal = FakeGoal(task_id="basic-1", mode=MODE_BASIC, ordered_colors=[BLUE])

        accepted = fsm.submit_goal(goal)
        self.assertIs(accepted.state, TaskStateName.VALIDATE_TASK)
        self.assertFalse(accepted.goal_rejected)

        result = fsm.advance_validation()
        self.assertIs(result.state, TaskStateName.SCAN_SCENE, fsm.last_validation_error)

        fsm.report_target("battery-blue-7", expected_color=BLUE)
        self.assertIs(fsm.state, TaskStateName.EXECUTE_ITEM)
        self.assertEqual(fsm.current_slot, "T0")
        self.assertIn("T0", ALLOWED_SLOTS)

        finished = complete_one_item(fsm, "T0", BLUE)
        self.assertIs(finished.state, TaskStateName.FINISHED)

        outcome = fsm.build_outcome()
        self.assertTrue(outcome.success)
        self.assertEqual(outcome.error_code, ErrorCodes.OK)
        self.assertEqual(outcome.completed_count, 1)
        self.assertEqual(outcome.completed_slots, ["T0"])

    def test_pick_place_request_fields_match_action_contract(self):
        fsm, _clock = make_fsm()
        fsm.submit_goal(FakeGoal(task_id="pp-1", mode=MODE_BASIC, ordered_colors=[GREEN]))
        fsm.advance_validation()
        result = fsm.report_target("obj-green-1", expected_color=GREEN)
        request = result.find(ActionKind.CALL_PICK_PLACE)
        self.assertIsNotNone(request, "进入 EXECUTE_ITEM 必须产生 call_pick_place")
        payload = request.payload
        # 字段名必须与 PickPlace.action 的 Goal 完全一致
        self.assertEqual(
            sorted(payload.keys()),
            sorted(["request_id", "object_id", "expected_color", "target_slot", "placement_stability_sec"]),
        )
        self.assertEqual(payload["object_id"], "obj-green-1")
        self.assertEqual(payload["expected_color"], GREEN)
        self.assertEqual(payload["target_slot"], "T0")
        self.assertEqual(payload["request_id"], "pp-1#0")

    def test_reset_after_finished_allows_next_task(self):
        fsm, _clock = make_fsm()
        fsm.submit_goal(FakeGoal(task_id="a"))
        fsm.advance_validation()
        complete_one_item(fsm, "T0", RED)
        self.assertIs(fsm.state, TaskStateName.FINISHED)

        fsm.reset()
        self.assertIs(fsm.state, TaskStateName.WAIT_TASK)
        self.assertIsNone(fsm.current_task)
        self.assertEqual(fsm.completed_slots, [])

        fsm.submit_goal(FakeGoal(task_id="b", ordered_colors=[YELLOW]))
        self.assertIs(fsm.state, TaskStateName.VALIDATE_TASK)


class TestSequenceTaskSuccess(unittest.TestCase):
    """2. 序列任务三色成功流程，slot 固定映射 P1/P2/P3。"""

    def test_sequence_three_colors_to_p1_p2_p3(self):
        fsm, _clock = make_fsm()
        goal = FakeGoal(task_id="seq-1", mode=MODE_SEQUENCE, ordered_colors=[BLUE, RED, YELLOW])
        fsm.submit_goal(goal)
        fsm.advance_validation()

        expected = [("P1", BLUE), ("P2", RED), ("P3", YELLOW)]
        for index, (slot, color) in enumerate(expected):
            self.assertIs(fsm.state, TaskStateName.SCAN_SCENE, "第 %d 项" % index)
            self.assertEqual(fsm.current_index, index)
            result = fsm.report_target("obj-%d" % index, expected_color=color)
            request = result.find(ActionKind.CALL_PICK_PLACE)
            self.assertEqual(request.payload["target_slot"], slot)
            self.assertEqual(request.payload["expected_color"], color)
            result = fsm.report_item_result(
                {"success": True, "placement_verified": True, "error_code": ErrorCodes.OK}
            )
            self.assertIs(result.state, TaskStateName.RECORD_RESULT)

            if index < 2:
                result = fsm.record_completed_item(slot)
                self.assertIs(result.state, TaskStateName.SCAN_SCENE)
            else:
                result = fsm.record_completed_item(slot)
                self.assertIs(result.state, TaskStateName.FINISHED)

        outcome = fsm.build_outcome()
        self.assertTrue(outcome.success)
        self.assertEqual(outcome.completed_slots, ["P1", "P2", "P3"])
        self.assertEqual(outcome.completed_count, 3)
        self.assertEqual(fsm.completed_count, 3)

    def test_sequence_index_advances_only_on_record(self):
        fsm, _clock = make_fsm()
        fsm.submit_goal(FakeGoal(task_id="seq-2", mode=MODE_SEQUENCE, ordered_colors=[RED, BLUE, GREEN]))
        fsm.advance_validation()
        fsm.report_target("obj-0", expected_color=RED)
        self.assertEqual(fsm.current_index, 0, "完成前不得推进索引")
        fsm.report_item_result({"success": True, "placement_verified": True})
        fsm.record_completed_item("P1")
        self.assertEqual(fsm.current_index, 1)
        self.assertEqual(fsm.completed_slots, ["P1"])

    def test_slot_mapping_is_derived_from_mode(self):
        task = TaskRequest(task_id="t", mode=MODE_SEQUENCE, ordered_colors=[BLUE, RED, YELLOW], timeout_ms=1000)
        self.assertEqual(task.slots(), ["P1", "P2", "P3"])
        basic = TaskRequest(task_id="t", mode=MODE_BASIC, ordered_colors=[BLUE], timeout_ms=1000)
        self.assertEqual(basic.slots(), ["T0"])


class TestInvalidTaskRejection(unittest.TestCase):
    """3. 非法任务 → TASK_FAILED + INVALID_TASK(100)，且各种原因可区分。"""

    def _submit_and_advance(self, goal):
        fsm, _clock = make_fsm()
        fsm.submit_goal(goal)
        result = fsm.advance_validation()
        return fsm, result

    def test_empty_colors(self):
        fsm, result = self._submit_and_advance(FakeGoal(task_id="bad-1", mode=MODE_BASIC, ordered_colors=[]))
        self.assertIs(result.state, TaskStateName.TASK_FAILED)
        self.assertEqual(fsm.last_error_code, ErrorCodes.INVALID_TASK)
        self.assertEqual(fsm.last_validation_error, "empty_colors")
        self.assertFalse(fsm.build_outcome().success)

    def test_four_colors_rejected(self):
        fsm, result = self._submit_and_advance(
            FakeGoal(task_id="bad-2", mode=MODE_SEQUENCE, ordered_colors=[RED, BLUE, YELLOW, GREEN])
        )
        self.assertIs(result.state, TaskStateName.TASK_FAILED)
        self.assertEqual(fsm.last_error_code, ErrorCodes.INVALID_TASK)
        self.assertEqual(fsm.last_validation_error, "sequence_count")

    def test_basic_with_three_colors_rejected(self):
        fsm, result = self._submit_and_advance(
            FakeGoal(task_id="bad-3", mode=MODE_BASIC, ordered_colors=[RED, BLUE, YELLOW])
        )
        self.assertIs(result.state, TaskStateName.TASK_FAILED)
        self.assertEqual(fsm.last_validation_error, "basic_count")

    def test_duplicate_colors_rejected(self):
        fsm, result = self._submit_and_advance(
            FakeGoal(task_id="bad-4", mode=MODE_SEQUENCE, ordered_colors=[RED, RED, BLUE])
        )
        self.assertIs(result.state, TaskStateName.TASK_FAILED)
        self.assertEqual(fsm.last_error_code, ErrorCodes.INVALID_TASK)
        self.assertEqual(fsm.last_validation_error, "duplicate_color")

    def test_illegal_color_value_rejected(self):
        for bad_color in (0, 5, 99):
            with self.subTest(color=bad_color):
                fsm, result = self._submit_and_advance(
                    FakeGoal(task_id="bad-5", mode=MODE_BASIC, ordered_colors=[bad_color])
                )
                self.assertIs(result.state, TaskStateName.TASK_FAILED)
                self.assertEqual(fsm.last_error_code, ErrorCodes.INVALID_TASK)
                self.assertEqual(fsm.last_validation_error, "illegal_color")

    def test_timeout_ms_zero_rejected(self):
        fsm, result = self._submit_and_advance(
            FakeGoal(task_id="bad-6", mode=MODE_BASIC, ordered_colors=[BLUE], timeout_ms=0)
        )
        self.assertIs(result.state, TaskStateName.TASK_FAILED)
        self.assertEqual(fsm.last_error_code, ErrorCodes.INVALID_TASK)
        self.assertEqual(fsm.last_validation_error, "timeout_not_positive")

    def test_illegal_target_slot_rejected(self):
        """显式 target_slot 不在 {T0,P1,P2,P3} 或与模式不符时必须拒绝。"""
        cases = [
            ("basic_nonexistent", MODE_BASIC, [BLUE], "T1"),
            ("basic_wrong_p", MODE_BASIC, [BLUE], "P1"),
            ("basic_lowercase", MODE_BASIC, [BLUE], "t0"),
            ("basic_garbage", MODE_BASIC, [BLUE], "SOMEWHERE"),
            ("basic_p4", MODE_BASIC, [BLUE], "P4"),
            ("sequence_t0", MODE_SEQUENCE, [BLUE, RED, YELLOW], "T0"),
            ("sequence_wrong_order", MODE_SEQUENCE, [BLUE, RED, YELLOW], "P2"),
        ]
        for name, mode, colors, bad_slot in cases:
            with self.subTest(case=name):
                fsm, result = self._submit_and_advance(
                    FakeGoal(task_id="bad-7", mode=mode, ordered_colors=colors, target_slot=bad_slot)
                )
                self.assertIs(result.state, TaskStateName.TASK_FAILED, name)
                self.assertEqual(fsm.last_error_code, ErrorCodes.INVALID_TASK, name)
                self.assertIn(
                    fsm.last_validation_error,
                    ("illegal_target_slot", "target_slot_mismatch"),
                    "%s -> %s" % (name, fsm.last_validation_error),
                )

    def test_legal_target_slot_accepted(self):
        """与模式/次序一致的 target_slot 必须被接受。"""
        for mode, colors, slot in (
            (MODE_BASIC, [BLUE], "T0"),
            (MODE_SEQUENCE, [BLUE, RED, YELLOW], "P1"),
        ):
            with self.subTest(slot=slot):
                fsm, result = self._submit_and_advance(
                    FakeGoal(task_id="ok-slot", mode=mode, ordered_colors=colors, target_slot=slot)
                )
                self.assertIs(result.state, TaskStateName.SCAN_SCENE, fsm.last_validation_error)

    def test_empty_task_id_rejected_without_state_change(self):
        fsm, _clock = make_fsm()
        result = fsm.submit_goal(FakeGoal(task_id="   ", mode=MODE_BASIC, ordered_colors=[BLUE]))
        self.assertTrue(result.goal_rejected)
        self.assertIs(result.state, TaskStateName.WAIT_TASK, "空 task_id 在校验前即被拒绝")
        self.assertEqual(fsm.last_rejection["reason"], "empty_task_id")
        self.assertEqual(fsm.last_rejection["error_code"], ErrorCodes.INVALID_TASK)

    def test_unknown_mode_rejected(self):
        result = validate_task_request(FakeGoal(task_id="m", mode=7, ordered_colors=[RED]))
        self.assertFalse(result.ok)
        self.assertEqual(result.error_code, ErrorCodes.INVALID_TASK)
        self.assertEqual(result.reason, "unknown_mode")

    def test_validation_does_not_guess_or_complete(self):
        """非法口令不得被猜测/补全：不产生任何 call_pick_place。"""
        fsm, result = self._submit_and_advance(
            FakeGoal(task_id="bad-8", mode=MODE_SEQUENCE, ordered_colors=[RED, BLUE])
        )
        self.assertIs(result.state, TaskStateName.TASK_FAILED)
        self.assertFalse(result.has(ActionKind.CALL_PICK_PLACE))
        self.assertFalse(fsm.has_active_task)

    def test_valid_task_passes_validation(self):
        ok = validate_task_request(
            FakeGoal(task_id="ok-1", mode=MODE_SEQUENCE, ordered_colors=[BLUE, RED, YELLOW], timeout_ms=5000)
        )
        self.assertTrue(ok.ok)
        self.assertEqual(ok.normalized_slots, ["P1", "P2", "P3"])


class TestConcurrentGoalRejection(unittest.TestCase):
    """4. 同一时刻只允许一个进行中 Goal；第二个必须明确拒绝且不排队。"""

    def test_second_goal_rejected_while_executing(self):
        fsm, _clock = make_fsm()
        fsm.submit_goal(FakeGoal(task_id="first", mode=MODE_SEQUENCE, ordered_colors=[BLUE, RED, YELLOW]))
        fsm.advance_validation()
        fsm.report_target("obj-0", expected_color=BLUE)
        self.assertIs(fsm.state, TaskStateName.EXECUTE_ITEM)
        self.assertTrue(fsm.has_active_task)

        second = fsm.submit_goal(FakeGoal(task_id="second", ordered_colors=[GREEN]))
        self.assertTrue(second.goal_rejected, "第二个 Goal 必须被拒绝")
        self.assertIs(second.state, TaskStateName.EXECUTE_ITEM, "拒绝不得改变当前状态")
        self.assertEqual(fsm.last_rejection["reason"], "task_already_in_progress")
        self.assertEqual(fsm.last_rejection["active_task_id"], "first")
        self.assertEqual(fsm.last_rejection["rejected_task_id"], "second")
        self.assertIn("not queued", fsm.last_rejection["message"])

        # 原任务不受影响，可继续完成并保持原 slot 映射
        fsm.report_item_result({"success": True, "placement_verified": True})
        fsm.record_completed_item("P1")
        self.assertEqual(fsm.completed_slots, ["P1"])
        self.assertEqual(fsm.current_task.task_id, "first")

    def test_second_goal_rejected_in_scan_and_record_and_cancel(self):
        for prepare in ("scan", "record", "cancel"):
            with self.subTest(phase=prepare):
                fsm, _clock = make_fsm()
                fsm.submit_goal(FakeGoal(task_id="t1", mode=MODE_BASIC, ordered_colors=[RED]))
                fsm.advance_validation()
                if prepare == "scan":
                    pass  # 已是 SCAN_SCENE
                elif prepare == "record":
                    fsm.report_target("o", expected_color=RED)
                    fsm.report_item_result({"success": True, "placement_verified": True})
                else:
                    fsm.request_cancel()
                self.assertTrue(fsm.has_active_task, prepare)
                rejected = fsm.submit_goal(FakeGoal(task_id="t2", mode=MODE_BASIC, ordered_colors=[BLUE]))
                self.assertTrue(rejected.goal_rejected, prepare)
                self.assertEqual(fsm.last_rejection["reason"], "task_already_in_progress", prepare)

    def test_goal_accepted_after_previous_task_reset(self):
        fsm, _clock = make_fsm()
        fsm.submit_goal(FakeGoal(task_id="t1", mode=MODE_BASIC, ordered_colors=[RED]))
        fsm.advance_validation()
        complete_one_item(fsm, "T0", RED)
        self.assertIs(fsm.state, TaskStateName.FINISHED)
        self.assertFalse(fsm.has_active_task, "FINISHED 不是进行中状态")

        fsm.reset()
        accepted = fsm.submit_goal(FakeGoal(task_id="t2", mode=MODE_BASIC, ordered_colors=[BLUE]))
        self.assertFalse(accepted.goal_rejected)
        self.assertIs(accepted.state, TaskStateName.VALIDATE_TASK)


class TestCancelSemantics(unittest.TestCase):
    """5. 取消不得立即成功：CANCEL_PENDING →（确认停稳）→ CANCELLED；未确认 → FAULT(520)。"""

    def test_cancel_confirmed_leads_to_cancelled(self):
        fsm, _clock = make_fsm()
        fsm.submit_goal(FakeGoal(task_id="c1", mode=MODE_SEQUENCE, ordered_colors=[BLUE, RED, YELLOW]))
        fsm.advance_validation()
        fsm.report_target("obj-0", expected_color=BLUE)

        result = fsm.request_cancel()
        self.assertIs(result.state, TaskStateName.CANCEL_PENDING, "取消必须先进入 CANCEL_PENDING")
        self.assertTrue(result.has(ActionKind.CANCEL_CHILD), "必须向子任务发取消")
        self.assertNotEqual(result.state, TaskStateName.CANCELLED, "不得立即返回取消成功")

        before = fsm.build_outcome()
        self.assertFalse(before.success)

        confirmed = fsm.confirm_child_stopped()
        self.assertIs(confirmed.state, TaskStateName.CANCELLED)
        outcome = fsm.build_outcome()
        self.assertFalse(outcome.success, "取消不是成功")
        self.assertEqual(outcome.error_code, ErrorCodes.OK)

    def test_cancel_not_confirmed_times_out_to_fault_520(self):
        fsm, clock = make_fsm(state_cancel_pending_timeout_s=5.0)
        fsm.submit_goal(FakeGoal(task_id="c2", mode=MODE_BASIC, ordered_colors=[RED]))
        fsm.advance_validation()
        fsm.report_target("obj-0", expected_color=RED)
        fsm.request_cancel()
        self.assertIs(fsm.state, TaskStateName.CANCEL_PENDING)

        clock.advance(4.0)
        self.assertIsNone(fsm.check_timeouts(), "未到超时不得转换")

        clock.advance(1.5)  # 累计 5.5s > 5.0s
        result = fsm.check_timeouts()
        self.assertIsNotNone(result)
        self.assertIs(result.state, TaskStateName.FAULT)
        self.assertEqual(fsm.last_error_code, ErrorCodes.CANCEL_NOT_CONFIRMED)
        self.assertTrue(result.has(ActionKind.REQUEST_STOP))
        self.assertTrue(result.has(ActionKind.LOCK_ACTIONS))

    def test_cancel_from_scan_scene_and_record_result(self):
        for phase in ("scan", "record"):
            with self.subTest(phase=phase):
                fsm, _clock = make_fsm()
                fsm.submit_goal(FakeGoal(task_id="c3", mode=MODE_BASIC, ordered_colors=[RED]))
                fsm.advance_validation()
                if phase == "record":
                    fsm.report_target("o", expected_color=RED)
                    fsm.report_item_result({"success": True, "placement_verified": True})
                self.assertIs(fsm.request_cancel().state, TaskStateName.CANCEL_PENDING, phase)

    def test_cancel_while_waiting_is_ignored(self):
        fsm, _clock = make_fsm()
        result = fsm.request_cancel()
        self.assertIs(result.state, TaskStateName.WAIT_TASK, "空闲时取消不应引入 CANCEL_PENDING")

    def test_late_stop_confirmation_after_fault_does_not_cancel(self):
        """已进入 FAULT 后迟到的停稳确认不得把状态改成 CANCELLED。"""
        fsm, clock = make_fsm(state_cancel_pending_timeout_s=1.0)
        fsm.submit_goal(FakeGoal(task_id="c4", mode=MODE_BASIC, ordered_colors=[RED]))
        fsm.advance_validation()
        fsm.request_cancel()
        clock.advance(2.0)
        self.assertIs(fsm.check_timeouts().state, TaskStateName.FAULT)
        late = fsm.confirm_child_stopped()
        self.assertIs(late.state, TaskStateName.FAULT)
        self.assertEqual(fsm.last_error_code, ErrorCodes.CANCEL_NOT_CONFIRMED)


class TestScanRecoveryBudget(unittest.TestCase):
    """6. SCAN_SCENE 找不到目标 → 限次重试后 TASK_FAILED。"""

    def test_target_not_found_retries_then_fails(self):
        fsm, _clock = make_fsm()
        fsm.config.recovery_max_attempts = 1
        fsm.submit_goal(FakeGoal(task_id="s1", mode=MODE_BASIC, ordered_colors=[BLUE]))
        fsm.advance_validation()
        self.assertIs(fsm.state, TaskStateName.SCAN_SCENE)

        first = fsm.report_target_missing("no blue candidate")
        self.assertIs(first.state, TaskStateName.RECOVERY)
        self.assertEqual(fsm.recovery_attempts, 0, "进入 RECOVERY 不消耗预算")
        self.assertEqual(fsm.last_error_code, ErrorCodes.TARGET_NOT_FOUND)

        retry = fsm.request_retry()
        self.assertIs(retry.state, TaskStateName.SCAN_SCENE, "预算内应允许重扫")
        self.assertEqual(fsm.recovery_attempts, 1, "重试获批后计入已用预算")

        second = fsm.report_target_missing("still no blue candidate")
        self.assertIs(second.state, TaskStateName.RECOVERY)
        self.assertEqual(fsm.recovery_attempts, 1)

        denied = fsm.request_retry()
        self.assertIs(denied.state, TaskStateName.TASK_FAILED, "重试耗尽必须 TASK_FAILED")
        self.assertEqual(fsm.last_error_code, ErrorCodes.TARGET_NOT_FOUND)
        self.assertFalse(fsm.build_outcome().success)
        self.assertFalse(fsm.has_active_task)

    def test_recovery_max_attempts_zero_fails_immediately(self):
        fsm, _clock = make_fsm()
        fsm.config.recovery_max_attempts = 0
        fsm.submit_goal(FakeGoal(task_id="s2", mode=MODE_BASIC, ordered_colors=[BLUE]))
        fsm.advance_validation()
        fsm.report_target_missing()
        self.assertIs(fsm.state, TaskStateName.RECOVERY)
        self.assertIs(fsm.request_retry().state, TaskStateName.TASK_FAILED)

    def test_recovery_budget_is_reset_per_item(self):
        fsm, _clock = make_fsm()
        fsm.config.recovery_max_attempts = 1
        fsm.submit_goal(FakeGoal(task_id="s3", mode=MODE_SEQUENCE, ordered_colors=[BLUE, RED, YELLOW]))
        fsm.advance_validation()

        fsm.report_target_missing()
        self.assertIs(fsm.request_retry().state, TaskStateName.SCAN_SCENE)

        fsm.report_target("obj-0", expected_color=BLUE)
        fsm.report_item_result({"success": True, "placement_verified": True})
        fsm.record_completed_item("P1")
        self.assertEqual(fsm.recovery_attempts, 0, "进入下一项应重置重试预算")

        fsm.report_target_missing()
        self.assertIs(fsm.request_retry().state, TaskStateName.SCAN_SCENE, "第二项应有独立预算")

    def test_failed_placement_placement_not_verified_is_not_success(self):
        """§4.4：placement_verified=false 时不得记为完成。"""
        fsm, _clock = make_fsm()
        fsm.submit_goal(FakeGoal(task_id="s4", mode=MODE_BASIC, ordered_colors=[RED]))
        fsm.advance_validation()
        fsm.report_target("o", expected_color=RED)
        result = fsm.report_item_result({"success": True, "placement_verified": False})
        self.assertIs(result.state, TaskStateName.RECOVERY)
        self.assertEqual(fsm.last_error_code, ErrorCodes.PLACEMENT_FAILED)
        self.assertEqual(fsm.completed_count, 0)

    def test_target_found_without_object_id_is_rejected(self):
        fsm, _clock = make_fsm()
        fsm.submit_goal(FakeGoal(task_id="s5", mode=MODE_BASIC, ordered_colors=[RED]))
        fsm.advance_validation()
        result = fsm.report_target("", expected_color=RED)
        self.assertIs(result.state, TaskStateName.SCAN_SCENE, "无 object_id 不得进入 EXECUTE_ITEM")
        self.assertEqual(fsm.last_error_code, ErrorCodes.TARGET_NOT_FOUND)

    def test_expected_color_mismatch_is_rejected(self):
        fsm, _clock = make_fsm()
        fsm.submit_goal(FakeGoal(task_id="s6", mode=MODE_BASIC, ordered_colors=[RED]))
        fsm.advance_validation()
        result = fsm.report_target("o", expected_color=GREEN)
        self.assertIs(result.state, TaskStateName.SCAN_SCENE)
        self.assertEqual(fsm.last_error_code, ErrorCodes.INVALID_TASK)


class TestStageTimeout(unittest.TestCase):
    """7. 单阶段超时进入预期状态。"""

    def test_cancel_pending_timeout_already_covered(self):
        self.assertTrue(True)  # 见 TestCancelSemantics

    def test_scan_scene_timeout_goes_to_recovery(self):
        fsm, clock = make_fsm(state_scan_timeout_s=5.0)
        fsm.submit_goal(FakeGoal(task_id="t-scan", mode=MODE_BASIC, ordered_colors=[BLUE]))
        fsm.advance_validation()
        clock.advance(5.5)
        result = fsm.check_timeouts()
        self.assertIsNotNone(result)
        self.assertIs(result.state, TaskStateName.RECOVERY)
        self.assertEqual(fsm.last_error_code, ErrorCodes.TARGET_NOT_FOUND)

    def test_execute_item_timeout_goes_to_fault_with_motion_timeout(self):
        fsm, clock = make_fsm(state_execute_item_timeout_s=10.0)
        fsm.submit_goal(FakeGoal(task_id="t-exec", mode=MODE_BASIC, ordered_colors=[BLUE]))
        fsm.advance_validation()
        fsm.report_target("o", expected_color=BLUE)
        self.assertIs(fsm.state, TaskStateName.EXECUTE_ITEM)

        clock.advance(9.0)
        self.assertIsNone(fsm.check_timeouts(), "未超时不得转换")

        clock.advance(1.5)
        result = fsm.check_timeouts()
        self.assertIsNotNone(result)
        self.assertIs(result.state, TaskStateName.FAULT, "运动在途超时不得自动重发")
        self.assertEqual(fsm.last_error_code, ErrorCodes.MOTION_TIMEOUT)

    def test_validate_task_timeout_goes_to_task_failed(self):
        fsm, clock = make_fsm(state_validate_task_timeout_s=2.0)
        fsm.submit_goal(FakeGoal(task_id="t-val", mode=MODE_BASIC, ordered_colors=[BLUE]))
        self.assertIs(fsm.state, TaskStateName.VALIDATE_TASK)
        clock.advance(2.5)
        result = fsm.check_timeouts()
        self.assertIsNotNone(result)
        self.assertIs(result.state, TaskStateName.TASK_FAILED)

    def test_no_timeout_configured_never_fires(self):
        fsm, clock = make_fsm()
        clock.advance(10_000.0)
        self.assertIsNone(fsm.check_timeouts(), "WAIT_TASK 不设超时")
        self.assertIs(fsm.state, TaskStateName.WAIT_TASK)

    def test_task_level_deadline_goes_to_fault(self):
        fsm, clock = make_fsm()
        fsm.submit_goal(FakeGoal(task_id="t-deadline", mode=MODE_BASIC, ordered_colors=[BLUE], timeout_ms=3000))
        fsm.advance_validation()
        clock.advance(2.0)
        self.assertIsNone(fsm.check_task_deadline())
        clock.advance(1.5)
        result = fsm.check_task_deadline()
        self.assertIsNotNone(result)
        self.assertIs(result.state, TaskStateName.FAULT)
        self.assertEqual(fsm.last_error_code, ErrorCodes.MOTION_TIMEOUT)

    def test_timeout_does_not_fire_after_leaving_state(self):
        fsm, clock = make_fsm(state_scan_timeout_s=5.0)
        fsm.submit_goal(FakeGoal(task_id="t-move", mode=MODE_BASIC, ordered_colors=[BLUE]))
        fsm.advance_validation()
        clock.advance(4.0)
        fsm.report_target("o", expected_color=BLUE)
        self.assertIs(fsm.state, TaskStateName.EXECUTE_ITEM)
        self.assertIsNone(fsm.check_timeouts(), "进入新状态后计时应重置")


class TestMotionStatusUnknown(unittest.TestCase):
    """8. 运动状态未知(230) → FAULT，且不自动重试。"""

    def test_motion_status_unknown_goes_fault_without_retry(self):
        fsm, _clock = make_fsm()
        fsm.submit_goal(FakeGoal(task_id="u1", mode=MODE_BASIC, ordered_colors=[RED]))
        fsm.advance_validation()
        fsm.report_target("o", expected_color=RED)

        result = fsm.report_item_result(
            {
                "success": False,
                "placement_verified": False,
                "error_code": ErrorCodes.MOTION_STATUS_UNKNOWN,
                "message": "motion feedback lost",
            }
        )
        self.assertIs(result.state, TaskStateName.FAULT)
        self.assertEqual(fsm.last_error_code, ErrorCodes.MOTION_STATUS_UNKNOWN)
        self.assertEqual(fsm.recovery_attempts, 0, "不得进入 RECOVERY 消耗重试预算")
        self.assertTrue(result.has(ActionKind.REQUEST_STOP))
        self.assertTrue(result.has(ActionKind.LOCK_ACTIONS))
        self.assertFalse(result.has(ActionKind.CALL_PICK_PLACE), "不得自动重发运动指令")

    def test_recoverable_error_does_enter_recovery(self):
        fsm, _clock = make_fsm()
        fsm.submit_goal(FakeGoal(task_id="u2", mode=MODE_BASIC, ordered_colors=[RED]))
        fsm.advance_validation()
        fsm.report_target("o", expected_color=RED)
        result = fsm.report_item_result({"success": False, "error_code": ErrorCodes.PLANNING_FAILED})
        self.assertIs(result.state, TaskStateName.RECOVERY)
        self.assertEqual(fsm.recovery_attempts, 0, "进入 RECOVERY 尚未消耗预算")
        self.assertIs(fsm.request_retry().state, TaskStateName.SCAN_SCENE)
        self.assertEqual(fsm.recovery_attempts, 1, "重试获批后预算 +1")

    def test_other_non_recoverable_errors_go_fault(self):
        for code in (
            ErrorCodes.MOTION_TIMEOUT,
            ErrorCodes.SAFETY_INTERLOCK,
            ErrorCodes.HARDWARE_FAULT,
            ErrorCodes.UNLOCK_FAILED,
            ErrorCodes.OBJECT_DROPPED,
        ):
            with self.subTest(code=code):
                fsm, _clock = make_fsm()
                fsm.submit_goal(FakeGoal(task_id="u3", mode=MODE_BASIC, ordered_colors=[RED]))
                fsm.advance_validation()
                fsm.report_target("o", expected_color=RED)
                result = fsm.report_item_result({"success": False, "error_code": code})
                self.assertIs(result.state, TaskStateName.FAULT, ErrorCodes.name_of(code))
                self.assertEqual(fsm.last_error_code, code)

    def test_error_code_names_match_interface(self):
        self.assertEqual(ErrorCodes.INVALID_TASK, 100)
        self.assertEqual(ErrorCodes.TARGET_NOT_FOUND, 110)
        self.assertEqual(ErrorCodes.MOTION_STATUS_UNKNOWN, 230)
        self.assertEqual(ErrorCodes.CANCEL_NOT_CONFIRMED, 520)
        self.assertEqual(ErrorCodes.name_of(520), "CANCEL_NOT_CONFIRMED")


class TestStateSnapshotAndEvents(unittest.TestCase):
    """TaskState 快照字段与转移日志。"""

    def test_snapshot_tracks_progress(self):
        fsm, _clock = make_fsm()
        fsm.submit_goal(FakeGoal(task_id="snap", mode=MODE_SEQUENCE, ordered_colors=[BLUE, RED, YELLOW]))
        snapshot = fsm.snapshot()
        self.assertEqual(snapshot.task_id, "snap")
        self.assertEqual(snapshot.state, "VALIDATE_TASK")
        self.assertEqual(snapshot.total_count, 3)
        self.assertEqual(snapshot.completed_count, 0)

    def test_publish_state_emitted_on_every_transition(self):
        fsm, _clock = make_fsm()
        result = fsm.submit_goal(FakeGoal(task_id="pub", mode=MODE_BASIC, ordered_colors=[RED]))
        self.assertTrue(result.has(ActionKind.PUBLISH_STATE))
        self.assertEqual(result.find(ActionKind.PUBLISH_STATE).payload["snapshot"].state, "VALIDATE_TASK")

    def test_transition_log_records_history(self):
        fsm, _clock = make_fsm()
        fsm.submit_goal(FakeGoal(task_id="log", mode=MODE_BASIC, ordered_colors=[RED]))
        fsm.advance_validation()
        states = [entry[2] for entry in fsm.transition_log]
        self.assertIn("WAIT_TASK", states)
        self.assertIn("VALIDATE_TASK", states)
        self.assertIn("SCAN_SCENE", states)

    def test_unknown_event_is_ignored_safely(self):
        fsm, _clock = make_fsm()
        result = fsm.step("totally_unknown_event")
        self.assertIs(result.state, TaskStateName.WAIT_TASK)
        self.assertEqual(result.actions, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
