// Copyright 2026 meituan_challenge_ws contributors
//
// Task FSM 安全语义测试：并发 Goal 拒绝、超时处置、取消与停稳确认、恢复预算、
// 状态未知锁定、任务级截止时间。
//
// 对齐 Python src/mtc_task/test/test_task_fsm.py 与 tests/test_mock_end_to_end.py
// 中的故障路径用例。所有用例均为离线执行，不接触 ROS 图或硬件。

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "mtc_core_cpp/clock.hpp"
#include "mtc_core_cpp/error_codes.hpp"
#include "mtc_task_cpp/task_fsm.hpp"

namespace
{

using mtc::core::ErrorCodes;
using mtc::core::ManualClock;
using mtc::task::ActionKind;
using mtc::task::EventPayload;
using mtc::task::FsmsConfig;
using mtc::task::StepResult;
using mtc::task::TaskFsm;
using mtc::task::TaskRequest;
using mtc::task::TaskStateName;

TaskRequest make_basic_task(int color = 1, const std::string & task_id = "task-1")
{
  TaskRequest task;
  task.task_id = task_id;
  task.mode = mtc::task::TaskMode::MODE_BASIC;
  task.ordered_colors = {color};
  task.timeout_ms = 60000;
  return task;
}

TaskRequest make_sequence_task(const std::string & task_id = "task-seq")
{
  TaskRequest task;
  task.task_id = task_id;
  task.mode = mtc::task::TaskMode::MODE_SEQUENCE;
  task.ordered_colors = {1, 2, 3};
  task.timeout_ms = 60000;
  return task;
}

struct Harness
{
  std::shared_ptr<ManualClock> clock = std::make_shared<ManualClock>(1000.0);
  TaskFsm fsm{clock, FsmsConfig::defaults()};

  void boot()
  {
    fsm.start();
    fsm.report_self_check(true);
  }

  /// 受理任务并推进到 SCAN_SCENE。
  void reach_scan_scene(int color = 1, const std::string & task_id = "task-1")
  {
    fsm.submit_goal(make_basic_task(color, task_id));
    fsm.advance_validation();
  }

  /// 推进到 EXECUTE_ITEM。
  void reach_execute_item(const std::string & object_id = "battery_1", int color = 1)
  {
    reach_scan_scene(color);
    fsm.report_target(object_id, color);
  }

  /// 在 SCAN_SCENE 中制造一次目标丢失 -> RECOVERY。
  void fail_scan(const std::string & reason = "no unique candidate")
  {
    fsm.report_target_missing(reason);
  }
};

}  // namespace

// ---------------------------------------------------------------------------
// §7.2 并发 Goal 拒绝
// ---------------------------------------------------------------------------

TEST(TaskFsmConcurrency, SecondGoalRejectedInEveryActiveState)
{
  // 在活动的每个阶段尝试提交第二个 Goal：必须明确拒绝、不得排队、不得切换任务。
  const std::vector<TaskStateName> probe_states = {
    TaskStateName::VALIDATE_TASK,
    TaskStateName::SCAN_SCENE,
    TaskStateName::EXECUTE_ITEM,
    TaskStateName::RECORD_RESULT,
    TaskStateName::RECOVERY,
    TaskStateName::CANCEL_PENDING,
  };

  for (const auto probe_state : probe_states) {
    Harness harness;
    harness.boot();
    harness.fsm.submit_goal(make_basic_task(1, "task-active"));

    switch (probe_state) {
      case TaskStateName::VALIDATE_TASK:
        break;
      case TaskStateName::SCAN_SCENE:
        harness.fsm.advance_validation();
        break;
      case TaskStateName::EXECUTE_ITEM:
        harness.fsm.advance_validation();
        harness.fsm.report_target("battery_1", 1);
        break;
      case TaskStateName::RECORD_RESULT:
        harness.fsm.advance_validation();
        harness.fsm.report_target("battery_1", 1);
        harness.fsm.report_item_result(true, true, 0, "ok");
        break;
      case TaskStateName::RECOVERY:
        harness.fsm.advance_validation();
        harness.fsm.report_target_missing("not found");
        break;
      case TaskStateName::CANCEL_PENDING:
        harness.fsm.advance_validation();
        harness.fsm.request_cancel();
        break;
      default:
        break;
    }

    ASSERT_EQ(harness.fsm.state(), probe_state)
      << "harness failed to reach " << mtc::task::to_string(probe_state);

    const StepResult rejected = harness.fsm.submit_goal(make_basic_task(2, "task-second"));

    // 状态不变、任务不被切换
    EXPECT_EQ(rejected.state, probe_state);
    EXPECT_TRUE(rejected.goal_rejected);
    EXPECT_EQ(harness.fsm.state(), probe_state);
    ASSERT_TRUE(harness.fsm.current_task.has_value());
    EXPECT_EQ(harness.fsm.current_task->task_id, "task-active");

    // 拒绝原因必须明确，且说明"不排队"
    ASSERT_TRUE(harness.fsm.last_rejection.has_value());
    EXPECT_EQ(harness.fsm.last_rejection->reason, "task_already_in_progress");
    EXPECT_EQ(harness.fsm.last_rejection->error_code, ErrorCodes::EXECUTION_REJECTED);
    EXPECT_NE(harness.fsm.last_rejection->message.find("not queued"), std::string::npos);
    EXPECT_EQ(harness.fsm.last_rejection->active_task_id, "task-active");
    EXPECT_EQ(harness.fsm.last_rejection->rejected_task_id, "task-second");
    EXPECT_EQ(harness.fsm.last_rejection->active_state, mtc::task::to_string(probe_state));
  }
}

TEST(TaskFsmConcurrency, MissingGoalPayloadRejected)
{
  Harness harness;
  harness.boot();
  const StepResult rejected = harness.fsm.step(mtc::task::Event::GOAL_ACCEPTED, EventPayload{});
  EXPECT_TRUE(rejected.goal_rejected);
  EXPECT_EQ(rejected.state, TaskStateName::WAIT_TASK);
  ASSERT_TRUE(harness.fsm.last_rejection.has_value());
  EXPECT_EQ(harness.fsm.last_rejection->reason, "missing_goal");
  EXPECT_EQ(harness.fsm.last_rejection->error_code, ErrorCodes::INVALID_TASK);
  EXPECT_FALSE(harness.fsm.current_task.has_value());
}

TEST(TaskFsmConcurrency, UnparsableGoalRejected)
{
  Harness harness;
  harness.boot();
  const StepResult rejected = harness.fsm.submit_unparsable_goal("bad field types");
  EXPECT_TRUE(rejected.goal_rejected);
  EXPECT_EQ(rejected.state, TaskStateName::WAIT_TASK);
  ASSERT_TRUE(harness.fsm.last_rejection.has_value());
  EXPECT_EQ(harness.fsm.last_rejection->reason, "unparsable_goal");
  EXPECT_NE(
    harness.fsm.last_rejection->message.find("bad field types"), std::string::npos);
}

TEST(TaskFsmConcurrency, EmptyTaskIdRejected)
{
  Harness harness;
  harness.boot();
  TaskRequest task = make_basic_task();
  task.task_id = "   ";
  const StepResult rejected = harness.fsm.submit_goal(task);
  EXPECT_TRUE(rejected.goal_rejected);
  EXPECT_EQ(rejected.state, TaskStateName::WAIT_TASK);
  ASSERT_TRUE(harness.fsm.last_rejection.has_value());
  EXPECT_EQ(harness.fsm.last_rejection->reason, "empty_task_id");
  EXPECT_EQ(harness.fsm.last_rejection->error_code, ErrorCodes::INVALID_TASK);
}

TEST(TaskFsmConcurrency, RejectionDoesNotQueueSecondGoal)
{
  Harness harness;
  harness.boot();
  harness.fsm.submit_goal(make_basic_task(1, "task-active"));
  harness.fsm.advance_validation();

  harness.fsm.submit_goal(make_basic_task(2, "task-second"));
  harness.fsm.submit_goal(make_basic_task(3, "task-third"));
  EXPECT_EQ(harness.fsm.current_task->task_id, "task-active");

  // 完成当前任务后必须回到 FINISHED，而不是接着执行被排队的目标
  harness.fsm.report_target("battery_1", 1);
  harness.fsm.report_item_result(true, true, 0, "ok");
  EXPECT_EQ(harness.fsm.record_completed_item().state, TaskStateName::FINISHED);
  EXPECT_EQ(harness.fsm.current_task->task_id, "task-active");
}

// ---------------------------------------------------------------------------
// 超时处置（§7.1 默认处置表）
// ---------------------------------------------------------------------------

TEST(TaskFsmTimeout, WaitTaskAndTerminalStatesHaveNoTimeout)
{
  Harness harness;
  harness.boot();
  EXPECT_EQ(harness.fsm.state(), TaskStateName::WAIT_TASK);
  harness.clock->advance(10000.0);
  EXPECT_FALSE(harness.fsm.check_timeouts().has_value());
}

TEST(TaskFsmTimeout, ValidateTaskTimeoutGoesToTaskFailedWithInvalidTask)
{
  Harness harness;
  harness.boot();
  harness.fsm.submit_goal(make_basic_task());
  ASSERT_EQ(harness.fsm.state(), TaskStateName::VALIDATE_TASK);

  harness.clock->advance(2.0);   // VALIDATE_TASK 默认超时 2.0s
  const auto result = harness.fsm.check_timeouts();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->state, TaskStateName::TASK_FAILED);
  EXPECT_EQ(harness.fsm.last_error_code, ErrorCodes::INVALID_TASK);
  EXPECT_FALSE(harness.fsm.build_outcome().success);
}

TEST(TaskFsmTimeout, ScanSceneTimeoutGoesToRecoveryWithTargetNotFound)
{
  Harness harness;
  harness.boot();
  harness.reach_scan_scene();
  ASSERT_EQ(harness.fsm.state(), TaskStateName::SCAN_SCENE);

  harness.clock->advance(5.0);   // SCAN_SCENE 默认超时 5.0s
  const auto result = harness.fsm.check_timeouts();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->state, TaskStateName::RECOVERY);
  EXPECT_EQ(harness.fsm.last_error_code, ErrorCodes::TARGET_NOT_FOUND);
}

TEST(TaskFsmTimeout, ExecuteItemTimeoutGoesToFaultAndDoesNotAutoResend)
{
  Harness harness;
  harness.boot();
  harness.reach_execute_item();
  ASSERT_EQ(harness.fsm.state(), TaskStateName::EXECUTE_ITEM);

  harness.clock->advance(60.0);   // EXECUTE_ITEM 默认超时 60.0s
  const auto result = harness.fsm.check_timeouts();
  ASSERT_TRUE(result.has_value());

  // 运动状态不确定：必须进入 FAULT(220)，且不得自动重发
  EXPECT_EQ(result->state, TaskStateName::FAULT);
  EXPECT_EQ(harness.fsm.last_error_code, ErrorCodes::MOTION_TIMEOUT);
  EXPECT_TRUE(result->has(ActionKind::REQUEST_STOP));
  EXPECT_TRUE(result->has(ActionKind::LOCK_ACTIONS));

  // FAULT 是终态：继续推进不会再次触发任何动作（等价于"禁止自动重发"）
  const size_t log_after_fault = harness.fsm.transition_log.size();
  harness.clock->advance(1000.0);
  EXPECT_FALSE(harness.fsm.check_timeouts().has_value());
  EXPECT_EQ(harness.fsm.state(), TaskStateName::FAULT);
  EXPECT_EQ(harness.fsm.transition_log.size(), log_after_fault);
}

TEST(TaskFsmTimeout, RecordResultTimeoutMapsToPlacementFailed)
{
  Harness harness;
  harness.boot();
  harness.reach_execute_item();
  harness.fsm.report_item_result(true, true, 0, "ok");
  ASSERT_EQ(harness.fsm.state(), TaskStateName::RECORD_RESULT);

  harness.clock->advance(2.0);
  const auto result = harness.fsm.check_timeouts();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->state, TaskStateName::TASK_FAILED);
  EXPECT_EQ(harness.fsm.last_error_code, ErrorCodes::PLACEMENT_FAILED);
}

TEST(TaskFsmTimeout, RecoveryTimeoutGoesToTaskFailed)
{
  Harness harness;
  harness.boot();
  harness.reach_scan_scene();
  harness.fail_scan("lost");
  ASSERT_EQ(harness.fsm.state(), TaskStateName::RECOVERY);
  const double entered = harness.clock->now();

  harness.clock->set(entered + 10.0);   // RECOVERY 默认超时 10.0s
  const auto result = harness.fsm.check_timeouts();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->state, TaskStateName::TASK_FAILED);
}

TEST(TaskFsmTimeout, CustomTimeoutsAreInjectible)
{
  auto clock = std::make_shared<ManualClock>(0.0);
  FsmsConfig config = FsmsConfig::defaults();
  config.state_timeouts[TaskStateName::VALIDATE_TASK] = 0.25;

  TaskFsm fsm(clock, config);
  fsm.start();
  fsm.report_self_check(true);
  fsm.submit_goal(make_basic_task());
  ASSERT_EQ(fsm.state(), TaskStateName::VALIDATE_TASK);

  clock->advance(0.24);
  EXPECT_FALSE(fsm.check_timeouts().has_value());
  clock->advance(0.02);
  EXPECT_TRUE(fsm.check_timeouts().has_value());
}

TEST(TaskFsmTimeout, TimeoutTimerResetsOnEffectiveProgress)
{
  // guard 通过后必须重置计时基准，否则"长时间运行但持续有进展"会被误判超时。
  Harness harness;
  harness.boot();
  harness.reach_execute_item();
  harness.fsm.report_item_result(true, true, 0, "ok");
  ASSERT_EQ(harness.fsm.state(), TaskStateName::RECORD_RESULT);

  harness.clock->advance(1.9);
  EXPECT_FALSE(harness.fsm.check_timeouts().has_value());
}

// ---------------------------------------------------------------------------
// 任务级截止时间
// ---------------------------------------------------------------------------

TEST(TaskFsmDeadline, DeadlineExceededGoesToFaultWithMotionTimeout)
{
  Harness harness;
  harness.boot();
  TaskRequest task = make_basic_task();
  task.timeout_ms = 5000;
  harness.fsm.submit_goal(task);
  harness.fsm.advance_validation();

  EXPECT_FALSE(harness.fsm.check_task_deadline().has_value());
  harness.clock->advance(5.0);

  const auto result = harness.fsm.check_task_deadline();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->state, TaskStateName::FAULT);
  EXPECT_EQ(harness.fsm.last_error_code, ErrorCodes::MOTION_TIMEOUT);
  EXPECT_NE(harness.fsm.last_error_message.find("timeout_ms=5000"), std::string::npos);
}

TEST(TaskFsmDeadline, NoDeadlineWithoutActiveTask)
{
  Harness harness;
  harness.boot();
  harness.clock->advance(100000.0);
  EXPECT_FALSE(harness.fsm.check_task_deadline().has_value());
}

TEST(TaskFsmDeadline, ElapsedTaskTimeTracksInProgressTask)
{
  Harness harness;
  harness.boot();
  EXPECT_FALSE(harness.fsm.elapsed_task_time().has_value());

  harness.fsm.submit_goal(make_basic_task());
  harness.clock->advance(3.0);
  const auto elapsed = harness.fsm.elapsed_task_time();
  ASSERT_TRUE(elapsed.has_value());
  EXPECT_DOUBLE_EQ(*elapsed, 3.0);
}

// ---------------------------------------------------------------------------
// 取消与停稳确认（§4.3：不可无等待地返回 CANCELLED）
// ---------------------------------------------------------------------------

TEST(TaskFsmCancel, CancelFromScanSceneEntersCancelPendingWithChildCancel)
{
  Harness harness;
  harness.boot();
  harness.reach_scan_scene();

  const StepResult cancel = harness.fsm.request_cancel();
  EXPECT_EQ(cancel.state, TaskStateName::CANCEL_PENDING);

  const auto * child = cancel.find(ActionKind::CANCEL_CHILD);
  ASSERT_NE(child, nullptr);
  EXPECT_EQ(child->request_id, "task-1#0");

  // 尚未确认停稳：不得立即声称已取消
  EXPECT_NE(harness.fsm.state(), TaskStateName::CANCELLED);
}

TEST(TaskFsmCancel, ConfirmedStopLeadsToCancelled)
{
  Harness harness;
  harness.boot();
  harness.reach_scan_scene();
  harness.fsm.request_cancel();

  const StepResult stopped = harness.fsm.confirm_child_stopped();
  EXPECT_EQ(stopped.state, TaskStateName::CANCELLED);

  const auto outcome = harness.fsm.build_outcome();
  EXPECT_FALSE(outcome.success);
  EXPECT_EQ(outcome.error_code, ErrorCodes::OK);
  EXPECT_NE(outcome.message.find("cancelled after stop confirmation"), std::string::npos);
}

TEST(TaskFsmCancel, UnconfirmedStopTimesOutToFault520)
{
  Harness harness;
  harness.boot();
  harness.reach_scan_scene();
  harness.fsm.request_cancel();
  ASSERT_EQ(harness.fsm.state(), TaskStateName::CANCEL_PENDING);

  // 未超时前不得转移
  harness.clock->advance(4.9);
  EXPECT_FALSE(harness.fsm.check_timeouts().has_value());
  EXPECT_EQ(harness.fsm.state(), TaskStateName::CANCEL_PENDING);

  harness.clock->advance(0.1);   // CANCEL_PENDING 默认超时 5.0s
  const auto result = harness.fsm.check_timeouts();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->state, TaskStateName::FAULT);
  EXPECT_EQ(harness.fsm.last_error_code, ErrorCodes::CANCEL_NOT_CONFIRMED);
  EXPECT_TRUE(result->has(ActionKind::REQUEST_STOP));
  EXPECT_TRUE(result->has(ActionKind::LOCK_ACTIONS));
}

TEST(TaskFsmCancel, LateStopConfirmationAfterFaultDoesNotBecomeCancelled)
{
  Harness harness;
  harness.boot();
  harness.reach_scan_scene();
  harness.fsm.request_cancel();
  harness.clock->advance(5.0);
  ASSERT_EQ(harness.fsm.check_timeouts()->state, TaskStateName::FAULT);

  // 迟到的停稳确认不得把 FAULT 改写成 CANCELLED
  const StepResult late = harness.fsm.confirm_child_stopped();
  EXPECT_EQ(late.state, TaskStateName::FAULT);
  EXPECT_EQ(harness.fsm.last_error_code, ErrorCodes::CANCEL_NOT_CONFIRMED);
}

TEST(TaskFsmCancel, CancelFromExecuteItemIsAllowed)
{
  Harness harness;
  harness.boot();
  harness.reach_execute_item();
  ASSERT_EQ(harness.fsm.state(), TaskStateName::EXECUTE_ITEM);

  const StepResult cancel = harness.fsm.request_cancel();
  EXPECT_EQ(cancel.state, TaskStateName::CANCEL_PENDING);
  EXPECT_TRUE(cancel.has(ActionKind::CANCEL_CHILD));
}

TEST(TaskFsmCancel, CancelFromRecoveryIsAllowed)
{
  Harness harness;
  harness.boot();
  harness.reach_scan_scene();
  harness.fail_scan("lost");
  ASSERT_EQ(harness.fsm.state(), TaskStateName::RECOVERY);
  EXPECT_EQ(harness.fsm.request_cancel().state, TaskStateName::CANCEL_PENDING);
}

TEST(TaskFsmCancel, ItemUnsafeInCancelPendingGoesToFault)
{
  Harness harness;
  harness.boot();
  harness.reach_scan_scene();
  harness.fsm.request_cancel();

  const StepResult result = harness.fsm.report_item_unsafe(
    ErrorCodes::MOTION_STATUS_UNKNOWN, "motion state unknown during cancel");
  EXPECT_EQ(result.state, TaskStateName::FAULT);
  EXPECT_EQ(harness.fsm.last_error_code, ErrorCodes::MOTION_STATUS_UNKNOWN);
}

TEST(TaskFsmCancel, CancelInTerminalStateIsIgnored)
{
  Harness harness;
  harness.boot();
  harness.reach_scan_scene();
  harness.fsm.request_cancel();
  harness.fsm.confirm_child_stopped();
  ASSERT_EQ(harness.fsm.state(), TaskStateName::CANCELLED);

  EXPECT_EQ(harness.fsm.request_cancel().state, TaskStateName::CANCELLED);
}

// ---------------------------------------------------------------------------
// 恢复与重试预算
// ---------------------------------------------------------------------------

TEST(TaskFsmRecovery, TargetNotFoundRetryReturnsToScanScene)
{
  Harness harness;
  harness.boot();
  harness.reach_scan_scene();
  harness.fail_scan("no unique candidate");

  EXPECT_EQ(harness.fsm.state(), TaskStateName::RECOVERY);
  EXPECT_EQ(harness.fsm.last_error_code, ErrorCodes::TARGET_NOT_FOUND);
  EXPECT_EQ(harness.fsm.recovery_attempts, 0);   // 进入 RECOVERY 不占预算

  const StepResult retry = harness.fsm.request_retry();
  EXPECT_EQ(retry.state, TaskStateName::SCAN_SCENE);
  EXPECT_EQ(harness.fsm.recovery_attempts, 1);
}

TEST(TaskFsmRecovery, RetryBudgetExhaustionGoesToTaskFailed)
{
  Harness harness;
  harness.boot();
  harness.reach_scan_scene();

  harness.fail_scan("first failure");
  ASSERT_EQ(harness.fsm.request_retry().state, TaskStateName::SCAN_SCENE);

  // 第二次失败：预算已耗尽（recovery_max_attempts=1）
  harness.fail_scan("second failure");
  ASSERT_EQ(harness.fsm.state(), TaskStateName::RECOVERY);
  const StepResult exhausted = harness.fsm.request_retry();
  EXPECT_EQ(exhausted.state, TaskStateName::TASK_FAILED);
  EXPECT_FALSE(harness.fsm.build_outcome().success);
}

TEST(TaskFsmRecovery, DenyRetryGoesToTaskFailed)
{
  Harness harness;
  harness.boot();
  harness.reach_scan_scene();
  harness.fail_scan("lost");

  const StepResult denied = harness.fsm.deny_retry();
  EXPECT_EQ(denied.state, TaskStateName::TASK_FAILED);
  EXPECT_EQ(harness.fsm.recovery_attempts, 0);
}

TEST(TaskFsmRecovery, PlanningFailureIsRecoverable)
{
  Harness harness;
  harness.boot();
  harness.reach_execute_item();

  const StepResult result = harness.fsm.report_item_result(
    false, false, ErrorCodes::PLANNING_FAILED, "no IK solution");
  EXPECT_EQ(result.state, TaskStateName::RECOVERY);
  EXPECT_EQ(harness.fsm.last_error_code, ErrorCodes::PLANNING_FAILED);
}

TEST(TaskFsmRecovery, PoseStaleIsRecoverable)
{
  Harness harness;
  harness.boot();
  harness.reach_execute_item();

  const StepResult result = harness.fsm.report_item_result(
    false, false, ErrorCodes::POSE_STALE, "tf too old");
  EXPECT_EQ(result.state, TaskStateName::RECOVERY);
}

TEST(TaskFsmRecovery, ExecutionRejectedIsRecoverable)
{
  Harness harness;
  harness.boot();
  harness.reach_execute_item();

  const StepResult result = harness.fsm.report_item_result(
    false, false, ErrorCodes::EXECUTION_REJECTED, "backend rejected");
  EXPECT_EQ(result.state, TaskStateName::RECOVERY);
}

TEST(TaskFsmRecovery, RecoveryAttemptsResetAfterCompletedItem)
{
  auto clock = std::make_shared<ManualClock>(0.0);
  TaskFsm fsm(clock, FsmsConfig::defaults());
  fsm.start();
  fsm.report_self_check(true);
  fsm.submit_goal(make_sequence_task());
  fsm.advance_validation();

  // 第一项先失败一次再成功；成功后 recovery_attempts 必须归零
  fsm.report_target("battery_0", 1);
  fsm.report_item_result(false, false, ErrorCodes::TARGET_NOT_FOUND, "lost");
  ASSERT_EQ(fsm.state(), TaskStateName::RECOVERY);
  ASSERT_EQ(fsm.request_retry().state, TaskStateName::SCAN_SCENE);
  EXPECT_EQ(fsm.recovery_attempts, 1);

  fsm.report_target("battery_0", 1);
  fsm.report_item_result(true, true, 0, "ok");
  ASSERT_EQ(fsm.record_completed_item().state, TaskStateName::SCAN_SCENE);
  EXPECT_EQ(fsm.recovery_attempts, 0);
  EXPECT_EQ(fsm.completed_count, 1u);
}

// ---------------------------------------------------------------------------
// 不可重试错误：直接 FAULT 并保留真实错误码
// ---------------------------------------------------------------------------

TEST(TaskFsmUnsafeErrors, MotionStatusUnknownGoesToFaultWithRealCode)
{
  Harness harness;
  harness.boot();
  harness.reach_execute_item();

  const StepResult result = harness.fsm.report_item_result(
    false, false, ErrorCodes::MOTION_STATUS_UNKNOWN, "feedback stream lost");

  EXPECT_EQ(result.state, TaskStateName::FAULT);
  // FAULT 快照必须保留 230，而不是退化为 HARDWARE_FAULT(510)
  EXPECT_EQ(harness.fsm.last_error_code, ErrorCodes::MOTION_STATUS_UNKNOWN);
  EXPECT_EQ(harness.fsm.build_outcome().error_code, ErrorCodes::MOTION_STATUS_UNKNOWN);
  EXPECT_TRUE(result.has(ActionKind::REQUEST_STOP));
  EXPECT_TRUE(result.has(ActionKind::LOCK_ACTIONS));
}

TEST(TaskFsmUnsafeErrors, NonRecoverableCodesAllGoToFault)
{
  const std::vector<uint16_t> codes = {
    ErrorCodes::MOTION_STATUS_UNKNOWN,
    ErrorCodes::MOTION_TIMEOUT,
    ErrorCodes::SAFETY_INTERLOCK,
    ErrorCodes::HARDWARE_FAULT,
    ErrorCodes::UNLOCK_FAILED,
    ErrorCodes::RELEASE_NOT_VERIFIED,
    ErrorCodes::OBJECT_DROPPED,
    ErrorCodes::CANCEL_NOT_CONFIRMED,
  };

  for (const auto code : codes) {
    Harness harness;
    harness.boot();
    harness.reach_execute_item();

    const StepResult result = harness.fsm.report_item_result(
      false, false, static_cast<int>(code), "non recoverable");
    EXPECT_EQ(result.state, TaskStateName::FAULT)
      << "code " << code << " must go to FAULT";
    EXPECT_EQ(harness.fsm.last_error_code, code)
      << "code " << code << " must be preserved in FAULT";
  }
}

TEST(TaskFsmUnsafeErrors, RecoverableWhitelistIsConservative)
{
  const FsmsConfig config = FsmsConfig::defaults();

  for (const auto code : mtc::task::default_recoverable_errors()) {
    EXPECT_TRUE(config.is_recoverable(static_cast<int>(code)));
  }
  // 白名单之外（含未知码）一律不可重试
  EXPECT_FALSE(config.is_recoverable(ErrorCodes::MOTION_STATUS_UNKNOWN));
  EXPECT_FALSE(config.is_recoverable(ErrorCodes::SAFETY_INTERLOCK));
  EXPECT_FALSE(config.is_recoverable(9999));
  EXPECT_FALSE(config.is_recoverable(-1));
}

TEST(TaskFsmUnsafeErrors, FaultWithoutSpecificCodeDegradesToHardwareFault)
{
  // 进入 FAULT 时若没有任何具体错误码，必须退化为 HARDWARE_FAULT(510)，
  // 而不是把 OK(0) 当作"成功"留在快照里。
  auto clock = std::make_shared<ManualClock>(0.0);
  TaskFsm fsm(clock, FsmsConfig::defaults());
  fsm.start();

  // INIT -> SELF_CHECK_FAIL -> FAULT，且无具体错误码
  fsm.report_self_check(false);
  ASSERT_EQ(fsm.state(), TaskStateName::FAULT);
  EXPECT_EQ(fsm.last_error_code, ErrorCodes::HARDWARE_FAULT);
  EXPECT_EQ(
    fsm.last_error_message, "entered FAULT without a specific error code");
}

TEST(TaskFsmUnsafeErrors, FaultKeepsSpecificCodeWhenPresent)
{
  Harness harness;
  harness.boot();
  harness.reach_execute_item();

  harness.fsm.report_item_result(false, false, ErrorCodes::SAFETY_INTERLOCK, "guard tripped");
  ASSERT_EQ(harness.fsm.state(), TaskStateName::FAULT);
  // 有具体原因时不得被 HARDWARE_FAULT 覆盖
  EXPECT_EQ(harness.fsm.last_error_code, ErrorCodes::SAFETY_INTERLOCK);
}

TEST(TaskFsmUnsafeErrors, ItemFailureWithZeroCodeDefaultsToPlacementFailed)
{
  Harness harness;
  harness.boot();
  harness.reach_execute_item();

  const StepResult result = harness.fsm.report_item_result(false, false, 0, "unspecified");
  EXPECT_EQ(result.state, TaskStateName::RECOVERY);
  EXPECT_EQ(harness.fsm.last_error_code, ErrorCodes::PLACEMENT_FAILED);
}

// ---------------------------------------------------------------------------
// 终态与幂等（证明"确实在跑"，避免把"没动作"误判为"已锁定"）
// ---------------------------------------------------------------------------

TEST(TaskFsmSafety, RepeatedStepsInFaultProduceNoActions)
{
  Harness harness;
  harness.boot();
  harness.reach_execute_item();
  harness.fsm.report_item_result(false, false, ErrorCodes::MOTION_STATUS_UNKNOWN, "lost");
  ASSERT_EQ(harness.fsm.state(), TaskStateName::FAULT);

  const size_t log_size_after_fault = harness.fsm.transition_log.size();
  for (int i = 0; i < 20; ++i) {
    const StepResult result = harness.fsm.step(mtc::task::Event::ITEM_FAILED, EventPayload{});
    EXPECT_EQ(result.state, TaskStateName::FAULT);
    EXPECT_TRUE(result.actions.empty());
  }
  // 无转移发生 -> 转移日志不增长，也不会产生任何动作
  EXPECT_EQ(harness.fsm.transition_log.size(), log_size_after_fault);
}

TEST(TaskFsmSafety, GoalAfterTerminalRequiresReset)
{
  Harness harness;
  harness.boot();
  harness.reach_scan_scene();
  harness.fsm.report_target("battery_1", 1);
  harness.fsm.report_item_result(true, true, 0, "ok");
  harness.fsm.record_completed_item();
  ASSERT_EQ(harness.fsm.state(), TaskStateName::FINISHED);

  // FINISHED 不是活动状态，但也没有 goal_accepted 转移：必须被忽略而非静默受理
  const StepResult ignored = harness.fsm.submit_goal(make_basic_task(2, "task-next"));
  EXPECT_EQ(ignored.state, TaskStateName::FINISHED);
  EXPECT_FALSE(ignored.goal_rejected);
  EXPECT_NE(ignored.reason.find("no transition for"), std::string::npos);
  EXPECT_EQ(harness.fsm.current_task->task_id, "task-1");
}

TEST(TaskFaultAudit, ActionFinishesButOrdinaryResetAndNewTasksStayLocked)
{
  Harness h;
  h.boot(); h.reach_execute_item();
  h.fsm.report_item_unsafe(ErrorCodes::MOTION_STATUS_UNKNOWN, "lost");
  EXPECT_TRUE(h.fsm.action_finished());
  EXPECT_TRUE(h.fsm.fault_locked());
  EXPECT_FALSE(h.fsm.accepts_tasks());
  const auto previous_log = h.fsm.transition_log.size();
  EXPECT_EQ(h.fsm.reset().state, TaskStateName::FAULT);
  EXPECT_TRUE(h.fsm.submit_goal(make_basic_task(2, "next")).goal_rejected);
  h.fsm.confirm_child_stopped();
  h.fsm.report_item_unsafe(ErrorCodes::OK, "late old result");
  EXPECT_EQ(h.fsm.last_error_code, ErrorCodes::MOTION_STATUS_UNKNOWN);
  EXPECT_EQ(h.fsm.transition_log.size(), previous_log);
}
