// Copyright 2026 meituan_challenge_ws contributors
//
// Task FSM 正常流程、转移表完整性与终态语义测试。
//
// 对齐 Python src/mtc_task/test/test_task_fsm.py 中的正常路径用例。

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "mtc_core_cpp/clock.hpp"
#include "mtc_core_cpp/error_codes.hpp"
#include "mtc_task_cpp/task_fsm.hpp"

namespace
{

using mtc::core::ManualClock;
using mtc::task::ActionKind;
using mtc::task::EventPayload;
using mtc::task::FsmsConfig;
using mtc::task::StepResult;
using mtc::task::TaskFsm;
using mtc::task::TaskRequest;
using mtc::task::TaskStateName;

struct Harness
{
  std::shared_ptr<ManualClock> clock = std::make_shared<ManualClock>(1000.0);
  TaskFsm fsm{clock, FsmsConfig::defaults()};

  /// 把状态机推进到 WAIT_TASK。
  void boot()
  {
    fsm.start();
    fsm.report_self_check(true);
  }
};

TaskRequest make_basic_task(const std::string & task_id = "task-1", int color = 1)
{
  TaskRequest task;
  task.task_id = task_id;
  task.mode = mtc::task::TaskMode::MODE_BASIC;
  task.ordered_colors = {color};
  task.timeout_ms = 60000;
  return task;
}

TaskRequest make_sequence_task()
{
  TaskRequest task;
  task.task_id = "task-seq";
  task.mode = mtc::task::TaskMode::MODE_SEQUENCE;
  task.ordered_colors = {1, 2, 3};
  task.timeout_ms = 180000;
  return task;
}

}  // namespace

// ---------------------------------------------------------------------------
// 转移表完整性（等价于 Python 导入期 _self_check_tables）
// ---------------------------------------------------------------------------

TEST(TaskFsmTables, VerifyTablesReportsNoProblem)
{
  const auto problems = TaskFsm::verify_tables();
  for (const auto & problem : problems) {
    ADD_FAILURE() << problem;
  }
  EXPECT_TRUE(problems.empty());
}

TEST(TaskFsmTables, AllStatesHaveEntryActionAndTimeout)
{
  EXPECT_EQ(TaskFsm::entry_actions().size(), mtc::task::all_states().size());
  for (const auto state : mtc::task::all_states()) {
    EXPECT_TRUE(mtc::task::default_state_timeouts().count(state) == 1u)
      << "missing timeout for " << mtc::task::to_string(state);
  }
}

TEST(TaskFsmTables, EveryActiveStateRegistersConcurrentGoalRejection)
{
  // 每个"进行中"状态都必须显式登记 goal_accepted 的拒绝转移，
  // 否则 step() 会把它当作"无转移事件"静默忽略，等于没有拒绝。
  for (const auto state : mtc::task::active_task_states()) {
    bool found = false;
    for (const auto & transition : TaskFsm::transitions()) {
      if (transition.state == state && transition.event == std::string(mtc::task::Event::GOAL_ACCEPTED)) {
        found = true;
        EXPECT_EQ(transition.next_state, state) << "rejection must not change state";
        EXPECT_TRUE(transition.guard != nullptr) << "rejection must carry a guard";
        break;
      }
    }
    EXPECT_TRUE(found) << "state " << mtc::task::to_string(state)
                       << " lacks explicit goal_accepted rejection transition";
  }
}

// ---------------------------------------------------------------------------
// 启动与自检
// ---------------------------------------------------------------------------

TEST(TaskFsmBoot, InitToSelfCheckToWaitTask)
{
  Harness harness;
  EXPECT_EQ(harness.fsm.state(), TaskStateName::INIT);

  const StepResult started = harness.fsm.start();
  EXPECT_EQ(started.state, TaskStateName::SELF_CHECK);
  EXPECT_TRUE(started.has(ActionKind::PUBLISH_STATE));

  const StepResult ready = harness.fsm.report_self_check(true);
  EXPECT_EQ(ready.state, TaskStateName::WAIT_TASK);
  EXPECT_EQ(harness.fsm.substate, "WAIT_TASK");
  EXPECT_FALSE(harness.fsm.has_active_task());
}

TEST(TaskFsmBoot, SelfCheckFailureGoesToFaultWithStopAndLock)
{
  Harness harness;
  harness.fsm.start();
  const StepResult result = harness.fsm.report_self_check(false);

  EXPECT_EQ(result.state, TaskStateName::FAULT);
  // §3.1：FAULT 需停止请求并锁定新的动作
  EXPECT_TRUE(result.has(ActionKind::REQUEST_STOP));
  EXPECT_TRUE(result.has(ActionKind::LOCK_ACTIONS));
  EXPECT_EQ(harness.fsm.last_error_code, mtc::core::ErrorCodes::HARDWARE_FAULT);
}

TEST(TaskFsmBoot, InitTimeoutGoesToFault)
{
  Harness harness;
  // INIT 默认超时 5.0s（从不调用 start()）
  harness.clock->advance(5.0);
  const auto result = harness.fsm.check_timeouts();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->state, TaskStateName::FAULT);
  EXPECT_EQ(harness.fsm.last_error_code, mtc::core::ErrorCodes::HARDWARE_FAULT);
}

TEST(TaskFsmBoot, NoTimeoutBeforeDeadline)
{
  Harness harness;
  harness.clock->advance(4.9);
  EXPECT_FALSE(harness.fsm.check_timeouts().has_value());
}

// ---------------------------------------------------------------------------
// 正常单项流程（MODE_BASIC）
// ---------------------------------------------------------------------------

TEST(TaskFsmBasic, FullHappyPathReachesFinished)
{
  Harness harness;
  harness.boot();

  const StepResult accepted = harness.fsm.submit_goal(make_basic_task());
  EXPECT_EQ(accepted.state, TaskStateName::VALIDATE_TASK);
  EXPECT_TRUE(harness.fsm.has_active_task());
  ASSERT_TRUE(harness.fsm.current_task.has_value());
  EXPECT_EQ(harness.fsm.current_task->task_id, "task-1");

  EXPECT_EQ(harness.fsm.advance_validation().state, TaskStateName::SCAN_SCENE);

  const StepResult target = harness.fsm.report_target("battery_1", 1);
  EXPECT_EQ(target.state, TaskStateName::EXECUTE_ITEM);

  // 必须产出 PickPlace 调用意图，且携带 6D 目标所需的标识与颜色双重校验字段
  const auto * pick = target.find(ActionKind::CALL_PICK_PLACE);
  ASSERT_NE(pick, nullptr);
  EXPECT_EQ(pick->pick_place.object_id, "battery_1");
  EXPECT_EQ(pick->pick_place.expected_color, 1);
  EXPECT_EQ(pick->pick_place.target_slot, "T0");
  EXPECT_EQ(pick->pick_place.request_id, "task-1#0");

  // PUBLISH_STATE 永远最后追加
  ASSERT_FALSE(target.actions.empty());
  EXPECT_EQ(target.actions.back().kind, ActionKind::PUBLISH_STATE);

  const StepResult item = harness.fsm.report_item_result(true, true, 0, "placed");
  EXPECT_EQ(item.state, TaskStateName::RECORD_RESULT);

  const StepResult recorded = harness.fsm.record_completed_item();
  EXPECT_EQ(recorded.state, TaskStateName::FINISHED);

  const auto outcome = harness.fsm.build_outcome();
  EXPECT_TRUE(outcome.success);
  EXPECT_EQ(outcome.error_code, mtc::core::ErrorCodes::OK);
  EXPECT_EQ(outcome.completed_count, 1u);
  ASSERT_EQ(outcome.completed_slots.size(), 1u);
  EXPECT_EQ(outcome.completed_slots[0], "T0");
  EXPECT_EQ(outcome.message, "task finished: T0");
}

TEST(TaskFsmBasic, SuccessRequiresPlacementVerified)
{
  Harness harness;
  harness.boot();
  harness.fsm.submit_goal(make_basic_task());
  harness.fsm.advance_validation();
  harness.fsm.report_target("battery_1", 1);

  // PickPlace 声称 success 但未验证放置：必须记为 ITEM_FAILED，不得记完成。
  const StepResult result = harness.fsm.report_item_result(true, false, 0, "claimed ok");
  EXPECT_EQ(result.state, TaskStateName::RECOVERY);
  EXPECT_EQ(harness.fsm.last_error_code, mtc::core::ErrorCodes::PLACEMENT_FAILED);
  EXPECT_EQ(harness.fsm.last_error_message, "PickPlace reported success without placement_verified");
}

TEST(TaskFsmBasic, TargetFoundWithoutObjectIdIsRejected)
{
  Harness harness;
  harness.boot();
  harness.fsm.submit_goal(make_basic_task());
  harness.fsm.advance_validation();

  const StepResult result = harness.fsm.report_target("", std::nullopt);
  // 守卫拒绝：停留在 SCAN_SCENE，不得进入 EXECUTE_ITEM
  EXPECT_EQ(result.state, TaskStateName::SCAN_SCENE);
  EXPECT_EQ(harness.fsm.last_error_code, mtc::core::ErrorCodes::TARGET_NOT_FOUND);
}

TEST(TaskFsmBasic, ExpectedColorMismatchIsRejected)
{
  Harness harness;
  harness.boot();
  harness.fsm.submit_goal(make_basic_task("task-1", 1));
  harness.fsm.advance_validation();

  // 期望 color=1，感知回报 color=2 -> 拒绝，防止抓错
  const StepResult result = harness.fsm.report_target("battery_9", 2);
  EXPECT_EQ(result.state, TaskStateName::SCAN_SCENE);
  EXPECT_EQ(harness.fsm.last_error_code, mtc::core::ErrorCodes::INVALID_TASK);
}

TEST(TaskFsmBasic, ItemResultIgnoredOutsideExecuteItem)
{
  Harness harness;
  harness.boot();
  const StepResult result = harness.fsm.report_item_result(true, true, 0, "ok");
  EXPECT_EQ(result.state, TaskStateName::WAIT_TASK);
  EXPECT_EQ(result.reason, "item result ignored: state is WAIT_TASK, not EXECUTE_ITEM");
}

// ---------------------------------------------------------------------------
// 正常三项流程（MODE_SEQUENCE）
// ---------------------------------------------------------------------------

TEST(TaskFsmSequence, ThreeItemsCompleteInSlotOrder)
{
  Harness harness;
  harness.boot();

  harness.fsm.submit_goal(make_sequence_task());
  harness.fsm.advance_validation();

  const std::vector<std::string> expected_slots = {"P1", "P2", "P3"};
  const std::vector<int> expected_colors = {1, 2, 3};

  for (size_t index = 0; index < expected_slots.size(); ++index) {
    const StepResult target = harness.fsm.report_target(
      "battery_" + std::to_string(index), expected_colors[index]);
    ASSERT_EQ(target.state, TaskStateName::EXECUTE_ITEM);
    const auto * pick = target.find(ActionKind::CALL_PICK_PLACE);
    ASSERT_NE(pick, nullptr);
    EXPECT_EQ(pick->pick_place.target_slot, expected_slots[index]);
    EXPECT_EQ(pick->pick_place.expected_color, expected_colors[index]);
    EXPECT_EQ(pick->pick_place.request_id, "task-seq#" + std::to_string(index));

    EXPECT_EQ(
      harness.fsm.report_item_result(true, true, 0, "ok").state, TaskStateName::RECORD_RESULT);

    const StepResult recorded = harness.fsm.record_completed_item();
    if (index + 1u < expected_slots.size()) {
      // 还有剩余项 -> 回到 SCAN_SCENE
      EXPECT_EQ(recorded.state, TaskStateName::SCAN_SCENE);
    } else {
      // 全部完成 -> FINISHED
      EXPECT_EQ(recorded.state, TaskStateName::FINISHED);
    }
  }

  const auto outcome = harness.fsm.build_outcome();
  EXPECT_TRUE(outcome.success);
  EXPECT_EQ(outcome.completed_count, 3u);
  EXPECT_EQ(outcome.completed_slots, expected_slots);
}

TEST(TaskFsmSequence, InvalidSequenceFailsWithoutGuessing)
{
  Harness harness;
  harness.boot();

  TaskRequest task = make_sequence_task();
  task.ordered_colors = {1, 2};   // 必须恰好 3 项
  const StepResult accepted = harness.fsm.submit_goal(task);
  EXPECT_EQ(accepted.state, TaskStateName::VALIDATE_TASK);

  const StepResult validated = harness.fsm.advance_validation();
  EXPECT_EQ(validated.state, TaskStateName::TASK_FAILED);
  EXPECT_EQ(harness.fsm.last_error_code, mtc::core::ErrorCodes::INVALID_TASK);
  EXPECT_EQ(harness.fsm.last_validation_error, "sequence_count");
  EXPECT_FALSE(harness.fsm.build_outcome().success);
}

TEST(TaskFsmSequence, DuplicateSlotIsNotRecordedTwice)
{
  Harness harness;
  harness.boot();
  harness.fsm.submit_goal(make_basic_task());
  harness.fsm.advance_validation();
  harness.fsm.report_target("battery_1", 1);
  harness.fsm.report_item_result(true, true, 0, "ok");

  harness.fsm.record_completed_item("T0");
  EXPECT_EQ(harness.fsm.completed_count, 1u);

  // 重复记录同一工位不得重复计数
  harness.fsm.record_completed_item("T0");
  EXPECT_EQ(harness.fsm.completed_count, 1u);
}

// ---------------------------------------------------------------------------
// 终态复位
// ---------------------------------------------------------------------------

TEST(TaskFsmReset, ResetFromFinishedClearsTaskContext)
{
  Harness harness;
  harness.boot();
  harness.fsm.submit_goal(make_basic_task());
  harness.fsm.advance_validation();
  harness.fsm.report_target("battery_1", 1);
  harness.fsm.report_item_result(true, true, 0, "ok");
  harness.fsm.record_completed_item();
  ASSERT_EQ(harness.fsm.state(), TaskStateName::FINISHED);

  const StepResult reset = harness.fsm.reset();
  EXPECT_EQ(reset.state, TaskStateName::WAIT_TASK);
  EXPECT_FALSE(harness.fsm.current_task.has_value());
  EXPECT_EQ(harness.fsm.completed_count, 0u);
  EXPECT_TRUE(harness.fsm.completed_slots.empty());
  EXPECT_EQ(harness.fsm.last_error_code, mtc::core::ErrorCodes::OK);
  EXPECT_FALSE(harness.fsm.last_outcome.has_value());
  EXPECT_FALSE(harness.fsm.has_active_task());
}

TEST(TaskFsmReset, ResetAllowedFromTaskFailedAndCancelled)
{
  {
    Harness harness;
    harness.boot();
    TaskRequest task = make_basic_task();
    task.ordered_colors = {0};
    harness.fsm.submit_goal(task);
    ASSERT_EQ(harness.fsm.advance_validation().state, TaskStateName::TASK_FAILED);
    EXPECT_EQ(harness.fsm.reset().state, TaskStateName::WAIT_TASK);
  }
  {
    Harness harness;
    harness.boot();
    harness.fsm.submit_goal(make_basic_task());
    harness.fsm.advance_validation();
    ASSERT_EQ(harness.fsm.request_cancel().state, TaskStateName::CANCEL_PENDING);
    ASSERT_EQ(harness.fsm.confirm_child_stopped().state, TaskStateName::CANCELLED);
    EXPECT_EQ(harness.fsm.reset().state, TaskStateName::WAIT_TASK);
  }
}

// ---------------------------------------------------------------------------
// 快照与动作产出
// ---------------------------------------------------------------------------

TEST(TaskFsmSnapshot, SnapshotCarriesTaskAndProgress)
{
  Harness harness;
  harness.boot();
  harness.fsm.submit_goal(make_sequence_task());
  harness.fsm.advance_validation();

  auto snapshot = harness.fsm.snapshot();
  EXPECT_EQ(snapshot.task_id, "task-seq");
  EXPECT_EQ(snapshot.state, "SCAN_SCENE");
  EXPECT_EQ(snapshot.total_count, 3u);
  EXPECT_EQ(snapshot.completed_count, 0u);

  harness.fsm.report_target("battery_0", 1);
  harness.fsm.report_item_result(true, true, 0, "ok");
  harness.fsm.record_completed_item();

  snapshot = harness.fsm.snapshot();
  EXPECT_EQ(snapshot.completed_count, 1u);
  EXPECT_EQ(snapshot.state, "SCAN_SCENE");
}

TEST(TaskFsmSnapshot, EveryStepPublishesState)
{
  Harness harness;
  const StepResult started = harness.fsm.start();
  ASSERT_FALSE(started.actions.empty());
  EXPECT_EQ(started.actions.back().kind, ActionKind::PUBLISH_STATE);
  EXPECT_EQ(started.actions.back().snapshot.state, "SELF_CHECK");

  const StepResult ready = harness.fsm.report_self_check(true);
  ASSERT_FALSE(ready.actions.empty());
  EXPECT_EQ(ready.actions.back().kind, ActionKind::PUBLISH_STATE);
  EXPECT_EQ(ready.actions.back().snapshot.state, "WAIT_TASK");
}

TEST(TaskFsmSnapshot, TransitionLogRecordsPath)
{
  Harness harness;
  harness.boot();
  harness.fsm.submit_goal(make_basic_task());

  ASSERT_GE(harness.fsm.transition_log.size(), 3u);
  EXPECT_EQ(harness.fsm.transition_log[0].from_state, "INIT");
  EXPECT_EQ(harness.fsm.transition_log[0].to_state, "SELF_CHECK");
  EXPECT_EQ(harness.fsm.transition_log[1].from_state, "SELF_CHECK");
  EXPECT_EQ(harness.fsm.transition_log[1].to_state, "WAIT_TASK");
  EXPECT_EQ(harness.fsm.transition_log[2].from_state, "WAIT_TASK");
  EXPECT_EQ(harness.fsm.transition_log[2].to_state, "VALIDATE_TASK");
}

TEST(TaskFsmSnapshot, UnknownEventIsIgnoredNotFatal)
{
  Harness harness;
  const StepResult result = harness.fsm.step("no_such_event", EventPayload{});
  EXPECT_EQ(result.state, TaskStateName::INIT);
  EXPECT_TRUE(result.actions.empty());
  EXPECT_EQ(result.reason, "no transition for (INIT, no_such_event)");
  ASSERT_EQ(harness.fsm.ignored_events.size(), 1u);
  EXPECT_EQ(harness.fsm.ignored_events[0].first, "INIT");
  EXPECT_EQ(harness.fsm.ignored_events[0].second, "no_such_event");
}
