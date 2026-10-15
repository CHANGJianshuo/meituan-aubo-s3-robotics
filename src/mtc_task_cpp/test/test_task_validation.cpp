// Copyright 2026 meituan_challenge_ws contributors
//
// Task 请求校验测试（对齐 Python src/mtc_task/test/test_task_fsm.py 的校验用例）。

#include <gtest/gtest.h>

#include <string>
#include <limits>

#include "mtc_core_cpp/error_codes.hpp"
#include "mtc_task_cpp/task_validation.hpp"

namespace
{

using mtc::task::TaskRequest;
using mtc::task::ValidationResult;

TaskRequest basic_request()
{
  TaskRequest task;
  task.task_id = "task-1";
  task.mode = mtc::task::TaskMode::MODE_BASIC;
  task.ordered_colors = {1};
  task.timeout_ms = 60000;
  return task;
}

TaskRequest sequence_request()
{
  TaskRequest task;
  task.task_id = "task-seq";
  task.mode = mtc::task::TaskMode::MODE_SEQUENCE;
  task.ordered_colors = {1, 2, 3};
  task.timeout_ms = 180000;
  return task;
}

}  // namespace

TEST(TaskValidation, BasicModeAccepted)
{
  const ValidationResult result = mtc::task::validate_task_request(basic_request());
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.error_code, mtc::core::ErrorCodes::OK);
  EXPECT_EQ(result.reason, "ok");
  ASSERT_EQ(result.normalized_slots.size(), 1u);
  EXPECT_EQ(result.normalized_slots[0], "T0");
}

TEST(TaskValidation, SequenceModeMapsSlotsInOrder)
{
  const ValidationResult result = mtc::task::validate_task_request(sequence_request());
  EXPECT_TRUE(result.ok);
  ASSERT_EQ(result.normalized_slots.size(), 3u);
  EXPECT_EQ(result.normalized_slots[0], "P1");
  EXPECT_EQ(result.normalized_slots[1], "P2");
  EXPECT_EQ(result.normalized_slots[2], "P3");
}

TEST(TaskValidation, EmptyTaskIdRejected)
{
  TaskRequest task = basic_request();
  task.task_id = "   ";
  const ValidationResult result = mtc::task::validate_task_request(task);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error_code, mtc::core::ErrorCodes::INVALID_TASK);
  EXPECT_EQ(result.reason, "empty_task_id");
}

TEST(TaskValidation, UnknownModeRejected)
{
  TaskRequest task = basic_request();
  task.mode = 7;
  const ValidationResult result = mtc::task::validate_task_request(task);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "unknown_mode");
}

TEST(TaskValidation, EmptyColorsRejected)
{
  TaskRequest task = basic_request();
  task.ordered_colors.clear();
  const ValidationResult result = mtc::task::validate_task_request(task);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "empty_colors");
}

TEST(TaskValidation, BasicModeRequiresExactlyOneColor)
{
  TaskRequest task = basic_request();
  task.ordered_colors = {1, 2};
  const ValidationResult result = mtc::task::validate_task_request(task);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "basic_count");
}

TEST(TaskValidation, SequenceModeRequiresExactlyThreeColors)
{
  TaskRequest task = sequence_request();
  task.ordered_colors = {1, 2};
  const ValidationResult result = mtc::task::validate_task_request(task);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "sequence_count");
}

TEST(TaskValidation, ColorZeroUnknownIsIllegal)
{
  // COLOR_UNKNOWN=0 不是合法口令
  TaskRequest task = basic_request();
  task.ordered_colors = {0};
  const ValidationResult result = mtc::task::validate_task_request(task);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "illegal_color");
}

TEST(TaskValidation, ColorOutOfRangeIsIllegal)
{
  TaskRequest task = basic_request();
  task.ordered_colors = {5};
  const ValidationResult result = mtc::task::validate_task_request(task);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "illegal_color");
}

TEST(TaskValidation, DuplicateColorsRejected)
{
  TaskRequest task = sequence_request();
  task.ordered_colors = {1, 1, 2};
  const ValidationResult result = mtc::task::validate_task_request(task);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "duplicate_color");
}

TEST(TaskValidation, ZeroTimeoutRejected)
{
  TaskRequest task = basic_request();
  task.timeout_ms = 0;
  const ValidationResult result = mtc::task::validate_task_request(task);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "timeout_not_positive");
}

TEST(TaskValidation, NegativeStabilityRejected)
{
  TaskRequest task = basic_request();
  task.placement_stability_sec = -1.0;
  const ValidationResult result = mtc::task::validate_task_request(task);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "stability_not_positive");
}

TEST(TaskValidation, ExplicitSlotMustMatchDerivedSlot)
{
  TaskRequest task = basic_request();
  task.target_slot = std::string("P1");   // MODE_BASIC 推导为 T0
  const ValidationResult result = mtc::task::validate_task_request(task);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "target_slot_mismatch");
}

TEST(TaskValidation, ExplicitSlotMustBeAllowed)
{
  TaskRequest task = basic_request();
  task.target_slot = std::string("XX");
  const ValidationResult result = mtc::task::validate_task_request(task);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "illegal_target_slot");
}

TEST(TaskValidation, ExplicitSlotMatchingDerivedIsAccepted)
{
  TaskRequest task = basic_request();
  task.target_slot = std::string("T0");
  const ValidationResult result = mtc::task::validate_task_request(task);
  EXPECT_TRUE(result.ok);
}

TEST(TaskValidation, SlotNameValidation)
{
  EXPECT_TRUE(mtc::task::validate_slot_name("T0").ok);
  EXPECT_TRUE(mtc::task::validate_slot_name("P1").ok);
  EXPECT_TRUE(mtc::task::validate_slot_name("P3").ok);
  EXPECT_FALSE(mtc::task::validate_slot_name("P4").ok);
  EXPECT_FALSE(mtc::task::validate_slot_name("").ok);
}

TEST(TaskValidation, ExpectedSlotForIndex)
{
  EXPECT_EQ(
    mtc::task::expected_slot_for_index(mtc::task::TaskMode::MODE_BASIC, 0).value_or(""), "T0");
  // MODE_BASIC 下任意 index 都返回 T0（与 Python 一致）
  EXPECT_EQ(
    mtc::task::expected_slot_for_index(mtc::task::TaskMode::MODE_BASIC, 3).value_or(""), "T0");
  EXPECT_EQ(
    mtc::task::expected_slot_for_index(mtc::task::TaskMode::MODE_SEQUENCE, 0).value_or(""), "P1");
  EXPECT_EQ(
    mtc::task::expected_slot_for_index(mtc::task::TaskMode::MODE_SEQUENCE, 2).value_or(""), "P3");
  // 越界与未知模式返回 nullopt
  EXPECT_FALSE(
    mtc::task::expected_slot_for_index(mtc::task::TaskMode::MODE_SEQUENCE, 3).has_value());
  EXPECT_FALSE(mtc::task::expected_slot_for_index(9, 0).has_value());
}

TEST(TaskValidation, TaskRequestHelpers)
{
  TaskRequest task = sequence_request();
  EXPECT_EQ(task.mode_name(), "MODE_SEQUENCE");
  EXPECT_EQ(task.total_count(), 3u);
  const auto slots = task.slots();
  ASSERT_EQ(slots.size(), 3u);
  EXPECT_EQ(slots[0], "P1");
  EXPECT_EQ(slots[2], "P3");

  task.mode = mtc::task::TaskMode::MODE_BASIC;
  task.ordered_colors = {2};
  EXPECT_EQ(task.mode_name(), "MODE_BASIC");
  EXPECT_EQ(task.total_count(), 1u);
  EXPECT_EQ(task.slot_for_index(0).value_or(""), "T0");
}

TEST(TaskValidation, ModeNameOfUnknownMode)
{
  EXPECT_EQ(mtc::task::TaskMode::name_of(42), "UNKNOWN_MODE_42");
}

TEST(TaskValidationAudit, NonFiniteStabilityRejected)
{
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()}) {
    auto task = basic_request();
    task.placement_stability_sec = bad;
    EXPECT_FALSE(mtc::task::validate_task_request(task).ok);
  }
}

TEST(StabilityGate, ZeroStabilityRejected)
{
  auto task = basic_request(); task.placement_stability_sec = 0.0;
  EXPECT_FALSE(mtc::task::validate_task_request(task).ok);
}
TEST(StabilityGate, NegativeStabilityRejected)
{
  auto task = basic_request(); task.placement_stability_sec = -0.1;
  EXPECT_FALSE(mtc::task::validate_task_request(task).ok);
}
TEST(StabilityGate, NaNStabilityRejected)
{
  auto task = basic_request(); task.placement_stability_sec = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(mtc::task::validate_task_request(task).ok);
}
TEST(StabilityGate, InfiniteStabilityRejected)
{
  for (double value : {std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
    auto task = basic_request(); task.placement_stability_sec = value;
    EXPECT_FALSE(mtc::task::validate_task_request(task).ok);
  }
}
TEST(StabilityGate, PositiveStabilityAccepted)
{
  for (double value : {0.1, 1.5, 3.0, 5.0}) {
    auto task = basic_request(); task.placement_stability_sec = value;
    EXPECT_TRUE(mtc::task::validate_task_request(task).ok);
  }
}
TEST(StabilityGate, DefaultStabilityRemainsThreeSeconds)
{
  EXPECT_DOUBLE_EQ(basic_request().placement_stability_sec, 3.0);
  EXPECT_TRUE(mtc::task::validate_task_request(basic_request()).ok);
}
