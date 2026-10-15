// Copyright 2026 meituan_challenge_ws contributors
//
// PickPlace FSM 表结构、状态常量与启动校验测试。

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "fakes.hpp"
#include "mtc_core_cpp/error_codes.hpp"
#include "mtc_manipulation_cpp/pick_place_fsm.hpp"
#include "mtc_manipulation_cpp/tool_strategy.hpp"

namespace
{

using mtc::core::ManualClock;
using mtc::manipulation::PickPlaceConfig;
using mtc::manipulation::PickPlaceFsm;
using mtc::manipulation::PickPlaceState;
using mtc::manipulation::testing::FakeMotion;
using mtc::manipulation::testing::FakePerception;
using mtc::manipulation::testing::FakePlanner;
using mtc::manipulation::testing::FakeTool;

struct Fixture
{
  std::shared_ptr<ManualClock> clock = std::make_shared<ManualClock>(0.0);
  std::shared_ptr<FakePerception> perception = std::make_shared<FakePerception>();
  std::shared_ptr<FakePlanner> planner = std::make_shared<FakePlanner>();
  std::shared_ptr<FakeMotion> motion = std::make_shared<FakeMotion>();
  std::shared_ptr<FakeTool> tool = std::make_shared<FakeTool>();
  mtc::manipulation::ToolStrategyPtr strategy =
    mtc::manipulation::make_strategy(mtc::manipulation::TOOL_PASSIVE_HOOK_V1);

  PickPlaceConfig config()
  {
    perception->clock = clock;
    PickPlaceConfig value;
    value.tool_type = mtc::manipulation::TOOL_PASSIVE_HOOK_V1;
    value.placement_stability_sec = 3.0;
    value.state_timeouts = mtc::manipulation::default_state_timeouts();
    return value;
  }

  std::unique_ptr<PickPlaceFsm> make_fsm(
    PickPlaceConfig cfg, mtc::manipulation::ToolStrategyPtr tool_strategy = nullptr)
  {
    return std::make_unique<PickPlaceFsm>(
      perception, planner, motion, tool, clock,
      tool_strategy ? tool_strategy : strategy, cfg);
  }
};

}  // namespace

// ---------------------------------------------------------------------------
// 表结构与状态常量
// ---------------------------------------------------------------------------

TEST(PickPlaceTables, VerifyTablesReportsNoProblem)
{
  const auto problems = PickPlaceFsm::verify_tables();
  for (const auto & problem : problems) {
    ADD_FAILURE() << problem;
  }
  EXPECT_TRUE(problems.empty());
}

TEST(PickPlaceTables, EveryStateHasHandler)
{
  EXPECT_EQ(
    PickPlaceFsm::state_actions().size(),
    mtc::manipulation::all_pick_place_states().size());
}

TEST(PickPlaceTables, StateNamesMatchPythonValues)
{
  // feedback.substate 直接把这些字符串对外暴露，必须与 Python 逐字一致。
  EXPECT_STREQ(mtc::manipulation::to_string(PickPlaceState::RESOLVE_TARGET), "RESOLVE_TARGET");
  EXPECT_STREQ(mtc::manipulation::to_string(PickPlaceState::VERIFY_ENGAGEMENT), "VERIFY_ENGAGEMENT");
  EXPECT_STREQ(mtc::manipulation::to_string(PickPlaceState::VERIFY_ATTACHED), "VERIFY_ATTACHED");
  EXPECT_STREQ(mtc::manipulation::to_string(PickPlaceState::VERIFY_SEATED), "VERIFY_SEATED");
  EXPECT_STREQ(mtc::manipulation::to_string(PickPlaceState::VERIFY_PLACED), "VERIFY_PLACED");
  EXPECT_STREQ(mtc::manipulation::to_string(PickPlaceState::CANCEL_PENDING), "CANCEL_PENDING");
  EXPECT_STREQ(mtc::manipulation::to_string(PickPlaceState::CANCELLED), "CANCELLED");
  EXPECT_STREQ(mtc::manipulation::to_string(PickPlaceState::FAULT), "FAULT");
}

TEST(PickPlaceTables, PipelineMatchesDesignOrder)
{
  // 正常流水线顺序即设计 §3.2 的顺序，不得随意调整。
  const std::vector<PickPlaceState> expected = {
    PickPlaceState::RESOLVE_TARGET,
    PickPlaceState::PLAN_ACQUIRE,
    PickPlaceState::MOVE_PRE_ALIGN,
    PickPlaceState::ENGAGE,
    PickPlaceState::VERIFY_ENGAGEMENT,
    PickPlaceState::TEST_LIFT,
    PickPlaceState::VERIFY_ATTACHED,
    PickPlaceState::LIFT,
    PickPlaceState::TRANSPORT,
    PickPlaceState::MOVE_PRE_SEAT,
    PickPlaceState::SEAT,
    PickPlaceState::VERIFY_SEATED,
    PickPlaceState::UNLOAD,
    PickPlaceState::DISENGAGE,
    PickPlaceState::RETREAT,
    PickPlaceState::VERIFY_PLACED,
    PickPlaceState::SUCCESS,
  };
  EXPECT_EQ(mtc::manipulation::pipeline_states(), expected);
}

TEST(PickPlaceTables, TerminalAndMotionInFlightSets)
{
  EXPECT_EQ(mtc::manipulation::terminal_states().size(), 4u);
  EXPECT_TRUE(mtc::manipulation::terminal_states().count(PickPlaceState::SUCCESS) == 1u);
  EXPECT_TRUE(mtc::manipulation::terminal_states().count(PickPlaceState::FAILED) == 1u);
  EXPECT_TRUE(mtc::manipulation::terminal_states().count(PickPlaceState::FAULT) == 1u);
  EXPECT_TRUE(mtc::manipulation::terminal_states().count(PickPlaceState::CANCELLED) == 1u);
  // CANCEL_PENDING 不是终态：取消必须等停稳确认
  EXPECT_TRUE(mtc::manipulation::terminal_states().count(PickPlaceState::CANCEL_PENDING) == 0u);

  // 运动在途状态集合必须覆盖所有"指令已发出但可能仍在动"的状态
  for (const auto state : {
      PickPlaceState::MOVE_PRE_ALIGN, PickPlaceState::ENGAGE, PickPlaceState::TEST_LIFT,
      PickPlaceState::LIFT, PickPlaceState::TRANSPORT, PickPlaceState::MOVE_PRE_SEAT,
      PickPlaceState::SEAT, PickPlaceState::UNLOAD, PickPlaceState::DISENGAGE,
      PickPlaceState::RETREAT})
  {
    EXPECT_TRUE(mtc::manipulation::motion_in_flight_states().count(state) == 1u)
      << mtc::manipulation::to_string(state);
  }
  EXPECT_FALSE(mtc::manipulation::motion_in_flight_states().count(PickPlaceState::VERIFY_SEATED));
}

TEST(PickPlaceTables, DefaultTimeoutsCoverMotionStates)
{
  const auto timeouts = mtc::manipulation::default_state_timeouts();
  EXPECT_DOUBLE_EQ(timeouts.at(PickPlaceState::RESOLVE_TARGET), 2.0);
  EXPECT_DOUBLE_EQ(timeouts.at(PickPlaceState::MOVE_PRE_ALIGN), 30.0);
  EXPECT_DOUBLE_EQ(timeouts.at(PickPlaceState::TRANSPORT), 60.0);
  EXPECT_DOUBLE_EQ(timeouts.at(PickPlaceState::VERIFY_PLACED), 30.0);
}

// ---------------------------------------------------------------------------
// 启动校验
// ---------------------------------------------------------------------------

TEST(PickPlaceStart, IllegalTargetSlotFailsImmediatelyWithInvalidTask)
{
  Fixture fixture;
  auto fsm = fixture.make_fsm(fixture.config());

  fsm->start("battery_1", 1, "P9");

  EXPECT_EQ(fsm->state(), PickPlaceState::FAILED);
  EXPECT_EQ(fsm->error_code(), mtc::core::ErrorCodes::INVALID_TASK);
  EXPECT_NE(fsm->message().find("非法目标工位"), std::string::npos);
  EXPECT_FALSE(fsm->result().success);
  // 未产生任何运动或感知调用
  EXPECT_EQ(fixture.motion->execute_calls, 0);
  EXPECT_EQ(fixture.perception->calls, 0);
}

TEST(PickPlaceStart, AllowedSlotsAreAccepted)
{
  for (const auto & slot : mtc::manipulation::allowed_target_slots()) {
    Fixture fixture;
    auto fsm = fixture.make_fsm(fixture.config());
    fsm->start("battery_1", 1, slot);
    EXPECT_EQ(fsm->state(), PickPlaceState::RESOLVE_TARGET) << slot;
    EXPECT_EQ(fsm->error_code(), mtc::core::ErrorCodes::OK);
  }
  // 与 PickPlace.action 的约束一致
  EXPECT_EQ(mtc::manipulation::allowed_target_slots().size(), 4u);
}

TEST(PickPlaceStart, ToolTypeMismatchThrows)
{
  Fixture fixture;
  PickPlaceConfig config = fixture.config();
  config.tool_type = mtc::manipulation::TOOL_MAGNETIC_LATCH_V2;

  // 配置声明 V2，但注入的是 V1 策略：必须拒绝构造，
  // 否则证据语义不可信。
  EXPECT_THROW(
    {
      PickPlaceFsm fsm(
        fixture.perception, fixture.planner, fixture.motion, fixture.tool,
        fixture.clock, fixture.strategy, config);
      (void)fsm;
    },
    std::invalid_argument);
}

TEST(PickPlaceStart, RequiredConfigFieldsAreRecorded)
{
  Fixture fixture;
  auto fsm = fixture.make_fsm(fixture.config());

  fsm->start("battery_7", 3, "P2", "req-42", 1.5);

  EXPECT_EQ(fsm->object_id(), "battery_7");
  EXPECT_EQ(fsm->expected_color(), 3);
  EXPECT_EQ(fsm->target_slot(), "P2");
  EXPECT_EQ(fsm->request_id(), "req-42");
  // Goal 显式传入优先于配置默认值
  EXPECT_DOUBLE_EQ(fsm->placement_stability_sec(), 1.5);
  EXPECT_TRUE(fsm->started());
  EXPECT_EQ(fsm->state(), PickPlaceState::RESOLVE_TARGET);
}

TEST(PickPlaceStart, ConfigStabilityUsedWhenGoalOmitsIt)
{
  Fixture fixture;
  PickPlaceConfig config = fixture.config();
  config.placement_stability_sec = 4.25;
  auto fsm = fixture.make_fsm(config);

  fsm->start("battery_1", 1, "T0");
  EXPECT_DOUBLE_EQ(fsm->placement_stability_sec(), 4.25);
}

TEST(PickPlaceStart, PlacementVerifiedStartsFalseAndIsReadOnly)
{
  Fixture fixture;
  auto fsm = fixture.make_fsm(fixture.config());
  fsm->start("battery_1", 1, "T0");

  // 安全语义 1：初始必须为 false，且没有任何公开写入口
  EXPECT_FALSE(fsm->placement_verified());
  EXPECT_FALSE(fsm->result().placement_verified);
}

TEST(PickPlaceStart, MotionResendAllowedStartsTrue)
{
  Fixture fixture;
  auto fsm = fixture.make_fsm(fixture.config());
  fsm->start("battery_1", 1, "T0");
  EXPECT_TRUE(fsm->motion_resend_allowed());
  EXPECT_FALSE(fsm->stop_confirmed());
  EXPECT_EQ(fsm->recovery_attempts(), 0);
}

// ---------------------------------------------------------------------------
// feedback / progress
// ---------------------------------------------------------------------------

TEST(PickPlaceFeedback, FeedbackCarriesSubstateAndProgress)
{
  Fixture fixture;
  auto fsm = fixture.make_fsm(fixture.config());
  fsm->start("battery_1", 1, "T0");

  auto feedback = fsm->feedback();
  EXPECT_EQ(feedback.substate, "RESOLVE_TARGET");
  EXPECT_FLOAT_EQ(feedback.progress, 0.0f);
  EXPECT_FALSE(feedback.object_attached_estimated);

  // 推进到 VERIFY_PLACED 之前，progress 必须单调不减
  fsm->step();   // RESOLVE_TARGET -> PLAN_ACQUIRE
  const float after_first = fsm->feedback().progress;
  EXPECT_GT(after_first, 0.0f);
  EXPECT_LE(after_first, 1.0f);
}

TEST(PickPlaceFeedback, ProgressReachesOneOnlyAtSuccess)
{
  Fixture fixture;
  auto fsm = fixture.make_fsm(fixture.config());
  fsm->start("battery_1", 1, "T0");

  // 走到 FAULT 时 progress 不得为 1
  fsm->force_fault(mtc::core::ErrorCodes::HARDWARE_FAULT, "test fault");
  const auto feedback = fsm->feedback();
  EXPECT_EQ(feedback.substate, "FAULT");
  EXPECT_LT(feedback.progress, 1.0f);
}

TEST(PickPlaceFeedback, StateHistoryRecordsEnteredStates)
{
  Fixture fixture;
  auto fsm = fixture.make_fsm(fixture.config());
  fsm->start("battery_1", 1, "T0");

  ASSERT_FALSE(fsm->states_history().empty());
  EXPECT_EQ(fsm->states_history().front(), PickPlaceState::RESOLVE_TARGET);

  fsm->step();
  EXPECT_EQ(fsm->states_history().back(), PickPlaceState::PLAN_ACQUIRE);
  EXPECT_EQ(fsm->states_history().size(), 2u);
}

TEST(PickPlaceFeedback, EventCallbackReceivesStateEntered)
{
  Fixture fixture;
  auto fsm = fixture.make_fsm(fixture.config());

  std::vector<std::string> events;
  fsm->on_event = [&events](const std::string & event, PickPlaceState state) {
      events.push_back(event + ":" + mtc::manipulation::to_string(state));
    };

  fsm->start("battery_1", 1, "T0");
  ASSERT_GE(events.size(), 2u);
  EXPECT_EQ(events[0], "state_entered:RESOLVE_TARGET");
  EXPECT_EQ(events[1], "goal_accepted:RESOLVE_TARGET");
}

TEST(PickPlaceFeedback, CallbackExceptionDoesNotBreakSequencing)
{
  Fixture fixture;
  auto fsm = fixture.make_fsm(fixture.config());
  fsm->on_event = [](const std::string &, PickPlaceState) {
      throw std::runtime_error("callback failure");
    };

  // 回调异常不得传播，也不得破坏状态机时序
  EXPECT_NO_THROW(fsm->start("battery_1", 1, "T0"));
  EXPECT_NO_THROW(fsm->step());
  EXPECT_EQ(fsm->state(), PickPlaceState::PLAN_ACQUIRE);
}

// ---------------------------------------------------------------------------
// 终态幂等
// ---------------------------------------------------------------------------

TEST(PickPlaceTerminal, StepAtTerminalIsIdempotent)
{
  Fixture fixture;
  auto fsm = fixture.make_fsm(fixture.config());
  fsm->start("battery_1", 1, "T0");

  const auto fault = fsm->force_fault(mtc::core::ErrorCodes::SAFETY_INTERLOCK, "guard tripped");
  EXPECT_EQ(fault.state, PickPlaceState::FAULT);

  const auto history_size = fsm->states_history().size();
  const auto motion_calls = fixture.motion->execute_calls;
  for (int i = 0; i < 10; ++i) {
    const auto result = fsm->step();
    EXPECT_EQ(result.state, PickPlaceState::FAULT);
  }
  EXPECT_EQ(fsm->states_history().size(), history_size);
  EXPECT_EQ(fixture.motion->execute_calls, motion_calls);
}

TEST(PickPlaceTerminal, ForceFaultOnTerminalKeepsOriginalState)
{
  Fixture fixture;
  auto fsm = fixture.make_fsm(fixture.config());
  fsm->start("battery_1", 1, "P9");   // 立即 FAILED
  ASSERT_EQ(fsm->state(), PickPlaceState::FAILED);

  const auto result = fsm->force_fault(mtc::core::ErrorCodes::HARDWARE_FAULT, "late fault");
  EXPECT_EQ(result.state, PickPlaceState::FAILED);
  // 终态结论不得被迟到的 fault 请求改写
  EXPECT_EQ(fsm->error_code(), mtc::core::ErrorCodes::INVALID_TASK);
  EXPECT_NE(fsm->message().find("非法目标工位"), std::string::npos);
}
