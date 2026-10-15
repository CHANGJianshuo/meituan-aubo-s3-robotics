// Copyright 2026 meituan_challenge_ws contributors
//
// PickPlace FSM V1（被动舌规）正常闭环测试。
//
// 严格对齐 Python src/mtc_manipulation/test/test_pick_place_fsm.py
// 的 test_v1_happy_path_reaches_success。

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "fakes.hpp"
#include "mtc_core_cpp/error_codes.hpp"
#include "mtc_manipulation_cpp/pick_place_fsm.hpp"
#include "mtc_manipulation_cpp/tool_strategy.hpp"

namespace
{

using mtc::core::ErrorCodes;
using mtc::core::ManualClock;
using mtc::manipulation::PickPlaceConfig;
using mtc::manipulation::PickPlaceFsm;
using mtc::manipulation::PickPlaceState;
using mtc::manipulation::TOOL_PASSIVE_HOOK_V1;
using mtc::manipulation::testing::FakeMotion;
using mtc::manipulation::testing::FakePerception;
using mtc::manipulation::testing::FakePlanner;
using mtc::manipulation::testing::FakeTool;

/// 与 Python Harness 的 RUN_TICK_SECONDS 一致。
constexpr double kRunTickSeconds = 0.2;

struct Harness
{
  std::shared_ptr<ManualClock> clock = std::make_shared<ManualClock>(0.0);
  std::shared_ptr<FakePerception> perception = std::make_shared<FakePerception>();
  std::shared_ptr<FakePlanner> planner = std::make_shared<FakePlanner>();
  std::shared_ptr<FakeMotion> motion = std::make_shared<FakeMotion>();
  std::shared_ptr<FakeTool> tool = std::make_shared<FakeTool>();
  std::unique_ptr<PickPlaceFsm> fsm;

  Harness()
  {
    perception->clock = clock;
    PickPlaceConfig config;
    config.tool_type = TOOL_PASSIVE_HOOK_V1;
    config.placement_stability_sec = 3.0;
    config.state_timeouts = mtc::manipulation::default_state_timeouts();
    fsm = std::make_unique<PickPlaceFsm>(
      perception, planner, motion, tool, clock,
      mtc::manipulation::make_strategy(TOOL_PASSIVE_HOOK_V1), config);
  }

  void start(int color = 1, const std::string & slot = "P1")
  {
    fsm->start("battery_1", color, slot, "req-test");
  }

  /// 与 Python Harness.run 等价：逐步推进并在非终态时推进假时钟。
  mtc::manipulation::PickPlaceResult run(
    int max_ticks = 1200, double tick_seconds = kRunTickSeconds)
  {
    for (int i = 0; i < max_ticks; ++i) {
      fsm->step();
      const auto state = fsm->state();
      if (state == PickPlaceState::SUCCESS || state == PickPlaceState::FAILED ||
        state == PickPlaceState::FAULT || state == PickPlaceState::CANCELLED)
      {
        break;
      }
      clock->advance(tick_seconds);
    }
    return fsm->result();
  }

  std::vector<std::string> history() const
  {
    std::vector<std::string> names;
    for (const auto state : fsm->states_history()) {
      names.push_back(mtc::manipulation::to_string(state));
    }
    return names;
  }

  std::vector<std::string> motion_primitives() const
  {
    std::vector<std::string> names;
    for (const auto & command : motion->commands) {
      names.push_back(command.primitive_name);
    }
    return names;
  }
};

}  // namespace

TEST(PickPlaceV1, HappyPathReachesSuccess)
{
  Harness harness;
  harness.start();
  const auto result = harness.run();

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.final_state, PickPlaceState::SUCCESS);
  EXPECT_EQ(result.error_code, ErrorCodes::OK);
  // placement_verified 只能由 VERIFY_PLACED 成功路径设置
  EXPECT_TRUE(result.placement_verified);
  EXPECT_TRUE(harness.fsm->placement_verified());
  const auto history = harness.history();
  EXPECT_NE(std::find(history.begin(), history.end(), "VERIFY_PLACED"), history.end());
}

TEST(PickPlaceV1, StateSequenceMatchesDesignOrder)
{
  Harness harness;
  harness.start();
  harness.run();

  const std::vector<std::string> expected = {
    "RESOLVE_TARGET",
    "PLAN_ACQUIRE",
    "MOVE_PRE_ALIGN",
    "ENGAGE",
    "VERIFY_ENGAGEMENT",
    "TEST_LIFT",
    "VERIFY_ATTACHED",
    "LIFT",
    "TRANSPORT",
    "MOVE_PRE_SEAT",
    "SEAT",
    "VERIFY_SEATED",
    "UNLOAD",
    "DISENGAGE",
    "RETREAT",
    "VERIFY_PLACED",
    "SUCCESS",
  };
  // 完整流水线顺序必须与设计 §3.2 逐项一致
  EXPECT_EQ(harness.history(), expected);
}

TEST(PickPlaceV1, MotionCommandSequenceUsesOnlyNonToolPrimitives)
{
  Harness harness;
  harness.start();
  harness.run();

  // 只有 MOVE_PRE_ALIGN / TEST_LIFT / LIFT / TRANSPORT / MOVE_PRE_SEAT /
  // SEAT / RETREAT 会发运动指令；ENGAGE、UNLOAD、DISENGAGE 走 ToolPort。
  const std::vector<std::string> expected = {
    "side_insert",
    "test_lift",
    "lift",
    "transport",
    "pre_seat",
    "seat_contact",
    "retreat",
  };
  EXPECT_EQ(harness.motion_primitives(), expected);
}

TEST(PickPlaceV1, NoElectromagneticActionAtAll)
{
  Harness harness;
  harness.start();
  harness.run();

  // V1 完全不触碰电磁 IO
  EXPECT_EQ(harness.tool->unlock_calls, 0);
  EXPECT_TRUE(harness.tool->unlock_pulses.empty());
  EXPECT_EQ(
    harness.tool->io_events,
    (std::vector<std::string>{"ENGAGE:hook_engage", "disengage:side_withdraw"}));
  for (const auto & event : harness.tool->io_events) {
    EXPECT_EQ(event.find("electromagnetic"), std::string::npos);
  }
}

TEST(PickPlaceV1, ToolPrimitivesAreRecordedInOrder)
{
  Harness harness;
  harness.start();
  harness.run();

  // 经 ToolPort 的原语序列（含 engage / unload / disengage）
  const std::vector<std::string> expected = {
    "hook_engage",
    "test_lift",
    "unload",
    "side_withdraw",
  };
  EXPECT_EQ(harness.tool->received_primitives, expected);
}

TEST(PickPlaceV1, CommandIdsAreUnique)
{
  Harness harness;
  harness.start();
  harness.run();

  const auto & ids = harness.fsm->command_ids();
  const std::set<std::string> unique(ids.begin(), ids.end());
  EXPECT_EQ(unique.size(), ids.size());
  EXPECT_FALSE(ids.empty());
  // command_id 必须可追溯到本次请求
  for (const auto & id : ids) {
    EXPECT_NE(id.find("req-test"), std::string::npos) << id;
  }
}

TEST(PickPlaceV1, StabilityWindowIsActuallyElapsed)
{
  Harness harness;
  harness.start();
  harness.run();

  // 必须真的累计满 3.0s，而不是"一进入 VERIFY_PLACED 就成功"
  EXPECT_GE(harness.fsm->stability_verified_for_s(), 3.0 - 1e-9);
}

TEST(PickPlaceV1, ProgressReachesOneAtSuccess)
{
  Harness harness;
  harness.start();
  harness.run();

  const auto feedback = harness.fsm->feedback();
  EXPECT_EQ(feedback.substate, "SUCCESS");
  EXPECT_FLOAT_EQ(feedback.progress, 1.0f);
  // 成功路径上挂载估计必须为 true（VERIFY_ATTACHED 已通过）
  EXPECT_TRUE(feedback.object_attached_estimated);
}

TEST(PickPlaceV1, PlannerReceivesBothPhasesExactlyOnce)
{
  Harness harness;
  harness.start();
  harness.run();

  EXPECT_EQ(harness.planner->calls, 2);
  ASSERT_EQ(harness.planner->phases.size(), 2u);
  EXPECT_EQ(harness.planner->phases[0], "ACQUIRE");
  EXPECT_EQ(harness.planner->phases[1], "TRANSPORT");
  // 规划目标必须携带 6D 提环位姿与 frame_id
  ASSERT_TRUE(harness.planner->targets[0].has_ring_pose);
  EXPECT_EQ(harness.planner->targets[0].ring_pose.frame_id, "base_link");
  EXPECT_TRUE(harness.planner->targets[0].ring_pose.is_usable());
  EXPECT_EQ(harness.planner->targets[0].object_id, "battery_1");
  EXPECT_EQ(harness.planner->targets[0].slot, "P1");
}

TEST(PickPlaceV1, PlacementVerificationSkipsColorCheck)
{
  Harness harness;
  harness.start();
  harness.run();

  // 放置核验必须以 expected_color=nullopt 发起（-1 表示 nullopt）
  bool saw_nullopt_query = false;
  for (const int color : harness.perception->requested_colors) {
    if (color == -1) {
      saw_nullopt_query = true;
      break;
    }
  }
  EXPECT_TRUE(saw_nullopt_query);
}

TEST(PickPlaceV1, RunWithZeroTickDoesNotSucceedWithoutTime)
{
  // 若时钟完全不推进，稳定性窗口永不满足 —— 不得凭 step 次数宣告成功。
  Harness harness;
  harness.start();
  harness.run(200, 0.0);

  EXPECT_NE(harness.fsm->state(), PickPlaceState::SUCCESS);
  EXPECT_FALSE(harness.fsm->placement_verified());
}

TEST(PickPlaceV1, StepUntilTerminalReachesFailedOnPerceptionFailure)
{
  Harness harness;
  harness.perception->found = false;
  harness.start();

  // step_until_terminal 不推进时钟，因此只适合"不需等待稳定性窗口"的路径。
  const auto terminal = harness.fsm->step_until_terminal(1200);
  EXPECT_EQ(terminal, PickPlaceState::FAILED);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::TARGET_NOT_FOUND);
}

TEST(PickPlaceV1, ReadyToRunStatesAreReachedWithoutMotionOnPerceptionFailure)
{
  Harness harness;
  harness.perception->found = false;
  harness.start();
  harness.run(300);

  // 感知失败时不得发出任何运动指令，也不得进入 MOVE_PRE_ALIGN
  EXPECT_TRUE(harness.motion->commands.empty());
  const auto history = harness.history();
  EXPECT_EQ(
    std::find(history.begin(), history.end(), "MOVE_PRE_ALIGN"), history.end());
  // 感知失败可重试：RECOVERY 至少被访问一次
  EXPECT_NE(std::find(history.begin(), history.end(), "RECOVERY"), history.end());
  EXPECT_FALSE(harness.fsm->placement_verified());
}
