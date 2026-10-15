// Copyright 2026 meituan_challenge_ws contributors
//
// 用已合入 main 的真实 PickPlaceFsm 驱动 MoveItPlannerPort（假规划后端）：
//   * 规划输出必须通过 PickPlaceFsm::accept_plan()；
//   * 每个运动原语以**一条完整轨迹**经 MotionPort::execute_trajectory() 派发，
//     execute_joint_move() 调用次数为 0（禁止拆点）；
//   * V1 / V2 的单块电池流程到达 SUCCESS。
// 感知、工具、运动均为 mtc_manipulation_cpp/test/fakes.hpp 中的纯逻辑 Fake。

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "fake_backend.hpp"
#include "fakes.hpp"
#include "mtc_core_cpp/clock.hpp"
#include "mtc_manipulation_cpp/pick_place_fsm.hpp"
#include "mtc_manipulation_cpp/tool_strategy.hpp"
#include "mtc_moveit_planner/moveit_planner_port.hpp"

namespace
{

using mtc::core::ManualClock;
using mtc::manipulation::PickPlaceConfig;
using mtc::manipulation::PickPlaceFsm;
using mtc::manipulation::PickPlaceState;
using mtc::manipulation::TOOL_MAGNETIC_LATCH_V2;
using mtc::manipulation::TOOL_PASSIVE_HOOK_V1;
using mtc::manipulation::testing::FakeMotion;
using mtc::manipulation::testing::FakePerception;
using mtc::manipulation::testing::FakeTool;
using mtc::moveit_planner::MoveItPlannerPort;
using mtc::moveit_planner::default_planner_config;
using mtc::moveit_planner::testing::FakeBackend;

/// 在 FakeMotion 基础上接受整条轨迹（FakeMotion 默认继承基类的"拒绝轨迹"）。
class TrajectoryFakeMotion : public FakeMotion
{
public:
  std::vector<mtc::core::MotionCommand> trajectory_commands;

  std::optional<mtc::core::MotionResult> execute_trajectory(
    const mtc::core::MotionCommand & command) override
  {
    trajectory_commands.push_back(command);
    mtc::core::MotionResult result;
    result.exec_state = mtc::core::ExecState::SETTLED;
    result.success = command.trajectory.has_value() && command.trajectory->shape_is_valid();
    result.arrived = result.success;
    result.stop_confirmed = result.success;
    if (result.success) {
      result.final_position_rad = command.trajectory->points.back().positions_rad;
    } else {
      result.exec_state = mtc::core::ExecState::REJECTED;
      result.error_code = mtc::core::ErrorCodes::EXECUTION_REJECTED;
    }
    return result;
  }
};

struct Harness
{
  std::shared_ptr<ManualClock> clock = std::make_shared<ManualClock>(0.0);
  std::shared_ptr<FakePerception> perception = std::make_shared<FakePerception>();
  std::shared_ptr<FakeBackend> backend = std::make_shared<FakeBackend>();
  std::shared_ptr<MoveItPlannerPort> planner;
  std::shared_ptr<TrajectoryFakeMotion> motion = std::make_shared<TrajectoryFakeMotion>();
  std::shared_ptr<FakeTool> tool = std::make_shared<FakeTool>();
  std::unique_ptr<PickPlaceFsm> fsm;
  bool asynchronous;

  Harness(const std::string & tool_type, bool async)
  : asynchronous(async)
  {
    perception->clock = clock;
    planner = std::make_shared<MoveItPlannerPort>(
      backend, default_planner_config(), clock, asynchronous);
    PickPlaceConfig config;
    config.tool_type = tool_type;
    config.placement_stability_sec = 3.0;
    config.state_timeouts = mtc::manipulation::default_state_timeouts();
    fsm = std::make_unique<PickPlaceFsm>(
      perception, planner, motion, tool, clock,
      mtc::manipulation::make_strategy(tool_type), config);
  }

  mtc::manipulation::PickPlaceResult run(const std::string & slot)
  {
    fsm->start("battery_1", 1, slot, "req-planner");
    for (int i = 0; i < 4000; ++i) {
      fsm->step();
      const auto state = fsm->state();
      if (state == PickPlaceState::SUCCESS || state == PickPlaceState::FAILED ||
        state == PickPlaceState::FAULT || state == PickPlaceState::CANCELLED)
      {
        break;
      }
      if (asynchronous) {
        // 给后台规划线程真实时间完成；FSM 看门狗只看 ManualClock。
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        clock->advance(0.01);
      } else {
        clock->advance(0.2);
      }
    }
    return fsm->result();
  }
};

void expect_success_with_whole_trajectories(Harness & h, const std::string & slot)
{
  const auto result = h.run(slot);
  ASSERT_TRUE(result.success) << "final=" << to_string(result.final_state) << " code=" <<
    result.error_code << " msg=" << result.message;
  EXPECT_TRUE(result.placement_verified);
  EXPECT_EQ(h.motion->execute_calls, 0) << "trajectory must never be split into joint moves";
  ASSERT_FALSE(h.motion->trajectory_commands.empty());
  std::vector<std::string> names;
  for (const auto & c : h.motion->trajectory_commands) {
    names.push_back(c.primitive_name);
    EXPECT_FALSE(c.command_id.empty());
    ASSERT_TRUE(c.trajectory.has_value());
    EXPECT_EQ(c.trajectory->trajectory_id, c.command_id);
    EXPECT_TRUE(c.target_rad.empty());
    EXPECT_TRUE(c.simulation_only);
  }
  // FSM 实际派发的规划原语（hook_engage / unload / withdraw 等由 ToolPort 执行）。
  EXPECT_NE(std::find(names.begin(), names.end(), "test_lift"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "transport"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "retreat"), names.end());
}

TEST(FsmAcceptsPlannerOutput, V1SingleBatteryToT0Synchronous)
{
  Harness h(TOOL_PASSIVE_HOOK_V1, false);
  expect_success_with_whole_trajectories(h, "T0");
  EXPECT_EQ(h.motion->trajectory_commands.front().primitive_name, "side_insert");
}

TEST(FsmAcceptsPlannerOutput, V2SingleBatteryToT0Synchronous)
{
  Harness h(TOOL_MAGNETIC_LATCH_V2, false);
  expect_success_with_whole_trajectories(h, "T0");
  EXPECT_EQ(h.motion->trajectory_commands.front().primitive_name, "top_insert");
}

TEST(FsmAcceptsPlannerOutput, V1AsynchronousPlanningWithinPlanAcquireWatchdog)
{
  Harness h(TOOL_PASSIVE_HOOK_V1, true);
  expect_success_with_whole_trajectories(h, "T0");
}

TEST(FsmAcceptsPlannerOutput, PlanningFailureStopsBeforeAnyMotion)
{
  Harness h(TOOL_PASSIVE_HOOK_V1, false);
  h.backend->fail_labels = {"side_insert"};
  const auto result = h.run("T0");
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error_code, mtc::core::ErrorCodes::PLANNING_FAILED);
  EXPECT_TRUE(h.motion->trajectory_commands.empty());
  EXPECT_EQ(h.motion->execute_calls, 0);
}

}  // namespace
