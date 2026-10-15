// Copyright 2026 meituan_challenge_ws contributors
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <set>
#include "fakes.hpp"
#include "mtc_manipulation_cpp/pick_place_fsm.hpp"
#include "mtc_manipulation_cpp/motion_adapter_port.hpp"
#include "mtc_motion_execution_cpp/mock_motion_adapter.hpp"

namespace {
using namespace mtc::manipulation;
using namespace mtc::manipulation::testing;
using mtc::core::ErrorCodes;
struct Fixture
{
  std::shared_ptr<mtc::core::ManualClock> clock = std::make_shared<mtc::core::ManualClock>(100);
  std::shared_ptr<FakePerception> perception = std::make_shared<FakePerception>();
  std::shared_ptr<FakePlanner> planner = std::make_shared<FakePlanner>();
  std::shared_ptr<FakeMotion> motion = std::make_shared<FakeMotion>();
  std::shared_ptr<FakeTool> tool = std::make_shared<FakeTool>();
  std::unique_ptr<PickPlaceFsm> fsm;
  Fixture()
  {
    perception->clock = clock;
    build(motion);
  }
  void build(MotionPortPtr port, PickPlaceConfig config = {})
  {
    fsm = std::make_unique<PickPlaceFsm>(perception, planner, port, tool, clock,
      make_strategy(TOOL_PASSIVE_HOOK_V1), config);
  }
  void start(const std::string & id = "audit") {fsm->start("battery_1", 1, "P1", id);}
  void reach(PickPlaceState state)
  {
    for (int i = 0; i < 100 && fsm->state() != state &&
      !terminal_states().count(fsm->state()); ++i) {fsm->step();}
    ASSERT_EQ(fsm->state(), state);
  }
  void samples(int count)
  {
    for (int i = 0; i < count; ++i) {fsm->step(); clock->advance(0.2);}
  }
};

class CaptureTrajectory : public FakeMotion
{
public:
  std::vector<mtc::core::MotionCommand> trajectories;
  std::optional<mtc::core::MotionResult> execute_trajectory(
    const mtc::core::MotionCommand & command) override
  {
    trajectories.push_back(command);
    mtc::core::MotionResult r;
    r.exec_state = mtc::core::ExecState::REJECTED;
    r.error_code = ErrorCodes::EXECUTION_REJECTED;
    return r;
  }
};
}

TEST(PlacementAudit, NewFramesMustCoverFullThreeSeconds)
{
  Fixture h;
  h.start(); h.reach(PickPlaceState::VERIFY_PLACED);
  h.samples(15); // t=100..102.8. Fifteen distinct samples still do not span 3s.
  EXPECT_FALSE(h.fsm->placement_verified());
  h.fsm->step(); // A new frame at t=103.0.
  EXPECT_TRUE(h.fsm->placement_verified());
  EXPECT_EQ(h.fsm->state(), PickPlaceState::SUCCESS);
  EXPECT_NEAR(h.fsm->stability_verified_for_s(), 3.0, 1e-9);
}

TEST(PlacementAudit, FrozenTimestampCannotSucceedForFiveSeconds)
{
  Fixture h;
  h.perception->fixed_stamp = 100;
  h.start(); h.reach(PickPlaceState::VERIFY_PLACED);
  h.samples(26);
  EXPECT_FALSE(h.fsm->placement_verified());
  EXPECT_EQ(h.fsm->stability_verified_for_s(), 0.0);
}

TEST(PlacementAudit, InvalidOrStaleFrameInterruptsWindow)
{
  for (int failure = 0; failure < 4; ++failure) {
    Fixture h;
    h.start(); h.reach(PickPlaceState::VERIFY_PLACED);
    h.samples(11);
    EXPECT_GE(h.fsm->stability_verified_for_s(), 2.0 - 1e-9);
    h.perception->placement_stale = failure == 0;
    h.perception->placement_found = failure != 1;
    h.perception->placement_at_target = failure != 2;
    h.perception->placement_settled = failure != 3;
    h.fsm->step();
    EXPECT_EQ(h.fsm->stability_verified_for_s(), 0.0);
    h.clock->advance(0.2);
    h.perception->placement_stale = false;
    h.perception->placement_found = true;
    h.perception->placement_at_target = h.perception->placement_settled = true;
    h.samples(15);
    EXPECT_FALSE(h.fsm->placement_verified());
    h.fsm->step();
    EXPECT_TRUE(h.fsm->placement_verified());
  }
}

TEST(PlacementAudit, PerceptionStopsUpdatingCannotAccumulateTime)
{
  Fixture h;
  h.start(); h.reach(PickPlaceState::VERIFY_PLACED);
  h.samples(11);
  h.perception->always_pending = true;
  h.samples(26);
  EXPECT_FALSE(h.fsm->placement_verified());
  EXPECT_EQ(h.fsm->stability_verified_for_s(), 0.0);
  h.perception->always_pending = false;
  h.samples(15);
  EXPECT_FALSE(h.fsm->placement_verified());
}

TEST(PlacementAudit, FutureBackwardsAndNonFiniteStampsInterrupt)
{
  for (double stamp : {99.0, 110.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()}) {
    Fixture h;
    h.start(); h.reach(PickPlaceState::VERIFY_PLACED);
    h.samples(8);
    h.perception->fixed_stamp = stamp;
    h.fsm->step();
    EXPECT_FALSE(h.fsm->placement_verified());
    EXPECT_EQ(h.fsm->stability_verified_for_s(), 0.0);
  }
}

TEST(PlacementAudit, GapBetweenFreshFramesInterruptsContinuity)
{
  Fixture h;
  h.start(); h.reach(PickPlaceState::VERIFY_PLACED);
  h.samples(8);
  h.clock->advance(5.0);
  h.fsm->step(); // Fresh frame after silence cannot extend the old window.
  EXPECT_FALSE(h.fsm->placement_verified());
  EXPECT_EQ(h.fsm->stability_verified_for_s(), 0.0);
}

TEST(PlanningAudit, NonZeroPlannerTargetsAndFullPoseReachAdapter)
{
  Fixture h;
  auto adapter = std::make_shared<mtc::motion::MockMotionAdapter>(
    mtc::motion::MockMotionAdapterConfig(), h.clock);
  h.build(std::make_shared<MotionAdapterPort>(adapter, mtc::motion::ValidationConfig(), h.clock));
  auto pose = make_pose(0.3, -0.2, 0.8, "world", 100.0);
  pose.orientation = {0, 0, std::sqrt(0.5), std::sqrt(0.5)};
  h.perception->ring_override = pose;
  h.start(); h.reach(PickPlaceState::ENGAGE);
  const auto commands = adapter->call_log();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands[0].target_rad, h.planner->planned_target);
  ASSERT_TRUE(commands[0].intent);
  ASSERT_TRUE(commands[0].intent->target_pose);
  EXPECT_EQ(*commands[0].intent->target_pose, pose);
  EXPECT_EQ(h.planner->targets[0].ring_pose, pose);
  EXPECT_EQ(h.planner->targets[0].primitives[0].max_linear_speed,
    commands[0].intent->max_linear_speed);
}

TEST(PlanningAudit, MissingOutputDoesNotReachMockAdapter)
{
  Fixture h;
  auto adapter = std::make_shared<mtc::motion::MockMotionAdapter>(
    mtc::motion::MockMotionAdapterConfig(), h.clock);
  h.build(std::make_shared<MotionAdapterPort>(adapter, mtc::motion::ValidationConfig(), h.clock));
  h.planner->provide_commands = false;
  h.start(); h.samples(20);
  EXPECT_TRUE(adapter->call_log().empty());
  EXPECT_EQ(h.fsm->error_code(), ErrorCodes::PLANNING_FAILED);
}

TEST(PlanningAudit, WholeTimedTrajectoryIsNeverSplit)
{
  Fixture h;
  auto motion = std::make_shared<CaptureTrajectory>();
  h.build(motion);
  h.planner->provide_trajectory = true;
  h.start(); h.reach(PickPlaceState::MOVE_PRE_ALIGN);
  h.fsm->step();
  EXPECT_EQ(motion->execute_calls, 0);
  ASSERT_EQ(motion->trajectories.size(), 1u);
  const auto & c = motion->trajectories[0];
  ASSERT_TRUE(c.trajectory);
  EXPECT_EQ(c.trajectory->points.size(), 2u);
  EXPECT_DOUBLE_EQ(c.trajectory->points[1].time_from_start_s, 1.0);
  EXPECT_EQ(c.trajectory->header_frame_id, "base_link");
  EXPECT_TRUE(c.target_rad.empty());
}

TEST(PlanningAudit, MalformedPlanCannotExecute)
{
  Fixture h;
  h.planner->planned_target[1] = std::numeric_limits<double>::quiet_NaN();
  h.start(); h.samples(10);
  EXPECT_TRUE(h.motion->commands.empty());
  EXPECT_EQ(h.fsm->error_code(), ErrorCodes::PLANNING_FAILED);
}

TEST(CommandIdAudit, LongAndRepeatedRequestsRemainUniqueAcrossInstances)
{
  const std::string request(2048, 'x');
  std::set<std::string> ids;
  for (int instance = 0; instance < 2; ++instance) {
    Fixture h;
    for (int run = 0; run < 2; ++run) {
      h.start(request);
      h.reach(PickPlaceState::VERIFY_PLACED);
      h.samples(17);
      ASSERT_EQ(h.fsm->state(), PickPlaceState::SUCCESS);
      for (const auto & id : h.fsm->command_ids()) {
        EXPECT_EQ(id.substr(0, request.size()), request);
        EXPECT_GT(id.size(), request.size());
        EXPECT_TRUE(ids.insert(id).second);
      }
    }
  }
  EXPECT_EQ(ids.size(), 28u);
}

TEST(WatchdogAudit, LoadedContextCannotRecoverAndFaultCannotRestart)
{
  Fixture h;
  h.perception->always_pending = true;
  h.tool->holding = "holding";
  h.start(); h.clock->advance(2.1);
  const auto result = h.fsm->step();
  EXPECT_EQ(result.state, h.fsm->state());
  EXPECT_EQ(result.state, PickPlaceState::FAILED);
  EXPECT_FALSE(result.resumable);
  Fixture locked;
  locked.start();
  locked.fsm->force_fault(ErrorCodes::MOTION_STATUS_UNKNOWN, "feedback lost");
  EXPECT_THROW(locked.start("new-goal"), std::logic_error);
  EXPECT_EQ(locked.fsm->state(), PickPlaceState::FAULT);
}

TEST(WatchdogAudit, InvalidStabilityNeverStartsMotion)
{
  for (double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
    Fixture h;
    h.fsm->start("battery_1", 1, "P1", "bad", bad);
    EXPECT_EQ(h.fsm->state(), PickPlaceState::FAILED);
    EXPECT_EQ(h.motion->execute_calls, 0);
  }
}

TEST(PlacementGate, ZeroStabilityRejectedBeforeFirstFrame)
{
  Fixture h;
  h.fsm->start("battery_1", 1, "P1", "zero", 0.0);
  EXPECT_EQ(h.fsm->state(), PickPlaceState::FAILED);
  h.samples(30);
  EXPECT_EQ(h.fsm->error_code(), ErrorCodes::INVALID_TASK);
  EXPECT_FALSE(h.fsm->placement_verified());
  EXPECT_EQ(h.motion->execute_calls, 0);
  EXPECT_TRUE(h.planner->targets.empty());
}

TEST(PlacementGate, PositiveWindowNeedsTwoDistinctObservations)
{
  Fixture h;
  h.fsm->start("battery_1", 1, "P1", "tiny", 1e-12);
  h.reach(PickPlaceState::VERIFY_PLACED);
  h.fsm->step();
  EXPECT_FALSE(h.fsm->placement_verified());
  h.clock->advance(0.1);
  h.fsm->step();
  EXPECT_TRUE(h.fsm->placement_verified());
}

TEST(PlacementGate, InvalidConfiguredStabilityNeverStartsMotion)
{
  for (double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
    Fixture h;
    PickPlaceConfig config; config.placement_stability_sec = bad;
    h.build(h.motion, config); h.start(); h.samples(30);
    EXPECT_EQ(h.fsm->state(), PickPlaceState::FAILED);
    EXPECT_FALSE(h.fsm->placement_verified());
    EXPECT_EQ(h.motion->execute_calls, 0);
    EXPECT_TRUE(h.planner->targets.empty());
  }
}
