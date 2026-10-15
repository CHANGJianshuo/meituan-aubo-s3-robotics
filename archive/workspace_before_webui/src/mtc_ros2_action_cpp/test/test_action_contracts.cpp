// Copyright 2026 MTC contributors
#include <gtest/gtest.h>
#include "mtc_ros2_action_cpp/task_server.hpp"
#include "mtc_ros2_action_cpp/motion_server.hpp"
#include <cmath>
#include <limits>
TEST(ActionContracts, TaskConversionPreservesColorOrderIdentityAndDeadline)
{
  mtc_interfaces::action::ExecuteTask::Goal g;
  g.task_id = std::string(200, 'x'); g.mode = 2; g.ordered_colors = {3, 1, 4};
  g.timeout_ms = 12345; g.placement_stability_sec = 3.0f;
  auto r = mtc::ros2_action::task_request(g);
  EXPECT_EQ(r.task_id, g.task_id);
  EXPECT_EQ(r.ordered_colors, (std::vector<int>{3, 1, 4}));
  EXPECT_EQ(r.slots(), (std::vector<std::string>{"P1", "P2", "P3"}));
  EXPECT_EQ(r.timeout_ms, 12345u); EXPECT_EQ(r.placement_stability_sec, 3);
}
TEST(ActionContracts, RosFloatBoundaryDoesNotRelaxCoreUpperLimit)
{
  mtc_interfaces::action::ExecuteJointMove::Goal g;
  g.command_id = "mapped";
  auto config = mtc::motion::ValidationConfig();
  std::copy(config.expected_joint_names.begin(), config.expected_joint_names.end(), g.joint_names.begin());
  g.target_rad = {0.03, 0.01, 0.06, 0.01, 0.04, 0.02};
  g.velocity_scaling = g.acceleration_scaling = 0.2f; g.timeout_ms = 1000;
  mtc::motion::MockMotionAdapter adapter;
  auto c = mtc::ros2_action::joint_command(g);
  EXPECT_TRUE(mtc::motion::validate_joint_move(c, config, adapter.capabilities()).ok);
  EXPECT_EQ(c.target_rad, (std::vector<double>{0.03, 0.01, 0.06, 0.01, 0.04, 0.02}));
  EXPECT_FALSE(c.trajectory); EXPECT_TRUE(c.simulation_only);
  g.velocity_scaling = std::nextafter(0.2f, std::numeric_limits<float>::infinity());
  EXPECT_FALSE(mtc::motion::validate_joint_move(mtc::ros2_action::joint_command(g),
    config, adapter.capabilities()).ok);
}
TEST(ActionContracts, MockPlannerPreservesSixDPoseFrameStampAndContactConstraints)
{
  mtc::ros2_action::MockPlanner planner({});
  mtc::manipulation::PlanTarget request;
  mtc::core::MotionPrimitive p;
  p.name = "contact"; p.requires_contact = true; p.max_linear_speed = 0.01;
  mtc::core::Pose6D pose;
  pose.position = {0.1, -0.2, 0.3}; pose.orientation = {0, 0, 1, 0};
  pose.frame_id = "test_frame"; pose.stamp_s = 12.3; pose.valid = true;
  p.target_pose = pose; request.primitives = {p};
  auto r = planner.plan(request); ASSERT_TRUE(r); ASSERT_EQ(r->commands.size(), 1u);
  const auto & c = r->commands.front();
  ASSERT_TRUE(c.intent); ASSERT_TRUE(c.intent->target_pose);
  EXPECT_EQ(*c.intent->target_pose, pose);
  EXPECT_TRUE(c.requires_contact); EXPECT_EQ(c.intent->max_linear_speed, 0.01);
  EXPECT_TRUE(c.simulation_only); EXPECT_FALSE(c.trajectory);
}
TEST(ActionContracts, MockLoadEvidenceRemainsHoldingUntilVerifiedRelease)
{
  mtc::ros2_action::MockTool tool({});
  EXPECT_EQ(tool.holding_state(), "none");
  ASSERT_TRUE(tool.engage("ENGAGE", {})->ok);
  EXPECT_EQ(tool.holding_state(), "holding");
  ASSERT_TRUE(tool.unload({})->unloaded);
  EXPECT_EQ(tool.holding_state(), "holding");
  const auto released = tool.disengage({});
  ASSERT_TRUE(released->released.value_or(false));
  EXPECT_EQ(released->state, mtc::core::ToolStateCode::STATE_DETACHED);
  EXPECT_EQ(tool.holding_state(), "none");
}
TEST(ActionContracts, MockFailedReleaseNeverReportsNoLoad)
{
  mtc::ros2_action::MockOptions o; o.released = false;
  mtc::ros2_action::MockTool tool(o);
  ASSERT_TRUE(tool.engage("ENGAGE", {})->ok);
  EXPECT_FALSE(tool.disengage({})->ok);
  EXPECT_EQ(tool.holding_state(), "holding");
}
TEST(ActionContracts, TestLiftRequiresPriorSimulatedAttachmentEvidence)
{
  mtc::ros2_action::MockTool tool({});
  EXPECT_FALSE(tool.test_lift({})->has_geometry_evidence());
  ASSERT_TRUE(tool.engage("ENGAGE", {})->ok);
  EXPECT_TRUE(tool.test_lift({})->has_geometry_evidence());
}
TEST(ActionContracts, TrajectoryBridgeRefusesAndNeverSplitsIntoJointMoves)
{
  auto counts = std::make_shared<mtc::ros2_action::MockCallCounts>();
  auto clock = std::make_shared<mtc::core::ManualClock>(1.0);
  mtc::ros2_action::AsyncMockMotion bridge({}, clock, counts);
  mtc::core::MotionCommand c;
  c.trajectory = mtc::core::TimedTrajectory();
  auto r = bridge.execute_trajectory(c); ASSERT_TRUE(r);
  EXPECT_EQ(r->error_code, mtc::core::ErrorCodes::EXECUTION_REJECTED);
  EXPECT_EQ(counts->executions.load(), 0u);
  EXPECT_FALSE(bridge.adapter()->capabilities().timed_trajectory_supported);
}
TEST(ActionContracts, UnsupportedTrajectoryAndCartesianCapabilitiesCannotBeEnabled)
{
  mtc::ros2_action::MockOptions o; o.motion.timed_trajectory_supported = true;
  EXPECT_THROW(o.validate(), std::invalid_argument);
  o.motion.timed_trajectory_supported = false; o.motion.cartesian_motion_supported = true;
  EXPECT_THROW(o.validate(), std::invalid_argument);
}
