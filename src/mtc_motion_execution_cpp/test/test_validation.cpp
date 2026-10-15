// Copyright 2026 meituan_challenge_ws contributors
//
// 执行前校验测试（对齐 Python src/mtc_motion_execution/test/test_backends.py）。

#include <gtest/gtest.h>

#include <string>
#include <limits>
#include <vector>

#include "mtc_core_cpp/error_codes.hpp"
#include "mtc_motion_execution_cpp/validation.hpp"

namespace
{

using mtc::core::ErrorCodes;
using mtc::core::MotionCapabilities;
using mtc::core::MotionCommand;
using mtc::core::TimedTrajectory;
using mtc::core::TrajectoryPoint;
using mtc::motion::ValidationConfig;

MotionCapabilities joint_only_capabilities()
{
  MotionCapabilities capabilities;
  capabilities.backend_name = "mock";
  capabilities.simulation_only = true;
  capabilities.joint_goal_supported = true;
  capabilities.timed_trajectory_supported = false;
  capabilities.cancel_supported = true;
  return capabilities;
}

MotionCommand make_command(
  const std::string & command_id = "cmd-1",
  const std::vector<double> & target = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6},
  uint32_t timeout_ms = 5000)
{
  MotionCommand command;
  command.command_id = command_id;
  command.joint_names = {
    "shoulder_joint", "upperArm_joint", "foreArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint",
  };
  command.target_rad = target;
  command.velocity_scaling = 0.1;
  command.acceleration_scaling = 0.1;
  command.timeout_ms = timeout_ms;
  return command;
}

}  // namespace

TEST(Validation, ValidCommandPasses)
{
  const ValidationConfig config;
  const auto outcome =
    mtc::motion::validate_joint_move(make_command(), config, joint_only_capabilities());
  EXPECT_TRUE(outcome.ok);
  EXPECT_EQ(outcome.error_code, ErrorCodes::OK);
  EXPECT_EQ(outcome.message, "validated");
}

TEST(Validation, MissingJointGoalCapabilityRejected)
{
  MotionCapabilities capabilities = joint_only_capabilities();
  capabilities.joint_goal_supported = false;

  const auto outcome =
    mtc::motion::validate_joint_move(make_command(), ValidationConfig(), capabilities);
  EXPECT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.error_code, ErrorCodes::EXECUTION_REJECTED);
  EXPECT_NE(outcome.message.find("joint_goal_supported"), std::string::npos);
}

TEST(Validation, DimensionMismatchRejected)
{
  MotionCommand command = make_command();
  command.target_rad.pop_back();   // 少一个目标

  const auto outcome = mtc::motion::validate_joint_move(
    command, ValidationConfig(), joint_only_capabilities());
  EXPECT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.error_code, ErrorCodes::EXECUTION_REJECTED);
  EXPECT_NE(outcome.message.find("一一对应"), std::string::npos);
}

TEST(Validation, EmptyJointNamesRejected)
{
  MotionCommand command = make_command();
  command.joint_names.clear();
  command.target_rad.clear();

  const auto outcome = mtc::motion::validate_joint_move(
    command, ValidationConfig(), joint_only_capabilities());
  EXPECT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.error_code, ErrorCodes::EXECUTION_REJECTED);
}

TEST(Validation, DuplicateJointNamesRejected)
{
  MotionCommand command = make_command();
  command.joint_names[1] = command.joint_names[0];

  const auto outcome = mtc::motion::validate_joint_move(
    command, ValidationConfig(), joint_only_capabilities());
  EXPECT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.error_code, ErrorCodes::EXECUTION_REJECTED);
  EXPECT_NE(outcome.message.find("重复"), std::string::npos);
}

TEST(Validation, JointOrderMustMatchConfiguration)
{
  // 禁止假定固定 J1..J6 顺序：顺序不符必须拒绝
  MotionCommand command = make_command();
  std::swap(command.joint_names[0], command.joint_names[1]);
  std::swap(command.target_rad[0], command.target_rad[1]);

  const auto outcome = mtc::motion::validate_joint_move(
    command, ValidationConfig(), joint_only_capabilities());
  EXPECT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.error_code, ErrorCodes::EXECUTION_REJECTED);
  EXPECT_NE(outcome.message.find("顺序"), std::string::npos);
}

TEST(Validation, EmptyExpectedNamesSkipsOrderCheck)
{
  ValidationConfig config;
  config.expected_joint_names.clear();

  MotionCommand command = make_command();
  command.joint_names = {"j1", "j2", "j3", "j4", "j5", "j6"};

  const auto outcome =
    mtc::motion::validate_joint_move(command, config, joint_only_capabilities());
  EXPECT_TRUE(outcome.ok);
}

TEST(Validation, ZeroTimeoutRejected)
{
  const auto outcome = mtc::motion::validate_joint_move(
    make_command("cmd-1", {0, 0, 0, 0, 0, 0}, 0), ValidationConfig(),
    joint_only_capabilities());
  EXPECT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.error_code, ErrorCodes::EXECUTION_REJECTED);
  EXPECT_NE(outcome.message.find("timeout_ms"), std::string::npos);
}

TEST(Validation, VelocityScalingAboveLimitIsSafetyInterlock)
{
  MotionCommand command = make_command();
  command.velocity_scaling = 0.5;   // 默认上限 0.2

  const auto outcome = mtc::motion::validate_joint_move(
    command, ValidationConfig(), joint_only_capabilities());
  EXPECT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.error_code, ErrorCodes::SAFETY_INTERLOCK);
}

TEST(Validation, ZeroOrNegativeVelocityScalingRejected)
{
  MotionCommand command = make_command();
  command.velocity_scaling = 0.0;
  EXPECT_EQ(
    mtc::motion::validate_joint_move(command, ValidationConfig(), joint_only_capabilities())
      .error_code,
    ErrorCodes::SAFETY_INTERLOCK);

  command.velocity_scaling = -0.1;
  EXPECT_EQ(
    mtc::motion::validate_joint_move(command, ValidationConfig(), joint_only_capabilities())
      .error_code,
    ErrorCodes::SAFETY_INTERLOCK);
}

TEST(Validation, AccelerationScalingAboveLimitIsSafetyInterlock)
{
  MotionCommand command = make_command();
  command.acceleration_scaling = 0.9;

  const auto outcome = mtc::motion::validate_joint_move(
    command, ValidationConfig(), joint_only_capabilities());
  EXPECT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.error_code, ErrorCodes::SAFETY_INTERLOCK);
}

TEST(Validation, JointLimitsAreEnforced)
{
  ValidationConfig config;
  config.joint_limit_rad = {
    {-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0},
  };

  EXPECT_TRUE(
    mtc::motion::validate_joint_move(make_command(), config, joint_only_capabilities()).ok);

  MotionCommand out_of_range = make_command("cmd-2", {0.0, 0.0, 0.0, 0.0, 0.0, 5.0});
  const auto outcome =
    mtc::motion::validate_joint_move(out_of_range, config, joint_only_capabilities());
  EXPECT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.error_code, ErrorCodes::SAFETY_INTERLOCK);
  EXPECT_NE(outcome.message.find("wrist3_joint"), std::string::npos);
}

TEST(Validation, JointLimitDimensionMismatchRejected)
{
  ValidationConfig config;
  config.joint_limit_rad = {{-1.0, 1.0}};   // 只给 1 个，但目标是 6 个

  const auto outcome =
    mtc::motion::validate_joint_move(make_command(), config, joint_only_capabilities());
  EXPECT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.error_code, ErrorCodes::EXECUTION_REJECTED);
  EXPECT_NE(outcome.message.find("维度"), std::string::npos);
}

// ---------------------------------------------------------------------------
// 时间参数化轨迹校验（缺口 G1 的第一阶段守门人）
// ---------------------------------------------------------------------------

namespace
{

TimedTrajectory make_trajectory()
{
  TimedTrajectory trajectory;
  trajectory.trajectory_id = "traj-1";
  trajectory.joint_names = {
    "shoulder_joint", "upperArm_joint", "foreArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint",
  };
  trajectory.header_frame_id = "base_link";

  TrajectoryPoint p1;
  p1.time_from_start_s = 0.0;
  p1.positions_rad = {0, 0, 0, 0, 0, 0};
  TrajectoryPoint p2;
  p2.time_from_start_s = 0.5;
  p2.positions_rad = {0.05, -0.02, 0.01, 0, 0, 0};
  TrajectoryPoint p3;
  p3.time_from_start_s = 1.0;
  p3.positions_rad = {0.1, -0.04, 0.02, 0, 0, 0};
  trajectory.points = {p1, p2, p3};
  return trajectory;
}

}  // namespace

TEST(ValidationTrajectory, UnsupportedBackendIsRejectedNotDowngraded)
{
  // 未声明 timed_trajectory_supported 的后端必须被拒绝，
  // 且拒绝理由必须点明"禁止拆分"。
  const auto outcome = mtc::motion::validate_timed_trajectory(
    make_trajectory(), ValidationConfig(), joint_only_capabilities());
  EXPECT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.error_code, ErrorCodes::EXECUTION_REJECTED);
  EXPECT_NE(outcome.message.find("timed_trajectory_supported"), std::string::npos);
  EXPECT_NE(outcome.message.find("拆"), std::string::npos);
}

TEST(ValidationTrajectory, SupportedBackendAcceptsValidTrajectory)
{
  MotionCapabilities capabilities = joint_only_capabilities();
  capabilities.timed_trajectory_supported = true;

  const auto outcome =
    mtc::motion::validate_timed_trajectory(make_trajectory(), ValidationConfig(), capabilities);
  EXPECT_TRUE(outcome.ok);
}

TEST(ValidationTrajectory, InvalidShapeRejected)
{
  MotionCapabilities capabilities = joint_only_capabilities();
  capabilities.timed_trajectory_supported = true;

  TimedTrajectory trajectory = make_trajectory();
  trajectory.points.clear();
  auto outcome =
    mtc::motion::validate_timed_trajectory(trajectory, ValidationConfig(), capabilities);
  EXPECT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.error_code, ErrorCodes::EXECUTION_REJECTED);

  trajectory = make_trajectory();
  trajectory.points[1].positions_rad.pop_back();   // 维度不一致
  outcome = mtc::motion::validate_timed_trajectory(trajectory, ValidationConfig(), capabilities);
  EXPECT_FALSE(outcome.ok);
}

TEST(ValidationTrajectory, JointOrderMustMatchConfiguration)
{
  MotionCapabilities capabilities = joint_only_capabilities();
  capabilities.timed_trajectory_supported = true;

  TimedTrajectory trajectory = make_trajectory();
  std::swap(trajectory.joint_names[0], trajectory.joint_names[1]);
  for (auto & point : trajectory.points) {
    std::swap(point.positions_rad[0], point.positions_rad[1]);
  }

  const auto outcome =
    mtc::motion::validate_timed_trajectory(trajectory, ValidationConfig(), capabilities);
  EXPECT_FALSE(outcome.ok);
  EXPECT_NE(outcome.message.find("joint_names"), std::string::npos);
}

TEST(ValidationTrajectory, JointLimitsEnforcedOnEveryPoint)
{
  MotionCapabilities capabilities = joint_only_capabilities();
  capabilities.timed_trajectory_supported = true;

  ValidationConfig config;
  config.joint_limit_rad = {
    {-0.01, 0.01}, {-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0},
  };

  // 第二个路点的 shoulder 已超出 0.01
  const auto outcome =
    mtc::motion::validate_timed_trajectory(make_trajectory(), config, capabilities);
  EXPECT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.error_code, ErrorCodes::SAFETY_INTERLOCK);
}

TEST(ValidationAudit, NonFiniteTargetsRejectedWithoutLimits)
{
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
    auto c = make_command();
    c.target_rad[2] = bad;
    EXPECT_FALSE(mtc::motion::validate_joint_move(c, {}, joint_only_capabilities()).ok);
    c = make_command();
    c.velocity_scaling = bad;
    EXPECT_FALSE(mtc::motion::validate_joint_move(c, {}, joint_only_capabilities()).ok);
    c = make_command();
    c.acceleration_scaling = bad;
    EXPECT_FALSE(mtc::motion::validate_joint_move(c, {}, joint_only_capabilities()).ok);
  }
}

TEST(ValidationAudit, ExecutableBackendRequiresLimitsAndRejectsFakePlan)
{
  auto caps = joint_only_capabilities();
  caps.simulation_only = false;
  caps.associated_cancel_supported = true;
  ValidationConfig config;
  EXPECT_FALSE(mtc::motion::validate_joint_move(make_command(), config, caps).ok);
  config.joint_limit_rad.assign(6, {-1.0, 1.0});
  EXPECT_TRUE(mtc::motion::validate_joint_move(make_command(), config, caps).ok);
  auto c = make_command();
  c.simulation_only = true;
  EXPECT_FALSE(mtc::motion::validate_joint_move(c, config, caps).ok);
  config.joint_limit_rad[0] = {1.0, -1.0};
  EXPECT_FALSE(mtc::motion::validate_joint_move(make_command(), config, caps).ok);
}

TEST(ValidationAudit, TrajectoryRejectsExcessiveDynamicsAndInvalidLimits)
{
  auto caps = joint_only_capabilities();
  caps.timed_trajectory_supported = true;
  auto t = make_trajectory();
  t.points[1].velocities_rad_s.assign(6, 999.0);
  EXPECT_FALSE(mtc::motion::validate_timed_trajectory(t, {}, caps).ok);
  t = make_trajectory();
  t.points[1].accelerations_rad_s2.assign(6, 999.0);
  EXPECT_FALSE(mtc::motion::validate_timed_trajectory(t, {}, caps).ok);
  ValidationConfig config;
  config.max_joint_velocity_rad_s[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(mtc::motion::validate_timed_trajectory(make_trajectory(), config, caps).ok);
  config = {};
  config.max_velocity_scaling = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(mtc::motion::validate_joint_move(make_command(), config, caps).ok);
}
