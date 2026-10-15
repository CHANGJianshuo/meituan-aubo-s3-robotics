// Copyright 2026 meituan_challenge_ws contributors
//
// mtc_core_cpp 核心类型契约测试。
//
// 重点覆盖需求 5 的"6D 位姿保真"约束与需求 3 的"核心可离线测试"约束：
// 本文件不启动 ROS、不连接任何设备。

#include <gtest/gtest.h>

#include <string>
#include <limits>
#include <vector>

#include "mtc_core_cpp/clock.hpp"
#include "mtc_core_cpp/error_codes.hpp"
#include "mtc_core_cpp/exec_state.hpp"
#include "mtc_core_cpp/motion_primitive.hpp"
#include "mtc_core_cpp/pose.hpp"
#include "mtc_core_cpp/results.hpp"
#include "mtc_core_cpp/timed_trajectory.hpp"
#include "mtc_core_cpp/tool_state.hpp"

namespace
{

using mtc::core::DetectionResult;
using mtc::core::ErrorCodes;
using mtc::core::EvidenceLevel;
using mtc::core::ExecState;
using mtc::core::ManualClock;
using mtc::core::MotionCommand;
using mtc::core::MotionPrimitive;
using mtc::core::MotionResult;
using mtc::core::Pose6D;
using mtc::core::PoseEstimate;
using mtc::core::Quaternion;
using mtc::core::TimedTrajectory;
using mtc::core::ToolEvidence;
using mtc::core::ToolStateCode;
using mtc::core::TrajectoryPoint;
using mtc::core::Vector3;

Pose6D make_usable_pose(const std::string & frame_id = "base_link")
{
  Pose6D pose;
  pose.position = Vector3{0.4, -0.1, 0.25};
  pose.orientation = Quaternion{0.0, 0.0, 0.0, 1.0};
  pose.frame_id = frame_id;
  pose.stamp_s = 12.5;
  pose.valid = true;
  return pose;
}

}  // namespace

// ---------------------------------------------------------------------------
// 错误码
// ---------------------------------------------------------------------------

TEST(CoreErrorCodes, ValuesMatchFrozenInterfaceNumbers)
{
  // 逐项硬断言：这些数值来自 mtc_interfaces/msg/ErrorCodes.msg，属冻结契约。
  EXPECT_EQ(ErrorCodes::OK, 0u);
  EXPECT_EQ(ErrorCodes::INVALID_TASK, 100u);
  EXPECT_EQ(ErrorCodes::TARGET_NOT_FOUND, 110u);
  EXPECT_EQ(ErrorCodes::POSE_STALE, 120u);
  EXPECT_EQ(ErrorCodes::PLANNING_FAILED, 200u);
  EXPECT_EQ(ErrorCodes::EXECUTION_REJECTED, 210u);
  EXPECT_EQ(ErrorCodes::MOTION_TIMEOUT, 220u);
  EXPECT_EQ(ErrorCodes::MOTION_STATUS_UNKNOWN, 230u);
  EXPECT_EQ(ErrorCodes::ENGAGE_FAILED, 300u);
  EXPECT_EQ(ErrorCodes::ATTACH_NOT_VERIFIED, 310u);
  EXPECT_EQ(ErrorCodes::OBJECT_DROPPED, 320u);
  EXPECT_EQ(ErrorCodes::SEAT_NOT_VERIFIED, 400u);
  EXPECT_EQ(ErrorCodes::UNLOCK_FAILED, 410u);
  EXPECT_EQ(ErrorCodes::RELEASE_NOT_VERIFIED, 420u);
  EXPECT_EQ(ErrorCodes::PLACEMENT_FAILED, 430u);
  EXPECT_EQ(ErrorCodes::SAFETY_INTERLOCK, 500u);
  EXPECT_EQ(ErrorCodes::HARDWARE_FAULT, 510u);
  EXPECT_EQ(ErrorCodes::CANCEL_NOT_CONFIRMED, 520u);
}

TEST(CoreErrorCodes, NameOfKnownAndUnknown)
{
  EXPECT_EQ(ErrorCodes::name_of(ErrorCodes::MOTION_STATUS_UNKNOWN), "MOTION_STATUS_UNKNOWN");
  EXPECT_EQ(ErrorCodes::name_of(ErrorCodes::CANCEL_NOT_CONFIRMED), "CANCEL_NOT_CONFIRMED");
  EXPECT_EQ(ErrorCodes::name_of(0), "OK");
  EXPECT_EQ(ErrorCodes::name_of(9999), "UNKNOWN_9999");

  EXPECT_TRUE(ErrorCodes::is_known(ErrorCodes::OK));
  EXPECT_TRUE(ErrorCodes::is_known(ErrorCodes::PLACEMENT_FAILED));
  EXPECT_FALSE(ErrorCodes::is_known(9999));
}

// ---------------------------------------------------------------------------
// 工具状态与证据等级
// ---------------------------------------------------------------------------

TEST(CoreToolState, EvidenceLevelIsOrdered)
{
  EXPECT_LT(EvidenceLevel::NONE, EvidenceLevel::COMMAND_ONLY);
  EXPECT_LT(EvidenceLevel::COMMAND_ONLY, EvidenceLevel::GEOMETRY);
  EXPECT_LT(EvidenceLevel::GEOMETRY, EvidenceLevel::SENSOR_OR_VISION);

  ToolEvidence evidence;
  evidence.evidence_level = EvidenceLevel::COMMAND_ONLY;
  // 仅有指令级证据：不得据此认定挂载成立。
  EXPECT_FALSE(evidence.has_geometry_evidence());
  evidence.evidence_level = EvidenceLevel::GEOMETRY;
  EXPECT_TRUE(evidence.has_geometry_evidence());
  evidence.evidence_level = EvidenceLevel::SENSOR_OR_VISION;
  EXPECT_TRUE(evidence.has_geometry_evidence());
}

TEST(CoreToolState, NamesAreStable)
{
  EXPECT_EQ(ToolStateCode::name_of(ToolStateCode::STATE_ATTACHED), "STATE_ATTACHED");
  EXPECT_EQ(EvidenceLevel::name_of(EvidenceLevel::GEOMETRY), "EVIDENCE_GEOMETRY");
  EXPECT_EQ(EvidenceLevel::name_of(9), "UNKNOWN_EVIDENCE_9");
}

// ---------------------------------------------------------------------------
// 6D 位姿保真（需求 5 硬约束）
// ---------------------------------------------------------------------------

TEST(CorePose, FullSixDofIsPreserved)
{
  const Pose6D pose = make_usable_pose("tool0");
  EXPECT_EQ(pose.position.x, 0.4);
  EXPECT_EQ(pose.position.y, -0.1);
  EXPECT_EQ(pose.position.z, 0.25);
  // 姿态不得被丢弃
  EXPECT_EQ(pose.orientation.x, 0.0);
  EXPECT_EQ(pose.orientation.y, 0.0);
  EXPECT_EQ(pose.orientation.z, 0.0);
  EXPECT_EQ(pose.orientation.w, 1.0);
  // frame_id 与时间戳必须保留
  EXPECT_EQ(pose.frame_id, "tool0");
  EXPECT_EQ(pose.stamp_s, 12.5);
  EXPECT_TRUE(pose.is_usable());
}

TEST(CorePose, MissingFrameIdIsNotUsable)
{
  // frame_id 为空 ⇒ 无法确认坐标系 ⇒ 禁止作为运动目标
  const Pose6D pose = make_usable_pose("");
  EXPECT_FALSE(pose.is_usable());
}

TEST(CorePose, UnnormalizedQuaternionIsNotUsable)
{
  Pose6D pose = make_usable_pose();
  pose.orientation.w = 3.0;
  EXPECT_FALSE(pose.orientation.is_normalized());
  EXPECT_FALSE(pose.is_usable());
}

TEST(CorePose, InvalidFlagBlocksUsability)
{
  Pose6D pose = make_usable_pose();
  pose.valid = false;
  EXPECT_FALSE(pose.is_usable());
}

TEST(CorePose, PoseEstimateRequiresBothFlagAndUsablePose)
{
  PoseEstimate estimate;
  estimate.object_id = "battery_1";
  estimate.color = 1;
  estimate.has_ring_pose_value = true;
  estimate.ring_pose = make_usable_pose("base_link");
  estimate.ring_pose_valid = true;
  EXPECT_TRUE(estimate.has_ring_pose());

  // BatteryDetection.ring_pose_valid=false ⇒ 禁止用 ring_pose 生成运动目标
  estimate.ring_pose_valid = false;
  EXPECT_FALSE(estimate.has_ring_pose());

  // 位姿本身不可用（frame_id 缺失）同样禁止使用
  estimate.ring_pose_valid = true;
  estimate.ring_pose = make_usable_pose("");
  EXPECT_FALSE(estimate.has_ring_pose());
}

// ---------------------------------------------------------------------------
// 端口结果契约
// ---------------------------------------------------------------------------

TEST(CoreResults, DetectionOkRequiresAllConditions)
{
  DetectionResult result;
  result.found = true;
  result.unique = true;
  result.stale = false;
  PoseEstimate pose;
  pose.object_id = "battery_1";
  pose.has_ring_pose_value = true;
  pose.ring_pose = make_usable_pose();
  result.pose = pose;
  EXPECT_TRUE(result.ok());

  result.unique = false;
  EXPECT_FALSE(result.ok());
  result.unique = true;
  result.stale = true;
  EXPECT_FALSE(result.ok());
  result.stale = false;
  result.found = false;
  EXPECT_FALSE(result.ok());
}

TEST(CoreResults, MotionResultRequiresBothSuccessAndArrived)
{
  MotionResult result;
  result.success = true;
  result.arrived = false;
  // 发令成功 != 执行到位
  EXPECT_FALSE(result.ok());
  result.arrived = true;
  EXPECT_TRUE(result.ok());
}

TEST(CoreResults, StopResultSeparatesDeliveryFromConfirmation)
{
  mtc::core::StopResult stop;
  stop.request_delivered = true;
  stop.stop_confirmed = false;
  // 送达不等于停稳：两者是独立维度
  EXPECT_TRUE(stop.request_delivered);
  EXPECT_FALSE(stop.stop_confirmed);
}

// ---------------------------------------------------------------------------
// 运动命令与原语
// ---------------------------------------------------------------------------

TEST(CoreMotionCommand, ShapeValidation)
{
  MotionCommand command;
  command.command_id = "req-1#pre_align-001";
  command.joint_names = {"shoulder_joint", "upperArm_joint", "foreArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint"};
  command.target_rad = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  EXPECT_TRUE(command.shape_is_valid());

  // 名称与目标数量不一致
  MotionCommand mismatched = command;
  mismatched.target_rad.pop_back();
  EXPECT_FALSE(mismatched.shape_is_valid());

  // 重复关节名
  MotionCommand duplicated = command;
  duplicated.joint_names[1] = duplicated.joint_names[0];
  EXPECT_FALSE(duplicated.shape_is_valid());

  // 空
  MotionCommand empty;
  EXPECT_FALSE(empty.shape_is_valid());
}

TEST(CoreMotionPrimitive, EqualityIncludesContactFlag)
{
  MotionPrimitive a;
  a.name = "seat_contact";
  a.requires_contact = true;
  MotionPrimitive b = a;
  EXPECT_TRUE(a == b);
  b.requires_contact = false;
  EXPECT_FALSE(a == b);
}

// ---------------------------------------------------------------------------
// 时间参数化轨迹（缺口 G1 的能力载体）
// ---------------------------------------------------------------------------

TEST(CoreTimedTrajectory, ShapeValidationAndDuration)
{
  TimedTrajectory trajectory;
  trajectory.trajectory_id = "traj-1";
  trajectory.joint_names = {"shoulder_joint", "upperArm_joint"};
  trajectory.header_frame_id = "base_link";

  TrajectoryPoint p1;
  p1.time_from_start_s = 0.0;
  p1.positions_rad = {0.0, 0.0};
  TrajectoryPoint p2;
  p2.time_from_start_s = 0.5;
  p2.positions_rad = {0.05, -0.02};
  p2.velocities_rad_s = {0.1, -0.04};
  trajectory.points = {p1, p2};

  EXPECT_TRUE(trajectory.shape_is_valid());
  EXPECT_DOUBLE_EQ(trajectory.duration_s(), 0.5);
}

TEST(CoreTimedTrajectory, RejectsNonMonotonicTime)
{
  TimedTrajectory trajectory;
  trajectory.joint_names = {"j1"};
  TrajectoryPoint p1;
  p1.time_from_start_s = 1.0;
  p1.positions_rad = {0.0};
  TrajectoryPoint p2;
  p2.time_from_start_s = 0.5;   // 时间回退
  p2.positions_rad = {0.1};
  trajectory.points = {p1, p2};
  EXPECT_FALSE(trajectory.shape_is_valid());
}

TEST(CoreTimedTrajectory, RejectsDimensionMismatch)
{
  TimedTrajectory trajectory;
  trajectory.joint_names = {"j1", "j2"};
  TrajectoryPoint p1;
  p1.time_from_start_s = 0.1;
  p1.positions_rad = {0.0};   // 维度不足
  trajectory.points = {p1};
  EXPECT_FALSE(trajectory.shape_is_valid());
}

TEST(CoreTimedTrajectory, RejectsEmptyOrDuplicateJointNames)
{
  TimedTrajectory trajectory;
  TrajectoryPoint p1;
  p1.time_from_start_s = 0.1;
  p1.positions_rad = {0.0};
  trajectory.points = {p1};

  trajectory.joint_names = {};
  EXPECT_FALSE(trajectory.shape_is_valid());

  trajectory.joint_names = {"j1", "j1"};
  p1.positions_rad = {0.0, 0.0};
  trajectory.points = {p1};
  EXPECT_FALSE(trajectory.shape_is_valid());
}

// ---------------------------------------------------------------------------
// 时钟与执行状态
// ---------------------------------------------------------------------------

TEST(CoreClock, ManualClockIsDeterministic)
{
  ManualClock clock(100.0);
  EXPECT_DOUBLE_EQ(clock.now(), 100.0);
  clock.advance(2.5);
  EXPECT_DOUBLE_EQ(clock.now(), 102.5);
  // 负推进被忽略，保证单调
  clock.advance(-5.0);
  EXPECT_DOUBLE_EQ(clock.now(), 102.5);
}

TEST(CoreExecState, StringMappingIsStable)
{
  // 这些字符串直接写入 ExecuteJointMove.action 的 feedback.execution_state
  EXPECT_STREQ(mtc::core::to_string(ExecState::IDLE), "IDLE");
  EXPECT_STREQ(mtc::core::to_string(ExecState::MOVING), "MOVING");
  EXPECT_STREQ(mtc::core::to_string(ExecState::SETTLED), "SETTLED");
  EXPECT_STREQ(mtc::core::to_string(ExecState::CANCELLING), "CANCELLING");
  EXPECT_STREQ(mtc::core::to_string(ExecState::STOPPED), "STOPPED");
  EXPECT_STREQ(mtc::core::to_string(ExecState::STOP_UNKNOWN), "STOP_UNKNOWN");
  EXPECT_STREQ(mtc::core::to_string(ExecState::TIMEOUT), "TIMEOUT");
  EXPECT_STREQ(mtc::core::to_string(ExecState::REJECTED), "REJECTED");
  EXPECT_STREQ(mtc::core::to_string(ExecState::FAULT), "FAULT");
}

TEST(CoreAudit, PoseRejectsNonFinitePositionStampAndQuaternion)
{
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
    auto pose = make_usable_pose();
    pose.position.x = bad;
    EXPECT_FALSE(pose.is_usable());
    pose = make_usable_pose();
    pose.stamp_s = bad;
    EXPECT_FALSE(pose.is_usable());
    pose = make_usable_pose();
    pose.orientation.w = bad;
    EXPECT_FALSE(pose.is_usable());
  }
  auto pose = make_usable_pose();
  pose.stamp_s = 0.0; // Valid epoch zero in an explicitly injected clock domain.
  EXPECT_TRUE(pose.is_usable());
  pose.stamp_s = -0.01;
  EXPECT_FALSE(pose.is_usable());
  pose = make_usable_pose();
  pose.orientation = {0, 0, 0, 0};
  EXPECT_FALSE(pose.is_usable());
}

TEST(CoreAudit, TrajectoryRejectsNonFiniteFieldsAndNegativeTimes)
{
  TimedTrajectory valid;
  valid.joint_names = {"j1"};
  valid.points = {{0.0, {0.1}, {0.1}, {0.1}}, {1.0, {0.2}, {}, {}}};
  EXPECT_TRUE(valid.shape_is_valid());
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()}) {
    auto t = valid;
    t.points[0].time_from_start_s = bad;
    EXPECT_FALSE(t.shape_is_valid());
    t = valid;
    t.stamp_s = bad;
    EXPECT_FALSE(t.shape_is_valid());
    t = valid;
    t.points[0].positions_rad[0] = bad;
    EXPECT_FALSE(t.shape_is_valid());
    t = valid;
    t.points[0].velocities_rad_s[0] = bad;
    EXPECT_FALSE(t.shape_is_valid());
    t = valid;
    t.points[0].accelerations_rad_s2[0] = bad;
    EXPECT_FALSE(t.shape_is_valid());
  }
  valid.points[0].time_from_start_s = -0.1;
  EXPECT_FALSE(valid.shape_is_valid());
}
