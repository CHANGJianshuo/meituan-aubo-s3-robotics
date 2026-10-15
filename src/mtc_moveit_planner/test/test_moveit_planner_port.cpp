// Copyright 2026 meituan_challenge_ws contributors
//
// MoveItPlannerPort 组装逻辑测试（假后端，无需 MoveIt / move_group）。
// 原语列表由已合入的 ToolStrategy 生成，与 PickPlaceFsm 实际请求一致。

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fake_backend.hpp"
#include "mtc_core_cpp/clock.hpp"
#include "mtc_core_cpp/error_codes.hpp"
#include "mtc_manipulation_cpp/pick_place_types.hpp"
#include "mtc_manipulation_cpp/tool_strategy.hpp"
#include "mtc_moveit_planner/moveit_planner_port.hpp"

namespace
{

using mtc::core::ErrorCodes;
using mtc::core::ManualClock;
using mtc::core::PlanResult;
using mtc::manipulation::PickPlaceConfig;
using mtc::manipulation::PickPlaceState;
using mtc::manipulation::PlanTarget;
using mtc::manipulation::TOOL_MAGNETIC_LATCH_V2;
using mtc::manipulation::TOOL_PASSIVE_HOOK_V1;
using mtc::moveit_planner::CalibrationStatus;
using mtc::moveit_planner::MoveItPlannerPort;
using mtc::moveit_planner::PlannerConfig;
using mtc::moveit_planner::default_planner_config;
using mtc::moveit_planner::testing::FakeBackend;

constexpr double kTol = 1e-9;

mtc::core::PoseEstimate ring_estimate(const std::string & frame = "base_link", bool valid = true)
{
  mtc::core::PoseEstimate estimate;
  estimate.object_id = "battery_1";
  estimate.color = 1;
  estimate.has_ring_pose_value = true;
  estimate.ring_pose.position = mtc::core::Vector3{0.4, -0.1, 0.25};
  estimate.ring_pose.frame_id = frame;
  estimate.ring_pose.stamp_s = 1.0;
  estimate.ring_pose.valid = valid;
  estimate.ring_pose_valid = valid;
  estimate.stamp_s = 1.0;
  return estimate;
}

PlanTarget acquire_target(const std::string & tool, const mtc::core::PoseEstimate & ring)
{
  const auto strategy = mtc::manipulation::make_strategy(tool);
  PlanTarget target;
  target.phase = "ACQUIRE";
  target.object_id = ring.object_id;
  target.slot = "T0";
  if (ring.has_ring_pose()) {
    target.has_ring_pose = true;
    target.ring_pose = ring.ring_pose;
  }
  target.primitives = strategy->make_acquire_plan(ring);
  const auto extra = strategy->make_test_lift_plan(ring);
  target.primitives.insert(target.primitives.end(), extra.begin(), extra.end());
  for (const auto & p : target.primitives) {
    target.primitive_names.push_back(p.name);
  }
  return target;
}

PlanTarget transport_target(const std::string & tool, const std::string & slot)
{
  const auto strategy = mtc::manipulation::make_strategy(tool);
  const auto ring = ring_estimate();
  PlanTarget target;
  target.phase = "TRANSPORT";
  target.object_id = ring.object_id;
  target.slot = slot;
  target.has_ring_pose = true;
  target.ring_pose = ring.ring_pose;
  target.primitives = strategy->make_lift_plan(ring);
  for (const auto & extra : {
      strategy->make_transport_plan(ring, slot), strategy->make_seat_plan(slot),
      strategy->make_seat_contact_plan(slot), strategy->make_retreat_plan(slot)})
  {
    target.primitives.insert(target.primitives.end(), extra.begin(), extra.end());
  }
  for (const auto & p : target.primitives) {
    target.primitive_names.push_back(p.name);
  }
  return target;
}

struct Fixture
{
  std::shared_ptr<FakeBackend> backend = std::make_shared<FakeBackend>();
  std::shared_ptr<ManualClock> clock = std::make_shared<ManualClock>(10.0);
  PlannerConfig config = default_planner_config();

  std::unique_ptr<MoveItPlannerPort> port(bool asynchronous = false)
  {
    return std::make_unique<MoveItPlannerPort>(backend, config, clock, asynchronous);
  }
};

/// FSM 中 shoulder/upperArm/foreArm 的索引（与 PickPlaceConfig::joint_names 一致）。
double x_of(const mtc::core::TrajectoryPoint & p) {return p.positions_rad[0];}
double y_of(const mtc::core::TrajectoryPoint & p) {return p.positions_rad[1];}
double z_of(const mtc::core::TrajectoryPoint & p) {return p.positions_rad[2];}

void expect_fsm_compatible(const PlanResult & result, const PlanTarget & target)
{
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_EQ(result.commands.size(), target.primitives.size());
  const PickPlaceConfig fsm;
  for (size_t i = 0; i < result.commands.size(); ++i) {
    const auto & c = result.commands[i];
    EXPECT_EQ(c.primitive_name, target.primitives[i].name);
    ASSERT_TRUE(c.trajectory.has_value());
    EXPECT_TRUE(c.target_rad.empty());
    EXPECT_TRUE(c.trajectory->shape_is_valid());
    EXPECT_EQ(c.trajectory->joint_names, fsm.joint_names);
    EXPECT_EQ(c.trajectory->header_frame_id, "world");
    EXPECT_GT(c.velocity_scaling, 0.0);
    EXPECT_LE(c.velocity_scaling, fsm.default_velocity_scaling);
    EXPECT_GT(c.acceleration_scaling, 0.0);
    EXPECT_LE(c.acceleration_scaling, fsm.default_acceleration_scaling);
    EXPECT_GE(static_cast<double>(c.timeout_ms), c.trajectory->duration_s() * 1000.0);
    EXPECT_TRUE(c.simulation_only);
    ASSERT_TRUE(c.intent.has_value());
    EXPECT_EQ(c.intent->name, target.primitives[i].name);
  }
}

TEST(MoveItPlannerPort, V1AcquireReturnsOneFullTrajectoryPerPrimitive)
{
  Fixture f;
  const auto target = acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate());
  ASSERT_EQ(target.primitive_names, (std::vector<std::string>{"side_insert", "test_lift"}));
  const auto result = f.port()->plan_blocking(target);
  expect_fsm_compatible(result, target);
  EXPECT_EQ(result.error_code, ErrorCodes::OK);
  EXPECT_NE(result.message.find("simulation_only=true"), std::string::npos);
  EXPECT_NE(result.message.find("TCP=PLACEHOLDER"), std::string::npos);
}

TEST(MoveItPlannerPort, AcquireGeometryStandoffEngageGapAndTestLift)
{
  Fixture f;
  const auto result = f.port()->plan_blocking(acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate()));
  ASSERT_TRUE(result.ok) << result.message;
  // 预对准：提环 (0.4,-0.1,0.25) 沿插入轴 +Z 退 0.03（TCP 为单位变换）。
  const auto & pre_align = result.commands[0].trajectory->points;
  EXPECT_NEAR(x_of(pre_align.front()), 0.30, kTol);  // 从当前状态出发
  EXPECT_NEAR(x_of(pre_align.back()), 0.40, kTol);
  EXPECT_NEAR(y_of(pre_align.back()), -0.10, kTol);
  EXPECT_NEAR(z_of(pre_align.back()), 0.22, kTol);
  // 试提：起点假设 ToolPort 插入 0.03 后的位置，再上提 0.01。
  const auto & test_lift = result.commands[1].trajectory->points;
  EXPECT_NEAR(z_of(test_lift.front()), 0.25, kTol);
  EXPECT_NEAR(z_of(test_lift.back()), 0.26, kTol);

  ASSERT_EQ(f.backend->requests.size(), 3u);
  EXPECT_EQ(f.backend->requests[0].label, "side_insert");
  EXPECT_EQ(f.backend->requests[1].label, "simulated_ENGAGE_INSERT");
  EXPECT_EQ(f.backend->requests[2].label, "test_lift");
  EXPECT_EQ(f.backend->requests[0].mode, mtc::moveit_planner::MotionMode::FREE_SPACE);
  EXPECT_EQ(f.backend->requests[2].mode, mtc::moveit_planner::MotionMode::CARTESIAN);
  // 原语速度上限原样下传给后端做时间参数化。
  EXPECT_DOUBLE_EQ(f.backend->requests[2].max_linear_speed_m_s, 0.01);
}

TEST(MoveItPlannerPort, V2TopInsertIsPlanned)
{
  Fixture f;
  const auto target = acquire_target(TOOL_MAGNETIC_LATCH_V2, ring_estimate());
  ASSERT_EQ(target.primitive_names, (std::vector<std::string>{"top_insert", "test_lift"}));
  expect_fsm_compatible(f.port()->plan_blocking(target), target);
}

TEST(MoveItPlannerPort, TransportPhaseUsesSlotAndChainsStates)
{
  Fixture f;
  const auto target = transport_target(TOOL_PASSIVE_HOOK_V1, "T0");
  ASSERT_EQ(target.primitive_names, (std::vector<std::string>{
      "lift", "transport", "pre_seat", "seat_contact", "retreat"}));
  // 搬运阶段在 LIFT 状态规划：起点 = 试提结束后的当前位置。
  f.backend->start_xyz = mtc::core::Vector3{0.4, -0.1, 0.26};
  auto port = f.port();
  const auto result = port->plan_blocking(target);
  expect_fsm_compatible(result, target);

  const auto end = [&](size_t i) {return result.commands[i].trajectory->points.back();};
  const auto begin = [&](size_t i) {return result.commands[i].trajectory->points.front();};
  EXPECT_NEAR(z_of(end(0)), 0.31, kTol);                       // lift +0.05
  EXPECT_NEAR(x_of(end(1)), 0.35, kTol);                       // transport -> T0 上方
  EXPECT_NEAR(y_of(end(1)), -0.20, kTol);
  EXPECT_NEAR(z_of(end(1)), 0.23, kTol);                       // 0.13 + 0.10
  EXPECT_NEAR(z_of(end(2)), 0.15, kTol);                       // pre_seat 余量 0.02
  EXPECT_NEAR(z_of(end(3)), 0.13, kTol);                       // seat_contact 到工位高度
  EXPECT_NEAR(z_of(begin(4)), 0.10, kTol);                     // 假设退钩 0.03
  EXPECT_NEAR(z_of(end(4)), 0.20, kTol);                       // retreat +0.10
  for (size_t i = 1; i < 4; ++i) {
    EXPECT_EQ(begin(i).positions_rad, end(i - 1).positions_rad) << "primitive " << i;
  }
  const auto report = port->last_report();
  ASSERT_EQ(report.primitives.size(), 5u);
  EXPECT_TRUE(report.primitives[3].contact_intent_only);
  EXPECT_NE(report.primitives[4].start_assumption.find("WITHDRAW"), std::string::npos);
  EXPECT_NE(std::find(report.uncalibrated_inputs.begin(), report.uncalibrated_inputs.end(),
    "slot T0=PLACEHOLDER"), report.uncalibrated_inputs.end());
}

TEST(MoveItPlannerPort, PayloadAttachedFromTestLiftUntilRelease)
{
  Fixture f;
  auto acquire = f.port()->plan_blocking(acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate()));
  ASSERT_TRUE(acquire.ok) << acquire.message;
  // side_insert / 插入假设：无载荷；test_lift：挂上电池。
  ASSERT_EQ(f.backend->requests.size(), 3u);
  EXPECT_FALSE(f.backend->requests[0].payload.has_value());
  EXPECT_FALSE(f.backend->requests[1].payload.has_value());
  ASSERT_TRUE(f.backend->requests[2].payload.has_value());
  // 载荷中心在 TCP（单位 TCP 时即 ee_link）正下方 hang_below_tcp_m。
  const auto & payload = *f.backend->requests[2].payload;
  EXPECT_NEAR(payload.ee_to_payload.translation.x, 0.0, kTol);
  EXPECT_NEAR(payload.ee_to_payload.translation.y, 0.0, kTol);
  EXPECT_NEAR(payload.ee_to_payload.translation.z, -f.config.payload.hang_below_tcp_m, kTol);

  f.backend->requests.clear();
  f.backend->start_xyz = mtc::core::Vector3{0.4, -0.1, 0.26};
  auto transport = f.port()->plan_blocking(transport_target(TOOL_PASSIVE_HOOK_V1, "T0"));
  ASSERT_TRUE(transport.ok) << transport.message;
  // lift, transport, pre_seat, seat_contact 带载荷；退钩假设与 retreat 不带。
  ASSERT_EQ(f.backend->requests.size(), 6u);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_TRUE(f.backend->requests[i].payload.has_value()) << f.backend->requests[i].label;
  }
  EXPECT_EQ(f.backend->requests[4].label, "simulated_WITHDRAW");
  EXPECT_FALSE(f.backend->requests[4].payload.has_value());
  EXPECT_EQ(f.backend->requests[5].label, "retreat");
  EXPECT_FALSE(f.backend->requests[5].payload.has_value());
}

TEST(MoveItPlannerPort, PayloadCanBeDisabledButUnknownGeometryIsRefused)
{
  Fixture off;
  off.config.payload.enabled = false;
  ASSERT_TRUE(off.port()->plan_blocking(acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate())).ok);
  for (const auto & r : off.backend->requests) {
    EXPECT_FALSE(r.payload.has_value());
  }
  Fixture unknown;
  unknown.config.payload.status = CalibrationStatus::UNKNOWN;
  const auto result = unknown.port()->plan_blocking(
    acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate()));
  EXPECT_EQ(result.message.rfind("[PAYLOAD_UNKNOWN] test_lift", 0), 0u) << result.message;
}

TEST(MoveItPlannerPort, ReplanPrimitiveStartsAtMeasuredStateWithoutToolGapAssumption)
{
  Fixture f;
  const auto target = acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate());
  // 实测：ToolPort 接合后末端停在 (0.4, -0.1, 0.247)，而非假设的 0.25。
  const auto measured = f.backend->state_at(mtc::core::Vector3{0.4, -0.1, 0.247});
  const auto result = f.port()->replan_primitive(target, "test_lift", measured);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_EQ(result.commands.size(), 1u);
  EXPECT_EQ(result.commands[0].primitive_name, "test_lift");
  const auto & points = result.commands[0].trajectory->points;
  EXPECT_NEAR(z_of(points.front()), 0.247, kTol);
  EXPECT_NEAR(z_of(points.back()), 0.257, kTol);
  ASSERT_EQ(f.backend->requests.size(), 1u);  // 没有再做插入假设
  EXPECT_EQ(f.backend->requests[0].label, "test_lift");

  const auto missing = f.port()->replan_primitive(target, "no_such_primitive", measured);
  EXPECT_FALSE(missing.ok);
  EXPECT_EQ(missing.message.rfind("[EMPTY_TARGET]", 0), 0u) << missing.message;
}

TEST(MoveItPlannerPort, ReplanFromStateWithDuplicateJointNamesIsRefused)
{
  Fixture f;
  const auto target = acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate());
  auto measured = f.backend->state_at(mtc::core::Vector3{0.4, -0.1, 0.247});
  measured.names.push_back(measured.names.front());
  measured.positions.push_back(measured.positions.front() + 0.5);
  const auto result = f.port()->replan_primitive(target, "test_lift", measured);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error_code, ErrorCodes::PLANNING_FAILED);
  EXPECT_EQ(result.message.rfind("[INVALID_START_STATE]", 0), 0u) << result.message;
  EXPECT_EQ(f.backend->plan_calls.load(), 0);
}

TEST(MoveItPlannerPort, AnyPrimitiveFailureFailsWholePlan)
{
  Fixture f;
  f.backend->fail_labels = {"transport"};
  const auto result = f.port()->plan_blocking(transport_target(TOOL_PASSIVE_HOOK_V1, "T0"));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error_code, ErrorCodes::PLANNING_FAILED);
  EXPECT_TRUE(result.commands.empty());
  EXPECT_EQ(result.message.rfind("[NO_PLAN] transport", 0), 0u) << result.message;
}

TEST(MoveItPlannerPort, ToolOwnedPrimitiveIsRefused)
{
  Fixture f;
  auto target = acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate());
  const auto engage = mtc::manipulation::make_strategy(TOOL_PASSIVE_HOOK_V1)->make_engage_plan(
    ring_estimate());
  target.primitives = engage;
  const auto result = f.port()->plan_blocking(target);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.message.rfind("[TOOL_OWNED_PRIMITIVE] hook_engage", 0), 0u) << result.message;
  EXPECT_EQ(f.backend->plan_calls.load(), 0);
}

TEST(MoveItPlannerPort, UnknownSlotIsRefused)
{
  Fixture f;
  f.backend->start_xyz = mtc::core::Vector3{0.4, -0.1, 0.26};
  f.config.slots["T0"].status = CalibrationStatus::UNKNOWN;
  const auto result = f.port()->plan_blocking(transport_target(TOOL_PASSIVE_HOOK_V1, "T0"));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.message.rfind("[SLOT_POSE_UNKNOWN] transport", 0), 0u) << result.message;
}

TEST(MoveItPlannerPort, UntransformableFrameIsRefusedWithoutCopyingXyz)
{
  Fixture f;
  const auto result = f.port()->plan_blocking(
    acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate("camera_color_optical_frame")));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.message.rfind("[TF_UNAVAILABLE] side_insert", 0), 0u) << result.message;
  EXPECT_EQ(f.backend->plan_calls.load(), 0);
}

TEST(MoveItPlannerPort, MissingRingPoseIsRefused)
{
  Fixture f;
  const auto result = f.port()->plan_blocking(
    acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate("base_link", false)));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.message.rfind("[TARGET_POSE_MISSING] side_insert", 0), 0u) << result.message;
}

TEST(MoveItPlannerPort, NoRobotStateIsRefused)
{
  Fixture f;
  f.backend->has_current_state = false;
  const auto result = f.port()->plan_blocking(acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate()));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.message.rfind("[NO_CURRENT_STATE]", 0), 0u) << result.message;
}

TEST(MoveItPlannerPort, TrajectoryLongerThanStateWatchdogIsRefused)
{
  Fixture f;
  f.config.state_timeouts[PickPlaceState::MOVE_PRE_ALIGN] = 5.0;
  const auto result = f.port()->plan_blocking(acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate()));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.message.rfind("[EXCEEDS_STATE_WATCHDOG] side_insert", 0), 0u)
    << result.message;
}

/// 拆分提案：快速接近（side_approach）+ 慢速预对准（side_insert），两者都在 MOVE_PRE_ALIGN 执行。
PlanTarget split_acquire_target()
{
  auto target = acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate());
  auto approach = target.primitives.front();
  approach.name = "side_approach";
  approach.max_linear_speed = 0.15;
  approach.max_angular_speed = 0.3;
  target.primitives.insert(target.primitives.begin(), approach);
  target.primitive_names.insert(target.primitive_names.begin(), approach.name);
  return target;
}

TEST(MoveItPlannerPort, SplitApproachStopsShortOfStandoffAndKeepsSlowFinalSegment)
{
  Fixture f;
  const auto result = f.port()->plan_blocking(split_acquire_target());
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_EQ(result.commands.size(), 3u);
  EXPECT_EQ(result.commands[0].primitive_name, "side_approach");
  EXPECT_EQ(result.commands[1].primitive_name, "side_insert");
  // 接近段停在提环前 0.05 m（预对准 0.03 + 退让 0.02），预对准段只走最后 0.02 m。
  const auto & approach_end = result.commands[0].trajectory->points.back();
  const auto & insert_end = result.commands[1].trajectory->points.back();
  EXPECT_NEAR(z_of(approach_end), 0.25 - 0.05, 1e-9);
  EXPECT_NEAR(z_of(insert_end), 0.25 - 0.03, 1e-9);
  ASSERT_GE(f.backend->requests.size(), 2u);
  EXPECT_DOUBLE_EQ(f.backend->requests[0].max_linear_speed_m_s, 0.15);
  EXPECT_DOUBLE_EQ(f.backend->requests[1].max_linear_speed_m_s, 0.02);
}

TEST(MoveItPlannerPort, PrimitivesSharingAStateShareItsWatchdog)
{
  Fixture probe;
  probe.config.state_timeouts[PickPlaceState::MOVE_PRE_ALIGN] = 1000.0;
  auto port = probe.port();
  ASSERT_TRUE(port->plan_blocking(split_acquire_target()).ok);
  const double d1 = port->last_report().primitives[0].duration_s;
  const double d2 = port->last_report().primitives[1].duration_s;
  ASSERT_GT(d1, 0.0);
  ASSERT_GT(d2, 0.0);

  // 每段单独都在 80% 以内，但两段之和超过：必须拒绝。
  Fixture f;
  const double threshold = 0.5 * (std::max(d1, d2) + d1 + d2);
  f.config.state_timeouts[PickPlaceState::MOVE_PRE_ALIGN] = threshold / f.config.state_timeout_usage;
  const auto result = f.port()->plan_blocking(split_acquire_target());
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.message.rfind("[EXCEEDS_STATE_WATCHDOG] side_insert", 0), 0u) << result.message;
  EXPECT_NE(result.message.find("with earlier primitives in the same state"), std::string::npos)
    << result.message;
}

TEST(MoveItPlannerPort, StartMismatchAndMissingJointAreRefused)
{
  Fixture f;
  f.backend->start_elsewhere = true;
  auto result = f.port()->plan_blocking(acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate()));
  EXPECT_EQ(result.message.rfind("[START_STATE_MISMATCH]", 0), 0u) << result.message;

  Fixture g;
  g.backend->drop_last_joint = true;
  result = g.port()->plan_blocking(acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate()));
  EXPECT_EQ(result.message.rfind("[INVALID_TRAJECTORY]", 0), 0u) << result.message;
}

TEST(MoveItPlannerPort, TighterControllerJointLimitsAreEnforced)
{
  Fixture f;
  // 假后端 foreArm_joint = 末端 z；预对准终点 z=0.22 超过此限位。
  f.config.joint_position_limits["foreArm_joint"] = {-0.1, 0.2};
  const auto result = f.port()->plan_blocking(acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate()));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.message.rfind("[JOINT_LIMIT_VIOLATION] side_insert", 0), 0u) << result.message;
}

TEST(MoveItPlannerPort, ToolModelSetsTcpAndAttachesBodiesToEveryRequest)
{
  Fixture f;
  auto & tool = f.config.tool;
  tool.enabled = true;
  tool.flange_axis_cad = {0.0, 1.0, 0.0};
  tool.insertion_cad = {0.0, 0.0, 1.0};
  tool.hook_up_cad = {0.0, -1.0, 0.0};
  tool.tcp_cad_m = {0.0, 0.110, 0.045};
  mtc::moveit_planner::ToolBodySpec tongue;
  tongue.id = "tongue";
  tongue.size_m = {0.044, 0.005, 0.085};
  tool.bodies = {tongue};
  f.config.tool_status = CalibrationStatus::ESTIMATED;
  auto port = f.port();
  EXPECT_NEAR(port->config().ee_to_tcp.translation.z, 0.110, kTol);
  EXPECT_EQ(port->config().tcp_status, CalibrationStatus::ESTIMATED);
  const auto result = port->plan_blocking(acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate()));
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_NE(result.message.find("tool_geometry=ESTIMATED"), std::string::npos) << result.message;
  ASSERT_FALSE(f.backend->requests.empty());
  for (const auto & request : f.backend->requests) {
    ASSERT_EQ(request.tool_bodies.size(), 1u) << request.label;
    EXPECT_EQ(request.tool_bodies[0].id, "mtc_moveit_planner/tongue");
  }

  Fixture bad;
  bad.config.tool = tool;
  bad.config.tool.insertion_cad = {0.0, 1.0, 0.0};   // 与法兰轴平行：非法
  const auto refused = bad.port()->plan_blocking(acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate()));
  EXPECT_EQ(refused.message.rfind("[INVALID_CONFIG]", 0), 0u) << refused.message;
}

TEST(MoveItPlannerPort, ScalingAboveExecutorCapIsRefused)
{
  Fixture f;
  f.config.velocity_scaling = 0.5;
  const auto result = f.port()->plan_blocking(acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate()));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.message.rfind("[INVALID_CONFIG]", 0), 0u) << result.message;
}

TEST(MoveItPlannerPort, CalibrationGate)
{
  Fixture strict;
  strict.config.allow_uncalibrated_geometry = false;
  auto result = strict.port()->plan_blocking(
    acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate()));
  EXPECT_EQ(result.message.rfind("[TCP_UNKNOWN] side_insert", 0), 0u) << result.message;

  Fixture measured;
  measured.config.tcp_status = CalibrationStatus::MEASURED;
  measured.config.payload.status = CalibrationStatus::MEASURED;
  measured.config.force_simulation_only = false;
  result = measured.port()->plan_blocking(acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate()));
  ASSERT_TRUE(result.ok) << result.message;
  for (const auto & c : result.commands) {
    EXPECT_FALSE(c.simulation_only);
  }

  // 只要载荷尺寸仍是占位值，输出就必须保持 simulation_only。
  Fixture payload_placeholder;
  payload_placeholder.config.tcp_status = CalibrationStatus::MEASURED;
  payload_placeholder.config.force_simulation_only = false;
  result = payload_placeholder.port()->plan_blocking(
    acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate()));
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_TRUE(result.commands.back().simulation_only);
  EXPECT_NE(result.message.find("payload=PLACEHOLDER"), std::string::npos) << result.message;
}

TEST(MoveItPlannerPort, AsyncReturnsNulloptUntilDoneAndNeverDuplicatesWork)
{
  Fixture f;
  f.backend->delay_ms = 20;
  auto port = f.port(true);
  const auto target = acquire_target(TOOL_PASSIVE_HOOK_V1, ring_estimate());
  EXPECT_FALSE(port->plan(target).has_value());
  std::optional<PlanResult> result;
  for (int i = 0; i < 400 && !result; ++i) {
    result = port->plan(target);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(result.has_value());
  expect_fsm_compatible(*result, target);
  EXPECT_EQ(f.backend->plan_calls.load(), 3);  // side_insert + 插入假设 + test_lift，只做一次
}

}  // namespace
