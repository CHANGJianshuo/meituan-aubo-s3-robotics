// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_moveit_planner/planner_config.hpp"

#include <cmath>

namespace mtc
{
namespace moveit_planner
{

using mtc::manipulation::PickPlaceState;

const char * to_string(MotionMode mode)
{
  switch (mode) {
    case MotionMode::FREE_SPACE: return "FREE_SPACE";
    case MotionMode::CARTESIAN: return "CARTESIAN";
  }
  return "UNKNOWN";
}

const char * to_string(CalibrationStatus status)
{
  switch (status) {
    case CalibrationStatus::MEASURED: return "MEASURED";
    case CalibrationStatus::ESTIMATED: return "ESTIMATED";
    case CalibrationStatus::PLACEHOLDER: return "PLACEHOLDER";
    case CalibrationStatus::UNKNOWN: return "UNKNOWN";
  }
  return "UNKNOWN";
}

std::optional<CalibrationStatus> calibration_status_from_string(const std::string & text)
{
  if (text == "MEASURED") {return CalibrationStatus::MEASURED;}
  if (text == "ESTIMATED") {return CalibrationStatus::ESTIMATED;}
  if (text == "PLACEHOLDER") {return CalibrationStatus::PLACEHOLDER;}
  if (text == "UNKNOWN") {return CalibrationStatus::UNKNOWN;}
  return std::nullopt;
}

const char * to_string(GoalKind kind)
{
  switch (kind) {
    case GoalKind::RING_STANDOFF: return "RING_STANDOFF";
    case GoalKind::RELATIVE_UP: return "RELATIVE_UP";
    case GoalKind::SLOT_OFFSET: return "SLOT_OFFSET";
  }
  return "UNKNOWN";
}

const char * to_string(ToolGapBefore gap)
{
  switch (gap) {
    case ToolGapBefore::NONE: return "NONE";
    case ToolGapBefore::ENGAGE_INSERT: return "ENGAGE_INSERT";
    case ToolGapBefore::WITHDRAW: return "WITHDRAW";
  }
  return "UNKNOWN";
}

namespace
{

PrimitiveSpec spec(
  MotionMode mode, GoalKind goal, double distance_m, PickPlaceState state,
  ToolGapBefore gap = ToolGapBefore::NONE, double gap_distance_m = 0.0,
  bool contact_intent_only = false)
{
  PrimitiveSpec out;
  out.mode = mode;
  out.goal = goal;
  out.distance_m = distance_m;
  out.executing_state = state;
  out.gap_before = gap;
  out.gap_distance_m = gap_distance_m;
  out.contact_intent_only = contact_intent_only;
  return out;
}

SlotSpec placeholder_slot(double x, double y, double z)
{
  SlotSpec slot;
  slot.pose.position = mtc::core::Vector3{x, y, z};
  slot.pose.orientation = mtc::core::Quaternion{0.0, 0.0, 0.0, 1.0};
  slot.pose.frame_id = "base_link";
  slot.pose.stamp_s = 0.0;
  slot.pose.valid = true;
  slot.status = CalibrationStatus::PLACEHOLDER;
  return slot;
}

}  // namespace

PlannerConfig default_planner_config()
{
  PlannerConfig config;

  // 距离取自 config/motion_profiles.yaml（全部为占位值）：
  //   engage.distance_m=0.03（预对准退让 = 插入行程）、test_lift 0.01、lift 0.05、
  //   seat.distance_m=0.02（落座前余量）、disengage 0.03、retreat 0.10；
  //   config/manipulation.yaml slot_approach_height_m=0.10。
  constexpr double kInsertTravel = 0.03;
  constexpr double kTestLift = 0.01;
  constexpr double kLift = 0.05;
  constexpr double kSlotApproachHeight = 0.10;
  constexpr double kPreSeatClearance = 0.02;
  constexpr double kWithdrawTravel = 0.03;
  constexpr double kRetreat = 0.10;
  // "快速接近 + 慢速预对准"拆分提案中，快速段停在预对准位之前再退 2 cm（占位；V1 占位几何下
  // 2 cm 可达、5 cm 不可达，见 docs/planning/V1_FEASIBILITY.md）。
  constexpr double kApproachBackoff = 0.02;

  auto & p = config.primitives;
  // 抓取侧（PLAN_ACQUIRE 一次规划 acquire + test_lift）
  p["side_insert"] = spec(MotionMode::FREE_SPACE, GoalKind::RING_STANDOFF, kInsertTravel,
      PickPlaceState::MOVE_PRE_ALIGN);
  p["top_insert"] = spec(MotionMode::FREE_SPACE, GoalKind::RING_STANDOFF, kInsertTravel,
      PickPlaceState::MOVE_PRE_ALIGN);
  // 拆分提案的快速接近段（FSM 尚未发出这些原语；未发出时这两条规则不起作用）。
  p["side_approach"] = spec(MotionMode::FREE_SPACE, GoalKind::RING_STANDOFF,
      kInsertTravel + kApproachBackoff, PickPlaceState::MOVE_PRE_ALIGN);
  p["top_approach"] = spec(MotionMode::FREE_SPACE, GoalKind::RING_STANDOFF,
      kInsertTravel + kApproachBackoff, PickPlaceState::MOVE_PRE_ALIGN);
  p["test_lift"] = spec(MotionMode::CARTESIAN, GoalKind::RELATIVE_UP, kTestLift,
      PickPlaceState::TEST_LIFT, ToolGapBefore::ENGAGE_INSERT, kInsertTravel);
  // 搬运侧（LIFT 状态一次规划 lift + transport + pre_seat + seat_contact + retreat）
  p["lift"] = spec(MotionMode::CARTESIAN, GoalKind::RELATIVE_UP, kLift, PickPlaceState::LIFT);
  p["transport"] = spec(MotionMode::FREE_SPACE, GoalKind::SLOT_OFFSET, kSlotApproachHeight,
      PickPlaceState::TRANSPORT);
  p["pre_seat"] = spec(MotionMode::CARTESIAN, GoalKind::SLOT_OFFSET, kPreSeatClearance,
      PickPlaceState::MOVE_PRE_SEAT);
  p["seat_contact"] = spec(MotionMode::CARTESIAN, GoalKind::SLOT_OFFSET, 0.0,
      PickPlaceState::SEAT, ToolGapBefore::NONE, 0.0, true);
  // 试提之后、释放之前，末端挂着电池。
  for (const char * name : {"test_lift", "lift", "transport", "pre_seat", "seat_contact"}) {
    p[name].carries_payload = true;
  }
  p["retreat"] = spec(MotionMode::CARTESIAN, GoalKind::RELATIVE_UP, kRetreat,
      PickPlaceState::RETREAT, ToolGapBefore::WITHDRAW, kWithdrawTravel);

  // 由 ToolPort 执行的接触动作：规划器拒绝为其输出轨迹。
  config.tool_owned_primitives = {"hook_engage", "auto_lock", "unload", "side_withdraw", "withdraw"};

  // aubo_description/urdf/xacro/inc/aubo.ros2_control.xacro 的 command_interface min/max。
  const double two_pi = 2.0 * M_PI;
  config.joint_position_limits = {
    {"shoulder_joint", {-two_pi, two_pi}},
    {"upperArm_joint", {-two_pi, two_pi}},
    {"foreArm_joint", {-2.41, 2.41}},
    {"wrist1_joint", {-2.66, 2.66}},
    {"wrist2_joint", {-two_pi, two_pi}},
    {"wrist3_joint", {-2.23, 2.23}},
  };

  // 占位工位（base_link，单位 m）：只为 Mock 规划可达，非实测，禁止用于真实运动。
  config.slots["T0"] = placeholder_slot(0.35, -0.20, 0.13);
  config.slots["P1"] = placeholder_slot(0.35, -0.05, 0.13);
  config.slots["P2"] = placeholder_slot(0.35, 0.10, 0.13);
  config.slots["P3"] = placeholder_slot(0.35, 0.25, 0.13);
  return config;
}

}  // namespace moveit_planner
}  // namespace mtc

namespace mtc
{
namespace moveit_planner
{

void apply_tool_travel(
  PlannerConfig & config, double insert_travel_m, double withdraw_travel_m,
  double approach_backoff_m)
{
  auto & p = config.primitives;
  for (const char * name : {"side_insert", "top_insert"}) {
    if (p.count(name)) {p[name].distance_m = insert_travel_m;}
  }
  for (const char * name : {"side_approach", "top_approach"}) {
    if (p.count(name)) {p[name].distance_m = insert_travel_m + approach_backoff_m;}
  }
  if (p.count("test_lift")) {p["test_lift"].gap_distance_m = insert_travel_m;}
  if (p.count("retreat")) {p["retreat"].gap_distance_m = withdraw_travel_m;}
}

}  // namespace moveit_planner
}  // namespace mtc
