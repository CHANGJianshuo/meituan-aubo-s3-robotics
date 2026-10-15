// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_manipulation_cpp/tool_strategy.hpp"

#include <stdexcept>

#include "mtc_core_cpp/error_codes.hpp"

namespace mtc
{
namespace manipulation
{

namespace
{

/// 由观测构造原语，并保留完整 6D 位姿与 frame_id。
mtc::core::MotionPrimitive make_primitive(
  const std::string & name,
  const mtc::core::PoseEstimate & pose,
  double linear_speed,
  double angular_speed,
  bool requires_contact)
{
  mtc::core::MotionPrimitive primitive;
  primitive.name = name;
  primitive.target_label = pose.object_id;
  if (pose.has_ring_pose()) {
    primitive.target_pose = pose.ring_pose;
  }
  primitive.max_linear_speed = linear_speed;
  primitive.max_angular_speed = angular_speed;
  primitive.requires_contact = requires_contact;
  return primitive;
}

/// 由工位名构造原语（落座/撤离阶段的目标由工位标定给出，不由视觉给出）。
mtc::core::MotionPrimitive make_slot_primitive(
  const std::string & name,
  const std::string & target_slot,
  double linear_speed,
  double angular_speed,
  bool requires_contact)
{
  mtc::core::MotionPrimitive primitive;
  primitive.name = name;
  primitive.target_label = target_slot;
  primitive.max_linear_speed = linear_speed;
  primitive.max_angular_speed = angular_speed;
  primitive.requires_contact = requires_contact;
  return primitive;
}

}  // namespace

// ---------------------------------------------------------------------------
// 通用实现（各代共用）
// ---------------------------------------------------------------------------

mtc::core::MotionPrimitiveList ToolStrategy::make_test_lift_plan(
  const mtc::core::PoseEstimate & ring_pose) const
{
  // 低速短距试提：所有工具都做，用于获取挂载证据。
  return {make_primitive("test_lift", ring_pose, 0.01, 0.05, false)};
}

mtc::core::MotionPrimitiveList ToolStrategy::make_lift_plan(
  const mtc::core::PoseEstimate & ring_pose) const
{
  return {make_primitive("lift", ring_pose, 0.03, 0.1, false)};
}

mtc::core::MotionPrimitiveList ToolStrategy::make_transport_plan(
  const mtc::core::PoseEstimate & ring_pose, const std::string & target_slot) const
{
  mtc::core::MotionPrimitive primitive =
    make_primitive("transport", ring_pose, 0.15, 0.3, false);
  primitive.target_label = target_slot;
  return {primitive};
}

mtc::core::MotionPrimitiveList ToolStrategy::make_seat_plan(
  const std::string & target_slot) const
{
  // 落座前的预放置（自由空间下降）。
  return {make_slot_primitive("pre_seat", target_slot, 0.03, 0.1, false)};
}

mtc::core::MotionPrimitiveList ToolStrategy::make_seat_contact_plan(
  const std::string & target_slot) const
{
  // 受约束低速下降至获得台面承托；不得继续硬顶。
  return {make_slot_primitive(
    "seat_contact", target_slot, CONTACT_LINEAR_SPEED, 0.05, true)};
}

mtc::core::MotionPrimitiveList ToolStrategy::make_retreat_plan(
  const std::string & target_slot) const
{
  return {make_slot_primitive("retreat", target_slot, 0.03, 0.1, false)};
}

bool ToolStrategy::uses_io(const std::string & channel) const
{
  const auto channels = io_channels();
  for (const auto & item : channels) {
    if (item == channel) {
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// V1：被动舌规
// ---------------------------------------------------------------------------

std::string PassiveHookV1Strategy::type() const
{
  return TOOL_PASSIVE_HOOK_V1;
}

mtc::core::MotionPrimitiveList PassiveHookV1Strategy::make_acquire_plan(
  const mtc::core::PoseEstimate & ring_pose) const
{
  return {make_primitive(
    "side_insert", ring_pose, DEFAULT_LINEAR_SPEED, DEFAULT_ANGULAR_SPEED, false)};
}

mtc::core::MotionPrimitiveList PassiveHookV1Strategy::make_engage_plan(
  const mtc::core::PoseEstimate & ring_pose) const
{
  // 横向穿环 + 适当上提完成挂钩：接触敏感，低速。
  return {make_primitive("hook_engage", ring_pose, CONTACT_LINEAR_SPEED, 0.05, true)};
}

mtc::core::MotionPrimitiveList PassiveHookV1Strategy::make_release_plan(
  const std::string & target_slot) const
{
  return {make_slot_primitive("unload", target_slot, CONTACT_LINEAR_SPEED, 0.05, true)};
}

mtc::core::MotionPrimitiveList PassiveHookV1Strategy::make_disengage_plan(
  const std::string & target_slot) const
{
  // 沿标定的反向路径退出，倒钩不再承重后才能退钩。
  return {make_slot_primitive(
    "side_withdraw", target_slot, DEFAULT_LINEAR_SPEED, DEFAULT_ANGULAR_SPEED, true)};
}

bool PassiveHookV1Strategy::requires_unlock_pulse() const
{
  return false;
}

UnlockDecision PassiveHookV1Strategy::evaluate_unlock(
  bool seated_verified, bool unloaded_verified) const
{
  (void)seated_verified;
  (void)unloaded_verified;
  UnlockDecision decision;
  decision.allowed = false;
  decision.error_code = mtc::core::ErrorCodes::UNLOCK_FAILED;
  decision.reason = "V1 被动舌规无解锁机构，禁止任何解锁动作";
  return decision;
}

// ---------------------------------------------------------------------------
// V2：电磁锁止
// ---------------------------------------------------------------------------

MagneticLatchV2Strategy::MagneticLatchV2Strategy(uint32_t pulse_ms)
: pulse_ms_(pulse_ms)
{
}

std::string MagneticLatchV2Strategy::type() const
{
  return TOOL_MAGNETIC_LATCH_V2;
}

std::vector<std::string> MagneticLatchV2Strategy::io_channels() const
{
  return {"electromagnetic_unlock"};
}

mtc::core::MotionPrimitiveList MagneticLatchV2Strategy::make_acquire_plan(
  const mtc::core::PoseEstimate & ring_pose) const
{
  return {make_primitive(
    "top_insert", ring_pose, DEFAULT_LINEAR_SPEED, DEFAULT_ANGULAR_SPEED, false)};
}

mtc::core::MotionPrimitiveList MagneticLatchV2Strategy::make_engage_plan(
  const mtc::core::PoseEstimate & ring_pose) const
{
  // 从上方下降插入，触发机械锁止。
  return {make_primitive("auto_lock", ring_pose, CONTACT_LINEAR_SPEED, 0.05, true)};
}

mtc::core::MotionPrimitiveList MagneticLatchV2Strategy::make_release_plan(
  const std::string & target_slot) const
{
  // 小幅调整让锁扣不承重（不涉及电磁 IO）。
  return {make_slot_primitive("unload", target_slot, CONTACT_LINEAR_SPEED, 0.05, true)};
}

mtc::core::MotionPrimitiveList MagneticLatchV2Strategy::make_disengage_plan(
  const std::string & target_slot) const
{
  // 注意：不得假设一定能垂直向上退出，退出方向由 V2 CAD/实测确定。
  return {make_slot_primitive(
    "withdraw", target_slot, DEFAULT_LINEAR_SPEED, DEFAULT_ANGULAR_SPEED, true)};
}

bool MagneticLatchV2Strategy::requires_unlock_pulse() const
{
  return true;
}

UnlockDecision MagneticLatchV2Strategy::evaluate_unlock(
  bool seated_verified, bool unloaded_verified) const
{
  if (!seated_verified) {
    UnlockDecision decision;
    decision.allowed = false;
    decision.error_code = mtc::core::ErrorCodes::SEAT_NOT_VERIFIED;
    decision.reason = "未确认电池由桌面承载，禁止发出电磁解锁脉冲";
    return decision;
  }
  if (!unloaded_verified) {
    UnlockDecision decision;
    decision.allowed = false;
    decision.error_code = mtc::core::ErrorCodes::RELEASE_NOT_VERIFIED;
    decision.reason = "锁扣仍承担主要重量（未确认卸载），禁止解锁";
    return decision;
  }
  UnlockDecision decision;
  decision.allowed = true;
  decision.error_code = mtc::core::ErrorCodes::OK;
  decision.reason = "已确认落座且已卸载，允许解锁";
  decision.pulse_ms = pulse_ms_;
  return decision;
}

// ---------------------------------------------------------------------------
// 工厂
// ---------------------------------------------------------------------------

ToolStrategyPtr make_strategy(const std::string & tool_type, uint32_t pulse_ms)
{
  if (tool_type == TOOL_PASSIVE_HOOK_V1) {
    return std::make_shared<const PassiveHookV1Strategy>();
  }
  if (tool_type == TOOL_MAGNETIC_LATCH_V2) {
    return std::make_shared<const MagneticLatchV2Strategy>(pulse_ms);
  }
  throw std::invalid_argument(
    "未知工具类型: '" + tool_type + "'（可选: magnetic_latch_v2, passive_hook_v1）");
}

std::vector<std::string> primitive_names(const mtc::core::MotionPrimitiveList & primitives)
{
  std::vector<std::string> names;
  names.reserve(primitives.size());
  for (const auto & primitive : primitives) {
    names.push_back(primitive.name);
  }
  return names;
}

}  // namespace manipulation
}  // namespace mtc
