// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_motion_execution_cpp/validation.hpp"

#include <algorithm>
#include <cmath>
#include <set>

#include "mtc_core_cpp/error_codes.hpp"

namespace mtc
{
namespace motion
{

namespace
{

ValidationOutcome make_outcome(bool ok, uint16_t code, const std::string & message)
{
  ValidationOutcome outcome;
  outcome.ok = ok;
  outcome.error_code = code;
  outcome.message = message;
  return outcome;
}

bool has_duplicates(const std::vector<std::string> & names)
{
  const std::set<std::string> unique(names.begin(), names.end());
  return unique.size() != names.size();
}

bool valid_config(const ValidationConfig & config, size_t count, bool simulation)
{
  for (double v : {config.max_velocity_scaling, config.max_acceleration_scaling,
      config.settle_position_tolerance_rad, config.settle_velocity_tolerance_rad_s,
      config.settle_duration_s, config.feedback_max_age_s}) {
    if (!std::isfinite(v) || v <= 0.0) {return false;}
  }
  if (config.max_velocity_scaling > 1.0 || config.max_acceleration_scaling > 1.0) {return false;}
  if (!simulation && (config.expected_joint_names.size() != count || count != 6 ||
    config.joint_limit_rad.size() != count)) {return false;}
  if (!config.joint_limit_rad.empty() && config.joint_limit_rad.size() == count) {
    for (const auto & limit : config.joint_limit_rad) {
      if (!std::isfinite(limit.first) || !std::isfinite(limit.second) ||
        limit.first >= limit.second) {return false;}
    }
  }
  for (const auto & limits : {config.max_joint_velocity_rad_s,
      config.max_joint_acceleration_rad_s2}) {
    if (limits.size() != count) {return false;}
    for (double v : limits) {
      if (!std::isfinite(v) || v <= 0.0) {return false;}
    }
  }
  return true;
}

}  // namespace

ValidationOutcome validate_joint_move(
  const mtc::core::MotionCommand & request,
  const ValidationConfig & config,
  const mtc::core::MotionCapabilities & capabilities)
{
  if (!capabilities.joint_goal_supported) {
    return make_outcome(
      false, mtc::core::ErrorCodes::EXECUTION_REJECTED,
      "后端 " + capabilities.backend_name +
      " 未声明 joint_goal_supported，拒绝执行关节目标运动");
  }
  if (request.joint_names.size() != request.target_rad.size()) {
    return make_outcome(
      false, mtc::core::ErrorCodes::EXECUTION_REJECTED,
      "joint_names 与 target_rad 长度不一致：必须按索引一一对应");
  }
  if (request.joint_names.empty()) {
    return make_outcome(
      false, mtc::core::ErrorCodes::EXECUTION_REJECTED, "joint_names 不能为空");
  }
  if (request.command_id.empty() || request.trajectory ||
    !std::all_of(request.target_rad.begin(), request.target_rad.end(),
    [](double v) {return std::isfinite(v);}) ||
    std::any_of(request.joint_names.begin(), request.joint_names.end(),
    [](const std::string & name) {return name.empty();})) {
    return make_outcome(false, mtc::core::ErrorCodes::EXECUTION_REJECTED,
      "非法 command_id / 关节目标 / 轨迹表示");
  }
  if (!valid_config(config, request.joint_names.size(), capabilities.simulation_only) ||
    (request.simulation_only && !capabilities.simulation_only) ||
    (!capabilities.simulation_only && !capabilities.associated_cancel_supported)) {
    return make_outcome(false, mtc::core::ErrorCodes::SAFETY_INTERLOCK,
      "非法安全配置；可执行后端必须配置六关节限位与动力学上限");
  }
  if (has_duplicates(request.joint_names)) {
    return make_outcome(
      false, mtc::core::ErrorCodes::EXECUTION_REJECTED, "joint_names 存在重复项");
  }
  if (!config.expected_joint_names.empty() &&
    request.joint_names != config.expected_joint_names)
  {
    return make_outcome(
      false, mtc::core::ErrorCodes::EXECUTION_REJECTED,
      "joint_names 与配置的关节顺序不一致，禁止假定固定 J1..J6 顺序");
  }
  if (request.timeout_ms == 0u) {
    return make_outcome(
      false, mtc::core::ErrorCodes::EXECUTION_REJECTED, "timeout_ms 必须大于 0");
  }
  if (!(request.velocity_scaling > 0.0 &&
    request.velocity_scaling <= config.max_velocity_scaling))
  {
    return make_outcome(
      false, mtc::core::ErrorCodes::SAFETY_INTERLOCK,
      "velocity_scaling 超出已审核上限（未验收的高速动作禁止执行）");
  }
  if (!(request.acceleration_scaling > 0.0 &&
    request.acceleration_scaling <= config.max_acceleration_scaling))
  {
    return make_outcome(
      false, mtc::core::ErrorCodes::SAFETY_INTERLOCK,
      "acceleration_scaling 超出已审核上限");
  }
  if (!config.joint_limit_rad.empty()) {
    if (config.joint_limit_rad.size() != request.target_rad.size()) {
      return make_outcome(
        false, mtc::core::ErrorCodes::EXECUTION_REJECTED, "joint_limit_rad 维度与目标不一致");
    }
    for (size_t i = 0; i < request.target_rad.size(); ++i) {
      const double value = request.target_rad[i];
      const double low = config.joint_limit_rad[i].first;
      const double high = config.joint_limit_rad[i].second;
      if (!(value >= low && value <= high)) {
        return make_outcome(
          false, mtc::core::ErrorCodes::SAFETY_INTERLOCK,
          "关节 " + request.joint_names[i] + " 目标超出限位");
      }
    }
  }
  return make_outcome(true, mtc::core::ErrorCodes::OK, "validated");
}

ValidationOutcome validate_timed_trajectory(
  const mtc::core::TimedTrajectory & trajectory,
  const ValidationConfig & config,
  const mtc::core::MotionCapabilities & capabilities)
{
  if (!capabilities.timed_trajectory_supported) {
    return make_outcome(
      false, mtc::core::ErrorCodes::EXECUTION_REJECTED,
      "后端 " + capabilities.backend_name +
      " 未声明 timed_trajectory_supported，拒绝执行时间参数化轨迹；"
      "禁止把轨迹拆成多次单点调用");
  }
  if (!trajectory.shape_is_valid()) {
    return make_outcome(
      false, mtc::core::ErrorCodes::EXECUTION_REJECTED,
      "轨迹结构非法：关节名/路点维度/时间单调性校验未通过");
  }
  if (!valid_config(config, trajectory.joint_names.size(), capabilities.simulation_only) ||
    trajectory.trajectory_id.empty() ||
    (!capabilities.simulation_only && !capabilities.associated_cancel_supported)) {
    return make_outcome(false, mtc::core::ErrorCodes::SAFETY_INTERLOCK,
      "轨迹缺少身份或安全配置");
  }
  for (const auto & point : trajectory.points) {
    for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
      if ((!point.velocities_rad_s.empty() &&
        std::abs(point.velocities_rad_s[i]) > config.max_joint_velocity_rad_s[i]) ||
        (!point.accelerations_rad_s2.empty() &&
        std::abs(point.accelerations_rad_s2[i]) > config.max_joint_acceleration_rad_s2[i])) {
        return make_outcome(false, mtc::core::ErrorCodes::SAFETY_INTERLOCK,
          "轨迹速度或加速度超出已审核上限");
      }
    }
  }
  if (!config.expected_joint_names.empty() &&
    trajectory.joint_names != config.expected_joint_names)
  {
    return make_outcome(
      false, mtc::core::ErrorCodes::EXECUTION_REJECTED,
      "轨迹 joint_names 与配置的关节顺序不一致");
  }
  if (!config.joint_limit_rad.empty()) {
    if (config.joint_limit_rad.size() != trajectory.joint_names.size()) {
      return make_outcome(
        false, mtc::core::ErrorCodes::EXECUTION_REJECTED, "joint_limit_rad 维度与轨迹不一致");
    }
    for (const auto & point : trajectory.points) {
      for (size_t i = 0; i < point.positions_rad.size(); ++i) {
        const double value = point.positions_rad[i];
        const double low = config.joint_limit_rad[i].first;
        const double high = config.joint_limit_rad[i].second;
        if (!(value >= low && value <= high)) {
          return make_outcome(
            false, mtc::core::ErrorCodes::SAFETY_INTERLOCK,
            "轨迹路点超出关节限位：" + trajectory.joint_names[i]);
        }
      }
    }
  }
  return make_outcome(true, mtc::core::ErrorCodes::OK, "validated");
}

}  // namespace motion
}  // namespace mtc
