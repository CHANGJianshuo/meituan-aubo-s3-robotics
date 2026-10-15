// Copyright 2026 meituan_challenge_ws contributors
//
// 运动原语与运动命令（迁移自 Python pick_place_fsm.MotionPrimitive / MotionCommand，
// 以及 tool_strategy.py 的速度上限约定）。
//
// 职责边界：ToolStrategy 只产生运动原语（"有约束的局部运动意图"），
// 不直接调用 SDK、不发关节命令。执行由 PlannerPort / MotionPort 完成。

#ifndef MTC_CORE_CPP__MOTION_PRIMITIVE_HPP_
#define MTC_CORE_CPP__MOTION_PRIMITIVE_HPP_

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "mtc_core_cpp/pose.hpp"
#include "mtc_core_cpp/timed_trajectory.hpp"

namespace mtc
{
namespace core
{

/// 一条运动原语。
///
/// requires_contact 是安全关键位：接触阶段不得使用自由空间高速轨迹。
struct MotionPrimitive
{
  std::string name;
  std::string target_label;         ///< 目标标签/占位（本层不做 TF 求解，仅透传）
  /// 目标位姿（保留完整 6D + frame_id；未求解时为空）。
  std::optional<Pose6D> target_pose;
  double max_linear_speed = 0.05;   ///< m/s，由运动执行层强制
  double max_angular_speed = 0.2;   ///< rad/s
  bool requires_contact = false;

  bool is_usable() const
  {
    return !name.empty() && std::isfinite(max_linear_speed) && max_linear_speed > 0.0 &&
           std::isfinite(max_angular_speed) && max_angular_speed > 0.0 &&
           (!target_pose || target_pose->is_usable());
  }

  bool operator==(const MotionPrimitive & other) const
  {
    return name == other.name && target_label == other.target_label &&
           max_linear_speed == other.max_linear_speed &&
           max_angular_speed == other.max_angular_speed &&
           requires_contact == other.requires_contact;
  }
};

using MotionPrimitiveList = std::vector<MotionPrimitive>;

/// 等价于 ExecuteJointMove.action 的 Goal 字段集合。
///
/// 安全约束（对齐 ExecuteJointMove.action 注释）：
///   joint_names 必须与 target_rad 按索引一一对应，禁止假定 J1..J6 顺序。
struct MotionCommand
{
  std::string command_id;
  std::vector<std::string> joint_names;
  std::vector<double> target_rad;
  double velocity_scaling = 0.2;
  double acceleration_scaling = 0.2;
  uint32_t timeout_ms = 5000;
  std::string primitive_name;
  bool requires_contact = false;
  /// Planner output, distinct from the original intent. Exactly one representation.
  std::optional<TimedTrajectory> trajectory;
  std::optional<MotionPrimitive> intent;
  bool simulation_only = false;

  /// 结构自检：名称与目标数量一致、非空、无重复。
  bool shape_is_valid() const
  {
    if (joint_names.empty() || joint_names.size() != target_rad.size()) {
      return false;
    }
    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (joint_names[i].empty() || !std::isfinite(target_rad[i])) {
        return false;
      }
      for (size_t j = i + 1; j < joint_names.size(); ++j) {
        if (joint_names[i] == joint_names[j]) {
          return false;
        }
      }
    }
    return true;
  }
};

}  // namespace core
}  // namespace mtc

#endif  // MTC_CORE_CPP__MOTION_PRIMITIVE_HPP_
