// Copyright 2026 meituan_challenge_ws contributors
//
// 多点时间参数化轨迹类型（**纯 C++，不依赖 ROS 消息**）。
//
// 存在理由（见 docs/migration/INTERFACE_COMPAT_AUDIT_CPP.md 缺口 G1）：
//   现有 mtc_interfaces 只有 ExecuteJointMove.action（单次 6 关节目标），
//   无法承载 MoveIt 2 输出的多点时间参数化轨迹。需求明确禁止
//   "把多点轨迹拆成大量 ExecuteJointMove 调用"。
//
// 本类型因此作为 C++ 侧的**能力载体**先落地：它使 MotionAdapter 能够以
// "一条轨迹"为原子单位表达意图，并在 GetMotionCapabilities 中如实声明
// timed_trajectory_supported。真正的 ROS 2 Action / FollowJointTrajectory
// 绑定属于第二阶段，本阶段**只提供类型与 Mock 实现**，不新增 ROS 接口。

#ifndef MTC_CORE_CPP__TIMED_TRAJECTORY_HPP_
#define MTC_CORE_CPP__TIMED_TRAJECTORY_HPP_

#include <cstdint>
#include <cmath>
#include <string>
#include <vector>

namespace mtc
{
namespace core
{

/// 轨迹上的一个路点：时间 + 位置（必需）+ 速度/加速度（可选）。
struct TrajectoryPoint
{
  double time_from_start_s = 0.0;
  std::vector<double> positions_rad;
  std::vector<double> velocities_rad_s;      ///< 空表示未指定
  std::vector<double> accelerations_rad_s2;  ///< 空表示未指定
};

/// 时间参数化关节轨迹（语义等价于 trajectory_msgs/JointTrajectory
/// 与 control_msgs/FollowJointTrajectory 的 goal 组合）。
struct TimedTrajectory
{
  std::string trajectory_id;
  std::vector<std::string> joint_names;   ///< 顺序即 positions 的索引顺序
  std::vector<TrajectoryPoint> points;
  std::string header_frame_id;            ///< 轨迹参考坐标系（保留 TF 信息）
  double stamp_s = 0.0;

  /// 结构自检：关节名非空且唯一、至少 1 个路点、
  /// 每个路点维度与关节数一致、时间严格递增。
  bool shape_is_valid() const
  {
    if (joint_names.empty() || points.empty() || !std::isfinite(stamp_s) || stamp_s < 0.0) {
      return false;
    }
    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (joint_names[i].empty()) {
        return false;
      }
      for (size_t j = i + 1; j < joint_names.size(); ++j) {
        if (joint_names[i] == joint_names[j]) {
          return false;
        }
      }
    }
    double previous_time = -1.0;
    for (const auto & point : points) {
      if (point.positions_rad.size() != joint_names.size()) {
        return false;
      }
      if (!point.velocities_rad_s.empty() &&
        point.velocities_rad_s.size() != joint_names.size())
      {
        return false;
      }
      if (!point.accelerations_rad_s2.empty() &&
        point.accelerations_rad_s2.size() != joint_names.size())
      {
        return false;
      }
      for (const auto & values : {point.positions_rad, point.velocities_rad_s,
          point.accelerations_rad_s2}) {
        for (double value : values) {
          if (!std::isfinite(value)) {return false;}
        }
      }
      if (!std::isfinite(point.time_from_start_s) || point.time_from_start_s < 0.0 ||
        point.time_from_start_s <= previous_time) {
        return false;
      }
      previous_time = point.time_from_start_s;
    }
    return true;
  }

  /// 轨迹总时长（秒）。
  double duration_s() const
  {
    return points.empty() ? 0.0 : points.back().time_from_start_s;
  }
};

}  // namespace core
}  // namespace mtc

#endif  // MTC_CORE_CPP__TIMED_TRAJECTORY_HPP_
