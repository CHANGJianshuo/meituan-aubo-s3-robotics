// Copyright 2026 meituan_challenge_ws contributors
//
// 规划器原始轨迹 -> mtc::core::TimedTrajectory 的转换（纯 C++）。
//
// 约束来源：PickPlaceFsm::accept_plan() 要求
//   trajectory->joint_names == PickPlaceConfig::joint_names（逐元素、同顺序）。
// MoveIt 的关节顺序不保证与 FSM 一致（aubo_moveit_config/config/moveit_controllers.yaml
// 中 upperArm/foreArm 的列举顺序即与驱动控制器相反），因此这里**按名称重排**，
// 绝不按索引套用。缺关节、多关节、重复关节、时间不递增一律拒绝，不做修补。

#ifndef MTC_MOVEIT_PLANNER__TRAJECTORY_CONVERSION_HPP_
#define MTC_MOVEIT_PLANNER__TRAJECTORY_CONVERSION_HPP_

#include <string>
#include <vector>

#include "mtc_core_cpp/timed_trajectory.hpp"

namespace mtc
{
namespace moveit_planner
{

/// 与 trajectory_msgs/JointTrajectoryPoint 同构的中立结构（不依赖 ROS 消息）。
struct RawTrajectoryPoint
{
  double time_from_start_s = 0.0;
  std::vector<double> positions;
  std::vector<double> velocities;     ///< 可为空
  std::vector<double> accelerations;  ///< 可为空
};

struct RawTrajectory
{
  std::vector<std::string> joint_names;
  std::vector<RawTrajectoryPoint> points;
};

struct ConversionResult
{
  bool ok = false;
  std::string error;
  mtc::core::TimedTrajectory trajectory;
};

/// 按 expected_joint_order 重排并校验。frame_id 为空直接拒绝（不允许丢失坐标系）。
ConversionResult to_timed_trajectory(
  const RawTrajectory & raw,
  const std::vector<std::string> & expected_joint_order,
  const std::string & frame_id,
  double stamp_s);

/// 均匀时间拉伸：factor >= 1 时整体变慢（路径不变），速度 /factor，加速度 /factor²。
/// factor 非有限或 < 1 时不做任何修改并返回 false。
bool stretch_time(mtc::core::TimedTrajectory & trajectory, double factor);

/// 轨迹最后一个点的关节位置（按 trajectory.joint_names 顺序）；空轨迹返回空。
std::vector<double> final_positions(const mtc::core::TimedTrajectory & trajectory);

}  // namespace moveit_planner
}  // namespace mtc

#endif  // MTC_MOVEIT_PLANNER__TRAJECTORY_CONVERSION_HPP_
