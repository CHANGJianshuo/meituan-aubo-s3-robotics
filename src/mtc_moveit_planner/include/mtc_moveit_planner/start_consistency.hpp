// Copyright 2026 meituan_challenge_ws contributors
//
// 轨迹起点一致性检查（纯 C++，供执行侧在发送 FollowJointTrajectory 前调用）。
//
// 背景（PLANNING_HANDOFF R1）：PickPlaceFsm 会提前规划多段原语，其中 test_lift / retreat
// 的起点依赖 ToolPort 动作（接合 / 退钩）结束时的位置，规划时只能假设。执行前若实际关节
// 与轨迹首点偏差超过容差：MoveIt 执行会 ABORTED（allowed_start_tolerance 0.01 rad），
// 直接发给 JTC 会跳变。因此执行端必须先检查，不一致则拒绝并请求重规划
// （MoveItPlannerPort::replan_primitive），绝不能"补一段过渡"或放大容差。

#ifndef MTC_MOVEIT_PLANNER__START_CONSISTENCY_HPP_
#define MTC_MOVEIT_PLANNER__START_CONSISTENCY_HPP_

#include <string>
#include <vector>

#include "mtc_core_cpp/timed_trajectory.hpp"

namespace mtc
{
namespace moveit_planner
{

/// 与 MoveIt TrajectoryExecutionManager 默认 allowed_start_tolerance 一致。
constexpr double kDefaultStartToleranceRad = 0.01;

struct StartCheck
{
  bool ok = false;
  double max_deviation_rad = 0.0;
  std::string worst_joint;
  std::string message;
};

/// 按关节**名称**比较轨迹首点与实际关节位置；缺关节、非有限值、空轨迹、空关节集合、
/// 轨迹或实际状态中出现重复 / 空关节名一律不通过（实际状态多出的无关关节允许）。
StartCheck check_trajectory_start(
  const mtc::core::TimedTrajectory & trajectory,
  const std::vector<std::string> & actual_names,
  const std::vector<double> & actual_positions,
  double tolerance_rad = kDefaultStartToleranceRad);

}  // namespace moveit_planner
}  // namespace mtc

#endif  // MTC_MOVEIT_PLANNER__START_CONSISTENCY_HPP_
