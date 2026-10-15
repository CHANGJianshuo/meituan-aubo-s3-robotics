// Copyright 2026 meituan_challenge_ws contributors
//
// 规划证据输出：把 PlanResult / PlanReport / TimedTrajectory 写成可读 YAML，
// 供团队对接与复现（不是 ROS 参数文件，也不会被任何执行端读取）。

#ifndef MTC_MOVEIT_PLANNER__PLAN_REPORT_IO_HPP_
#define MTC_MOVEIT_PLANNER__PLAN_REPORT_IO_HPP_

#include <string>

#include "mtc_core_cpp/results.hpp"
#include "mtc_core_cpp/timed_trajectory.hpp"
#include "mtc_moveit_planner/moveit_planner_port.hpp"

namespace mtc
{
namespace moveit_planner
{

std::string pose_to_yaml(const mtc::core::Pose6D & pose, const std::string & indent);
std::string joint_state_to_yaml(const JointStateSnapshot & state, const std::string & indent);
std::string trajectory_to_yaml(
  const mtc::core::TimedTrajectory & trajectory, const std::string & indent);
std::string plan_to_yaml(
  const mtc::core::PlanResult & result, const PlanReport & report, const std::string & indent);

/// 写文件（父目录不存在时创建）；失败返回 false。
bool write_text_file(const std::string & path, const std::string & content);

/// 本地时间戳，形如 20261011_193512。
std::string timestamp_for_filename();

}  // namespace moveit_planner
}  // namespace mtc

#endif  // MTC_MOVEIT_PLANNER__PLAN_REPORT_IO_HPP_
