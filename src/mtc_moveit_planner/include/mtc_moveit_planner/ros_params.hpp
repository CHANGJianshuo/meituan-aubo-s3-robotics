// Copyright 2026 meituan_challenge_ws contributors
//
// 从 ROS 参数（config/planner.yaml）读取规划配置；缺省值取 default_planner_config()。

#ifndef MTC_MOVEIT_PLANNER__ROS_PARAMS_HPP_
#define MTC_MOVEIT_PLANNER__ROS_PARAMS_HPP_

#include <string>

#include "mtc_moveit_planner/move_group_backend.hpp"
#include "mtc_moveit_planner/planner_config.hpp"

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace mtc
{
namespace moveit_planner
{

/// 读取失败（状态字符串非法、数组长度不对）时抛 std::runtime_error，不静默回退。
PlannerConfig load_planner_config(rclcpp::Node & node);
MoveGroupBackendOptions load_backend_options(rclcpp::Node & node);

}  // namespace moveit_planner
}  // namespace mtc

#endif  // MTC_MOVEIT_PLANNER__ROS_PARAMS_HPP_
