// Copyright 2026 meituan_challenge_ws contributors
//
// 关节空间直线路径的"末端限速"时间最优参数化（纯 C++，无 MoveIt 依赖）。
//
// 背景：原做法是 TOTG 后按"整条轨迹的末端速度峰值"做**均匀**时间拉伸。关节插值路径上末端
// 线/角速度沿路径变化很大，均匀拉伸会让大部分路段远低于上限（V1 预对准实测平均角速度仅为上限
// 的 40% 左右）。本模块在路径参数 s 上逐段施加全部速度上限，再做前向/后向加速度约束积分，
// 得到满足**同样**上限、但不浪费时间的速度剖面。上限本身不变：
//   * 每个关节 |q̇_j| <= 关节速度上限 × velocity_scaling；
//   * 每个关节 |q̈_j| <= 关节加速度上限 × acceleration_scaling；
//   * 末端线速度 <= max_linear_speed，末端角速度 <= max_angular_speed（逐段，按路点 FK 位移计算）。
//
// 只接受**关节空间直线**（所有路段方向相同）：此时 dq/ds 为常向量，加速度约束与 s 无关，
// 前向/后向积分即为时间最优解；路段内速度单调变化，不会超过两端的段上限。
// 末端速度按相邻路点之间的位移估计，调用方仍应对最终轨迹做密采样复核。

#ifndef MTC_MOVEIT_PLANNER__EE_LIMITED_RETIMING_HPP_
#define MTC_MOVEIT_PLANNER__EE_LIMITED_RETIMING_HPP_

#include <string>
#include <vector>

namespace mtc
{
namespace moveit_planner
{

struct EeLimitedRetimingInput
{
  std::vector<std::vector<double>> waypoints;   ///< 关节位置路点（同一直线上，含首尾）
  std::vector<double> segment_linear_m;          ///< 每段末端位移（m），size = 路点数 - 1
  std::vector<double> segment_angular_rad;       ///< 每段末端转角（rad），size = 路点数 - 1
  std::vector<double> joint_velocity_limits;     ///< 已乘 velocity_scaling（rad/s，> 0）
  std::vector<double> joint_acceleration_limits; ///< 已乘 acceleration_scaling（rad/s²，> 0）
  double max_linear_speed_m_s = 0.0;             ///< <= 0 表示不限制
  double max_angular_speed_rad_s = 0.0;          ///< <= 0 表示不限制
};

struct EeLimitedRetimingResult
{
  bool ok = false;
  std::string message;
  std::vector<double> time_from_start_s;               ///< 每个路点的时刻，首点为 0
  std::vector<std::vector<double>> velocities;         ///< 每个路点的关节速度
  std::vector<std::vector<double>> accelerations;      ///< 每个路点的关节加速度
  double duration_s = 0.0;
};

/// 计算时间最优剖面；输入不合法（维度不符、非直线、上限非正、非有限值）时 ok=false。
EeLimitedRetimingResult retime_straight_joint_path(const EeLimitedRetimingInput & input);

}  // namespace moveit_planner
}  // namespace mtc

#endif  // MTC_MOVEIT_PLANNER__EE_LIMITED_RETIMING_HPP_
