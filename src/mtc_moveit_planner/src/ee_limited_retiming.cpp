// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_moveit_planner/ee_limited_retiming.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mtc
{
namespace moveit_planner
{

namespace
{

EeLimitedRetimingResult failed(const std::string & message)
{
  EeLimitedRetimingResult result;
  result.message = message;
  return result;
}

bool all_finite(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double v) {return std::isfinite(v);});
}

}  // namespace

EeLimitedRetimingResult retime_straight_joint_path(const EeLimitedRetimingInput & input)
{
  const size_t n_points = input.waypoints.size();
  if (n_points < 2) {
    return failed("need at least 2 waypoints");
  }
  const size_t dof = input.waypoints.front().size();
  if (dof == 0 || input.joint_velocity_limits.size() != dof ||
    input.joint_acceleration_limits.size() != dof)
  {
    return failed("joint dimension mismatch");
  }
  const size_t n_seg = n_points - 1;
  if (input.segment_linear_m.size() != n_seg || input.segment_angular_rad.size() != n_seg) {
    return failed("segment measure count must equal waypoint count - 1");
  }
  for (size_t j = 0; j < dof; ++j) {
    if (!(input.joint_velocity_limits[j] > 0.0) || !(input.joint_acceleration_limits[j] > 0.0) ||
      !std::isfinite(input.joint_velocity_limits[j]) ||
      !std::isfinite(input.joint_acceleration_limits[j]))
    {
      return failed("joint velocity/acceleration limits must be positive and finite");
    }
  }
  for (const auto & q : input.waypoints) {
    if (q.size() != dof || !all_finite(q)) {
      return failed("waypoints must have equal size and finite values");
    }
  }
  if (!all_finite(input.segment_linear_m) || !all_finite(input.segment_angular_rad) ||
    !std::isfinite(input.max_linear_speed_m_s) || !std::isfinite(input.max_angular_speed_rad_s))
  {
    return failed("non-finite end-effector measure or cap");
  }

  // 路径方向 u（单位向量）与每段长度 ds；要求所有段与 u 同向（关节空间直线）。
  std::vector<double> delta(dof);
  double total = 0.0;
  for (size_t j = 0; j < dof; ++j) {
    delta[j] = input.waypoints.back()[j] - input.waypoints.front()[j];
    total += delta[j] * delta[j];
  }
  total = std::sqrt(total);
  if (!(total > 1e-9)) {
    return failed("path has zero length");
  }
  std::vector<double> u(dof);
  for (size_t j = 0; j < dof; ++j) {
    u[j] = delta[j] / total;
  }
  std::vector<double> ds(n_seg);
  for (size_t k = 0; k < n_seg; ++k) {
    double along = 0.0;
    double off = 0.0;
    for (size_t j = 0; j < dof; ++j) {
      along += (input.waypoints[k + 1][j] - input.waypoints[k][j]) * u[j];
    }
    for (size_t j = 0; j < dof; ++j) {
      const double d = input.waypoints[k + 1][j] - input.waypoints[k][j] - along * u[j];
      off += d * d;
    }
    if (!(along > 1e-12) || std::sqrt(off) > 1e-6 + 1e-6 * total) {
      return failed("waypoints are not on one forward joint-space straight line");
    }
    ds[k] = along;
  }

  // 关节约束 -> 路径速度 / 加速度上限（直线路径上为常数）。
  double sdot_joint = std::numeric_limits<double>::infinity();
  double sddot_max = std::numeric_limits<double>::infinity();
  for (size_t j = 0; j < dof; ++j) {
    if (std::fabs(u[j]) > 1e-12) {
      sdot_joint = std::min(sdot_joint, input.joint_velocity_limits[j] / std::fabs(u[j]));
      sddot_max = std::min(sddot_max, input.joint_acceleration_limits[j] / std::fabs(u[j]));
    }
  }

  // 每段的路径速度上限：关节 + 末端线速度 + 末端角速度。
  std::vector<double> seg_cap(n_seg, sdot_joint);
  for (size_t k = 0; k < n_seg; ++k) {
    if (input.max_linear_speed_m_s > 0.0 && input.segment_linear_m[k] > 0.0) {
      seg_cap[k] = std::min(seg_cap[k],
          input.max_linear_speed_m_s * ds[k] / input.segment_linear_m[k]);
    }
    if (input.max_angular_speed_rad_s > 0.0 && input.segment_angular_rad[k] > 0.0) {
      seg_cap[k] = std::min(seg_cap[k],
          input.max_angular_speed_rad_s * ds[k] / input.segment_angular_rad[k]);
    }
  }

  // 路点速度上限 = 相邻两段上限的较小者；首尾静止。前向 / 后向加速度积分。
  std::vector<double> v(n_points, 0.0);
  for (size_t k = 1; k + 1 < n_points; ++k) {
    v[k] = std::min(seg_cap[k - 1], seg_cap[k]);
  }
  for (size_t k = 0; k < n_seg; ++k) {
    v[k + 1] = std::min(v[k + 1], std::sqrt(v[k] * v[k] + 2.0 * sddot_max * ds[k]));
  }
  for (size_t k = n_seg; k-- > 0; ) {
    v[k] = std::min(v[k], std::sqrt(v[k + 1] * v[k + 1] + 2.0 * sddot_max * ds[k]));
  }

  EeLimitedRetimingResult result;
  result.time_from_start_s.assign(n_points, 0.0);
  std::vector<double> seg_acc(n_seg, 0.0);
  for (size_t k = 0; k < n_seg; ++k) {
    const double v_sum = v[k] + v[k + 1];
    if (!(v_sum > 0.0)) {
      return failed("zero path speed on a segment (a cap is zero?)");
    }
    const double dt = 2.0 * ds[k] / v_sum;   // 段内匀加速
    result.time_from_start_s[k + 1] = result.time_from_start_s[k] + dt;
    seg_acc[k] = (v[k + 1] * v[k + 1] - v[k] * v[k]) / (2.0 * ds[k]);
  }
  result.velocities.assign(n_points, std::vector<double>(dof, 0.0));
  result.accelerations.assign(n_points, std::vector<double>(dof, 0.0));
  for (size_t k = 0; k < n_points; ++k) {
    // 路点处取相邻段加速度中绝对值较小者，保证 |q̈| 不超过任一段的上限。
    double a = 0.0;
    if (k == 0) {
      a = seg_acc.front();
    } else if (k == n_seg) {
      a = seg_acc.back();
    } else {
      a = std::fabs(seg_acc[k - 1]) < std::fabs(seg_acc[k]) ? seg_acc[k - 1] : seg_acc[k];
    }
    for (size_t j = 0; j < dof; ++j) {
      result.velocities[k][j] = u[j] * v[k];
      result.accelerations[k][j] = u[j] * a;
    }
  }
  result.duration_s = result.time_from_start_s.back();
  result.ok = std::isfinite(result.duration_s) && result.duration_s > 0.0;
  result.message = result.ok ? "ok" : "non-finite duration";
  return result;
}

}  // namespace moveit_planner
}  // namespace mtc
