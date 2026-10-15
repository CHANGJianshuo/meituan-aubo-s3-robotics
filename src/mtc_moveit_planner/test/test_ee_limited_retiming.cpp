// Copyright 2026 meituan_challenge_ws contributors

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "mtc_moveit_planner/ee_limited_retiming.hpp"

namespace
{

using mtc::moveit_planner::EeLimitedRetimingInput;
using mtc::moveit_planner::retime_straight_joint_path;

/// 两关节直线路径 (0,0) -> (L, L/2)，n 段等分。
EeLimitedRetimingInput line(double length, int n)
{
  EeLimitedRetimingInput in;
  for (int k = 0; k <= n; ++k) {
    const double s = static_cast<double>(k) / n;
    in.waypoints.push_back({length * s, 0.5 * length * s});
  }
  in.segment_linear_m.assign(n, 0.0);
  in.segment_angular_rad.assign(n, 0.0);
  in.joint_velocity_limits = {0.2, 0.2};
  in.joint_acceleration_limits = {0.5, 0.5};
  return in;
}

TEST(EeLimitedRetiming, JointLimitedTrapezoidMatchesClosedForm)
{
  const auto in = line(2.0, 200);
  const auto r = retime_straight_joint_path(in);
  ASSERT_TRUE(r.ok) << r.message;
  // 关节 0 主导：|u0| = 1/sqrt(1.25)；按关节 0 的行程 2 rad、0.2 rad/s、0.5 rad/s² 的梯形。
  const double expected = 2.0 / 0.2 + 0.2 / 0.5;
  EXPECT_NEAR(r.duration_s, expected, 0.02);
  for (size_t k = 0; k < r.velocities.size(); ++k) {
    EXPECT_LE(std::fabs(r.velocities[k][0]), 0.2 + 1e-9);
    EXPECT_LE(std::fabs(r.accelerations[k][0]), 0.5 + 1e-9);
  }
  EXPECT_DOUBLE_EQ(r.velocities.front()[0], 0.0);
  EXPECT_DOUBLE_EQ(r.velocities.back()[0], 0.0);
}

TEST(EeLimitedRetiming, AngularCapHoldsPerSegmentAndBeatsUniformStretch)
{
  // 末端转角沿路径不均匀：中段每段转得多，两端转得少（与 V1 关节插值的情形类似）。
  auto in = line(2.0, 100);
  in.joint_velocity_limits = {10.0, 10.0};
  in.joint_acceleration_limits = {10.0, 10.0};
  in.max_angular_speed_rad_s = 0.1;
  double total_angle = 0.0;
  for (int k = 0; k < 100; ++k) {
    const double x = (k + 0.5) / 100.0;
    in.segment_angular_rad[k] = 0.002 + 0.06 * std::exp(-std::pow((x - 0.5) / 0.08, 2.0));
    total_angle += in.segment_angular_rad[k];
  }
  const auto r = retime_straight_joint_path(in);
  ASSERT_TRUE(r.ok) << r.message;
  for (int k = 0; k < 100; ++k) {
    const double dt = r.time_from_start_s[k + 1] - r.time_from_start_s[k];
    EXPECT_LE(in.segment_angular_rad[k] / dt, 0.1 * (1.0 + 1e-9)) << "segment " << k;
  }
  // 下界：总转角 / 上限；均匀拉伸：最大单段转角决定的匀速时长。
  const double lower_bound = total_angle / 0.1;
  const double peak = *std::max_element(in.segment_angular_rad.begin(), in.segment_angular_rad.end());
  const double uniform = 100 * peak / 0.1;
  EXPECT_GE(r.duration_s, lower_bound - 1e-9);
  EXPECT_LT(r.duration_s, 0.5 * uniform);
}

TEST(EeLimitedRetiming, ShortPathIsTriangularAndLinearCapApplies)
{
  auto in = line(0.01, 10);
  const auto r = retime_straight_joint_path(in);
  ASSERT_TRUE(r.ok) << r.message;
  const double peak = std::max_element(r.velocities.begin(), r.velocities.end(),
      [](const auto & a, const auto & b) {return a[0] < b[0];})->at(0);
  EXPECT_LT(peak, 0.2);   // 没到速度上限

  auto lin = line(1.0, 50);
  lin.joint_velocity_limits = {10.0, 10.0};
  lin.joint_acceleration_limits = {100.0, 100.0};
  lin.max_linear_speed_m_s = 0.02;
  lin.segment_linear_m.assign(50, 0.004);   // 共 0.2 m
  const auto rl = retime_straight_joint_path(lin);
  ASSERT_TRUE(rl.ok) << rl.message;
  // 匀速段 0.2 m / 0.02 m/s = 10 s；首末段按"段内匀加速"从静止起步，各多出一个段时长（保守）。
  const double segment_time = 0.004 / 0.02;
  EXPECT_GE(rl.duration_s, 0.2 / 0.02 - 1e-9);
  EXPECT_LE(rl.duration_s, 0.2 / 0.02 + 2.0 * segment_time + 1e-6);
}

TEST(EeLimitedRetiming, InvalidInputsAreRejected)
{
  auto bent = line(1.0, 10);
  bent.waypoints[5][1] += 0.05;   // 不在同一直线
  EXPECT_FALSE(retime_straight_joint_path(bent).ok);

  auto backwards = line(1.0, 10);
  std::swap(backwards.waypoints[3], backwards.waypoints[4]);
  EXPECT_FALSE(retime_straight_joint_path(backwards).ok);

  auto sizes = line(1.0, 10);
  sizes.segment_angular_rad.pop_back();
  EXPECT_FALSE(retime_straight_joint_path(sizes).ok);

  auto limits = line(1.0, 10);
  limits.joint_acceleration_limits[1] = 0.0;
  EXPECT_FALSE(retime_straight_joint_path(limits).ok);

  auto nan = line(1.0, 10);
  nan.segment_linear_m[2] = std::nan("");
  EXPECT_FALSE(retime_straight_joint_path(nan).ok);

  EXPECT_FALSE(retime_straight_joint_path(line(0.0, 10)).ok);
}

}  // namespace
