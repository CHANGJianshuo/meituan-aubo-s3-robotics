// Copyright 2026 meituan_challenge_ws contributors

#include <gtest/gtest.h>

#include <cmath>

#include "mtc_moveit_planner/pose_math.hpp"

namespace
{

using mtc::core::Pose6D;
using mtc::core::Quaternion;
using mtc::core::Vector3;
using mtc::moveit_planner::Transform;
using mtc::moveit_planner::angle_between;
using mtc::moveit_planner::apply;
using mtc::moveit_planner::compose;
using mtc::moveit_planner::inverse;
using mtc::moveit_planner::offset_along_local_axis;
using mtc::moveit_planner::rotate;
using mtc::moveit_planner::translate_in_frame;

constexpr double kEps = 1e-9;

Quaternion about_z(double angle)
{
  return Quaternion{0.0, 0.0, std::sin(angle / 2.0), std::cos(angle / 2.0)};
}

Quaternion about_x(double angle)
{
  return Quaternion{std::sin(angle / 2.0), 0.0, 0.0, std::cos(angle / 2.0)};
}

Pose6D pose(double x, double y, double z, Quaternion q, const std::string & frame = "world")
{
  Pose6D p;
  p.position = Vector3{x, y, z};
  p.orientation = q;
  p.frame_id = frame;
  p.stamp_s = 1.5;
  p.valid = true;
  return p;
}

TEST(PoseMath, RotateAboutZ)
{
  const auto v = rotate(about_z(M_PI / 2.0), Vector3{1.0, 0.0, 0.0});
  EXPECT_NEAR(v.x, 0.0, kEps);
  EXPECT_NEAR(v.y, 1.0, kEps);
  EXPECT_NEAR(v.z, 0.0, kEps);
}

TEST(PoseMath, ComposeWithInverseIsIdentity)
{
  const Transform t{Vector3{0.1, -0.2, 0.3}, about_x(0.7)};
  const auto identity = compose(t, inverse(t));
  EXPECT_NEAR(identity.translation.x, 0.0, kEps);
  EXPECT_NEAR(identity.translation.y, 0.0, kEps);
  EXPECT_NEAR(identity.translation.z, 0.0, kEps);
  EXPECT_NEAR(angle_between(identity.rotation, Quaternion{}), 0.0, 1e-6);
}

TEST(PoseMath, OffsetAlongLocalAxisFollowsOrientation)
{
  // 绕 x 转 90°：局部 +Z 指向父系 -Y。
  const auto base = pose(0.4, 0.0, 0.2, about_x(M_PI / 2.0), "base_link");
  const auto moved = offset_along_local_axis(base, 2, -0.03);
  EXPECT_NEAR(moved.position.x, 0.4, kEps);
  EXPECT_NEAR(moved.position.y, 0.03, kEps);
  EXPECT_NEAR(moved.position.z, 0.2, kEps);
  EXPECT_EQ(moved.frame_id, "base_link");
  EXPECT_DOUBLE_EQ(moved.stamp_s, 1.5);
  EXPECT_NEAR(angle_between(moved.orientation, base.orientation), 0.0, 1e-6);
}

TEST(PoseMath, TcpCompensationRoundTrip)
{
  // ee_link -> hook_tcp：沿 ee z 前伸 0.12 m 且绕 z 转 30°。
  const Transform ee_to_tcp{Vector3{0.0, 0.0, 0.12}, about_z(M_PI / 6.0)};
  const auto ee = pose(0.3, 0.1, 0.4, about_x(M_PI));
  const auto tcp = apply(ee, ee_to_tcp);
  // 绕 x 转 180° 后 ee 的 +Z 指向下方，钩尖应在 ee 下方 0.12 m。
  EXPECT_NEAR(tcp.position.z, 0.28, kEps);
  const auto back = apply(tcp, inverse(ee_to_tcp));
  EXPECT_NEAR(back.position.x, ee.position.x, kEps);
  EXPECT_NEAR(back.position.y, ee.position.y, kEps);
  EXPECT_NEAR(back.position.z, ee.position.z, kEps);
  EXPECT_NEAR(angle_between(back.orientation, ee.orientation), 0.0, 1e-6);
}

TEST(PoseMath, TranslateInFrameKeepsOrientation)
{
  const auto base = pose(0.3, 0.1, 0.4, about_z(0.4));
  const auto up = translate_in_frame(base, Vector3{0.0, 0.0, 0.05});
  EXPECT_NEAR(up.position.z, 0.45, kEps);
  EXPECT_EQ(up.orientation, base.orientation);
}

}  // namespace
