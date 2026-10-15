// Copyright 2026 meituan_challenge_ws contributors

#include <gtest/gtest.h>

#include <cmath>

#include "mtc_moveit_planner/tool_model.hpp"

namespace
{

using mtc::core::Quaternion;
using mtc::core::Vector3;
using namespace mtc::moveit_planner;  // NOLINT(build/namespaces)

constexpr double kTol = 1e-9;

/// 舌规V1.STEP：CAD +Y 离开法兰，舌头沿 +Z，倒钩朝 -Y；挂环点 (0, 110, 45) mm。
ToolModelSpec hook_v1(double yaw_deg = 0.0)
{
  ToolModelSpec spec;
  spec.enabled = true;
  spec.mount_yaw_rad = yaw_deg * M_PI / 180.0;
  spec.mate_offset_m = -0.0005;
  spec.flange_axis_cad = {0.0, 1.0, 0.0};
  spec.insertion_cad = {0.0, 0.0, 1.0};
  spec.hook_up_cad = {0.0, -1.0, 0.0};
  spec.tcp_cad_m = {0.0, 0.110, 0.045};
  ToolBodySpec tongue;
  tongue.id = "tongue";
  tongue.size_m = {0.044, 0.005, 0.085};
  tongue.cad_pose.translation = {0.0, 0.110, 0.0325};
  ToolBodySpec disc;
  disc.id = "disc";
  disc.shape = BodyShape::CYLINDER;
  disc.size_m = {0.075, 0.075, 0.015};
  disc.cad_pose.rotation = normalized(Quaternion{-std::sqrt(0.5), 0.0, 0.0, std::sqrt(0.5)});
  spec.bodies = {tongue, disc};
  return spec;
}

void expect_vec(const Vector3 & a, const Vector3 & b)
{
  EXPECT_NEAR(a.x, b.x, kTol);
  EXPECT_NEAR(a.y, b.y, kTol);
  EXPECT_NEAR(a.z, b.z, kTol);
}

TEST(ToolModel, HookTcpSitsBelowFlangeAndPointsAlongTongue)
{
  const auto frames = compute_tool_frames(hook_v1());
  ASSERT_TRUE(frames.valid) << frames.message;
  // 挂环点：沿法兰轴离端面 110.5 mm，沿舌头方向（ee +X）45 mm。
  expect_vec(frames.ee_to_tcp.translation, {0.045, 0.0, 0.1105});
  // TCP z = 插入方向（舌头方向，垂直于法兰轴）；TCP x = 钩口方向（朝法兰，即 ee -Z）。
  expect_vec(rotate(frames.ee_to_tcp.rotation, {0.0, 0.0, 1.0}), {1.0, 0.0, 0.0});
  expect_vec(rotate(frames.ee_to_tcp.rotation, {1.0, 0.0, 0.0}), {0.0, 0.0, -1.0});
}

TEST(ToolModel, MountYawRotatesTongueAboutFlangeAxis)
{
  const auto frames = compute_tool_frames(hook_v1(90.0));
  ASSERT_TRUE(frames.valid);
  expect_vec(frames.ee_to_tcp.translation, {0.0, 0.045, 0.1105});
  expect_vec(rotate(frames.ee_to_tcp.rotation, {0.0, 0.0, 1.0}), {0.0, 1.0, 0.0});
}

TEST(ToolModel, BodiesAreExpressedInEeLinkWithScopedIds)
{
  const auto spec = hook_v1();
  const auto bodies = tool_bodies_in_ee(spec, compute_tool_frames(spec));
  ASSERT_EQ(bodies.size(), 2u);
  EXPECT_EQ(bodies[0].id, "mtc_moveit_planner/tongue");
  expect_vec(bodies[0].ee_to_payload.translation, {0.0325, 0.0, 0.1105});
  // 安装盘圆柱轴（局部 z）应沿法兰轴（ee +Z），中心在端面外 0.5 mm。
  EXPECT_EQ(bodies[1].shape, BodyShape::CYLINDER);
  expect_vec(rotate(bodies[1].ee_to_payload.rotation, {0.0, 0.0, 1.0}), {0.0, 0.0, 1.0});
  expect_vec(bodies[1].ee_to_payload.translation, {0.0, 0.0, 0.0005});
}

TEST(ToolModel, NonPerpendicularDirectionsAreRejected)
{
  auto spec = hook_v1();
  spec.insertion_cad = {0.0, 1.0, 1.0};
  EXPECT_FALSE(compute_tool_frames(spec).valid);
  EXPECT_TRUE(tool_bodies_in_ee(spec, compute_tool_frames(spec)).empty());
}

TEST(ToolModel, HookUpAlignmentKeepsInsertionAxisAndPointsHookUp)
{
  // 插入轴沿世界 +X，滚转任意（这里让局部 x 朝下）。
  const auto ring = quaternion_from_axes({0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0});
  const auto aligned = align_roll_to_up(ring, {0.0, 0.0, 1.0}, 30.0 * M_PI / 180.0);
  expect_vec(rotate(aligned, {0.0, 0.0, 1.0}), {1.0, 0.0, 0.0});
  expect_vec(rotate(aligned, {1.0, 0.0, 0.0}), {0.0, 0.0, 1.0});

  // 竖直插入（V2 从上往下）不调整。
  const auto down = quaternion_from_axes({1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, -1.0});
  const auto kept = align_roll_to_up(down, {0.0, 0.0, 1.0}, 30.0 * M_PI / 180.0);
  EXPECT_NEAR(angle_between(kept, down), 0.0, 1e-9);
}

}  // namespace
