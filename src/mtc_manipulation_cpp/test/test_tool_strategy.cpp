// Copyright 2026 meituan_challenge_ws contributors
//
// 工具策略 V1/V2 测试。
//
// 对齐 Python src/mtc_manipulation/test/test_pick_place_fsm.py 中关于
// 原语、解锁前置条件与 IO 通道的断言。

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "fakes.hpp"
#include "mtc_core_cpp/error_codes.hpp"
#include "mtc_manipulation_cpp/tool_strategy.hpp"

namespace
{

using mtc::core::ErrorCodes;
using mtc::manipulation::MagneticLatchV2Strategy;
using mtc::manipulation::PassiveHookV1Strategy;
using mtc::manipulation::TOOL_MAGNETIC_LATCH_V2;
using mtc::manipulation::TOOL_PASSIVE_HOOK_V1;
using mtc::manipulation::testing::make_pose_estimate;

mtc::core::PoseEstimate estimate()
{
  return make_pose_estimate("battery_1", 1);
}

}  // namespace

// ---------------------------------------------------------------------------
// V1：被动舌规
// ---------------------------------------------------------------------------

TEST(ToolStrategyV1, TypeAndPrimitives)
{
  PassiveHookV1Strategy strategy;
  EXPECT_EQ(strategy.type(), TOOL_PASSIVE_HOOK_V1);

  EXPECT_EQ(
    mtc::manipulation::primitive_names(strategy.make_acquire_plan(estimate())),
    std::vector<std::string>{"side_insert"});
  EXPECT_EQ(
    mtc::manipulation::primitive_names(strategy.make_engage_plan(estimate())),
    std::vector<std::string>{"hook_engage"});
  EXPECT_EQ(
    mtc::manipulation::primitive_names(strategy.make_release_plan("P1")),
    std::vector<std::string>{"unload"});
  EXPECT_EQ(
    mtc::manipulation::primitive_names(strategy.make_disengage_plan("P1")),
    std::vector<std::string>{"side_withdraw"});
}

TEST(ToolStrategyV1, SharedPrimitives)
{
  PassiveHookV1Strategy strategy;
  EXPECT_EQ(
    mtc::manipulation::primitive_names(strategy.make_test_lift_plan(estimate())),
    std::vector<std::string>{"test_lift"});
  EXPECT_EQ(
    mtc::manipulation::primitive_names(strategy.make_lift_plan(estimate())),
    std::vector<std::string>{"lift"});
  EXPECT_EQ(
    mtc::manipulation::primitive_names(strategy.make_transport_plan(estimate(), "P2")),
    std::vector<std::string>{"transport"});
  EXPECT_EQ(
    mtc::manipulation::primitive_names(strategy.make_seat_plan("P2")),
    std::vector<std::string>{"pre_seat"});
  EXPECT_EQ(
    mtc::manipulation::primitive_names(strategy.make_seat_contact_plan("P2")),
    std::vector<std::string>{"seat_contact"});
  EXPECT_EQ(
    mtc::manipulation::primitive_names(strategy.make_retreat_plan("P2")),
    std::vector<std::string>{"retreat"});
}

TEST(ToolStrategyV1, ContactPrimitivesAreMarkedAndSlow)
{
  PassiveHookV1Strategy strategy;

  const auto engage = strategy.make_engage_plan(estimate());
  ASSERT_EQ(engage.size(), 1u);
  // 接触敏感动作必须标记 requires_contact，且速度受约束
  EXPECT_TRUE(engage[0].requires_contact);
  EXPECT_LE(engage[0].max_linear_speed, PassiveHookV1Strategy::CONTACT_LINEAR_SPEED);

  const auto acquire = strategy.make_acquire_plan(estimate());
  ASSERT_EQ(acquire.size(), 1u);
  EXPECT_FALSE(acquire[0].requires_contact);
}

TEST(ToolStrategyV1, PrimitivePreservesSixDofTargetPose)
{
  PassiveHookV1Strategy strategy;
  const auto primitives = strategy.make_acquire_plan(estimate());

  ASSERT_EQ(primitives.size(), 1u);
  ASSERT_TRUE(primitives[0].target_pose.has_value());
  // 需求 5：不得退化成只有 XYZ —— frame_id 与姿态必须保留
  EXPECT_EQ(primitives[0].target_pose->frame_id, "base_link");
  EXPECT_TRUE(primitives[0].target_pose->is_usable());
  EXPECT_EQ(primitives[0].target_pose->position.x, 0.4);
  EXPECT_EQ(primitives[0].target_pose->orientation.w, 1.0);
}

TEST(ToolStrategyV1, NoUnlockPulseAndNoIoChannels)
{
  PassiveHookV1Strategy strategy;
  EXPECT_FALSE(strategy.requires_unlock_pulse());
  EXPECT_TRUE(strategy.io_channels().empty());
  EXPECT_FALSE(strategy.uses_io("electromagnetic_unlock"));

  // V1 无解锁机构：任何解锁请求都必须被拒绝
  const auto decision = strategy.evaluate_unlock(true, true);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(decision.error_code, ErrorCodes::UNLOCK_FAILED);
  EXPECT_NE(decision.reason.find("禁止任何解锁动作"), std::string::npos);

  // 即使前置条件不满足也不得放行
  EXPECT_FALSE(strategy.evaluate_unlock(false, false).allowed);
}

// ---------------------------------------------------------------------------
// V2：电磁锁止
// ---------------------------------------------------------------------------

TEST(ToolStrategyV2, TypeAndPrimitives)
{
  MagneticLatchV2Strategy strategy;
  EXPECT_EQ(strategy.type(), TOOL_MAGNETIC_LATCH_V2);

  EXPECT_EQ(
    mtc::manipulation::primitive_names(strategy.make_acquire_plan(estimate())),
    std::vector<std::string>{"top_insert"});
  EXPECT_EQ(
    mtc::manipulation::primitive_names(strategy.make_engage_plan(estimate())),
    std::vector<std::string>{"auto_lock"});
  EXPECT_EQ(
    mtc::manipulation::primitive_names(strategy.make_release_plan("P3")),
    std::vector<std::string>{"unload"});
  // V2 退出原语是 withdraw，绝不是 V1 的 side_withdraw
  EXPECT_EQ(
    mtc::manipulation::primitive_names(strategy.make_disengage_plan("P3")),
    std::vector<std::string>{"withdraw"});
}

TEST(ToolStrategyV2, RequiresUnlockAndDeclaresIoChannel)
{
  MagneticLatchV2Strategy strategy;
  EXPECT_TRUE(strategy.requires_unlock_pulse());
  EXPECT_TRUE(strategy.uses_io("electromagnetic_unlock"));
  EXPECT_FALSE(strategy.uses_io("some_other_channel"));
  EXPECT_EQ(strategy.unlock_pulse_ms(), MagneticLatchV2Strategy::DEFAULT_PULSE_MS);
}

TEST(ToolStrategyV2, UnlockRejectedWhenSeatNotVerified)
{
  MagneticLatchV2Strategy strategy;
  const auto decision = strategy.evaluate_unlock(false, true);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(decision.error_code, ErrorCodes::SEAT_NOT_VERIFIED);
  EXPECT_NE(decision.reason.find("禁止发出电磁解锁脉冲"), std::string::npos);
}

TEST(ToolStrategyV2, UnlockRejectedWhenNotUnloaded)
{
  MagneticLatchV2Strategy strategy;
  const auto decision = strategy.evaluate_unlock(true, false);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(decision.error_code, ErrorCodes::RELEASE_NOT_VERIFIED);
  EXPECT_NE(decision.reason.find("未确认卸载"), std::string::npos);
}

TEST(ToolStrategyV2, UnlockAllowedOnlyWhenBothConditionsHold)
{
  MagneticLatchV2Strategy strategy(250);
  const auto decision = strategy.evaluate_unlock(true, true);
  EXPECT_TRUE(decision.allowed);
  EXPECT_EQ(decision.error_code, ErrorCodes::OK);
  // 脉冲宽度必须来自策略配置，不得硬编码
  EXPECT_EQ(decision.pulse_ms, 250u);
}

TEST(ToolStrategyV2, SeatedFalseUnloadedFalseStillRejectsSeatedFirst)
{
  MagneticLatchV2Strategy strategy;
  const auto decision = strategy.evaluate_unlock(false, false);
  EXPECT_FALSE(decision.allowed);
  // 优先报告"未落座"（更根本的前置缺失）
  EXPECT_EQ(decision.error_code, ErrorCodes::SEAT_NOT_VERIFIED);
}

// ---------------------------------------------------------------------------
// 工厂
// ---------------------------------------------------------------------------

TEST(ToolStrategyFactory, CreatesBothGenerations)
{
  const auto v1 = mtc::manipulation::make_strategy(TOOL_PASSIVE_HOOK_V1);
  ASSERT_NE(v1, nullptr);
  EXPECT_EQ(v1->type(), TOOL_PASSIVE_HOOK_V1);
  EXPECT_FALSE(v1->requires_unlock_pulse());

  const auto v2 = mtc::manipulation::make_strategy(TOOL_MAGNETIC_LATCH_V2, 300);
  ASSERT_NE(v2, nullptr);
  EXPECT_EQ(v2->type(), TOOL_MAGNETIC_LATCH_V2);
  EXPECT_TRUE(v2->requires_unlock_pulse());
}

TEST(ToolStrategyFactory, UnknownTypeThrows)
{
  EXPECT_THROW(
    {mtc::manipulation::make_strategy("nonexistent_tool");},
    std::invalid_argument);
}

TEST(ToolStrategyFactory, EmptyStringThrows)
{
  EXPECT_THROW({mtc::manipulation::make_strategy("");}, std::invalid_argument);
}

// ---------------------------------------------------------------------------
// 原语在缺少提环位姿时仍可生成（但目标位姿为空）
// ---------------------------------------------------------------------------

TEST(ToolStrategy, PrimitiveWithoutUsableRingPoseHasNoTargetPose)
{
  PassiveHookV1Strategy strategy;
  // ring_pose_valid=False ⇒ 不得给出可用的目标位姿
  const auto primitives = strategy.make_acquire_plan(
    make_pose_estimate("battery_1", 1, false, true, false, false, true));

  ASSERT_EQ(primitives.size(), 1u);
  EXPECT_FALSE(primitives[0].target_pose.has_value());
  EXPECT_EQ(primitives[0].name, "side_insert");
}
