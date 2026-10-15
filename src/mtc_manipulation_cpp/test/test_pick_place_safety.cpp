// Copyright 2026 meituan_challenge_ws contributors
//
// PickPlace FSM 安全语义与故障路径测试。
//
// 逐条覆盖 pick_place_fsm.py 的八条安全语义与需求 5 的硬约束：
//   目标丢失 / 规划失败 / 看门狗超时 / 取消与停稳确认 / 状态未知锁定与重发禁止 /
//   挂载证据不足 / 放置稳定性 / placement_verified 唯一写入口。

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "fakes.hpp"
#include "mtc_core_cpp/error_codes.hpp"
#include "mtc_manipulation_cpp/pick_place_fsm.hpp"
#include "mtc_manipulation_cpp/tool_strategy.hpp"

namespace
{

using mtc::core::ErrorCodes;
using mtc::core::ManualClock;
using mtc::manipulation::PickPlaceConfig;
using mtc::manipulation::PickPlaceFsm;
using mtc::manipulation::PickPlaceState;
using mtc::manipulation::TOOL_PASSIVE_HOOK_V1;
using mtc::manipulation::testing::FakeMotion;
using mtc::manipulation::testing::FakePerception;
using mtc::manipulation::testing::FakePlanner;
using mtc::manipulation::testing::FakeTool;

struct Harness
{
  std::shared_ptr<ManualClock> clock = std::make_shared<ManualClock>(0.0);
  std::shared_ptr<FakePerception> perception = std::make_shared<FakePerception>();
  std::shared_ptr<FakePlanner> planner = std::make_shared<FakePlanner>();
  std::shared_ptr<FakeMotion> motion = std::make_shared<FakeMotion>();
  std::shared_ptr<FakeTool> tool = std::make_shared<FakeTool>();
  std::unique_ptr<PickPlaceFsm> fsm;

  Harness()
  {
    perception->clock = clock;
    PickPlaceConfig config;
    config.tool_type = TOOL_PASSIVE_HOOK_V1;
    config.placement_stability_sec = 3.0;
    config.state_timeouts = mtc::manipulation::default_state_timeouts();
    fsm = std::make_unique<PickPlaceFsm>(
      perception, planner, motion, tool, clock,
      mtc::manipulation::make_strategy(TOOL_PASSIVE_HOOK_V1), config);
  }

  void start()
  {
    fsm->start("battery_1", 1, "P1", "req-test");
  }

  /// 只推进状态、不推进时钟（用于"无超时"场景）。
  PickPlaceState advance_to(PickPlaceState target, int max_ticks = 200)
  {
    for (int i = 0; i < max_ticks; ++i) {
      if (fsm->state() == target) {
        return fsm->state();
      }
      if (mtc::manipulation::terminal_states().count(fsm->state()) == 1u) {
        return fsm->state();
      }
      fsm->step();
    }
    return fsm->state();
  }

  PickPlaceState run(double tick_seconds = 0.2, int max_ticks = 1200)
  {
    for (int i = 0; i < max_ticks; ++i) {
      fsm->step();
      if (mtc::manipulation::terminal_states().count(fsm->state()) == 1u) {
        break;
      }
      clock->advance(tick_seconds);
    }
    return fsm->state();
  }
};

/// 暴露 protected 的唯一写入口，用于验证"非 VERIFY_PLACED 不得写入"。
class ExposedFsm : public PickPlaceFsm
{
public:
  using PickPlaceFsm::PickPlaceFsm;
  void force_set_placement_verified() {set_placement_verified();}
};

}  // namespace

// ---------------------------------------------------------------------------
// 安全语义 1：placement_verified 唯一写入口
// ---------------------------------------------------------------------------

TEST(PickPlaceSafety, PlacementVerifiedCannotBeWrittenOutsideVerifyPlaced)
{
  Harness harness;
  PickPlaceConfig config;
  config.tool_type = TOOL_PASSIVE_HOOK_V1;
  config.state_timeouts = mtc::manipulation::default_state_timeouts();

  auto fsm = std::make_unique<ExposedFsm>(
    harness.perception, harness.planner, harness.motion, harness.tool, harness.clock,
    mtc::manipulation::make_strategy(TOOL_PASSIVE_HOOK_V1), config);
  fsm->start("battery_1", 1, "P1", "req-test");
  ASSERT_EQ(fsm->state(), PickPlaceState::RESOLVE_TARGET);

  // 违规写入必须立刻失败，而不是"静默成功"
  EXPECT_THROW(fsm->force_set_placement_verified(), std::logic_error);
  EXPECT_FALSE(fsm->placement_verified());
}

TEST(PickPlaceSafety, PlacementVerifiedOnlyTrueAfterSuccessfulVerifyPlaced)
{
  Harness harness;
  harness.start();
  harness.run();

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::SUCCESS);
  EXPECT_TRUE(harness.fsm->placement_verified());
  // 成功路径必须真的经过 VERIFY_PLACED
  const auto & history = harness.fsm->states_history();
  EXPECT_NE(
    std::find(history.begin(), history.end(), PickPlaceState::VERIFY_PLACED), history.end());
}

// ---------------------------------------------------------------------------
// 感知失败
// ---------------------------------------------------------------------------

TEST(PickPlaceSafety, TargetNotFoundIsRecoverableThenFails)
{
  Harness harness;
  harness.perception->found = false;
  harness.start();
  const auto state = harness.run(0.1, 300);

  EXPECT_EQ(state, PickPlaceState::FAILED);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::TARGET_NOT_FOUND);
  EXPECT_FALSE(harness.fsm->placement_verified());
  EXPECT_TRUE(harness.motion->commands.empty());
}

TEST(PickPlaceSafety, NonUniqueTargetIsRejected)
{
  Harness harness;
  harness.perception->unique = false;
  harness.start();
  harness.run(0.1, 300);

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAILED);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::TARGET_NOT_FOUND);
}

TEST(PickPlaceSafety, StalePoseIsRejectedWithPoseStale)
{
  Harness harness;
  harness.perception->stale = true;
  harness.start();
  harness.run(0.1, 300);

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAILED);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::POSE_STALE);
}

TEST(PickPlaceSafety, InvalidRingPoseBlocksMotion)
{
  Harness harness;
  harness.perception->ring_pose_valid = false;
  harness.start();
  harness.run(0.1, 300);

  // ring_pose_valid=false 时禁止生成运动目标
  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAILED);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::TARGET_NOT_FOUND);
  EXPECT_TRUE(harness.motion->commands.empty());
}

TEST(PickPlaceSafety, UnknownColorIsRejectedEvenWhenExpected)
{
  Harness harness;
  harness.perception->color = 0;   // COLOR_UNKNOWN
  harness.start();
  harness.run(0.1, 300);

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAILED);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::TARGET_NOT_FOUND);
  EXPECT_TRUE(harness.motion->commands.empty());
}

TEST(PickPlaceSafety, ColorMismatchIsRejectedToPreventWrongPick)
{
  Harness harness;
  harness.perception->color = 2;   // 期望 1
  harness.start();
  harness.run(0.1, 300);

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAILED);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::TARGET_NOT_FOUND);
  EXPECT_TRUE(harness.motion->commands.empty());
  EXPECT_NE(harness.fsm->message().find("拒绝抓错颜色"), std::string::npos);
}

// ---------------------------------------------------------------------------
// 规划失败
// ---------------------------------------------------------------------------

TEST(PickPlaceSafety, PlanningFailureIsRecoverable)
{
  Harness harness;
  harness.planner->ok = false;
  harness.planner->error_code = ErrorCodes::PLANNING_FAILED;
  harness.planner->message = "no IK solution";
  harness.start();
  harness.run(0.1, 300);

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAILED);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::PLANNING_FAILED);
  EXPECT_TRUE(harness.motion->commands.empty());
  // 规划失败不得进入 MOVE_PRE_ALIGN
  const auto & history = harness.fsm->states_history();
  EXPECT_EQ(
    std::find(history.begin(), history.end(), PickPlaceState::MOVE_PRE_ALIGN), history.end());
}

TEST(PickPlaceSafety, TransportPlanningFailureGoesToFaultAfterAttachment)
{
  Harness harness;
  harness.start();
  // 先通过接近段规划
  harness.advance_to(PickPlaceState::MOVE_PRE_ALIGN);
  ASSERT_EQ(harness.fsm->state(), PickPlaceState::MOVE_PRE_ALIGN);

  // 挂载之后的搬运规划失败：必须 FAULT（不允许继续搬运）
  harness.planner->ok = false;
  harness.planner->error_code = ErrorCodes::PLANNING_FAILED;
  harness.planner->message = "post-attach planning failed";
  harness.run(0.2, 600);

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::PLANNING_FAILED);
}

// ---------------------------------------------------------------------------
// 看门狗超时
// ---------------------------------------------------------------------------

TEST(PickPlaceSafety, NonMotionStateTimeoutReportsMotionTimeout)
{
  Harness harness;
  harness.perception->always_pending = true;   // 感知永不返回
  harness.start();
  ASSERT_EQ(harness.fsm->state(), PickPlaceState::RESOLVE_TARGET);

  // RESOLVE_TARGET 超时 2.0s：非运动状态 -> MOTION_TIMEOUT(220) 且可恢复。
  //
  // D-2: return state, internal state, history and recovery context must agree.
  harness.clock->advance(2.1);
  const auto result = harness.fsm->step();

  EXPECT_EQ(result.state, PickPlaceState::RECOVERY);
  EXPECT_EQ(result.error_code, ErrorCodes::MOTION_TIMEOUT);
  EXPECT_TRUE(result.resumable);
  EXPECT_EQ(harness.fsm->state(), PickPlaceState::RECOVERY);
  EXPECT_EQ(harness.fsm->states_history().back(), PickPlaceState::RECOVERY);
  harness.fsm->step();
  EXPECT_EQ(harness.fsm->state(), PickPlaceState::RESOLVE_TARGET);
  EXPECT_EQ(harness.fsm->recovery_attempts(), 1);
  EXPECT_EQ(harness.fsm->step().state, PickPlaceState::RESOLVE_TARGET);
  // 非运动在途状态不锁定重发
  EXPECT_TRUE(harness.fsm->motion_resend_allowed());
}

TEST(PickPlaceSafety, MotionInFlightTimeoutGoesToFault230AndLocksResend)
{
  Harness harness;
  harness.motion->pending_ticks = 1000000;   // 运动端口永不返回
  harness.start();
  harness.advance_to(PickPlaceState::MOVE_PRE_ALIGN);
  ASSERT_EQ(harness.fsm->state(), PickPlaceState::MOVE_PRE_ALIGN);
  harness.fsm->step();   // 发令并进入等待

  // MOVE_PRE_ALIGN 超时 30.0s：运动在途 -> 状态未知 -> FAULT(230)
  harness.clock->advance(30.1);
  harness.fsm->step();

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::MOTION_STATUS_UNKNOWN);
  EXPECT_FALSE(harness.fsm->motion_resend_allowed());
  EXPECT_NE(harness.fsm->message().find("禁止重发"), std::string::npos);
}

TEST(PickPlaceSafety, NoTimeoutBeforeDeadline)
{
  Harness harness;
  harness.perception->always_pending = true;
  harness.start();

  harness.clock->advance(1.9);
  const auto before = harness.fsm->step();
  EXPECT_EQ(before.state, PickPlaceState::RESOLVE_TARGET);   // 仍在等待

  harness.clock->advance(0.2);
  const auto after = harness.fsm->step();
  EXPECT_EQ(after.state, PickPlaceState::RECOVERY);          // 已超时
  EXPECT_EQ(after.error_code, ErrorCodes::MOTION_TIMEOUT);
}

// ---------------------------------------------------------------------------
// 安全语义 7：运动状态未知 -> FAULT 且禁止自动重发
// ---------------------------------------------------------------------------

TEST(PickPlaceSafety, MotionStatusUnknownLocksAndDoesNotResend)
{
  Harness harness;
  harness.motion->unknown_primitives = {"side_insert"};
  harness.start();
  harness.advance_to(PickPlaceState::MOVE_PRE_ALIGN);
  harness.fsm->step();

  ASSERT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::MOTION_STATUS_UNKNOWN);
  EXPECT_FALSE(harness.fsm->motion_resend_allowed());

  // 锁定后继续 step 不得产生任何新的运动命令
  const size_t executes = harness.motion->execute_calls;
  for (int i = 0; i < 20; ++i) {
    harness.fsm->step();
  }
  EXPECT_EQ(harness.motion->execute_calls, executes);
  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
}

TEST(PickPlaceSafety, MotionTimeoutLocksAndDoesNotResend)
{
  Harness harness;
  harness.motion->timeout_primitives = {"side_insert"};
  harness.start();
  harness.advance_to(PickPlaceState::MOVE_PRE_ALIGN);
  harness.fsm->step();

  ASSERT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::MOTION_TIMEOUT);
  EXPECT_FALSE(harness.fsm->motion_resend_allowed());
}

TEST(PickPlaceSafety, InFlightCommandIsReusedNeverResent)
{
  Harness harness;
  harness.motion->pending_ticks = 2;   // 前两次返回"在途"
  harness.start();

  harness.fsm->step();   // RESOLVE_TARGET -> PLAN_ACQUIRE
  harness.fsm->step();   // PLAN_ACQUIRE -> MOVE_PRE_ALIGN
  ASSERT_EQ(harness.fsm->state(), PickPlaceState::MOVE_PRE_ALIGN);

  harness.fsm->step();   // 在途，等待
  EXPECT_EQ(harness.fsm->state(), PickPlaceState::MOVE_PRE_ALIGN);
  harness.fsm->step();   // 仍在途
  EXPECT_EQ(harness.fsm->state(), PickPlaceState::MOVE_PRE_ALIGN);
  harness.fsm->step();   // 成功 -> ENGAGE
  EXPECT_EQ(harness.fsm->state(), PickPlaceState::ENGAGE);

  EXPECT_EQ(harness.motion->execute_calls, 3);
  ASSERT_EQ(harness.motion->command_ids.size(), 3u);
  // 三次调用必须复用同一条命令：绝不因为"等待"而重新发令
  EXPECT_EQ(harness.motion->command_ids[0], harness.motion->command_ids[1]);
  EXPECT_EQ(harness.motion->command_ids[1], harness.motion->command_ids[2]);
}

TEST(PickPlaceSafety, ProgressIsNotBlockedByInFlightPorts)
{
  // 运动在途时 step() 必须继续返回"当前状态"而不是异常或终态。
  Harness harness;
  harness.motion->pending_ticks = 5;
  harness.start();
  harness.advance_to(PickPlaceState::MOVE_PRE_ALIGN);

  for (int i = 0; i < 3; ++i) {
    const auto result = harness.fsm->step();
    EXPECT_EQ(result.state, PickPlaceState::MOVE_PRE_ALIGN);
    EXPECT_FALSE(result.terminal());
  }
}

// ---------------------------------------------------------------------------
// 安全语义 3 / 2：ENGAGE 与挂载证据
// ---------------------------------------------------------------------------

TEST(PickPlaceSafety, EngageFailureGoesToEngageFailed300)
{
  Harness harness;
  harness.tool->engage_ok = false;
  harness.tool->engage_error_code = ErrorCodes::ENGAGE_FAILED;
  harness.tool->engage_message = "穿环未完成";
  harness.start();
  harness.run(0.1, 400);

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAILED);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::ENGAGE_FAILED);
  // 未挂载不得进入 LIFT / TRANSPORT
  const auto & history = harness.fsm->states_history();
  EXPECT_EQ(std::find(history.begin(), history.end(), PickPlaceState::LIFT), history.end());
  EXPECT_EQ(
    std::find(history.begin(), history.end(), PickPlaceState::TRANSPORT), history.end());
}

TEST(PickPlaceSafety, InsufficientEngageEvidenceIsRejected)
{
  Harness harness;
  // 只有指令级证据：不得据此认定接合成立
  harness.tool->engage_evidence_level = mtc::core::EvidenceLevel::COMMAND_ONLY;
  harness.start();
  harness.run(0.1, 400);

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAILED);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::ENGAGE_FAILED);
  EXPECT_NE(harness.fsm->message().find("证据等级不足"), std::string::npos);
}

TEST(PickPlaceSafety, UnexpectedToolStateAfterEngageIsHardwareFault)
{
  Harness harness;
  harness.tool->engage_state = mtc::core::ToolStateCode::STATE_DETACHED;
  harness.start();
  harness.run(0.1, 400);

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::HARDWARE_FAULT);
}

TEST(PickPlaceSafety, AttachNotVerifiedBlocksLiftAndTransport)
{
  Harness harness;
  harness.tool->test_lift_ok = true;
  harness.tool->test_lift_evidence_level = mtc::core::EvidenceLevel::COMMAND_ONLY;
  harness.perception->followed = false;   // 视觉随动也无法补足证据
  harness.start();
  harness.run(0.1, 400);

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAILED);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::ATTACH_NOT_VERIFIED);
  EXPECT_FALSE(harness.fsm->object_attached_estimated());
  const auto & history = harness.fsm->states_history();
  EXPECT_EQ(std::find(history.begin(), history.end(), PickPlaceState::LIFT), history.end());
}

TEST(PickPlaceSafety, VisionFollowCanCompensateForWeakEvidence)
{
  Harness harness;
  harness.tool->test_lift_evidence_level = mtc::core::EvidenceLevel::COMMAND_ONLY;
  harness.perception->followed = true;   // 视觉确认物体随工具运动
  harness.start();
  harness.run();

  // 独立视觉证据成立时允许继续
  EXPECT_EQ(harness.fsm->state(), PickPlaceState::SUCCESS);
}

TEST(PickPlaceSafety, TestLiftFailureIsReported)
{
  Harness harness;
  harness.tool->test_lift_ok = false;
  harness.tool->test_lift_error_code = ErrorCodes::ATTACH_NOT_VERIFIED;
  harness.tool->test_lift_message = "试提失败";
  harness.start();
  harness.run(0.1, 400);

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAILED);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::ATTACH_NOT_VERIFIED);
}

TEST(PickPlaceSafety, HoldingStateBlocksAutomaticRetry)
{
  Harness harness;
  harness.tool->engage_ok = false;
  harness.tool->engage_error_code = ErrorCodes::ENGAGE_FAILED;
  // 工具可能已挂载物体：不得自动重试
  harness.tool->holding = "holding";
  harness.start();
  harness.run(0.1, 400);

  // 不可恢复 -> 直接 FAILED（而不是进入 RECOVERY 重试）
  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAILED);
  EXPECT_EQ(harness.fsm->recovery_attempts(), 0);
}

TEST(PickPlaceSafety, UnknownHoldingStateIsConservative)
{
  Harness harness;
  harness.tool->engage_ok = false;
  harness.tool->holding = "unknown";
  harness.start();
  harness.run(0.1, 400);

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAILED);
  EXPECT_EQ(harness.fsm->recovery_attempts(), 0);
}

// ---------------------------------------------------------------------------
// 安全语义 4：SEAT 未确认桌面承托
// ---------------------------------------------------------------------------

TEST(PickPlaceSafety, SeatNotVerifiedBlocksUnload)
{
  Harness harness;
  harness.tool->seat_verify_ok = false;
  harness.tool->seated = false;
  harness.tool->seat_error_code = ErrorCodes::SEAT_NOT_VERIFIED;
  harness.start();
  harness.run(0.1, 600);

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::SEAT_NOT_VERIFIED);
  // 未确认承托禁止进入 UNLOAD / DISENGAGE
  EXPECT_EQ(harness.tool->unload_calls, 0);
  EXPECT_EQ(harness.tool->disengage_calls, 0);
  EXPECT_EQ(harness.tool->unlock_calls, 0);
  const auto & history = harness.fsm->states_history();
  EXPECT_EQ(std::find(history.begin(), history.end(), PickPlaceState::UNLOAD), history.end());
}

// ---------------------------------------------------------------------------
// 安全语义 8：放置稳定性窗口
// ---------------------------------------------------------------------------

TEST(PickPlaceSafety, PlacementNotAtTargetFailsAfterWindow)
{
  Harness harness;
  harness.perception->placement_at_target = false;
  harness.start();
  harness.run(0.2, 600);

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAILED);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::PLACEMENT_FAILED);
  EXPECT_FALSE(harness.fsm->placement_verified());
  EXPECT_NE(harness.fsm->message().find("核验窗口耗尽"), std::string::npos);
}

TEST(PickPlaceSafety, PlacementStaleSampleResetsStabilityTimer)
{
  Harness harness;
  harness.perception->placement_stale = true;
  harness.start();
  harness.run(0.2, 600);

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAILED);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::PLACEMENT_FAILED);
  EXPECT_FALSE(harness.fsm->placement_verified());
}

TEST(PickPlaceSafety, PlacementNotSettledPreventsSuccess)
{
  Harness harness;
  harness.perception->placement_settled = false;
  harness.start();
  harness.run(0.2, 600);

  EXPECT_FALSE(harness.fsm->placement_verified());
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::PLACEMENT_FAILED);
}

TEST(PickPlaceSafety, PlacementFoundFalsePreventsSuccess)
{
  Harness harness;
  harness.perception->placement_found = false;
  harness.start();
  harness.run(0.2, 600);

  EXPECT_FALSE(harness.fsm->placement_verified());
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::PLACEMENT_FAILED);
}

TEST(PickPlaceSafety, VerifyPlacementDisabledNeverClaimsSuccess)
{
  Harness harness;
  PickPlaceConfig config;
  config.tool_type = TOOL_PASSIVE_HOOK_V1;
  config.verify_placement = false;
  config.state_timeouts = mtc::manipulation::default_state_timeouts();
  auto fsm = std::make_unique<PickPlaceFsm>(
    harness.perception, harness.planner, harness.motion, harness.tool, harness.clock,
    mtc::manipulation::make_strategy(TOOL_PASSIVE_HOOK_V1), config);
  fsm->start("battery_1", 1, "P1", "req-test");

  for (int i = 0; i < 400; ++i) {
    fsm->step();
    if (mtc::manipulation::terminal_states().count(fsm->state()) == 1u) {
      break;
    }
    harness.clock->advance(0.2);
  }

  // 关闭验证时不得声明放置成功
  EXPECT_NE(fsm->state(), PickPlaceState::SUCCESS);
  EXPECT_FALSE(fsm->placement_verified());
  EXPECT_EQ(fsm->error_code(), ErrorCodes::PLACEMENT_FAILED);
}

TEST(PickPlaceSafety, SuccessRequiresFullStabilityWindow)
{
  Harness harness;
  harness.start();

  // 进入 VERIFY_PLACED 后只推进不足 3.0s：不得成功
  harness.advance_to(PickPlaceState::VERIFY_PLACED);
  ASSERT_EQ(harness.fsm->state(), PickPlaceState::VERIFY_PLACED);

  for (int i = 0; i < 10; ++i) {   // 10 × 0.2 = 2.0s < 3.0s
    harness.fsm->step();
    harness.clock->advance(0.2);
  }
  EXPECT_EQ(harness.fsm->state(), PickPlaceState::VERIFY_PLACED);
  EXPECT_FALSE(harness.fsm->placement_verified());

  // 补足窗口后才允许 SUCCESS
  harness.fsm->step();
  harness.clock->advance(0.2);
  for (int i = 0; i < 10; ++i) {
    harness.fsm->step();
    if (harness.fsm->state() == PickPlaceState::SUCCESS) {
      break;
    }
    harness.clock->advance(0.2);
  }
  EXPECT_EQ(harness.fsm->state(), PickPlaceState::SUCCESS);
  EXPECT_TRUE(harness.fsm->placement_verified());
}

// ---------------------------------------------------------------------------
// 安全语义 6：取消与停稳确认
// ---------------------------------------------------------------------------

TEST(PickPlaceSafety, CancelWithConfirmedStopReachesCancelled)
{
  Harness harness;
  harness.start();
  ASSERT_EQ(harness.fsm->state(), PickPlaceState::RESOLVE_TARGET);

  harness.fsm->request_cancel("test cancel");
  harness.fsm->step();

  // 与 Python 一致：取消请求在同一 step 内驱动 CANCEL_PENDING 的处理器；
  // 停稳确认成立时该 step 即到达 CANCELLED（CANCEL_PENDING 仍会进入历史）。
  EXPECT_EQ(harness.fsm->state(), PickPlaceState::CANCELLED);
  EXPECT_TRUE(harness.fsm->stop_confirmed());
  EXPECT_EQ(harness.motion->stop_calls, 1);

  const auto & history = harness.fsm->states_history();
  EXPECT_NE(
    std::find(history.begin(), history.end(), PickPlaceState::CANCEL_PENDING), history.end());
  EXPECT_FALSE(harness.fsm->placement_verified());
  EXPECT_FALSE(harness.fsm->result().success);
}

TEST(PickPlaceSafety, CancelWithoutStopConfirmationGoesToFault520)
{
  Harness harness;
  harness.motion->stop_confirms = false;
  harness.motion->stop_message = "停止请求已送达，但尚未确认停稳";
  harness.start();
  harness.fsm->request_cancel("test cancel");
  harness.fsm->step();

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::CANCEL_NOT_CONFIRMED);
  EXPECT_FALSE(harness.fsm->stop_confirmed());
  // 未确认停稳时禁止自动重发
  EXPECT_FALSE(harness.fsm->motion_resend_allowed());

  // 收到 cancel 不得立即声称安全停止：CANCEL_PENDING 之后绝不得出现 CANCELLED
  const auto & history = harness.fsm->states_history();
  const auto cancel_it =
    std::find(history.begin(), history.end(), PickPlaceState::CANCEL_PENDING);
  ASSERT_NE(cancel_it, history.end());
  EXPECT_EQ(
    std::find(cancel_it, history.end(), PickPlaceState::CANCELLED), history.end());
}

TEST(PickPlaceSafety, CancelNeverReachesCancelledWithoutConfirmation)
{
  Harness harness;
  harness.motion->stop_confirms = false;
  harness.start();
  harness.fsm->request_cancel("cancel");

  // 即使反复 step，也不得在未确认停稳的情况下进入 CANCELLED
  for (int i = 0; i < 10; ++i) {
    harness.fsm->step();
    EXPECT_NE(harness.fsm->state(), PickPlaceState::CANCELLED);
  }
  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
}

TEST(PickPlaceSafety, RequestStopIsCalledAtMostOnce)
{
  Harness harness;
  harness.start();
  harness.fsm->request_cancel("cancel");
  harness.fsm->step();

  for (int i = 0; i < 5; ++i) {
    harness.fsm->step();
  }
  EXPECT_EQ(harness.motion->stop_calls, 1);
}

TEST(PickPlaceSafety, CancelInTerminalStateIsIgnored)
{
  Harness harness;
  harness.start();
  harness.run();
  ASSERT_EQ(harness.fsm->state(), PickPlaceState::SUCCESS);

  const auto before = harness.motion->stop_calls;
  harness.fsm->request_cancel("late cancel");
  harness.fsm->step();

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::SUCCESS);
  EXPECT_EQ(harness.motion->stop_calls, before);
}

TEST(PickPlaceSafety, CancelDuringMotionStillRequiresConfirmation)
{
  Harness harness;
  harness.motion->stop_confirms = false;
  harness.start();
  harness.advance_to(PickPlaceState::MOVE_PRE_ALIGN);
  ASSERT_EQ(harness.fsm->state(), PickPlaceState::MOVE_PRE_ALIGN);

  harness.fsm->request_cancel("cancel during motion");
  harness.fsm->step();

  // 运动在途时取消同样必须先确认停稳
  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::CANCEL_NOT_CONFIRMED);
  EXPECT_EQ(harness.motion->stop_calls, 1);
}

// ---------------------------------------------------------------------------
// 恢复预算
// ---------------------------------------------------------------------------

TEST(PickPlaceSafety, RecoveryRetriesThenFails)
{
  Harness harness;
  harness.perception->found = false;
  harness.start();
  harness.run(0.1, 400);

  // 感知失败可重试，但预算耗尽后必须 FAILED
  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAILED);
  const auto & history = harness.fsm->states_history();
  const auto recoveries = std::count(
    history.begin(), history.end(), PickPlaceState::RECOVERY);
  EXPECT_GE(recoveries, 1);
}

TEST(PickPlaceSafety, RecoveryIsRefusedWhenMotionResendDisallowed)
{
  Harness harness;
  harness.motion->unknown_primitives = {"side_insert"};
  harness.start();
  harness.advance_to(PickPlaceState::MOVE_PRE_ALIGN);
  harness.fsm->step();

  // 状态未知后即使外部强制回到 RECOVERY，也不得重发
  ASSERT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
  EXPECT_FALSE(harness.fsm->motion_resend_allowed());

  const size_t executes = harness.motion->execute_calls;
  for (int i = 0; i < 10; ++i) {
    harness.fsm->step();
  }
  EXPECT_EQ(harness.motion->execute_calls, executes);
}

TEST(PickPlaceAudit, RepeatedFrameForFiveSecondsCannotSucceed)
{
  Harness h;
  h.perception->fixed_stamp = 0.0;
  h.start();
  ASSERT_EQ(h.advance_to(PickPlaceState::VERIFY_PLACED), PickPlaceState::VERIFY_PLACED);
  for (int i = 0; i <= 25; ++i) {
    h.fsm->step();
    h.clock->advance(0.2);
  }
  EXPECT_FALSE(h.fsm->placement_verified());
  EXPECT_NE(h.fsm->state(), PickPlaceState::SUCCESS);
}

TEST(PickPlaceAudit, MissingPlanOutputNeverExecutesZeroTarget)
{
  Harness h;
  h.planner->provide_commands = false;
  h.start();
  h.run();
  EXPECT_TRUE(h.motion->commands.empty());
  EXPECT_FALSE(h.fsm->result().success);
}
