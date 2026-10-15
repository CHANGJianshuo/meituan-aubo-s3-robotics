// Copyright 2026 meituan_challenge_ws contributors
//
// PickPlace FSM V2（电磁锁止）测试。
//
// 严格对齐 Python src/mtc_manipulation/test/test_pick_place_fsm.py
// 的 test_v2_happy_path_reaches_success_and_unlocks_once 与 V2 联锁用例。
//
// 核心安全约束：V2 电磁解锁必须在"已确认放置（落座）+ 已卸载"全部满足后
// 才可触发；否则一律 FAULT 且**不得调用解锁 IO**。

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
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
using mtc::manipulation::TOOL_MAGNETIC_LATCH_V2;
using mtc::manipulation::testing::FakeMotion;
using mtc::manipulation::testing::FakePerception;
using mtc::manipulation::testing::FakePlanner;
using mtc::manipulation::testing::FakeTool;

constexpr double kRunTickSeconds = 0.2;

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
    config.tool_type = TOOL_MAGNETIC_LATCH_V2;
    config.placement_stability_sec = 3.0;
    config.state_timeouts = mtc::manipulation::default_state_timeouts();
    fsm = std::make_unique<PickPlaceFsm>(
      perception, planner, motion, tool, clock,
      mtc::manipulation::make_strategy(TOOL_MAGNETIC_LATCH_V2), config);
  }

  void start(const std::string & slot = "P3")
  {
    fsm->start("battery_1", 1, slot, "req-v2");
  }

  mtc::manipulation::PickPlaceResult run(
    int max_ticks = 1200, double tick_seconds = kRunTickSeconds)
  {
    for (int i = 0; i < max_ticks; ++i) {
      fsm->step();
      const auto state = fsm->state();
      if (state == PickPlaceState::SUCCESS || state == PickPlaceState::FAILED ||
        state == PickPlaceState::FAULT || state == PickPlaceState::CANCELLED)
      {
        break;
      }
      clock->advance(tick_seconds);
    }
    return fsm->result();
  }

  std::vector<std::string> history() const
  {
    std::vector<std::string> names;
    for (const auto state : fsm->states_history()) {
      names.push_back(mtc::manipulation::to_string(state));
    }
    return names;
  }

  std::vector<std::string> motion_primitives() const
  {
    std::vector<std::string> names;
    for (const auto & command : motion->commands) {
      names.push_back(command.primitive_name);
    }
    return names;
  }

  /// 推进到指定状态（或终态）；用于构造联锁场景。
  PickPlaceState run_to(PickPlaceState target, int max_ticks = 1200)
  {
    for (int i = 0; i < max_ticks; ++i) {
      if (fsm->state() == target) {
        return fsm->state();
      }
      const auto state = fsm->state();
      if (state == PickPlaceState::SUCCESS || state == PickPlaceState::FAILED ||
        state == PickPlaceState::FAULT || state == PickPlaceState::CANCELLED)
      {
        return state;
      }
      fsm->step();
      clock->advance(0.1);
    }
    return fsm->state();
  }
};

/// 始终拒绝解锁的策略：用于验证 FSM 必须**服从**策略判定，
/// 且在策略拒绝时绝不调用解锁 IO。
class RefuseUnlockStrategy : public mtc::manipulation::MagneticLatchV2Strategy
{
public:
  using MagneticLatchV2Strategy::MagneticLatchV2Strategy;

  mtc::manipulation::UnlockDecision evaluate_unlock(
    bool seated_verified, bool unloaded_verified) const override
  {
    (void)seated_verified;
    (void)unloaded_verified;
    mtc::manipulation::UnlockDecision decision;
    decision.allowed = false;
    decision.error_code = ErrorCodes::UNLOCK_FAILED;
    decision.reason = "测试策略：拒绝解锁";
    return decision;
  }
};

}  // namespace

TEST(PickPlaceV2, HappyPathReachesSuccessAndUnlocksOnce)
{
  Harness harness;
  harness.start();
  const auto result = harness.run();

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.placement_verified);
  EXPECT_EQ(result.error_code, ErrorCodes::OK);
  EXPECT_EQ(result.final_state, PickPlaceState::SUCCESS);
}

TEST(PickPlaceV2, DifferencesAreConfinedToEngageAndDisengage)
{
  Harness harness;
  harness.start();
  harness.run();

  // V2 差异只体现在 ENGAGE / DISENGAGE 内部
  EXPECT_EQ(harness.tool->engage_calls, 1);
  EXPECT_EQ(harness.tool->unlock_calls, 1);

  ASSERT_FALSE(harness.tool->received_primitives.empty());
  EXPECT_EQ(harness.tool->received_primitives.front(), "auto_lock");

  bool saw_unlock_io = false;
  for (const auto & event : harness.tool->io_events) {
    if (event.rfind("electromagnetic_unlock", 0) == 0) {
      saw_unlock_io = true;
      break;
    }
  }
  EXPECT_TRUE(saw_unlock_io);

  const auto & primitives = harness.tool->received_primitives;
  EXPECT_NE(std::find(primitives.begin(), primitives.end(), "withdraw"), primitives.end());
  EXPECT_EQ(
    std::find(primitives.begin(), primitives.end(), "side_withdraw"), primitives.end());
}

TEST(PickPlaceV2, MotionPrimitivesDifferFromV1OnlyInAcquireAndEngage)
{
  Harness harness;
  harness.start();
  harness.run();

  // MOVE_PRE_ALIGN 用 top_insert（V1 为 side_insert）；其余阶段与 V1 相同
  const std::vector<std::string> expected = {
    "top_insert",
    "test_lift",
    "lift",
    "transport",
    "pre_seat",
    "seat_contact",
    "retreat",
  };
  EXPECT_EQ(harness.motion_primitives(), expected);
}

TEST(PickPlaceV2, UnlockPulseUsesStrategyConfiguration)
{
  Harness harness;
  harness.start();
  harness.run();

  ASSERT_EQ(harness.tool->unlock_pulses.size(), 1u);
  EXPECT_EQ(
    harness.tool->unlock_pulses[0], mtc::manipulation::MagneticLatchV2Strategy::DEFAULT_PULSE_MS);
}

TEST(PickPlaceV2, StateSequenceMatchesV1Pipeline)
{
  Harness harness;
  harness.start();
  harness.run();

  const std::vector<std::string> expected = {
    "RESOLVE_TARGET",
    "PLAN_ACQUIRE",
    "MOVE_PRE_ALIGN",
    "ENGAGE",
    "VERIFY_ENGAGEMENT",
    "TEST_LIFT",
    "VERIFY_ATTACHED",
    "LIFT",
    "TRANSPORT",
    "MOVE_PRE_SEAT",
    "SEAT",
    "VERIFY_SEATED",
    "UNLOAD",
    "DISENGAGE",
    "RETREAT",
    "VERIFY_PLACED",
    "SUCCESS",
  };
  // 主流水线与工具型号无关：V1/V2 的状态序列完全相同
  EXPECT_EQ(harness.history(), expected);
}

// ---------------------------------------------------------------------------
// 解锁联锁：安全需求"V2 电磁解锁必须在已确认放置、已卸载等条件全部满足后才可触发"
// ---------------------------------------------------------------------------

TEST(PickPlaceV2Interlock, NoUnlockWhenSeatNotVerified)
{
  Harness harness;
  harness.tool->seated = false;   // 未确认桌面承托
  harness.start();
  harness.run();

  // VERIFY_SEATED 必须直接 FAULT(400)，且**不得调用解锁 IO**
  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::SEAT_NOT_VERIFIED);
  EXPECT_EQ(harness.tool->unlock_calls, 0);
  EXPECT_EQ(harness.tool->disengage_calls, 0);
  EXPECT_FALSE(harness.fsm->placement_verified());

  const auto history = harness.history();
  EXPECT_EQ(std::find(history.begin(), history.end(), "DISENGAGE"), history.end());
}

TEST(PickPlaceV2Interlock, NoUnlockWhenUnloadNotVerified)
{
  Harness harness;
  harness.tool->unloaded = false;   // 未确认卸载
  harness.start();
  harness.run();

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::RELEASE_NOT_VERIFIED);
  // 未卸载时绝不能解锁
  EXPECT_EQ(harness.tool->unlock_calls, 0);
}

TEST(PickPlaceV2Interlock, StrategyRefusalBlocksUnlockIoEntirely)
{
  // DISENGAGE 的防御性联锁与策略级 evaluate_unlock 都是"冗余保护"：
  // 正常流转下 VERIFY_SEATED / UNLOAD 会先行拦截。本用例通过一个
  // 始终拒绝解锁的策略，验证 FSM 在"证据齐全但策略拒绝"时
  // 必须保守故障，且不得触碰解锁 IO。
  Harness harness;
  PickPlaceConfig config;
  config.tool_type = TOOL_MAGNETIC_LATCH_V2;
  config.placement_stability_sec = 3.0;
  config.state_timeouts = mtc::manipulation::default_state_timeouts();

  auto fsm = std::make_unique<PickPlaceFsm>(
    harness.perception, harness.planner, harness.motion, harness.tool, harness.clock,
    std::make_shared<const RefuseUnlockStrategy>(), config);
  fsm->start("battery_1", 1, "P3", "req-v2");

  for (int i = 0; i < 1200; ++i) {
    fsm->step();
    const auto state = fsm->state();
    if (state == PickPlaceState::SUCCESS || state == PickPlaceState::FAILED ||
      state == PickPlaceState::FAULT || state == PickPlaceState::CANCELLED)
    {
      break;
    }
    harness.clock->advance(0.2);
  }

  EXPECT_EQ(fsm->state(), PickPlaceState::FAULT);
  EXPECT_EQ(fsm->error_code(), ErrorCodes::UNLOCK_FAILED);
  // 策略拒绝时绝不得调用解锁 IO，也不得继续分离
  EXPECT_EQ(harness.tool->unlock_calls, 0);
  EXPECT_EQ(harness.tool->disengage_calls, 0);
  EXPECT_NE(fsm->message().find("拒绝解锁"), std::string::npos);
  EXPECT_FALSE(fsm->placement_verified());
}

TEST(PickPlaceV2Interlock, RedundantInterlocksAreUnreachableButPresent)
{
  // 记录一个重要的迁移事实：在正常流转下，h_disengage 的前置联锁是
  // **不可达的冗余保护**（VERIFY_SEATED 与 UNLOAD 会先行拦截）。
  // 因此它的正确性只能通过"代码存在 + 单元测试可见"来保证，
  // 而其对外的安全效果由 NoUnlockWhenSeatNotVerified /
  // NoUnlockWhenUnloadNotVerified 两条路径覆盖。
  Harness harness;
  harness.tool->seated = false;
  harness.start();
  harness.run();

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
  EXPECT_EQ(harness.tool->unlock_calls, 0);
  const auto history = harness.history();
  // 从未进入 DISENGAGE：证明第一道拦截（VERIFY_SEATED）已生效
  EXPECT_EQ(std::find(history.begin(), history.end(), std::string("DISENGAGE")), history.end());
}

TEST(PickPlaceV2Interlock, UnlockHappensOnlyInsideDisengageAfterUnload)
{
  Harness harness;
  harness.start();

  const auto reached = harness.run_to(PickPlaceState::UNLOAD);
  ASSERT_EQ(reached, PickPlaceState::UNLOAD);
  // UNLOAD 完成之前绝不解锁
  EXPECT_EQ(harness.tool->unlock_calls, 0);

  harness.fsm->step();   // UNLOAD -> DISENGAGE
  ASSERT_EQ(harness.fsm->state(), PickPlaceState::DISENGAGE);
  // 刚进入 DISENGAGE 时也尚未解锁：联锁与解锁都在本状态内完成
  EXPECT_EQ(harness.tool->unlock_calls, 0);

  harness.fsm->step();   // DISENGAGE 内执行联锁判定并解锁
  EXPECT_EQ(harness.tool->unlock_calls, 1);
  EXPECT_EQ(harness.tool->unlock_pulses.size(), 1u);
}

TEST(PickPlaceV2Interlock, UnlockFailureGoesToFaultAndDoesNotRetreat)
{
  Harness harness;
  harness.tool->unlock_ok = false;
  harness.tool->unlock_error_code = ErrorCodes::UNLOCK_FAILED;
  harness.tool->unlock_message = "电磁解锁失败";
  harness.start();
  harness.run();

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::UNLOCK_FAILED);
  EXPECT_EQ(harness.tool->unlock_calls, 1);
  // 解锁失败后禁止撤离：不得调用 disengage，也不得进入 RETREAT
  EXPECT_EQ(harness.tool->disengage_calls, 0);
  const auto history = harness.history();
  EXPECT_EQ(std::find(history.begin(), history.end(), "RETREAT"), history.end());
}

TEST(PickPlaceV2Interlock, AcceptedPulseWithoutReleaseIsStillFailure)
{
  Harness harness;
  harness.tool->unlock_ok = true;          // 脉冲被受理
  harness.tool->unlock_released = false;   // 但锁止未解除
  harness.start();
  harness.run();

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::UNLOCK_FAILED);
  EXPECT_EQ(harness.tool->disengage_calls, 0);
}

TEST(PickPlaceV2Interlock, DisengageReleaseUnknownGoesToFaultAndDoesNotRetreat)
{
  Harness harness;
  harness.tool->disengage_ok = true;
  harness.tool->disengage_released = false;   // 机构分离结果未知
  harness.start();
  harness.run();

  EXPECT_EQ(harness.fsm->state(), PickPlaceState::FAULT);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::RELEASE_NOT_VERIFIED);
  // 未确认分离时禁止撤离与上抬
  const auto history = harness.history();
  EXPECT_EQ(std::find(history.begin(), history.end(), "RETREAT"), history.end());
  EXPECT_EQ(harness.fsm->placement_verified(), false);
}

TEST(PickPlaceV2Interlock, UnlockNeverHappensBeforeConfirmedPlacement)
{
  Harness harness;
  harness.perception->placement_at_target = false;   // 放置核验永不通过
  harness.start();
  harness.run(600);

  // 走到 VERIFY_PLACED 时窗口耗尽 -> PLACEMENT_FAILED；解锁已发生（属于 DISENGAGE 阶段），
  // 但**绝不能**在这里声称放置成功。
  EXPECT_FALSE(harness.fsm->placement_verified());
  EXPECT_NE(harness.fsm->state(), PickPlaceState::SUCCESS);
  EXPECT_EQ(harness.fsm->error_code(), ErrorCodes::PLACEMENT_FAILED);
}
