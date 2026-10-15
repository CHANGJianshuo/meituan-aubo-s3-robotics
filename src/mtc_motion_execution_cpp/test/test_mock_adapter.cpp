// Copyright 2026 meituan_challenge_ws contributors
//
// Mock / Disabled 运动适配器测试。
//
// 覆盖需求 5 的：到位、超时、状态未知、取消已确认/未确认、
// 以及"轨迹不得被拆成多次单点调用"。

#include <gtest/gtest.h>

#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "mtc_core_cpp/clock.hpp"
#include "mtc_core_cpp/error_codes.hpp"
#include "mtc_motion_execution_cpp/mock_motion_adapter.hpp"
#include "mtc_motion_execution_cpp/executor_core.hpp"

namespace
{

using mtc::core::ErrorCodes;
using mtc::core::ExecState;
using mtc::core::ManualClock;
using mtc::core::MotionCommand;
using mtc::core::TimedTrajectory;
using mtc::core::TrajectoryPoint;
using mtc::motion::DisabledMotionAdapter;
using mtc::motion::MockMotionAdapter;
using mtc::motion::MockMotionAdapterConfig;
using mtc::motion::ValidationConfig;

struct Fixture
{
  std::shared_ptr<ManualClock> clock = std::make_shared<ManualClock>(0.0);

  MotionCommand make_command(
    const std::string & command_id = "cmd-1",
    uint32_t timeout_ms = 5000) const
  {
    MotionCommand command;
    command.command_id = command_id;
    command.joint_names = {
      "shoulder_joint", "upperArm_joint", "foreArm_joint",
      "wrist1_joint", "wrist2_joint", "wrist3_joint",
    };
    command.target_rad = {0.05, -0.02, 0.01, 0.0, 0.0, 0.0};
    command.velocity_scaling = 0.1;
    command.acceleration_scaling = 0.1;
    command.timeout_ms = timeout_ms;
    return command;
  }

  TimedTrajectory make_trajectory() const
  {
    TimedTrajectory trajectory;
    trajectory.trajectory_id = "traj-1";
    trajectory.joint_names = {
      "shoulder_joint", "upperArm_joint", "foreArm_joint",
      "wrist1_joint", "wrist2_joint", "wrist3_joint",
    };
    TrajectoryPoint point;
    point.time_from_start_s = 0.1;
    point.positions_rad = {0.01, 0.0, 0.0, 0.0, 0.0, 0.0};
    trajectory.points = {point};
    return trajectory;
  }
};

}  // namespace

TEST(MockClockGate, MockDefaultClockExecutionContract)
{
  Fixture fixture;
  auto adapter = std::make_shared<MockMotionAdapter>();
  mtc::motion::MotionExecutorCore core(adapter);
  const auto result = core.execute_joint_move(fixture.make_command());
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_FALSE(result.status_unknown);
}

TEST(MockClockGate, MockSharedManualClockSettlementSucceeds)
{
  Fixture fixture;
  auto adapter = std::make_shared<MockMotionAdapter>(MockMotionAdapterConfig(), fixture.clock);
  mtc::motion::MotionExecutorCore core(adapter, {}, fixture.clock);
  EXPECT_TRUE(core.execute_joint_move(fixture.make_command()).success);
  const auto evidence = adapter->stop_evidence();
  ASSERT_TRUE(evidence);
  EXPECT_NEAR(evidence->feedback.stamp_s - evidence->stationary_since_s, 0.2, 1e-9);
  EXPECT_DOUBLE_EQ(evidence->feedback.stamp_s, fixture.clock->now());
}

TEST(MockClockGate, MockInsufficientStationaryIntervalRejected)
{
  class FrozenClock : public ManualClock {public: double now() const override {return 100.0;}};
  auto clock = std::make_shared<FrozenClock>();
  auto adapter = std::make_shared<MockMotionAdapter>(MockMotionAdapterConfig(), clock);
  mtc::motion::MotionExecutorCore core(adapter, {}, clock);
  Fixture fixture;
  const auto result = core.execute_joint_move(fixture.make_command());
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.stop_confirmed);
  EXPECT_TRUE(core.motion_state_unknown());
}

TEST(MockClockGate, TinyPositiveSettleDurationCannotConfirmZeroInterval)
{
  class FrozenClock : public ManualClock {public: double now() const override {return 100.0;}};
  auto clock = std::make_shared<FrozenClock>();
  ValidationConfig config;
  config.settle_duration_s = 1e-12;
  auto adapter = std::make_shared<MockMotionAdapter>(MockMotionAdapterConfig(), clock, config);
  mtc::motion::MotionExecutorCore core(adapter, config, clock);
  Fixture fixture;
  const auto result = core.execute_joint_move(fixture.make_command());
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.stop_confirmed);
  EXPECT_TRUE(core.motion_state_unknown());
}

TEST(MockClockGate, MockExecutorClockMismatchRejected)
{
  Fixture fixture;
  auto adapter_clock = std::make_shared<ManualClock>(100);
  auto executor_clock = std::make_shared<ManualClock>(100);
  auto adapter = std::make_shared<MockMotionAdapter>(MockMotionAdapterConfig(), adapter_clock);
  mtc::motion::MotionExecutorCore core(adapter, {}, executor_clock);
  const auto result = core.execute_joint_move(fixture.make_command());
  EXPECT_EQ(result.exec_state, ExecState::REJECTED);
  EXPECT_FALSE(result.status_unknown);
  EXPECT_TRUE(adapter->call_log().empty());
  EXPECT_DOUBLE_EQ(adapter_clock->now(), 100.0);
}

TEST(MockClockGate, ExplicitRealTimeOrInvalidMockClockIsRejected)
{
  EXPECT_THROW(MockMotionAdapter(MockMotionAdapterConfig(),
    std::make_shared<mtc::core::SteadyClock>()), std::invalid_argument);
  for (double invalid : {-1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(MockMotionAdapter(MockMotionAdapterConfig(),
      std::make_shared<ManualClock>(invalid)), std::invalid_argument);
  }
}

TEST(GoalCancelGate, AdapterScopedCancelCannotStopDifferentOwner)
{
  Fixture fixture;
  MockMotionAdapter adapter(MockMotionAdapterConfig(), fixture.clock);
  ASSERT_TRUE(adapter.force_begin_in_flight("B"));
  const auto before = adapter.joint_feedback();
  const auto stop = adapter.request_stop_for_command("old cancel", std::string("A"));
  EXPECT_FALSE(stop.request_delivered);
  EXPECT_EQ(stop.error_code, ErrorCodes::EXECUTION_REJECTED);
  EXPECT_TRUE(adapter.has_in_flight());
  EXPECT_FALSE(adapter.stop_confirmed_flag());
  EXPECT_DOUBLE_EQ(adapter.joint_feedback().stamp_s, before.stamp_s);
  EXPECT_FALSE(adapter.stop_evidence());
  adapter.force_end_in_flight();
}

TEST(MockClockGate, VirtualClockCannotAuthorizeExecutableBackend)
{
  class ExecutableClaim : public MockMotionAdapter
  {
  public:
    mtc::core::MotionCapabilities capabilities() const override
    {
      auto c = MockMotionAdapter::capabilities(); c.simulation_only = false; return c;
    }
  };
  auto adapter = std::make_shared<ExecutableClaim>();
  ValidationConfig config;
  config.joint_limit_rad.assign(6, {-3.0, 3.0});
  mtc::motion::MotionExecutorCore core(adapter, config);
  Fixture fixture;
  const auto result = core.execute_joint_move(fixture.make_command());
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error_code, ErrorCodes::SAFETY_INTERLOCK);
  EXPECT_NE(result.message.find("virtual ManualClock is forbidden"), std::string::npos);
  EXPECT_TRUE(adapter->call_log().empty());
}

TEST(MockClockGate, StaleFeedbackCannotConfirmStop)
{
  Fixture fixture;
  auto adapter = std::make_shared<MockMotionAdapter>(MockMotionAdapterConfig(), fixture.clock);
  mtc::motion::MotionExecutorCore core(adapter, {}, fixture.clock);
  EXPECT_TRUE(core.execute_joint_move(fixture.make_command()).success);
  fixture.clock->advance(ValidationConfig().feedback_max_age_s + 0.01);
  EXPECT_FALSE(core.cancel_goal("cmd-1", "completed but feedback stale").stop_confirmed);
  EXPECT_TRUE(core.motion_state_unknown());
}

TEST(MockClockGate, ExplicitStationarySampleRenewsOnlyTheMatchingCompletedCommand)
{
  Fixture fixture;
  auto adapter = std::make_shared<MockMotionAdapter>(MockMotionAdapterConfig(), fixture.clock);
  mtc::motion::MotionExecutorCore core(adapter, {}, fixture.clock);
  ASSERT_TRUE(core.execute_joint_move(fixture.make_command()).success);
  const auto old = adapter->stop_evidence(); ASSERT_TRUE(old);
  fixture.clock->advance(0.8);
  EXPECT_FALSE(adapter->sample_stationary_feedback("other-command"));
  EXPECT_DOUBLE_EQ(adapter->stop_evidence()->feedback.stamp_s, old->feedback.stamp_s);
  ASSERT_TRUE(adapter->sample_stationary_feedback("cmd-1"));
  const auto current = adapter->stop_evidence(); ASSERT_TRUE(current);
  EXPECT_DOUBLE_EQ(current->feedback.stamp_s, fixture.clock->now());
  EXPECT_DOUBLE_EQ(current->stationary_since_s, old->stationary_since_s);
  EXPECT_EQ(current->command_id, old->command_id);
  EXPECT_EQ(current->feedback.position_rad, old->feedback.position_rad);
  EXPECT_EQ(current->feedback.velocity_rad_s, old->feedback.velocity_rad_s);
  EXPECT_EQ(adapter->call_log().size(), 1u);
  EXPECT_TRUE(core.cancel_goal("cmd-1", "fresh stationary observation").stop_confirmed);
  EXPECT_FALSE(core.motion_state_unknown());
  EXPECT_TRUE(core.execute_joint_move(fixture.make_command("cmd-2")).success);
  EXPECT_FALSE(adapter->sample_stationary_feedback("cmd-1"));
}

TEST(MockClockGate, StationarySamplingCannotInventMissingOrInFlightEvidence)
{
  Fixture fixture;
  MockMotionAdapter adapter(MockMotionAdapterConfig(), fixture.clock);
  EXPECT_FALSE(adapter.sample_stationary_feedback(""));
  EXPECT_FALSE(adapter.sample_stationary_feedback("never-executed"));
  EXPECT_FALSE(adapter.stop_evidence());
  EXPECT_FALSE(adapter.request_stop_for_command("early", std::string("reserved")).stop_confirmed);
  EXPECT_FALSE(adapter.sample_stationary_feedback("reserved"));
  ASSERT_TRUE(adapter.force_begin_in_flight("active"));
  EXPECT_FALSE(adapter.sample_stationary_feedback("active"));
  EXPECT_FALSE(adapter.stop_evidence());
  EXPECT_TRUE(adapter.has_in_flight());
  adapter.force_end_in_flight();
}

TEST(MockClockGate, StationarySamplingCannotClearUnknownStateOrTimeout)
{
  for (const auto & failure : {"status_unknown", "timeout"}) {
    SCOPED_TRACE(failure);
    Fixture fixture;
    MockMotionAdapterConfig options; options.fail_at = failure;
    auto adapter = std::make_shared<MockMotionAdapter>(options, fixture.clock);
    mtc::motion::MotionExecutorCore core(adapter, {}, fixture.clock);
    ASSERT_FALSE(core.execute_joint_move(fixture.make_command()).success);
    fixture.clock->advance(0.8);
    EXPECT_FALSE(adapter->sample_stationary_feedback("cmd-1"));
    EXPECT_FALSE(adapter->stop_evidence());
    EXPECT_FALSE(core.cancel_goal("cmd-1", "unknown state").stop_confirmed);
    EXPECT_TRUE(core.motion_state_unknown());
    EXPECT_FALSE(core.execute_joint_move(fixture.make_command("cmd-2")).success);
    EXPECT_EQ(adapter->call_log().size(), 1u);
  }
}

TEST(MockClockGate, StationarySamplingRejectsRewoundAndNonfiniteClocks)
{
  for (const double now : {0.1, -1.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()}) {
    SCOPED_TRACE(now);
    Fixture fixture;
    auto adapter = std::make_shared<MockMotionAdapter>(MockMotionAdapterConfig(), fixture.clock);
    mtc::motion::MotionExecutorCore core(adapter, {}, fixture.clock);
    ASSERT_TRUE(core.execute_joint_move(fixture.make_command()).success);
    const auto old = adapter->stop_evidence(); ASSERT_TRUE(old);
    fixture.clock->set(now);
    EXPECT_FALSE(adapter->sample_stationary_feedback("cmd-1"));
    EXPECT_DOUBLE_EQ(adapter->stop_evidence()->feedback.stamp_s, old->feedback.stamp_s);
    EXPECT_FALSE(core.cancel_goal("cmd-1", "invalid clock").stop_confirmed);
    EXPECT_TRUE(core.motion_state_unknown());
  }
}

TEST(MockAudit, VirtualSettlementStartsAfterArrivalAndRejectsInvalidTravel)
{
  for (double travel : {0.0, 0.5}) {
    Fixture fixture;
    MockMotionAdapterConfig config;
    config.travel_time_s = travel;
    MockMotionAdapter adapter(config, fixture.clock);
    EXPECT_TRUE(adapter.execute_joint_move(fixture.make_command()).success);
    const auto evidence = adapter.stop_evidence();
    ASSERT_TRUE(evidence);
    EXPECT_DOUBLE_EQ(evidence->stationary_since_s, travel);
    EXPECT_DOUBLE_EQ(evidence->feedback.stamp_s, travel + ValidationConfig().settle_duration_s);
  }
  for (double travel : {-1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()}) {
    Fixture fixture;
    MockMotionAdapterConfig config;
    config.travel_time_s = travel;
    MockMotionAdapter adapter(config, fixture.clock);
    EXPECT_EQ(adapter.execute_joint_move(fixture.make_command()).exec_state, ExecState::REJECTED);
    EXPECT_FALSE(adapter.stop_evidence());
    EXPECT_DOUBLE_EQ(fixture.clock->now(), 0.0);
  }
}

TEST(MockAdapter, CapabilitiesAreDeclaredExplicitly)
{
  MockMotionAdapter adapter(MockMotionAdapterConfig(), std::make_shared<ManualClock>());
  const auto capabilities = adapter.capabilities();
  EXPECT_EQ(capabilities.backend_name, "mock");
  EXPECT_TRUE(capabilities.joint_goal_supported);
  EXPECT_TRUE(capabilities.cancel_supported);
  // 未实测启用的能力必须为 false，不得因为"理论上支持"就置 true
  EXPECT_FALSE(capabilities.timed_trajectory_supported);
  EXPECT_FALSE(capabilities.cartesian_motion_supported);
}

TEST(MockAdapter, SuccessfulMoveSettlesAndConfirmsStop)
{
  Fixture fixture;
  MockMotionAdapter adapter(
    MockMotionAdapterConfig{0.5, false, false, true, "", "mock", true, false, false, true},
    fixture.clock);

  const auto result = adapter.execute_joint_move(fixture.make_command());
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.arrived);
  EXPECT_TRUE(result.ok());
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_EQ(result.exec_state, ExecState::SETTLED);
  EXPECT_EQ(result.error_code, ErrorCodes::OK);
  EXPECT_FALSE(result.status_unknown);
  EXPECT_FALSE(result.timed_out);
  // 模拟时间被推进到行程时间
  EXPECT_DOUBLE_EQ(fixture.clock->now(), 0.5 + ValidationConfig().settle_duration_s);
}

TEST(MockAdapter, RejectedRequestAllowsImmediateResend)
{
  Fixture fixture;
  // 期望关节顺序与请求不符 -> 后端直接拒绝，从未开始运动
  MockMotionAdapter adapter(MockMotionAdapterConfig(), fixture.clock);

  MotionCommand command = fixture.make_command();
  command.joint_names[0] = "not_a_joint";

  const auto result = adapter.execute_joint_move(command);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.exec_state, ExecState::REJECTED);
  EXPECT_EQ(result.error_code, ErrorCodes::EXECUTION_REJECTED);
  // 从未开始运动 -> 允许立即重发
  EXPECT_TRUE(result.retry_allowed_without_reobservation());
  EXPECT_FALSE(adapter.has_in_flight());
}

TEST(MockAdapter, SecondCommandWhileInFlightIsRejected)
{
  Fixture fixture;
  MockMotionAdapter adapter(MockMotionAdapterConfig(), fixture.clock);

  // 人为构造"运动在途"（单线程下覆盖并发路径）
  ASSERT_TRUE(adapter.force_begin_in_flight("cmd-inflight"));

  const auto result = adapter.execute_joint_move(fixture.make_command("cmd-2"));
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.exec_state, ExecState::REJECTED);
  EXPECT_EQ(result.error_code, ErrorCodes::EXECUTION_REJECTED);
  // 拒绝理由必须指出当前在途命令
  EXPECT_NE(result.message.find("cmd-inflight"), std::string::npos);
  EXPECT_NE(result.message.find("cmd-2"), std::string::npos);
  // 从未开始运动 -> 允许立即重发
  EXPECT_TRUE(result.retry_allowed_without_reobservation());

  adapter.force_end_in_flight();
  EXPECT_FALSE(adapter.has_in_flight());
}

TEST(MockAdapter, TimeoutIsReportedAndBlocksAutoResend)
{
  Fixture fixture;
  MockMotionAdapterConfig config;
  config.travel_time_s = 99.0;   // 远大于预算 -> 超时
  MockMotionAdapter adapter(config, fixture.clock);

  const auto result = adapter.execute_joint_move(fixture.make_command("cmd-1", 1000));
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.arrived);
  EXPECT_EQ(result.exec_state, ExecState::TIMEOUT);
  EXPECT_EQ(result.error_code, ErrorCodes::MOTION_TIMEOUT);
  EXPECT_TRUE(result.timed_out);
  // 可能仍在运动：必须标记状态未知，禁止自动重发
  EXPECT_TRUE(result.status_unknown);
  EXPECT_FALSE(result.retry_allowed_without_reobservation());
}

TEST(MockAdapter, StatusUnknownLocksAndBlocksAutoResend)
{
  Fixture fixture;
  MockMotionAdapterConfig config;
  config.status_unknown = true;
  MockMotionAdapter adapter(config, fixture.clock);

  const auto result = adapter.execute_joint_move(fixture.make_command());
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.exec_state, ExecState::STOP_UNKNOWN);
  EXPECT_EQ(result.error_code, ErrorCodes::MOTION_STATUS_UNKNOWN);
  EXPECT_TRUE(result.status_unknown);
  EXPECT_FALSE(result.stop_confirmed);
  EXPECT_NE(result.message.find("禁止自动重发"), std::string::npos);
  EXPECT_FALSE(result.retry_allowed_without_reobservation());
  // 反馈断流后不得残留执行权
  EXPECT_FALSE(adapter.has_in_flight());
}

TEST(MockAdapter, FailAtStatusUnknownIsEquivalent)
{
  Fixture fixture;
  MockMotionAdapterConfig config;
  config.fail_at = "status_unknown";
  MockMotionAdapter adapter(config, fixture.clock);

  const auto result = adapter.execute_joint_move(fixture.make_command());
  EXPECT_TRUE(result.status_unknown);
  EXPECT_EQ(result.error_code, ErrorCodes::MOTION_STATUS_UNKNOWN);
}

TEST(MockAdapter, StopWithoutInFlightIsConfirmedAndReported)
{
  Fixture fixture;
  MockMotionAdapter adapter(MockMotionAdapterConfig(), fixture.clock);

  const auto stop = adapter.request_stop("cancel requested");
  EXPECT_TRUE(stop.request_delivered);
  EXPECT_FALSE(stop.stop_confirmed);
  EXPECT_FALSE(adapter.stop_confirmed_flag());
}

TEST(MockAdapter, StopWithInFlightAndConfirmationSucceeds)
{
  Fixture fixture;
  MockMotionAdapterConfig config;
  config.cancel_confirms = true;
  MockMotionAdapter adapter(config, fixture.clock);

  ASSERT_TRUE(adapter.force_begin_in_flight("cmd-inflight"));
  const auto stop = adapter.request_stop("cancel");
  EXPECT_TRUE(stop.request_delivered);
  EXPECT_TRUE(stop.stop_confirmed);
  EXPECT_TRUE(adapter.stop_confirmed_flag());
  // 确认停稳后执行权必须释放
  EXPECT_FALSE(adapter.has_in_flight());
}

TEST(MockAdapter, CancelWithoutConfirmationDoesNotClaimStopped)
{
  Fixture fixture;
  MockMotionAdapterConfig config;
  config.cancel_confirms = false;   // 后端拒绝确认停稳
  MockMotionAdapter adapter(config, fixture.clock);

  ASSERT_TRUE(adapter.force_begin_in_flight("cmd-inflight"));
  const auto stop = adapter.request_stop("cancel");

  // 送达 != 停稳：绝不能因为请求被受理就宣告已安全停止
  EXPECT_TRUE(stop.request_delivered);
  EXPECT_FALSE(stop.stop_confirmed);
  EXPECT_FALSE(adapter.stop_confirmed_flag());
  EXPECT_NE(stop.message.find("尚未确认停稳"), std::string::npos);
}

TEST(MockAdapter, StopConfirmationFlagIsExposedSeparately)
{
  Fixture fixture;
  MockMotionAdapter adapter(MockMotionAdapterConfig(), fixture.clock);

  // 初始未确认停稳（与 Python MockMotionBackend 的 _stop_confirmed=False 一致）
  const auto initial = adapter.explicit_stop_confirmation();
  ASSERT_TRUE(initial.has_value());
  EXPECT_FALSE(*initial);

  // 停止请求送达且后端确认后，显式标志才变为 true
  adapter.force_begin_in_flight("cmd-evidence");
  adapter.request_stop("cancel");
  const auto after_stop = adapter.explicit_stop_confirmation();
  ASSERT_TRUE(after_stop.has_value());
  EXPECT_TRUE(*after_stop);
  EXPECT_TRUE(adapter.stop_confirmed_flag());
}

TEST(MockAdapter, TrajectoryIsRejectedAndNotSplitIntoJointMoves)
{
  Fixture fixture;
  MockMotionAdapter adapter(MockMotionAdapterConfig(), fixture.clock);

  const auto result = adapter.execute_trajectory(fixture.make_trajectory());

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.exec_state, ExecState::REJECTED);
  EXPECT_EQ(result.error_code, ErrorCodes::EXECUTION_REJECTED);
  EXPECT_NE(result.message.find("拆"), std::string::npos);

  // 关键断言：轨迹**没有**被降级成多次单点调用
  EXPECT_TRUE(adapter.call_log().empty());
  ASSERT_EQ(adapter.trajectory_call_log().size(), 1u);
  EXPECT_EQ(adapter.trajectory_call_log()[0].trajectory_id, "traj-1");
}

TEST(MockAdapter, TrajectoryAcceptedWhenCapabilityDeclared)
{
  Fixture fixture;
  MockMotionAdapterConfig config;
  config.timed_trajectory_supported = true;
  MockMotionAdapter adapter(config, fixture.clock);

  const auto result = adapter.execute_trajectory(fixture.make_trajectory());
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.arrived);
  EXPECT_TRUE(adapter.call_log().empty());   // 仍然没有被拆分
}

TEST(MockAdapter, BaseClassDefaultTrajectoryRejectsWithoutSplitting)
{
  // DisabledMotionAdapter 未覆写 execute_trajectory：
  // 基类默认实现必须显式拒绝，而不是降级。
  DisabledMotionAdapter adapter;
  TimedTrajectory trajectory;
  trajectory.joint_names = {"shoulder_joint"};
  TrajectoryPoint point;
  point.time_from_start_s = 0.1;
  point.positions_rad = {0.0};
  trajectory.points = {point};

  const auto result = adapter.execute_trajectory(trajectory);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error_code, ErrorCodes::EXECUTION_REJECTED);
  EXPECT_NE(result.message.find("拆"), std::string::npos);
}

// ---------------------------------------------------------------------------
// Disabled 适配器与工厂
// ---------------------------------------------------------------------------

TEST(DisabledAdapter, AlwaysRejectsAndNeverClaimsCapability)
{
  DisabledMotionAdapter adapter;
  const auto capabilities = adapter.capabilities();
  EXPECT_EQ(capabilities.backend_name, "disabled");
  EXPECT_FALSE(capabilities.joint_goal_supported);
  EXPECT_FALSE(capabilities.timed_trajectory_supported);
  EXPECT_FALSE(capabilities.cancel_supported);

  MotionCommand command;
  command.command_id = "cmd-1";

  const auto result = adapter.execute_joint_move(command);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.exec_state, ExecState::REJECTED);
  EXPECT_EQ(result.error_code, ErrorCodes::EXECUTION_REJECTED);

  const auto stop = adapter.request_stop("stop");
  EXPECT_FALSE(stop.request_delivered);
  EXPECT_FALSE(stop.stop_confirmed);
  EXPECT_NE(stop.message.find("未送达"), std::string::npos);
}

TEST(AdapterFactory, UnknownBackendFallsBackToDisabled)
{
  // 未知名称一律回到 disabled，绝不默认连真实设备
  const auto adapter = mtc::motion::make_adapter("real_aubo");
  ASSERT_NE(adapter, nullptr);
  EXPECT_EQ(adapter->capabilities().backend_name, "disabled");
  EXPECT_FALSE(adapter->capabilities().joint_goal_supported);

  const auto empty = mtc::motion::make_adapter("");
  EXPECT_EQ(empty->capabilities().backend_name, "disabled");
}

TEST(AdapterFactory, MockBackendIsCaseInsensitive)
{
  const auto clock = std::make_shared<ManualClock>();
  const auto adapter = mtc::motion::make_adapter("MOCK", MockMotionAdapterConfig(), clock);
  ASSERT_NE(adapter, nullptr);
  EXPECT_EQ(adapter->capabilities().backend_name, "mock");
  EXPECT_TRUE(adapter->capabilities().joint_goal_supported);
}
