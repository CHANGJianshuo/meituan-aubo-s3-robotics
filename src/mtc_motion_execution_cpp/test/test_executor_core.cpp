// Copyright 2026 meituan_challenge_ws contributors
//
// 运动执行核心（七步握手）测试。
//
// 覆盖需求 5 的：Goal Accepted 不等于执行成功、并发冲突防护、
// 状态未知锁定与恢复、停止请求送达不等于停稳。

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "mtc_core_cpp/clock.hpp"
#include "mtc_core_cpp/error_codes.hpp"
#include "mtc_motion_execution_cpp/executor_core.hpp"
#include "mtc_motion_execution_cpp/mock_motion_adapter.hpp"

namespace
{

using mtc::core::ErrorCodes;
using mtc::core::ExecState;
using mtc::core::ManualClock;
using mtc::core::MotionCapabilities;
using mtc::core::MotionCommand;
using mtc::core::MotionResult;
using mtc::core::StopResult;
using mtc::motion::MockMotionAdapter;
using mtc::motion::MockMotionAdapterConfig;
using mtc::motion::MotionAdapter;
using mtc::motion::MotionExecutorCore;
using mtc::motion::ValidationConfig;

MotionCommand make_command(const std::string & command_id = "cmd-1", uint32_t timeout_ms = 5000)
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

/// 重入适配器：在"运动执行中"再次调用执行核心，用于验证并发拒绝。
///
/// 这真实模拟了"运动未结束时又收到第二个 Goal"的场景，
/// 而不是靠人为声明在途状态。
class ReentrantAdapter : public MotionAdapter
{
public:
  MotionExecutorCore * core = nullptr;
  MotionResult nested_result;
  bool nested_done = false;
  bool throw_on_execute = false;
  bool report_status_unknown = false;
  bool timeout_without_unknown = false;
  std::string last_command;

  MotionCapabilities capabilities() const override
  {
    MotionCapabilities capabilities;
    capabilities.backend_name = "reentrant";
    capabilities.simulation_only = true;
    capabilities.joint_goal_supported = true;
    capabilities.cancel_supported = true;
    return capabilities;
  }

  MotionResult execute_joint_move(const MotionCommand & command) override
  {
    if (throw_on_execute) {
      throw std::runtime_error("simulated backend crash");
    }
    if (core != nullptr && !nested_done) {
      nested_done = true;
      MotionCommand nested = make_command("cmd-nested");
      nested_result = core->execute_joint_move(nested);
    }

    last_command = command.command_id;
    MotionResult result;
    if (timeout_without_unknown) {
      result.timed_out = true;
      result.exec_state = ExecState::TIMEOUT;
      return result;
    }
    if (report_status_unknown) {
      result.success = false;
      result.arrived = false;
      result.exec_state = ExecState::STOP_UNKNOWN;
      result.error_code = ErrorCodes::MOTION_STATUS_UNKNOWN;
      result.status_unknown = true;
      result.message = "feedback lost";
      return result;
    }
    result.success = true;
    result.arrived = true;
    result.stop_confirmed = true;
    result.exec_state = ExecState::SETTLED;
    result.error_code = ErrorCodes::OK;
    result.final_position_rad = command.target_rad;
    result.message = "settled";
    return result;
  }

  std::optional<mtc::core::StopEvidence> stop_evidence() const override
  {
    if (last_command.empty() || timeout_without_unknown || report_status_unknown) {return std::nullopt;}
    mtc::core::StopEvidence e;
    e.command_id = last_command;
    e.exec_state = ExecState::SETTLED;
    e.communication_ok = e.queue_empty = e.stationary_interval_verified = true;
    e.feedback.stamp_s = mtc::core::SteadyClock().now();
    e.stationary_since_s = e.feedback.stamp_s - 0.2;
    e.feedback.position_rad.assign(6, 0.0);
    e.feedback.velocity_rad_s.assign(6, 0.0);
    return e;
  }
  StopResult request_stop(const std::string & reason) override
  {
    StopResult result;
    result.request_delivered = true;
    result.stop_confirmed = true;
    result.message = reason;
    return result;
  }

  mtc::core::JointFeedback joint_feedback() const override {return mtc::core::JointFeedback{};}
};

/// 不支持 cancel 的适配器：用于验证"不得伪造 stop_confirmed"。
class NoCancelAdapter : public MotionAdapter
{
public:
  MotionCapabilities capabilities() const override
  {
    MotionCapabilities capabilities;
    capabilities.backend_name = "no_cancel";
    capabilities.joint_goal_supported = true;
    capabilities.cancel_supported = false;
    return capabilities;
  }

  MotionResult execute_joint_move(const MotionCommand &) override {return MotionResult{};}

  StopResult request_stop(const std::string & reason) override
  {
    StopResult result;
    result.request_delivered = true;
    result.stop_confirmed = true;   // 假装确认；核心必须忽略它
    result.message = reason;
    return result;
  }

  mtc::core::JointFeedback joint_feedback() const override {return mtc::core::JointFeedback{};}
};

}  // namespace

// ---------------------------------------------------------------------------
// 正常握手
// ---------------------------------------------------------------------------

TEST(ExecutorCore, SuccessfulMoveGoesThroughSevenStepHandshake)
{
  const auto clock = std::make_shared<ManualClock>(0.0);
  auto adapter = std::make_shared<MockMotionAdapter>(MockMotionAdapterConfig(), clock);
  MotionExecutorCore core(adapter, ValidationConfig(), clock);

  const auto result = core.execute_joint_move(make_command());

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.arrived);
  EXPECT_TRUE(result.ok());
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_EQ(core.exec_state(), ExecState::SETTLED);

  const auto record = core.handshake("cmd-1");
  ASSERT_TRUE(record.has_value());
  ASSERT_FALSE(record->steps.empty());
  EXPECT_EQ(record->steps[0], "PREPARE");
  EXPECT_NE(
    std::find(record->steps.begin(), record->steps.end(), "ACCEPT"), record->steps.end());
  EXPECT_NE(
    std::find(record->steps.begin(), record->steps.end(), "VALIDATE"), record->steps.end());
  EXPECT_NE(
    std::find(record->steps.begin(), record->steps.end(), "SEND"), record->steps.end());
  EXPECT_NE(
    std::find(record->steps.begin(), record->steps.end(), "CONFIRM:settled"),
    record->steps.end());
}

TEST(ExecutorCore, CapabilitiesAreForwardedFromAdapter)
{
  const auto clock = std::make_shared<ManualClock>(0.0);
  auto adapter = std::make_shared<MockMotionAdapter>(MockMotionAdapterConfig(), clock);
  MotionExecutorCore core(adapter, ValidationConfig(), clock);

  EXPECT_EQ(core.capabilities().backend_name, "mock");
  EXPECT_TRUE(core.capabilities().joint_goal_supported);
  EXPECT_FALSE(core.capabilities().timed_trajectory_supported);
}

TEST(ExecutorCore, ValidationFailureRejectsWithoutStartingMotion)
{
  const auto clock = std::make_shared<ManualClock>(0.0);
  auto adapter = std::make_shared<MockMotionAdapter>(MockMotionAdapterConfig(), clock);
  MotionExecutorCore core(adapter, ValidationConfig(), clock);

  MotionCommand command = make_command();
  command.velocity_scaling = 0.9;   // 超出上限

  const auto result = core.execute_joint_move(command);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.exec_state, ExecState::REJECTED);
  EXPECT_EQ(result.error_code, ErrorCodes::SAFETY_INTERLOCK);
  // 从未开始运动 -> 允许立即重发
  EXPECT_TRUE(result.retry_allowed_without_reobservation());
  EXPECT_TRUE(adapter->call_log().empty());

  const auto record = core.handshake("cmd-1");
  ASSERT_TRUE(record.has_value());
  EXPECT_NE(
    std::find(record->steps.begin(), record->steps.end(),
      "VALIDATE_REJECTED:" + result.message), record->steps.end());
}

// ---------------------------------------------------------------------------
// 并发拒绝（需求 5：同时只允许一个有效运动目标）
// ---------------------------------------------------------------------------

TEST(ExecutorCore, ConcurrentGoalDuringMotionIsRejected)
{
  const auto clock = std::make_shared<mtc::core::SteadyClock>();
  auto adapter = std::make_shared<ReentrantAdapter>();
  MotionExecutorCore core(adapter, ValidationConfig(), clock);
  adapter->core = &core;

  const auto outer = core.execute_joint_move(make_command("cmd-outer"));
  EXPECT_TRUE(outer.success);

  ASSERT_TRUE(adapter->nested_done);
  // 嵌套请求必须被拒绝，且错误码为 210
  EXPECT_FALSE(adapter->nested_result.success);
  EXPECT_EQ(adapter->nested_result.exec_state, ExecState::REJECTED);
  EXPECT_EQ(adapter->nested_result.error_code, ErrorCodes::EXECUTION_REJECTED);
  EXPECT_NE(adapter->nested_result.message.find("cmd-outer"), std::string::npos);

  const auto record = core.handshake("cmd-nested");
  ASSERT_TRUE(record.has_value());
  EXPECT_NE(
    std::find(record->steps.begin(), record->steps.end(), "ACCEPT_REJECTED:in_flight=cmd-outer"),
    record->steps.end());
}

// ---------------------------------------------------------------------------
// 状态未知锁定与人工恢复
// ---------------------------------------------------------------------------

TEST(ExecutorCore, StatusUnknownLocksSubsequentRequests)
{
  const auto clock = std::make_shared<ManualClock>(0.0);
  MockMotionAdapterConfig config;
  config.status_unknown = true;
  auto adapter = std::make_shared<MockMotionAdapter>(config, clock);
  MotionExecutorCore core(adapter, ValidationConfig(), clock);

  const auto first = core.execute_joint_move(make_command("cmd-1"));
  EXPECT_FALSE(first.success);
  EXPECT_TRUE(first.status_unknown);
  EXPECT_TRUE(core.motion_state_unknown());
  EXPECT_EQ(core.exec_state(), ExecState::STOP_UNKNOWN);

  // 第二次请求必须在 PREPARE 阶段就被拒绝，且不得触达适配器
  const size_t calls_before = adapter->call_log().size();
  const auto second = core.execute_joint_move(make_command("cmd-2"));
  EXPECT_FALSE(second.success);
  EXPECT_EQ(second.error_code, ErrorCodes::MOTION_STATUS_UNKNOWN);
  EXPECT_EQ(second.exec_state, ExecState::STOP_UNKNOWN);
  EXPECT_TRUE(second.status_unknown);
  EXPECT_NE(second.message.find("禁止自动重发"), std::string::npos);
  EXPECT_EQ(adapter->call_log().size(), calls_before);   // 未再次发令

  const auto record = core.handshake("cmd-2");
  ASSERT_TRUE(record.has_value());
  EXPECT_NE(
    std::find(record->steps.begin(), record->steps.end(),
      "PREPARE_REJECTED:motion_state_unknown"), record->steps.end());
}

TEST(ExecutorCore, ManualConfirmationClearsUnknownLock)
{
  const auto clock = std::make_shared<ManualClock>(0.0);
  MockMotionAdapterConfig config;
  config.status_unknown = true;
  auto adapter = std::make_shared<MockMotionAdapter>(config, clock);
  MotionExecutorCore core(adapter, ValidationConfig(), clock);

  core.execute_joint_move(make_command("cmd-1"));
  ASSERT_TRUE(core.motion_state_unknown());

  // 仅确认停稳**不能**解除锁定：这是给人用的显式出口
  core.clear_unknown_after_manual_confirmation("现场核实：机器人已静止且无负载");

  EXPECT_FALSE(core.motion_state_unknown());
  EXPECT_EQ(core.exec_state(), ExecState::IDLE);
  EXPECT_TRUE(core.snapshot().stop_confirmed);

  const auto record = core.handshake("manual");
  ASSERT_TRUE(record.has_value());
  ASSERT_FALSE(record->steps.empty());
  EXPECT_NE(
    record->steps[0].find("MANUAL_CONFIRMATION"), std::string::npos);
}

TEST(ExecutorCore, BackendExceptionIsTreatedAsUnknownState)
{
  const auto clock = std::make_shared<mtc::core::SteadyClock>();
  auto adapter = std::make_shared<ReentrantAdapter>();
  adapter->throw_on_execute = true;
  MotionExecutorCore core(adapter, ValidationConfig(), clock);

  const auto result = core.execute_joint_move(make_command("cmd-1"));
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.status_unknown);
  EXPECT_EQ(result.error_code, ErrorCodes::MOTION_STATUS_UNKNOWN);
  EXPECT_FALSE(result.stop_confirmed);
  EXPECT_TRUE(core.motion_state_unknown());
  EXPECT_NE(result.message.find("禁止自动重发"), std::string::npos);
}

TEST(ExecutorCore, TimeoutMarksStateUnknown)
{
  const auto clock = std::make_shared<ManualClock>(0.0);
  MockMotionAdapterConfig config;
  config.travel_time_s = 99.0;
  auto adapter = std::make_shared<MockMotionAdapter>(config, clock);
  MotionExecutorCore core(adapter, ValidationConfig(), clock);

  const auto result = core.execute_joint_move(make_command("cmd-1", 1000));
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.timed_out);
  EXPECT_TRUE(result.status_unknown);
  EXPECT_TRUE(core.motion_state_unknown());
  EXPECT_FALSE(result.retry_allowed_without_reobservation());
}

TEST(ExecutorCore, BackendRejectionPropagatesRejectedStateAllowingResend)
{
  const auto clock = std::make_shared<ManualClock>(0.0);
  auto adapter = std::make_shared<MockMotionAdapter>(MockMotionAdapterConfig(), clock);
  MotionExecutorCore core(adapter, ValidationConfig(), clock);

  // 后端已被占用：核心自身校验通过，但适配器的唯一执行权拒绝该请求。
  ASSERT_TRUE(adapter->force_begin_in_flight("cmd-inflight"));

  const auto result = core.execute_joint_move(make_command("cmd-1"));
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.exec_state, ExecState::REJECTED);
  EXPECT_EQ(result.error_code, ErrorCodes::EXECUTION_REJECTED);
  // 从未开始运动 -> 允许立即重发
  EXPECT_TRUE(result.retry_allowed_without_reobservation());
  // 后端"拒绝"不是"状态未知"，不得锁定后续动作
  EXPECT_FALSE(core.motion_state_unknown());
}

// ---------------------------------------------------------------------------
// 停止请求：送达不等于停稳
// ---------------------------------------------------------------------------

TEST(ExecutorCore, StopWithoutInFlightIsConfirmed)
{
  const auto clock = std::make_shared<ManualClock>(0.0);
  auto adapter = std::make_shared<MockMotionAdapter>(MockMotionAdapterConfig(), clock);
  MotionExecutorCore core(adapter, ValidationConfig(), clock);

  const auto stop = core.request_stop("cancel requested");
  EXPECT_TRUE(stop.request_delivered);
  EXPECT_FALSE(stop.stop_confirmed);
  EXPECT_EQ(core.exec_state(), ExecState::CANCELLING);
}

TEST(ExecutorCore, StopDeliveryIsNotConfirmationWhenBackendCannotConfirm)
{
  const auto clock = std::make_shared<ManualClock>(0.0);
  MockMotionAdapterConfig config;
  config.cancel_confirms = false;
  auto adapter = std::make_shared<MockMotionAdapter>(config, clock);
  MotionExecutorCore core(adapter, ValidationConfig(), clock);

  ASSERT_TRUE(adapter->force_begin_in_flight("cmd-inflight"));
  const auto stop = core.request_stop("cancel");

  // 送达 != 停稳
  EXPECT_TRUE(stop.request_delivered);
  EXPECT_FALSE(stop.stop_confirmed);
  EXPECT_EQ(core.exec_state(), ExecState::CANCELLING);
  EXPECT_FALSE(core.snapshot().stop_confirmed);
}

TEST(ExecutorCore, UnsupportedCancelNeverFabricatesStopConfirmed)
{
  const auto clock = std::make_shared<mtc::core::SteadyClock>();
  auto adapter = std::make_shared<NoCancelAdapter>();
  MotionExecutorCore core(adapter, ValidationConfig(), clock);

  const auto stop = core.request_stop("cancel");

  // 后端自称 request_delivered/stop_confirmed 均为 true，
  // 但它未声明 cancel_supported：核心必须拒绝背书。
  EXPECT_FALSE(stop.request_delivered);
  EXPECT_FALSE(stop.stop_confirmed);
  EXPECT_EQ(core.exec_state(), ExecState::STOP_UNKNOWN);
  EXPECT_TRUE(core.motion_state_unknown());
  EXPECT_NE(stop.message.find("cancel_supported"), std::string::npos);
}

TEST(ExecutorCore, StopAfterUnknownStateIsNeverConfirmed)
{
  const auto clock = std::make_shared<ManualClock>(0.0);
  MockMotionAdapterConfig config;
  config.status_unknown = true;
  auto adapter = std::make_shared<MockMotionAdapter>(config, clock);
  MotionExecutorCore core(adapter, ValidationConfig(), clock);

  core.execute_joint_move(make_command("cmd-1"));
  ASSERT_TRUE(core.motion_state_unknown());

  // 即使后端报告已确认，状态未知时也不得宣告停稳。
  // 执行状态停在 CANCELLING（取消中），绝不进入 STOPPED；
  // 这与 Python MotionExecutorCore.request_stop 的语义逐字一致。
  const auto stop = core.request_stop("cancel", std::string("cmd-1"));
  EXPECT_FALSE(stop.stop_confirmed);
  EXPECT_EQ(mtc::core::to_string(core.exec_state()), "CANCELLING");
  EXPECT_TRUE(core.motion_state_unknown());
  EXPECT_FALSE(core.snapshot().stop_confirmed);
}

TEST(ExecutorCore, SnapshotReflectsInternalState)
{
  const auto clock = std::make_shared<ManualClock>(0.0);
  auto adapter = std::make_shared<MockMotionAdapter>(MockMotionAdapterConfig(), clock);
  MotionExecutorCore core(adapter, ValidationConfig(), clock);

  auto snapshot = core.snapshot();
  EXPECT_EQ(snapshot.exec_state, ExecState::IDLE);
  EXPECT_FALSE(snapshot.active_command_id.has_value());
  EXPECT_FALSE(snapshot.stop_confirmed);   // Startup has no stationary evidence.
  EXPECT_FALSE(snapshot.motion_state_unknown);

  core.execute_joint_move(make_command("cmd-1"));
  snapshot = core.snapshot();
  EXPECT_EQ(snapshot.exec_state, ExecState::SETTLED);
  EXPECT_TRUE(snapshot.stop_confirmed);
  EXPECT_EQ(snapshot.last_error_code, ErrorCodes::OK);
}

TEST(ExecutorAudit, DeliveryWithoutEvidenceAndWithoutCommandNeverConfirms)
{
  const auto clock = std::make_shared<mtc::core::SteadyClock>();
  auto adapter = std::make_shared<ReentrantAdapter>();
  MotionExecutorCore core(adapter);
  const auto stop = core.request_stop("delivered but robot state unknown");
  EXPECT_TRUE(stop.request_delivered);
  EXPECT_FALSE(stop.stop_confirmed);
}

TEST(ExecutorAudit, TimeoutWithoutUnknownFlagStillLocks)
{
  const auto clock = std::make_shared<mtc::core::SteadyClock>();
  auto adapter = std::make_shared<ReentrantAdapter>();
  adapter->timeout_without_unknown = true;
  MotionExecutorCore core(adapter);
  const auto first = core.execute_joint_move(make_command());
  EXPECT_TRUE(first.timed_out);
  EXPECT_TRUE(core.motion_state_unknown());
  EXPECT_EQ(core.execute_joint_move(make_command("cmd-2")).error_code,
    ErrorCodes::MOTION_STATUS_UNKNOWN);
}

TEST(ExecutorAudit, CompletedCommandIdCannotExecuteTwice)
{
  auto clock = std::make_shared<ManualClock>();
  auto adapter = std::make_shared<MockMotionAdapter>(MockMotionAdapterConfig(), clock);
  MotionExecutorCore core(adapter, {}, clock);
  EXPECT_TRUE(core.execute_joint_move(make_command("unique")).success);
  EXPECT_EQ(core.execute_joint_move(make_command("unique")).exec_state, ExecState::REJECTED);
  EXPECT_EQ(adapter->call_log().size(), 1u);
}

TEST(ExecutorAudit, CancelBeforeBackendDispatchIsSticky)
{
  auto clock = std::make_shared<ManualClock>();
  MockMotionAdapter adapter(MockMotionAdapterConfig(), clock);
  EXPECT_FALSE(adapter.request_stop_for_command("early cancel", std::string("future")).stop_confirmed);
  const auto result = adapter.execute_joint_move(make_command("future"));
  EXPECT_EQ(result.exec_state, ExecState::REJECTED);
  EXPECT_DOUBLE_EQ(clock->now(), 0.0);
  EXPECT_EQ(adapter.joint_feedback().position_rad, std::vector<double>(6, 0.0));
}
