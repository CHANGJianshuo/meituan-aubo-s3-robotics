// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_motion_execution_cpp/mock_motion_adapter.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "mtc_core_cpp/error_codes.hpp"

namespace mtc
{
namespace motion
{

namespace
{

mtc::core::MotionResult make_result(
  bool success, bool arrived, mtc::core::ExecState state, uint16_t code,
  const std::string & message)
{
  mtc::core::MotionResult result;
  result.success = success;
  result.arrived = arrived;
  result.exec_state = state;
  result.error_code = code;
  result.message = message;
  return result;
}

}  // namespace

MockMotionAdapter::MockMotionAdapter(
  MockMotionAdapterConfig config, mtc::core::ClockPtr clock, ValidationConfig validation)
: config_(std::move(config)),
  clock_(clock ? std::move(clock) : std::make_shared<mtc::core::ManualClock>()),
  validation_(std::move(validation))
{
  if (!dynamic_cast<mtc::core::ManualClock *>(clock_.get()) ||
    !std::isfinite(clock_->now()) || clock_->now() < 0.0) {
    throw std::invalid_argument("MockMotionAdapter requires a finite nonnegative ManualClock; no real-time simulation");
  }
  capabilities_.backend_name = config_.backend_name;
  capabilities_.simulation_only = true;
  capabilities_.associated_cancel_supported = true;
  capabilities_.joint_goal_supported = config_.joint_goal_supported;
  capabilities_.timed_trajectory_supported = config_.timed_trajectory_supported;
  capabilities_.cartesian_motion_supported = config_.cartesian_motion_supported;
  capabilities_.cancel_supported = config_.cancel_supported;
  capabilities_.detail = config_.detail;

  position_.assign(validation_.expected_joint_names.size(), 0.0);
  velocity_.assign(validation_.expected_joint_names.size(), 0.0);
}

mtc::core::MotionCapabilities MockMotionAdapter::capabilities() const
{
  return capabilities_;
}

mtc::core::JointFeedback MockMotionAdapter::joint_feedback() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  mtc::core::JointFeedback feedback;
  feedback.position_rad = position_;
  feedback.velocity_rad_s = velocity_;
  feedback.stamp_s = clock_->now();
  return feedback;
}

void MockMotionAdapter::advance_time(double seconds)
{
  if (seconds <= 0.0) {
    return;
  }
  // Constructor enforces offline ManualClock. No SteadyClock fabrication or real waiting.
  static_cast<mtc::core::ManualClock *>(clock_.get())->advance(seconds);
}

mtc::core::MotionResult MockMotionAdapter::execute_joint_move(
  const mtc::core::MotionCommand & command)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  call_log_.push_back(command);
  if (cancelled_ids_.count(command.command_id)) {
    return make_result(false, false, mtc::core::ExecState::REJECTED,
      mtc::core::ErrorCodes::EXECUTION_REJECTED, "command cancelled before backend dispatch");
  }

  const ValidationOutcome validation =
    validate_joint_move(command, validation_, capabilities_);
  if (!validation.ok) {
    last_validation_ = validation;
    return make_result(
      false, false, mtc::core::ExecState::REJECTED, validation.error_code, validation.message);
  }
  if (!std::isfinite(config_.travel_time_s) || config_.travel_time_s < 0.0) {
    return make_result(false, false, mtc::core::ExecState::REJECTED,
      mtc::core::ErrorCodes::EXECUTION_REJECTED, "invalid Mock travel time");
  }

  const auto acquired = guard_.acquire(command.command_id);
  if (!acquired.first) {
    return make_result(
      false, false, mtc::core::ExecState::REJECTED,
      mtc::core::ErrorCodes::EXECUTION_REJECTED, acquired.second);
  }

  evidence_.reset();
  active_command_ = command.command_id;
  cancel_requested_ = false;
  stop_confirmed_ = false;

  if (config_.fail_at == "status_unknown" || config_.status_unknown) {
    // 反馈断流：状态不确定，必须锁定动作，禁止自动重发。
    guard_.release(command.command_id);
    active_command_.reset();
    mtc::core::MotionResult result = make_result(
      false, false, mtc::core::ExecState::STOP_UNKNOWN,
      mtc::core::ErrorCodes::MOTION_STATUS_UNKNOWN,
      "反馈断流，运动状态不确定；已锁定新的运动请求，禁止自动重发");
    result.status_unknown = true;
    result.final_position_rad = position_;
    return result;
  }

  const double budget_s = static_cast<double>(command.timeout_ms) / 1000.0;
  const double simulated_s = std::min(config_.travel_time_s, budget_s);

  if (config_.fail_at == "timeout" || config_.timeout ||
    config_.travel_time_s + validation_.settle_duration_s > budget_s) {
    advance_time(budget_s);
    if (cancel_requested_) {
      guard_.release(command.command_id);
      active_command_.reset();
      mtc::core::MotionResult result = make_result(
        false, false,
        stop_confirmed_ ? mtc::core::ExecState::STOPPED : mtc::core::ExecState::STOP_UNKNOWN,
        stop_confirmed_ ? mtc::core::ErrorCodes::OK : mtc::core::ErrorCodes::CANCEL_NOT_CONFIRMED,
        stop_confirmed_ ? "已取消并确认停稳" : "已取消，但后端未确认停稳");
      result.stop_confirmed = stop_confirmed_;
      result.status_unknown = !stop_confirmed_;
      result.final_position_rad = position_;
      return result;
    }
    guard_.release(command.command_id);
    active_command_.reset();
    mtc::core::MotionResult result = make_result(
      false, false, mtc::core::ExecState::TIMEOUT, mtc::core::ErrorCodes::MOTION_TIMEOUT,
      "运动超时（预算 " + std::to_string(command.timeout_ms) + " ms）");
    result.timed_out = true;
    result.status_unknown = true;
    result.final_position_rad = position_;
    return result;
  }

  // Constant-time virtual movement: no sleep or duration-proportional loop under the lock.
  // Concurrent dispatch/cancel is serialized for this instantaneous offline simulation.
  advance_time(simulated_s);

  if (cancel_requested_) {
    guard_.release(command.command_id);
    active_command_.reset();
    mtc::core::MotionResult result = make_result(
      false, false,
      stop_confirmed_ ? mtc::core::ExecState::STOPPED : mtc::core::ExecState::STOP_UNKNOWN,
      stop_confirmed_ ? mtc::core::ErrorCodes::OK : mtc::core::ErrorCodes::CANCEL_NOT_CONFIRMED,
      stop_confirmed_ ? "已取消并确认停稳" : "已取消，但后端未确认停稳");
    result.stop_confirmed = stop_confirmed_;
    result.status_unknown = !stop_confirmed_;
    result.final_position_rad = position_;
    return result;
  }

  position_ = command.target_rad;
  velocity_.assign(position_.size(), 0.0);
  const double stationary_since = clock_->now();
  advance_time(validation_.settle_duration_s);
  guard_.release(command.command_id);
  active_command_.reset();

  mtc::core::MotionResult result = make_result(
    true, true, mtc::core::ExecState::SETTLED, mtc::core::ErrorCodes::OK, "已到位并停稳");
  result.stop_confirmed = true;
  stop_confirmed_ = true;
  mtc::core::StopEvidence evidence;
  evidence.command_id = command.command_id;
  evidence.exec_state = mtc::core::ExecState::SETTLED;
  evidence.communication_ok = true;
  evidence.queue_empty = true;
  evidence.stationary_interval_verified = true;
  evidence.feedback = joint_feedback();
  evidence.stationary_since_s = stationary_since;
  evidence_ = evidence; // Virtual stationary interval, exclusively for offline Mock.
  result.final_position_rad = position_;
  return result;
}

mtc::core::StopResult MockMotionAdapter::request_stop(const std::string & reason)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  mtc::core::StopResult result;

  if (!active_command_.has_value()) {
    // Missing command records never constitute stationary evidence.
    result.request_delivered = true;
    result.stop_confirmed = evidence_.has_value() && stop_confirmed_;
    result.message = "无在途命令；停稳必须由关联证据确认：" + reason;
    return result;
  }

  cancel_requested_ = true;
  stop_confirmed_ = config_.cancel_confirms;
  result.request_delivered = true;
  result.stop_confirmed = stop_confirmed_;

  if (stop_confirmed_) {
    velocity_.assign(velocity_.size(), 0.0);
    const double stationary_since = clock_->now();
    advance_time(validation_.settle_duration_s);
    mtc::core::StopEvidence evidence;
    evidence.command_id = *active_command_;
    evidence.exec_state = mtc::core::ExecState::STOPPED;
    evidence.communication_ok = true;
    evidence.queue_empty = true;
    evidence.stationary_interval_verified = true;
    evidence.feedback = joint_feedback();
    evidence.stationary_since_s = stationary_since;
    evidence_ = evidence;
    guard_.release(*active_command_);
    result.message = "停止请求已送达并确认停稳（原因：" + reason + "）";
    return result;
  }
  result.message = "停止请求已送达，但尚未确认停稳（原因：" + reason + "）";
  return result;
}

std::optional<bool> MockMotionAdapter::explicit_stop_confirmation() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return stop_confirmed_;
}

mtc::core::StopResult MockMotionAdapter::request_stop_for_command(
  const std::string & reason, const std::optional<std::string> & command_id)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (command_id && (command_id->empty() ||
    (active_command_ && active_command_ != command_id))) {
    mtc::core::StopResult result;
    result.error_code = mtc::core::ErrorCodes::EXECUTION_REJECTED;
    result.message = "command_id mismatch; current Mock motion was not stopped";
    return result;
  }
  if (command_id) {
    cancelled_ids_.insert(*command_id);
    if (!active_command_ && (!evidence_ || evidence_->command_id != *command_id)) {
      mtc::core::StopResult result;
      result.request_delivered = true;
      result.message = "goal-scoped cancel reserved before dispatch; no stationary evidence";
      return result;
    }
  }
  return request_stop(reason);
}

std::optional<mtc::core::StopEvidence> MockMotionAdapter::stop_evidence() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return evidence_;
}

bool MockMotionAdapter::sample_stationary_feedback(const std::string & command_id)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (command_id.empty() || !evidence_ || evidence_->command_id != command_id ||
    active_command_ || !guard_.is_free() || !stop_confirmed_ ||
    !evidence_->communication_ok || !evidence_->queue_empty ||
    !evidence_->stationary_interval_verified) {return false;}

  // This adapter owns the entire instantaneous virtual plant under this lock.
  // A new execution invalidates evidence before changing that plant. Observe its
  // current state; do not relabel a cached Result or invent a stationary interval.
  auto sample = joint_feedback();
  if (!std::isfinite(sample.stamp_s) || !std::isfinite(evidence_->feedback.stamp_s) ||
    sample.stamp_s < evidence_->feedback.stamp_s ||
    sample.position_rad != evidence_->feedback.position_rad ||
    sample.velocity_rad_s != evidence_->feedback.velocity_rad_s) {return false;}
  evidence_->feedback = std::move(sample);
  return true;
}

bool MockMotionAdapter::force_begin_in_flight(const std::string & command_id)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const auto acquired = guard_.acquire(command_id);
  if (!acquired.first) {
    return false;
  }
  active_command_ = command_id;
  cancel_requested_ = false;
  stop_confirmed_ = false;
  evidence_.reset();
  return true;
}

void MockMotionAdapter::force_end_in_flight()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  guard_.force_release();
  active_command_.reset();
  cancel_requested_ = false;
}

mtc::core::MotionResult MockMotionAdapter::execute_trajectory(
  const mtc::core::TimedTrajectory & trajectory)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // 记录调用以便测试断言"轨迹只被当作一条轨迹提交一次"，
  // 而不是被拆成多次单点调用。
  trajectory_call_log_.push_back(trajectory);

  const ValidationOutcome validation =
    validate_timed_trajectory(trajectory, validation_, capabilities_);
  if (!validation.ok) {
    return make_result(
      false, false, mtc::core::ExecState::REJECTED, validation.error_code, validation.message);
  }
  return make_result(
    true, true, mtc::core::ExecState::SETTLED, mtc::core::ErrorCodes::OK,
    "轨迹已在 Mock 后端执行完毕（" + std::to_string(trajectory.points.size()) + " 个路点）");
}

// ---------------------------------------------------------------------------
// DisabledMotionAdapter
// ---------------------------------------------------------------------------

DisabledMotionAdapter::DisabledMotionAdapter(std::string reason)
: reason_(std::move(reason))
{
}

mtc::core::MotionCapabilities DisabledMotionAdapter::capabilities() const
{
  mtc::core::MotionCapabilities capabilities;
  capabilities.backend_name = "disabled";
  capabilities.joint_goal_supported = false;
  capabilities.timed_trajectory_supported = false;
  capabilities.cartesian_motion_supported = false;
  capabilities.cancel_supported = false;
  capabilities.detail = reason_;
  return capabilities;
}

mtc::core::MotionResult DisabledMotionAdapter::execute_joint_move(
  const mtc::core::MotionCommand & command)
{
  return make_result(
    false, false, mtc::core::ExecState::REJECTED, mtc::core::ErrorCodes::EXECUTION_REJECTED,
    reason_ + "（command_id=" + command.command_id + "）");
}

mtc::core::StopResult DisabledMotionAdapter::request_stop(const std::string & reason)
{
  mtc::core::StopResult result;
  result.request_delivered = false;
  result.stop_confirmed = false;
  result.message = reason_ + "；未送达任何真实停止请求（原因：" + reason + "）";
  return result;
}

mtc::core::JointFeedback DisabledMotionAdapter::joint_feedback() const
{
  return mtc::core::JointFeedback{};
}

// ---------------------------------------------------------------------------
// 工厂
// ---------------------------------------------------------------------------

MotionAdapterPtr make_adapter(
  const std::string & name,
  MockMotionAdapterConfig config,
  mtc::core::ClockPtr clock,
  ValidationConfig validation)
{
  std::string normalized = name;
  std::transform(
    normalized.begin(), normalized.end(), normalized.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});

  if (normalized == "mock") {
    return std::make_shared<MockMotionAdapter>(std::move(config), std::move(clock),
             std::move(validation));
  }
  return std::make_shared<DisabledMotionAdapter>(
    "不支持或未启用的运动后端 \"" + name + "\"；仅 mock 可用于离线运行");
}

}  // namespace motion
}  // namespace mtc
