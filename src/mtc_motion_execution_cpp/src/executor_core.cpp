// Copyright 2026 meituan_challenge_ws contributors
#include "mtc_motion_execution_cpp/executor_core.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace mtc { namespace motion {
namespace {
mtc::core::MotionResult rejected(uint16_t code, const std::string & message, bool unknown = false)
{
  mtc::core::MotionResult result;
  result.error_code = code;
  result.message = message;
  result.status_unknown = unknown;
  result.exec_state = unknown ? mtc::core::ExecState::STOP_UNKNOWN : mtc::core::ExecState::REJECTED;
  return result;
}
}
void HandshakeRecord::mark(const std::string & step, const std::string & detail)
{
  steps.push_back(detail.empty() ? step : step + ":" + detail);
}
MotionExecutorCore::MotionExecutorCore(
  MotionAdapterPtr adapter, ValidationConfig config, mtc::core::ClockPtr clock)
: adapter_(std::move(adapter)), config_(std::move(config))
{
  if (!adapter_) {throw std::invalid_argument("MotionAdapter must not be null");}
  adapter_clock_ = adapter_->execution_clock();
  clock_ = clock ? std::move(clock) : (adapter_clock_ ? adapter_clock_ :
    std::make_shared<mtc::core::SteadyClock>());
}
mtc::core::MotionCapabilities MotionExecutorCore::capabilities() const
{
  return adapter_->capabilities();
}
ExecutorSnapshot MotionExecutorCore::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {exec_state_, active_command_id_, last_error_code_, motion_state_unknown_, stop_confirmed_};
}
std::optional<HandshakeRecord> MotionExecutorCore::handshake(const std::string & id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = handshakes_.find(id);
  if (found == handshakes_.end()) {return std::nullopt;}
  return found->second; // No pointer into concurrently mutated storage.
}
bool MotionExecutorCore::evidence_valid(
  const std::optional<mtc::core::StopEvidence> & evidence,
  const std::optional<std::string> & command_id) const
{
  using mtc::core::ExecState;
  if (!evidence || !command_id || command_id->empty() ||
    (adapter_clock_ && adapter_clock_ != clock_)) {return false;}
  const auto & e = *evidence;
  const auto & f = e.feedback;
  const double now = clock_->now();
  for (double threshold : {config_.feedback_max_age_s, config_.settle_duration_s,
      config_.settle_velocity_tolerance_rad_s}) {
    if (!std::isfinite(threshold) || threshold <= 0.0) {return false;}
  }
  const double stationary_for = f.stamp_s - e.stationary_since_s;
  const double time_tolerance = std::min(1e-9, config_.settle_duration_s * 1e-9);
  if (e.command_id != *command_id || !e.communication_ok || !e.queue_empty ||
    !e.stationary_interval_verified ||
    (e.exec_state != ExecState::STOPPED && e.exec_state != ExecState::SETTLED) ||
    !std::isfinite(now) || !std::isfinite(f.stamp_s) || f.stamp_s < 0.0 ||
    !std::isfinite(e.stationary_since_s) || e.stationary_since_s < 0.0 ||
    f.stamp_s > now || now - f.stamp_s > config_.feedback_max_age_s ||
    !std::isfinite(stationary_for) || stationary_for <= 0.0 ||
    stationary_for + time_tolerance < config_.settle_duration_s ||
    f.position_rad.size() != config_.expected_joint_names.size() ||
    f.velocity_rad_s.size() != f.position_rad.size() || f.position_rad.empty())
  {return false;}
  for (size_t i = 0; i < f.position_rad.size(); ++i) {
    if (!std::isfinite(f.position_rad[i]) || !std::isfinite(f.velocity_rad_s[i]) ||
      std::abs(f.velocity_rad_s[i]) > config_.settle_velocity_tolerance_rad_s)
    {return false;}
  }
  return true;
}
mtc::core::MotionResult MotionExecutorCore::execute_joint_move(
  const mtc::core::MotionCommand & command)
{
  using mtc::core::ErrorCodes;
  using mtc::core::ExecState;
  HandshakeRecord record{command.command_id, {"PREPARE"}};
  uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motion_state_unknown_ || stops_pending_ != 0) {
      record.mark("PREPARE_REJECTED", "motion_state_unknown");
      handshakes_[command.command_id] = record;
      return rejected(ErrorCodes::MOTION_STATUS_UNKNOWN, "运动状态未确认；禁止自动重发", true);
    }
    if (active_command_id_) {
      record.mark("ACCEPT_REJECTED", "in_flight=" + *active_command_id_);
      if (command.command_id != *active_command_id_) {handshakes_[command.command_id] = record;}
      return rejected(ErrorCodes::EXECUTION_REJECTED, "已有在途命令 " + *active_command_id_);
    }
    if (executed_ids_.count(command.command_id)) {
      return rejected(ErrorCodes::EXECUTION_REJECTED, "command_id already dispatched; duplicate refused");
    }
    active_command_id_ = command.command_id;
    generation = ++generation_;
    cancel_requested_ = false;
    stop_confirmed_ = false;
    exec_state_ = ExecState::ACCEPTED;
    record.mark("ACCEPT");
  }
  mtc::core::MotionResult result;
  std::optional<mtc::core::StopEvidence> evidence;
  try {
    const auto capabilities = adapter_->capabilities();
    auto validation = validate_joint_move(command, config_, capabilities);
    const double now = clock_->now();
    if ((adapter_clock_ && adapter_clock_ != clock_) || !std::isfinite(now) || now < 0.0) {
      validation = {false, ErrorCodes::EXECUTION_REJECTED,
        "execution clock mismatch/invalid: Mock requires the shared ManualClock"};
    } else if (!capabilities.simulation_only &&
      dynamic_cast<mtc::core::ManualClock *>(clock_.get())) {
      validation = {false, ErrorCodes::SAFETY_INTERLOCK,
        "virtual ManualClock is forbidden for executable backends"};
    }
    if (!validation.ok) {
      result = rejected(validation.error_code, validation.message);
      record.mark("VALIDATE_REJECTED", validation.message);
    } else {
      record.mark("VALIDATE");
      bool cancelled_before_send;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelled_before_send = cancel_requested_;
        if (!cancelled_before_send) {
          exec_state_ = ExecState::MOVING;
          executed_ids_.insert(command.command_id);
        }
      }
      if (cancelled_before_send) {
        result = rejected(ErrorCodes::EXECUTION_REJECTED, "cancelled before dispatch");
      } else {
        // No state lock is held during backend execution or callbacks.
        record.mark("SEND");
        record.mark("MONITOR_REQUEST", "MOVING");
        result = adapter_->execute_joint_move(command);
        evidence = adapter_->stop_evidence();
      }
    }
  } catch (const std::exception & exc) {
    record.mark("MONITOR_EXCEPTION", exc.what());
    result = rejected(ErrorCodes::MOTION_STATUS_UNKNOWN,
      std::string("后端异常；禁止自动重发：") + exc.what(), true);
  } catch (...) {
    record.mark("MONITOR_EXCEPTION", "unknown");
    result = rejected(ErrorCodes::MOTION_STATUS_UNKNOWN, "后端异常；禁止自动重发", true);
  }
  const bool confirmed = result.stop_confirmed && evidence_valid(evidence, command.command_id);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != generation_ || active_command_id_ != command.command_id) {
      return rejected(ErrorCodes::MOTION_STATUS_UNKNOWN, "late result; identity mismatch", true);
    }
    const bool never_executed = result.exec_state == ExecState::REJECTED &&
      !result.status_unknown && !result.timed_out && !result.success;
    const bool terminal_evidence = confirmed &&
      (result.exec_state == ExecState::SETTLED || result.exec_state == ExecState::STOPPED);
    motion_state_unknown_ = motion_state_unknown_ || result.status_unknown ||
      (!never_executed && !terminal_evidence) || (cancel_requested_ && !stop_confirmed_);
    result.stop_confirmed = confirmed && !motion_state_unknown_;
    if (motion_state_unknown_) {
      result.status_unknown = true;
      result.success = false;
      result.arrived = false;
      if (result.error_code == ErrorCodes::OK) {result.error_code = ErrorCodes::MOTION_STATUS_UNKNOWN;}
      exec_state_ = ExecState::STOP_UNKNOWN;
      result.exec_state = ExecState::STOP_UNKNOWN;
    } else {
      result.success = result.success && result.arrived && result.stop_confirmed &&
        result.exec_state == ExecState::SETTLED && !result.timed_out && !cancel_requested_;
      exec_state_ = result.exec_state;
    }
    stop_confirmed_ = result.stop_confirmed;
    last_error_code_ = result.error_code;
    if (never_executed && !motion_state_unknown_) {executed_ids_.erase(command.command_id);}
    last_command_id_ = command.command_id;
    active_command_id_.reset();
    record.mark("MONITOR", mtc::core::to_string(result.exec_state));
    record.mark(result.success ? "CONFIRM" : "STOP_OR_FAULT", result.success ? "settled" : result.message);
    handshakes_[command.command_id] = record;
  }
  return result;
}
mtc::core::StopResult MotionExecutorCore::request_stop(
  const std::string & reason, std::optional<std::string> requested_id)
{
  return requested_id ? cancel_goal(*requested_id, reason) : supervisory_stop(reason);
}
mtc::core::StopResult MotionExecutorCore::cancel_goal(
  const std::string & command_id, const std::string & reason)
{
  return stop_impl(reason, command_id);
}
mtc::core::StopResult MotionExecutorCore::supervisory_stop(const std::string & reason)
{
  return stop_impl(reason, std::nullopt);
}
mtc::core::StopResult MotionExecutorCore::stop_impl(
  const std::string & reason, const std::optional<std::string> & requested_id)
{
  std::optional<std::string> expected;
  uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    expected = active_command_id_ ? active_command_id_ : last_command_id_;
    if (requested_id && (requested_id->empty() || requested_id != expected)) {
      mtc::core::StopResult rejected_stop;
      rejected_stop.error_code = mtc::core::ErrorCodes::EXECUTION_REJECTED;
      rejected_stop.message = "command_id mismatch: requested=" + *requested_id +
        " current=" + expected.value_or("<none>");
      return rejected_stop; // BEFORE cancel/state/pending updates or any backend call.
    }
    generation = generation_;
    ++stops_pending_;
    // Pins this generation even if its execution result drains outside this lock.
    // New admission remains forbidden until all accepted stop calls return.
    cancel_requested_ = true;
    stop_confirmed_ = false;
    exec_state_ = mtc::core::ExecState::CANCELLING;
  }
  mtc::core::StopResult result;
  bool cancel_supported = false;
  bool confirmed = false;
  try {
    const auto capabilities = adapter_->capabilities();
    cancel_supported = capabilities.cancel_supported;
    result = (requested_id || capabilities.associated_cancel_supported) ?
      adapter_->request_stop_for_command(reason, expected) : adapter_->request_stop(reason);
    confirmed = cancel_supported && result.stop_confirmed &&
      evidence_valid(adapter_->stop_evidence(), expected);
    if (!cancel_supported) {
      result.request_delivered = false;
      result.message = "cancel_supported=false: " + result.message;
    }
  } catch (...) {
    result = {};
    result.message = "stop backend exception; state unknown";
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    --stops_pending_;
    confirmed = confirmed && !motion_state_unknown_ && generation == generation_;
    motion_state_unknown_ = motion_state_unknown_ || !confirmed;
    stop_confirmed_ = confirmed;
    result.stop_confirmed = confirmed;
    exec_state_ = confirmed ? mtc::core::ExecState::STOPPED :
      (cancel_supported ? mtc::core::ExecState::CANCELLING : mtc::core::ExecState::STOP_UNKNOWN);
    if (!confirmed) {
      last_error_code_ = mtc::core::ErrorCodes::CANCEL_NOT_CONFIRMED;
      result.error_code = last_error_code_;
    }
  }
  return result;
}
void MotionExecutorCore::clear_unknown_after_manual_confirmation(const std::string & note)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (note.find_first_not_of(" \t\r\n") == std::string::npos || active_command_id_ || stops_pending_) {
    throw std::logic_error("manual confirmation needs a note and no outstanding callbacks");
  }
  motion_state_unknown_ = false;
  cancel_requested_ = false;
  exec_state_ = mtc::core::ExecState::IDLE;
  stop_confirmed_ = true;
  last_error_code_ = mtc::core::ErrorCodes::OK;
  auto & record = handshakes_["manual"];
  record.command_id = "manual";
  record.mark("MANUAL_CONFIRMATION", note);
}
} } // namespace mtc::motion
