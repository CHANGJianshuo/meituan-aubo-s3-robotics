// Copyright 2026 MTC contributors
#include "mtc_ros2_action_cpp/mock_ports.hpp"
#include <cmath>
#include <stdexcept>
#include <mtc_manipulation_cpp/tool_strategy.hpp>

namespace mtc::ros2_action
{
using mtc::core::ErrorCodes;
void MockOptions::validate() const
{
  if (!std::isfinite(tick_seconds) || tick_seconds <= 0 || tick_seconds > 0.1 ||
    result_delay_ms < 0 || result_delay_ms > 10000) {
    throw std::invalid_argument("invalid offline clock/delivery configuration");
  }
  mtc::manipulation::make_strategy(pick.tool_type);
  if (motion.timed_trajectory_supported || motion.cartesian_motion_supported) {
    throw std::invalid_argument("Phase 2A cannot advertise trajectory/Cartesian execution");
  }
}
MockPerception::MockPerception(MockOptions o, mtc::core::ClockPtr c, int color)
: options_(std::move(o)), clock_(std::move(c)), color_(color) {}
std::optional<mtc::core::DetectionResult> MockPerception::get_detection(
  const std::string & id, std::optional<int> expected, double)
{
  mtc::core::DetectionResult r;
  r.found = options_.target_found;
  r.unique = true;
  mtc::core::PoseEstimate p;
  p.object_id = id;
  p.color = color_;
  p.stamp_s = clock_->now();
  p.ring_pose.position = {0.4, -0.1, 0.25};
  p.ring_pose.orientation = {0, 0, 0, 1};
  p.ring_pose.frame_id = "base_link";
  p.ring_pose.stamp_s = p.stamp_s;
  p.ring_pose.valid = true;
  p.body_pose = p.ring_pose;
  p.has_ring_pose_value = p.has_body_pose_value = true;
  p.confidence = 0.9f;
  p.followed = options_.attached;
  p.at_target = !expected.has_value();
  p.settled = options_.placement_stable;
  r.pose = p;  // Each poll is a new, timestamped simulated observation.
  return r;
}
std::optional<mtc::core::PlanResult> MockPlanner::plan(
  const mtc::manipulation::PlanTarget & target)
{
  mtc::core::PlanResult r;
  r.ok = options_.plan_available;
  r.error_code = r.ok ? ErrorCodes::OK : ErrorCodes::PLANNING_FAILED;
  r.message = r.ok ? "explicit offline joint plan" : "Mock planning failure";
  if (r.ok) {
    for (const auto & intent : target.primitives) {
      mtc::core::MotionCommand c;
      c.primitive_name = intent.name;
      c.intent = intent;
      c.requires_contact = intent.requires_contact;
      c.joint_names = options_.pick.joint_names;
      c.target_rad = {0.05, -0.02, 0.01, 0.03, -0.04, 0.02};
      c.simulation_only = true;  // Never admissible on an executable backend.
      c.velocity_scaling = options_.pick.default_velocity_scaling;
      c.acceleration_scaling = options_.pick.default_acceleration_scaling;
      c.timeout_ms = options_.pick.default_timeout_ms;
      r.commands.push_back(c);
    }
  }
  return r;
}
mtc::core::ToolEvidence MockTool::attachment() const
{
  mtc::core::ToolEvidence e;
  e.ok = options_.attached;
  e.verified = options_.attached;
  e.state = options_.attached ? mtc::core::ToolStateCode::STATE_ATTACHED :
    mtc::core::ToolStateCode::STATE_UNKNOWN;
  e.evidence_level = options_.attached ? mtc::core::EvidenceLevel::GEOMETRY :
    mtc::core::EvidenceLevel::NONE;
  e.error_code = e.ok ? ErrorCodes::OK : ErrorCodes::ATTACH_NOT_VERIFIED;
  e.message = "simulated geometric attachment evidence; no IO";
  return e;
}
std::optional<mtc::core::ToolEvidence> MockTool::engage(
  const std::string &, const mtc::core::MotionPrimitiveList &)
{
  const auto e = attachment();
  holding_ = e.ok;
  unknown_load_ = !e.ok;
  return e;
}
std::optional<mtc::core::ToolEvidence> MockTool::test_lift(
  const mtc::core::MotionPrimitiveList &)
{
  auto e = attachment();
  if (!holding_ || unknown_load_) {
    e.ok = e.verified = false;
    e.state = mtc::core::ToolStateCode::STATE_UNKNOWN;
    e.evidence_level = mtc::core::EvidenceLevel::NONE;
    e.error_code = ErrorCodes::ATTACH_NOT_VERIFIED;
  }
  return e;
}
mtc::core::ToolEvidence MockTool::seat_verify()
{
  auto e = attachment();
  e.ok = e.seated = options_.seated;
  e.error_code = e.ok ? ErrorCodes::OK : ErrorCodes::SEAT_NOT_VERIFIED;
  return e;
}
std::optional<mtc::core::ToolEvidence> MockTool::unload(
  const mtc::core::MotionPrimitiveList &)
{
  auto e = seat_verify();
  e.unloaded = e.seated;
  return e;
}
std::optional<mtc::core::ToolEvidence> MockTool::unlock(uint32_t)
{
  auto e = seat_verify();
  e.ok = e.ok && options_.released;
  e.released = e.ok;
  e.error_code = e.ok ? ErrorCodes::OK : ErrorCodes::UNLOCK_FAILED;
  if (e.ok) {
    holding_ = unknown_load_ = false;
    e.state = mtc::core::ToolStateCode::STATE_DETACHED;
  }
  return e;
}
std::optional<mtc::core::ToolEvidence> MockTool::disengage(
  const mtc::core::MotionPrimitiveList &)
{
  auto e = seat_verify();
  e.ok = e.ok && options_.released;
  e.released = e.ok;
  e.error_code = e.ok ? ErrorCodes::OK : ErrorCodes::RELEASE_NOT_VERIFIED;
  if (e.ok) {
    holding_ = unknown_load_ = false;
    e.state = mtc::core::ToolStateCode::STATE_DETACHED;
  }
  return e;
}
AsyncMockMotion::AsyncMockMotion(MockOptions o, std::shared_ptr<mtc::core::ManualClock> c,
  std::shared_ptr<MockCallCounts> counts)
: options_(std::move(o)), adapter_(std::make_shared<CountingMockAdapter>(
    options_.motion, c, std::move(counts))),
  executor_(std::make_shared<mtc::motion::MotionExecutorCore>(adapter_,
    mtc::motion::ValidationConfig(), c)) {}
bool AsyncMockMotion::execution_ready() const
{
  return !execution_.valid() || execution_.wait_for(std::chrono::seconds(0)) ==
    std::future_status::ready;
}
std::optional<mtc::core::MotionResult> AsyncMockMotion::execute_joint_move(
  const mtc::core::MotionCommand & c)
{
  if (last_id_ != c.command_id) {
    if (!execution_ready() || stopping_.valid()) {
      throw std::logic_error("new motion before previous worker drained");
    }
    if (execution_.valid()) {execution_.get();}
    last_id_ = c.command_id;
    result_.reset();
    stop_.reset();
    deliver_at_ = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(options_.result_delay_ms);
    execution_ = std::async(std::launch::async,
      [executor = executor_, c]() {return executor->execute_joint_move(c);});
    return std::nullopt;
  }
  if (!execution_ready() || std::chrono::steady_clock::now() < deliver_at_) {
    return std::nullopt;
  }
  if (!result_ && execution_.valid()) {result_ = execution_.get();}
  return result_;
}
bool AsyncMockMotion::poll_stop(const std::string & reason)
{
  if (stop_) {return true;}
  // Freeze FSM dispatch while cancelling; wait asynchronously for the constant-time
  // Mock execute to register its identity. Never call global stop on an absent ID.
  if (!execution_ready()) {return false;}
  if (!stopping_.valid()) {
    stopping_ = std::async(std::launch::async,
      [executor = executor_, id = last_id_, reason, available = options_.stop_evidence_available]() {
        auto r = executor->cancel_goal(id, reason);
        if (!available) {
          r.stop_confirmed = false;
          r.error_code = ErrorCodes::CANCEL_NOT_CONFIRMED;
          r.message = "Mock scenario withholds independent stop evidence";
        }
        return r;
      });
    return false;
  }
  if (stopping_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {return false;}
  stop_ = stopping_.get();
  return true;
}
mtc::core::StopResult AsyncMockMotion::request_stop(const std::string &)
{
  if (stop_) {return *stop_;}
  mtc::core::StopResult r;
  r.error_code = ErrorCodes::CANCEL_NOT_CONFIRMED;
  r.message = "no completed associated stop evidence";
  return r;
}
}  // namespace mtc::ros2_action
