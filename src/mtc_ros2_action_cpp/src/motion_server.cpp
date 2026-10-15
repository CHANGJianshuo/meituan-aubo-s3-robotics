// Copyright 2026 MTC contributors
#include "mtc_ros2_action_cpp/motion_server.hpp"
#include <algorithm>

namespace mtc::ros2_action
{
using mtc::core::ErrorCodes;
mtc::core::MotionCommand joint_command(const mtc_interfaces::action::ExecuteJointMove::Goal & g)
{
  mtc::core::MotionCommand c;
  c.command_id = g.command_id;
  c.joint_names.assign(g.joint_names.begin(), g.joint_names.end());
  c.target_rad.assign(g.target_rad.begin(), g.target_rad.end());
  const mtc::motion::ValidationConfig config;
  // ROS float32 encodes 0.2 as 0.20000000298... . Canonicalize only the
  // exact representable boundary; every other value is checked unchanged.
  c.velocity_scaling = g.velocity_scaling == static_cast<float>(config.max_velocity_scaling) ?
    config.max_velocity_scaling : static_cast<double>(g.velocity_scaling);
  c.acceleration_scaling = g.acceleration_scaling == static_cast<float>(config.max_acceleration_scaling) ?
    config.max_acceleration_scaling : static_cast<double>(g.acceleration_scaling);
  c.timeout_ms = g.timeout_ms;
  c.simulation_only = true;
  return c;
}
MotionServer::MotionServer(MockOptions options, const rclcpp::NodeOptions & node)
: Node("mtc_mock_joint_move", node), options_(std::move(options)),
  group_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
  slot_(std::make_shared<ActionSlot<Action>>()),
  clock_(std::make_shared<mtc::core::ManualClock>(1.0)),
  motion_(std::make_shared<AsyncMockMotion>(options_, clock_, counts_))
{
  options_.validate();
  server_ = make_server<Action>(this, "/mtc/motion/execute_joint_move", slot_, group_,
    [capabilities = motion_->adapter()->capabilities()](const Action::Goal & g) {
      return g.command_id.find_first_not_of(" \t\r\n") != std::string::npos &&
        mtc::motion::validate_joint_move(joint_command(g),
        mtc::motion::ValidationConfig(), capabilities).ok;
    }, [](const Action::Goal & g) {return g.command_id;});
  timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {tick();}, group_);
}
MotionServer::~MotionServer() {timer_->cancel(); motion_.reset();}
void MotionServer::tick()
{
  if (slot_->pending) {
    slot_->active = std::move(slot_->pending);
    command_ = joint_command(*slot_->active->get_goal());
    deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(command_.timeout_ms);
    motion_->execute_joint_move(command_);
  }
  if (!slot_->active) {
    if (cleanup_stop_ && motion_->poll_stop("fault cleanup of original motion command")) {
      cleanup_stop_ = false;
    }
    return;
  }
  const auto sample = motion_->feedback();
  auto feedback = std::make_shared<Action::Feedback>();
  if (sample.position_rad.size() == 6) {
    std::copy(sample.position_rad.begin(), sample.position_rad.end(), feedback->actual_position_rad.begin());
  }
  if (sample.velocity_rad_s.size() == 6) {
    std::copy(sample.velocity_rad_s.begin(), sample.velocity_rad_s.end(), feedback->actual_velocity_rad_s.begin());
  }
  const auto snapshot = motion_->snapshot();
  feedback->execution_state = mtc::core::to_string(snapshot.exec_state);
  feedback->progress = snapshot.stop_confirmed ? 1.0f : 0.0f;
  slot_->active->publish_feedback(feedback);
  const bool timeout = std::chrono::steady_clock::now() >= deadline_;
  if (slot_->cancel || timeout) {
    if (slot_->cancel && !slot_->active->is_canceling()) {return;}
    if (!motion_->poll_stop(slot_->cancel ? "associated ROS motion cancel" : "motion deadline")) {return;}
    const auto stop = motion_->request_stop("");
    mtc::core::MotionResult result;
    result.stop_confirmed = stop.stop_confirmed;
    result.status_unknown = !stop.stop_confirmed;
    result.final_position_rad = motion_->feedback().position_rad;
    result.timed_out = timeout && !slot_->cancel;
    result.error_code = slot_->cancel ? (stop.stop_confirmed ? ErrorCodes::OK :
      ErrorCodes::CANCEL_NOT_CONFIRMED) : ErrorCodes::MOTION_TIMEOUT;
    result.message = slot_->cancel ? stop.message : "ROS motion deadline exceeded: " + stop.message;
    finish(result, slot_->cancel && stop.stop_confirmed);
    return;
  }
  if (auto result = motion_->execute_joint_move(command_)) {finish(*result, false);}
}
void MotionServer::finish(const mtc::core::MotionResult & r, bool canceled)
{
  auto result = std::make_shared<Action::Result>();
  result->success = r.success;
  result->stop_confirmed = r.stop_confirmed;
  if (r.final_position_rad.size() == 6) {
    std::copy(r.final_position_rad.begin(), r.final_position_rad.end(), result->final_position_rad.begin());
  }
  result->error_code = r.error_code;
  result->message = r.message;
  slot_->locked = r.status_unknown || motion_->snapshot().motion_state_unknown ||
    ((r.timed_out || slot_->cancel) && !r.stop_confirmed);
  cleanup_stop_ = slot_->locked;
  if (canceled && slot_->active->is_canceling()) {slot_->active->canceled(result);}
  else if (r.success && r.stop_confirmed && !slot_->cancel) {slot_->active->succeed(result);}
  else {slot_->active->abort(result);}
  slot_->release();
}
}  // namespace mtc::ros2_action
