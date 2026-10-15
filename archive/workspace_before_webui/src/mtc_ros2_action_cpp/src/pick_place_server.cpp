// Copyright 2026 MTC contributors
#include "mtc_ros2_action_cpp/pick_place_server.hpp"
#include <algorithm>
#include <cmath>

namespace mtc::ros2_action
{
bool valid_pick_goal(const mtc_interfaces::action::PickPlace::Goal & g)
{
  const auto & slots = mtc::manipulation::allowed_target_slots();
  return g.request_id.find_first_not_of(" \t\r\n") != std::string::npos &&
    g.object_id.find_first_not_of(" \t\r\n") != std::string::npos &&
    g.expected_color >= 1 && g.expected_color <= 4 &&
    std::find(slots.begin(), slots.end(), g.target_slot) != slots.end() &&
    std::isfinite(g.placement_stability_sec) && g.placement_stability_sec > 0;
}
PickPlaceServer::PickPlaceServer(MockOptions options, const rclcpp::NodeOptions & node)
: Node("mtc_mock_pick_place", node), options_(std::move(options)),
  group_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
  slot_(std::make_shared<ActionSlot<Action>>())
{
  options_.validate();
  tool_ = std::make_shared<MockTool>(options_);
  server_ = make_server<Action>(this, "/mtc/manipulation/pick_place", slot_, group_,
    valid_pick_goal, [](const Action::Goal & g) {return g.request_id;});
  timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {tick();}, group_);
}
PickPlaceServer::~PickPlaceServer()
{
  timer_->cancel();
  fsm_.reset();
  motion_.reset();  // std::future joins owned constant-time Mock workers.
}
void PickPlaceServer::tick()
{
  using mtc::manipulation::PickPlaceState;
  if (slot_->pending) {
    slot_->active = std::move(slot_->pending);
    const auto g = slot_->active->get_goal();
    clock_ = std::make_shared<mtc::core::ManualClock>(1.0);
    motion_ = std::make_shared<AsyncMockMotion>(options_, clock_, counts_);
    fsm_ = std::make_unique<mtc::manipulation::PickPlaceFsm>(
      std::make_shared<MockPerception>(options_, clock_, g->expected_color),
      std::make_shared<MockPlanner>(options_), motion_, tool_,
      clock_, mtc::manipulation::make_strategy(options_.pick.tool_type), options_.pick);
    fsm_->start(g->object_id, g->expected_color, g->target_slot, g->request_id,
      g->placement_stability_sec);
  }
  if (!slot_->active) {
    if (cleanup_stop_ && motion_->poll_stop("fault cleanup of original command")) {
      cleanup_stop_ = false;
    }
    return;
  }
  try {
    if (slot_->cancel) {
      // ACK may arrive before ROS has installed CANCELING. Do not emit a result yet.
      if (!slot_->active->is_canceling()) {return;}
      if (!motion_->poll_stop("associated PickPlace cancel")) {return;}
      fsm_->request_cancel("ROS Goal cancel");
    } else {
      clock_->advance(options_.tick_seconds);
    }
    fsm_->step();
  } catch (const std::exception & e) {
    fsm_->force_fault(mtc::core::ErrorCodes::MOTION_STATUS_UNKNOWN, e.what());
  }
  const auto f = fsm_->feedback();
  auto feedback = std::make_shared<Action::Feedback>();
  feedback->substate = f.substate;
  feedback->progress = f.progress;
  feedback->object_attached_estimated = f.object_attached_estimated;
  feedback->detail = f.detail;
  if (!motion_->last_command_id().empty()) {
    feedback->detail += " | Mock command=" + motion_->last_command_id();
  }
  slot_->active->publish_feedback(feedback);
  if (!mtc::manipulation::terminal_states().count(fsm_->state())) {return;}
  const auto r = fsm_->result();
  auto result = std::make_shared<Action::Result>();
  result->success = r.success;
  result->placement_verified = r.placement_verified;
  result->error_code = r.error_code;
  result->message = r.message;
  // Ending the Action does not clear an attached/unknown load. Keep the Tool
  // simulation and admission lock until an explicit future recovery contract.
  slot_->locked = fsm_->state() == PickPlaceState::FAULT || tool_->holding_state() != "none";
  if (r.success && r.placement_verified && !slot_->cancel) {
    slot_->active->succeed(result);
  } else if (fsm_->state() == PickPlaceState::CANCELLED && fsm_->stop_confirmed() &&
    slot_->active->is_canceling()) {
    slot_->active->canceled(result);
  } else {
    slot_->active->abort(result);
  }
  cleanup_stop_ = slot_->locked;
  slot_->release();
}
}  // namespace mtc::ros2_action
