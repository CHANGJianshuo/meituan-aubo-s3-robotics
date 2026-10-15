// Copyright 2026 MTC contributors
#include "mtc_ros2_action_cpp/task_server.hpp"
#include <algorithm>

namespace mtc::ros2_action
{
using mtc::core::ErrorCodes;
using mtc::task::TaskStateName;
mtc::task::TaskRequest task_request(const mtc_interfaces::action::ExecuteTask::Goal & g)
{
  mtc::task::TaskRequest r;
  r.task_id = g.task_id;
  r.mode = g.mode;
  r.ordered_colors.assign(g.ordered_colors.begin(), g.ordered_colors.end());
  r.timeout_ms = g.timeout_ms;
  r.placement_stability_sec = g.placement_stability_sec;
  return r;
}
TaskServer::TaskServer(mtc::task::FsmsConfig config, const rclcpp::NodeOptions & node)
: Node("mtc_mock_task", node),
  group_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
  slot_(std::make_shared<ActionSlot<Action>>()),
  clock_(std::make_shared<mtc::core::SteadyClock>()), fsm_(clock_, config)
{
  client_ = rclcpp_action::create_client<Child>(this, "/mtc/manipulation/pick_place", group_);
  server_ = make_server<Action>(this, "/mtc/task/execute", slot_, group_,
    [](const Action::Goal & g) {
      return g.task_id.find_first_not_of(" \t\r\n") != std::string::npos &&
        mtc::task::validate_task_request(task_request(g)).ok;
    }, [](const Action::Goal & g) {return g.task_id;});
  timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {tick();}, group_);
}
TaskServer::~TaskServer() {timer_->cancel();}
void TaskServer::apply(const mtc::task::StepResult & step)
{
  for (const auto & action : step.actions) {
    switch (action.kind) {
      case mtc::task::ActionKind::CALL_PICK_PLACE:
        unsent_ = action.pick_place;
        break;
      case mtc::task::ActionKind::CANCEL_CHILD:
        if (child_ && child_->request_base == action.request_id) {
          child_->cancel_wanted = true;
        }
        unsent_.reset();  // A cancelled intent must never be dispatched later.
        break;
      case mtc::task::ActionKind::REQUEST_STOP:
        if (child_) {child_->cancel_wanted = true;}
        unsent_.reset();  // Scoped cancellation; no global motion service call.
        break;
      case mtc::task::ActionKind::LOCK_ACTIONS:
        slot_->locked = true;
        break;
      case mtc::task::ActionKind::PUBLISH_STATE:
        break;  // Complete feedback snapshot is published on each timer tick.
    }
  }
}
void TaskServer::send_child(const mtc::task::PickPlaceGoal & request)
{
  auto inbox = std::make_shared<ChildInbox>();
  inbox->request_base = request.request_id;
  child_ = inbox;
  Child::Goal goal;
  goal.request_id = request.request_id + "/attempt-" + std::to_string(++attempt_);
  goal.object_id = request.object_id;
  goal.expected_color = static_cast<uint8_t>(request.expected_color);
  goal.target_slot = request.target_slot;
  goal.placement_stability_sec = request.placement_stability_sec;
  auto options = rclcpp_action::Client<Child>::SendGoalOptions();
  options.goal_response_callback = [inbox](ChildHandle::SharedPtr handle) {
      inbox->handle = handle;
      inbox->answered = true;
    };
  options.feedback_callback = [inbox](ChildHandle::SharedPtr handle,
      const Child::Feedback::ConstSharedPtr feedback) {
      if (inbox->handle && inbox->handle->get_goal_id() == handle->get_goal_id()) {
        inbox->feedback = feedback;
      }
    };
  options.result_callback = [inbox](const ChildHandle::WrappedResult & result) {
      if (inbox->handle && inbox->handle->get_goal_id() == result.goal_id) {
        inbox->result = result;
      }
    };
  client_->async_send_goal(goal, options);
}
void TaskServer::poll_child()
{
  if (!child_) {return;}
  if (child_->cancel_wanted && child_->handle && !child_->cancel_sent && !child_->result) {
    child_->cancel_sent = true;
    // No ACK callback invokes confirm_child_stopped. Only a terminal result can.
    client_->async_cancel_goal(child_->handle);
  }
  if (!slot_->active) {
    if (child_->result || (child_->answered && !child_->handle)) {child_.reset();}
    return;
  }
  if (child_->feedback) {fsm_.substate = child_->feedback->substate;}
  if (child_->answered && !child_->handle) {
    const auto code = fsm_.state() == TaskStateName::CANCEL_PENDING ?
      ErrorCodes::CANCEL_NOT_CONFIRMED : ErrorCodes::EXECUTION_REJECTED;
    apply(fsm_.report_item_unsafe(code, "PickPlace Goal rejected; no stop evidence"));
    child_.reset();
    return;
  }
  if (!child_->result) {return;}
  const auto r = *child_->result;
  child_.reset();
  if (fsm_.state() == TaskStateName::CANCEL_PENDING) {
    if (r.code == rclcpp_action::ResultCode::CANCELED && r.result &&
      !r.result->success && !r.result->placement_verified && r.result->error_code == ErrorCodes::OK) {
      // Contract with this package's PickPlace server: CANCELED iff FSM confirmed
      // its associated stop. SUCCEEDED and Cancel ACK cannot stand in for it.
      apply(fsm_.confirm_child_stopped());
    } else {
      apply(fsm_.report_item_unsafe(r.result && r.result->error_code != ErrorCodes::OK ?
        r.result->error_code : ErrorCodes::CANCEL_NOT_CONFIRMED,
        r.result ? r.result->message : "child returned no cancellation result"));
    }
    return;
  }
  if (fsm_.state() != TaskStateName::EXECUTE_ITEM) {return;}
  if (!r.result || r.code == rclcpp_action::ResultCode::UNKNOWN ||
    r.code == rclcpp_action::ResultCode::CANCELED) {
    apply(fsm_.report_item_unsafe(ErrorCodes::MOTION_STATUS_UNKNOWN,
      "unexpected/missing child result; no automatic resend"));
  } else {
    const bool success = r.code == rclcpp_action::ResultCode::SUCCEEDED &&
      r.result->success && r.result->error_code == ErrorCodes::OK;
    apply(fsm_.report_item_result(success, r.result->placement_verified,
      r.result->error_code, r.result->message));
  }
}
void TaskServer::tick()
{
  if (!initialized_) {
    fsm_.start();
    // Software-only self check; no robot/controller or IO check is claimed.
    fsm_.report_self_check(mtc::task::TaskFsm::verify_tables().empty());
    initialized_ = true;
    slot_->locked = fsm_.fault_locked();
  }
  if (slot_->pending) {
    slot_->active = std::move(slot_->pending);
    if (fsm_.action_finished() && !fsm_.fault_locked()) {fsm_.reset();}
    apply(fsm_.submit_goal(task_request(*slot_->active->get_goal())));
  }
  if (!slot_->active) {poll_child(); return;}
  if (fsm_.state() == TaskStateName::VALIDATE_TASK) {apply(fsm_.advance_validation());}
  if (slot_->cancel && slot_->active->is_canceling() &&
    fsm_.state() != TaskStateName::CANCEL_PENDING && !fsm_.action_finished()) {
    apply(fsm_.request_cancel());
    if (!child_) {
      apply(fsm_.report_item_unsafe(ErrorCodes::CANCEL_NOT_CONFIRMED,
        "no associated child stop evidence; cancellation is not safe-confirmed"));
    }
  }
  poll_child();
  if (auto s = fsm_.check_task_deadline()) {apply(*s);}
  if (auto s = fsm_.check_timeouts()) {apply(*s);}
  if (!slot_->cancel) {
    switch (fsm_.state()) {
      case TaskStateName::SCAN_SCENE: {
        // Explicit synthetic scene observation; no detector or tool is connected.
        const auto color = fsm_.current_task->ordered_colors.at(fsm_.current_index);
        MockPerception scene(MockOptions(), clock_, color);
        const auto id = "mock_battery_color_" + std::to_string(color);
        const auto detection = scene.get_detection(id, color, 0.3);
        if (detection && detection->ok() && detection->pose->color == color) {
          apply(fsm_.report_target(detection->pose->object_id, color));
        } else {apply(fsm_.report_target_missing("Mock scene observation unavailable"));}
        break;
      }
      case TaskStateName::RECORD_RESULT: apply(fsm_.record_completed_item()); break;
      case TaskStateName::RECOVERY: apply(fsm_.request_retry()); break;
      default: break;
    }
  }
  if (unsent_ && !slot_->cancel && client_->action_server_is_ready()) {
    const auto goal = *unsent_;
    unsent_.reset();
    send_child(goal);
  }
  auto f = std::make_shared<Action::Feedback>();
  f->current_index = static_cast<uint8_t>(fsm_.current_index);
  f->total_count = static_cast<uint8_t>(fsm_.total_count());
  f->state = fsm_.state_name();
  f->current_slot = fsm_.current_slot;
  f->current_object_id = fsm_.current_object_id;
  f->substate = fsm_.substate;
  slot_->active->publish_feedback(f);
  if (fsm_.action_finished()) {finish();}
}
void TaskServer::finish()
{
  const auto outcome = fsm_.build_outcome();
  auto result = std::make_shared<Action::Result>();
  result->success = outcome.success;
  result->completed_count = outcome.completed_count;
  result->completed_slots = outcome.completed_slots;
  result->error_code = outcome.error_code;
  result->message = outcome.message;
  slot_->locked = slot_->locked || fsm_.fault_locked();
  if (outcome.success && !slot_->cancel) {slot_->active->succeed(result);}
  else if (fsm_.state() == TaskStateName::CANCELLED && slot_->active->is_canceling()) {
    slot_->active->canceled(result);
  } else {slot_->active->abort(result);}
  slot_->release();
}
}  // namespace mtc::ros2_action
