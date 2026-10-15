// Copyright 2026 MTC contributors
#pragma once
#include <mtc_interfaces/action/execute_task.hpp>
#include <mtc_interfaces/action/pick_place.hpp>
#include <mtc_task_cpp/task_fsm.hpp>
#include "mtc_ros2_action_cpp/action_slot.hpp"
#include "mtc_ros2_action_cpp/mock_ports.hpp"

namespace mtc::ros2_action
{
class TaskServer : public rclcpp::Node
{
public:
  explicit TaskServer(mtc::task::FsmsConfig config = mtc::task::FsmsConfig::defaults(),
    const rclcpp::NodeOptions & node = rclcpp::NodeOptions());
  ~TaskServer() override;
private:
  using Action = mtc_interfaces::action::ExecuteTask;
  using Child = mtc_interfaces::action::PickPlace;
  using ChildHandle = rclcpp_action::ClientGoalHandle<Child>;
  // Captured by value in client callbacks. Old callbacks can only change their
  // old inbox; they never reach the FSM or this Node after destruction.
  struct ChildInbox
  {
    std::string request_base;
    ChildHandle::SharedPtr handle;
    bool answered = false;
    bool cancel_wanted = false;
    bool cancel_sent = false;
    std::optional<ChildHandle::WrappedResult> result;
    Child::Feedback::ConstSharedPtr feedback;
  };
  void tick();
  void apply(const mtc::task::StepResult & step);
  void send_child(const mtc::task::PickPlaceGoal & goal);
  void poll_child();
  void finish();
  rclcpp::CallbackGroup::SharedPtr group_;
  std::shared_ptr<ActionSlot<Action>> slot_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp_action::Client<Child>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<mtc::core::SteadyClock> clock_;
  mtc::task::TaskFsm fsm_;
  bool initialized_ = false;
  size_t attempt_ = 0;
  std::optional<mtc::task::PickPlaceGoal> unsent_;
  std::shared_ptr<ChildInbox> child_;
};
mtc::task::TaskRequest task_request(const mtc_interfaces::action::ExecuteTask::Goal & goal);
}  // namespace mtc::ros2_action
