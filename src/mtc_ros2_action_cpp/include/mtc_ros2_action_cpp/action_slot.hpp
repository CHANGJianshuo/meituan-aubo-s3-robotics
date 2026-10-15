// Copyright 2026 MTC contributors
#pragma once
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace mtc::ros2_action
{
// All access belongs to one mutually-exclusive ROS callback group. Worker threads
// never touch this admission slot or any FSM. Reservation precedes ACCEPT.
template<class Action>
struct ActionSlot
{
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  std::optional<rclcpp_action::GoalUUID> owner;
  std::shared_ptr<Handle> pending;
  std::shared_ptr<Handle> active;
  std::set<std::string> used_ids;
  bool cancel = false;
  bool locked = false;

  void release()
  {
    pending.reset();
    active.reset();
    owner.reset();
    cancel = false;  // locked deliberately survives action completion.
  }
};

template<class Action, class Validator, class Id>
typename rclcpp_action::Server<Action>::SharedPtr make_server(
  rclcpp::Node * node, const std::string & name,
  const std::shared_ptr<ActionSlot<Action>> & slot,
  rclcpp::CallbackGroup::SharedPtr group, Validator validate, Id id)
{
  return rclcpp_action::create_server<Action>(node, name,
    [slot, validate, id](const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const typename Action::Goal> goal) {
      const auto key = id(*goal);
      if (slot->locked || slot->owner || slot->used_ids.count(key) || !validate(*goal)) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      slot->owner = uuid;
      slot->used_ids.insert(key);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [slot](std::shared_ptr<typename ActionSlot<Action>::Handle> handle) {
      if (!slot->owner || *slot->owner != handle->get_goal_id() || slot->locked) {
        return rclcpp_action::CancelResponse::REJECT;
      }
      slot->cancel = true;  // ACK is not a stationary result.
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [slot](std::shared_ptr<typename ActionSlot<Action>::Handle> handle) {
      slot->pending = handle;
    }, rcl_action_server_get_default_options(), group);
}
}  // namespace mtc::ros2_action
