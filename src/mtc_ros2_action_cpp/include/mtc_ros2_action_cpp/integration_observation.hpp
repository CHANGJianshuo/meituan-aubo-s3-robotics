// Copyright 2026 MTC contributors
// Read-only integration observations, authored with OpenAI Codex, 2026-10-12.
#pragma once
#include <iomanip>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include "mtc_ros2_action_cpp/action_slot.hpp"
namespace mtc::ros2_action
{
inline std::string json_string(const std::string & text)
{
  std::ostringstream out;
  out << '"';
  for (unsigned char c : text) {
    if (c == '"' || c == '\\') {out << '\\' << c;}
    else if (c < 32) {out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);}
    else {out << c;}
  }
  out << '"';
  return out.str();
}
inline std::string goal_hex(const rclcpp_action::GoalUUID & id)
{
  std::ostringstream out;
  for (auto c : id) {out << std::hex << std::setw(2) << std::setfill('0') << int(c);}
  return out.str();
}
template<class Action>
std_msgs::msg::String integration_observation(rclcpp::Node & node,
  const ActionSlot<Action> & slot, const std::string & state, const std::string & extra = "")
{
  std_msgs::msg::String msg;
  std::ostringstream out;
  out << "{\"scope\":\"cpp_mock\",\"stamp_ns\":" << node.now().nanoseconds()
      << ",\"node\":" << json_string(node.get_fully_qualified_name())
      << ",\"state\":" << json_string(state)
      << ",\"admission_open\":" << (!slot.owner && !slot.locked ? "true" : "false")
      << ",\"locked\":" << (slot.locked ? "true" : "false")
      << ",\"cancel_requested\":" << (slot.cancel ? "true" : "false")
      << ",\"goal_id\":" << json_string(slot.owner ? goal_hex(*slot.owner) : "")
      << extra << "}";
  msg.data = out.str();
  return msg;
}
}  // namespace mtc::ros2_action
