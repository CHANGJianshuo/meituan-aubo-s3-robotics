// Copyright 2026 MTC contributors
#pragma once
#include <mtc_interfaces/action/execute_joint_move.hpp>
#include "mtc_ros2_action_cpp/action_slot.hpp"
#include "mtc_ros2_action_cpp/mock_ports.hpp"

namespace mtc::ros2_action
{
class MotionServer : public rclcpp::Node
{
public:
  explicit MotionServer(MockOptions options = {},
    const rclcpp::NodeOptions & node = rclcpp::NodeOptions());
  ~MotionServer() override;
  std::shared_ptr<const MockCallCounts> mock_call_counts() const {return counts_;}
private:
  using Action = mtc_interfaces::action::ExecuteJointMove;
  void tick();
  void finish(const mtc::core::MotionResult & result, bool canceled);
  MockOptions options_;
  std::shared_ptr<MockCallCounts> counts_ = std::make_shared<MockCallCounts>();
  rclcpp::CallbackGroup::SharedPtr group_;
  std::shared_ptr<ActionSlot<Action>> slot_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<mtc::core::ManualClock> clock_;
  std::shared_ptr<AsyncMockMotion> motion_;
  mtc::core::MotionCommand command_;
  std::chrono::steady_clock::time_point deadline_;
  bool cleanup_stop_ = false;
};
mtc::core::MotionCommand joint_command(const mtc_interfaces::action::ExecuteJointMove::Goal &);
}  // namespace mtc::ros2_action
