// Copyright 2026 MTC contributors
#pragma once
#include <mtc_interfaces/action/pick_place.hpp>
#include <mtc_manipulation_cpp/pick_place_fsm.hpp>
#include "mtc_ros2_action_cpp/action_slot.hpp"
#include "mtc_ros2_action_cpp/mock_ports.hpp"

namespace mtc::ros2_action
{
class PickPlaceServer : public rclcpp::Node
{
public:
  explicit PickPlaceServer(MockOptions options = {},
    const rclcpp::NodeOptions & node = rclcpp::NodeOptions());
  ~PickPlaceServer() override;
  std::shared_ptr<const MockCallCounts> mock_call_counts() const {return counts_;}
private:
  using Action = mtc_interfaces::action::PickPlace;
  void tick();
  MockOptions options_;
  std::shared_ptr<MockCallCounts> counts_ = std::make_shared<MockCallCounts>();
  rclcpp::CallbackGroup::SharedPtr group_;
  std::shared_ptr<ActionSlot<Action>> slot_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<mtc::core::ManualClock> clock_;
  std::shared_ptr<AsyncMockMotion> motion_;
  std::shared_ptr<MockTool> tool_;
  std::unique_ptr<mtc::manipulation::PickPlaceFsm> fsm_;
  bool cleanup_stop_ = false;
};
bool valid_pick_goal(const mtc_interfaces::action::PickPlace::Goal & goal);
}  // namespace mtc::ros2_action
