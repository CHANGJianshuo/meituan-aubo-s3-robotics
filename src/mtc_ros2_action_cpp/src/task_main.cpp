// Copyright 2026 MTC contributors
#include "mtc_ros2_action_cpp/task_server.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mtc::ros2_action::TaskServer>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
