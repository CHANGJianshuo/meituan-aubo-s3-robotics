// Copyright 2026 MTC contributors
#pragma once
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;
class RosFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);
    setenv("ROS_DOMAIN_ID", std::to_string(30 + getpid() % 170).c_str(), 1);
    setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
    unsetenv("ROS_STATIC_PEERS");
    unsetenv("ROS_LOCALHOST_ONLY");
    unsetenv("FASTRTPS_DEFAULT_PROFILES_FILE");
    unsetenv("FASTDDS_DEFAULT_PROFILES_FILE");
    unsetenv("CYCLONEDDS_URI");
    setenv("ROS_LOG_DIR", "/tmp/mtc_phase2a_test_logs", 1);
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    client_node = std::make_shared<rclcpp::Node>("phase2a_test_client");
    executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4);
    executor->add_node(client_node);
  }
  void start(std::vector<std::shared_ptr<rclcpp::Node>> nodes)
  {
    servers = std::move(nodes);
    for (auto & n : servers) {executor->add_node(n);}
    spin = std::thread([this]() {executor->spin();});
  }
  void TearDown() override
  {
    executor->cancel();
    if (spin.joinable()) {spin.join();}
    for (auto & n : servers) {executor->remove_node(n);}
    executor->remove_node(client_node);
    servers.clear();
    executor.reset();
    client_node.reset();
  }
  static bool eventually(const std::function<bool()> & f, std::chrono::milliseconds limit = 5s)
  {
    const auto end = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < end) {
      if (f()) {return true;}
      std::this_thread::sleep_for(5ms);  // Test thread only; bounded wait.
    }
    return f();
  }
  template<class T> static bool ready(T & f, std::chrono::milliseconds timeout = 10s)
  {return f.wait_for(timeout) == std::future_status::ready;}
  std::shared_ptr<rclcpp::Node> client_node;
  std::vector<std::shared_ptr<rclcpp::Node>> servers;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread spin;
};
