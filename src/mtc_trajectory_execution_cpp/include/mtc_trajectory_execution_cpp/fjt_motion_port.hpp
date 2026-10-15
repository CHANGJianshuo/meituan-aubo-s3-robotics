// Copyright 2026 MTC contributors
#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <mtc_manipulation_cpp/pick_place_types.hpp>
#include "mtc_trajectory_execution_cpp/trajectory_validator.hpp"
#include "mtc_trajectory_execution_cpp/execution_state.hpp"
namespace mtc::trajectory
{
// The supplied node must be spun by its owner's executor. No FSM callback is invoked here.
// Default disabled. Runtime GenericSystem, active controller and fresh state proofs required.
class FjtMotionPort final : public mtc::manipulation::MotionPort
{
public:
  FjtMotionPort(rclcpp::Node::SharedPtr node, Config config);
  ~FjtMotionPort() override;
  FjtMotionPort(const FjtMotionPort &) = delete;
  FjtMotionPort & operator=(const FjtMotionPort &) = delete;
  std::optional<mtc::core::MotionResult> execute_joint_move(
    const mtc::core::MotionCommand & command) override;
  std::optional<mtc::core::MotionResult> execute_trajectory(
    const mtc::core::MotionCommand & command) override;
  // Polling stop: direct FSM use conservatively faults if confirmation isn't ready yet.
  mtc::core::StopResult request_stop(const std::string & reason) override;
  // Additional scoped API; never upgrades mismatched/old IDs to global cancel.
  mtc::core::StopResult cancel_command(const std::string & id, const std::string & reason);
  ExecutionSnapshot snapshot() const;
private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};
}  // namespace mtc::trajectory
