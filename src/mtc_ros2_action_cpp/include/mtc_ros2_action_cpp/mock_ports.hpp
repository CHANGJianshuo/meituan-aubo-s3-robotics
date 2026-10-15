// Copyright 2026 MTC contributors
#pragma once
#include <chrono>
#include <atomic>
#include <future>
#include <memory>
#include <optional>
#include <mtc_manipulation_cpp/pick_place_types.hpp>
#include <mtc_motion_execution_cpp/executor_core.hpp>
#include <mtc_motion_execution_cpp/mock_motion_adapter.hpp>

namespace mtc::ros2_action
{
// Explicit offline scenario. No device address, plugin loader, RPC, tool IO, or
// executable trajectory backend exists in this package.
struct MockOptions
{
  bool target_found = true;
  bool plan_available = true;
  bool attached = true;
  bool seated = true;
  bool released = true;
  bool placement_stable = true;
  bool stop_evidence_available = true;
  double tick_seconds = 0.02;
  int result_delay_ms = 100;  // Delay delivery, not physical/virtual movement.
  mtc::motion::MockMotionAdapterConfig motion;
  mtc::manipulation::PickPlaceConfig pick;
  void validate() const;
};

struct MockCallCounts
{
  std::atomic<size_t> executions{0};
  std::atomic<size_t> stops{0};
};
class CountingMockAdapter : public mtc::motion::MockMotionAdapter
{
public:
  CountingMockAdapter(mtc::motion::MockMotionAdapterConfig options,
    mtc::core::ClockPtr clock, std::shared_ptr<MockCallCounts> counts)
  : MockMotionAdapter(std::move(options), std::move(clock)), counts_(std::move(counts)) {}
  mtc::core::MotionResult execute_joint_move(const mtc::core::MotionCommand & c) override
  {
    ++counts_->executions;
    return MockMotionAdapter::execute_joint_move(c);
  }
  mtc::core::StopResult request_stop_for_command(const std::string & reason,
    const std::optional<std::string> & id) override
  {
    ++counts_->stops;
    return MockMotionAdapter::request_stop_for_command(reason, id);
  }
private:
  std::shared_ptr<MockCallCounts> counts_;
};

class MockPerception : public mtc::manipulation::PerceptionPort
{
public:
  MockPerception(MockOptions options, mtc::core::ClockPtr clock, int color);
  std::optional<mtc::core::DetectionResult> get_detection(
    const std::string &, std::optional<int>, double) override;
private:
  MockOptions options_;
  mtc::core::ClockPtr clock_;
  int color_;
};
class MockPlanner : public mtc::manipulation::PlannerPort
{
public:
  explicit MockPlanner(MockOptions options) : options_(std::move(options)) {}
  std::optional<mtc::core::PlanResult> plan(
    const mtc::manipulation::PlanTarget &) override;
private:
  MockOptions options_;
};
class MockTool : public mtc::manipulation::ToolPort
{
public:
  explicit MockTool(MockOptions options) : options_(std::move(options)) {}
  std::optional<mtc::core::ToolEvidence> engage(const std::string &,
    const mtc::core::MotionPrimitiveList &) override;
  std::optional<mtc::core::ToolEvidence> test_lift(
    const mtc::core::MotionPrimitiveList &) override;
  mtc::core::ToolEvidence seat_verify() override;
  std::optional<mtc::core::ToolEvidence> unload(
    const mtc::core::MotionPrimitiveList &) override;
  std::optional<mtc::core::ToolEvidence> unlock(uint32_t) override;
  std::optional<mtc::core::ToolEvidence> disengage(
    const mtc::core::MotionPrimitiveList &) override;
  std::string holding_state() override
  {return unknown_load_ ? "unknown" : (holding_ ? "holding" : "none");}
private:
  mtc::core::ToolEvidence attachment() const;
  MockOptions options_;
  bool holding_ = false;
  bool unknown_load_ = false;
};

// Timer-owned polling bridge. At most one execute and one associated stop worker.
// Cached complete results are drained before a new command; no detached threads.
class AsyncMockMotion : public mtc::manipulation::MotionPort
{
public:
  AsyncMockMotion(MockOptions options, std::shared_ptr<mtc::core::ManualClock> clock,
    std::shared_ptr<MockCallCounts> counts = std::make_shared<MockCallCounts>());
  std::optional<mtc::core::MotionResult> execute_joint_move(
    const mtc::core::MotionCommand &) override;
  mtc::core::StopResult request_stop(const std::string &) override;
  bool poll_stop(const std::string & reason);
  bool execution_ready() const;
  const std::string & last_command_id() const {return last_id_;}
  mtc::motion::ExecutorSnapshot snapshot() const {return executor_->snapshot();}
  mtc::core::JointFeedback feedback() const {return adapter_->joint_feedback();}
  std::shared_ptr<mtc::motion::MockMotionAdapter> adapter() const {return adapter_;}
private:
  MockOptions options_;
  std::shared_ptr<mtc::motion::MockMotionAdapter> adapter_;
  std::shared_ptr<mtc::motion::MotionExecutorCore> executor_;
  std::string last_id_;
  std::future<mtc::core::MotionResult> execution_;
  std::optional<mtc::core::MotionResult> result_;
  std::future<mtc::core::StopResult> stopping_;
  std::optional<mtc::core::StopResult> stop_;
  std::chrono::steady_clock::time_point deliver_at_;
};
}  // namespace mtc::ros2_action
