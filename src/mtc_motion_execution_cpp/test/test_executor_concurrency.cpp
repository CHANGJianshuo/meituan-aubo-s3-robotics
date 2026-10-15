// Copyright 2026 meituan_challenge_ws contributors
// Offline races with bounded waits. No ROS graph, RPC, IO, or hardware.
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>
#include "mtc_motion_execution_cpp/executor_core.hpp"
#include "mtc_motion_execution_cpp/in_flight_guard.hpp"
#include "mtc_motion_execution_cpp/mock_motion_adapter.hpp"

namespace {
using namespace std::chrono_literals;
using mtc::core::ErrorCodes;
using mtc::core::ExecState;

mtc::core::MotionCommand command(const std::string & id)
{
  mtc::core::MotionCommand c;
  c.command_id = id;
  c.joint_names = mtc::motion::ValidationConfig().expected_joint_names;
  c.target_rad.assign(6, 0.05);
  return c;
}

class BlockingAdapter : public mtc::motion::MotionAdapter
{
public:
  std::shared_ptr<mtc::core::ManualClock> clock = std::make_shared<mtc::core::ManualClock>(100);
  std::atomic<int> executions{0}, stop_calls{0};
  bool report_timeout = false;
  bool stop_has_evidence = false;
  bool throw_on_stop = false;
  bool moving_feedback = false;
  bool wrong_identity = false;
  bool queue_busy = false;
  bool old_feedback = false;
  bool short_interval = false;
  bool disconnected = false;
  bool invalid_feedback = false;
  bool entered = false, released = false;
  std::string active;
  mutable std::mutex mutex;
  std::condition_variable cv;

  mtc::core::MotionCapabilities capabilities() const override
  {
    mtc::core::MotionCapabilities c;
    c.simulation_only = true;
    c.joint_goal_supported = c.cancel_supported = true;
    c.associated_cancel_supported = true;
    return c;
  }
  mtc::core::MotionResult execute_joint_move(const mtc::core::MotionCommand & c) override
  {
    ++executions;
    std::unique_lock<std::mutex> lock(mutex);
    active = c.command_id;
    entered = true;
    cv.notify_all();
    cv.wait_for(lock, 2s, [this] {return released;});
    mtc::core::MotionResult result;
    result.exec_state = report_timeout ? ExecState::TIMEOUT :
      (stop_calls ? ExecState::STOPPED : ExecState::SETTLED);
    result.timed_out = report_timeout;
    result.stop_confirmed = !report_timeout;
    result.success = result.arrived = !report_timeout && stop_calls == 0;
    return result;
  }
  bool wait_entered()
  {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, 1s, [this] {return entered;});
  }
  void release()
  {
    std::lock_guard<std::mutex> lock(mutex);
    released = true;
    cv.notify_all();
  }
  mtc::core::StopResult request_stop(const std::string &) override
  {
    ++stop_calls;
    if (throw_on_stop) {throw std::runtime_error("stop link failure");}
    mtc::core::StopResult result;
    result.request_delivered = true;
    result.stop_confirmed = true; // Boolean alone is deliberately insufficient.
    return result;
  }
  mtc::core::JointFeedback joint_feedback() const override {return {};}
  mtc::core::StopResult request_stop_for_command(const std::string & reason,
    const std::optional<std::string> & id) override
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (id && *id != active) {return {false, false, "ID mismatch", ErrorCodes::EXECUTION_REJECTED};}
    }
    return request_stop(reason);
  }
  std::optional<mtc::core::StopEvidence> stop_evidence() const override
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (active.empty() || (stop_calls && !stop_has_evidence)) {return std::nullopt;}
    mtc::core::StopEvidence e;
    e.command_id = wrong_identity ? "old-goal" : active;
    e.exec_state = moving_feedback ? ExecState::MOVING : (stop_calls ? ExecState::STOPPED : ExecState::SETTLED);
    e.communication_ok = !disconnected;
    e.queue_empty = !queue_busy;
    e.stationary_interval_verified = true;
    e.feedback.stamp_s = clock->now() - (old_feedback ? 1.0 : 0.0);
    e.stationary_since_s = e.feedback.stamp_s - (short_interval ? 0.01 : 0.2);
    e.feedback.position_rad.assign(6, 0.05);
    e.feedback.velocity_rad_s.assign(6, moving_feedback ? 0.1 : 0.0);
    if (invalid_feedback) {e.feedback.velocity_rad_s[0] = std::numeric_limits<double>::quiet_NaN();}
    return e;
  }
};

// The worker is always released and joined before the executor/adapter die.
struct RunningGoal
{
  std::shared_ptr<BlockingAdapter> adapter = std::make_shared<BlockingAdapter>();
  mtc::motion::MotionExecutorCore core{adapter, {}, adapter->clock};
  std::future<mtc::core::MotionResult> running;
  explicit RunningGoal(const std::string & id = "command_B")
  {
    adapter->stop_has_evidence = true;
    running = std::async(std::launch::async, [this, id] {return core.execute_joint_move(command(id));});
    EXPECT_TRUE(adapter->wait_entered());
  }
  ~RunningGoal()
  {
    adapter->release();
    if (running.valid()) {running.wait();}
  }
  mtc::core::MotionResult finish() {adapter->release(); return running.get();}
};
}

TEST(GoalCancelGate, StaleGoalCancelDoesNotCallBackend)
{
  RunningGoal h;
  const auto stop = h.core.request_stop("late callback", "command_A");
  EXPECT_FALSE(stop.request_delivered);
  EXPECT_FALSE(stop.stop_confirmed);
  EXPECT_EQ(stop.error_code, ErrorCodes::EXECUTION_REJECTED);
  EXPECT_NE(stop.message.find("command_id mismatch"), std::string::npos);
  EXPECT_EQ(h.adapter->stop_calls, 0);
  EXPECT_TRUE(h.finish().success);
}

TEST(GoalCancelGate, StaleGoalCancelPreservesCurrentOwner)
{
  RunningGoal h;
  h.core.request_stop("old cancel", "command_A");
  EXPECT_EQ(h.core.snapshot().active_command_id, "command_B");
  EXPECT_EQ(h.adapter->stop_calls, 0);
  EXPECT_TRUE(h.finish().success);
}

TEST(GoalCancelGate, StaleGoalCancelDoesNotModifyExecutionState)
{
  RunningGoal h;
  const auto before = h.core.snapshot();
  h.core.request_stop("old cancel", "command_A");
  const auto after = h.core.snapshot();
  EXPECT_EQ(after.exec_state, before.exec_state);
  EXPECT_EQ(after.active_command_id, before.active_command_id);
  EXPECT_EQ(after.motion_state_unknown, before.motion_state_unknown);
  EXPECT_EQ(after.stop_confirmed, before.stop_confirmed);
  EXPECT_EQ(after.last_error_code, before.last_error_code);
  EXPECT_EQ(h.adapter->stop_calls, 0);
  EXPECT_TRUE(h.finish().success);
}

TEST(GoalCancelGate, StaleGoalCancelCannotInterruptNewGoal)
{
  RunningGoal h;
  for (int i = 0; i < 20; ++i) {h.core.request_stop("old cancel", "command_A");}
  EXPECT_EQ(h.adapter->stop_calls, 0);
  EXPECT_TRUE(h.finish().success);
  // Also verifies that stale calls leave no pending stop or cancel flag behind.
  EXPECT_TRUE(h.core.execute_joint_move(command("command_C")).success);
  EXPECT_EQ(h.adapter->executions, 2);
}

TEST(GoalCancelGate, MatchingGoalCancelStillWorks)
{
  RunningGoal h;
  const auto stop = h.core.cancel_goal("command_B", "matching cancel");
  EXPECT_TRUE(stop.request_delivered);
  EXPECT_TRUE(stop.stop_confirmed);
  EXPECT_EQ(h.adapter->stop_calls, 1);
  const auto result = h.finish();
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_FALSE(h.core.motion_state_unknown());
}

TEST(GoalCancelGate, ConcurrentGoalReplacementAndLateCancel)
{
  auto adapter = std::make_shared<BlockingAdapter>();
  adapter->stop_has_evidence = true;
  mtc::motion::MotionExecutorCore core(adapter, {}, adapter->clock);
  for (int round = 0; round < 10; ++round) {
    const std::string old_id = "A-" + std::to_string(round);
    const std::string new_id = "B-" + std::to_string(round);
    adapter->release();
    EXPECT_TRUE(core.execute_joint_move(command(old_id)).success);
    {
      std::lock_guard<std::mutex> lock(adapter->mutex);
      adapter->entered = adapter->released = false;
    }
    auto late = std::async(std::launch::async, [&] {
      std::unique_lock<std::mutex> lock(adapter->mutex);
      const bool ready = adapter->cv.wait_for(lock, 1s, [&] {return adapter->active == new_id;});
      lock.unlock();
      EXPECT_TRUE(ready);
      return core.request_stop("old result callback", old_id);
    });
    auto running = std::async(std::launch::async, [&] {return core.execute_joint_move(command(new_id));});
    EXPECT_TRUE(adapter->wait_entered());
    EXPECT_FALSE(late.get().request_delivered);
    EXPECT_EQ(adapter->stop_calls, 0);
    EXPECT_EQ(core.snapshot().active_command_id, new_id);
    adapter->release();
    EXPECT_TRUE(running.get().success);
  }
}

TEST(ExecutorConcurrency, SixteenThreadsHaveOneOwner)
{
  mtc::motion::InFlightGuard guard;
  std::atomic<bool> go{false};
  std::atomic<int> owners{0};
  std::vector<std::thread> threads;
  for (int i = 0; i < 16; ++i) {
    threads.emplace_back([&, i] {
      while (!go.load()) {std::this_thread::yield();}
      if (guard.acquire("thread-" + std::to_string(i)).first) {++owners;}
      (void)guard.in_flight();
      guard.release("not-the-owner");
    });
  }
  go = true;
  for (auto & t : threads) {t.join();}
  EXPECT_EQ(owners, 1);
  ASSERT_TRUE(guard.in_flight());
  guard.release(*guard.in_flight());
  EXPECT_TRUE(guard.is_free());
}

TEST(ExecutorConcurrency, GoalsAndStopRespondWhileExecutionWaits)
{
  auto adapter = std::make_shared<BlockingAdapter>();
  mtc::motion::MotionExecutorCore core(adapter, {}, adapter->clock);
  auto running = std::async(std::launch::async, [&] {return core.execute_joint_move(command("owner"));});
  const bool entered = adapter->wait_entered();
  EXPECT_TRUE(entered);
  std::vector<std::future<mtc::core::MotionResult>> goals;
  for (int i = 0; i < 8; ++i) {
    goals.push_back(std::async(std::launch::async, [&, i] {
      return core.execute_joint_move(command("other-" + std::to_string(i)));
    }));
  }
  for (auto & goal : goals) {EXPECT_EQ(goal.get().exec_state, ExecState::REJECTED);}
  EXPECT_EQ(core.snapshot().active_command_id, "owner");
  auto stopping = std::async(std::launch::async, [&] {return core.request_stop("cancel", "owner");});
  EXPECT_EQ(stopping.wait_for(200ms), std::future_status::ready);
  EXPECT_FALSE(stopping.get().stop_confirmed);
  EXPECT_THROW(core.clear_unknown_after_manual_confirmation("operator"), std::logic_error);
  adapter->release();
  EXPECT_TRUE(running.get().status_unknown);
  EXPECT_EQ(adapter->executions, 1);
  EXPECT_TRUE(core.motion_state_unknown());
  EXPECT_EQ(core.execute_joint_move(command("new")).error_code, ErrorCodes::MOTION_STATUS_UNKNOWN);
}

TEST(ExecutorConcurrency, DuplicateStopsAndLateResultCannotUnlock)
{
  auto adapter = std::make_shared<BlockingAdapter>();
  adapter->report_timeout = true;
  mtc::motion::MotionExecutorCore core(adapter, {}, adapter->clock);
  auto running = std::async(std::launch::async, [&] {return core.execute_joint_move(command("owner"));});
  EXPECT_TRUE(adapter->wait_entered());
  std::vector<std::future<mtc::core::StopResult>> stops;
  for (int i = 0; i < 8; ++i) {
    stops.push_back(std::async(std::launch::async, [&] {return core.request_stop("cancel", "owner");}));
  }
  for (auto & stop : stops) {EXPECT_FALSE(stop.get().stop_confirmed);}
  adapter->release();
  const auto result = running.get();
  EXPECT_TRUE(result.timed_out);
  EXPECT_TRUE(result.status_unknown);
  EXPECT_TRUE(core.motion_state_unknown());
  EXPECT_EQ(adapter->executions, 1);
}

TEST(ExecutorConcurrency, ConfirmedStopRetainsOwnerUntilResultDrains)
{
  auto adapter = std::make_shared<BlockingAdapter>();
  adapter->stop_has_evidence = true;
  mtc::motion::MotionExecutorCore core(adapter, {}, adapter->clock);
  auto running = std::async(std::launch::async, [&] {return core.execute_joint_move(command("owner"));});
  EXPECT_TRUE(adapter->wait_entered());
  EXPECT_TRUE(core.request_stop("cancel", "owner").stop_confirmed);
  EXPECT_EQ(core.execute_joint_move(command("next")).exec_state, ExecState::REJECTED);
  adapter->release();
  const auto result = running.get();
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_FALSE(core.motion_state_unknown());
}

TEST(ExecutorConcurrency, InvalidEvidenceCannotConfirmStop)
{
  for (int fault = 0; fault < 8; ++fault) {
    auto adapter = std::make_shared<BlockingAdapter>();
    adapter->stop_has_evidence = true;
    adapter->moving_feedback = fault == 0;
    adapter->wrong_identity = fault == 1;
    adapter->queue_busy = fault == 2;
    adapter->old_feedback = fault == 3;
    adapter->short_interval = fault == 4;
    adapter->disconnected = fault == 5;
    adapter->invalid_feedback = fault == 6;
    adapter->throw_on_stop = fault == 7;
    mtc::motion::MotionExecutorCore core(adapter, {}, adapter->clock);
    auto running = std::async(std::launch::async, [&] {return core.execute_joint_move(command("owner"));});
    EXPECT_TRUE(adapter->wait_entered());
    const auto stop = core.request_stop("cancel", "owner");
    EXPECT_FALSE(stop.stop_confirmed) << fault;
    adapter->release();
    EXPECT_TRUE(running.get().status_unknown) << fault;
    EXPECT_TRUE(core.motion_state_unknown());
  }
}

TEST(ExecutorConcurrency, OldGoalCancelCannotReplaceNewOwner)
{
  auto adapter = std::make_shared<BlockingAdapter>();
  adapter->stop_has_evidence = true;
  mtc::motion::MotionExecutorCore core(adapter, {}, adapter->clock);
  auto running = std::async(std::launch::async, [&] {return core.execute_joint_move(command("current"));});
  EXPECT_TRUE(adapter->wait_entered());
  EXPECT_FALSE(core.request_stop("late cancel", "old-goal").stop_confirmed);
  EXPECT_EQ(core.snapshot().active_command_id, "current");
  EXPECT_EQ(adapter->stop_calls, 0);
  adapter->release();
  EXPECT_TRUE(running.get().success);
}

TEST(GoalCancelGate, SupervisoryStopStillStopsCurrentGoal)
{
  RunningGoal h;
  EXPECT_TRUE(h.core.supervisory_stop("operator stop").stop_confirmed);
  EXPECT_EQ(h.adapter->stop_calls, 1);
  EXPECT_TRUE(h.finish().stop_confirmed);
}

TEST(GoalCancelGate, EmptyOrUnknownGoalIdHasNoSideEffect)
{
  RunningGoal h;
  EXPECT_EQ(h.core.cancel_goal("", "invalid").error_code, ErrorCodes::EXECUTION_REJECTED);
  EXPECT_EQ(h.core.cancel_goal("not-a-goal", "invalid").error_code, ErrorCodes::EXECUTION_REJECTED);
  EXPECT_EQ(h.adapter->stop_calls, 0);
  EXPECT_TRUE(h.finish().success);
}

TEST(GoalCancelGate, CompletedGoalCancelCannotStopLaterCompletedGoal)
{
  RunningGoal h("A");
  EXPECT_TRUE(h.finish().success);
  EXPECT_TRUE(h.core.execute_joint_move(command("B")).success);
  const auto before = h.core.snapshot();
  EXPECT_FALSE(h.core.cancel_goal("A", "very late").request_delivered);
  EXPECT_EQ(h.adapter->stop_calls, 0);
  const auto after = h.core.snapshot();
  EXPECT_EQ(after.exec_state, before.exec_state);
  EXPECT_EQ(after.stop_confirmed, before.stop_confirmed);
  EXPECT_EQ(after.motion_state_unknown, before.motion_state_unknown);
  // Matching latest completed identity retains the same evidence requirement.
  EXPECT_TRUE(h.core.cancel_goal("B", "after completion").stop_confirmed);
  EXPECT_EQ(h.adapter->stop_calls, 1);
}

TEST(GoalCancelGate, CancelRacingBackendDispatchRemainsSticky)
{
  class DispatchBarrier : public mtc::motion::MockMotionAdapter
  {
  public:
    explicit DispatchBarrier(mtc::core::ClockPtr clock) : MockMotionAdapter({}, clock) {}
    std::mutex gate_mutex;
    std::condition_variable gate_cv;
    bool entered = false, released = false;
    std::atomic<int> stop_calls{0};
    mtc::core::MotionResult execute_joint_move(const mtc::core::MotionCommand & c) override
    {
      std::unique_lock<std::mutex> lock(gate_mutex);
      entered = true; gate_cv.notify_all();
      gate_cv.wait_for(lock, 2s, [&] {return released;});
      lock.unlock();
      return MockMotionAdapter::execute_joint_move(c);
    }
    mtc::core::StopResult request_stop_for_command(const std::string & reason,
      const std::optional<std::string> & id) override
    {
      ++stop_calls;
      return MockMotionAdapter::request_stop_for_command(reason, id);
    }
  };
  auto clock = std::make_shared<mtc::core::ManualClock>();
  auto adapter = std::make_shared<DispatchBarrier>(clock);
  mtc::motion::MotionExecutorCore core(adapter, {}, clock);
  auto running = std::async(std::launch::async, [&] {return core.execute_joint_move(command("B"));});
  {
    std::unique_lock<std::mutex> lock(adapter->gate_mutex);
    EXPECT_TRUE(adapter->gate_cv.wait_for(lock, 1s, [&] {return adapter->entered;}));
  }
  EXPECT_FALSE(core.cancel_goal("B", "before Mock dispatch").stop_confirmed);
  EXPECT_EQ(adapter->stop_calls, 1);
  {
    std::lock_guard<std::mutex> lock(adapter->gate_mutex);
    adapter->released = true; adapter->gate_cv.notify_all();
  }
  EXPECT_FALSE(running.get().success);
  EXPECT_DOUBLE_EQ(clock->now(), 0.0);
  EXPECT_EQ(adapter->joint_feedback().position_rad, std::vector<double>(6, 0.0));
}
