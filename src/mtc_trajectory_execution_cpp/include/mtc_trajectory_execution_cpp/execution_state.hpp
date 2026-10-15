// Copyright 2026 MTC contributors
#pragma once
#include <string>
#include <mtc_core_cpp/results.hpp>
namespace mtc::trajectory
{
enum class GoalOwnership {NO_ACTIVE_GOAL, OWN_GOAL, FOREIGN_GOAL, UNKNOWN};
struct ExecutionSnapshot
{
  std::string command_id, goal_uuid, detail;
  GoalOwnership ownership = GoalOwnership::UNKNOWN;
  // Fresh means locally received within proof_max_age, not publisher-time proof.
  bool status_received = false, status_fresh = false;
  size_t status_messages = 0, unknown_active_goals = 0;
  // Legacy count: all active UUIDs not yet associated, including Unknown.
  bool ready = false, in_flight = false, fault_locked = false;
  bool cancel_acknowledged = false, terminal_received = false;
  size_t goals_sent = 0, cancel_requests = 0, feedback_samples = 0, stationary_samples = 0, foreign_active_goals = 0;
  double stationary_observed_seconds = 0.0;
  std::optional<mtc::core::MotionResult> result;
};
}  // namespace mtc::trajectory
