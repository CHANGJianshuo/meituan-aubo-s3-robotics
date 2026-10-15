// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_task_cpp/task_validation.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace mtc
{
namespace task
{

namespace
{

ValidationResult fail(const std::string & reason, const std::string & message)
{
  ValidationResult result;
  result.ok = false;
  result.error_code = mtc::core::ErrorCodes::INVALID_TASK;
  result.reason = reason;
  result.message = message;
  return result;
}

}  // namespace

std::optional<std::string> expected_slot_for_index(
  int mode, size_t index, const std::optional<std::string> & explicit_slot)
{
  (void)explicit_slot;
  if (mode == TaskMode::MODE_BASIC) {
    return std::string(Slots::T0);
  }
  if (mode == TaskMode::MODE_SEQUENCE) {
    static const std::vector<std::string> kSequenceSlots = {
      Slots::P1, Slots::P2, Slots::P3,
    };
    if (index < kSequenceSlots.size()) {
      return kSequenceSlots[index];
    }
    return std::nullopt;
  }
  return std::nullopt;
}

ValidationResult validate_task_request(const TaskRequest & task)
{
  const std::string task_id = task.task_id;
  if (task_id.find_first_not_of(" \t\n\r\f\v") == std::string::npos) {
    return fail("empty_task_id", "task_id must not be empty");
  }

  const int mode = task.mode;
  if (mode != TaskMode::MODE_BASIC && mode != TaskMode::MODE_SEQUENCE) {
    return fail(
      "unknown_mode",
      "unsupported mode=" + std::to_string(mode) +
      "; expected MODE_BASIC(1) or MODE_SEQUENCE(2)");
  }

  const auto & colors = task.ordered_colors;
  if (colors.empty()) {
    return fail("empty_colors", "ordered_colors must not be empty");
  }
  if (mode == TaskMode::MODE_BASIC && colors.size() != 1u) {
    return fail(
      "basic_count",
      "MODE_BASIC requires exactly 1 color, got " + std::to_string(colors.size()));
  }
  if (mode == TaskMode::MODE_SEQUENCE && colors.size() != 3u) {
    return fail(
      "sequence_count",
      "MODE_SEQUENCE requires exactly 3 colors, got " + std::to_string(colors.size()));
  }

  const auto & allowed_colors = valid_colors();
  for (const int color : colors) {
    if (std::find(allowed_colors.begin(), allowed_colors.end(), color) == allowed_colors.end()) {
      return fail(
        "illegal_color",
        "illegal color " + std::to_string(color) + "; allowed [1, 2, 3, 4]");
    }
  }

  {
    const std::set<int> unique_colors(colors.begin(), colors.end());
    if (unique_colors.size() != colors.size()) {
      std::string joined;
      for (size_t i = 0; i < colors.size(); ++i) {
        if (i != 0u) {
          joined += ", ";
        }
        joined += std::to_string(colors[i]);
      }
      return fail("duplicate_color", "ordered_colors must be mutually distinct, got [" + joined + "]");
    }
  }

  if (task.timeout_ms == 0u) {
    return fail("timeout_not_positive", "timeout_ms must be > 0, got 0");
  }

  if (!std::isfinite(task.placement_stability_sec) || task.placement_stability_sec <= 0.0) {
    return fail("stability_not_positive", "placement_stability_sec must be finite and > 0");
  }

  std::vector<std::string> derived;
  derived.reserve(colors.size());
  for (size_t i = 0; i < colors.size(); ++i) {
    const auto slot = expected_slot_for_index(mode, i);
    if (!slot.has_value()) {
      return fail(
        "slot_mapping",
        "cannot derive target slots for mode=" + std::to_string(mode) +
        " count=" + std::to_string(colors.size()));
    }
    derived.push_back(*slot);
  }

  if (task.target_slot.has_value() && !task.target_slot->empty()) {
    const std::string & explicit_slot = *task.target_slot;
    const auto & allowed_slots = Slots::all();
    if (std::find(allowed_slots.begin(), allowed_slots.end(), explicit_slot) == allowed_slots.end()) {
      return fail(
        "illegal_target_slot",
        "target_slot '" + explicit_slot + "' not allowed; allowed [T0, P1, P2, P3]");
    }
    if (explicit_slot != derived.front()) {
      return fail(
        "target_slot_mismatch",
        "target_slot '" + explicit_slot + "' conflicts with mode=" + TaskMode::name_of(mode) +
        " derived slot '" + derived.front() + "'");
    }
  }

  ValidationResult result;
  result.ok = true;
  result.error_code = mtc::core::ErrorCodes::OK;
  result.reason = "ok";
  result.message = "task accepted";
  result.normalized_slots = derived;
  return result;
}

ValidationResult validate_slot_name(const std::string & slot)
{
  const auto & allowed_slots = Slots::all();
  if (std::find(allowed_slots.begin(), allowed_slots.end(), slot) == allowed_slots.end()) {
    return fail(
      "illegal_target_slot",
      "target_slot '" + slot + "' not allowed; allowed [T0, P1, P2, P3]");
  }
  ValidationResult result;
  result.ok = true;
  result.error_code = mtc::core::ErrorCodes::OK;
  result.reason = "ok";
  result.message = "slot accepted";
  result.normalized_slots = {slot};
  return result;
}

}  // namespace task
}  // namespace mtc
