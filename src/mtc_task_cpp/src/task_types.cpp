// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_task_cpp/task_types.hpp"

#include <algorithm>

#include "mtc_task_cpp/task_validation.hpp"

namespace mtc
{
namespace task
{

// ---------------------------------------------------------------------------
// 状态
// ---------------------------------------------------------------------------

const char * to_string(TaskStateName state)
{
  switch (state) {
    case TaskStateName::INIT: return "INIT";
    case TaskStateName::SELF_CHECK: return "SELF_CHECK";
    case TaskStateName::WAIT_TASK: return "WAIT_TASK";
    case TaskStateName::VALIDATE_TASK: return "VALIDATE_TASK";
    case TaskStateName::SCAN_SCENE: return "SCAN_SCENE";
    case TaskStateName::EXECUTE_ITEM: return "EXECUTE_ITEM";
    case TaskStateName::RECORD_RESULT: return "RECORD_RESULT";
    case TaskStateName::RECOVERY: return "RECOVERY";
    case TaskStateName::CANCEL_PENDING: return "CANCEL_PENDING";
    case TaskStateName::FINISHED: return "FINISHED";
    case TaskStateName::TASK_FAILED: return "TASK_FAILED";
    case TaskStateName::FAULT: return "FAULT";
    case TaskStateName::CANCELLED: return "CANCELLED";
  }
  return "UNKNOWN";
}

const std::vector<TaskStateName> & all_states()
{
  static const std::vector<TaskStateName> kStates = {
    TaskStateName::INIT,
    TaskStateName::SELF_CHECK,
    TaskStateName::WAIT_TASK,
    TaskStateName::VALIDATE_TASK,
    TaskStateName::SCAN_SCENE,
    TaskStateName::EXECUTE_ITEM,
    TaskStateName::RECORD_RESULT,
    TaskStateName::RECOVERY,
    TaskStateName::CANCEL_PENDING,
    TaskStateName::FINISHED,
    TaskStateName::TASK_FAILED,
    TaskStateName::FAULT,
    TaskStateName::CANCELLED,
  };
  return kStates;
}

const std::vector<TaskStateName> & active_task_states()
{
  static const std::vector<TaskStateName> kStates = {
    TaskStateName::VALIDATE_TASK,
    TaskStateName::SCAN_SCENE,
    TaskStateName::EXECUTE_ITEM,
    TaskStateName::RECORD_RESULT,
    TaskStateName::RECOVERY,
    TaskStateName::CANCEL_PENDING,
  };
  return kStates;
}

const std::vector<TaskStateName> & terminal_states()
{
  static const std::vector<TaskStateName> kStates = {
    TaskStateName::FINISHED,
    TaskStateName::TASK_FAILED,
    TaskStateName::FAULT,
    TaskStateName::CANCELLED,
  };
  return kStates;
}

// ---------------------------------------------------------------------------
// 模式、颜色、工位
// ---------------------------------------------------------------------------

std::string TaskMode::name_of(int mode)
{
  switch (mode) {
    case MODE_BASIC: return "MODE_BASIC";
    case MODE_SEQUENCE: return "MODE_SEQUENCE";
    default: break;
  }
  return "UNKNOWN_MODE_" + std::to_string(mode);
}

const std::vector<int> & valid_colors()
{
  static const std::vector<int> kColors = {1, 2, 3, 4};
  return kColors;
}

const std::vector<std::string> & Slots::all()
{
  static const std::vector<std::string> kSlots = {T0, P1, P2, P3};
  return kSlots;
}

// ---------------------------------------------------------------------------
// TaskRequest
// ---------------------------------------------------------------------------

std::string TaskRequest::mode_name() const
{
  return TaskMode::name_of(mode);
}

std::optional<std::string> TaskRequest::slot_for_index(size_t index) const
{
  return expected_slot_for_index(mode, index, target_slot);
}

std::vector<std::string> TaskRequest::slots() const
{
  std::vector<std::string> result;
  result.reserve(total_count());
  for (size_t i = 0; i < total_count(); ++i) {
    result.push_back(slot_for_index(i).value_or(""));
  }
  return result;
}

// ---------------------------------------------------------------------------
// 动作意图
// ---------------------------------------------------------------------------

const char * to_string(ActionKind kind)
{
  switch (kind) {
    case ActionKind::CALL_PICK_PLACE: return "call_pick_place";
    case ActionKind::CANCEL_CHILD: return "cancel_child";
    case ActionKind::REQUEST_STOP: return "request_stop";
    case ActionKind::PUBLISH_STATE: return "publish_state";
    case ActionKind::LOCK_ACTIONS: return "lock_actions";
  }
  return "unknown";
}

const ActionRequest * StepResult::find(ActionKind kind) const
{
  for (const auto & action : actions) {
    if (action.kind == kind) {
      return &action;
    }
  }
  return nullptr;
}

bool StepResult::has(ActionKind kind) const
{
  return find(kind) != nullptr;
}

std::vector<ActionKind> StepResult::action_kinds() const
{
  std::vector<ActionKind> kinds;
  kinds.reserve(actions.size());
  for (const auto & action : actions) {
    kinds.push_back(action.kind);
  }
  return kinds;
}

// ---------------------------------------------------------------------------
// 默认配置
// ---------------------------------------------------------------------------

const std::map<TaskStateName, std::optional<double>> & default_state_timeouts()
{
  static const std::map<TaskStateName, std::optional<double>> kTimeouts = {
    {TaskStateName::INIT, 5.0},
    {TaskStateName::SELF_CHECK, 10.0},
    {TaskStateName::WAIT_TASK, std::nullopt},
    {TaskStateName::VALIDATE_TASK, 2.0},
    {TaskStateName::SCAN_SCENE, 5.0},
    {TaskStateName::EXECUTE_ITEM, 60.0},
    {TaskStateName::RECORD_RESULT, 2.0},
    {TaskStateName::RECOVERY, 10.0},
    {TaskStateName::CANCEL_PENDING, 5.0},
    {TaskStateName::FINISHED, std::nullopt},
    {TaskStateName::TASK_FAILED, std::nullopt},
    {TaskStateName::FAULT, std::nullopt},
    {TaskStateName::CANCELLED, std::nullopt},
  };
  return kTimeouts;
}

const std::vector<uint16_t> & default_recoverable_errors()
{
  // 仅"明确安全可重试"且不涉及在途运动不确定性者：
  //   110/120/200 纯感知与纯规划失败；
  //   430 PLACEMENT_FAILED 表示未落到判定区或稳定性不足，电池已由台面承托、
  //       工具已分离（否则会报 400/410/420），因此允许限次重试；
  //   210 EXECUTION_REJECTED 表示目标不合法/后端不支持，未产生运动，可重新规划一次。
  static const std::vector<uint16_t> kErrors = {
    mtc::core::ErrorCodes::TARGET_NOT_FOUND,
    mtc::core::ErrorCodes::POSE_STALE,
    mtc::core::ErrorCodes::PLANNING_FAILED,
    mtc::core::ErrorCodes::PLACEMENT_FAILED,
    mtc::core::ErrorCodes::EXECUTION_REJECTED,
  };
  return kErrors;
}

const std::vector<uint16_t> & non_recoverable_errors()
{
  static const std::vector<uint16_t> kErrors = {
    mtc::core::ErrorCodes::MOTION_STATUS_UNKNOWN,
    mtc::core::ErrorCodes::MOTION_TIMEOUT,
    mtc::core::ErrorCodes::SAFETY_INTERLOCK,
    mtc::core::ErrorCodes::HARDWARE_FAULT,
    mtc::core::ErrorCodes::UNLOCK_FAILED,
    mtc::core::ErrorCodes::RELEASE_NOT_VERIFIED,
    mtc::core::ErrorCodes::OBJECT_DROPPED,
    mtc::core::ErrorCodes::CANCEL_NOT_CONFIRMED,
  };
  return kErrors;
}

FsmsConfig FsmsConfig::defaults()
{
  FsmsConfig config;
  config.recovery_max_attempts = 1;
  config.default_timeout_s = 10.0;
  config.state_timeouts = default_state_timeouts();
  config.recoverable_errors = default_recoverable_errors();
  return config;
}

std::optional<double> FsmsConfig::timeout_for(TaskStateName state) const
{
  const auto it = state_timeouts.find(state);
  if (it != state_timeouts.end()) {
    return it->second;
  }
  return std::optional<double>(default_timeout_s);
}

bool FsmsConfig::is_recoverable(int error_code) const
{
  for (const auto code : recoverable_errors) {
    if (static_cast<int>(code) == error_code) {
      return true;
    }
  }
  return false;
}

}  // namespace task
}  // namespace mtc
