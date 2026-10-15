// Copyright 2026 meituan_challenge_ws contributors
//
// Task FSM（第一层）的公共类型：状态、事件、请求、快照、动作意图、配置。
//
// 迁移自 src/mtc_task/mtc_task/task_fsm.py 与 task_validation.py。
// 行为基线见 docs/migration/PYTHON_BEHAVIOR_BASELINE.md，映射见
// docs/migration/FSM_BEHAVIOR_MAPPING.md。
//
// 本文件**不包含任何 ROS 依赖**，可在无 ROS 图、无设备的环境下编译与测试。

#ifndef MTC_TASK_CPP__TASK_TYPES_HPP_
#define MTC_TASK_CPP__TASK_TYPES_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "mtc_core_cpp/error_codes.hpp"

namespace mtc
{
namespace task
{

/// §3.1 定义的 13 个 Task FSM 状态名。
enum class TaskStateName
{
  INIT = 0,
  SELF_CHECK,
  WAIT_TASK,
  VALIDATE_TASK,
  SCAN_SCENE,
  EXECUTE_ITEM,
  RECORD_RESULT,
  RECOVERY,
  CANCEL_PENDING,
  FINISHED,
  TASK_FAILED,
  FAULT,
  CANCELLED,
};

/// 状态名与 Python `TaskStateName.value` 逐字一致（用于 feedback 与日志对齐）。
const char * to_string(TaskStateName state);

/// 全部状态（顺序与枚举一致，供自检遍历）。
const std::vector<TaskStateName> & all_states();

/// 处于这些状态时视为"有进行中任务"（§7.2 单 Goal 并发拒绝判定依据）。
const std::vector<TaskStateName> & active_task_states();

/// Action execution has ended. FAULT additionally remains locked; ordinary RESET is forbidden.
const std::vector<TaskStateName> & terminal_states();

/// 事件名常量。
struct Event
{
  static constexpr const char * SELF_CHECK_OK = "self_check_ok";
  static constexpr const char * SELF_CHECK_FAIL = "self_check_fail";

  static constexpr const char * GOAL_ACCEPTED = "goal_accepted";
  static constexpr const char * GOAL_REJECTED = "goal_rejected";

  static constexpr const char * VALIDATED = "task_validated";
  static constexpr const char * VALIDATION_FAILED = "task_validation_failed";

  static constexpr const char * TARGET_FOUND = "target_found";
  static constexpr const char * TARGET_NOT_FOUND = "target_not_found";
  static constexpr const char * SCAN_FAILED = "scan_failed";

  static constexpr const char * ITEM_SUCCEEDED = "item_succeeded";
  static constexpr const char * ITEM_FAILED = "item_failed";
  static constexpr const char * ITEM_UNSAFE = "item_unsafe";

  static constexpr const char * RETRY_APPROVED = "retry_approved";
  static constexpr const char * RETRY_DENIED = "retry_denied";

  static constexpr const char * RECORD_OK = "record_ok";
  static constexpr const char * RECORD_FAILED = "record_failed";

  static constexpr const char * CANCEL_REQUESTED = "cancel_requested";
  static constexpr const char * CHILD_STOPPED_CONFIRMED = "child_stopped_confirmed";

  static constexpr const char * RESET = "reset";
  static constexpr const char * TIMEOUT = "timeout";
};

/// 任务模式（对齐 ExecuteTask.action 的 MODE_BASIC / MODE_SEQUENCE）。
struct TaskMode
{
  static constexpr uint8_t MODE_BASIC = 1;
  static constexpr uint8_t MODE_SEQUENCE = 2;

  static std::string name_of(int mode);
};

/// BatteryDetection.msg 合法颜色集合（COLOR_UNKNOWN=0 不是合法口令）。
const std::vector<int> & valid_colors();

/// 工位常量。
struct Slots
{
  static constexpr const char * T0 = "T0";
  static constexpr const char * P1 = "P1";
  static constexpr const char * P2 = "P2";
  static constexpr const char * P3 = "P3";

  static const std::vector<std::string> & all();
};

/// 已规范化的任务请求（对应 ExecuteTask.action 的 Goal 字段）。
struct TaskRequest
{
  std::string task_id;
  int mode = 0;
  std::vector<int> ordered_colors;
  uint32_t timeout_ms = 0;
  /// ExecuteTask.action 本身没有 target_slot 字段；调用方可选附加以便交叉校验。
  std::optional<std::string> target_slot;
  double placement_stability_sec = 3.0;

  std::string mode_name() const;
  size_t total_count() const {return ordered_colors.size();}
  /// 按设计约定推导第 index 项的目标工位；非法情形返回 nullopt。
  std::optional<std::string> slot_for_index(size_t index) const;
  std::vector<std::string> slots() const;
};

/// TaskState.msg 的内容（不含 Header）。
struct TaskStateSnapshot
{
  std::string task_id;
  std::string state;
  std::string substate;
  uint8_t completed_count = 0;
  uint8_t total_count = 0;
  uint16_t last_error_code = mtc::core::ErrorCodes::OK;
  std::string detail;
};

/// ExecuteTask.action 的 Result 内容。
struct TaskOutcome
{
  bool success = false;
  uint8_t completed_count = 0;
  std::vector<std::string> completed_slots;
  uint16_t error_code = mtc::core::ErrorCodes::OK;
  std::string message;
};

/// PickPlace.action Goal 的字段集合（作为动作意图的载荷透传）。
struct PickPlaceGoal
{
  std::string request_id;
  std::string object_id;
  int expected_color = 0;
  std::string target_slot;
  float placement_stability_sec = 3.0f;
};

/// FSM 产出的动作意图；由 ROS 外壳翻译成真实调用。
enum class ActionKind
{
  CALL_PICK_PLACE = 0,
  CANCEL_CHILD,
  REQUEST_STOP,
  PUBLISH_STATE,
  LOCK_ACTIONS,
};

const char * to_string(ActionKind kind);

/// 动作意图（含所有可能载荷；按 kind 取用对应字段）。
struct ActionRequest
{
  ActionKind kind = ActionKind::PUBLISH_STATE;
  PickPlaceGoal pick_place;          ///< CALL_PICK_PLACE
  std::string request_id;            ///< CANCEL_CHILD
  TaskStateSnapshot snapshot;        ///< PUBLISH_STATE
  std::string reason;                ///< REQUEST_STOP / LOCK_ACTIONS
};

/// 事件载荷：把 Python 的 `payload: Dict[str, Any]` 类型化为固定字段集合。
///
/// 说明：C++ 侧不再需要"无法解析的 goal 对象"，因为外壳负责把 ROS Goal
/// 转换为 TaskRequest。为保持行为基线中的拒绝语义，仍保留解析失败通道：
///   goal 为空 + parse_error 非空 -> "unparsable_goal"
///   goal 为空 + parse_error 为空 -> "missing_goal"
struct EventPayload
{
  std::optional<TaskRequest> goal;
  std::string goal_parse_error;

  std::string object_id;
  std::optional<int> expected_color;

  std::optional<int> error_code;
  std::string message;
  std::string source;

  bool success = false;
  bool placement_verified = false;
};

/// step() 返回值：新状态 + 需要调用的方执行的动作列表。
struct StepResult
{
  TaskStateName state = TaskStateName::INIT;
  std::vector<ActionRequest> actions;
  std::string reason;
  bool goal_rejected = false;

  std::string state_name() const {return to_string(state);}
  bool has(ActionKind kind) const;
  const ActionRequest * find(ActionKind kind) const;
  std::vector<ActionKind> action_kinds() const;
};

/// 可注入配置：每状态超时 + 恢复策略。
struct FsmsConfig
{
  /// `recovery_max_attempts` 指**额外重试**次数（不含首次尝试）。
  int recovery_max_attempts = 1;
  double default_timeout_s = 10.0;
  std::map<TaskStateName, std::optional<double>> state_timeouts;
  /// 允许自动重试的错误白名单（保守：未列入者一律不可重试）。
  std::vector<uint16_t> recoverable_errors;

  /// 与 Python DEFAULT_STATE_TIMEOUTS / DEFAULT_RECOVERABLE_ERRORS 等价的默认配置。
  static FsmsConfig defaults();

  std::optional<double> timeout_for(TaskStateName state) const;
  bool is_recoverable(int error_code) const;
};

/// 仅这些错误允许自动重试：均为"明确安全可重试"且不涉及在途运动不确定性。
const std::vector<uint16_t> & default_recoverable_errors();

/// 明确禁止自动重试的错误（保留常量供审计与外壳判断）。
const std::vector<uint16_t> & non_recoverable_errors();

/// 每状态默认超时（秒）。nullopt = 不设超时（等待态 / 终态 / 主动取消中）。
const std::map<TaskStateName, std::optional<double>> & default_state_timeouts();

}  // namespace task
}  // namespace mtc

#endif  // MTC_TASK_CPP__TASK_TYPES_HPP_
