// Copyright 2026 meituan_challenge_ws contributors
//
// Task FSM 实现。逐条对齐 Python src/mtc_task/mtc_task/task_fsm.py 的：
//   TRANSITIONS / ENTRY_ACTIONS / 守卫 / 进入处理 / 超时映射 / 并发拒绝。

#include "mtc_task_cpp/task_fsm.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace mtc
{
namespace task
{

namespace
{

/// 构造一条转移（等价于 Python 的 _t 辅助函数）。
TaskFsm::Transition make_transition(
  TaskStateName state,
  const char * event,
  TaskStateName next_state,
  bool (TaskFsm::*guard)(const EventPayload &) = nullptr,
  std::optional<TaskStateName> fallback = std::nullopt,
  std::optional<TaskStateName> else_state = std::nullopt,
  const char * note = "")
{
  TaskFsm::Transition transition;
  transition.state = state;
  transition.event = event;
  transition.next_state = next_state;
  transition.guard = guard;
  transition.fallback = fallback;
  transition.else_state = else_state;
  transition.note = note;
  return transition;
}

bool contains_state(const std::vector<TaskStateName> & states, TaskStateName state)
{
  return std::find(states.begin(), states.end(), state) != states.end();
}

}  // namespace

// ---------------------------------------------------------------------------
// 转移表
// ---------------------------------------------------------------------------

const std::vector<TaskFsm::Transition> & TaskFsm::transitions()
{
  static const std::vector<Transition> kTransitions = {
    // --- 启动与自检 -------------------------------------------------------
    make_transition(
      TaskStateName::INIT, Event::SELF_CHECK_OK, TaskStateName::SELF_CHECK,
      nullptr, std::nullopt, std::nullopt, "参数/依赖就绪"),
    make_transition(
      TaskStateName::INIT, Event::SELF_CHECK_FAIL, TaskStateName::FAULT,
      nullptr, std::nullopt, std::nullopt, "初始化失败"),
    make_transition(
      TaskStateName::SELF_CHECK, Event::SELF_CHECK_OK, TaskStateName::WAIT_TASK,
      nullptr, std::nullopt, std::nullopt, "设备正常/通信就绪"),
    make_transition(
      TaskStateName::SELF_CHECK, Event::SELF_CHECK_FAIL, TaskStateName::FAULT,
      nullptr, std::nullopt, std::nullopt, "自检失败"),

    // --- 任务受理（§7.2：同一时刻只允许一个进行中 Goal）------------------
    make_transition(
      TaskStateName::WAIT_TASK, Event::GOAL_ACCEPTED, TaskStateName::VALIDATE_TASK,
      &TaskFsm::guard_goal_admissible),
    // 以下显式登记"任务进行中再次收到 Goal"：必须明确拒绝，且不得排队。
    // 若缺少这些条目，step() 会把它当作"无转移事件"静默忽略，等于没有拒绝。
    make_transition(
      TaskStateName::VALIDATE_TASK, Event::GOAL_ACCEPTED, TaskStateName::VALIDATE_TASK,
      &TaskFsm::guard_goal_denied),
    make_transition(
      TaskStateName::SCAN_SCENE, Event::GOAL_ACCEPTED, TaskStateName::SCAN_SCENE,
      &TaskFsm::guard_goal_denied),
    make_transition(
      TaskStateName::EXECUTE_ITEM, Event::GOAL_ACCEPTED, TaskStateName::EXECUTE_ITEM,
      &TaskFsm::guard_goal_denied),
    make_transition(
      TaskStateName::RECORD_RESULT, Event::GOAL_ACCEPTED, TaskStateName::RECORD_RESULT,
      &TaskFsm::guard_goal_denied),
    make_transition(
      TaskStateName::RECOVERY, Event::GOAL_ACCEPTED, TaskStateName::RECOVERY,
      &TaskFsm::guard_goal_denied),
    make_transition(
      TaskStateName::CANCEL_PENDING, Event::GOAL_ACCEPTED, TaskStateName::CANCEL_PENDING,
      &TaskFsm::guard_goal_denied),

    // --- 任务校验（§4.3）--------------------------------------------------
    make_transition(TaskStateName::VALIDATE_TASK, Event::VALIDATED, TaskStateName::SCAN_SCENE),
    make_transition(
      TaskStateName::VALIDATE_TASK, Event::VALIDATION_FAILED, TaskStateName::TASK_FAILED,
      nullptr, std::nullopt, std::nullopt, "非法任务不得猜测补全"),
    make_transition(
      TaskStateName::VALIDATE_TASK, Event::GOAL_REJECTED, TaskStateName::TASK_FAILED,
      nullptr, std::nullopt, std::nullopt, "受理后被撤回"),

    // --- 场景扫描（§3.1）--------------------------------------------------
    make_transition(
      TaskStateName::SCAN_SCENE, Event::TARGET_FOUND, TaskStateName::EXECUTE_ITEM,
      &TaskFsm::guard_target_usable),
    make_transition(
      TaskStateName::SCAN_SCENE, Event::TARGET_NOT_FOUND, TaskStateName::RECOVERY,
      nullptr, std::nullopt, std::nullopt, "重新感知；限次"),
    make_transition(
      TaskStateName::SCAN_SCENE, Event::SCAN_FAILED, TaskStateName::TASK_FAILED,
      nullptr, std::nullopt, std::nullopt, "重试预算已耗尽"),
    make_transition(
      TaskStateName::SCAN_SCENE, Event::CANCEL_REQUESTED, TaskStateName::CANCEL_PENDING),

    // --- 单项执行（§3.1 / §7.2）------------------------------------------
    make_transition(
      TaskStateName::EXECUTE_ITEM, Event::ITEM_SUCCEEDED, TaskStateName::RECORD_RESULT),
    make_transition(
      TaskStateName::EXECUTE_ITEM, Event::ITEM_FAILED, TaskStateName::RECOVERY,
      &TaskFsm::guard_error_recoverable, TaskStateName::FAULT, std::nullopt,
      "不可重试（如 230）-> FAULT"),
    make_transition(
      TaskStateName::EXECUTE_ITEM, Event::ITEM_UNSAFE, TaskStateName::FAULT,
      nullptr, std::nullopt, std::nullopt, "危险/不确定状态"),
    make_transition(
      TaskStateName::EXECUTE_ITEM, Event::CANCEL_REQUESTED, TaskStateName::CANCEL_PENDING),

    // --- 记录结果（不依赖发令返回码）--------------------------------------
    // 单条转移表达二选一：还有项 -> SCAN_SCENE；全部完成 -> FINISHED。
    make_transition(
      TaskStateName::RECORD_RESULT, Event::RECORD_OK, TaskStateName::FINISHED,
      &TaskFsm::guard_all_items_done, std::nullopt, TaskStateName::SCAN_SCENE,
      "all_items_done ? FINISHED : SCAN_SCENE"),
    make_transition(
      TaskStateName::RECORD_RESULT, Event::RECORD_FAILED, TaskStateName::TASK_FAILED),
    make_transition(
      TaskStateName::RECORD_RESULT, Event::CANCEL_REQUESTED, TaskStateName::CANCEL_PENDING),

    // --- 恢复（仅安全可重试错误）------------------------------------------
    // 预算不足时经 fallback 直接 TASK_FAILED，避免"守卫拒绝->原地不动"导致重试死循环。
    make_transition(
      TaskStateName::RECOVERY, Event::RETRY_APPROVED, TaskStateName::SCAN_SCENE,
      &TaskFsm::guard_retry_budget_available, TaskStateName::TASK_FAILED, std::nullopt,
      "预算耗尽 -> TASK_FAILED"),
    make_transition(
      TaskStateName::RECOVERY, Event::RETRY_DENIED, TaskStateName::TASK_FAILED,
      nullptr, std::nullopt, std::nullopt, "重试耗尽/不可重试"),
    make_transition(
      TaskStateName::RECOVERY, Event::CANCEL_REQUESTED, TaskStateName::CANCEL_PENDING),

    // --- 取消（§4.3：不可无等待地返回 CANCELED）--------------------------
    make_transition(
      TaskStateName::CANCEL_PENDING, Event::CHILD_STOPPED_CONFIRMED, TaskStateName::CANCELLED,
      nullptr, std::nullopt, std::nullopt, "停稳+负载已确认"),
    make_transition(
      TaskStateName::CANCEL_PENDING, Event::TIMEOUT, TaskStateName::FAULT,
      &TaskFsm::guard_cancel_timeout, std::nullopt, std::nullopt, "520"),
    make_transition(
      TaskStateName::CANCEL_PENDING, Event::ITEM_UNSAFE, TaskStateName::FAULT),

    // --- 终态复位 ---------------------------------------------------------
    make_transition(TaskStateName::FINISHED, Event::RESET, TaskStateName::WAIT_TASK),
    make_transition(
      TaskStateName::TASK_FAILED, Event::RESET, TaskStateName::WAIT_TASK,
      nullptr, std::nullopt, std::nullopt, "复位并确认可运行"),
    make_transition(
      TaskStateName::CANCELLED, Event::RESET, TaskStateName::WAIT_TASK,
      nullptr, std::nullopt, std::nullopt, "复位并确认可运行"),

    // --- 每状态超时去向（守卫仅在真的超时后放行）--------------------------
    make_transition(
      TaskStateName::INIT, Event::TIMEOUT, TaskStateName::FAULT, &TaskFsm::guard_state_timed_out),
    make_transition(
      TaskStateName::SELF_CHECK, Event::TIMEOUT, TaskStateName::FAULT,
      &TaskFsm::guard_state_timed_out),
    make_transition(
      TaskStateName::VALIDATE_TASK, Event::TIMEOUT, TaskStateName::TASK_FAILED,
      &TaskFsm::guard_state_timed_out),
    make_transition(
      TaskStateName::SCAN_SCENE, Event::TIMEOUT, TaskStateName::RECOVERY,
      &TaskFsm::guard_state_timed_out, std::nullopt, std::nullopt, "扫描超时->限次"),
    make_transition(
      TaskStateName::EXECUTE_ITEM, Event::TIMEOUT, TaskStateName::FAULT,
      &TaskFsm::guard_state_timed_out, std::nullopt, std::nullopt,
      "220：运动状态不确定，不自动重发"),
    make_transition(
      TaskStateName::RECORD_RESULT, Event::TIMEOUT, TaskStateName::TASK_FAILED,
      &TaskFsm::guard_state_timed_out),
    make_transition(
      TaskStateName::RECOVERY, Event::TIMEOUT, TaskStateName::TASK_FAILED,
      &TaskFsm::guard_state_timed_out),
  };
  return kTransitions;
}

const std::map<TaskStateName, TaskFsm::EntryHandler> & TaskFsm::entry_actions()
{
  static const std::map<TaskStateName, EntryHandler> kEntryActions = {
    {TaskStateName::INIT, nullptr},
    {TaskStateName::SELF_CHECK, &TaskFsm::enter_self_check},
    {TaskStateName::WAIT_TASK, &TaskFsm::enter_wait_task},
    {TaskStateName::VALIDATE_TASK, &TaskFsm::enter_validate_task},
    {TaskStateName::SCAN_SCENE, &TaskFsm::enter_scan_scene},
    {TaskStateName::EXECUTE_ITEM, &TaskFsm::enter_execute_item},
    {TaskStateName::RECORD_RESULT, nullptr},
    {TaskStateName::RECOVERY, &TaskFsm::enter_recovery},
    {TaskStateName::CANCEL_PENDING, &TaskFsm::enter_cancel_pending},
    {TaskStateName::FINISHED, &TaskFsm::enter_terminal_ok},
    {TaskStateName::TASK_FAILED, &TaskFsm::enter_task_failed},
    {TaskStateName::FAULT, &TaskFsm::enter_fault},
    {TaskStateName::CANCELLED, &TaskFsm::enter_cancelled},
  };
  return kEntryActions;
}

std::vector<std::string> TaskFsm::verify_tables()
{
  std::vector<std::string> problems;

  const auto & entries = entry_actions();
  for (const auto state : all_states()) {
    if (entries.find(state) == entries.end()) {
      problems.push_back(std::string("ENTRY_ACTIONS missing state: ") + to_string(state));
    }
    if (default_state_timeouts().find(state) == default_state_timeouts().end()) {
      problems.push_back(std::string("DEFAULT_STATE_TIMEOUTS missing state: ") + to_string(state));
    }
  }

  std::vector<std::pair<TaskStateName, std::string>> seen;
  for (const auto & transition : transitions()) {
    // fallback / else_state 只对"守卫拒绝后必须离开"的分支有意义，必须与 guard 同现。
    if (transition.guard == nullptr &&
      (transition.fallback.has_value() || transition.else_state.has_value()))
    {
      problems.push_back(
        std::string("fallback/else_state requires a guard: ") + to_string(transition.state) +
        "/" + transition.event);
    }
    const auto key = std::make_pair(transition.state, transition.event);
    if (std::find(seen.begin(), seen.end(), key) != seen.end()) {
      problems.push_back(
        std::string("duplicated (state, event) key: ") + to_string(transition.state) +
        "/" + transition.event);
    }
    seen.push_back(key);
  }

  return problems;
}

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------

TaskFsm::TaskFsm(mtc::core::ClockPtr clock, const FsmsConfig & config, TaskStateName initial_state)
: clock_(clock ? std::move(clock) : std::make_shared<mtc::core::SteadyClock>()),
  config_(config),
  state_(initial_state)
{
  for (const auto & transition : transitions()) {
    transition_index_[std::make_pair(transition.state, transition.event)] = &transition;
  }
  state_entered_at_ = clock_->now();
}

// ---------------------------------------------------------------------------
// 只读视图
// ---------------------------------------------------------------------------

bool TaskFsm::has_active_task() const
{
  return contains_state(active_task_states(), state_);
}

bool TaskFsm::action_finished() const
{
  return contains_state(terminal_states(), state_);
}

double TaskFsm::elapsed_in_state() const
{
  return clock_->now() - state_entered_at_;
}

std::optional<double> TaskFsm::elapsed_task_time() const
{
  if (!task_started_at.has_value()) {
    return std::nullopt;
  }
  return clock_->now() - *task_started_at;
}

TaskStateSnapshot TaskFsm::snapshot(const std::string & detail) const
{
  TaskStateSnapshot snapshot_value;
  snapshot_value.task_id = current_task.has_value() ? current_task->task_id : "";
  snapshot_value.state = to_string(state_);
  snapshot_value.substate = substate;
  snapshot_value.completed_count = static_cast<uint8_t>(completed_count);
  snapshot_value.total_count = static_cast<uint8_t>(total_count());
  snapshot_value.last_error_code = last_error_code;
  snapshot_value.detail = detail.empty() ? last_error_message : detail;
  return snapshot_value;
}

TaskOutcome TaskFsm::build_outcome() const
{
  TaskOutcome outcome;
  outcome.completed_count = static_cast<uint8_t>(completed_count);
  outcome.completed_slots = completed_slots;

  if (state_ == TaskStateName::FINISHED) {
    outcome.success = true;
    outcome.error_code = mtc::core::ErrorCodes::OK;
    std::string joined;
    for (size_t i = 0; i < completed_slots.size(); ++i) {
      if (i != 0u) {
        joined += ",";
      }
      joined += completed_slots[i];
    }
    outcome.message = "task finished: " + joined;
    return outcome;
  }

  if (state_ == TaskStateName::CANCELLED) {
    outcome.success = false;
    outcome.error_code = mtc::core::ErrorCodes::OK;
    outcome.message = "cancelled after stop confirmation (completed " +
      std::to_string(completed_count) + ")";
    return outcome;
  }

  outcome.success = false;
  outcome.error_code = last_error_code;
  if (!last_error_message.empty()) {
    outcome.message = last_error_message;
  } else {
    outcome.message = std::string(to_string(state_)) + " (" +
      mtc::core::ErrorCodes::name_of(last_error_code) + ")";
  }
  return outcome;
}

// ---------------------------------------------------------------------------
// 超时看门狗
// ---------------------------------------------------------------------------

std::optional<StepResult> TaskFsm::check_timeouts()
{
  if (state_timeout_expired()) {
    return step(Event::TIMEOUT, EventPayload{});
  }
  return std::nullopt;
}

std::optional<StepResult> TaskFsm::check_task_deadline()
{
  if (!current_task.has_value() || !has_active_task()) {
    return std::nullopt;
  }
  const auto elapsed = elapsed_task_time();
  if (!elapsed.has_value()) {
    return std::nullopt;
  }
  const double limit_s = static_cast<double>(current_task->timeout_ms) / 1000.0;
  if (*elapsed < limit_s) {
    return std::nullopt;
  }

  last_error_code = mtc::core::ErrorCodes::MOTION_TIMEOUT;
  last_error_message = "task " + current_task->task_id + " exceeded timeout_ms=" +
    std::to_string(current_task->timeout_ms) + " (elapsed " +
    std::to_string(*elapsed) + "s)";
  return enter_state(TaskStateName::FAULT, "task_deadline_exceeded", EventPayload{});
}

// ---------------------------------------------------------------------------
// 核心入口
// ---------------------------------------------------------------------------

StepResult TaskFsm::step(const std::string & event, const EventPayload & payload)
{
  const auto found = transition_index_.find(std::make_pair(state_, event));
  if (found == transition_index_.end()) {
    ignored_events.emplace_back(to_string(state_), event);
    StepResult result;
    result.state = state_;
    result.reason = std::string("no transition for (") + to_string(state_) + ", " + event + ")";
    return result;
  }

  const Transition & transition = *found->second;

  if (transition.guard != nullptr) {
    const bool allowed = (this->*(transition.guard))(payload);
    if (!allowed) {
      const bool is_goal_guard =
        transition.guard == &TaskFsm::guard_goal_admissible ||
        transition.guard == &TaskFsm::guard_goal_denied;
      if (is_goal_guard) {
        // §7.2 并发拒绝；两种情况都保留守卫写入的具体原因
        if (has_active_task()) {
          register_second_goal_rejection(payload);
        }
        StepResult result;
        result.state = state_;
        result.reason = last_rejection.has_value() ? last_rejection->message : "goal rejected by guard";
        result.goal_rejected = true;
        return result;
      }
      if (transition.fallback.has_value()) {
        return enter_state(
          *transition.fallback, event, payload,
          "guard denied -> fallback");
      }
      if (transition.else_state.has_value()) {
        return enter_state(
          *transition.else_state, event, payload,
          "guard denied -> else_state");
      }
      ignored_events.emplace_back(to_string(state_), event);
      StepResult result;
      result.state = state_;
      result.reason = "guard rejected " + event;
      return result;
    }
    // 转移被批准：在这里重置计时基准，保证后续超时判定从"本次有效进展"开始，
    // 而不是上一次进入状态的时刻（对齐 Python 的 _touch() 语义）。
    touch();
  }

  return enter_state(transition.next_state, event, payload);
}

StepResult TaskFsm::enter_state(
  TaskStateName next_state, const std::string & event, const EventPayload & payload,
  const std::string & note)
{
  const TaskStateName previous = state_;
  previous_state_ = previous;
  state_ = next_state;
  touch();
  transition_log.push_back(
    TransitionRecord{to_string(previous), event, to_string(next_state), note});

  std::vector<ActionRequest> actions;
  const auto & entries = entry_actions();
  const auto found = entries.find(next_state);
  if (found != entries.end() && found->second != nullptr) {
    auto produced = (this->*(found->second))(payload);
    actions.insert(actions.end(), produced.begin(), produced.end());
  }

  ActionRequest publish;
  publish.kind = ActionKind::PUBLISH_STATE;
  publish.snapshot = snapshot();
  actions.push_back(publish);

  StepResult result;
  result.state = state_;
  result.actions = std::move(actions);
  result.reason = event;
  return result;
}

// ---------------------------------------------------------------------------
// 进入处理函数
// ---------------------------------------------------------------------------

std::vector<ActionRequest> TaskFsm::enter_self_check(const EventPayload & payload)
{
  (void)payload;
  substate = "SELF_CHECK";
  return {};
}

std::vector<ActionRequest> TaskFsm::enter_wait_task(const EventPayload & payload)
{
  (void)payload;
  // 终态复位：清空上一轮任务上下文，恢复可受理状态。
  substate = "WAIT_TASK";
  current_task.reset();
  current_index = 0;
  current_slot.clear();
  current_object_id.clear();
  completed_slots.clear();
  completed_count = 0;
  recovery_attempts = 0;
  last_error_code = mtc::core::ErrorCodes::OK;
  last_error_message.clear();
  last_validation_error.clear();
  last_outcome.reset();
  task_started_at.reset();
  return {};
}

std::vector<ActionRequest> TaskFsm::enter_validate_task(const EventPayload & payload)
{
  (void)payload;
  // 受理 Goal 后立刻做全量校验（§4.3），并把结论翻译成事件。
  substate = "VALIDATE_TASK";
  if (current_task.has_value()) {
    const ValidationResult validation = validate_task_request(*current_task);
    if (validation.ok) {
      pending_validation_event_ = Event::VALIDATED;
    } else {
      last_error_code = validation.error_code;
      last_error_message = validation.message;
      last_validation_error = validation.reason;
      pending_validation_event_ = Event::VALIDATION_FAILED;
    }
  } else {
    last_error_code = mtc::core::ErrorCodes::INVALID_TASK;
    last_error_message = "no task to validate";
    last_validation_error = "missing_task";
    pending_validation_event_ = Event::VALIDATION_FAILED;
  }
  return {};
}

std::vector<ActionRequest> TaskFsm::enter_scan_scene(const EventPayload & payload)
{
  (void)payload;
  current_object_id.clear();
  substate = "SCAN_SCENE";
  return {};
}

std::vector<ActionRequest> TaskFsm::enter_execute_item(const EventPayload & payload)
{
  if (!payload.object_id.empty()) {
    current_object_id = payload.object_id;
  }
  substate = "EXECUTE_ITEM";
  current_slot = slot_for_current_index();

  ActionRequest action;
  action.kind = ActionKind::CALL_PICK_PLACE;
  action.pick_place = build_pick_place_request(current_slot);
  return {action};
}

std::vector<ActionRequest> TaskFsm::enter_recovery(const EventPayload & payload)
{
  const int error_code = extract_error_code(payload, mtc::core::ErrorCodes::TARGET_NOT_FOUND);
  last_error_code = static_cast<uint16_t>(error_code);
  last_error_message = !payload.message.empty() ?
    payload.message : mtc::core::ErrorCodes::name_of(static_cast<uint16_t>(error_code));
  substate = "RECOVERY";
  // 注意：recovery_attempts 统计"已批准的重试次数"，进入 RECOVERY 本身不计数，
  // 因此首次尝试不占用预算（recovery_max_attempts=1 恰好允许一次重试）。
  return {};
}

std::vector<ActionRequest> TaskFsm::enter_cancel_pending(const EventPayload & payload)
{
  (void)payload;
  substate = "CANCEL_PENDING";
  ActionRequest action;
  action.kind = ActionKind::CANCEL_CHILD;
  action.request_id = current_request_id();
  return {action};
}

std::vector<ActionRequest> TaskFsm::enter_terminal_ok(const EventPayload & payload)
{
  (void)payload;
  substate = "FINISHED";
  current_slot.clear();
  current_object_id.clear();
  last_error_code = mtc::core::ErrorCodes::OK;
  last_error_message.clear();
  last_outcome = build_outcome();
  return {};
}

std::vector<ActionRequest> TaskFsm::enter_task_failed(const EventPayload & payload)
{
  (void)payload;
  substate = "TASK_FAILED";
  if (last_error_code == mtc::core::ErrorCodes::OK) {
    last_error_code = mtc::core::ErrorCodes::INVALID_TASK;
    last_error_message = "task failed";
  }
  last_outcome = build_outcome();
  return {};
}

std::vector<ActionRequest> TaskFsm::enter_cancelled(const EventPayload & payload)
{
  (void)payload;
  substate = "CANCELLED";
  last_outcome = build_outcome();
  return {};
}

std::vector<ActionRequest> TaskFsm::enter_fault(const EventPayload & payload)
{
  (void)payload;
  substate = "FAULT";
  // 保留已由守卫/超时翻译好的具体错误码；仅在没有更具体原因时才退化。
  if (last_error_code == mtc::core::ErrorCodes::OK) {
    last_error_code = mtc::core::ErrorCodes::HARDWARE_FAULT;
    last_error_message = "entered FAULT without a specific error code";
  }
  last_outcome = build_outcome();
  // §3.1：FAULT 需停止请求并锁定新的动作，等待人工排查
  ActionRequest stop;
  stop.kind = ActionKind::REQUEST_STOP;
  ActionRequest lock;
  lock.kind = ActionKind::LOCK_ACTIONS;
  return {stop, lock};
}

// ---------------------------------------------------------------------------
// 守卫函数
// ---------------------------------------------------------------------------

bool TaskFsm::guard_goal_denied(const EventPayload & payload)
{
  (void)payload;
  // §7.2：已有进行中 Goal 时，第二个 Goal 一律拒绝，不得排队执行。
  // 守卫总是返回 false，由 step() 依据 has_active_task 登记拒绝原因。
  return false;
}

bool TaskFsm::guard_goal_admissible(const EventPayload & payload)
{
  if (has_active_task()) {
    return false;
  }

  if (!payload.goal.has_value()) {
    GoalRejection rejection;
    if (!payload.goal_parse_error.empty()) {
      rejection.reason = "unparsable_goal";
      rejection.error_code = mtc::core::ErrorCodes::INVALID_TASK;
      rejection.message = "cannot parse goal: " + payload.goal_parse_error;
    } else {
      rejection.reason = "missing_goal";
      rejection.error_code = mtc::core::ErrorCodes::INVALID_TASK;
      rejection.message = "goal payload missing";
    }
    last_rejection = rejection;
    return false;
  }

  const TaskRequest & task = *payload.goal;
  if (task.task_id.find_first_not_of(" \t\n\r\f\v") == std::string::npos) {
    GoalRejection rejection;
    rejection.reason = "empty_task_id";
    rejection.error_code = mtc::core::ErrorCodes::INVALID_TASK;
    rejection.message = "task_id must not be empty";
    last_rejection = rejection;
    return false;
  }

  current_task = task;
  current_index = 0;
  completed_slots.clear();
  completed_count = 0;
  recovery_attempts = 0;
  last_error_code = mtc::core::ErrorCodes::OK;
  last_error_message.clear();
  last_validation_error.clear();
  last_rejection.reset();
  last_outcome.reset();
  task_started_at = clock_->now();
  return true;
}

bool TaskFsm::guard_target_usable(const EventPayload & payload)
{
  // object_id 必须在任务执行前由感知选定，执行中不得静默切换。
  if (payload.object_id.empty()) {
    last_error_code = mtc::core::ErrorCodes::TARGET_NOT_FOUND;
    last_error_message = "target_found without object_id";
    return false;
  }
  // 颜色双重校验：只要调用方给出 expected_color，就必须与当前项颜色一致。
  // 若当前项颜色不可确定（索引越界/无任务），保守拒绝（对齐 Python
  // `expected not in (None, self._current_color())` 的失败分支）。
  const auto expected = current_color();
  if (payload.expected_color.has_value()) {
    if (!expected.has_value() || *payload.expected_color != *expected) {
      last_error_code = mtc::core::ErrorCodes::INVALID_TASK;
      last_error_message = "expected_color mismatch on target_found";
      return false;
    }
  }
  current_object_id = payload.object_id;
  return true;
}

bool TaskFsm::guard_error_recoverable(const EventPayload & payload)
{
  // 只有"明确安全可重试"的错误才允许进入 RECOVERY。
  // MOTION_STATUS_UNKNOWN(230) 等不确定在途运动错误返回 false，
  // 由转移表的 fallback 直接进入 FAULT（不自动重试）。
  const int error_code = extract_error_code(payload, mtc::core::ErrorCodes::HARDWARE_FAULT);
  last_error_code = static_cast<uint16_t>(error_code);
  last_error_message = !payload.message.empty() ?
    payload.message : mtc::core::ErrorCodes::name_of(static_cast<uint16_t>(error_code));
  return config_.is_recoverable(error_code);
}

bool TaskFsm::guard_all_items_done(const EventPayload & payload)
{
  (void)payload;
  // 全部完成 -> FINISHED；否则由转移表的 else_state 回到 SCAN_SCENE。
  return total_count() > 0u && completed_count >= total_count();
}

bool TaskFsm::guard_retry_budget_available(const EventPayload & payload)
{
  (void)payload;
  // 重试预算：recovery_max_attempts 指**额外重试**次数（不含首次尝试）。
  // recovery_attempts 只在重试被批准时 +1；预算耗尽则由转移表的
  // fallback 转入 TASK_FAILED，不会出现"守卫拒绝后原地不动"的死循环。
  if (!config_.is_recoverable(static_cast<int>(last_error_code))) {
    return false;
  }
  if (recovery_attempts >= std::max(0, config_.recovery_max_attempts)) {
    return false;
  }
  recovery_attempts += 1;
  return true;
}

bool TaskFsm::guard_cancel_timeout(const EventPayload & payload)
{
  (void)payload;
  // CANCEL_PENDING 超时 -> FAULT(520)；仅真超时才放行该转移。
  if (!state_timeout_expired()) {
    return false;
  }
  last_error_code = mtc::core::ErrorCodes::CANCEL_NOT_CONFIRMED;
  last_error_message =
    "cancel not confirmed before timeout (child never reported stopped)";
  return true;
}

bool TaskFsm::guard_state_timed_out(const EventPayload & payload)
{
  (void)payload;
  if (!state_timeout_expired()) {
    return false;
  }
  apply_timeout_error();
  return true;
}

// ---------------------------------------------------------------------------
// 超时辅助
// ---------------------------------------------------------------------------

bool TaskFsm::state_timeout_expired(std::optional<double> now) const
{
  const auto timeout = config_.timeout_for(state_);
  if (!timeout.has_value()) {
    return false;
  }
  const double current = now.has_value() ? *now : clock_->now();
  return (current - state_entered_at_) >= *timeout;
}

void TaskFsm::apply_timeout_error()
{
  // 把超时翻译成本层错误码（见设计文档 §7.1 默认处置）。
  static const std::map<TaskStateName, uint16_t> kMapping = {
    {TaskStateName::INIT, mtc::core::ErrorCodes::HARDWARE_FAULT},
    {TaskStateName::SELF_CHECK, mtc::core::ErrorCodes::HARDWARE_FAULT},
    {TaskStateName::VALIDATE_TASK, mtc::core::ErrorCodes::INVALID_TASK},
    {TaskStateName::SCAN_SCENE, mtc::core::ErrorCodes::TARGET_NOT_FOUND},
    {TaskStateName::EXECUTE_ITEM, mtc::core::ErrorCodes::MOTION_TIMEOUT},
    {TaskStateName::RECORD_RESULT, mtc::core::ErrorCodes::PLACEMENT_FAILED},
    {TaskStateName::RECOVERY, mtc::core::ErrorCodes::TARGET_NOT_FOUND},
    {TaskStateName::CANCEL_PENDING, mtc::core::ErrorCodes::CANCEL_NOT_CONFIRMED},
  };
  const auto found = kMapping.find(state_);
  const uint16_t code = (found != kMapping.end()) ? found->second : mtc::core::ErrorCodes::HARDWARE_FAULT;
  last_error_code = code;
  last_error_message = std::string("state ") + to_string(state_) + " timed out after " +
    std::to_string(elapsed_in_state()) + "s -> " + mtc::core::ErrorCodes::name_of(code);
}

// ---------------------------------------------------------------------------
// 内部工具
// ---------------------------------------------------------------------------

int TaskFsm::extract_error_code(const EventPayload & payload, int default_code) const
{
  if (payload.error_code.has_value()) {
    return *payload.error_code;
  }
  return default_code;
}

void TaskFsm::touch()
{
  state_entered_at_ = clock_->now();
}

std::optional<int> TaskFsm::current_color() const
{
  if (!current_task.has_value()) {
    return std::nullopt;
  }
  if (current_index >= current_task->ordered_colors.size()) {
    return std::nullopt;
  }
  return current_task->ordered_colors[current_index];
}

std::string TaskFsm::slot_for_current_index() const
{
  if (!current_task.has_value()) {
    return "";
  }
  return current_task->slot_for_index(current_index).value_or("");
}

std::string TaskFsm::current_request_id() const
{
  if (!current_task.has_value()) {
    return "";
  }
  return current_task->task_id + "#" + std::to_string(current_index);
}

PickPlaceGoal TaskFsm::build_pick_place_request(const std::string & slot) const
{
  PickPlaceGoal goal;
  goal.request_id = current_request_id();
  goal.object_id = current_object_id;
  goal.expected_color = current_color().value_or(0);
  goal.target_slot = slot;
  goal.placement_stability_sec =
    current_task.has_value() ? static_cast<float>(current_task->placement_stability_sec) : 0.0f;
  return goal;
}

void TaskFsm::register_second_goal_rejection(const EventPayload & payload)
{
  // 并发拒绝：记录原因、返回拒绝结果，不排队、不切换任务。
  std::string requested_id;
  if (payload.goal.has_value()) {
    requested_id = payload.goal->task_id;
  }
  const std::string active_id = current_task.has_value() ? current_task->task_id : "";

  GoalRejection rejection;
  rejection.reason = "task_already_in_progress";
  rejection.error_code = mtc::core::ErrorCodes::EXECUTION_REJECTED;
  rejection.message = "rejected: task '" + active_id + "' already in progress in state " +
    to_string(state_) + "; a second concurrent goal is not queued";
  rejection.active_task_id = active_id;
  rejection.rejected_task_id = requested_id;
  rejection.active_state = to_string(state_);
  last_rejection = rejection;
}

// ---------------------------------------------------------------------------
// 便捷驱动接口
// ---------------------------------------------------------------------------

StepResult TaskFsm::start()
{
  EventPayload payload;
  payload.source = "start";
  return step(Event::SELF_CHECK_OK, payload);
}

StepResult TaskFsm::report_self_check(bool ok)
{
  EventPayload payload;
  payload.source = "self_check";
  return step(ok ? Event::SELF_CHECK_OK : Event::SELF_CHECK_FAIL, payload);
}

StepResult TaskFsm::submit_goal(const TaskRequest & goal)
{
  if (fault_locked()) {
    StepResult result;
    result.state = state_;
    result.goal_rejected = true;
    result.reason = "FAULT locked; supervised manual recovery interface required";
    return result;
  }
  EventPayload payload;
  payload.goal = goal;
  return step(Event::GOAL_ACCEPTED, payload);
}

StepResult TaskFsm::submit_unparsable_goal(const std::string & parse_error)
{
  EventPayload payload;
  payload.goal_parse_error = parse_error;
  return step(Event::GOAL_ACCEPTED, payload);
}

std::optional<std::string> TaskFsm::take_pending_validation_event()
{
  const auto event = pending_validation_event_;
  pending_validation_event_.reset();
  return event;
}

StepResult TaskFsm::advance_validation()
{
  if (state_ != TaskStateName::VALIDATE_TASK) {
    StepResult result;
    result.state = state_;
    result.reason = "not in VALIDATE_TASK";
    return result;
  }

  auto event = take_pending_validation_event();
  if (!event.has_value()) {
    // 没有缓存结论时现场重算一次，保证接口自洽。
    const ValidationResult validation =
      current_task.has_value() ? validate_task_request(*current_task) : ValidationResult{};
    if (validation.ok) {
      event = std::string(Event::VALIDATED);
    } else {
      event = std::string(Event::VALIDATION_FAILED);
      last_error_code = validation.error_code;
      last_error_message = validation.message;
      last_validation_error = validation.reason;
    }
  }
  return step(*event, EventPayload{});
}

StepResult TaskFsm::report_target(
  const std::string & object_id, std::optional<int> expected_color)
{
  EventPayload payload;
  payload.object_id = object_id;
  payload.expected_color = expected_color;
  return step(Event::TARGET_FOUND, payload);
}

StepResult TaskFsm::report_target_missing(const std::string & reason)
{
  EventPayload payload;
  payload.error_code = mtc::core::ErrorCodes::TARGET_NOT_FOUND;
  payload.message = reason;
  return step(Event::TARGET_NOT_FOUND, payload);
}

StepResult TaskFsm::report_scan_failed(const std::string & reason)
{
  EventPayload payload;
  payload.error_code = mtc::core::ErrorCodes::TARGET_NOT_FOUND;
  payload.message = reason;
  return step(Event::SCAN_FAILED, payload);
}

StepResult TaskFsm::report_item_result(
  bool success, bool placement_verified, int error_code, const std::string & message)
{
  if (state_ != TaskStateName::EXECUTE_ITEM) {
    StepResult result;
    result.state = state_;
    result.reason = std::string("item result ignored: state is ") + to_string(state_) +
      ", not EXECUTE_ITEM";
    return result;
  }

  if (success) {
    if (!placement_verified) {
      // §4.4：placement_verified 必须在 VERIFY_PLACED 通过后才为 true；
      // 反向约束——未验证放置不得记为完成。
      EventPayload payload;
      payload.error_code = mtc::core::ErrorCodes::PLACEMENT_FAILED;
      payload.message = "PickPlace reported success without placement_verified";
      return step(Event::ITEM_FAILED, payload);
    }
    EventPayload payload;
    payload.success = true;
    payload.placement_verified = true;
    payload.error_code = error_code;
    payload.message = message;
    return step(Event::ITEM_SUCCEEDED, payload);
  }

  const auto & non_recoverable = non_recoverable_errors();
  if (std::find(non_recoverable.begin(), non_recoverable.end(), static_cast<uint16_t>(error_code)) !=
    non_recoverable.end())
  {
    // 先登记具体错误码，再走 ITEM_UNSAFE -> FAULT，
    // 使 FAULT 快照保留真实原因（如 230），而不是退化为 HARDWARE_FAULT。
    last_error_code = static_cast<uint16_t>(error_code);
    last_error_message = !message.empty() ?
      message : mtc::core::ErrorCodes::name_of(static_cast<uint16_t>(error_code));
    EventPayload payload;
    payload.error_code = error_code;
    payload.message = last_error_message;
    return step(Event::ITEM_UNSAFE, payload);
  }

  EventPayload payload;
  payload.error_code = (error_code != 0) ? error_code : mtc::core::ErrorCodes::PLACEMENT_FAILED;
  payload.message = message;
  return step(Event::ITEM_FAILED, payload);
}

StepResult TaskFsm::report_item_unsafe(int error_code, const std::string & message)
{
  if (fault_locked()) {return step(Event::ITEM_UNSAFE, EventPayload{});}
  last_error_code = static_cast<uint16_t>(error_code);
  last_error_message = message;
  EventPayload payload;
  payload.error_code = error_code;
  payload.message = message;
  return step(Event::ITEM_UNSAFE, payload);
}

StepResult TaskFsm::report_record_failed(const std::string & reason)
{
  EventPayload payload;
  payload.error_code = mtc::core::ErrorCodes::PLACEMENT_FAILED;
  payload.message = reason;
  return step(Event::RECORD_FAILED, payload);
}

StepResult TaskFsm::record_completed_item(const std::optional<std::string> & slot)
{
  if (state_ != TaskStateName::RECORD_RESULT) {
    StepResult result;
    result.state = state_;
    result.reason = "record ignored: not RECORD_RESULT";
    return result;
  }

  const std::string resolved = slot.has_value() ? *slot : current_slot;
  if (!resolved.empty() &&
    std::find(completed_slots.begin(), completed_slots.end(), resolved) == completed_slots.end())
  {
    completed_slots.push_back(resolved);
  }
  completed_count = completed_slots.size();
  recovery_attempts = 0;
  current_index += 1;
  return step(Event::RECORD_OK, EventPayload{});
}

StepResult TaskFsm::request_retry()
{
  return step(Event::RETRY_APPROVED, EventPayload{});
}

StepResult TaskFsm::deny_retry()
{
  return step(Event::RETRY_DENIED, EventPayload{});
}

StepResult TaskFsm::request_cancel()
{
  return step(Event::CANCEL_REQUESTED, EventPayload{});
}

StepResult TaskFsm::confirm_child_stopped()
{
  return step(Event::CHILD_STOPPED_CONFIRMED, EventPayload{});
}

StepResult TaskFsm::report_goal_rejected(const std::string & reason)
{
  EventPayload payload;
  payload.message = reason;
  return step(Event::GOAL_REJECTED, payload);
}

StepResult TaskFsm::reset()
{
  return step(Event::RESET, EventPayload{});
}

}  // namespace task
}  // namespace mtc
