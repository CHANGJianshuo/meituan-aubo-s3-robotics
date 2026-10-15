// Copyright 2026 meituan_challenge_ws contributors
//
// Task FSM（第一层）：整轮比赛任务状态机。
//
// 迁移自 src/mtc_task/mtc_task/task_fsm.py，结构约定保持一致：
//   * 状态转移集中在 transitions()：`(state, event) -> (next_state, guard, fallback, else_state)`；
//   * 进入状态时的数据更新与动作产出集中在 entry_actions()：
//     `state -> handler`，处理函数命名以 enter_ 开头；
//   * 超时不散落 if：每个状态经 guard_state_timed_out 校验后以 timeout 事件重新查表，
//     "状态超时去哪个状态"同样由转移表描述。
//
// 原迁移的实现差异（本次审计另明确 FAULT 执行终态与锁定契约）：
//   1. guard 由字符串名改为成员函数指针，编译期即可发现拼写错误；
//   2. payload 由 Dict[str, Any] 改为类型化的 EventPayload；
//   3. `_pending_validation_event` 由动态属性改为 optional 成员。
//
// 本类不依赖 ROS，也不做任何真实 IO：所有外部副作用都以 ActionRequest 形式返回。

#ifndef MTC_TASK_CPP__TASK_FSM_HPP_
#define MTC_TASK_CPP__TASK_FSM_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mtc_core_cpp/clock.hpp"
#include "mtc_task_cpp/task_types.hpp"
#include "mtc_task_cpp/task_validation.hpp"

namespace mtc
{
namespace task
{

/// 转移表的可追溯记录（对齐 Python transition_log 元组）。
struct TransitionRecord
{
  std::string from_state;
  std::string event;
  std::string to_state;
  std::string note;
};

/// 并发拒绝的记录（对齐 Python last_rejection dict）。
struct GoalRejection
{
  std::string reason;
  uint16_t error_code = mtc::core::ErrorCodes::OK;
  std::string message;
  std::string active_task_id;
  std::string rejected_task_id;
  std::string active_state;
};

/// 单一事件线程所有：事件、Goal 受理和查询均须串行化。
/// FAULT 结束当前执行但保持任务锁定，普通 RESET 没有复位权限。
class TaskFsm
{
public:
  /// 一条状态转移。
  ///
  /// * `guard` 为 nullptr 表示无条件转移；
  /// * `fallback` 守卫拒绝且**必须离开当前状态**时使用（如不可重试错误 -> FAULT）；
  /// * `else_state` 守卫拒绝且仍要转移、但去向不同时使用（如"还有任务项"二选一）。
  ///   这样可避免在转移表中出现重复的 (state, event) 键。
  struct Transition
  {
    TaskStateName state = TaskStateName::INIT;
    std::string event;
    TaskStateName next_state = TaskStateName::INIT;
    bool (TaskFsm::*guard)(const EventPayload &) = nullptr;
    std::optional<TaskStateName> fallback;
    std::optional<TaskStateName> else_state;
    std::string note;
  };

  using EntryHandler = std::vector<ActionRequest> (TaskFsm::*)(const EventPayload &);

  explicit TaskFsm(
    mtc::core::ClockPtr clock = nullptr,
    const FsmsConfig & config = FsmsConfig::defaults(),
    TaskStateName initial_state = TaskStateName::INIT);

  virtual ~TaskFsm() = default;

  // ---------------------------------------------------------------------
  // 转移表与进入动作表（集中定义，便于审计与自检）
  // ---------------------------------------------------------------------

  static const std::vector<Transition> & transitions();
  static const std::map<TaskStateName, EntryHandler> & entry_actions();

  /// 表结构自检：返回问题清单（空表示通过）。
  ///
  /// 等价于 Python 的 `_self_check_tables()`，但改为可测试的返回值形式，
  /// 使 GTest 能直接断言"没有静默漏配"。
  static std::vector<std::string> verify_tables();

  // ---------------------------------------------------------------------
  // 只读视图
  // ---------------------------------------------------------------------

  TaskStateName state() const {return state_;}
  std::string state_name() const {return to_string(state_);}
  std::optional<TaskStateName> previous_state() const {return previous_state_;}

  /// 是否已有进行中的 Goal（§7.2 并发拒绝判定依据）。
  bool has_active_task() const;
  bool fault_locked() const {return state_ == TaskStateName::FAULT;}
  bool accepts_tasks() const {return state_ == TaskStateName::WAIT_TASK;}
  bool action_finished() const;

  size_t total_count() const
  {
    return current_task.has_value() ? current_task->total_count() : 0u;
  }

  /// 自进入当前状态以来的停留时间（秒）。
  double elapsed_in_state() const;

  /// 当前任务已耗时（秒）；无进行中任务时返回 nullopt。
  std::optional<double> elapsed_task_time() const;

  TaskStateSnapshot snapshot(const std::string & detail = "") const;
  /// 按当前状态生成 ExecuteTask.Result 内容。
  TaskOutcome build_outcome() const;

  /// 是否有待消费的校验结论（由 advance_validation 消费一次）。
  bool has_pending_validation_event() const {return pending_validation_event_.has_value();}

  // ---------------------------------------------------------------------
  // 核心入口
  // ---------------------------------------------------------------------

  /// 推进状态机，返回新状态与动作请求列表。
  StepResult step(const std::string & event, const EventPayload & payload = EventPayload());

  /// 当前状态已超时则返回一次 timeout 驱动结果；否则返回 nullopt。
  ///
  /// 计时基准是**进入该状态的时刻**：超时表示"在该状态停留过久"。
  /// 本方法不做任何心跳刷新，否则周期性调用会无限推迟超时。
  std::optional<StepResult> check_timeouts();

  /// 任务级截止时间（timeout_ms）看门狗。
  ///
  /// 超时说明可能仍有在途运动，按 §7.1 处置进入 FAULT（220），不自动重发。
  std::optional<StepResult> check_task_deadline();

  // ---------------------------------------------------------------------
  // 便捷驱动接口（ROS 外壳与测试共用）
  // ---------------------------------------------------------------------

  StepResult start();                    ///< INIT -> SELF_CHECK
  StepResult report_self_check(bool ok);
  StepResult submit_goal(const TaskRequest & goal);
  /// 无法解析的 Goal：等价于 Python 的 "unparsable_goal" 拒绝。
  StepResult submit_unparsable_goal(const std::string & parse_error);
  StepResult advance_validation();
  StepResult report_target(
    const std::string & object_id, std::optional<int> expected_color = std::nullopt);
  StepResult report_target_missing(const std::string & reason = "no unique candidate for current color");
  StepResult report_scan_failed(const std::string & reason = "scan retry budget exhausted");
  StepResult report_item_result(
    bool success, bool placement_verified, int error_code, const std::string & message);
  StepResult report_item_unsafe(int error_code, const std::string & message);
  StepResult record_completed_item(const std::optional<std::string> & slot = std::nullopt);
  StepResult report_record_failed(const std::string & reason = "record failed");
  StepResult request_retry();
  StepResult deny_retry();
  StepResult request_cancel();
  StepResult confirm_child_stopped();
  StepResult report_goal_rejected(const std::string & reason = "goal withdrawn after acceptance");
  StepResult reset();

  /// 取出 enter_validate_task 产生的待发事件（由外壳/测试消费一次）。
  std::optional<std::string> take_pending_validation_event();

  // ---------------------------------------------------------------------
  // 可观测状态（供外壳与测试；命名对齐 Python 公开属性）
  // ---------------------------------------------------------------------

  std::optional<TaskRequest> current_task;
  size_t current_index = 0;
  std::string current_slot;
  std::string current_object_id;
  std::string substate;
  std::vector<std::string> completed_slots;
  size_t completed_count = 0;
  int recovery_attempts = 0;
  uint16_t last_error_code = mtc::core::ErrorCodes::OK;
  std::string last_error_message;
  std::string last_validation_error;
  std::optional<GoalRejection> last_rejection;
  std::optional<TaskOutcome> last_outcome;
  std::optional<double> task_started_at;
  std::vector<TransitionRecord> transition_log;
  std::vector<std::pair<std::string, std::string>> ignored_events;

  const FsmsConfig & config() const {return config_;}

protected:
  // -- 守卫函数（全部以 guard_ 开头，供转移表引用）-----------------------

  /// §7.2：已有进行中 Goal 时，第二个 Goal 一律拒绝，不得排队执行。
  bool guard_goal_denied(const EventPayload & payload);
  /// WAIT_TASK 下受理 Goal：先做最基本的 task_id 检查，再进入 VALIDATE_TASK。
  bool guard_goal_admissible(const EventPayload & payload);
  /// object_id 必须在任务执行前由感知选定，执行中不得静默切换。
  bool guard_target_usable(const EventPayload & payload);
  /// 只有"明确安全可重试"的错误才允许进入 RECOVERY。
  bool guard_error_recoverable(const EventPayload & payload);
  /// 全部完成 -> FINISHED；否则由转移表的 else_state 回到 SCAN_SCENE。
  bool guard_all_items_done(const EventPayload & payload);
  /// 重试预算：预算耗尽由转移表的 fallback 转入 TASK_FAILED。
  bool guard_retry_budget_available(const EventPayload & payload);
  /// CANCEL_PENDING 超时 -> FAULT(520)；仅真超时才放行该转移。
  bool guard_cancel_timeout(const EventPayload & payload);
  bool guard_state_timed_out(const EventPayload & payload);

  // -- 进入处理函数（全部以 enter_ 开头，供 entry_actions 引用）----------

  std::vector<ActionRequest> enter_self_check(const EventPayload & payload);
  std::vector<ActionRequest> enter_wait_task(const EventPayload & payload);
  std::vector<ActionRequest> enter_validate_task(const EventPayload & payload);
  std::vector<ActionRequest> enter_scan_scene(const EventPayload & payload);
  std::vector<ActionRequest> enter_execute_item(const EventPayload & payload);
  std::vector<ActionRequest> enter_recovery(const EventPayload & payload);
  std::vector<ActionRequest> enter_cancel_pending(const EventPayload & payload);
  std::vector<ActionRequest> enter_terminal_ok(const EventPayload & payload);
  std::vector<ActionRequest> enter_task_failed(const EventPayload & payload);
  std::vector<ActionRequest> enter_cancelled(const EventPayload & payload);
  std::vector<ActionRequest> enter_fault(const EventPayload & payload);

  // -- 内部 -------------------------------------------------------------

  StepResult enter_state(
    TaskStateName next_state, const std::string & event, const EventPayload & payload,
    const std::string & note = "");

  bool state_timeout_expired(std::optional<double> now = std::nullopt) const;
  void apply_timeout_error();
  int extract_error_code(const EventPayload & payload, int default_code) const;
  void touch();
  std::optional<int> current_color() const;
  std::string slot_for_current_index() const;
  std::string current_request_id() const;
  PickPlaceGoal build_pick_place_request(const std::string & slot) const;
  void register_second_goal_rejection(const EventPayload & payload);

  mtc::core::ClockPtr clock_;
  FsmsConfig config_;
  TaskStateName state_ = TaskStateName::INIT;
  std::optional<TaskStateName> previous_state_;
  double state_entered_at_ = 0.0;
  std::optional<std::string> pending_validation_event_;
  std::map<std::pair<TaskStateName, std::string>, const Transition *> transition_index_;
};

}  // namespace task
}  // namespace mtc

#endif  // MTC_TASK_CPP__TASK_FSM_HPP_
