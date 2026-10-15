// Copyright 2026 meituan_challenge_ws contributors
//
// 单块电池抓放状态机（第二层 PickPlace FSM）C++ 实现。
//
// 迁移自 Python src/mtc_manipulation/mtc_manipulation/pick_place_fsm.py。
//
// 安全语义（均有 GTest 覆盖，逐条对应 Python 的 8 条安全语义）：
//   1. placement_verified 只允许由 VERIFY_PLACED 成功路径写入，且仅在该处写；
//   2. VERIFY_ATTACHED 必须有几何级（或更高）挂载证据，否则 ATTACH_NOT_VERIFIED=310，
//      禁止进入 LIFT / TRANSPORT；
//   3. ENGAGE 失败 -> ENGAGE_FAILED=300，且只有确认"无载荷"时才允许退回重试；
//   4. SEAT 后未确认桌面承托 -> SEAT_NOT_VERIFIED=400，禁止进入 UNLOAD / DISENGAGE；
//   5. DISENGAGE 解锁失败或解锁结果未知 -> UNLOCK_FAILED=410 / RELEASE_NOT_VERIFIED=420
//      -> FAULT，禁止强行抽出 / 上抬；
//   6. 取消 -> CANCEL_PENDING，等待 MotionPort.request_stop() 返回 stop_confirmed；
//      未确认 -> FAULT(CANCEL_NOT_CONFIRMED=520)，绝不因收到 cancel 就声称已安全停止；
//   7. 运动状态未知 MOTION_STATUS_UNKNOWN=230 -> FAULT，并禁止自动重发同一条运动命令
//      （进入 FAULT 后 motion_resend_allowed=false，即使人工调用 step() 也不会重发）；
//   8. VERIFY_PLACED 必须在目标区域内连续稳定 placement_stability_sec（默认 3.0 s）
//      才允许 SUCCESS，否则 PLACEMENT_FAILED=430 -> FAILED。

#ifndef MTC_MANIPULATION_CPP__PICK_PLACE_FSM_HPP_
#define MTC_MANIPULATION_CPP__PICK_PLACE_FSM_HPP_

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "mtc_manipulation_cpp/pick_place_types.hpp"
#include "mtc_manipulation_cpp/tool_strategy.hpp"

namespace mtc
{
namespace manipulation
{

/// 单一事件线程所有：start/step/cancel 与查询均须串行化。
/// ROS 多线程回调应入队；阻塞运动由工作线程运行并经线程安全执行器取消。
class PickPlaceFsm
{
public:
  /// 状态处理器返回：nullopt 表示"端口未完成，需要等待"。
  using HandlerOutcome = std::optional<StepResult>;
  using Handler = HandlerOutcome (PickPlaceFsm::*)();

  PickPlaceFsm(
    PerceptionPortPtr perception,
    PlannerPortPtr planner,
    MotionPortPtr motion,
    ToolPortPtr tool,
    mtc::core::ClockPtr clock,
    ToolStrategyPtr strategy,
    PickPlaceConfig config = PickPlaceConfig());

  virtual ~PickPlaceFsm() = default;

  // ---------------------------------------------------------------------
  // 生命周期
  // ---------------------------------------------------------------------

  /// 校验并受理一次抓放请求。非法目标工位立即 INVALID_TASK。
  ///
  /// placement_stability_sec 优先级：Goal 显式传入 > 配置默认值。
  void start(
    const std::string & object_id,
    int expected_color,
    const std::string & target_slot,
    const std::string & request_id = "",
    std::optional<double> placement_stability_sec = std::nullopt);

  /// 记录取消请求。真正的安全停止必须在 CANCEL_PENDING 中由后端确认。
  void request_cancel(const std::string & reason = "");

  // ---------------------------------------------------------------------
  // 单步推进
  // ---------------------------------------------------------------------

  StepResult step();
  PickPlaceState step_until_terminal(int max_ticks = 500);

  /// 驱动到终态。tick_seconds > 0 时会推进假时钟（仅适用于 ManualClock）。
  PickPlaceResult run(int max_ticks = 500, double tick_seconds = 0.0);

  PickPlaceResult result() const;
  PickPlaceFeedback feedback() const;

  /// 外部监护（如 Action 执行步数上限）判定危险/不确定时，强制进入 FAULT。
  ///
  /// 默认 resend_allowed=false：一旦判定运动状态不确定，禁止自动重发任何指令。
  StepResult force_fault(
    uint16_t code, const std::string & message, bool resend_allowed = false);

  // ---------------------------------------------------------------------
  // 只读视图
  // ---------------------------------------------------------------------

  PickPlaceState state() const {return state_;}
  std::string state_name() const {return to_string(state_);}
  uint16_t error_code() const {return error_code_;}
  const std::string & message() const {return message_;}

  /// 安全语义 1 的只读出口：唯一写入口是私有的 set_placement_verified()。
  bool placement_verified() const {return placement_verified_;}

  /// 运动重发是否仍被允许（安全语义 7）。
  bool motion_resend_allowed() const {return motion_resend_allowed_;}

  bool stop_confirmed() const {return stop_confirmed_;}
  int recovery_attempts() const {return recovery_attempts_;}
  bool object_attached_estimated() const {return object_attached_estimated_;}
  double stability_verified_for_s() const {return stability_verified_for_s_;}
  int step_count() const {return step_count_;}
  bool started() const {return started_;}

  const std::string & request_id() const {return request_id_;}
  const std::string & object_id() const {return object_id_;}
  const std::string & target_slot() const {return target_slot_;}
  int expected_color() const {return expected_color_;}
  double placement_stability_sec() const {return placement_stability_sec_;}
  const PickPlaceConfig & config() const {return config_;}

  const std::vector<PickPlaceState> & states_history() const {return states_history_;}
  const std::vector<std::string> & command_ids() const {return command_ids_;}
  const std::vector<std::string> & primitive_calls() const {return primitive_calls_;}

  /// 事件回调：异常不得破坏状态机时序（内部捕获）。
  std::function<void(const std::string &, PickPlaceState)> on_event;

  /// 表结构自检：返回问题清单（空表示通过）。
  static std::vector<std::string> verify_tables();

  /// 状态处理器表（集中定义，便于审计）。
  ///
  /// 迁移对应 Python 的模块级 `_STATE_ACTIONS`；在类内定义以便取到
  /// protected 处理器的成员函数指针。
  static const std::map<PickPlaceState, Handler> & state_actions();

protected:
  // -- 状态处理器（对应 Python _h_* 方法）--------------------------------

  HandlerOutcome h_resolve_target();
  HandlerOutcome h_plan_acquire();
  HandlerOutcome h_move_pre_align();
  HandlerOutcome h_engage();
  HandlerOutcome h_verify_engagement();
  HandlerOutcome h_test_lift();
  HandlerOutcome h_verify_attached();
  HandlerOutcome h_lift();
  HandlerOutcome h_transport();
  HandlerOutcome h_move_pre_seat();
  HandlerOutcome h_seat();
  HandlerOutcome h_verify_seated();
  HandlerOutcome h_unload();
  HandlerOutcome h_disengage();
  HandlerOutcome h_retreat();
  HandlerOutcome h_verify_placed();
  HandlerOutcome h_recovery();
  HandlerOutcome h_cancel_pending();
  HandlerOutcome h_success();
  HandlerOutcome h_failed();
  HandlerOutcome h_fault();
  HandlerOutcome h_cancelled();

  // -- 内部 -------------------------------------------------------------

  void enter_state(PickPlaceState state);
  void emit(const std::string & event, PickPlaceState state);
  std::string next_command_id(const std::string & label);
  float progress() const;

  /// 安全语义 1 的唯一写入口。
  ///
  /// 若当前状态不是 VERIFY_PLACED，抛出 std::logic_error：
  /// 宁可让违规写入立即失败，也不允许"放置已验证"被静默伪造。
  void set_placement_verified();

  StepResult make_error(
    uint16_t code, const std::string & message, bool resumable = false,
    std::optional<PickPlaceState> next_state = std::nullopt);

  StepResult make_fault(
    uint16_t code, const std::string & message, bool resend_allowed = true);

  /// 是否允许退回重试：必须确认当前无载荷且运动状态已知。
  bool is_resumable_context();

  /// 构造并缓存运动命令：同一状态内的多次 step 必须复用同一条命令。
  ///
  /// 安全语义：运动在途时（端口返回 nullopt）绝不允许重新发令。
  mtc::core::MotionCommand cached_command(
    const std::string & label, const mtc::core::MotionPrimitive & primitive);

  mtc::core::MotionCommand make_command(
    const std::string & label, const mtc::core::MotionPrimitive & primitive);

  /// 把运动后端结果映射为 FSM 转移；返回 nullopt 表示成功到位。
  std::optional<StepResult> motion_failure(const mtc::core::MotionResult & result);

  /// 取缓存的原语列表；未缓存时调用 factory 并缓存。
  ///
  /// 返回引用指向内部缓存，生命周期与 FSM 一致。
  const mtc::core::MotionPrimitiveList & primitives(
    const std::string & key,
    const std::function<mtc::core::MotionPrimitiveList()> & factory);

  void clear_port_results();

  std::optional<StepResult> placement_wait_or_fail(const std::string & detail);
  bool accept_plan(const mtc::core::PlanResult & plan,
    const mtc::core::MotionPrimitiveList & requested);
  std::optional<mtc::core::MotionResult> execute_motion(const mtc::core::MotionCommand & command);

  // -- 缓存辅助（每种端口结果独立缓存，键为 state:label）-----------------

  std::string make_cache_key(const std::string & label) const;

  /// 带缓存的端口调用。
  ///
  /// 返回 false 表示端口尚未返回（等价于 Python 的 `_do` 返回 _RUN），
  /// 调用方应立即 `return std::nullopt;` 进入等待。
  /// 返回 true 时 out 已被填充（可能来自缓存，因此副作用只发生一次）。
  template<typename T, typename Fn>
  bool resolve_cached(
    std::map<std::string, T> & cache, const std::string & label, Fn && call, T & out)
  {
    const std::string key = make_cache_key(label);
    const auto found = cache.find(key);
    if (found != cache.end()) {
      out = found->second;
      return true;
    }
    auto value = call();
    if (!value.has_value()) {
      return false;
    }
    cache.emplace(key, *value);
    out = *value;
    return true;
  }

  // ---------------------------------------------------------------------
  // 状态
  // ---------------------------------------------------------------------

  PerceptionPortPtr perception_;
  PlannerPortPtr planner_;
  MotionPortPtr motion_;
  ToolPortPtr tool_;
  mtc::core::ClockPtr clock_;
  ToolStrategyPtr strategy_;
  PickPlaceConfig config_;

  // --- 请求上下文 ---
  std::string request_id_;
  std::string object_id_;
  int expected_color_ = 0;
  std::string target_slot_;
  double placement_stability_sec_ = 3.0;

  // --- 运行状态 ---
  PickPlaceState state_ = PickPlaceState::RESOLVE_TARGET;
  uint16_t error_code_ = mtc::core::ErrorCodes::OK;
  std::string message_;
  std::vector<PickPlaceState> states_history_;
  /// 安全语义 1：private + 唯一写入口，从类型层面阻止外部直接写 true。
  bool placement_verified_ = false;
  bool object_attached_estimated_ = false;

  // --- 内部簿记 ---
  double state_entered_at_ = 0.0;
  double started_at_ = 0.0;
  bool started_ = false;
  int step_count_ = 0;
  int recovery_attempts_ = 0;
  bool resumable_error_ = false;
  std::optional<PickPlaceState> last_resumable_state_;
  bool cancel_pending_ = false;
  std::string cancel_reason_;
  bool stop_confirmed_ = false;
  bool motion_resend_allowed_ = true;
  int command_seq_ = 0;
  uint64_t invocation_id_ = 0;
  std::vector<std::string> command_ids_;
  std::vector<std::string> primitive_calls_;
  std::optional<double> stability_started_at_;
  std::optional<double> placement_last_stamp_;
  std::optional<double> placement_last_received_at_;
  double stability_verified_for_s_ = 0.0;
  std::optional<mtc::core::Vector3> placement_last_position_;

  // --- 端口结果缓存（每状态每键只执行一次副作用）---
  std::map<std::string, mtc::core::DetectionResult> detection_cache_;
  std::map<std::string, mtc::core::PlanResult> plan_cache_;
  std::map<std::string, mtc::core::MotionResult> motion_cache_;
  std::map<std::string, mtc::core::ToolEvidence> tool_cache_;
  std::map<std::string, mtc::core::MotionPrimitiveList> primitives_cache_;
  /// 停止请求缓存：CANCEL_PENDING 内只允许调用一次 request_stop。
  std::map<std::string, mtc::core::StopResult> stop_cache_;

  // --- 跨状态证据（供 DISENGAGE 前置联锁与 VERIFY_ATTACHED 复用）---
  std::optional<mtc::core::PoseEstimate> target_pose_;
  std::optional<mtc::core::ToolEvidence> engage_evidence_;
  std::optional<mtc::core::ToolEvidence> test_lift_evidence_;
  std::optional<mtc::core::ToolEvidence> seat_evidence_;
  std::optional<mtc::core::ToolEvidence> unload_evidence_;
  std::optional<mtc::core::ToolEvidence> unlock_evidence_;
  std::optional<mtc::core::ToolEvidence> disengage_evidence_;
  std::optional<mtc::core::MotionResult> test_lift_motion_;

  // --- 在途命令（同一状态内复用，绝不重发）---
  std::optional<mtc::core::MotionCommand> pending_command_;
  std::string pending_command_label_;
  std::map<std::string, mtc::core::MotionCommand> planned_commands_;

  static constexpr int MAX_COMMAND_ID = 1000000000;
};

}  // namespace manipulation
}  // namespace mtc

#endif  // MTC_MANIPULATION_CPP__PICK_PLACE_FSM_HPP_
