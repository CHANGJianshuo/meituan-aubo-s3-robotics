// Copyright 2026 meituan_challenge_ws contributors
//
// PickPlace FSM 实现。逐条对齐 Python
// src/mtc_manipulation/mtc_manipulation/pick_place_fsm.py 的：
//   _STATE_ACTIONS / 状态处理器 / 看门狗 / 取消 / 恢复 / 八条安全语义。

#include "mtc_manipulation_cpp/pick_place_fsm.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace mtc
{
namespace manipulation
{

// ---------------------------------------------------------------------------
// 状态处理器表
// ---------------------------------------------------------------------------

const std::map<PickPlaceState, PickPlaceFsm::Handler> & PickPlaceFsm::state_actions()
{
  static const std::map<PickPlaceState, Handler> kActions = {
    {PickPlaceState::RESOLVE_TARGET, &PickPlaceFsm::h_resolve_target},
    {PickPlaceState::PLAN_ACQUIRE, &PickPlaceFsm::h_plan_acquire},
    {PickPlaceState::MOVE_PRE_ALIGN, &PickPlaceFsm::h_move_pre_align},
    {PickPlaceState::ENGAGE, &PickPlaceFsm::h_engage},
    {PickPlaceState::VERIFY_ENGAGEMENT, &PickPlaceFsm::h_verify_engagement},
    {PickPlaceState::TEST_LIFT, &PickPlaceFsm::h_test_lift},
    {PickPlaceState::VERIFY_ATTACHED, &PickPlaceFsm::h_verify_attached},
    {PickPlaceState::LIFT, &PickPlaceFsm::h_lift},
    {PickPlaceState::TRANSPORT, &PickPlaceFsm::h_transport},
    {PickPlaceState::MOVE_PRE_SEAT, &PickPlaceFsm::h_move_pre_seat},
    {PickPlaceState::SEAT, &PickPlaceFsm::h_seat},
    {PickPlaceState::VERIFY_SEATED, &PickPlaceFsm::h_verify_seated},
    {PickPlaceState::UNLOAD, &PickPlaceFsm::h_unload},
    {PickPlaceState::DISENGAGE, &PickPlaceFsm::h_disengage},
    {PickPlaceState::RETREAT, &PickPlaceFsm::h_retreat},
    {PickPlaceState::VERIFY_PLACED, &PickPlaceFsm::h_verify_placed},
    {PickPlaceState::RECOVERY, &PickPlaceFsm::h_recovery},
    {PickPlaceState::CANCEL_PENDING, &PickPlaceFsm::h_cancel_pending},
    {PickPlaceState::SUCCESS, &PickPlaceFsm::h_success},
    {PickPlaceState::FAILED, &PickPlaceFsm::h_failed},
    {PickPlaceState::FAULT, &PickPlaceFsm::h_fault},
    {PickPlaceState::CANCELLED, &PickPlaceFsm::h_cancelled},
  };
  return kActions;
}

std::vector<std::string> PickPlaceFsm::verify_tables()
{
  std::vector<std::string> problems;
  const auto & actions = state_actions();

  for (const auto state : all_pick_place_states()) {
    if (actions.find(state) == actions.end()) {
      problems.push_back(std::string("_STATE_ACTIONS missing state: ") + to_string(state));
    }
  }
  // 运动在途状态必须都有看门狗超时，否则"状态未知"无法被及时判定。
  for (const auto state : motion_in_flight_states()) {
    if (default_state_timeouts().find(state) == default_state_timeouts().end()) {
      problems.push_back(std::string("motion-in-flight state lacks timeout: ") + to_string(state));
    }
  }
  // 流水线状态必须是全部状态的子集且不含终态。
  for (const auto state : pipeline_states()) {
    if (std::find(all_pick_place_states().begin(), all_pick_place_states().end(), state) ==
      all_pick_place_states().end())
    {
      problems.push_back(std::string("pipeline state not in all_states: ") + to_string(state));
    }
  }
  return problems;
}

// ---------------------------------------------------------------------------
// 构造与生命周期
// ---------------------------------------------------------------------------

PickPlaceFsm::PickPlaceFsm(
  PerceptionPortPtr perception,
  PlannerPortPtr planner,
  MotionPortPtr motion,
  ToolPortPtr tool,
  mtc::core::ClockPtr clock,
  ToolStrategyPtr strategy,
  PickPlaceConfig config)
: perception_(std::move(perception)),
  planner_(std::move(planner)),
  motion_(std::move(motion)),
  tool_(std::move(tool)),
  clock_(clock ? std::move(clock) : std::make_shared<mtc::core::SteadyClock>()),
  strategy_(std::move(strategy)),
  config_(std::move(config))
{
  if (config_.state_timeouts.empty()) {
    config_.state_timeouts = default_state_timeouts();
  }
  if (strategy_) {
    // 工具策略与实际工具类型必须一致，否则证据语义不可信。
    const std::string declared = strategy_->type();
    if (declared != config_.tool_type) {
      throw std::invalid_argument(
        "tool_type 配置(" + config_.tool_type + ")与策略(" + declared + ")不一致");
    }
  }
  placement_stability_sec_ = config_.placement_stability_sec;
}

void PickPlaceFsm::start(
  const std::string & object_id,
  int expected_color,
  const std::string & target_slot,
  const std::string & request_id,
  std::optional<double> placement_stability_sec)
{
  if (started_ && (terminal_states().count(state_) == 0u ||
    state_ == PickPlaceState::FAULT || !motion_resend_allowed_)) {
    throw std::logic_error("cannot restart an active or fault-locked PickPlace FSM");
  }
  static std::atomic<uint64_t> invocation{0};
  invocation_id_ = ++invocation;
  request_id_ = request_id;
  object_id_ = object_id;
  expected_color_ = expected_color;
  target_slot_ = target_slot;
  // 优先级：Goal 显式传入 > 运行时配置
  placement_stability_sec_ = placement_stability_sec.has_value() ?
    *placement_stability_sec : config_.placement_stability_sec;

  started_ = true;
  started_at_ = clock_->now();
  step_count_ = 0;
  recovery_attempts_ = 0;
  cancel_pending_ = false;
  cancel_reason_.clear();
  stop_confirmed_ = false;
  command_seq_ = 0;
  command_ids_.clear();
  primitive_calls_.clear();
  stability_started_at_.reset();
  placement_last_stamp_.reset();
  placement_last_received_at_.reset();
  stability_verified_for_s_ = 0.0;
  placement_last_position_.reset();
  pending_command_.reset();
  pending_command_label_.clear();
  object_attached_estimated_ = false;
  error_code_ = mtc::core::ErrorCodes::OK;
  message_.clear();
  resumable_error_ = false;
  last_resumable_state_.reset();
  motion_resend_allowed_ = true;
  placement_verified_ = false;
  states_history_.clear();
  target_pose_.reset();
  engage_evidence_.reset();
  test_lift_evidence_.reset();
  seat_evidence_.reset();
  unload_evidence_.reset();
  unlock_evidence_.reset();
  disengage_evidence_.reset();
  test_lift_motion_.reset();
  clear_port_results();

  if (!std::isfinite(placement_stability_sec_) || placement_stability_sec_ <= 0.0 ||
    !std::isfinite(config_.perception_max_age_s) || config_.perception_max_age_s <= 0.0) {
    error_code_ = mtc::core::ErrorCodes::INVALID_TASK;
    message_ = "stability and perception age must be finite and strictly positive";
    enter_state(PickPlaceState::FAILED);
    return;
  }
  const auto & allowed = allowed_target_slots();
  if (std::find(allowed.begin(), allowed.end(), target_slot_) == allowed.end()) {
    error_code_ = mtc::core::ErrorCodes::INVALID_TASK;
    message_ = "非法目标工位: '" + target_slot_ + "'（仅允许 T0/P1/P2/P3）";
    enter_state(PickPlaceState::FAILED);
    return;
  }

  enter_state(PickPlaceState::RESOLVE_TARGET);
  emit("goal_accepted", state_);
}

void PickPlaceFsm::request_cancel(const std::string & reason)
{
  // 真正的安全停止必须在 CANCEL_PENDING 中由后端确认。
  if (terminal_states().count(state_) == 1u) {
    return;
  }
  if (state_ == PickPlaceState::CANCEL_PENDING) {
    return;
  }
  cancel_pending_ = true;
  cancel_reason_ = reason.empty() ? "cancel requested" : reason;
  emit("cancel_requested", state_);
}

// ---------------------------------------------------------------------------
// step
// ---------------------------------------------------------------------------

StepResult PickPlaceFsm::step()
{
  step_count_ += 1;

  if (terminal_states().count(state_) == 1u) {
    StepResult result;
    result.state = state_;
    result.error_code = error_code_;
    result.message = message_;
    return result;
  }

  // 取消优先级最高：但绝不在此直接声称已停稳。
  if (cancel_pending_ && state_ != PickPlaceState::CANCEL_PENDING) {
    enter_state(PickPlaceState::CANCEL_PENDING);
  }

  const auto & actions = state_actions();
  const auto found = actions.find(state_);
  if (found == actions.end() || found->second == nullptr) {
    // 防御：未知状态 -> FAULT
    return make_fault(
      mtc::core::ErrorCodes::HARDWARE_FAULT,
      std::string("未实现的状态处理: ") + to_string(state_));
  }

  HandlerOutcome outcome;
  try {
    outcome = (this->*(found->second))();
  } catch (const std::exception & exc) {
    return make_fault(mtc::core::ErrorCodes::HARDWARE_FAULT,
      std::string("port exception; state uncertain: ") + exc.what(), false);
  } catch (...) {
    return make_fault(mtc::core::ErrorCodes::HARDWARE_FAULT, "port exception; state uncertain", false);
  }

  if (!outcome.has_value()) {
    // 端口未完成：等待，并做看门狗超时判定。
    const auto timeout = config_.state_timeouts.find(state_);
    const double elapsed = clock_->now() - state_entered_at_;
    if (timeout != config_.state_timeouts.end() && elapsed > timeout->second) {
      if (motion_in_flight_states().count(state_) == 1u) {
        // 超时后机器人是否还在动未知：禁止自动重发。
        return make_fault(
          mtc::core::ErrorCodes::MOTION_STATUS_UNKNOWN,
          std::string("状态 ") + to_string(state_) + " 超过 " +
          std::to_string(timeout->second) + "s 未返回，运动状态未知，禁止重发",
          false);
      }
      // Apply the same transition bookkeeping as a completed handler.
      // Placement silence ends verification; a loaded/unknown context cannot retry.
      const auto previous = state_;
      const bool recoverable = state_ != PickPlaceState::VERIFY_PLACED && is_resumable_context();
      const auto error = make_error(
        state_ == PickPlaceState::VERIFY_PLACED ? mtc::core::ErrorCodes::PLACEMENT_FAILED :
        mtc::core::ErrorCodes::MOTION_TIMEOUT,
        std::string("状态 ") + to_string(state_) + " 超时 " + std::to_string(timeout->second) + "s",
        recoverable);
      resumable_error_ = error.resumable;
      last_resumable_state_ = previous;
      enter_state(error.state);
      return error;
    }
    StepResult result;
    result.state = state_;
    result.error_code = error_code_;
    result.message = message_;
    return result;
  }

  StepResult result = *outcome;
  if (result.state != state_) {
    error_code_ = result.error_code;
    message_ = result.message;
    resumable_error_ = result.resumable;
    last_resumable_state_ = state_;
    enter_state(result.state);
    StepResult moved;
    moved.state = result.state;
    moved.error_code = result.error_code;
    moved.resumable = result.resumable;
    moved.message = result.message;
    return moved;
  }
  return result;
}

PickPlaceState PickPlaceFsm::step_until_terminal(int max_ticks)
{
  for (int i = 0; i < max_ticks; ++i) {
    step();
    if (terminal_states().count(state_) == 1u) {
      break;
    }
  }
  return state_;
}

PickPlaceResult PickPlaceFsm::run(int max_ticks, double tick_seconds)
{
  for (int i = 0; i < max_ticks; ++i) {
    step();
    if (terminal_states().count(state_) == 1u) {
      break;
    }
    if (tick_seconds > 0.0) {
      if (auto * manual = dynamic_cast<mtc::core::ManualClock *>(clock_.get())) {
        manual->advance(tick_seconds);
      }
    }
  }
  return result();
}

PickPlaceResult PickPlaceFsm::result() const
{
  PickPlaceResult value;
  value.success = state_ == PickPlaceState::SUCCESS;
  value.placement_verified = placement_verified_;
  value.error_code = error_code_;
  value.message = message_;
  value.final_state = state_;
  value.states_history = states_history_;
  return value;
}

PickPlaceFeedback PickPlaceFsm::feedback() const
{
  PickPlaceFeedback value;
  value.substate = to_string(state_);
  value.progress = progress();
  value.object_attached_estimated = object_attached_estimated_;
  value.detail = message_;
  return value;
}

// ---------------------------------------------------------------------------
// 内部：状态与错误
// ---------------------------------------------------------------------------

float PickPlaceFsm::progress() const
{
  if (state_ == PickPlaceState::SUCCESS) {
    return 1.0f;
  }
  const auto & pipeline = pipeline_states();
  if (pipeline.empty()) {
    return 0.0f;
  }
  const double total = static_cast<double>(pipeline.size() - 1u);
  const auto it = std::find(pipeline.begin(), pipeline.end(), state_);
  if (it == pipeline.end()) {
    return 0.0f;
  }
  const double index = static_cast<double>(std::distance(pipeline.begin(), it));
  return static_cast<float>(std::min(1.0, std::max(0.0, index / total)));
}

void PickPlaceFsm::enter_state(PickPlaceState state)
{
  state_ = state;
  state_entered_at_ = clock_->now();
  // 离开旧状态即释放"在途命令"引用：再次进入同一状态必须生成新的 command_id，
  // 而在同一次状态访问内（端口未返回时）必须复用同一条命令，绝不重发。
  pending_command_.reset();
  pending_command_label_.clear();
  states_history_.push_back(state);
  emit("state_entered", state);
}

void PickPlaceFsm::emit(const std::string & event, PickPlaceState state)
{
  if (!on_event) {
    return;
  }
  try {
    on_event(event, state);
  } catch (...) {
    // 回调异常不得破坏状态机时序
  }
}

std::string PickPlaceFsm::next_command_id(const std::string & label)
{
  command_seq_ += 1;
  const std::string prefix = request_id_.empty() ? "req" : request_id_;
  std::ostringstream stream;
  stream << prefix << "#" << invocation_id_ << ":" << label << "-"
         << std::setfill('0') << std::setw(3) << command_seq_;
  const std::string command_id = stream.str();
  command_ids_.push_back(command_id);
  return command_id;
}

void PickPlaceFsm::set_placement_verified()
{
  // 安全语义 1 的唯一写入口。
  if (state_ != PickPlaceState::VERIFY_PLACED) {
    throw std::logic_error(
      std::string("placement_verified 只允许由 VERIFY_PLACED 设置，当前状态: ") +
      to_string(state_));
  }
  placement_verified_ = true;
}

StepResult PickPlaceFsm::make_error(
  uint16_t code, const std::string & message, bool resumable,
  std::optional<PickPlaceState> next_state)
{
  if (!next_state.has_value()) {
    next_state = resumable ? PickPlaceState::RECOVERY : PickPlaceState::FAILED;
  }
  error_code_ = code;
  message_ = message;
  emit("error", state_);

  StepResult result;
  result.state = *next_state;
  result.error_code = code;
  result.resumable = resumable;
  result.message = message;
  return result;
}

StepResult PickPlaceFsm::make_fault(
  uint16_t code, const std::string & message, bool resend_allowed)
{
  // 进入 FAULT。resend_allowed=false 用于运动状态未知，禁止自动重发。
  if (!resend_allowed) {
    motion_resend_allowed_ = false;
  }
  error_code_ = code;
  message_ = message;
  emit("fault", state_);
  // 关键：必须真正切换到 FAULT。否则 handler 直接返回 make_fault(...) 时，
  // step() 会认为"状态未变化"而继续推进，导致锁定失效。
  enter_state(PickPlaceState::FAULT);

  StepResult result;
  result.state = PickPlaceState::FAULT;
  result.error_code = code;
  result.resumable = false;
  result.message = message;
  return result;
}

StepResult PickPlaceFsm::force_fault(
  uint16_t code, const std::string & message, bool resend_allowed)
{
  if (terminal_states().count(state_) == 1u) {
    StepResult result;
    result.state = state_;
    result.error_code = error_code_;
    result.message = message_;
    return result;
  }
  return make_fault(code, message, resend_allowed);
}

bool PickPlaceFsm::is_resumable_context()
{
  // 是否允许退回重试：必须确认当前无载荷且运动状态已知。
  if (!motion_resend_allowed_) {
    return false;
  }
  std::string holding;
  try {
    holding = tool_->holding_state();
  } catch (...) {
    // holding_state 查询失败，保守判定为不明确。
    return false;
  }
  if (holding != "none") {
    return false;
  }
  return true;
}

void PickPlaceFsm::clear_port_results()
{
  detection_cache_.clear();
  plan_cache_.clear();
  motion_cache_.clear();
  tool_cache_.clear();
  primitives_cache_.clear();
  stop_cache_.clear();
  pending_command_.reset();
  pending_command_label_.clear();
  planned_commands_.clear();
}

std::string PickPlaceFsm::make_cache_key(const std::string & label) const
{
  return std::string(to_string(state_)) + ":" + label;
}

const mtc::core::MotionPrimitiveList & PickPlaceFsm::primitives(
  const std::string & key, const std::function<mtc::core::MotionPrimitiveList()> & factory)
{
  const std::string cache_key = std::string("__primitives__:") + key;
  const auto found = primitives_cache_.find(cache_key);
  if (found != primitives_cache_.end()) {
    return found->second;
  }
  const auto inserted = primitives_cache_.emplace(cache_key, factory());
  return inserted.first->second;
}

mtc::core::MotionCommand PickPlaceFsm::cached_command(
  const std::string & label, const mtc::core::MotionPrimitive & primitive)
{
  // 构造并缓存运动命令：同一状态内的多次 step 必须复用同一条命令。
  //
  // 安全语义：运动在途时（端口返回 nullopt）绝不允许重新发令。
  const std::string key = std::string(to_string(state_)) + ":" + label;
  if (pending_command_.has_value() && pending_command_label_ == key) {
    return *pending_command_;
  }
  mtc::core::MotionCommand command = make_command(label, primitive);
  pending_command_ = command;
  pending_command_label_ = key;
  return command;
}

mtc::core::MotionCommand PickPlaceFsm::make_command(
  const std::string & label, const mtc::core::MotionPrimitive & primitive)
{
  const auto found = planned_commands_.find(primitive.name);
  if (found == planned_commands_.end()) {
    throw std::logic_error("missing executable planner output: " + primitive.name);
  }
  mtc::core::MotionCommand command = found->second;
  command.command_id = next_command_id(label);
  command.primitive_name = primitive.name;
  command.requires_contact = primitive.requires_contact;
  command.intent = primitive;
  if (command.trajectory) {command.trajectory->trajectory_id = command.command_id;}

  return command;
}

bool PickPlaceFsm::accept_plan(const mtc::core::PlanResult & plan,
  const mtc::core::MotionPrimitiveList & requested)
{
  std::map<std::string, mtc::core::MotionCommand> accepted;
  if (!plan.ok || requested.empty() || plan.commands.size() != requested.size()) {return false;}
  for (const auto & primitive : requested) {
    if (!primitive.is_usable()) {return false;}
    const auto found = std::find_if(plan.commands.begin(), plan.commands.end(),
      [&primitive](const auto & c) {return c.primitive_name == primitive.name;});
    if (found == plan.commands.end() || accepted.count(primitive.name)) {return false;}
    auto command = *found;
    if (!std::isfinite(command.velocity_scaling) || command.velocity_scaling <= 0.0 ||
      command.velocity_scaling > config_.default_velocity_scaling ||
      !std::isfinite(command.acceleration_scaling) || command.acceleration_scaling <= 0.0 ||
      command.acceleration_scaling > config_.default_acceleration_scaling || command.timeout_ms == 0)
    {return false;}
    if (command.trajectory) {
      if (!command.target_rad.empty() || !command.trajectory->shape_is_valid() ||
        command.trajectory->joint_names != config_.joint_names ||
        command.trajectory->header_frame_id.empty() ||
        command.trajectory->duration_s() * 1000.0 > command.timeout_ms) {return false;}
    } else if (!command.shape_is_valid() || command.joint_names != config_.joint_names) {
      return false;
    }
    command.intent = primitive;
    command.requires_contact = primitive.requires_contact;
    accepted.emplace(primitive.name, command);
  }
  for (const auto & entry : accepted) {planned_commands_[entry.first] = entry.second;}
  return true;
}

std::optional<mtc::core::MotionResult> PickPlaceFsm::execute_motion(
  const mtc::core::MotionCommand & command)
{
  return command.trajectory ? motion_->execute_trajectory(command) : motion_->execute_joint_move(command);
}

std::optional<StepResult> PickPlaceFsm::motion_failure(const mtc::core::MotionResult & result)
{
  // 把运动后端结果映射为 FSM 转移；返回 nullopt 表示成功到位。
  if (result.status_unknown) {
    // 安全语义 7：运动状态未知 -> FAULT 且禁止自动重发。
    motion_resend_allowed_ = false;
    return make_fault(
      mtc::core::ErrorCodes::MOTION_STATUS_UNKNOWN,
      result.message.empty() ? "运动状态未知（反馈断流/结果不确定），禁止自动重发" : result.message,
      false);
  }
  if (result.timed_out) {
    motion_resend_allowed_ = false;
    return make_fault(
      mtc::core::ErrorCodes::MOTION_TIMEOUT,
      result.message.empty() ? "运动指令超时，机器人可能仍在运动，禁止自动重发" : result.message,
      false);
  }
  if (!motion_resend_allowed_ && !result.ok()) {
    return make_fault(
      result.error_code != 0 ? result.error_code : mtc::core::ErrorCodes::MOTION_STATUS_UNKNOWN,
      "运动结果不可用且状态未知，禁止重发：" + result.message,
      false);
  }
  if (result.ok()) {
    return std::nullopt;
  }
  return make_error(
    result.error_code != 0 ? result.error_code : mtc::core::ErrorCodes::MOTION_TIMEOUT,
    result.message.empty() ? "运动未到位" : result.message,
    is_resumable_context());
}

// ---------------------------------------------------------------------------
// 状态处理器
// ---------------------------------------------------------------------------

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_resolve_target()
{
  // 安全语义：感知不可靠（颜色不符 / 过期 / 不唯一）时绝不运动。
  mtc::core::DetectionResult detection;
  if (!resolve_cached(
      detection_cache_, "detection",
      [this]() {
        return perception_->get_detection(
          object_id_, std::optional<int>(expected_color_), config_.perception_max_age_s);
      },
      detection))
  {
    return std::nullopt;
  }

  if (!detection.found) {
    return make_error(
      mtc::core::ErrorCodes::TARGET_NOT_FOUND,
      detection.message.empty() ? ("未找到目标对象 " + object_id_) : detection.message, true);
  }
  if (!detection.unique) {
    return make_error(
      mtc::core::ErrorCodes::TARGET_NOT_FOUND,
      detection.message.empty() ? "目标颜色/ID 不唯一" : detection.message, true);
  }
  if (detection.stale) {
    return make_error(
      mtc::core::ErrorCodes::POSE_STALE,
      detection.message.empty() ? "感知/TF 数据过期" : detection.message, true);
  }
  if (!detection.pose.has_value() || !detection.pose->has_ring_pose()) {
    return make_error(
      mtc::core::ErrorCodes::TARGET_NOT_FOUND,
      "提环位姿不可用（ring_pose_valid=false）", true);
  }

  const mtc::core::PoseEstimate & pose = *detection.pose;
  // 安全语义（设计 V0.1 §4.4）：expected_color 是防抓错的**双重校验**。
  // FSM 必须独立比对感知颜色，不能只依赖端口内部校验；
  // 颜色不匹配或颜色未知（COLOR_UNKNOWN=0）一律拒绝运动。
  if (expected_color_ != 0) {
    const int observed_color = pose.color;
    if (observed_color == 0) {
      return make_error(
        mtc::core::ErrorCodes::TARGET_NOT_FOUND,
        "目标颜色未知（COLOR_UNKNOWN），拒绝在颜色未确认时运动；object_id=" + object_id_,
        true);
    }
    if (observed_color != expected_color_) {
      return make_error(
        mtc::core::ErrorCodes::TARGET_NOT_FOUND,
        "目标颜色不匹配：期望 color=" + std::to_string(expected_color_) +
        "，感知 color=" + std::to_string(observed_color) +
        "（拒绝抓错颜色）；object_id=" + object_id_,
        true);
    }
  }
  target_pose_ = pose;

  StepResult result;
  result.state = PickPlaceState::PLAN_ACQUIRE;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_plan_acquire()
{
  // 规划接近段 + 接合段 + 试提段原语。
  //
  // 注意：这里刻意"预热"三段原语缓存（与 Python 逐行一致），
  // 因此 engage / test_lift 的结果在本状态内只被缓存、不被使用。
  const auto & acquire = primitives(
    "acquire", [this]() {return strategy_->make_acquire_plan(*target_pose_);});
  const auto & engage = primitives(
    "engage", [this]() {return strategy_->make_engage_plan(*target_pose_);});
  const auto & test_lift = primitives(
    "test_lift", [this]() {return strategy_->make_test_lift_plan(*target_pose_);});
  (void)engage;
  (void)test_lift;

  mtc::core::PlanResult plan;
  if (!resolve_cached(
      plan_cache_, "plan_acquire",
      [this, &acquire]() {
        PlanTarget target;
        target.phase = "ACQUIRE";
        target.object_id = object_id_;
        target.slot = target_slot_;
        if (target_pose_.has_value() && target_pose_->has_ring_pose()) {
          target.has_ring_pose = true;
          target.ring_pose = target_pose_->ring_pose;
        }
        target.primitives = acquire;
        const auto extra = strategy_->make_test_lift_plan(*target_pose_);
        target.primitives.insert(target.primitives.end(), extra.begin(), extra.end());
        target.primitive_names = primitive_names(target.primitives);
        return planner_->plan(target);
      },
      plan))
  {
    return std::nullopt;
  }

  if (!plan.ok) {
    return make_error(
      plan.error_code != 0 ? plan.error_code : mtc::core::ErrorCodes::PLANNING_FAILED,
      plan.message.empty() ? "接近段规划失败" : plan.message, true);
  }
  auto required = acquire;
  required.insert(required.end(), test_lift.begin(), test_lift.end());
  if (!accept_plan(plan, required)) {
    return make_error(mtc::core::ErrorCodes::PLANNING_FAILED,
      "planner reported success without complete executable output", is_resumable_context());
  }
  StepResult result;
  result.state = PickPlaceState::MOVE_PRE_ALIGN;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_move_pre_align()
{
  const auto & primitives_list = primitives(
    "acquire", [this]() {return strategy_->make_acquire_plan(*target_pose_);});
  if (primitives_list.empty()) {
    return make_error(mtc::core::ErrorCodes::PLANNING_FAILED, "接近段无运动原语", true);
  }
  const auto command = cached_command("pre_align", primitives_list.front());
  primitive_calls_.push_back(primitives_list.front().name);

  mtc::core::MotionResult motion;
  if (!resolve_cached(
      motion_cache_, "motion",
      [this, &command]() {return execute_motion(command);}, motion))
  {
    return std::nullopt;
  }
  if (auto failure = motion_failure(motion); failure.has_value()) {
    return failure;
  }
  StepResult result;
  result.state = PickPlaceState::ENGAGE;
  return result;
}

// ---- V1/V2 差异区（一）：ENGAGE 与 VERIFY_ENGAGEMENT -----------------------

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_engage()
{
  const auto & primitives_list = primitives(
    "engage", [this]() {return strategy_->make_engage_plan(*target_pose_);});
  for (const auto & primitive : primitives_list) {
    primitive_calls_.push_back(primitive.name);
  }

  mtc::core::ToolEvidence evidence;
  if (!resolve_cached(
      tool_cache_, "engage",
      [this, &primitives_list]() {return tool_->engage("ENGAGE", primitives_list);}, evidence))
  {
    return std::nullopt;
  }
  engage_evidence_ = evidence;

  StepResult result;
  result.state = PickPlaceState::VERIFY_ENGAGEMENT;
  result.error_code = evidence.error_code;
  result.message = evidence.message;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_verify_engagement()
{
  const auto evidence = engage_evidence_.value_or(mtc::core::ToolEvidence{});
  if (!evidence.ok) {
    // 安全语义 3：ENGAGE 失败 -> 300，只有确认无载荷才允许退回重试。
    return make_error(
      evidence.error_code != 0 ? evidence.error_code : mtc::core::ErrorCodes::ENGAGE_FAILED,
      evidence.message.empty() ? "接合（穿环/锁止）未完成" : evidence.message,
      is_resumable_context());
  }
  if (evidence.evidence_level < mtc::core::EvidenceLevel::GEOMETRY) {
    // 仅有指令级证据：不得据此认定接合成立。
    return make_error(
      mtc::core::ErrorCodes::ENGAGE_FAILED,
      "接合证据等级不足（evidence_level=" + std::to_string(evidence.evidence_level) + "）",
      is_resumable_context());
  }
  if (evidence.state != mtc::core::ToolStateCode::STATE_ATTACHED) {
    return make_fault(
      mtc::core::ErrorCodes::HARDWARE_FAULT,
      "接合后工具状态异常: " + std::to_string(evidence.state));
  }
  StepResult result;
  result.state = PickPlaceState::TEST_LIFT;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_test_lift()
{
  const auto & primitives_list = primitives(
    "test_lift", [this]() {return strategy_->make_test_lift_plan(*target_pose_);});
  if (primitives_list.empty()) {
    return make_error(mtc::core::ErrorCodes::PLANNING_FAILED, "试提段无运动原语", false);
  }

  if (config_.verify_test_lift) {
    mtc::core::ToolEvidence evidence;
    if (!resolve_cached(
        tool_cache_, "test_lift",
        [this, &primitives_list]() {return tool_->test_lift(primitives_list);}, evidence))
    {
      return std::nullopt;
    }
    test_lift_evidence_ = evidence;
    if (!evidence.ok) {
      return make_error(
        evidence.error_code != 0 ? evidence.error_code : mtc::core::ErrorCodes::ATTACH_NOT_VERIFIED,
        evidence.message.empty() ? "试提失败" : evidence.message,
        is_resumable_context());
    }
  }

  const auto command = cached_command("test_lift", primitives_list.front());
  primitive_calls_.push_back(primitives_list.front().name);

  mtc::core::MotionResult motion;
  if (!resolve_cached(
      motion_cache_, "motion",
      [this, &command]() {return execute_motion(command);}, motion))
  {
    return std::nullopt;
  }
  if (auto failure = motion_failure(motion); failure.has_value()) {
    return failure;
  }
  test_lift_motion_ = motion;

  StepResult result;
  result.state = PickPlaceState::VERIFY_ATTACHED;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_verify_attached()
{
  // 安全语义 2：必须有几何级（或更高）挂载证据，否则 310，禁止运输。
  mtc::core::ToolEvidence evidence;
  if (test_lift_evidence_.has_value()) {
    evidence = *test_lift_evidence_;
  } else {
    const auto & primitives_list = primitives(
      "test_lift", [this]() {return strategy_->make_test_lift_plan(*target_pose_);});
    if (!resolve_cached(
        tool_cache_, "test_lift",
        [this, &primitives_list]() {return tool_->test_lift(primitives_list);}, evidence))
    {
      return std::nullopt;
    }
    test_lift_evidence_ = evidence;
  }

  bool vision_follow = false;
  if (evidence.evidence_level < mtc::core::EvidenceLevel::GEOMETRY) {
    // 退一步：尝试用"视觉随动确认"补足独立证据。
    mtc::core::DetectionResult detection;
    if (!resolve_cached(
        detection_cache_, "follow_detection",
        [this]() {
          return perception_->get_detection(
            object_id_, std::optional<int>(expected_color_), config_.perception_max_age_s);
        },
        detection))
    {
      return std::nullopt;
    }
    vision_follow = detection.ok() && detection.pose.has_value() && detection.pose->followed;
  }

  if (!(evidence.has_geometry_evidence() || vision_follow)) {
    object_attached_estimated_ = false;
    return make_error(
      mtc::core::ErrorCodes::ATTACH_NOT_VERIFIED,
      "缺少挂载证据（evidence_level=" + std::to_string(evidence.evidence_level) +
      ", vision_follow=" + (vision_follow ? "true" : "false") +
      "），禁止进入 LIFT/TRANSPORT",
      is_resumable_context());
  }

  object_attached_estimated_ = true;
  emit("object_attached_verified", state_);
  StepResult result;
  result.state = PickPlaceState::LIFT;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_lift()
{
  const auto & primitives_list = primitives(
    "lift", [this]() {return strategy_->make_lift_plan(*target_pose_);});

  mtc::core::PlanResult plan;
  if (!resolve_cached(
      plan_cache_, "plan_transport",
      [this, &primitives_list]() {
        PlanTarget target;
        target.phase = "TRANSPORT";
        target.object_id = object_id_;
        target.slot = target_slot_;
        target.has_ring_pose = true;
        target.ring_pose = target_pose_->ring_pose;
        target.primitives = primitives_list;
        for (const auto & extra : {
            strategy_->make_transport_plan(*target_pose_, target_slot_),
            strategy_->make_seat_plan(target_slot_),
            strategy_->make_seat_contact_plan(target_slot_),
            strategy_->make_retreat_plan(target_slot_)}) {
          target.primitives.insert(target.primitives.end(), extra.begin(), extra.end());
        }
        target.primitive_names = primitive_names(target.primitives);
        return planner_->plan(target);
      },
      plan))
  {
    return std::nullopt;
  }

  if (!plan.ok) {
    return make_fault(
      plan.error_code != 0 ? plan.error_code : mtc::core::ErrorCodes::PLANNING_FAILED,
      plan.message.empty() ? "已挂载物体后的搬运规划失败" : plan.message);
  }
  if (primitives_list.empty()) {
    return make_fault(mtc::core::ErrorCodes::PLANNING_FAILED, "提升段无运动原语");
  }

  auto required = primitives_list;
  for (const auto & extra : {
      strategy_->make_transport_plan(*target_pose_, target_slot_),
      strategy_->make_seat_plan(target_slot_), strategy_->make_seat_contact_plan(target_slot_),
      strategy_->make_retreat_plan(target_slot_)}) {
    required.insert(required.end(), extra.begin(), extra.end());
  }
  if (!accept_plan(plan, required)) {
    return make_fault(mtc::core::ErrorCodes::PLANNING_FAILED,
      "missing executable transport/release output", false);
  }
  const auto command = cached_command("lift", primitives_list.front());
  primitive_calls_.push_back(primitives_list.front().name);

  mtc::core::MotionResult motion;
  if (!resolve_cached(
      motion_cache_, "motion",
      [this, &command]() {return execute_motion(command);}, motion))
  {
    return std::nullopt;
  }
  if (auto failure = motion_failure(motion); failure.has_value()) {
    return failure;
  }
  StepResult result;
  result.state = PickPlaceState::TRANSPORT;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_transport()
{
  const auto & primitives_list = primitives(
    "transport",
    [this]() {return strategy_->make_transport_plan(*target_pose_, target_slot_);});
  if (primitives_list.empty()) {
    return make_fault(mtc::core::ErrorCodes::PLANNING_FAILED, "搬运段无运动原语");
  }
  const auto command = cached_command("transport", primitives_list.front());
  primitive_calls_.push_back(primitives_list.front().name);

  mtc::core::MotionResult motion;
  if (!resolve_cached(
      motion_cache_, "motion",
      [this, &command]() {return execute_motion(command);}, motion))
  {
    return std::nullopt;
  }
  if (auto failure = motion_failure(motion); failure.has_value()) {
    return failure;
  }
  StepResult result;
  result.state = PickPlaceState::MOVE_PRE_SEAT;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_move_pre_seat()
{
  const auto & primitives_list = primitives(
    "seat", [this]() {return strategy_->make_seat_plan(target_slot_);});
  if (primitives_list.empty()) {
    return make_fault(mtc::core::ErrorCodes::PLANNING_FAILED, "落座段无运动原语");
  }
  const auto command = cached_command("pre_seat", primitives_list.front());
  primitive_calls_.push_back(primitives_list.front().name);

  mtc::core::MotionResult motion;
  if (!resolve_cached(
      motion_cache_, "motion",
      [this, &command]() {return execute_motion(command);}, motion))
  {
    return std::nullopt;
  }
  if (auto failure = motion_failure(motion); failure.has_value()) {
    return failure;
  }
  StepResult result;
  result.state = PickPlaceState::SEAT;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_seat()
{
  const auto & primitives_list = primitives(
    "seat_contact", [this]() {return strategy_->make_seat_contact_plan(target_slot_);});
  if (primitives_list.empty()) {
    return make_fault(mtc::core::ErrorCodes::PLANNING_FAILED, "接触下降段无运动原语");
  }
  const auto command = cached_command("seat", primitives_list.front());
  primitive_calls_.push_back(primitives_list.front().name);

  mtc::core::MotionResult motion;
  if (!resolve_cached(
      motion_cache_, "motion",
      [this, &command]() {return execute_motion(command);}, motion))
  {
    return std::nullopt;
  }
  if (auto failure = motion_failure(motion); failure.has_value()) {
    return failure;
  }
  StepResult result;
  result.state = PickPlaceState::VERIFY_SEATED;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_verify_seated()
{
  // 安全语义 4：未确认桌面承托 -> 400，禁止进入 UNLOAD/DISENGAGE。
  mtc::core::ToolEvidence evidence;
  if (!resolve_cached(
      tool_cache_, "seat_verify", [this]() {return std::optional<mtc::core::ToolEvidence>(
        tool_->seat_verify());}, evidence))
  {
    return std::nullopt;
  }
  seat_evidence_ = evidence;

  if (!(evidence.ok && evidence.seated)) {
    return make_fault(
      evidence.error_code != 0 ? evidence.error_code : mtc::core::ErrorCodes::SEAT_NOT_VERIFIED,
      evidence.message.empty() ?
      ("未确认电池由桌面承载（seated=" + std::string(evidence.seated ? "true" : "false") +
      "），禁止释放/解锁") : evidence.message);
  }
  StepResult result;
  result.state = PickPlaceState::UNLOAD;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_unload()
{
  const auto & primitives_list = primitives(
    "release", [this]() {return strategy_->make_release_plan(target_slot_);});
  for (const auto & primitive : primitives_list) {
    primitive_calls_.push_back(primitive.name);
  }

  mtc::core::ToolEvidence evidence;
  if (!resolve_cached(
      tool_cache_, "unload",
      [this, &primitives_list]() {return tool_->unload(primitives_list);}, evidence))
  {
    return std::nullopt;
  }
  unload_evidence_ = evidence;

  if (!(evidence.ok && evidence.unloaded)) {
    const uint16_t code = evidence.error_code != 0 ?
      evidence.error_code : mtc::core::ErrorCodes::RELEASE_NOT_VERIFIED;
    if (code == mtc::core::ErrorCodes::RELEASE_NOT_VERIFIED ||
      code == mtc::core::ErrorCodes::UNLOCK_FAILED)
    {
      return make_fault(
        code, evidence.message.empty() ? "卸载（释放主要重量）未确认" : evidence.message);
    }
    return make_error(
      code, evidence.message.empty() ? "卸载未确认" : evidence.message,
      is_resumable_context());
  }
  StepResult result;
  result.state = PickPlaceState::DISENGAGE;
  return result;
}

// ---- V1/V2 差异区（二）：DISENGAGE ----------------------------------------

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_disengage()
{
  // 安全语义 5：V2 解锁失败/结果未知 -> 410/420 -> FAULT，禁止硬拉上抬。
  //
  // 防御性联锁：到达 DISENGAGE 时"已确认落座 + 已卸载"的证据必须仍然成立。
  // 该检查不依赖状态流转顺序，确保未来改动无法绕过设计 §7.2 的禁止行为。
  const bool seated = seat_evidence_.has_value() && seat_evidence_->seated;
  const bool unloaded = unload_evidence_.has_value() && unload_evidence_->unloaded;
  const bool needs_unlock = strategy_->requires_unlock_pulse();

  if (needs_unlock) {
    if (!seated) {
      return make_fault(
        mtc::core::ErrorCodes::SEAT_NOT_VERIFIED,
        "DISENGAGE 前置联锁失败：未确认落座，禁止解锁/退出");
    }
    if (!unloaded) {
      return make_fault(
        mtc::core::ErrorCodes::RELEASE_NOT_VERIFIED,
        "DISENGAGE 前置联锁失败：未确认卸载，禁止解锁/退出");
    }
  }

  const auto & primitives_list = primitives(
    "disengage", [this]() {return strategy_->make_disengage_plan(target_slot_);});
  for (const auto & primitive : primitives_list) {
    primitive_calls_.push_back(primitive.name);
  }

  if (needs_unlock) {
    const UnlockDecision decision = strategy_->evaluate_unlock(seated, unloaded);
    if (!decision.allowed) {
      // 前置条件不满足：不得调用解锁 IO，直接保守故障。
      return make_fault(decision.error_code, "拒绝解锁：" + decision.reason);
    }

    mtc::core::ToolEvidence unlock;
    if (!resolve_cached(
        tool_cache_, "unlock",
        [this, &decision]() {return tool_->unlock(decision.pulse_ms);}, unlock))
    {
      return std::nullopt;
    }
    unlock_evidence_ = unlock;
    if (!unlock.ok) {
      return make_fault(
        unlock.error_code != 0 ? unlock.error_code : mtc::core::ErrorCodes::UNLOCK_FAILED,
        unlock.message.empty() ? "电磁解锁失败" : unlock.message);
    }
    if (!unlock.released.value_or(false)) {
      // 解锁脉冲被受理但锁止未解除：不得撤离（设计 §10.1）。
      return make_fault(
        mtc::core::ErrorCodes::UNLOCK_FAILED,
        unlock.message.empty() ? "解锁请求已受理但锁止状态未解除" : unlock.message);
    }
  }

  mtc::core::ToolEvidence evidence;
  if (!resolve_cached(
      tool_cache_, "disengage",
      [this, &primitives_list]() {return tool_->disengage(primitives_list);}, evidence))
  {
    return std::nullopt;
  }
  disengage_evidence_ = evidence;

  if (!(evidence.ok && evidence.released.value_or(false))) {
    return make_fault(
      evidence.error_code != 0 ? evidence.error_code : mtc::core::ErrorCodes::RELEASE_NOT_VERIFIED,
      evidence.message.empty() ?
      "机构分离（退钩/退出）结果未知，禁止撤离与上抬" : evidence.message);
  }
  StepResult result;
  result.state = PickPlaceState::RETREAT;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_retreat()
{
  const auto & primitives_list = primitives(
    "retreat", [this]() {return strategy_->make_retreat_plan(target_slot_);});
  if (primitives_list.empty()) {
    return make_fault(mtc::core::ErrorCodes::PLANNING_FAILED, "撤离段无运动原语");
  }
  const auto command = cached_command("retreat", primitives_list.front());
  primitive_calls_.push_back(primitives_list.front().name);

  mtc::core::MotionResult motion;
  if (!resolve_cached(
      motion_cache_, "motion",
      [this, &command]() {return execute_motion(command);}, motion))
  {
    return std::nullopt;
  }
  if (auto failure = motion_failure(motion); failure.has_value()) {
    return failure;
  }
  StepResult result;
  result.state = PickPlaceState::VERIFY_PLACED;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_verify_placed()
{
  // 安全语义 8：目标区域 + 连续稳定 placement_stability_sec 才 SUCCESS。
  if (!config_.verify_placement) {
    return make_error(
      mtc::core::ErrorCodes::PLACEMENT_FAILED,
      "placement 验证被配置关闭，不得声明放置成功");
  }

  const double now = clock_->now();
  const auto reset_window = [this]() {
      stability_started_at_.reset();
      stability_verified_for_s_ = 0.0;
    };
  // Poll pending perception; completed observations are never put in a result cache.
  const auto observed = perception_->get_detection(object_id_, std::nullopt,
    config_.perception_max_age_s);
  if (!observed) {
    if (placement_last_received_at_ &&
      now - *placement_last_received_at_ > config_.perception_max_age_s) {reset_window();}
    return placement_wait_or_fail("waiting for a new placement observation");
  }
  const auto & detection = *observed;
  const bool usable = detection.ok() && detection.pose->object_id == object_id_ &&
    detection.pose->has_body_pose() && std::isfinite(detection.pose->stamp_s) &&
    detection.pose->stamp_s >= 0.0 && detection.pose->stamp_s <= now &&
    now - detection.pose->stamp_s <= config_.perception_max_age_s &&
    detection.pose->body_pose.stamp_s <= now &&
    now - detection.pose->body_pose.stamp_s <= config_.perception_max_age_s &&
    detection.pose->body_pose.stamp_s == detection.pose->stamp_s;
  if (!usable) {
    reset_window();
    return placement_wait_or_fail("invalid or stale placement observation");
  }
  const double stamp = detection.pose->stamp_s;
  if (placement_last_stamp_ && stamp <= *placement_last_stamp_) {
    if (stamp < *placement_last_stamp_ || !detection.pose->at_target ||
      !detection.pose->settled || !placement_last_received_at_ ||
      now - *placement_last_received_at_ > config_.perception_max_age_s) {reset_window();}
    return placement_wait_or_fail("duplicate or backwards placement timestamp");
  }
  // Gaps longer than the allowed perception age break continuity, even with a new frame.
  if (placement_last_stamp_ && stamp - *placement_last_stamp_ > config_.perception_max_age_s) {
    reset_window();
  }
  placement_last_stamp_ = stamp;
  placement_last_received_at_ = now;
  if (!detection.pose->at_target || !detection.pose->settled) {
    reset_window();
    return placement_wait_or_fail("placement target lost or moving");
  }
  if (!stability_started_at_) {stability_started_at_ = stamp;}
  stability_verified_for_s_ = stamp - *stability_started_at_;
  const double time_tolerance = std::min(1e-9, placement_stability_sec_ * 1e-9);
  if (stability_verified_for_s_ > 0.0 &&
    stability_verified_for_s_ + time_tolerance >= placement_stability_sec_) {
    set_placement_verified();
    StepResult result;
    result.state = PickPlaceState::SUCCESS;
    return result;
  }
  return placement_wait_or_fail("放置后稳定性累计 " +
    std::to_string(stability_verified_for_s_) + "s");
}

std::optional<StepResult> PickPlaceFsm::placement_wait_or_fail(const std::string & detail)
{
  // 放置核验窗口内继续观察；窗口耗尽仍未稳定 -> PLACEMENT_FAILED=430 -> FAILED。
  const auto found = config_.state_timeouts.find(PickPlaceState::VERIFY_PLACED);
  const double timeout = (found != config_.state_timeouts.end()) ? found->second : 30.0;
  if (clock_->now() - state_entered_at_ > timeout) {
    return make_error(
      mtc::core::ErrorCodes::PLACEMENT_FAILED, detail + "（核验窗口耗尽）");
  }
  return std::nullopt;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_recovery()
{
  // 仅对"确认无载荷且运动状态已知"的可恢复错误重试。
  recovery_attempts_ += 1;
  const bool can_retry =
    recovery_attempts_ <= config_.recovery_max_attempts &&
    resumable_error_ &&
    is_resumable_context();

  if (!can_retry) {
    return make_error(
      error_code_,
      "恢复失败（attempt=" + std::to_string(recovery_attempts_) + "/" +
      std::to_string(config_.recovery_max_attempts) +
      ", resumable=" + (resumable_error_ ? "true" : "false") + "）：" + message_,
      false);
  }
  clear_port_results();
  StepResult result;
  result.state = PickPlaceState::RESOLVE_TARGET;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_success()
{
  StepResult result;
  result.state = PickPlaceState::SUCCESS;
  result.error_code = error_code_;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_failed()
{
  StepResult result;
  result.state = PickPlaceState::FAILED;
  result.error_code = error_code_;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_fault()
{
  StepResult result;
  result.state = PickPlaceState::FAULT;
  result.error_code = error_code_;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_cancelled()
{
  StepResult result;
  result.state = PickPlaceState::CANCELLED;
  result.error_code = error_code_;
  return result;
}

PickPlaceFsm::HandlerOutcome PickPlaceFsm::h_cancel_pending()
{
  // 安全语义 6：必须等 MotionPort 确认停稳；未确认一律 FAULT(520)。
  //
  // 停止请求在 CANCEL_PENDING 内只允许提交一次（缓存保证），
  // 但**每次 step 都重新读取结论**，避免把"未确认"缓存成"已停稳"。
  const std::string key = make_cache_key("request_stop");
  const auto found = stop_cache_.find(key);
  mtc::core::StopResult stop_result;
  if (found != stop_cache_.end()) {
    stop_result = found->second;
  } else {
    stop_result = motion_->request_stop(
      cancel_reason_.empty() ? "cancel requested" : cancel_reason_);
    stop_cache_.emplace(key, stop_result);
  }

  stop_confirmed_ = stop_result.stop_confirmed;
  if (!stop_result.stop_confirmed) {
    return make_fault(
      mtc::core::ErrorCodes::CANCEL_NOT_CONFIRMED,
      "取消后未确认停稳（request_delivered=" +
      std::string(stop_result.request_delivered ? "true" : "false") + "）：" +
      stop_result.message,
      false);
  }

  error_code_ = mtc::core::ErrorCodes::OK;
  message_ = "取消已确认停稳（request_delivered=" +
    std::string(stop_result.request_delivered ? "true" : "false") + "）";
  emit("stop_confirmed", state_);
  StepResult result;
  result.state = PickPlaceState::CANCELLED;
  return result;
}

}  // namespace manipulation
}  // namespace mtc
