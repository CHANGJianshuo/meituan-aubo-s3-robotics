// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_moveit_planner/moveit_planner_port.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <utility>

#include "mtc_core_cpp/error_codes.hpp"
#include "mtc_moveit_planner/pose_math.hpp"
#include "mtc_moveit_planner/start_consistency.hpp"
#include "mtc_moveit_planner/trajectory_conversion.hpp"

namespace mtc
{
namespace moveit_planner
{

namespace
{

using mtc::core::ErrorCodes;
using mtc::core::PlanResult;
using mtc::core::Pose6D;

constexpr double kStartToleranceRad = 1e-3;

std::string fmt(double value)
{
  std::ostringstream os;
  os << std::setprecision(6) << value;
  return os.str();
}

bool scaling_ok(double value, double cap)
{
  return std::isfinite(value) && value > 0.0 && value <= cap;
}

/// 轨迹首点与期望起点（按名称）是否一致。
bool starts_at(const mtc::core::TimedTrajectory & trajectory, const JointStateSnapshot & start)
{
  return check_trajectory_start(
    trajectory, start.names, start.positions, kStartToleranceRad).ok;
}

}  // namespace

MoveItPlannerPort::MoveItPlannerPort(
  PlanningBackendPtr backend,
  PlannerConfig config,
  mtc::core::ClockPtr clock,
  bool asynchronous)
: backend_(std::move(backend)),
  config_(std::move(config)),
  clock_(std::move(clock)),
  asynchronous_(asynchronous)
{
  // 工具模型是 TCP 的唯一来源：启用时由 CAD 几何计算 ee_link -> hook_tcp 与工具碰撞体。
  if (config_.tool.enabled) {
    const auto frames = compute_tool_frames(config_.tool);
    if (frames.valid) {
      config_.ee_to_tcp = frames.ee_to_tcp;
      config_.tcp_status = config_.tool_status;
      tool_bodies_ = tool_bodies_in_ee(config_.tool, frames);
    } else {
      tool_error_ = frames.message;
    }
  }
}

MoveItPlannerPort::~MoveItPlannerPort()
{
  std::map<std::string, std::future<PlanResult>> pending;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending.swap(jobs_);
  }
  for (auto & entry : pending) {
    if (entry.second.valid()) {
      entry.second.wait();
    }
  }
}

std::string MoveItPlannerPort::make_key(const mtc::manipulation::PlanTarget & target)
{
  std::ostringstream os;
  os << std::setprecision(17);
  os << target.phase << '|' << target.object_id << '|' << target.slot << '|';
  for (const auto & primitive : target.primitives) {
    os << primitive.name << ',';
  }
  os << '|' << target.has_ring_pose;
  if (target.has_ring_pose) {
    const auto & p = target.ring_pose;
    os << '|' << p.frame_id << ',' << p.stamp_s << ',' << p.position.x << ',' << p.position.y <<
      ',' << p.position.z << ',' << p.orientation.x << ',' << p.orientation.y << ',' <<
      p.orientation.z << ',' << p.orientation.w;
  }
  return os.str();
}

std::optional<PlanResult> MoveItPlannerPort::plan(const mtc::manipulation::PlanTarget & target)
{
  if (!asynchronous_) {
    return plan_blocking(target);
  }
  const auto key = make_key(target);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = jobs_.find(key);
  if (found != jobs_.end()) {
    if (found->second.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return std::nullopt;
    }
    PlanResult result = found->second.get();
    jobs_.erase(found);
    return result;
  }
  jobs_.emplace(key, std::async(std::launch::async, [this, target]() {
      return plan_blocking(target);
    }));
  return std::nullopt;
}

PlanReport MoveItPlannerPort::last_report() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_report_;
}

PlanResult MoveItPlannerPort::plan_blocking(
  const mtc::manipulation::PlanTarget & target,
  const std::optional<JointStateSnapshot> & start_override)
{
  PlanOptions options;
  options.start = start_override;
  return plan_blocking(target, options);
}

PlanResult MoveItPlannerPort::replan_primitive(
  const mtc::manipulation::PlanTarget & original,
  const std::string & primitive_name,
  const JointStateSnapshot & current)
{
  mtc::manipulation::PlanTarget target = original;
  target.phase = original.phase + "/REPLAN";
  target.primitives.clear();
  target.primitive_names.clear();
  for (const auto & primitive : original.primitives) {
    if (primitive.name == primitive_name) {
      target.primitives.push_back(primitive);
      target.primitive_names.push_back(primitive.name);
      break;
    }
  }
  PlanOptions options;
  options.start = current;
  options.skip_tool_gaps = true;
  return plan_blocking(target, options);
}

PlanResult MoveItPlannerPort::plan_blocking(
  const mtc::manipulation::PlanTarget & target, const PlanOptions & options)
{
  const auto & start_override = options.start;
  std::lock_guard<std::mutex> backend_lock(backend_mutex_);

  PlanReport report;
  report.phase = target.phase;

  auto finish = [this, &report](PlanResult result) {
      std::lock_guard<std::mutex> lock(mutex_);
      last_report_ = report;
      return result;
    };
  auto fail = [&](const std::string & reason, const std::string & primitive,
      const std::string & detail) {
      PlanResult failed;
      failed.ok = false;
      failed.error_code = ErrorCodes::PLANNING_FAILED;
      failed.primitives = target.primitives;
      failed.message = "[" + reason + "] " + (primitive.empty() ? "plan" : primitive) + ": " +
        detail;
      report.simulation_only = true;
      return finish(failed);
    };

  if (!backend_) {
    return fail("NO_BACKEND", "", "planning backend is not configured");
  }
  report.planning_frame = backend_->planning_frame();
  report.end_effector_link = backend_->end_effector_link();
  if (target.primitives.empty()) {
    return fail("EMPTY_TARGET", "", "PlanTarget has no primitives");
  }
  if (!scaling_ok(config_.velocity_scaling, config_.scaling_cap) ||
    !scaling_ok(config_.acceleration_scaling, config_.scaling_cap) ||
    config_.joint_names.empty() || !(config_.timeout_factor >= 1.0) ||
    report.planning_frame.empty())
  {
    return fail("INVALID_CONFIG", "",
             "scaling must be in (0, " + fmt(config_.scaling_cap) +
             "], timeout_factor >= 1, joint names and planning frame non-empty");
  }

  std::optional<JointStateSnapshot> start = start_override;
  if (!start) {
    start = backend_->current_state();
  }
  if (!start) {
    return fail("NO_CURRENT_STATE", "", "robot state unavailable (is /joint_states published?)");
  }
  if (!start->shape_is_valid()) {
    return fail("INVALID_START_STATE", "",
             "start state must have matching names/positions and unique, non-empty joint names");
  }
  report.start_state = *start;
  JointStateSnapshot chain = *start;
  std::string start_assumption = start_override ? "explicit_start_state" : "current_state";

  std::vector<std::string> uncalibrated;
  auto accept_calibration = [&](const std::string & item, CalibrationStatus status) {
      if (status == CalibrationStatus::MEASURED) {
        return true;
      }
      if (status == CalibrationStatus::UNKNOWN || !config_.allow_uncalibrated_geometry) {
        return false;
      }
      const std::string entry = item + "=" + to_string(status);
      if (std::find(uncalibrated.begin(), uncalibrated.end(), entry) == uncalibrated.end()) {
        uncalibrated.push_back(entry);
      }
      return true;
    };

  if (!tool_error_.empty()) {
    return fail("INVALID_CONFIG", "", "tool model: " + tool_error_);
  }
  if (config_.tool.enabled && !accept_calibration("tool_geometry", config_.tool_status)) {
    return fail("TOOL_UNKNOWN", "", "tool geometry status is UNKNOWN");
  }

  const Transform tcp_to_ee = inverse(config_.ee_to_tcp);
  double stamp = clock_ ? clock_->now() : 0.0;
  if (!std::isfinite(stamp) || stamp < 0.0) {
    stamp = 0.0;
  }

  std::optional<AttachedPayload> payload;   // 试提时附着，释放后不再附着
  bool payload_released = false;

  std::vector<mtc::core::MotionCommand> commands;
  // 同一 PickPlace 状态内顺序执行的原语共用一个看门狗：按状态累计时长。
  std::map<mtc::manipulation::PickPlaceState, double> state_duration;
  for (const auto & primitive : target.primitives) {
    PrimitivePlanReport entry;
    entry.primitive_name = primitive.name;
    entry.start_assumption = start_assumption;
    auto fail_primitive = [&](const std::string & reason, const std::string & detail) {
        entry.ok = false;
        entry.reason = reason;
        entry.message = detail;
        report.primitives.push_back(entry);
        return fail(reason, primitive.name, detail);
      };

    if (config_.tool_owned_primitives.count(primitive.name) != 0u) {
      return fail_primitive("TOOL_OWNED_PRIMITIVE",
               "executed by ToolPort; the planner must not emit a trajectory for it");
    }
    const auto spec_it = config_.primitives.find(primitive.name);
    if (spec_it == config_.primitives.end()) {
      return fail_primitive("UNKNOWN_PRIMITIVE", "no planning rule for this primitive");
    }
    const PrimitiveSpec & spec = spec_it->second;
    entry.mode = to_string(spec.mode);
    entry.goal_kind = to_string(spec.goal);
    entry.contact_intent_only = spec.contact_intent_only;
    if (!primitive.is_usable()) {
      return fail_primitive("INVALID_PRIMITIVE",
               "empty name, non-positive speed limit or unusable target_pose");
    }

    auto make_request = [&](const std::string & label, MotionMode mode, const Pose6D & goal) {
        BackendRequest request;
        request.label = label;
        request.tool_bodies = tool_bodies_;
        request.mode = mode;
        request.ee_goal = goal;
        request.start = chain;
        request.velocity_scaling = config_.velocity_scaling;
        request.acceleration_scaling = config_.acceleration_scaling;
        request.max_linear_speed_m_s = primitive.max_linear_speed;
        request.max_angular_speed_rad_s = primitive.max_angular_speed;
        request.planning_time_s = config_.planning_time_s;
        request.cartesian_step_m = config_.cartesian_step_m;
        request.min_cartesian_fraction = config_.min_cartesian_fraction;
        request.joint_bounds = config_.joint_position_limits;
        request.prefer_straight_line =
          mode == MotionMode::FREE_SPACE && config_.prefer_straight_line_free_space;
        return request;
      };

    // ---- 原语前的 ToolPort 动作：只假设其终点，不输出轨迹 ----
    if (spec.gap_before != ToolGapBefore::NONE && !options.skip_tool_gaps) {
      if (!accept_calibration("TCP", config_.tcp_status)) {
        return fail_primitive("TCP_UNKNOWN", "ee_link -> hook_tcp is not available");
      }
      const auto ee_now = backend_->end_effector_pose(chain);
      if (!ee_now) {
        return fail_primitive("FK_FAILED", "cannot compute end-effector pose for tool gap");
      }
      const double sign = spec.gap_before == ToolGapBefore::ENGAGE_INSERT ? 1.0 : -1.0;
      const auto tcp_gap = offset_along_local_axis(
        apply(*ee_now, config_.ee_to_tcp), config_.insertion_axis, sign * spec.gap_distance_m);
      const auto gap = backend_->plan(make_request(
          std::string("simulated_") + to_string(spec.gap_before), MotionMode::CARTESIAN,
          apply(tcp_gap, tcp_to_ee)));
      if (!gap.ok || !gap.end_state.shape_is_valid()) {
        return fail_primitive("TOOL_GAP_SIMULATION_FAILED", gap.reason + ": " + gap.message);
      }
      chain = gap.end_state;
      entry.start_assumption = std::string("simulated ") + to_string(spec.gap_before) + " " +
        fmt(spec.gap_distance_m) + " m along TCP axis " + std::to_string(config_.insertion_axis) +
        " (ToolPort motion, not emitted)";
    }

    // ---- 目标位姿（planning frame 中的 ee_link）----
    Pose6D ee_goal;
    if (spec.goal == GoalKind::RING_STANDOFF) {
      std::optional<Pose6D> ring;
      if (primitive.target_pose && primitive.target_pose->is_usable()) {
        ring = *primitive.target_pose;
      } else if (target.has_ring_pose && target.ring_pose.is_usable()) {
        ring = target.ring_pose;
      }
      if (!ring) {
        return fail_primitive("TARGET_POSE_MISSING",
                 "no usable ring pose (valid, frame_id, unit quaternion) in primitive or target");
      }
      if (!accept_calibration("TCP", config_.tcp_status)) {
        return fail_primitive("TCP_UNKNOWN", "ee_link -> hook_tcp is not available");
      }
      const auto ring_in_frame = backend_->transform_to_planning_frame(*ring);
      if (!ring_in_frame) {
        return fail_primitive("TF_UNAVAILABLE",
                 "cannot transform ring pose from '" + ring->frame_id + "' to '" +
                 report.planning_frame + "'");
      }
      Pose6D ring_goal = *ring_in_frame;
      if (config_.hook_up_alignment && config_.insertion_axis == 2) {
        // 钩口朝上：只信提环的插入轴；绕插入轴的滚转对舌规由重力决定（感知给出的滚转不可靠）。
        ring_goal.orientation = align_roll_to_up(
          ring_goal.orientation, mtc::core::Vector3{0.0, 0.0, 1.0}, config_.hook_up_min_tilt_rad);
      }
      const auto tcp_goal = offset_along_local_axis(
        ring_goal, config_.insertion_axis, -spec.distance_m);
      ee_goal = apply(tcp_goal, tcp_to_ee);
    } else if (spec.goal == GoalKind::RELATIVE_UP) {
      const auto ee_now = backend_->end_effector_pose(chain);
      if (!ee_now) {
        return fail_primitive("FK_FAILED", "cannot compute end-effector pose at start state");
      }
      ee_goal = translate_in_frame(*ee_now, mtc::core::Vector3{0.0, 0.0, spec.distance_m});
    } else {
      const std::string & slot_name = target.slot;
      if (!primitive.target_label.empty() && config_.slots.count(primitive.target_label) != 0u &&
        primitive.target_label != slot_name)
      {
        return fail_primitive("SLOT_MISMATCH",
                 "primitive targets '" + primitive.target_label + "' but request slot is '" +
                 slot_name + "'");
      }
      const auto slot_it = config_.slots.find(slot_name);
      if (slot_it == config_.slots.end() ||
        slot_it->second.status == CalibrationStatus::UNKNOWN)
      {
        return fail_primitive("SLOT_POSE_UNKNOWN", "no pose for slot '" + slot_name + "'");
      }
      if (!accept_calibration("slot " + slot_name, slot_it->second.status)) {
        return fail_primitive("SLOT_NOT_CALIBRATED",
                 "slot '" + slot_name + "' is " + to_string(slot_it->second.status));
      }
      if (!accept_calibration("TCP", config_.tcp_status)) {
        return fail_primitive("TCP_UNKNOWN", "ee_link -> hook_tcp is not available");
      }
      if (!slot_it->second.pose.is_usable()) {
        return fail_primitive("SLOT_POSE_INVALID", "slot pose has no frame or invalid quaternion");
      }
      const auto slot_in_frame = backend_->transform_to_planning_frame(slot_it->second.pose);
      if (!slot_in_frame) {
        return fail_primitive("TF_UNAVAILABLE",
                 "cannot transform slot '" + slot_name + "' from '" +
                 slot_it->second.pose.frame_id + "' to '" + report.planning_frame + "'");
      }
      const auto ee_now = backend_->end_effector_pose(chain);
      if (!ee_now) {
        return fail_primitive("FK_FAILED", "cannot compute end-effector pose at start state");
      }
      // 搬运/落座保持抓取时的工具姿态（不旋转电池），只改变位置。
      Pose6D tcp_goal = apply(*ee_now, config_.ee_to_tcp);
      tcp_goal.position = slot_in_frame->position;
      tcp_goal.position.z += spec.distance_m;
      ee_goal = apply(tcp_goal, tcp_to_ee);
    }
    ee_goal.valid = true;
    entry.ee_goal = ee_goal;

    // ---- 载荷：试提起附着在 ee_link 上，释放后（非 carries_payload 原语）移除 ----
    if (spec.carries_payload && config_.payload.enabled && !payload_released) {
      if (!payload) {
        if (!accept_calibration("payload", config_.payload.status)) {
          return fail_primitive("PAYLOAD_UNKNOWN", "battery payload geometry is not available");
        }
        const auto ee_now = backend_->end_effector_pose(chain);
        if (!ee_now) {
          return fail_primitive("FK_FAILED", "cannot compute end-effector pose to attach payload");
        }
        // 电池靠重力挂在钩尖（TCP）正下方，姿态与 planning frame 对齐。
        const auto tcp_now = apply(*ee_now, config_.ee_to_tcp);
        Transform world_payload;
        world_payload.translation = tcp_now.position;
        world_payload.translation.z -= config_.payload.hang_below_tcp_m;
        AttachedPayload attached;
        attached.size_m = config_.payload.size_m;
        attached.touch_links = config_.payload.touch_links;
        attached.ee_to_payload = compose(inverse(to_transform(*ee_now)), world_payload);
        payload = attached;
      }
    } else if (payload) {
      payload.reset();
      payload_released = true;
    }
    entry.payload_attached = payload.has_value();

    // ---- 规划 ----
    auto request = make_request(primitive.name, spec.mode, ee_goal);
    request.payload = payload;
    const auto planned = backend_->plan(request);
    entry.planning_time_s = planned.planning_time_s;
    entry.cartesian_fraction = planned.cartesian_fraction;
    entry.time_scale_applied = planned.time_scale_applied;
    entry.max_linear_speed_m_s = planned.max_linear_speed_m_s;
    entry.max_angular_speed_rad_s = planned.max_angular_speed_rad_s;
    entry.method = planned.method;
    entry.backend_message = planned.message;
    if (!planned.ok) {
      return fail_primitive(planned.reason.empty() ? "NO_PLAN" : planned.reason, planned.message);
    }

    auto converted = to_timed_trajectory(
      planned.trajectory, config_.joint_names, report.planning_frame, stamp);
    if (!converted.ok) {
      return fail_primitive("INVALID_TRAJECTORY", converted.error);
    }
    const auto & trajectory = converted.trajectory;
    if (!starts_at(trajectory, chain)) {
      return fail_primitive("START_STATE_MISMATCH",
               "trajectory does not start at the expected start state (tolerance " +
               fmt(kStartToleranceRad) + " rad)");
    }
    for (size_t j = 0; j < trajectory.joint_names.size(); ++j) {
      const auto limit = config_.joint_position_limits.find(trajectory.joint_names[j]);
      if (limit == config_.joint_position_limits.end()) {
        continue;
      }
      for (const auto & point : trajectory.points) {
        const double value = point.positions_rad[j];
        if (value < limit->second.first || value > limit->second.second) {
          return fail_primitive("JOINT_LIMIT_VIOLATION",
                   trajectory.joint_names[j] + "=" + fmt(value) + " rad outside [" +
                   fmt(limit->second.first) + ", " + fmt(limit->second.second) + "]");
        }
      }
    }
    const double duration = trajectory.duration_s();
    const auto watchdog_it = config_.state_timeouts.find(spec.executing_state);
    const double watchdog = watchdog_it != config_.state_timeouts.end() ? watchdog_it->second : 0.0;
    entry.state_watchdog_s = watchdog;
    entry.point_count = trajectory.points.size();
    entry.duration_s = duration;
    const double state_total = state_duration[spec.executing_state] + duration;
    if (watchdog > 0.0 && state_total > watchdog * config_.state_timeout_usage) {
      const bool shared = state_total > duration;
      return fail_primitive("EXCEEDS_STATE_WATCHDOG",
               "trajectory lasts " + fmt(duration) + " s" +
               (shared ? " (" + fmt(state_total) + " s with earlier primitives in the same state)" :
               std::string()) + ", more than " +
               fmt(config_.state_timeout_usage * 100.0) + "% of the " +
               mtc::manipulation::to_string(spec.executing_state) + " watchdog (" +
               fmt(watchdog) + " s)");
    }
    state_duration[spec.executing_state] = state_total;
    const auto timeout_ms = static_cast<uint32_t>(
      std::ceil(duration * config_.timeout_factor * 1000.0)) + config_.timeout_margin_ms;
    entry.timeout_ms = timeout_ms;

    mtc::core::MotionCommand command;
    command.joint_names = trajectory.joint_names;
    command.target_rad.clear();
    command.velocity_scaling = config_.velocity_scaling;
    command.acceleration_scaling = config_.acceleration_scaling;
    command.timeout_ms = timeout_ms;
    command.primitive_name = primitive.name;
    command.requires_contact = primitive.requires_contact;
    command.intent = primitive;
    command.trajectory = trajectory;
    commands.push_back(std::move(command));

    entry.ok = true;
    report.primitives.push_back(entry);
    chain = planned.end_state.shape_is_valid() ?
      planned.end_state :
      JointStateSnapshot{trajectory.joint_names, final_positions(trajectory)};
    start_assumption = "previous_primitive_end";
  }

  const bool simulation_only = config_.force_simulation_only || !uncalibrated.empty();
  for (auto & command : commands) {
    command.simulation_only = simulation_only;
  }
  report.simulation_only = simulation_only;
  report.uncalibrated_inputs = uncalibrated;

  PlanResult result;
  result.ok = true;
  result.error_code = ErrorCodes::OK;
  result.primitives = target.primitives;
  result.commands = std::move(commands);
  std::ostringstream message;
  message << "planned " << result.commands.size() << " primitive(s) in frame '" <<
    report.planning_frame << "' for '" << report.end_effector_link <<
    "'; simulation_only=" << (simulation_only ? "true" : "false");
  if (!uncalibrated.empty()) {
    message << " (uncalibrated:";
    for (const auto & item : uncalibrated) {
      message << ' ' << item;
    }
    message << ')';
  }
  result.message = message.str();
  return finish(result);
}

}  // namespace moveit_planner
}  // namespace mtc
