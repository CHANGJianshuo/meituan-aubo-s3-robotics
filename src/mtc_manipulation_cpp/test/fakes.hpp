// Copyright 2026 meituan_challenge_ws contributors
//
// PickPlace FSM 测试替身（fakes）。
//
// 迁移自 Python src/mtc_manipulation/mtc_manipulation/fakes.py 的语义，
// 但改为 C++ 的"std::optional 返回 nullopt = 在途未完成"契约。
//
// 设计要点：
//   * 每个端口方法都可由测试脚本化控制（成功/失败/在途/证据等级）；
//   * "在途"通过在返回 nullopt 之前消耗 pending 计数实现，
//     使测试可以逐 step 观察看门狗与复用同一条 command_id 的行为；
//   * 所有 fakes 都有调用日志，用于断言"没有重发运动命令"。

#ifndef MTC_MANIPULATION_CPP__TEST__FAKES_HPP_
#define MTC_MANIPULATION_CPP__TEST__FAKES_HPP_

#include <algorithm>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "mtc_manipulation_cpp/pick_place_types.hpp"

namespace mtc
{
namespace manipulation
{
namespace testing
{

// ---------------------------------------------------------------------------
// 构造辅助
// ---------------------------------------------------------------------------

inline mtc::core::Pose6D make_pose(
  double x = 0.4, double y = -0.1, double z = 0.25,
  const std::string & frame_id = "base_link", double stamp_s = 0.0)
{
  mtc::core::Pose6D pose;
  pose.position = mtc::core::Vector3{x, y, z};
  pose.orientation = mtc::core::Quaternion{0.0, 0.0, 0.0, 1.0};
  pose.frame_id = frame_id;
  pose.stamp_s = stamp_s;
  pose.valid = true;
  return pose;
}

inline mtc::core::PoseEstimate make_pose_estimate(
  const std::string & object_id = "battery_1",
  int color = 1,
  bool at_target = false,
  bool settled = true,
  bool followed = false,
  bool ring_pose_valid = true,
  bool usable_pose = true)
{
  mtc::core::PoseEstimate estimate;
  estimate.object_id = object_id;
  estimate.color = color;
  estimate.has_ring_pose_value = true;
  estimate.ring_pose = make_pose(0.4, -0.1, 0.25, usable_pose ? "base_link" : "", 1.0);
  estimate.has_body_pose_value = true;
  estimate.body_pose = make_pose(0.4, -0.1, 0.20, usable_pose ? "base_link" : "", 1.0);
  estimate.confidence = 0.9f;
  estimate.stamp_s = 1.0;
  estimate.ring_pose_valid = ring_pose_valid;
  estimate.at_target = at_target;
  estimate.settled = settled;
  estimate.followed = followed;
  return estimate;
}

// ---------------------------------------------------------------------------
// FakePerception
// ---------------------------------------------------------------------------

class FakePerception : public PerceptionPort
{
public:
  mtc::core::ClockPtr clock;
  std::optional<double> fixed_stamp;
  std::optional<mtc::core::Pose6D> ring_override;
  // --- 抓取前核验（expected_color 有值）---
  bool found = true;
  bool unique = true;
  bool stale = false;
  int color = 1;
  bool valid_ring_pose = true;   ///< 位姿本身可用（frame_id/四元数）
  bool ring_pose_valid = true;   ///< BatteryDetection.ring_pose_valid
  bool followed = true;

  // --- 放置后核验（expected_color == nullopt，与 Python placement_* 字段一致）---
  bool placement_found = true;
  bool placement_stale = false;
  bool placement_at_target = true;
  bool placement_settled = true;

  std::string message;

  /// 前 N 次调用返回 nullopt（模拟感知未返回）。
  int pending_ticks = 0;
  /// true 时后续所有调用都返回 nullopt（模拟感知永久断流）。
  bool always_pending = false;

  int calls = 0;
  /// 记录每次查询的 expected_color（-1 表示 nullopt）。
  std::vector<int> requested_colors;

  std::optional<mtc::core::DetectionResult> get_detection(
    const std::string & object_id,
    std::optional<int> expected_color,
    double max_age_s) override
  {
    (void)max_age_s;
    ++calls;
    requested_colors.push_back(expected_color.value_or(-1));

    if (always_pending || pending_ticks > 0) {
      if (pending_ticks > 0) {
        --pending_ticks;
      }
      return std::nullopt;
    }

    mtc::core::DetectionResult result;
    result.message = message;

    if (!expected_color.has_value()) {
      // 放置后核验：不做颜色校验，只看是否在目标区域且稳定。
      result.found = placement_found;
      result.unique = true;
      result.stale = placement_stale;
      mtc::core::PoseEstimate estimate = make_pose_estimate(
        object_id, color, placement_at_target, placement_settled, false, true, true);
      estimate.stamp_s = fixed_stamp.value_or(clock ? clock->now() : 0.0);
      estimate.body_pose.stamp_s = estimate.stamp_s;
      estimate.ring_pose.stamp_s = estimate.stamp_s;
      result.pose = estimate;
      return result;
    }

    // 抓取前核验：found/unique/stale 与颜色、提环位姿有效性。
    result.found = found;
    result.unique = unique;
    result.stale = stale;
    mtc::core::PoseEstimate estimate = make_pose_estimate(
      object_id, color, false, true, followed, ring_pose_valid, valid_ring_pose);
    if (ring_override) {estimate.ring_pose = *ring_override;}
    result.pose = estimate;
    return result;
  }
};

// ---------------------------------------------------------------------------
// FakePlanner
// ---------------------------------------------------------------------------

class FakePlanner : public PlannerPort
{
public:
  bool ok = true;
  bool provide_commands = true;
  bool provide_trajectory = false;
  std::vector<double> planned_target = {0.05, -0.02, 0.01, 0.03, -0.04, 0.02};
  uint16_t error_code = mtc::core::ErrorCodes::OK;
  std::string message;

  /// 前 N 次调用返回 nullopt。
  int pending_ticks = 0;

  int calls = 0;
  std::vector<std::string> phases;
  std::vector<PlanTarget> targets;

  std::optional<mtc::core::PlanResult> plan(const PlanTarget & target) override
  {
    ++calls;
    phases.push_back(target.phase);
    targets.push_back(target);

    if (pending_ticks > 0) {
      --pending_ticks;
      return std::nullopt;
    }

    mtc::core::PlanResult result;
    result.ok = ok;
    result.error_code = error_code;
    result.message = message;
    if (provide_commands) {
      for (const auto & primitive : target.primitives) {
        mtc::core::MotionCommand command;
        command.primitive_name = primitive.name;
        command.joint_names = PickPlaceConfig().joint_names;
        command.target_rad = planned_target;
        command.intent = primitive;
        command.simulation_only = true; // Fake output cannot reach an executable backend.
        if (provide_trajectory) {
          mtc::core::TimedTrajectory trajectory;
          trajectory.joint_names = command.joint_names;
          trajectory.header_frame_id = "base_link";
          trajectory.points = {{0.0, planned_target, {}, {}}, {1.0, planned_target, {}, {}}};
          command.trajectory = trajectory;
          command.target_rad.clear();
        }
        result.commands.push_back(command);
      }
    }
    return result;
  }
};

// ---------------------------------------------------------------------------
// FakeMotion
// ---------------------------------------------------------------------------

class FakeMotion : public MotionPort
{
public:
  /// 前 N 次 execute_joint_move 返回 nullopt（在途）。
  int pending_ticks = 0;
  /// 对包含这些原语名的命令返回"状态未知"。
  std::vector<std::string> unknown_primitives;
  /// 对包含这些原语名的命令返回"超时"。
  std::vector<std::string> timeout_primitives;
  /// 对包含这些原语名的命令返回"未到位"（可恢复失败）。
  std::vector<std::string> failed_primitives;
  uint16_t failed_error_code = mtc::core::ErrorCodes::MOTION_TIMEOUT;
  std::string failed_message = "运动未到位";

  /// 停止请求行为。
  bool stop_request_delivered = true;
  bool stop_confirms = true;
  std::string stop_message = "停止请求已送达并确认停稳";

  int execute_calls = 0;
  int stop_calls = 0;
  /// 每次 execute_joint_move 收到的 command_id；用于断言"同一状态内不重发"。
  std::vector<std::string> command_ids;
  std::vector<mtc::core::MotionCommand> commands;

  std::optional<mtc::core::MotionResult> execute_joint_move(
    const mtc::core::MotionCommand & command) override
  {
    ++execute_calls;
    command_ids.push_back(command.command_id);
    commands.push_back(command);

    if (pending_ticks > 0) {
      --pending_ticks;
      return std::nullopt;
    }

    mtc::core::MotionResult result;
    result.exec_state = mtc::core::ExecState::SETTLED;
    result.final_position_rad = command.target_rad;

    if (contains(unknown_primitives, command.primitive_name)) {
      result.success = false;
      result.arrived = false;
      result.status_unknown = true;
      result.error_code = mtc::core::ErrorCodes::MOTION_STATUS_UNKNOWN;
      result.message = "反馈断流，运动状态不确定";
      result.exec_state = mtc::core::ExecState::STOP_UNKNOWN;
      return result;
    }
    if (contains(timeout_primitives, command.primitive_name)) {
      // 与 Python FakeMotion 一致：超时**只**设 timed_out，
      // 不设 status_unknown（否则 FSM 会优先按 230 处理）。
      result.success = false;
      result.arrived = false;
      result.timed_out = true;
      result.status_unknown = false;
      result.error_code = mtc::core::ErrorCodes::MOTION_TIMEOUT;
      result.message = "fake 超时: " + command.primitive_name;
      result.exec_state = mtc::core::ExecState::TIMEOUT;
      return result;
    }
    if (contains(failed_primitives, command.primitive_name)) {
      result.success = false;
      result.arrived = false;
      result.error_code = failed_error_code;
      result.message = failed_message;
      result.exec_state = mtc::core::ExecState::REJECTED;
      return result;
    }

    result.success = true;
    result.arrived = true;
    result.stop_confirmed = true;
    result.error_code = mtc::core::ErrorCodes::OK;
    result.message = "已到位并停稳";
    return result;
  }

  mtc::core::StopResult request_stop(const std::string & reason) override
  {
    ++stop_calls;
    (void)reason;
    mtc::core::StopResult result;
    result.request_delivered = stop_request_delivered;
    result.stop_confirmed = stop_confirms;
    result.message = stop_message;
    return result;
  }

private:
  static bool contains(const std::vector<std::string> & values, const std::string & value)
  {
    return std::find(values.begin(), values.end(), value) != values.end();
  }
};

// ---------------------------------------------------------------------------
// FakeTool
// ---------------------------------------------------------------------------

class FakeTool : public ToolPort
{
public:
  /// 默认给"几何级"证据，使正常流程可以通过 VERIFY_ATTACHED。
  uint8_t engage_evidence_level = mtc::core::EvidenceLevel::GEOMETRY;
  uint8_t engage_state = mtc::core::ToolStateCode::STATE_ATTACHED;
  bool engage_ok = true;
  uint16_t engage_error_code = mtc::core::ErrorCodes::OK;
  std::string engage_message;

  bool test_lift_ok = true;
  uint8_t test_lift_evidence_level = mtc::core::EvidenceLevel::GEOMETRY;
  uint16_t test_lift_error_code = mtc::core::ErrorCodes::ATTACH_NOT_VERIFIED;
  std::string test_lift_message;

  bool seated = true;
  bool seat_verify_ok = true;
  uint16_t seat_error_code = mtc::core::ErrorCodes::OK;
  std::string seat_message;

  bool unloaded = true;
  bool unload_ok = true;
  uint16_t unload_error_code = mtc::core::ErrorCodes::OK;
  std::string unload_message;

  bool unlock_ok = true;
  bool unlock_released = true;
  uint16_t unlock_error_code = mtc::core::ErrorCodes::OK;
  std::string unlock_message;

  bool disengage_ok = true;
  bool disengage_released = true;
  uint16_t disengage_error_code = mtc::core::ErrorCodes::OK;
  std::string disengage_message;

  std::string holding = "none";

  /// 各方法前 N 次调用返回 nullopt。
  int engage_pending_ticks = 0;
  int test_lift_pending_ticks = 0;
  int seat_verify_pending_ticks = 0;
  int unload_pending_ticks = 0;
  int unlock_pending_ticks = 0;
  int disengage_pending_ticks = 0;

  /// 故障注入：这些方法被调用时抛异常（模拟硬故障）。
  bool throw_on_engage = false;

  // 调用记录
  int engage_calls = 0;
  int test_lift_calls = 0;
  int seat_verify_calls = 0;
  int unload_calls = 0;
  int unlock_calls = 0;
  int disengage_calls = 0;
  int holding_calls = 0;
  std::vector<uint32_t> unlock_pulses;
  /// 机构 IO 事件；与 Python FakeTool 一致，只记录 engage / disengage / unlock。
  std::vector<std::string> io_events;
  /// 所有经 ToolPort 传入的原语名（按调用顺序）。
  std::vector<std::string> received_primitives;
  std::vector<std::string> engage_phases;

  std::optional<mtc::core::ToolEvidence> engage(
    const std::string & phase,
    const mtc::core::MotionPrimitiveList & primitives) override
  {
    ++engage_calls;
    engage_phases.push_back(phase);
    for (const auto & primitive : primitives) {
      received_primitives.push_back(primitive.name);
      io_events.push_back("ENGAGE:" + primitive.name);
    }
    if (throw_on_engage) {
      throw std::runtime_error("simulated tool failure");
    }
    if (engage_pending_ticks > 0) {
      --engage_pending_ticks;
      return std::nullopt;
    }
    mtc::core::ToolEvidence evidence;
    evidence.ok = engage_ok;
    evidence.state = engage_state;
    evidence.evidence_level = engage_evidence_level;
    evidence.verified = engage_ok;
    evidence.error_code = engage_error_code;
    evidence.message = engage_message;
    return evidence;
  }

  std::optional<mtc::core::ToolEvidence> test_lift(
    const mtc::core::MotionPrimitiveList & primitives) override
  {
    ++test_lift_calls;
    for (const auto & primitive : primitives) {
      received_primitives.push_back(primitive.name);
    }
    if (test_lift_pending_ticks > 0) {
      --test_lift_pending_ticks;
      return std::nullopt;
    }
    mtc::core::ToolEvidence evidence;
    evidence.ok = test_lift_ok;
    evidence.state = engage_state;
    evidence.evidence_level = test_lift_evidence_level;
    evidence.verified = test_lift_ok;
    evidence.error_code = test_lift_error_code;
    evidence.message = test_lift_message;
    return evidence;
  }

  mtc::core::ToolEvidence seat_verify() override
  {
    ++seat_verify_calls;
    if (seat_verify_pending_ticks > 0) {
      --seat_verify_pending_ticks;
    }
    mtc::core::ToolEvidence evidence;
    evidence.ok = seat_verify_ok;
    evidence.seated = seated;
    evidence.error_code = seat_error_code;
    evidence.message = seat_message;
    return evidence;
  }

  std::optional<mtc::core::ToolEvidence> unload(
    const mtc::core::MotionPrimitiveList & primitives) override
  {
    ++unload_calls;
    for (const auto & primitive : primitives) {
      received_primitives.push_back(primitive.name);
    }
    if (unload_pending_ticks > 0) {
      --unload_pending_ticks;
      return std::nullopt;
    }
    mtc::core::ToolEvidence evidence;
    evidence.ok = unload_ok;
    evidence.unloaded = unloaded;
    evidence.error_code = unload_error_code;
    evidence.message = unload_message;
    return evidence;
  }

  std::optional<mtc::core::ToolEvidence> unlock(uint32_t pulse_ms) override
  {
    ++unlock_calls;
    unlock_pulses.push_back(pulse_ms);
    io_events.push_back("electromagnetic_unlock:" + std::to_string(pulse_ms));
    if (unlock_pending_ticks > 0) {
      --unlock_pending_ticks;
      return std::nullopt;
    }
    mtc::core::ToolEvidence evidence;
    evidence.ok = unlock_ok;
    evidence.locked = !unlock_released;
    evidence.released = unlock_released;
    evidence.error_code = unlock_error_code;
    evidence.message = unlock_message;
    return evidence;
  }

  std::optional<mtc::core::ToolEvidence> disengage(
    const mtc::core::MotionPrimitiveList & primitives) override
  {
    ++disengage_calls;
    for (const auto & primitive : primitives) {
      received_primitives.push_back(primitive.name);
      io_events.push_back("disengage:" + primitive.name);
    }
    if (disengage_pending_ticks > 0) {
      --disengage_pending_ticks;
      return std::nullopt;
    }
    mtc::core::ToolEvidence evidence;
    evidence.ok = disengage_ok;
    evidence.released = disengage_released;
    evidence.error_code = disengage_error_code;
    evidence.message = disengage_message;
    return evidence;
  }

  std::string holding_state() override
  {
    ++holding_calls;
    return holding;
  }
};

}  // namespace testing
}  // namespace manipulation
}  // namespace mtc

#endif  // MTC_MANIPULATION_CPP__TEST__FAKES_HPP_
