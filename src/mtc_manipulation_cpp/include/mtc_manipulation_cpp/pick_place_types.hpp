// Copyright 2026 meituan_challenge_ws contributors
//
// PickPlace FSM（第二层）的公共类型、端口契约与配置。
//
// 迁移自 Python src/mtc_manipulation/mtc_manipulation/pick_place_fsm.py。
// 行为基线见 docs/migration/PYTHON_BEHAVIOR_BASELINE.md，
// 映射关系见 docs/migration/FSM_BEHAVIOR_MAPPING.md。
//
// 与 Python 的差异（仅实现手段，不改语义）：
//   1. "端口返回 None 表示调用未完成" -> 统一用 std::optional 的 nullopt 表达；
//   2. payload 由 Dict/Any 改为类型化结构；
//   3. 位姿由 Tuple[float,float,float] 升级为 core::Pose6D
//      （保留 6D 姿态、frame_id 与时间戳，见需求 5）；
//   4. 端口结果缓存按类型分离为多个 map（类型安全）。
//
// 本文件不依赖 ROS，也不做任何真实 IO。

#ifndef MTC_MANIPULATION_CPP__PICK_PLACE_TYPES_HPP_
#define MTC_MANIPULATION_CPP__PICK_PLACE_TYPES_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "mtc_core_cpp/clock.hpp"
#include "mtc_core_cpp/motion_primitive.hpp"
#include "mtc_core_cpp/pose.hpp"
#include "mtc_core_cpp/results.hpp"
#include "mtc_core_cpp/tool_state.hpp"
#include "mtc_core_cpp/timed_trajectory.hpp"

namespace mtc
{
namespace manipulation
{

/// 允许的目标工位（与 PickPlace.action / 设计 §3.1 一致）。
const std::vector<std::string> & allowed_target_slots();

/// 工具类型字符串（与 core::ToolTypes 一致，此处提供便捷别名）。
constexpr const char * TOOL_PASSIVE_HOOK_V1 = mtc::core::ToolTypes::PASSIVE_HOOK_V1;
constexpr const char * TOOL_MAGNETIC_LATCH_V2 = mtc::core::ToolTypes::MAGNETIC_LATCH_V2;

/// 安全语义 1 的锚点：placement_verified 只允许由该状态写入。
constexpr const char * PLACEMENT_VERIFIED_OWNER_STATE = "VERIFY_PLACED";

/// 状态枚举（顺序即设计 §3.2 的正常流水线顺序）。
enum class PickPlaceState
{
  RESOLVE_TARGET = 0,
  PLAN_ACQUIRE,
  MOVE_PRE_ALIGN,
  ENGAGE,
  VERIFY_ENGAGEMENT,
  TEST_LIFT,
  VERIFY_ATTACHED,
  LIFT,
  TRANSPORT,
  MOVE_PRE_SEAT,
  SEAT,
  VERIFY_SEATED,
  UNLOAD,
  DISENGAGE,
  RETREAT,
  VERIFY_PLACED,
  SUCCESS,
  RECOVERY,
  FAILED,
  FAULT,
  CANCEL_PENDING,
  CANCELLED,
};

/// 状态名与 Python `PickPlaceState.value` 逐字一致（feedback.substate 直接对外）。
const char * to_string(PickPlaceState state);

/// 全部状态（枚举顺序）。
const std::vector<PickPlaceState> & all_pick_place_states();

/// 终态：step() 到达后不再变化。
const std::set<PickPlaceState> & terminal_states();

/// 运动在途状态（这些状态超时说明"机器人是否还在动"不确定 -> FAULT，禁止重发）。
const std::set<PickPlaceState> & motion_in_flight_states();

/// 正常流水线（用于 progress 计算）。
const std::vector<PickPlaceState> & pipeline_states();

/// 抓放参数（数值为保守默认值，未做实机标定，见设计 §8）。
struct PickPlaceConfig
{
  std::string tool_type = TOOL_PASSIVE_HOOK_V1;
  double placement_stability_sec = 3.0;
  double perception_max_age_s = 0.3;
  int recovery_max_attempts = 1;
  bool verify_test_lift = true;
  bool verify_placement = true;
  std::vector<std::string> joint_names = {
    "shoulder_joint", "upperArm_joint", "foreArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint",
  };
  double default_velocity_scaling = 0.2;
  double default_acceleration_scaling = 0.2;
  uint32_t default_timeout_ms = 5000;
  /// 每状态看门狗超时（秒）。运动在途状态超时 = 状态不确定 -> FAULT。
  std::map<PickPlaceState, double> state_timeouts;
};

/// 默认每状态超时（与 Python PickPlaceConfig.state_timeouts 逐项一致）。
std::map<PickPlaceState, double> default_state_timeouts();

/// 单次 step() 的返回：状态 + 错误码 + 是否可恢复。
struct StepResult
{
  PickPlaceState state = PickPlaceState::RESOLVE_TARGET;
  uint16_t error_code = mtc::core::ErrorCodes::OK;
  bool resumable = false;
  std::string message;

  bool terminal() const {return terminal_states().count(state) == 1u;}
};

/// 等价于 PickPlace.action Result。
struct PickPlaceResult
{
  bool success = false;
  bool placement_verified = false;
  uint16_t error_code = mtc::core::ErrorCodes::OK;
  std::string message;
  PickPlaceState final_state = PickPlaceState::RESOLVE_TARGET;
  std::vector<PickPlaceState> states_history;
};

/// 等价于 PickPlace.action Feedback。
struct PickPlaceFeedback
{
  std::string substate;
  float progress = 0.0f;
  bool object_attached_estimated = false;
  std::string detail;
};

// ---------------------------------------------------------------------------
// 端口（Ports）
//
// 统一约定：返回 std::optional<T>，**nullopt 表示调用尚未完成**，
// 调用方应返回"等待"并由状态看门狗负责超时判定。
// 这与 Python 的"返回 None 表示仍在执行"严格等价。
// ---------------------------------------------------------------------------

/// 规划请求的目标描述（替代 Python 的 Dict[str, Any]）。
struct PlanTarget
{
  std::string phase;
  std::string object_id;
  std::string slot;
  bool has_ring_pose = false;
  mtc::core::Pose6D ring_pose;   ///< 保留 6D 与 frame_id
  std::vector<std::string> primitive_names;
  mtc::core::MotionPrimitiveList primitives; ///< Full intent, including pose and constraints.
};

class PerceptionPort
{
public:
  virtual ~PerceptionPort() = default;

  /// expected_color 为 nullopt 时跳过颜色校验（用于放置后核验）。
  virtual std::optional<mtc::core::DetectionResult> get_detection(
    const std::string & object_id,
    std::optional<int> expected_color,
    double max_age_s) = 0;
};

class PlannerPort
{
public:
  virtual ~PlannerPort() = default;

  /// 只做"规划可行性 + 运动原语返回"，不掌控关节控制权。
  virtual std::optional<mtc::core::PlanResult> plan(const PlanTarget & target) = 0;
};

class MotionPort
{
public:
  virtual ~MotionPort() = default;

  /// 返回 nullopt 表示"仍在执行"（由调用方按状态看门狗等待）。
  virtual std::optional<mtc::core::MotionResult> execute_joint_move(
    const mtc::core::MotionCommand & command) = 0;

  /// One complete trajectory or an explicit refusal; never split into joint moves.
  virtual std::optional<mtc::core::MotionResult> execute_trajectory(
    const mtc::core::MotionCommand & command)
  {
    (void)command;
    mtc::core::MotionResult result;
    result.exec_state = mtc::core::ExecState::REJECTED;
    result.error_code = mtc::core::ErrorCodes::EXECUTION_REJECTED;
    result.message = "trajectory execution unsupported; splitting forbidden";
    return result;
  }

  /// 软件停止请求；stop_confirmed 必须由后端独立确认。
  virtual mtc::core::StopResult request_stop(const std::string & reason) = 0;
};

class ToolPort
{
public:
  virtual ~ToolPort() = default;

  virtual std::optional<mtc::core::ToolEvidence> engage(
    const std::string & phase,
    const mtc::core::MotionPrimitiveList & primitives) = 0;

  virtual std::optional<mtc::core::ToolEvidence> test_lift(
    const mtc::core::MotionPrimitiveList & primitives) = 0;

  virtual mtc::core::ToolEvidence seat_verify() = 0;

  virtual std::optional<mtc::core::ToolEvidence> unload(
    const mtc::core::MotionPrimitiveList & primitives) = 0;

  virtual std::optional<mtc::core::ToolEvidence> unlock(uint32_t pulse_ms = 200) = 0;

  virtual std::optional<mtc::core::ToolEvidence> disengage(
    const mtc::core::MotionPrimitiveList & primitives) = 0;

  /// 返回 "none" / "holding" / "unknown"：决定失败后是否允许自动重试。
  virtual std::string holding_state() = 0;
};

using PerceptionPortPtr = std::shared_ptr<PerceptionPort>;
using PlannerPortPtr = std::shared_ptr<PlannerPort>;
using MotionPortPtr = std::shared_ptr<MotionPort>;
using ToolPortPtr = std::shared_ptr<ToolPort>;

}  // namespace manipulation
}  // namespace mtc

#endif  // MTC_MANIPULATION_CPP__PICK_PLACE_TYPES_HPP_
