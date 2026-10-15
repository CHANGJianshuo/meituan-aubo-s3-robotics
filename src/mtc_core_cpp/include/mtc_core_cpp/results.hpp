// Copyright 2026 meituan_challenge_ws contributors
//
// 端口返回类型（迁移自 Python pick_place_fsm.py / backends.py 的 dataclass 契约）。
//
// C++ 化约定（与 Python 语义严格等价）：
//   Python 中"端口返回 None 表示调用未完成、需要继续等待"，
//   C++ 中统一表达为 `std::optional<T>` 的 **nullopt**。
//   因此这些结构体本身只承载"已有结论"的结果；"在途"由 optional 表达。

#ifndef MTC_CORE_CPP__RESULTS_HPP_
#define MTC_CORE_CPP__RESULTS_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mtc_core_cpp/error_codes.hpp"
#include "mtc_core_cpp/exec_state.hpp"
#include "mtc_core_cpp/motion_primitive.hpp"
#include "mtc_core_cpp/pose.hpp"
#include "mtc_core_cpp/tool_state.hpp"

namespace mtc
{
namespace core
{

/// 感知查询结论（PerceptionPort.get_detection）。
struct DetectionResult
{
  bool found = false;
  bool unique = true;
  bool stale = false;
  std::optional<PoseEstimate> pose;
  uint16_t error_code = ErrorCodes::OK;
  std::string message;

  /// 只有"找到 + 唯一 + 新鲜 + 有位姿"才可用于生成运动目标。
  bool ok() const {return found && unique && !stale && pose.has_value();}
};

/// 规划结论（PlannerPort.plan）。
struct PlanResult
{
  bool ok = false;
  uint16_t error_code = ErrorCodes::OK;
  std::string message;
  MotionPrimitiveList primitives;
  /// One complete executable output per requested motion primitive. No default targets.
  std::vector<MotionCommand> commands;
};

/// 运动执行结论（等价于 ExecuteJointMove.action 的 Result/Feedback 可观测部分）。
struct MotionResult
{
  bool success = false;
  bool arrived = false;
  bool stop_confirmed = false;
  std::vector<double> final_position_rad;
  uint16_t error_code = ErrorCodes::OK;
  std::string message;
  /// 运动是否可能仍在进行 / 状态不确定。为 true 时上层**禁止**自动重发。
  bool status_unknown = false;
  bool timed_out = false;
  ExecState exec_state = ExecState::IDLE;

  /// 到位判定：必须同时 success 与 arrived，缺一不可。
  bool ok() const {return success && arrived;}

  /// 只有明确被拒绝、且从未开始运动的请求才允许立即重发。
  ///
  /// 这是"状态未知时禁止自动重发"的正面出口：其余失败一律要求先重新观测
  /// （重新感知 / 重新规划 / 人工确认）后才允许再次发令。
  bool retry_allowed_without_reobservation() const
  {
    return !success && exec_state == ExecState::REJECTED && !status_unknown && !timed_out;
  }
};

/// 取消 / 停止请求的结果。
///
/// 安全语义：`request_delivered` 只表示请求送达执行适配层；
/// `stop_confirmed` 必须由运动后端独立确认，二者绝不可互相替代。
struct StopResult
{
  bool request_delivered = false;
  bool stop_confirmed = false;
  std::string message;
  /// Internal C++ rejection/stop status; does not change any ROSIDL field.
  uint16_t error_code = ErrorCodes::OK;
};

/// 工具端口返回的机构证据（对应 ToolState.msg）。
struct ToolEvidence
{
  bool ok = false;
  uint8_t state = ToolStateCode::STATE_UNKNOWN;
  uint8_t evidence_level = EvidenceLevel::NONE;
  bool verified = false;
  std::optional<bool> locked;
  std::optional<bool> released;
  bool seated = false;
  bool unloaded = false;
  bool object_present = false;
  uint16_t error_code = ErrorCodes::OK;
  std::string message;

  /// 是否具备几何级或更强的挂载证据。
  bool has_geometry_evidence() const {return evidence_level >= EvidenceLevel::GEOMETRY;}
};

/// 运动后端能力声明。
///
/// 设计约束（设计 V0.1 §4.9）：
///   capability 只表示**当前已连接且经实测启用**的通路能力，
///   不得仅因 SDK 声称存在某方法就置 true。
struct MotionCapabilities
{
  std::string backend_name = "unknown";
  bool joint_goal_supported = false;
  bool timed_trajectory_supported = false;
  bool cartesian_motion_supported = false;
  bool cancel_supported = false;
  std::string detail;
  /// Explicit offline capability; never infer this from backend_name.
  bool simulation_only = false;
  bool associated_cancel_supported = false;
};

/// 关节反馈快照。
struct JointFeedback
{
  std::vector<double> position_rad;
  std::vector<double> velocity_rad_s;
  double stamp_s = 0.0;
};

/// Independent backend evidence. All stamps share the executor's monotonic clock.
/// The backend must check EVERY sample in this interval, not just its endpoints.
struct StopEvidence
{
  std::string command_id;
  ExecState exec_state = ExecState::STOP_UNKNOWN;
  bool communication_ok = false;
  bool queue_empty = false;
  bool stationary_interval_verified = false;
  double stationary_since_s = 0.0;
  JointFeedback feedback;
};

}  // namespace core
}  // namespace mtc

#endif  // MTC_CORE_CPP__RESULTS_HPP_
