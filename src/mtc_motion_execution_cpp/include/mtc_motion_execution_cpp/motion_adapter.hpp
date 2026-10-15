// Copyright 2026 meituan_challenge_ws contributors
//
// 运动适配器接口：C++ Motion Adapter（第一阶段只定义接口与 Mock 实现）。
//
// 迁移自 Python src/mtc_motion_execution/mtc_motion_execution/backends.py MotionBackend，
// 并依据 C++ 迁移审计（docs/migration/INTERFACE_COMPAT_AUDIT_CPP.md 缺口 G1）
// 增加时间参数化轨迹能力位。
//
// 能力边界（需求 4 明确约束）：
//   * 第一阶段只适配"给定六关节目标并验证真实到位"的能力；
//   * **禁止**把一条多点轨迹拆成大量 ExecuteJointMove 调用；
//     因此 execute_trajectory 的默认实现是**显式拒绝**，而不是逐点降级。
//
// 实现者必须保证：
//   * 单在途命令：同一时刻只接受一个未结束的 command_id；
//   * 到位判定同时满足位置误差、持续静止与执行队列清空；
//   * 取消/停止只表示"请求已送达"，必须另有 stop_confirmed 依据。

#ifndef MTC_MOTION_EXECUTION_CPP__MOTION_ADAPTER_HPP_
#define MTC_MOTION_EXECUTION_CPP__MOTION_ADAPTER_HPP_

#include <memory>
#include <optional>
#include <string>

#include "mtc_core_cpp/exec_state.hpp"
#include "mtc_core_cpp/clock.hpp"
#include "mtc_core_cpp/motion_primitive.hpp"
#include "mtc_core_cpp/results.hpp"
#include "mtc_core_cpp/timed_trajectory.hpp"

namespace mtc
{
namespace motion
{

class MotionAdapter
{
public:
  virtual ~MotionAdapter() = default;

  /// 后端能力声明。
  ///
  /// 设计约束（V0.1 §4.9）：capability 只表示**当前已连接且经实测启用**的通路能力，
  /// 不能仅因为 SDK 宣称存在某方法就置 true。
  virtual mtc::core::MotionCapabilities capabilities() const = 0;

  /// Immutable clock domain for feedback/evidence, shared for the adapter lifetime.
  /// Null means the ordinary monotonic real-time domain; no virtual time is inferred.
  virtual mtc::core::ClockPtr execution_clock() const {return nullptr;}

  /// 执行一次单目标 6 关节运动（等价于 ExecuteJointMove.action 的能力）。
  ///
  /// 返回值的语义：
  ///   * exec_state == SETTLED 且 arrived == true  -> 已到位并停稳；
  ///   * exec_state == REJECTED                    -> 从未开始运动，允许立即重发；
  ///   * status_unknown == true                    -> 状态不确定，**禁止**自动重发；
  ///   * timed_out == true                         -> 可能仍在运动，**禁止**自动重发。
  virtual mtc::core::MotionResult execute_joint_move(const mtc::core::MotionCommand & command) = 0;

  /// 软件停止请求。
  ///
  /// 返回的 `stop_confirmed` 必须由后端独立确认；调用方不得据
  /// `request_delivered` 直接宣告安全停止。
  virtual mtc::core::StopResult request_stop(const std::string & reason) = 0;

  /// Preserve a cancel that races dispatch for this command. Executable adapters must
  /// implement this association. Never upgrade a scoped cancel to a global stop.
  virtual mtc::core::StopResult request_stop_for_command(
    const std::string & reason, const std::optional<std::string> & command_id)
  {
    if (!command_id) {return request_stop(reason);}
    mtc::core::StopResult result;
    result.error_code = mtc::core::ErrorCodes::EXECUTION_REJECTED;
    result.message = "goal-scoped cancel unsupported; global stop was not called";
    return result;
  }

  /// 关节反馈快照。
  virtual mtc::core::JointFeedback joint_feedback() const = 0;

  /// 后端提供的"显式停稳标志"。
  ///
  /// 返回 nullopt 表示后端**不提供**该依据；此时调用方必须退回到保守判断，
  /// 绝不允许仅凭 StopResult::request_delivered 宣告已停稳。
  /// 迁移自 Python 对 `stop_confirmed_flag()` 的鸭子类型探测。
  virtual std::optional<bool> explicit_stop_confirmation() const {return std::nullopt;}

  /// Thread-safe snapshot with command identity and a continuously stationary interval.
  /// execute/stop may overlap. Implementations must preserve a cancel that races dispatch;
  /// acceptance/cancellation callbacks alone cannot populate this evidence.
  virtual std::optional<mtc::core::StopEvidence> stop_evidence() const {return std::nullopt;}

  /// 执行一条时间参数化轨迹（多点、带时间戳）。
  ///
  /// **默认实现显式拒绝**（EXECUTION_REJECTED），理由有两条：
  ///   1. 未声明 timed_trajectory_supported 的后端不应假装支持；
  ///   2. 更重要的是：绝不允许实现者在这里偷偷把轨迹拆成多次
  ///      execute_joint_move —— 那会破坏时间同步语义，掩盖真实能力缺口。
  ///
  /// 第二阶段将由 ROS 2 外壳把本方法绑定到 control_msgs/FollowJointTrajectory。
  virtual mtc::core::MotionResult execute_trajectory(
    const mtc::core::TimedTrajectory & trajectory);
};

using MotionAdapterPtr = std::shared_ptr<MotionAdapter>;

}  // namespace motion
}  // namespace mtc

#endif  // MTC_MOTION_EXECUTION_CPP__MOTION_ADAPTER_HPP_
