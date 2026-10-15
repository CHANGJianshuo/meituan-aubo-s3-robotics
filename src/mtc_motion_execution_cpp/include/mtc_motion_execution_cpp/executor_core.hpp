// Copyright 2026 meituan_challenge_ws contributors
//
// 运动执行核心：唯一执行权 + 能力校验 + 到位确认。
//
// 迁移自 Python src/mtc_motion_execution/mtc_motion_execution/executor_core.py。
// 把设计 V0.1 §6.3 的七步握手落成可测试的状态转换：
//   PREPARE -> ACCEPT -> VALIDATE -> SEND -> MONITOR -> CONFIRM -> STOP/FAULT
//
// 关键安全语义：
//   * 只有同时满足到位、停稳、队列清空才返回 success=true；
//   * 后端拒绝（REJECTED，从未开始运动）才允许立即重发；
//   * 状态未知（STOP_UNKNOWN）时置位 unknown 标志，上层禁止自动重发；
//   * 取消不等于已停稳，只有 stop_confirmed 才敢声称安全停止。

#ifndef MTC_MOTION_EXECUTION_CPP__EXECUTOR_CORE_HPP_
#define MTC_MOTION_EXECUTION_CPP__EXECUTOR_CORE_HPP_

#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "mtc_motion_execution_cpp/motion_adapter.hpp"
#include "mtc_motion_execution_cpp/validation.hpp"
#include "mtc_core_cpp/clock.hpp"

namespace mtc
{
namespace motion
{

/// 每一步握手的可追溯记录（设计 V0.1 §10.2 要求的日志字段）。
struct HandshakeRecord
{
  std::string command_id;
  std::vector<std::string> steps;

  void mark(const std::string & step, const std::string & detail = "");
};

/// 执行器快照。
struct ExecutorSnapshot
{
  mtc::core::ExecState exec_state = mtc::core::ExecState::IDLE;
  std::optional<std::string> active_command_id;
  uint16_t last_error_code = 0;
  bool motion_state_unknown = false;
  bool stop_confirmed = false;
};

class MotionExecutorCore
{
public:
  explicit MotionExecutorCore(
    MotionAdapterPtr adapter,
    ValidationConfig config = ValidationConfig(), mtc::core::ClockPtr clock = nullptr);

  // -- 只读查询 ----------------------------------------------------------

  const MotionAdapterPtr & adapter() const {return adapter_;}
  mtc::core::MotionCapabilities capabilities() const;
  mtc::core::ExecState exec_state() const {return snapshot().exec_state;}
  bool motion_state_unknown() const {return snapshot().motion_state_unknown;}
  ExecutorSnapshot snapshot() const;

  /// 取得某次命令的握手记录副本；不存在时返回 nullopt。
  std::optional<HandshakeRecord> handshake(const std::string & command_id) const;

  // -- 主流程 ------------------------------------------------------------

  /// 执行一次关节目标运动。绝不抛出（后端异常按"状态未知"处理）。
  mtc::core::MotionResult execute_joint_move(const mtc::core::MotionCommand & command);

  /// Compatibility wrapper: ID supplied -> cancel_goal; no ID -> supervisory_stop.
  ///
  /// **送达不等于停稳**：调用方不得据 request_delivered 直接宣告安全停止。
  /// `in_flight_command_id` 必须匹配内部命令记录；调用方不能覆盖执行所有权。
  mtc::core::StopResult request_stop(
    const std::string & reason,
    std::optional<std::string> in_flight_command_id = std::nullopt);

  /// Cancel exactly this goal. A mismatched/empty ID has no executor/backend side effect.
  mtc::core::StopResult cancel_goal(const std::string & command_id, const std::string & reason);

  /// Explicit supervisory software stop of the current motion; not a safety E-stop.
  mtc::core::StopResult supervisory_stop(const std::string & reason);

  /// 仅在人工现场核实后调用：解除"状态未知"锁定。
  ///
  /// 这是给人用的显式出口，任何自动流程都不得调用它。
  void clear_unknown_after_manual_confirmation(const std::string & note);

private:
  mtc::core::StopResult stop_impl(const std::string & reason,
    const std::optional<std::string> & requested_id);
  bool evidence_valid(const std::optional<mtc::core::StopEvidence> & evidence,
    const std::optional<std::string> & command_id) const;
  mutable std::mutex mutex_;
  mtc::core::ClockPtr clock_;
  mtc::core::ClockPtr adapter_clock_;
  MotionAdapterPtr adapter_;
  ValidationConfig config_;
  mtc::core::ExecState exec_state_ = mtc::core::ExecState::IDLE;
  std::optional<std::string> active_command_id_;
  uint16_t last_error_code_ = 0;
  bool motion_state_unknown_ = false;
  bool stop_confirmed_ = false;  ///< Startup is not stationary evidence.
  bool cancel_requested_ = false;
  size_t stops_pending_ = 0;
  uint64_t generation_ = 0;
  std::optional<std::string> last_command_id_;
  std::set<std::string> executed_ids_;
  std::map<std::string, HandshakeRecord> handshakes_;
};

}  // namespace motion
}  // namespace mtc

#endif  // MTC_MOTION_EXECUTION_CPP__EXECUTOR_CORE_HPP_
