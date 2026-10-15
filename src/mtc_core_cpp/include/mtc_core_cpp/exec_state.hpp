// Copyright 2026 meituan_challenge_ws contributors
//
// 运动执行状态枚举（迁移自 Python mtc_motion_execution/backends.py ExecState）。
//
// 严格区分"已接受 / 正在运动 / 已到位 / 取消中 / 停止确认 / 状态未知"：
// 这些状态在 ExecuteJointMove.action 的 feedback.execution_state 中直接对外暴露，
// 不允许用布尔值把它们压扁（否则取消未确认会被误读为已停止）。

#ifndef MTC_CORE_CPP__EXEC_STATE_HPP_
#define MTC_CORE_CPP__EXEC_STATE_HPP_

#include <string>

namespace mtc
{
namespace core
{

enum class ExecState
{
  IDLE = 0,
  ACCEPTED,
  MOVING,
  SETTLED,
  CANCELLING,
  STOPPED,
  STOP_UNKNOWN,
  TIMEOUT,
  REJECTED,
  FAULT,
};

/// 与 Python 侧 `ExecState.value` 字符串逐字一致（用于 feedback 与日志对齐）。
inline const char * to_string(ExecState state)
{
  switch (state) {
    case ExecState::IDLE: return "IDLE";
    case ExecState::ACCEPTED: return "ACCEPTED";
    case ExecState::MOVING: return "MOVING";
    case ExecState::SETTLED: return "SETTLED";
    case ExecState::CANCELLING: return "CANCELLING";
    case ExecState::STOPPED: return "STOPPED";
    case ExecState::STOP_UNKNOWN: return "STOP_UNKNOWN";
    case ExecState::TIMEOUT: return "TIMEOUT";
    case ExecState::REJECTED: return "REJECTED";
    case ExecState::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

}  // namespace core
}  // namespace mtc

#endif  // MTC_CORE_CPP__EXEC_STATE_HPP_
