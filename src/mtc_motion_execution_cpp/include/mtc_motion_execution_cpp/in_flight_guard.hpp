// Copyright 2026 meituan_challenge_ws contributors
//
// 唯一执行权（single in-flight command）。
//
// 迁移自 Python src/mtc_motion_execution/mtc_motion_execution/backends.py InFlightGuard。
//
// 设计依据：设计 V0.1 §2"控制命令所有权"与 §7.2"正在执行运动时禁止第二个 Goal"。
// 需求 5 明确："同时只允许一个有效运动目标，必须防止并发 Goal 冲突。"
// 本类是实现该约束的**唯一**位置，任何适配器都不得绕过。

#ifndef MTC_MOTION_EXECUTION_CPP__IN_FLIGHT_GUARD_HPP_
#define MTC_MOTION_EXECUTION_CPP__IN_FLIGHT_GUARD_HPP_

#include <optional>
#include <mutex>
#include <string>
#include <utility>

namespace mtc
{
namespace motion
{

class InFlightGuard
{
public:
  /// 尝试取得执行权。返回 (是否成功, 说明)。
  std::pair<bool, std::string> acquire(const std::string & command_id);

  /// 释放执行权；仅当 command_id 与当前持有者一致时才生效。
  void release(const std::string & command_id);

  bool is_free() const {return !in_flight().has_value();}
  std::optional<std::string> in_flight() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return in_flight_;
  }

  /// 仅供人工现场核实后的显式复位（用于"状态未知已解除"的场景）。
  /// 自动流程不得调用。
  void force_release();

private:
  mutable std::mutex mutex_;
  std::optional<std::string> in_flight_;
};

}  // namespace motion
}  // namespace mtc

#endif  // MTC_MOTION_EXECUTION_CPP__IN_FLIGHT_GUARD_HPP_
