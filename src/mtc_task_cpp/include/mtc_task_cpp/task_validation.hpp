// Copyright 2026 meituan_challenge_ws contributors
//
// 任务请求的规范化与合法性校验（迁移自 Python mtc_task/task_validation.py）。
//
// 规则（设计 §3.1 任务顺序约定 + §4.3 校验）：
//   * task_id 非空；
//   * timeout_ms > 0；
//   * MODE_BASIC：ordered_colors 恰好 1 项，目标 T0；
//   * MODE_SEQUENCE：ordered_colors 恰好 3 项且互不重复，目标按数组次序固定映射 P1/P2/P3；
//   * 所有颜色必须属于 {1,2,3,4}（COLOR_UNKNOWN=0 不是合法口令）；
//   * target_slot（若显式给出）只允许 T0/P1/P2/P3，且必须与模式/次序推导出的工位一致。
//
// 任何一条不满足都返回 INVALID_TASK（100），FSM 侧转 TASK_FAILED，不猜测、不补全。

#ifndef MTC_TASK_CPP__TASK_VALIDATION_HPP_
#define MTC_TASK_CPP__TASK_VALIDATION_HPP_

#include <string>
#include <vector>

#include "mtc_task_cpp/task_types.hpp"

namespace mtc
{
namespace task
{

struct ValidationResult
{
  bool ok = false;
  uint16_t error_code = 0;
  /// 机器可读的失败原因（如 "empty_task_id"），与 Python 侧 reason 逐字一致。
  std::string reason;
  std::string message;
  std::vector<std::string> normalized_slots;
};

/// 按设计约定推导第 index 项的目标工位；非法情形返回 nullopt。
std::optional<std::string> expected_slot_for_index(
  int mode, size_t index, const std::optional<std::string> & explicit_slot = std::nullopt);

/// 全量校验一个 ExecuteTask 请求。
ValidationResult validate_task_request(const TaskRequest & task);

/// 单独校验一个工位名（供上层核对 PickPlace 目标）。
ValidationResult validate_slot_name(const std::string & slot);

}  // namespace task
}  // namespace mtc

#endif  // MTC_TASK_CPP__TASK_VALIDATION_HPP_
