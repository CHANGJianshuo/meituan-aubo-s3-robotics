// Copyright 2026 meituan_challenge_ws contributors
//
// ToolState.msg 的常量镜像 + 证据等级（有序，可比较）。
//
// 安全语义（迁移自 pick_place_fsm.py 安全语义 2）：
//   state 是"当前估计"，evidence_level 是"支撑该估计的证据强度"，二者不得混为一谈。
//   只有 evidence_level >= GEOMETRY 时，才允许把 state 当作可用证据。

#ifndef MTC_CORE_CPP__TOOL_STATE_HPP_
#define MTC_CORE_CPP__TOOL_STATE_HPP_

#include <cstdint>
#include <string>

namespace mtc
{
namespace core
{

/// 对齐 mtc_interfaces/msg/ToolState.msg 的 state 常量。
struct ToolStateCode
{
  static constexpr uint8_t STATE_UNKNOWN = 0;
  static constexpr uint8_t STATE_DETACHED = 1;
  static constexpr uint8_t STATE_ENGAGING = 2;
  static constexpr uint8_t STATE_ATTACHED = 3;
  static constexpr uint8_t STATE_RELEASING = 4;
  static constexpr uint8_t STATE_FAULT = 5;

  static std::string name_of(uint8_t state);
};

/// 对齐 mtc_interfaces/msg/ToolState.msg 的 evidence_level 常量。
/// 数值越大证据越强，可直接做 >= 比较（与 Python EvidenceLevel 一致）。
struct EvidenceLevel
{
  static constexpr uint8_t NONE = 0;
  static constexpr uint8_t COMMAND_ONLY = 1;
  static constexpr uint8_t GEOMETRY = 2;
  static constexpr uint8_t SENSOR_OR_VISION = 3;

  static std::string name_of(uint8_t level);
};

/// 工具类型字符串（与 PickPlace.action / 设计 §5 一致）。
struct ToolTypes
{
  static constexpr const char * PASSIVE_HOOK_V1 = "passive_hook_v1";
  static constexpr const char * MAGNETIC_LATCH_V2 = "magnetic_latch_v2";
};

}  // namespace core
}  // namespace mtc

#endif  // MTC_CORE_CPP__TOOL_STATE_HPP_
