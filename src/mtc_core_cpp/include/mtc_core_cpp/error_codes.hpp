// Copyright 2026 meituan_challenge_ws contributors
//
// 错误码集中定义（C++ 单一来源）。
//
// 数值与 src/mtc_interfaces/msg/ErrorCodes.msg **逐项对齐**，不可擅自变更。
// 变更语义必须同步更新 docs/migration/INTERFACE_CHANGELOG.md 与
// docs/migration/INTERFACE_COMPAT_AUDIT_CPP.md。
//
// 迁移说明：Python 侧在 5 个模块中各自镜像了同一套常量
// （task_fsm.py / pick_place_fsm.py / interlocks.py / backends.py / bridge_core.py）。
// C++ 侧刻意收敛为**唯一定义**，并通过 test_error_codes_msg_parity 在编译期外
// 与 .msg 文件做一致性校验，避免再次散落。

#ifndef MTC_CORE_CPP__ERROR_CODES_HPP_
#define MTC_CORE_CPP__ERROR_CODES_HPP_

#include <cstdint>
#include <string>

namespace mtc
{
namespace core
{

/// 错误码常量容器（对齐 mtc_interfaces/msg/ErrorCodes.msg）。
struct ErrorCodes
{
  static constexpr uint16_t OK = 0;
  static constexpr uint16_t INVALID_TASK = 100;
  static constexpr uint16_t TARGET_NOT_FOUND = 110;
  static constexpr uint16_t POSE_STALE = 120;
  static constexpr uint16_t PLANNING_FAILED = 200;
  static constexpr uint16_t EXECUTION_REJECTED = 210;
  static constexpr uint16_t MOTION_TIMEOUT = 220;
  static constexpr uint16_t MOTION_STATUS_UNKNOWN = 230;
  static constexpr uint16_t ENGAGE_FAILED = 300;
  static constexpr uint16_t ATTACH_NOT_VERIFIED = 310;
  static constexpr uint16_t OBJECT_DROPPED = 320;
  static constexpr uint16_t SEAT_NOT_VERIFIED = 400;
  static constexpr uint16_t UNLOCK_FAILED = 410;
  static constexpr uint16_t RELEASE_NOT_VERIFIED = 420;
  static constexpr uint16_t PLACEMENT_FAILED = 430;
  static constexpr uint16_t SAFETY_INTERLOCK = 500;
  static constexpr uint16_t HARDWARE_FAULT = 510;
  static constexpr uint16_t CANCEL_NOT_CONFIRMED = 520;

  /// 人类可读名称；未知码返回 "UNKNOWN_<n>"。
  static std::string name_of(uint16_t code);

  /// 该码是否在 ErrorCodes.msg 中已登记（用于自检，防止魔法数字）。
  static bool is_known(uint16_t code);
};

}  // namespace core
}  // namespace mtc

#endif  // MTC_CORE_CPP__ERROR_CODES_HPP_
