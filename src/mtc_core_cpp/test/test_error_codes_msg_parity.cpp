// Copyright 2026 meituan_challenge_ws contributors
//
// 一致性校验：C++ 错误码常量 vs mtc_interfaces/msg/ErrorCodes.msg 原文。
//
// 目的：C++ 侧把错误码收敛为唯一定义后，必须有一个**自动**手段证明它与
// 接口定义仍然一致，否则收敛会退化成"另一种散落"。
//
// 行为约定（需求 7：不得把未执行的测试声明为通过）：
//   若 .msg 文件不可读（例如独立构建本包、未同步 src/），测试显式 GTEST_SKIP，
//   并打印原因；**绝不**在未校验的情况下返回通过。

#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

#include "mtc_core_cpp/error_codes.hpp"

#ifndef MTC_INTERFACES_MSG_DIR
#error "MTC_INTERFACES_MSG_DIR must be defined by CMake"
#endif

namespace
{

/// 解析 ErrorCodes.msg 中的 `uint16 NAME=值` 行。
std::map<std::string, uint16_t> parse_msg_constants(
  const std::string & path, bool & opened, std::string & error)
{
  std::map<std::string, uint16_t> constants;
  std::ifstream stream(path);
  if (!stream.is_open()) {
    opened = false;
    error = "cannot open " + path;
    return constants;
  }
  opened = true;

  std::string line;
  while (std::getline(stream, line)) {
    // 去掉行尾注释
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }
    std::istringstream parts(line);
    std::string type;
    std::string assignment;
    if (!(parts >> type >> assignment)) {
      continue;
    }
    if (type != "uint16") {
      continue;
    }
    const auto eq = assignment.find('=');
    if (eq == std::string::npos) {
      continue;
    }
    const std::string name = assignment.substr(0, eq);
    const std::string value_text = assignment.substr(eq + 1);
    if (name.empty() || value_text.empty()) {
      continue;
    }
    try {
      constants[name] = static_cast<uint16_t>(std::stoul(value_text));
    } catch (const std::exception &) {
      continue;
    }
  }
  return constants;
}

}  // namespace

TEST(ErrorCodesMsgParity, ConstantsMatchInterfacesMsg)
{
  const std::string path = std::string(MTC_INTERFACES_MSG_DIR) + "/ErrorCodes.msg";

  bool opened = false;
  std::string error;
  const auto constants = parse_msg_constants(path, opened, error);

  if (!opened) {
    GTEST_SKIP() << "接口定义不可读，跳过一致性校验（未校验，不视为通过）：" << error;
  }
  ASSERT_FALSE(constants.empty()) << "ErrorCodes.msg 解析结果为空：" << path;

  // C++ 常量 -> .msg 常量名
  const std::map<std::string, uint16_t> cpp_values = {
    {"OK", mtc::core::ErrorCodes::OK},
    {"INVALID_TASK", mtc::core::ErrorCodes::INVALID_TASK},
    {"TARGET_NOT_FOUND", mtc::core::ErrorCodes::TARGET_NOT_FOUND},
    {"POSE_STALE", mtc::core::ErrorCodes::POSE_STALE},
    {"PLANNING_FAILED", mtc::core::ErrorCodes::PLANNING_FAILED},
    {"EXECUTION_REJECTED", mtc::core::ErrorCodes::EXECUTION_REJECTED},
    {"MOTION_TIMEOUT", mtc::core::ErrorCodes::MOTION_TIMEOUT},
    {"MOTION_STATUS_UNKNOWN", mtc::core::ErrorCodes::MOTION_STATUS_UNKNOWN},
    {"ENGAGE_FAILED", mtc::core::ErrorCodes::ENGAGE_FAILED},
    {"ATTACH_NOT_VERIFIED", mtc::core::ErrorCodes::ATTACH_NOT_VERIFIED},
    {"OBJECT_DROPPED", mtc::core::ErrorCodes::OBJECT_DROPPED},
    {"SEAT_NOT_VERIFIED", mtc::core::ErrorCodes::SEAT_NOT_VERIFIED},
    {"UNLOCK_FAILED", mtc::core::ErrorCodes::UNLOCK_FAILED},
    {"RELEASE_NOT_VERIFIED", mtc::core::ErrorCodes::RELEASE_NOT_VERIFIED},
    {"PLACEMENT_FAILED", mtc::core::ErrorCodes::PLACEMENT_FAILED},
    {"SAFETY_INTERLOCK", mtc::core::ErrorCodes::SAFETY_INTERLOCK},
    {"HARDWARE_FAULT", mtc::core::ErrorCodes::HARDWARE_FAULT},
    {"CANCEL_NOT_CONFIRMED", mtc::core::ErrorCodes::CANCEL_NOT_CONFIRMED},
  };

  for (const auto & entry : cpp_values) {
    const auto it = constants.find(entry.first);
    ASSERT_NE(it, constants.end()) << "ErrorCodes.msg 缺少常量：" << entry.first;
    EXPECT_EQ(it->second, entry.second)
      << "错误码 " << entry.first << " 在 C++ 与 .msg 之间不一致";
  }

  // 反向检查：.msg 中不应存在 C++ 未登记的常量，防止漏迁移。
  for (const auto & entry : constants) {
    EXPECT_TRUE(cpp_values.find(entry.first) != cpp_values.end())
      << "ErrorCodes.msg 中存在 C++ 未登记的常量：" << entry.first;
  }
}
