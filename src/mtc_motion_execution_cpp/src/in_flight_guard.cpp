// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_motion_execution_cpp/in_flight_guard.hpp"

namespace mtc
{
namespace motion
{

std::pair<bool, std::string> InFlightGuard::acquire(const std::string & command_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (command_id.empty()) {
    return {false, "command_id 不能为空"};
  }
  if (in_flight_.has_value()) {
    return {
      false,
      "已有在途命令 " + *in_flight_ + "，拒绝并发运动请求 " + command_id};
  }
  in_flight_ = command_id;
  return {true, "acquired"};
}

void InFlightGuard::release(const std::string & command_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (in_flight_.has_value() && *in_flight_ == command_id) {
    in_flight_.reset();
  }
}

void InFlightGuard::force_release()
{
  std::lock_guard<std::mutex> lock(mutex_);
  in_flight_.reset();
}

}  // namespace motion
}  // namespace mtc
