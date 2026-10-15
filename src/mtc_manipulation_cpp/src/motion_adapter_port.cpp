// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_manipulation_cpp/motion_adapter_port.hpp"

#include <utility>

namespace mtc
{
namespace manipulation
{

MotionAdapterPort::MotionAdapterPort(mtc::motion::MotionAdapterPtr adapter,
  mtc::motion::ValidationConfig config, mtc::core::ClockPtr clock)
: adapter_(std::move(adapter)), executor_(adapter_, std::move(config), std::move(clock))
{
}

std::optional<mtc::core::MotionResult> MotionAdapterPort::execute_joint_move(
  const mtc::core::MotionCommand & command)
{
  // 同步契约；规划意图通过后仍须经执行器校验与停稳证据检查。
  if (!command.intent || !command.intent->is_usable() ||
    command.intent->name != command.primitive_name ||
    (!adapter_->capabilities().simulation_only && !command.intent->target_pose)) {
    mtc::core::MotionResult failure;
    failure.exec_state = mtc::core::ExecState::REJECTED;
    failure.error_code = mtc::core::ErrorCodes::PLANNING_FAILED;
    failure.message = "missing executable planning intent or calibrated 6D target";
    return failure;
  }
  return executor_.execute_joint_move(command);
}

mtc::core::StopResult MotionAdapterPort::request_stop(const std::string & reason)
{
  // 执行器检查独立证据；后端受理停止请求不构成停稳。
  return executor_.request_stop(reason);
}

}  // namespace manipulation
}  // namespace mtc
