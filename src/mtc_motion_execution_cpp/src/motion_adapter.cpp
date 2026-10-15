// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_motion_execution_cpp/motion_adapter.hpp"

#include "mtc_core_cpp/error_codes.hpp"

namespace mtc
{
namespace motion
{

mtc::core::MotionResult MotionAdapter::execute_trajectory(
  const mtc::core::TimedTrajectory & trajectory)
{
  // 默认实现刻意拒绝，而不是降级为多次单点调用。
  //
  // 理由（需求 4）："不得把多点轨迹拆成大量 ExecuteJointMove 调用"。
  // 若在此静默降级，能力缺口会被掩盖，时间同步语义会被破坏。
  (void)trajectory;
  mtc::core::MotionResult result;
  result.success = false;
  result.arrived = false;
  result.stop_confirmed = false;
  result.error_code = mtc::core::ErrorCodes::EXECUTION_REJECTED;
  result.message =
    "该运动适配器未实现时间参数化轨迹执行；"
    "禁止把多点轨迹拆成多次单点调用，请改用支持 timed_trajectory 的后端";
  result.exec_state = mtc::core::ExecState::REJECTED;
  return result;
}

}  // namespace motion
}  // namespace mtc
