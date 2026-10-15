// Copyright 2026 meituan_challenge_ws contributors
//
// 执行前校验（迁移自 Python backends.py ValidationConfig + validate_joint_move）。
//
// 核心原则：任何不通过都必须**拒绝**，不允许把不支持的运动表示悄悄"变形"成
// 另一种表示去执行。

#ifndef MTC_MOTION_EXECUTION_CPP__VALIDATION_HPP_
#define MTC_MOTION_EXECUTION_CPP__VALIDATION_HPP_

#include <string>
#include <utility>
#include <vector>

#include "mtc_core_cpp/motion_primitive.hpp"
#include "mtc_core_cpp/results.hpp"
#include "mtc_core_cpp/timed_trajectory.hpp"

namespace mtc
{
namespace motion
{

struct ValidationConfig
{
  /// 期望的关节名与顺序。为空表示不做顺序校验。
  ///
  /// 默认值与官方 AUBO 驱动 URDF 的关节命名一致
  /// （见 docs/migration/AUBO_DRIVER_FAKE_HARDWARE_FIX_AUDIT.md §6）。
  std::vector<std::string> expected_joint_names = {
    "shoulder_joint", "upperArm_joint", "foreArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint",
  };
  double max_velocity_scaling = 0.2;
  double max_acceleration_scaling = 0.2;
  /// 每关节 (low, high)；仅 simulation_only 后端允许为空。
  std::vector<std::pair<double, double>> joint_limit_rad;
  double settle_position_tolerance_rad = 0.001;
  double settle_velocity_tolerance_rad_s = 0.001;
  double settle_duration_s = 0.2;
  double feedback_max_age_s = 0.3;
  /// 离线默认值，不代表 AUBO 实机认证参数；可执行后端须完成配置审核。
  std::vector<double> max_joint_velocity_rad_s = {1, 1, 1, 1, 1, 1};
  std::vector<double> max_joint_acceleration_rad_s2 = {2, 2, 2, 2, 2, 2};
};

/// 执行前校验结论。
struct ValidationOutcome
{
  bool ok = false;
  uint16_t error_code = 0;
  std::string message;
};

/// 校验一次关节目标运动请求。
///
/// 返回 (是否通过, 错误码, 说明)。
ValidationOutcome validate_joint_move(
  const mtc::core::MotionCommand & request,
  const ValidationConfig & config,
  const mtc::core::MotionCapabilities & capabilities);

/// 校验一条时间参数化轨迹（第二阶段 ROS 2 绑定前的守门人）。
ValidationOutcome validate_timed_trajectory(
  const mtc::core::TimedTrajectory & trajectory,
  const ValidationConfig & config,
  const mtc::core::MotionCapabilities & capabilities);

}  // namespace motion
}  // namespace mtc

#endif  // MTC_MOTION_EXECUTION_CPP__VALIDATION_HPP_
