// Copyright 2026 meituan_challenge_ws contributors
//
// 离线 Mock 运动适配器 + 真实后端占位（Disabled）。
//
// 迁移自 Python backends.py MockMotionBackend / DisabledMotionBackend。
//
// 安全声明：MockMotionAdapter **永远不会连接任何真实设备**；
// DisabledMotionAdapter 始终拒绝，绝不产生任何真实运动。

#ifndef MTC_MOTION_EXECUTION_CPP__MOCK_MOTION_ADAPTER_HPP_
#define MTC_MOTION_EXECUTION_CPP__MOCK_MOTION_ADAPTER_HPP_

#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "mtc_core_cpp/clock.hpp"
#include "mtc_core_cpp/timed_trajectory.hpp"
#include "mtc_motion_execution_cpp/in_flight_guard.hpp"
#include "mtc_motion_execution_cpp/motion_adapter.hpp"
#include "mtc_motion_execution_cpp/validation.hpp"

namespace mtc
{
namespace motion
{

/// Mock 适配器的可配置行为，用于覆盖
/// "到位 / 超时 / 取消已确认 / 取消未确认 / 状态未知 / 不支持轨迹"等路径。
struct MockMotionAdapterConfig
{
  double travel_time_s = 0.5;
  bool timeout = false;
  bool status_unknown = false;
  bool cancel_confirms = true;
  /// 故障注入点："" / "status_unknown" / "timeout" / "reject"
  std::string fail_at;
  std::string backend_name = "mock";
  bool joint_goal_supported = true;
  bool timed_trajectory_supported = false;
  bool cartesian_motion_supported = false;
  bool cancel_supported = true;
  std::string detail = "离线 Mock 适配器；不具备轨迹跟踪与笛卡尔运动能力";
};

class MockMotionAdapter : public MotionAdapter
{
public:
  /// Missing clock creates a private offline ManualClock, exposed by execution_clock().
  /// Explicit clocks must be ManualClock; SteadyClock is rejected, never virtually advanced.
  explicit MockMotionAdapter(
    MockMotionAdapterConfig config = MockMotionAdapterConfig(),
    mtc::core::ClockPtr clock = nullptr,
    ValidationConfig validation = ValidationConfig());

  mtc::core::MotionCapabilities capabilities() const override;
  mtc::core::ClockPtr execution_clock() const override {return clock_;}
  mtc::core::MotionResult execute_joint_move(const mtc::core::MotionCommand & command) override;
  mtc::core::StopResult request_stop(const std::string & reason) override;
  mtc::core::StopResult request_stop_for_command(const std::string & reason,
    const std::optional<std::string> & command_id) override;
  mtc::core::JointFeedback joint_feedback() const override;
  std::optional<bool> explicit_stop_confirmation() const override;
  std::optional<mtc::core::StopEvidence> stop_evidence() const override;
  mtc::core::MotionResult execute_trajectory(const mtc::core::TimedTrajectory & trajectory) override;

  /// 最近一次停止请求是否已被确认停稳（显式语义，供上层判断）。
  bool stop_confirmed_flag() const {return explicit_stop_confirmation().value_or(false);}

  /// 可观测的执行轨迹记录，便于断言"没有把轨迹拆成多次调用"。
  std::vector<mtc::core::MotionCommand> call_log() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return call_log_;
  }
  std::vector<mtc::core::TimedTrajectory> trajectory_call_log() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return trajectory_call_log_;
  }

  /// 是否仍有未结束的在途命令。
  bool has_in_flight() const {return !guard_.is_free();}

  // ---------------------------------------------------------------------
  // 测试钩子（仅供单线程单元测试构造"运动在途"场景）
  //
  // 生产代码**不得**调用：真实在途状态必须由 execute_joint_move 的
  // 实际执行过程产生，人为声明在途会掩盖真实并发语义。
  // ---------------------------------------------------------------------

  /// 人为占用执行权并声明在途命令。已被占用时返回 false。
  bool force_begin_in_flight(const std::string & command_id);

  /// 清除人为声明的在途命令。
  void force_end_in_flight();

private:
  mutable std::recursive_mutex mutex_;
  std::optional<mtc::core::StopEvidence> evidence_;
  std::set<std::string> cancelled_ids_;
  /// 推进模拟时间：仅当注入的是 ManualClock 时生效，
  /// 以便单元测试在不睡眠的前提下验证超时。
  void advance_time(double seconds);

  MockMotionAdapterConfig config_;
  mtc::core::ClockPtr clock_;
  ValidationConfig validation_;
  ValidationOutcome last_validation_;
  InFlightGuard guard_;
  mtc::core::MotionCapabilities capabilities_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  bool cancel_requested_ = false;
  bool stop_confirmed_ = false;
  std::optional<std::string> active_command_;
  std::vector<mtc::core::MotionCommand> call_log_;
  std::vector<mtc::core::TimedTrajectory> trajectory_call_log_;
};

/// 真实后端占位：始终拒绝，绝不产生任何真实运动。
///
/// 任何试图在未授权情况下驱动真实设备的调用都会在这里被明确拒绝，
/// 而不是静默成功或抛出难以定位的异常。
class DisabledMotionAdapter : public MotionAdapter
{
public:
  explicit DisabledMotionAdapter(
    std::string reason = "真实运动后端默认禁用，需现场明确授权与实测验证");

  mtc::core::MotionCapabilities capabilities() const override;
  mtc::core::MotionResult execute_joint_move(const mtc::core::MotionCommand & command) override;
  mtc::core::StopResult request_stop(const std::string & reason) override;
  mtc::core::JointFeedback joint_feedback() const override;

  const std::string & reason() const {return reason_;}

private:
  std::string reason_;
};

/// 按名称构造适配器。未知名称一律回到 Disabled，**绝不默认连真实设备**。
MotionAdapterPtr make_adapter(
  const std::string & name,
  MockMotionAdapterConfig config = MockMotionAdapterConfig(),
  mtc::core::ClockPtr clock = nullptr,
  ValidationConfig validation = ValidationConfig());

}  // namespace motion
}  // namespace mtc

#endif  // MTC_MOTION_EXECUTION_CPP__MOCK_MOTION_ADAPTER_HPP_
