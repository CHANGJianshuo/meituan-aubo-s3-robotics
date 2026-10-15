// Copyright 2026 meituan_challenge_ws contributors
//
// 工具策略（末端执行器）抽象与 V1/V2 两代具体实现。
//
// 迁移自 Python src/mtc_manipulation/mtc_manipulation/tool_strategy.py。
//
// 职责边界（重要）：
//   ToolStrategy **只产生运动原语（MotionPrimitive）**，即"有约束的局部运动意图"，
//   不直接调用任何 SDK、不发送关节命令、不掌控运动执行层。
//   真正执行由 FSM 通过 PlannerPort / MotionPort / ToolPort 完成。
//
//   V1 与 V2 的差异必须被封装在本文件（以及测试用的 ToolPort 实现）中，
//   FSM 的 RESOLVE_TARGET -> ... -> VERIFY_PLACED 主流水线与工具型号无关。
//
// V1 差异：side_insert / hook_engage / unload / side_withdraw，无任何电磁或夹爪 IO 依赖。
// V2 差异：top_insert / auto_lock / unload / electromagnetic_unlock / withdraw；
//         解锁前置条件必须是"已确认落座（seated verified）且载荷已卸除（unloaded）"。

#ifndef MTC_MANIPULATION_CPP__TOOL_STRATEGY_HPP_
#define MTC_MANIPULATION_CPP__TOOL_STRATEGY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "mtc_core_cpp/motion_primitive.hpp"
#include "mtc_core_cpp/pose.hpp"
#include "mtc_manipulation_cpp/pick_place_types.hpp"

namespace mtc
{
namespace manipulation
{

/// 解锁前置条件判定结果。
struct UnlockDecision
{
  bool allowed = false;
  uint16_t error_code = 0;
  std::string reason;
  uint32_t pulse_ms = 200;
};

class ToolStrategy
{
public:
  virtual ~ToolStrategy() = default;

  static constexpr double DEFAULT_LINEAR_SPEED = 0.02;
  static constexpr double DEFAULT_ANGULAR_SPEED = 0.1;
  static constexpr double CONTACT_LINEAR_SPEED = 0.005;

  // -- 必须实现 ----------------------------------------------------------

  /// 返回工具类型字符串，必须与 PickPlaceConfig.tool_type 一致。
  virtual std::string type() const = 0;

  virtual mtc::core::MotionPrimitiveList make_acquire_plan(
    const mtc::core::PoseEstimate & ring_pose) const = 0;

  virtual mtc::core::MotionPrimitiveList make_engage_plan(
    const mtc::core::PoseEstimate & ring_pose) const = 0;

  virtual mtc::core::MotionPrimitiveList make_release_plan(
    const std::string & target_slot) const = 0;

  virtual mtc::core::MotionPrimitiveList make_disengage_plan(
    const std::string & target_slot) const = 0;

  /// 是否需要电磁/机构解锁脉冲（V2 为 true，V1 为 false）。
  virtual bool requires_unlock_pulse() const = 0;

  virtual UnlockDecision evaluate_unlock(
    bool seated_verified, bool unloaded_verified) const = 0;

  /// 该类工具会使用到的机构 IO 名称集合（用于离线断言"V1 不触碰电磁 IO"）。
  virtual std::vector<std::string> io_channels() const {return {};}

  // -- 通用实现（各代共用）----------------------------------------------

  /// 低速短距试提：所有工具都做，用于获取挂载证据。
  virtual mtc::core::MotionPrimitiveList make_test_lift_plan(
    const mtc::core::PoseEstimate & ring_pose) const;

  virtual mtc::core::MotionPrimitiveList make_lift_plan(
    const mtc::core::PoseEstimate & ring_pose) const;

  virtual mtc::core::MotionPrimitiveList make_transport_plan(
    const mtc::core::PoseEstimate & ring_pose, const std::string & target_slot) const;

  /// 落座前的预放置（自由空间下降）。
  virtual mtc::core::MotionPrimitiveList make_seat_plan(
    const std::string & target_slot) const;

  /// 受约束低速下降至获得台面承托；不得继续硬顶。
  virtual mtc::core::MotionPrimitiveList make_seat_contact_plan(
    const std::string & target_slot) const;

  virtual mtc::core::MotionPrimitiveList make_retreat_plan(
    const std::string & target_slot) const;

  bool uses_io(const std::string & channel) const;
};

using ToolStrategyPtr = std::shared_ptr<const ToolStrategy>;

/// V1：纯机械舌规。侧向穿环 -> 挂钩 -> 放下卸载 -> 退钩。
///
/// 该策略**不存在解锁动作**，io_channels 为空；任何解锁请求都必须被拒绝。
class PassiveHookV1Strategy : public ToolStrategy
{
public:
  std::string type() const override;
  mtc::core::MotionPrimitiveList make_acquire_plan(
    const mtc::core::PoseEstimate & ring_pose) const override;
  mtc::core::MotionPrimitiveList make_engage_plan(
    const mtc::core::PoseEstimate & ring_pose) const override;
  mtc::core::MotionPrimitiveList make_release_plan(
    const std::string & target_slot) const override;
  mtc::core::MotionPrimitiveList make_disengage_plan(
    const std::string & target_slot) const override;
  bool requires_unlock_pulse() const override;
  UnlockDecision evaluate_unlock(
    bool seated_verified, bool unloaded_verified) const override;
  std::vector<std::string> io_channels() const override {return {};}
};

/// V2：上方插入 + 机械锁止 + 电磁解锁后退出。
///
/// 解锁前置条件（硬约束）：seated_verified AND unloaded_verified，
/// 否则拒绝解锁并返回明确错误（未落座 -> 400；已落座但未卸载 -> 420）。
class MagneticLatchV2Strategy : public ToolStrategy
{
public:
  static constexpr uint32_t DEFAULT_PULSE_MS = 200;

  explicit MagneticLatchV2Strategy(uint32_t pulse_ms = DEFAULT_PULSE_MS);

  std::string type() const override;
  mtc::core::MotionPrimitiveList make_acquire_plan(
    const mtc::core::PoseEstimate & ring_pose) const override;
  mtc::core::MotionPrimitiveList make_engage_plan(
    const mtc::core::PoseEstimate & ring_pose) const override;
  mtc::core::MotionPrimitiveList make_release_plan(
    const std::string & target_slot) const override;
  mtc::core::MotionPrimitiveList make_disengage_plan(
    const std::string & target_slot) const override;
  bool requires_unlock_pulse() const override;
  UnlockDecision evaluate_unlock(
    bool seated_verified, bool unloaded_verified) const override;
  std::vector<std::string> io_channels() const override;

  uint32_t unlock_pulse_ms() const {return pulse_ms_;}

private:
  uint32_t pulse_ms_;
};

/// 按类型名构造策略。未知类型抛出 std::invalid_argument。
ToolStrategyPtr make_strategy(const std::string & tool_type, uint32_t pulse_ms = 200);

/// 提取原语名序列（便于日志与测试断言）。
std::vector<std::string> primitive_names(const mtc::core::MotionPrimitiveList & primitives);

}  // namespace manipulation
}  // namespace mtc

#endif  // MTC_MANIPULATION_CPP__TOOL_STRATEGY_HPP_
