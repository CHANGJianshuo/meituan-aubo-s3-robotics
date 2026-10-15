// Copyright 2026 meituan_challenge_ws contributors
//
// 规划层配置：原语 -> 规划方式表、工位位姿、TCP、速度缩放与超时策略。
//
// 数值默认来自 config/motion_profiles.yaml 与 config/manipulation.yaml（均为占位值），
// 工位与 TCP 由韩明赫（Calibration/TF/TCP）提供实测值后替换。
// 任一几何输入不是 MEASURED 时，输出强制 simulation_only=true。

#ifndef MTC_MOVEIT_PLANNER__PLANNER_CONFIG_HPP_
#define MTC_MOVEIT_PLANNER__PLANNER_CONFIG_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "mtc_core_cpp/pose.hpp"
#include "mtc_manipulation_cpp/pick_place_types.hpp"
#include "mtc_moveit_planner/planning_backend.hpp"
#include "mtc_moveit_planner/pose_math.hpp"
#include "mtc_moveit_planner/tool_model.hpp"

namespace mtc
{
namespace moveit_planner
{

/// 标定状态，与 TF_AND_CALIBRATION_HANDOFF 的 MEASURED / ESTIMATED / UNKNOWN 对齐；
/// PLACEHOLDER 表示规划层为仿真临时填写的值。
enum class CalibrationStatus
{
  MEASURED,
  ESTIMATED,
  PLACEHOLDER,
  UNKNOWN,
};

const char * to_string(CalibrationStatus status);
std::optional<CalibrationStatus> calibration_status_from_string(const std::string & text);

/// 原语目标如何求得。
enum class GoalKind
{
  RING_STANDOFF,  ///< 提环位姿沿插入轴后退 distance_m（预对准位）
  RELATIVE_UP,    ///< 从起点末端位姿沿 planning frame +Z 平移 distance_m
  SLOT_OFFSET,    ///< 工位位姿上方 distance_m，保持当前工具姿态
};

/// 原语之前存在、但不由规划器输出的工具动作（ToolPort 负责）。
/// 规划器只能**假设**其结束位置来计算后续起点，并在报告中写明。
enum class ToolGapBefore
{
  NONE,
  ENGAGE_INSERT,  ///< 沿插入轴前进 gap_distance_m（hook_engage / auto_lock）
  WITHDRAW,       ///< 沿插入轴后退 gap_distance_m（side_withdraw / withdraw）
};

const char * to_string(GoalKind kind);
const char * to_string(ToolGapBefore gap);

struct PrimitiveSpec
{
  MotionMode mode = MotionMode::FREE_SPACE;
  GoalKind goal = GoalKind::RELATIVE_UP;
  double distance_m = 0.0;
  ToolGapBefore gap_before = ToolGapBefore::NONE;
  double gap_distance_m = 0.0;
  /// 执行该原语的 PickPlace 状态；用其看门狗限制轨迹时长。
  mtc::manipulation::PickPlaceState executing_state = mtc::manipulation::PickPlaceState::LIFT;
  /// 接触类：只作为规划意图 / 仿真验证，不代表可安全执行。
  bool contact_intent_only = false;
  /// 规划该原语时末端挂着电池（试提之后、释放之前）。
  bool carries_payload = false;
};

/// 电池载荷的规划几何（PLACEHOLDER，待实物尺寸）。
struct PayloadSpec
{
  bool enabled = true;
  mtc::core::Vector3 size_m{0.06, 0.06, 0.10};
  /// 载荷中心位于钩尖（TCP）正下方（planning frame -Z）的距离：电池靠重力挂在提环下方。
  double hang_below_tcp_m = 0.07;
  std::vector<std::string> touch_links{"ee_link", "wrist3_Link"};
  CalibrationStatus status = CalibrationStatus::PLACEHOLDER;
};

struct SlotSpec
{
  mtc::core::Pose6D pose;  ///< 工位中心（落座时 TCP 所在高度为 z + 0）
  CalibrationStatus status = CalibrationStatus::UNKNOWN;
};

struct PlannerConfig
{
  /// FSM 期望的关节顺序（PickPlaceConfig::joint_names）。
  std::vector<std::string> joint_names = mtc::manipulation::PickPlaceConfig().joint_names;

  double velocity_scaling = 0.1;       ///< config/manipulation.yaml default_velocity_scaling
  double acceleration_scaling = 0.1;   ///< config/manipulation.yaml default_acceleration_scaling
  double scaling_cap = 0.2;            ///< 执行层与 PickPlaceConfig 的硬上限

  double planning_time_s = 1.0;        ///< PLAN_ACQUIRE 看门狗仅 2.0 s，单次必须小于它
  double cartesian_step_m = 0.005;
  double min_cartesian_fraction = 1.0; ///< 直线段必须完整，不接受部分路径

  double timeout_factor = 1.5;         ///< timeout_ms = duration * factor + margin
  uint32_t timeout_margin_ms = 1000;
  double state_timeout_usage = 0.8;    ///< 轨迹时长不得超过对应状态看门狗的 80%
  std::map<mtc::manipulation::PickPlaceState, double> state_timeouts =
    mtc::manipulation::default_state_timeouts();

  /// 额外关节位置限位（rad，[min, max]）。MoveIt 只用 URDF 限位；
  /// aubo.ros2_control.xacro 中 foreArm/wrist1/wrist3 的指令限位更窄，在此复核。
  std::map<std::string, std::pair<double, double>> joint_position_limits;

  PayloadSpec payload;
  /// 静态场景障碍物（桌面等），由工具在规划前写入 move_group。
  std::vector<SceneBox> scene_boxes;

  /// FREE_SPACE 原语先试末端直线（最短、可预测），不可行再用 OMPL。
  bool prefer_straight_line_free_space = true;

  /// 提环 / TCP 局部坐标中的插入方向轴（TF_CONVENTIONS §2.1 提案：+Z）。
  int insertion_axis = 2;

  /// ee_link -> hook_tcp。未标定时为单位变换；tool.enabled 时由工具模型计算。
  Transform ee_to_tcp;
  CalibrationStatus tcp_status = CalibrationStatus::PLACEHOLDER;

  /// 末端工具模型（CAD 几何）：启用时决定 TCP，并把工具外形作为碰撞体随每次规划下发。
  ToolModelSpec tool;
  CalibrationStatus tool_status = CalibrationStatus::PLACEHOLDER;

  /// 钩口朝上：RING_STANDOFF 目标只取提环的插入轴，绕插入轴的滚转改为 TCP x 指向世界 +Z。
  /// 插入轴与竖直方向夹角小于 hook_up_min_tilt_rad（如 V2 竖直下插）时不调整。
  bool hook_up_alignment = true;
  double hook_up_min_tilt_rad = 0.5236;   ///< 30°

  std::map<std::string, SlotSpec> slots;
  std::map<std::string, PrimitiveSpec> primitives;
  std::set<std::string> tool_owned_primitives;

  /// 允许使用非 MEASURED 的几何输入（今晚 Mock 必须为 true）。
  bool allow_uncalibrated_geometry = true;
  /// 无论标定状态如何都标记 simulation_only（团队批准前保持 true）。
  bool force_simulation_only = true;
};

/// 默认配置：原语表 + 工具原语清单 + 占位工位（仅仿真）。
PlannerConfig default_planner_config();

/// 按工具行程更新原语表：预对准退让 = 插入行程，快速接近再多退 approach_backoff，
/// 试提前的插入假设 = 插入行程，撤离前的退钩假设 = 退钩行程。
void apply_tool_travel(
  PlannerConfig & config, double insert_travel_m, double withdraw_travel_m,
  double approach_backoff_m);

}  // namespace moveit_planner
}  // namespace mtc

#endif  // MTC_MOVEIT_PLANNER__PLANNER_CONFIG_HPP_
