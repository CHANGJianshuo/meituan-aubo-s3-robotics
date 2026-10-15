// Copyright 2026 meituan_challenge_ws contributors
//
// 规划后端抽象：把"MoveIt 怎么规划"与"PlannerPort 怎么组装结果"分开。
//
// MoveItPlannerPort 只依赖本接口，因此可以用假后端做离线 GTest；
// 真正的 MoveGroupInterface + tf2 实现在 move_group_backend.hpp（ROS 外壳）。

#ifndef MTC_MOVEIT_PLANNER__PLANNING_BACKEND_HPP_
#define MTC_MOVEIT_PLANNER__PLANNING_BACKEND_HPP_

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "mtc_core_cpp/pose.hpp"
#include "mtc_moveit_planner/pose_math.hpp"
#include "mtc_moveit_planner/trajectory_conversion.hpp"

namespace mtc
{
namespace moveit_planner
{

/// 规划组的关节状态快照（名称与位置按索引对应，不假定顺序）。
struct JointStateSnapshot
{
  std::vector<std::string> names;
  std::vector<double> positions;

  /// 非空、名称与位置数量一致、关节名非空且不重复（重名会让按名取值产生歧义）。
  bool shape_is_valid() const
  {
    if (names.empty() || names.size() != positions.size()) {
      return false;
    }
    std::set<std::string> seen;
    for (const auto & name : names) {
      if (name.empty() || !seen.insert(name).second) {
        return false;
      }
    }
    return true;
  }
};

/// 随末端一起运动的载荷（电池）碰撞体：长方体，位姿相对规划末端 link（ee_link）固定。
/// 只附着在**规划起点状态**上参与碰撞检查，不修改 move_group 的全局规划场景。
enum class BodyShape
{
  BOX,        ///< size_m = 长 × 宽 × 高
  CYLINDER,   ///< size_m = (直径, 直径, 长度)，轴沿局部 z
};

/// 附着在 ee_link 上的碰撞体（电池载荷、工具外形）。
struct AttachedPayload
{
  std::string id = "mtc_moveit_planner/battery_payload";  ///< 带本模块前缀，见 scene_ids.hpp
  BodyShape shape = BodyShape::BOX;
  mtc::core::Vector3 size_m{0.06, 0.06, 0.10};
  Transform ee_to_payload;                 ///< ee_link -> 载荷中心
  std::vector<std::string> touch_links{"ee_link", "wrist3_Link"};
};

/// 静态场景障碍物（长方体），写入 move_group 规划场景。
struct SceneBox
{
  std::string id;                          ///< 本地 ID（如 "table"），写入场景时加本模块前缀
  mtc::core::Pose6D pose;                  ///< 中心位姿（frame_id 必须可 TF 到 planning frame）
  mtc::core::Vector3 size_m;
};

enum class MotionMode
{
  FREE_SPACE,  ///< OMPL 关节空间规划到末端位姿
  CARTESIAN,   ///< 末端直线（computeCartesianPath）
};

const char * to_string(MotionMode mode);

struct BackendRequest
{
  std::string label;                ///< 原语名或"simulated_*"，仅用于日志
  MotionMode mode = MotionMode::FREE_SPACE;
  mtc::core::Pose6D ee_goal;        ///< 规划末端 link（ee_link）的目标位姿，已做 TCP 补偿
  JointStateSnapshot start;         ///< 显式起点；不使用"隐式当前状态"
  double velocity_scaling = 0.1;
  double acceleration_scaling = 0.1;
  double max_linear_speed_m_s = 0.0;     ///< <=0 表示不限制
  double max_angular_speed_rad_s = 0.0;  ///< <=0 表示不限制
  double planning_time_s = 1.0;
  double cartesian_step_m = 0.005;
  double min_cartesian_fraction = 1.0;
  bool avoid_collisions = true;
  /// 规划时随末端一起运动的载荷；空表示末端无载荷。
  std::optional<AttachedPayload> payload;
  /// 工具外形碰撞体：每次规划都附着在起点状态上（不改全局场景）。
  std::vector<AttachedPayload> tool_bodies;
  /// FREE_SPACE 时先尝试末端直线（完整、无碰撞、满足关节约束才采用），失败再用 OMPL。
  bool prefer_straight_line = false;
  /// 额外关节位置约束（rad）：作为 MoveIt 路径约束下发，使解落在控制器限位内。
  std::map<std::string, std::pair<double, double>> joint_bounds;
  /// FREE_SPACE 的目标构型限制；不限制起点或中途构型。空表示不限。
  /// 用于空载时回到后续笛卡尔运动所需的解支，不能扩大模型/控制器限位。
  std::map<std::string, std::pair<double, double>> goal_joint_bounds;
};

struct BackendResult
{
  bool ok = false;
  std::string reason;               ///< 失败分类：NO_PLAN / CARTESIAN_INCOMPLETE / ...
  std::string message;
  double planning_time_s = 0.0;
  double cartesian_fraction = 0.0;  ///< 仅 CARTESIAN 有意义
  double time_scale_applied = 1.0;  ///< 为满足末端线速度/角速度上限而做的均匀拉伸倍数
  std::string method;               ///< cartesian_preferred / joint_interpolated_ik / ompl_seeded_ik / ompl / cartesian
  double max_linear_speed_m_s = 0.0;   ///< 拉伸后实测末端最大线速度
  double max_angular_speed_rad_s = 0.0;
  RawTrajectory trajectory;         ///< 已时间参数化（含速度/加速度）
  JointStateSnapshot end_state;
};

class PlanningBackend
{
public:
  virtual ~PlanningBackend() = default;

  /// 规划参考坐标系（MoveIt planning frame）。
  virtual std::string planning_frame() const = 0;

  /// 规划末端 link 名（PlannerPort 的 ee 目标对应此 link）。
  virtual std::string end_effector_link() const = 0;

  /// 当前机器人状态（来自 /joint_states）；不可得返回 nullopt。
  virtual std::optional<JointStateSnapshot> current_state() = 0;

  /// 给定关节状态下末端 link 在 planning frame 中的位姿（正运动学）。
  virtual std::optional<mtc::core::Pose6D> end_effector_pose(const JointStateSnapshot & state) = 0;

  /// 用 TF 把位姿变换到 planning frame；TF 不连通 / 超时返回 nullopt（绝不复制 XYZ 兜底）。
  virtual std::optional<mtc::core::Pose6D> transform_to_planning_frame(
    const mtc::core::Pose6D & pose) = 0;

  virtual BackendResult plan(const BackendRequest & request) = 0;

  /// 用静态障碍物替换本后端此前写入的场景对象；返回是否成功（默认实现：不支持）。
  virtual bool apply_scene(const std::vector<SceneBox> & boxes)
  {
    return boxes.empty();
  }
};

using PlanningBackendPtr = std::shared_ptr<PlanningBackend>;

}  // namespace moveit_planner
}  // namespace mtc

#endif  // MTC_MOVEIT_PLANNER__PLANNING_BACKEND_HPP_
