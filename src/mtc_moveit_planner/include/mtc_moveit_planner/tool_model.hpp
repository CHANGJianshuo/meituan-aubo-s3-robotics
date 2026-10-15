// Copyright 2026 meituan_challenge_ws contributors
//
// 末端工具模型（纯 C++）：把工具 CAD 坐标系下的几何换算到 ee_link。
//
// 用途：
//   * TCP：由 CAD 中的"挂环点"与插入方向 / 钩口方向得到 ee_link -> hook_tcp；
//   * 碰撞体：工具外形（安装盘、立柱、舌头、倒钩……）作为附着体随每次规划下发。
//
// 约定：
//   * CAD 中给出三根方向：flange_axis（离开法兰的方向，对应 ee_link +Z）、
//     insertion（舌头插入方向）、hook_up（钩口 / 倒钩朝向，应与 insertion 垂直）；
//   * mount_yaw_rad：工具绕法兰轴的安装角——insertion 在 ee_link 中为 (cos ψ, sin ψ, 0)；
//   * mate_offset_m：CAD 中贴合法兰端面的平面沿 flange_axis 的坐标；
//   * TCP 坐标系：z = 插入方向，x = 钩口方向，y = z × x（与规划器 insertion_axis = 2 一致）。

#ifndef MTC_MOVEIT_PLANNER__TOOL_MODEL_HPP_
#define MTC_MOVEIT_PLANNER__TOOL_MODEL_HPP_

#include <string>
#include <vector>

#include "mtc_moveit_planner/planning_backend.hpp"
#include "mtc_moveit_planner/pose_math.hpp"

namespace mtc
{
namespace moveit_planner
{

struct ToolBodySpec
{
  std::string id;                           ///< 本地 ID（场景中加 mtc_moveit_planner/ 前缀）
  BodyShape shape = BodyShape::BOX;
  mtc::core::Vector3 size_m;                ///< BOX：长宽高；CYLINDER：(直径, 直径, 长度)，轴沿局部 z
  Transform cad_pose;                       ///< 几何中心在 CAD 坐标系中的位姿
};

struct ToolModelSpec
{
  bool enabled = false;
  std::string name;
  std::string source;                       ///< 几何来源（文件名 / 版本）
  double mount_yaw_rad = 0.0;
  double mate_offset_m = 0.0;
  mtc::core::Vector3 flange_axis_cad{0.0, 0.0, 1.0};
  mtc::core::Vector3 insertion_cad{1.0, 0.0, 0.0};
  mtc::core::Vector3 hook_up_cad{0.0, 0.0, -1.0};
  mtc::core::Vector3 tcp_cad_m;             ///< 挂环点（CAD）
  std::vector<ToolBodySpec> bodies;
};

/// CAD -> ee_link 的刚体变换；方向非法（零向量 / 不正交）时 valid=false。
struct ToolFrames
{
  bool valid = false;
  std::string message;
  Transform ee_from_cad;
  Transform ee_to_tcp;
};

ToolFrames compute_tool_frames(const ToolModelSpec & spec);

/// 工具碰撞体在 ee_link 中的位姿（ID 已加本模块前缀）；frames 无效时返回空。
std::vector<AttachedPayload> tool_bodies_in_ee(const ToolModelSpec & spec, const ToolFrames & frames);

/// "钩口朝上"：保持 ring 的插入轴（局部 z）不变，把绕插入轴的滚转改为局部 x 尽量指向 up。
/// 插入轴与 up 的夹角小于 min_tilt_rad（近竖直插入，如 V2）时原样返回。
mtc::core::Quaternion align_roll_to_up(
  const mtc::core::Quaternion & ring, const mtc::core::Vector3 & up, double min_tilt_rad);

}  // namespace moveit_planner
}  // namespace mtc

#endif  // MTC_MOVEIT_PLANNER__TOOL_MODEL_HPP_
