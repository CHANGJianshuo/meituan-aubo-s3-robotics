// Copyright 2026 meituan_challenge_ws contributors
//
// 最小刚体变换工具（纯 C++，基于 mtc_core_cpp::Pose6D）。
//
// 只做规划层需要的三件事：TCP 补偿（ee_link <-> hook_tcp）、沿局部轴偏移
// （预对准退让 / 插入 / 退出）、沿规划坐标系平移（试提 / 抬升 / 落座高度）。
// 所有运算保留 frame_id 与 stamp，不做任何坐标系之间的隐式换算；
// 跨坐标系必须先经后端 TF 转换（见 PlanningBackend::transform_to_planning_frame）。

#ifndef MTC_MOVEIT_PLANNER__POSE_MATH_HPP_
#define MTC_MOVEIT_PLANNER__POSE_MATH_HPP_

#include <algorithm>
#include <cmath>

#include "mtc_core_cpp/pose.hpp"

namespace mtc
{
namespace moveit_planner
{

/// 刚体变换 T = (t, q)：先旋转 q，再平移 t。
struct Transform
{
  mtc::core::Vector3 translation;
  mtc::core::Quaternion rotation;  ///< 默认单位四元数
};

inline mtc::core::Quaternion normalized(const mtc::core::Quaternion & q)
{
  const double n = q.norm();
  if (!(n > 0.0) || !std::isfinite(n)) {
    return mtc::core::Quaternion{};
  }
  return mtc::core::Quaternion{q.x / n, q.y / n, q.z / n, q.w / n};
}

/// Hamilton 积 a * b（先 b 后 a）。
inline mtc::core::Quaternion multiply(
  const mtc::core::Quaternion & a, const mtc::core::Quaternion & b)
{
  return mtc::core::Quaternion{
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

inline mtc::core::Quaternion conjugate(const mtc::core::Quaternion & q)
{
  return mtc::core::Quaternion{-q.x, -q.y, -q.z, q.w};
}

inline mtc::core::Vector3 rotate(const mtc::core::Quaternion & q, const mtc::core::Vector3 & v)
{
  const mtc::core::Quaternion p{v.x, v.y, v.z, 0.0};
  const auto r = multiply(multiply(q, p), conjugate(q));
  return mtc::core::Vector3{r.x, r.y, r.z};
}

inline Transform to_transform(const mtc::core::Pose6D & pose)
{
  return Transform{pose.position, normalized(pose.orientation)};
}

/// a * b：先在 b 的局部系表达，再变换到 a 的父系。
inline Transform compose(const Transform & a, const Transform & b)
{
  const auto rotated = rotate(a.rotation, b.translation);
  return Transform{
    mtc::core::Vector3{
      a.translation.x + rotated.x, a.translation.y + rotated.y, a.translation.z + rotated.z},
    normalized(multiply(a.rotation, b.rotation))};
}

inline Transform inverse(const Transform & a)
{
  const auto q_inv = conjugate(normalized(a.rotation));
  const auto t = rotate(q_inv, a.translation);
  return Transform{mtc::core::Vector3{-t.x, -t.y, -t.z}, q_inv};
}

/// pose ∘ local：结果仍在 pose.frame_id 中，保留 stamp / valid。
inline mtc::core::Pose6D apply(const mtc::core::Pose6D & pose, const Transform & local)
{
  const auto composed = compose(to_transform(pose), local);
  mtc::core::Pose6D out = pose;
  out.position = composed.translation;
  out.orientation = composed.rotation;
  return out;
}

/// 沿位姿自身的局部轴（0=x, 1=y, 2=z）移动 distance 米。
inline mtc::core::Pose6D offset_along_local_axis(
  const mtc::core::Pose6D & pose, int axis, double distance)
{
  Transform local;
  if (axis == 0) {
    local.translation.x = distance;
  } else if (axis == 1) {
    local.translation.y = distance;
  } else {
    local.translation.z = distance;
  }
  return apply(pose, local);
}

/// 沿父坐标系（即 pose.frame_id）平移，不改变姿态。
inline mtc::core::Pose6D translate_in_frame(
  const mtc::core::Pose6D & pose, const mtc::core::Vector3 & delta)
{
  mtc::core::Pose6D out = pose;
  out.position.x += delta.x;
  out.position.y += delta.y;
  out.position.z += delta.z;
  return out;
}

inline double dot(const mtc::core::Vector3 & a, const mtc::core::Vector3 & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline mtc::core::Vector3 cross(const mtc::core::Vector3 & a, const mtc::core::Vector3 & b)
{
  return mtc::core::Vector3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline mtc::core::Vector3 scaled(const mtc::core::Vector3 & v, double s)
{
  return mtc::core::Vector3{v.x * s, v.y * s, v.z * s};
}

inline mtc::core::Vector3 unit(const mtc::core::Vector3 & v)
{
  const double n = std::sqrt(dot(v, v));
  return n > 0.0 ? scaled(v, 1.0 / n) : mtc::core::Vector3{};
}

/// 由三根正交单位轴（父系中表达，即旋转矩阵的三列）求四元数。
inline mtc::core::Quaternion quaternion_from_axes(
  const mtc::core::Vector3 & x, const mtc::core::Vector3 & y, const mtc::core::Vector3 & z)
{
  const double m00 = x.x, m01 = y.x, m02 = z.x;
  const double m10 = x.y, m11 = y.y, m12 = z.y;
  const double m20 = x.z, m21 = y.z, m22 = z.z;
  const double trace = m00 + m11 + m22;
  mtc::core::Quaternion q;
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q = {(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s};
  } else if (m00 > m11 && m00 > m22) {
    const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
    q = {0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s};
  } else if (m11 > m22) {
    const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
    q = {(m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s};
  } else {
    const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
    q = {(m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s};
  }
  return normalized(q);
}

/// 两姿态之间的最小旋转角（弧度，范围 [0, π]）。
inline double angle_between(const mtc::core::Quaternion & a, const mtc::core::Quaternion & b)
{
  const auto qa = normalized(a);
  const auto qb = normalized(b);
  const double dot = std::fabs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w);
  return 2.0 * std::acos(std::min(1.0, dot));
}

inline double distance_between(const mtc::core::Vector3 & a, const mtc::core::Vector3 & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace moveit_planner
}  // namespace mtc

#endif  // MTC_MOVEIT_PLANNER__POSE_MATH_HPP_
