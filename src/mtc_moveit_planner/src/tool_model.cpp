// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_moveit_planner/tool_model.hpp"

#include <cmath>

#include "mtc_moveit_planner/scene_ids.hpp"

namespace mtc
{
namespace moveit_planner
{

namespace
{

using mtc::core::Vector3;

Vector3 add(const Vector3 & a, const Vector3 & b) {return Vector3{a.x + b.x, a.y + b.y, a.z + b.z};}

}  // namespace

ToolFrames compute_tool_frames(const ToolModelSpec & spec)
{
  ToolFrames frames;
  const auto w = unit(spec.flange_axis_cad);
  const auto u = unit(spec.insertion_cad);
  const auto h = unit(spec.hook_up_cad);
  if (dot(w, w) < 0.5 || dot(u, u) < 0.5 || dot(h, h) < 0.5) {
    frames.message = "tool directions must be non-zero";
    return frames;
  }
  if (std::fabs(dot(u, w)) > 1e-6 || std::fabs(dot(u, h)) > 1e-6) {
    frames.message = "insertion must be perpendicular to flange axis and hook-up direction";
    return frames;
  }
  // CAD 基 (u, w×u, w) -> ee 基 (u', ẑ×u', ẑ)，u' = (cos ψ, sin ψ, 0)。
  const Vector3 u_ee{std::cos(spec.mount_yaw_rad), std::sin(spec.mount_yaw_rad), 0.0};
  const Vector3 z_ee{0.0, 0.0, 1.0};
  const auto q_cad = quaternion_from_axes(u, cross(w, u), w);       // CAD 中的该正交基
  const auto q_ee = quaternion_from_axes(u_ee, cross(z_ee, u_ee), z_ee);
  frames.ee_from_cad.rotation = normalized(multiply(q_ee, conjugate(q_cad)));
  // 贴合面上的轴心点映射到 ee_link 原点。
  frames.ee_from_cad.translation = scaled(rotate(frames.ee_from_cad.rotation, w), -spec.mate_offset_m);

  const auto tcp_origin = add(rotate(frames.ee_from_cad.rotation, spec.tcp_cad_m),
      frames.ee_from_cad.translation);
  const auto z_t = rotate(frames.ee_from_cad.rotation, u);
  const auto x_t = rotate(frames.ee_from_cad.rotation, h);
  frames.ee_to_tcp = Transform{tcp_origin, quaternion_from_axes(x_t, cross(z_t, x_t), z_t)};
  frames.valid = true;
  frames.message = "ok";
  return frames;
}

std::vector<AttachedPayload> tool_bodies_in_ee(const ToolModelSpec & spec, const ToolFrames & frames)
{
  std::vector<AttachedPayload> out;
  if (!frames.valid) {
    return out;
  }
  for (const auto & body : spec.bodies) {
    AttachedPayload attached;
    attached.id = scoped_scene_id(body.id);
    attached.shape = body.shape;
    attached.size_m = body.size_m;
    attached.ee_to_payload = compose(frames.ee_from_cad, body.cad_pose);
    attached.touch_links = {"ee_link", "wrist3_Link"};
    out.push_back(attached);
  }
  return out;
}

mtc::core::Quaternion align_roll_to_up(
  const mtc::core::Quaternion & ring, const mtc::core::Vector3 & up, double min_tilt_rad)
{
  const auto q = normalized(ring);
  const auto z = unit(rotate(q, Vector3{0.0, 0.0, 1.0}));
  const auto n_up = unit(up);
  const double tilt = std::acos(std::max(-1.0, std::min(1.0, std::fabs(dot(z, n_up)))));
  if (tilt < min_tilt_rad) {
    return q;
  }
  const auto x = unit(add(n_up, scaled(z, -dot(n_up, z))));
  return quaternion_from_axes(x, cross(z, x), z);
}

}  // namespace moveit_planner
}  // namespace mtc
