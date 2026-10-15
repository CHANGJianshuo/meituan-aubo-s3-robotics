// Copyright 2026 MTC contributors
#include "mtc_trajectory_execution_cpp/trajectory_validator.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace mtc::trajectory
{
void Config::validate() const
{
  const auto unique = std::set<std::string>(joints.begin(), joints.end());
  if (joints.size() != 6 || unique.size() != 6 || unique.count("") || limits.size() != 6 ||
    controller.empty() || controller.find('/') != std::string::npos ||
    controller_manager.empty() || controller_manager.front() != '/' ||
    hardware_component.empty() || joint_state_topic.empty() || joint_state_topic.front() != '/' ||
    max_points < 2 || max_timeout_ms == 0)
  {throw std::invalid_argument("six unique joints, absolute endpoints and explicit limits required");}
  for (double v : {start_tolerance, final_tolerance, stationary_velocity, settle_duration,
      feedback_max_age, proof_max_age, max_duration, max_start_delay})
  {if (!std::isfinite(v) || v <= 0) {throw std::invalid_argument("invalid positive configuration");}}
  if (max_duration > 120 || feedback_max_age > 1 || proof_max_age > 2) {
    throw std::invalid_argument("observation or execution capability exceeded");
  }
  for (const auto & n : joints) {
    auto it = limits.find(n);
    if (it == limits.end()) {throw std::invalid_argument("missing joint limit");}
    const auto & l = it->second;
    if (!std::isfinite(l.minimum) || !std::isfinite(l.maximum) || l.minimum >= l.maximum ||
      !std::isfinite(l.velocity) || l.velocity <= 0 ||
      !std::isfinite(l.acceleration) || l.acceleration <= 0)
    {throw std::invalid_argument("invalid joint limits");}
  }
}
bool near(const std::vector<double> & a, const std::vector<double> & b, double tolerance)
{
  if (a.empty() || a.size() != b.size()) {return false;}
  for (size_t i = 0; i < a.size(); ++i) {
    if (!std::isfinite(a[i]) || !std::isfinite(b[i]) || std::abs(a[i] - b[i]) > tolerance) {
      return false;
    }
  }
  return true;
}
Validation convert(const mtc::core::MotionCommand & c, const Config & cfg)
{
  auto bad = [](const std::string & e) {return Validation{std::nullopt, e};};
  try {cfg.validate();} catch (const std::exception & e) {return bad(e.what());}
  if (!c.simulation_only || c.command_id.empty() || !c.trajectory || !c.target_rad.empty() ||
    c.requires_contact || (c.intent && (!c.intent->is_usable() || c.intent->requires_contact)))
  {return bad("only identified simulation-only complete joint trajectories; contact unsupported");}
  if (!std::isfinite(c.velocity_scaling) || c.velocity_scaling <= 0 || c.velocity_scaling > 0.2 ||
    !std::isfinite(c.acceleration_scaling) || c.acceleration_scaling <= 0 ||
    c.acceleration_scaling > 0.2 || c.timeout_ms == 0 || c.timeout_ms > cfg.max_timeout_ms)
  {return bad("invalid scaling or timeout capability");}
  const auto & t = *c.trajectory;
  if (!t.shape_is_valid() || t.trajectory_id.empty() || t.points.size() < 2 ||
    t.points.size() > cfg.max_points || t.joint_names.size() != cfg.joints.size() ||
    t.points.front().time_from_start_s != 0.0 || t.duration_s() > cfg.max_duration ||
    t.duration_s() + cfg.settle_duration + cfg.max_start_delay >= c.timeout_ms / 1000.0 ||
    t.stamp_s > std::numeric_limits<int32_t>::max())
  {return bad("invalid trajectory shape, complete endpoints, duration, stamp or deadline");}
  if (!c.joint_names.empty() && c.joint_names != t.joint_names) {
    return bad("command joint names must agree with trajectory representation");
  }
  std::vector<size_t> indices;
  for (const auto & name : cfg.joints) {
    auto it = std::find(t.joint_names.begin(), t.joint_names.end(), name);
    if (it == t.joint_names.end()) {return bad("missing or unexpected controller joint");}
    indices.push_back(static_cast<size_t>(it - t.joint_names.begin()));
  }
  trajectory_msgs::msg::JointTrajectory out;
  out.joint_names = cfg.joints;
  out.header.frame_id = t.header_frame_id;
  const auto stamp_ns = static_cast<int64_t>(std::llround(t.stamp_s * 1e9));
  if (stamp_ns / 1000000000LL > std::numeric_limits<int32_t>::max()) {
    return bad("ROS stamp overflow");
  }
  out.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
  out.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
  int64_t last_ns = -1;
  for (size_t p = 0; p < t.points.size(); ++p) {
    const auto & in = t.points[p];
    trajectory_msgs::msg::JointTrajectoryPoint point;
    const auto ns = static_cast<int64_t>(std::llround(in.time_from_start_s * 1e9));
    if (ns <= last_ns) {return bad("time ordering lost at ROS nanosecond precision");}
    last_ns = ns;
    point.time_from_start.sec = static_cast<int32_t>(ns / 1000000000LL);
    point.time_from_start.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    for (size_t j = 0; j < indices.size(); ++j) {
      const auto i = indices[j];
      const auto & l = cfg.limits.at(cfg.joints[j]);
      const auto x = in.positions_rad[i];
      if (x < l.minimum || x > l.maximum) {return bad("joint position limit exceeded");}
      point.positions.push_back(x);
      if (!in.velocities_rad_s.empty()) {
        auto v = in.velocities_rad_s[i];
        if (std::abs(v) > l.velocity) {return bad("velocity limit exceeded");}
        point.velocities.push_back(v);
      }
      if (!in.accelerations_rad_s2.empty()) {
        auto a = in.accelerations_rad_s2[i];
        if (std::abs(a) > l.acceleration) {return bad("acceleration limit exceeded");}
        point.accelerations.push_back(a);
      }
      if (p && std::abs(x - t.points[p-1].positions_rad[i]) /
        (in.time_from_start_s - t.points[p-1].time_from_start_s) > l.velocity)
      {return bad("segment average velocity exceeds configured limit");}
    }
    // Uniform derivative availability is necessary for predictable JTC interpolation.
    if (p && (point.velocities.empty() != out.points.front().velocities.empty() ||
      point.accelerations.empty() != out.points.front().accelerations.empty()))
    {return bad("inconsistent derivative fields");}
    if (!point.accelerations.empty() && point.velocities.empty()) {
      return bad("acceleration without velocity unsupported");
    }
    // JTC spline interpolation can exceed waypoint limits. Bernstein control hulls
    // give a conservative bound for the whole linear/cubic/quintic segment and derivatives.
    if (p) {
      const auto & prev=out.points.back();
      const double dt=in.time_from_start_s-t.points[p-1].time_from_start_s;
      for (size_t j=0;j<indices.size();++j) {
        const auto & l=cfg.limits.at(cfg.joints[j]);
        std::vector<double> b{prev.positions[j],point.positions[j]};
        if (!point.accelerations.empty()) {
          b={prev.positions[j],prev.positions[j]+prev.velocities[j]*dt/5,
            prev.positions[j]+2*prev.velocities[j]*dt/5+prev.accelerations[j]*dt*dt/20,
            point.positions[j]-2*point.velocities[j]*dt/5+point.accelerations[j]*dt*dt/20,
            point.positions[j]-point.velocities[j]*dt/5,point.positions[j]};
        } else if (!point.velocities.empty()) {
          b={prev.positions[j],prev.positions[j]+prev.velocities[j]*dt/3,
            point.positions[j]-point.velocities[j]*dt/3,point.positions[j]};
        }
        const double degree=b.size()-1;
        for (double x:b) {if (!std::isfinite(x) || x<l.minimum || x>l.maximum) {
            return bad("interpolated position cannot be bounded within limits");}}
        std::vector<double> velocities;
        for (size_t k=1;k<b.size();++k) {
          double v=degree*(b[k]-b[k-1])/dt;
          if (!std::isfinite(v) || std::abs(v)>l.velocity) {
            return bad("interpolated velocity cannot be bounded within limits");}
          velocities.push_back(v);
        }
        for (size_t k=1;k<velocities.size();++k) {
          double a=(degree-1)*(velocities[k]-velocities[k-1])/dt;
          if (!std::isfinite(a) || std::abs(a)>l.acceleration) {
            return bad("interpolated acceleration cannot be bounded within limits");}
        }
      }
    }
    out.points.push_back(std::move(point));
  }
  return {out, ""};
}
std::optional<JointSample> map_sample(const sensor_msgs::msg::JointState & msg, const Config & c)
{
  if (msg.name.size() != c.joints.size() || msg.position.size() != msg.name.size() ||
    msg.velocity.size() != msg.name.size() || msg.header.stamp.sec < 0 ||
    msg.header.stamp.nanosec >= 1000000000u ||
    std::set<std::string>(msg.name.begin(), msg.name.end()).size() != msg.name.size())
  {return std::nullopt;}
  JointSample out;
  out.stamp_ns = static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL + msg.header.stamp.nanosec;
  if (out.stamp_ns <= 0) {return std::nullopt;}
  for (const auto & n : c.joints) {
    auto it = std::find(msg.name.begin(), msg.name.end(), n);
    if (it == msg.name.end()) {return std::nullopt;}
    auto i = static_cast<size_t>(it - msg.name.begin());
    if (!std::isfinite(msg.position[i]) || !std::isfinite(msg.velocity[i]) || std::abs(msg.velocity[i])>c.limits.at(n).velocity ||
      msg.position[i] < c.limits.at(n).minimum || msg.position[i] > c.limits.at(n).maximum)
    {return std::nullopt;}
    out.positions.push_back(msg.position[i]); out.velocities.push_back(msg.velocity[i]);
  }
  return out;
}
}  // namespace mtc::trajectory
