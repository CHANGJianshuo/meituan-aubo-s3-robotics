// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_moveit_planner/plan_report_io.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace mtc
{
namespace moveit_planner
{

namespace
{

std::string num(double value)
{
  std::ostringstream os;
  os << std::setprecision(10) << value;
  return os.str();
}

std::string list(const std::vector<double> & values)
{
  std::ostringstream os;
  os << '[';
  for (size_t i = 0; i < values.size(); ++i) {
    os << (i ? ", " : "") << num(values[i]);
  }
  os << ']';
  return os.str();
}

std::string list(const std::vector<std::string> & values)
{
  std::ostringstream os;
  os << '[';
  for (size_t i = 0; i < values.size(); ++i) {
    os << (i ? ", " : "") << '"' << values[i] << '"';
  }
  os << ']';
  return os.str();
}

std::string quoted(const std::string & text)
{
  std::string out = "\"";
  for (char c : text) {
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    out += c;
  }
  return out + "\"";
}

}  // namespace

std::string pose_to_yaml(const mtc::core::Pose6D & pose, const std::string & indent)
{
  std::ostringstream os;
  os << indent << "frame_id: " << quoted(pose.frame_id) << '\n';
  os << indent << "position_m: " << list({pose.position.x, pose.position.y, pose.position.z}) <<
    '\n';
  os << indent << "orientation_xyzw: " <<
    list({pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w}) << '\n';
  return os.str();
}

std::string joint_state_to_yaml(const JointStateSnapshot & state, const std::string & indent)
{
  std::ostringstream os;
  os << indent << "joint_names: " << list(state.names) << '\n';
  os << indent << "positions_rad: " << list(state.positions) << '\n';
  return os.str();
}

std::string trajectory_to_yaml(
  const mtc::core::TimedTrajectory & trajectory, const std::string & indent)
{
  std::ostringstream os;
  os << indent << "trajectory_id: " << quoted(trajectory.trajectory_id) << '\n';
  os << indent << "header_frame_id: " << quoted(trajectory.header_frame_id) << '\n';
  os << indent << "stamp_s: " << num(trajectory.stamp_s) << '\n';
  os << indent << "shape_is_valid: " << (trajectory.shape_is_valid() ? "true" : "false") << '\n';
  os << indent << "duration_s: " << num(trajectory.duration_s()) << '\n';
  os << indent << "point_count: " << trajectory.points.size() << '\n';
  os << indent << "joint_names: " << list(trajectory.joint_names) << '\n';
  os << indent << "points:\n";
  for (const auto & point : trajectory.points) {
    os << indent << "  - time_from_start_s: " << num(point.time_from_start_s) << '\n';
    os << indent << "    positions_rad: " << list(point.positions_rad) << '\n';
    if (!point.velocities_rad_s.empty()) {
      os << indent << "    velocities_rad_s: " << list(point.velocities_rad_s) << '\n';
    }
    if (!point.accelerations_rad_s2.empty()) {
      os << indent << "    accelerations_rad_s2: " << list(point.accelerations_rad_s2) << '\n';
    }
  }
  return os.str();
}

std::string plan_to_yaml(
  const mtc::core::PlanResult & result, const PlanReport & report, const std::string & indent)
{
  std::ostringstream os;
  os << indent << "phase: " << quoted(report.phase) << '\n';
  os << indent << "ok: " << (result.ok ? "true" : "false") << '\n';
  os << indent << "error_code: " << result.error_code << '\n';
  os << indent << "message: " << quoted(result.message) << '\n';
  os << indent << "planning_frame: " << quoted(report.planning_frame) << '\n';
  os << indent << "end_effector_link: " << quoted(report.end_effector_link) << '\n';
  os << indent << "simulation_only: " << (report.simulation_only ? "true" : "false") << '\n';
  os << indent << "uncalibrated_inputs: " << list(report.uncalibrated_inputs) << '\n';
  os << indent << "start_state:\n" << joint_state_to_yaml(report.start_state, indent + "  ");
  os << indent << "primitives:\n";
  for (const auto & p : report.primitives) {
    os << indent << "  - primitive_name: " << quoted(p.primitive_name) << '\n';
    os << indent << "    ok: " << (p.ok ? "true" : "false") << '\n';
    if (!p.ok) {
      os << indent << "    reason: " << quoted(p.reason) << '\n';
    }
    os << indent << "    message: " << quoted(p.message) << '\n';
    os << indent << "    mode: " << quoted(p.mode) << '\n';
    os << indent << "    method: " << quoted(p.method) << '\n';
    os << indent << "    backend_message: " << quoted(p.backend_message) << '\n';
    os << indent << "    payload_attached: " << (p.payload_attached ? "true" : "false") << '\n';
    os << indent << "    goal_kind: " << quoted(p.goal_kind) << '\n';
    os << indent << "    start_assumption: " << quoted(p.start_assumption) << '\n';
    os << indent << "    contact_intent_only: " << (p.contact_intent_only ? "true" : "false") <<
      '\n';
    os << indent << "    ee_goal:\n" << pose_to_yaml(p.ee_goal, indent + "      ");
    os << indent << "    planning_time_s: " << num(p.planning_time_s) << '\n';
    os << indent << "    cartesian_fraction: " << num(p.cartesian_fraction) << '\n';
    os << indent << "    time_scale_applied: " << num(p.time_scale_applied) << '\n';
    os << indent << "    max_ee_linear_speed_m_s: " << num(p.max_linear_speed_m_s) << '\n';
    os << indent << "    max_ee_angular_speed_rad_s: " << num(p.max_angular_speed_rad_s) << '\n';
    os << indent << "    duration_s: " << num(p.duration_s) << '\n';
    os << indent << "    point_count: " << p.point_count << '\n';
    os << indent << "    timeout_ms: " << p.timeout_ms << '\n';
    os << indent << "    state_watchdog_s: " << num(p.state_watchdog_s) << '\n';
  }
  os << indent << "commands:\n";
  for (const auto & c : result.commands) {
    os << indent << "  - primitive_name: " << quoted(c.primitive_name) << '\n';
    os << indent << "    command_id: " << quoted(c.command_id) << "  # FSM 在派发时赋值\n";
    os << indent << "    velocity_scaling: " << num(c.velocity_scaling) << '\n';
    os << indent << "    acceleration_scaling: " << num(c.acceleration_scaling) << '\n';
    os << indent << "    timeout_ms: " << c.timeout_ms << '\n';
    os << indent << "    requires_contact: " << (c.requires_contact ? "true" : "false") << '\n';
    os << indent << "    simulation_only: " << (c.simulation_only ? "true" : "false") << '\n';
    os << indent << "    target_rad: " << list(c.target_rad) << '\n';
    if (c.trajectory) {
      os << indent << "    trajectory:\n" << trajectory_to_yaml(*c.trajectory, indent + "      ");
    }
  }
  return os.str();
}

bool write_text_file(const std::string & path, const std::string & content)
{
  try {
    const std::filesystem::path p(path);
    if (p.has_parent_path()) {
      std::filesystem::create_directories(p.parent_path());
    }
    std::ofstream out(path);
    if (!out) {
      return false;
    }
    out << content;
    return static_cast<bool>(out);
  } catch (const std::exception &) {
    return false;
  }
}

std::string timestamp_for_filename()
{
  const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm local{};
  localtime_r(&now, &local);
  std::ostringstream os;
  os << std::put_time(&local, "%Y%m%d_%H%M%S");
  return os.str();
}

}  // namespace moveit_planner
}  // namespace mtc
