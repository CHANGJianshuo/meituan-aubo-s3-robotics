// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_moveit_planner/trajectory_conversion.hpp"

#include <cmath>
#include <map>
#include <set>

namespace mtc
{
namespace moveit_planner
{

namespace
{

bool reorder(
  const std::vector<double> & source, const std::vector<size_t> & index,
  std::vector<double> & out)
{
  if (source.empty()) {
    out.clear();
    return true;
  }
  if (source.size() != index.size()) {
    return false;
  }
  out.assign(index.size(), 0.0);
  for (size_t i = 0; i < index.size(); ++i) {
    out[i] = source[index[i]];
  }
  return true;
}

}  // namespace

ConversionResult to_timed_trajectory(
  const RawTrajectory & raw,
  const std::vector<std::string> & expected_joint_order,
  const std::string & frame_id,
  double stamp_s)
{
  ConversionResult result;
  if (frame_id.empty()) {
    result.error = "frame_id is empty";
    return result;
  }
  if (expected_joint_order.empty()) {
    result.error = "expected joint order is empty";
    return result;
  }
  if (raw.points.empty()) {
    result.error = "trajectory has no points";
    return result;
  }

  std::map<std::string, size_t> source_index;
  for (size_t i = 0; i < raw.joint_names.size(); ++i) {
    if (!source_index.emplace(raw.joint_names[i], i).second) {
      result.error = "duplicate joint in planner output: " + raw.joint_names[i];
      return result;
    }
  }
  if (raw.joint_names.size() != expected_joint_order.size()) {
    result.error = "planner output has " + std::to_string(raw.joint_names.size()) +
      " joints, expected " + std::to_string(expected_joint_order.size());
    return result;
  }
  std::vector<size_t> index;
  index.reserve(expected_joint_order.size());
  for (const auto & name : expected_joint_order) {
    const auto found = source_index.find(name);
    if (found == source_index.end()) {
      result.error = "planner output is missing joint: " + name;
      return result;
    }
    index.push_back(found->second);
  }

  mtc::core::TimedTrajectory trajectory;
  trajectory.joint_names = expected_joint_order;
  trajectory.header_frame_id = frame_id;
  trajectory.stamp_s = stamp_s;
  trajectory.points.reserve(raw.points.size());
  for (size_t p = 0; p < raw.points.size(); ++p) {
    const auto & source = raw.points[p];
    mtc::core::TrajectoryPoint point;
    point.time_from_start_s = source.time_from_start_s;
    if (!reorder(source.positions, index, point.positions_rad) ||
      point.positions_rad.size() != index.size() ||
      !reorder(source.velocities, index, point.velocities_rad_s) ||
      !reorder(source.accelerations, index, point.accelerations_rad_s2))
    {
      result.error = "point " + std::to_string(p) + " has inconsistent dimensions";
      return result;
    }
    trajectory.points.push_back(std::move(point));
  }

  if (!trajectory.shape_is_valid()) {
    result.error = "converted trajectory failed shape check "
      "(non-finite value, negative stamp or non-increasing time_from_start)";
    return result;
  }
  result.ok = true;
  result.trajectory = std::move(trajectory);
  return result;
}

bool stretch_time(mtc::core::TimedTrajectory & trajectory, double factor)
{
  if (!std::isfinite(factor) || factor < 1.0) {
    return false;
  }
  for (auto & point : trajectory.points) {
    point.time_from_start_s *= factor;
    for (auto & v : point.velocities_rad_s) {
      v /= factor;
    }
    for (auto & a : point.accelerations_rad_s2) {
      a /= factor * factor;
    }
  }
  return true;
}

std::vector<double> final_positions(const mtc::core::TimedTrajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return {};
  }
  return trajectory.points.back().positions_rad;
}

}  // namespace moveit_planner
}  // namespace mtc
