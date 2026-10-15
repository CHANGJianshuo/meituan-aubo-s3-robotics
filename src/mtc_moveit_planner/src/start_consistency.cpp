// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_moveit_planner/start_consistency.hpp"

#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>

namespace mtc
{
namespace moveit_planner
{

namespace
{

/// 返回第一个空名或重复名；全部唯一且非空时返回 nullptr。
const std::string * first_bad_name(const std::vector<std::string> & names)
{
  std::set<std::string> seen;
  for (const auto & name : names) {
    if (name.empty() || !seen.insert(name).second) {
      return &name;
    }
  }
  return nullptr;
}

}  // namespace

StartCheck check_trajectory_start(
  const mtc::core::TimedTrajectory & trajectory,
  const std::vector<std::string> & actual_names,
  const std::vector<double> & actual_positions,
  double tolerance_rad)
{
  StartCheck check;
  if (trajectory.points.empty() ||
    trajectory.points.front().positions_rad.size() != trajectory.joint_names.size())
  {
    check.message = "trajectory has no valid first point";
    return check;
  }
  // 空关节集合必须拒绝：否则下面的比较一次都不执行，最大偏差保持 0 而误判为通过（fail-open）。
  if (trajectory.joint_names.empty()) {
    check.message = "trajectory has no joints";
    return check;
  }
  if (actual_names.empty()) {
    check.message = "actual state has no joints";
    return check;
  }
  if (actual_names.size() != actual_positions.size() || !std::isfinite(tolerance_rad) ||
    tolerance_rad < 0.0)
  {
    check.message = "invalid actual state or tolerance";
    return check;
  }
  // 重名会让"按名字取值"有歧义（取到哪一个取决于顺序），必须拒绝而不是取第一个。
  if (const auto * bad = first_bad_name(trajectory.joint_names)) {
    check.message = bad->empty() ? "trajectory has an empty joint name" :
      "trajectory has duplicate joint name " + *bad;
    check.worst_joint = *bad;
    return check;
  }
  if (const auto * bad = first_bad_name(actual_names)) {
    check.message = bad->empty() ? "actual state has an empty joint name" :
      "actual state has duplicate joint name " + *bad;
    check.worst_joint = *bad;
    return check;
  }
  const auto & first = trajectory.points.front().positions_rad;
  for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    size_t index = actual_names.size();
    for (size_t j = 0; j < actual_names.size(); ++j) {
      if (actual_names[j] == trajectory.joint_names[i]) {
        index = j;
        break;
      }
    }
    if (index == actual_names.size()) {
      check.message = "actual state is missing joint " + trajectory.joint_names[i];
      check.worst_joint = trajectory.joint_names[i];
      return check;
    }
    const double deviation = std::fabs(first[i] - actual_positions[index]);
    if (!std::isfinite(deviation)) {
      check.message = "non-finite value for joint " + trajectory.joint_names[i];
      check.worst_joint = trajectory.joint_names[i];
      return check;
    }
    if (deviation > check.max_deviation_rad || check.worst_joint.empty()) {
      check.max_deviation_rad = deviation;
      check.worst_joint = trajectory.joint_names[i];
    }
  }
  // 只有真正比较过至少一个关节才可能通过（防御性，空集合已在上面拒绝）。
  check.ok = !check.worst_joint.empty() && check.max_deviation_rad <= tolerance_rad;
  std::ostringstream os;
  os << std::setprecision(6) << "max start deviation " << check.max_deviation_rad << " rad at " <<
    check.worst_joint << (check.ok ? " <= " : " > ") << tolerance_rad << " rad";
  check.message = os.str();
  return check;
}

}  // namespace moveit_planner
}  // namespace mtc
