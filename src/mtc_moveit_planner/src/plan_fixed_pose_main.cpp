// Copyright 2026 meituan_challenge_ws contributors
//
// P0 证据工具：从当前机器人状态规划到一个**固定、可复现**的 6D 目标位姿，
// 输出返回状态、规划耗时与完整时间参数化轨迹（YAML）。只规划，不执行。
//
// 默认目标：SRDF 命名状态 "home" 的 ee_link 正运动学位姿（保证可达、可复现）。
// 也可用参数 target.mode:=pose 指定 target.frame_id / target.xyz / target.quat_xyzw。

#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "mtc_moveit_planner/move_group_backend.hpp"
#include "mtc_moveit_planner/plan_report_io.hpp"
#include "mtc_moveit_planner/pose_math.hpp"
#include "mtc_moveit_planner/ros_params.hpp"
#include "mtc_moveit_planner/trajectory_conversion.hpp"

namespace
{

using mtc::moveit_planner::BackendRequest;
using mtc::moveit_planner::JointStateSnapshot;
using mtc::moveit_planner::MotionMode;
using mtc::moveit_planner::MoveGroupBackend;

std::string num(double v)
{
  std::ostringstream os;
  os << std::setprecision(10) << v;
  return os.str();
}

int run(const std::shared_ptr<rclcpp::Node> & node)
{
  const auto logger = node->get_logger();
  const auto planner_config = mtc::moveit_planner::load_planner_config(*node);
  const auto options = mtc::moveit_planner::load_backend_options(*node);
  const std::string output_dir = node->get_parameter_or<std::string>("output_dir", "planning_evidence");
  const std::string target_mode = node->get_parameter_or<std::string>("target.mode", "named_state_fk");
  const std::string named = node->get_parameter_or<std::string>("target.named_state", "home");

  auto backend = std::make_shared<MoveGroupBackend>(node, options);
  if (!backend->apply_scene(planner_config.scene_boxes)) {
    RCLCPP_ERROR(logger, "failed to apply planning scene obstacles");
    return 2;
  }
  RCLCPP_INFO(logger, "planning scene: %zu static obstacle(s) applied",
    planner_config.scene_boxes.size());
  RCLCPP_INFO(logger, "planning frame='%s' ee_link='%s' group='%s' pipeline='%s' planner='%s'",
    backend->planning_frame().c_str(), backend->end_effector_link().c_str(),
    options.planning_group.c_str(), options.planning_pipeline.c_str(), options.planner_id.c_str());

  std::optional<JointStateSnapshot> start;
  for (int i = 0; i < 10 && !start; ++i) {
    start = backend->current_state();
  }
  if (!start) {
    RCLCPP_ERROR(logger, "no current robot state from /joint_states");
    return 2;
  }

  mtc::core::Pose6D target;
  if (target_mode == "named_state_fk") {
    const auto js = backend->named_state(named);
    if (!js) {
      RCLCPP_ERROR(logger, "SRDF named state '%s' not found", named.c_str());
      return 2;
    }
    const auto fk = backend->end_effector_pose(*js);
    if (!fk) {
      RCLCPP_ERROR(logger, "FK failed for named state '%s'", named.c_str());
      return 2;
    }
    target = *fk;
  } else {
    const auto xyz = node->get_parameter_or<std::vector<double>>("target.xyz", {});
    const auto q = node->get_parameter_or<std::vector<double>>("target.quat_xyzw", {});
    if (xyz.size() != 3 || q.size() != 4) {
      RCLCPP_ERROR(logger, "target.xyz needs 3 values and target.quat_xyzw needs 4");
      return 2;
    }
    target.frame_id = node->get_parameter_or<std::string>("target.frame_id", backend->planning_frame());
    target.position = mtc::core::Vector3{xyz[0], xyz[1], xyz[2]};
    target.orientation = mtc::core::Quaternion{q[0], q[1], q[2], q[3]};
    target.valid = true;
    if (!target.orientation.is_normalized()) {
      RCLCPP_ERROR(logger, "target quaternion is not normalized (norm=%f)", target.orientation.norm());
      return 2;
    }
  }

  BackendRequest request;
  request.label = "fixed_pose_test";
  request.mode = MotionMode::FREE_SPACE;
  request.ee_goal = target;
  request.start = *start;
  request.velocity_scaling = planner_config.velocity_scaling;
  request.acceleration_scaling = planner_config.acceleration_scaling;
  request.max_linear_speed_m_s = node->get_parameter_or("target.max_linear_speed_m_s", 0.0);
  request.max_angular_speed_rad_s = node->get_parameter_or("target.max_angular_speed_rad_s", 0.0);
  request.planning_time_s = planner_config.planning_time_s;
  request.joint_bounds = planner_config.joint_position_limits;

  if (planner_config.tool.enabled) {
    // 工具外形随规划一起做碰撞检查（与 MoveItPlannerPort 一致）。
    const auto frames = mtc::moveit_planner::compute_tool_frames(planner_config.tool);
    if (!frames.valid) {
      RCLCPP_ERROR(logger, "tool model: %s", frames.message.c_str());
      return 2;
    }
    request.tool_bodies = mtc::moveit_planner::tool_bodies_in_ee(planner_config.tool, frames);
  }
  const auto result = backend->plan(request);
  const double stamp = node->now().seconds();
  mtc::moveit_planner::ConversionResult converted;
  double position_error = -1.0;
  double angle_error = -1.0;
  if (result.ok) {
    converted = mtc::moveit_planner::to_timed_trajectory(
      result.trajectory, planner_config.joint_names, backend->planning_frame(), stamp);
    const auto target_in_frame = backend->transform_to_planning_frame(target);
    const auto reached = backend->end_effector_pose(result.end_state);
    if (target_in_frame && reached) {
      position_error = mtc::moveit_planner::distance_between(
        reached->position, target_in_frame->position);
      angle_error = mtc::moveit_planner::angle_between(
        reached->orientation, target_in_frame->orientation);
    }
  }
  const bool ok = result.ok && converted.ok;

  std::ostringstream yaml;
  yaml << "# mtc_moveit_planner plan_fixed_pose 证据（Mock / Fake Hardware，仅规划，未执行）\n";
  yaml << "tool: plan_fixed_pose\n";
  yaml << "wall_time: \"" << mtc::moveit_planner::timestamp_for_filename() << "\"\n";
  yaml << "ros_time_s: " << num(stamp) << '\n';
  yaml << "configuration:\n";
  yaml << "  planning_group: \"" << options.planning_group << "\"\n";
  yaml << "  planning_frame: \"" << backend->planning_frame() << "\"\n";
  yaml << "  end_effector_link: \"" << backend->end_effector_link() << "\"\n";
  yaml << "  planning_pipeline: \"" << options.planning_pipeline << "\"\n";
  yaml << "  planner_id: \"" << options.planner_id << "\"\n";
  yaml << "  velocity_scaling: " << num(request.velocity_scaling) << '\n';
  yaml << "  acceleration_scaling: " << num(request.acceleration_scaling) << '\n';
  yaml << "  planning_time_limit_s: " << num(request.planning_time_s) << '\n';
  yaml << "start_state:\n" << mtc::moveit_planner::joint_state_to_yaml(*start, "  ");
  yaml << "target:\n";
  yaml << "  mode: \"" << target_mode << "\"\n";
  if (target_mode == "named_state_fk") {
    yaml << "  named_state: \"" << named << "\"\n";
  }
  yaml << "  ee_link_pose:\n" << mtc::moveit_planner::pose_to_yaml(target, "    ");
  yaml << "result:\n";
  yaml << "  ok: " << (ok ? "true" : "false") << '\n';
  yaml << "  reason: \"" << (result.ok ? (converted.ok ? "" : "INVALID_TRAJECTORY") : result.reason) << "\"\n";
  yaml << "  message: \"" << (result.ok && !converted.ok ? converted.error : result.message) << "\"\n";
  yaml << "  planning_time_s: " << num(result.planning_time_s) << '\n';
  yaml << "  time_scale_applied: " << num(result.time_scale_applied) << '\n';
  yaml << "  max_ee_linear_speed_m_s: " << num(result.max_linear_speed_m_s) << '\n';
  yaml << "  max_ee_angular_speed_rad_s: " << num(result.max_angular_speed_rad_s) << '\n';
  yaml << "  final_position_error_m: " << num(position_error) << '\n';
  yaml << "  final_orientation_error_rad: " << num(angle_error) << '\n';
  if (ok) {
    yaml << "  end_state:\n" << mtc::moveit_planner::joint_state_to_yaml(result.end_state, "    ");
    yaml << "trajectory:  # mtc::core::TimedTrajectory，关节顺序 = PickPlaceConfig::joint_names\n";
    yaml << mtc::moveit_planner::trajectory_to_yaml(converted.trajectory, "  ");
  }

  const std::string path = output_dir + "/fixed_pose_plan_" +
    mtc::moveit_planner::timestamp_for_filename() + ".yaml";
  if (!mtc::moveit_planner::write_text_file(path, yaml.str())) {
    RCLCPP_ERROR(logger, "failed to write %s", path.c_str());
    return 3;
  }
  if (ok) {
    RCLCPP_INFO(logger,
      "PLAN OK: planning_time=%.3f s, duration=%.3f s, points=%zu, final_pos_err=%.2e m, "
      "final_rot_err=%.2e rad -> %s",
      result.planning_time_s, converted.trajectory.duration_s(), converted.trajectory.points.size(),
      position_error, angle_error, path.c_str());
  } else {
    RCLCPP_ERROR(logger, "PLAN FAILED [%s] %s -> %s", result.reason.c_str(),
      result.message.c_str(), path.c_str());
  }
  return ok ? 0 : 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("mtc_plan_fixed_pose", node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});
  int rc = 1;
  try {
    rc = run(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "fatal: %s", error.what());
    rc = 4;
  }
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return rc;
}
