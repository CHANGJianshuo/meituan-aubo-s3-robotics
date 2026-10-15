// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_moveit_planner/ros_params.hpp"
#include "mtc_moveit_planner/scene_ids.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace mtc
{
namespace moveit_planner
{

namespace
{

CalibrationStatus status_param(
  rclcpp::Node & node, const std::string & name, CalibrationStatus fallback)
{
  const std::string text = node.get_parameter_or<std::string>(name, to_string(fallback));
  const auto status = calibration_status_from_string(text);
  if (!status) {
    throw std::runtime_error("parameter " + name + " must be MEASURED/ESTIMATED/PLACEHOLDER/UNKNOWN");
  }
  return *status;
}

std::vector<double> vector_param(
  rclcpp::Node & node, const std::string & name, const std::vector<double> & fallback,
  size_t expected_size)
{
  const auto values = node.get_parameter_or<std::vector<double>>(name, fallback);
  if (values.size() != expected_size) {
    throw std::runtime_error(
      "parameter " + name + " must have " + std::to_string(expected_size) + " elements");
  }
  return values;
}

}  // namespace

PlannerConfig load_planner_config(rclcpp::Node & node)
{
  PlannerConfig config = default_planner_config();
  config.velocity_scaling = node.get_parameter_or("planner.velocity_scaling", config.velocity_scaling);
  config.acceleration_scaling =
    node.get_parameter_or("planner.acceleration_scaling", config.acceleration_scaling);
  config.planning_time_s = node.get_parameter_or("planner.planning_time_s", config.planning_time_s);
  config.cartesian_step_m = node.get_parameter_or("planner.cartesian_step_m", config.cartesian_step_m);
  config.insertion_axis = static_cast<int>(
    node.get_parameter_or<int64_t>("planner.insertion_axis", config.insertion_axis));
  config.allow_uncalibrated_geometry = node.get_parameter_or(
    "planner.allow_uncalibrated_geometry", config.allow_uncalibrated_geometry);
  config.force_simulation_only =
    node.get_parameter_or("planner.force_simulation_only", config.force_simulation_only);

  const auto tcp_xyz = vector_param(node, "tcp.xyz", {0.0, 0.0, 0.0}, 3);
  const auto tcp_q = vector_param(node, "tcp.quat_xyzw", {0.0, 0.0, 0.0, 1.0}, 4);
  config.ee_to_tcp.translation = mtc::core::Vector3{tcp_xyz[0], tcp_xyz[1], tcp_xyz[2]};
  config.ee_to_tcp.rotation = normalized(mtc::core::Quaternion{tcp_q[0], tcp_q[1], tcp_q[2], tcp_q[3]});
  config.tcp_status = status_param(node, "tcp.status", config.tcp_status);

  std::vector<std::string> slot_names;
  for (const auto & entry : config.slots) {
    slot_names.push_back(entry.first);
  }
  slot_names = node.get_parameter_or<std::vector<std::string>>("slots.names", slot_names);
  for (const auto & name : slot_names) {
    SlotSpec slot = config.slots.count(name) ? config.slots[name] : SlotSpec{};
    const std::vector<double> fallback{slot.pose.position.x, slot.pose.position.y, slot.pose.position.z};
    const auto xyz = vector_param(node, "slots." + name + ".xyz", fallback, 3);
    slot.pose.position = mtc::core::Vector3{xyz[0], xyz[1], xyz[2]};
    slot.pose.frame_id = node.get_parameter_or<std::string>(
      "slots." + name + ".frame_id", slot.pose.frame_id.empty() ? "base_link" : slot.pose.frame_id);
    slot.pose.orientation = mtc::core::Quaternion{0.0, 0.0, 0.0, 1.0};
    slot.pose.valid = true;
    slot.status = status_param(node, "slots." + name + ".status", slot.status);
    config.slots[name] = slot;
  }
  config.payload.enabled = node.get_parameter_or("payload.enabled", config.payload.enabled);
  const auto payload_size = vector_param(node, "payload.size_m",
      {config.payload.size_m.x, config.payload.size_m.y, config.payload.size_m.z}, 3);
  config.payload.size_m = mtc::core::Vector3{payload_size[0], payload_size[1], payload_size[2]};
  config.payload.hang_below_tcp_m =
    node.get_parameter_or("payload.hang_below_tcp_m", config.payload.hang_below_tcp_m);
  config.payload.status = status_param(node, "payload.status", config.payload.status);

  // ---- 工具行程与"钩口朝上" ----
  const double insert_travel = node.get_parameter_or("planner.insert_travel_m", 0.03);
  const double withdraw_travel = node.get_parameter_or("planner.withdraw_travel_m", 0.03);
  const double approach_backoff = node.get_parameter_or("planner.approach_backoff_m", 0.02);
  if (!(insert_travel > 0.0) || !(withdraw_travel > 0.0) || !(approach_backoff >= 0.0)) {
    throw std::runtime_error("planner.insert/withdraw_travel_m must be > 0, approach_backoff_m >= 0");
  }
  apply_tool_travel(config, insert_travel, withdraw_travel, approach_backoff);
  config.hook_up_alignment =
    node.get_parameter_or("planner.hook_up_alignment", config.hook_up_alignment);
  config.hook_up_min_tilt_rad = node.get_parameter_or(
    "planner.hook_up_min_tilt_deg", config.hook_up_min_tilt_rad * 180.0 / M_PI) * M_PI / 180.0;

  // ---- 末端工具模型（CAD 几何，CAD 坐标系，米） ----
  auto & tool = config.tool;
  tool.enabled = node.get_parameter_or("tool.enabled", false);
  if (tool.enabled) {
    const auto vec3 = [&node](const std::string & name, const std::vector<double> & fallback) {
        const auto v = vector_param(node, name, fallback, 3);
        return mtc::core::Vector3{v[0], v[1], v[2]};
      };
    tool.name = node.get_parameter_or<std::string>("tool.name", "");
    tool.source = node.get_parameter_or<std::string>("tool.source", "");
    config.tool_status = status_param(node, "tool.status", CalibrationStatus::PLACEHOLDER);
    tool.mount_yaw_rad = node.get_parameter_or("tool.mount_yaw_deg", 0.0) * M_PI / 180.0;
    tool.mate_offset_m = node.get_parameter_or("tool.mate_offset_m", 0.0);
    tool.flange_axis_cad = vec3("tool.flange_axis_cad", {0.0, 0.0, 1.0});
    tool.insertion_cad = vec3("tool.insertion_cad", {1.0, 0.0, 0.0});
    tool.hook_up_cad = vec3("tool.hook_up_cad", {0.0, 0.0, -1.0});
    tool.tcp_cad_m = vec3("tool.tcp_cad_m", {0.0, 0.0, 0.0});
    const auto body_names = node.get_parameter_or<std::vector<std::string>>(
      "tool.bodies.names", std::vector<std::string>{});
    const std::string id_error = validate_local_scene_ids(body_names);
    if (!id_error.empty()) {
      throw std::runtime_error("parameter tool.bodies.names: " + id_error);
    }
    for (const auto & name : body_names) {
      ToolBodySpec body;
      body.id = "tool_" + name;
      const std::string base = "tool.bodies." + name + ".";
      const auto shape = node.get_parameter_or<std::string>(base + "shape", "box");
      if (shape != "box" && shape != "cylinder") {
        throw std::runtime_error("parameter " + base + "shape must be box or cylinder");
      }
      body.shape = shape == "cylinder" ? BodyShape::CYLINDER : BodyShape::BOX;
      body.size_m = vec3(base + "size_m", {});
      body.cad_pose.translation = vec3(base + "xyz_m", {});
      const auto q = vector_param(node, base + "quat_xyzw", {0.0, 0.0, 0.0, 1.0}, 4);
      body.cad_pose.rotation = normalized(mtc::core::Quaternion{q[0], q[1], q[2], q[3]});
      if (!(body.size_m.x > 0.0) || !(body.size_m.y > 0.0) || !(body.size_m.z > 0.0)) {
        throw std::runtime_error("parameter " + base + "size_m must be positive");
      }
      tool.bodies.push_back(body);
    }
  }

  const auto box_names = node.get_parameter_or<std::vector<std::string>>(
    "scene.boxes.names", std::vector<std::string>{});
  const std::string id_error = validate_local_scene_ids(box_names);
  if (!id_error.empty()) {
    throw std::runtime_error("parameter scene.boxes.names: " + id_error);
  }
  for (const auto & name : box_names) {
    SceneBox box;
    box.id = name;
    const auto xyz = vector_param(node, "scene.boxes." + name + ".xyz", {}, 3);
    const auto size = vector_param(node, "scene.boxes." + name + ".size_m", {}, 3);
    const auto q = vector_param(node, "scene.boxes." + name + ".quat_xyzw", {0.0, 0.0, 0.0, 1.0}, 4);
    box.pose.frame_id = node.get_parameter_or<std::string>(
      "scene.boxes." + name + ".frame_id", "base_link");
    box.pose.position = mtc::core::Vector3{xyz[0], xyz[1], xyz[2]};
    box.pose.orientation = mtc::core::Quaternion{q[0], q[1], q[2], q[3]};
    box.pose.valid = true;
    box.size_m = mtc::core::Vector3{size[0], size[1], size[2]};
    config.scene_boxes.push_back(box);
  }
  return config;
}

MoveGroupBackendOptions load_backend_options(rclcpp::Node & node)
{
  MoveGroupBackendOptions options;
  options.planning_group = node.get_parameter_or("backend.planning_group", options.planning_group);
  options.end_effector_link =
    node.get_parameter_or("backend.end_effector_link", options.end_effector_link);
  options.planning_pipeline =
    node.get_parameter_or("backend.planning_pipeline", options.planning_pipeline);
  options.planner_id = node.get_parameter_or("backend.planner_id", options.planner_id);
  options.tf_timeout_s = node.get_parameter_or("backend.tf_timeout_s", options.tf_timeout_s);
  options.state_timeout_s = node.get_parameter_or("backend.state_timeout_s", options.state_timeout_s);
  options.ik_timeout_s = node.get_parameter_or("backend.ik_timeout_s", options.ik_timeout_s);
  options.ik_continuation_steps = static_cast<int>(node.get_parameter_or<int64_t>(
      "backend.ik_continuation_steps", options.ik_continuation_steps));
  options.ik_seed_timeout_s =
    node.get_parameter_or("backend.ik_seed_timeout_s", options.ik_seed_timeout_s);
  options.ee_limited_retiming =
    node.get_parameter_or("backend.ee_limited_retiming", options.ee_limited_retiming);
  return options;
}

}  // namespace moveit_planner
}  // namespace mtc
