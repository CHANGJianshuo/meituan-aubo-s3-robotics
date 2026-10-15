// Copyright 2026 meituan_challenge_ws contributors
//
// 单块电池基础任务的运动阶段规划（Mock，仅规划不执行）。
//
// 用已合入的 ToolStrategy 生成与 PickPlaceFsm **完全相同**的两次规划请求：
//   ACQUIRE   = make_acquire_plan + make_test_lift_plan            （PLAN_ACQUIRE 状态）
//   TRANSPORT = lift + transport + pre_seat + seat_contact + retreat（LIFT 状态）
// 第二次以第一次 test_lift 轨迹终点为起点（模拟 FSM 中试提已执行完）。
// 提环位姿与工位均为占位值（见 config/planner.yaml），输出 simulation_only=true。

#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "mtc_core_cpp/clock.hpp"
#include "mtc_manipulation_cpp/tool_strategy.hpp"
#include "mtc_moveit_planner/move_group_backend.hpp"
#include "mtc_moveit_planner/moveit_planner_port.hpp"
#include "mtc_moveit_planner/plan_report_io.hpp"
#include "mtc_moveit_planner/pose_math.hpp"
#include "mtc_moveit_planner/ros_params.hpp"
#include "mtc_moveit_planner/trajectory_conversion.hpp"

namespace
{

using mtc::manipulation::PlanTarget;
using mtc::moveit_planner::JointStateSnapshot;
using mtc::moveit_planner::MoveGroupBackend;
using mtc::moveit_planner::MoveItPlannerPort;

PlanTarget make_target(
  const std::string & phase, const mtc::core::PoseEstimate & ring, const std::string & slot,
  const mtc::core::MotionPrimitiveList & primitives)
{
  PlanTarget target;
  target.phase = phase;
  target.object_id = ring.object_id;
  target.slot = slot;
  target.has_ring_pose = ring.has_ring_pose();
  target.ring_pose = ring.ring_pose;
  target.primitives = primitives;
  for (const auto & p : primitives) {
    target.primitive_names.push_back(p.name);
  }
  return target;
}

void log_report(const rclcpp::Logger & logger, const mtc::core::PlanResult & result,
  const mtc::moveit_planner::PlanReport & report)
{
  RCLCPP_INFO(logger, "[%s] ok=%s code=%u %s", report.phase.c_str(), result.ok ? "true" : "false",
    result.error_code, result.message.c_str());
  for (const auto & p : report.primitives) {
    RCLCPP_INFO(logger,
      "  %-13s %-10s %-19s %s ok=%-5s plan=%.3fs dur=%.2fs pts=%zu scale=%.2f v=%.3fm/s w=%.3frad/s "
      "watchdog=%.0fs %s%s",
      p.primitive_name.c_str(), p.mode.c_str(), p.method.c_str(),
      p.payload_attached ? "[payload]" : "         ", p.ok ? "true" : "false",
      p.planning_time_s,
      p.duration_s, p.point_count, p.time_scale_applied, p.max_linear_speed_m_s,
      p.max_angular_speed_rad_s, p.state_watchdog_s,
      p.contact_intent_only ? "[contact intent only] " : "",
      p.ok ? p.start_assumption.c_str() : (p.reason + ": " + p.message).c_str());
    if (!p.backend_message.empty() && p.backend_message != "ok") {
      RCLCPP_INFO(logger, "      backend: %s", p.backend_message.c_str());
    }
  }
}

int run(const std::shared_ptr<rclcpp::Node> & node)
{
  const auto logger = node->get_logger();
  const auto config = mtc::moveit_planner::load_planner_config(*node);
  const auto options = mtc::moveit_planner::load_backend_options(*node);
  const std::string output_dir = node->get_parameter_or<std::string>("output_dir", "planning_evidence");
  const std::string tool = node->get_parameter_or<std::string>(
    "sequence.tool_type", mtc::manipulation::TOOL_PASSIVE_HOOK_V1);
  const std::string slot = node->get_parameter_or<std::string>("sequence.slot", "T0");
  // 规划起点：SRDF 命名状态（默认 "home"）；为空则用 /joint_states 当前状态。只规划，不移动机器人。
  const std::string start_named_state =
    node->get_parameter_or<std::string>("sequence.start_named_state", "home");
  // start_mode = "named_state"：从上面的命名状态出发；
  // start_mode = "ready"：先算"就绪位"——第一个接近原语的目标再沿插入轴后退 ready_backoff_m，
  // 工具姿态已与提环对齐——再从就绪位出发规划（命名状态只作 IK 种子与准备动作起点）。
  const std::string start_mode =
    node->get_parameter_or<std::string>("sequence.start_mode", "named_state");
  const double ready_backoff_m = node->get_parameter_or("sequence.ready_backoff_m", 0.10);
  // 准备动作（命名状态 -> 就绪位）的末端速度上限：默认与接近原语相同；
  // 设为 transport 的 0.15 m/s / 0.3 rad/s 可评估"快速接近 + 慢速预对准"拆分方案。
  const double setup_linear = node->get_parameter_or(
    "sequence.setup_max_linear_speed", mtc::manipulation::ToolStrategy::DEFAULT_LINEAR_SPEED);
  const double setup_angular = node->get_parameter_or(
    "sequence.setup_max_angular_speed", mtc::manipulation::ToolStrategy::DEFAULT_ANGULAR_SPEED);

  mtc::core::PoseEstimate ring;
  ring.object_id = node->get_parameter_or<std::string>("sequence.object_id", "battery_1");
  ring.color = static_cast<int>(node->get_parameter_or<int64_t>("sequence.color", 1));
  const auto xyz = node->get_parameter_or<std::vector<double>>(
    "sequence.ring.xyz", {0.40, -0.05, 0.13});
  const auto q = node->get_parameter_or<std::vector<double>>(
    "sequence.ring.quat_xyzw", {0.0, 0.70710678, 0.0, 0.70710678});
  if (xyz.size() != 3 || q.size() != 4) {
    RCLCPP_ERROR(logger, "sequence.ring.xyz needs 3 values, sequence.ring.quat_xyzw needs 4");
    return 2;
  }
  ring.has_ring_pose_value = true;
  ring.ring_pose_valid = true;
  ring.ring_pose.frame_id = node->get_parameter_or<std::string>("sequence.ring.frame_id", "base_link");
  ring.ring_pose.position = mtc::core::Vector3{xyz[0], xyz[1], xyz[2]};
  ring.ring_pose.orientation = mtc::core::Quaternion{q[0], q[1], q[2], q[3]};
  ring.ring_pose.stamp_s = 0.0;
  ring.ring_pose.valid = true;

  const auto strategy = mtc::manipulation::make_strategy(tool);
  auto backend = std::make_shared<MoveGroupBackend>(node, options);
  if (!backend->apply_scene(config.scene_boxes)) {
    RCLCPP_ERROR(logger, "failed to apply planning scene obstacles");
    return 2;
  }
  RCLCPP_INFO(logger, "planning scene: %zu static obstacle(s) applied",
    config.scene_boxes.size());
  MoveItPlannerPort port(backend, config, std::make_shared<mtc::core::SteadyClock>(), false);

  auto acquire_primitives = strategy->make_acquire_plan(ring);
  const auto test_lift = strategy->make_test_lift_plan(ring);
  acquire_primitives.insert(acquire_primitives.end(), test_lift.begin(), test_lift.end());
  const auto acquire = make_target("ACQUIRE", ring, slot, acquire_primitives);
  std::optional<JointStateSnapshot> acquire_start;
  if (!start_named_state.empty()) {
    acquire_start = backend->named_state(start_named_state);
    if (!acquire_start) {
      RCLCPP_ERROR(logger, "SRDF named state '%s' not found", start_named_state.c_str());
      return 2;
    }
  }
  std::ostringstream ready_yaml;
  if (start_mode == "ready") {
    if (!acquire_start) {
      RCLCPP_ERROR(logger, "start_mode=ready needs sequence.start_named_state as IK seed");
      return 2;
    }
    const auto & first = acquire_primitives.front().name;
    const auto spec = config.primitives.find(first);
    const auto ring_in_frame = backend->transform_to_planning_frame(ring.ring_pose);
    if (spec == config.primitives.end() || !ring_in_frame) {
      RCLCPP_ERROR(logger, "cannot derive ready pose for primitive '%s'", first.c_str());
      return 2;
    }
    auto ring_goal = *ring_in_frame;
    if (port.config().hook_up_alignment && port.config().insertion_axis == 2) {
      ring_goal.orientation = mtc::moveit_planner::align_roll_to_up(
        ring_goal.orientation, mtc::core::Vector3{0.0, 0.0, 1.0}, port.config().hook_up_min_tilt_rad);
    }
    const auto tcp_ready = mtc::moveit_planner::offset_along_local_axis(
      ring_goal, port.config().insertion_axis, -(spec->second.distance_m + ready_backoff_m));
    const auto ee_ready = mtc::moveit_planner::apply(
      tcp_ready, mtc::moveit_planner::inverse(port.config().ee_to_tcp));
    std::string info;
    const auto ready = backend->solve_ik_max_margin(
      ee_ready, *acquire_start, config.joint_position_limits, info, port.tool_bodies());
    if (!ready) {
      RCLCPP_ERROR(logger, "ready pose: %s", info.c_str());
      return 1;
    }
    std::ostringstream joints;
    for (size_t i = 0; i < ready->names.size(); ++i) {
      joints << (i ? ", " : "") << ready->names[i] << "=" << std::fixed << std::setprecision(4) <<
        ready->positions[i];
    }
    RCLCPP_INFO(logger, "ready pose (%s + %.3f m back along insertion axis): %s; %s",
      first.c_str(), ready_backoff_m, joints.str().c_str(), info.c_str());

    // 仅供参考：从命名状态到就绪位的一次性准备动作（不属于任何 PickPlace 状态，FSM 尚无此动作）。
    mtc::moveit_planner::BackendRequest setup;
    setup.label = "setup_to_ready";
    setup.mode = mtc::moveit_planner::MotionMode::FREE_SPACE;
    setup.ee_goal = ee_ready;
    setup.start = *acquire_start;
    setup.velocity_scaling = config.velocity_scaling;
    setup.acceleration_scaling = config.acceleration_scaling;
    setup.max_linear_speed_m_s = setup_linear;
    setup.max_angular_speed_rad_s = setup_angular;
    setup.planning_time_s = config.planning_time_s;
    setup.cartesian_step_m = config.cartesian_step_m;
    setup.prefer_straight_line = true;
    setup.joint_bounds = config.joint_position_limits;
    setup.tool_bodies = port.tool_bodies();
    const auto setup_result = backend->plan(setup);
    double setup_duration = 0.0;
    if (setup_result.ok && !setup_result.trajectory.points.empty()) {
      setup_duration = setup_result.trajectory.points.back().time_from_start_s;
    }
    RCLCPP_INFO(logger,
      "setup move %s -> ready (reference only, caps %.3f m/s %.3f rad/s): %s method=%s "
      "duration=%.2fs %s", start_named_state.c_str(), setup_linear, setup_angular,
      setup_result.ok ? "ok" : "FAILED", setup_result.method.c_str(), setup_duration,
      setup_result.ok ? "" : setup_result.message.c_str());
    ready_yaml << "ready_pose:\n";
    ready_yaml << "  derived_from: \"" << first << " goal + " << ready_backoff_m <<
      " m back along insertion axis\"\n";
    ready_yaml << "  joint_names: [";
    for (size_t i = 0; i < ready->names.size(); ++i) {
      ready_yaml << (i ? ", " : "") << "\"" << ready->names[i] << "\"";
    }
    ready_yaml << "]\n  positions_rad: [";
    for (size_t i = 0; i < ready->positions.size(); ++i) {
      ready_yaml << (i ? ", " : "") << std::setprecision(6) << ready->positions[i];
    }
    ready_yaml << "]\n  ik_info: \"" << info << "\"\n";
    ready_yaml << "  setup_move_reference_only:\n";
    ready_yaml << "    from: \"" << start_named_state << "\"\n";
    ready_yaml << "    ok: " << (setup_result.ok ? "true" : "false") << "\n";
    ready_yaml << "    method: \"" << setup_result.method << "\"\n";
    ready_yaml << "    duration_s: " << setup_duration << "\n";
    ready_yaml << "    max_linear_speed_m_s: " << setup_linear << "\n";
    ready_yaml << "    max_angular_speed_rad_s: " << setup_angular << "\n";
    ready_yaml << "    note: \"one-time move outside PickPlace; no FSM state/watchdog owns it yet\"\n";
    acquire_start = ready;
  } else if (start_mode != "named_state") {
    RCLCPP_ERROR(logger, "sequence.start_mode must be named_state or ready");
    return 2;
  }
  const auto acquire_result = port.plan_blocking(acquire, acquire_start);
  const auto acquire_report = port.last_report();
  log_report(logger, acquire_result, acquire_report);

  std::ostringstream yaml;
  yaml << "# mtc_moveit_planner plan_battery_sequence 证据（Mock，仅规划，未执行）\n";
  yaml << "# 提环位姿与工位均为占位值；接触段只代表规划意图，不代表可安全执行。\n";
  yaml << "tool: plan_battery_sequence\n";
  yaml << "wall_time: \"" << mtc::moveit_planner::timestamp_for_filename() << "\"\n";
  yaml << "tool_type: \"" << tool << "\"\n";
  yaml << "slot: \"" << slot << "\"\n";
  yaml << "start: \"" << (start_mode == "ready" ? std::string("ready pose (see ready_pose)") :
    start_named_state.empty() ? std::string("current /joint_states") :
    "SRDF named state " + start_named_state) << "\"\n";
  yaml << ready_yaml.str();
  yaml << "ring_pose:\n" << mtc::moveit_planner::pose_to_yaml(ring.ring_pose, "  ");
  yaml << "tcp:\n";
  yaml << "  status: \"" << mtc::moveit_planner::to_string(config.tcp_status) << "\"\n";
  yaml << "  xyz: [" << config.ee_to_tcp.translation.x << ", " << config.ee_to_tcp.translation.y <<
    ", " << config.ee_to_tcp.translation.z << "]\n";
  yaml << "phases:\n";
  yaml << "  - " << mtc::moveit_planner::plan_to_yaml(acquire_result, acquire_report, "    ").substr(4);

  bool ok = acquire_result.ok;
  if (acquire_result.ok && !acquire_result.commands.empty() &&
    acquire_result.commands.back().trajectory)
  {
    const auto & last = *acquire_result.commands.back().trajectory;
    const JointStateSnapshot after_test_lift{
      last.joint_names, mtc::moveit_planner::final_positions(last)};
    auto transport_primitives = strategy->make_lift_plan(ring);
    for (const auto & extra : {
        strategy->make_transport_plan(ring, slot), strategy->make_seat_plan(slot),
        strategy->make_seat_contact_plan(slot), strategy->make_retreat_plan(slot)})
    {
      transport_primitives.insert(transport_primitives.end(), extra.begin(), extra.end());
    }
    const auto transport = make_target("TRANSPORT", ring, slot, transport_primitives);
    const auto transport_result = port.plan_blocking(transport, after_test_lift);
    const auto transport_report = port.last_report();
    log_report(logger, transport_result, transport_report);
    yaml << "  - " <<
      mtc::moveit_planner::plan_to_yaml(transport_result, transport_report, "    ").substr(4);
    ok = ok && transport_result.ok;
  }

  const std::string path = output_dir + "/battery_sequence_" + tool + "_" + slot + "_" +
    mtc::moveit_planner::timestamp_for_filename() + ".yaml";
  if (!mtc::moveit_planner::write_text_file(path, yaml.str())) {
    RCLCPP_ERROR(logger, "failed to write %s", path.c_str());
    return 3;
  }
  RCLCPP_INFO(logger, "%s -> %s", ok ? "SEQUENCE PLAN OK" : "SEQUENCE PLAN FAILED", path.c_str());
  return ok ? 0 : 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("mtc_plan_battery_sequence", node_options);
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
