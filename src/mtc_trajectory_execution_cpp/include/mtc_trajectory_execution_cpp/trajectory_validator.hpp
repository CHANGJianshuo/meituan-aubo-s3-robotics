// Copyright 2026 MTC contributors
#pragma once
#include <map>
#include <optional>
#include <string>
#include <vector>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <mtc_core_cpp/motion_primitive.hpp>

namespace mtc::trajectory
{
struct JointLimit {double minimum, maximum, velocity, acceleration;};
struct Config
{
  // Never inferred from simulation_only or a backend name. Runtime proofs also required.
  bool enable_fake_execution = false;
  std::string controller_manager = "/controller_manager";
  std::string controller = "joint_trajectory_controller";
  std::string hardware_component = "auboHardwareInterface";
  std::string joint_state_topic = "/joint_states";
  std::vector<std::string> joints = {"shoulder_joint", "upperArm_joint", "foreArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint"};
  std::map<std::string, JointLimit> limits;  // Mandatory; no implicit device limits.
  double start_tolerance = 0.01, final_tolerance = 0.005;
  double stationary_velocity = 0.005, settle_duration = 0.2;
  double feedback_max_age = 0.3, proof_max_age = 0.7;
  double max_duration = 60.0, max_start_delay = 0.2;
  uint32_t max_timeout_ms = 120000;
  size_t max_points = 10000;
  void validate() const;
  std::string action_name() const {return "/" + controller + "/follow_joint_trajectory";}
};
struct JointSample
{
  std::vector<double> positions, velocities;
  int64_t stamp_ns = 0;
};
struct Validation
{
  std::optional<trajectory_msgs::msg::JointTrajectory> trajectory;
  std::string error;
};
// Pure conversion; every vector is mapped by name to configured controller order.
Validation convert(const mtc::core::MotionCommand & command, const Config & config);
std::optional<JointSample> map_sample(
  const sensor_msgs::msg::JointState & state, const Config & config);
bool near(const std::vector<double> & a, const std::vector<double> & b, double tolerance);
}  // namespace mtc::trajectory
