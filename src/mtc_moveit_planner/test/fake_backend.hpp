// Copyright 2026 meituan_challenge_ws contributors
//
// 离线假规划后端：用"笛卡尔坐标即关节值"的玩具运动学验证 MoveItPlannerPort 的组装逻辑。
//   shoulder_joint = x, upperArm_joint = y, foreArm_joint = z, 三个腕关节恒为 0；
//   末端姿态恒为单位四元数。
// 关节顺序刻意与 FSM 不同（upperArm/foreArm 交换），用来验证按名称重排。
// 仅用于测试，不代表 AUBO S3 运动学，输出不能送往任何执行后端。

#ifndef MTC_MOVEIT_PLANNER__TEST__FAKE_BACKEND_HPP_
#define MTC_MOVEIT_PLANNER__TEST__FAKE_BACKEND_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "mtc_moveit_planner/planning_backend.hpp"
#include "mtc_moveit_planner/pose_math.hpp"

namespace mtc
{
namespace moveit_planner
{
namespace testing
{

class FakeBackend : public PlanningBackend
{
public:
  std::vector<std::string> names = {
    "shoulder_joint", "foreArm_joint", "upperArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint"};
  mtc::core::Vector3 start_xyz{0.30, 0.0, 0.40};
  bool has_current_state = true;

  std::set<std::string> fail_labels;   ///< 这些 label 的请求返回 NO_PLAN
  bool start_elsewhere = false;        ///< 轨迹首点偏离请求起点
  bool drop_last_joint = false;        ///< 输出缺一个关节
  int delay_ms = 0;

  std::atomic<int> plan_calls{0};
  std::vector<BackendRequest> requests;

  std::string planning_frame() const override {return "world";}
  std::string end_effector_link() const override {return "ee_link";}

  std::optional<JointStateSnapshot> current_state() override
  {
    if (!has_current_state) {
      return std::nullopt;
    }
    return state_at(start_xyz);
  }

  std::optional<mtc::core::Pose6D> end_effector_pose(const JointStateSnapshot & state) override
  {
    const auto xyz = xyz_of(state);
    if (!xyz) {
      return std::nullopt;
    }
    mtc::core::Pose6D pose;
    pose.position = *xyz;
    pose.frame_id = "world";
    pose.valid = true;
    return pose;
  }

  std::optional<mtc::core::Pose6D> transform_to_planning_frame(
    const mtc::core::Pose6D & pose) override
  {
    if (pose.frame_id != "world" && pose.frame_id != "base_link") {
      return std::nullopt;
    }
    auto out = pose;
    out.frame_id = "world";
    return out;
  }

  BackendResult plan(const BackendRequest & request) override
  {
    ++plan_calls;
    requests.push_back(request);
    if (delay_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    BackendResult result;
    if (fail_labels.count(request.label) != 0u) {
      result.reason = "NO_PLAN";
      result.message = "fake planner failure";
      return result;
    }
    const auto from = xyz_of(request.start);
    if (!from) {
      result.reason = "INVALID_START";
      return result;
    }
    auto start = *from;
    if (start_elsewhere) {
      start.x += 0.05;
    }
    const auto goal = request.ee_goal.position;
    const double distance = distance_between(start, goal);
    const double speed = request.max_linear_speed_m_s > 0.0 ? request.max_linear_speed_m_s : 0.1;
    const double duration = std::max(0.1, distance / speed);
    constexpr int kSegments = 4;

    result.trajectory.joint_names = names;
    for (int i = 0; i <= kSegments; ++i) {
      const double s = static_cast<double>(i) / kSegments;
      const mtc::core::Vector3 p{
        start.x + (goal.x - start.x) * s,
        start.y + (goal.y - start.y) * s,
        start.z + (goal.z - start.z) * s};
      RawTrajectoryPoint point;
      point.time_from_start_s = duration * s;
      point.positions = state_at(p).positions;
      point.velocities.assign(names.size(), 0.0);
      result.trajectory.points.push_back(point);
    }
    if (drop_last_joint) {
      result.trajectory.joint_names.pop_back();
      for (auto & point : result.trajectory.points) {
        point.positions.pop_back();
        point.velocities.pop_back();
      }
    }
    result.ok = true;
    result.planning_time_s = 0.01;
    result.cartesian_fraction = request.mode == MotionMode::CARTESIAN ? 1.0 : 0.0;
    result.end_state = state_at(goal);
    return result;
  }

  JointStateSnapshot state_at(const mtc::core::Vector3 & xyz) const
  {
    JointStateSnapshot state;
    state.names = names;
    state.positions.assign(names.size(), 0.0);
    for (size_t i = 0; i < names.size(); ++i) {
      if (names[i] == "shoulder_joint") {state.positions[i] = xyz.x;}
      if (names[i] == "upperArm_joint") {state.positions[i] = xyz.y;}
      if (names[i] == "foreArm_joint") {state.positions[i] = xyz.z;}
    }
    return state;
  }

  static std::optional<mtc::core::Vector3> xyz_of(const JointStateSnapshot & state)
  {
    if (!state.shape_is_valid()) {
      return std::nullopt;
    }
    mtc::core::Vector3 xyz;
    int found = 0;
    for (size_t i = 0; i < state.names.size(); ++i) {
      if (state.names[i] == "shoulder_joint") {xyz.x = state.positions[i]; ++found;}
      if (state.names[i] == "upperArm_joint") {xyz.y = state.positions[i]; ++found;}
      if (state.names[i] == "foreArm_joint") {xyz.z = state.positions[i]; ++found;}
    }
    if (found != 3) {
      return std::nullopt;
    }
    return xyz;
  }
};

}  // namespace testing
}  // namespace moveit_planner
}  // namespace mtc

#endif  // MTC_MOVEIT_PLANNER__TEST__FAKE_BACKEND_HPP_
