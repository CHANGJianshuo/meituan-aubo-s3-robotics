// Copyright 2026 meituan_challenge_ws contributors

#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "mtc_manipulation_cpp/pick_place_types.hpp"
#include "mtc_moveit_planner/trajectory_conversion.hpp"

namespace
{

using mtc::moveit_planner::RawTrajectory;
using mtc::moveit_planner::RawTrajectoryPoint;
using mtc::moveit_planner::stretch_time;
using mtc::moveit_planner::to_timed_trajectory;

const std::vector<std::string> & fsm_order()
{
  static const auto order = mtc::manipulation::PickPlaceConfig().joint_names;
  return order;
}

/// moveit_controllers.yaml 的列举顺序（upperArm / foreArm 与驱动控制器相反）。
RawTrajectory moveit_ordered()
{
  RawTrajectory raw;
  raw.joint_names = {"shoulder_joint", "foreArm_joint", "upperArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint"};
  raw.points.push_back(RawTrajectoryPoint{0.0, {1, 3, 2, 4, 5, 6}, {0, 0, 0, 0, 0, 0}, {}});
  raw.points.push_back(RawTrajectoryPoint{0.5, {1.1, 3.3, 2.2, 4.4, 5.5, 6.6},
      {0.1, 0.3, 0.2, 0.4, 0.5, 0.6}, {1, 3, 2, 4, 5, 6}});
  return raw;
}

TEST(TrajectoryConversion, ReordersByNameNotIndex)
{
  const auto result = to_timed_trajectory(moveit_ordered(), fsm_order(), "world", 2.0);
  ASSERT_TRUE(result.ok) << result.error;
  const auto & t = result.trajectory;
  EXPECT_EQ(t.joint_names, fsm_order());
  EXPECT_EQ(t.header_frame_id, "world");
  EXPECT_DOUBLE_EQ(t.stamp_s, 2.0);
  ASSERT_EQ(t.points.size(), 2u);
  EXPECT_EQ(t.points[0].positions_rad, (std::vector<double>{1, 2, 3, 4, 5, 6}));
  EXPECT_EQ(t.points[1].velocities_rad_s, (std::vector<double>{0.1, 0.2, 0.3, 0.4, 0.5, 0.6}));
  EXPECT_EQ(t.points[1].accelerations_rad_s2, (std::vector<double>{1, 2, 3, 4, 5, 6}));
  EXPECT_TRUE(t.points[0].accelerations_rad_s2.empty());
  EXPECT_TRUE(t.shape_is_valid());
  EXPECT_DOUBLE_EQ(t.duration_s(), 0.5);
}

TEST(TrajectoryConversion, RejectsMissingExtraOrDuplicateJoints)
{
  auto missing = moveit_ordered();
  missing.joint_names.back() = "gripper_joint";
  EXPECT_FALSE(to_timed_trajectory(missing, fsm_order(), "world", 0.0).ok);

  auto extra = moveit_ordered();
  extra.joint_names.push_back("rail_joint");
  for (auto & point : extra.points) {
    point.positions.push_back(0.0);
  }
  EXPECT_FALSE(to_timed_trajectory(extra, fsm_order(), "world", 0.0).ok);

  auto duplicate = moveit_ordered();
  duplicate.joint_names[1] = "shoulder_joint";
  EXPECT_FALSE(to_timed_trajectory(duplicate, fsm_order(), "world", 0.0).ok);
}

TEST(TrajectoryConversion, RejectsInvalidShapes)
{
  auto empty = moveit_ordered();
  empty.points.clear();
  EXPECT_FALSE(to_timed_trajectory(empty, fsm_order(), "world", 0.0).ok);

  EXPECT_FALSE(to_timed_trajectory(moveit_ordered(), fsm_order(), "", 0.0).ok);

  auto short_point = moveit_ordered();
  short_point.points[1].positions.pop_back();
  EXPECT_FALSE(to_timed_trajectory(short_point, fsm_order(), "world", 0.0).ok);

  auto non_increasing = moveit_ordered();
  non_increasing.points[1].time_from_start_s = 0.0;
  EXPECT_FALSE(to_timed_trajectory(non_increasing, fsm_order(), "world", 0.0).ok);

  auto nan_value = moveit_ordered();
  nan_value.points[1].positions[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(to_timed_trajectory(nan_value, fsm_order(), "world", 0.0).ok);
}

TEST(TrajectoryConversion, StretchTimeKeepsPathAndScalesDerivatives)
{
  auto converted = to_timed_trajectory(moveit_ordered(), fsm_order(), "world", 0.0);
  ASSERT_TRUE(converted.ok);
  auto t = converted.trajectory;
  ASSERT_TRUE(stretch_time(t, 2.0));
  EXPECT_DOUBLE_EQ(t.duration_s(), 1.0);
  EXPECT_DOUBLE_EQ(t.points[1].positions_rad[1], 2.2);
  EXPECT_DOUBLE_EQ(t.points[1].velocities_rad_s[1], 0.1);
  EXPECT_DOUBLE_EQ(t.points[1].accelerations_rad_s2[1], 0.5);
  EXPECT_FALSE(stretch_time(t, 0.5));
  EXPECT_DOUBLE_EQ(t.duration_s(), 1.0);
}

}  // namespace
