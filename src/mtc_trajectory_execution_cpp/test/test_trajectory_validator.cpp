#include <gtest/gtest.h>
#include <limits>
#include <algorithm>
#include "mtc_trajectory_execution_cpp/trajectory_validator.hpp"
using namespace mtc::trajectory;
Config config()
{
  Config c; for (const auto & n : c.joints) {c.limits[n] = {-2, 2, 1, 10};} return c;
}
mtc::core::MotionCommand command()
{
  auto cfg = config(); mtc::core::MotionCommand c;
  c.command_id = "command-1"; c.simulation_only = true;
  mtc::core::TimedTrajectory t; t.trajectory_id = "plan-1"; t.joint_names = cfg.joints;
  t.header_frame_id = "base_link";
  for (int i = 0; i < 3; ++i) {
    mtc::core::TrajectoryPoint p; p.time_from_start_s = i * 0.3;
    p.positions_rad.assign(6, i * 0.01); p.velocities_rad_s.assign(6, 0);
    p.accelerations_rad_s2.assign(6, 0); t.points.push_back(p);
  }
  c.trajectory = t; return c;
}
TEST(TrajectoryValidator, CompleteTrajectoryPreserved) {
  auto c = command(); c.trajectory->stamp_s = 12.25;
  auto r = convert(c, config()); ASSERT_TRUE(r.trajectory) << r.error;
  EXPECT_EQ(r.trajectory->points.size(), 3u); EXPECT_EQ(r.trajectory->header.frame_id, "base_link");
  EXPECT_EQ(r.trajectory->header.stamp.sec, 12); EXPECT_EQ(r.trajectory->header.stamp.nanosec, 250000000u);
  EXPECT_EQ(r.trajectory->points.back().time_from_start.nanosec, 600000000u);
  EXPECT_EQ(r.trajectory->points.back().positions, c.trajectory->points.back().positions_rad);
}
TEST(TrajectoryValidator, JointOrderMapsAllFields) {
  auto c = command(); std::reverse(c.trajectory->joint_names.begin(), c.trajectory->joint_names.end());
  for (auto & p : c.trajectory->points) {for (size_t j=0;j<6;++j) {
    p.positions_rad[j]=j*0.01; p.velocities_rad_s[j]=j*0.02; p.accelerations_rad_s2[j]=j*0.03;
  }}
  auto r=convert(c,config()); ASSERT_TRUE(r.trajectory) << r.error;
  EXPECT_DOUBLE_EQ(r.trajectory->points[0].positions[0],0.05);
  EXPECT_DOUBLE_EQ(r.trajectory->points[0].velocities[0],0.1);
  EXPECT_DOUBLE_EQ(r.trajectory->points[0].accelerations[0],0.15);
}
#define INVALID_TEST(Name, Mutation) TEST(TrajectoryValidator, Name) {auto c=command(); Mutation; EXPECT_FALSE(convert(c,config()).trajectory);}
INVALID_TEST(EmptyTrajectoryRejected,c.trajectory->points.clear())
INVALID_TEST(SinglePointRejected,c.trajectory->points.resize(1))
INVALID_TEST(DuplicateJointRejected,c.trajectory->joint_names[1]=c.trajectory->joint_names[0])
INVALID_TEST(MissingJointRejected,c.trajectory->joint_names[1]="unknown")
INVALID_TEST(NonIncreasingTimeRejected,c.trajectory->points[1].time_from_start_s=0)
INVALID_TEST(NanPositionRejected,c.trajectory->points[0].positions_rad[0]=std::numeric_limits<double>::quiet_NaN())
INVALID_TEST(InfVelocityRejected,c.trajectory->points[0].velocities_rad_s[0]=std::numeric_limits<double>::infinity())
INVALID_TEST(InfAccelerationRejected,c.trajectory->points[0].accelerations_rad_s2[0]=std::numeric_limits<double>::infinity())
INVALID_TEST(DimensionMismatchRejected,c.trajectory->points[0].positions_rad.pop_back())
INVALID_TEST(FirstPointNotZeroRejected,c.trajectory->points[0].time_from_start_s=0.01)
INVALID_TEST(PositionLimitRejected,c.trajectory->points[0].positions_rad[0]=3)
INVALID_TEST(VelocityLimitRejected,c.trajectory->points[0].velocities_rad_s[0]=2)
INVALID_TEST(AccelerationLimitRejected,c.trajectory->points[0].accelerations_rad_s2[0]=11)
INVALID_TEST(TimeoutZeroRejected,c.timeout_ms=0)
INVALID_TEST(InsufficientTimeoutRejected,c.timeout_ms=400)
INVALID_TEST(RealCommandRejected,c.simulation_only=false)
INVALID_TEST(ContactRequestRejected,c.requires_contact=true)
INVALID_TEST(NanTimeRejected,c.trajectory->points[1].time_from_start_s=std::numeric_limits<double>::quiet_NaN())
INVALID_TEST(NegativeStampRejected,c.trajectory->stamp_s=-1)
INVALID_TEST(NanStampRejected,c.trajectory->stamp_s=std::numeric_limits<double>::quiet_NaN())
INVALID_TEST(NanosecondTimeCollisionRejected,c.trajectory->points[1].time_from_start_s=1e-12)
INVALID_TEST(MixedRepresentationsRejected,c.target_rad.assign(6,0))
TEST(TrajectoryValidator, MissingLimitsRejectConfiguration) {Config c; EXPECT_THROW(c.validate(),std::invalid_argument);}
TEST(TrajectoryValidator, ZeroSettleRejected) {auto c=config(); c.settle_duration=0; EXPECT_THROW(c.validate(),std::invalid_argument);}
TEST(TrajectoryValidator, JointSampleRequiresVelocityAndIdentity) {
  sensor_msgs::msg::JointState s; auto c=config(); s.name=c.joints;
  s.header.stamp.sec=1; s.position.assign(6,0); EXPECT_FALSE(map_sample(s,c));
  s.velocity.assign(6,0); ASSERT_TRUE(map_sample(s,c));
  s.name[1]=s.name[0]; EXPECT_FALSE(map_sample(s,c));
}

TEST(TrajectoryValidator, SplinePositionOvershootRejected) {
  auto c=command(); for(auto & p:c.trajectory->points) {
    p.positions_rad.assign(6,1.99); p.velocities_rad_s.assign(6,0.2);
  } EXPECT_FALSE(convert(c,config()).trajectory);
}
TEST(TrajectoryValidator, SplineVelocityOvershootRejected) {
  auto c=command(); c.trajectory->points[1].positions_rad.assign(6,0.1);
  c.trajectory->points[2].positions_rad.assign(6,0.2);
  EXPECT_FALSE(convert(c,config()).trajectory);
}
TEST(TrajectoryValidator, SplineAccelerationOvershootRejected) {
  auto c=command(); auto cfg=config(); for(auto & kv:cfg.limits) {kv.second.acceleration=0.1;}
  EXPECT_FALSE(convert(c,cfg).trajectory);
}
