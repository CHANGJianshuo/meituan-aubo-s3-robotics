// Copyright 2026 MTC contributors
#include "ros_fixture.hpp"
#include "mtc_ros2_action_cpp/motion_server.hpp"
#include <action_msgs/srv/cancel_goal.hpp>
#include <limits>
using Move = mtc_interfaces::action::ExecuteJointMove;
using MoveHandle = rclcpp_action::ClientGoalHandle<Move>;
using mtc::core::ErrorCodes;
class MotionAction : public RosFixture
{
protected:
  void setup(mtc::ros2_action::MockOptions options = {})
  {
    server = std::make_shared<mtc::ros2_action::MotionServer>(options);
    client = rclcpp_action::create_client<Move>(client_node, "/mtc/motion/execute_joint_move");
    start({server}); ASSERT_TRUE(client->wait_for_action_server(3s));
  }
  Move::Goal goal(std::string id = "move_1")
  {
    Move::Goal g; g.command_id = id;
    const auto names = mtc::motion::ValidationConfig().expected_joint_names;
    std::copy(names.begin(), names.end(), g.joint_names.begin());
    g.target_rad = {0.05, -0.02, 0.01, 0.03, -0.04, 0.02};
    g.velocity_scaling = 0.2f; g.acceleration_scaling = 0.2f; g.timeout_ms = 2000;
    return g;
  }
  MoveHandle::SharedPtr send(const Move::Goal & g)
  {
    auto options = rclcpp_action::Client<Move>::SendGoalOptions();
    options.feedback_callback = [this](MoveHandle::SharedPtr, Move::Feedback::ConstSharedPtr f) {
        ++feedback_count;
        EXPECT_FALSE(f->execution_state.empty());
        for (double p : f->actual_position_rad) {EXPECT_TRUE(std::isfinite(p));}
      };
    auto f = client->async_send_goal(g, options); EXPECT_TRUE(ready(f));
    return ready(f, 0ms) ? f.get() : nullptr;
  }
  std::shared_ptr<mtc::ros2_action::MotionServer> server;
  rclcpp_action::Client<Move>::SharedPtr client;
  std::atomic<size_t> feedback_count{0};
};
TEST_F(MotionAction, LegalNonzeroGoalAndFeedbackUseExecutorResult)
{
  setup(); auto g = goal(); auto h = send(g); ASSERT_NE(h, nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(r.result->success); EXPECT_TRUE(r.result->stop_confirmed);
  EXPECT_EQ(r.result->final_position_rad, g.target_rad);
  EXPECT_GT(feedback_count.load(), 0u);
  EXPECT_EQ(server->mock_call_counts()->executions.load(), 1u);
}
TEST_F(MotionAction, IllegalGoalRejectedWithoutBackendCall)
{
  setup(); auto g = goal(); g.target_rad[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(send(g), nullptr);
  g = goal(); g.joint_names[0] = "bad"; EXPECT_EQ(send(g), nullptr);
  g = goal(); g.timeout_ms = 0; EXPECT_EQ(send(g), nullptr);
  g = goal(); g.acceleration_scaling = 1; EXPECT_EQ(send(g), nullptr);
  EXPECT_EQ(server->mock_call_counts()->executions.load(), 0u);
}
TEST_F(MotionAction, ConcurrentGoalIsRejectedAndFirstCompletes)
{
  setup(); auto h = send(goal()); ASSERT_NE(h, nullptr);
  EXPECT_EQ(send(goal("second")), nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  EXPECT_TRUE(f.get().result->success);
}
TEST_F(MotionAction, MatchingCancelStopsExactlyOnce)
{
  setup(); auto h = send(goal()); ASSERT_NE(h, nullptr);
  ASSERT_TRUE(eventually([this]() {return server->mock_call_counts()->executions > 0;}));
  auto c = client->async_cancel_goal(h); ASSERT_TRUE(ready(c));
  EXPECT_FALSE(c.get()->goals_canceling.empty());
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_TRUE(r.result->stop_confirmed); EXPECT_FALSE(r.result->success);
  EXPECT_EQ(server->mock_call_counts()->stops.load(), 1u);
}
TEST_F(MotionAction, UnconfirmedCancelAborts520AndLocks)
{
  mtc::ros2_action::MockOptions o; o.stop_evidence_available = false;
  setup(o); auto h = send(goal()); ASSERT_NE(h, nullptr);
  ASSERT_TRUE(eventually([this]() {return server->mock_call_counts()->executions > 0;}));
  auto c = client->async_cancel_goal(h); ASSERT_TRUE(ready(c));
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(r.result->stop_confirmed);
  EXPECT_EQ(r.result->error_code, ErrorCodes::CANCEL_NOT_CONFIRMED);
  EXPECT_EQ(send(goal("after_fault")), nullptr);
}
TEST_F(MotionAction, StaleCancelCannotReachBackendOrInterruptNewOwner)
{
  setup(); auto a = send(goal()); ASSERT_NE(a, nullptr);
  auto af = client->async_get_result(a); ASSERT_TRUE(ready(af)); EXPECT_TRUE(af.get().result->success);
  auto b = send(goal("new_owner")); ASSERT_NE(b, nullptr);
  auto service = client_node->create_client<action_msgs::srv::CancelGoal>(
    "/mtc/motion/execute_joint_move/_action/cancel_goal");
  ASSERT_TRUE(service->wait_for_service(3s));
  auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
  request->goal_info.goal_id.uuid = a->get_goal_id();
  auto cancel = service->async_send_request(request); ASSERT_TRUE(ready(cancel));
  EXPECT_TRUE(cancel.get()->goals_canceling.empty());
  auto bf = client->async_get_result(b); ASSERT_TRUE(ready(bf)); EXPECT_TRUE(bf.get().result->success);
  EXPECT_EQ(server->mock_call_counts()->stops.load(), 0u);
  EXPECT_EQ(server->mock_call_counts()->executions.load(), 2u);
}
TEST_F(MotionAction, ExecutorTimeoutKeepsOriginal220AndRejectsNewMotion)
{
  mtc::ros2_action::MockOptions o; o.motion.timeout = true;
  setup(o); auto h = send(goal()); ASSERT_NE(h, nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_EQ(r.result->error_code, ErrorCodes::MOTION_TIMEOUT);
  EXPECT_FALSE(r.result->stop_confirmed);
  EXPECT_EQ(send(goal("after_timeout")), nullptr);
  EXPECT_EQ(server->mock_call_counts()->executions.load(), 1u);
}
TEST_F(MotionAction, RosDeadlineIndependentlyRequestsAssociatedStop)
{
  mtc::ros2_action::MockOptions o; o.result_delay_ms = 2000;
  setup(o); auto g = goal(); g.timeout_ms = 800;
  auto h = send(g); ASSERT_NE(h, nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(r.result->error_code, ErrorCodes::MOTION_TIMEOUT);
  EXPECT_FALSE(r.result->success); EXPECT_TRUE(r.result->stop_confirmed);
  EXPECT_EQ(server->mock_call_counts()->stops.load(), 1u);
}
