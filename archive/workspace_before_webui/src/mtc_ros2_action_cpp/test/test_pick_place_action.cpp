// Copyright 2026 MTC contributors
#include "ros_fixture.hpp"
#include "mtc_ros2_action_cpp/pick_place_server.hpp"
#include <limits>
#include <action_msgs/srv/cancel_goal.hpp>
using Pick = mtc_interfaces::action::PickPlace;
using Handle = rclcpp_action::ClientGoalHandle<Pick>;
using mtc::core::ErrorCodes;
class PickAction : public RosFixture
{
protected:
  void setup(mtc::ros2_action::MockOptions options = {})
  {
    server = std::make_shared<mtc::ros2_action::PickPlaceServer>(options);
    client = rclcpp_action::create_client<Pick>(client_node, "/mtc/manipulation/pick_place");
    start({server});
    ASSERT_TRUE(client->wait_for_action_server(3s));
  }
  Pick::Goal goal(std::string id = "pick_1")
  {
    Pick::Goal g;
    g.request_id = id;
    g.object_id = "mock_battery_1";
    g.expected_color = 1;
    g.target_slot = "T0";
    g.placement_stability_sec = 0.2f;
    return g;
  }
  Handle::SharedPtr send(const Pick::Goal & g)
  {
    auto options = rclcpp_action::Client<Pick>::SendGoalOptions();
    options.feedback_callback = [this](Handle::SharedPtr, const Pick::Feedback::ConstSharedPtr f) {
        std::lock_guard<std::mutex> lock(mutex);
        states.push_back(f->substate);
        if (f->detail.find("Mock command=") != std::string::npos) {dispatched = true;}
        EXPECT_GE(f->progress, 0);
        EXPECT_LE(f->progress, 1);
      };
    auto f = client->async_send_goal(g, options);
    EXPECT_TRUE(ready(f));
    return ready(f, 0ms) ? f.get() : nullptr;
  }
  bool seen(const std::string & name)
  {
    std::lock_guard<std::mutex> lock(mutex);
    return std::find(states.begin(), states.end(), name) != states.end();
  }
  std::shared_ptr<mtc::ros2_action::PickPlaceServer> server;
  rclcpp_action::Client<Pick>::SharedPtr client;
  std::mutex mutex;
  std::vector<std::string> states;
  std::atomic<bool> dispatched{false};
};
TEST_F(PickAction, LegalGoalSucceedsOnlyAfterPlacementFeedback)
{
  setup();
  auto h = send(goal()); ASSERT_NE(h, nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  const auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(r.result->success);
  EXPECT_TRUE(r.result->placement_verified);
  EXPECT_EQ(r.result->error_code, ErrorCodes::OK);
  EXPECT_TRUE(seen("VERIFY_PLACED"));
  EXPECT_TRUE(seen("VERIFY_ATTACHED"));
}
TEST_F(PickAction, InvalidParametersRejectedBeforeExecution)
{
  setup();
  auto g = goal(); g.expected_color = 0; EXPECT_EQ(send(g), nullptr);
  g = goal(); g.target_slot = "P4"; EXPECT_EQ(send(g), nullptr);
  g = goal(); g.object_id = ""; EXPECT_EQ(send(g), nullptr);
  for (float n : {0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity()}) {
    g = goal(); g.placement_stability_sec = n; EXPECT_EQ(send(g), nullptr);
  }
  auto h = send(goal()); ASSERT_NE(h, nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  EXPECT_TRUE(f.get().result->success);
}
TEST_F(PickAction, ConcurrentSecondGoalRejected)
{
  setup();
  auto h = send(goal()); ASSERT_NE(h, nullptr);
  EXPECT_EQ(send(goal("second")), nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  EXPECT_TRUE(f.get().result->success);
}
TEST_F(PickAction, MatchingCancelRequiresIndependentStationaryEvidence)
{
  setup();
  auto h = send(goal()); ASSERT_NE(h, nullptr);
  ASSERT_TRUE(eventually([this]() {return dispatched.load();}));
  auto c = client->async_cancel_goal(h); ASSERT_TRUE(ready(c));
  EXPECT_FALSE(c.get()->goals_canceling.empty());
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  const auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_FALSE(r.result->success);
  EXPECT_FALSE(r.result->placement_verified);
  EXPECT_EQ(r.result->error_code, ErrorCodes::OK);
}
TEST_F(PickAction, CancelWithoutStopEvidenceAbortsAndKeepsFaultLocked)
{
  mtc::ros2_action::MockOptions o; o.stop_evidence_available = false;
  setup(o);
  auto h = send(goal()); ASSERT_NE(h, nullptr);
  ASSERT_TRUE(eventually([this]() {return dispatched.load();}));
  auto c = client->async_cancel_goal(h); ASSERT_TRUE(ready(c));
  EXPECT_FALSE(c.get()->goals_canceling.empty());
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  const auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(r.result->error_code, ErrorCodes::CANCEL_NOT_CONFIRMED);
  EXPECT_FALSE(r.result->placement_verified);
  EXPECT_EQ(send(goal("after_fault")), nullptr);
}
TEST_F(PickAction, StaleGoalCancelCannotInterruptNewGoal)
{
  setup();
  auto a = send(goal()); ASSERT_NE(a, nullptr);
  auto af = client->async_get_result(a); ASSERT_TRUE(ready(af));
  ASSERT_TRUE(af.get().result->success);
  auto b = send(goal("new_goal")); ASSERT_NE(b, nullptr);
  auto cancel = client_node->create_client<action_msgs::srv::CancelGoal>(
    "/mtc/manipulation/pick_place/_action/cancel_goal");
  ASSERT_TRUE(cancel->wait_for_service(3s));
  auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
  request->goal_info.goal_id.uuid = a->get_goal_id();
  auto c = cancel->async_send_request(request); ASSERT_TRUE(ready(c));
  EXPECT_TRUE(c.get()->goals_canceling.empty());
  auto bf = client->async_get_result(b); ASSERT_TRUE(ready(bf));
  EXPECT_EQ(bf.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(server->mock_call_counts()->stops.load(), 0u);
  EXPECT_EQ(send(goal()), nullptr);  // Business request ID cannot be replayed.
}
TEST_F(PickAction, MotionStatusUnknownNeverResendsOrAutoClearsFault)
{
  mtc::ros2_action::MockOptions o; o.motion.status_unknown = true;
  setup(o);
  auto h = send(goal()); ASSERT_NE(h, nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  const auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(r.result->error_code, ErrorCodes::MOTION_STATUS_UNKNOWN);
  EXPECT_EQ(send(goal("after_unknown")), nullptr);
  EXPECT_EQ(server->mock_call_counts()->executions.load(), 1u);
}
TEST_F(PickAction, MissingPlanNeverSucceeds)
{
  mtc::ros2_action::MockOptions o; o.plan_available = false;
  setup(o);
  auto h = send(goal()); ASSERT_NE(h, nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  const auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(r.result->error_code, ErrorCodes::PLANNING_FAILED);
  EXPECT_FALSE(r.result->placement_verified);
}
TEST_F(PickAction, ConcurrentRosRequestsHaveExactlyOneOwner)
{
  setup();
  std::shared_future<Handle::SharedPtr> a, b;
  std::atomic<int> barrier{0};
  std::thread first([&]() {
    ++barrier;
    while (barrier < 2) {std::this_thread::yield();}
    a = client->async_send_goal(goal("race_a"));
  });
  std::thread second([&]() {
    ++barrier;
    while (barrier < 2) {std::this_thread::yield();}
    b = client->async_send_goal(goal("race_b"));
  });
  first.join(); second.join();
  ASSERT_TRUE(ready(a)); ASSERT_TRUE(ready(b));
  const auto ah = a.get(), bh = b.get();
  EXPECT_EQ(static_cast<int>(!!ah) + static_cast<int>(!!bh), 1);
  auto winner = ah ? ah : bh; ASSERT_NE(winner, nullptr);
  auto result = client->async_get_result(winner); ASSERT_TRUE(ready(result));
  EXPECT_TRUE(result.get().result->success);
}
TEST_F(PickAction, InFlightWatchdogFaultStillDrainsAssociatedStopWorker)
{
  mtc::ros2_action::MockOptions o;
  o.pick.state_timeouts[mtc::manipulation::PickPlaceState::MOVE_PRE_ALIGN] = 0.01;
  setup(o); auto h = send(goal()); ASSERT_NE(h, nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(r.result->error_code, ErrorCodes::MOTION_STATUS_UNKNOWN);
  EXPECT_EQ(send(goal("after_watchdog")), nullptr);
  EXPECT_TRUE(eventually([this]() {return server->mock_call_counts()->stops.load() == 1;}));
  EXPECT_EQ(server->mock_call_counts()->executions.load(), 1u);
}
TEST_F(PickAction, CancelWhileCarryingStopsButKeepsLoadAdmissionLocked)
{
  setup(); auto h = send(goal()); ASSERT_NE(h, nullptr);
  ASSERT_TRUE(eventually([this]() {return seen("TRANSPORT");}));
  auto c = client->async_cancel_goal(h); ASSERT_TRUE(ready(c));
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(r.result->error_code, ErrorCodes::OK);
  EXPECT_EQ(send(goal("carrying_must_not_restart")), nullptr);
}
TEST_F(PickAction, UnverifiedAttachmentLoadCannotBeClearedByNewGoal)
{
  mtc::ros2_action::MockOptions o; o.attached = false;
  setup(o); auto h = send(goal()); ASSERT_NE(h, nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(r.result->success);
  const auto count = server->mock_call_counts()->executions.load();
  EXPECT_EQ(send(goal("unknown_load_must_not_restart")), nullptr);
  EXPECT_EQ(server->mock_call_counts()->executions.load(), count);
}
