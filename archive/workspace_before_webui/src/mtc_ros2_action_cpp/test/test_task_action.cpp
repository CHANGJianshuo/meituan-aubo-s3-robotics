// Copyright 2026 MTC contributors
#include "ros_fixture.hpp"
#include "mtc_ros2_action_cpp/task_server.hpp"
#include "mtc_ros2_action_cpp/pick_place_server.hpp"
#include <action_msgs/srv/cancel_goal.hpp>
using Task = mtc_interfaces::action::ExecuteTask;
using TaskHandle = rclcpp_action::ClientGoalHandle<Task>;
using mtc::core::ErrorCodes;
class TaskAction : public RosFixture
{
protected:
  void setup(mtc::ros2_action::MockOptions options = {},
    mtc::task::FsmsConfig config = mtc::task::FsmsConfig::defaults(), bool with_child = true)
  {
    parent = std::make_shared<mtc::ros2_action::TaskServer>(config);
    client = rclcpp_action::create_client<Task>(client_node, "/mtc/task/execute");
    std::vector<std::shared_ptr<rclcpp::Node>> nodes{parent};
    if (with_child) {
      child = std::make_shared<mtc::ros2_action::PickPlaceServer>(options);
      nodes.push_back(child);
    }
    start(nodes);
    ASSERT_TRUE(client->wait_for_action_server(3s));
  }
  Task::Goal goal(std::string id = "task_1")
  {
    Task::Goal g;
    g.task_id = id; g.mode = Task::Goal::MODE_BASIC;
    g.ordered_colors = {1}; g.timeout_ms = 10000; g.placement_stability_sec = 0.2f;
    return g;
  }
  TaskHandle::SharedPtr send(const Task::Goal & g)
  {
    auto options = rclcpp_action::Client<Task>::SendGoalOptions();
    options.feedback_callback = [this](TaskHandle::SharedPtr,
      const Task::Feedback::ConstSharedPtr f) {
        std::lock_guard<std::mutex> lock(mutex);
        feedback.push_back(*f);
      };
    auto f = client->async_send_goal(g, options);
    EXPECT_TRUE(ready(f));
    return ready(f, 0ms) ? f.get() : nullptr;
  }
  bool child_moving()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return std::any_of(feedback.begin(), feedback.end(), [](auto & f) {
      return f.substate == "MOVE_PRE_ALIGN";
    });
  }
  void cancel_after_dispatch(const TaskHandle::SharedPtr & h)
  {
    ASSERT_TRUE(eventually([this]() {return child && child->mock_call_counts()->executions > 0;}));
    auto c = client->async_cancel_goal(h); ASSERT_TRUE(ready(c));
    EXPECT_FALSE(c.get()->goals_canceling.empty());
  }
  std::shared_ptr<mtc::ros2_action::TaskServer> parent;
  std::shared_ptr<mtc::ros2_action::PickPlaceServer> child;
  rclcpp_action::Client<Task>::SharedPtr client;
  std::mutex mutex;
  std::vector<Task::Feedback> feedback;
};
TEST_F(TaskAction, BasicTaskUsesRealChildActionAndRecordsT0)
{
  setup();
  auto h = send(goal()); ASSERT_NE(h, nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(r.result->success);
  EXPECT_EQ(r.result->completed_count, 1);
  EXPECT_EQ(r.result->completed_slots, std::vector<std::string>{"T0"});
  EXPECT_GT(child->mock_call_counts()->executions.load(), 0u);
  EXPECT_TRUE(child_moving());
  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_TRUE(std::any_of(feedback.begin(), feedback.end(), [](auto & f) {
    return f.total_count == 1 && f.current_slot == "T0" &&
      f.current_object_id == "mock_battery_color_1";
  }));
}
TEST_F(TaskAction, SequenceTaskRecordsAllSlotsInOrder)
{
  setup();
  auto g = goal(); g.mode = Task::Goal::MODE_SEQUENCE; g.ordered_colors = {3, 1, 2};
  auto h = send(g); ASSERT_NE(h, nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_TRUE(r.result->success);
  EXPECT_EQ(r.result->completed_count, 3);
  EXPECT_EQ(r.result->completed_slots, (std::vector<std::string>{"P1", "P2", "P3"}));
}
TEST_F(TaskAction, InvalidColorsAndConcurrentTaskRejected)
{
  setup();
  auto g = goal(); g.ordered_colors = {0}; EXPECT_EQ(send(g), nullptr);
  g = goal(); g.mode = Task::Goal::MODE_SEQUENCE; g.ordered_colors = {1, 1, 3};
  EXPECT_EQ(send(g), nullptr);
  g = goal(); g.placement_stability_sec = 0; EXPECT_EQ(send(g), nullptr);
  auto h = send(goal()); ASSERT_NE(h, nullptr);
  EXPECT_EQ(send(goal("second")), nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  EXPECT_TRUE(f.get().result->success);
}
TEST_F(TaskAction, CancelAckWaitsForChildFinalStopResult)
{
  setup();
  auto h = send(goal()); ASSERT_NE(h, nullptr);
  cancel_after_dispatch(h);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(r.result->completed_count, 0);
  EXPECT_EQ(r.result->error_code, ErrorCodes::OK);
  EXPECT_EQ(child->mock_call_counts()->stops.load(), 1u);
}
TEST_F(TaskAction, UnconfirmedChildCancelPreserves520AndLocksParent)
{
  mtc::ros2_action::MockOptions o; o.stop_evidence_available = false;
  setup(o);
  auto h = send(goal()); ASSERT_NE(h, nullptr);
  cancel_after_dispatch(h);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(r.result->error_code, ErrorCodes::CANCEL_NOT_CONFIRMED);
  EXPECT_EQ(r.result->completed_count, 0);
  EXPECT_EQ(send(goal("after_fault")), nullptr);
}
TEST_F(TaskAction, ChildStatusUnknownPropagatesOriginalError)
{
  mtc::ros2_action::MockOptions o; o.motion.status_unknown = true;
  setup(o);
  auto h = send(goal()); ASSERT_NE(h, nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_EQ(r.result->error_code, ErrorCodes::MOTION_STATUS_UNKNOWN);
  EXPECT_EQ(r.result->completed_count, 0);
  EXPECT_EQ(child->mock_call_counts()->executions.load(), 1u);
  EXPECT_EQ(send(goal("after_fault")), nullptr);
}
TEST_F(TaskAction, ChildPlanningFailurePreservesErrorWithoutFakeSuccess)
{
  mtc::ros2_action::MockOptions o; o.plan_available = false;
  setup(o);
  auto h = send(goal()); ASSERT_NE(h, nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_FALSE(r.result->success);
  EXPECT_EQ(r.result->error_code, ErrorCodes::PLANNING_FAILED);
  EXPECT_EQ(r.result->completed_count, 0);
  EXPECT_TRUE(r.result->completed_slots.empty());
  EXPECT_EQ(child->mock_call_counts()->executions.load(), 0u);
}
TEST_F(TaskAction, TaskDeadlineAbortsAndCancelsOriginalChild)
{
  setup();
  auto g = goal(); g.timeout_ms = 150;
  auto h = send(g); ASSERT_NE(h, nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(r.result->error_code, ErrorCodes::MOTION_TIMEOUT);
  EXPECT_EQ(send(goal("after_deadline")), nullptr);
  EXPECT_TRUE(eventually([this]() {return child->mock_call_counts()->stops.load() == 1;}));
}
TEST_F(TaskAction, MissingChildNeverFabricatesTaskSuccess)
{
  setup({}, mtc::task::FsmsConfig::defaults(), false);
  auto g = goal(); g.timeout_ms = 100;
  auto h = send(g); ASSERT_NE(h, nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  const auto r = f.get();
  EXPECT_EQ(r.result->error_code, ErrorCodes::MOTION_TIMEOUT);
  EXPECT_FALSE(r.result->success);
  EXPECT_EQ(r.result->completed_count, 0);
}
TEST_F(TaskAction, ChildCancelAckWithoutTerminalEvidenceCannotCancelParent)
{
  using Pick = mtc_interfaces::action::PickPlace;
  auto adversary = std::make_shared<rclcpp::Node>("mock_child_ack_only");
  std::atomic<bool> cancel_ack{false};
  std::atomic<bool> accepted{false};
  auto retained = std::make_shared<std::shared_ptr<rclcpp_action::ServerGoalHandle<Pick>>>();
  auto fake = rclcpp_action::create_server<Pick>(adversary, "/mtc/manipulation/pick_place",
    [](const auto &, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
    [&cancel_ack](auto) {cancel_ack = true; return rclcpp_action::CancelResponse::ACCEPT;},
    [retained, &accepted](auto h) {*retained = h; accepted = true;});
  // Retain the accepted Goal: discarding its handle would abort it automatically.
  // This scenario deliberately sends only a Cancel ACK, never stop evidence.
  auto config = mtc::task::FsmsConfig::defaults();
  config.state_timeouts[mtc::task::TaskStateName::CANCEL_PENDING] = 0.05;
  parent = std::make_shared<mtc::ros2_action::TaskServer>(config);
  client = rclcpp_action::create_client<Task>(client_node, "/mtc/task/execute");
  start({parent, adversary}); ASSERT_TRUE(client->wait_for_action_server(3s));
  auto h = send(goal()); ASSERT_NE(h, nullptr);
  ASSERT_TRUE(eventually([this]() {
    std::lock_guard<std::mutex> lock(mutex);
    return std::any_of(feedback.begin(), feedback.end(), [](auto & f) {return f.state == "EXECUTE_ITEM";});
  }));
  ASSERT_TRUE(eventually([&accepted]() {return accepted.load();}));
  auto cancel = client->async_cancel_goal(h); ASSERT_TRUE(ready(cancel));
  EXPECT_FALSE(cancel.get()->goals_canceling.empty());
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_TRUE(cancel_ack.load());
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(r.result->error_code, ErrorCodes::CANCEL_NOT_CONFIRMED);
  EXPECT_EQ(r.result->completed_count, 0);
  auto cleanup = std::make_shared<Pick::Result>();
  cleanup->error_code = ErrorCodes::CANCEL_NOT_CONFIRMED;
  (*retained)->abort(cleanup);  // Test teardown does not manufacture CANCELED.
}
TEST_F(TaskAction, StaleParentCancelCannotReachNewChild)
{
  setup();
  auto a = send(goal()); ASSERT_NE(a, nullptr);
  auto af = client->async_get_result(a); ASSERT_TRUE(ready(af)); ASSERT_TRUE(af.get().result->success);
  auto b = send(goal("new_parent")); ASSERT_NE(b, nullptr);
  auto service = client_node->create_client<action_msgs::srv::CancelGoal>(
    "/mtc/task/execute/_action/cancel_goal");
  ASSERT_TRUE(service->wait_for_service(3s));
  auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
  request->goal_info.goal_id.uuid = a->get_goal_id();
  auto cancel = service->async_send_request(request); ASSERT_TRUE(ready(cancel));
  EXPECT_TRUE(cancel.get()->goals_canceling.empty());
  auto bf = client->async_get_result(b); ASSERT_TRUE(ready(bf)); EXPECT_TRUE(bf.get().result->success);
  EXPECT_EQ(child->mock_call_counts()->stops.load(), 0u);
}
TEST_F(TaskAction, ChildSuccessWithoutPlacementEvidenceIsNotRecorded)
{
  using Pick = mtc_interfaces::action::PickPlace;
  auto adversary = std::make_shared<rclcpp::Node>("mock_child_invalid_evidence");
  auto fake = rclcpp_action::create_server<Pick>(adversary, "/mtc/manipulation/pick_place",
    [](const auto &, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
    [](auto) {return rclcpp_action::CancelResponse::REJECT;},
    [](auto handle) {
      auto r = std::make_shared<Pick::Result>();
      r->success = true; r->placement_verified = false;
      handle->succeed(r);  // Deliberately invalid upstream evidence, never a positive Mock.
    });
  parent = std::make_shared<mtc::ros2_action::TaskServer>();
  client = rclcpp_action::create_client<Task>(client_node, "/mtc/task/execute");
  start({parent, adversary}); ASSERT_TRUE(client->wait_for_action_server(3s));
  auto h = send(goal()); ASSERT_NE(h, nullptr);
  auto f = client->async_get_result(h); ASSERT_TRUE(ready(f));
  auto r = f.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(r.result->error_code, ErrorCodes::PLACEMENT_FAILED);
  EXPECT_EQ(r.result->completed_count, 0);
}
