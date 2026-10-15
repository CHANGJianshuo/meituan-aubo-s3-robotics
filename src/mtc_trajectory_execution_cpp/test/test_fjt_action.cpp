// Real ROS Action transport, entirely local software. No robot SDK or driver.
#include <gtest/gtest.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <action_msgs/msg/goal_status_array.hpp>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include "mtc_trajectory_execution_cpp/fjt_motion_port.hpp"
using namespace std::chrono_literals;
using namespace mtc::trajectory;
using Fjt=control_msgs::action::FollowJointTrajectory;
using ServerHandle=rclcpp_action::ServerGoalHandle<Fjt>;
using Hw=controller_manager_msgs::srv::ListHardwareComponents;
using Cs=controller_manager_msgs::srv::ListControllers;
class FakeController : public rclcpp::Node
{
public:
  Config cfg;
  std::atomic<size_t> goals{0}, cancels{0}, last_state_count{0};
  std::atomic<bool> reject{false}, aborted{false}, ack_only{false}, no_result{false};
  std::atomic<bool> publish_state{true}, stale_state{false}, missing_velocity{false};
  std::atomic<bool> moving{false}, wrong_final{false}, feedback_enabled{true}, active_controller{true};
  std::atomic<bool> generic{true}, repeated_stamp{false};
  std::atomic<bool> publish_status{true}, hold_response{false}, pending_status{false};
  std::atomic<bool> hide_active_status{false}, invalid_status{false}, reject_cancel{false}, hold_terminal_status{false};
  std::atomic<size_t> status_publications{0}, pending_requests{0};
  void release_response() {pending_status=false; {std::lock_guard<std::mutex> lock(gate_mutex); released=true;} gate.notify_all();}
  void release_terminal_status() {std::lock_guard<std::mutex> lock(mutex); hold_terminal_status=false; status_cache=latest_status;}
  size_t active_statuses() {std::lock_guard<std::mutex> lock(mutex); return std::count_if(status_cache.status_list.begin(),status_cache.status_list.end(),[](const auto & s) {return s.status>=1 && s.status<=3;});}
  void publish_latched_empty() {status_pub->publish(action_msgs::msg::GoalStatusArray{});}
  size_t terminal_statuses() {std::lock_guard<std::mutex> lock(mutex); return std::count_if(status_cache.status_list.begin(),status_cache.status_list.end(),[](const auto & s) {return s.status>=4;});}
  rclcpp_action::GoalUUID request_id() {std::lock_guard<std::mutex> lock(mutex); return pending_uuid;}
  FakeController(Config c):Node("fake_controller"),cfg(std::move(c))
  {
    hw=create_service<Hw>(cfg.controller_manager+"/list_hardware_components",
      [this](std::shared_ptr<Hw::Request>,std::shared_ptr<Hw::Response> r) {
        controller_manager_msgs::msg::HardwareComponentState h; h.name=cfg.hardware_component;
        h.plugin_name=generic ? "mock_components/GenericSystem" : "aubo_driver/AuboHardwareInterface";
        h.state.id=lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        for (auto & n:cfg.joints) {for (auto suffix:{"/position","/velocity"}) {
          controller_manager_msgs::msg::HardwareInterface i; i.name=n+suffix; i.is_available=true; h.state_interfaces.push_back(i);
        }} r->component.push_back(h);
      });
    cs=create_service<Cs>(cfg.controller_manager+"/list_controllers",
      [this](std::shared_ptr<Cs::Request>,std::shared_ptr<Cs::Response> r) {
        controller_manager_msgs::msg::ControllerState c; c.name=cfg.controller;
        c.type="joint_trajectory_controller/JointTrajectoryController"; c.state=active_controller ? "active" : "inactive";
        for (auto & j:cfg.joints) {c.claimed_interfaces.push_back(j+"/position");} r->controller.push_back(c);
        c=controller_manager_msgs::msg::ControllerState(); c.name="joint_state_broadcaster"; c.state="active"; c.type="joint_state_broadcaster/JointStateBroadcaster"; r->controller.push_back(c);
      });
    pub=create_publisher<sensor_msgs::msg::JointState>(cfg.joint_state_topic,rclcpp::SensorDataQoS());
    status_pub=create_publisher<action_msgs::msg::GoalStatusArray>(cfg.action_name()+"/test_status_evidence",
      rclcpp::QoS(1).reliable().transient_local());
    native_status=create_subscription<action_msgs::msg::GoalStatusArray>(cfg.action_name()+"/_action/status",
      rclcpp::QoS(1).reliable().transient_local(),[this](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex); latest_status=*msg;
        if(!hold_terminal_status || std::none_of(msg->status_list.begin(),msg->status_list.end(),
          [](const auto & s) {return s.status>=4;})) {status_cache=*msg;}
      });
    action_group=create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    server=rclcpp_action::create_server<Fjt>(this,cfg.action_name(),
      [this](const auto & uuid,auto goal) {
        ++goals;
        if(hold_response && goal->trajectory.header.frame_id!="foreign") {
          {std::lock_guard<std::mutex> lock(mutex); pending_uuid=uuid;}
          ++pending_requests;
          std::unique_lock<std::mutex> lock(gate_mutex);
          gate.wait_for(lock,2s,[this]() {return released;});
        }
        return reject ? rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](auto h) {++cancels; std::lock_guard<std::mutex> lock(mutex); canceled_uuid=h->get_goal_id(); return reject_cancel ? rclcpp_action::CancelResponse::REJECT : rclcpp_action::CancelResponse::ACCEPT;},
      [this](auto h) {std::lock_guard<std::mutex> lock(mutex); handle=h; received=h->get_goal()->trajectory; began=std::chrono::steady_clock::now();},rcl_action_server_get_default_options(),action_group);
    timer=create_wall_timer(10ms,[this]() {tick();});
  }
  trajectory_msgs::msg::JointTrajectory trajectory() {std::lock_guard<std::mutex> lock(mutex); return received;}
  rclcpp_action::GoalUUID goal_id() {std::lock_guard<std::mutex> lock(mutex); return handle ? handle->get_goal_id() : rclcpp_action::GoalUUID{};}
  rclcpp_action::GoalUUID cancel_id() {std::lock_guard<std::mutex> lock(mutex); return canceled_uuid;}
private:
  std::mutex gate_mutex;
  std::condition_variable gate;
  bool released=false;
  rclcpp_action::GoalUUID pending_uuid{};
  rclcpp::CallbackGroup::SharedPtr action_group;
  rclcpp::Publisher<action_msgs::msg::GoalStatusArray>::SharedPtr status_pub;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr native_status;
  action_msgs::msg::GoalStatusArray status_cache,latest_status;
  std::mutex mutex;
  std::shared_ptr<ServerHandle> handle;
  trajectory_msgs::msg::JointTrajectory received;
  rclcpp_action::GoalUUID canceled_uuid{};
  std::vector<double> position=std::vector<double>(6,0.0);
  std::chrono::steady_clock::time_point began;
  rclcpp::Time last_stamp{0,0,RCL_ROS_TIME};
  rclcpp::Service<Hw>::SharedPtr hw;
  rclcpp::Service<Cs>::SharedPtr cs;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
  rclcpp_action::Server<Fjt>::SharedPtr server;
  rclcpp::TimerBase::SharedPtr timer;
  void tick()
  {
    std::lock_guard<std::mutex> lock(mutex);
    auto stamp=now(); if (stale_state) {stamp=stamp-rclcpp::Duration::from_seconds(2);}
    if (repeated_stamp && last_stamp.nanoseconds()) {stamp=last_stamp;} else {last_stamp=stamp;}
    if (handle) {
      auto f=std::make_shared<Fjt::Feedback>(); f->header.stamp=stamp; f->joint_names=cfg.joints;
      f->actual.positions=position; f->actual.velocities.assign(6,moving ? 0.1 : 0.0);
      if (feedback_enabled) {handle->publish_feedback(f);}
      auto r=std::make_shared<Fjt::Result>();
      if (handle->is_canceling() && !ack_only) {r->error_code=Fjt::Result::SUCCESSFUL; handle->canceled(r); handle.reset();}
      else if (!no_result && !ack_only && std::chrono::steady_clock::now()-began>150ms) {
        if (aborted) {r->error_code=Fjt::Result::PATH_TOLERANCE_VIOLATED; r->error_string="simulated controller abort"; handle->abort(r);}
        else {
          if (!wrong_final) {position=received.points.back().positions;}
          r->error_code=Fjt::Result::SUCCESSFUL; handle->succeed(r);
        } handle.reset();
      }
    }
    if(publish_status) {
      auto evidence=status_cache;
      if(pending_status && pending_requests) {
        action_msgs::msg::GoalStatus status; status.goal_info.goal_id.uuid=pending_uuid;
        status.goal_info.stamp=now(); status.status=action_msgs::msg::GoalStatus::STATUS_ACCEPTED;
        if(std::none_of(evidence.status_list.begin(),evidence.status_list.end(),[&](const auto & x) {
          return x.goal_info.goal_id.uuid==pending_uuid;
        })) {evidence.status_list.push_back(status);}
      }
      if(hide_active_status) {
        evidence.status_list.erase(std::remove_if(evidence.status_list.begin(),evidence.status_list.end(),
          [](const auto & s) {return s.status>=1 && s.status<=3;}),evidence.status_list.end());
      }
      if(invalid_status) {
        action_msgs::msg::GoalStatus bad; bad.status=action_msgs::msg::GoalStatus::STATUS_EXECUTING;
        bad.goal_info.stamp=now(); evidence.status_list.push_back(bad); // Invalid zero UUID.
      }
      status_pub->publish(evidence); ++status_publications;
    }
    if (publish_state) {
      sensor_msgs::msg::JointState s; s.header.stamp=stamp; s.name=cfg.joints; s.position=position;
      if (!missing_velocity) {s.velocity.assign(6,moving ? 0.1 : 0.0);} pub->publish(s); ++last_state_count;
    }
  }
};
class FjtAction : public ::testing::Test
{
protected:
  Config cfg;
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<FakeController> server;
  std::unique_ptr<FjtMotionPort> port;
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions(),4};
  std::thread spin;
  static void SetUpTestSuite()
  {
    setenv("RMW_IMPLEMENTATION","rmw_fastrtps_cpp",1);
    setenv("ROS_DOMAIN_ID",std::to_string(50+getpid()%150).c_str(),1);
    setenv("ROS_AUTOMATIC_DISCOVERY_RANGE","LOCALHOST",1);
    setenv("ROS_LOG_DIR","/tmp/mtc_phase2b_tests",1);
    for (auto key:{"ROS_STATIC_PEERS","ROS_LOCALHOST_ONLY","FASTDDS_DEFAULT_PROFILES_FILE","FASTRTPS_DEFAULT_PROFILES_FILE","CYCLONEDDS_URI"}) {unsetenv(key);}
    rclcpp::init(0,nullptr);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    auto name=std::string(::testing::UnitTest::GetInstance()->current_test_info()->name());
    cfg.controller="jtc_"+name; cfg.controller_manager="/cm_"+name; cfg.joint_state_topic="/js_"+name;
    cfg.enable_fake_execution=true; cfg.settle_duration=name=="StationaryWindowCannotGrowWithoutNewSamples" ? 0.15 : 0.05; cfg.max_start_delay=0.05;
    for (const auto & n:cfg.joints) {cfg.limits[n]={-2,2,1,100};}
    // Test-only server relay provides explicit live Status evidence. Native Action
    // Goal/Feedback/Result/Cancel are unchanged; no Port private state is accessed.
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args","-r",cfg.action_name()+"/_action/status:="+cfg.action_name()+"/test_status_evidence"});
    node=std::make_shared<rclcpp::Node>("fjt_client",options); server=std::make_shared<FakeController>(cfg);
    if(name=="NoStatusBeforeDispatchFailsClosed" || name=="LatchedEmptyStatusCannotAuthorizeDispatch") {
      server->publish_status=false;
      if(name=="LatchedEmptyStatusCannotAuthorizeDispatch") {server->publish_latched_empty();}
    }
    port=std::make_unique<FjtMotionPort>(node,cfg); executor.add_node(node); executor.add_node(server);
    spin=std::thread([this]() {executor.spin();});
    if(name=="NoStatusBeforeDispatchFailsClosed" || name=="LatchedEmptyStatusCannotAuthorizeDispatch") {
      ASSERT_TRUE(wait([this]() {return server->last_state_count>=25;})); return;
    }
    ASSERT_TRUE(wait([this]() {return port->snapshot().ready;})) << port->snapshot().detail;
  }
  void TearDown() override
  {
    server->release_response(); port.reset(); executor.cancel(); if (spin.joinable()) {spin.join();}
    executor.remove_node(node); executor.remove_node(server); server.reset(); node.reset();
  }
  static bool wait(const std::function<bool()> & f,std::chrono::milliseconds limit=3s)
  {
    auto end=std::chrono::steady_clock::now()+limit;
    while (std::chrono::steady_clock::now()<end) {if (f()) {return true;} std::this_thread::sleep_for(5ms);} return f();
  }
  mtc::core::MotionCommand command(std::string id="cmd")
  {
    mtc::core::MotionCommand c; c.command_id=id; c.simulation_only=true; c.timeout_ms=1500;
    mtc::core::TimedTrajectory t; t.trajectory_id="plan"; t.joint_names=cfg.joints; t.header_frame_id="base_link";
    for (int i=0;i<3;++i) {mtc::core::TrajectoryPoint p; p.time_from_start_s=0.06*i;
      p.positions_rad.assign(6,0.01*i); p.velocities_rad_s.assign(6,0); p.accelerations_rad_s2.assign(6,0); t.points.push_back(p);}
    c.trajectory=t; return c;
  }
  void send(const mtc::core::MotionCommand & c)
  {
    auto r=port->execute_trajectory(c); ASSERT_FALSE(r) << (r ? r->message : "");
    ASSERT_TRUE(wait([this]() {return !port->snapshot().goal_uuid.empty();}));
  }
  mtc::core::MotionResult result(const mtc::core::MotionCommand & c)
  {
    std::optional<mtc::core::MotionResult> r;
    EXPECT_TRUE(wait([&]() {r=port->execute_trajectory(c); return bool(r);},3s)); return r.value_or(mtc::core::MotionResult{});
  }
};
TEST_F(FjtAction, CompleteTrajectoryOneGoalAndFeedback) {
  auto c=command(); send(c); auto r=result(c); EXPECT_TRUE(r.ok()) << r.message; EXPECT_TRUE(r.stop_confirmed);
  auto t=server->trajectory(); EXPECT_EQ(t.points.size(),3u); EXPECT_EQ(t,c.trajectory ? *convert(c,cfg).trajectory : t);
  EXPECT_EQ(server->goals,1u); EXPECT_GT(port->snapshot().feedback_samples,0u);
}
TEST_F(FjtAction, RepeatedTicksNeverResend) {
  auto c=command(); send(c); for (int i=0;i<100;++i) {port->execute_trajectory(c);} EXPECT_TRUE(result(c).success);
  for (int i=0;i<20;++i) {EXPECT_TRUE(port->execute_trajectory(c)->success);} EXPECT_EQ(server->goals,1u);
}
TEST_F(FjtAction, ConcurrentGoalsSingleOwner) {
  server->no_result=true; auto a=command("a"),b=command("b"); std::optional<mtc::core::MotionResult> ra,rb;
  std::thread t1([&](){ra=port->execute_trajectory(a);}),t2([&](){rb=port->execute_trajectory(b);}); t1.join(); t2.join();
  EXPECT_NE(bool(ra),bool(rb)); EXPECT_TRUE(wait([&](){return server->goals==1;}));
  port->request_stop("cleanup"); EXPECT_TRUE(wait([&](){return port->snapshot().result.has_value();})); EXPECT_EQ(server->goals,1u);
}
TEST_F(FjtAction, StartMismatchDoesNotSend) {
  auto c=command(); for(auto & p:c.trajectory->points) {p.positions_rad[0]+=0.1;}
  auto r=port->execute_trajectory(c); ASSERT_TRUE(r); EXPECT_EQ(r->exec_state,mtc::core::ExecState::REJECTED); EXPECT_EQ(server->goals,0u);
  EXPECT_NE(r->message.find("trajectory start differs"),std::string::npos);
}
TEST_F(FjtAction, CorrectCancelUsesExactUuid) {
  server->no_result=true; auto c=command(); send(c); auto uuid=server->goal_id();
  auto stop=port->cancel_command(c.command_id,"test"); EXPECT_FALSE(stop.stop_confirmed);
  auto r=result(c); EXPECT_FALSE(r.success); EXPECT_TRUE(r.stop_confirmed) << r.message;
  EXPECT_EQ(server->cancel_id(),uuid); EXPECT_EQ(server->cancels,1u);
}
TEST_F(FjtAction, CancelBeforeGoalResponsePreserved) {
  server->no_result=true; auto c=command(); ASSERT_FALSE(port->execute_trajectory(c));
  EXPECT_FALSE(port->cancel_command(c.command_id,"immediate").stop_confirmed);
  auto r=result(c); EXPECT_TRUE(r.stop_confirmed) << r.message; EXPECT_EQ(server->cancels,1u);
}
TEST_F(FjtAction, DuplicateCancelOnlyOneBackendCall) {
  server->no_result=true; auto c=command(); send(c);
  for (int i=0;i<50;++i) {port->cancel_command(c.command_id,"repeat");}
  EXPECT_TRUE(result(c).stop_confirmed); EXPECT_EQ(server->cancels,1u);
}
TEST_F(FjtAction, StaleCancelDoesNotAffectNewOwner) {
  auto a=command("old"); send(a); ASSERT_TRUE(result(a).success);
  auto b=command("new"); for(auto & p:b.trajectory->points) {for(auto & x:p.positions_rad) {x+=0.02;}}
  server->no_result=true; send(b); auto before=port->snapshot();
  EXPECT_FALSE(port->cancel_command(a.command_id,"late").stop_confirmed);
  EXPECT_EQ(server->cancels,0u); EXPECT_EQ(port->snapshot().command_id,b.command_id);
  EXPECT_EQ(port->snapshot().goal_uuid,before.goal_uuid); EXPECT_FALSE(port->snapshot().fault_locked);
  port->cancel_command(b.command_id,"cleanup"); EXPECT_TRUE(result(b).stop_confirmed);
}
TEST_F(FjtAction, CancelWithoutOwnerNeverGlobalStops) {
  auto r=port->request_stop("no owner"); EXPECT_FALSE(r.stop_confirmed); EXPECT_FALSE(r.request_delivered); EXPECT_EQ(server->cancels,0u);
}
TEST_F(FjtAction, AckOnlyCannotConfirmStopAndTimeoutLocks) {
  server->ack_only=true; auto c=command(); c.timeout_ms=400; send(c);
  port->cancel_command(c.command_id,"test"); ASSERT_TRUE(wait([&](){return port->snapshot().cancel_acknowledged;}));
  EXPECT_FALSE(port->request_stop("poll").stop_confirmed); auto r=result(c);
  EXPECT_TRUE(r.timed_out); EXPECT_TRUE(r.status_unknown); EXPECT_FALSE(r.stop_confirmed);
  auto next=port->execute_trajectory(command("next")); ASSERT_TRUE(next); EXPECT_TRUE(next->status_unknown); EXPECT_EQ(server->goals,1u);
}
TEST_F(FjtAction, ExecutionTimeoutLocksAndNeverResends) {
  server->no_result=true; auto c=command(); c.timeout_ms=400; send(c); auto r=result(c);
  EXPECT_TRUE(r.timed_out); EXPECT_TRUE(port->snapshot().fault_locked);
  for(int i=0;i<10;++i) {port->execute_trajectory(c); port->execute_trajectory(command("new"));}
  EXPECT_EQ(server->goals,1u); EXPECT_LE(server->cancels,1u);
}
TEST_F(FjtAction, ControllerAbortedRemainsFaultLocked) {
  server->aborted=true; auto c=command(); send(c); auto r=result(c); EXPECT_FALSE(r.success);
  EXPECT_TRUE(port->snapshot().fault_locked); EXPECT_NE(r.error_code,0);
  port->execute_trajectory(command("new")); EXPECT_EQ(server->goals,1u);
}
TEST_F(FjtAction, StaleJointFeedbackLocks) {
  server->no_result=true; auto c=command(); send(c); server->stale_state=true;
  auto r=result(c); EXPECT_TRUE(r.status_unknown); EXPECT_FALSE(r.stop_confirmed); EXPECT_TRUE(port->snapshot().fault_locked);
}
TEST_F(FjtAction, LostJointUpdatesCannotSettle) {
  server->no_result=true; auto c=command(); send(c); server->publish_state=false;
  auto r=result(c); EXPECT_TRUE(r.status_unknown); EXPECT_FALSE(r.stop_confirmed);
}
TEST_F(FjtAction, InsufficientStationaryIntervalDoesNotSucceed) {
  auto c=command(); send(c); server->moving=true; auto r=result(c);
  EXPECT_FALSE(r.success); EXPECT_FALSE(r.stop_confirmed); EXPECT_TRUE(port->snapshot().fault_locked);
}
TEST_F(FjtAction, WrongFinalPositionCannotSucceed) {
  server->wrong_final=true; auto c=command(); c.timeout_ms=500; send(c); auto r=result(c);
  EXPECT_FALSE(r.arrived); EXPECT_FALSE(r.success); EXPECT_FALSE(r.stop_confirmed); EXPECT_TRUE(r.timed_out);
}
TEST_F(FjtAction, NoAssociatedFeedbackCannotSucceed) {
  server->feedback_enabled=false; auto c=command(); send(c); auto r=result(c);
  EXPECT_FALSE(r.success); EXPECT_TRUE(port->snapshot().fault_locked);
}
TEST_F(FjtAction, HardwareProofLostLocksAndCancelsOwnedGoal) {
  server->no_result=true; auto c=command(); send(c); server->generic=false;
  EXPECT_TRUE(result(c).status_unknown); EXPECT_TRUE(wait([&](){return server->cancels==1;}));
}
TEST_F(FjtAction, WrongPluginRejectsBeforeSend) {
  server->generic=false; ASSERT_TRUE(wait([&](){return !port->snapshot().ready;}));
  auto r=port->execute_trajectory(command()); ASSERT_TRUE(r); EXPECT_EQ(server->goals,0u);
}
TEST_F(FjtAction, InactiveControllerRejectsBeforeSend) {
  server->active_controller=false; ASSERT_TRUE(wait([&](){return !port->snapshot().ready;}));
  EXPECT_TRUE(port->execute_trajectory(command())); EXPECT_EQ(server->goals,0u);
}
TEST_F(FjtAction, MissingVelocityEvidenceRejectsBeforeSend) {
  server->missing_velocity=true; ASSERT_TRUE(wait([&](){return !port->snapshot().ready;}));
  EXPECT_TRUE(port->execute_trajectory(command())); EXPECT_EQ(server->goals,0u);
}
TEST_F(FjtAction, ReplayedJointStampLocks) {
  server->no_result=true; auto c=command(); send(c); server->repeated_stamp=true;
  EXPECT_TRUE(result(c).status_unknown); EXPECT_FALSE(port->snapshot().result->stop_confirmed);
}
TEST_F(FjtAction, GoalRejectionMeansNeverExecuted) {
  server->reject=true; auto c=command(); ASSERT_FALSE(port->execute_trajectory(c)); auto r=result(c);
  EXPECT_EQ(r.exec_state,mtc::core::ExecState::REJECTED); EXPECT_TRUE(r.retry_allowed_without_reobservation());
  EXPECT_FALSE(port->snapshot().fault_locked); EXPECT_EQ(server->goals,1u);
}
TEST_F(FjtAction, ChangedCommandIdContentCannotResend) {
  server->no_result=true; auto c=command(); send(c); auto changed=c; changed.trajectory->points.back().positions_rad[0]=0.03;
  auto r=port->execute_trajectory(changed); ASSERT_TRUE(r); EXPECT_EQ(r->exec_state,mtc::core::ExecState::REJECTED); EXPECT_EQ(server->goals,1u);
  port->request_stop("cleanup"); EXPECT_TRUE(result(c).stop_confirmed);
}
TEST_F(FjtAction, SecondPortCannotOwnEndpoint) {
  EXPECT_THROW(FjtMotionPort(node,cfg),std::invalid_argument); EXPECT_EQ(server->goals,0u);
}
TEST_F(FjtAction, DefaultDisabledNeverSends) {
  port.reset(); cfg.enable_fake_execution=false; port=std::make_unique<FjtMotionPort>(node,cfg);
  EXPECT_TRUE(port->execute_trajectory(command())); EXPECT_EQ(server->goals,0u);
}
TEST_F(FjtAction, DestroyDuringExecutionOnlyCancelsOwnedGoal) {
  server->no_result=true; auto c=command(); send(c); auto uuid=server->goal_id(); port.reset();
  ASSERT_TRUE(wait([&](){return server->cancels==1;})); EXPECT_EQ(server->cancel_id(),uuid);
  EXPECT_THROW(FjtMotionPort(node,cfg),std::invalid_argument); // destruction is not recovery
}

TEST_F(FjtAction, HistoricalSettlementRequiresFreshCurrentEvidence) {
  auto c=command(); send(c); ASSERT_TRUE(result(c).stop_confirmed);
  EXPECT_TRUE(port->request_stop("current").stop_confirmed);
  server->publish_state=false; ASSERT_TRUE(wait([&](){return !port->snapshot().ready;}));
  EXPECT_FALSE(port->request_stop("expired").stop_confirmed); EXPECT_EQ(server->cancels,0u);
}
TEST_F(FjtAction, ConcurrentGoalReplacementAndLateCancel) {
  auto a=command("old"); send(a); ASSERT_TRUE(result(a).success);
  auto b=command("new"); for(auto & p:b.trajectory->points) {for(auto & x:p.positions_rad) {x+=0.02;}}
  server->no_result=true;
  std::thread old([&](){for(int i=0;i<100;++i) {port->cancel_command(a.command_id,"late");}});
  auto r=port->execute_trajectory(b); old.join(); ASSERT_FALSE(r);
  ASSERT_TRUE(wait([&](){return port->snapshot().command_id==b.command_id && !port->snapshot().goal_uuid.empty();}));
  EXPECT_EQ(server->cancels,0u); EXPECT_FALSE(port->snapshot().fault_locked);
  port->cancel_command(b.command_id,"cleanup"); EXPECT_TRUE(result(b).stop_confirmed);
}
TEST_F(FjtAction, OtherTrajectoryPublisherRejectsDispatch) {
  auto other=node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/"+cfg.controller+"/joint_trajectory",10);
  ASSERT_TRUE(wait([&](){return !port->snapshot().ready;}));
  EXPECT_TRUE(port->execute_trajectory(command())); EXPECT_EQ(server->goals,0u);
}
TEST_F(FjtAction, ForeignActionClientRejectsDispatch) {
  auto other=rclcpp_action::create_client<Fjt>(server,cfg.action_name());
  ASSERT_TRUE(wait([&](){return !port->snapshot().ready;}));
  EXPECT_TRUE(port->execute_trajectory(command())); EXPECT_EQ(server->goals,0u);
}
TEST_F(FjtAction, SingleTargetEntryExplicitlyRejected) {
  auto r=port->execute_joint_move(command()); ASSERT_TRUE(r); EXPECT_EQ(r->exec_state,mtc::core::ExecState::REJECTED); EXPECT_EQ(server->goals,0u);
}

TEST_F(FjtAction, StationaryWindowCannotGrowWithoutNewSamples) {
  auto c=command(); send(c);
  ASSERT_TRUE(wait([&](){return port->snapshot().stationary_samples>=2;}));
  server->publish_state=false; auto r=result(c);
  EXPECT_FALSE(r.success); EXPECT_FALSE(r.stop_confirmed); EXPECT_TRUE(r.status_unknown);
  EXPECT_LT(port->snapshot().stationary_observed_seconds,cfg.settle_duration);
}

TEST_F(FjtAction, ForeignInFlightGoalCannotBePreemptedAfterClientLeaves) {
  server->no_result=true;
  auto outsider=rclcpp_action::create_client<Fjt>(server,cfg.action_name());
  ASSERT_TRUE(wait([&](){return outsider->action_server_is_ready();}));
  Fjt::Goal goal; goal.trajectory=*convert(command("foreign"),cfg).trajectory;
  auto response=outsider->async_send_goal(goal);
  ASSERT_TRUE(wait([&](){return response.wait_for(0s)==std::future_status::ready;}));
  ASSERT_TRUE(response.get()); outsider.reset();
  ASSERT_TRUE(wait([&](){return port->snapshot().foreign_active_goals==1;}));
  auto r=port->execute_trajectory(command("ours")); ASSERT_TRUE(r);
  EXPECT_EQ(server->goals,1u); EXPECT_EQ(port->snapshot().goals_sent,0u); EXPECT_EQ(server->cancels,0u);
}

// Reuse unchanged test-only perception/tool fixtures; no public runtime dependency on them.
#include "../../mtc_manipulation_cpp/test/fakes.hpp"
#include <mtc_manipulation_cpp/pick_place_fsm.hpp>
class ExplicitTrajectoryPlanner : public mtc::manipulation::PlannerPort
{
public:
  mtc::core::MotionCommand prototype;
  explicit ExplicitTrajectoryPlanner(mtc::core::MotionCommand c):prototype(std::move(c)) {}
  std::optional<mtc::core::PlanResult> plan(const mtc::manipulation::PlanTarget & target) override
  {
    mtc::core::PlanResult result; result.ok=true;
    for (const auto & primitive:target.primitives) {
      auto c=prototype; c.primitive_name=primitive.name; c.intent=primitive;
      result.commands.push_back(c);
    } return result;
  }
};
TEST_F(FjtAction, ExistingPickPlaceFsmDelegatesWholeTrajectoryThroughPort) {
  auto clock=std::make_shared<mtc::core::ManualClock>();
  auto perception=std::make_shared<mtc::manipulation::testing::FakePerception>(); perception->clock=clock;
  auto planner=std::make_shared<ExplicitTrajectoryPlanner>(command());
  auto tool=std::make_shared<mtc::manipulation::testing::FakeTool>();
  auto injected=std::shared_ptr<mtc::manipulation::MotionPort>(port.get(),[](auto *) {});
  mtc::manipulation::PickPlaceConfig config; config.state_timeouts=mtc::manipulation::default_state_timeouts();
  mtc::manipulation::PickPlaceFsm fsm(perception,planner,injected,tool,clock,
    mtc::manipulation::make_strategy(config.tool_type),config);
  fsm.start("battery_1",1,"P1","fjt-injection",3.0);
  ASSERT_TRUE(wait([&](){fsm.step(); clock->advance(0.001); return fsm.state()==mtc::manipulation::PickPlaceState::ENGAGE;}));
  EXPECT_EQ(server->goals,1u); EXPECT_EQ(server->trajectory().points.size(),3u);
  EXPECT_TRUE(port->snapshot().result->stop_confirmed);
  EXPECT_FALSE(fsm.result().placement_verified); // First motion only, no fake full pick/place success.
}
TEST_F(FjtAction, DirectFsmCancelKeepsOriginalFaultWhenStopIsStillPending) {
  server->no_result=true;
  auto clock=std::make_shared<mtc::core::ManualClock>();
  auto perception=std::make_shared<mtc::manipulation::testing::FakePerception>(); perception->clock=clock;
  auto planner=std::make_shared<ExplicitTrajectoryPlanner>(command());
  auto tool=std::make_shared<mtc::manipulation::testing::FakeTool>();
  auto injected=std::shared_ptr<mtc::manipulation::MotionPort>(port.get(),[](auto *) {});
  mtc::manipulation::PickPlaceConfig config; config.state_timeouts=mtc::manipulation::default_state_timeouts();
  mtc::manipulation::PickPlaceFsm fsm(perception,planner,injected,tool,clock,
    mtc::manipulation::make_strategy(config.tool_type),config);
  fsm.start("battery_1",1,"P1","fjt-cancel-injection",3.0);
  ASSERT_TRUE(wait([&](){fsm.step(); clock->advance(0.001); return port->snapshot().goals_sent==1;}));
  fsm.request_cancel("test"); fsm.step();
  EXPECT_EQ(fsm.state(),mtc::manipulation::PickPlaceState::FAULT);
  EXPECT_EQ(fsm.result().error_code,mtc::core::ErrorCodes::CANCEL_NOT_CONFIRMED);
  ASSERT_TRUE(wait([&](){return port->snapshot().result && port->snapshot().result->stop_confirmed;}));
  fsm.step(); EXPECT_EQ(fsm.state(),mtc::manipulation::PickPlaceState::FAULT); EXPECT_EQ(server->cancels,1u);
}

TEST_F(FjtAction, CompletedGoalSameIdSameContentReturnsCachedResult) {
  auto c=command(); send(c); auto original=result(c); ASSERT_TRUE(original.success);
  for(int i=0;i<10;++i) {auto r=port->execute_trajectory(c); ASSERT_TRUE(r); EXPECT_TRUE(r->success); EXPECT_EQ(r->message,original.message);}
  EXPECT_EQ(server->goals,1u); EXPECT_EQ(server->cancels,0u);
}
TEST_F(FjtAction, CompletedGoalSameIdDifferentTrajectoryRejected) {
  auto c=command(); send(c); ASSERT_TRUE(result(c).success);
  auto changed=c; changed.trajectory->points.back().positions_rad[0]+=0.001;
  auto r=port->execute_trajectory(changed); ASSERT_TRUE(r); EXPECT_EQ(r->exec_state,mtc::core::ExecState::REJECTED);
  EXPECT_EQ(server->goals,1u); EXPECT_EQ(server->cancels,0u);
}
TEST_F(FjtAction, CompletedGoalSameIdDifferentTimeoutRejected) {
  auto c=command(); send(c); ASSERT_TRUE(result(c).success); ++c.timeout_ms;
  auto r=port->execute_trajectory(c); ASSERT_TRUE(r); EXPECT_EQ(r->exec_state,mtc::core::ExecState::REJECTED);
  EXPECT_EQ(server->goals,1u); EXPECT_EQ(server->cancels,0u);
}
TEST_F(FjtAction, CompletedGoalSameIdDifferentSafetySemanticsRejected) {
  auto c=command(); send(c); ASSERT_TRUE(result(c).success);
  for(int field=0;field<4;++field) {
    auto changed=c;
    if(field==0) {changed.simulation_only=false;}
    if(field==1) {changed.requires_contact=true;}
    if(field==2) {changed.velocity_scaling=0.1;}
    if(field==3) {changed.acceleration_scaling=0.1;}
    auto r=port->execute_trajectory(changed); ASSERT_TRUE(r); EXPECT_EQ(r->exec_state,mtc::core::ExecState::REJECTED);
  }
  EXPECT_EQ(server->goals,1u); EXPECT_EQ(server->cancels,0u);
}
TEST_F(FjtAction, CompletedGoalIdentityMismatchNeverSendsAgain) {
  auto c=command("completed"); send(c); ASSERT_TRUE(result(c).success);
  server->aborted=true; auto next=command("faulted");
  for(auto & p:next.trajectory->points) {for(auto & x:p.positions_rad) {x+=0.02;}}
  send(next); ASSERT_FALSE(result(next).success); ASSERT_TRUE(port->snapshot().fault_locked);
  auto changed=c; changed.trajectory->trajectory_id="another-plan";
  auto r=port->execute_trajectory(changed); ASSERT_TRUE(r); EXPECT_EQ(r->exec_state,mtc::core::ExecState::REJECTED);
  EXPECT_TRUE(port->snapshot().fault_locked); EXPECT_EQ(server->goals,2u); EXPECT_EQ(server->cancels,0u);
}
TEST_F(FjtAction, InFlightIdentityIncludesOriginalOrderAndIntentPose) {
  server->no_result=true; auto c=command();
  mtc::core::MotionPrimitive intent; intent.name="pre_align";
  mtc::core::Pose6D pose; pose.valid=true; pose.frame_id="base_link"; intent.target_pose=pose; c.intent=intent;
  send(c);
  for(int field=0;field<4;++field) {
    auto changed=c;
    if(field==0) {std::reverse(changed.trajectory->joint_names.begin(),changed.trajectory->joint_names.end());}
    if(field==1) {changed.intent->target_pose->position.x=0.01;}
    if(field==2) {changed.trajectory->trajectory_id="changed";}
    if(field==3) {changed.velocity_scaling=0.1;}
    auto r=port->execute_trajectory(changed); ASSERT_TRUE(r); EXPECT_EQ(r->exec_state,mtc::core::ExecState::REJECTED);
  }
  EXPECT_EQ(server->goals,1u); EXPECT_EQ(server->cancels,0u);
  port->cancel_command(c.command_id,"cleanup"); EXPECT_TRUE(result(c).stop_confirmed);
}

TEST_F(FjtAction, CompletedIdentityPreservesEveryOriginalTrajectoryField) {
  auto c=command(); mtc::core::MotionPrimitive intent; intent.name="pre_align";
  mtc::core::Pose6D pose; pose.valid=true; pose.frame_id="base_link"; intent.target_pose=pose; c.intent=intent;
  send(c); ASSERT_TRUE(result(c).success);
  for(int field=0;field<14;++field) {
    auto changed=c; auto & t=*changed.trajectory;
    if(field==0) {std::reverse(t.joint_names.begin(),t.joint_names.end());}
    if(field==1) {t.points[1].positions_rad[0]+=0.001;}
    if(field==2) {t.points[1].velocities_rad_s[0]=0.001;}
    if(field==3) {t.points[1].accelerations_rad_s2[0]=0.001;}
    if(field==4) {t.points[1].time_from_start_s+=0.001;}
    if(field==5) {t.header_frame_id="other_frame";}
    if(field==6) {t.stamp_s=0.0000000001;} // Same ROS nanoseconds after rounding, different intent.
    if(field==7) {t.trajectory_id="other_plan";}
    if(field==8) {changed.primitive_name="other";}
    if(field==9) {changed.intent->target_pose->orientation.z=0.0001;}
    if(field==10) {changed.intent->target_pose->stamp_s=1;}
    if(field==11) {changed.intent->max_linear_speed=0.04;}
    if(field==12) {changed.intent->target_label="other";}
    if(field==13) {changed.joint_names=cfg.joints;}
    auto r=port->execute_trajectory(changed); ASSERT_TRUE(r); EXPECT_EQ(r->exec_state,mtc::core::ExecState::REJECTED) << field;
  }
  EXPECT_TRUE(port->execute_trajectory(c)->success); EXPECT_EQ(server->goals,1u); EXPECT_EQ(server->cancels,0u);
}
TEST_F(FjtAction, FaultedIdentityMismatchPreservesLockAndResult) {
  server->aborted=true; auto c=command(); send(c); auto original=result(c);
  ASSERT_TRUE(port->snapshot().fault_locked); auto before=port->snapshot(); auto changed=c; ++changed.timeout_ms;
  auto r=port->execute_trajectory(changed); ASSERT_TRUE(r); EXPECT_EQ(r->exec_state,mtc::core::ExecState::REJECTED);
  EXPECT_EQ(port->execute_trajectory(c)->message,original.message);
  EXPECT_TRUE(port->snapshot().fault_locked); EXPECT_EQ(port->snapshot().goal_uuid,before.goal_uuid);
  EXPECT_EQ(server->goals,1u); EXPECT_EQ(server->cancels,0u);
}

TEST_F(FjtAction, MonotonicPlannerStampCannotBecomeRosScheduledStart) {
  auto c=command(); c.trajectory->stamp_s=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  ASSERT_TRUE(convert(c,cfg).trajectory); // Structurally valid, incompatible clock contract.
  auto r=port->execute_trajectory(c); ASSERT_TRUE(r);
  EXPECT_NE(r->message.find("scheduled stamp stale"),std::string::npos);
  EXPECT_EQ(server->goals,0u); EXPECT_EQ(server->cancels,0u); EXPECT_FALSE(port->snapshot().fault_locked);
}
TEST_F(FjtAction, PastPlannerGenerationStampDoesNotDispatch) {
  auto c=command(); c.trajectory->stamp_s=node->now().seconds()-1;
  ASSERT_TRUE(convert(c,cfg).trajectory);
  auto r=port->execute_trajectory(c); ASSERT_TRUE(r);
  EXPECT_NE(r->message.find("scheduled stamp stale"),std::string::npos);
  EXPECT_EQ(server->goals,0u); EXPECT_EQ(server->cancels,0u); EXPECT_FALSE(port->snapshot().fault_locked);
}

TEST_F(FjtAction, NoStatusBeforeDispatchFailsClosed) {
  EXPECT_FALSE(port->snapshot().ready);
  auto r=port->execute_trajectory(command()); EXPECT_TRUE(r);
  EXPECT_EQ(port->snapshot().goals_sent,0u); EXPECT_EQ(server->goals,0u);
}
TEST_F(FjtAction, StatusBeforeGoalResponsePreservesOwnership) {
  server->hold_response=true; server->pending_status=true; server->no_result=true;
  auto c=command(); ASSERT_FALSE(port->execute_trajectory(c));
  ASSERT_TRUE(wait([&]() {return server->pending_requests==1 && server->status_publications>=30;}));
  EXPECT_TRUE(port->snapshot().goal_uuid.empty());
  EXPECT_EQ(port->snapshot().ownership,GoalOwnership::UNKNOWN);
  EXPECT_EQ(port->snapshot().unknown_active_goals,1u);
  EXPECT_EQ(port->snapshot().foreign_active_goals,1u); // Unassociated, never guessed Own.
  EXPECT_FALSE(port->snapshot().ready); EXPECT_EQ(server->cancels,0u);
  server->release_response(); ASSERT_TRUE(wait([&]() {return !port->snapshot().goal_uuid.empty();}));
  EXPECT_EQ(server->goal_id(),server->request_id());
  port->cancel_command(c.command_id,"cleanup"); EXPECT_TRUE(result(c).stop_confirmed);
}

TEST_F(FjtAction, ForeignGoalBeforeOwnGoalRejected) {
  server->no_result=true;
  auto outsider=rclcpp_action::create_client<Fjt>(server,cfg.action_name());
  ASSERT_TRUE(wait([&]() {return outsider->action_server_is_ready();}));
  Fjt::Goal goal; goal.trajectory=*convert(command("foreign"),cfg).trajectory; goal.trajectory.header.frame_id="foreign";
  auto response=outsider->async_send_goal(goal);
  ASSERT_TRUE(wait([&]() {return response.wait_for(0s)==std::future_status::ready;})); ASSERT_TRUE(response.get());
  outsider.reset(); ASSERT_TRUE(wait([&]() {return port->snapshot().ownership==GoalOwnership::FOREIGN_GOAL;}));
  auto r=port->execute_trajectory(command()); ASSERT_TRUE(r); EXPECT_EQ(r->exec_state,mtc::core::ExecState::REJECTED);
  EXPECT_EQ(server->goals,1u); EXPECT_EQ(port->snapshot().goals_sent,0u); EXPECT_EQ(server->cancels,0u);
}
TEST_F(FjtAction, GoalResponseBeforeStatusMatchesUuid) {
  server->hide_active_status=true; server->no_result=true;
  auto c=command(); send(c); auto uuid=server->goal_id();
  EXPECT_FALSE(port->snapshot().goal_uuid.empty()); EXPECT_EQ(port->snapshot().ownership,GoalOwnership::OWN_GOAL);
  server->hide_active_status=false;
  ASSERT_TRUE(wait([&]() {return port->snapshot().status_messages>=20;}));
  EXPECT_EQ(port->snapshot().foreign_active_goals,0u); EXPECT_EQ(port->snapshot().unknown_active_goals,0u);
  EXPECT_FALSE(port->snapshot().fault_locked);
  port->cancel_command(c.command_id,"cleanup"); EXPECT_TRUE(result(c).stop_confirmed);
  EXPECT_EQ(server->cancel_id(),uuid); EXPECT_EQ(server->goals,1u);
}
TEST_F(FjtAction, ForeignGoalDuringPendingResponseFailsClosed) {
  server->hold_response=true; server->no_result=true;
  auto c=command(); ASSERT_FALSE(port->execute_trajectory(c));
  ASSERT_TRUE(wait([&]() {return server->pending_requests==1;})); auto own=server->request_id();
  // DDS client names are per node/endpoint, not a count of objects. This genuine
  // second client shares the public Node, demonstrating the topology limitation
  // without weakening any Port gate or touching private state.
  auto outsider=rclcpp_action::create_client<Fjt>(node,cfg.action_name());
  Fjt::Goal goal; goal.trajectory=*convert(command("foreign"),cfg).trajectory; goal.trajectory.header.frame_id="foreign";
  auto response=outsider->async_send_goal(goal);
  ASSERT_TRUE(wait([&]() {return response.wait_for(0s)==std::future_status::ready;})); ASSERT_TRUE(response.get());
  ASSERT_TRUE(wait([&]() {return port->snapshot().unknown_active_goals==1;}));
  EXPECT_EQ(port->snapshot().ownership,GoalOwnership::UNKNOWN); EXPECT_FALSE(port->snapshot().ready);
  EXPECT_TRUE(port->snapshot().goal_uuid.empty()); EXPECT_EQ(server->cancels,0u);
  auto blocked=port->execute_trajectory(command("next")); ASSERT_TRUE(blocked);
  EXPECT_EQ(port->snapshot().goals_sent,1u);
  // Drop that client and hide active Status before acceptance: the observed
  // foreign UUID must remain remembered even if it disappears from later arrays.
  outsider.reset(); server->hide_active_status=true;
  ASSERT_TRUE(wait([&]() {return port->snapshot().foreign_active_goals==0;}));
  server->release_response();
  ASSERT_TRUE(wait([&]() {return port->snapshot().fault_locked && server->cancels==1;}));
  EXPECT_EQ(server->cancel_id(),own); EXPECT_EQ(server->goals,2u); EXPECT_EQ(port->snapshot().goals_sent,1u);
  ASSERT_TRUE(wait([&]() {return port->snapshot().terminal_received;}));
  EXPECT_TRUE(port->snapshot().fault_locked); EXPECT_FALSE(port->snapshot().result->stop_confirmed);
}
TEST_F(FjtAction, CancelBeforeGoalResponseUsesOwnedUuidOnly) {
  server->hold_response=true; server->pending_status=true; server->no_result=true;
  auto c=command(); ASSERT_FALSE(port->execute_trajectory(c));
  ASSERT_TRUE(wait([&]() {return server->pending_requests==1 && port->snapshot().unknown_active_goals==1;}));
  auto own=server->request_id(); auto stop=port->cancel_command(c.command_id,"pending cancel");
  EXPECT_FALSE(stop.request_delivered); EXPECT_FALSE(stop.stop_confirmed); EXPECT_EQ(server->cancels,0u);
  EXPECT_FALSE(port->cancel_command("stale-id","wrong owner").request_delivered);
  server->release_response(); auto r=result(c); EXPECT_TRUE(r.stop_confirmed); EXPECT_FALSE(r.success);
  EXPECT_EQ(server->cancel_id(),own); EXPECT_EQ(server->cancels,1u); EXPECT_EQ(server->goals,1u);
}
TEST_F(FjtAction, HistoricalTerminalStatusDoesNotBecomeActive) {
  auto c=command(); send(c); ASSERT_TRUE(result(c).success);
  ASSERT_TRUE(wait([&]() {return server->terminal_statuses()>0 && port->snapshot().ownership==GoalOwnership::NO_ACTIVE_GOAL;}));
  EXPECT_TRUE(port->snapshot().status_received); EXPECT_TRUE(port->snapshot().status_fresh);
  EXPECT_EQ(port->snapshot().foreign_active_goals,0u); EXPECT_EQ(server->cancels,0u);
  auto next=command("next"); for(auto & p:next.trajectory->points) {for(auto & x:p.positions_rad) {x+=0.02;}}
  send(next); EXPECT_TRUE(result(next).success); EXPECT_EQ(server->goals,2u);
}
TEST_F(FjtAction, StatusEvidenceLossDoesNotAuthorizeDispatch) {
  ASSERT_TRUE(port->snapshot().status_received); server->publish_status=false;
  ASSERT_TRUE(wait([&]() {return !port->snapshot().status_fresh;}));
  EXPECT_EQ(port->snapshot().ownership,GoalOwnership::UNKNOWN); EXPECT_FALSE(port->snapshot().ready);
  auto r=port->execute_trajectory(command()); ASSERT_TRUE(r); EXPECT_EQ(server->goals,0u); EXPECT_EQ(server->cancels,0u);
}
TEST_F(FjtAction, LateResultDoesNotReleaseFaultLock) {
  server->no_result=true; server->reject_cancel=true; auto c=command(); send(c); auto own=server->goal_id();
  server->publish_status=false; ASSERT_TRUE(wait([&]() {return port->snapshot().fault_locked;}));
  auto original=port->snapshot().result; ASSERT_TRUE(original); EXPECT_TRUE(original->status_unknown);
  ASSERT_TRUE(wait([&]() {return server->cancels==1;})); EXPECT_EQ(server->cancel_id(),own);
  server->no_result=false; ASSERT_TRUE(wait([&]() {return port->snapshot().terminal_received;}));
  EXPECT_TRUE(port->snapshot().fault_locked); EXPECT_EQ(port->snapshot().result->message,original->message);
  EXPECT_FALSE(port->snapshot().result->success); EXPECT_FALSE(port->snapshot().result->stop_confirmed);
  auto r=port->execute_trajectory(command("next")); ASSERT_TRUE(r); EXPECT_TRUE(r->status_unknown); EXPECT_EQ(server->goals,1u);
}
TEST_F(FjtAction, NormalFakeTrajectoryStillExecutesOnce) {
  auto c=command(); send(c); auto r=result(c); EXPECT_TRUE(r.success); EXPECT_TRUE(r.stop_confirmed);
  EXPECT_EQ(server->trajectory().points.size(),3u); EXPECT_EQ(server->goals,1u); EXPECT_EQ(server->cancels,0u);
  EXPECT_GT(port->snapshot().status_messages,0u); EXPECT_FALSE(port->snapshot().fault_locked);
}
TEST_F(FjtAction, LatchedEmptyStatusCannotAuthorizeDispatch) {
  EXPECT_FALSE(port->snapshot().status_received); EXPECT_FALSE(port->snapshot().ready);
  auto r=port->execute_trajectory(command()); ASSERT_TRUE(r); EXPECT_EQ(server->goals,0u); EXPECT_EQ(server->cancels,0u);
}
TEST_F(FjtAction, InvalidStatusDoesNotAuthorizeDispatch) {
  server->invalid_status=true; ASSERT_TRUE(wait([&]() {return !port->snapshot().status_fresh;}));
  EXPECT_TRUE(port->snapshot().status_received); EXPECT_EQ(port->snapshot().ownership,GoalOwnership::UNKNOWN);
  auto r=port->execute_trajectory(command()); ASSERT_TRUE(r); EXPECT_EQ(server->goals,0u); EXPECT_EQ(server->cancels,0u);
}
TEST_F(FjtAction, PendingResponseTimeoutRetainsSessionAndLateHandle) {
  server->hold_response=true; server->pending_status=true; server->no_result=true;
  auto c=command(); c.timeout_ms=400; ASSERT_FALSE(port->execute_trajectory(c));
  ASSERT_TRUE(wait([&]() {return port->snapshot().fault_locked;})); auto own=server->request_id(); auto before=port->snapshot();
  ASSERT_TRUE(before.result); EXPECT_TRUE(before.result->timed_out); EXPECT_EQ(before.command_id,c.command_id);
  EXPECT_TRUE(before.goal_uuid.empty()); EXPECT_EQ(server->cancels,0u);
  auto r=port->execute_trajectory(command("next")); ASSERT_TRUE(r); EXPECT_TRUE(r->status_unknown);
  server->release_response(); ASSERT_TRUE(wait([&]() {return server->cancels==1 && port->snapshot().terminal_received;}));
  EXPECT_EQ(server->cancel_id(),own); EXPECT_TRUE(port->snapshot().fault_locked);
  EXPECT_TRUE(port->snapshot().result->timed_out); EXPECT_FALSE(port->snapshot().result->stop_confirmed);
  EXPECT_EQ(port->snapshot().goals_sent,1u); EXPECT_EQ(server->goals,1u);
}
TEST_F(FjtAction, RejectedResponseAfterUnknownStatusKeepsFaultLock) {
  server->hold_response=true; server->pending_status=true; server->reject=true;
  auto c=command(); ASSERT_FALSE(port->execute_trajectory(c));
  ASSERT_TRUE(wait([&]() {return port->snapshot().unknown_active_goals==1;}));
  server->release_response(); ASSERT_TRUE(wait([&]() {return port->snapshot().terminal_received;}));
  EXPECT_TRUE(port->snapshot().fault_locked); EXPECT_TRUE(port->snapshot().goal_uuid.empty());
  EXPECT_TRUE(port->snapshot().result->status_unknown); EXPECT_EQ(server->goals,1u); EXPECT_EQ(server->cancels,0u);
}

TEST_F(FjtAction, TerminalResultBeforeStatusDoesNotAuthorizeSuccessor) {
  server->no_result=true; auto c=command(); send(c);
  ASSERT_TRUE(wait([&]() {return server->active_statuses()==1;}));
  server->hold_terminal_status=true; server->no_result=false;
  ASSERT_TRUE(result(c).success); // Real terminal Result and separate JointState settle.
  EXPECT_EQ(port->snapshot().ownership,GoalOwnership::UNKNOWN); EXPECT_FALSE(port->snapshot().ready);
  auto next=command("next"); for(auto & p:next.trajectory->points) {for(auto & x:p.positions_rad) {x+=0.02;}}
  auto r=port->execute_trajectory(next); ASSERT_TRUE(r); EXPECT_EQ(r->exec_state,mtc::core::ExecState::REJECTED);
  EXPECT_EQ(server->goals,1u); EXPECT_EQ(server->cancels,0u);
  server->release_terminal_status(); ASSERT_TRUE(wait([&]() {return port->snapshot().ready;}));
  EXPECT_EQ(port->snapshot().ownership,GoalOwnership::NO_ACTIVE_GOAL);
  send(next); EXPECT_TRUE(result(next).success); EXPECT_EQ(server->goals,2u);
}

// Backport-only concurrency checks: the existing strict live Status fixture is
// retained. The real Action transport runs in the four-thread executor above.
TEST_F(FjtAction, ConcurrentPollingAndCancel) {
  server->no_result=true; auto c=command(); send(c); const auto own=server->goal_id();
  std::atomic<bool> invalid{false};
  std::thread polling([&]() {
    for(int i=0;i<150;++i) {
      auto r=port->execute_trajectory(c);
      if(r && r->success) {invalid=true;}
      std::this_thread::sleep_for(1ms);
    }
  });
  std::thread canceling([&]() {for(int i=0;i<150;++i) {port->cancel_command(c.command_id,"concurrent"); std::this_thread::sleep_for(1ms);}});
  polling.join(); canceling.join();
  auto r=result(c); EXPECT_FALSE(invalid); EXPECT_TRUE(r.stop_confirmed); EXPECT_FALSE(r.success);
  EXPECT_EQ(server->goals,1u); EXPECT_EQ(server->cancels,1u); EXPECT_EQ(server->cancel_id(),own);
}
TEST_F(FjtAction, RepeatedScopedCancelUnderContention) {
  server->no_result=true; auto c=command(); send(c); const auto own=server->goal_id();
  std::vector<std::thread> callers;
  for(int n=0;n<4;++n) {callers.emplace_back([&]() {for(int i=0;i<100;++i) {port->cancel_command(c.command_id,"repeat");}});}
  for(auto & t:callers) {t.join();}
  EXPECT_TRUE(result(c).stop_confirmed); EXPECT_EQ(server->cancels,1u); EXPECT_EQ(server->goals,1u);
  EXPECT_EQ(server->cancel_id(),own);
}
TEST_F(FjtAction, ConcurrentSnapshotPolling) {
  auto c=command(); send(c); std::atomic<bool> bad{false};
  std::thread observer([&]() {
    for(int i=0;i<200;++i) {
      auto s=port->snapshot();
      if(s.command_id!=c.command_id || s.goals_sent>1 || s.cancel_requests || s.fault_locked) {bad=true;}
      std::this_thread::sleep_for(1ms);
    }
  });
  auto r=result(c); observer.join(); EXPECT_FALSE(bad); EXPECT_TRUE(r.success);
  EXPECT_EQ(server->trajectory().points.size(),3u); EXPECT_EQ(server->goals,1u); EXPECT_EQ(server->cancels,0u);
}
TEST_F(FjtAction, ConcurrentResponseAndCancelPreservesUuid) {
  server->hold_response=true; server->pending_status=true; server->no_result=true;
  auto c=command(); ASSERT_FALSE(port->execute_trajectory(c));
  ASSERT_TRUE(wait([&]() {return server->pending_requests==1;})); const auto own=server->request_id();
  std::thread canceling([&]() {for(int i=0;i<100;++i) {port->cancel_command(c.command_id,"before response"); std::this_thread::sleep_for(1ms);}});
  std::thread releasing([&]() {server->release_response();}); releasing.join(); canceling.join();
  auto r=result(c); EXPECT_TRUE(r.stop_confirmed); EXPECT_FALSE(r.success);
  EXPECT_EQ(server->goals,1u); EXPECT_EQ(server->cancels,1u); EXPECT_EQ(server->cancel_id(),own);
}
TEST_F(FjtAction, DestroyDuringPendingAction) {
  server->hold_response=true; server->pending_status=true; server->no_result=true;
  auto c=command(); ASSERT_FALSE(port->execute_trajectory(c));
  ASSERT_TRUE(wait([&]() {return server->pending_requests==1;})); const auto own=server->request_id();
  EXPECT_TRUE(port->snapshot().goal_uuid.empty()); port.reset(); server->release_response();
  EXPECT_TRUE(wait([&]() {return server->cancels==1;})); EXPECT_EQ(server->cancel_id(),own);
  EXPECT_EQ(server->goals,1u); EXPECT_THROW(FjtMotionPort(node,cfg),std::invalid_argument);
}
TEST_F(FjtAction, LateGoalResponseAfterFaultRetainsOriginalSession) {
  server->hold_response=true; server->pending_status=true; server->no_result=true;
  auto c=command(); c.timeout_ms=400; ASSERT_FALSE(port->execute_trajectory(c));
  ASSERT_TRUE(wait([&]() {return server->pending_requests==1;})); const auto own=server->request_id();
  ASSERT_TRUE(wait([&]() {return port->snapshot().fault_locked;}));
  auto before=port->execute_trajectory(c); ASSERT_TRUE(before); EXPECT_TRUE(before->timed_out);
  auto next=port->execute_trajectory(command("next")); ASSERT_TRUE(next); EXPECT_TRUE(next->status_unknown);
  server->release_response(); ASSERT_TRUE(wait([&]() {return server->cancels==1 && port->snapshot().terminal_received;}));
  EXPECT_EQ(server->cancel_id(),own); EXPECT_EQ(port->snapshot().command_id,c.command_id);
  auto after=port->execute_trajectory(c); ASSERT_TRUE(after); EXPECT_EQ(after->exec_state,before->exec_state);
  EXPECT_FALSE(after->stop_confirmed); EXPECT_TRUE(port->snapshot().fault_locked); EXPECT_EQ(server->goals,1u);
}
TEST_F(FjtAction, RepeatedSessionPollingAndScopedCancel) {
  server->no_result=true;
  for(int pass=0;pass<5;++pass) {
    ASSERT_TRUE(wait([&]() {return port->snapshot().ready;}));
    auto c=command("sequence_"+std::to_string(pass)); send(c); const auto own=server->goal_id();
    std::thread polling([&]() {for(int i=0;i<60;++i) {port->execute_trajectory(c); port->snapshot(); std::this_thread::sleep_for(1ms);}});
    std::thread canceling([&]() {for(int i=0;i<60;++i) {port->cancel_command(c.command_id,"scoped"); std::this_thread::sleep_for(1ms);}});
    polling.join(); canceling.join(); auto r=result(c);
    ASSERT_TRUE(r.stop_confirmed) << r.message; EXPECT_FALSE(r.success);
    EXPECT_EQ(server->goals,size_t(pass+1)); EXPECT_EQ(server->cancels,size_t(pass+1)); EXPECT_EQ(server->cancel_id(),own);
  }
}
TEST_F(FjtAction, ConcurrentCompletedIdentityRejection) {
  auto c=command(); send(c); ASSERT_TRUE(result(c).success); auto changed=c; ++changed.timeout_ms;
  std::atomic<bool> bad{false};
  std::thread original([&]() {for(int i=0;i<150;++i) {auto r=port->execute_trajectory(c); if(!r || !r->success) {bad=true;}}});
  std::thread mismatch([&]() {for(int i=0;i<150;++i) {auto r=port->execute_trajectory(changed); if(!r || r->exec_state!=mtc::core::ExecState::REJECTED) {bad=true;}}});
  original.join(); mismatch.join(); EXPECT_FALSE(bad); EXPECT_EQ(server->goals,1u); EXPECT_EQ(server->cancels,0u);
  EXPECT_FALSE(port->snapshot().fault_locked);
}
