// Copyright 2026 MTC contributors
#include "mtc_trajectory_execution_cpp/fjt_motion_port.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <action_msgs/msg/goal_status_array.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace mtc::trajectory
{
using Fjt = control_msgs::action::FollowJointTrajectory;
using Handle = rclcpp_action::ClientGoalHandle<Fjt>;
using Hardware = controller_manager_msgs::srv::ListHardwareComponents;
using Controllers = controller_manager_msgs::srv::ListControllers;
using Steady = std::chrono::steady_clock;
using Time = Steady::time_point;
using mtc::core::MotionResult;
using mtc::core::StopResult;
using mtc::core::ExecState;
using E = mtc::core::ErrorCodes;
static double seconds(Time a, Time b) {return std::chrono::duration<double>(a-b).count();}
static MotionResult rejected(const std::string & why)
{
  MotionResult r; r.exec_state=ExecState::REJECTED; r.error_code=E::EXECUTION_REJECTED;
  r.message=why; return r;
}
static std::string uuid_text(const rclcpp_action::GoalUUID & u)
{
  std::ostringstream s; for (auto b:u) {s<<std::hex<<std::setw(2)<<std::setfill('0')<<int(b);} return s.str();
}
// Compare the original, immutable command semantics before ROS normalization.
// In particular MotionPrimitive::operator== omits target_pose, so compare it here.
static bool same_command(const mtc::core::MotionCommand & a,const mtc::core::MotionCommand & b)
{
  if (a.command_id!=b.command_id || a.joint_names!=b.joint_names || a.target_rad!=b.target_rad ||
    a.velocity_scaling!=b.velocity_scaling || a.acceleration_scaling!=b.acceleration_scaling ||
    a.timeout_ms!=b.timeout_ms || a.primitive_name!=b.primitive_name ||
    a.requires_contact!=b.requires_contact || a.simulation_only!=b.simulation_only ||
    a.intent.has_value()!=b.intent.has_value() || a.trajectory.has_value()!=b.trajectory.has_value()) {return false;}
  if (a.intent && (!(*a.intent==*b.intent) || a.intent->target_pose!=b.intent->target_pose)) {return false;}
  if (!a.trajectory) {return true;}
  const auto & x=*a.trajectory; const auto & y=*b.trajectory;
  if (x.trajectory_id!=y.trajectory_id || x.joint_names!=y.joint_names ||
    x.header_frame_id!=y.header_frame_id || x.stamp_s!=y.stamp_s || x.points.size()!=y.points.size()) {return false;}
  for (size_t i=0;i<x.points.size();++i) {
    const auto & p=x.points[i]; const auto & q=y.points[i];
    if (p.time_from_start_s!=q.time_from_start_s || p.positions_rad!=q.positions_rad ||
      p.velocities_rad_s!=q.velocities_rad_s || p.accelerations_rad_s2!=q.accelerations_rad_s2) {return false;}
  }
  return true;
}
struct FjtMotionPort::Impl : std::enable_shared_from_this<Impl>
{
  struct Session
  {
    const mtc::core::MotionCommand identity;
    explicit Session(const mtc::core::MotionCommand & c) : identity(c),id(c.command_id) {}
    std::set<rclcpp_action::GoalUUID> pending_status_ids;
    std::string id, uuid;
    trajectory_msgs::msg::JointTrajectory trajectory;
    uint32_t timeout_ms;
    Time sent, terminal_at{}, stationary_since{}, last_stationary{};
    size_t stationary_samples=0, feedback_samples=0;
    int64_t stationary_stamp_ns=0;
    bool terminal=false, cancel=false, cancel_sent=false, cancel_ack=false;
    rclcpp_action::ResultCode code=rclcpp_action::ResultCode::UNKNOWN;
    int controller_code=Fjt::Result::INVALID_GOAL;
    std::string controller_message;
    Handle::SharedPtr handle;
    std::optional<MotionResult> result;
  };
  rclcpp::Node::SharedPtr node;
  Config cfg;
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp_action::Client<Fjt>::SharedPtr client;
  rclcpp::Client<Hardware>::SharedPtr hardware;
  rclcpp::Client<Controllers>::SharedPtr controllers;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_subscription;
  std::vector<action_msgs::msg::GoalStatus> goal_statuses;
  bool status_received=false, status_valid=false;
  size_t status_messages=0;
  Time status_at{};
  Time idle_since{}, idle_last{};
  int64_t idle_stamp_ns=0;
  size_t idle_samples=0;
  rclcpp::TimerBase::SharedPtr timer;
  mutable std::mutex mutex;
  std::vector<std::function<void()>> outgoing; // At most one send + one scoped cancel per Session.
  std::shared_ptr<Session> active;
  struct Completed
  {
    const mtc::core::MotionCommand identity;
    const MotionResult result;
  };
  std::map<std::string, Completed> completed;
  std::optional<JointSample> sample;
  Time sample_at{}, hw_at{}, controller_at{}, last_proof{};
  std::optional<std::pair<int64_t,Time>> hw_pending, controllers_pending;
  bool hw_ok=false, controller_ok=false, locked=false, closed=false;
  std::string detail="waiting for runtime Fake Hardware and state proofs";
  size_t goals_sent=0, cancels_sent=0;
  std::string lease_key;
  Impl(rclcpp::Node::SharedPtr n, Config c) : node(std::move(n)),cfg(std::move(c))
  {cfg.validate(); if (!node) {throw std::invalid_argument("node is required");}}

  // Trusted isolated graph check, not authentication against malicious nodes.
  bool exclusive_graph() const
  {
    if (node->count_publishers(cfg.joint_state_topic)!=1 ||
      node->count_publishers("/"+cfg.controller+"/joint_trajectory")!=0) {return false;}
    size_t action_servers=0, action_clients=0, hw_servers=0, controller_servers=0;
    for (const auto & n:node->get_node_graph_interface()->get_node_names_and_namespaces()) {
      auto services=node->get_service_names_and_types_by_node(n.first,n.second);
      action_servers+=services.count(cfg.action_name()+"/_action/send_goal");
      hw_servers+=services.count(cfg.controller_manager+"/list_hardware_components");
      controller_servers+=services.count(cfg.controller_manager+"/list_controllers");
      auto clients=node->get_node_graph_interface()->get_client_names_and_types_by_node(n.first,n.second);
      action_clients+=clients.count(cfg.action_name()+"/_action/send_goal");
    }
    return action_servers==1 && action_clients==1 && hw_servers==1 && controller_servers==1;
  }
  size_t foreign_goals() const
  {
    size_t count=0;
    for (const auto & status:goal_statuses) {
      if (status.status==action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        status.status==action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
        status.status==action_msgs::msg::GoalStatus::STATUS_CANCELING) {
        if (!active || (active->handle && status.goal_info.goal_id.uuid!=active->handle->get_goal_id())) {++count;}
        else if (!active->handle) {++count;}
      }
    }
    return count;
  }
  bool status_usable(Time now) const
  {
    // Status has no publication timestamp. This bounds local reception age only;
    // it is never a cross-process execution lock or proof against replay.
    return status_received && status_valid && seconds(now,status_at)<=cfg.proof_max_age;
  }
  GoalOwnership ownership(Time now) const
  {
    if (!status_usable(now)) {return GoalOwnership::UNKNOWN;}
    if (active && !active->handle && !active->terminal) {return GoalOwnership::UNKNOWN;}
    if (foreign_goals()) {return GoalOwnership::FOREIGN_GOAL;}
    if (active && active->handle) {
      if (!active->terminal) {return GoalOwnership::OWN_GOAL;}
      for (const auto & s:goal_statuses) {
        if (s.status==action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
          s.status==action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
          s.status==action_msgs::msg::GoalStatus::STATUS_CANCELING) {return GoalOwnership::UNKNOWN;}
      }
    }
    return GoalOwnership::NO_ACTIVE_GOAL;
  }
  void on_status(const action_msgs::msg::GoalStatusArray & msg)
  {
    std::lock_guard<std::mutex> lock(mutex); if (closed) {return;}
    status_received=true; ++status_messages; status_at=Steady::now(); status_valid=true;
    std::set<rclcpp_action::GoalUUID> seen;
    for (const auto & s:msg.status_list) {
      const auto & id=s.goal_info.goal_id.uuid;
      if (!seen.insert(id).second || std::all_of(id.begin(),id.end(),[](auto b) {return b==0;}) ||
        s.status<action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        s.status>action_msgs::msg::GoalStatus::STATUS_ABORTED ||
        s.goal_info.stamp.sec<0 || s.goal_info.stamp.nanosec>=1000000000u ||
        (s.goal_info.stamp.sec==0 && s.goal_info.stamp.nanosec==0)) {status_valid=false;}
      if (active && !active->handle && !active->terminal &&
        (s.status==action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        s.status==action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
        s.status==action_msgs::msg::GoalStatus::STATUS_CANCELING)) {active->pending_status_ids.insert(id);}
    }
    goal_statuses=msg.status_list;
  }
  bool idle_verified() const
  {
    return sample && idle_samples>=2 && seconds(idle_last,idle_since)>=cfg.settle_duration &&
      (sample->stamp_ns-idle_stamp_ns)/1e9>=cfg.settle_duration;
  }
  bool environment_healthy(Time now) const
  {
    if (!cfg.enable_fake_execution || !hw_ok || !controller_ok || !sample ||
      seconds(now,hw_at)>cfg.proof_max_age || seconds(now,controller_at)>cfg.proof_max_age ||
      seconds(now,sample_at)>cfg.feedback_max_age || !client->action_server_is_ready()) {return false;}
    const double age=(node->now().nanoseconds()-sample->stamp_ns)/1e9;
    return age>=-0.01 && age<=cfg.feedback_max_age && exclusive_graph();
  }
  bool healthy(Time now) const
  {
    if (!environment_healthy(now) || !status_usable(now) || foreign_goals() ||
      (active && !active->handle && !active->terminal)) {return false;}
    if ((!active || active->result) && !idle_verified()) {return false;}
    // A result may arrive before its terminal Status; do not dispatch a successor
    // while the live Status still calls that owned UUID active.
    if (active && active->result) {
      for (const auto & s:goal_statuses) {
        if (s.status==action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
          s.status==action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
          s.status==action_msgs::msg::GoalStatus::STATUS_CANCELING) {return false;}
      }
    }
    return true;
  }
  void fault(const std::string & why, bool timeout=false)
  {
    locked=true; detail=why;
    if (active && !active->result) {
      MotionResult r; r.status_unknown=true; r.timed_out=timeout;
      r.exec_state=timeout ? ExecState::TIMEOUT : ExecState::STOP_UNKNOWN;
      r.error_code=timeout ? E::MOTION_TIMEOUT : E::MOTION_STATUS_UNKNOWN;
      r.message=why; active->result=r;
      active->cancel=true;  // preserve ownership for late acceptance/cancellation
    }
  }
  void issue_cancel()
  {
    if (!active || !active->cancel || !active->handle || active->terminal || active->cancel_sent) {return;}
    auto s=active; s->cancel_sent=true;
    std::weak_ptr<Impl> weak=shared_from_this(); auto action_client=client;
    // rclcpp_action invokes callbacks while holding internal Goal locks. Never
    // acquire those locks while holding mutex: pin the Session, then drain outside.
    outgoing.push_back([weak,s,action_client]() {
      try {
        action_client->async_cancel_goal(s->handle,[weak,s](auto f) {
          auto self=weak.lock(); if (!self) {return;}
          std::lock_guard<std::mutex> lock(self->mutex);
          if (self->active!=s || self->closed) {return;}
          try {
            auto response=f.get();
            for (const auto & info:response->goals_canceling) {
              if (info.goal_id.uuid==s->handle->get_goal_id()) {s->cancel_ack=true;}
            }
          } catch (const std::exception & e) {self->fault(std::string("cancel response exception: ")+e.what());}
        });
        if (auto self=weak.lock()) {std::lock_guard<std::mutex> lock(self->mutex); ++self->cancels_sent;}
      } catch (const std::exception & e) {
        if (auto self=weak.lock()) {
          std::lock_guard<std::mutex> lock(self->mutex);
          // An owned terminal Result can win before the queued Cancel is sent.
          // UnknownGoalHandle then supplies no ACK; Result + fresh samples remain required.
          if (!self->closed && self->active==s && !s->terminal) {self->fault(std::string("cancel exception: ")+e.what());}
        }
      }
    });
  }
  void drain_outgoing()
  {
    std::vector<std::function<void()>> work;
    {std::lock_guard<std::mutex> lock(mutex); work.swap(outgoing);}
    for (auto & action:work) {action();}
  }
  void refresh_proofs(Time now)
  {
    if (hw_pending && seconds(now,hw_pending->second)>cfg.proof_max_age) {
      hardware->remove_pending_request(hw_pending->first); hw_pending.reset(); hw_ok=false;
    }
    if (controllers_pending && seconds(now,controllers_pending->second)>cfg.proof_max_age) {
      controllers->remove_pending_request(controllers_pending->first); controllers_pending.reset(); controller_ok=false;
    }
    if (seconds(now,last_proof)<0.1) {return;} last_proof=now;
    auto weak=weak_from_this();
    if (!hw_pending && hardware->service_is_ready()) {
      auto f=hardware->async_send_request(std::make_shared<Hardware::Request>(),[weak](rclcpp::Client<Hardware>::SharedFuture response) {
        auto self=weak.lock(); if (!self) {return;}
        std::lock_guard<std::mutex> lock(self->mutex); self->hw_pending.reset();
        auto r=response.get(); self->hw_at=Steady::now(); self->hw_ok=false;
        if (r->component.size()!=1) {return;}
        const auto & h=r->component[0];
        if (h.name!=self->cfg.hardware_component || h.plugin_name!="mock_components/GenericSystem" ||
          h.state.id!=lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {return;}
        std::set<std::string> available;
        for (const auto & i:h.state_interfaces) {if (i.is_available) {available.insert(i.name);}}
        for (const auto & j:self->cfg.joints) {
          if (!available.count(j+"/position") || !available.count(j+"/velocity")) {return;}
        }
        self->hw_ok=true;
      });
      hw_pending=std::make_pair(f.request_id,now);
    }
    if (!controllers_pending && controllers->service_is_ready()) {
      auto f=controllers->async_send_request(std::make_shared<Controllers::Request>(),[weak](rclcpp::Client<Controllers>::SharedFuture response) {
        auto self=weak.lock(); if (!self) {return;}
        std::lock_guard<std::mutex> lock(self->mutex); self->controllers_pending.reset();
        auto r=response.get(); self->controller_at=Steady::now(); self->controller_ok=false;
        std::set<std::string> expected;
        for (const auto & j:self->cfg.joints) {expected.insert(j+"/position");}
        size_t matches=0; bool broadcaster=false;
        for (const auto & c:r->controller) {
          if (c.name=="joint_state_broadcaster" && c.state=="active" &&
            c.type=="joint_state_broadcaster/JointStateBroadcaster") {broadcaster=true;}
          if (c.name==self->cfg.controller) {
            if (c.state!="active" || c.type!="joint_trajectory_controller/JointTrajectoryController" ||
              std::set<std::string>(c.claimed_interfaces.begin(),c.claimed_interfaces.end())!=expected) {return;}
            ++matches;
          } else {
            for (const auto & i:c.claimed_interfaces) {
              for (const auto & j:self->cfg.joints) {if (i.rfind(j+"/",0)==0) {return;}}
            }
          }
        }
        self->controller_ok=matches==1 && broadcaster;
      });
      controllers_pending=std::make_pair(f.request_id,now);
    }
  }
  void on_sample(const sensor_msgs::msg::JointState & msg)
  {
    std::lock_guard<std::mutex> lock(mutex); if (closed) {return;}
    auto mapped=map_sample(msg,cfg); auto now=Steady::now();
    const auto ros_now=node->now().nanoseconds();
    if (!mapped || mapped->stamp_ns>ros_now+10000000LL ||
      ros_now-mapped->stamp_ns>static_cast<int64_t>(cfg.feedback_max_age*1e9) ||
      (sample && mapped->stamp_ns<=sample->stamp_ns))
    {
      sample.reset(); idle_samples=0;
      if (active) {active->stationary_samples=0; active->stationary_since={};}
      if (active && !active->result) {fault("invalid, stale or non-increasing JointState timestamp"); issue_cancel();}
      return;
    }
    const auto previous=sample; const auto previous_at=sample_at;
    sample=mapped; sample_at=now;
    bool idle=true;
    for(double v:mapped->velocities) {idle=idle && std::abs(v)<=cfg.stationary_velocity;}
    if (previous) {
      const double dt=(mapped->stamp_ns-previous->stamp_ns)/1e9;
      for(size_t j=0;j<cfg.joints.size();++j) {
        idle=idle && dt>0 && std::abs(mapped->positions[j]-previous->positions[j])/dt<=cfg.stationary_velocity;
      }
    }
    if (!idle || (previous && seconds(now,previous_at)>cfg.feedback_max_age)) {idle_samples=0;}
    else {
      if (!idle_samples) {idle_since=now; idle_stamp_ns=mapped->stamp_ns;}
      ++idle_samples; idle_last=now;
    }
    if (!active || !active->terminal) {return;}
    auto & s=*active;
    bool stationary=now>=s.terminal_at;
    for (double v:mapped->velocities) {stationary=stationary && std::abs(v)<=cfg.stationary_velocity;}
    if (previous) {
      const auto dt=(mapped->stamp_ns-previous->stamp_ns)/1e9;
      for (size_t j=0;j<cfg.joints.size();++j) {
        stationary=stationary && dt>0 && std::abs(mapped->positions[j]-previous->positions[j])/dt<=cfg.stationary_velocity;
      }
    }
    const bool success=s.code==rclcpp_action::ResultCode::SUCCEEDED && s.controller_code==Fjt::Result::SUCCESSFUL;
    if (success && !s.cancel) {stationary=stationary && near(mapped->positions,s.trajectory.points.back().positions,cfg.final_tolerance);}
    if (!stationary || (previous && seconds(now,previous_at)>cfg.feedback_max_age)) {
      s.stationary_samples=0; s.stationary_since={}; return;
    }
    if (!s.stationary_samples) {s.stationary_since=now; s.stationary_stamp_ns=mapped->stamp_ns;}
    ++s.stationary_samples; s.last_stationary=now;
  }
  void tick_state()
  {
    std::lock_guard<std::mutex> lock(mutex); if (closed || !cfg.enable_fake_execution) {return;}
    auto now=Steady::now();
    try {
      refresh_proofs(now);
      if (!active || active->result) {issue_cancel(); return;}
      auto & s=*active;
      if (seconds(now,s.sent)>=s.timeout_ms/1000.0) {fault("execution or stationary confirmation deadline exceeded",true); issue_cancel(); return;}
      if (!environment_healthy(now) || !status_usable(now)) {
        fault("runtime Fake/controller/state/graph or live Status evidence lost"); issue_cancel(); return;
      }
      // Sending is irrevocable. Unknown UUIDs cannot be guessed Own, but retain
      // the pending Session until its bounded response/timeout. No next Goal or
      // cancel without a matching handle. Remember every UUID across updates.
      if (!s.handle && !s.terminal) {return;}
      if (foreign_goals()) {fault("unassociated active Goal UUID; execution ownership lost"); issue_cancel(); return;}
      issue_cancel();
      if (!s.terminal || s.stationary_samples<2 || seconds(s.last_stationary,s.stationary_since)<cfg.settle_duration ||
        (sample->stamp_ns-s.stationary_stamp_ns)/1e9<cfg.settle_duration ||
        seconds(now,s.last_stationary)>cfg.feedback_max_age) {return;}
      MotionResult r; r.stop_confirmed=true; r.final_position_rad=sample->positions;
      if (s.code==rclcpp_action::ResultCode::SUCCEEDED && s.controller_code==Fjt::Result::SUCCESSFUL &&
        !s.cancel && s.feedback_samples>0 && near(sample->positions,s.trajectory.points.back().positions,cfg.final_tolerance))
      {r.success=true; r.arrived=true; r.exec_state=ExecState::SETTLED; r.message="FJT successful; fresh post-terminal stationary interval and final joints verified";}
      else if (s.cancel && s.code==rclcpp_action::ResultCode::CANCELED)
      {r.exec_state=ExecState::STOPPED; r.message="owned Goal terminal CANCELED; independent stationary interval verified";}
      else {r.exec_state=ExecState::FAULT; r.error_code=E::HARDWARE_FAULT; r.message="controller terminal failure: "+s.controller_message; locked=true;}
      s.result=r; completed.emplace(s.id,Completed{s.identity,r}); detail=r.message;
    } catch (const std::exception & e) {fault(std::string("runtime exception: ")+e.what()); issue_cancel();}
  }
  void tick() {tick_state(); drain_outgoing();}
  void init()
  {
    group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client=rclcpp_action::create_client<Fjt>(node,cfg.action_name(),group);
    hardware=node->create_client<Hardware>(cfg.controller_manager+"/list_hardware_components",rclcpp::ServicesQoS(),group);
    controllers=node->create_client<Controllers>(cfg.controller_manager+"/list_controllers",rclcpp::ServicesQoS(),group);
    auto weak=weak_from_this(); rclcpp::SubscriptionOptions options; options.callback_group=group;
    subscription=node->create_subscription<sensor_msgs::msg::JointState>(cfg.joint_state_topic,rclcpp::SensorDataQoS(),
      [weak](sensor_msgs::msg::JointState::ConstSharedPtr msg) {if (auto self=weak.lock()) {self->on_sample(*msg);}},options);
    // Event-driven transient-local history is not current no-goal evidence.
    // Require live publications. An idle native server may remain BLOCKED;
    // this Port never synthesizes an initial empty Status or a heartbeat.
    status_subscription=node->create_subscription<action_msgs::msg::GoalStatusArray>(
      cfg.action_name()+"/_action/status",rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
      [weak](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
        if (auto self=weak.lock()) {self->on_status(*msg);}
      },options);
    timer=node->create_wall_timer(std::chrono::milliseconds(10),[weak]() {if (auto self=weak.lock()) {self->tick();}},group);
  }
  void send(const std::shared_ptr<Session> & s)
  {
    try {send_unchecked(s);} catch (const std::exception & e) {
      std::lock_guard<std::mutex> lock(mutex);
      if (!closed && active==s) {fault(std::string("dispatch proof exception: ")+e.what()); issue_cancel();}
    }
  }
  void send_unchecked(const std::shared_ptr<Session> & s)
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (closed || active!=s || locked) {return;}
      if (!environment_healthy(Steady::now()) || !status_usable(Steady::now()) || foreign_goals()) {
        fault("execution authority lost before wire dispatch"); return;
      }
      if (!near(sample->positions,s->trajectory.points.front().positions,cfg.start_tolerance) ||
        std::any_of(sample->velocities.begin(),sample->velocities.end(),
          [this](double v) {return std::abs(v)>cfg.stationary_velocity;})) {
        fault("starting JointState changed before wire dispatch"); return;
      }
      if (s->trajectory.header.stamp.sec || s->trajectory.header.stamp.nanosec) {
        const double delay=(rclcpp::Time(s->trajectory.header.stamp).nanoseconds()-node->now().nanoseconds())/1e9;
        if (delay<0 || delay>cfg.max_start_delay) {fault("scheduled execution expired before wire dispatch"); return;}
      }
    }
    auto weak=weak_from_this(); auto cancel_client=client;
    rclcpp_action::Client<Fjt>::SendGoalOptions options;
    // Keep this request waitable executable after Port destruction until its
    // response can cancel only the original accepted handle. No Impl is retained.
    options.goal_response_callback=[weak,s,cancel_client,response_group=group](Handle::SharedPtr handle) {
      (void)response_group;
      auto self=weak.lock();
      if (!self) {if (handle) {try {cancel_client->async_cancel_goal(handle);} catch (...) {}} return;}
      std::unique_lock<std::mutex> lock(self->mutex);
      if (self->closed || self->active!=s) {
        lock.unlock();
        if (handle) {try {cancel_client->async_cancel_goal(handle);} catch (...) {}} return;
      }
      if (!handle) {
        if (!s->pending_status_ids.empty()) {self->fault("Goal rejected after unknown active UUID observed");}
        if (!s->result) {s->result=rejected("FJT Goal rejected before execution"); self->completed.emplace(s->id,Completed{s->identity,*s->result});}
        s->terminal=true; return;
      }
      s->handle=handle; s->uuid=uuid_text(handle->get_goal_id());
      for (const auto & id:s->pending_status_ids) {
        if (id!=handle->get_goal_id()) {self->fault("foreign UUID observed during pending Goal response"); break;}
      }
      self->issue_cancel();
    };
    options.feedback_callback=[weak,s](Handle::SharedPtr handle, const std::shared_ptr<const Fjt::Feedback> f) {
      auto self=weak.lock(); if (!self) {return;}
      std::lock_guard<std::mutex> lock(self->mutex);
      if (self->closed || self->active!=s || !s->handle || handle->get_goal_id()!=s->handle->get_goal_id() || s->terminal) {return;}
      sensor_msgs::msg::JointState msg; msg.header=f->header; msg.name=f->joint_names;
      msg.position=f->actual.positions; msg.velocity=f->actual.velocities;
      auto sample=map_sample(msg,self->cfg);
      const double age=sample ? (self->node->now().nanoseconds()-sample->stamp_ns)/1e9 : 1e9;
      if (!sample || age< -0.01 || age>self->cfg.feedback_max_age) {
        self->fault("invalid or stale associated FJT feedback"); self->issue_cancel(); return;
      }
      ++s->feedback_samples;
    };
    options.result_callback=[weak,s](const Handle::WrappedResult & result) {
      auto self=weak.lock(); if (!self) {return;}
      std::lock_guard<std::mutex> lock(self->mutex);
      if (self->closed || self->active!=s || !s->handle || result.goal_id!=s->handle->get_goal_id() || s->terminal) {return;}
      s->terminal=true; s->terminal_at=Steady::now(); s->code=result.code;
      if (result.result) {s->controller_code=result.result->error_code; s->controller_message=result.result->error_string;}
      s->stationary_samples=0; s->stationary_since={};
    };
    Fjt::Goal goal; goal.trajectory=s->trajectory;
    try {
      client->async_send_goal(goal,options);
      std::lock_guard<std::mutex> lock(mutex); ++goals_sent;
    } catch (const std::exception & e) {
      std::lock_guard<std::mutex> lock(mutex);
      if (!closed && active==s) {fault(std::string("wire send exception: ")+e.what()); issue_cancel();}
    }
  }
};
static std::mutex leases_mutex;
static std::set<std::string> leases;
FjtMotionPort::FjtMotionPort(rclcpp::Node::SharedPtr node, Config config)
: impl_(std::make_shared<Impl>(std::move(node),std::move(config)))
{
  if (impl_->cfg.enable_fake_execution) {
    const char * range=std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE");
    const char * peers=std::getenv("ROS_STATIC_PEERS");
    if (!range || std::string(range)!="LOCALHOST" || (peers && *peers) || impl_->node->get_clock()->get_clock_type()!=RCL_ROS_TIME ||
      (impl_->node->has_parameter("use_sim_time") && impl_->node->get_parameter("use_sim_time").as_bool()))
    {throw std::invalid_argument("Fake execution requires LOCALHOST, no static peers and wall-driven ROS time");}
    impl_->lease_key=std::to_string(impl_->node->get_node_base_interface()->get_context()->get_domain_id())+impl_->cfg.action_name();
    std::lock_guard<std::mutex> lock(leases_mutex);
    if (!leases.insert(impl_->lease_key).second) {throw std::invalid_argument("endpoint already owned or quarantined");}
  }
  try {impl_->init();} catch (...) {
    std::lock_guard<std::mutex> lock(leases_mutex); leases.erase(impl_->lease_key); throw;
  }
}
FjtMotionPort::~FjtMotionPort()
{
  auto self=impl_; bool safe=false;
  {
    std::lock_guard<std::mutex> lock(self->mutex);
    if (self->active && !self->active->terminal) {
      self->active->cancel=true; self->issue_cancel();
    }
    safe=!self->locked && (!self->active || (self->active->result &&
      (self->active->result->stop_confirmed || self->active->result->exec_state==ExecState::REJECTED)));
    self->closed=true; self->timer->cancel();
  }
  self->drain_outgoing();
  if (safe) {std::lock_guard<std::mutex> lock(leases_mutex); leases.erase(self->lease_key);}
}
std::optional<MotionResult> FjtMotionPort::execute_joint_move(const mtc::core::MotionCommand &)
{return rejected("single-point entry unsupported; submit one complete TimedTrajectory");}
std::optional<MotionResult> FjtMotionPort::execute_trajectory(const mtc::core::MotionCommand & c)
{
  auto self=impl_; std::lock_guard<std::mutex> lock(self->mutex);
  auto done=self->completed.find(c.command_id);
  if (done!=self->completed.end()) {
    if (!same_command(c,done->second.identity)) {return rejected("command identity reused with changed content");}
    return done->second.result;
  }
  if (self->active && self->active->id==c.command_id) {
    if (!same_command(c,self->active->identity)) {return rejected("command identity reused with changed content");}
    return self->active->result; // nullopt while pending; immutable result after FAULT.
  }
  if (self->locked) {MotionResult r; r.error_code=E::MOTION_STATUS_UNKNOWN; r.exec_state=ExecState::FAULT;
    r.status_unknown=true; r.message="fault locked; no automatic reset or resend"; return r;}
  if (self->active && !self->active->result) {return rejected("another Goal owns execution");}
  if (self->completed.size()>=10000) {return rejected("command ledger capacity reached");}
  auto validation=convert(c,self->cfg); if (!validation.trajectory) {return rejected(validation.error);}
  try {
    if (!self->healthy(Steady::now())) {return rejected("Fake Hardware/controller/Action/fresh state/exclusive graph proof missing");}
    auto & t=*validation.trajectory;
    if (!near(self->sample->positions,t.points.front().positions,self->cfg.start_tolerance)) {
      return rejected("trajectory start differs from current named JointState; no jump permitted");
    }
    for (auto v:self->sample->velocities) {if (std::abs(v)>self->cfg.stationary_velocity) {return rejected("current joints moving");}}
    if (t.header.stamp.sec || t.header.stamp.nanosec) {
      const auto delay=(rclcpp::Time(t.header.stamp).nanoseconds()-self->node->now().nanoseconds())/1e9;
      if (delay<0 || delay>self->cfg.max_start_delay) {return rejected("scheduled stamp stale or beyond capability");}
    }
    auto session=std::make_shared<Impl::Session>(c);
    session->trajectory=t; session->timeout_ms=c.timeout_ms; session->sent=Steady::now();
    self->active=session;
    const auto weak=self->weak_from_this();
    self->outgoing.push_back([weak,session]() {if (auto self=weak.lock()) {self->send(session);}});
    return std::nullopt;
  } catch (const std::exception & e) {
    self->fault(std::string("send/graph exception: ")+e.what());
    if (self->active) {return self->active->result;}
    MotionResult r; r.status_unknown=true; r.error_code=E::MOTION_STATUS_UNKNOWN; r.exec_state=ExecState::FAULT; r.message=e.what(); return r;
  }
}
StopResult FjtMotionPort::cancel_command(const std::string & id,const std::string & reason)
{
  auto self=impl_; std::lock_guard<std::mutex> lock(self->mutex);
  StopResult r;
  if (id.empty() || !self->active || self->active->id!=id) {
    r.error_code=E::EXECUTION_REJECTED; r.message="no matching owned Goal; no cancel sent"; return r;
  }
  auto & s=*self->active;
  if (s.result) {
    bool fresh=false;
    try {fresh=self->healthy(Steady::now()) && s.stationary_samples>=2 &&
      seconds(s.last_stationary,s.stationary_since)>=self->cfg.settle_duration &&
      self->sample && (self->sample->stamp_ns-s.stationary_stamp_ns)/1e9>=self->cfg.settle_duration &&
      seconds(Steady::now(),s.last_stationary)<=self->cfg.feedback_max_age;} catch (...) {}
    r.stop_confirmed=s.result->stop_confirmed && fresh;
    r.request_delivered=s.cancel_ack;
    r.message=r.stop_confirmed ? "owned Goal currently independently settled" : "historical result is not current stationary evidence";
    if (!r.stop_confirmed) {r.error_code=E::CANCEL_NOT_CONFIRMED;} return r;
  }
  s.cancel=true; self->detail=reason; self->issue_cancel();
  r.request_delivered=s.cancel_ack; r.message="associated cancel pending; ACK is not stationary evidence";
  if (self->locked) {r.error_code=E::CANCEL_NOT_CONFIRMED;}
  return r;
}
StopResult FjtMotionPort::request_stop(const std::string & reason)
{
  std::string id; {std::lock_guard<std::mutex> lock(impl_->mutex); if (impl_->active) {id=impl_->active->id;}}
  return cancel_command(id,reason);  // Switching Goals cannot retarget the pinned identity.
}
ExecutionSnapshot FjtMotionPort::snapshot() const
{
  auto self=impl_; std::lock_guard<std::mutex> lock(self->mutex); ExecutionSnapshot s;
  try {s.ready=!self->locked && self->healthy(Steady::now());} catch (...) {s.ready=false;}
  s.status_received=self->status_received; s.status_fresh=self->status_usable(Steady::now());
  s.status_messages=self->status_messages; s.ownership=self->ownership(Steady::now());
  if (self->active && !self->active->handle && !self->active->terminal) {s.unknown_active_goals=self->foreign_goals();}
  s.foreign_active_goals=self->foreign_goals(); s.fault_locked=self->locked; s.goals_sent=self->goals_sent; s.cancel_requests=self->cancels_sent; s.detail=self->detail;
  if (self->active) {auto & a=*self->active; s.command_id=a.id; s.goal_uuid=a.uuid;
    s.in_flight=!a.terminal; s.cancel_acknowledged=a.cancel_ack; s.terminal_received=a.terminal;
    s.feedback_samples=a.feedback_samples; s.stationary_samples=a.stationary_samples;
    if (a.stationary_samples) {s.stationary_observed_seconds=seconds(a.last_stationary,a.stationary_since);}
    s.result=a.result;}
  return s;
}
}  // namespace mtc::trajectory
