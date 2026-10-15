// Manual Fake-only verification program. Not a robot bringup or production planner.
#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "mtc_trajectory_execution_cpp/fjt_motion_port.hpp"
using namespace std::chrono_literals;
int main(int argc,char ** argv)
{
  if (argc!=3 || std::string(argv[1])!="--verify-generic-system" ||
    (std::string(argv[2])!="success" && std::string(argv[2])!="cancel" && std::string(argv[2])!="mismatch" && std::string(argv[2])!="identity" && std::string(argv[2])!="topology"))
  {std::cerr<<"仅显式 Fake 验证：--verify-generic-system success|cancel|mismatch|identity|topology\n"; return 2;}
  std::string mode=argv[2]; rclcpp::init(argc,argv);
  int code=1;
  try {
    auto node=std::make_shared<rclcpp::Node>("mtc_fake_trajectory_probe");
    mtc::trajectory::Config cfg; cfg.enable_fake_execution=true;
    // Narrow software-test envelope, not calibrated device safety limits.
    for (auto & n:cfg.joints) {cfg.limits[n]={-0.5,0.5,0.2,0.5};}
    std::optional<mtc::trajectory::JointSample> current;
    auto sub=node->create_subscription<sensor_msgs::msg::JointState>(cfg.joint_state_topic,rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::JointState::ConstSharedPtr s) {current=mtc::trajectory::map_sample(*s,cfg);});
    mtc::trajectory::FjtMotionPort port(node,cfg);
    rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
    auto start=std::chrono::steady_clock::now();
    while (rclcpp::ok() && std::chrono::steady_clock::now()-start<10s) {
      executor.spin_some(); if (port.snapshot().ready && current) {break;} std::this_thread::sleep_for(5ms);
    }
    size_t servers=0,clients=0;
    for(const auto & n:node->get_node_graph_interface()->get_node_names_and_namespaces()) {
      const auto endpoint=cfg.action_name()+"/_action/send_goal";
      servers+=node->get_service_names_and_types_by_node(n.first,n.second).count(endpoint);
      if(node->get_node_graph_interface()->get_client_names_and_types_by_node(n.first,n.second).count(endpoint)) {
        ++clients; std::cout<<"FJT client node="<<n.second<<"/"<<n.first<<"\n";
      }
    }
    auto proof=port.snapshot();
    std::cout<<"FJT servers="<<servers<<" clients="<<clients<<" direct_trajectory_publishers="
      <<node->count_publishers("/"+cfg.controller+"/joint_trajectory")
      <<" foreign_active_goals="<<proof.foreign_active_goals<<" status_received="<<proof.status_received
      <<" status_fresh="<<proof.status_fresh<<" unknown_active_goals="<<proof.unknown_active_goals<<" ready="<<proof.ready<<"\n";
    if(mode=="topology") {
      std::cout<<(proof.ready ? "PASS" : "BLOCKED")<<": Fake execution ownership gate; no Goal sent\n";
      executor.remove_node(node); rclcpp::shutdown(); return proof.ready ? 0 : 2;
    }
    if (!port.snapshot().ready || !current) {throw std::runtime_error("BLOCKED: GenericSystem/active controller/fresh state/exclusive graph proof unavailable");}
    mtc::core::MotionCommand c; c.command_id="generic_"+mode+"_"+std::to_string(node->now().nanoseconds());
    c.simulation_only=true; c.timeout_ms=6000;
    mtc::core::TimedTrajectory t; t.trajectory_id="explicit_fake_three_point_probe";
    t.header_frame_id="base_link"; t.joint_names=cfg.joints; std::reverse(t.joint_names.begin(),t.joint_names.end());
    for (int i=0;i<3;++i) {
      mtc::core::TrajectoryPoint p; p.time_from_start_s=i;
      p.positions_rad=current->positions; for (auto & x:p.positions_rad) {x+=0.01*i;}
      if (mode=="mismatch") {p.positions_rad[0]+=0.1;}
      std::reverse(p.positions_rad.begin(),p.positions_rad.end());
      p.velocities_rad_s.assign(6,0); p.accelerations_rad_s2.assign(6,0); t.points.push_back(p);
    }
    c.trajectory=t;
    auto r=port.execute_trajectory(c); start=std::chrono::steady_clock::now(); bool canceled=false;
    while (!r && rclcpp::ok() && std::chrono::steady_clock::now()-start<8s) {
      executor.spin_some();
      if (mode=="cancel" && !canceled && std::chrono::steady_clock::now()-start>500ms) {
        auto stop=port.cancel_command(c.command_id,"explicit Fake probe cancel");
        std::cout<<"cancel initial stop_confirmed="<<stop.stop_confirmed<<"\n"; canceled=true;
      }
      r=port.execute_trajectory(c); std::this_thread::sleep_for(5ms);
    }
    auto s=port.snapshot();
    if (!r) {throw std::runtime_error("NOT RUN: no final execution conclusion");}
    std::cout<<"mode="<<mode<<" goals_sent="<<s.goals_sent<<" cancels_sent="<<s.cancel_requests
      <<" feedback_samples="<<s.feedback_samples<<" UUID="<<s.goal_uuid<<"\n"
      <<"success="<<r->success<<" arrived="<<r->arrived<<" stop_confirmed="<<r->stop_confirmed
      <<" timed_out="<<r->timed_out<<" status_unknown="<<r->status_unknown<<" fault_locked="<<s.fault_locked
      <<" error="<<r->error_code<<" message="<<r->message<<"\nfinal_positions=";
    for(auto x:r->final_position_rad) {std::cout<<x<<",";} std::cout<<"\n";
    bool pass=mode=="mismatch" ? (r->exec_state==mtc::core::ExecState::REJECTED && s.goals_sent==0 &&
      r->message.find("trajectory start differs")!=std::string::npos) :
      (r->stop_confirmed && !r->status_unknown && !s.fault_locked && s.goals_sent==1 &&
      ((mode=="success" || mode=="identity") ? r->ok() : (!r->success && r->exec_state==mtc::core::ExecState::STOPPED && s.cancel_requests==1)));
    if(mode=="identity" && pass) {
      const auto cached=port.execute_trajectory(c);
      pass=cached && cached->ok();
      for(int field=0;field<3;++field) {
        auto changed=c;
        if(field==0) {changed.trajectory->points.back().positions_rad[0]+=0.001;}
        if(field==1) {++changed.timeout_ms;}
        if(field==2) {changed.simulation_only=false;}
        auto refusal=port.execute_trajectory(changed);
        pass=pass && refusal && refusal->exec_state==mtc::core::ExecState::REJECTED;
      }
      auto after=port.snapshot();
      pass=pass && after.goals_sent==1 && after.cancel_requests==0 && !after.fault_locked;
      std::cout<<"identity reread + 3 changed-content refusals; goals_sent="<<after.goals_sent
        <<" cancels_sent="<<after.cancel_requests<<"\n";
    }
    std::cout<<(pass ? "PASS" : "FAIL")<<": GenericSystem "<<mode<<"\n"; code=pass ? 0 : 1;
    executor.remove_node(node);
  } catch(const std::exception & e) {std::cerr<<e.what()<<"\n";}
  rclcpp::shutdown(); return code;
}
