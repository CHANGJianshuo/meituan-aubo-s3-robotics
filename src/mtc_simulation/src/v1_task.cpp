// Gazebo-only adapters for the existing C++ PickPlace FSM. No hardware driver or tool IO.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/string.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <mtc_manipulation_cpp/pick_place_fsm.hpp>
#include <mtc_moveit_planner/move_group_backend.hpp>
#include <mtc_moveit_planner/trajectory_conversion.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <fstream>
#include <future>
#include <mutex>
#include <thread>
#include <filesystem>
using namespace std::chrono_literals;
namespace c=mtc::core;
namespace m=mtc::manipulation;
namespace p=mtc::moveit_planner;
using Json=nlohmann::json;
using Fjt=control_msgs::action::FollowJointTrajectory;
using Handle=rclcpp_action::ClientGoalHandle<Fjt>;
const std::vector<std::string> names={"shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"};
double wall(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
double norm(c::Vector3 a){return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);}
c::Vector3 sub(c::Vector3 a,c::Vector3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
c::Pose6D pose(double x,double y,double z,c::Quaternion q={0,0,0,1}){c::Pose6D r;r.position={x,y,z};r.orientation=q;r.frame_id="world";r.valid=true;return r;}
struct SimClock: c::Clock {rclcpp::Node::SharedPtr node;explicit SimClock(rclcpp::Node::SharedPtr n):node(n){}double now()const override{return node->now().seconds();}};
struct Observation {c::Pose6D body,tcp;double speed=1,angular_speed=1,wall_stamp=0;};

class Session {
public:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<SimClock> clock;
  std::unique_ptr<p::MoveGroupBackend> backend;
  rclcpp_action::Client<Fjt>::SharedPtr client;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr update_scene;
  std::string contact_pair;
  bool contact_allowed=false;
  Json cfg,report={{"simulation_only",true},{"success",false},{"objects",Json::array()}};
  Json replay_commands=Json::array();
  std::string directory,object,slot;
  c::Pose6D ring,target;
  c::Vector3 delivery_direction{-1,0,0},acquire_direction{-1,0,0};
  c::Quaternion tcp_to_body,fixed_orientation;
  bool keep_orientation=false;
  double direction=1,initial_z=0,backoff=.060,target_surface_z=.650;
  double insertion_tcp_height=.060;
  double feedback_wall_timeout=2.0;
  bool holding=false;
  std::mutex mutex,log_mutex,active_mutex;
  std::map<std::string,Observation> observations;
  std::map<std::string,double> joints,velocities;
  double joint_wall=0,contact_wall=0;
  Json contacts;
  Handle::SharedPtr active;
  Handle::SharedPtr active_goal(){std::lock_guard<std::mutex> lock(active_mutex);return active;}
  void set_active(Handle::SharedPtr handle){std::lock_guard<std::mutex> lock(active_mutex);active=std::move(handle);}
  bool cancel_active(){
    Handle::SharedPtr handle;
    {std::lock_guard<std::mutex> lock(active_mutex);handle=std::move(active);active.reset();}
    if(!handle)return false;
    try{client->async_cancel_goal(handle);log("cancel_requested",{{"stop_verified",false}});return true;}
    catch(const std::exception & e){log("cancel_error",{{"detail",e.what()}});return false;}
  }
  std::ofstream events,trace,commands;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr poses_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr contact_sub;
  Session(rclcpp::Node::SharedPtr n):node(n),clock(std::make_shared<SimClock>(n)){
    directory=node->get_parameter("output_dir").as_string();
    std::ifstream f(node->get_parameter("scenario_file").as_string());f>>cfg;
    if(cfg.contains("replay_commands")){
      std::ifstream input(cfg.at("replay_commands").get<std::string>());std::string line;
      if(!input)throw std::runtime_error("replay commands unavailable");
      while(std::getline(input,line))if(!line.empty())replay_commands.push_back(Json::parse(line));
      if(replay_commands.empty())throw std::runtime_error("empty replay commands");
    }
    feedback_wall_timeout=cfg.at("feedback_wall_timeout_s");
    insertion_tcp_height=cfg.value("insertion_tcp_height_above_support_m",.060);
    const double retreat_height=cfg.value("retreat_height_m",.150);
    if(!std::isfinite(retreat_height)||retreat_height<.080||retreat_height>.150)
      throw std::runtime_error("retreat lift must remain between 80 and 150 mm");
    if(cfg.contains("fixed_tcp_orientation_xyzw")){
      const auto q=cfg.at("fixed_tcp_orientation_xyzw").get<std::vector<double>>();
      if(q.size()!=4||!std::all_of(q.begin(),q.end(),[](double v){return std::isfinite(v);})||
         std::abs(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]-1.)>1e-6)
        throw std::runtime_error("fixed tool orientation must be a finite unit quaternion");
      fixed_orientation={q[0],q[1],q[2],q[3]};keep_orientation=true;
      if(!cfg.contains("approach_overhead_tcp_z"))throw std::runtime_error("fixed orientation requires an overhead approach height");
    }
    // Nominal V1 opening: body top 50 mm, bar underside 68 mm.
    // Plate underside is TCP - 2.5 mm; lip top is TCP + 7.5 mm.
    if(!std::isfinite(insertion_tcp_height)||insertion_tcp_height<=.0525||insertion_tcp_height>=.0605)
      throw std::runtime_error("V1 insertion height must clear body and crossbar");
    if(!cfg.at("simulation_only").get<bool>() || cfg.at("tool_type")!="passive_hook_v1" || !node->get_parameter("use_sim_time").as_bool())
      throw std::runtime_error("explicit V1 simulation configuration required");
    events.open(directory+"/events.jsonl");trace.open(directory+"/physics.jsonl");commands.open(directory+"/commands.jsonl");
    poses_sub=node->create_subscription<tf2_msgs::msg::TFMessage>("/simulation/ground_truth",10,[this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg){
      std::lock_guard<std::mutex> lock(mutex);
      c::Pose6D tcp;
      for(const auto & t:msg->transforms)if(t.child_frame_id=="tcp")tcp=convert(t);
      for(const auto & t:msg->transforms)if(t.child_frame_id.find("battery_")==0){
        auto next=convert(t);auto & o=observations[t.child_frame_id];double dt=next.stamp_s-o.body.stamp_s;
        if(o.body.valid && dt>0){o.speed=norm(sub(next.position,o.body.position))/dt;
          const auto & a=next.orientation;const auto & b=o.body.orientation;
          double d=std::fabs(a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w);
          o.angular_speed=2*std::acos(std::min(1.0,d))/dt;}
        o.body=next;o.tcp=tcp;o.wall_stamp=wall();
      }
      Json row={{"sim_time",tcp.stamp_s},{"tcp",{tcp.position.x,tcp.position.y,tcp.position.z}},{"objects",Json::object()}};
      row["tcp_quat"]={tcp.orientation.x,tcp.orientation.y,tcp.orientation.z,tcp.orientation.w};
      row["joint_positions"]=joints;row["joint_velocities"]=velocities;
      for(auto & [id,o]:observations)row["objects"][id]={{"xyz",{o.body.position.x,o.body.position.y,o.body.position.z}},
        {"quat",{o.body.orientation.x,o.body.orientation.y,o.body.orientation.z,o.body.orientation.w}},{"speed",o.speed},{"angular_speed",o.angular_speed}};
      trace<<row.dump()<<'\n';
    });
    joint_sub=node->create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::JointState::ConstSharedPtr msg){
      std::lock_guard<std::mutex> lock(mutex);if(msg->position.size()!=msg->name.size()||msg->velocity.size()!=msg->name.size())return;
      for(size_t i=0;i<msg->name.size();++i){joints[msg->name[i]]=msg->position[i];velocities[msg->name[i]]=msg->velocity[i];}joint_wall=wall();
    });
    contact_sub=node->create_subscription<std_msgs::msg::String>("/simulation/contacts",10,[this](std_msgs::msg::String::ConstSharedPtr msg){
      auto j=Json::parse(msg->data,nullptr,false);if(j.is_discarded())return;
      std::lock_guard<std::mutex> lock(mutex);contacts=std::move(j);contact_wall=wall();
    });
    client=rclcpp_action::create_client<Fjt>(node,"/arm_controller/follow_joint_trajectory");
  }
  static c::Pose6D convert(const geometry_msgs::msg::TransformStamped & t){
    const auto & a=t.transform.translation;const auto & q=t.transform.rotation;
    auto r=pose(a.x,a.y,a.z,{q.x,q.y,q.z,q.w});r.frame_id=t.header.frame_id;r.stamp_s=rclcpp::Time(t.header.stamp).seconds();return r;
  }
  void log(const std::string & event,Json data=Json::object()){
    std::lock_guard<std::mutex> lock(log_mutex);data["event"]=event;data["object"]=object;data["sim_time"]=clock->now();
    events<<data.dump()<<std::endl;RCLCPP_INFO(node->get_logger(),"%s",data.dump().c_str());
  }
  Observation observed(const std::string & id=""){
    std::lock_guard<std::mutex> lock(mutex);auto it=observations.find(id.empty()?object:id);
    if(it==observations.end()||wall()-it->second.wall_stamp>feedback_wall_timeout||clock->now()-it->second.body.stamp_s>.3)throw std::runtime_error("stale physics observation");
    return it->second;
  }
  bool contact(const std::string & kind){
    std::lock_guard<std::mutex> lock(mutex);
    if(wall()-contact_wall>feedback_wall_timeout||!contacts.contains("current"))return false;
    for(auto it=contacts["current"].begin();it!=contacts["current"].end();++it)
      if(it.key().find(object)!=std::string::npos && it.value()["kind"]==kind)return true;
    return false;
  }
  void guard(){
    std::lock_guard<std::mutex> lock(mutex);
    if(wall()-joint_wall>feedback_wall_timeout||wall()-contact_wall>feedback_wall_timeout)
      throw std::runtime_error("joint/contact feedback lost; wall ages="+std::to_string(wall()-joint_wall)+"/"+std::to_string(wall()-contact_wall));
    if(contacts.value("session_id","")!=cfg.at("session_id").get<std::string>()||!contacts.value("gazebo_control_plugin",false))
      throw std::runtime_error("live Gazebo plugin/session identity mismatch");
    if(contacts.contains("total"))for(auto & row:contacts["total"]){
      if((row["kind"]=="unexpected"||row["kind"]=="tool_contact") && row["max_depth_m"].get<double>()>.001)
        throw std::runtime_error("physical collision penetration > 1 mm");
    }
  }
  void initialize(){
    get_scene=node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    update_scene=node->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    using Srv=controller_manager_msgs::srv::ListHardwareComponents;
    auto service=node->create_client<Srv>("/controller_manager/list_hardware_components");
    if(!service->wait_for_service(20s))throw std::runtime_error("no Gazebo controller manager");
    auto f=service->async_send_request(std::make_shared<Srv::Request>());
    if(f.wait_for(5s)!=std::future_status::ready)throw std::runtime_error("hardware identity timeout");
    auto hardware=f.get()->component;
    // gz_ros2_control 1.2.20 imports hardware without setting plugin_name/type.
    // Empty metadata is accepted only with live Gazebo SystemPluginInfo and a
    // per-run token, checked by guard(), in addition to this active component.
    if(hardware.size()!=1||hardware[0].name!="V1GazeboSystem"||
       (!hardware[0].plugin_name.empty()&&hardware[0].plugin_name!="gz_ros2_control/GazeboSimSystem")||hardware[0].state.label!="active")
      throw std::runtime_error("GazeboSimSystem must be the only active hardware component");
    if(!client->wait_for_action_server(10s))throw std::runtime_error("no trajectory server");
    p::MoveGroupBackendOptions options;options.state_timeout_s=10;options.ik_timeout_s=.10;options.ik_seed_timeout_s=.10;
    options.ik_full_joint_samples=96;
    backend=std::make_unique<p::MoveGroupBackend>(node,options);
    robot_model_loader::RobotModelLoader loader(node,"robot_description");
    auto model=loader.getModel();
    for(const auto & name:{"base_link","shoulder_Link","upperArm_Link","foreArm_Link","wrist1_Link","wrist2_Link","wrist3_Link","v1_hook"}){
      const auto * link=model?model->getLinkModel(name):nullptr;
      if(!link||link->getShapes().empty())throw std::runtime_error(std::string("missing MoveIt collision geometry: ")+name);
    }
    auto end=wall()+15;std::string initialization_error;
    while(wall()<end){try{observed("battery_A");guard();return;}
      catch(const std::exception & e){initialization_error=e.what();std::this_thread::sleep_for(50ms);}}
    throw std::runtime_error("simulation observations unavailable: "+initialization_error);
  }
  void select(char id,const std::string & destination){
    object="battery_"+std::string(1,id);slot=destination;holding=false;
    auto o=observed();initial_z=o.body.position.z;
    direction=(id=='A'||id=='B')?-1:1;
    backoff=cfg.at("approach_backoff_m").at(direction>0?"left":"right");
    acquire_direction={direction,0,0};
    if(cfg.contains("acquire_directions")){
      auto v=cfg.at("acquire_directions").at(std::string(1,id));acquire_direction={v[0],v[1],0};
    }
    auto q=p::quaternion_from_axes({0,0,1},{acquire_direction.y,-acquire_direction.x,0},acquire_direction);
    if(keep_orientation)q=fixed_orientation;
    ring=pose(o.body.position.x,o.body.position.y,o.body.position.z+insertion_tcp_height,q);
    if(id=='A'&&cfg.contains("source_anchor")){
      auto a=cfg.at("source_anchor").at("base_xyz_m");
      c::Vector3 expected{a[0],a[1],a[2].get<double>()-.080};
      if(norm(sub(o.body.position,expected))>.001)throw std::runtime_error("source object moved from fixed measured anchor");
      ring.position={a[0],a[1],a[2].get<double>()-.080+insertion_tcp_height};
    }
    tcp_to_body=p::multiply(p::conjugate(q),o.body.orientation);
    auto d=cfg.at("delivery_direction").at(direction>0?"left":"right");
    delivery_direction={d[0],d[1],0};
    auto delivery_q=p::quaternion_from_axes({0,0,1},{delivery_direction.y,-delivery_direction.x,0},delivery_direction);
    if(keep_orientation)delivery_q=fixed_orientation;
    auto v=cfg["targets"][slot];target_surface_z=v[2];target=pose(v[0],v[1],target_surface_z+.060,delivery_q);
    log("selected",{{"slot",slot},{"direction",direction},{"backoff_m",backoff}});
  }
  c::Pose6D goal(const std::string & label){
    auto r=ring;
    if(label=="side_insert"){r.position.x-=acquire_direction.x*backoff;r.position.y-=acquire_direction.y*backoff;}
    else if(label=="hook_engage"){}
    else if(label=="test_lift")r.position.z+=.025+(.060-insertion_tcp_height);
    else if(label=="lift")r.position.z=cfg.at("transport_tcp_z");
    else if(label=="transport"){r=target;r.position.z=cfg.at("transport_tcp_z");}
    else if(label=="pre_seat"){r=target;r.position.z+=.0255;}
    else if(label=="seat_contact"){r=target;r.position.z+=.0050;}
    else if(label=="unload"){r=target;r.position.z+=insertion_tcp_height-.060;}
    else if(label=="side_withdraw"||label=="retreat"){
      r=target;r.position.z+=insertion_tcp_height-.060;
      r.position.x-=delivery_direction.x*.060;r.position.y-=delivery_direction.y*.060;
      if(label=="retreat")r.position.z+=cfg.value("retreat_height_m",.150);
    }
    else throw std::runtime_error("unknown primitive "+label);
    return r;
  }
  void scene(bool payload){
    std::vector<p::SceneBox> boxes;
    // 0.3 mm clearance is solely the contact surface tolerance of the planner.
    if(cfg.contains("scene_boxes")){
      for(const auto & b:cfg.at("scene_boxes")){
        auto x=b.at("xyz");auto d=b.at("size");double yaw=b.value("yaw",0.0);
        boxes.push_back({b.at("id"),pose(x[0],x[1],x[2],{0,0,std::sin(yaw/2),std::cos(yaw/2)}),{d[0],d[1],d[2]}});
      }
    }else boxes.push_back({"table",pose(.17,.043,.6247),{.68,.86,.05}});
    std::map<std::string,Observation> snapshot;
    {std::lock_guard<std::mutex> lock(mutex);snapshot=observations;}
    for(const auto & [id,o]:snapshot){
      if(id==object&&payload)continue;
      auto r=o.body;r.position.z+=(id==object?.025:.040);
      boxes.push_back({id,r,{.07,.08,id==object?.05:.08}});
      // Preserve the opening and the bar. Only the intended hook/bar pair is
      // allowed during test lift; approach must also avoid the handle itself.
      if(id==object){for(double x:{-.03,.03}){
        auto s=o.body;auto d=p::rotate(s.orientation,{x,0,.060});s.position={s.position.x+d.x,s.position.y+d.y,s.position.z+d.z};
        boxes.push_back({id+(x<0?"_left":"_right"),s,{.01,.026,.02}});
      }
        auto bar=o.body;bar.position.z+=.074;
        boxes.push_back({id+"_bar",bar,{.07,.026,.012}});
      }
    }
    if(!backend->apply_scene(boxes))throw std::runtime_error("planning scene rejected");
  }
  void allow_handle_contact(bool allow){
    const std::string bar="mtc_moveit_planner/"+object+"_bar";
    if(contact_pair==bar&&contact_allowed==allow)return;
    if(!get_scene->wait_for_service(2s)||!update_scene->wait_for_service(2s))throw std::runtime_error("collision policy service unavailable");
    auto request=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components=moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto fetched=get_scene->async_send_request(request);
    if(fetched.wait_for(2s)!=std::future_status::ready)throw std::runtime_error("collision policy read timeout");
    collision_detection::AllowedCollisionMatrix acm(fetched.get()->scene.allowed_collision_matrix);
    acm.setEntry("v1_hook",bar,allow);
    auto change=std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();change->scene.is_diff=true;
    acm.getMessage(change->scene.allowed_collision_matrix);
    auto applied=update_scene->async_send_request(change);
    if(applied.wait_for(2s)!=std::future_status::ready||!applied.get()->success)throw std::runtime_error("collision policy update failed");
    contact_pair=bar;contact_allowed=allow;
  }
  c::MotionCommand planned(const c::MotionPrimitive & primitive,const p::JointStateSnapshot & start,bool payload){
    if(!replay_commands.empty()){
      for(const auto & saved:replay_commands)if(saved.at("object")==object&&saved.at("primitive")==primitive.name){
        if(saved.at("joint_names").get<std::vector<std::string>>()!=names||!saved.at("simulation_only").get<bool>())
          throw std::runtime_error("invalid replay identity");
        c::TimedTrajectory trajectory;trajectory.joint_names=names;trajectory.header_frame_id="world";
        for(const auto & row:saved.at("points")){
          c::TrajectoryPoint point;point.time_from_start_s=row.at("t");
          point.positions_rad=row.at("positions").get<std::vector<double>>();
          point.velocities_rad_s=row.at("velocities").get<std::vector<double>>();
          point.accelerations_rad_s2=row.at("accelerations").get<std::vector<double>>();trajectory.points.push_back(point);
        }
        if(!trajectory.shape_is_valid())throw std::runtime_error("invalid replay trajectory");
        c::MotionCommand cmd;cmd.primitive_name=primitive.name;cmd.joint_names=names;cmd.trajectory=trajectory;
        cmd.simulation_only=true;cmd.requires_contact=primitive.requires_contact;cmd.velocity_scaling=.15;cmd.acceleration_scaling=.10;
        cmd.timeout_ms=static_cast<uint32_t>((trajectory.duration_s()+4)*1000);cmd.intent=primitive;
        log("replay_trajectory",{{"primitive",primitive.name},{"duration_s",trajectory.duration_s()}});return cmd;
      }
      throw std::runtime_error("missing replay primitive "+primitive.name);
    }
    if(primitive.name=="side_insert"&&cfg.value("resume_from_prealigned",false)){
      // An explicitly resumed scene must already match the preinsertion pose.
      // Keep its measured joints; a zero-distance Cartesian plan is degenerate.
      guard();const auto measured=observed().tcp;const auto expected=goal("side_insert");
      const double position_error=norm(sub(measured.position,expected.position));
      const auto a=p::normalized(measured.orientation),b=p::normalized(expected.orientation);
      const double dot=std::fabs(a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w);
      const double angle_error=2*std::acos(std::min(1.0,dot));
      if(position_error>.0001||angle_error>.01*M_PI/180.)
        throw std::runtime_error("prealigned resume pose does not match insertion approach");
      std::map<std::string,double> positions;
      for(size_t i=0;i<start.names.size();++i)positions[start.names[i]]=start.positions[i];
      c::TrajectoryPoint point;point.time_from_start_s=0;
      for(const auto & name:names)point.positions_rad.push_back(positions.at(name));
      point.velocities_rad_s.assign(names.size(),0.);point.accelerations_rad_s2.assign(names.size(),0.);
      c::TimedTrajectory trajectory;trajectory.joint_names=names;trajectory.header_frame_id="world";
      trajectory.points.push_back(point);point.time_from_start_s=.5;trajectory.points.push_back(point);
      c::MotionCommand cmd;cmd.primitive_name=primitive.name;cmd.joint_names=names;cmd.trajectory=trajectory;
      cmd.simulation_only=true;cmd.requires_contact=false;cmd.timeout_ms=4500;cmd.intent=primitive;
      log("prealigned_hold",{{"position_error_m",position_error},{"angle_error_rad",angle_error}});
      return cmd;
    }
    allow_handle_contact(primitive.name=="test_lift");
    auto tcp=goal(primitive.name);
    p::Transform ee_to_tcp{{.045,0,.1105},{0,std::sqrt(.5),0,std::sqrt(.5)}};
    const double yaw=cfg.at("tool_mount_yaw_deg").get<double>()*M_PI/180.0;
    ee_to_tcp=p::compose(p::Transform{{0,0,0},{0,0,std::sin(yaw/2),std::cos(yaw/2)}},ee_to_tcp);
    auto ee=p::compose(p::Transform{tcp.position,tcp.orientation},p::inverse(ee_to_tcp));
    p::BackendRequest req;req.label=primitive.name;req.start=start;req.ee_goal=pose(ee.translation.x,ee.translation.y,ee.translation.z,ee.rotation);
    req.mode=(primitive.name=="side_insert")?p::MotionMode::FREE_SPACE:p::MotionMode::CARTESIAN;
    if(req.mode==p::MotionMode::FREE_SPACE){
      req.goal_joint_bounds["shoulder_joint"]={-M_PI,M_PI};
      req.goal_joint_bounds["foreArm_joint"]=direction<0?std::pair<double,double>{0,2.41}:std::pair<double,double>{-2.41,0};
    }
    req.prefer_straight_line=true;req.velocity_scaling=.15;req.acceleration_scaling=.10;
    req.max_linear_speed_m_s=std::min(primitive.max_linear_speed,.05);
    req.max_angular_speed_rad_s=std::min(primitive.max_angular_speed,.10);
    req.cartesian_step_m=.002;req.planning_time_s=2.0;
    log("planning_goal",{{"primitive",primitive.name},{"xyz",{ee.translation.x,ee.translation.y,ee.translation.z}},
      {"quat",{ee.rotation.x,ee.rotation.y,ee.rotation.z,ee.rotation.w}},{"frame",backend->planning_frame()}});
    if(payload){
      p::AttachedPayload load;load.id="mtc_moveit_planner/held_battery";load.size_m={.07,.08,.0795};
      // Conservative full battery envelope, including its handle. The 0.5 mm
      // inset at the base is the planner's surface contact tolerance.
      auto body_center=tcp;body_center.position.z-=.02525;
      body_center.orientation=p::multiply(tcp.orientation,tcp_to_body);
      load.ee_to_payload=p::compose(p::inverse(p::Transform{req.ee_goal.position,req.ee_goal.orientation}),p::Transform{body_center.position,body_center.orientation});
      load.touch_links={"v1_hook","tcp","ee_link","wrist3_Link"};req.payload=load;
    }
    const auto final_ee=req.ee_goal;
    const bool overhead=primitive.name=="side_insert"&&cfg.contains("approach_overhead_tcp_z");
    if(keep_orientation&&primitive.name=="side_insert"){
      // Keep the measured orientation through vertical clearance, horizontal
      // transit and descent. A free-space fallback could rotate the tool even
      // with equal endpoint orientations, so none is permitted in this mode.
      const auto current=backend->end_effector_pose(start);
      if(!current)throw std::runtime_error("no FK for fixed-orientation approach");
      const auto a=p::normalized(current->orientation),b=p::normalized(final_ee.orientation);
      const double dot=std::fabs(a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w);
      if(2*std::acos(std::min(1.,dot))>.001)
        throw std::runtime_error("start differs from the fixed tool orientation");
      auto high=final_ee;high.position.z+=cfg.at("approach_overhead_tcp_z").get<double>()-tcp.position.z;
      auto above_current=*current;above_current.orientation=final_ee.orientation;above_current.position.z=high.position.z;
      c::TimedTrajectory joined;joined.joint_names=names;joined.header_frame_id="world";
      auto previous=*current;req.mode=p::MotionMode::CARTESIAN;req.goal_joint_bounds.clear();
      for(const auto & step:std::vector<std::pair<std::string,c::Pose6D>>{
          {"prealign_vertical_clearance",above_current},{"prealign_horizontal",high},{"prealign_vertical_descent",final_ee}}){
        if(norm(sub(step.second.position,previous.position))<1e-6)continue;
        req.ee_goal=step.second;
        const auto planned_step=backend->plan(req);
        log("plan",{{"primitive",step.first},{"ok",planned_step.ok},{"reason",planned_step.reason},
                    {"detail",planned_step.message},{"fraction",planned_step.cartesian_fraction},{"fixed_orientation",true}});
        if(!planned_step.ok)throw std::runtime_error(step.first+": "+planned_step.message);
        auto converted_step=p::to_timed_trajectory(planned_step.trajectory,names,"world",0.0);
        if(!converted_step.ok)throw std::runtime_error(converted_step.error);
        const double offset=joined.points.empty()?0.:joined.duration_s();
        for(size_t i=joined.points.empty()?0:1;i<converted_step.trajectory.points.size();++i){
          auto point=converted_step.trajectory.points[i];point.time_from_start_s+=offset;joined.points.push_back(point);
        }
        req.start.names=names;req.start.positions=p::final_positions(joined);previous=step.second;
      }
      if(!joined.shape_is_valid())throw std::runtime_error("invalid fixed-orientation approach");
      c::MotionCommand cmd;cmd.primitive_name=primitive.name;cmd.joint_names=names;cmd.trajectory=joined;
      cmd.simulation_only=true;cmd.requires_contact=false;cmd.velocity_scaling=.15;cmd.acceleration_scaling=.10;
      cmd.timeout_ms=static_cast<uint32_t>((joined.duration_s()+4)*1000);cmd.intent=primitive;return cmd;
    }
    if(overhead)req.ee_goal.position.z+=cfg.at("approach_overhead_tcp_z").get<double>()-tcp.position.z;
    auto result=backend->plan(req);
    log("plan",{{"primitive",primitive.name},{"ok",result.ok},{"reason",result.reason},{"detail",result.message},{"fraction",result.cartesian_fraction},{"method",result.method}});
    if(!result.ok)throw std::runtime_error(primitive.name+": "+result.reason+" "+result.message);
    auto converted=p::to_timed_trajectory(result.trajectory,names,"world",0.0);
    if(!converted.ok)throw std::runtime_error(converted.error);
    if(overhead){
      req.start.names=names;req.start.positions=p::final_positions(converted.trajectory);
      req.ee_goal=final_ee;req.mode=p::MotionMode::CARTESIAN;req.goal_joint_bounds.clear();
      auto descent=backend->plan(req);
      log("plan",{{"primitive","prealign_vertical_descent"},{"ok",descent.ok},{"reason",descent.reason},{"detail",descent.message},{"fraction",descent.cartesian_fraction}});
      if(!descent.ok)throw std::runtime_error("prealign descent: "+descent.message);
      auto next=p::to_timed_trajectory(descent.trajectory,names,"world",0.0);
      if(!next.ok)throw std::runtime_error(next.error);
      const double offset=converted.trajectory.duration_s();
      for(size_t i=1;i<next.trajectory.points.size();++i){auto point=next.trajectory.points[i];point.time_from_start_s+=offset;converted.trajectory.points.push_back(point);}
    }
    c::MotionCommand cmd;cmd.primitive_name=primitive.name;cmd.joint_names=names;cmd.trajectory=converted.trajectory;
    cmd.simulation_only=true;cmd.requires_contact=primitive.requires_contact;cmd.velocity_scaling=.15;cmd.acceleration_scaling=.10;
    cmd.timeout_ms=static_cast<uint32_t>((cmd.trajectory->duration_s()+4)*1000);cmd.intent=primitive;
    return cmd;
  }
  c::MotionResult execute(c::MotionCommand cmd){
    c::MotionResult result;
    try{
      if(!cmd.simulation_only||!cmd.trajectory||!cmd.trajectory->shape_is_valid())throw std::runtime_error("invalid simulation trajectory");
      guard();auto current=backend->current_state();if(!current)throw std::runtime_error("no measured start");
      std::map<std::string,double> positions;
      for(size_t i=0;i<current->names.size();++i)positions[current->names[i]]=current->positions[i];
      double error=0;for(size_t i=0;i<names.size();++i)error=std::max(error,std::abs(positions.at(names[i])-cmd.trajectory->points.front().positions_rad[i]));
      if(error>.01){
        if(!replay_commands.empty())throw std::runtime_error("recorded trajectory start mismatch; replay never replans");
        if(!cmd.intent)throw std::runtime_error("start mismatch without intent");
        log("replan_from_measured_start",{{"max_error_rad",error},{"primitive",cmd.primitive_name}});
        cmd=planned(*cmd.intent,*current,holding && cmd.primitive_name!="retreat");
      }
      Fjt::Goal g;g.trajectory.joint_names=names; // stamp=0 explicitly means immediate ROS execution.
      for(const auto & v:cmd.trajectory->points){trajectory_msgs::msg::JointTrajectoryPoint x;x.positions=v.positions_rad;x.velocities=v.velocities_rad_s;
        x.accelerations=v.accelerations_rad_s2;x.time_from_start=rclcpp::Duration::from_seconds(v.time_from_start_s);g.trajectory.points.push_back(x);}
      Json saved={{"simulation_only",true},{"object",object},{"primitive",cmd.primitive_name},{"frame","world"},
                  {"stamp_semantics","ROS immediate (0)"},{"joint_names",names},{"points",Json::array()}};
      for(const auto & v:cmd.trajectory->points)saved["points"].push_back({{"t",v.time_from_start_s},{"positions",v.positions_rad},
        {"velocities",v.velocities_rad_s},{"accelerations",v.accelerations_rad_s2}});
      commands<<saved.dump()<<std::endl;
      auto sent=client->async_send_goal(g);
      if(sent.wait_for(3s)!=std::future_status::ready)throw std::runtime_error("goal acceptance unknown");
      auto handle=sent.get();if(!handle)throw std::runtime_error("controller rejected trajectory");set_active(handle);
      auto done=client->async_get_result(handle);double deadline=wall()+cmd.trajectory->duration_s()*5+10;
      log("execute",{{"primitive",cmd.primitive_name},{"duration_s",cmd.trajectory->duration_s()}});
      while(done.wait_for(20ms)!=std::future_status::ready){
        guard();auto observed_load=observed();
        if(holding && (cmd.primitive_name=="lift"||cmd.primitive_name=="transport"||cmd.primitive_name=="pre_seat")){
          const auto delta=sub(observed_load.tcp.position,observed_load.body.position);
          if(std::hypot(delta.x,delta.y)>.008||std::abs(delta.z-.0655)>.007)
            throw std::runtime_error("load no longer follows passive hook; cancelling trajectory");
        }
        if(wall()>deadline)throw std::runtime_error("trajectory wall timeout");
      }
      auto outcome=done.get();set_active(nullptr);
      if(outcome.code!=rclcpp_action::ResultCode::SUCCEEDED||!outcome.result||outcome.result->error_code!=0)
        throw std::runtime_error("trajectory action failed: "+(outcome.result?outcome.result->error_string:"no result"));
      double stable=-1,end=wall()+3;
      while(wall()<end){
        guard();bool good=true;
        {std::lock_guard<std::mutex> lock(mutex);
          for(size_t i=0;i<names.size();++i)good=good&&std::abs(joints.at(names[i])-g.trajectory.points.back().positions[i])<.006&&std::abs(velocities.at(names[i]))<.02;
        }
        if(good){if(stable<0)stable=wall();if(wall()-stable>.20){result.success=true;result.arrived=true;result.exec_state=c::ExecState::SETTLED;break;}}
        else {stable=-1;}
        std::this_thread::sleep_for(20ms);
      }
      if(!result.success)throw std::runtime_error("result received without measured stationary arrival");
      log("arrived",{{"primitive",cmd.primitive_name}});
    }catch(const std::exception & e){
      cancel_active();
      result.error_code=c::ErrorCodes::MOTION_STATUS_UNKNOWN;result.status_unknown=true;result.message=e.what();
      log("motion_failed",{{"detail",e.what()}});
    }
    return result;
  }
  c::MotionResult local(const std::string & name,double speed,bool contact_required){
    auto current=backend->current_state();if(!current)throw std::runtime_error("no measured start");
    c::MotionPrimitive primitive;primitive.name=name;primitive.max_linear_speed=speed;primitive.max_angular_speed=.05;primitive.requires_contact=contact_required;
    return execute(planned(primitive,*current,holding && name!="side_withdraw"));
  }
};

class Perception:public m::PerceptionPort {
  Session & s;
public:
  explicit Perception(Session & session):s(session){}
  std::optional<c::DetectionResult> get_detection(const std::string & id,std::optional<int> color,double max_age)override{
    c::DetectionResult r;
    try{auto o=s.observed(id);c::PoseEstimate p;p.object_id=id;p.color=id.back()-'A'+1;
      if(color&&*color!=p.color){r.message="color mismatch";return r;}
      p.body_pose=o.body;p.has_body_pose_value=true;p.ring_pose=s.ring;p.ring_pose.stamp_s=o.body.stamp_s;
      p.has_ring_pose_value=true;p.stamp_s=o.body.stamp_s;p.confidence=1;
      double xy=std::hypot(o.body.position.x-s.target.position.x,o.body.position.y-s.target.position.y);
      auto up=mtc::moveit_planner::rotate(o.body.orientation,{0,0,1});
      p.at_target=xy<.015&&std::abs(o.body.position.z-s.target_surface_z)<.002&&up.z>.995;
      p.settled=o.speed<.003&&o.angular_speed<.04;
      p.followed=o.body.position.z-s.initial_z>.012;
      r.found=true;r.pose=p;r.stale=s.clock->now()-p.stamp_s>max_age;
    }catch(const std::exception & e){r.stale=true;r.message=e.what();}
    return r;
  }
};

class Planner:public m::PlannerPort {
  Session & s;std::future<c::PlanResult> future;
public:
  explicit Planner(Session & session):s(session){}
  std::optional<c::PlanResult> plan(const m::PlanTarget & target)override{
    if(!future.valid())future=std::async(std::launch::async,[this,target]{
      c::PlanResult out;
      try{
        const bool transport=target.phase=="TRANSPORT";s.scene(transport);
        auto state=s.backend->current_state();if(!state)throw std::runtime_error("no joint state");
        for(const auto & primitive:target.primitives){
          if(primitive.name=="test_lift"||primitive.name=="retreat"){
            c::MotionPrimitive local;local.name=primitive.name=="test_lift"?"hook_engage":"side_withdraw";
            auto preceding=s.planned(local,*state,false);
            state->names=names;state->positions=p::final_positions(*preceding.trajectory);
          }
          auto cmd=s.planned(primitive,*state,transport&&primitive.name!="retreat");
          state->names=names;state->positions=p::final_positions(*cmd.trajectory);
          out.commands.push_back(cmd);
        }
        out.ok=true;out.primitives=target.primitives;
      }catch(const std::exception & e){out.error_code=c::ErrorCodes::PLANNING_FAILED;out.message=e.what();}
      return out;
    });
    if(future.wait_for(0s)!=std::future_status::ready)return std::nullopt;
    return future.get();
  }
};
class Motion:public m::MotionPort {
  Session & s;std::future<c::MotionResult> future;
public:
  explicit Motion(Session & session):s(session){}
  std::optional<c::MotionResult> execute_joint_move(const c::MotionCommand &)override{c::MotionResult r;r.error_code=c::ErrorCodes::EXECUTION_REJECTED;return r;}
  std::optional<c::MotionResult> execute_trajectory(const c::MotionCommand & cmd)override{
    if(!future.valid())future=std::async(std::launch::async,[this,cmd]{return s.execute(cmd);});
    if(future.wait_for(0s)!=std::future_status::ready)return std::nullopt;
    return future.get();
  }
  c::StopResult request_stop(const std::string &)override{
    c::StopResult r;r.request_delivered=s.cancel_active();
    r.message="cancel requested; no independent stop proof yet";return r;
  }
};
class Tool:public m::ToolPort {
  Session & s;std::future<c::ToolEvidence> future;
  c::ToolEvidence evidence(bool ok,const std::string & message){c::ToolEvidence e;e.ok=ok;e.verified=ok;
    e.evidence_level=ok?c::EvidenceLevel::GEOMETRY:c::EvidenceLevel::NONE;e.state=c::ToolStateCode::STATE_ATTACHED;e.message=message;return e;}
  template<class F>std::optional<c::ToolEvidence> run(F f){
    if(!future.valid())future=std::async(std::launch::async,[this,f]{try{return f();}catch(const std::exception & e){return evidence(false,e.what());}});
    if(future.wait_for(0s)!=std::future_status::ready)return std::nullopt;
    return future.get();
  }
public:
  explicit Tool(Session & session):s(session){}
  std::optional<c::ToolEvidence> engage(const std::string &,const c::MotionPrimitiveList &)override{
    return run([this]{auto motion=s.local("hook_engage",.01,true);auto o=s.observed();
      bool fit=norm(sub(o.tcp.position,s.ring.position))<.002;
      s.log("engagement_geometry",{{"tcp_error_m",norm(sub(o.tcp.position,s.ring.position))}});
      return evidence(motion.ok()&&fit,"measured tongue insertion; attachment requires test lift");});
  }
  std::optional<c::ToolEvidence> test_lift(const c::MotionPrimitiveList &)override{
    return run([this]{
      double start=s.clock->now(),required=s.cfg.value("lift_stability_s",0.0),deadline=wall()+10;
      while(true){
        s.guard();auto o=s.observed();double lift=o.body.position.z-s.initial_z;
        double xy=std::hypot(o.body.position.x-o.tcp.position.x,o.body.position.y-o.tcp.position.y);
        bool ok=lift>.012&&xy<.006&&s.contact("tool_contact")&&!s.contact("support");
        if(!ok||wall()>deadline){s.holding=false;return evidence(false,"trial lift stability failed");}
        if(s.clock->now()-start>=required){
          s.holding=true;s.log("test_lift_evidence",{{"lift_m",lift},{"relative_xy_m",xy},{"tool_contact",true},{"ok",true},{"stable_s",s.clock->now()-start}});
          return evidence(true,"post-lift measured object rise and stable tool contact, no support contact");
        }
        std::this_thread::sleep_for(20ms);
      }
    });
  }
  c::ToolEvidence seat_verify()override{
    auto o=s.observed();bool ok=s.contact("support")&&std::abs(o.body.position.z-s.target_surface_z)<.002&&o.speed<.005;
    auto e=evidence(ok,"physics table contact + body height + low velocity");e.seated=ok;
    if(ok){s.holding=false;s.scene(false);} // The stationary object must not follow the hook during unloading.
    s.log("seat_evidence",{{"ok",ok},{"z",o.body.position.z}});return e;
  }
  std::optional<c::ToolEvidence> unload(const c::MotionPrimitiveList &)override{
    return run([this]{auto motion=s.local("unload",.005,true);auto o=s.observed();
      bool ok=motion.ok()&&s.contact("support")&&!s.contact("tool_contact")&&o.speed<.005;
      auto e=evidence(ok,"table supports battery and hook contact cleared");e.unloaded=ok;e.seated=ok;return e;});
  }
  std::optional<c::ToolEvidence> unlock(uint32_t)override{return evidence(false,"V1 has no unlock IO");}
  std::optional<c::ToolEvidence> disengage(const c::MotionPrimitiveList &)override{
    return run([this]{if(!s.contact("support")||s.contact("tool_contact"))return evidence(false,"withdrawal interlock failed");
      s.holding=false;s.scene(false);auto motion=s.local("side_withdraw",.01,true);auto o=s.observed();
      bool ok=motion.ok()&&s.contact("support")&&!s.contact("tool_contact")&&norm(sub(o.tcp.position,s.target.position))>.055;
      auto e=evidence(ok,"measured lateral separation with continued table support");e.released=ok;e.state=c::ToolStateCode::STATE_DETACHED;return e;});
  }
  std::string holding_state()override{return s.holding?"holding":"unknown";}
};

int main(int argc,char ** argv){
  rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("v1_task",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;executor.add_node(node);std::thread spin([&]{executor.spin();});
  int rc=1;
  try{
    Session s(node);s.initialize();std::string order=node->get_parameter("order").as_string();
    bool success=true;size_t index=0;
    for(char id:order){
      s.select(id,s.cfg["task"]=="basic"?"T0":"P"+std::to_string(++index));
      auto perception=std::make_shared<Perception>(s);auto planner=std::make_shared<Planner>(s);
      auto motion=std::make_shared<Motion>(s);auto tool=std::make_shared<Tool>(s);
      m::PickPlaceConfig config;config.recovery_max_attempts=0;
      // Existing FSM calls test_lift before motion when true. False obtains its
      // mandatory evidence in VERIFY_ATTACHED, after physical motion completes.
      config.verify_test_lift=false;config.state_timeouts=m::default_state_timeouts();
      config.state_timeouts[m::PickPlaceState::PLAN_ACQUIRE]=30; // Full scene preflight, simulation only.
      config.state_timeouts[m::PickPlaceState::MOVE_PRE_ALIGN]=180; // Long CAD-tool reorientation at original speed limits.
      m::PickPlaceFsm fsm(perception,planner,motion,tool,s.clock,std::make_shared<m::PassiveHookV1Strategy>(),config);
      fsm.on_event=[&](const std::string & event,m::PickPlaceState state){s.log(event,{{"state",m::to_string(state)}});};
      fsm.start(s.object,id-'A'+1,s.slot,"gazebo-v1-"+s.object);
      double end=wall()+900;
      while(!m::terminal_states().count(fsm.state())){
        if(wall()>end||!rclcpp::ok()){fsm.force_fault(c::ErrorCodes::MOTION_TIMEOUT,"simulation task wall deadline",false);break;}
        try{fsm.step();}catch(const std::exception & e){fsm.force_fault(c::ErrorCodes::MOTION_STATUS_UNKNOWN,e.what(),false);}
        std::this_thread::sleep_for(20ms);
      }
      auto r=fsm.result();Json row={{"object",s.object},{"slot",s.slot},{"success",r.success},{"placement_verified",r.placement_verified},
        {"error_code",r.error_code},{"message",r.message},{"stability_s",fsm.stability_verified_for_s()},{"states",Json::array()}};
      for(auto state:r.states_history)row["states"].push_back(m::to_string(state));
      s.report["objects"].push_back(row);s.log("object_result",row);
      if(!r.success){success=false;motion->request_stop("FSM failed");break;}
    }
    if(success&&s.cfg["task"]=="sequence"){
      // Retain a fresh independent sample after the last completion event.
      // Otherwise process exit may truncate the trace a few ms before SUCCESS.
      const double final_observation_time=s.clock->now()+.1;
      const double deadline=wall()+s.feedback_wall_timeout;bool received=false;
      while(wall()<deadline){
        s.guard();
        if(s.observed().body.stamp_s>=final_observation_time){received=true;break;}
        std::this_thread::sleep_for(20ms);
      }
      if(!received)throw std::runtime_error("no fresh physics sample after sequence completion");
      s.log("final_observation",{{"observation_stamp_s",s.observed().body.stamp_s}});
    }
    s.report["success"]=success;s.report["order"]=order;s.report["task"]=s.cfg["task"];
    {std::lock_guard<std::mutex> lock(s.mutex);s.report["contacts"]=s.contacts;}
    std::ofstream(s.directory+"/result.json")<<s.report.dump(2)<<'\n';rc=success?0:1;
  }catch(const std::exception & e){RCLCPP_ERROR(node->get_logger(),"%s",e.what());}
  executor.cancel();spin.join();rclcpp::shutdown();return rc;
}
