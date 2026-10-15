// Observation only: never sets a pose, adds an attachment, or applies forces.
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/SystemPluginInfo.hh>
#include <gz/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <map>
#include <set>
namespace mtc {
class V1Observer : public gz::sim::System, public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate {
  using Entity = gz::sim::Entity;
  using Json = nlohmann::json;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr poses_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr contacts_;
  std::map<std::string, Entity> links_;
  Json totals_ = Json::object();
  Json window_ = Json::object();
  int64_t last_ = -1;
  bool enabled_ = false;
  bool gazebo_control_plugin_ = false;
  std::string session_id_;
public:
  void Configure(const Entity &, const std::shared_ptr<const sdf::Element> & config,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    if(config->HasElement("session_id"))session_id_=config->Get<std::string>("session_id");
    if (!rclcpp::ok()) {int argc=0; rclcpp::init(argc,nullptr);}
    node_ = std::make_shared<rclcpp::Node>("v1_physics_observer");
    poses_ = node_->create_publisher<tf2_msgs::msg::TFMessage>("/simulation/ground_truth", 10);
    contacts_ = node_->create_publisher<std_msgs::msg::String>("/simulation/contacts", 10);
  }
  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager & ecm) override {
    if (enabled_) return;
    auto robot=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name("mtc_s3"));
    if (robot==gz::sim::kNullEntity) return;
    auto info=ecm.Component<gz::sim::components::SystemPluginInfo>(robot);
    if(info)for(const auto & plugin:info->Data().plugins())
      if(plugin.name()=="gz_ros2_control::GazeboSimROS2ControlPlugin")gazebo_control_plugin_=true;
    if(!gazebo_control_plugin_)return;
    auto tcp=gz::sim::Model(robot).LinkByName(ecm,"tcp");
    if (tcp==gz::sim::kNullEntity) return;
    links_["tcp"]=tcp;
    links_["base_link"]=gz::sim::Model(robot).LinkByName(ecm,"base_link");
    for (char c='A';c<='D';++c) {
      std::string name="battery_"+std::string(1,c);
      auto m=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name(name));
      if(m!=gz::sim::kNullEntity) links_[name]=gz::sim::Model(m).LinkByName(ecm,"body");
    }
    // A reviewed scene may contain only the task's target battery. Require
    // robot base/TCP and at least one battery, not all four legacy props.
    if(links_.size()<3) return;
    std::vector<Entity> collisions;
    ecm.Each<gz::sim::components::Collision>([&](const Entity & e, const auto *){collisions.push_back(e);return true;});
    for(auto e:collisions) if(!ecm.Component<gz::sim::components::ContactSensorData>(e))
      ecm.CreateComponent(e,gz::sim::components::ContactSensorData());
    enabled_=true;
  }
  void PostUpdate(const gz::sim::UpdateInfo & info,const gz::sim::EntityComponentManager & ecm) override {
    if(info.paused || !enabled_) return;
    const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    std::set<std::pair<Entity,Entity>> seen;
    ecm.Each<gz::sim::components::Collision,gz::sim::components::ContactSensorData>(
      [&](const Entity &,const auto *,const auto * data){
        for(const auto & c:data->Data().contact()) {
          Entity a=c.collision1().id(),b=c.collision2().id(); if(a>b) std::swap(a,b);
          if(!seen.insert({a,b}).second)continue;
          std::string x=gz::sim::scopedName(a,ecm,"::",false),y=gz::sim::scopedName(b,ecm,"::",false);
          auto has=[](const std::string & s,const std::string & p){return s.find(p)!=std::string::npos;};
          const bool battery=has(x,"battery_")||has(y,"battery_");
          const bool table=has(x,"::table::")||has(y,"::table::");
          const bool tool=has(x,"v1_")||has(y,"v1_");
          std::string kind=battery&&table?"support":battery&&tool?"tool_contact":table&&(has(x,"base_link")||has(y,"base_link"))?"base_mount":"unexpected";
          if(kind=="tool_contact"&&(has(x,"part_0")||has(y,"part_0")))kind="unexpected";
          double depth=0;for(double d:c.depth())depth=std::max(depth,d);
          std::string key=x+" / "+y;
          if(!totals_.contains(key))totals_[key]={{"kind",kind},{"steps",0},{"max_depth_m",0.0}};
          auto & row=totals_[key];row["steps"]=row["steps"].get<uint64_t>()+1;
          row["max_depth_m"]=std::max(depth,row["max_depth_m"].get<double>());
          row["last_sim_time"]=ns*1e-9;
          window_[key]={{"kind",kind},{"depth_m",depth}};
        }return true;
      });
    if(ns-last_<20000000)return;
    last_=ns;
    tf2_msgs::msg::TFMessage msg;
    for(const auto & [name,entity]:links_) {
      auto p=gz::sim::worldPose(entity,ecm);
      geometry_msgs::msg::TransformStamped t;t.header.stamp=rclcpp::Time(ns);t.header.frame_id="world";t.child_frame_id=name;
      t.transform.translation.x=p.Pos().X();t.transform.translation.y=p.Pos().Y();t.transform.translation.z=p.Pos().Z();
      t.transform.rotation.x=p.Rot().X();t.transform.rotation.y=p.Rot().Y();t.transform.rotation.z=p.Rot().Z();t.transform.rotation.w=p.Rot().W();msg.transforms.push_back(t);
    }
    poses_->publish(msg);
    std_msgs::msg::String msg2;msg2.data=Json({{"sim_time",ns*1e-9},{"session_id",session_id_},
      {"gazebo_control_plugin",gazebo_control_plugin_},{"current",window_},{"total",totals_}}).dump();contacts_->publish(msg2);
    window_=Json::object();
  }
};
}
GZ_ADD_PLUGIN(mtc::V1Observer,gz::sim::System,mtc::V1Observer::ISystemConfigure,
              mtc::V1Observer::ISystemPreUpdate,mtc::V1Observer::ISystemPostUpdate)
