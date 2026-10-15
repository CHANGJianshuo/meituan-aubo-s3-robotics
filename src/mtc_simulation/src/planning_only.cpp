// Same no-controller contract as phase2c PlanningOnlyManager; isolated simulation plugin.
#include <moveit/controller_manager/controller_manager.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace mtc::simulation {
class PlanningOnly final : public moveit_controller_manager::MoveItControllerManager {
public:
  void initialize(const rclcpp::Node::SharedPtr &) override {}
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string &) override {return nullptr;}
  void getControllersList(std::vector<std::string> & v) override {v.clear();}
  void getActiveControllers(std::vector<std::string> & v) override {v.clear();}
  void getControllerJoints(const std::string &, std::vector<std::string> & v) override {v.clear();}
  ControllerState getControllerState(const std::string &) override {return {};}
  bool switchControllers(const std::vector<std::string> &, const std::vector<std::string> &) override {return false;}
};
}
PLUGINLIB_EXPORT_CLASS(mtc::simulation::PlanningOnly, moveit_controller_manager::MoveItControllerManager)
