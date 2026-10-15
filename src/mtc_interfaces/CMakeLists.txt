cmake_minimum_required(VERSION 3.16)
project(mtc_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# 接口清单
#
# 新增接口来源：docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md
# 既有接口（PlanMotion.action / Latch.srv）：源文件缺失，本包不复制、不重定义，
# 详见 docs/migration/INTERFACE_CHANGELOG.md 的兼容性说明与补齐清单。
# ---------------------------------------------------------------------------
set(msg_files
  "msg/BatteryDetection.msg"
  "msg/BatteryDetectionArray.msg"
  "msg/ToolState.msg"
  "msg/TaskState.msg"
  "msg/RobotState.msg"
  "msg/ErrorCodes.msg"
)

set(srv_files
  "srv/GetMotionCapabilities.srv"
  "srv/TriggerUnlock.srv"
  "srv/RequestStop.srv"
)

set(action_files
  "action/ExecuteTask.action"
  "action/PickPlace.action"
  "action/ExecuteJointMove.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
