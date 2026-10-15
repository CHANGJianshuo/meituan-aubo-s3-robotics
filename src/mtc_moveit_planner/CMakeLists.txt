cmake_minimum_required(VERSION 3.8)
project(mtc_moveit_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mtc_core_cpp REQUIRED)
find_package(mtc_motion_execution_cpp REQUIRED)
find_package(mtc_manipulation_cpp REQUIRED)
find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# 核心层：PlannerPort 实现 + 轨迹转换 + 配置（不依赖 MoveIt / rclcpp）
# ---------------------------------------------------------------------------
add_library(mtc_moveit_planner_core SHARED
  src/trajectory_conversion.cpp
  src/planner_config.cpp
  src/moveit_planner_port.cpp
  src/start_consistency.cpp
  src/scene_ids.cpp
  src/ee_limited_retiming.cpp
  src/tool_model.cpp
)
target_include_directories(mtc_moveit_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mtc_moveit_planner_core PUBLIC
  mtc_core_cpp::mtc_core_cpp
  mtc_manipulation_cpp::mtc_manipulation_cpp
  Threads::Threads
)
target_compile_features(mtc_moveit_planner_core PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# ROS 外壳：MoveGroupInterface + tf2 后端与命令行工具
# ---------------------------------------------------------------------------
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(shape_msgs REQUIRED)

add_library(mtc_moveit_planner_moveit SHARED
  src/move_group_backend.cpp
  src/plan_report_io.cpp
  src/ros_params.cpp
)
target_include_directories(mtc_moveit_planner_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mtc_moveit_planner_moveit PUBLIC mtc_moveit_planner_core)
ament_target_dependencies(mtc_moveit_planner_moveit PUBLIC
  rclcpp geometry_msgs trajectory_msgs moveit_msgs moveit_core moveit_ros_planning
  moveit_ros_planning_interface tf2 tf2_ros tf2_geometry_msgs geometric_shapes shape_msgs)

add_executable(plan_fixed_pose src/plan_fixed_pose_main.cpp)
target_link_libraries(plan_fixed_pose mtc_moveit_planner_moveit)

add_executable(plan_battery_sequence src/plan_battery_sequence_main.cpp)
target_link_libraries(plan_battery_sequence mtc_moveit_planner_moveit)

install(TARGETS mtc_moveit_planner_core mtc_moveit_planner_moveit
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS plan_fixed_pose plan_battery_sequence
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# 测试（只链接核心层，不需要 move_group 运行）
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_shutdown_evidence test/test_shutdown_evidence.py)

  ament_add_gtest(test_pose_math test/test_pose_math.cpp)
  target_link_libraries(test_pose_math mtc_moveit_planner_core)

  ament_add_gtest(test_trajectory_conversion test/test_trajectory_conversion.cpp)
  target_link_libraries(test_trajectory_conversion mtc_moveit_planner_core)

  ament_add_gtest(test_start_consistency test/test_start_consistency.cpp)
  target_link_libraries(test_start_consistency mtc_moveit_planner_core)

  ament_add_gtest(test_scene_ids test/test_scene_ids.cpp)
  target_link_libraries(test_scene_ids mtc_moveit_planner_core)

  ament_add_gtest(test_ee_limited_retiming test/test_ee_limited_retiming.cpp)
  target_link_libraries(test_ee_limited_retiming mtc_moveit_planner_core)

  ament_add_gtest(test_tool_model test/test_tool_model.cpp)
  target_link_libraries(test_tool_model mtc_moveit_planner_core)

  ament_add_gtest(test_moveit_planner_port test/test_moveit_planner_port.cpp)
  target_link_libraries(test_moveit_planner_port mtc_moveit_planner_core)

  # 用已合入的真实 PickPlaceFsm 校验本包输出能通过 accept_plan 并按整条轨迹派发。
  # fakes.hpp 位于 mtc_manipulation_cpp/test（非安装 API），仅在同仓库测试中引用。
  ament_add_gtest(test_fsm_accepts_planner_output test/test_fsm_accepts_planner_output.cpp)
  target_include_directories(test_fsm_accepts_planner_output PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../mtc_manipulation_cpp/test)
  target_link_libraries(test_fsm_accepts_planner_output mtc_moveit_planner_core)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(mtc_moveit_planner_core mtc_moveit_planner_moveit)
ament_export_dependencies(mtc_core_cpp mtc_manipulation_cpp mtc_motion_execution_cpp
  rclcpp geometry_msgs trajectory_msgs moveit_msgs moveit_core moveit_ros_planning
  moveit_ros_planning_interface tf2 tf2_ros tf2_geometry_msgs geometric_shapes shape_msgs)
ament_package()
