cmake_minimum_required(VERSION 3.16)
project(mtc_trajectory_execution_cpp)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
foreach(dep action_msgs rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs
    controller_manager_msgs lifecycle_msgs mtc_core_cpp mtc_manipulation_cpp)
  find_package(${dep} REQUIRED)
endforeach()
add_library(${PROJECT_NAME} SHARED src/trajectory_validator.cpp src/fjt_motion_port.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC action_msgs rclcpp rclcpp_action control_msgs
  trajectory_msgs sensor_msgs controller_manager_msgs lifecycle_msgs
  mtc_core_cpp mtc_manipulation_cpp)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
add_executable(fake_trajectory_probe src/fake_trajectory_probe.cpp)
target_link_libraries(fake_trajectory_probe ${PROJECT_NAME})
install(TARGETS fake_trajectory_probe DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectory_validator test/test_trajectory_validator.cpp)
  target_link_libraries(test_trajectory_validator ${PROJECT_NAME})
  ament_add_gtest(test_fjt_action test/test_fjt_action.cpp TIMEOUT 120)
  target_link_libraries(test_fjt_action ${PROJECT_NAME})
endif()
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(action_msgs rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs
  controller_manager_msgs lifecycle_msgs mtc_core_cpp mtc_manipulation_cpp)
ament_package()
