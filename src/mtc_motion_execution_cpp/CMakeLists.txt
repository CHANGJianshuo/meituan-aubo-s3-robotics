cmake_minimum_required(VERSION 3.8)
project(mtc_motion_execution_cpp)

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
find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# 运动适配层：MotionAdapter 接口 + Mock/Disabled 实现 + 七步握手执行核心
#
# 第一阶段交付接口与 Mock 实现；标准 ROS 2 Action 适配（ExecuteJointMove /
# FollowJointTrajectory 的 goal/cancel/result 绑定）属于第二阶段。
# 本包不依赖 rclcpp。
# ---------------------------------------------------------------------------
add_library(mtc_motion_execution_cpp SHARED
  src/in_flight_guard.cpp
  src/validation.cpp
  src/motion_adapter.cpp
  src/executor_core.cpp
  src/mock_motion_adapter.cpp
)
target_include_directories(mtc_motion_execution_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mtc_motion_execution_cpp PUBLIC mtc_core_cpp::mtc_core_cpp Threads::Threads)
target_compile_features(mtc_motion_execution_cpp PUBLIC cxx_std_17)

install(TARGETS mtc_motion_execution_cpp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/
  DESTINATION include
)

# ---------------------------------------------------------------------------
# 测试
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_in_flight_guard test/test_in_flight_guard.cpp)
  target_link_libraries(test_in_flight_guard mtc_motion_execution_cpp)

  ament_add_gtest(test_validation test/test_validation.cpp)
  target_link_libraries(test_validation mtc_motion_execution_cpp)

  ament_add_gtest(test_mock_adapter test/test_mock_adapter.cpp)
  target_link_libraries(test_mock_adapter mtc_motion_execution_cpp)

  ament_add_gtest(test_executor_core test/test_executor_core.cpp)
  target_link_libraries(test_executor_core mtc_motion_execution_cpp)

  ament_add_gtest(test_executor_concurrency test/test_executor_concurrency.cpp TIMEOUT 30)
  target_link_libraries(test_executor_concurrency mtc_motion_execution_cpp Threads::Threads)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(mtc_motion_execution_cpp)
ament_export_dependencies(mtc_core_cpp Threads)
ament_package()
