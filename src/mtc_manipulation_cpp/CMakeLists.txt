cmake_minimum_required(VERSION 3.8)
project(mtc_manipulation_cpp)

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

# ---------------------------------------------------------------------------
# PickPlace FSM 核心库（第二层状态机）+ V1/V2 工具策略 + 运动适配桥接
#
# 刻意不依赖 rclcpp：状态机核心不依赖真实设备即可完整测试（需求 3）。
# ROS 2 Action 外壳（PickPlace 的 goal/cancel/result 绑定）属于第二阶段。
# ---------------------------------------------------------------------------
add_library(mtc_manipulation_cpp SHARED
  src/pick_place_types.cpp
  src/tool_strategy.cpp
  src/pick_place_fsm.cpp
  src/motion_adapter_port.cpp
)
target_include_directories(mtc_manipulation_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mtc_manipulation_cpp PUBLIC
  mtc_core_cpp::mtc_core_cpp
  mtc_motion_execution_cpp::mtc_motion_execution_cpp
)
target_compile_features(mtc_manipulation_cpp PUBLIC cxx_std_17)

install(TARGETS mtc_manipulation_cpp
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

  ament_add_gtest(test_pick_place_tables test/test_pick_place_tables.cpp)
  target_link_libraries(test_pick_place_tables mtc_manipulation_cpp)

  ament_add_gtest(test_tool_strategy test/test_tool_strategy.cpp)
  target_link_libraries(test_tool_strategy mtc_manipulation_cpp)

  ament_add_gtest(test_pick_place_fsm_v1 test/test_pick_place_fsm_v1.cpp)
  target_link_libraries(test_pick_place_fsm_v1 mtc_manipulation_cpp)

  ament_add_gtest(test_pick_place_fsm_v2 test/test_pick_place_fsm_v2.cpp)
  target_link_libraries(test_pick_place_fsm_v2 mtc_manipulation_cpp)

  ament_add_gtest(test_pick_place_safety test/test_pick_place_safety.cpp)
  target_link_libraries(test_pick_place_safety mtc_manipulation_cpp)

  ament_add_gtest(test_pick_place_audit test/test_pick_place_audit.cpp)
  target_link_libraries(test_pick_place_audit mtc_manipulation_cpp)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(mtc_manipulation_cpp)
ament_export_dependencies(mtc_core_cpp mtc_motion_execution_cpp)
ament_package()
