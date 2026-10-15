cmake_minimum_required(VERSION 3.8)
project(mtc_task_cpp)

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

# ---------------------------------------------------------------------------
# Task FSM 核心库（第一层状态机）
#
# 刻意不依赖 rclcpp：状态机核心不依赖真实设备即可完整测试（需求 3）。
# ROS 2 Action 外壳（ExecuteTask 的 goal/cancel/result 绑定）属于第二阶段。
# ---------------------------------------------------------------------------
add_library(mtc_task_cpp SHARED
  src/task_types.cpp
  src/task_validation.cpp
  src/task_fsm.cpp
)
target_include_directories(mtc_task_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# mtc_core_cpp 导出的是带命名空间的 imported target，其 INTERFACE_INCLUDE_DIRECTORIES
# 会随 PUBLIC 链接自动传递给本包及其测试。
target_link_libraries(mtc_task_cpp PUBLIC mtc_core_cpp::mtc_core_cpp)
target_compile_features(mtc_task_cpp PUBLIC cxx_std_17)

install(TARGETS mtc_task_cpp
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

  ament_add_gtest(test_task_validation test/test_task_validation.cpp)
  target_link_libraries(test_task_validation mtc_task_cpp)

  ament_add_gtest(test_task_fsm_core test/test_task_fsm_core.cpp)
  target_link_libraries(test_task_fsm_core mtc_task_cpp)

  ament_add_gtest(test_task_fsm_safety test/test_task_fsm_safety.cpp)
  target_link_libraries(test_task_fsm_safety mtc_task_cpp)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(mtc_task_cpp)
ament_package()
