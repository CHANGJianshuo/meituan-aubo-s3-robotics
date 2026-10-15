cmake_minimum_required(VERSION 3.8)
project(mtc_core_cpp)

# ---------------------------------------------------------------------------
# C++17（本工作空间统一语言标准）
# ---------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# 核心库：**刻意不依赖 rclcpp / ROS 消息包**
#
# 需求 3：状态机核心不直接调用 AUBO SDK、不直接访问硬件、不依赖真实设备才能测试。
# 本包因此只依赖 C++17 标准库；GTest 用例可在无 ROS 图、无硬件的情况下运行。
# ---------------------------------------------------------------------------
add_library(mtc_core_cpp SHARED
  src/error_codes.cpp
)
target_include_directories(mtc_core_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# 明确不链接任何 ROS 运行时库
target_compile_features(mtc_core_cpp PUBLIC cxx_std_17)

install(TARGETS mtc_core_cpp
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

  ament_add_gtest(test_core_types test/test_core_types.cpp)
  target_link_libraries(test_core_types mtc_core_cpp)

  ament_add_gtest(test_error_codes_msg_parity test/test_error_codes_msg_parity.cpp)
  target_link_libraries(test_error_codes_msg_parity mtc_core_cpp)
  # 传入 ErrorCodes.msg 所在目录，使测试能对照真实接口定义校验常量数值。
  # 若目录不存在，测试会显式 GTEST_SKIP 而不是静默通过。
  target_compile_definitions(test_error_codes_msg_parity PRIVATE
    MTC_INTERFACES_MSG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../mtc_interfaces/msg"
  )
endif()

# 三者的组合保证下游既拿到 imported target，也拿到 *_INCLUDE_DIRS / *_LIBRARIES：
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(mtc_core_cpp)
ament_package()
