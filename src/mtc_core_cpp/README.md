# mtc_core_cpp

AUBO S3 竞赛任务的 **C++17 状态机共享核心类型**。

## 定位

本包是 C++ 迁移第一阶段新增的**最底层包**，只承载跨包共享的类型与常量：

| 头文件 | 内容 |
|---|---|
| `error_codes.hpp` | 18 个错误码常量（对齐 `mtc_interfaces/msg/ErrorCodes.msg`） |
| `tool_state.hpp` | `ToolState.msg` 常量镜像 + 有序证据等级 `EvidenceLevel` |
| `pose.hpp` | **完整 6D 位姿** `Pose6D`（位置 + 四元数 + `frame_id` + 时间戳）与 `PoseEstimate` |
| `motion_primitive.hpp` | `MotionPrimitive`（含 `requires_contact` 安全位）/ `MotionCommand` |
| `timed_trajectory.hpp` | 多点时间参数化轨迹 `TimedTrajectory`（无 ROS 依赖） |
| `results.hpp` | 端口结果契约：`DetectionResult` / `PlanResult` / `MotionResult` / `StopResult` / `ToolEvidence` / `MotionCapabilities` |
| `exec_state.hpp` | 执行状态枚举（`ExecuteJointMove.feedback.execution_state` 的取值） |
| `clock.hpp` | 可注入时钟：`Clock` / `SteadyClock` / `ManualClock` |

## 关键设计约束

1. **刻意不依赖 `rclcpp`，也不依赖任何 ROS 消息包** —— 状态机核心可在无 ROS 图、
   无硬件、无真实设备的环境下被 GTest 完整验证。
2. **错误码单一来源** —— Python 侧曾在 5 个模块中各自镜像同一套常量；
   C++ 收敛为唯一定义，并以 `test_error_codes_msg_parity` 直接解析
   `ErrorCodes.msg` 做**双向**一致性校验（不匹配即 FAIL，接口不可读即 SKIP 而非静默通过）。
3. **6D 位姿保真** —— `Pose6D` 不提供"只有 XYZ"的构造路径；
   `is_usable()` 要求 `frame_id` 非空且四元数归一化。
   依据：需求"必须保留完整 6D Pose、frame_id 和 TF 信息，不得退化成只有 XYZ"。
4. **轨迹不得被拆分** —— `TimedTrajectory` 是"一条轨迹"的原子载体；
   `MotionAdapter::execute_trajectory` 的默认实现**显式拒绝**而非降级为多次单点调用。

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash
cd ~/meituan_challenge_ws
colcon build --packages-select mtc_core_cpp
colcon test  --packages-select mtc_core_cpp
colcon test-result --all
```

期望：**21 个用例 PASS，0 失败，0 跳过**。

## 依赖

* 构建：`ament_cmake`
* 测试：`ament_cmake_gtest`
* 运行：**仅 C++17 标准库**

## 相关文档

* [`docs/migration/FSM_BEHAVIOR_MAPPING.md`](../../docs/migration/FSM_BEHAVIOR_MAPPING.md)
* [`docs/migration/CPP_MIGRATION_TEST_REPORT.md`](../../docs/migration/CPP_MIGRATION_TEST_REPORT.md)
* [`docs/migration/PHASE2_ROS2_INTEGRATION_PLAN.md`](../../docs/migration/PHASE2_ROS2_INTEGRATION_PLAN.md)
