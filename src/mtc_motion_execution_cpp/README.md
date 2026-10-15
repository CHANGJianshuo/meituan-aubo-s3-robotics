# mtc_motion_execution_cpp

运动执行适配层的 **C++17 迁移实现**。

对既有 Python 参考实现
`src/mtc_motion_execution/mtc_motion_execution/{backends,executor_core}.py`。

## 交付内容

| 组件 | 文件 | 说明 |
|---|---|---|
| **Motion Adapter 接口** | `include/.../motion_adapter.hpp` | 第一阶段新增的抽象接口；含单点关节目标与**时间参数化轨迹**两条能力位 |
| `InFlightGuard` | `include/.../in_flight_guard.hpp` | **唯一执行权**：同一时刻只允许一个未结束命令 |
| `ValidationConfig` / `validate_joint_move` | `include/.../validation.hpp` | 执行前能力与目标校验 |
| `MotionExecutorCore` | `include/.../executor_core.hpp` | 七步握手：`PREPARE → ACCEPT → VALIDATE → SEND → MONITOR → CONFIRM → STOP/FAULT` |
| `MockMotionAdapter` | `include/.../mock_motion_adapter.hpp` | 离线 Mock；行为可配置（到位/超时/未知/取消确认/不确认/轨迹拒绝） |
| `DisabledMotionAdapter` | 同上 | 真实后端占位，**始终拒绝** |
| `make_adapter` | 同上 | 工厂；**未知名称一律回退 disabled**，绝不默认连真实设备 |

## 关键安全语义

1. `MotionResult::ok()` 要求 **`success && arrived`** —— 发令成功不等于到位。
2. `StopResult` 严格分离 **`request_delivered`（送达）** 与
   **`stop_confirmed`（独立确认停稳）**；后端未声明 `cancel_supported` 时
   **绝不允许伪造** `stop_confirmed`。
3. 状态未知（`status_unknown`）时置位未知标志，**锁住后续所有运动请求**；
   解除锁定只能通过 `clear_unknown_after_manual_confirmation()`（**给人用的显式出口，
   自动流程不得调用**）。
4. `MotionResult::retry_allowed_without_reobservation()` 只在
   **明确被拒绝且从未开始运动**时为真 —— 这是"状态未知禁止自动重发"的正面出口。
5. `MotionAdapter::execute_trajectory()` 的**默认实现显式拒绝**，
   绝不允许把一条多点轨迹拆成多次 `execute_joint_move`。

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash
cd ~/meituan_challenge_ws
colcon build --packages-select mtc_core_cpp mtc_motion_execution_cpp
colcon test  --packages-select mtc_motion_execution_cpp
colcon test-result --all
```

期望：**55 个用例 PASS，0 失败，0 跳过**。

| 可执行文件 | 用例数 | 覆盖 |
|---|---|---|
| `test_in_flight_guard` | 6 | 并发运动请求拒绝、所有权释放 |
| `test_validation` | 18 | 能力/维度/顺序/缩放/限位校验 + 轨迹守门 |
| `test_mock_adapter` | 17 | Mock 行为可控性、轨迹拒绝且不拆分、工厂回退 |
| `test_executor_core` | 14 | 七步握手、并发拒绝、状态未知锁定、停止语义 |

## 依赖

* 构建：`ament_cmake`、`mtc_core_cpp`
* 测试：`ament_cmake_gtest`
* **不依赖 `rclcpp`** —— 标准 ROS 2 Action 适配属于第二阶段

## 已知缺口（第一阶段**不实现**）

* 多点时间参数化轨迹目前**只有类型与拒绝语义**，没有可用的 ROS 通路；
  `TimedTrajectory` 是第二阶段的接口载体。
* 设计提案见
  [`docs/migration/PHASE2_ROS2_INTEGRATION_PLAN.md`](../../docs/migration/PHASE2_ROS2_INTEGRATION_PLAN.md) §3。
