# mtc_motion_execution

规划后**运动执行**与后端抽象层。默认 `backend=mock`；真实 AUBO 后端在获得现场明确授权并完成实测验证前**始终 disabled**。

> 本包不实现运动规划算法。规划结果由 `mtc_motion_planning` 提供。
> 本文档描述的是**离线 Mock 能力**，不代表任何实机运动已通过验证。

## 1. 职责

| 做 | 不做 |
|---|---|
| 受理受控运动请求、能力校验、执行、反馈、取消、停稳确认 | 不根据颜色或任务语义改变行为 |
| 声明后端真实具备的能力（`GetMotionCapabilities`） | 不把规划成功当作执行成功 |
| 区分“已接受 / 运动中 / 已到位 / 取消中 / 已停稳 / 状态未知” | 不把 `JointTrajectory` 拆成高频 `moveJoint` 循环 |
| 单在途命令、并发拒绝、状态未知时锁定 | 不在状态未知时自动重发或自动回 Home |

## 2. 七步握手（对应设计文档 §6.3）

`PREPARE → ACCEPT → VALIDATE → SEND → MONITOR → CONFIRM → STOP/FAULT`

每次执行都会留下可追溯记录（`MotionExecutorCore.handshake(command_id)`），字段与设计文档 §10.2 的日志要求对应。

## 3. 后端与能力

| 后端 | `joint_goal_supported` | `timed_trajectory_supported` | `cartesian_motion_supported` | `cancel_supported` |
|---|---|---|---|---|
| `mock` | ✅ | ❌ | ❌ | ✅ |
| `disabled`（默认兜底） | ❌ | ❌ | ❌ | ❌ |

`make_backend()` 对未知名称（`real`/`aubo`/`gazebo`/空串等）**一律返回 `DisabledMotionBackend`**，不会静默启用任何真实通路。此行为有单元测试覆盖。

**能力声明规则：** capability 只表示“当前已连接且实测启用”。不得因为 SDK 文档声称存在某方法就置 `true`。

## 4. 关键安全语义

1. **到位判定**：失败即失败，超时与反馈断流都标记 `motion_state_unknown=True`，此时 `retry_allowed_without_reobservation` 为 `false`。
2. **取消 ≠ 停稳**：`request_stop()` 返回 `(请求是否送达, 说明, 是否已确认停稳)`。只有后端显式确认标志为真时才算停稳；运动状态未知时**永不**宣告已停稳。
3. **唯一执行权**：在途命令未结束时，第二个请求被拒绝（`REJECTED`，错误码 `210`）。这类拒绝从未开始运动，是唯一允许立即重发的场景。
4. **能力不符即拒绝**：backend 未声明 `joint_goal_supported` 时直接拒绝，不做“变形执行”。
5. **缩放上限**：`velocity_scaling` / `acceleration_scaling` 超出已审核上限（默认 0.2）返回 `SAFETY_INTERLOCK=500`；未验收的高速动作禁止执行。
6. **关节顺序**：`joint_names` 与 `target_rad` 必须按索引一一对应，且与配置的 `expected_joint_names` 一致；否则拒绝。**禁止假定固定 J1..J6 顺序。**
7. **人工解锁出口**：`clear_unknown_after_manual_confirmation()` 只能由现场人员在核实机械臂确实静止后调用，任何自动流程都不得调用。

## 5. 离线运行

```bash
# 单元测试（不需要 ROS 2 环境）
cd /home/robot/meituan_challenge_ws/src/mtc_motion_execution
python3 test/test_backends.py
```

节点入口（需要 ROS 2 环境与 `mtc_interfaces` 已构建）：

```bash
ros2 run mtc_motion_execution motion_executor --ros-args -p backend:=mock
```

| 接口 | 名称 | 类型 |
|---|---|---|
| Action Server | `/mtc/motion/execute_joint_move` | `mtc_interfaces/action/ExecuteJointMove` |
| Service | `/mtc/motion/capabilities` | `mtc_interfaces/srv/GetMotionCapabilities` |
| Service | `/mtc/motion/request_stop` | `mtc_interfaces/srv/RequestStop` |
| Topic | `/mtc/robot/state` | `mtc_interfaces/msg/RobotState`（RELIABLE, KEEP_LAST(10)） |

## 6. 验证状态

| 项 | 状态 |
|---|---|
| 离线单元测试（20 项） | **PASS**（Python 3.12.3，2026-10-09） |
| ROS 2 Jazzy `colcon build` | **NOT RUN**（本机无 Jazzy，按任务约定不做编译验证） |
| 真实 AUBO S3 运动 | **NOT RUN / 禁止**（需现场明确授权） |

## 7. 未实现与下一步

- `ExecuteTrajectory` / 标准 `FollowJointTrajectory` 后端：阶段二候选，需先实测驱动是否支持带时间戳轨迹。
- 笛卡尔受约束运动（进环/退钩）：需先验证后端能力并标定工具几何。
- 真实后端实现：见 `mtc_aubo_bridge`（默认 disabled）。
