# mtc_moveit_planner —— MoveIt 2 规划适配层（PlannerPort 实现）

> **状态：Mock / Fake Hardware 专用，只规划、不执行。** 输出默认 `simulation_only=true`；
> V1 TCP / 工具外形来自 CAD（ESTIMATED）；工位、提环位姿仍为 PLACEHOLDER。负责人：常建烁。
> 最新结果：[V1 可行性](../../docs/planning/V1_FEASIBILITY.md)、
> [P0 / move_group 退出验收](../../docs/planning/MOVE_GROUP_SHUTDOWN_AUDIT.md)。

## 1. 它做什么

把 `PickPlaceFsm` 的规划请求（`mtc::manipulation::PlanTarget`）交给 MoveIt 2，
逐原语返回**完整时间参数化轨迹**（`PlanResult.commands[i].trajectory`），满足
`PickPlaceFsm::accept_plan()` 的全部检查：

| 检查 | 本包保证 |
|---|---|
| 每个原语一条命令，按 `primitive_name` 对应 | 是；任一原语失败则整体 `ok=false`、`commands` 为空 |
| `trajectory.joint_names == PickPlaceConfig::joint_names` | 按**名称**重排（MoveIt 顺序不可信） |
| `target_rad` 为空、`header_frame_id` 非空 | 是（frame = MoveIt planning frame `world`） |
| `velocity/acceleration_scaling ∈ (0, 0.2]` | 默认 0.1，配置超 0.2 直接拒绝 |
| `timeout_ms >= 轨迹时长` | `ceil(时长×1.5)+1000 ms`；时长超过对应 FSM 状态看门狗 80% 时拒绝 |
| 失败可追溯 | `error_code=PLANNING_FAILED(200)`，`message` 以 `[原因] 原语: 详情` 开头，不静默重试 |

```text
PickPlaceFsm ──PlanTarget──▶ MoveItPlannerPort (纯 C++，可离线测试)
                                 │ 原语表 / TCP 补偿 / 工位查表 / 起点串接 / 结果校验
                                 ▼
                           PlanningBackend 接口
                                 ├─ MoveGroupBackend：MoveGroupInterface + tf2（ROS 外壳）
                                 └─ FakeBackend：GTest 用玩具运动学
```

## 2. 原语规划规则

| 原语 | 执行状态（看门狗） | 方式 | 目标 | 起点 |
|---|---|---|---|---|
| `side_insert`(V1) / `top_insert`(V2) | MOVE_PRE_ALIGN (30 s) | FREE_SPACE | 提环位姿沿插入轴 (+Z) 后退：V1 **0.06 m**，V2 **0.03 m**（预对准位） | 当前状态 |
| `test_lift` | TEST_LIFT (15 s) | 笛卡尔直线 | +Z 0.01 m | **假设** ToolPort 完成对应插入行程后 |
| `lift` | LIFT (30 s) | 笛卡尔直线 | +Z 0.05 m | 当前状态（试提已执行） |
| `transport` | TRANSPORT (60 s) | OMPL | 工位上方 0.10 m，保持抓取姿态 | 上一段终点 |
| `pre_seat` | MOVE_PRE_SEAT (30 s) | 笛卡尔直线 | 工位上方 0.02 m | 上一段终点 |
| `seat_contact` | SEAT (20 s) | 笛卡尔直线 | 工位高度（**接触意图，仅仿真**） | 上一段终点 |
| `retreat` | RETREAT (30 s) | 笛卡尔直线 | +Z 0.10 m | **假设** ToolPort 完成对应退钩行程后 |
| `hook_engage` / `auto_lock` / `unload` / `side_withdraw` / `withdraw` | — | **拒绝**（ToolPort 负责） | — | — |

FREE_SPACE 原语先尝试末端直线，再评估带固定种子的 IK 与逐点碰撞检查的关节插值，最后回退 OMPL；
控制器限位（foreArm ±2.41、wrist1 ±2.66、wrist3 ±2.23）以 MoveIt 路径约束下发并在输出前复核。

插入/退钩行程由本包 `config/planner.yaml` 配置，V2 使用 `sequence_v2_override.yaml`；其余距离仍含占位值。
末端线/角速度上限取原语自身的 `max_linear_speed` / `max_angular_speed`，
关节插值路径启用末端限速时间参数化，其他路径在 TOTG 后按需均匀拉伸时间。

## 2A. 规划场景与起点一致性

- **静态障碍物**：`scene.boxes`（默认占位桌面），工具启动时经 `/apply_planning_scene` 写入。
- **对象 ID 隔离**：本包写入场景的对象一律带前缀 `mtc_moveit_planner/`（如 `mtc_moveit_planner/table`、
  `mtc_moveit_planner/battery_payload`）；`apply_scene` 只增删带该前缀的对象，不会覆盖或删除其他模块的同名对象，
  并会清掉本包上次进程遗留的对象。配置里写本地 ID（`[A-Za-z0-9_.-]`，不得重复、不得自带前缀）。
- **电池载荷**：`carries_payload` 原语（test_lift → seat_contact）在起点 RobotState 上附着长方体，不改全局场景。
- **起点检查**：执行侧发送前调用
  ```cpp
  #include "mtc_moveit_planner/start_consistency.hpp"
  auto check = mtc::moveit_planner::check_trajectory_start(*cmd.trajectory, names, positions);  // 0.01 rad
  if (!check.ok) { /* 拒绝；用 MoveItPlannerPort::replan_primitive(target, cmd.primitive_name, measured) 重规划 */ }
  ```
  轨迹或实测状态中出现**空关节集合、重复 / 空关节名**一律判不通过（不会"取第一个"）；`replan_primitive()` 收到重名的实测状态
  返回 `[INVALID_START_STATE]`。

## 3. 构建与测试

```bash
bash src/mtc_moveit_planner/scripts/build_driver_ws.sh   # AUBO 驱动 5 包（独立目录）
bash src/mtc_moveit_planner/scripts/build_planner.sh     # FSM Core 4 包 + 本包 + GTest
```

## 4. 运行（需要 Fake 控制 + move_group 已启动）

```bash
source src/mtc_moveit_planner/scripts/env.sh
ros2 launch mtc_moveit_planner plan_tool.launch.py program:=plan_fixed_pose
ros2 launch mtc_moveit_planner plan_tool.launch.py program:=plan_battery_sequence
```

或一键：启动 Fake + move_group → 只读采集（ROS graph / TF 图 / URDF / SRDF）→ 规划 → 退出：

```bash
bash src/mtc_moveit_planner/scripts/run_planning_evidence.sh
```

证据写入 `docs/planning/evidence/<时间戳>/`。脚本同时检查 launch 的每个子进程是否正常退出；
崩溃、信号升级、缺少正常退出记录均为 FAIL，并返回非零退出码。
系统 MoveIt 2.12.4 在本机存在插件卸载退出缺陷；诊断与显式启用的临时规避方法见
[退出验收报告](../../docs/planning/MOVE_GROUP_SHUTDOWN_AUDIT.md)。

## 5. 已知限制（详见 docs/planning/PLANNING_HANDOFF.md）

1. 碰撞场景含占位桌面、电池载荷和 V1 CAD 工具外形；缺相机、工位支架，取货电池本体未作为障碍物。
2. V1 TCP 按 CAD 估计，安装方向和贴合面待实测；V2 TCP 仍为单位变换（PLACEHOLDER）。
3. `test_lift` / `retreat` 起点依赖对 ToolPort 动作的假设；执行端必须核对起点偏差。
4. 只读 Fake 环境中验证；不调用 execute()，不发送 FollowJointTrajectory。
