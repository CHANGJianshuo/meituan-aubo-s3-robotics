# INTERFACE_CHANGELOG —— 接口契约变更与兼容性记录

> 阶段：V3 P3
> 依据：`docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md`（下称 V0.1）
> 记录原则：**先核对再编码**；有歧义字段、颜色枚举、错误码、单位与 frame_id 均按 ROSIDL 语法做最小必要修正，并逐条记录；严禁与旧接口同名不同义冲突。

---

## 1. 新增接口清单（本包 `src/mtc_interfaces`）

| 接口 | 类型 | 设计来源 | 实现状态 |
|---|---|---|---|
| `action/ExecuteTask.action` | Action | V0.1 §4.3 | 已实现 |
| `action/PickPlace.action` | Action | V0.1 §4.4 | 已实现 |
| `action/ExecuteJointMove.action` | Action | V0.1 §4.8 | 已实现 |
| `msg/BatteryDetection.msg` | Msg | V0.1 §4.5 | 已实现 |
| `msg/BatteryDetectionArray.msg` | Msg | V0.1 §4.5 | 已实现 |
| `msg/ToolState.msg` | Msg | V0.1 §4.6 | 已实现 |
| `msg/TaskState.msg` | Msg | V0.1 §4.7 | 已实现 |
| `msg/RobotState.msg` | Msg | V0.1 §4.7 | 已实现 |
| `srv/GetMotionCapabilities.srv` | Srv | V0.1 §4.9 | 已实现 |
| `srv/TriggerUnlock.srv` | Srv | V0.1 §4.10 | 已实现 |
| `srv/RequestStop.srv` | Srv | V0.1 §4.10 | 已实现 |
| `msg/ErrorCodes.msg` | Msg | V0.1 §7.1 错误码表 | **本次新增（V0.1 未定义该文件）** |

---

## 2. 相对 V0.1 草案的字段级差异

| 编号 | 接口 | 变更 | 类型 | 理由 |
|---|---|---|---|---|
| C1 | `ExecuteTask.action` | Goal 增加 `float32 placement_stability_sec` | **追加（兼容）** | V0.1 §1.1 要求“释放后稳定 3 秒”，但 §4.3 Goal 未携带该参数；追加而非改变既有字段，消费者可忽略。默认值由 `config/manipulation.yaml` 提供 |
| C2 | `ExecuteTask.action` | Feedback 增加 `string substate` | **追加（兼容）** | 供 UI/日志观察当前 PickPlace 子状态，V0.1 §9 时序图需要该可观测性 |
| C3 | `PickPlace.action` | Goal 增加 `float32 placement_stability_sec` | **追加（兼容）** | 同上，支持按任务项覆盖稳定性时长 |
| C4 | `msg/ErrorCodes.msg` | 新增只含常量的消息文件 | **新增** | V0.1 §7.1 给出错误码表但未规定承载位置。集中常量可避免 C++/Python 侧散落魔法数字；**仅新增，不改动任何既有接口** |
| C5 | `ExecuteJointMove.action` | 增加常量 `COMMAND_OK` / `COMMAND_REJECTED` / `COMMAND_UNKNOWN` | **追加（兼容）** | 为 `execution_state` 与结果语义提供稳定标识，避免字符串拼写漂移 |
| C6 | `ExecuteJointMove.action` | `Feedback.execution_state` 取值集合文档化为执行状态枚举 | **语义明确化** | V0.1 字段已存在但取值未定义；现明确为 `IDLE/ACCEPTED/MOVING/SETTLED/CANCELLING/STOPPED/STOP_UNKNOWN/TIMEOUT/REJECTED/FAULT`（实现见 `mtc_motion_execution.backends.ExecState`） |

**未变更项：** V0.1 已给出的其余字段（名称、类型、顺序）**逐字段照抄**，未做重命名或类型改动。

---

## 3. 既有接口 `PlanMotion.action` / `Latch.srv`：本次**不重定义**

| 项 | 说明 |
|---|---|
| V0.1 的明确要求 | §4.2：“`PlanMotion.action` 与交接工程已有 `Latch.srv` 的真实字段尚未获得源文件，本文件**不擅自重定义这两个已存在类型**”；§12 第 6 条：“拿到真实代码后进行接口对照，不在缺乏代码时臆测实现细节” |
| 本机实测 | 旧工程 `/home/operator/meituan_challenge` **不存在**；全盘查找 `PlanMotion.action` 无命中（见 `SOURCE_AUDIT.md` §4） |
| 本次处理 | **不创建** `action/PlanMotion.action`，**不创建** `srv/Latch.srv`；新接口一律使用**不同名称**（`ExecuteJointMove` 等），不存在同名不同义冲突 |
| 兼容性影响 | 新工程当前**不提供** `PlanMotion` 规划接口——这是如实反映“源缺失”，而不是声称已保留 |
| 风险 | 若后续拿到旧源码，必须逐字段对照后再决定是**原样保留**还是**追加兼容字段**；不得直接覆盖旧契约 |
| 后续动作 | 见 §5 补齐清单 |

**规划与执行的边界（V0.1 §4.2、V3 P3 强制要求）：**
- 规划器（`mtc_motion_planning`）只返回规划结果；
- 执行层（`mtc_motion_execution`）只接受其**已明确声明支持**的运动表示；
- 当前后端只声明 `joint_goal_supported=true`，`timed_trajectory_supported=false`、`cartesian_motion_supported=false`；
- 因此新工程**不提供** `ExecuteTrajectory.action`（V0.1 §4.2 标注为“阶段二候选”），**不声明**完整轨迹执行能力，**不会**把轨迹拆成多次 `moveJoint` 调用。此约束由 `test/test_backends.py::test_trajectory_not_split_into_multiple_calls` 与 `test_mock_backend_declares_no_trajectory_capability` 覆盖。

---

## 4. 兼容性策略声明

1. **追加优先：** 对已发布 `.msg/.srv/.action` 采用追加兼容字段策略；不兼容更改必须升级接口版本并更新消费者（V0.1 §4.1）。
2. **单位与坐标：** 位姿用米 + 四元数（`geometry_msgs/PoseStamped`），关节用弧度，时间字段显式标注 `_ms` 或 `_sec` 后缀。`ExecuteJointMove` 的 `target_rad` / `final_position_rad` / `actual_position_rad` 全部为弧度；`ExecuteTask.timeout_ms` 为毫秒；`placement_stability_sec` 为秒。
3. **颜色枚举：** 统一由 `BatteryDetection.msg` 承载（`COLOR_UNKNOWN=0/ RED=1/ BLUE=2/ YELLOW=3/ GREEN=4`），其他消息引用同一数值集合，不重复定义不同编号。
4. **frame_id：** 每条 `PoseStamped.header.frame_id` 必须可转换到 `base_link`；`ring_pose_valid=false` 时禁止使用 `ring_pose`。详见 `docs/architecture/TF_CONVENTIONS.md`。
5. **命名空间：** 新建业务接口统一前缀 `/mtc/`；`/joint_states`、`/tf`、`/tf_static` 保持 ROS 2 常规名称。
6. **接口版本：** 本包 `package.xml` 版本 `0.1.0`，对应设计文档 V0.1 草案；冻结为 V0.2 前需完成 §5 的源码对齐。

### 4.1 接口命名与话题/服务绑定表

| 绑定名称 | 类型 | 提供方 |
|---|---|---|
| `/mtc/task/execute` | `ExecuteTask` | `mtc_task/task_executor` |
| `/mtc/task/state` | `TaskState` | `mtc_task/task_executor` |
| `/mtc/manipulation/pick_place` | `PickPlace` | `mtc_manipulation/pick_place_server` |
| `/mtc/perception/batteries` | `BatteryDetectionArray` | 感知节点（本次未实现，见报告） |
| `/mtc/motion/execute_joint_move` | `ExecuteJointMove` | `mtc_motion_execution/motion_executor` |
| `/mtc/motion/capabilities` | `GetMotionCapabilities` | `mtc_motion_execution/motion_executor` |
| `/mtc/motion/request_stop` | `RequestStop` | `mtc_motion_execution/motion_executor` |
| `/mtc/robot/state` | `RobotState` | `mtc_motion_execution` / `mtc_aubo_bridge` |
| `/mtc/tool/state` | `ToolState` | `mtc_tool/tool_manager` |
| `/mtc/tool/trigger_unlock` | `TriggerUnlock` | `mtc_tool/tool_manager` |

> V0.1 §7.2 联锁表使用的名称是 `/mtc/safety/stop_motion`；本实现绑定为 `/mtc/motion/request_stop`，语义相同（软件停止请求）。**这是一处显式偏差**，已在此登记；如需严格对齐 V0.1 名称，可在 bringup 中通过重映射实现。

---

## 5. 源码补齐后的对齐清单（待执行）

拿到旧工程源码后，按以下顺序完成接口对齐，**每步都必须记录结果**：

1. 计算并记录 `PlanMotion.action` 与 `Latch.srv` 的 SHA256；
2. 逐字段列出真实字段（名称/类型/单位/默认值），与本文件 §3 的记述对照，标注哪些文档记述有误；
3. 决策：**原样保留** / **追加兼容字段** / **升级版本并行发布**（禁止原地破坏性修改）；
4. 若 `PlanMotion` 的 `mode`、`target_pose`、`goal_state` 等字段语义与 V0.1 理解不同，必须更新 `docs/architecture/` 与消费方（`mtc_manipulation`）适配；
5. 核实 `Latch.srv` 的真实语义：**不得**把 V1 纯机械舌规当作可开关锁扣（V0.1 §11）；
6. 将结果追加到本文件并递增接口版本号。

---

## 6. 验证状态

| 验证项 | 状态 | 说明 |
|---|---|---|
| ROSIDL 语法结构自检（分隔符、字段、常量） | 见 `IMPLEMENTATION_REPORT.md` | 使用脚本自检，非 `rosidl` 官方生成器 |
| `colcon build --packages-select mtc_interfaces` | **NOT RUN** | 本机无 ROS 2 Jazzy；按任务约定不做编译验证 |
| `ros2 interface show` 可显示 | **NOT RUN** | 同上 |
| Python/C++ 消息类型加载 | **NOT RUN** | 同上 |

> **不得**将上述 `NOT RUN` 表述为“编译成功”。复现命令见 `SOURCE_AUDIT.md` §8。
