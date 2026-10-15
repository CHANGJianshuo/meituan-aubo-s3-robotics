---
audit_date: 2026-10-11
audit_target: src/mtc_interfaces（全部 action/msg/srv 定义）及其在 src/**/*.py 中的实际使用
audit_method: 静态只读审计（read / grep / glob）；未编译、未运行任何节点
audit_tooling: 未使用 bash（本机沙箱后端不可用）
auditor: DSH 子代理（接口审计员）
compiled: false
runtime_executed: false
build_status: NOT RUN
runtime_status: NOT RUN
files_modified_by_audit: none
---

# mtc_interfaces 接口兼容性审计报告（面向 C++ 迁移）

> 审计对象：`/home/robot/meituan_challenge_ws/src/mtc_interfaces` 全部接口定义 + 6 个 Python 包中的实际使用点
> 审计方式：仅使用 `read` / `grep` / `glob` 只读工具；**未修改、未创建、未删除任何文件**
> 证据形式：全部为 `文件:行号`
> 结论口径：`已具备` / `部分具备` / `缺少`，不使用模糊表述

---

## 0. 审计范围与静态性声明

### 0.1 审计范围清单

| 类别 | 文件 | 行数 |
|---|---|---|
| Action | `src/mtc_interfaces/action/ExecuteTask.action` | 28 |
| Action | `src/mtc_interfaces/action/PickPlace.action` | 22 |
| Action | `src/mtc_interfaces/action/ExecuteJointMove.action` | 29 |
| Msg | `msg/BatteryDetection.msg` / `BatteryDetectionArray.msg` / `ToolState.msg` / `TaskState.msg` / `RobotState.msg` / `ErrorCodes.msg` | 15 / 5 / 22 / 10 / 13 / 23 |
| Srv | `srv/GetMotionCapabilities.srv` / `TriggerUnlock.srv` / `RequestStop.srv` | 11 / 9 / 8 |
| 构建 | `src/mtc_interfaces/CMakeLists.txt`（57 行）、`package.xml`（30 行）、`README.md`（80 行） | — |
| 迁移文档 | `docs/migration/INTERFACE_CHANGELOG.md`(112) / `MIGRATION_MAP.md`(86) / `SOURCE_AUDIT.md`(242) | — |
| 辅助依据 | `docs/architecture/TF_CONVENTIONS.md`(113) | — |

接口数量核对：`CMakeLists.txt:22-41` 声明的 `msg_files`=6、`srv_files`=3、`action_files`=3，与磁盘文件逐一对应，**无清单外文件、无清单缺项**。

### 0.2 本次审计的静态性声明（重要）

1. 本次审计是**纯静态只读审计**：只使用 `read` / `grep` / `glob` 读取文本，**未使用 bash**，**未修改任何文件**。
2. **未编译**：未执行 `colcon build`，未执行 `rosidl_generate_interfaces`。**编译状态 = NOT RUN**。本机仅有 ROS 2 Humble 且 `ROS_DISTRO=humble` 预置，按项目约定不做编译验证（`SOURCE_AUDIT.md:41-49`、`INTERFACE_CHANGELOG.md:105-112`）。
3. **未运行**：未启动任何节点、未执行 `ros2 interface show`、未加载任何 Python/C++ 消息类型。**运行状态 = NOT RUN**。
4. 因此本报告中"接口是否可被 C++ 正确生成"的结论，**均来自源码与静态引用关系**，不得被解读为"编译通过"或"运行可用"。
5. 报告中所有"未使用/仅声明"判断，指在 `src/**/*.py`（含测试替身）范围内检索无引用；**不含**仓外上位机、UI 或未来 C++ 消费者。

---

## 1. 接口清单表

### 1.1 Action

#### 1.1.1 `ExecuteTask.action`
提供方：`mtc_task/task_executor`（`task_executor.py:203-206` 建 ActionServer）；使用方：比赛启动程序 / 上位机（仓库内无实现）

| 段 | 字段/常量 | 类型 | 语义注释（来源行） |
|---|---|---|---|
| 常量 | `MODE_BASIC=1` / `MODE_SEQUENCE=2` | `uint8` | 任务模式（:6-7） |
| Goal | `task_id` | `string` | 本轮唯一任务 ID，用于**全链路日志关联**（:9） |
| Goal | `mode` | `uint8` | MODE_BASIC 或 MODE_SEQUENCE（:10） |
| Goal | `ordered_colors` | `uint8[]` | 按次序颜色口令，取值见 BatteryDetection.msg（:11） |
| Goal | `timeout_ms` | `uint32` | 任务级截止时间，必须 > 0（:12） |
| Goal | `placement_stability_sec` | `float32` | 追加字段，缺省由参数提供（:13） |
| Result | `success` / `completed_count` / `completed_slots` / `error_code` / `message` | `bool/uint8/string[]/uint16/string` | error_code 取值见 ErrorCodes.msg（:16-20） |
| Feedback | `current_index` / `total_count` / `state` / `current_slot` / `current_object_id` / `substate` | `uint8/uint8/string×4` | 当前 FSM 状态名与 PickPlace 子状态（:23-28） |
| 头部语义 | — | — | **"一次只受理一个 Goal"（:3）** |

#### 1.1.2 `PickPlace.action`
提供方：`mtc_manipulation/pick_place_server`（`pick_place_server.py:667-680`）；使用方：`mtc_task`（`task_executor.py:84` ActionClient）

| 段 | 字段 | 类型 | 语义注释（来源行） |
|---|---|---|---|
| Goal | `request_id` | `string` | 唯一请求 ID（:6） |
| Goal | `object_id` | `string` | 感知给出的唯一目标对象 ID，**执行过程中不得静默切换**（:3、:7） |
| Goal | `expected_color` | `uint8` | 防抓错的双重校验颜色（:8） |
| Goal | `target_slot` | `string` | 仅允许 T0 / P1 / P2 / P3（:9） |
| Goal | `placement_stability_sec` | `float32` | 追加字段（:10） |
| Result | `success` / `placement_verified` / `error_code` / `message` | — | `placement_verified` **仅当 VERIFY_PLACED 通过后才可为 true**（:14） |
| Feedback | `substate` / `progress` / `object_attached_estimated` / `detail` | `string/float32/bool/string` | progress 0.0-1.0（:19-22） |

#### 1.1.3 `ExecuteJointMove.action`
提供方：`mtc_motion_execution/motion_executor`（`motion_executor_node.py:91-99`）；使用方：`mtc_manipulation/pick_place_server`（`pick_place_server.py:672-674`）

| 段 | 字段/常量 | 类型 | 语义注释（来源行） |
|---|---|---|---|
| 常量 | `COMMAND_OK=ok` / `COMMAND_REJECTED=rejected` / `COMMAND_UNKNOWN=unknown` | `string` | 为 execution_state 提供稳定标识（:6-8） |
| Goal | `command_id` | `string` | **无唯一性/幂等语义注释**（:11） |
| Goal | `joint_names` | `string[6]` | 必须与 target_rad 按索引一一对应，**禁止假定 J1..J6 顺序**（:12） |
| Goal | `target_rad` | `float64[6]` | 六关节目标，弧度（:13） |
| Goal | `velocity_scaling` / `acceleration_scaling` | `float32` | 相对"已审核运动原语上限"的缩放，0 < x ≤ 1（:14-15） |
| Goal | `timeout_ms` | `uint32` | （:16） |
| Result | `success` / `stop_confirmed` / `final_position_rad` / `error_code` / `message` | — | `stop_confirmed` **仅在后端确认静止后为 true**（:20） |
| Feedback | `actual_position_rad` / `actual_velocity_rad_s` / `progress` / `execution_state` | `float64[6]×2, float32, string` | execution_state 取值见 ExecState（:26-29） |
| **能力边界** | — | — | **"只适配给定六关节目标并验证真实到位；不承诺中间点逐点时间同步；禁止把轨迹拆成高频调用本接口来伪装轨迹控制"（:3-4）** |

### 1.2 Msg

| 消息 | 字段 | 语义注释 | 行 |
|---|---|---|---|
| `BatteryDetection` | `COLOR_UNKNOWN=0 / RED=1 / BLUE=2 / YELLOW=3 / GREEN=4` | 颜色枚举权威来源 | :4-8 |
| | `object_id` | 稳定唯一 ID，同一物理电池跨帧一致 | :10 |
| | `color` | — | :11 |
| | `body_pose` | `geometry_msgs/PoseStamped`，电池主体中心 | :12 |
| | `ring_pose` | `geometry_msgs/PoseStamped`，提环局部抓取参考系，轴向见 TF_CONVENTIONS.md | :13 |
| | `confidence` | `float32` | :14 |
| | `ring_pose_valid` | **false 时禁止使用 ring_pose 生成运动目标** | :15 |
| `BatteryDetectionArray` | `header` | 感知按 RELIABLE + KEEP_LAST(5) 发布 | :4 |
| | `BatteryDetection[] detections` | — | :5 |
| `ToolState` | `STATE_UNKNOWN=0` … `STATE_FAULT=5` | 工具状态 | :4-9 |
| | `EVIDENCE_NONE=0` … `EVIDENCE_SENSOR_OR_VISION=3` | 证据等级 | :11-14 |
| | `header` / `state` / `evidence_level` / `verified` / `tool_type` / `object_id` / `detail` | **state 是估计，verified 是独立维度，二者不得混为一谈**；tool_type ∈ {passive_hook_v1, magnetic_latch_v2} | :16-22 |
| `TaskState` | `header` / `task_id` / `state` / `substate` / `completed_count` / `total_count` / `last_error_code` / `detail` | 任务状态快照 | :3-10 |
| `RobotState` | `header` / `communication_ok` / `robot_ready` / `motion_active` / `safety_normal` / `stop_confirmed` / `simulation_mode` / `controller_mode` / `detail` | 由适配层规约后的信息；**底层 SDK 原始状态码必须另行记日志，不得用布尔值丢弃关键安全原因** | :5-13 |
| `ErrorCodes` | 18 个 `uint16` 常量（OK=0 … CANCEL_NOT_CONFIRMED=520） | 只承载常量不承载字段；变更语义必须同步 INTERFACE_CHANGELOG | :6-23 |

### 1.3 Srv

| 服务 | 段 | 字段 | 语义注释 | 行 |
|---|---|---|---|---|
| `GetMotionCapabilities` | Resp | `joint_goal_supported` / `timed_trajectory_supported` / `cartesian_motion_supported` / `cancel_supported` / `backend_name` / `detail` | capability 只表示**当前已连接且经实测启用**的通路能力，不得仅因 SDK 声称即置 true | :6-11 |
| `TriggerUnlock` | Req | `request_id` / `pulse_ms` | 仅允许 tool_manager 在 VERIFY_SEATED + UNLOAD 之后调用 | :5-6 |
| | Resp | `accepted` / `message` | **accepted=true 只表示解锁脉冲请求已受理，不表示电池已释放** | :8-9 |
| `RequestStop` | Req | `reason` | 软件停止请求，**不是紧急停止**，不保证网络失效时能停下 | :5 |
| | Resp | `request_delivered` / `message` | **request_delivered 只表示请求送达执行适配层；stop_confirmed 必须另行确认** | :7-8 |

### 1.4 话题/服务绑定与提供方/使用方

绑定表依据 `INTERFACE_CHANGELOG.md:73-84`、`README.md:14-37`，代码实测交叉核对：

| 绑定名称 | 类型 | 提供方（代码证据） | 仓库内使用方 |
|---|---|---|---|
| `/mtc/task/execute` | ExecuteTask | `task_executor.py:63,203-206` | 无（上位机，未实现） |
| `/mtc/task/state` | TaskState | `task_executor.py:65,200-202` | **无订阅方** |
| `/mtc/manipulation/pick_place` | PickPlace | `pick_place_server.py:492` | `task_executor.py:64,84` |
| `/mtc/motion/execute_joint_move` | ExecuteJointMove | `motion_executor_node.py:30,91-99` | `pick_place_server.py:203,212,672-674` |
| `/mtc/motion/capabilities` | GetMotionCapabilities | `motion_executor_node.py:31,100-102` | 无 |
| `/mtc/motion/request_stop` | RequestStop | `motion_executor_node.py:32,103-104` | `safety_supervisor.py:47,111-115` |
| `/mtc/robot/state` | RobotState | **`motion_executor_node.py:33,105` 与 `bridge_node.py:26,75` 两个发布者** | `safety_supervisor.py:44,122` |
| `/mtc/tool/state` | ToolState | `tool_manager_node.py:37,80` | `safety_supervisor.py:45,123` |
| `/mtc/tool/trigger_unlock` | TriggerUnlock | `tool_manager_node.py:38,81` | `pick_place_server.py:675` |
| `/mtc/perception/batteries` | BatteryDetectionArray | 感知节点**未实现**（`pick_place_server.py:15` 注释声明） | `pick_place_server.py:670`（订阅方，无发布方） |
| `/mtc/bridge/capabilities` | **GetMotionCapabilities（类型复用）** | `bridge_node.py:27,77-78` | 无 |
| `/mtc/bridge/identity` | **GetMotionCapabilities（类型复用）** | `bridge_node.py:28,79-81` | 无 |
| `/mtc/bridge/request_stop` | **RequestStop（类型复用）** | `bridge_node.py:29,82-83` | 无 |
| 未实现 | PlanMotion.action / Latch.srv | **刻意不存在**（`CMakeLists.txt:19-20`、`INTERFACE_CHANGELOG.md:43-52`、`SOURCE_AUDIT.md:116`） | — |

---

## 2. 实际使用点矩阵

标记：R=读取，W=写入，`-`=仓库内无访问，`★`=镜像常量绕开接口常量

### 2.1 `ExecuteTask`

| 字段 | 访问 | 证据（文件:行号） |
|---|---|---|
| `MODE_BASIC/MODE_SEQUENCE` | ★ 硬编码 | `task_validation.py:24-25`（未引用 `ExecuteTask.MODE_*`） |
| `task_id` | R | `task_validation.py:90,121-124`；`task_fsm.py:1071-1073`；`task_executor.py:254` |
| `mode` | R | `task_validation.py:90,126-130` |
| `ordered_colors` | R | `task_validation.py:90,132-162`；`task_executor.py:376`；`task_fsm.py:1040-1042` |
| `timeout_ms` | R | `task_validation.py:90,164-168` |
| `placement_stability_sec` | R | `task_validation.py:46,90,93-94,170-176`；`task_fsm.py:1062` |
| `success` | W | `task_executor.py:282,294,335` |
| `completed_count` | W | `task_executor.py:283,295,336` |
| `completed_slots` | W | `task_executor.py:284,296,337` |
| `error_code` | W | `task_executor.py:285,297-299,338` |
| `message` | W | `task_executor.py:286,300,339` |
| `current_index` | W | `task_executor.py:496` |
| `total_count` | W | `task_executor.py:497` |
| `state` | W | `task_executor.py:498` |
| `current_slot` | W | `task_executor.py:499` |
| `current_object_id` | W | `task_executor.py:500` |
| `substate` | W | `task_executor.py:501` |

### 2.2 `PickPlace`

| 字段 | 访问 | 证据 |
|---|---|---|
| `request_id` | W / R | W: `task_executor.py:97`；R: `pick_place_server.py:505` |
| `object_id` | W / R | W: `task_executor.py:98`；R: `pick_place_server.py:502` |
| `expected_color` | W / R | W: `task_executor.py:99`；R: `pick_place_server.py:503` |
| `target_slot` | W / R | W: `task_executor.py:100`；R: `pick_place_server.py:504` |
| `placement_stability_sec` | W / R | W: `task_executor.py:101`；R: `pick_place_server.py:506-508` |
| `success` | W / R | W: `pick_place_server.py:540`；R: `task_executor.py:129` |
| `placement_verified` | W / R | W: `pick_place_server.py:541`；R: `task_executor.py:130,360` |
| `error_code` | W / R | W: `pick_place_server.py:542`；R: `task_executor.py:131,358` |
| `message` | W / R | W: `pick_place_server.py:543`；R: `task_executor.py:132` |
| `substate` | W / R | W: `pick_place_server.py:563`；R: `task_executor.py:368,511` |
| `progress` | W / R | W: `pick_place_server.py:564`；R: `task_executor.py:512` |
| `object_attached_estimated` | W / R | W: `pick_place_server.py:565`；R: `task_executor.py:513` |
| `detail` | W / R | W: `pick_place_server.py:566`；R: `task_executor.py:514` |

### 2.3 `ExecuteJointMove`

| 字段 | 访问 | 证据 |
|---|---|---|
| `COMMAND_OK/REJECTED/UNKNOWN` | **-** | 全仓库 grep 零命中 |
| `command_id` | W / R | W: `pick_place_server.py:227`；R: `motion_executor_node.py:134` |
| `joint_names` | W / R | W: `pick_place_server.py:228`；R: `motion_executor_node.py:135` → `backends.py:192-203` |
| `target_rad` | W / R | W: `pick_place_server.py:229`；R: `motion_executor_node.py:136` → `backends.py:192-222` |
| `velocity_scaling` | W / R | W: `pick_place_server.py:230`；R: `motion_executor_node.py:137` → `backends.py:206-209` |
| `acceleration_scaling` | W / R | W: `pick_place_server.py:231`；R: `motion_executor_node.py:138` → `backends.py:210-213` |
| `timeout_ms` | W / R | W: `pick_place_server.py:232`；R: `motion_executor_node.py:139` → `backends.py:204,331` |
| `success` | W / R | W: `motion_executor_node.py:142`；R: `pick_place_server.py:257` |
| `stop_confirmed` | W / R | W: `motion_executor_node.py:143`；R: `pick_place_server.py:261,290`；`executor_core.py:175`；`backends.py:87` |
| `final_position_rad` | W / R | W: `motion_executor_node.py:146-149`；R: `pick_place_server.py:262` |
| `error_code` | W / R | W: `motion_executor_node.py:144`；R: `pick_place_server.py:263` |
| `message` | W / R | W: `motion_executor_node.py:145`；R: `pick_place_server.py:264` |
| `actual_position_rad` | W | `motion_executor_node.py:155`（**仅终点发布一次**） |
| `actual_velocity_rad_s` | W | `motion_executor_node.py:156`（**仓库内无读取方**） |
| `progress` | W | `motion_executor_node.py:157`（恒为 1.0 或 0.0） |
| `execution_state` | W | `motion_executor_node.py:158`（取 `ExecState` 枚举值，`backends.py:20-30`） |

### 2.4 Msg

| 消息.字段 | 访问 | 证据 |
|---|---|---|
| `BatteryDetection.COLOR_*` | ★ 硬编码 | `task_validation.py:30-31`、`config_loader.py:55-63`、`mock_pipeline.py:302-303` |
| `BatteryDetection.object_id` | R | `pick_place_server.py:102,105,114`；`task_executor.py:393-394` |
| `BatteryDetection.color` | R | `pick_place_server.py:116-117,151` |
| `BatteryDetection.body_pose` | R（**仅取 position 与 header.stamp**） | `pick_place_server.py:124-127,143-147,153` |
| `BatteryDetection.ring_pose` | R（**仅取 position**） | `pick_place_server.py:143-147,152` |
| `BatteryDetection.confidence` | R | `pick_place_server.py:154`；`task_executor.py:400-403` |
| `BatteryDetection.ring_pose_valid` | R | `pick_place_server.py:115,156`；`task_executor.py:398-403` |
| `BatteryDetectionArray.header` | **-** | 从未读取 |
| `BatteryDetectionArray.detections` | R | `pick_place_server.py:101,106` |
| `ToolState.STATE_FAULT` | R（唯一被引用的 state 常量） | `safety_supervisor.py:139` |
| `ToolState` 其余 `STATE_*` / `EVIDENCE_*` | ★ 镜像 | `codes.py:25-54`、`pick_place_fsm.py:61-75`、`mock_pipeline.py:521-525` |
| `ToolState.header` | W | `tool_manager_node.py:128-130`（含 `frame_id="tool"`）；`pick_place_server.py:578-580`（仅 stamp） |
| `ToolState.state` | W / R | W: `tool_manager_node.py:131`、`pick_place_server.py:581`；R: `interlocks.py:875`、`safety_supervisor.py:139` |
| `ToolState.evidence_level` | W / R | W: `tool_manager_node.py:132`、`pick_place_server.py:582`；R: `interlocks.py:877` |
| `ToolState.verified` | W / R | W: `tool_manager_node.py:133`、`pick_place_server.py:583-585`；R: `interlocks.py:876` |
| `ToolState.tool_type` | W（**无读取方**） | `tool_manager_node.py:134`、`pick_place_server.py:586` |
| `ToolState.object_id` | W（**无读取方**） | `tool_manager_node.py:135`、`pick_place_server.py:587` |
| `ToolState.detail` | W / R | W: `tool_manager_node.py:136`、`pick_place_server.py:588-591`；R: `interlocks.py:878`、`safety_supervisor.py:141` |
| `TaskState.*`（全部 8 字段） | W（**无订阅方**） | `task_executor.py:482-489` |
| `RobotState.header` | W（**无读取方**） | `motion_executor_node.py:198`、`bridge_node.py:124`、`safety_supervisor.py:190` |
| `RobotState.communication_ok` | W / R | W: `motion_executor_node.py:201`、`bridge_node.py:127`、`safety_supervisor.py:191`；R: `interlocks.py:843` |
| `RobotState.robot_ready` | W / R | W: `motion_executor_node.py:202`、`bridge_node.py:128`、`safety_supervisor.py:192`；R: `interlocks.py:845` |
| `RobotState.motion_active` | W / R | W: `motion_executor_node.py:203`、**`bridge_node.py:129`（硬编码 False）**、`safety_supervisor.py:193`；R: `interlocks.py:846` |
| `RobotState.safety_normal` | W / R | W: `motion_executor_node.py:204`、`bridge_node.py:130`、`safety_supervisor.py:194-195`；R: `interlocks.py:844` |
| `RobotState.stop_confirmed` | W / R | W: `motion_executor_node.py:205`、`bridge_node.py:131`、`safety_supervisor.py:196`；R: `interlocks.py:847,855` |
| `RobotState.simulation_mode` | W / R | W: `motion_executor_node.py:206`、`bridge_node.py:132`、`safety_supervisor.py:197`；R: `interlocks.py:849,862` |
| `RobotState.controller_mode` | W / R | W: `motion_executor_node.py:207`、`bridge_node.py:133`、`safety_supervisor.py:198`；R: `interlocks.py:848,862` |
| `RobotState.detail` | W / R | W: `motion_executor_node.py:208-212`；R: `interlocks.py:850,869` |
| `ErrorCodes.*` | ★ 镜像（唯一真实引用见 §3.1） | `tool_manager_node.py:27,41-44,174-175` |

### 2.5 Srv

| 服务.字段 | 访问 | 证据 |
|---|---|---|
| `GetMotionCapabilities.joint_goal_supported` | W / R | W: `motion_executor_node.py:176`、`bridge_node.py:89,100`；R: `backends.py:189` |
| `.timed_trajectory_supported` | W（**无读取方**） | `motion_executor_node.py:177`、`bridge_node.py:90,101` |
| `.cartesian_motion_supported` | W（**无读取方**） | `motion_executor_node.py:178`、`bridge_node.py:91,102` |
| `.cancel_supported` | W / R | W: `motion_executor_node.py:179`、`bridge_node.py:92,103`；R: `executor_core.py:196` |
| `.backend_name` | W | `motion_executor_node.py:180`、`bridge_node.py:93,104` |
| `.detail` | W | `motion_executor_node.py:181`、`bridge_node.py:94,105-108`（**身份校验结果被塞进 detail JSON**） |
| `TriggerUnlock.request_id` | W / R | W: `pick_place_server.py:387`（**恒为空，见 §3.3**）；R: `tool_manager_node.py:157,171,178-193` |
| `TriggerUnlock.pulse_ms` | W / R | W: `pick_place_server.py:388`、`tool_strategy.py:320`；R: `tool_manager_node.py:150-153` |
| `TriggerUnlock.accepted` | W / R | W: `tool_manager_node.py:162`；R: `pick_place_server.py:412,419` |
| `TriggerUnlock.message` | W / R | W: `tool_manager_node.py:163-167`；R: `pick_place_server.py:423` |
| `RequestStop.reason` | W / R | W: `safety_supervisor.py:53-56`；R: `motion_executor_node.py:186,190`、`bridge_node.py:112,116` |
| `RequestStop.request_delivered` | W / R | W: `motion_executor_node.py:187`、`bridge_node.py:113`；R: `interlocks.py:733-734` |
| `RequestStop.message` | W / R | W: `motion_executor_node.py:188`、`bridge_node.py:114`；R: `interlocks.py:734` |

---

## 3. 未使用 / 仅声明字段清单

### 3.1 完全零引用（常量类）— 严重

| 接口项 | 声明位置 | 事实 | 影响 |
|---|---|---|---|
| `ExecuteJointMove.COMMAND_OK/COMMAND_REJECTED/COMMAND_UNKNOWN` | `ExecuteJointMove.action:6-8` | **全仓库零引用**（grep `COMMAND_OK\|COMMAND_REJECTED\|COMMAND_UNKNOWN` 无命中）。`execution_state` 实际由 `backends.ExecState` 枚举填充（`motion_executor_node.py:158`） | 接口声明了字符串常量契约，实现走了另一套枚举，C++ 侧无法据此对齐 |
| `ExecuteTask.MODE_BASIC/MODE_SEQUENCE` | `ExecuteTask.action:6-7` | 零引用；`task_validation.py:24-25` 重新硬编码 1/2 | 双份定义，任一侧改动不会同步 |
| `BatteryDetection.COLOR_UNKNOWN/RED/BLUE/YELLOW/GREEN` | `BatteryDetection.msg:4-8` | 零引用；三处硬编码镜像：`task_validation.py:30-31`、`config_loader.py:55-63`、`mock_pipeline.py:302-303` | 同上 |
| `ErrorCodes.*`（18 个） | `ErrorCodes.msg:6-23` | 唯一真实引用是 `tool_manager_node.py:27,41-44`（`getattr(ErrorCodesMsg, name, fallback)`，只取 `SAFETY_INTERLOCK`/`UNLOCK_FAILED` 两个，`tool_manager_node.py:174-175`）。其余 6 处全是本地镜像：`task_fsm.py:70-90`、`pick_place_fsm.py:80-100`、`interlocks.py:50-67`、`backends.py:36-42`、`bridge_core.py:19-23`、`codes.py:84-101` | **ErrorCodes.msg 事实上未被作为契约使用**；`task_executor.py:55` 从 `task_fsm`（而非 `mtc_interfaces.msg`）导入 `ErrorCodes` |
| `motion_executor_node.py:24` 的 `import ErrorCodes` | — | 导入后**从未使用**（文件内无 `ErrorCodes.` 引用） | 死导入；且 `backends.py:33-35` 注释声称"motion_executor_node 会断言两者一致"，**该断言在代码中不存在** |
| `ToolState.STATE_*`（除 FAULT 外）/`EVIDENCE_*` | `ToolState.msg:4-14` | 仅 `STATE_FAULT` 被引用（`safety_supervisor.py:139`）；其余走 `codes.py:25-54` 镜像 | 契约与镜像并存 |

### 3.2 只写未读（仓库内无读取方）

| 字段 | 写入位置 | 说明 |
|---|---|---|
| `TaskState` 全 8 字段 | `task_executor.py:482-489` | 无任何订阅方（设计上供 UI，未实现） |
| `ExecuteTask.Feedback` 全 6 字段 | `task_executor.py:496-501` | 经 DDS 发出，仓库内无消费者 |
| `ExecuteJointMove.Feedback.actual_velocity_rad_s` | `motion_executor_node.py:156` | `RosMotionPort` 发 goal 时**未传 feedback_callback**（`pick_place_server.py:234`），无任何消费者 |
| `ExecuteJointMove.Feedback` 其余 3 字段 | `motion_executor_node.py:155-158` | 同上 |
| `RobotState.header` / `TaskState.header` / `ToolState.header` / `BatteryDetectionArray.header` | `motion_executor_node.py:198`、`task_executor.py:482`、`tool_manager_node.py:128-130`、`bridge_node.py:124`、`safety_supervisor.py:190` | **frame_id 从未被读取**；`stamp` 仅在 `pick_place_server.py:124-129` 用于过期判定 |
| `ToolState.tool_type` / `ToolState.object_id` | `tool_manager_node.py:134-135`、`pick_place_server.py:586-587` | `interlocks.py:875-878` 只读 state/verified/evidence_level/detail |
| `GetMotionCapabilities.timed_trajectory_supported` / `cartesian_motion_supported` | `motion_executor_node.py:177-178`、`bridge_node.py:90-91,101-102` | 只有写与日志，无任何代码据其分支（`executor_core.py:196` 只查 `cancel_supported`） |

### 3.3 字段存在但运行时值无意义（接口—实现背离）

| 字段 | 证据 | 事实 |
|---|---|---|
| `TriggerUnlock.Request.request_id` | `pick_place_server.py:387`：`getattr(self, "_current_request_id", "")` | `RosToolPort` 类**从未定义** `_current_request_id`（grep 显示该名字只出现在 `task_fsm.py:852,1049,1058` 与本行的 getattr 兜底），故实际发出恒为 `""`。TriggerUnlock 的 request_id 关联语义**形同虚设** |
| `ExecuteJointMove.Feedback.progress` | `motion_executor_node.py:157,159-160` | feedback 只在 execute 结束前发布**一次**，值恒为 `1.0`/`0.0`，不具备进度语义 |
| `ExecuteJointMove.Result.final_position_rad` / `Feedback.actual_position_rad` | `motion_executor_node.py:146-149,155-156` | 不足 6 维时用 `math.nan` 补齐后发出，**NaN 会经 DDS 传播** |
| `RobotState.motion_active` | `bridge_node.py:129` | 硬编码 `False`，而另一发布者的同话题消息为真实值（见 §5 R1） |
| `GetMotionCapabilities`（`/mtc/bridge/identity`） | `bridge_node.py:97-109` | 类型被复用承载"设备身份校验"语义，结果只能塞进 `detail` JSON 字符串 |

---

## 4. C++ 迁移的接口缺口分析

### 4.a 现有多点轨迹执行能力缺口 —— 结论：**缺少（明确缺少对应 Action）**

**证据链：**

1. `CMakeLists.txt:37-41` 的 `action_files` 只有 3 项：`ExecuteTask`、`PickPlace`、`ExecuteJointMove`。**不存在 `ExecuteTrajectory.action`，也不存在 `FollowJointTrajectory` 的任何封装**。
2. `CMakeLists.txt:11-13` 的 `find_package` 只有 `std_msgs`、`geometry_msgs`、`builtin_interfaces`；`package.xml:16-18` 的 `<depend>` 同样只有这三项。**没有 `trajectory_msgs`、`control_msgs` 依赖**。全 `src/` 中 grep `trajectory_msgs|control_msgs|JointTrajectory|FollowJointTrajectory` 仅命中：`GetMotionCapabilities.srv:7`（能力布尔位）、若干 `timed_trajectory_supported=False` 硬编码、以及 `src/mtc_tool/test/test_tool_manager.py:60` 把 `"FollowJointTrajectory"` 列为**禁止出现在工具层**的关键词。**没有任何类型定义或依赖。**
3. `ExecuteJointMove.action:3-4` 明文限定能力边界：**"只适配'给定六关节目标并验证真实到位'的能力。不承诺中间点逐点时间同步；禁止把轨迹拆成高频调用本接口来伪装轨迹控制。"**
4. 目标/反馈维度被**固定为 6**：`ExecuteJointMove.action:12-13` 为 `string[6] joint_names` / `float64[6] target_rad`，:21/:26-27 同为固定 6 维。ROSIDL 生成 `std::array<..., 6>`，**在类型层面无法承载变长、非 6 关节或部分关节的轨迹**。
5. 时间与运动学信息承载能力不足：`velocity_scaling`/`acceleration_scaling`（:14-15）是**单一标量缩放**，无法承载 `JointTrajectoryPoint` 的 `positions/velocities/accelerations/effort/time_from_start`；`timeout_ms`（:16）是整段超时，不是轨迹时间参数化。
6. `GetMotionCapabilities.srv:7` 声明了 `timed_trajectory_supported`，但**没有任何接口能消费它为 true 时的运动表示**——能力位与数据通道不匹配。
7. 工程决策已被文档固化并测试覆盖：`INTERFACE_CHANGELOG.md:54-58` 明确"新工程**不提供** `ExecuteTrajectory.action`（V0.1 §4.2 标注为'阶段二候选'）"；`src/mtc_motion_execution/README.md:75` 记载"`ExecuteTrajectory` / 标准 `FollowJointTrajectory` 后端：阶段二候选"。
8. `src/mtc_motion_planning/` 只有 `README.md`（glob 确认），规划器未迁移；`SOURCE_AUDIT.md:165` 记载历史实测边界"**未**验证完整 `FollowJointTrajectory`"。

**C++ 侧后果：** 若 C++ 侧要执行 MoveIt 2 规划出的 `trajectory_msgs/JointTrajectory`，**必须新增接口**（例如 `mtc_interfaces/action/ExecuteTrajectory.action`，或直接采用 `control_msgs/action/FollowJointTrajectory`），并同步在 `CMakeLists.txt`/`package.xml` 增加 `trajectory_msgs`/`control_msgs` 依赖。现有 `ExecuteJointMove` **在类型层面就无法承载该数据**，且其注释明确禁止以高频多次调用模拟轨迹——沿现有接口实现必然违约。

### 4.b 6D Pose / frame_id / TF 信息保真度 —— 结论：**接口层"部分具备"，消费侧"已退化为纯 XYZ"，协方差"缺少"**

逐个判断：

| 接口项 | PoseStamped？ | frame_id | 时间戳 | orientation | 协方差 |
|---|---|---|---|---|---|
| `BatteryDetection.body_pose`（`BatteryDetection.msg:12`） | **已具备** `geometry_msgs/PoseStamped` | **已具备**（随 PoseStamped 携带） | **已具备** | **已具备**（Pose.orientation） | **缺少**（PoseStamped 无 covariance） |
| `BatteryDetection.ring_pose`（:13） | **已具备** | **已具备** | **已具备** | **已具备** | **缺少** |
| `BatteryDetection.confidence`（:14） | — | — | — | — | 以 `float32` 标量代替协方差（**不是协方差**） |
| `ExecuteTask.*` | **缺少**（只有颜色枚举与槽位名，`ExecuteTask.action:11`） | 缺少 | 缺少 | 缺少 | 缺少 |
| `PickPlace.*` | **缺少**（Goal 只有 `target_slot` 字符串，`PickPlace.action:9`；无任何位姿字段） | 缺少 | 缺少 | 缺少 | 缺少 |
| `ExecuteJointMove.*` | **缺少**（纯关节空间，无笛卡尔位姿） | 缺少 | 缺少 | 缺少 | 缺少 |
| `TaskState` / `RobotState` / `ToolState` | **缺少**（只有 `std_msgs/Header`，无位姿载荷） | Header 可携带但**从未被读取** | Header 只写不读 | 缺少 | 缺少 |

**关键退化证据（接口→Python 消费侧）：**

1. `pick_place_fsm.py:184-194` 的 `PoseEstimate` docstring 自称"**替代 geometry_msgs/PoseStamped**"，但字段是 `Optional[Tuple[float, float, float]]`（:190-191）——**orientation 与 frame_id 被彻底丢弃**。
2. `pick_place_server.py:138-157` 的 `_to_pose()`：`_xyz()`（:140-147）只取 `position.x/y/z`；调用点为 `:152 ring_pose=_xyz(...)`、`:153 body_pose=_xyz(...)`。**没有任何一行读取 `pose.orientation` 或 `pose_stamped.header.frame_id`。**
3. `pick_place_server.py:124-129` 是全仓库**唯一**读取位姿时间戳的地方，且只用它做 stale 判定，不读 frame_id。
4. 全 `src/**/*.py` 中 `frame_id` 仅 3 处命中：`tool_manager_node.py:130`（**写入** `"tool"`）、`config_loader.py:725-727`（校验配置项 `slot_pose_frame_id` 非空）、`strategy.py:37`（本地 `PoseTarget.frame_id` 数据类字段）。**没有任何 ROS 消息的 frame_id 被消费。**
5. 与设计约定直接冲突：`TF_CONVENTIONS.md:94-95` 规定"位置 米 / 姿态 **四元数**（`geometry_msgs/PoseStamped`）"、`:26-28` 规定"所有业务位姿最终必须能转换到 `base_link`；变换超时或数据过期时**拒绝生成运动目标**"。当前实现**既无 TF 查询、也无四元数传递**，该硬性规则在代码中无落点。`INTERFACE_CHANGELOG.md:67` 的 frame_id 兼容性策略同样只是文档声明。
6. 协方差：`BatteryDetection` 未使用 `geometry_msgs/PoseWithCovarianceStamped`（全仓库零引用），因此**感知不确定度无法以标准协方差承载**，只有单标量 `confidence`。

**结论：** 接口定义层对 `BatteryDetection` 的两条位姿**已具备**完整的 `PoseStamped`（含 frame_id、时间戳、四元数）；但（i）**缺少**协方差承载；（ii）`ExecuteTask`/`PickPlace`/`ExecuteJointMove` 三个 Action **缺少**任何位姿载体；（iii）**已存在的 frame_id/orientation 在 Python 侧 100% 被丢弃**。C++ 侧若严格按现有 Python 行为实现，将无法完成 `base_link` 下的规划；若按接口定义实现，则与现网 Python 消费方的语义不一致（Python 侧拿不到姿态）。

### 4.c 取消与停止确认 —— 结论：**缺少完整链路（RequestStop 只能表达"送达"；ExecuteJointMove 的取消路径断裂）**

**已具备的部分：**

- `RequestStop.srv:5,7-8` 正确地把"送达"与"停稳"分离：响应只有 `request_delivered`，注释明确"`stop_confirmed` 必须另行确认"。
- 独立的停稳证据通道存在：`RobotState.msg:10 stop_confirmed`（发布 `motion_executor_node.py:205`、`bridge_node.py:131`、`safety_supervisor.py:196`）；`ExecuteJointMove.action:20 stop_confirmed`。
- 消费侧语义正确：`interlocks.py:854-855` "只有执行层的独立证据可以置位 stop_confirmed"；`safety_supervisor.py:156-158` "异步服务回执只更新送达状态，绝不置位 stop_confirmed"；`interlocks.py:975-979` 有独立的"停稳超时"升级判定。
- `ErrorCodes.CANCEL_NOT_CONFIRMED=520` 已定义（`ErrorCodes.msg:23`），Mock 后端在"取消但未确认"时返回它（`backends.py:346-352,376-382`）。

**缺口 1（严重）：`ExecuteJointMove` 的 ROS 层取消不触发任何停止动作。**

- `motion_executor_node.py:124-126`：`_on_cancel()` **只打印日志并 `return CancelResponse.ACCEPT`**，不调用 `self._core.request_stop(...)`。
- `motion_executor_node.py:128-171`：`_on_execute()` 全程**没有任何 `goal_handle.is_cancel_requested` 检查**（grep `is_cancel_requested` 在 `mtc_motion_execution` 下零命中），也从不调用 `self._core.request_stop`。
- 后果：客户端对 `ExecuteJointMove` 发起 `cancel_goal_async()` 后，后端 `MockMotionBackend` 的 `_cancel_requested` 只有经 `/mtc/motion/request_stop` 服务（`motion_executor_node.py:184-193`）才会被置位（`backends.py:290`）。**仅取消 Goal 不会让运动停止**。
- 后果：`Result` 只在 execute 结束时构造，取消路径**无法产出 `stop_confirmed`**；`ErrorCodes.CANCEL_NOT_CONFIRMED=520` 在 Action 结果层面实际不可达。

**缺口 2（严重）：`RequestStop.srv` 无关联标识，无法把"某次停止请求"与"随后的停稳确认"绑定。**

- `RequestStop.srv:5-8` 只有 `reason`（字符串）→ `request_delivered` + `message`。**没有 request_id、没有 command_id、没有时间戳、没有序列号。**
- `motion_executor_node.py:184-188` 的响应即使内部拿到了 `_confirmed`（`executor_core.py:184-218` 的第三个返回值），也**直接丢弃**（`_confirmed` 未使用），因为 srv 定义里没有可承载的字段。
- 关联只能靠 `reason` 字符串匹配 + `interlocks.py:975-979` 的时间窗推断，属于**启发式**；多源停止（安全节点 / 任务节点 / 手动）无法区分。
- `interlocks.py:907-915` 的 `note_stop_delivery()` 同样只能记录一个全局 `last_delivery` 布尔，**无请求身份**。

**缺口 3（中）：停稳确认的粒度不足。**

- `RobotState.stop_confirmed`（`RobotState.msg:10`）是**单布尔、无在途命令身份**；`motion_executor_node.py:203` 已把 `active_command_id` 存在，但只能塞进 `detail` 文本（:210 `active_command={...}`）。C++ 侧要判断"是哪个命令被确认停稳"必须解析自然语言字符串。
- `interlocks.py:500-523` 的 `note_stop_confirmed` 也以全局布尔为主体。

**缺口 4（中）：`StopResult` 是 Python 内部类型，不构成跨语言契约。**

- `pick_place_fsm.py:272-276` 的 `StopResult(request_delivered, stop_confirmed, message)` 并不对应任何 ROSIDL 类型；`pick_place_server.py:267-295` 把它映射为"`cancel_goal_async()` → `get_result_async()` → 读 `result.stop_confirmed`"。这条链路**完全依赖 `ExecuteJointMove` 取消路径能产出 `stop_confirmed`**——而缺口 1 表明它不能。故 `pick_place_fsm.py:1455-1466`（`if not stop.stop_confirmed: → MOTION_STATUS_UNKNOWN`）在当前实现下**必然走向 unknown 分支**。

### 4.d 并发 Goal 冲突防护 —— 结论：**接口层无体现；仅在实现层以 goal_callback 兜底；并且存在同话题双发布者的绑定缺陷**

**接口层已有的"弱"语义：**

| 接口 | 字段/注释 | 是否构成并发防护 |
|---|---|---|
| `ExecuteTask.action:3` | "一次只受理一个 Goal" | 仅**注释**；无字段承载 |
| `ExecuteTask.action:9 task_id` | "本轮唯一任务 ID，用于**全链路日志关联**" | 语义是日志关联，**不是幂等键**；接口未声明重复 task_id 必须拒绝 |
| `PickPlace.action:6 request_id` | "唯一请求 ID" | 无去重/幂等语义说明 |
| `ExecuteJointMove.action:11 command_id` | 无任何语义注释 | 无唯一性声明；去重仅存在于 `bridge_core.py:401-404`（`command_id 重复：…（禁止重复执行同一命令）`）与 `bridge_core.py:406-408`（单在途） |
| 任何接口 | 无 | **没有任何字段表示"当前在途命令 ID"或"goal 唯一性/幂等键"** |

**实现层防护（不在接口内）：** `motion_executor_node.py:113-122`（`_on_goal` 拒绝在途与 unknown）、`executor_core.py:132-138`（core 双重把关）、`task_executor.py:280-288`（execute_callback 二次拒绝）、`pick_place_server.py:521`。

**接口层缺口：**

1. **无 goal 唯一性字段**：ROS 2 Action 的 `goal_id` 由 rclpy/rclcpp 内部生成，**不在 `.action` 文件中**，C++ 与 Python 两侧无法通过 IDL 约定幂等策略。`command_id`/`request_id`/`task_id` 三个 ID 的**唯一性强度、去重窗口、重复时的错误码**均未在接口中定义（`bridge_core.py:343` 的 `command_id_retention: int = 64` 是**实现私有**参数，不可跨语言协商）。
2. **无"在途命令"结构化字段**：`RobotState.motion_active` 只是布尔（`RobotState.msg:8`），`active_command_id` 被迫写入 `detail` 文本（`motion_executor_node.py:210`）。
3. **`RequestStop` 无法指定要停的目标**：无 `command_id` 字段，无法表达"停这条命令"（`RequestStop.srv:5`）。
4. **同话题双发布者（高危）**：`/mtc/robot/state` 由 `motion_executor_node.py:33,105` **和** `bridge_node.py:26,75` 同时发布，而 `RobotState` **没有 source/节点标识字段**。`safety_supervisor.py:44,122` 订阅该话题时无法区分数据来源；`bridge_node.py:129` 硬编码 `motion_active=False`，会以"最后到达者胜出"的方式污染安全监听的输入。这属于接口层缺失（缺 `source`/`node_id` 字段）导致的**并发写冲突**，无法在实现层彻底修复。

---

## 5. 兼容性风险清单

若 C++ 实现**严格按现有字段语义**实现，将遇到以下矛盾：

### R1（高危）`/mtc/robot/state` 多发布者混流，缺 source 字段
- 证据：`motion_executor_node.py:33,105` ↔ `bridge_node.py:26,75`；`bridge_node.py:129` `msg.motion_active = False`。
- 矛盾：C++ 侧订阅者无法判断消息来自执行层还是 Bridge；`motion_active` 在 Bridge 消息中恒假，会覆盖真实运动状态。
- 处理：**必须新增接口能力**（`RobotState` 追加 `string source` / `string node_name`，或拆分为两个话题）。**不得**在 C++ 侧用"最后一条消息"妥协。

### R2（高危）缺多点时间参数化轨迹 Action
- 证据：`CMakeLists.txt:37-41`、`package.xml:16-18`、`ExecuteJointMove.action:3-4,12-13`、`INTERFACE_CHANGELOG.md:54-58`。
- 矛盾：C++/MoveIt 2 产出的 `trajectory_msgs/JointTrajectory` **无接口可送**；强行拆成多次 `ExecuteJointMove` 直接违反该 Action 注释。
- 处理：**必须新增接口**（`ExecuteTrajectory.action` 或采用 `control_msgs/FollowJointTrajectory`）+ 新增 `trajectory_msgs`/`control_msgs` 依赖；并且**必须保持** `ExecuteJointMove` 现有语义不变。

### R3（高危）`ExecuteJointMove` 取消路径无法产出 `stop_confirmed`
- 证据：`motion_executor_node.py:124-126`（cancel 只 ACCEPT）、:128-171（无 `is_cancel_requested` 检查）、`pick_place_server.py:267-295`（依赖 `cancel_goal_async()` 后的 result.stop_confirmed）、`pick_place_fsm.py:1455-1466`。
- 矛盾：C++ 侧若按 `ExecuteJointMove.action:20` 的注释"`stop_confirmed` 仅在后端确认静止后为 true"实现，就必须让取消路径能返回该 result——但当前 `.action` 的 Result 只在 execute 结束产生，取消与结果之间**没有定义任何中间确认消息**。
- 处理：**必须新增接口能力**（最小方案：`ExecuteJointMove` 的停稳确认通道，或新增 `QueryMotionStatus.srv`）；或明确约定取消后由 `RequestStop` + `RobotState` 承担确认（但 R4 的关联缺口必须一并解决）。**必须保持** Result 中 `stop_confirmed` 的"独立证据"语义不被削弱。

### R4（高）`RequestStop` 缺关联 ID，停止请求与停稳确认无法配对
- 证据：`RequestStop.srv:5-8`；`motion_executor_node.py:184-188` 丢弃 `_confirmed`；`interlocks.py:907-915,975-979`。
- 矛盾：C++ 侧无法实现"某次停止请求 → 对应停稳确认"的确定性判定；多源停止无法区分。
- 处理：**必须新增接口字段**（Request 追加 `string request_id` / `string command_id`；Response 追加 `bool stop_confirmed`/`uint16 error_code` 属**不兼容变更**，需版本升级）。**必须保持** `request_delivered ≠ stop_confirmed` 的语义（`safety_supervisor.py:6-8`、`interlocks.py:21-22`）。

### R5（高）Pose 语义在接口→消费侧退化，C++ 侧无法对齐
- 证据：`BatteryDetection.msg:12-13`（PoseStamped）vs `pick_place_fsm.py:190-191`（`Tuple[float,float,float]`）、`pick_place_server.py:140-147`；`TF_CONVENTIONS.md:26-28,94-95`。
- 矛盾：C++ 侧若按 `.msg` 定义使用四元数与 frame_id，则与 Python 消费方（只传 XYZ）产生**语义不对等**；若按 Python 行为丢弃姿态，则违反 `TF_CONVENTIONS.md` 的硬性规则并无法完成 `base_link` 下的 6D 规划。
- 处理：`BatteryDetection` 的 `PoseStamped` 定义**必须保持不变**；**必须**修正 Python 适配层或为 C++ 侧定义明确的 6D 边界。协方差属**必须新增**。

### R6（中）接口常量与实现镜像双轨，且存在与实现不符的文档断言
- 证据：`ExecuteJointMove.action:6-8` 零引用；`ErrorCodes.msg` 仅 1 处软引用（`tool_manager_node.py:41-44`）vs 6 处镜像；`backends.py:33-35` 声称"motion_executor_node 会断言两者一致"但**该断言不存在**；`motion_executor_node.py:24` 死导入。
- 矛盾：C++ 侧引用 `mtc_interfaces::msg::ErrorCodes::EXECUTION_REJECTED` 时，Python 侧用的是 `task_fsm.ErrorCodes`/`interlocks.ERR_*` 等独立定义；任一侧漂移不会编译失败。`execution_state` 走 `ExecState` 枚举而非接口字符串常量。
- 处理：**必须保持不变**的是各错误码**数值**（已一致：100/110/120/200/210/220/230/300/310/320/400/410/420/430/500/510/520）；**应当新增**：把 `ExecState` 的 10 个取值（`backends.py:20-30`）固化为接口常量。

### R7（中）固定长度数组与 NaN 填充
- 证据：`ExecuteJointMove.action:12-13,21,26-27`（`[6]`）；`motion_executor_node.py:147-149,155-156`（`math.nan` 补齐）。
- 矛盾：ROSIDL 固定长度生成 `std::array<T,6>`，**无法表达非 6 关节或部分关节目标**；NaN 经 DDS 传出后，C++ 侧若按"必为有效浮点"处理会得到 NaN 传播。
- 处理：若需支持变长关节集，**必须新增接口**（变长 `sequence<float64>`）；至少**必须保持** `[6]` 语义并在 C++ 侧显式处理 NaN。

### R8（中）服务类型复用导致语义挤压
- 证据：`bridge_node.py:27-29,77-83` 把 `GetMotionCapabilities` 同时用于 `/mtc/bridge/capabilities` 与 `/mtc/bridge/identity`，把 `RequestStop` 用于 `/mtc/bridge/request_stop`；身份校验结论只能写进 `detail` JSON（`bridge_node.py:105-108`）。
- 矛盾：同名类型在不同绑定上语义不同，C++ 侧按类型分发会误判；`detail` 成为事实上的非结构化扩展点。
- 处理：**应当新增**明确类型（如 `GetRobotIdentity.srv`）；`GetMotionCapabilities` 的 6 个字段**必须保持不变**。

### R9（低）`TriggerUnlock.request_id` 恒空 + `accepted` 语义易误用
- 证据：`pick_place_server.py:387`（`getattr(self, "_current_request_id", "")` 恒空）；`TriggerUnlock.srv:3-4,8`；`tool_manager_node.py:168-171`。
- 矛盾：C++ 侧若按 `request_id` 做解锁请求追踪将失效；`accepted=true` 不得被当作"已释放"（现 Python 侧正确遵守，`pick_place_server.py:413-424` 始终返回 `ok=False`）。
- 处理：`accepted` 的"受理≠释放"语义**必须保持不变**；`request_id` 的承载**应当修复**（Python 侧），或新增独立的释放确认接口。

### R10（低）规划接口 `PlanMotion.action` 缺失，规划—执行边界无跨语言契约
- 证据：`CMakeLists.txt:19-20`、`INTERFACE_CHANGELOG.md:43-52,90-99`、`SOURCE_AUDIT.md:116,136`、`pick_place_server.py:160-187`（`RosPlannerPort` 只做透传）。
- 矛盾：C++ 侧规划器与执行层之间**没有已发布的 ROSIDL 契约**；`PickPlace.action` 的 Goal 里也没有规划结果字段。
- 处理：**必须新增**规划接口，或明确 `ExecuteTrajectory` 直接承接 MoveIt 输出。**必须保持**"源缺失时不重定义同名类型"的既有纪律（不得凭文档记述臆造 `PlanMotion` 字段）。

### 5.1 必须新增 vs 必须保持不变（汇总）

| 类别 | 项目 |
|---|---|
| **必须新增接口/类型** | ① 多点时间参数化轨迹 Action（R2，含 `trajectory_msgs`/`control_msgs` 依赖）② 停稳确认的可关联通道或 `RequestStop` 关联 ID（R3/R4，属不兼容变更需版本升级）③ `RobotState` 的 source/节点标识或话题拆分（R1）④ 位姿协方差承载（R5）⑤ 设备身份校验独立类型（R8）⑥ 规划↔执行契约（R10） |
| **必须保持不变** | ① `ExecuteJointMove` 的"单次 6 关节目标 + 到位验证 + 不承诺逐点同步"边界（`ExecuteJointMove.action:3-4`）② `RequestStop.request_delivered ≠ stop_confirmed`（`RequestStop.srv:7-8`）③ `TriggerUnlock.accepted ≠ 已释放`（`TriggerUnlock.srv:8-9`）④ `placement_verified` 仅 VERIFY_PLACED 可置真（`PickPlace.action:14`）⑤ `ToolState.state` 与 `verified` 为独立维度（`ToolState.msg:2,17-19`）⑥ 颜色枚举 0-4 与错误码数值全集（`BatteryDetection.msg:4-8`、`ErrorCodes.msg:6-23`）⑦ `BatteryDetection.body_pose/ring_pose` 保持 `geometry_msgs/PoseStamped`（`BatteryDetection.msg:12-13`）⑧ `ring_pose_valid=false` 禁止生成运动目标（:15）⑨ `GetMotionCapabilities` 只声明"实测启用"能力（`GetMotionCapabilities.srv:3-4`）⑩ 不重定义源缺失的 `PlanMotion.action`/`Latch.srv`（`CMakeLists.txt:19-20`） |

---

## 6. 对 C++ 迁移的强制约束

> 本章是给 C++ 迁移实施的约束条款。**约束 A 为"不得改动"，约束 B 为"建议新增（待审计批准，本阶段不实现）"。**
> 本阶段**不实现任何新增接口**，不修改 `CMakeLists.txt` / `package.xml` / 任何 `.action|.msg|.srv`。

### 6.1 约束 A：必须保持字段与语义不变（硬约束）

C++ 侧**禁止**对下列条目做重命名、类型变更、顺序调整或语义弱化；如需变更，必须先升级 `package.xml` 版本（当前 `package.xml:5` = `0.1.0`）并按 `INTERFACE_CHANGELOG.md:64-69` 走不兼容变更流程。

| 编号 | 约束条目 | 依据 |
|---|---|---|
| A1 | `ExecuteJointMove` 的能力边界不得扩张：它只是"单次 6 关节目标 + 到位验证"，**不得**被改造为轨迹容器，**不得**用高频多次调用伪装轨迹控制 | `ExecuteJointMove.action:3-4` |
| A2 | `ExecuteJointMove.joint_names` 与 `target_rad` 必须按索引一一对应，**禁止假定 J1..J6 固定顺序**；`[6]` 固定长度与校验逻辑（长度一致性、重复项、与配置顺序一致性、限位）必须保留 | `ExecuteJointMove.action:12-13`；`backends.py:192-222` |
| A3 | `ExecuteJointMove.Result.stop_confirmed` 必须只在"后端确认静止"时为 true，**不得**因"收到停止请求"或"goal 被取消"而置真 | `ExecuteJointMove.action:20`；`executor_core.py:203-218` |
| A4 | `RequestStop` 的"送达 ≠ 停稳"语义必须保留：`request_delivered` 只表示送达执行适配层 | `RequestStop.srv:3,7-8`；`safety_supervisor.py:6-8`；`interlocks.py:21-22` |
| A5 | `RequestStop` 必须是"软件停止请求、非紧急停止"，不得被宣传或实现为安全急停的替代 | `RequestStop.srv:2` |
| A6 | `TriggerUnlock.accepted=true` 只表示脉冲请求受理，**不得**表示电池已释放；解锁后的撤离必须有独立证据 | `TriggerUnlock.srv:3-4,8-9`；`pick_place_server.py:413-424` |
| A7 | `TriggerUnlock` 只允许在 VERIFY_SEATED + UNLOAD 之后调用，联锁门不得放宽 | `TriggerUnlock.srv:2`；`tool_manager_node.py:155-161` |
| A8 | `PickPlace.placement_verified` 仅允许由 VERIFY_PLACED 成功路径置真；其余状态必须为 false | `PickPlace.action:14`；`pick_place_fsm.py:114-115,792-798` |
| A9 | `PickPlace.object_id` 在单次执行过程中不得静默切换目标 | `PickPlace.action:3` |
| A10 | `PickPlace.target_slot` 仅允许 `T0/P1/P2/P3`，其他取值必须以 `INVALID_TASK=100` 拒绝 | `PickPlace.action:9`；`task_validation.py:34,183-195` |
| A11 | `ToolState.state` 与 `ToolState.verified` 必须保持**两个独立维度**，不得合并为一个布尔 | `ToolState.msg:2,17-19` |
| A12 | `ToolState.evidence_level` 的四级语义与 `EVIDENCE_COMMAND_ONLY` 不构成独立证据的判定必须保留 | `ToolState.msg:11-14`；`codes.py:64-72` |
| A13 | `ErrorCodes` 全部 18 个常量的**数值**不得改动（0/100/110/120/200/210/220/230/300/310/320/400/410/420/430/500/510/520） | `ErrorCodes.msg:6-23`；与 `codes.py:84-101` 逐项一致 |
| A14 | 颜色枚举 `COLOR_UNKNOWN=0/RED=1/BLUE=2/YELLOW=3/GREEN=4` 不得重新编号；`COLOR_UNKNOWN(0)` 不得作为合法任务口令 | `BatteryDetection.msg:4-8`；`task_validation.py:29-30`；`config_loader.py:702` |
| A15 | `BatteryDetection.body_pose` / `ring_pose` 必须保持 `geometry_msgs/PoseStamped`（含 frame_id、时间戳、四元数），**不得**降级为 `Point`/`Vector3`/裸 `float64[3]` | `BatteryDetection.msg:12-13`；`TF_CONVENTIONS.md:94-95` |
| A16 | `ring_pose_valid=false` 时禁止使用 `ring_pose` 生成运动目标 | `BatteryDetection.msg:15`；`task_executor.py:396-403`；`pick_place_fsm.py:936-941` |
| A17 | 所有业务位姿必须可转换到 `base_link`；变换超时或数据过期必须**拒绝生成运动目标** | `TF_CONVENTIONS.md:26-28` |
| A18 | `GetMotionCapabilities` 的 capability 必须只反映"当前已连接且经实测启用"的能力，**不得**因 SDK 文档声称存在即置 true；`timed_trajectory_supported` 在没有轨迹接口前**必须保持 false** | `GetMotionCapabilities.srv:2-4`；`INTERFACE_CHANGELOG.md:57` |
| A19 | `RobotState` 不得用布尔值丢弃关键安全原因，底层 SDK 原始状态码必须另行记日志 | `RobotState.msg:2-3` |
| A20 | 单位契约不得改动：位置 m、四元数姿态、关节 rad、`*_ms` 毫秒、`*_sec` 秒 | `INTERFACE_CHANGELOG.md:65`；`TF_CONVENTIONS.md:90-102` |
| A21 | 新接口一律使用**不同名称**，禁止与旧工程既有类型同名不同义；**不得**在本阶段创建 `PlanMotion.action` / `Latch.srv` 的臆测字段版本 | `CMakeLists.txt:19-20`；`INTERFACE_CHANGELOG.md:43-52`；`SOURCE_AUDIT.md:136` |
| A22 | 业务接口统一 `/mtc/` 前缀；`/joint_states`、`/tf`、`/tf_static` 保持 ROS 2 常规名称 | `INTERFACE_CHANGELOG.md:68` |
| A23 | 接口包只定义契约，**不得**引入任何真实设备连接能力 | `package.xml:6-9`；`README.md:3` |
| A24 | 编译前必须显式切到 ROS 2 Jazzy；`ROS_DISTRO=humble` 预置状态下产生的任何构建结论**不得**作为验收依据 | `SOURCE_AUDIT.md:41-49,238` |

### 6.2 约束 B：建议新增接口设计（**待审计批准，本阶段不实现**）

> 以下仅为设计提案，用于启动评审。**本次审计不创建任何文件、不修改 `CMakeLists.txt`/`package.xml`。**
> 每一项都标注了它对应解决的缺口编号（§4/§5）与它必须遵守的既有约束。

#### B1 `action/ExecuteTrajectory.action` —— 多点时间参数化轨迹执行
- 对应缺口：4.a、R2；依据 `CMakeLists.txt:37-41`（无此 Action）、`package.xml:16-18`（无依赖）。
- 设计要点（建议）：
  - Goal：`string command_id`；`string trajectory_id`；`trajectory_msgs/JointTrajectory trajectory`（承载 `joint_names`、`points[].positions/velocities/accelerations/effort/time_from_start`）；`float32 velocity_scaling`；`float32 acceleration_scaling`；`uint32 timeout_ms`；`bool require_settle_after`。
  - Result：`bool success`；`bool stop_confirmed`；`uint16 error_code`；`string message`；`float64[] final_position_rad`（**变长**，与 B2 一并决议是否放宽固定 6）。
  - Feedback：`float64[] actual_position_rad`；`float64[] actual_velocity_rad_s`；`float32 trajectory_progress`；`builtin_interfaces/Duration time_from_start_actual`；`string execution_state`。
- 依赖影响：需在 `CMakeLists.txt:11-13` 与 `package.xml:16-18` 增加 `trajectory_msgs`（采用 `FollowJointTrajectory` 时另加 `control_msgs`）。
- 必须遵守：A1（不得改写 `ExecuteJointMove` 语义，二者并存）、A3/A4（停稳与送达分离）、A18（能力位与数据通道必须一致）。
- 备选方案：直接采用 `control_msgs/action/FollowJointTrajectory`（生态兼容性最好，代价是语义由外部包定义，需单独说明与 A21 的关系）。

#### B2 `action/ExecuteJointMove.action` 的变长维度方案（**不兼容变更，需版本升级**）
- 对应缺口：4.a 第 4 点、R7。
- 设计要点：将 `string[6] joint_names` / `float64[6] target_rad` 放宽为 `sequence<string>` / `sequence<float64>`，并保留"长度一致 + 与配置顺序一致 + 限位"校验。
- 影响：破坏现有 `std::array<T,6>`/Python 定长赋值语义，**必须**升级接口版本并行发布，**不得**原地修改。
- 必须遵守：A2、A3。

#### B3 `RobotState` 来源标识（或话题拆分）
- 对应缺口：4.d 第 4 点、R1；依据 `motion_executor_node.py:33,105` 与 `bridge_node.py:26,75` 双发布。
- 设计要点（择一）：
  - 方案 1（追加兼容）：`RobotState` 追加 `string source`（取值如 `motion_executor` / `aubo_bridge` / `safety_supervisor`）与 `string source_instance_id`。
  - 方案 2（拆分）：执行层与 Bridge 分别发布到 `/mtc/robot/state` 与 `/mtc/bridge/state`，`RobotState` 定义不变。
- 必须遵守：A19；且必须解决 `bridge_node.py:129` 硬编码 `motion_active=False` 的语义冲突。

#### B4 `srv/QueryMotionStatus.srv` —— 停稳/在途状态查询（含命令身份）
- 对应缺口：4.c 缺口 1/3、4.d 第 2 点、R3。
- 设计要点（建议）：
  - Request：`string command_id`（可空，表示"当前在途命令"）。
  - Response：`bool motion_active`；`string active_command_id`；`string execution_state`；`bool stop_confirmed`；`bool motion_state_unknown`；`uint16 error_code`；`string detail`。
- 作用：为"取消 → 停稳确认"提供**可关联、可轮询**的确定性通道，替代解析 `RobotState.detail` 文本。
- 必须遵守：A3、A4、A13（`error_code` 复用现有数值）。

#### B5 `srv/RequestStop.srv` 关联标识（**不兼容变更，需版本升级**）
- 对应缺口：4.c 缺口 2、R4。
- 设计要点：Request 追加 `string request_id`、`string command_id`（可为空）；Response 追加 `bool stop_confirmed`、`uint16 error_code`。
- 影响：Response 追加字段对 Python/C++ 消费者为**不兼容变更**（现有 `interlocks.py:733-734` 用 `hasattr` 探测，需同步评审），**必须**版本升级并同步更新 `INTERFACE_CHANGELOG.md`。
- 必须遵守：A4、A5。

#### B6 `srv/GetRobotIdentity.srv` —— 设备身份校验独立类型
- 对应缺口：R8；依据 `bridge_node.py:27-29,77-83,97-109` 的类型复用与 `detail` JSON 挤压。
- 设计要点：Response：`bool identity_ok`；`string controller_model`；`string serial_number`；`bool identity_verified`；`string reason`；`string detail`。
- 必须遵守：A18（能力声明与身份校验必须分开）、A21。

#### B7 位姿协方差承载
- 对应缺口：4.b 协方差、R5。
- 设计要点（择一）：
  - 方案 1：`BatteryDetection` 追加 `geometry_msgs/PoseWithCovarianceStamped ring_pose_cov`（**仅追加，不改动现有两字段**，对应 A15 的"保持不变"）。
  - 方案 2：追加 `float64[36] ring_pose_covariance` 与 `float64[36] body_pose_covariance`。
- 必须遵守：A15、A16、A17（协方差有效性需参与"是否拒绝生成运动目标"的判定）。

#### B8 `action/PlanMotion.action` 或规划↔执行契约
- 对应缺口：R10；依据 `CMakeLists.txt:19-20`、`INTERFACE_CHANGELOG.md:43-52`。
- 设计要点：**本阶段不设计字段**。按 `INTERFACE_CHANGELOG.md:90-99` 的补齐清单，须先拿到旧工程 `PlanMotion.action` 真实源码、计算 SHA256、逐字段对照后再决策（原样保留 / 追加兼容 / 版本并行）。
- 必须遵守：A21（**严禁**在无源码时臆造字段）。

#### B9 `ExecuteJointMove` 执行状态枚举固化（收敛 R6）
- 对应缺口：3.1 首行（`COMMAND_*` 零引用）、R6。
- 设计要点：把 `backends.py:20-30` 的 10 个 `ExecState` 取值（`IDLE/ACCEPTED/MOVING/SETTLED/CANCELLING/STOPPED/STOP_UNKNOWN/TIMEOUT/REJECTED/FAULT`）固化为接口常量，并决议与现有 `COMMAND_OK/REJECTED/UNKNOWN`（`ExecuteJointMove.action:6-8`）的关系（保留 / 废弃 / 映射）。
- 影响：若废弃 `COMMAND_*` 属**不兼容变更**，需版本升级。
- 必须遵守：A13（数值不变）。

### 6.3 约束执行顺序建议

1. 先做**零风险**动作：B3 方案 1（追加字段）、B7 方案 1（追加字段）、B6（新增类型）——均为纯追加或新增，不影响既有语义。
2. 再做**需新增依赖**的动作：B1（轨迹 Action）。
3. 最后做**不兼容变更**：B2、B5、B9——必须先升级 `package.xml` 版本（`package.xml:5`）并在 `INTERFACE_CHANGELOG.md` 逐条登记（依据 `INTERFACE_CHANGELOG.md:64`）。
4. B8 在拿到旧源码前**一律挂起**（依据 `SOURCE_AUDIT.md:215-217` R3）。

---

## 7. 编译与运行状态声明（NOT RUN）

| 验证项 | 状态 | 说明 |
|---|---|---|
| 静态只读审计（read/grep/glob） | **DONE** | 本次审计的全部证据来源 |
| `colcon build --packages-select mtc_interfaces` | **NOT RUN** | 本机无 ROS 2 Jazzy；按约定不做编译验证（`SOURCE_AUDIT.md:41-49`） |
| `colcon build --symlink-install`（全仓） | **NOT RUN** | 同上 |
| `ros2 interface show mtc_interfaces/...` | **NOT RUN** | 同上（`README.md:74`、`INTERFACE_CHANGELOG.md:109`） |
| C++ 消息类型生成 / 编译 | **NOT RUN** | 无 C++ 消费方代码，未生成 |
| Python 消息类型 import 验证 | **NOT RUN** | 未加载任何 `mtc_interfaces.*` 类型（`INTERFACE_CHANGELOG.md:110`） |
| 任何节点启动 | **NOT RUN** | 未启动 `task_executor` / `pick_place_server` / `motion_executor` / `safety_supervisor` / `tool_manager` / `mtc_aubo_bridge` |
| 任何真实设备连接 | **NOT RUN** | 未连接 AUBO S3、未触发任何 IO |
| `colcon test` | **NOT RUN** | 同上 |

> **严禁**将上述任何 `NOT RUN` 表述为"编译成功""接口可用""已验证"。
> 复现命令见 `SOURCE_AUDIT.md:225-238`（须先显式切换到 ROS 2 Jazzy）。
> 本报告新增内容**未改动**任何接口文件、构建文件或 Python 源码；本次审计对 `src/` 与 `config/` 的写入次数为 **0**。

---

## 8. 结论摘要

| 审计问题 | 结论 | 决定证据 |
|---|---|---|
| 4.a 多点轨迹执行能力 | **缺少** | `CMakeLists.txt:37-41`（仅 3 个 Action）、`package.xml:16-18`（无 trajectory_msgs/control_msgs）、`ExecuteJointMove.action:3-4,12-13`（固定 6 维 + 禁止拆分伪装）、`INTERFACE_CHANGELOG.md:54-58` |
| 4.b 6D Pose/frame_id 保真度 | 接口层 **部分具备**；Python 消费侧 **已退化为纯 XYZ**；协方差 **缺少** | `BatteryDetection.msg:12-13`（具备 PoseStamped）vs `pick_place_fsm.py:190-191`、`pick_place_server.py:140-147`（丢 orientation/frame_id）；`TF_CONVENTIONS.md:94-95`；全仓库 frame_id 仅 3 处且无一处读取 ROS 消息 |
| 4.c 取消→停稳确认链路 | **缺少**（链路断裂） | `motion_executor_node.py:124-126`（cancel 仅 ACCEPT）、:128-171（无 `is_cancel_requested`）、`RequestStop.srv:5-8`（无关联 ID）、`pick_place_server.py:267-295` 依赖跨不过去的 `result.stop_confirmed` |
| 4.d 并发 Goal 冲突防护 | 接口层 **缺少**；且存在同话题双发布者缺陷 | 仅注释 `ExecuteTask.action:3`；`command_id`/`request_id` 无幂等语义；`RobotState` 无在途命令字段（被迫写 `detail`，`motion_executor_node.py:210`）；`/mtc/robot/state` 双发布（`motion_executor_node.py:33,105` ↔ `bridge_node.py:26,75`） |
| 未使用字段 | `ExecuteJointMove.COMMAND_*` **零引用**；`ErrorCodes.msg` **事实未作为契约**（6 处镜像）；`TaskState` 全字段、`ExecuteTask.Feedback` 全字段、所有 `header.frame_id` **只写不读** | 见 §3 |
| 文件修改 | **0 次**（全程仅 read/grep/glob，未使用 bash） | — |
| 编译状态 | **NOT RUN** | `SOURCE_AUDIT.md:41-49`、`INTERFACE_CHANGELOG.md:105-112` |
| 运行状态 | **NOT RUN** | 未启动任何节点、未连接任何设备 |

**一句话结论：** 现有 `mtc_interfaces` 是一套**面向"单次 6 关节位运动 + 布尔级状态"的 MVP 契约**——它在"送达≠停稳""受理≠释放""state≠verified"这三条安全语义上做得正确且明确，但**在多点时间参数化轨迹（4.a）、6D 位姿与协方差承载（4.b）、可关联的取消—停稳确认（4.c）、以及接口级并发防护与状态来源标识（4.d）四个方向上均存在明确缺口**；其中 4.a 与 4.c 属于"沿现有接口无法实现"，必须新增接口；而 §6.1 约束 A 列出的 24 条字段与语义，则**必须原样保持不变**。所有新增接口设计见 §6.2，**待审计批准，本阶段不实现**。
