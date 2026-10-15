# PLANNING_HANDOFF —— MoveIt 2 运动规划交接（10/11 晚 P0）

负责人：常建烁｜分支：`feature/moveit-planner`｜包：`src/mtc_moveit_planner`｜通俗说明：[README.md](README.md)

> **适用范围：Mock / Fake Hardware，只规划、不执行。** 本文所有"PASS"都只针对
> `mock_components/GenericSystem` 模拟环境中的规划结果，**不代表实机可执行**。
> TCP、工位、提环位姿均为 PLACEHOLDER；输出强制 `simulation_only=true`。

## 0. 结论速览

证据目录：[`evidence/20261011_075828/`](evidence/20261011_075828/)（Fake Hardware + move_group 实际运行，2026-10-11 20:59 CST）。

| 交付项（任务书 03） | 结果 | 证据 |
|---|---|---|
| 1. 配置核验（group / model / 关节 / frame / EE / 控制器） | **PASS**（源码 + 运行时） | §1；`ros_graph.txt`、`frames_*.pdf`、`tf2_echo_*.txt`、`aubo_S3_fake_expanded.urdf`、`aubo_robot.srdf` |
| 2. 固定 6D Pose → 完整时间参数化轨迹 | **PASS** | `fixed_pose_plan_*.yaml`；§2 |
| 3. PlannerPort 转换层 + 映射 | **PASS**（29 个 GTest；真实 PickPlaceFsm 接受并整条派发） | `src/mtc_moveit_planner/`；§3、§7 |
| 4a. 单块电池运动阶段 —— V2 `magnetic_latch_v2` → T0 | **PASS（仅仿真意图）**：7 段全部由 MoveIt 规划 | `battery_sequence_magnetic_latch_v2_*.yaml`；§4 |
| 4b. 单块电池运动阶段 —— V1 `passive_hook_v1` → T0 | **FAIL**：`side_insert` 116 s > MOVE_PRE_ALIGN 看门狗 30 s×80%（原语限速 0.02 m/s 所致，见 R6） | `battery_sequence_passive_hook_v1_*.yaml` |
| 碰撞场景 / TCP 可信 | **不可信**（仅自碰撞；TCP=单位变换） | §5 |
| 真实机械臂执行 | **NOT RUN**（未授权；本包不执行任何轨迹） | — |

## 1. 配置核验（任务书 03-1）

核验来源：固定版本源码（静态）+ 本机 Fake 环境运行时输出（动态，见 §6 证据目录）。

| 项目 | 核验值 | 来源 / 证据 |
|---|---|---|
| 驱动源码 | `AuboRobot/aubo_ros2_driver` jazzy @ `53aab83735236f0c262ecb11e04ef2600241d9d9` | 与 NUC 验证环境同一提交 |
| description 子模块 | `AuboRobot/aubo_description` @ `47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1` | 同上 |
| Fake 补丁 | `docs/migration/patches/aubo_description_fake_hardware_jazzy.patch`，打补丁后 `aubo.ros2_control.xacro` SHA256 = `c0f6311b…e52f6`，与归档 `locally_fixed` **逐字节一致** | `cmp` + `sha256sum` |
| robot model | `aubo_description/urdf/xacro/inc/aubo_ros2.xacro`，参数 `aubo_type:=aubo_S3 use_fake_hardware:=true` | 展开结果存于证据目录 `aubo_S3_fake_expanded.urdf` |
| SRDF | `aubo_moveit_config/config/aubo_robot.srdf`（launch 默认；**不是** `aubo_i5.srdf`） | `aubo_moveit.launch.py` |
| Planning Group | `manipulator`，chain `base_link → ee_link` | SRDF L12-13 |
| **Planning frame** | **`world`**（运行时 `getPlanningFrame()` 返回值；与 `base_link` 重合） | 规划工具运行时 `MoveGroupInterface::getPlanningFrame()` |
| base frame | `base_link`；URDF 根为 `world`，`world_joint` 为单位固定变换 ⇒ `world ≡ base_link` | URDF + `tf2_echo world base_link` |
| **EE link** | `ee_link`：父 `wrist3_Link`，xyz=0，rpy=(0,0,π/2)。**腕部坐标，不含任何工具** | `aubo_ros2_macro.xacro` L6-9 |
| 关节（FSM 顺序） | `shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint` | 驱动 `aubo_controllers.yaml`、`PickPlaceConfig` |
| 关节限位（rad，URDF，MoveIt 使用） | shoulder ±2π；upperArm ±π；foreArm ±2.8798；wrist1/2/3 ±2π | `aubo_S3.urdf` |
| 关节限位（rad，ros2_control 指令接口） | shoulder ±2π；upperArm ±2π；**foreArm ±2.41；wrist1 ±2.66**；wrist2 ±2π；**wrist3 ±2.23** | `aubo.ros2_control.xacro`；本包对轨迹逐点复核，超限 ⇒ `JOINT_LIMIT_VIOLATION` |
| 控制器 | `joint_trajectory_controller`（JTC），Action `/joint_trajectory_controller/follow_joint_trajectory`（`control_msgs/action/FollowJointTrajectory`）；`joint_state_broadcaster` | 驱动 yaml + 运行时 `list_controllers` |
| MoveIt 控制器映射 | `moveit_controllers.yaml` 中关节列举为 shoulder, **foreArm, upperArm**, … —— 与 JTC 顺序不同，**必须按名称映射** | 源码 |
| IK | KDL，`timeout=0.005 s`，`attempts=3` | `kinematics.yaml` |
| 规划管线 | 仅 OMPL（本包用 `RRTConnectkConfigDefault`）；**未加载 Pilz**，直线段用 `computeCartesianPath` | `aubo_moveit.launch.py` |
| MoveIt joint_limits | 默认 velocity scaling 1.0、acceleration scaling 0.1 —— 本包显式使用 0.1/0.1 | `joint_limits.yaml` |

**发现的不一致（已处理或需团队知悉）：**

1. SRDF 的 `virtual_joint fixed_base: world → base_link` 与 URDF 自带的 `world` link 重复：
   URDF 根已是 `world`，MoveIt 会跳过该 virtual joint，planning frame 仍为 `world`。不影响规划。
2. `docs/architecture/TF_CONVENTIONS.md` 中的法兰坐标系 `tool0` **在实际 URDF 中不存在**，应以 `ee_link` 为准。
3. `aubo_moveit.launch.py` 的 `aubo_type` 默认是 `aubo_i5`，**必须显式传 `aubo_type:=aubo_S3`**。
4. MoveIt 按 URDF 限位规划，但 `aubo.ros2_control.xacro` 中 foreArm/wrist1/wrist3 的指令限位更窄；
   本包在输出前按更窄限位逐点复核。两者都不是实机软限位证明。
5. `ee_link` 与 `wrist3_Link` 原点重合；它是否正好位于物理法兰安装面，需对照 AUBO 手册 / 实物核对（韩明赫）。

## 2. 固定 6D Pose 规划（任务书 03-2）

**目标**：SRDF 命名状态 `home` 的 `ee_link` 正运动学位姿（保证可达、可复现）：
`world` 中 position = (0.35637, −0.13259, 0.37908) m，orientation(xyzw) = (0.99945, 0.00001, −0.03305, 0.00035)。
**起点**：Fake Hardware 初始状态（六关节 0 rad，SRDF `up`）。**调用**：`MoveGroupInterface::plan()`，
pipeline `ompl` / `RRTConnectkConfigDefault`，velocity/acceleration scaling 0.1/0.1，规划时限 1.0 s，
控制器限位以路径约束下发，结果经 TOTG 时间参数化。

| 项 | 实测 |
|---|---|
| 返回状态 | `SUCCESS` |
| 规划耗时 | 0.042 s |
| 轨迹 | 159 点，时长 15.73 s，含 `time_from_start` / positions / velocities / accelerations |
| 终点误差 | 位置 9.9e-5 m，姿态 9.3e-4 rad |
| 末端最大速度 | 0.074 m/s，0.339 rad/s |
| 关节顺序 | `shoulder, upperArm, foreArm, wrist1, wrist2, wrist3`（= FSM 顺序），`header_frame_id=world` |

完整轨迹：[`evidence/20261011_075828/fixed_pose_plan_20261011_075906.yaml`](evidence/20261011_075828/fixed_pose_plan_20261011_075906.yaml)。

**观察（轨迹有效性）**：终点 `wrist2_joint = −4.713 rad`，而 SRDF `home` 为 `+1.570 rad`——末端位姿相同，
IK 选了相差 2π 的另一分支（腕部多转 270°）。OMPL 为随机规划，多次运行时长在 5.9–20.9 s 间波动。
固定工位建议改用**关节空间目标**或带种子的 IK，以获得可重复的轨迹。

## 3. PlannerPort 字段映射（任务书 03-3）

实现：`src/mtc_moveit_planner/src/moveit_planner_port.cpp`（`MoveItPlannerPort`）。
**未改动** `mtc_interfaces` 与 FSM Core 任何公共头文件。

### 3.1 输入 `PlanTarget` → 规划请求

| `PlanTarget` 字段 | 用途 |
|---|---|
| `phase` | 仅日志/去重键（FSM 发 `ACQUIRE` / `TRANSPORT`） |
| `object_id`、`slot` | 去重键；`slot` 用于查工位位姿（`T0/P1/P2/P3`） |
| `ring_pose` / `primitives[i].target_pose` | 提环 6D 位姿（保留 `frame_id`、四元数）；经 **tf2** 变换到 planning frame，变换失败 ⇒ `TF_UNAVAILABLE`，**绝不复制 XYZ** |
| `primitives[i].name` | 查原语规则表（§4）；ToolPort 原语直接拒绝 |
| `primitives[i].max_linear_speed / max_angular_speed` | 末端线速度/角速度上限（时间均匀拉伸） |
| `primitives[i].requires_contact` | 原样写入 `MotionCommand.requires_contact` |
| `primitives[i].target_label` | 工位类原语与 `slot` 交叉校验（不一致 ⇒ `SLOT_MISMATCH`） |

> 注意：`transport` 原语的 `target_pose` 是**抓取时的提环位姿**（`ToolStrategy::make_transport_plan` 由
> `make_primitive(…ring_pose…)` 构造），目的地只在 `target_label="T0"`。本包对工位类原语**只按工位名查表**，
> 不使用其 `target_pose`。

### 3.2 输出 `PlanResult` / `MotionCommand`

| 字段 | 取值 |
|---|---|
| `PlanResult.ok` | 全部原语成功才为 true |
| `PlanResult.error_code` | 成功 `OK(0)`；任何失败 `PLANNING_FAILED(200)` |
| `PlanResult.message` | 成功：帧/EE/`simulation_only`/未标定输入；失败：`[原因] 原语: 详情` |
| `PlanResult.primitives` | 原样回传请求原语 |
| `PlanResult.commands[i]` | 与 `primitives[i]` 一一对应 |
| `MotionCommand.command_id` | 空（FSM `make_command()` 派发时赋值，并写入 `trajectory_id`） |
| `MotionCommand.trajectory` | **整条** `TimedTrajectory`：关节名（FSM 顺序）、`points[].time_from_start_s / positions_rad / velocities_rad_s / accelerations_rad_s2`、`header_frame_id=world`、`stamp_s` |
| `MotionCommand.target_rad` | 空（有轨迹时 FSM 要求为空） |
| `velocity_scaling / acceleration_scaling` | 0.1 / 0.1（上限 0.2） |
| `timeout_ms` | `ceil(时长 × 1.5 × 1000) + 1000` |
| `simulation_only` | **true**（`force_simulation_only` 或任一几何输入非 MEASURED） |

### 3.3 失败原因码

`NO_CURRENT_STATE`、`EMPTY_TARGET`、`INVALID_CONFIG`、`UNKNOWN_PRIMITIVE`、`TOOL_OWNED_PRIMITIVE`、
`INVALID_PRIMITIVE`、`TARGET_POSE_MISSING`、`TF_UNAVAILABLE`、`TCP_UNKNOWN`、`SLOT_POSE_UNKNOWN`、
`SLOT_NOT_CALIBRATED`、`SLOT_MISMATCH`、`FK_FAILED`、`NO_PLAN`（附 MoveIt 错误码）、`CARTESIAN_INCOMPLETE`、
`TIME_PARAMETERIZATION_FAILED`、`TOOL_GAP_SIMULATION_FAILED`、`INVALID_TRAJECTORY`、`START_STATE_MISMATCH`、
`EXCEEDS_STATE_WATCHDOG`、`JOINT_LIMIT_VIOLATION`。全部映射为 `PLANNING_FAILED(200)`，不静默重试。

### 3.4 异步语义

`plan()` 首次调用启动后台规划并返回 `nullopt`；FSM 下一次 `step()` 以同一目标再调用时返回结果。
同一目标（阶段/对象/工位/原语/提环位姿）只规划一次；ROS 回调与 FSM 事件线程不阻塞等待 MoveIt。

## 4. 单块电池基础任务运动阶段（任务书 03-4）

FSM 实际发出两次规划请求（与 `PickPlaceFsm::h_plan_acquire` / `h_lift` 一致）：

| # | 原语 | FSM 状态 | 方式 | 目标 | 起点 | 性质 |
|---|---|---|---|---|---|---|
| ACQUIRE-1 | `side_insert`(V1) / `top_insert`(V2) = **预抓取+接近** | MOVE_PRE_ALIGN | OMPL | 提环沿插入轴后退 0.03 m | 当前状态 | 可规划（仿真） |
| — | `hook_engage` / `auto_lock` = 接合 | ENGAGE | ToolPort | — | — | **不由规划器输出** |
| ACQUIRE-2 | `test_lift` = **试提** | TEST_LIFT | 直线 +Z 0.01 m | — | **假设插入 0.03 m 后** | 接触相关，仅仿真 |
| TRANSPORT-1 | `lift` = **抬起** | LIFT | 直线 +Z 0.05 m | — | 当前状态 | 可规划（仿真） |
| TRANSPORT-2 | `transport` = **搬运至 T0** | TRANSPORT | OMPL | 工位上方 0.10 m，保持姿态 | 上一段终点 | 可规划（仿真；未建电池附着体） |
| TRANSPORT-3 | `pre_seat` = **下降** | MOVE_PRE_SEAT | 直线 | 工位上方 0.02 m | 上一段终点 | 可规划（仿真） |
| TRANSPORT-4 | `seat_contact` = **落座** | SEAT | 直线 | 工位高度 | 上一段终点 | **接触意图，仅仿真** |
| — | `unload` / `side_withdraw` / `withdraw` | UNLOAD / DISENGAGE | ToolPort | — | — | **不由规划器输出** |
| TRANSPORT-5 | `retreat` = **撤离** | RETREAT | 直线 +Z 0.10 m | — | **假设退出 0.03 m 后** | 可规划（仿真） |

**实测（起点 = SRDF `home`，提环占位位姿 `base_link` (0.40, −0.05, 0.20)，T0 占位 (0.35, −0.20, 0.10)）：**

V2 `magnetic_latch_v2`（插入方向朝下，ring quat = (1,0,0,0)）—— **全部 7 段 PASS**：

| 段 | 方法 | 规划耗时 | 时长 | 点数 | 时间拉伸 | 末端最大线速度 | 看门狗 |
|---|---|---|---|---|---|---|---|
| `top_insert` | 末端直线（优先） | 0.007 s | 13.18 s | 35 | ×3.93 | 0.020 m/s | 30 s |
| `test_lift` | 直线 | 0.001 s | 1.88 s | 9 | ×2.52 | 0.010 m/s | 15 s |
| `lift` | 直线 | 0.002 s | 3.28 s | 18 | ×1.99 | 0.029 m/s | 30 s |
| `transport` | 末端直线（优先） | 0.005 s | 2.25 s | 24 | ×1.00 | 0.134 m/s | 60 s |
| `pre_seat` | 直线 | 0.003 s | 5.13 s | 23 | ×2.40 | 0.029 m/s | 30 s |
| `seat_contact` | 直线（**接触意图**） | 0.001 s | 7.75 s | 12 | ×7.27 | 0.005 m/s | 20 s |
| `retreat` | 直线 | 0.003 s | 5.86 s | 26 | ×2.44 | 0.029 m/s | 30 s |

V1 `passive_hook_v1`（侧向插入，插入方向 +X）—— **FAIL**：`side_insert` 末端直线不可行，回退 OMPL；
OMPL 原始轨迹按 0.1 缩放约 11–13 s，但为满足原语限速 0.02 m/s / 0.1 rad/s 需拉伸 ×7–10，
得 91–196 s（多次尝试、多种占位姿态），均超过 MOVE_PRE_ALIGN 看门狗 30 s 的 80%，按设计拒绝输出（`EXCEEDS_STATE_WATCHDOG`）。
→ 见风险 R6，需要孙嘉明决定（拆分"快速自由空间接近 + 慢速预对准"，或调整限速/看门狗）。

修复过程中的历史失败（已解决，留档）：首轮 V1 OMPL 解 `wrist1 = 2.688 rad` 超出控制器限位 ±2.66
（`JOINT_LIMIT_VIOLATION`）→ 改为把控制器限位作为路径约束下发后不再出现。

## 5. 碰撞场景与 TCP 可信度

| 项目 | 当前状态 | 可信度 |
|---|---|---|
| 碰撞场景 | 仅机器人自身（SRDF 自碰撞矩阵）。**无**桌面、底图、电池、工具、相机、工位支架 | ❌ 不可信：不能证明真实场景无碰撞 |
| 电池附着体 | 未建（接合确认后应 attach，释放后 detach，见 TF_CONVENTIONS §1 规则 4） | ❌ |
| TCP | `ee_link → hook_tcp` = 单位变换（PLACEHOLDER）；规划实际把 `ee_link` 送到提环处 | ❌ 不可信 |
| 工位 T0/P1/P2/P3 | 规划层占位坐标（`config/planner.yaml`），非实测 | ❌ 不可信 |
| 提环轴向 | 采用 TF_CONVENTIONS §2.1 **提案**：`+Z` = 插入方向 | ⚠️ 待确认 |
| 速度/距离 | `motion_profiles.yaml` / `ToolStrategy` 常量，全部占位 | ⚠️ 待标定 |

## 6. 复现命令与证据

环境：Ubuntu 24.04（WSL2）、ROS 2 Jazzy、MoveIt 2.12.4、ros2_control 4.48.1、GCC 13。

```bash
# 0) 系统依赖（一次性，需要 sudo）
bash src/mtc_moveit_planner/scripts/setup_ros_jazzy.sh   # 包列表同 docs/setup/AUBO_ROS2_DRIVER_JAZZY.md §2 的包列表

# 1) 驱动工作空间（独立目录；固定提交 + 归档 Fake 补丁，脚本先核对指纹）
export AUBO_WS=~/meituan/aubo_s3_mock_ws MTC_WS=~/meituan/meituan_challenge_ws
git clone --branch jazzy https://github.com/AuboRobot/aubo_ros2_driver.git $AUBO_WS/src/aubo_ros2_driver
git -C $AUBO_WS/src/aubo_ros2_driver switch --detach 53aab83735236f0c262ecb11e04ef2600241d9d9
git -C $AUBO_WS/src/aubo_ros2_driver submodule update --init --recursive
git -C $AUBO_WS/src/aubo_ros2_driver/aubo_description apply \
  $MTC_WS/docs/migration/patches/aubo_description_fake_hardware_jazzy.patch
bash $MTC_WS/src/mtc_moveit_planner/scripts/build_driver_ws.sh

# 2) FSM Core + 规划包 + GTest
bash $MTC_WS/src/mtc_moveit_planner/scripts/build_planner.sh

# 3) 一键：Fake 控制 + move_group → 只读采集 → 规划 → 退出（只结束自己的进程组）
MTC_RMW=rmw_cyclonedds_cpp bash $MTC_WS/src/mtc_moveit_planner/scripts/run_planning_evidence.sh
```

**环境偏差（如实记录）**：本机 WSL2 `.wslconfig` 为 `networkingMode=mirrored`，Fast DDS 同机节点发现不稳定
（`controller_manager` 收不到 `/robot_description`，4 次启动仅 1 次成功）；`ros2` CLI 连接本机 daemon 端口超时。
因此本机证据使用 `rmw_cyclonedds_cpp`（2/2 启动成功）并以 rclpy 只读快照替代 CLI。**NUC 按 SOP 使用 Fast DDS**；
规划结果与 RMW 无关。RMW 由 `MTC_RMW` 环境变量选择，默认仍为 `rmw_fastrtps_cpp`。

## 7. 测试

```text
colcon test (build/planner)：
  mtc_moveit_planner        test_pose_math 5 / test_trajectory_conversion 4 /
                            test_moveit_planner_port 16 / test_fsm_accepts_planner_output 4  → 29 GTest，0 失败
  mtc_core_cpp 25、mtc_motion_execution_cpp 93、mtc_manipulation_cpp 126（colcon test-result 计数）→ 0 失败
```

`test_fsm_accepts_planner_output` 用 main 上的**真实 `PickPlaceFsm`**（感知/工具/运动为纯逻辑 Fake，规划为本包 + 假后端）：
V1、V2 单块电池到 T0 均达 `SUCCESS`，规划输出全部通过 `accept_plan()`，每个运动原语以整条轨迹经
`MotionPort::execute_trajectory()` 派发，`execute_joint_move()` 调用次数为 0；规划失败时不发生任何运动。
另含异步规划（`nullopt` → 结果，不重复规划）在 PLAN_ACQUIRE 2 s 看门狗内完成的用例。

## 7A. 第二轮（feature/planning-scene-start-v1）：规划场景、起点一致性、V1

| 项 | 做法 | 结果 |
|---|---|---|
| 静态规划场景 | `config/planner.yaml` 的 `scene.boxes`（PLACEHOLDER 桌面：顶面 z=-0.005）经 `/apply_planning_scene` 写入；对象 ID 带前缀 `mtc_moveit_planner/`，只增删该前缀的对象（见 §7B） | 碰撞反证（目标在桌面内部）**被拒绝** ✅ |
| 电池载荷 | 从 `test_lift` 起到 `seat_contact`，在**规划起点 RobotState** 上附着长方体（PLACEHOLDER 0.06×0.06×0.10 m，挂在 TCP 正下方 0.07 m），不修改全局场景；`retreat` 起不再附着 | V2 全流程在桌面 + 载荷下 7 段 PASS ✅ |
| 落座几何 | 工位/提环占位高度改为 0.13 m：载荷底面距桌面 1 cm；**最后的接触闭合不由规划完成**（需力控/柔顺，属执行与工具） | — |
| 起点一致性 | 新增公共函数 `check_trajectory_start()`（按名称比较首点，容差默认 0.01 rad，与 MoveIt allowed_start_tolerance 一致）与 `MoveItPlannerPort::replan_primitive()`（以实测状态为起点、不再假设 ToolPort 行程，只重规划一段） | 供 FollowJointTrajectory 适配在发送前调用 ✅ |
| V1 规划 | FREE_SPACE 依次：末端直线 → 以起点为种子求 IK（候选解须满足控制器限位）→ **关节空间直线插值，逐点经 `/check_state_validity` 校验（含桌面与载荷）** → OMPL 到该关节目标 → 位姿目标 OMPL | `side_insert` 从 116–196 s（OMPL 随机）降到 39.6–42.1 s，仍 > 24 s（30 s×80%）❌。**更正**：此轮"可重复"的结论不成立，见 §7B |

证据：[`evidence/20261011_113932/`](evidence/20261011_113932/)（提交 `63d24e3`）。另：固定 6D Pose 规划现在同样走"种子 IK + 关节插值"，
结果确定（5.91 s / 61 点，终点误差 3.7e-6 m），解决了第一轮"IK 分支跳变、OMPL 时长 5.9–20.9 s 波动"的问题。

**V1 仍阻断的量化原因**（第四轮更正：占位几何下实际需转 180°，下限 31.4 s，见 §7C）：从 `home`（工具朝下）到侧插预对准（工具水平）需转 90°，原语角速度上限 0.1 rad/s
⇒ 理论下限 15.7 s；末端直线只可行 50%（fraction=0.506），改用关节空间插值后转动不均匀，峰值角速度 0.098 rad/s 已贴上限，得 39.59 s（§7B 后稳定）。
**未放宽任何安全参数**。可行方向（需孙嘉明决定）：V1 增加"工具已水平"的就绪位，或把"快速接近"与"慢速预对准"拆成两个原语。

## 7B. 第三轮（#4 评审意见）：重复关节名、场景对象 ID 隔离、V1 选解确定化

基于合并 #3、#6 后的最新 `main`（`9fea32e`）全新构建复测。

| 项 | 做法 | 结果 |
|---|---|---|
| 起点检查拒绝重复关节名 | `check_trajectory_start()` 对轨迹 `joint_names` 与实测 `names` 先查重复 / 空名，出现即判不通过（消息 `duplicate joint name …`），不"取第一个"；`JointStateSnapshot::shape_is_valid()` 同样拒绝，`replan_primitive()` / `plan_blocking()` 收到重名起点返回 `[INVALID_START_STATE]` 且不调用后端 | GTest ✅（`DuplicateOrEmptyJointNamesAreRejected`、`ReplanFromStateWithDuplicateJointNamesIsRefused`） |
| 场景对象 ID 隔离 | 新增 `scene_ids.hpp`：所有对象加前缀 `mtc_moveit_planner/`；配置写本地 ID，校验字符集 `[A-Za-z0-9_.-]`、不重复、不得自带前缀（参数加载即报错）；`apply_scene` 只移除"带本前缀且本次不需要"的对象（含进程重启前遗留的），**绝不触碰不带前缀的对象**；附着电池 ID 改为 `mtc_moveit_planner/battery_payload` | GTest ✅（3 项）；Fake move_group 反证 ✅：预置其他模块的 `table`、`perception/battery_3` 与本模块遗留 `mtc_moveit_planner/old_fixture`，规划后前两者原样保留、遗留被清除、本模块桌面为 `mtc_moveit_planner/table`（`scene_isolation.log`） |
| V1 选解确定化 | 复测发现上一轮"39.6–42.1 s 可重复"不成立：同一请求有时 123 s——单次 KDL IK 在种子不收敛时随机重启，落到另一组解。改为收集候选（位姿延拓 + 起点与腕关节 ±90° 组合的 27 个固定种子），按"关节插值在 TOTG + 末端限速下的预计时长"排序后逐个做逐点碰撞校验。注意关节跨度最小≠时长最短（跨度 3.45 rad 的解 123 s，2.79 rad 的解 39.6 s） | 最终代码连续 5 次 Fake 运行均为 **39.59 s**；`side_insert` 规划耗时 0.41–0.51 s（PLAN_ACQUIRE 看门狗 2 s）；仍 > 24 s ❌，保留为已知阻断，**未改任何安全参数** |

测试：`mtc_moveit_planner` **40 项 GTest**（原 35 + 新增 5）全部通过；与 `mtc_core_cpp`、`mtc_task_cpp`、`mtc_motion_execution_cpp`、
`mtc_manipulation_cpp`、`mtc_ros2_action_cpp` 合计 424 项测试 0 失败。
证据：[`evidence/20261011_204211/`](evidence/20261011_204211/)（提交 `5b16415`）：固定位姿 PASS（5.91 s / 61 点，终点误差 3.7e-6 m）、
场景 ID 隔离 PASS、碰撞反证 PASS、V2 7 段 PASS、V1 FAIL（`EXCEEDS_STATE_WATCHDOG` 39.59 s）。

## 7C. 第四轮：V1 可行性与舌规 CAD

详见 [V1_FEASIBILITY.md](V1_FEASIBILITY.md)。按 `舌规V1.STEP` 建立工具模型（TCP、碰撞体、钩口朝上、插入行程 60 mm）后，
**状态机不改**，Fake 中 V1 单块电池 7 段全部成功，`side_insert` 13.31 s（原 35.04 s）。舌规安装方向是关键：
舌头朝插入方向 13.3 s，差 90° 约 20 s，装反不可行。此前"需转 180°、必须拆分状态机"的结论来自占位 TCP，已更正；
拆分提案（PR #10）降为备选。

## 8. 风险与阻断清单

> 10/12 P0 / P1 补验见 [MOVE_GROUP_SHUTDOWN_AUDIT.md](MOVE_GROUP_SHUTDOWN_AUDIT.md)：
> 空关节集合拒绝已修复；原始系统 MoveIt 2.12.4 退出仍有外部生命周期缺陷，已提供对照证据与显式临时规避。
> 旧证据中的“所有节点消失”不证明正常退出；新证据脚本逐个检查子进程退出并在失败时返回非零。

| # | 问题 | 影响 | 责任人 / 需谁确认 |
|---|---|---|---|
| R1 | **预规划起点过期**：`test_lift` 在 ToolPort 接合前规划，`retreat` 在卸载/退钩前规划；本包只能假设 ToolPort 行程（0.03 m）推算起点 | 起点偏差 > 0.01 rad 时 MoveIt 执行 ABORTED；直接发给 JTC 会跳变 | 孙嘉明：MotionAdapter 发送前核对起点并拒绝/触发重规划；**已提供** `check_trajectory_start()` 与 `replan_primitive()` 供执行侧调用 |
| R2 | PLAN_ACQUIRE 看门狗 2.0 s；ACQUIRE 需多次求解 | 真实规划耗时接近上限时 FSM 超时 | 孙嘉明：评估是否放宽；Fake 实测 V1 `side_insert` 0.41–0.51 s、V2 各段 < 0.01 s |
| R3 | `transport` 原语 `target_pose` 为提环位姿而非工位 | 若有人按字段名使用会把电池搬回原处 | 孙嘉明确认设计意图 |
| R4 | 舌规 TCP 按 CAD 估计（安装方向、贴合面未核对）；工位、提环轴向、相机外参未标定 | 输出只能 `simulation_only` | 韩明赫（MEASURED/ESTIMATED/UNKNOWN 表） |
| R5 | 碰撞场景：已有占位桌面 + 电池载荷（ID 前缀 `mtc_moveit_planner/` 隔离）；仍缺工具外形、相机、工位支架，取货电池本体未作为障碍物（需 ACM 允许钩子进入提环） | 不能证明真实场景无碰撞 | 常建烁（现场按实测补） |
| R6 | **V1 预对准时长取决于舌规安装方向**：按 CAD，舌头朝插入方向 13.31 s，差 90° 约 20 s，装反（180°）不可行 | 若装错方向 PLAN_ACQUIRE 即失败 | 现场装配 / 孙嘉明：确认安装方向；拆分提案 PR #10 作备选 |
| R8 | URDF 与 ros2_control 指令限位不一致（foreArm/wrist1/wrist3） | MoveIt 可能给出超出控制器限位的解；本包已拒绝此类输出，但实机软限位仍未知 | 孙嘉明 / 现场确认实机软限位 |
| R7 | `Pose6D.stamp_s` 为 FSM 时基，tf2 查询取最新变换 | 动态 TF（相机）时间同步需统一时基 | 韩明赫 / 孙嘉明 |
| R9 | **V1 插入 / 退钩无人执行**：舌规没有电机，插入 60 mm、退钩 60 mm 实际是机械臂动作；规划模块按设计不为 hook_engage / side_withdraw 出轨迹，C++ ToolPort 目前只有 Mock | 真机上 V1 无法完成挂钩 | **孙嘉明**：确定由谁规划和执行这两段低速直线（规划模块可以提供） |

## 9. 明天现场需要谁配合

- 韩明赫：TCP（V1/V2 分别）、T0/P1/P2/P3 位姿、ring_frame 轴向、相机外参；`ee_link` 是否在法兰面。
- 孙嘉明：R1/R2/R3 决定；MotionAdapter 接收 `MotionCommand.trajectory` 的整条提交。
- 张叔阳：统一启动顺序中加入 `move_group` 与本包；ROS graph 中确认 `/move_group` 与 TF。
- 孙浩然：`ring_pose` 的 `frame_id` 与时间戳来源（须能经 TF 变换到 `world`）。
