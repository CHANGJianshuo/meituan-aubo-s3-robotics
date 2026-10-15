# meituan_challenge_ws

**美团第四届低空经济与具身智能挑战赛｜AUBO S3 机械臂 ROS 2 工程**

**Phase 1 C++ FSM Core v0.1 已完成第一阶段离线安全验收，并合并到 `main`。** 合并提交为
`c2be85724111f6fb4dfcc62a2c974f7be1233cba`，本地 Git tag `fsm-core-v0.1.0`
指向该提交，作为团队的 FSM Core 开发基线。

**独立 ROS 2 Jazzy 驱动与 MoveIt 2 Mock 验证已完成；完整 FSM + MoveIt 2 集成仍在进行。**
**Phase 2A C++ ROS 2 Action 已完成离线 Mock 验证，并合入 `main`（交接基线 `c0f7017`）。**
Action 外壳复用现有 FSM；
外层 ROS Action 与 MoveIt 2 C++ Planner Adapter、FollowJointTrajectory 之间的统一接线
尚需验收。独立 Mock 成功不代表已完成这些集成，也不代表
AUBO S3 实机抓放验收通过。

开发环境：Ubuntu 24.04、ROS 2 Jazzy、C++17。当前只允许 Mock / Fake / 离线开发；
构建和测试不授权连接控制柜、调用真实 RPC/RTDE、发送实机轨迹或触发实体工具 IO。

本分支增加 [V1 Gazebo Harmonic 仿真](src/mtc_simulation/README.md)：保留原始电池 CAD、
基础/顺序任务底图与点位，直接运行 C++ `PickPlaceFsm → MoveIt → FollowJointTrajectory →
gz_ros2_control`。电池由舌规与把手的物理接触承托；输入位姿使用 Gazebo 真值。
运行命令见该包说明，验证范围、证据与实机交接见 [V1 仿真报告](docs/simulation/V1_SIMULATION_REPORT.md)。
外层 ROS Task/PickPlace Action 接线、真实视觉与实机标定
仍需单独验收；这条仿真链路不替代它们。

## 1. 从这里开始

- [FSM 开发者使用指南](README_FSM.md)：四个 C++ 包、主要 API、Port 契约、示例、构建与测试。
- [Phase 2A C++ ROS Action 指南](src/mtc_ros2_action_cpp/README.md)：三个 Mock Action Server、子 Goal 关联、取消与故障锁定，以及运行/测试方法。
- [Phase 2A 验收报告](docs/migration/PHASE2A_ROS2_ACTION_REPORT.md)：13包构建、358项C++及UBSan验证、取消/故障边界与未完成项。
- [ROS 2 驱动与 MoveIt 2 部署手册](docs/setup/AUBO_ROS2_DRIVER_JAZZY.md)：已验证环境、Mock 启动、控制器/TF 检查和 10 月 12 日团队 SOP。
- [第一阶段最终安全验收报告](docs/migration/CPP_FSM_FINAL_SAFETY_GATE.md)：缺陷复现、修复、318/269/15 测试证据及 TSan 限制。
- [第二阶段 ROS 2 集成计划](docs/migration/PHASE2_ROS2_INTEGRATION_PLAN.md)：后续 Action / Planner / Mock 集成工作。
- [Python ROS 接线修复与复验](docs/migration/ROS_WIRING_FIX_20261011.md)：B01/B02/B04 最小补丁、真实 ROS 回归，以及仍未通过的实际 Mock 任务链路。
- [原版本开发指南](README_FSM_v0.1.md)：保留原入口，当前使用说明以 `README_FSM.md` 为准。

## 2. 项目目标与已完成范围

项目面向固定工位电池识别、提环抓取和按颜色顺序摆放：基础任务将指定颜色电池
放入 T0；序列任务按口令将三块电池依次放入 P1、P2、P3。
V1 使用被动舌规；V2 使用机械锁止和电磁解锁策略。两种策略均已有核心逻辑及
Fake 测试，实体机构、工具 IO、TCP/负载和工位标定尚未验收。

四个 C++ 核心包均已合入 `main`，核心逻辑不依赖 `rclcpp`：

| 包 | 职责 | 主要入口 |
|---|---|---|
| [mtc_core_cpp](src/mtc_core_cpp/README.md) | 共享错误码、Pose6D、MotionCommand、TimedTrajectory、Clock 与证据类型 | [公共头文件](src/mtc_core_cpp/include/mtc_core_cpp/) |
| [mtc_task_cpp](src/mtc_task_cpp/README.md) | 整轮 Task FSM、任务校验、动作意图、重试和故障锁定 | [TaskFsm](src/mtc_task_cpp/include/mtc_task_cpp/task_fsm.hpp) |
| [mtc_manipulation_cpp](src/mtc_manipulation_cpp/README.md) | 单块电池 PickPlace FSM、V1/V2 ToolStrategy、四类 Port | [PickPlaceFsm](src/mtc_manipulation_cpp/include/mtc_manipulation_cpp/pick_place_fsm.hpp) |
| [mtc_motion_execution_cpp](src/mtc_motion_execution_cpp/README.md) | 单在途命令、执行校验、关联取消、停稳证据、Mock/Disabled Adapter | [MotionExecutorCore](src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/executor_core.hpp) |

Phase 2A 的 [mtc_ros2_action_cpp](src/mtc_ros2_action_cpp/README.md) 提供 ROS 2
外壳，四个核心包继续保持与 ROS 2 隔离：

```text
TaskServer → ROS PickPlace Action Client → PickPlaceServer → PickPlaceFsm
                                                            ├─ MockPerception
                                                            ├─ MockPlanner
                                                            ├─ MockTool
                                                            └─ 本地 AsyncMockMotion
                                                                 → MotionExecutorCore
                                                                 → MockMotionAdapter

独立 MotionServer → 另一套本地 MotionExecutorCore → MockMotionAdapter
待集成：MoveIt 2 Planner / FollowJointTrajectory / ros2_control Mock Hardware
```

**Task → PickPlace Action 通信已有 Mock 测试；C++ PickPlace 与独立 Motion Action
尚未连接，完整 Task → PickPlace → Motion Action → Mock Hardware 执行链路未打通。**
两类 FSM 继续采用单一事件线程所有权，运动执行/取消由外壳的 worker 处理；旧
Python FSM、接口定义和迁移记录均保留。

## 3. 编译与全量离线测试

唯一开发仓库：`/home/robot/meituan_challenge_ws`。
已具备 Jazzy、colcon、ament 和各包依赖后，在仓库根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/robot/meituan_challenge_ws
colcon list

# 使用独立目录，保留历史 build/install/log
colcon --log-base log/phase2a_mock build \
  --build-base build/phase2a_mock --install-base install/phase2a_mock
source install/phase2a_mock/setup.bash
colcon --log-base log/phase2a_mock test \
  --build-base build/phase2a_mock --install-base install/phase2a_mock
colcon test-result --test-result-base build/phase2a_mock --all

# 核验原318项及新增40项C++测试身份，并显式运行未注册的离线E2E
python3 src/mtc_ros2_action_cpp/test/verify_results.py --build-base build/phase2a_mock
python3 tests/test_mock_end_to_end.py
```

仅构建四个 C++ 包、定向 GTest、消费者 CMake 和 sanitizer 方法见
[开发者指南](README_FSM.md)。这些命令无需真实机器人；不要为测试启动真实驱动。

## 4. 最终验收记录

以下为 [2026-10-11 Phase 2A 报告](docs/migration/PHASE2A_ROS2_ACTION_REPORT.md)
记录的实际执行结果；本次文档交接更新没有重新运行构建与全量回归。

| 验证项 | 结果 |
|---|---|
| Jazzy 工作空间构建 | 13/13 包 PASS |
| C++ GTest | 358 PASS；20个可执行文件，原318项全部保留，Action新增40项 |
| Python pytest | 269 PASS |
| 显式 Python 离线 E2E | 15 PASS / FAIL 0 / SKIP 0 |
| C++ UBSan | 358 PASS；四个核心包及Action包 |
| XML manifest、git diff --check | PASS |
| TSan 动态竞态回归 | 本轮 NOT RUN；历史运行时错误记录保留，不代表已排除所有数据竞争 |
| Phase 2A C++ ROS 2 Action | Mock 验证 PASS；Task → PickPlace，独立 Motion Action |
| 完整 FSM + MoveIt Planner / FollowJointTrajectory / Mock Hardware 闭环 | 尚未完成、未验收 |
| AUBO S3 实机抓放与实体 IO | NOT RUN，未授权 |

358 + 269 + 15 为642个独立测试场景。colcon汇总包含CTest包装和lint条目，不能
重复计数。15项离线E2E使用Python脚本自带MockPlanner，不代表MoveIt集成通过。
报告中的全目录 `colcon test-result --all --verbose` 因保留的历史TSan失败返回
FAIL；当前13包的结果分别检查通过。上面的独立构建目录用于隔离历史结果；若
复用目录，必须确认测试结果属于本次构建，不能将旧XML当作新验收证据。

## 5. 安全契约

- Goal-scoped Cancel 使用 `cancel_goal(command_id, reason)`；错ID直接拒绝且后端停止调用为0。`supervisory_stop(reason)` 是明确的监督停止，不能替代旧 Goal 的取消。
- `request_delivered`、Action accepted/canceled 都不能证明停稳；需要关联命令、控制器状态、有效反馈、持续静止及队列清空等独立证据。软件 Stop 不是安全急停。
- 执行超时或运动状态未知时禁止自动重发。FAULT 结束当前执行但继续锁定，不能通过普通 RESET 或自动流程伪造人工恢复。
- 放置成功须由新鲜、有效、可区分的观测支撑。`placement_stability_sec` 必须有限且大于0；默认3.0秒是工程参数，不是比赛官方最低时间。
- 规划输出必须进入执行命令；缺失规划结果时拒绝，不能执行默认全零目标。多点轨迹不得拆成多次 ExecuteJointMove。
- Mock Adapter 和执行器共享同一个 ManualClock 实例；虚拟时间不用于真实后端。未配置关节限位的 Mock 成功不代表实机安全。

## 6. ROS 2 驱动与 MoveIt 2 环境

官方驱动参考工作空间 `/home/robot/aubo_driver_validation_ws` **只读**。其
`install_no_symlink` 环境已有 GenericSystem、200Hz controller、FollowJointTrajectory
和 MoveIt Plan → Execute 的实际成功日志。该环境包含一处已归档的 Fake Hardware
Xacro 修复，纯上游 clone 不能直接当作同一已验证环境。

[AUBO S3 ROS 2 Jazzy 驱动部署与运行手册](docs/setup/AUBO_ROS2_DRIVER_JAZZY.md)
列出准确包名、Launch 参数、部署前置、三终端 Mock SOP，以及历史起点不匹配和
MoveIt 退出异常。手册明确区分独立 Mock、待完成的 FSM 集成、需要现场单独授权的
真实硬件操作；不提供实机运动命令。新机器修复快照和完整部署仍需独立复验。

## 7. 系统集成交接（张叔阳）

交接基线为 `main@c0f7017`。先阅读 [C++ Action 包指南](src/mtc_ros2_action_cpp/README.md)、
[Phase 2A 验收报告](docs/migration/PHASE2A_ROS2_ACTION_REPORT.md) 和
[驱动与 MoveIt 2 Mock 手册](docs/setup/AUBO_ROS2_DRIVER_JAZZY.md)。

| C++ Action | 类型 | 当前关系 |
|---|---|---|
| `/mtc/task/execute` | `mtc_interfaces/action/ExecuteTask` | TaskServer通过ROS Client调用PickPlace |
| `/mtc/manipulation/pick_place` | `mtc_interfaces/action/PickPlace` | 使用现有FSM及本地Mock Ports/执行器 |
| `/mtc/motion/execute_joint_move` | `mtc_interfaces/action/ExecuteJointMove` | 独立Mock Server，仅六关节单点目标；未接入PickPlace |

现有 [mock_bringup.launch.py](src/mtc_bringup/launch/mock_bringup.launch.py) 仍启动
旧Python `mtc_task/task_executor`、`mtc_manipulation/pick_place_server`、
`mtc_motion_execution/motion_executor`，以及Python Tool/Safety节点，**不是C++
Bringup**。旧包继续保留；不要在同一ROS domain同时启动提供同名Action的Python
和C++ Server。新包目前以三个 `ros2 run` 入口独立启动，环境隔离及Mock操作示例
见包指南；主工作空间尚无独立的C++ Mock Bringup Launch。若按本文独立目录构建，
需将包指南中的overlay source改为 `install/phase2a_mock/setup.bash`，其余终端
统一使用同一构建环境，避免加载旧二进制。

张叔阳接下来的交付：

1. 在独立开发分支建立C++ Mock Bringup，明确启动节点及依赖顺序；保持后端为Mock，避免同名Server共存。
2. 在隔离的本机ROS domain复验Task → PickPlace Action通信，包括Goal、Feedback、Result、关联Cancel、子失败及超时；不能将Cancel ACK视为停稳。
3. 核查Node、Topic、Action类型和唯一Server、QoS、命名空间及日志，记录正常结果和异常定位方法。
4. 提供可独立复现的Mock启动SOP，列明每个终端的source、启动顺序、预期输出、检查命令、保持运行的进程和正常结束方法。
5. 为后续MoveIt 2集成准备Launch及测试环境，明确Planner/整条轨迹/停稳证据的适配边界；独立驱动Mock成功不能代替完整FSM闭环验收。

交接详细检查命令见 [FSM指南的交接章节](README_FSM.md#系统集成交接张叔阳)。
本阶段不连接真实机器人、不启用实体工具IO；FAULT、未知运动和负载锁定继续保留，
不得通过重启节点或自动复位宣称恢复安全状态。

## 8. 仓库目录与参考资料

```text
meituan_challenge_ws/
├── README.md / README_FSM.md     # 项目入口与开发者指南
├── src/
│   ├── mtc_{core,task,manipulation,motion_execution}_cpp/ # 四个核心包
│   ├── mtc_ros2_action_cpp/     # Phase 2A C++ Mock Action 外壳
│   ├── mtc_interfaces/          # 既有Action/Message/Service契约
│   ├── mtc_task/、mtc_manipulation/、mtc_motion_execution/
│   ├── mtc_tool/、mtc_safety/、mtc_aubo_bridge/、mtc_bringup/
│   ├── mtc_moveit_planner/、mtc_trajectory_execution_cpp/
│   ├── mtc_simulation/           # V1 物理场景、执行及独立验收
│   └── mtc_description/、mtc_motion_planning/ # 保留入口
├── config/、scripts/、tests/
├── docs/                        # 架构、参考、迁移及验收证据
└── build/、install/、log/        # 本地生成产物，不提交Git
```

当前 `colcon list` 注册16个包；上文13包结果是 Phase 2A 的历史验收范围。
description/planning 目录仍为保留入口。官方驱动参考工作空间 `/home/robot/aubo_driver_validation_ws`
已有独立的 Jazzy Mock Hardware / FollowJointTrajectory / MoveIt 验证记录，
只读参考；这不等于本仓库的新 C++ FSM 已完成同一集成。

- [Python/C++ 行为映射](docs/migration/FSM_BEHAVIOR_MAPPING.md)
- [C++ 接口兼容审计](docs/migration/INTERFACE_COMPAT_AUDIT_CPP.md)
- [上一轮审计修复报告](docs/migration/CODEX_CPP_FSM_AUDIT_FIX_REPORT.md)
- [官方驱动 Fake Hardware 修复审计](docs/migration/AUBO_DRIVER_FAKE_HARDWARE_FIX_AUDIT.md)
- [架构设计 v0.1](docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md)：目标设计，以当前代码与验收报告核实完成范围。
- [历史源码审计](docs/migration/SOURCE_AUDIT.md)、[迁移映射](docs/migration/MIGRATION_MAP.md)、[历史工作空间重构提示词](docs/prompts/DSH_AUBO_S3_独立Workspace重构提示词_v3.md)：保留历史，不作为重新迁移或清空仓库的指令。
- [AUBO S3 交接参考](docs/reference/AUBO_S3.md)：历史记录，不是当前设备状态证明。

仓库路径已于2026-10-09按用户要求从 `/home/meituan_challenge_ws` 迁至当前路径，
保留Git历史。旧工程及官方驱动工作空间只读。开发者从 `main` 建独立分支；
接口或安全语义变化需同步README和验收证据，不将Mock结果宣称为实机许可。
