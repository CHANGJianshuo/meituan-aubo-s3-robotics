# PR #4 评审问题在 #9 的处理：空关节集合与 MoveIt 退出

2026-10-12，分支 `feature/planning-v1-feasibility`。范围：本机 WSL2 / ROS 2 Jazzy / AUBO S3 Fake。
证据：[evidence/20261012_shutdown/](evidence/20261012_shutdown/)。没有真实设备连接、轨迹执行或 FJT goal。

## 1. 验收结论

| 项目 | 结论 |
|---|---|
| P0：空关节集合被起点一致性检查放行 | **已修复**，提交 `d9cda08`；本轮重新验证通过 |
| P1：确认退出问题归属 | **调查完成**：无业务请求、无本项目规划节点、无 launch 时，系统 MoveIt 仍在析构链崩溃；Fast DDS / CycloneDDS 均复现 |
| 原始系统 MoveIt 正常退出 | **FAIL**，未全局修补或替换 `/opt/ros/jazzy` |
| 显式保留两个 MoveIt 插件库的临时规避 | 空载直接启动（两种 RMW）、launch 对照及完整 Fake 规划流程通过；仅为本机已测版本的规避，不等同于修复上游 |
| 本项目退出验收 | **已修正**：进程消失、launch 返回 0 不再被当成正常退出；要求每个启动的子进程都有对应 PID 的正常退出记录，且没有信号升级 |

不改 0.01 rad 起点容差、原语速度、FSM 看门狗或下游 FJT 安全校验；不改接口与状态机。
#9 仍叠在 #4 的分支上，本轮没有合并或改动 PR 基准。

## 2. P0 复核

`check_trajectory_start()` 现在拒绝轨迹关节名为空、实际关节名为空的输入，并要求至少比较过一个关节。
负面测试 `EmptyJointSetsAreRejected` 覆盖空轨迹集合、空实测集合，以及两者同时为空。
历史修复记录包含撤回修复后该测试失败的反证；本轮重新运行包含它的全部 53 项规划包 GTest，全部通过。

## 3. P1 的受控对照

环境版本（完整版本号见各目录 `result.json`）：

- `moveit_ros_move_group` / `moveit_ros_planning` 2.12.4；`rclcpp` 28.1.22。
- `rmw_cyclonedds_cpp` 2.2.4；`rmw_fastrtps_cpp` 8.4.4。
- 系统二进制 `/opt/ros/jazzy/lib/moveit_ros_move_group/move_group`。
- 空载诊断生成 AUBO S3 **Fake** URDF（检查插件必须为 `mock_components/GenericSystem`），不启动驱动、控制器或规划客户端。
- 每次在空 ROS 域启动，只发一次 SIGINT；launch 模式只信号父进程，由 launch 转发。
- 因空载没有关节状态发布者，仅诊断参数 `planning_scene_monitor_options.wait_for_initial_state_timeout=0.0`；没有放宽运行中的起点校验。完整流程保持驱动原配置与 10 s 初始状态等待。

| 目录 | 实验 | 实际结果 |
|---|---|---|
| `baseline_cyclone` | 直接启动，CycloneDDS | SIGSEGV，子进程 −11，约 2.24 s 后回收 |
| `baseline_fast` | 直接启动，Fast DDS | SIGSEGV，子进程 −11，约 8.37 s 后回收 |
| `baseline_launch` | 通过 launch，CycloneDDS | 栈打印中超过默认 5 s，launch 升级 SIGTERM，子进程 −15；**launch 自身返回 0** |
| `baseline_gdb` | gdb 直接启动，CycloneDDS | 捕获主线程 SIGSEGV，保存所有线程栈；**gdb 自身返回 0，不代表目标正常退出** |
| `disabled_execution` | `allow_trajectory_execution=false` | 仍然 SIGSEGV，−11 |
| `retained_cyclone` | 保留两个插件库，直接启动 | 子进程 0，约 0.10 s |
| `retained_fast` | 同上，Fast DDS | 子进程 0，约 0.20 s |
| `retained_launch` | 同上，通过 launch | 子进程有正常退出记录，无信号升级，约 0.10 s |

GDB 主线程调用链（[完整线程栈](evidence/20261012_shutdown/baseline_gdb/process.log)）：

```text
main → MoveItCpp::~MoveItCpp
     → TrajectoryExecutionManager::~TrajectoryExecutionManager
     → rclcpp::Node::~Node → NodeBase::~NodeBase
     → rclcpp::CallbackGroup::~CallbackGroup → SIGSEGV
```

这是系统 MoveIt 的退出路径。该进程没有加载本项目 PlannerPort；不需要任何规划请求就能触发。
因此可以排除“本项目规划请求是必要触发条件”和“launch 转发信号是必要触发条件”。
两种 RMW 同样失败，也不能用切换 DDS 作为退出修复。

### 为什么会等到 SIGTERM，且有时 −11、有时 −15

本机 `core_pattern` 为 `|/wsl-capture-crash %t %E %p %s`。
直接运行的 `result.json` 捕获了 `CoreDumping: 1`，其中 `disabled_execution` 的等待点是 `pipe_write`。
另一次直接观察中，该状态持续到了 SIGINT 后第 8 秒。说明至少存在“崩溃后等待 WSL 转储”的延迟，不能据此宣称 Executor 死锁。

launch 对照与完整原始流程在 **backward 栈打印尚未结束时**就达到 5 s 并收到 SIGTERM，返回 −15；
直接运行有足够时间完成崩溃处理则返回 −11。不同阶段的返回码与评审结果相符，但不把所有延迟都归因于同一个转储阶段。
延长 SIGINT 等待时间只能改变是否升级信号，不能把 SIGSEGV 变成正常退出。

### 生命周期问题的证据与边界

控制变量只增加以下两个系统库的 `LD_PRELOAD`，让动态加载器在进程运行期间保留它们：

```text
libmoveit_simple_controller_manager.so
libmoveit_move_group_default_capabilities.so
```

原始失败变为正常退出，支持**插件库卸载早于仍需清理的 ROS 对象/控制块**这一生命周期解释。
这是对本地实验的归因，不是声称已完成 rclcpp / class_loader 的全链路源码修复。
实验中只提前销毁 TEM 的 controller node / executor，崩溃会转移到外层 node 清理，因此没有把这种局部析构改法提交为有效修复。

上游核对：MoveIt 的 [#1597](https://github.com/moveit/moveit2/issues/1597) 曾记录类似析构栈，
[#3680](https://github.com/moveit/moveit2/issues/3680) 也报告 Jazzy 退出段错误；它们是相关线索，不能仅凭标题认定同一缺陷已修复。
在 [2.12.4 MoveItCpp 源码](https://github.com/moveit/moveit2/blob/2.12.4/moveit_ros/planning/moveit_cpp/src/moveit_cpp.cpp)
中 TEM 会在构造时创建，故 `allow_trajectory_execution=false` 不会绕开其生命周期。

## 4. 本仓库改动与验证

- `probe_move_group_shutdown.py`：生成 Fake 参数；检查空 ROS 域；直接 / launch / gdb 启动；记录实际退出与转储状态；保存可复现参数及版本。gdb/launch 的成功返回码不掩盖目标失败。
- `check_launch_shutdown.py`：逐个匹配启动进程的名称与 PID，缺正常退出记录、异常死亡或 SIGTERM/SIGKILL 升级均拒绝。仅有“ROS 图为空”不能通过。
- `run_planning_evidence.sh`：只给 launch 父进程发初次 SIGINT，等待回收并核验日志；清理失败、规划失败或读图失败最终返回非零。没有扩大退出等待时间来掩盖缺陷。
- `MTC_MOVE_GROUP_PIN_PLUGINS=1`：**默认关闭**，只对本脚本的 move_group launch 子树启用上述库保留，并在 SUMMARY 中明确记录 WARN。其他 Fake 控制与规划工具不设置该变量。
- 新增 5 项 Python 回归：历史崩溃日志拒绝、历史正常控制退出接受、缺记录拒绝、PID 不匹配拒绝、信号升级即使最终正常退出仍拒绝。

重新构建规划包，运行七个 C++ 包测试：**510 项 GTest + 新增 5 项 Python 用例，均通过**。
`colcon test-result` 输出 546 tests / 0 errors / 0 failures / 0 skipped，其中还计入 CTest 的测试程序条目；
不把这一汇总数称为 546 个 C++ GTest。[构建](evidence/20261012_shutdown/planner_build.log)、
[规划包测试](evidence/20261012_shutdown/planner_test.log)、[其他六包](evidence/20261012_shutdown/related_tests.log)、
[汇总](evidence/20261012_shutdown/test_summary.txt)。

完整 Fake 流程对照：

| 项目 | 原始库加载方式 `full_stock/` | 显式保留插件 `full_retained/` |
|---|---|---|
| 固定 Pose、场景 ID 隔离、桌面碰撞拒绝、工具碰撞拒绝 | PASS | PASS |
| V1 七段、V2 七段 | PASS | PASS |
| move_group 正常退出 | **FAIL**（−15） | **PASS** |
| Fake 控制正常退出 | PASS | PASS |
| 脚本退出码 | **1**（没有误报整体成功） | **0** |

## 5. 复现与使用

仓库根目录执行；输出目录必须是新目录，防止旧日志混入：

```bash
source src/mtc_moveit_planner/scripts/env.sh
python3 src/mtc_moveit_planner/scripts/probe_move_group_shutdown.py /tmp/mg-stock \
  --domain 136 --rmw rmw_cyclonedds_cpp
# 当前系统预期返回 1：ready=true，目标收到 SIGSEGV。
# Fast DDS 对照改 --rmw rmw_fastrtps_cpp；另可使用 --mode launch 或 --mode gdb。
python3 src/mtc_moveit_planner/scripts/probe_move_group_shutdown.py /tmp/mg-retained \
  --domain 136 --rmw rmw_cyclonedds_cpp --pin-plugins
```

完整 Fake 规避模式（先按包 README 构建）：

```bash
MTC_RMW=rmw_cyclonedds_cpp MTC_MOVE_GROUP_PIN_PLUGINS=1 \
  bash src/mtc_moveit_planner/scripts/run_planning_evidence.sh /tmp/planning-retained
```

本轮构建目录为 `install/planner_r4`，所以实测命令另传了 `MTC_INSTALL=$PWD/install/planner_r4`。
不用该参数时沿用 `env.sh` 的 `install/planner` 默认值，应确保选中的是最新构建。
去掉 `MTC_MOVE_GROUP_PIN_PLUGINS=1` 即恢复原始启动方式；未修改全局库、驱动或 NUC 环境。

后续正式解决需在部署环境验证上游版本或完整的插件生命周期修复，不能把此处 LD_PRELOAD 规避当成真实执行验收。
V1 现场待确认项（安装角、TCP、60 mm 插入/退钩执行方）仍见 [V1_FEASIBILITY.md](V1_FEASIBILITY.md)。
