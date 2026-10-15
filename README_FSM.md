# FSM Core 开发者使用指南（美团无人机挑战赛）

> **当前状态：Phase 1 FSM Core v0.1 已完成；Phase 2A C++ ROS 2 Action 已完成离线 Mock 验证，两者均已合入 `main`。** 交接基线为 `main@c0f7017`；Core历史合并提交为 `c2be85724111f6fb4dfcc62a2c974f7be1233cba`，本地 tag `fsm-core-v0.1.0` 仍指向该Core版本，不包含后续Action包。
>
> **适用范围：C++17 FSM Core / ROS 2 Action 离线 Mock 开发。** 完整FSM + MoveIt 2 / FollowJointTrajectory / ros2_control Mock Hardware闭环尚未完成，未进行AUBO S3实机验收。严禁将本文示例改为直接驱动真实机械臂。

## 1. 团队交接：现在完成了什么？

本项目分两层状态机：

```text
比赛任务请求 (ExecuteTask)
    │
    ▼
C++ Task FSM (mtc_task_cpp)          任务口令、顺序、重试、取消与结果记录
    │  输出 CALL_PICK_PLACE 等 ActionRequest（动作意图）
    ▼
C++ PickPlace FSM (mtc_manipulation_cpp)
    ├─ PerceptionPort              感知目标、提环/主体 6D 位姿和放置状态
    ├─ PlannerPort                 规划结果（完整可执行关节命令或时间轨迹）
    ├─ MotionPort                  执行/取消运动，必须确认到位与停稳
    └─ ToolPort                    V1/V2 末端机构操作与独立验证证据
           │
           ▼
     Motion Executor / Mock Adapter (mtc_motion_execution_cpp)
           │
           ▼
     [待集成：MoveIt 2 / FollowJointTrajectory / S3 Fake Hardware]
```

### 四个 C++ 包

| 包 | 职责 | 代码入口 |
|---|---|---|
| `mtc_core_cpp` | 共享类型、错误码、6D Pose、TimedTrajectory、时钟与结果契约 | [src/mtc_core_cpp/include/mtc_core_cpp/](src/mtc_core_cpp/include/mtc_core_cpp/) |
| `mtc_task_cpp` | 整轮比赛 Task FSM、任务校验和动作意图 | [src/mtc_task_cpp/include/mtc_task_cpp/task_fsm.hpp](src/mtc_task_cpp/include/mtc_task_cpp/task_fsm.hpp) |
| `mtc_manipulation_cpp` | 单块电池 PickPlace FSM、V1/V2 ToolStrategy、Port 接口 | [src/mtc_manipulation_cpp/include/mtc_manipulation_cpp/pick_place_fsm.hpp](src/mtc_manipulation_cpp/include/mtc_manipulation_cpp/pick_place_fsm.hpp)、[Port契约](src/mtc_manipulation_cpp/include/mtc_manipulation_cpp/pick_place_types.hpp) |
| `mtc_motion_execution_cpp` | 单在途命令控制、校验、停稳证据、Mock/Disabled 运动适配器 | [src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/](src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/) |

**已实现：** 核心状态机和共享 C++ API；V1/V2 工具策略；纯逻辑 Fake 与 Mock 单元测试；单命令/整条轨迹的数据表达与保守执行校验。

**Phase 2A已实现并通过Mock验证：** 独立包 `mtc_ros2_action_cpp` 提供三个C++
Action Server，复用核心FSM；Task → PickPlace使用真正的ROS Action Client/Server。
PickPlace使用本地Mock运动端口，独立Motion Action尚未连接到PickPlace。

**未实现/未验收：** MoveIt 2 C++ Planner、`FollowJointTrajectory`端到端适配、完整
S3 Fake Hardware抓放闭环和真实硬件控制。独立驱动工作空间的MoveIt 2 Mock验证
已经完成，证据与SOP见 [驱动手册](docs/setup/AUBO_ROS2_DRIVER_JAZZY.md)，不能等同
于本仓库完整FSM执行链路已打通。

## 2. 开发基线与团队使用方式

`main@c0f7017` 已包含 Final Gate 与 Phase 2A；`fsm-core-v0.1.0` 为Core历史本地标签，
目标提交为 `c2be857`。过期 Goal 误停、Mock 时基、零放置时长及零静止跨度等
安全问题已有修复和负面测试；当前main用于离线Action系统集成及后续Mock开发。
完整 TSan 尚未完成，不能称为已完成动态竞态验收或实机验收。

首次克隆仅在目标目录不存在时执行；已有工作区先检查状态，不覆盖或重新初始化：

```bash
git clone https://github.com/F1u0rite/meituan_challenge_ws.git /home/robot/meituan_challenge_ws
cd /home/robot/meituan_challenge_ws
git status --short
git branch --show-current
git log -1 --oneline

# 从本地main创建自己的开发分支
git switch -c feature/my-module-name main
```

已有干净仓库可按团队流程更新本地 `main` 后再开分支。不要在有未提交工作时
直接切换或覆盖。本文不要求推送、修改现有标签或操作任何真实设备。

## 3. 环境、编译与测试

基线：Ubuntu 24.04、ROS 2 Jazzy、C++17、`colcon`、`ament_cmake`、
`ament_cmake_gtest`，以及已注册Python包的pytest依赖。
实际依赖见各包 `package.xml` / `CMakeLists.txt`；核心库不依赖ROS运行时，
Action包依赖`rclcpp`、`rclcpp_action`和`mtc_interfaces`。
下面各组命令在仓库根目录执行，不清空已有build/install/log。

### 完整工作空间

```bash
source /opt/ros/jazzy/setup.bash
cd /home/robot/meituan_challenge_ws
colcon list
colcon --log-base log/phase2a_mock build \
  --build-base build/phase2a_mock --install-base install/phase2a_mock
source install/phase2a_mock/setup.bash
colcon --log-base log/phase2a_mock test \
  --build-base build/phase2a_mock --install-base install/phase2a_mock
colcon test-result --test-result-base build/phase2a_mock --all
python3 src/mtc_ros2_action_cpp/test/verify_results.py --build-base build/phase2a_mock
python3 tests/test_mock_end_to_end.py
```

新身份核验脚本比较原318项、Action新增40项与实际XML，覆盖358项及20个
可执行文件，拒绝缺失、重复、失败、跳过或未完成项。历史脚本
`tests/verify_cpp_audit_results.py` 固定16个可执行文件，不适用于当前完整集合。
E2E脚本未被colcon注册，必须单独执行；它是纯Python Mock，
不是C++ ROS 2 E2E，也未接入真正的MoveIt Planner。

### 仅四个 C++ 核心包

在新终端先加载Jazzy，再构建和加载对应目录，避免混用其他overlay：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/robot/meituan_challenge_ws
colcon --log-base log/fsm_core build \
  --build-base build/fsm_core --install-base install/fsm_core \
  --packages-up-to mtc_core_cpp mtc_task_cpp mtc_motion_execution_cpp mtc_manipulation_cpp
source install/fsm_core/setup.bash
colcon --log-base log/fsm_core test \
  --build-base build/fsm_core --install-base install/fsm_core \
  --packages-select mtc_core_cpp mtc_task_cpp mtc_motion_execution_cpp mtc_manipulation_cpp
colcon test-result --test-result-base build/fsm_core --all
```

此入口仅运行四个核心包的318项GTest，不执行Action包、Python包、接口XML或
E2E，不能据此宣称全工作空间回归通过。完整身份核验需先执行五个C++包的全部
测试并使用上一节的新脚本，不能对仅四包的结果调用它。

### 定向 GTest

先完成相应目录的构建。在加载 `install/phase2a_mock/setup.bash` 的终端可执行：

```bash
build/phase2a_mock/mtc_motion_execution_cpp/test_executor_concurrency \
  --gtest_filter='GoalCancelGate.*'
build/phase2a_mock/mtc_motion_execution_cpp/test_mock_adapter \
  --gtest_filter='MockClockGate.*'
build/phase2a_mock/mtc_manipulation_cpp/test_pick_place_audit \
  --gtest_filter='PlacementAudit.*:PlacementGate.*:WatchdogAudit.*'
```

定向运行不能替代colcon全量结果与身份核验。

### 已记录的验收结果与 sanitizer

以下结果来自 [2026-10-11 Phase 2A报告](docs/migration/PHASE2A_ROS2_ACTION_REPORT.md)，
本次README交接更新没有重新执行构建或全量回归：

| 验证 | 记录结果 |
|---|---|
| Jazzy全工作空间编译 | 13/13包 PASS |
| C++ GTest | 358 PASS；原318项全部保留，Action新增40项，20个可执行文件 |
| Python pytest | 269 PASS |
| 显式Python离线E2E | 15 PASS / FAIL 0 / SKIP 0 |
| C++ UBSan | 358 PASS，四个核心包及Action包 |
| XML、git diff --check | PASS |
| TSan | 本轮 NOT RUN；历史运行时`unexpected memory mapping`记录仍保留 |
| Phase 2A C++ ROS 2 Action | Mock验证PASS，Task → PickPlace及独立Motion Action |
| 完整FSM + MoveIt Planner / FollowJointTrajectory / Mock Hardware闭环 | 尚未完成，未验收 |
| 实机与实体IO | NOT RUN，未授权 |

C++包分布：core23、task87、manipulation120、motion88、action40。
358+269+15为642个独立测试场景；CTest包装/lint条目不能重复计数。Python E2E使用
脚本MockPlanner，不代表MoveIt成功；C++ Action通信证据来自Action包注册测试。

报告记录全目录 `colcon test-result --all --verbose` 因历史TSan失败返回FAIL，
当前13包结果分别检查通过。上面的独立目录可隔离历史结果；复用目录时仍需核对
当前构建、执行和XML身份。UBSan命令与历史结果说明见Phase 2A报告第4、5节。
TSan本轮没有运行，普通并发测试或UBSan不能替代动态竞态检测；不修改系统ASLR
或安全设置，也不将未执行项写成PASS。

### 共享类型 API

公共namespace为 `mtc::core`，CMake导出target为 `mtc_core_cpp::mtc_core_cpp`。

| 类型/入口 | 用法与契约 |
|---|---|
| [Clock / ManualClock / SteadyClock](src/mtc_core_cpp/include/mtc_core_cpp/clock.hpp) | 注入单调时间；ManualClock仅离线显式推进，相同数值起点不代表同一时基 |
| [Pose6D / PoseEstimate](src/mtc_core_cpp/include/mtc_core_cpp/pose.hpp) | 保留位置、四元数、frame_id和时间戳；有限性、姿态及新鲜度须校验 |
| [MotionPrimitive / MotionCommand](src/mtc_core_cpp/include/mtc_core_cpp/motion_primitive.hpp) | 前者为意图/约束，后者为规划后的关节目标或整条轨迹；按关节名与顺序关联 |
| [TimedTrajectory](src/mtc_core_cpp/include/mtc_core_cpp/timed_trajectory.hpp) | 多点、时间参数化轨迹；shape_is_valid只是结构校验，还需执行层配置校验 |
| [PlanResult / MotionResult / StopResult / StopEvidence](src/mtc_core_cpp/include/mtc_core_cpp/results.hpp) | 规划输出、运动结果、请求送达与独立停稳证据分别表达，不能互相推导 |
| [ErrorCodes](src/mtc_core_cpp/include/mtc_core_cpp/error_codes.hpp) | 共享错误码，与既有接口常量对照测试 |

### Motion Executor API

公共namespace为 `mtc::motion`，CMake导出target为
`mtc_motion_execution_cpp::mtc_motion_execution_cpp`。

| API | 含义 |
|---|---|
| `MotionExecutorCore(adapter, config, clock)` | 唯一执行权、线程安全状态与严格证据检查；不持有状态锁等待后端执行 |
| `execute_joint_move(command)` | 同步执行入口；实际完成/失败结果返回，不是异步ROS Goal接口 |
| `cancel_goal(command_id, reason)` | 仅取消关联Goal；错ID错误210，后端0停止且当前状态不变 |
| `supervisory_stop(reason)` | 显式监督软件停止，与Goal取消分别表达，不是安全急停 |
| `request_stop(reason, optional_id)` | 兼容入口：有ID路由关联取消，无ID路由监督停止 |
| `snapshot()` / `handshake(id)` | 加锁快照/握手副本，避免引用并发修改的存储 |
| `motion_state_unknown()` | 状态未知时禁止新运动与自动重发 |
| `clear_unknown_after_manual_confirmation(note)` | 仅人工确认出口；要求记录及无在途回调，自动流程不得调用 |
| `validate_joint_move()` / `validate_timed_trajectory()` | [执行前校验](src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/validation.hpp)，包括能力、有限数值、限位/动力学等 |

[MotionAdapter](src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/motion_adapter.hpp)
是同步后端契约：`capabilities()`、`execution_clock()`、`execute_joint_move()`、
`request_stop_for_command()`、`joint_feedback()`、`stop_evidence()`等。
默认关联取消显式拒绝，不降级为全局停止；execute/stop可能并发，后端必须保证
命令关联、取消预约和生命周期。默认 `execute_trajectory()` 拒绝，禁止静默拆分。
当前Mock也拒绝真正轨迹跟踪，不能只打开能力位就宣称已经实现。

[MockMotionAdapter / DisabledMotionAdapter](src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/mock_motion_adapter.hpp)
分别提供离线模拟与明确拒绝。默认Mock创建ManualClock；默认执行器共享它，
显式时钟必须为同一个实例。真实可执行后端不能继承虚拟时间授权，不能把
无关节限位的Mock通过当作实机参数审核通过。

### Motion Executor 离线 Mock 最小装配

下面只用于单元测试或隔离的 Mock 程序，不产生 ROS 控制命令。默认 Mock 已具有共享虚拟时钟，但**建议显式传入同一个 `ManualClock` 实例**：

```cpp
#include <memory>
#include "mtc_core_cpp/clock.hpp"
#include "mtc_motion_execution_cpp/mock_motion_adapter.hpp"
#include "mtc_motion_execution_cpp/executor_core.hpp"

int main()
{
  auto mock_clock = std::make_shared<mtc::core::ManualClock>(100.0);
  mtc::motion::ValidationConfig validation;
  auto adapter = std::make_shared<mtc::motion::MockMotionAdapter>(
      mtc::motion::MockMotionAdapterConfig{}, mock_clock, validation);
  mtc::motion::MotionExecutorCore executor(adapter, validation, mock_clock);

  // 仅为离线Mock构造的非零关节目标，不是Planner输出或实机姿态。
  mtc::core::MotionCommand command;
  command.command_id = "offline-motion-demo-1";
  command.joint_names = validation.expected_joint_names;
  command.target_rad = {0.05, -0.05, 0.05, 0.0, 0.0, 0.0};
  command.simulation_only = true;
  const auto result = executor.execute_joint_move(command);
  // Goal-scoped取消：executor.cancel_goal(command.command_id, "用户取消");
  // 监督停止：executor.supervisory_stop("监督停止");
  return result.success && result.stop_confirmed && !result.status_unknown ? 0 : 1;
}
```

CMake 中运动执行库的依赖为 `mtc_motion_execution_cpp::mtc_motion_execution_cpp`；完整可运行装配请参照 [src/mtc_motion_execution_cpp/test/test_mock_adapter.cpp](src/mtc_motion_execution_cpp/test/test_mock_adapter.cpp)。

### 无设备冒烟示例：运行 Task FSM 产生动作意图

以下代码只推进纯 C++ 状态机，**不会发送 ROS Action 或运动命令**。将其放入你自己的 `ament_cmake` 包 `src/fsm_task_smoke.cpp`：

```cpp
#include <iostream>
#include "mtc_task_cpp/task_fsm.hpp"
#include "mtc_task_cpp/task_validation.hpp"

int main()
{
  using namespace mtc::task;
  TaskFsm fsm;
  fsm.start();
  fsm.report_self_check(true);
  if (!fsm.accepts_tasks()) {
    std::cerr << "FSM is not ready for a task\n";
    return 1;
  }

  TaskRequest goal;
  goal.task_id = "offline-demo-1";
  goal.mode = TaskMode::MODE_BASIC;
  goal.ordered_colors = {1};  // RED
  goal.timeout_ms = 60000;
  goal.placement_stability_sec = 3.0;

  const auto check = validate_task_request(goal);
  if (!check.ok) {
    std::cerr << check.message << '\n';
    return 1;
  }

  fsm.submit_goal(goal);       // WAIT_TASK -> VALIDATE_TASK
  fsm.advance_validation();   // VALIDATE_TASK -> SCAN_SCENE
  const auto step = fsm.report_target("mock-battery-1", 1);
  if (const auto * action = step.find(ActionKind::CALL_PICK_PLACE)) {
    std::cout << "Would call PickPlace: object=" << action->pick_place.object_id
              << ", slot=" << action->pick_place.target_slot << '\n';
  }
  // 这是动作意图，不代表下层已经执行。
  return 0;
}
```

消费者 `CMakeLists.txt` 示例（在一个**独立的** `ament_cmake` 包中）：

```cmake
cmake_minimum_required(VERSION 3.8)
project(fsm_task_smoke LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
# 当前Task导出target引用core target，消费者显式加载它
find_package(mtc_core_cpp REQUIRED)
find_package(mtc_task_cpp REQUIRED)

add_executable(fsm_task_smoke src/fsm_task_smoke.cpp)
target_compile_features(fsm_task_smoke PRIVATE cxx_std_17)
target_link_libraries(fsm_task_smoke PRIVATE mtc_task_cpp::mtc_task_cpp)
install(TARGETS fsm_task_smoke DESTINATION lib/${PROJECT_NAME})
ament_package()
```

消费者 `package.xml` 还需声明 `ament_cmake` 为构建工具依赖、`mtc_core_cpp` 和 `mtc_task_cpp` 为依赖；这是**代码示例而非一个已提交的独立示例包**，可直接参考 [src/mtc_task_cpp/test/test_task_fsm_core.cpp](src/mtc_task_cpp/test/test_task_fsm_core.cpp) 的测试装配方式。`MODE_BASIC` 仅需要一种颜色，对应 T0；`MODE_SEQUENCE` 需要三种不同颜色，按顺序对应 P1/P2/P3。颜色值：1 红、2 蓝、3 黄、4 绿；0 表示未知，不是合法口令。

## 4. 如何对接 Task FSM

入口：`#include "mtc_task_cpp/task_fsm.hpp"`。

| 方法或类型 | 含义 |
|---|---|
| `TaskFsm::start()`、`report_self_check(bool)` | 初始化并完成自检，进入 `WAIT_TASK` |
| `submit_goal(TaskRequest)` | 接收一轮任务；先通过 `validate_task_request()` 验证 |
| `advance_validation()` | 消费 Task FSM 的待验证结果并推进至扫描 |
| `report_target(object_id, color)` | 上报识别结果；可能产出 `CALL_PICK_PLACE` |
| `report_item_result(success, placement_verified, error_code, message)` | 只有下层 PickPlace **真正结束并验证**后才调用 |
| `record_completed_item()` | 完成记录后推进下一项或整个任务结束 |
| `check_timeouts()` / `check_task_deadline()` | 应由外部事件循环周期触发的看门狗 |
| `request_cancel()`、`confirm_child_stopped()` | 取消与确认下层真正停止，是两个不同事件 |
| `StepResult.actions` / `ActionKind` | FSM 输出的**动作请求**：外壳负责转换成 ROS 调用 |
| `action_finished()` 与 `fault_locked()` | **执行结束**与**故障锁定**不能混为一谈 |

**关键契约：** 不要直接在状态机内部做 ROS 发布、网络 I/O 或机械臂动作；外部适配层读取 `StepResult.actions`，执行 `CALL_PICK_PLACE` / `CANCEL_CHILD` / `REQUEST_STOP` / `PUBLISH_STATE` / `LOCK_ACTIONS`，再把异步完成结果反馈给 FSM。`FAULT` 结束当前执行，但不能被普通 `reset()` 或重建对象用来绕过受监督的恢复协议。

## 5. 如何对接 PickPlace FSM

入口：

```cpp
#include "mtc_manipulation_cpp/pick_place_fsm.hpp"
#include "mtc_manipulation_cpp/pick_place_types.hpp"
#include "mtc_manipulation_cpp/tool_strategy.hpp"
```

与 Task FSM 不同，**PickPlace FSM 必须注入四类 Port 和一个 Clock，不能单独靠调用 `step()` 就执行抓放**：

```cpp
// 示意：各 Port 类型由队友的适配模块提供；这里不构造真实设备对象。
// PerceptionPortPtr perception = ...;
// PlannerPortPtr planner = ...;
// MotionPortPtr motion = ...;     // Mock/Disabled only
// ToolPortPtr tool = ...;         // Mock only
// mtc::core::ClockPtr clock = ...;

mtc::manipulation::PickPlaceConfig config;
config.tool_type = mtc::manipulation::TOOL_PASSIVE_HOOK_V1;
config.placement_stability_sec = 3.0;
config.state_timeouts = mtc::manipulation::default_state_timeouts();

// 当以上依赖均已实现并注入后：
// mtc::manipulation::PickPlaceFsm fsm(
//     perception, planner, motion, tool, clock,
//     mtc::manipulation::make_strategy(config.tool_type), config);
// fsm.start("battery_1", 1, "P1", "req-demo-1", 3.0);
// 在同一 FSM 事件线程中逐次 fsm.step() 并读取 fsm.feedback()。
```

这段是**接口装配示意，不是独立可运行的完整抓放程序**；想看到可运行的纯逻辑 Fake，请从 [src/mtc_manipulation_cpp/test/test_pick_place_fsm_v1.cpp](src/mtc_manipulation_cpp/test/test_pick_place_fsm_v1.cpp) 与 [test/fakes.hpp](src/mtc_manipulation_cpp/test/fakes.hpp) 复制测试思路。`test/fakes.hpp` 位于测试目录，**不是安装后提供的稳定公共 API**。

### Port 的责任与交付标准

| Port | 队友应该实现什么 | 不能做什么 |
|---|---|---|
| `PerceptionPort::get_detection()` | 返回 `DetectionResult`：目标 ID/颜色、唯一性、新鲜度、完整 `PoseEstimate`；放置后核验需要持续产生带新时间戳的观测 | 丢掉姿态或 `frame_id`；拿旧帧冒充新帧；伪造“物体已稳定” |
| `PlannerPort::plan()` | 为每个 `PlanTarget.primitives` 返回对应的 `PlanResult.commands`，包括关节目标或整条 `TimedTrajectory`；保留 6D 位姿、TF、速度/接触约束 | 只返回 `ok=true`；使用默认全零关节目标；把轨迹拆成多次单点调用 |
| `MotionPort::execute_joint_move(command)` / `execute_trajectory(command)` / `request_stop(reason)` | 运动入口传MotionCommand；整条轨迹保存在command.trajectory；支持异步在途语义和命令ID去重，停止需要独立停稳证据 | `accepted` 就当作 `arrived`，`request_delivered` 就当作 `stop_confirmed`；状态未知时重发 |
| `ToolPort` | 实现 `engage` / `test_lift` / `seat_verify` / `unload` / `unlock` / `disengage` / `holding_state`，返回独立 `ToolEvidence` | 解锁请求受理就报告已释放；在未落座或未卸载时解除 V2 锁止 |

**异步语义：** 感知、规划、运动执行及大部分工具方法返回 `std::optional<Result>`，其中 `std::nullopt` 表示尚未完成。`MotionPort::request_stop()` 直接返回 `StopResult`，`ToolPort::seat_verify()` 直接返回 `ToolEvidence`，`holding_state()` 返回字符串，不能统一当作optional接口。FSM 之后可能再次查询同一命令；**端口必须按 command_id 去重/复用结果，不可每次调用都重新发送运动指令**。注意 `VERIFY_PLACED` 需要不同的、新鲜观测，不能一味缓存成功帧。

[MotionAdapterPort](src/mtc_manipulation_cpp/include/mtc_manipulation_cpp/motion_adapter_port.hpp)
当前为同步桥接：关节运动经MotionExecutorCore校验后立即返回结果，不能在串行
FSM事件线程用它等待长轨迹。它未覆盖整条轨迹执行，因此继承MotionPort的明确
拒绝；异步轨迹端口是第二阶段工作。其无ID `request_stop()` 表示监督停止，
不能作为外部ROS Goal-scoped Cancel入口。

纯逻辑Fake位于测试目录，Fake规划命令标记 `simulation_only=true`，不能送到
可执行后端。规划必须逐原语返回 `PlanResult.commands`；FSM保留意图/6D约束，
赋予本次command_id，按关节命令或完整轨迹派发。缺失结果不会执行默认回零。

**V1/V2：** 默认 `TOOL_PASSIVE_HOOK_V1` 为被动舌规，不调用电磁解锁；`TOOL_MAGNETIC_LATCH_V2` 允许在落座、卸载两项独立核验完成后申请解锁。`ToolStrategy` 负责生成意图，不直接操作 IO。

**Final Gate 特别注意（取消接口）：** `MotionExecutorCore` 现在区分 `cancel_goal(command_id, reason)`（只取消指定命令）和 `supervisory_stop(reason)`（明确的监督停止）。**ROS Goal 的取消回调必须调用前者并携带正确 ID，不得使用无 ID 的监督停止替代。** 旧 Goal 取消被明确拒绝后，不应触发后端 Stop；`StopResult::request_delivered` 与 `stop_confirmed` 仍是两种独立状态。可对照 `test_executor_concurrency.cpp` 中的 `GoalCancelGate.*` 测试。

**放置稳定窗口：** `placement_stability_sec` 默认 3.0 秒；**零、负数、NaN 和 Inf 均属非法输入，必须在运动前拒绝**。即使传入极小正数，也至少需要两个不同时间戳且符合新鲜度要求的观测；首帧累计时间为 0 时不得宣告成功。3.0 秒是工程默认值，不是官方赛题给出的最低值。

**并发：** Task FSM 和 PickPlace FSM 对象本身遵循**单一事件线程所有权**。ROS 2 多线程回调应把事件送入串行队列；不能从多个 ROS 回调线程同时调用 `step()/start()/request_cancel()`。`MotionExecutorCore` 独立处理执行/停止竞争，但不能代替 FSM 的事件串行化。

## 6. Phase 2A ROS 2 Action 与后续集成边界

核心库继续与ROS 2隔离，已实现的外壳位于
[src/mtc_ros2_action_cpp](src/mtc_ros2_action_cpp/README.md)。三个Action使用原有
`mtc_interfaces`契约，没有修改FSM状态转换或ROSIDL字段：

| Action | 类型 | 当前可执行入口 |
|---|---|---|
| `/mtc/task/execute` | `mtc_interfaces/action/ExecuteTask` | `ros2 run mtc_ros2_action_cpp mock_task_server` |
| `/mtc/manipulation/pick_place` | `mtc_interfaces/action/PickPlace` | `ros2 run mtc_ros2_action_cpp mock_pick_place_server` |
| `/mtc/motion/execute_joint_move` | `mtc_interfaces/action/ExecuteJointMove` | `ros2 run mtc_ros2_action_cpp mock_motion_server` |

```text
TaskServer → ROS PickPlace Action Client → PickPlaceServer → PickPlaceFsm
                                              ↓ MockPerception / MockPlanner / MockTool
                                              ↓ 本地 AsyncMockMotion
                                      MotionExecutorCore → MockMotionAdapter

MotionServer（独立） → 另一套 MotionExecutorCore → MockMotionAdapter
```

ExecuteTask Goal转为 `TaskRequest` 后交给TaskFsm；`CALL_PICK_PLACE`产生真正的ROS
子Goal，`CANCEL_CHILD`关联保存的子Goal handle/UUID。PickPlace Feedback/Result来自
原FSM。ROS回调只登记Goal/Cancel和子事件；同一MutuallyExclusive CallbackGroup
中的Timer推进FSM，执行和停止放到owned worker，以非阻塞方式轮询结果。

**C++ PickPlace与独立Motion Action尚未连接。** Motion Server仅支持六关节单点目标，
没有整条轨迹入口，不能拆分多点轨迹来宣称轨迹执行。PickPlace现在使用本地Mock
执行器；三种Action存在不等于完整Task → PickPlace → Motion Action → Mock Hardware
链路已打通。完整行为、测试及限制见 [Phase 2A报告](docs/migration/PHASE2A_ROS2_ACTION_REPORT.md)。

取消受理与停稳确认继续分离；本包PickPlace只有核心 `CANCELLED && stop_confirmed`
才返回ROS CANCELED。既有PickPlace Result没有独立的停稳证据字段，Task仅信任本包
加强契约的Mock Server最终 `CANCELED/OK`，不能将第三方Server或Cancel ACK视为
停止证明。FAULT、未知运动及挂载/未知负载保持锁定，没有自动人工复位。

后续接入需要核对完整 `Pose6D`、TF、时戳/时基、规划结果和独立停稳证据；MoveIt 2
完整时间参数化轨迹应由未来适配器作为一次FollowJointTrajectory Goal提交。目前未
实现该适配，不得称为已完成机器人控制闭环。

### 系统集成交接（张叔阳）

交接以 `main@c0f7017` 为基线。现有
[mock_bringup.launch.py](src/mtc_bringup/launch/mock_bringup.launch.py) 仍启动旧Python
`mtc_task/task_executor`、`mtc_manipulation/pick_place_server`、
`mtc_motion_execution/motion_executor`，以及Tool/Safety节点；它不是C++ Bringup。
旧Python包暂不删除；同一domain内不得同时启动提供同名Action的Python/C++ Server。

下一步在独立分支交付：

1. 建立独立C++ Mock Bringup，明确节点、依赖顺序、命名空间与Mock环境隔离，避免同名Server共存。
2. 复验Task → PickPlace ROS通信；覆盖Goal/Feedback/Result、关联Cancel、子失败、超时及FAULT锁定。独立Motion Action另行验证，不虚构PickPlace → Motion连接。
3. 检查Node、Topic、Action类型、唯一Server、QoS和日志，将观察结果保存为可追溯证据。
4. 提供团队可独立复现的Mock启动SOP，包括每个终端的source、运行顺序、预期结果、出错定位和正常退出；目前可参考 [Action包的逐终端方法](src/mtc_ros2_action_cpp/README.md)。
5. 为MoveIt 2后续集成准备Launch和测试环境，参考 [驱动Mock部署手册](docs/setup/AUBO_ROS2_DRIVER_JAZZY.md)，区分独立驱动验证与完整FSM闭环。

完成包指南中的source和本机domain隔离后，可在观察终端检查。若按本文独立目录
构建，将包指南中的overlay source改为 `install/phase2a_mock/setup.bash`，所有
终端加载同一构建环境：

```bash
ros2 node list --no-daemon --spin-time 2
ros2 node info /mtc_mock_task
ros2 node info /mtc_mock_pick_place
ros2 action list -t
ros2 action info /mtc/task/execute -t
ros2 action info /mtc/manipulation/pick_place -t
ros2 topic list -t --include-hidden-topics
ros2 topic info /mtc/task/execute/_action/feedback --verbose
ros2 topic info /mtc/manipulation/pick_place/_action/feedback --verbose
```

Task/PickPlace同域时每个Action应只有一个对应C++ Server，节点名分别为
`/mtc_mock_task`、`/mtc_mock_pick_place`。独立启动Motion后检查
`ros2 node info /mtc_mock_joint_move` 和
`ros2 action info /mtc/motion/execute_joint_move -t`。
Topic检查用于核对发布/订阅端、QoS及匹配情况；不能仅看Action名称存在就判定通信
成功。SOP应记录 `ROS_LOG_DIR` 与实际日志位置，并展示Feedback及最终Result证据。

本交接仅授权软件Mock；不连接真实机器人、不调用实体工具IO。不得更改既有停止、
FAULT、未知状态或负载安全语义，也不得以自动重启清除锁定来宣称完成安全恢复。

## 7. 队友开发分工及 PR 验收

推荐以当前 `main`（已包含Phase 2A）作为共同基线，在独立分支实现；
`fsm-core-v0.1.0` 仅用于追溯Core历史版本：

| 方向 | 建议分支 | 首先交付 |
|---|---|---|
| 感知 | `feature/perception-adapter` | `PerceptionPort` Mock/转换测试、6D Pose、TF/时间戳拒绝场景 |
| 规划 | `feature/moveit-planner` | `PlannerPort` Mock/规划输出测试，`PlanResult.commands` 完整性 |
| 运动执行 | `feature/motion-controller` | Mock `FollowJointTrajectory` 单 Goal + 取消/停稳证据测试 |
| 系统集成（张叔阳） | `feature/cpp-mock-bringup` | 独立C++ Mock Launch、Task → PickPlace通信检查及可复现SOP |
| 工具 | `feature/tool-adapter` | V1/V2 `ToolPort` Fake/Mock 与解锁联锁测试 |

**提交 PR 前：** 必须附上相应 `colcon build`、`colcon test` 的实际结果，说明是否触及 IDL、TF、并发/停稳契约；包含负面测试；所有实机能力继续默认禁用。接口变更先讨论，不要各自修改同一 `.action/.msg/.srv` 使契约漂移。

## 8. 常见错误与注意事项

- **找不到包或头文件：** 确认已经 `source /opt/ros/jazzy/setup.bash`，构建后在当前终端 `source install/phase2a_mock/setup.bash`（仅核心目录则加载 `install/fsm_core/setup.bash`），消费者在 `package.xml`/CMake 中声明所依赖的 C++ 包。
- **`test/fakes.hpp` 找不到：** 测试 Fake 不属于安装的公共 include 目录；不要把它当作正式后端。
- **FSM 一直等待：** 检查 Port 是否返回 `std::nullopt`、是否由外部事件循环继续 tick、是否向 FSM 投递了对应完成事件；不要绕过看门狗。
- **规划器报告 `ok=true` 但不执行：** `PlanResult.commands` 必须与请求原语一一对应，并满足完整轨迹/关节目标校验；不能只返回 `ok`。
- **取消已请求但结果是 FAULT：** 停止请求被受理不等于后端已确认停稳，检查 StopEvidence 与关联 command_id；不要改写结果掩盖故障。
- **Mock 运行时钟：** Final Gate 后 `MockMotionAdapter` 默认创建 `ManualClock`，`MotionExecutorCore` 若未显式传入时钟，会沿用适配器的同一实例；如显式注入时钟，**必须是同一 `ClockPtr` 实例**。不同 `ManualClock` 即使起点相同也会被拒绝；Mock 不支持 `SteadyClock`。
- **想直接控制 S3：** 此 README 未授权任何实体控制；当前C++仅完成Core及Action软件Mock。使用 `mock_components/GenericSystem` 的C++ FSM集成属于尚未完成的第二阶段。

## 9. 源码和进一步阅读

- [C++ Action包指南](src/mtc_ros2_action_cpp/README.md)：当前三个Server的启动、线程所有权及安全契约。
- [Phase 2A交付报告](docs/migration/PHASE2A_ROS2_ACTION_REPORT.md)：13包/358项C++/269项Python/15项E2E/358项UBSan实际证据；TSan本轮NOT RUN。
- [AUBO S3 Jazzy驱动Mock手册](docs/setup/AUBO_ROS2_DRIVER_JAZZY.md)：独立驱动环境与后续集成参考。

- [src/mtc_task_cpp/test/test_task_fsm_core.cpp](src/mtc_task_cpp/test/test_task_fsm_core.cpp)：Task FSM 从 INIT 到 CALL_PICK_PLACE 的真实测试使用方式。
- [src/mtc_manipulation_cpp/test/test_pick_place_fsm_v1.cpp](src/mtc_manipulation_cpp/test/test_pick_place_fsm_v1.cpp)：V1 PickPlace 完整 Fake 测试流程。
- [src/mtc_manipulation_cpp/test/test_pick_place_fsm_v2.cpp](src/mtc_manipulation_cpp/test/test_pick_place_fsm_v2.cpp)：V2 工具策略流程。
- [src/mtc_manipulation_cpp/test/test_pick_place_audit.cpp](src/mtc_manipulation_cpp/test/test_pick_place_audit.cpp)：规划输出和新鲜放置观测的安全回归。
- [src/mtc_motion_execution_cpp/test/test_executor_concurrency.cpp](src/mtc_motion_execution_cpp/test/test_executor_concurrency.cpp)：多线程执行和取消测试。
- [docs/migration/FSM_BEHAVIOR_MAPPING.md](docs/migration/FSM_BEHAVIOR_MAPPING.md)：Python ↔ C++ 行为映射。
- [docs/migration/INTERFACE_COMPAT_AUDIT_CPP.md](docs/migration/INTERFACE_COMPAT_AUDIT_CPP.md)：ROS 接口兼容与后续设计缺口。
- [docs/migration/CODEX_CPP_FSM_AUDIT_FIX_REPORT.md](docs/migration/CODEX_CPP_FSM_AUDIT_FIX_REPORT.md)：上一轮审计修复与测试报告。
- [docs/migration/CPP_FSM_FINAL_SAFETY_GATE.md](docs/migration/CPP_FSM_FINAL_SAFETY_GATE.md)：Final Gate 修复、318/269/15 回归及未完成的 TSan 动态验收记录。

---

**维护约定：** [README_FSM.md](README_FSM.md) 为正式开发者指南入口。每当 Port ABI/API 或 FSM 安全语义发生变化，负责 PR 的开发者必须同步更新指南和测试证据；版本信息以实际commit/tag为准。`main` 是团队共享的Mock/集成开发基线，不是实机控制许可。
