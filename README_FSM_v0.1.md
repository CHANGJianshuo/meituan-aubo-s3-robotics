# FSM Core v0.1 开发者使用指南（版本化入口）

正式入口为 [README_FSM.md](README_FSM.md)；本文保留版本化文件名，内容同步至main合并后的基线。

> **当前状态：FSM Core v0.1 已完成第一阶段离线安全验收并合入 `main`。** 合并提交为 `c2be85724111f6fb4dfcc62a2c974f7be1233cba`；已存在的本地 Git tag `fsm-core-v0.1.0` 指向该提交。本指南依据合入的 C++ API 与 Final Gate 报告编写。第二阶段 ROS 2 Action、MoveIt 2 Planner 与 Mock Hardware 集成尚未完成。
>
> **适用范围：C++17 FSM Core / 离线 Mock 开发，不是已完成的 ROS 2 / MoveIt 2 / AUBO S3 实机控制系统。** 严禁将本文中的示例改为直接驱动真实机械臂。

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
     [第二阶段：ROS 2 Action / MoveIt 2 / FollowJointTrajectory / S3 Fake Hardware]
```

### 四个 C++ 包

| 包 | 职责 | 代码入口 |
|---|---|---|
| `mtc_core_cpp` | 共享类型、错误码、6D Pose、TimedTrajectory、时钟与结果契约 | [src/mtc_core_cpp/include/mtc_core_cpp/](src/mtc_core_cpp/include/mtc_core_cpp/) |
| `mtc_task_cpp` | 整轮比赛 Task FSM、任务校验和动作意图 | [src/mtc_task_cpp/include/mtc_task_cpp/task_fsm.hpp](src/mtc_task_cpp/include/mtc_task_cpp/task_fsm.hpp) |
| `mtc_manipulation_cpp` | 单块电池 PickPlace FSM、V1/V2 ToolStrategy、Port 接口 | [src/mtc_manipulation_cpp/include/mtc_manipulation_cpp/pick_place_fsm.hpp](src/mtc_manipulation_cpp/include/mtc_manipulation_cpp/pick_place_fsm.hpp)、[Port契约](src/mtc_manipulation_cpp/include/mtc_manipulation_cpp/pick_place_types.hpp) |
| `mtc_motion_execution_cpp` | 单在途命令控制、校验、停稳证据、Mock/Disabled 运动适配器 | [src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/](src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/) |

**已实现：** 核心状态机和共享 C++ API；V1/V2 工具策略；纯逻辑 Fake 与 Mock 单元测试；单命令/整条轨迹的数据表达与保守执行校验。

**未实现/未验收：** 新 C++ FSM 的 ROS 2 Action Server、MoveIt 2 C++ Planner、`FollowJointTrajectory` 端到端适配、完整 S3 Fake Hardware 抓放闭环和真实硬件控制。仓库保留部分历史 Python ROS 节点，但不等于新 C++ FSM 已经接入了这些节点。

## 2. 开发基线与团队使用方式

`main` 已合并 Final Gate 修复，`fsm-core-v0.1.0` 为当前已存在的本地版本标签，
目标提交为 `c2be857`。过期 Goal 误停、Mock 时基、零放置时长及零静止跨度等
安全问题已有修复和负面测试；这份基线用于离线开发及下一阶段 Mock 集成。
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
实际依赖见各包 `package.xml` / `CMakeLists.txt`；核心库不依赖ROS运行时。
下面各组命令在仓库根目录执行，不清空已有build/install/log。

### 完整工作空间

```bash
source /opt/ros/jazzy/setup.bash
cd /home/robot/meituan_challenge_ws
colcon list
colcon --log-base log/final_gate build \
  --build-base build/final_gate --install-base install/final_gate
source install/final_gate/setup.bash
colcon --log-base log/final_gate test \
  --build-base build/final_gate --install-base install/final_gate
colcon test-result --test-result-base build/final_gate --all
python3 tests/verify_cpp_audit_results.py --build-base build/final_gate
python3 tests/test_mock_end_to_end.py
```

身份核验脚本检查原290项及当前全部318项是否真的执行，拒绝缺失、重复、失败、
跳过或未完成项。E2E脚本未被colcon注册，必须单独执行；它是纯Python Mock，
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
python3 tests/verify_cpp_audit_results.py --build-base build/fsm_core
```

此入口不执行Python包、XML或E2E，不能据此宣称全工作空间回归通过。

### 定向 GTest

先完成相应目录的构建。在加载 `install/final_gate/setup.bash` 的终端可执行：

```bash
build/final_gate/mtc_motion_execution_cpp/test_executor_concurrency \
  --gtest_filter='GoalCancelGate.*'
build/final_gate/mtc_motion_execution_cpp/test_mock_adapter \
  --gtest_filter='MockClockGate.*'
build/final_gate/mtc_manipulation_cpp/test_pick_place_audit \
  --gtest_filter='PlacementAudit.*:PlacementGate.*:WatchdogAudit.*'
```

定向运行不能替代colcon全量结果与身份核验。

### 已记录的验收结果与 sanitizer

以下结果来自 [2026-10-11最终安全验收报告](docs/migration/CPP_FSM_FINAL_SAFETY_GATE.md)，
本次README更新没有重新执行全量回归：

| 验证 | 记录结果 |
|---|---|
| Jazzy全工作空间编译 | 12/12包 PASS |
| C++ GTest | 318 PASS，原290项全部保留，新增28项，16个可执行文件 |
| Python pytest | 269 PASS |
| 显式离线E2E | 15 PASS / FAIL 0 / SKIP 0 |
| XML、UBSan、git diff --check | PASS |
| TSan | 未完成，NOT RUN；运行时启动报 `unexpected memory mapping` |
| C++ ROS Action / MoveIt Planner / Mock Hardware集成 | 尚未完成，NOT RUN |
| 实机与实体IO | NOT RUN，未授权 |

C++包分布：core23、task87、manipulation120、motion88。
318+269+15为602个测试场景；colcon汇总608项包含包装/lint条目，不能重复计数。
E2E使用脚本MockPlanner，可选规划模块缺失不代表真正Planner集成通过。
UBSan/TSan独立构建目录及实际错误记录见上述报告第7节。复跑sanitizer时沿用
独立目录，不修改系统ASLR或安全设置；TSan启动失败不能标为PASS。

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

## 6. 与 ROS 2 的关系：接口存在 ≠ C++ Action 已集成

仓库有 `mtc_interfaces` 中的 `ExecuteTask.action`、`PickPlace.action`、`ExecuteJointMove.action` 等契约，旧 Python 包也有 ROS 2 适配代码；但**当前新 C++ FSM 核心库没有实现对应的 C++ `rclcpp_action` Server，也没有自动接入 MoveIt 2**。这是下一阶段的交付范围。

未来集成时：

- `ExecuteTask` Goal → `TaskRequest` → `TaskFsm`，`StepResult.actions` → ROS 2 子操作。
- `PickPlace` Goal → `PickPlaceFsm::start()`；把 `feedback()` 转成 Action Feedback，把 `result()` 转成 Result。
- Perception 的 `geometry_msgs/PoseStamped` → 完整 `Pose6D`，必须核对 TF、坐标系和时间戳。
- MoveIt 2 完整时间参数化轨迹 → 一次 `control_msgs/action/FollowJointTrajectory` Goal；禁止多次 `ExecuteJointMove` 伪造轨迹。
- `Goal accepted` ≠ `motion complete`；`cancel accepted` ≠ `stop confirmed`；`placement_verified` 只有真正连续新观测通过才可置真。

在完成正式 ROS 2 Mock 闭环前，不得宣称新的 C++ FSM 已有“开箱即用的机器人控制节点”。

## 7. 队友开发分工及 PR 验收

推荐以已合入的 `main`/`fsm-core-v0.1.0` 作为共同基线，在独立分支实现：

| 方向 | 建议分支 | 首先交付 |
|---|---|---|
| 感知 | `feature/perception-adapter` | `PerceptionPort` Mock/转换测试、6D Pose、TF/时间戳拒绝场景 |
| 规划 | `feature/moveit-planner` | `PlannerPort` Mock/规划输出测试，`PlanResult.commands` 完整性 |
| 运动执行 | `feature/motion-controller` | Mock `FollowJointTrajectory` 单 Goal + 取消/停稳证据测试 |
| ROS 2 | `feature/ros2-action` | `ExecuteTask`/`PickPlace` C++ Action 外壳，串行 FSM 事件队列 |
| 工具 | `feature/tool-adapter` | V1/V2 `ToolPort` Fake/Mock 与解锁联锁测试 |

**提交 PR 前：** 必须附上相应 `colcon build`、`colcon test` 的实际结果，说明是否触及 IDL、TF、并发/停稳契约；包含负面测试；所有实机能力继续默认禁用。接口变更先讨论，不要各自修改同一 `.action/.msg/.srv` 使契约漂移。

## 8. 常见错误与注意事项

- **找不到包或头文件：** 确认已经 `source /opt/ros/jazzy/setup.bash`，构建后在当前终端 `source install/final_gate/setup.bash`（仅核心目录则加载 `install/fsm_core/setup.bash`），消费者在 `package.xml`/CMake 中声明所依赖的 C++ 包。
- **`test/fakes.hpp` 找不到：** 测试 Fake 不属于安装的公共 include 目录；不要把它当作正式后端。
- **FSM 一直等待：** 检查 Port 是否返回 `std::nullopt`、是否由外部事件循环继续 tick、是否向 FSM 投递了对应完成事件；不要绕过看门狗。
- **规划器报告 `ok=true` 但不执行：** `PlanResult.commands` 必须与请求原语一一对应，并满足完整轨迹/关节目标校验；不能只返回 `ok`。
- **取消已请求但结果是 FAULT：** 停止请求被受理不等于后端已确认停稳，检查 StopEvidence 与关联 command_id；不要改写结果掩盖故障。
- **Mock 运行时钟：** Final Gate 后 `MockMotionAdapter` 默认创建 `ManualClock`，`MotionExecutorCore` 若未显式传入时钟，会沿用适配器的同一实例；如显式注入时钟，**必须是同一 `ClockPtr` 实例**。不同 `ManualClock` 即使起点相同也会被拒绝；Mock 不支持 `SteadyClock`。
- **想直接控制 S3：** 此 README 未授权任何实体控制；当前仅完成纯逻辑Mock。使用 `mock_components/GenericSystem` 的C++ FSM集成属于尚未完成的第二阶段。

## 9. 源码和进一步阅读

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
