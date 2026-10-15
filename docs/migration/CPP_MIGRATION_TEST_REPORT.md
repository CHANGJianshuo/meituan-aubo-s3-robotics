# C++ 架构迁移第一阶段：编译与测试报告

| 项目 | 值 |
|---|---|
| 报告时间 | 2026-10-11（Asia/Hong_Kong） |
| 分支 | `feature/jazzy-cpp-fsm`（基线 `main@bd914fb`） |
| 工作空间 | `/home/robot/meituan_challenge_ws` |
| 操作系统 | Ubuntu 24.04.4 LTS (noble) |
| ROS 2 | Jazzy（`/opt/ros/jazzy`） |
| 编译器 | g++ 13.3.0 |
| CMake | 3.28.3 |
| C++ 标准 | C++17（`CMAKE_CXX_EXTENSIONS OFF`） |
| 构建方式 | `colcon build`（**从零**：先删除 `build/ install/ log/`） |
| 测试方式 | `colcon test` + `colcon test-result --all` |

> **执行声明**：本报告中的每一个 PASS 都对应一次真实执行。
> **未执行**的项目一律标注 **NOT RUN**，绝不以"应该能通过"代替证据。

---

## 1. 结论摘要

| 判定 | 数量 | 说明 |
|---|---|---|
| **PASS** | 512 个测试用例 | 257 个新增 C++ GTest + 255 个既有 Python 基线 pytest |
| **FAIL** | 1 项检查（重复计数为 2） | `mtc_interfaces` 的 `xmllint` —— **既有缺陷，非本次引入** |
| **NOT RUN** | 见 §6 | 实机、MoveIt 集成、ROS Action 外壳、端到端 Mock、`tests/` 下 15 例 |

**关键结论**：
1. 4 个新增 C++ 包**从零全量构建通过**，无编译错误、**无警告**（`-Wall -Wextra -Wpedantic`）。
2. 新增 C++ 测试 **257/257 PASS，0 skip**。
3. 既有工程**未受任何影响**：12 个包全部构建成功，既有 Python 基线测试 **255/255 PASS**。
4. 唯一的 FAIL 来自既有 `mtc_interfaces/package.xml` 的结构问题，该文件
   **与 `main` 逐字节一致**（`git diff main` 为空），本次未修改。

---

## 2. 构建结果

### 2.1 干净构建命令

```bash
cd /home/robot/meituan_challenge_ws
rm -rf build/ install/ log/          # 从零验证可复现性
source /opt/ros/jazzy/setup.bash
colcon build
```

### 2.2 构建输出

```
Starting >>> mtc_core_cpp
Finished <<< mtc_core_cpp [4.01s]
Starting >>> mtc_motion_execution_cpp
Starting >>> mtc_task_cpp
Finished <<< mtc_motion_execution_cpp [5.83s]
Finished <<< mtc_task_cpp [5.84s]
Finished <<< mtc_interfaces [11.7s]
Starting >>> mtc_aubo_bridge
Starting >>> mtc_manipulation
Starting >>> mtc_motion_execution
Starting >>> mtc_safety
Starting >>> mtc_task
Starting >>> mtc_tool
Finished <<< mtc_aubo_bridge [1.76s]
Finished <<< mtc_task [1.76s]
Finished <<< mtc_motion_execution [1.81s]
Finished <<< mtc_manipulation [1.92s]
Finished <<< mtc_safety [2.10s]
Finished <<< mtc_tool [2.19s]
Starting >>> mtc_bringup
Finished <<< mtc_bringup [0.98s]
Finished <<< mtc_manipulation_cpp [5.12s]

Summary: 12 packages finished [15.1s]
```

| 判定 | 内容 |
|---|---|
| **PASS** | 12/12 包构建成功 |
| **PASS** | 编译告警数：**0**（新增 C++ 包在 `-Wall -Wextra -Wpedantic` 下） |
| **PASS** | 依赖解析：新增 4 包按拓扑正确排序（core → motion_execution/task → manipulation） |

### 2.3 新增包清单

| 包 | 类型 | 源文件 | 依赖 |
|---|---|---|---|
| `mtc_core_cpp` | ament_cmake | 1 `.cpp` + 7 `.hpp` | **仅 C++17 标准库** |
| `mtc_task_cpp` | ament_cmake | 3 `.cpp` + 3 `.hpp` | `mtc_core_cpp` |
| `mtc_motion_execution_cpp` | ament_cmake | 5 `.cpp` + 3 `.hpp` | `mtc_core_cpp` |
| `mtc_manipulation_cpp` | ament_cmake | 4 `.cpp` + 3 `.hpp` | `mtc_core_cpp`, `mtc_motion_execution_cpp` |

**无 `rclcpp` 依赖**（经 grep 验证：源码中不含 `#include <rclcpp...>`）。
**无 SDK / 设备 / 网络访问**（经 grep 验证：不含 `aubo_sdk`、`socket`、`robot_ip`、
`serial`、`/dev/tty`）。

---

## 3. 新增 C++ 测试结果

### 3.1 汇总

| 包 | GTest 可执行文件 | 用例数 | 结果 |
|---|---|---|---|
| `mtc_core_cpp` | `test_core_types` | 20 | **PASS** |
| | `test_error_codes_msg_parity` | 1 | **PASS** |
| `mtc_task_cpp` | `test_task_validation` | 19 | **PASS** |
| | `test_task_fsm_core` | 21 | **PASS** |
| | `test_task_fsm_safety` | 39 | **PASS** |
| `mtc_motion_execution_cpp` | `test_in_flight_guard` | 6 | **PASS** |
| | `test_validation` | 18 | **PASS** |
| | `test_mock_adapter` | 17 | **PASS** |
| | `test_executor_core` | 14 | **PASS** |
| `mtc_manipulation_cpp` | `test_pick_place_tables` | 20 | **PASS** |
| | `test_tool_strategy` | 15 | **PASS** |
| | `test_pick_place_fsm_v1` | 13 | **PASS** |
| | `test_pick_place_fsm_v2` | 14 | **PASS** |
| | `test_pick_place_safety` | 40 | **PASS** |
| **合计** | **14 个可执行文件** | **257** | **257 PASS / 0 FAIL / 0 SKIP** |

`colcon test-result` 原始输出（新增包部分）：

```
build/mtc_core_cpp/test_results/mtc_core_cpp/test_core_types.gtest.xml: 20 tests, 0 errors, 0 failures, 0 skipped
build/mtc_core_cpp/test_results/mtc_core_cpp/test_error_codes_msg_parity.gtest.xml: 1 tests, 0 errors, 0 failures, 0 skipped
build/mtc_task_cpp/test_results/mtc_task_cpp/test_task_fsm_core.gtest.xml: 21 tests, 0 errors, 0 failures, 0 skipped
build/mtc_task_cpp/test_results/mtc_task_cpp/test_task_fsm_safety.gtest.xml: 39 tests, 0 errors, 0 failures, 0 skipped
build/mtc_task_cpp/test_results/mtc_task_cpp/test_task_validation.gtest.xml: 19 tests, 0 errors, 0 failures, 0 skipped
build/mtc_motion_execution_cpp/.../test_executor_core.gtest.xml: 14 tests, 0 errors, 0 failures, 0 skipped
build/mtc_motion_execution_cpp/.../test_in_flight_guard.gtest.xml: 6 tests, 0 errors, 0 failures, 0 skipped
build/mtc_motion_execution_cpp/.../test_mock_adapter.gtest.xml: 17 tests, 0 errors, 0 failures, 0 skipped
build/mtc_motion_execution_cpp/.../test_validation.gtest.xml: 18 tests, 0 errors, 0 failures, 0 skipped
build/mtc_manipulation_cpp/.../test_pick_place_fsm_v1.gtest.xml: 13 tests, 0 errors, 0 failures, 0 skipped
build/mtc_manipulation_cpp/.../test_pick_place_fsm_v2.gtest.xml: 14 tests, 0 errors, 0 failures, 0 skipped
build/mtc_manipulation_cpp/.../test_pick_place_safety.gtest.xml: 40 tests, 0 errors, 0 failures, 0 skipped
build/mtc_manipulation_cpp/.../test_pick_place_tables.gtest.xml: 20 tests, 0 errors, 0 failures, 0 skipped
build/mtc_manipulation_cpp/.../test_tool_strategy.gtest.xml: 15 tests, 0 errors, 0 failures, 0 skipped
```

> **0 skipped 的意义**：`test_error_codes_msg_parity` 在接口定义不可读时会 `GTEST_SKIP`。
> 本报告中 skip = 0，说明该测试**真实执行了**与 `ErrorCodes.msg` 的双向一致性校验。

### 3.2 需求 6 要求覆盖的场景 → 对应测试

| 需求 6 场景 | 覆盖测试（部分） | 结果 |
|---|---|---|
| 正常抓放流程 | `PickPlaceV1.HappyPathReachesSuccess`、`PickPlaceV1.StateSequenceMatchesDesignOrder`、`PickPlaceV2.HappyPathReachesSuccessAndUnlocksOnce` | PASS |
| V1/V2 策略 | `PickPlaceV2.DifferencesAreConfinedToEngageAndDisengage`、`ToolStrategyV1.*`(7)、`ToolStrategyV2.*`(6) | PASS |
| 目标丢失 | `TargetNotFoundIsRecoverableThenFails`、`TargetNotUniqueIsRejected`、`StalePoseIsRejectedWithPoseStale`、`InvalidRingPoseBlocksMotion`、`UnknownColorIsRejectedEvenWhenExpected`、`ColorMismatchIsRejectedToPreventWrongPick` | PASS |
| 规划失败 | `PlanningFailureIsRecoverable`、`TransportPlanningFailureGoesToFaultAfterAttachment` | PASS |
| 动作超时 | `MotionInFlightTimeoutGoesToFault230AndLocksResend`、`MotionTimeoutLocksAndDoesNotResend`、`NonMotionStateTimeoutReportsMotionTimeout`、`NoTimeoutBeforeDeadline`、`TaskFsmTimeout.*`(6) | PASS |
| 取消 | `CancelWithConfirmedStopReachesCancelled`、`CancelDuringMotionStillRequiresConfirmation`、`TaskFsmCancel.*`(7) | PASS |
| 停止确认失败 | `CancelWithoutStopConfirmationGoesToFault520`、`CancelNeverReachesCancelledWithoutConfirmation`、`RequestStopIsCalledAtMostOnce`、`ExecutorCore.StopDeliveryIsNotConfirmationWhenBackendCannotConfirm`、`ExecutorCore.UnsupportedCancelNeverFabricatesStopConfirmed`、`ExecutorCore.StopAfterUnknownStateIsNeverConfirmed` | PASS |
| 重复 Goal | `TaskFsmConcurrency.SecondGoalRejectedInEveryActiveState`（遍历 6 个活动状态）、`RejectionDoesNotQueueSecondGoal`、`MissingGoalPayloadRejected`、`UnparsableGoalRejected`、`EmptyTaskIdRejected`、`ExecutorCore.ConcurrentGoalDuringMotionIsRejected`、`InFlightGuard.SecondAcquireIsRejectedWithInFlightId` | PASS |
| 状态未知锁定与恢复 | `MotionStatusUnknownLocksAndDoesNotResend`、`RecoveryIsRefusedWhenMotionResendDisallowed`、`TaskFsmUnsafeErrors.MotionStatusUnknownGoesToFaultWithRealCode`、`TaskFsmUnsafeErrors.NonRecoverableCodesAllGoToFault`(8 码遍历)、`ExecutorCore.StatusUnknownLocksSubsequentRequests`、`ExecutorCore.ManualConfirmationClearsUnknownLock`、`ExecutorCore.BackendExceptionIsTreatedAsUnknownState` | PASS |
| `placement_verified` 唯一写入口 | `PlacementVerifiedCannotBeWrittenOutsideVerifyPlaced`（违规写入抛 `std::logic_error`）、`PlacementVerifiedOnlyTrueAfterSuccessfulVerifyPlaced` | PASS |
| V2 解锁前置条件 | `NoUnlockWhenSeatNotVerified`、`NoUnlockWhenUnloadNotVerified`、`StrategyRefusalBlocksUnlockIoEntirely`、`UnlockFailureGoesToFaultAndDoesNotRetreat`、`AcceptedPulseWithoutReleaseIsStillFailure`、`DisengageReleaseUnknownGoesToFaultAndDoesNotRetreat`、`UnlockHappensOnlyInsideDisengageAfterUnload` | PASS |
| 在途不重发 | `InFlightCommandIsReusedNeverResent`（断言 3 次调用复用同一 `command_id`） | PASS |
| 轨迹不得拆分为单点 | `TrajectoryIsRejectedAndNotSplitIntoJointMoves`（断言 `call_log()` 为空） | PASS |
| 6D 位姿保真 | `CorePose.*`(5)、`PrimitivePreservesSixDofTargetPose`、`PickPlaceV1.PlannerReceivesBothPhasesExactlyOnce` | PASS |

---

## 4. 既有工程回归结果（确认未破坏）

### 4.1 Python 基线测试（**本次实际执行**）

| 包 | 用例数 | 结果 |
|---|---|---|
| `mtc_task` | 56 | **PASS** |
| `mtc_manipulation` | 45 | **PASS** |
| `mtc_motion_execution` | 20 | **PASS** |
| `mtc_safety` | 70 | **PASS** |
| `mtc_tool` | 39 | **PASS** |
| `mtc_aubo_bridge` | 25 | **PASS** |
| `mtc_bringup` | 0（无 pytest 用例） | — |
| **合计** | **255** | **255 PASS / 0 FAIL** |

`colcon test-result` 原始输出：

```
build/mtc_aubo_bridge/pytest.xml:    25 tests, 0 errors, 0 failures, 0 skipped
build/mtc_manipulation/pytest.xml:   45 tests, 0 errors, 0 failures, 0 skipped
build/mtc_motion_execution/pytest.xml: 20 tests, 0 errors, 0 failures, 0 skipped
build/mtc_safety/pytest.xml:         70 tests, 0 errors, 0 failures, 0 skipped
build/mtc_task/pytest.xml:           56 tests, 0 errors, 0 failures, 0 skipped
build/mtc_tool/pytest.xml:           39 tests, 0 errors, 0 failures, 0 skipped
```

> 这同时为 [`FSM_BEHAVIOR_MAPPING.md`](FSM_BEHAVIOR_MAPPING.md) §10.1 的"行为基线"
> 提供了**执行证据**（此前审计阶段为 NOT RUN）。

### 4.2 既有文件未被修改

```bash
$ git status --short
?? docs/migration/AUBO_DRIVER_FAKE_HARDWARE_FIX_AUDIT.md
?? docs/migration/FSM_BEHAVIOR_MAPPING.md
?? docs/migration/INTERFACE_COMPAT_AUDIT_CPP.md
?? docs/migration/PYTHON_BEHAVIOR_BASELINE.md
?? docs/migration/patches/
?? src/mtc_core_cpp/
?? src/mtc_manipulation_cpp/
?? src/mtc_motion_execution_cpp/
?? src/mtc_task_cpp/
```

全部为 `??`（新增未跟踪），**无 `M`（修改）、无 `D`（删除）、无 `R`（重命名）**。
→ 既有 Python 参考实现、设计文档、`mtc_interfaces`、配置文件均**逐字节未变**。

---

## 5. FAIL 项详情（既有缺陷，非本次引入）

### 5.1 `mtc_interfaces` xmllint

```
File 'package.xml' is invalid:
/home/robot/meituan_challenge_ws/src/mtc_interfaces/package.xml:24:
  element test_depend: Schemas validity error : Element 'test_depend': This element is
  not expected. Expected is one of ( member_of_group, export ).
/home/robot/meituan_challenge_ws/src/mtc_interfaces/package.xml fails to validate
```

**成因**：`package.xml`（format 3）中 `<test_depend>` 出现在 `<member_of_group>` **之后**，
不符合 RELAX NG schema 规定的元素顺序（`test_depend` 必须位于 `member_of_group` 与
`export` 之前）。

**归属证据**：

```bash
$ git diff main -- src/mtc_interfaces/package.xml
（无输出）
$ git diff main --stat -- src/mtc_interfaces/
（无输出）
```

→ 该文件与 `main` **完全一致**，本次迁移**未触碰**。

**影响**：

* 仅影响 `ament_xmllint` 静态检查，**不影响**消息/服务/Action 的生成与使用
  （`mtc_interfaces` 构建成功、8 个下游包全部构建成功）。
* 不影响本次新增的 C++ 实现（C++ 侧不依赖 `package.xml` 而是直接使用生成的头文件）。

**建议**：作为**独立变更**提交审计。修复方式是调整元素顺序（约 3 行位置移动），
不改变任何字段或语义。**本次未擅自修改**，理由见
[`FSM_BEHAVIOR_MAPPING.md`](FSM_BEHAVIOR_MAPPING.md) §9 的同类原则——
接口包的任何改动都应由审计决定，避免混入迁移提交。

---

## 6. NOT RUN 声明

以下项目**本次未执行**，不构成任何结论：

| # | 项目 | 原因 |
|---|---|---|
| N1 | 实机 AUBO S3 连接与运动 | **任务禁止**（严格限制条款）；无 `robot_ip` 连接、无设备访问 |
| N2 | 真实工具 IO / 电磁解锁实体动作 | **任务禁止**（不得触发实体工具或更改设备安全设置） |
| N3 | MoveIt 2 C++ API 集成 | 不在第一阶段范围（第二阶段交付） |
| N4 | ROS 2 Action 外壳（goal/cancel/result 绑定） | 不在第一阶段范围（第二阶段交付） |
| N5 | `Mock` 端到端集成（C++ FSM ↔ ros2_control ↔ mock_components） | 不在第一阶段范围（第二阶段交付） |
| N6 | `tests/test_mock_end_to_end.py`（15 例） | 该文件**未被任何包注册**，`colcon test` 不收集；本次未单独运行 |
| N7 | 官方驱动工作空间的 Mock 复跑 | 严格遵守"仅用于读取"；本次未启动其任何 launch |
| N8 | 实机路径 `aubo_driver/AuboHardwareInterface` | 从未启动；`use_fake_hardware=false` 分支未执行 |
| N9 | 多点时间参数化轨迹的实际执行 | 第一阶段仅提供类型与"显式拒绝"语义；无后端声明支持 |

---

## 7. 复现步骤

```bash
# 1) 环境
source /opt/ros/jazzy/setup.bash        # 注意：不要用 Humble

# 2) 从零构建
cd /home/robot/meituan_challenge_ws
rm -rf build/ install/ log/
colcon build

# 3) 仅运行新增 C++ 测试
colcon test --packages-select \
  mtc_core_cpp mtc_task_cpp mtc_motion_execution_cpp mtc_manipulation_cpp
colcon test-result --all

# 4) 全量（含既有 Python 基线）
colcon test
colcon test-result --all

# 5) 单独查看新增包的用例明细
for t in build/mtc_*_cpp/test_*; do [ -x "$t" ] && "$t" --gtest_brief=1; done
```

**期望结果**：
* `colcon build`：12 packages finished，0 error，0 warning
* 新增 C++ 包：257 tests, 0 failures, 0 skipped
* 既有 Python 包：255 tests, 0 failures, 0 skipped
* `mtc_interfaces`：2 failures（xmllint，既有）

### 7.1 环境注意事项

* 当前 shell 的 `ROS_DISTRO` 默认可能是 `humble`，构建前必须 `source /opt/ros/jazzy/setup.bash`
  并确认 `echo $ROS_DISTRO` 为 `jazzy`，否则会混用不同发行版的路径。
* 若 `colcon build` 报 `mtc_core_cpp/error_codes.hpp: No such file or directory`，
  说明 `mtc_core_cpp` 未被先构建或 `install/` 未 source；应先
  `colcon build --packages-select mtc_core_cpp`。

---

## 8. 判定矩阵（需求 7 要求）

| 交付物 | 状态 | 证据 |
|---|---|---|
| 新 C++ FSM 核心源码 | **PASS** | 4 个新包，15 个源文件，0 编译告警 |
| `CMakeLists.txt` / `package.xml` 迁移 | **PASS** | 12/12 包构建成功 |
| GTest 单元测试 | **PASS** | 257 例，0 失败，0 跳过 |
| Python FSM 与 C++ FSM 行为映射表 | **PASS** | [`FSM_BEHAVIOR_MAPPING.md`](FSM_BEHAVIOR_MAPPING.md)（含 §9 差异清单 9 项） |
| 现有接口兼容性审计 | **PASS** | [`INTERFACE_COMPAT_AUDIT_CPP.md`](INTERFACE_COMPAT_AUDIT_CPP.md)（609 行） |
| 编译及测试结果 | **PASS** | 本文件 |
| 第二阶段 ROS 2 / MoveIt 集成计划 | **PASS** | [`PHASE2_ROS2_INTEGRATION_PLAN.md`](PHASE2_ROS2_INTEGRATION_PLAN.md) |
| 官方驱动修复审计 | **PASS** | [`AUBO_DRIVER_FAKE_HARDWARE_FIX_AUDIT.md`](AUBO_DRIVER_FAKE_HARDWARE_FIX_AUDIT.md) + 归档补丁（反向应用校验通过） |
| 实机能力 | **NOT RUN** | 任务禁止，见 §6 N1/N2/N8 |
| ROS 2 / MoveIt 集成 | **NOT RUN** | 第二阶段，见 §6 N3/N4/N5 |
