# C++ FSM 第一阶段最终安全验收

验收日期：2026-10-11。仓库：`/home/robot/meituan_challenge_ws`。
基线：`fix/codex-cpp-fsm-audit@e2c48088b7c58fe34615d636f88da85b2f859ee6`。
交付分支：`fix/codex-cpp-fsm-final-gate`。

结论：本轮三个问题已复现并修复。自审还发现并修复了极小正停稳时长可由
绝对比较容差接受零静止跨度的问题。当前离线逻辑验收 PASS；可以进入第二阶段
ROS 2 Jazzy / MoveIt 2 **Mock** 集成开发。TSan 动态竞态验收尚未完成，不能声称
已证明无数据竞争，也不能据本报告进入实机联调。本轮完成后停止，未开展第二阶段。

## 1. 环境、边界和 Git 检查

开始执行了 `git status --short`、`git branch --show-current`、`git log -6 --oneline`、
`git remote -v`，确认工作树干净、分支和完整 HEAD 与上述基线一致，远端为
`https://github.com/F1u0rite/meituan_challenge_ws`，然后新建交付分支。
使用 `/opt/ros/jazzy/setup.bash`，colcon 工作空间包含12个包；C++17构建。

所有执行均为纯逻辑、Mock、Fake 或离线测试。没有启动真实驱动、连接控制柜、
调用真实 RPC/RTDE、触发实体 IO、SSH 到设备或修改官方驱动验证工作空间。
没有合并/修改 main、重写历史、强制推送或推送远端，没有删除旧工程和历史
build/install/log。软件停止仍是普通控制请求，不是安全急停。

## 2. 修改前复现及失败证据

先添加定向测试而保留基线实现，使用独立的 `build/final_gate_repro`、
`install/final_gate_repro`、`log/final_gate_repro` 编译，再运行相应 GTest filter。
下面4份 XML 共记录9个真实失败用例；不是根据源码推测失败：

| 本地证据文件（工作空间相对路径） | 失败数 | 实际观测 |
|---|---:|---|
| `log/final_gate_repro/stale_cancel_before.gtest.xml` | 2 | A的迟到取消在B运行时 `request_delivered=true`，`stop_calls=1`；B从MOVING变成CANCELLING、未知锁定置位、last_error变为520，B不能正常成功 |
| `log/final_gate_repro/mock_clock_before.gtest.xml` | 2 | 默认Mock经执行器执行不能确认停稳；两个起点同为100的不同ManualClock没有在派发前拒绝，后端时间推进到100.7 |
| `log/final_gate_repro/task_stability_before.gtest.xml` | 2 | `placement_stability_sec=0`校验通过；TaskRequest缺省值实际为0而非3 |
| `log/final_gate_repro/placement_before.gtest.xml` | 3 | 0时长没有立即失败且实际执行7次运动并成功；极小正窗口在首帧零累计时间即通过 |

复现涉及：`GoalCancelGate.StaleGoalCancelDoesNotCallBackend`、
`GoalCancelGate.StaleGoalCancelDoesNotModifyExecutionState`、
`MockClockGate.MockDefaultClockExecutionContract`、
`MockClockGate.MockExecutorClockMismatchRejected`、
`StabilityGate.ZeroStabilityRejected`、`StabilityGate.DefaultStabilityRemainsThreeSeconds`、
`WatchdogAudit.InvalidStabilityNeverStartsMotion`、
`PlacementGate.ZeroStabilityRejectedBeforeFirstFrame`、
`PlacementGate.PositiveWindowNeedsTwoDistinctObservations`。

自审时，在前两个修复提交完成后、停稳容差修复前，又执行
`MockClockGate.TinyPositiveSettleDurationCannotConfirmZeroInterval`：FrozenClock的
`now()`始终为100，`settle_duration_s=1e-12`，实际静止跨度为0，但返回
`success=true / stop_confirmed=true / motion_state_unknown=false`。
该1个失败记录在 `log/final_gate_repro/tiny_stop_interval_before.gtest.xml`。
这是额外边界复现，不能混称为最初9个基线失败之一。

定向执行形式为：

```bash
source /opt/ros/jazzy/setup.bash
build/final_gate_repro/mtc_motion_execution_cpp/test_executor_concurrency \
  --gtest_filter='GoalCancelGate.StaleGoalCancelDoesNotCallBackend:GoalCancelGate.StaleGoalCancelDoesNotModifyExecutionState' \
  --gtest_output=xml:log/final_gate_repro/stale_cancel_before.gtest.xml
```

其余文件分别由 `test_mock_adapter`、`test_task_validation`、`test_pick_place_audit`
的上述用例filter产生。修复后的复跑使用final_gate目录，不覆盖这些失败证据。
额外停稳边界用例修改前使用final_gate可执行文件运行。日志和构建产物保留在本地，
未加入Git；本报告记录关键输出，便于无本地构建目录的读者审查。

## 3. 根因及关键修复

### P0：过期 Goal 取消误停当前 Goal

原 `request_stop()` 虽计算关联匹配，仍无条件修改取消标志、停止计数及执行状态，
并用当前命令ID调用后端。关联校验只限制最后的确认，不能阻止误发停止。
原 `OldGoalCancelCannotReplaceNewOwner` 还把当前Goal因旧取消变为未知视为预期，
因此只保住owner断言不足以证明安全。

执行器现在显式提供 `cancel_goal(id, reason)` 和 `supervisory_stop(reason)`；
兼容 `request_stop(reason, optional_id)` 按是否提供ID路由。空ID或不匹配ID在
同一个状态mutex内直接返回错误210及 `command_id mismatch`，在此之前没有
后端调用、pending计数或任何共享状态修改。

匹配的身份判断和pending计数/取消状态更新在同一临界区完成。已接受的停止
阻止新Goal受理，即使执行结果先返回，也要等待停止调用返回；后端执行/停止
期间不持有执行器状态锁。generation检查、单在途所有权、未知状态黏性锁定、
人工确认要求和独立停稳证据校验继续保留。

MotionAdapter默认关联取消改为显式拒绝，不能自动降级为全局停止。
Mock关联取消也拒绝与当前owner不匹配的ID，并保留派发前的同ID取消预约。
匹配取消、重复取消、结果先返回、后端异常、未确认停稳等原有回归继续执行。
新测试使用真正的worker线程、mutex/CV和实际后端调用计数；迟到取消必须满足
`EXPECT_EQ(adapter->stop_calls, 0)`，同时B成功且后续C可正常受理。

### P1：Mock时基和停稳证据不一致

原Mock默认SteadyClock，但 `advance_time()` 仅能推进ManualClock，瞬时Mock执行
无法产生实际经过的停稳区间。Mock与执行器也未强制共享虚拟时基。

Mock现在默认创建有效ManualClock，通过 `execution_clock()` 提供不可变的时钟
域。执行器未显式注入时采用同一实例；显式传不同实例，即使起点相同，也在
运动派发前拒绝。显式SteadyClock、负数/非有限Mock初始时钟抛出带原因的构造
异常。未来声明 `simulation_only=false` 的后端禁止使用ManualClock授权执行。
虚拟时间仅用于离线Mock，不引入真实等待或sleep。

静止起点在推进停稳区间之前采集，反馈时间在之后采集。命令关联、执行状态、
通信、队列清空、连续静止声明、反馈新鲜度、有限位置/速度和速度阈值均继续
校验。自审补强：实际静止跨度必须有限且严格大于0，比较容差为
`min(1e-9, settle_duration_s * 1e-9)`，不能用绝对EPS接受零跨度。
未缩短默认停稳时间、未放宽证据或伪造SteadyClock流逝。时间戳0仍可表示有效
虚拟纪元；是否停稳由实际正区间及其余证据决定。

### P1：零时长绕过放置稳定核验

C++ Task/PickPlace和Python Task原允许0；Python PickPlace入口缺少有限正值
检查。Task内部缺省0还会把缺省请求导向该路径。原绝对EPS也允许极小正配置
在第一帧的零累计时间成功。

两侧Task/PickPlace现在拒绝NaN、正负Inf、负数和0，错误码100。
Task validation reason统一为 `stability_not_positive`。非法Goal/配置在规划、
运动及工具IO之前失败。缺省内部请求与PickPlace保留3.0秒；Python缺失字段
使用3.0，明确给0仍拒绝，不静默替换。ROSIDL字段完全没有修改，收到字段值0
也应拒绝。3.0秒是当前工程默认值，**不是比赛官方规定的最低时长**。

当前策略接受有限正时长，未新增未经依据的官方下限。成功判据额外要求实际
累计时间大于0，容差为 `min(1e-9, window * 1e-9)`。C++原有新鲜可区分观测、
重复帧拒绝、过期和不稳定重置等判据保持有效。Python本轮只同步参数与零累计
时间语义；历史缓存/规划差异仍按上一轮报告限定，Python E2E不代表C++ ROS集成。

## 4. 修改文件清单（相对本轮基线）

| 文件 | 作用 |
|---|---|
| `src/mtc_core_cpp/include/mtc_core_cpp/clock.hpp` | 共享ManualClock及离线边界注释 |
| `src/mtc_core_cpp/include/mtc_core_cpp/results.hpp` | StopResult附加内部错误码，供明确取消拒绝 |
| `src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/executor_core.hpp` | 两类停止API、时钟域及停止实现声明 |
| `src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/motion_adapter.hpp` | 时钟域接口、关联取消不得全局降级 |
| `src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/mock_motion_adapter.hpp` | Mock时钟接口和默认行为声明 |
| `src/mtc_motion_execution_cpp/src/executor_core.cpp` | 原子身份门禁、时钟校验、正停稳跨度 |
| `src/mtc_motion_execution_cpp/src/mock_motion_adapter.cpp` | 有效共享ManualClock、实际虚拟区间、关联取消 |
| `src/mtc_motion_execution_cpp/test/test_executor_concurrency.cpp` | 真线程取消竞争、旧断言修正 |
| `src/mtc_motion_execution_cpp/test/test_mock_adapter.cpp` | 时基、证据及adapter关联负面测试 |
| `src/mtc_task_cpp/include/mtc_task_cpp/task_types.hpp` | 内部Task/PickPlace Goal缺省3.0 |
| `src/mtc_task_cpp/src/task_validation.cpp` | 严格有限正稳定时长 |
| `src/mtc_task_cpp/test/test_task_validation.cpp` | 参数合法性、缺省值回归 |
| `src/mtc_manipulation_cpp/src/pick_place_fsm.cpp` | 启动校验、正累计时间判据 |
| `src/mtc_manipulation_cpp/test/test_pick_place_audit.cpp` | 非法Goal/配置零运动、首帧边界 |
| `src/mtc_task/mtc_task/task_fsm.py` | TaskRequest缺省3.0 |
| `src/mtc_task/mtc_task/task_validation.py` | 缺省归一化及有限正校验 |
| `src/mtc_task/test/test_task_fsm.py` | Python参数回归 |
| `src/mtc_manipulation/mtc_manipulation/pick_place_fsm.py` | 对等入口校验及正累计时间 |
| `src/mtc_manipulation/test/test_pick_place_fsm.py` | Python非法参数零运动及首帧回归 |
| `tests/verify_cpp_audit_results.py` | 原290身份、本轮全部新增身份及XML执行状态核验 |
| `docs/migration/FSM_BEHAVIOR_MAPPING.md` | D-11新契约及本轮统计 |
| `docs/migration/CPP_FSM_FINAL_SAFETY_GATE.md` | 本报告 |

没有修改 `src/mtc_interfaces` 或任何 `.action/.msg/.srv`，没有删除历史实现、
文档或测试。C++新增virtual方法和结果字段需要依赖包重新编译，不宣称二进制
ABI不变；12包完整构建和跨包测试通过，没有混用旧安装产物的错误API/ABI证据。

## 5. 新增并实际执行的 GTest（28项）

下表全名为 `suite.case`。全部位于已注册的测试可执行文件中，无新增未注册文件。

| suite | case | 场景 |
|---|---|---|
| GoalCancelGate | StaleGoalCancelDoesNotCallBackend | 旧ID后端stop_calls=0及明确错误 |
| GoalCancelGate | StaleGoalCancelPreservesCurrentOwner | 当前owner保持B |
| GoalCancelGate | StaleGoalCancelDoesNotModifyExecutionState | 状态、停稳、未知及错误码不变 |
| GoalCancelGate | StaleGoalCancelCannotInterruptNewGoal | 重复20次旧取消，B/C正常执行，无pending遗留 |
| GoalCancelGate | MatchingGoalCancelStillWorks | 同ID取消送达并独立确认 |
| GoalCancelGate | ConcurrentGoalReplacementAndLateCancel | 两线程、10轮A结束/B运行/迟到A取消，后端0停止 |
| GoalCancelGate | SupervisoryStopStillStopsCurrentGoal | 显式监督停止仍有效 |
| GoalCancelGate | EmptyOrUnknownGoalIdHasNoSideEffect | 空/未知ID拒绝且0停止 |
| GoalCancelGate | CompletedGoalCancelCannotStopLaterCompletedGoal | A不能取消已结束B，匹配B仍受证据约束 |
| GoalCancelGate | CancelRacingBackendDispatchRemainsSticky | 派发前匹配取消预约，虚拟时间和关节均不移动 |
| GoalCancelGate | AdapterScopedCancelCannotStopDifferentOwner | Mock本身拒绝错ID且owner/证据不变 |
| MockClockGate | MockDefaultClockExecutionContract | 默认Mock与默认执行器共享虚拟时基 |
| MockClockGate | MockSharedManualClockSettlementSucceeds | 显式共享实例，实际0.2秒静止证据 |
| MockClockGate | MockInsufficientStationaryIntervalRejected | FrozenClock零区间不能确认 |
| MockClockGate | MockExecutorClockMismatchRejected | 不同实例同起点拒绝，0运动调用 |
| MockClockGate | ExplicitRealTimeOrInvalidMockClockIsRejected | SteadyClock和非法Mock初始时间拒绝 |
| MockClockGate | VirtualClockCannotAuthorizeExecutableBackend | 实际可执行能力不能继承Mock虚拟时间授权 |
| MockClockGate | StaleFeedbackCannotConfirmStop | 完成后反馈过期仍不能确认停止 |
| MockClockGate | TinyPositiveSettleDurationCannotConfirmZeroInterval | 极小正停稳配置不能由EPS接受零区间 |
| StabilityGate | ZeroStabilityRejected | 0拒绝 |
| StabilityGate | NegativeStabilityRejected | 负数拒绝 |
| StabilityGate | NaNStabilityRejected | NaN拒绝 |
| StabilityGate | InfiniteStabilityRejected | 正负Inf拒绝 |
| StabilityGate | PositiveStabilityAccepted | 有限正数接受 |
| StabilityGate | DefaultStabilityRemainsThreeSeconds | 缺省保持3.0 |
| PlacementGate | ZeroStabilityRejectedBeforeFirstFrame | 0 Goal在任何规划/运动前拒绝 |
| PlacementGate | InvalidConfiguredStabilityNeverStartsMotion | 非法配置规划/运动均0次 |
| PlacementGate | PositiveWindowNeedsTwoDistinctObservations | 极小正窗口首帧不成功，新帧且正累计时间才成功 |

用户要求的 `InvalidStabilityNeverStartsMotion` 对应已存在的
`WatchdogAudit.InvalidStabilityNeverStartsMotion`，本轮扩展为0/负数/NaN/正负Inf；
它属于原290身份之一，不重复计入新增。
原 `ExecutorConcurrency.OldGoalCancelCannotReplaceNewOwner` 的断言改为B正常成功
且后端0停止；原测试身份保留，修正了其容忍错误取消的预期。

Python新增11项也被colcon pytest实际收集：

* `TestStabilityFinalGate.test_zero_stability_rejected`
* `TestStabilityFinalGate.test_negative_stability_rejected`
* `TestStabilityFinalGate.test_nan_stability_rejected`
* `TestStabilityFinalGate.test_infinite_stability_rejected`
* `TestStabilityFinalGate.test_positive_stability_accepted`
* `TestStabilityFinalGate.test_default_stability_is_three_seconds`
* `TestStabilityFinalGate.test_explicit_zero_is_not_replaced_with_default`
* `test_final_gate_invalid_stability_never_starts_motion`
* `test_final_gate_invalid_configured_stability_never_starts_motion`
* `test_final_gate_positive_window_requires_positive_elapsed_time`
* `test_final_gate_default_stability_remains_three_seconds`

## 6. 完整构建、回归及身份核验

实际执行命令（未清空旧构建目录）：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log/final_gate build \
  --build-base build/final_gate --install-base install/final_gate
source install/final_gate/setup.bash
colcon --log-base log/final_gate test \
  --build-base build/final_gate --install-base install/final_gate
colcon test-result --test-result-base build/final_gate --all
python3 tests/verify_cpp_audit_results.py --build-base build/final_gate
python3 tests/test_mock_end_to_end.py
git diff --check
```

独立目录首次完整构建12包完成（16.6s）；追加停稳边界修复后再次执行完整工作空间
构建命令，12包完成（2.90s，增量编译），全量测试12包完成（3.16s）。
最终实际输出：

```text
Summary: 12 packages finished [2.90s]
Summary: 12 packages finished [3.16s]
Summary: 608 tests, 0 errors, 0 failures, 0 skipped
PASS: original 290/290 executed; 28 new cases executed
PASS: 318 distinct cases in 16 GTest executables; no failure/error/skip
  mtc_core_cpp: 23
  mtc_manipulation_cpp: 120
  mtc_motion_execution_cpp: 88
  mtc_task_cpp: 87
E2E RESULT: PASS 15 / FAIL 0 / SKIP 0
```

608是colcon XML总项数，包含16个GTest CTest包装条目、接口包3个CTest条目和
2个非空lint结果条目；不能称为608个独立业务场景。
真实测试身份为318 C++ +269 Python，另显式E2E15，共602个测试场景。

| 验证项 | PASS | FAIL | SKIP | NOT RUN | 说明 |
|---|---:|---:|---:|---:|---|
| Jazzy完整工作空间构建 | 12包 | 0 | 0 | 0 | 当前全部源码构建 |
| C++ GTest | 318 | 0 | 0 | 0 | 原290 +新增28，16个可执行文件 |
| Python pytest | 269 | 0 | 0 | 0 | aubo25、manipulation52、motion20、safety70、task63、tool39 |
| 显式离线E2E | 15 | 0 | 0 | 0 | 独立执行，纯Python Mock，无rclpy |
| XML manifest xmllint | 1 | 0 | 0 | 0 | 由mtc_interfaces注册执行，无manifest改动 |
| git diff --check | 1 | 0 | 0 | 0 | 无输出，退出0 |
| UBSan全量回归 | 318 C++及269 Python | 0 | 0 | 0 | 完整工作空间，见下一节 |
| TSan动态竞态回归 | 0 | 启动失败，见下一节 | 0 | 全部测试体 | 无可执行的TSan用例结果 |
| ROS Action / MoveIt Mock图集成 | 0 | 0 | 0 | 本轮未执行 | 第二阶段范围，本轮未扩展 |
| 实机运动与实体IO | 0 | 0 | 0 | 全部 | 明确禁止 |

`mtc_bringup/pytest.xml`和copyright结果均为0项，不能算作新增覆盖。
E2E依赖检查显示可选 `mtc_motion_planning` 模块 `SKIPPED(ModuleNotFoundError)`，
使用脚本自带MockPlanner；15个场景均实际执行，场景SKIP=0。这不能证明真正
MoveIt Planner已接入或ROS Action端到端通过。

身份核验脚本默认基线改为e2c4808及290项，可用 `--baseline` / `--baseline-count`
核验旧257基线。它比较基线身份、当前源码身份和实际结果XML，拒绝缺失、重复、
意外多余、failure/error/skipped以及非 `run/completed` 条目，要求16份GTest XML。
不能仅凭测试文件存在认定通过。本轮原290个身份全部实际执行且没有丢失。

## 7. UBSan / TSan

UBSan实际命令：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log/final_gate_ubsan build \
  --build-base build/final_gate_ubsan --install-base install/final_gate_ubsan \
  --cmake-args '-DCMAKE_CXX_FLAGS=-fsanitize=undefined -fno-sanitize-recover=all -g -O1' \
  -DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=undefined -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=undefined
source install/final_gate_ubsan/setup.bash
colcon --log-base log/final_gate_ubsan test \
  --build-base build/final_gate_ubsan --install-base install/final_gate_ubsan
colcon test-result --test-result-base build/final_gate_ubsan --all
python3 tests/verify_cpp_audit_results.py --build-base build/final_gate_ubsan
```

首次12包构建44.4s。最后补强后全工作空间增量构建5.48s、全量测试2.62s；
608总项、0 errors、0 failures、0 skipped，318个GTest身份全部完成，269 Python
完成，无UBSan诊断。抽查动态链接确认使用该目录的C++库及 `libubsan.so.1`。
**UBSan PASS**；Python测试通过不意味着Python代码接受了C++ sanitizer检测。

TSan实际尝试core及motion包（故障发生后未扩展到剩余包）：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log/final_gate_tsan build \
  --packages-select mtc_core_cpp mtc_motion_execution_cpp --executor sequential \
  --build-base build/final_gate_tsan --install-base install/final_gate_tsan \
  --cmake-args '-DCMAKE_CXX_FLAGS=-fsanitize=thread -g -O1' \
  -DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=thread -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread
source install/final_gate_tsan/setup.bash
colcon --log-base log/final_gate_tsan test \
  --packages-select mtc_core_cpp mtc_motion_execution_cpp \
  --build-base build/final_gate_tsan --install-base install/final_gate_tsan
colcon test-result --test-result-base build/final_gate_tsan --all
```

两包构建成功；最终复试7个GTest可执行文件均没有正常生成结果。colcon记录
`14 tests, 7 errors, 7 failures, 0 skipped`，这是7个启动失败包装条目和7个
`missing_result`错误，不是14个已执行的业务测试失败。并发测试实际错误保留于：
`build/final_gate_tsan/mtc_motion_execution_cpp/test_results/mtc_motion_execution_cpp/test_executor_concurrency.gtest.xml`：

```text
FATAL: ThreadSanitizer: unexpected memory mapping 0x79d69e672000-0x79d69eb00000
```

其余实际记录见 `build/final_gate_tsan/*/Testing/Temporary/LastTest.log` 和
`log/final_gate_tsan`。个别文件仅有missing_result，无详细错误，不能据此声称
每个文件都输出相同mapping错误。**启动尝试 FAIL；TSan测试体/完整竞态回归 NOT RUN**。
没有修改ASLR或系统安全设置，没有将普通多线程GTest PASS替代TSan PASS。

## 8. 自审、风险及第二阶段前置条件

本轮自审逐项检查错ID早返回、pending门禁、generation、执行结果/取消互相
交错、后端无状态锁调用、Mock取消预约和完成后证据、时间边界、非法参数前置
拒绝及跨包声明/调用。范围内未留下已知未修复的上述P0问题。
普通多线程测试无死锁、重复发令或误停新Goal；严格证据及重复帧/超时等原有
回归通过，但测试结果不是实机安全证明。

第二阶段开发前需明确并落实：

1. 在支持的TSan环境复跑本轮及原有并发测试；当前动态竞态证据缺口仍开放。
2. ROS Goal取消必须绑定原command_id调用 `cancel_goal`；不能把迟到的Goal取消
   接到无ID监督停止。节点关闭/线程退出必须等待所有执行和停止回调结束，
   adapter/executor生命周期覆盖所有worker；Task/PickPlace核心仍由串行事件线程拥有。
3. 真实可执行adapter必须实现关联取消与派发竞态保留，采用同一真实单调时基，
   明确ROS时间戳转换。停稳证据须来自控制器反馈、队列状态和连续静止采样，
   不能用Action accepted/canceled填充；虚拟时间仅限simulation_only离线路径。
4. 接入真正Planner后保留完整规划输出、6D Pose/frame/stamp/约束；多点轨迹作为
   一条FollowJointTrajectory提交，不能拆成单点运动。配置并校验实机限位、
   速度/加速度、TCP/负载、反馈年龄和项目稳定参数。本轮未新增Planner或ROSIDL。
5. FAULT/未知状态保持锁定；人工恢复接口仍需第二阶段明确授权、现场证据和
   审计记录。自动流程不得调用现有manual confirmation方法伪造确认。
6. 命令ID/handshake记录仍为进程内管理，长时间运行的记录保留策略、跨进程
   唯一性和重启时的控制器状态重新关联需集成阶段设计；不能以进程重启解除未知运动。
7. Python参考实现历史缓存和规划差异仍见上一轮审计报告。本轮窄范围同步不使
   Python E2E成为C++可执行安全路径或实机许可。

建议准入仅为ROS 2 Jazzy + MoveIt 2 Mock集成开发；在上述证据及接口闭环前，
不宣称通过第二阶段验收，不批准真实机械臂或实体工具联调。

## 9. 提交与工作树

本轮代码提交：

* `7a94baa` — `fix(motion): isolate goal cancellation and share offline mock clock`
* `a8fedaf` — `fix(fsm): require finite positive placement stability in C++ and Python`
* `1edcd5ccd2325048d6b04a0c67b1ebb75f9e656b` — `fix(motion): reject zero observed stationary intervals`

行为映射、身份核验脚本和本报告作为后续文档提交。文档提交自身ID通过
`git log -1 -- docs/migration/CPP_FSM_FINAL_SAFETY_GATE.md`取得（避免把自身哈希
写入自身造成递归）。交付时再次核验分支、`git diff --check`和
`git status --short`；工作树干净。最终回复提供文档提交HEAD。
没有合并main，没有推送远端；旧审计提交、报告、测试和构建证据均保留。
