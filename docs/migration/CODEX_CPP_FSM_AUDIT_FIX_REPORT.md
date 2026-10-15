# C++ FSM 第一阶段安全审计修复报告

日期：2026-10-11（Asia/Hong_Kong）。本报告记录实际源码修复与离线验证。

## 1. 结论与边界

P0-1～P0-5 与 P1-1～P1-5 已落实到源码和回归测试。四个 C++ 核心包形成了
进入第二阶段 Mock 集成的保守基线：没有规划输出不运动，没有关联停稳证据
不确认停止，未知/未停稳的超时不重发，重复感知帧不能累计放置稳定时间。
Task 的 FAULT 结束当前执行，但仍拒绝普通复位和新任务。

最终普通 Jazzy 全工作空间构建成功，全部注册回归通过。实际执行 **290 个 C++
用例（257 原有 + 33 新增）**、**258 个 Python 用例（255 原有 + 3 新增）**，
以及显式执行的 **15 个离线端到端场景**。XML manifest 校验通过。
这些都是 Mock/Fake/纯逻辑结果，不代表实机抓放或 ROS Action 联调验收。

ThreadSanitizer 曾直接执行 6 项并发测试并通过，但后续完整运行受到容器地址
映射限制，无法稳定启动，详见 §5；不能宣称完整数据竞争检测通过。

整个任务未连接机器人、未调用真实 RPC/RTDE、未访问实体 IO、未 SSH、未运行
真实硬件分支，未修改 `/home/robot/aubo_driver_validation_ws`，未启动机器人
ROS graph。普通软件 Stop 没有被作为安全急停。没有合并 main 或推送。

## 2. 基线与独立复核

| 检查 | 实际结果 |
|---|---|
| 唯一开发路径 | `/home/robot/meituan_challenge_ws` |
| 初始工作树 | `git status` 干净，没有用户未提交修改 |
| 初始分支 / HEAD | `feature/jazzy-cpp-fsm` / `db9683af2394ce1c529fedddb3c8cb219e12f467` |
| 初始提交与远端检查 | 已执行 `git log -5 --oneline`、`git remote -v` |
| origin | `https://github.com/F1u0rite/meituan_challenge_ws.git` |
| 修复分支 | 从上述 HEAD 创建 `fix/codex-cpp-fsm-audit` |
| ROS 环境 | 初始 shell 的 ROS_DISTRO 未设置；source `/opt/ros/jazzy/setup.bash` 后为 `jazzy` |
| 编译器 / 标准 | GCC 13.3.0 / C++17 |
| 工作空间包 | `colcon list` 共 12 个包，包含 4 个 C++ 包 |

修改前阅读了 `CPP_MIGRATION_TEST_REPORT.md`、`FSM_BEHAVIOR_MAPPING.md`、
`INTERFACE_COMPAT_AUDIT_CPP.md`、`PHASE2_ROS2_INTEGRATION_PLAN.md`，以及四包
头文件、实现、全部原有测试。新增负面测试先在原实现上运行，实际复现四项失败：

| 修改实现前的负面测试 | 基线实测失败 |
|---|---|
| `ExecutorAudit.DeliveryWithoutEvidenceAndWithoutCommandNeverConfirms` | 没有命令关联或停止证据，送达却被报告为停稳 |
| `ExecutorAudit.TimeoutWithoutUnknownFlagStillLocks` | 后端 `timed_out=true/status_unknown=false` 没有锁定，下一次调用没有按 230 拒绝 |
| `PickPlaceAudit.RepeatedFrameForFiveSecondsCannotSucceed` | 重复缓存观测配合时钟推进进入 SUCCESS |
| `PickPlaceAudit.MissingPlanOutputNeverExecutesZeroTarget` | 规划仅返回 ok，仍发出默认关节命令并完成流程 |

其他问题由源码触发条件、旧测试断言和补充回归复核。并非所有发现都用同一套
修改前负面测试复现；下文明确区分源码证据与实际失败。Task 的“普通 RESET
可解锁 FAULT”不是实际基线行为，是旧行为映射文档的错误描述；源码转移表
原本没有该转移，本次没有添加自动解锁。

## 3. 逐项修复

### P0-1 停止送达与停稳证据

触发：执行器无 active_command_id 时，旧 `request_stop()` 把请求送达用于
停止确认；后端的简单 stop_confirmed 布尔值也缺少可核验的反馈。原测试
`ExecutorCore.StopWithoutInFlightIsConfirmed` 直接认可了这一错误。

修改文件：`mtc_core_cpp/include/.../results.hpp`、运动包的 `motion_adapter.hpp`、
`executor_core.hpp/.cpp`、`mock_motion_adapter.hpp/.cpp` 及对应测试。

- 增加 `StopEvidence`：命令 ID、执行状态、通信健康、队列清空、持续静止区间
  的后端判据、区间起点、末帧关节位置/速度/时间。证据与执行器使用同一时基。
- `evidence_valid()` 核验命令关联、STOPPED/SETTLED、有限且维度正确的反馈、
  速度容差、反馈新鲜度、非未来时间，以及至少 settle_duration_s 的静止区间。
  后端须检查区间内所有反馈，不能仅比较两端或利用 Action accepted/canceled。
- 无内部命令记录、关联不符、断联、运动中、短区间、旧反馈、缺失证据均返回
  stop_confirmed=false 并保守锁定。状态未知不会因后续 Stop 自动解除。
- 人工出口 `clear_unknown_after_manual_confirmation()` 保留，必须有非空人工
  核实说明，且所有执行/停止回调已退出；FSM 自动路径没有调用这个出口。
- Mock 的关联证据仅用于虚拟状态；正常到位后单独推进静止时间，静止区间
  不再借用运动期间的时间。实体后端不能沿用 Mock 的证据生成方式。

新增测试：`ExecutorAudit.DeliveryWithoutEvidenceAndWithoutCommandNeverConfirms`、
`ExecutorConcurrency.InvalidEvidenceCannotConfirmStop`（八种无效证据/异常）、
`MockAudit.VirtualSettlementStartsAfterArrivalAndRejectsInvalidTravel`。
原来的两个 StopWithoutInFlight 测试身份保留，断言改为不确认，避免删除原257身份。

### P0-2 新鲜观测支持连续稳定窗口

触发：旧 `h_verify_placed()` 使用 resolve_cached，历史合格帧被反复消费；
稳定时间来自 step 时钟，与独立采样数量无关。旧稳定性测试缺少冻结帧反例。

修改文件：操纵包 `pick_place_fsm.hpp/.cpp`、测试 `fakes.hpp`、旧 FSM 测试装配，
新增 `test_pick_place_audit.cpp`。

- VERIFY_PLACED 不缓存已完成观测；nullopt 与已接收帧严格区分。
- 以观测时间戳为水位，只有严格更新的帧可以推进窗口；要求匹配 object_id，
  found/unique/not-stale、有效完整 body_pose，观测与位姿时间一致且不过期。
- NaN/Inf、未来/负时间、回退时间、目标丢失、未稳定、无效/过期观测中断窗口。
  采样间隔超过 perception_max_age_s 同样中断；空轮询不会增加稳定时间。
- 时间语义仍是 `placement_stability_sec` 秒，累计值改为最后新帧与首帧的
  观测时间差；默认连续新帧覆盖满 3 秒才成功。时戳 0 是允许的离线时基起点，
  仍受新鲜度检查，不把 0 一概当无效。

新增测试：`PickPlaceAudit.RepeatedFrameForFiveSecondsCannotSucceed`，以及
`PlacementAudit` 的六项测试：满3秒、冻结5秒、无效/过期、断流、未来/回退/非有限
时戳、采样长间隔。观察停止更新时继续调用 step 不会伪造放置成功。

### P0-3 规划到执行的数据流

触发：旧 PlanResult 没有执行目标，make_command 给六关节填默认零；规划 ok
不代表获得可执行输出，运动阶段忽略规划。旧 Fake 只返回 ok，掩盖了缺口。

修改文件：核心 `motion_primitive.hpp/results.hpp`，操纵包的
`pick_place_types.hpp`、`pick_place_fsm.hpp/.cpp`、`motion_adapter_port.hpp/.cpp`，
Fake 与相关测试。

- `MotionPrimitive` 表达意图；PlanTarget 保留完整原语、6D Pose、frame_id、
  stamp 与接触/速度约束；PlanResult.commands 表达显式规划输出。
- MotionCommand 携带原始 intent，并明确选择关节目标或完整 TimedTrajectory。
  acquire/test_lift 与 transport/preseat/contact/retreat 两个规划批次必须提供
  全部相应输出。缺失、重复/错误原语、维度/数值非法、时间超预算等显式拒绝。
- make_command 只复制规划结果并赋予唯一执行 ID，删除默认全零占位构造。
  非零目标、完整位姿与约束传到 MotionAdapterPort，再经执行器校验和确认。
- 多点轨迹整条交给 execute_trajectory，绝不拆成单点调用。当前桥接默认
  显式拒绝轨迹，待第二阶段单 Goal FollowJointTrajectory 实现；没有新增 action。
- 纯逻辑 FakePlanner 明确生成非零、simulation_only 的输出；可执行后端不能
  接收该标记。缺少固定工位校准位姿的真实路径也会拒绝，绝不执行默认姿态。

新增测试：`PickPlaceAudit.MissingPlanOutputNeverExecutesZeroTarget`，
`PlanningAudit.NonZeroPlannerTargetsAndFullPoseReachAdapter`（进入真实代码的 Mock
Adapter 桥接）、`MissingOutputDoesNotReachMockAdapter`、`WholeTimedTrajectoryIsNeverSplit`、
`MalformedPlanCannotExecute`；Fake 标记隔离另由 ValidationAudit 覆盖。

### P0-4 超时与异常锁定

触发：旧执行器只复制 result.status_unknown，超时标志与独立停稳依据没有
共同决定锁定；错误的后端组合可使新 Goal 再次发出。

修改文件：运动包 `executor_core.cpp`、核心 `results.hpp`、对应测试。

终态 SETTLED/STOPPED 必须同时有有效关联停稳证据；超时且缺少停稳依据、
非终态结果、状态未知、通信/后端异常均锁定。result.status_unknown=false
不能压过 timed_out 或证据缺失。仅 REJECTED 且明确未执行、非超时、非未知的
拒绝允许按契约重试。锁定时返回 STOP_UNKNOWN，取消 success/arrived，不抹去
timed_out 或具体错误码。后续执行在 PREPARE 拒绝，不能再次触达后端。

新增测试：`ExecutorAudit.TimeoutWithoutUnknownFlagStillLocks`；并发测试
`DuplicateStopsAndLateResultCannotUnlock` 覆盖超时/取消竞争。原异常、状态未知、
拒绝允许重试测试也实际通过。

### P0-5 多线程执行与取消

触发：旧 InFlightGuard 与核心对 optional/标志的读写无同步；原并发测试仅
同线程嵌套执行，没有覆盖真实线程交错。

修改文件：运动包 `in_flight_guard.hpp/.cpp`、`executor_core.hpp/.cpp`、
`motion_adapter.hpp`、`mock_motion_adapter.hpp/.cpp`、`CMakeLists.txt`，
核心 `clock.hpp`，新增 `test_executor_concurrency.cpp`。

- 短 mutex 保护执行所有权/快照，InFlightGuard 操作原子化；握手记录返回副本，
  不暴露并发变动容器中的指针。ManualClock 使用原子更新。
- 后端执行、停止请求与证据查询均不持执行器状态锁；停止可以在执行等待期间
  处理。执行回调退出前保留 owner，即使已确认停稳也不能开启下一执行。
- 世代、命令关联与 stops_pending 保护取消/结果竞争；错误旧 ID 不能覆盖当前
  owner；迟到结果不清除锁定。重复执行已派发 ID 被拒绝。
- request_stop_for_command 契约要求后端处理取消先于派发的竞态；Mock 保存取消
  ID，之后同 ID 派发在任何虚拟运动前拒绝。可执行后端必须声明关联取消能力。
- Mock 仅短时同步虚拟计算，不睡眠、不等待实际轨迹，也不再执行随模拟时长
  增长的循环。真实阻塞竞争由独立 BlockingAdapter + std::thread/async 验证。
- 两个 FSM 本体由单一事件线程所有；ROS 多线程事件须入队，不能并行调用
  FSM 的 step/Goal/cancel/查询。调用方须保持执行器和端口存活至所有调用退出。

新增并发测试六项：16线程争抢、8个并发Goal与及时Stop、重复Cancel与超时结果、
确认停止后等待结果退出、八种无效证据、旧Goal取消。CV/后端等待有时间界限，
测试 CTest 超时30秒；正常回归未出现死锁、重复执行或自动解除锁定。
另有 `ExecutorAudit.CompletedCommandIdCannotExecuteTwice` 和
`CancelBeforeBackendDispatchIsSticky`。

### P1-1 D-2 看门狗两侧同步

源码复核：返回 RECOVERY 而内部停留在旧状态，旧 C++ 测试明确断言了缺陷。

修改：C++ `pick_place_fsm.cpp` 的无结果看门狗分支保存 resumable、前状态并真正
enter_state；Python 仅对应分支做同样修改。返回状态、内部状态、进入时间与
历史一致。无载荷且允许重发才能恢复；在途运动仍 FAULT/230 禁止重发。
本次 C++ 放置断流按 P0-2 专用契约失败，不尝试携带物体重新抓取。

测试：原 `NonMotionStateTimeoutReportsMotionTimeout` 新断言；
`WatchdogAudit.LoadedContextCannotRecoverAndFaultCannotRestart`；Python 新增
`test_watchdog_non_motion_updates_state_history_and_retry_context`、
`test_watchdog_non_motion_loaded_context_does_not_retry`、
`test_watchdog_motion_timeout_remains_fault_locked`。两侧恢复预算与进入时间均验证。
`FSM_BEHAVIOR_MAPPING.md` 的 D-2 已更新为已修复，不再描述为应保留的基线特征。

### P1-2 数值与可执行配置

源码复核：原 Pose 仅检查姿态/标记，位置与时间缺少有限性约束；轨迹 NaN 时间
可绕过单调检查；无关节限位时非有限目标没有可靠拒绝。

修改：`pose.hpp`、`timed_trajectory.hpp`、`motion_primitive.hpp`、
运动 `validation.hpp/.cpp`，另补 Task/PickPlace 的非有限稳定窗口检查。
拒绝非有限位置/关节值/速度/加速度/时间、不合法四元数、负时间、非正或非有限
安全阈值、反向/无效限位、动力学上限维度不符及过大速度/加速度。
关节 scaling 必须在有限的配置上限内。轨迹第一个相对时间0、Pose/轨迹 header
stamp0 仍可按类型契约使用；命令 timeout_ms=0 则拒绝。

无位置限位仅限声明 simulation_only 的离线后端；可执行后端必须配置完整六
关节名与位置限位、动力学上限、关联取消能力。默认动力学值是离线默认值，
不代表经过 AUBO 实机审核；第二阶段必须替换为经过模型/控制器验证的配置。

测试：两个 CoreAudit、三个 ValidationAudit、
`WatchdogAudit.InvalidStabilityNeverStartsMotion`、
`TaskValidationAudit.NonFiniteStabilityRejected`，以及原限位/轨迹测试。

### P1-3 唯一命令 ID

源码复核：32字节 snprintf 缓冲区会截断长 request_id 后面的序号；重复 request
还会在重新 start 时重置计数，导致再次使用已执行 ID。

修改：`PickPlaceFsm::next_command_id()` 使用动态字符串，格式为
`request#process_invocation:label-sequence`；进程内全局原子调用号区分实例/重复
请求，序号不截断。执行器额外拒绝已派发 ID 重复执行。

测试：`CommandIdAudit.LongAndRepeatedRequestsRemainUniqueAcrossInstances` 使用
2048字符请求 ID、2个实例、各2轮，验证28个 ID 不重复且前缀完整。跨进程重启的
身份仍须第二阶段结合 ROS Goal UUID/启动身份设计，不宣称永久全局唯一。

### P1-4 Task FAULT 契约

源码复核：FAULT 不在原终态集合，也没有普通 RESET 转移。后者是安全锁定，
应保留；前者需要给 Action 外壳明确结束依据，不能混同解锁。

修改：Task `task_types.hpp/.cpp`、`task_fsm.hpp/.cpp`；FAULT 加入执行终态，
新增 action_finished/fault_locked/accepts_tasks 查询。故障拒绝新 Goal，忽略迟到
结果以保留原故障码，没有添加 `(FAULT, RESET)`，停止确认也不能解除锁定。
现有接口缺少安全人工复位协议，本阶段故障保持锁定。执行器人工确认出口
不能被当作 Task FSM 的人工恢复协议，也不能由自动流程调用。

测试：`TaskFaultAudit.ActionFinishesButOrdinaryResetAndNewTasksStayLocked` 验证
执行结束、普通RESET/新Goal/停止确认/迟到旧结果均不解锁；原故障安全测试通过。
第二阶段需要受监督人工授权、载荷/工具确认、关联停稳证据及重新自检协议，
完成后才能重新允许任务。本阶段没有为了方便添加自动复位。

### P1-5 XML manifest

修改唯一文件 `src/mtc_interfaces/package.xml`：test_depend 移到 member_of_group
之前，符合已安装 package format 3 schema 的序列顺序。全部 `.action/.msg/.srv`
内容保持不变。独立提交 `ff2434e`，实际 xmllint 从既有失败变为通过。

## 4. 新增测试与收集依据

两个新可执行文件 `test_executor_concurrency`（6项）、`test_pick_place_audit`
（13项）已通过 ament_add_gtest 注册；其余14项加入原可执行文件。原14个
GTest 可执行文件保留，现在共16个。Python 新增3项在原有已注册pytest文件。

下列 **33个新增GTest身份** 均在最终实际结果XML中具有 status=run、
result=completed，无 error/failure/skipped；不依据文件存在宣称通过。

```text
CommandIdAudit.LongAndRepeatedRequestsRemainUniqueAcrossInstances
CoreAudit.PoseRejectsNonFinitePositionStampAndQuaternion
CoreAudit.TrajectoryRejectsNonFiniteFieldsAndNegativeTimes
ExecutorAudit.CancelBeforeBackendDispatchIsSticky
ExecutorAudit.CompletedCommandIdCannotExecuteTwice
ExecutorAudit.DeliveryWithoutEvidenceAndWithoutCommandNeverConfirms
ExecutorAudit.TimeoutWithoutUnknownFlagStillLocks
ExecutorConcurrency.ConfirmedStopRetainsOwnerUntilResultDrains
ExecutorConcurrency.DuplicateStopsAndLateResultCannotUnlock
ExecutorConcurrency.GoalsAndStopRespondWhileExecutionWaits
ExecutorConcurrency.InvalidEvidenceCannotConfirmStop
ExecutorConcurrency.OldGoalCancelCannotReplaceNewOwner
ExecutorConcurrency.SixteenThreadsHaveOneOwner
MockAudit.VirtualSettlementStartsAfterArrivalAndRejectsInvalidTravel
PickPlaceAudit.MissingPlanOutputNeverExecutesZeroTarget
PickPlaceAudit.RepeatedFrameForFiveSecondsCannotSucceed
PlacementAudit.FrozenTimestampCannotSucceedForFiveSeconds
PlacementAudit.FutureBackwardsAndNonFiniteStampsInterrupt
PlacementAudit.GapBetweenFreshFramesInterruptsContinuity
PlacementAudit.InvalidOrStaleFrameInterruptsWindow
PlacementAudit.NewFramesMustCoverFullThreeSeconds
PlacementAudit.PerceptionStopsUpdatingCannotAccumulateTime
PlanningAudit.MalformedPlanCannotExecute
PlanningAudit.MissingOutputDoesNotReachMockAdapter
PlanningAudit.NonZeroPlannerTargetsAndFullPoseReachAdapter
PlanningAudit.WholeTimedTrajectoryIsNeverSplit
TaskFaultAudit.ActionFinishesButOrdinaryResetAndNewTasksStayLocked
TaskValidationAudit.NonFiniteStabilityRejected
ValidationAudit.ExecutableBackendRequiresLimitsAndRejectsFakePlan
ValidationAudit.NonFiniteTargetsRejectedWithoutLimits
ValidationAudit.TrajectoryRejectsExcessiveDynamicsAndInvalidLimits
WatchdogAudit.InvalidStabilityNeverStartsMotion
WatchdogAudit.LoadedContextCannotRecoverAndFaultCannotRestart
```

新增离线核验工具 `tests/verify_cpp_audit_results.py` 从指定基线的git对象读取
原有 TEST 身份，再核对当前源码身份与实际GTest XML，缺失、重复、失败、跳过
均令指定命令失败。支持 ament 给 classname 加包名前缀的结果格式；实际验证
原257全部执行、新增33全部执行。它是指定验证命令，不是新的colcon测试套件。

原V1顺序测试使用了两个临时history容器的begin/end，存在无效迭代器风险；本次
改为同一副本容器。旧Fake感知装配注入同一时钟，FakePlanner返回显式非零输出，
以适应新契约，没有删除旧测试。

## 5. 实际构建与测试结果

保留原 build/install/log；独立普通输出在 `build/audit_clean`、
`install/audit_clean`、`log/audit_clean`，没有删除或清空任何既有目录。
常规增量构建与回归先通过，再在新目录做完整构建和回归。最终源码自审修正
Mock 静止区间之后，再跑全工作空间构建、全量注册测试和身份核验。

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log/audit_clean build \
  --build-base build/audit_clean --install-base install/audit_clean
colcon --log-base log/audit_clean test \
  --build-base build/audit_clean --install-base install/audit_clean
colcon test-result --test-result-base build/audit_clean --all
python3 tests/verify_cpp_audit_results.py --build-base build/audit_clean
python3 tests/test_mock_end_to_end.py
git diff --check
```

实际输出摘要：

```text
独立首次完整 build: Summary: 12 packages finished [16.2s]
最终增量 build:     Summary: 12 packages finished [6.12s]
最终 colcon test:   Summary: 12 packages finished [2.78s]
test-result:       Summary: 569 tests, 0 errors, 0 failures, 0 skipped
身份核验:          PASS: original 257/257 executed; 33 new cases executed
                   PASS: 290 distinct cases in 16 GTest executables; no failure/error/skip
显式 E2E:          E2E RESULT: PASS 15 / FAIL 0 / SKIP 0
diff --check:      无输出，退出0
```

colcon 的569不是569个独立业务场景：包含290个GTest、258个pytest，另外21个
CTest包装/接口检查条目。不能将包装项再计为新增功能用例。

| 执行范围 | PASS | FAIL | SKIP / NOT RUN |
|---|---:|---:|---|
| mtc_core_cpp | 23 | 0 | 0 |
| mtc_motion_execution_cpp | 69 | 0 | 0 |
| mtc_task_cpp | 81 | 0 | 0 |
| mtc_manipulation_cpp | 117 | 0 | 0 |
| Python: aubo_bridge/manipulation/motion/safety/task/tool | 25/48/20/70/56/39，合计258 | 0 | 0 |
| 显式纯Python Mock E2E | 15 | 0 | 0 |
| package XML xmllint | 1检查 | 0 | 0 |
| 接口包 lint_cmake | 1检查 | 0 | 0 |
| mtc_bringup pytest | 0（没有注册用例） | 0 | 不宣称具有测试覆盖 |
| copyright checker | 0文件检查 | 0 | 不宣称覆盖 |
| ROS C++ Action / MoveIt / ros2_control Mock graph集成 | — | — | NOT RUN，本次第一阶段未实现/启动 |
| 实体机器人 / IO / RPC / RTDE | — | — | NOT RUN，任务明确禁止 |

普通实际场景合计563项PASS、0项FAIL、0项SKIP；XML等静态检查另列。
E2E提示可选 `mtc_motion_planning` 模块不可用，使用脚本内MockPlanner；15场景
实际全部执行，无场景跳过。该脚本验证Python链路，不能当作C++ ROS集成证明，
也不能替代新的C++感知/规划/停止证据负面测试。

证据：最终普通 colcon stdout/stderr 位于
`log/audit_clean/test_2026-10-11_14-26-29/`；GTest XML 位于
`build/audit_clean/<package>/test_results/<package>/*.gtest.xml`；Python位于
`build/audit_clean/<package>/pytest.xml`，XML检查位于接口包 test_results。
日志与生成结果保留于本地忽略目录，不纳入源码提交。

ThreadSanitizer：使用 GCC `-fsanitize=thread -g -O1` 独立构建核心/运动两包，
初次直接运行 test_executor_concurrency 的6项通过，ldd证实链接独立instrumented
库与 libtsan.so.2。最终扩大为两包全部注册测试时，7个可执行进程在GTest产生
结果前退出，报 `FATAL: ThreadSanitizer: unexpected memory mapping`；实际
test-result 输出 **7 errors、7 failures**（7个缺失结果错误与7个CTest失败）。
之后非PIE链接尝试仍不能稳定启动，直接并发测试进程退出139。没有更改ASLR
或系统安全设置，也没有删除失败结果。记录在 `log/audit_tsan/` 和
`build/audit_tsan/*/test_results/`。因此最终完整TSan标记 **NOT RUN（运行时受限，
启动检查FAIL）**，不是业务用例失败，更不是完整PASS；第二阶段前应在支持
TSan的环境重跑。普通多线程用例全部实际通过，但这不构成无数据竞争的形式证明。

UndefinedBehaviorSanitizer：独立全工作空间使用
`-fsanitize=undefined -fno-sanitize-recover=all -g -O1`，共享库/可执行文件也链接
`-fsanitize=undefined`，输出目录 `build/audit_ubsan` / `install/audit_ubsan`，日志
`log/audit_ubsan`。实际 build **12 packages finished [39.2s]**，test **12 packages
finished [2.80s]**，test-result **569 tests, 0 errors, 0 failures, 0 skipped**；
原257与新增33身份再次全部核验通过，未报告未定义行为。UBSan不能替代TSan
数据竞争检查，重复的同一套场景也不再次加到563业务场景总数中。

## 6. 自审与兼容性

- 检查所有新增C++副作用路径，仅Mock/Disabled工厂，没有真实设备SDK、网络
  命令或实体IO调用。Python只修改授权D-2及其测试，接口只改XML，无IDL变更。
- 所有下游C++包在独立目录重新编译并链接；MotionAdapterPort进入真实执行器
  的桥接回归通过，没有发现当前源码跨包API/ABI使用错误。
- C++结构增加字段/虚方法，握手返回值由内部指针改为optional副本；不是对旧
  二进制的ABI兼容保证。后续所有下游需同版本重编译，不能混用旧so/旧头文件。
- 停稳证据核验与后端调用分开，执行器不持状态锁等待后端。停止未确认和异常
  锁定不可由旧结果覆盖；已执行命令ID不可重复派发。
- 稳定窗口不从FSM调用次数推断；完整轨迹不拆分；缺规划没有回零占位路径。
  原有工具联锁仍禁止未落座/未卸载/未确认解锁时继续撤离。

## 7. 第二阶段前置条件及剩余风险

1. 在受支持的环境完成稳定TSan运行，并在ROS多线程Action外壳验证Goal、Cancel、
   超时、节点关闭与迟到回调的生命周期。FSM必须串行事件队列，执行器/端口必须
   保持存活至工作线程与回调退出。本阶段同步桥接不能替代异步ROS执行包装。
2. 真正MoveIt规划端须返回与每个意图关联的完整关节目标/时间轨迹，校验TF、
   6D位姿时基、模型/TCP/载荷/控制器身份、接触速度和固定工位校准；缺参数继续
   拒绝。本阶段没有实现MoveIt C++ Planner，Fake输出不可启用为可执行后端。
3. 多点轨迹须一次FollowJointTrajectory提交，并复用同样的单在途、超时与停稳
   契约。当前MotionAdapterPort整条拒绝；Mock轨迹成功只用于离线表示测试，
   不代表完成实体轨迹执行或具有硬件停稳证明。可选速度/加速度字段没有填入时，
   真实规划/控制器需完成时间参数化与动力学约束验证，不能绕过限值。
4. 可执行后端须实现关联取消的派发前竞态保护和连续反馈静止判据；断流、队列
   未清、关联不明不得确认。ROS感知/控制器与单调执行时钟必须有明确时间契约。
5. Task人工恢复需受监督授权、停稳/负载/工具独立确认和重新自检接口；现有IDL
   不够，本阶段保留FAULT。不能重建FSM对象来替代人工恢复，也不能自动调用
   执行器的人工确认方法。
6. 命令调用号是进程内唯一；跨重启应使用Goal UUID/启动身份。执行器的已派发
   ID与握手、Mock取消ID记录当前不做淘汰；长期运行需保留防重放语义下的日志/
   缓存容量与生命周期策略，不能简单清空后重用旧命令ID。
7. 本次未获授权修复Python侧其他P0，因此Python参考的历史感知缓存、规划占位
   等逻辑不具备本次C++安全增强。Python回归只证明授权改动未破坏原测试，不可
   用于宣称该参考实现符合新的可执行运动契约。

## 8. 提交与相对基线变更清单

| 提交 | 内容 |
|---|---|
| `ff2434e` | 接口XML顺序修复，独立提交 |
| `a729c8b` | 核心类型/数值校验、关联停稳、超时锁定、取消/执行同步、并发测试 |
| `ef0739f` | 显式规划输出、完整轨迹/意图传递、新鲜放置观测、长ID与操纵回归 |
| `0fa796b` | Python D-2同步与3个测试 |
| `13f21a4` | Task故障执行结束与锁定契约、有限稳定时间校验 |
| 报告提交 | 包含本报告、行为映射更新及离线身份核验工具；通过 `git log -1` 查询，避免自引用提交哈希 |

相对 `feature/jazzy-cpp-fsm` 的全部改动按路径列出如下（无删除/重命名）：

```text
M docs/migration/FSM_BEHAVIOR_MAPPING.md
A docs/migration/CODEX_CPP_FSM_AUDIT_FIX_REPORT.md
M src/mtc_core_cpp/include/mtc_core_cpp/clock.hpp
M src/mtc_core_cpp/include/mtc_core_cpp/motion_primitive.hpp
M src/mtc_core_cpp/include/mtc_core_cpp/pose.hpp
M src/mtc_core_cpp/include/mtc_core_cpp/results.hpp
M src/mtc_core_cpp/include/mtc_core_cpp/timed_trajectory.hpp
M src/mtc_core_cpp/test/test_core_types.cpp
M src/mtc_interfaces/package.xml
M src/mtc_manipulation/mtc_manipulation/pick_place_fsm.py
M src/mtc_manipulation/test/test_pick_place_fsm.py
M src/mtc_manipulation_cpp/CMakeLists.txt
M src/mtc_manipulation_cpp/include/mtc_manipulation_cpp/motion_adapter_port.hpp
M src/mtc_manipulation_cpp/include/mtc_manipulation_cpp/pick_place_fsm.hpp
M src/mtc_manipulation_cpp/include/mtc_manipulation_cpp/pick_place_types.hpp
M src/mtc_manipulation_cpp/src/motion_adapter_port.cpp
M src/mtc_manipulation_cpp/src/pick_place_fsm.cpp
M src/mtc_manipulation_cpp/test/fakes.hpp
A src/mtc_manipulation_cpp/test/test_pick_place_audit.cpp
M src/mtc_manipulation_cpp/test/test_pick_place_fsm_v1.cpp
M src/mtc_manipulation_cpp/test/test_pick_place_fsm_v2.cpp
M src/mtc_manipulation_cpp/test/test_pick_place_safety.cpp
M src/mtc_manipulation_cpp/test/test_pick_place_tables.cpp
M src/mtc_motion_execution_cpp/CMakeLists.txt
M src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/executor_core.hpp
M src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/in_flight_guard.hpp
M src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/mock_motion_adapter.hpp
M src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/motion_adapter.hpp
M src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/validation.hpp
M src/mtc_motion_execution_cpp/src/executor_core.cpp
M src/mtc_motion_execution_cpp/src/in_flight_guard.cpp
M src/mtc_motion_execution_cpp/src/mock_motion_adapter.cpp
M src/mtc_motion_execution_cpp/src/validation.cpp
A src/mtc_motion_execution_cpp/test/test_executor_concurrency.cpp
M src/mtc_motion_execution_cpp/test/test_executor_core.cpp
M src/mtc_motion_execution_cpp/test/test_mock_adapter.cpp
M src/mtc_motion_execution_cpp/test/test_validation.cpp
M src/mtc_task_cpp/include/mtc_task_cpp/task_fsm.hpp
M src/mtc_task_cpp/include/mtc_task_cpp/task_types.hpp
M src/mtc_task_cpp/src/task_fsm.cpp
M src/mtc_task_cpp/src/task_types.cpp
M src/mtc_task_cpp/src/task_validation.cpp
M src/mtc_task_cpp/test/test_task_fsm_safety.cpp
M src/mtc_task_cpp/test/test_task_validation.cpp
A tests/verify_cpp_audit_results.py
```

交付工作树状态：clean，报告提交后以 `git status --short` 复核；生成的
build/install/log保持本地忽略状态。本分支不合并main、不强推、不清空旧工程。
