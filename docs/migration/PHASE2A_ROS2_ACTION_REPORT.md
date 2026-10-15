# Phase 2A：C++ ROS 2 Action 离线 Mock 交付报告

日期：2026-10-11（Asia/Hong_Kong）。开发基线为 `main@1d940f7`，工作分支为
`feature/phase2a-cpp-actions`。实现提交：
`e1b269062a35062043dc5d9bb80d19678e8dc0d5`。
测试DDS隔离提交：`412270c860f2228000a105fb617f37bae938ff0c`，显式使用
Fast DDS / LOCALHOST并清除继承的Profile与静态peer设置；普通与UBSan测试再次通过。

新增 [mtc_ros2_action_cpp](../../src/mtc_ros2_action_cpp/README.md)，交付 PickPlace、
ExecuteTask 和 ExecuteJointMove 三个 C++ Action Server。结果只适用于离线
Mock 软件；没有启动真实驱动、连接控制柜、调用 RPC/RTDE 或实体工具 IO。
没有合并 main、推送远端或修改官方驱动工作空间。

## 1. 先行审计和接口决定

审阅四包的 FSM/API、Port、Clock、证据类型、执行器取消实现，以及
`mtc_interfaces/action/{PickPlace,ExecuteTask,ExecuteJointMove}.action`。
同时复核第一阶段最终安全验收与第二阶段计划；旧计划的257项基线已经被
最终报告的318项替代，本轮不按旧计划改写或重复实现 FSM。

| 对象 | 核对结论与实现方式 |
|---|---|
| TaskFsm | `submit_goal` / `advance_validation` / `report_target` / `report_item_result` / deadline API可复用；动作意图由外壳分发，FAULT不RESET |
| PickPlaceFsm | `start` / `step` / `request_cancel` / `feedback` / `result`复用；新鲜稳定帧和工具联锁继续由原核心验证 |
| MotionExecutorCore | 同步单点执行放到worker；只按原command_id调用`cancel_goal`，错ID拒绝与generation pin保持原实现 |
| PlannerPort | 返回完整commands；本轮显式Mock关节计划带非零目标和`simulation_only=true`，不是MoveIt逆运动学或碰撞规划 |
| MotionPort | `nullopt`表示未完成，可非阻塞轮询；Stop没有pending类型，因此外壳先异步取得Stop结果，再交给原FSM处理 |
| TaskRequest | 是`mtc_task_cpp`的C++结构体，**不是ROS msg**；从ExecuteTask Goal逐字段转换，不新增消息 |
| PickPlace Result | 无`stop_confirmed`字段。只对本包加强契约的Server信任`CANCELED/OK`；第三方证据独立核验仍存在接口缺口 |
| ExecuteJointMove | 六个名称/目标按索引保留；仅单点。不承载轨迹，不拆分多点轨迹 |

`git diff 1d940f7 --` 四个核心包、`mtc_interfaces`及旧Python逻辑包为空。
FSM状态转移、原有测试和`.action/.msg/.srv`字段均未修改。

## 2. 实现、线程与生命周期

新包使用ament_cmake、C++17、rclcpp、rclcpp_action、action_msgs和现有接口/核心库；
导出`mtc_ros2_action_cpp::mtc_ros2_action_cpp`及公共头文件。ROS依赖未进入核心包。

* `action_slot.hpp`：一个MutuallyExclusive CallbackGroup管理Goal/Cancel/Timer；
  返回ACCEPT之前预留UUID owner，拒绝非法参数、并发Goal、重复业务ID和锁定请求。
  ROS回调不调用FSM，仅记录pending和cancel；只有Timer推进、查询FSM。
* `pick_place_server.cpp`：注入显式Mock感知/规划/工具/异步运动Port；周期发布全部
  Feedback字段，Result取自FSM。仅`CANCELLED && stop_confirmed`返回ROS CANCELED。
  FAULT、挂载或未知负载保留admission lock，Tool实例跨Goal保留；不自动构造
  空载新FSM来清除负载。原命令的清理Stop在Action结束后继续异步收尾。
* `task_server.cpp`：通过真正的ROS PickPlace Client提交`CALL_PICK_PLACE`；按
  原request_id加唯一attempt编号关联子Goal，并保存其实际UUID/handle。子回调只
  写原inbox，迟到回调不能推进新任务。`CANCEL_CHILD`核对原ID后取消该handle。
  Cancel ACK没有停止确认副作用；完成计数/工位由原FSM在验证子Result后记录。
* `motion_server.cpp`：复用MotionExecutorCore。保留Goal六关节数据，发布后端
  Feedback，返回执行器Result；Cancel/ROS交付期限都提交关联Stop。超时或未知
  且未停稳继续锁定，原始220/230/520错误语义保留。
* `mock_ports.cpp`：execute与stop各最多一个owned worker，future只做零等待
  轮询，`get()`只用于ready结果；没有detached thread或trajectory sleep。
  Adapter/Executor共享ManualClock，Backend始终MockMotionAdapter。默认100ms
  只延迟结果交付，虚拟运动本身瞬时推进；Task/ROS期限另用SteadyClock。
* 每个新感知样本随虚拟tick更新时间戳；几何挂载、落座、卸载和释放为显式
  模拟证据。MockTool维护holding/unknown，释放后状态为DETACHED；新包Result
  没有硬编码成功来跳过FSM，反例测试使用故意错误的子Result验证防护。
* timed trajectory / Cartesian capability无法由MockOptions开启；构造时拒绝。
  整条trajectory调用明确拒绝，实际后端单点调用数为0。

Timer/ROS回调不等完整运动；worker持有执行器/Adapter的shared ownership，不
持有FSM或Node裸指针。Task子回调捕获inbox而不是Node裸指针。正常销毁先退出
Executor并移除Node，再销毁实例、回收future。停止软件不等于安全急停。

## 3. 调试中发现的缺陷及修复

| 实际失败/风险 | 原因与处理 |
|---|---|
| 首次CMake configure FAIL | ament plain链接签名与PUBLIC签名冲突；统一PUBLIC，不修改核心导出 |
| 首轮PickPlace正常取消用例FAIL | Feedback显示MOVE_PRE_ALIGN时，首条命令可能尚未派发；没有证据返回520正确。测试等待实际派发，不降低停止条件 |
| 首轮旧Goal测试抛异常 | Client已忘记完成的Goal，未真正发送旧取消。改用Action CancelGoal服务发送原UUID，断言实际backend stop_calls=0 |
| 首轮Motion合法Goal被拒绝 | ROS float32的0.2转double略大于核心0.2上限。只规范化上限的精确float表示，nextafter更大值仍拒绝；不放宽核心阈值 |
| ACK-only测试替身提前结束 | 丢弃ServerGoalHandle会自动abort；保留handle，仅ACK、不提供结果，再验证Task CANCEL_PENDING超时520 |
| 测试替身编译FAIL | Jazzy ServerGoalHandle没有ClientGoalHandle的SharedPtr别名；改用显式std::shared_ptr |
| FAULT后Stop worker可能只poll一次 | 改为Action结束后继续非阻塞poll原命令清理，始终保留锁定 |
| Mock负载可被新FSM清掉 | Tool实例跨Goal保留；holding/unknown阻止新Goal。新增搬运中取消及挂载未验证的回归 |

失败日志保留在本次`log/build_2026-10-11_*`与`log/test_2026-10-11_*`，没有删除
旧构建或测试结果。编译失败后执行的旧二进制结果未用于最终PASS判定；最终
流程使用`set -e`先确认构建成功，再跑最新测试和身份核验。

## 4. 实际执行结果

| 检查 | 结果 | 证据范围 |
|---|---|---|
| 完整工作空间build | PASS：13/13包 | `colcon build --parallel-workers 2`实际完成；之后最终新包和依赖增量6/6成功 |
| 新Action包 | PASS：40项，FAIL 0 / SKIP 0 | PickPlace 12、Task 12、Motion 8、契约单元测试8；4个注册GTest可执行文件 |
| 原有C++ FSM | PASS：318项，身份全部保留 | 基线`1d940f7`与实际XML比较；不是仅统计文件存在 |
| C++合计 | PASS：358项，20个可执行文件 | core23 / task87 / manipulation120 / motion88 / action40 |
| Python注册测试 | PASS：269项，FAIL 0 / SKIP 0 | bridge25 / manipulation52 / motion20 / safety70 / task63 / tool39；bringup报告0项 |
| 显式离线E2E | PASS：15，FAIL 0 / SKIP 0 | 实际运行`python3 tests/test_mock_end_to_end.py`；纯Python Mock |
| UBSan | PASS：358项C++，FAIL 0 / SKIP 0 | 四个核心与新包全部instrumented；新包最终代码再次回归，身份核验通过 |
| XML / diff | PASS | 既有接口xmllint实际PASS；新manifest`ament_xmllint`通过；`git diff --check`通过 |
| CMake消费者 | PASS | `/tmp/mtc_phase2a_consumer`仅find_package新包、链接导出目标，成功编译运行 |
| TSan | NOT RUN（本轮未运行） | 历史运行时`unexpected memory mapping`记录仍保留；本轮并发测试PASS不等于动态竞态检测完成 |
| MoveIt / FollowJointTrajectory / ros2_control接入 | NOT RUN / 未实现 | 本轮不增加轨迹控制通路 |
| 实机 / 工具IO | NOT RUN | 禁止、未授权 |

E2E的可选`mtc_motion_planning`模块未就绪，脚本使用自身MockPlanner；依赖栏的
SKIPPED不是业务场景SKIP。本轮15个场景确实执行，不能把它解释为MoveIt成功。
测试进程只运行软件Mock，以本机隔离domain启动Client/Server；无驱动Launch。

关键输出：

```text
Summary: 13 packages finished
PASS: 原有318项身份全部保留并执行；本轮新增40项
PASS: 358项独立GTest / 20个可执行文件，FAIL 0 / SKIP 0
E2E RESULT: PASS 15 / FAIL 0 / SKIP 0

# UBSan的CTest包装条目与GTest合计，不是378个独立场景：
Summary: 378 tests, 0 errors, 0 failures, 0 skipped
```

### 全目录test-result的历史失败

按要求真实执行`colcon test-result --all --verbose`，**返回1 / FAIL**。最后总览
包含历史build子目录和本轮两套构建，输出`3412 tests, 14 errors, 14 failures,
0 skipped`；失败来自保留的`audit_tsan*`结果（`FATAL: ThreadSanitizer:
unexpected memory mapping`），不是本轮Action用例。没有删除目录来制造总览PASS。

逐个`build/mtc_*/`调用`test-result --test-result-base ... --all`，当前13包均无
failure/error/skip；本轮独立GTest身份核验另外覆盖358项。当前普通构建汇总包含
358 GTest + 269 pytest + CTest/lint包装，不能用全目录3412作为业务用例数量。
本轮独立业务场景为358 + 269 + 15 = 642。

## 5. 可复现命令和日志

普通构建/测试按用户要求执行，详细运行方法见包README：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/robot/meituan_challenge_ws
colcon build --packages-up-to mtc_ros2_action_cpp
source install/setup.bash
colcon test --packages-select mtc_ros2_action_cpp
colcon test-result --all --verbose
colcon test-result --test-result-base build/mtc_ros2_action_cpp --all --verbose
python3 src/mtc_ros2_action_cpp/test/verify_results.py --build-base build
```

身份核验需先执行五个C++包的测试。它固定保留原318项，并从当前测试源码与XML
核对新增身份、完成状态、重复、SKIP和20个可执行文件。历史核验脚本固定16个
可执行文件，未篡改，使用本包新脚本覆盖扩展后的集合。

UBSan实际命令使用独立目录，不清空历史：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/robot/meituan_challenge_ws
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base log/phase2a_ubsan build \
  --build-base build/phase2a_ubsan --install-base install/phase2a_ubsan \
  --packages-up-to mtc_ros2_action_cpp --parallel-workers 2 --cmake-args \
  '-DCMAKE_CXX_FLAGS=-fsanitize=undefined -fno-sanitize-recover=all -fno-omit-frame-pointer' \
  '-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=undefined' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=undefined'
source install/phase2a_ubsan/setup.bash
export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
colcon --log-base log/phase2a_ubsan test \
  --build-base build/phase2a_ubsan --install-base install/phase2a_ubsan \
  --packages-select mtc_core_cpp mtc_task_cpp mtc_manipulation_cpp \
  mtc_motion_execution_cpp mtc_ros2_action_cpp
colcon test-result --test-result-base build/phase2a_ubsan --all
python3 src/mtc_ros2_action_cpp/test/verify_results.py --build-base build/phase2a_ubsan
```

最终普通构建日志：`log/build_2026-10-11_22-59-11/`；Action测试日志：
`log/test_2026-10-11_22-59-17/`。全工作空间测试及之前修复过程的日志也保留。
包实际GTest输出在`build/mtc_ros2_action_cpp/ament_cmake_gtest/`，XML在各包
`test_results`；Python为`build/mtc_*/pytest.xml`。UBSan日志在
`log/phase2a_ubsan/`。ROS测试进程日志写`/tmp/mtc_phase2a_test_logs`，不写驱动目录。

## 6. 已覆盖的安全场景与后续边界

合法基础/序列任务来自显式Mock端口证据；非法颜色/工位/数值、并发和重复请求
拒绝；完整Feedback、Result映射、子失败原码传播、缺少子Server、Goal/状态期限、
FAULT保留、真实多线程Goal争夺、旧UUID零Stop、ACK-only不得安全取消、缺放置
证据不得计数、搬运中取消后仍保持负载锁定均有实际测试。

进入Phase 2B的建议：**Phase 2A可信离线Mock Action链路可作为集成基线**。
接入MoveIt规划和整条轨迹前仍须制定异步运动/停稳证据、时基转换、唯一执行权、
负载恢复与接口方案；本轮不预先扩展ROSIDL。

* PickPlace Result不能独立携带停稳证据；CANCELED/OK契约只对本包可信Server
  有效，第三方/真实后端接入是阻断边界。不能收到任意Server的CANCELED就宣称
  机器人停稳，也不能因子SUCCEEDED而确认取消。
* PickPlace使用本地Mock执行器；独立Motion Action是另一套Mock状态，不是
  PickPlace已接入Motion ROS Client，更不是两个执行器可同时控制同一硬件。
* FAULT或挂载/未知负载没有人工复位接口，保持锁定。流程没有自动人工确认、
  自动clear_unknown或自动释放；重启Mock进程不能当作真实机器人恢复证明。
* 感知、几何工具证据及非零关节计划均为模拟。无真实相机、工具IO、TCP/负载
  标定、规划/碰撞检查、Gazebo或实体机构验收。
* 4线程Executor和竞争测试通过；TSan本轮NOT RUN，不能宣称已排除所有数据竞争。
  ROS/DDS系统库未在本轮重新instrument，UBSan结论限定于本轮构建的代码。

10月12日实机联调与10月15日视频安排不扩大本次授权；本次停止在软件Mock交付。
该分支不自动合并或push，交付时工作树通过最终Git状态检查。报告作为独立文档
提交，代码提交ID如开头所列；最后文档提交可由`git log -1 --oneline`核对。

## 7. 新增测试身份

以下列表由本次已编译的测试源码提取；真实执行与状态由上述XML核验确认。

### test_action_contracts.cpp（8项）

- `ActionContracts.TaskConversionPreservesColorOrderIdentityAndDeadline`
- `ActionContracts.RosFloatBoundaryDoesNotRelaxCoreUpperLimit`
- `ActionContracts.MockPlannerPreservesSixDPoseFrameStampAndContactConstraints`
- `ActionContracts.MockLoadEvidenceRemainsHoldingUntilVerifiedRelease`
- `ActionContracts.MockFailedReleaseNeverReportsNoLoad`
- `ActionContracts.TestLiftRequiresPriorSimulatedAttachmentEvidence`
- `ActionContracts.TrajectoryBridgeRefusesAndNeverSplitsIntoJointMoves`
- `ActionContracts.UnsupportedTrajectoryAndCartesianCapabilitiesCannotBeEnabled`

### test_motion_action.cpp（8项）

- `MotionAction.LegalNonzeroGoalAndFeedbackUseExecutorResult`
- `MotionAction.IllegalGoalRejectedWithoutBackendCall`
- `MotionAction.ConcurrentGoalIsRejectedAndFirstCompletes`
- `MotionAction.MatchingCancelStopsExactlyOnce`
- `MotionAction.UnconfirmedCancelAborts520AndLocks`
- `MotionAction.StaleCancelCannotReachBackendOrInterruptNewOwner`
- `MotionAction.ExecutorTimeoutKeepsOriginal220AndRejectsNewMotion`
- `MotionAction.RosDeadlineIndependentlyRequestsAssociatedStop`

### test_pick_place_action.cpp（12项）

- `PickAction.LegalGoalSucceedsOnlyAfterPlacementFeedback`
- `PickAction.InvalidParametersRejectedBeforeExecution`
- `PickAction.ConcurrentSecondGoalRejected`
- `PickAction.MatchingCancelRequiresIndependentStationaryEvidence`
- `PickAction.CancelWithoutStopEvidenceAbortsAndKeepsFaultLocked`
- `PickAction.StaleGoalCancelCannotInterruptNewGoal`
- `PickAction.MotionStatusUnknownNeverResendsOrAutoClearsFault`
- `PickAction.MissingPlanNeverSucceeds`
- `PickAction.ConcurrentRosRequestsHaveExactlyOneOwner`
- `PickAction.InFlightWatchdogFaultStillDrainsAssociatedStopWorker`
- `PickAction.CancelWhileCarryingStopsButKeepsLoadAdmissionLocked`
- `PickAction.UnverifiedAttachmentLoadCannotBeClearedByNewGoal`

### test_task_action.cpp（12项）

- `TaskAction.BasicTaskUsesRealChildActionAndRecordsT0`
- `TaskAction.SequenceTaskRecordsAllSlotsInOrder`
- `TaskAction.InvalidColorsAndConcurrentTaskRejected`
- `TaskAction.CancelAckWaitsForChildFinalStopResult`
- `TaskAction.UnconfirmedChildCancelPreserves520AndLocksParent`
- `TaskAction.ChildStatusUnknownPropagatesOriginalError`
- `TaskAction.ChildPlanningFailurePreservesErrorWithoutFakeSuccess`
- `TaskAction.TaskDeadlineAbortsAndCancelsOriginalChild`
- `TaskAction.MissingChildNeverFabricatesTaskSuccess`
- `TaskAction.ChildCancelAckWithoutTerminalEvidenceCannotCancelParent`
- `TaskAction.StaleParentCancelCannotReachNewChild`
- `TaskAction.ChildSuccessWithoutPlacementEvidenceIsNotRecorded`
