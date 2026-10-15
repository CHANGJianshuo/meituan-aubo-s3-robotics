# Phase 2B：FollowJointTrajectory 执行端验收报告

日期：2026-10-12（Asia/Hong_Kong）。分支：`feature/phase2b-trajectory-execution`。
开发基线：`main@a04ac6831dd3963e6b1ec4b9e27214eb564f9f24`；开始时工作树干净。
实现提交：`b47b5fed8da403c3ae350c07902a9a0b998849e5`。
本报告作为后续独立文档提交；报告提交SHA由`git log -1 --oneline`核对。
最终交付要求工作树干净、不合并main、不push；最后Git状态检查另行确认。

本轮结果只属于离线软件和GenericSystem。未连接控制柜、真实AUBO、RPC/RTDE/SDK，
未触发实体工具IO，未修改驱动工作空间或其他成员Planner分支。

## 1. 结论与验收边界

新增 [mtc_trajectory_execution_cpp](../../src/mtc_trajectory_execution_cpp/README.md)。
独立FjtMotionPort完成整条轨迹转换、异步ROS Action、关联取消、起点核验、反馈
停稳窗口、超时与FAULT锁定。三个验收级别均有本轮实际执行证据：

| 项目 | 状态 | 实际范围 |
|---|---|---|
| Level 1转换/校验 | PASS：30项 | 纯C++逻辑，消息类型转换；无ROS节点或驱动 |
| Level 2真实ROS Action | PASS：36项 | 本地Fake FJT Server、Client、状态Topic及查询服务；4线程Executor |
| Level 3 GenericSystem | PASS：3场景 | 完整三点执行、起点不一致零Goal拒绝、UUID关联Cancel及停稳 |
| 完整工作空间build | PASS：14/14包 | 本轮全量构建；最终新包修改另行增量构建成功 |
| C++回归 | PASS：424项 | 原358项全部保留并重新执行，新增66项，22个GTest可执行文件 |
| Python注册测试 | PASS：269项 | 本轮colcon执行并核对pytest XML，FAIL 0 / SKIP 0 |
| Python显式离线E2E | PASS：15项 | 实际运行原脚本，自带MockPlanner；非C++/MoveIt闭环 |
| UBSan | PASS：424项C++ | 六个C++包重新instrument和执行；最终新包增量后66项再次通过 |
| manifest XML / diff / 导出消费者 | PASS | ament_xmllint、git diff --check；只find新包的消费者编译并以disabled模式运行 |
| TSan | NOT RUN | 本轮未执行；保留历史运行时失败，不声称排除所有竞态 |
| MoveIt Planner接入 | NOT RUN | 未使用或合并`feature/moveit-planner` |
| 完整PickPlace / Task + MoveIt + FJT闭环 | BLOCKED | 尚需规划分支、异步停止桥接与接触能力方案；仅验证原FSM首个运动阶段注入 |
| 真实硬件及工具IO | NOT RUN | 未授权，通路禁用 |

424 + 269 + 15 = 708个独立测试场景；GenericSystem三场景另列。新包colcon汇总
68包含66个GTest及2个CTest包装；UBSan汇总446包含424个GTest及22个包装，不能
当作额外独立业务测试。新包FAIL 0 / SKIP 0，当前14包分别检查无error/failure/skip。
全目录`colcon test-result --all`本轮NOT RUN；历史`audit_tsan*`及其他构建结果未删除，
不把以前的失败或重复结果混入本轮包统计。

## 2. 审计、架构选择和接口阻断

实际审阅TimedTrajectory、MotionCommand、MotionPort、PickPlaceFsm、MotionAdapter、
MotionExecutorCore、Phase 2A外壳，以及 [Phase 2A报告](PHASE2A_ROS2_ACTION_REPORT.md)
和 [驱动部署手册](../setup/AUBO_ROS2_DRIVER_JAZZY.md)。

MotionPort轨迹方法接收MotionCommand，返回optional；MotionAdapter轨迹方法只接收
TimedTrajectory，返回同步MotionResult。当前MotionExecutorCore没有整条轨迹执行
入口，不能直接把两者等同。本轮以独立异步MotionPort实现ROS Client，既有Core/
MotionAdapter/Phase 2A保持原样，不经ExecuteJointMove拆分轨迹，也不新增ROSIDL。

原PickPlace FSM经`accept_plan`/`make_command`保留规划轨迹并调用MotionPort，现有
构造函数可注入本Port。真实注入测试证明首个MOVE_PRE_ALIGN将三点计划提交为一个
FJT Goal并推进到ENGAGE，未伪造完整放置成功。

**停止桥接阻断：** 原`h_cancel_pending`同步调用并缓存一次request_stop，未确认
就FAULT(520)。FJT的Cancel/终态/停稳窗口均是异步事件，直接注入时必须保留这一
保守行为。测试证明直接取消进入FAULT，后台后续停稳不能自动清除FSM故障。
未来宿主须在FSM外关联取消、冻结派发、非阻塞观察停止证据后再推进取消事件，
失败或超时force_fault；不允许阻塞FSM事件线程或修改原安全状态转换。本轮没有
实现该宿主，也没有改Phase 2A的硬编码Mock默认构造路径。

## 3. 新增文件与关键实现

仅新增12个包文件和本报告，共13个文件；没有修改任何既有文件：

| 文件（相对新包） | 关键作用 |
|---|---|
| `CMakeLists.txt`、`package.xml` | ament_cmake/C++17、ROS Action/控制查询依赖、共享库导出及2个注册GTest |
| `include/mtc_trajectory_execution_cpp/trajectory_validator.hpp` | 显式Fake启用、六关节限位和观察窗口配置、转换/按名称采样契约 |
| `include/mtc_trajectory_execution_cpp/fjt_motion_port.hpp` | MotionPort实现、额外command_id关联取消、非阻塞停止查询 |
| `include/mtc_trajectory_execution_cpp/execution_state.hpp` | ID/UUID、调用计数、终态、观察窗口、外部在途Goal和FAULT快照 |
| `src/trajectory_validator.cpp` | 完整点序列与所有字段重排；有限性/维度/限位/时间精度/timeout；spline整段保守边界 |
| `src/fjt_motion_port.cpp` | 异步Goal/Feedback/Result、单owner、缓存去重、runtime GenericSystem门禁、关联Cancel和故障隔离 |
| `src/fake_trajectory_probe.cpp` | 必须显式参数开启的Fake三场景验证，读取实际起点，不修改JointState |
| `test/test_trajectory_validator.cpp` | 30项转换/数值/spline负面测试 |
| `test/test_fjt_action.cpp` | 36项真实ROS测试；独立服务/状态替身、真实并发及原FSM注入 |
| `test/verify_results.py` | 比较基线358身份、已编译注册集合与实际XML，拒绝漏跑、重复、FAIL、SKIP |
| `README.md` | 中文API/安全边界、构建测试、Fake SOP及后续集成阻断 |

默认enable_fake_execution=false。显式开启仍要求LOCALHOST、无静态peers、非虚拟
ROS时钟，以及新鲜的硬件/控制器查询和JointState；plugin必须GenericSystem且Active。
实例与图检查拒绝额外执行owner、同名服务/Server、外部Action Client或直接Topic
命令发布者。Action Status中遗留的外部在途Goal阻止抢占，即使原Client已退出。

库无trajectory sleep、无detach、无global cancel/Stop。外部Executor管理回调，内部
mutex只保护短状态操作；服务和Action均异步，回调里的future.get仅取已就绪响应。
Session对象和UUID隔离旧回调；超时和通信失效仍保留原执行权，只发送一次关联取消。
FAULT没有自动恢复接口；未确认结束时析构也保留进程端点隔离，best-effort取消
不能被当作已停稳或人工恢复。

规划轨迹必须simulation_only=true，完整六关节且起止点齐全；接触/Cartesian请求
拒绝。位置/速度/加速度分别按名称映射；使用Bernstein凸包对JTC linear/cubic/
quintic整段及导数给出保守限位边界，不能仅看路点数值。该检查可能保守拒绝可行
计划，不是碰撞规划，也不改变或二次重定时Planner输出。

成功须匹配Goal终态、控制器SUCCESSFUL、关联Feedback、最终位置和终态后的实际
新JointState静止窗口。窗口用新样本的steady接收跨度和ROS stamp跨度，不能由Timer
空转累计；所有观测需新鲜且连续，速度及位置变化率均满足静止阈值。历史执行结果
只作为记录重读；停止查询重新核对当前证据，失去更新不能复用历史stop_confirmed。

## 4. 实际调试失败与自审修复

- 初次异步实现编译FAIL：Jazzy的节点名称/Client图查询位于NodeGraphInterface，不能直接调用Node；Service回调需显式SharedFuture类型，泛型lambda不满足function_traits。按安装头文件修正后构建PASS。失败日志保留在`log/phase2b/build_2026-10-12_00-17-17/`。
- 自审发现历史停稳查询可以永久复用旧结论，改为重新验证当前状态与观察窗口；`HistoricalSettlementRequiresFreshCurrentEvidence`证明更新停止后不能继续确认。
- 自审发现Timer时间可超过最后新采样，改为用实际新观测跨度；`StationaryWindowCannotGrowWithoutNewSamples`证明停止更新不能假装满足观察窗口。
- 自审补充spline整段边界和外部遗留Goal拒绝，避免路点超限盲区或抢占其他owner；新增插值超限和外部Client离开后的Action Status测试。
- 最终Fake起点负例最初先触发spline速度拒绝，验证程序的PASS条件过宽，不能据此证明起点检查。将所有路点整体平移且要求明确的`trajectory start differs`错误；最新普通/UBSan测试及GenericSystem复跑均通过，零Goal断言保留。未使用初次宽判据输出作为最终起点验收。

## 5. 本轮 GenericSystem 实证

环境实测Ubuntu 24.04.4 LTS、ROS 2 Jazzy、GCC 13/C++17；rclcpp 28.1.22、
control_msgs 5.10.0、controller_manager_msgs 4.48.1、JTC 4.42.1。
只读驱动源码`src/aubo_ros2_driver`为`jazzy@53aab83735236f0c262ecb11e04ef2600241d9d9`。
参考仓库原来已有` m aubo_description`（已归档Fake修复），本轮前后状态相同，
没有覆盖该修复。使用已安装`install_no_symlink`，不重编译或写入参考目录。

每次启动前实际展开S3 Xacro，验证唯一plugin=GenericSystem、六关节、生成模型已
存在；专用domain218空图、LOCALHOST且清除DDS profile/static peers。日志写/tmp，
PYTHONDONTWRITEBYTECODE=1。实际启动命令：

```bash
ros2 launch aubo_ros2_driver aubo_control.launch.py   aubo_type:=aubo_S3 use_fake_hardware:=true   initial_joint_controller:=joint_trajectory_controller launch_rviz:=false
```

运行时硬件查询显示auboHardwareInterface / GenericSystem / active(id=3)、200Hz、
六个position接口claimed、position/velocity状态available。两controller Active；
目标`/joint_trajectory_controller/follow_joint_trajectory`类型为FJT，唯一Server。
JointState实际顺序为foreArm、shoulder、upperArm、wrist1/2/3，不等于控制器顺序；
适配器正确映射。effort为NaN的未使用字段不构成力/负载证据；本通路只验证位置/
速度。GenericSystem并非物理模型或真实编码器。

probe从当时的JointState取得起点，三点时间0/1/2秒、positions/velocities/
accelerations完整，目标输入关节顺序反转后由转换器重排；最后增加0.02rad。
窄软件限位±0.5rad、速度0.2rad/s、加速度0.5rad/s²，不是实机安全标定值。
最终真实输出摘要：

```text
success: goals_sent=1 cancels_sent=0 feedback_samples=40
UUID=4a07c9a9ba8b18c2032a89415c585139
success=1 arrived=1 stop_confirmed=1 timed_out=0 status_unknown=0 fault_locked=0
final_positions=0.045,0.045,0.045,0.045,0.045,0.045

mismatch: goals_sent=0 cancels_sent=0
error=210: trajectory start differs from current named JointState; no jump permitted

cancel initial stop_confirmed=0
cancel: goals_sent=1 cancels_sent=1 feedback_samples=10
UUID=b6d60890deba8b62fb6a3dcefb128e49
success=0 arrived=0 stop_confirmed=1 timed_out=0 status_unknown=0 fault_locked=0
final_positions=0.0499063,0.0499063,0.0499063,0.0499063,0.0499063,0.0499063
```

最后输出来自同一Fake启动的后续复跑；先前正常Goal终点0.02rad、取消后0.025rad，
所以后续成功终点为0.045rad。没有覆盖起点或把状态强制回零。控制器日志同时记录
Received/Accepted Goal、Goal reached success，以及Cancel active Goal；不只凭接口
存在或客户端ACCEPTED作结论。

输出保存在`/tmp/mtc_phase2b_generic_{success,mismatch,cancel}_final.txt`和硬件/
控制器/Action查询文件。最终Launch日志：
`/tmp/mtc_phase2b_generic_final_logs/2026-10-12-00-39-21-553531-asusNUC05-141447/`。
本地/tmp与build/log不提交Git；报告保留关键摘要。

**环境限制：** manager明确报告Enforcing command limits disabled，并跳过部分
`{-2*pi}`/`{2*pi}`限位解析；无FIFO权限。适配器独立强制配置限位和spline边界，
不能声称驱动限位或硬实时通过。Ctrl+C时pal_statistics报告context invalid的
publisher线程错误，但两进程exit 0，controller和GenericSystem deactivate/shutdown，
最终空图确认；该错误原样保留，未修只读驱动。

启动前后842个受核验源码/配置文件集合及SHA256全部一致；没有修改、增加或删除
这些参考文件，驱动Git状态也保持原样。本轮两次Fake启动均已正常结束。

## 6. 可复现验证与日志

完整构建实际执行`colcon --log-base log/phase2b build --parallel-workers 2`，输出
`Summary: 14 packages finished`。随后最终新包改动按packages-up-to增量构建和测试。
全量注册测试实际执行`colcon --log-base log/phase2b_full test --executor sequential`，
输出14包完成；当前各包test-result独立查询通过。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/robot/meituan_challenge_ws
colcon build --packages-up-to mtc_trajectory_execution_cpp
source install/setup.bash
colcon test --packages-select mtc_trajectory_execution_cpp
colcon test-result --test-result-base build/mtc_trajectory_execution_cpp --all --verbose
colcon test --packages-select mtc_core_cpp mtc_task_cpp mtc_manipulation_cpp   mtc_motion_execution_cpp mtc_ros2_action_cpp mtc_trajectory_execution_cpp --executor sequential
python3 src/mtc_trajectory_execution_cpp/test/verify_results.py --build-base build
python3 tests/test_mock_end_to_end.py
```

最新普通新包日志`log/phase2b/build_2026-10-12_00-42-36/`和
`log/phase2b/test_2026-10-12_00-42-44/`；全量注册测试日志
`log/phase2b_full/test_2026-10-12_00-40-43/`；原358项定向回归在
`log/phase2b_regression/test_2026-10-12_00-35-09/`。XML位于各包
`build/<package>/test_results/`，pytest为`build/<package>/pytest.xml`。
E2E输出`/tmp/mtc_phase2b_python_e2e.txt`，实际15/15；负面用例的预期错误日志不是
回归失败。消费者位于`/tmp/mtc_phase2b_consumer`，只find新包后编译并disabled运行。

UBSan实际使用独立目录，先在Jazzy新终端执行：

```bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base log/phase2b_ubsan build   --build-base build/phase2b_ubsan --install-base install/phase2b_ubsan   --packages-up-to mtc_ros2_action_cpp mtc_trajectory_execution_cpp --parallel-workers 1   --cmake-args '-DCMAKE_CXX_FLAGS=-fsanitize=undefined -fno-sanitize-recover=all -fno-omit-frame-pointer'   '-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=undefined' '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=undefined'
source install/phase2b_ubsan/setup.bash
export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
colcon --log-base log/phase2b_ubsan test   --build-base build/phase2b_ubsan --install-base install/phase2b_ubsan   --packages-select mtc_core_cpp mtc_task_cpp mtc_manipulation_cpp   mtc_motion_execution_cpp mtc_ros2_action_cpp mtc_trajectory_execution_cpp --executor sequential
colcon test-result --test-result-base build/phase2b_ubsan --all
python3 src/mtc_trajectory_execution_cpp/test/verify_results.py --build-base build/phase2b_ubsan
```

全套7包build、6个C++包test实际通过；最后起点案例改动后增量build和新包66项再次
通过，身份核验424项完整。最新UBSan日志`log/phase2b_ubsan/build_2026-10-12_00-44-29/`
及`test_2026-10-12_00-44-42/`。ROS/DDS系统库未重新instrument；TSan未运行，不能
以UBSan代替数据竞争检测。未修改ASLR或系统安全设置。

## 7. 后续阻断与准入建议

独立执行端通过三层软件/Fake验收，可供负责人开展后续受控Mock联合集成准备；
不是完整比赛抓放或实机准入许可。

1. 等Planner负责人合并并回归`feature/moveit-planner`，验证真实规划输出与本执行端的时间、顺序、限位和spline边界兼容；本轮没有读取/修改该分支工作文件或自动合并。
2. 实现显式Port注入宿主和非阻塞取消证据桥接，保持既有FSM FAULT/负载锁定；Phase 2A当前默认Mock路径不替换。
3. 接触/Cartesian能力尚不支持，不能让SEAT等requires_contact工序进入本通路，也不能擅自清除约束位；完整抓放需要独立的约束与验证方案。
4. 实机停止/队列/编码器证据、设备身份认证、碰撞检查、TCP/负载、工具IO、人工恢复接口和安全急停均未验收；本包只支持GenericSystem。
5. 端点隔离和单owner按受信任本机ROS图设计；DDS发现存在传播延迟，不能抵抗恶意伪造或替代实体安全系统。宿主正常退出必须先收齐终态与停止证据；强制退出不保证Cancel送达。
6. TSan仍NOT RUN；需在支持的环境另做动态竞态验收。保守spline边界可能拒绝实际可行计划，需用负责人合入的规划轨迹补充兼容回归，不放宽安全检查来赶进度。

未执行的完整FSM/MoveIt和实机项目保持BLOCKED/NOT RUN。没有发现需放宽Core或
ROSIDL才能完成本次独立执行端的要求；停止桥接限制已用真实测试固定并明确交接。

## 8. 新增测试身份（实际XML执行确认）

### test_trajectory_validator（30项，全部PASS）

- `TrajectoryValidator.CompleteTrajectoryPreserved`
- `TrajectoryValidator.JointOrderMapsAllFields`
- `TrajectoryValidator.EmptyTrajectoryRejected`
- `TrajectoryValidator.SinglePointRejected`
- `TrajectoryValidator.DuplicateJointRejected`
- `TrajectoryValidator.MissingJointRejected`
- `TrajectoryValidator.NonIncreasingTimeRejected`
- `TrajectoryValidator.NanPositionRejected`
- `TrajectoryValidator.InfVelocityRejected`
- `TrajectoryValidator.InfAccelerationRejected`
- `TrajectoryValidator.DimensionMismatchRejected`
- `TrajectoryValidator.FirstPointNotZeroRejected`
- `TrajectoryValidator.PositionLimitRejected`
- `TrajectoryValidator.VelocityLimitRejected`
- `TrajectoryValidator.AccelerationLimitRejected`
- `TrajectoryValidator.TimeoutZeroRejected`
- `TrajectoryValidator.InsufficientTimeoutRejected`
- `TrajectoryValidator.RealCommandRejected`
- `TrajectoryValidator.ContactRequestRejected`
- `TrajectoryValidator.NanTimeRejected`
- `TrajectoryValidator.NegativeStampRejected`
- `TrajectoryValidator.NanStampRejected`
- `TrajectoryValidator.NanosecondTimeCollisionRejected`
- `TrajectoryValidator.MixedRepresentationsRejected`
- `TrajectoryValidator.MissingLimitsRejectConfiguration`
- `TrajectoryValidator.ZeroSettleRejected`
- `TrajectoryValidator.JointSampleRequiresVelocityAndIdentity`
- `TrajectoryValidator.SplinePositionOvershootRejected`
- `TrajectoryValidator.SplineVelocityOvershootRejected`
- `TrajectoryValidator.SplineAccelerationOvershootRejected`

### test_fjt_action（36项，全部PASS）

- `FjtAction.CompleteTrajectoryOneGoalAndFeedback`
- `FjtAction.RepeatedTicksNeverResend`
- `FjtAction.ConcurrentGoalsSingleOwner`
- `FjtAction.StartMismatchDoesNotSend`
- `FjtAction.CorrectCancelUsesExactUuid`
- `FjtAction.CancelBeforeGoalResponsePreserved`
- `FjtAction.DuplicateCancelOnlyOneBackendCall`
- `FjtAction.StaleCancelDoesNotAffectNewOwner`
- `FjtAction.CancelWithoutOwnerNeverGlobalStops`
- `FjtAction.AckOnlyCannotConfirmStopAndTimeoutLocks`
- `FjtAction.ExecutionTimeoutLocksAndNeverResends`
- `FjtAction.ControllerAbortedRemainsFaultLocked`
- `FjtAction.StaleJointFeedbackLocks`
- `FjtAction.LostJointUpdatesCannotSettle`
- `FjtAction.InsufficientStationaryIntervalDoesNotSucceed`
- `FjtAction.WrongFinalPositionCannotSucceed`
- `FjtAction.NoAssociatedFeedbackCannotSucceed`
- `FjtAction.HardwareProofLostLocksAndCancelsOwnedGoal`
- `FjtAction.WrongPluginRejectsBeforeSend`
- `FjtAction.InactiveControllerRejectsBeforeSend`
- `FjtAction.MissingVelocityEvidenceRejectsBeforeSend`
- `FjtAction.ReplayedJointStampLocks`
- `FjtAction.GoalRejectionMeansNeverExecuted`
- `FjtAction.ChangedCommandIdContentCannotResend`
- `FjtAction.SecondPortCannotOwnEndpoint`
- `FjtAction.DefaultDisabledNeverSends`
- `FjtAction.DestroyDuringExecutionOnlyCancelsOwnedGoal`
- `FjtAction.HistoricalSettlementRequiresFreshCurrentEvidence`
- `FjtAction.ConcurrentGoalReplacementAndLateCancel`
- `FjtAction.OtherTrajectoryPublisherRejectsDispatch`
- `FjtAction.ForeignActionClientRejectsDispatch`
- `FjtAction.SingleTargetEntryExplicitlyRejected`
- `FjtAction.StationaryWindowCannotGrowWithoutNewSamples`
- `FjtAction.ForeignInFlightGoalCannotBePreemptedAfterClientLeaves`
- `FjtAction.ExistingPickPlaceFsmDelegatesWholeTrajectoryThroughPort`
- `FjtAction.DirectFsmCancelKeepsOriginalFaultWhenStopIsStillPending`
