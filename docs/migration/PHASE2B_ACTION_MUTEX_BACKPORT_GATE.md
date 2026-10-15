# Phase 2B Action Mutex Fix Backport & PR #7 Final Merge Gate

日期：2026-10-12。仅软件、真实本机 Fake ROS Action 和 GenericSystem 状态预检。

## 结论与 Git

**PASS：Phase 2B 独立模块的 Action mutex 修复及合并验收。** 普通/UBSan各486项通过，
原478项全部保留，新增8项；40轮×9项重复并发/身份测试全部通过，不能计作新增用例。
**BLOCKED：TSan运行环境、原生GenericSystem严格Status冷启动、标准MoveIt共存。**
真实硬件 **NOT RUN**。本报告不宣称Planner/FJT/GenericSystem联合执行成功。

- 当前分支：feature/phase2b-trajectory-execution。
- 原基线/远端PR #7：681bb64bfb8cd482d1e78950cef4cfe395b21bff。
- 实现提交HEAD：62580baec41390a054690bd156d248d6e35bca99；本报告随后单独提交，最终文档HEAD通过git log查询。
- 来源分支/远端均为 feature/phase2c-single-execution-owner，
  4a67503492e6bf0f678c6bbca04413e6c670ae30；队列实现来自其09b0333代码提交。
- 最新origin/main：9fea32eee901a3a0e5c3d2ad3a0f3c12aa51093b，包含PR #6。
- 初始工作树干净；目标分支ff-only拉取后仍681bb64。没有重写历史、整体cherry-pick、
  rebase、修改main/Phase2C/成员分支、push或正式merge。
- git merge-tree --write-tree origin/main 62580baec41390a054690bd156d248d6e35bca99 返回0，无冲突，候选tree
  7f08a71e0375641e22ae31b3c11f51a79aba240d。仅静态合并检查；本轮测试对象为Phase2B
  加本次修复，不把静态merge-tree声称为最新main候选的实际编译。
- 当前远端PR仍681bb64，尚未包含本轮修复；必须手工Push后才能以本轮结果复审。

## Static Analysis：原有锁序与修复

安装版Jazzy rclcpp_action/client.hpp：async_send_goal完成发送后维护goal_handles_mutex_；
async_cancel_goal同样取该锁；handle_feedback_message在该锁内调用用户反馈回调。
Port原execute_trajectory/issue_cancel在Impl::mutex内调用上述API，用户回调反过来
获取Impl::mutex，因此存在 Port→Action 与 Action→Port 的反向锁序。
安装源码：[Client实现](/opt/ros/jazzy/include/rclcpp_action/rclcpp_action/client.hpp)。

历史Phase2C有一次ConcurrentGoalsSingleOwner运行120秒超时，但未取得线程堆栈。
**本轮没有复现或证明真实死锁**；上述结论为安装源码静态风险分析。40轮通过也不是
不存在所有死锁或数据竞争的数学证明。

```mermaid
flowchart LR
    A[Port mutex内：校验 / 固定Session / 排队] --> B[释放Port mutex]
    B --> C[10ms tick / 有界队列drain]
    C --> D[锁外async_send_goal或scoped async_cancel_goal]
    D --> E[回调锁内核对原Session和UUID]
```

- 一个活动Session最多排队一个send及一个scoped cancel；创建Session只排队一次，
  同ID轮询不排队。cancel_sent在锁内置位，重复Cancel不能再次排队。
- 10ms受管理的事件tick先处理状态，再锁内swap出站队列，释放mutex后调用Action API。
  析构也在设置closed/取消timer后锁外drain，不依赖未管理的detached thread。
- send前锁内重新检查原Session、closed/FAULT、GenericSystem/controller/Action/JS/
  graph及严格Status证据；重新检查按名称起点、静止速度和ROS调度启动时间。
  然后锁外调用API。ROS图存在发现/查询到派发的窗口，不提供跨进程原子互斥或认证。
- 所有异常只在重新锁定后、核对active==原Session时更新；FAULT不自动解除。
  若独立Own终态先于队列Cancel到达，UnknownGoalHandle不提供ACK，也不取代停稳证据。
- Goal Response在closed/旧Session分支先解锁，再仅取消该原请求返回的Handle。
  weak Impl保护状态生命周期；Session保存完整不可变命令；Handle只在原响应赋值一次，
  后续操作不会重定向到当前的新Goal。没有global cancel。

## Fake ROS Action Tests：实际新增发现与修复

第一次新增测试：4 PASS/4 FAIL。其中DestroyDuringPendingAction实际观测到：
请求到达Server，Port在Handle前析构，随后释放Server响应，却三秒内Cancel=0，UUID
为空。后续三项环境预检也失败，并出现重复client Node名称提示。完整日志保留：
/tmp/phase2b_mutex_new_initial.log 和对应XML。这是实际生命周期复现，**不是死锁证明**。

原因：专用callback group原只由Impl持有。pending请求虽保留Client/Session，但Port
销毁后callback group不再可执行，迟到响应无法处理。修复：原goal_response_callback
显式持有response_group直至响应完成；不强持有已销毁Impl，收到Own Handle后只取消
该Handle。只有Node/Executor仍在运行时才能处理迟到响应；通信中断/宿主关闭不代表
取消已经完成，quarantine保留，不能声称安全结束。

第二次：7 PASS/1 FAIL，剩余新用例把timeout设为150ms，低于现有120ms轨迹加停稳/启动
余量要求，被正确拒绝，不能制造接受。改为合法400ms并继续人为延迟Response，原能力
检查未放宽。最终8项均PASS，后续完整回归及40轮重复均通过。
析构路径偶有Fake Server提示cancel response无法送达，因为Client已结束；此路径不
报告ACK或Stop Confirmed，测试只验证原UUID请求及保留quarantine，没有伪造停稳。

## 新增测试与原有场景

新增8项，全部在test_fjt_action.cpp，真实rclcpp_action Client/Server与4线程
MultiThreadedExecutor，既有严格Status测试relay保留；无Port私有状态注入。

| 新增GTest（FjtAction.） | 场景 |
|---|---|
| ConcurrentPollingAndCancel | 结果轮询与关联取消竞争；Goal1/Cancel1、独立停稳 |
| RepeatedScopedCancelUnderContention | 4线程共400次取消；Backend Cancel1、正确UUID |
| ConcurrentSnapshotPolling | Snapshot与Feedback/Result并发；完整三点正常单Goal |
| ConcurrentResponseAndCancelPreservesUuid | 响应与取消乱序；仅原UUID取消一次 |
| DestroyDuringPendingAction | Handle前析构、迟到Handle；仅Own取消，不能重建绕过quarantine |
| LateGoalResponseAfterFaultRetainsOriginalSession | 超时后迟到响应/结果；保留Session和FAULT、拒绝下一Goal |
| RepeatedSessionPollingAndScopedCancel | 5个不同Session依次轮询/取消，逐个准确UUID和单次派发 |
| ConcurrentCompletedIdentityRejection | 完成记录并发读取及内容变更拒绝，Goal仍1/Cancel0 |

旧测试文件前缀逐字保持，原61项FJT Action主体没有修改；校验30项仍全部运行。
原ConcurrentGoalsSingleOwner、CancelBeforeGoalResponseUsesOwnedUuidOnly、
DuplicateCancelOnlyOneBackendCall、LateResultDoesNotReleaseFaultLock、
CompletedGoalSameIdDifferentTrajectoryRejected、NormalFakeTrajectoryStillExecutesOnce等保留。
默认禁用、Unknown UUID、fresh停止证据、spline及完整命令身份等负面用例均通过。

重复测试采用40个独立进程，每轮9项（原ConcurrentGoalsSingleOwner + 新8项），
每轮40秒外部期限；独立进程避免把故障后刻意保留的endpoint quarantine当成自动复位。
全部40轮、360次用例执行通过，0失败/0超时；逐轮XML核对9个身份，重复不增加独立数量。
证据：/tmp/phase2b_mutex_repeat/{00..39}.log/.xml与summary.json。

## 构建与全量回归

普通8包构建PASS；7包测试PASS。独立目录保留历史build/install/log，没有删除旧TSan。

| 包 | 普通/UBSan每轮独立GTest |
|---|---:|
| mtc_core_cpp | 23 |
| mtc_task_cpp | 87 |
| mtc_manipulation_cpp | 120 |
| mtc_motion_execution_cpp | 88 |
| mtc_ros2_action_cpp | 40 |
| mtc_moveit_planner | 29 |
| mtc_trajectory_execution_cpp | 99（30校验 + 61原Action + 8新增） |
| 合计 | **486 PASS；FAIL0/ERROR0/SKIP0；26个可执行文件** |

verify_results.py把基线从463升级为固定681bb64的478，继续要求源码声明、实际二进制
注册、实际XML身份一致、每身份仅一次且完成、无missing/failure/error/skip。未降低检查。
各包colcon test-result单独核对，CTest包装数量未重复算作独立GTest。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/robot/meituan_challenge_ws
colcon --log-base log/phase2b_mutex build \
  --build-base build/phase2b_mutex --install-base install/phase2b_mutex \
  --packages-up-to mtc_trajectory_execution_cpp mtc_moveit_planner mtc_ros2_action_cpp \
  --parallel-workers 2
source install/phase2b_mutex/setup.bash
colcon --log-base log/phase2b_mutex_regression test \
  --build-base build/phase2b_mutex --install-base install/phase2b_mutex \
  --packages-select mtc_core_cpp mtc_task_cpp mtc_manipulation_cpp \
  mtc_motion_execution_cpp mtc_ros2_action_cpp mtc_moveit_planner \
  mtc_trajectory_execution_cpp --executor sequential
python3 src/mtc_trajectory_execution_cpp/test/verify_results.py --build-base build/phase2b_mutex
```

最终普通结果：/tmp/phase2b_mutex_regression.log、/tmp/phase2b_mutex_identity.log；
初次全构建/tmp/phase2b_mutex_build_initial.log，后续目标构建/tmp/phase2b_mutex_build_new.log。
完整colcon日志位于log/phase2b_mutex*，XML为build/phase2b_mutex各包/test_results。
本轮没有重新运行Python269/离线E2E15，不引用历史数字为PASS：**NOT RUN**。

## UBSan / TSan

UBSan独立build/phase2b_mutex_ubsan和install/phase2b_mutex_ubsan：8包构建/7包测试PASS，
原478+新增8全部实际执行486 PASS。CXX_FLAGS为-fsanitize=undefined
-fno-sanitize-recover=all -fno-omit-frame-pointer，exe/shared链接-fsanitize=undefined；
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1。系统ROS/DDS/MoveIt未全部重新插桩。
证据/tmp/phase2b_mutex_ubsan_build.log、_test.log、_identity.log。

TSan独立build/phase2b_mutex_tsan、install/phase2b_mutex_tsan：4包构建完成；实际启动
ConcurrentPollingAndCancel在进入GTest前退出66：

```text
FATAL: ThreadSanitizer: unexpected memory mapping 0x5b4c634d4000-0x5b4c63915000
```

**BLOCKED：TSan运行环境；完整TSan回归NOT RUN，实际GTest执行0。**
/tmp/phase2b_mutex_tsan_build.log、_run.log、_result.json保存实际记录。
没有改ASLR、安全设置、降低检查或删除历史日志。UBSan PASS不等于没有数据竞争。

## GenericSystem Tests：严格Status保持BLOCKED

本轮运行只读已验证aubo_control.launch.py，明确aubo_type:=aubo_S3、
use_fake_hardware:=true、launch_rviz:=false。展开Xacro前确认已有S3生成模型，
静态仅mock_components/GenericSystem；Domain216、LOCALHOST、无static peers/profile，
先确认域空，再启动自己的Fake进程。没有move_group/真实设备/SDK/IO。

真实服务：唯一GenericSystem Active，JTC与joint_state_broadcaster均Active，收到72个
JointState样本，六关节名完整。拓扑预检：FJT Server1/Client1、direct publishers0。
原生Status未收到：status_received=0/status_fresh=0/ready=0；topology模式返回2，
**BLOCKED，Goal0/Cancel0**。此模式根本不调用execute_trajectory。
没有人工空Status、预热Goal、heartbeat、放宽检查或ControlledFakeStart。

自己的GenericSystem Launch正常exit0；驱动源/安装5656个既存文件SHA256前后相同。
实际证据：/tmp/phase2b_mutex_native/{probe.log,control.log,result.json,fake_expanded.urdf}，
Supervisor与只读检查日志/tmp/phase2b_mutex_native_gate.log。
原生三点执行/起点/UUID取消/Planner联合执行本轮 **NOT RUN**，不能用Fake Server替代。

## 修改文件与剩余边界

1. src/mtc_trajectory_execution_cpp/src/fjt_motion_port.cpp：出站队列、锁外API、异常归属、
   发送前门禁复核、迟到响应callback group生命周期。
2. src/mtc_trajectory_execution_cpp/test/test_fjt_action.cpp：仅追加8项真实ROS并发测试。
3. src/mtc_trajectory_execution_cpp/test/verify_results.py：固定478项基线。
4. src/mtc_trajectory_execution_cpp/README.md：本轮结果和范围说明。
5. docs/migration/PHASE2B_ACTION_MUTEX_BACKPORT_GATE.md：本报告。

公共头、CMake、Core/Manipulation/FSM/Planner/Phase2A/ROSIDL没有改变。
没有ControlledFakeStart、verify_empty_domain、ExecutionAuthority、controlled_start、
authority_established、PlanningOnlyManager、DispatchAdapter、single_owner或时间戳重写。
默认执行仍禁用，原Status QoS/年龄/UUID未知策略、完整命令记录和停稳条件保留。

仍BLOCKED：原生JTC冷启动Status、标准MoveIt第二FJT Client、完整Planner/FJT联合执行、
FSM异步Stop Bridge；均属Phase2C后续架构，不在本轮独立模块合并条件内。
通信永久中断时不能保证取消送达，必须保留未知/FAULT/quarantine；图检查不是认证。
真实设备、工具IO和实体停止验证 **NOT RUN**，禁止据此授权实机。

## PR #7 合并建议

**建议在手工Push本次实现和报告、PR复审及CI确认后合并更新后的PR #7，作为独立
Phase2B模块。** 已满足本轮锁序、478身份、新增8、UBSan、UUID/命令身份、默认禁用、
模块隔离条件。TSan环境限制及GenericSystem BLOCKED如实披露，无新增未解决P0。
当前远端681bb64没有本修复，不能把本地PASS当成远端PR已更新。

后续由负责人执行（本轮没有执行）：

```bash
git switch feature/phase2b-trajectory-execution
git status --short
git fetch origin
git merge-base --is-ancestor origin/feature/phase2b-trajectory-execution HEAD
# 上一步成功、工作树干净且确认没有新分叉后：
git push origin HEAD:feature/phase2b-trajectory-execution
```

在GitHub PR #7核对新HEAD、限定上述5文件、复审锁序/析构测试及CI证据；review/checks
通过后才由负责人手工合并。若远端新增提交，不强推或重置，先检查差异。本轮停止，
不自动push/merge、修改Phase2C历史或开始Phase2C P1。
