# Phase 2B Ownership Final Review / PR #7 Merge Gate

日期：2026-10-12。仅软件 Fake Action、LOCALHOST DDS 和 GenericSystem 只读预检。
没有真实机器人连接、SDK/RPC/RTDE、实体运动或工具 IO。未开展 Phase 2C。

## 结论与 Git 身份

**PASS：针对性所有权修复及 478 项回归。BLOCKED：原生 GenericSystem 执行验收。
当前不建议直接合并远端 PR #7。**

- 分支：`feature/phase2b-trajectory-execution`。
- 开始时本地与远端功能分支均为 `45f497ce3444e255463faf41f7b736aa9ad07d7b`，工作树干净。
- 已执行 fetch；新增的是其他成员远端分支提交，未覆盖、合并或修改这些分支。
- GitHub API 确认 [PR #7](https://github.com/F1u0rite/meituan_challenge_ws/pull/7)
  为 open、未合并，head 为上述 45f497c。
- 代码验收提交：`b8857de49ec5405812f87887b1407092af7335a5`，提交信息为
  `fix(motion): harden FJT pending-goal ownership and status evidence`。
- 本报告和包 README 随后独立提交；文档提交 SHA 用
  `git log -1 --format=%H -- docs/migration/PHASE2B_OWNERSHIP_FINAL_REVIEW.md`
  查询，最终交付 SHA 和工作树状态见任务回复。
- 不 push、不创建 PR、不合并 PR/main，不修改其他成员模块。

## 实际复现与修复前后对比

读取原实现，发现 pending Handle 为空且活动数为 1 时，把未关联数直接改为 0；
默认空 goal_statuses 则让从未收到 Status 的实例看起来没有外部 Goal。

先扩展测试专用 Fake Server，再运行旧生产实现。服务、Goal、Feedback、Result、Cancel
仍是真实 rclcpp_action；Status 使用真实 DDS 的受控测试发布/转发流。没有访问 Port
私有状态，没有直接构造成功 MotionResult 来代替 Action。

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log/ownership_repro build --packages-up-to mtc_trajectory_execution_cpp
source install/setup.bash
build/mtc_trajectory_execution_cpp/test_fjt_action \
  --gtest_filter='FjtAction.NoStatusBeforeDispatchFailsClosed:FjtAction.StatusBeforeGoalResponsePreservesOwnership' \
  --gtest_output=xml:/tmp/mtc_ownership_before.xml
```

修改前实际运行 2 项，**0 PASS / 2 FAIL**：

| 用例 | 旧实现实际输出 | 修复后 |
|---|---|---|
| NoStatusBeforeDispatchFailsClosed | ready=true；goals_sent=1；Server goals=1 | ready=false，REJECTED，Goal 0 |
| StatusBeforeGoalResponsePreservesOwnership | Handle 尚未到，Status 有一个 UUID，却 unassociated count=0、ready=true | Unknown，未关联数 1，ready=false；原 Session 保留 |

原始输出 `/tmp/mtc_ownership_before.log`、XML 均保留；修复后同样两项实际通过，
记录在 `/tmp/mtc_ownership_after.log`。初次测试夹具编译有回调类型错误，已修正后重新
编译运行；当时旧二进制显示的“0 tests”没有被计入验收。

第二项通过 Fake Server 在真实 send_goal 回调中暂缓 Response，按该请求真实 UUID
发布受控 Status，制造客户端可见的顺序。它是人为构造的 ROS 消息时序复现，不能
声称原生 Jazzy Server 自然按此顺序发送，也没有复现真实机器人误停。
外部 Goal 的旧实现潜在错误归属由相同分支静态分析支持；修复后的真实外部 Client
竞态测试已执行，但没有把“旧实现取消了外部 Goal”写成已发生的运行事实。

## Jazzy Action 行为与证据边界

官方 [Server 源码](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp_action/src/server.cpp)
显示 Status 在 Goal 状态变化时发布，非周期心跳；native 发送 Response 后再发布
接受/执行状态，但独立 DDS 通道及回调调度不保证客户端处理顺序。运行时 JTC
Status Publisher 为 RELIABLE / TRANSIENT_LOCAL，Deadline 与 Lifespan 为 Infinite。

官方 [Client 源码](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp_action/include/rclcpp_action/client.hpp)
在发送请求时生成 UUID，收到接受 Response 才向应用提供携带该原请求 UUID 的 Handle。
本适配器不侵入其内部或重写 Client，以 Handle 精确关联原始 Session。

GoalStatusArray 没有消息发布时刻；goal_info.stamp 是 Goal 接受时间，不能当作 Status
心跳时间。空数组只是一份“没有列出 Goal”的观测；transient-local 保留的数组也不
证明此刻没有活动 Goal。没有消息则是 Unknown，不能解释成空数组。

本轮内部 Status 订阅改为 RELIABLE / VOLATILE，避免用历史缓存初始化当前证据。
还检查是否收到、消息合法性及本地接收年龄不超过现有 proof_max_age（默认 0.7s）。
这会拒绝静默、过期或无效数据；**接收年龄不是发布时刻证明**，不能识别所有延迟、
重放或不诚实发布者，也不自动建立绝对互斥。

ROS 图发现异步，按 node/endpoint 查询的 client 名称可能合并同 Node 的多个对象。
本轮真实外部 Client 测试在同一公开 Node 上建立第二个 client，保留所有现有图门禁，
证明仅靠拓扑不能构成对象计数或严格的跨进程独占锁。没有原子“查询空闲并占有”的
标准 Action 操作；检查后仍可能发生不可观测的新请求。

因此本包只在受信任、隔离、确认 GenericSystem 的软件环境使用上述有限证据，
不能推广为实机执行权或安全停机证明。观察到证据不足或冲突时 Fail-Closed；严格
独占需要后续明确的单执行管理者/证据契约，本轮没有实现或放宽这一前置条件。

## 所有权模型与十种时序

新增诊断枚举 `GoalOwnership`，同时保留 status_received、status_fresh、状态消息数
与 unknown_active_goals。status_fresh 指有限的本地接收可用性。
兼容字段 foreign_active_goals 保守统计所有尚未关联的活动 UUID，包括 Unknown；
不能把这个计数当作“每个 UUID 都已确定为 Foreign”。

| 分类 | 条件与含义 |
|---|---|
| UNKNOWN | 无 Status、无效/过期，或 Response 未到，或 Result/Active Status 互相矛盾 |
| OWN_GOAL | 匹配本请求 Handle 的 UUID；不是按活动 Goal 数量推断 |
| FOREIGN_GOAL | 无 pending 归属歧义时，活动 UUID 与本实例已知 Handle 不匹配 |
| NO_ACTIVE_GOAL | 可用 Status 不含活动 Goal，且无待确认 Session 或终态矛盾；仍不是全局锁 |

| 时序 | 修复后的行为 |
|---|---|
| 1. send_goal 前 | 必须满足 Fake/Active Controller/Action/新鲜 JointState/起点/限位/图及 Status 门禁，否则零 Goal |
| 2. 已发送，Response 未到 | 保留不可变命令和 Session，ready=false，不派发后继 Goal；不能猜测 Own |
| 3. Status 先到 | 保存所有观察到的活动 UUID，Unknown；有界等 Response，不删除 Session、不取消未知 UUID |
| 4. Response 先到 | Handle 的原请求 UUID 建立 Own 关联；后续 Status 按精确 UUID 分类 |
| 5. 外部 Goal 已存在 | 活动外部 UUID 阻止首条 Goal，不抢占、不 Cancel 外部 Goal |
| 6. pending 期间出现外部 Goal | 即使后续 Status 不再显示它，观测 UUID 记录仍保留；Handle 到达后不匹配即 FAULT，只 Cancel 自己 |
| 7. Response 拒绝 | 无矛盾观测则维持“从未执行”的拒绝结果；曾观察到未关联活动 UUID 则保守锁定，不全局取消 |
| 8. Response 延迟/丢失 | timeout 或证据丢失进入 FAULT，保留 Session；迟到 Handle 只关联原请求并执行其保留取消 |
| 9. Cancel 早于 Handle | 保存 cancel=true，Backend Cancel 0；Handle 到达后只发一次该 UUID 的 Cancel，ACK 不等于停稳 |
| 10. Result 终结、Status 有历史 | 已终结记录不当作活动；若仍是旧 Active Status，则 Unknown、禁止后继，等有效终态观测 |

tick 将环境证据、Status 证据、待响应和已关联执行分开。pending 的“等待”不代表
健康授权：不能报告 success/stop_confirmed，不能放行新 Goal。timeout 明确有界，
事件线程不等待完整运动或阻塞等待 Response。

异常/FAULT 结果不被迟到 Result 覆写。active、命令 ID、取消意图及 UUID 关联保留；
原有生命周期 quarantine 仍防止销毁/重建 Port 绕过锁定。没有人工复位伪造、global
cancel、Stop ACK 推导停稳或自动重发。软件 Stop 不是安全急停。

## 修改文件与保持不变的逻辑

| 文件 | 修改 |
|---|---|
| [fjt_motion_port.cpp](../../src/mtc_trajectory_execution_cpp/src/fjt_motion_port.cpp) | 移除 count==1 归属推断；Status 初始化/校验/接收年龄；pending UUID 记录与精确核对；终态 Status 门禁 |
| [execution_state.hpp](../../src/mtc_trajectory_execution_cpp/include/mtc_trajectory_execution_cpp/execution_state.hpp) | 所有权及 Status 诊断字段 |
| [test_fjt_action.cpp](../../src/mtc_trajectory_execution_cpp/test/test_fjt_action.cpp) | 受控真实 DDS Status、Response 门控、15 项异步测试 |
| [verify_results.py](../../src/mtc_trajectory_execution_cpp/test/verify_results.py) | 固定原463项，源码声明/二进制注册/XML三方核验 |
| [fake_trajectory_probe.cpp](../../src/mtc_trajectory_execution_cpp/src/fake_trajectory_probe.cpp) | 打印 Status 与 Unknown 证据，保持原发令保护 |
| [包 README](../../src/mtc_trajectory_execution_cpp/README.md) | 本轮结果、Status 契约及 GenericSystem BLOCKED |
| 本报告 | 时序、结果与合并门禁 |

未修改 FSM、MotionAdapter、MoveIt Planner、Phase 2A Action、ROSIDL、Launch、配置或
驱动工作空间。command_id 全字段身份、一次完整轨迹、按名称映射、spline 边界、
起点拒绝、Goal-scoped Cancel、独立停稳证据、FAULT/Timeout 锁与外部 Client 检查保留。

ExecutionSnapshot 新字段改变该包 C++ 结构布局：既有二进制消费者须重新编译；
本仓库测试及 probe 已重建，没有宣称旧二进制 ABI 兼容。Core/ROSIDL API 没有变更。

## 新增测试与实际回归

15 项均在真实 `test_fjt_action` 中、MultiThreadedExecutor（4线程）运行，服务响应
门控有 2 秒上界，spin 线程由夹具拥有并 join，无 detached thread，无 Port 私有状态注入。
Status 是测试专用真实 DDS 流：原生 Status 转发及显式软件心跳，支持暂停、延迟和
矛盾输入；它不被当作原生 JTC 已有心跳，也没有替 GenericSystem 注入任何证据。

1. NoStatusBeforeDispatchFailsClosed
2. ForeignGoalBeforeOwnGoalRejected
3. StatusBeforeGoalResponsePreservesOwnership
4. GoalResponseBeforeStatusMatchesUuid
5. ForeignGoalDuringPendingResponseFailsClosed
6. CancelBeforeGoalResponseUsesOwnedUuidOnly
7. HistoricalTerminalStatusDoesNotBecomeActive
8. StatusEvidenceLossDoesNotAuthorizeDispatch
9. LateResultDoesNotReleaseFaultLock
10. NormalFakeTrajectoryStillExecutesOnce
11. LatchedEmptyStatusCannotAuthorizeDispatch
12. InvalidStatusDoesNotAuthorizeDispatch
13. PendingResponseTimeoutRetainsSessionAndLateHandle
14. RejectedResponseAfterUnknownStatusKeepsFaultLock
15. TerminalResultBeforeStatusDoesNotAuthorizeSuccessor

第五项实际创建第二个 Client 并让外部 Goal 保持活动；pending 时禁止认领，随后还
隐藏该 Status，证明 UUID 记录不会丢失。原请求 Handle 迟到后只取消原 UUID，
本 Port goals_sent 仍为1；Server goals=2是Own请求加Foreign请求，不是重发。
其他拒绝场景检查零 Goal/Cancel；正常场景完整三点一次提交；早取消初始停稳=false。

第九项在 Status 丢失、取消被拒后收到真实迟到 SUCCEEDED Result，FAULT/原错误和
stop_confirmed=false 保持。第十五项真实 Result 先到、旧 Active Status 暂缓更新时
后继 Goal 被拒，只有终态 Status 放行后才允许下一条。

| 包 | 独立 GTest PASS |
|---|---:|
| mtc_core_cpp | 23 |
| mtc_task_cpp | 87 |
| mtc_manipulation_cpp | 120 |
| mtc_motion_execution_cpp | 88 |
| mtc_ros2_action_cpp | 40 |
| mtc_moveit_planner | 29 |
| mtc_trajectory_execution_cpp | 91（30 校验 + 61 Action） |
| 合计 | 478，原463项 + 新15项 |

**普通/UBSan 各478 PASS，FAIL 0 / ERROR 0 / SKIP 0；26 个实际可执行文件。**
没有把重复执行、CTest 包装器、源码存在或旧报告数字当作独立测试 PASS。
本轮 Python/E2E **NOT RUN**，沿用旧报告只作历史证据，不计入本轮结果。

## 构建、核验与 Sanitizer

Jazzy 下先 packages-up-to mtc_trajectory_execution_cpp（4个依赖包）构建通过，
再联合 Planner/Phase 2A（8个依赖包）构建通过。现有目录未清空。

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log/ownership_gate build --packages-up-to \
  mtc_trajectory_execution_cpp mtc_moveit_planner mtc_ros2_action_cpp --parallel-workers 2
source install/setup.bash
colcon --log-base log/ownership_gate test --packages-select \
  mtc_core_cpp mtc_task_cpp mtc_manipulation_cpp mtc_motion_execution_cpp \
  mtc_ros2_action_cpp mtc_moveit_planner mtc_trajectory_execution_cpp --executor sequential
python3 src/mtc_trajectory_execution_cpp/test/verify_results.py --build-base build
colcon test-result --all --verbose
```

当前七包 scoped 结果均 PASS；全目录汇总为 **4977 tests / 14 errors / 14 failures**，
失败全部在旧 audit_tsan/final_gate_tsan 的 Core/Motion CTest 目录。未删除任何日志。
当前结果另保留 `/tmp/mtc_ownership_current_scoped_results.log` 与身份 XML。

独立 UBSan 目录 build/ownership_ubsan、install/ownership_ubsan、log/ownership_ubsan：

```bash
source /opt/ros/jazzy/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base log/ownership_ubsan build \
  --build-base build/ownership_ubsan --install-base install/ownership_ubsan \
  --packages-up-to mtc_trajectory_execution_cpp mtc_moveit_planner mtc_ros2_action_cpp \
  --parallel-workers 1 --cmake-args \
  '-DCMAKE_CXX_FLAGS=-fsanitize=undefined -fno-sanitize-recover=all -fno-omit-frame-pointer' \
  '-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=undefined' '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=undefined'
source install/ownership_ubsan/setup.bash
export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
colcon --log-base log/ownership_ubsan test \
  --build-base build/ownership_ubsan --install-base install/ownership_ubsan \
  --packages-select mtc_core_cpp mtc_task_cpp mtc_manipulation_cpp mtc_motion_execution_cpp \
  mtc_ros2_action_cpp mtc_moveit_planner mtc_trajectory_execution_cpp --executor sequential
python3 src/mtc_trajectory_execution_cpp/test/verify_results.py --build-base build/ownership_ubsan
```

UBSan 8包构建/7包测试实际 PASS；系统 ROS/DDS/MoveIt 未全部重新插桩。
TSan **NOT RUN**；历史 unexpected memory mapping 记录保留，不将旧尝试算作本轮。
未调整 ASLR 或其他系统安全设置。UBSan 与多线程测试不等于证明不存在所有数据竞争。

## GenericSystem 实际结果与未执行项

只读 Xacro 展开证明唯一插件 mock_components/GenericSystem，目标 S3 Xacro 已存在，
避免 Launch 的兼容逻辑写入驱动目录。隔离 domain=220、LOCALHOST、无static peers，
启动前 domain 空；日志写 /tmp，PYTHONDONTWRITEBYTECODE=1。
实际服务查询：auboHardwareInterface Active、GenericSystem、200Hz；JTC和broadcaster
Active；Status Publisher 1、RELIABLE/TRANSIENT_LOCAL。没有启动真实模式。

运行 success 探针预检 10 秒，实际输出：

```text
FJT servers=1 clients=1 direct_trajectory_publishers=0 foreign_active_goals=0
status_received=0 status_fresh=0 unknown_active_goals=0 ready=0
BLOCKED: GenericSystem/active controller/fresh state/exclusive graph proof unavailable
```

这是 Status 初始化不足造成的 **BLOCKED，Goal 0**，不是轨迹执行 PASS。
没有预热 Goal、发布空数组、制造心跳或放宽默认门禁来继续运动。

| 用户验收项 | GenericSystem 本轮结果 | Fake Action 对应验证 |
|---|---|---|
| A 正常三点 | BLOCKED：实际预检拒绝，未执行轨迹 | PASS：真实三点单 Goal/结果/到位/静止窗口 |
| B 起点不一致 | NOT RUN：A 的先决证据不足 | PASS：原始起点拒绝用例，零 Goal |
| C UUID Cancel | NOT RUN：没有已授权的 Own Goal | PASS：早取消及准确 UUID，ACK 不直接停稳 |
| D command_id 内容重用 | NOT RUN：没有首次完成轨迹 | PASS：原完整身份回归全部保留通过 |
| E 外部 Goal / 乱序 | NOT RUN：未对原生JTC人为制造活动Goal | PASS：本轮受控真实DDS/Client/Server用例 |
| F MoveIt 共存 | BLOCKED：实际启动并检查，客户端2，零Goal | PASS：原外部 Client 门禁回归未放宽 |

F 的实际输出还显示 /moveit_simple_controller_manager 和本 Port 两个 Client，
server=1、direct publishers=0、Status 未初始化。两项独立门禁均不满足，不能把
拒绝掩饰成可联合执行。未修改 MoveIt 配置或发送规划/执行请求。

终止本轮拥有的进程后 domain 空。GenericSystem control 正常 exit 0；move_group
SIGINT 后仍需 Launch 升级 SIGTERM（exit -15），正常退出子检查 FAIL，未扩大修复范围。
驱动源/安装842个文件 SHA256运行前后完全一致；其预存 dirty 子模块未恢复或修改。
没有清理其他成员 ROS 节点。

## 剩余局限与合并意见

- **BLOCKED：原生 Status 契约。** 原生 JTC 没有初始实时空状态，也没有周期心跳。
  仅靠标准 Action 不提供严格原子执行权证明；需要明确独占执行管理和可信的实时证据
  契约，再重新验收原生 GenericSystem。不能通过模拟证据、预热运动或参数放宽解决。
- **BLOCKED：MoveIt第二执行客户端、Planner/FJT时基、异步FSM取消宿主。** 维持上一轮
  限制，不在本轮修改 Planner、FSM、MoveIt执行配置或实现 Phase 2C。
- 本轮只证明有界异步等待、准确关联、失败锁定和受控软件时序；未知 DDS 客户端、
  发现延迟、同 Node 客户端数量、恶意/延迟 Status 都不构成强身份或全局互斥保证。
- Snapshot结构变化要求消费者重建；TSan、Python/E2E本轮未运行；真实硬件未授权。

**当前不建议直接合并 PR #7。** 远端 PR 尚未包含本轮修复；即使负责人后续同步了
本地提交，也不能以“GenericSystem执行验收通过”合并，必须先决定并验收缺失的证据
契约及上述集成门禁。修复本身可供独立代码审查，不能把审查通过等同于可运行完整链路。
本轮停止在代码、Fake验证和清楚记录的阻断，不自动 push/merge 或进入下一阶段。
