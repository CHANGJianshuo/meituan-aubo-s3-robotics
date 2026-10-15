# Phase 2B Final Audit & Integration Gate

日期：2026-10-12。仅离线软件、真实 ROS Action Fake Server 和
`mock_components/GenericSystem` 验证。没有连接真实机器人、控制柜或实体工具 IO。

## 结论

**PASS：命令身份修复、main 集成、构建及回归。BLOCKED：默认 MoveIt 与 FJT 联合执行。**
可创建供审查的 PR，但本报告不建议以“联合执行链路已打通”为理由合并 main；
执行权、时间戳及异步取消宿主三个阻断需由负责人明确解决并重新验收。
没有自动 push、创建 PR、合并 main 或修改其他成员分支。
最终文档提交后要求 `git status --short` 空输出；实际检查结果见任务最终回复。

| 本轮检查 | 结果 | 实际证据 |
|---|---|---|
| 完整工作空间构建 | PASS | 15/15 包；含新合入 Planner |
| 指定联合构建 | PASS | packages-up-to Planner、FJT、Phase 2A；8/8 包 |
| 独立 GTest 身份及执行核验 | PASS | 463 项、26 个可执行文件，FAIL 0 / ERROR 0 / SKIP 0 |
| 原 424 项身份 | PASS | 原 358 项 Core/Action + 原 66 项 Phase 2B 全部保留并执行 |
| 本轮新增测试 | PASS | 10 项；合入 main 的 Planner 另有 29 项 |
| Python pytest | PASS | 269 项；FAIL 0 / ERROR 0 / SKIP 0 |
| Python 离线 E2E | PASS | 15 项；FAIL 0 / SKIP 0 |
| 独立 UBSan | PASS | 同样 463 项，halt_on_error=1；7 个 C++ 包均实际执行 |
| TSan | NOT RUN | 本轮未尝试；保留历史 unexpected memory mapping 日志 |
| GenericSystem 功能 | PASS | 完整三点、起点拒绝、UUID Cancel、完成身份拒绝四项 |
| GenericSystem + move_group + FJT | BLOCKED | FJT 客户端为 2；ready=0，零 Goal |
| move_group 的 SIGINT 正常退出 | FAIL | 5 秒后 Launch 升级 SIGTERM，进程 exit -15 |
| 驱动工作空间不变 | PASS | 源/安装 842 个文件 SHA256 与运行前一致 |
| 真实硬件、接触执行、完整抓放 | NOT RUN | 未授权；接触执行保持显式拒绝 |

历史 424 项数字未用作本轮 PASS；以上是本轮 XML、日志和实际程序输出。

## Git 基线与变更范围

- 当前分支：`feature/phase2b-trajectory-execution`；开始时工作树干净。
- 保留历史提交：`b47b5fed8da403c3ae350c07902a9a0b998849e5`、
  `688d80d63ba2196e3c1753b69d18c1bfa30919b3`，均确认是当前 HEAD 的祖先。
- 本轮先 `git fetch origin`；main 为 `f7eaa2c0d2887aeada4fd635112c63c237962ff7`，
  Git 历史确认 PR #3 MoveIt Planner 已合并。
- GitHub API 对 PR #4 实际返回 `state=closed, merged=false, merged_at=null`。
  PR #4 的 Planning Scene / Start Consistency 功能不能假定已在 main。
- 先提交 P0 修复：`d491a96`；工作树干净后再次 fetch 并合入 origin/main。
- 无冲突合并提交：`3750d9959fb32c8073754cd675478bbcd897cf8f`。
- 代码与验证工具验收 HEAD：`2e00a36bd60da518cac465b2edc5d71699ce9234`。本报告及 README 随后单独提交；
  文档提交身份用 `git log -1 --format=%H -- docs/migration/PHASE2B_FINAL_INTEGRATION_GATE.md`
  获取，最终交付 SHA 在任务回复中提供。

本轮修改文件（不包含从 main 原样合入的 53 个 Planner/证据文件）：

| 文件 | 修改 |
|---|---|
| [fjt_motion_port.cpp](../../src/mtc_trajectory_execution_cpp/src/fjt_motion_port.cpp) | 不可变命令副本与统一全字段比较 |
| [test_fjt_action.cpp](../../src/mtc_trajectory_execution_cpp/test/test_fjt_action.cpp) | 10 项真实 ROS Action 回归 |
| [fake_trajectory_probe.cpp](../../src/mtc_trajectory_execution_cpp/src/fake_trajectory_probe.cpp) | identity、无发令 topology 诊断 |
| [verify_results.py](../../src/mtc_trajectory_execution_cpp/test/verify_results.py) | 原 424 项身份及新 Planner 的注册/执行核验 |
| [包 README](../../src/mtc_trajectory_execution_cpp/README.md) | 当前结果与联合集成阻断 |
| 本报告 | 本轮实际验收记录及 PR 准备说明 |

FSM、MotionAdapter、ROSIDL、Phase 2A 默认 Mock Server、Planner 源码和驱动配置均未修改。
不对原有 Python 包做任何修改。

## P0：完成 command_id 内容重用

根因：完成账本仅保存 `MotionResult`，命中 ID 后直接返回旧结果；在途路径比较
归一化后的 ROS 轨迹及 timeout，遗漏 trajectory_id、原始关节顺序、scaling、
simulation_only、intent Pose 等。归一化可能抹去原始命令内容差异。

先添加真实 `rclcpp_action` Fake Server 测试，再编译旧实现运行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
build/mtc_trajectory_execution_cpp/test_fjt_action \
  --gtest_filter='FjtAction.CompletedGoal*:FjtAction.InFlightIdentityIncludesOriginalOrderAndIntentPose' \
  --gtest_output=xml:/tmp/mtc_gate_before.xml
```

修改前实际结果：6 项运行，1 PASS / 5 FAIL。失败为不同轨迹、不同 timeout、不同
安全字段、FAULT 后历史 ID 变化以及在途原始字段差异；同内容缓存读取通过。
原始输出保留在 `/tmp/mtc_gate_before.log` 与 XML；修复后这 6 项全部 PASS。

修复采用不可变的 `const MotionCommand identity` 深复制：Session 在发 Goal 前保存，
完成账本保存同样的 identity 与不可变 result。持有原有 mutex 时，所有在途、已终结、
已缓存结果路径均先调用同一个 `same_command()`。没有 hash 或容差比较。

比较覆盖 command_id、command joint_names/target_rad、timeout、两项 scaling、
primitive_name、requires_contact、simulation_only、intent 是否存在及全部字段、
完整 Pose（位置、四元数、frame、stamp、valid）、trajectory 是否存在、trajectory_id、
原始关节名称和顺序、frame、stamp、每个点的时间及 positions/velocities/accelerations。
特别补比 `MotionPrimitive::operator==` 未包含的 target_pose，未改公共 Core API。

相同内容可以继续轮询或读取历史结果；不同内容明确返回 REJECTED，并且不改变所有权、
不发送 Goal/Cancel、不清除 FAULT。旧成功记录只是历史事实，不能作为当前停稳证据；
新 Goal 仍需原有健康和停稳门禁。完成账本仍限 10000 条、不淘汰、不自动清锁。
保存完整轨迹会增加账本内存，轮询比较为 O(路点数)，应在宿主层限制任务资源。

新增测试均属于 `FjtAction`，由 CMake 注册的 `test_fjt_action` 实际运行：

1. `CompletedGoalSameIdSameContentReturnsCachedResult`
2. `CompletedGoalSameIdDifferentTrajectoryRejected`
3. `CompletedGoalSameIdDifferentTimeoutRejected`
4. `CompletedGoalSameIdDifferentSafetySemanticsRejected`
5. `CompletedGoalIdentityMismatchNeverSendsAgain`
6. `InFlightIdentityIncludesOriginalOrderAndIntentPose`
7. `CompletedIdentityPreservesEveryOriginalTrajectoryField`
8. `FaultedIdentityMismatchPreservesLockAndResult`
9. `MonotonicPlannerStampCannotBecomeRosScheduledStart`
10. `PastPlannerGenerationStampDoesNotDispatch`

前四项及字段枚举用例验证 Goal 接收计数保持 1、Cancel 计数 0。第五项先完成旧 Goal，
再以第二 Goal 的 ABORTED 建立 FAULT，旧 ID 内容变更拒绝后计数保持 2、Cancel 0、
锁保持不变；第八项直接检查故障命令原结果、UUID、锁与计数不变。
迟到 Cancel、结果竞争、超时、外部在途 Goal 等原有测试全部保留并实际通过。

## MoveIt Planner 与执行端契约

读取 `move_group_backend.cpp`、`moveit_planner_port.cpp`、`trajectory_conversion.cpp`
及 FSM `make_command()`；没有改动其实现。

| 项目 | 审计结论 |
|---|---|
| 关节名称/顺序 | Planner 按 FSM 名称重排，FJT 再按控制器名称映射；没有按索引猜顺序 |
| 路点与时间 | TOTG 输出完整多点，经因子放慢，速度除因子、加速度除因子平方；保留完整轨迹 |
| t=0 | 本轮真实规划首点为 0；Planner 通用转换允许非零首点，但 FJT 能力明确要求 0 |
| 位置/速度/加速度 | 维度及有限性有校验；FJT 还要求一致的导数字段和整个插值段的保守边界 |
| command_id / trajectory_id | Planner 输出为空；FSM make_command 派发时分别赋值，直接绕过 FSM 会被拒绝 |
| simulation_only | Planner 默认 force_simulation_only=true；FJT 仅接受 true，不能用于真实硬件 |
| timeout | Planner 默认 duration×1.5 + 1000ms；还检查 FSM 看门狗，FJT 独立检查时长、能力与停稳余量 |
| 位置与动态限位 | Planner 的位置限位可选，FJT 六关节限位必配；本轮探针 ±0.5rad / 0.2rad/s / 0.5rad/s² |
| spline | Bernstein 控制点包络检查保留；拒绝意味着无法证明插值安全，不自动取消校验 |
| 起点 | Planner 比较规划链起点；FJT 额外比较新鲜实际 JointState，不自动跳到首点 |
| requires_contact | 接触阶段属于当前不支持能力，保持拒绝；不是自由空间规划成功就可以执行 |
| stamp_s | **BLOCKED**：Planner 是生成/观测时刻，可能为 SteadyClock；FJT 非零 stamp 是 ROS 计划启动时间 |

本轮使用现有 `plan_tool.launch.py program:=plan_fixed_pose`，输出目录在 `/tmp`，
真正调用 MoveIt 仅规划，未调用 execute 或发送 FJT Goal。结果：160 点，首点 0，
总长 15.8704 秒，规划成功。使用独立 CMake consumer 链接已安装的 Planner core 与
FJT 库，编译与链接通过，确认安装导出及跨包 API/ABI 消费可用；读取这份输出并调用
两侧实际转换 API（没有 Node、Action Client 或 Goal）：

```text
Planner conversion=1 points=160 first_t=0 duration=15.8704
Before FSM assigns IDs: only identified simulation-only complete joint trajectories; contact unsupported
With explicit diagnostic IDs, current probe envelope: velocity limit exceeded
No Node, Action Client, or Goal created.
```

这里是 **Planner 输出有效但超出当前窄执行能力**，不等于规划无效；未扩大实际执行限位。
即使换成适当的经审核限位，仍须独立证明 spline、当前起点、执行权与时间契约。
历史规划证据只能辅助理解，不能替代本轮运行结果。

`MoveItPlannerPort` 使用注入 Clock 写 stamp；现有 sequence 工具注入 SteadyClock。
fixed_pose 工具使用 node->now()，但它仍是已过去的生成时间。FJT 对两类不合理启动
时刻均零 Goal 拒绝，由新增真实 Action 测试确认。需要明确区分“生成时刻”与
“调度启动时刻”的宿主契约；不能静默覆写为 0。此处留为集成设计阻断，不改公共 API。

## 执行权运行检查

隔离 ROS_DOMAIN_ID=219，LOCALHOST，禁用 static peers/额外 DDS profiles；启动前 domain
为空。Xacro 展开仅一个 GenericSystem；服务查询返回 auboHardwareInterface Active、
200Hz、六关节 position 被 JTC 声明，position/velocity 状态可用。
joint_state_broadcaster 和 joint_trajectory_controller 都 Active。

| 场景 | FJT Server | FJT Client | 直接轨迹 Publisher | 外部在途 Goal | Port 健康 |
|---|---:|---:|---:|---:|---|
| Controller 单独（参考） | 1 | 0 | 0 | 无本轮外部 Goal | 无 Port |
| A Controller + Port | 1 | 1 | 0 | 0 | ready=1，四项验证通过 |
| B Controller + move_group | 1 | 1 | 0 | 未发送执行 Goal | 无 Port；两 controller Active |
| C Controller + move_group + Port | 1 | 2 | 0 | 0 | ready=0，BLOCKED，零 Goal |

B 的额外客户端为 `/moveit_simple_controller_manager`，来自 move_group 的
MoveItSimpleControllerManager；运行图存在 `/execute_trajectory` 和 `/move_action`，
日志加载 ExecuteTrajectory 等能力，不能把该客户端当成不可执行的被动观察者。
业务后端仅调用 plan()/computeCartesianPath()，不代表 move_group 的执行能力已禁用。
`allow_trajectory_execution` 的本轮查询返回 Parameter not set，不能假定是 false。

C 的实际程序输出：

```text
FJT client node=//mtc_fake_trajectory_probe
FJT client node=//moveit_simple_controller_manager
FJT servers=1 clients=2 direct_trajectory_publishers=0 foreign_active_goals=0 ready=0
BLOCKED: Fake execution ownership gate; no Goal sent
```

保持 exclusive_graph 原样，未删除检查、未允许 >=1、未抢占未知 Goal。
外部状态 UUID 与直接轨迹 Topic 检测仍启用。DDS 图是受信任隔离软件环境的拓扑证据，
不是强身份认证，也不能证明恶意/竞态节点永远不发命令。

最小后续方案：建立独立且经审核的 planning-only move_group Launch，禁用其轨迹执行
管理与执行入口，保留规划服务；所有实际 FJT 请求集中到唯一执行管理者。随后重新
验证客户端图、未知在途 Goal、直接 Topic、Controller 所有权和取消路径。现有只读
Launch 没有暴露此开关；本轮未改驱动配置或 MoveIt 内部配置来绕过门禁。

## GenericSystem 实际验证

在上述静态与运行时双重确认后才执行探针，每条三点轨迹只提交一个 FJT Goal：

| 模式 | 结果 | 实际统计 |
|---|---|---|
| success | PASS | Goal 1 / Cancel 0 / Feedback 40；终点六关节 0.02，stop_confirmed=true |
| mismatch | PASS | 明确 trajectory start differs；Goal 0 / Cancel 0 |
| cancel | PASS | Goal 1 / Cancel 1 / Feedback 10；UUID b8256576b418307012b63f0b5a01886d |
| identity | PASS | Goal 1 / Cancel 0 / Feedback 39；同内容重读、改路点/timeout/simulation 三次拒绝 |
| topology + move_group | BLOCKED | 客户端 2、ready=0；不发送运动 Goal |

Cancel 首次 stop_confirmed=false；只有控制器明确终结 CANCELED、持续新鲜反馈和
静止窗口满足条件才确认。success/identity 检查控制器结果、实际终点及持续静止反馈。
这些证据来自理想化 GenericSystem，不是实机动力学或安全急停证明。

环境仍报告 controller_manager 未启用命令限位、部分 URDF `{±2*pi}` 无法解析和
FIFO 权限缺失。未改驱动来掩盖；FJT 配置的有限范围及插值检查独立生效。
Controller 正常 deactivate/shutdown，进程 exit 0；move_group SIGINT 后升级 SIGTERM，
该退出子检查 FAIL。仅关闭本轮启动的进程，最终 domain 空，没有清理其他成员节点。
驱动已有 `aubo_description` dirty 状态为运行前即存在，未恢复或修改。

## 构建、测试与复现

只加载 Jazzy 编译主仓库，保留原 build/install/log，不执行清空命令。

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log/phase2b_gate build --packages-up-to \
  mtc_moveit_planner mtc_trajectory_execution_cpp mtc_ros2_action_cpp --parallel-workers 2
colcon --log-base log/phase2b_gate_full build --parallel-workers 2
source install/setup.bash
colcon --log-base log/phase2b_gate test --packages-select \
  mtc_core_cpp mtc_task_cpp mtc_manipulation_cpp mtc_motion_execution_cpp \
  mtc_ros2_action_cpp mtc_moveit_planner mtc_trajectory_execution_cpp --executor sequential
python3 src/mtc_trajectory_execution_cpp/test/verify_results.py --build-base build
colcon test-result --all --verbose
```

身份脚本将源码历史 424 项、当前全部注册二进制 inventory 和完成 XML 逐一比对，
不是根据测试文件存在或 colcon 退出码判断。各包当前独立 GTest 数量：

| 包 | PASS |
|---|---:|
| mtc_core_cpp | 23 |
| mtc_task_cpp | 87 |
| mtc_manipulation_cpp | 120 |
| mtc_motion_execution_cpp | 88 |
| mtc_ros2_action_cpp | 40 |
| mtc_moveit_planner | 29 |
| mtc_trajectory_execution_cpp | 76（转换 30 + Action 46） |
| 合计 | 463 |

colcon scoped 汇总还计入 CTest 包装器，不能重复算作独立 GTest。
全目录初次汇总为 3967 tests / 14 errors / 14 failures / 0 skipped，其中包含历史目录。
最终全目录实际汇总为 **4458 tests / 14 errors / 14 failures / 0 skipped**；
14 个失败均在旧 audit_tsan、final_gate_tsan 的 Core/Motion CTest 中，未删除。
本轮各包 scoped 与身份核验均无失败。
历史 TSan 日志如 `build/audit_tsan/mtc_motion_execution_cpp/Testing/Temporary/LastTest_20261011-0627.log`
仍保留 `FATAL: ThreadSanitizer: unexpected memory mapping`。没有删除失败日志或修改 ASLR。

本轮另外执行六个 Python 包、bringup 与 interfaces 的已注册测试；269 项 Python 为
bridge 25、manipulation 52、motion 20、safety 70、task 63、tool 39。
首次包选择拼成 mtc_bridge 后 colcon 警告 unknown package；已纠正为 mtc_aubo_bridge
并实际执行，不把该第一次未执行算作 PASS。bringup 没有业务用例，接口 XML 测试通过。
`python3 tests/test_mock_end_to_end.py` 实际输出 PASS 15 / FAIL 0 / SKIP 0。

UBSan 独立目录 `build/phase2b_final_gate_ubsan`、`install/phase2b_final_gate_ubsan`、
`log/phase2b_final_gate_ubsan`；8 个依赖包构建、7 个 C++ 包测试：

```bash
source /opt/ros/jazzy/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base log/phase2b_final_gate_ubsan build \
  --build-base build/phase2b_final_gate_ubsan --install-base install/phase2b_final_gate_ubsan \
  --packages-up-to mtc_moveit_planner mtc_ros2_action_cpp mtc_trajectory_execution_cpp \
  --parallel-workers 1 --cmake-args \
  '-DCMAKE_CXX_FLAGS=-fsanitize=undefined -fno-sanitize-recover=all -fno-omit-frame-pointer' \
  '-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=undefined' '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=undefined'
source install/phase2b_final_gate_ubsan/setup.bash
export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
colcon --log-base log/phase2b_final_gate_ubsan test \
  --build-base build/phase2b_final_gate_ubsan --install-base install/phase2b_final_gate_ubsan \
  --packages-select mtc_core_cpp mtc_task_cpp mtc_manipulation_cpp mtc_motion_execution_cpp \
  mtc_ros2_action_cpp mtc_moveit_planner mtc_trajectory_execution_cpp --executor sequential
python3 src/mtc_trajectory_execution_cpp/test/verify_results.py --build-base build/phase2b_final_gate_ubsan
```

463 项 UBSan 实际 PASS；系统 ROS、DDS、MoveIt 二进制未重新插桩，不能推导其全部无 UB。
TSan 本轮 NOT RUN，历史环境错误不冒充本轮尝试。SKIP 0 不包括主动 NOT RUN 的硬件/TSan。

当前主要日志位于 `/tmp/mtc_gate_*.log`、`/tmp/mtc_gate_runtime_*.txt` 及上述 colcon 日志。
GenericSystem 启动步骤依照 [已核验驱动指南](../setup/AUBO_ROS2_DRIVER_JAZZY.md)，始终显式
`aubo_type:=aubo_S3 use_fake_hardware:=true`；不得换成真实模式。
探针新增 identity/topology 参数的复现方法见 [包 README](../../src/mtc_trajectory_execution_cpp/README.md)。

## 阻断与 PR 准备

1. **BLOCKED：执行权。** 默认 move_group 持有另一个能执行的 FJT Client；需要 planning-only
   服务和唯一执行管理者，不能只接受两个客户端。
2. **BLOCKED：时间契约。** 生成/观测时间与 ROS 调度启动时间尚未区分；应由明确宿主
   契约处理，不在本轮擅自改 Core/Planner 公共接口。
3. **BLOCKED：取消宿主。** 现有 FSM 同步 request_stop 一次、未确认即 FAULT(520)。异步
   Port 直接注入不能自动解决；Phase 2C 宿主应非阻塞等待证据、冻结派发、超时留 FAULT。
4. **FAIL：move_group 正常退出。** 应在专用 Mock Bringup 中核查退出生命周期；本轮未扩展重构。
5. 实机限位、TCP/TF 标定、接触/工具控制、Planning Scene 新分支和完整任务闭环均未验收。
   本轮没有修 V1 side_insert 看门狗、修改标定或引入真实后端。

建议 PR 标题：`fix(motion): enforce complete FJT command identity and record integration gate`。
建议正文：修复同 command_id 不同内容返回旧成功；保留 Cancel/FAULT 与图门禁；合入 PR #3
基线并通过 463 GTest、269 pytest、15 E2E、463 UBSan 与四项 GenericSystem；联合
MoveIt/FJT 因执行权及时基 BLOCKED，不能声称完整执行集成。TSan NOT RUN。

**可创建审查 PR；当前不建议将该分支作为“联合集成验收通过”合并 main。** 若只审查独立
Fake-only 执行端和身份修复，负责人可按明确限定范围评估合并；阻断解决前不得开启完整宿主
或进入真实设备操作。本轮停止在此，不开展 Phase 2C。
