# Phase 2A：C++ ROS 2 Action 离线 Mock 外壳

本包复用 `TaskFsm`、`PickPlaceFsm` 和 `MotionExecutorCore`，没有复制 FSM 或修改其
状态转换。ROS 依赖只出现在本包，四个核心包保持零 ROS 依赖。所有端口均为
显式离线模拟；无控制柜地址、驱动加载、RPC/RTDE、MoveIt 或实体工具 IO 通路。

| Action | 类型 | 可执行程序 |
|---|---|---|
| `/mtc/manipulation/pick_place` | `mtc_interfaces/action/PickPlace` | `mock_pick_place_server` |
| `/mtc/task/execute` | `mtc_interfaces/action/ExecuteTask` | `mock_task_server` |
| `/mtc/motion/execute_joint_move` | `mtc_interfaces/action/ExecuteJointMove` | `mock_motion_server` |

独立驱动的 GenericSystem / MoveIt Mock 验证与本包是两个环境。本包**没有接入
MoveIt Planner、FollowJointTrajectory 或 ros2_control Mock Hardware**，不能用
Action 成功宣称完整第二阶段或实机验收完成。三个 ROSIDL 定义保持不变。

## 编译与测试

在 `/home/robot/meituan_challenge_ws` 执行，保留已有 build/install/log：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/robot/meituan_challenge_ws
colcon build --packages-up-to mtc_ros2_action_cpp
source install/setup.bash
colcon test --packages-select mtc_ros2_action_cpp
colcon test-result --all --verbose

# 仓库build下保留历史sanitizer目录时，上述总览会包含旧失败；本包单独验收：
colcon test-result --test-result-base build/mtc_ros2_action_cpp --all --verbose

# 核心回归；身份核验要求这五包的最新XML均存在
colcon test --packages-select mtc_core_cpp mtc_task_cpp \
  mtc_manipulation_cpp mtc_motion_execution_cpp mtc_ros2_action_cpp
python3 src/mtc_ros2_action_cpp/test/verify_results.py --build-base build
```

历史 `tests/verify_cpp_audit_results.py` 固定16个可执行文件；本包使用新的核验脚本，
比较 Phase 2A 前的318个完整测试身份、本分支新增用例和实际 XML，检查遗漏、
重复、失败、SKIP 及未注册用例。没有改写历史测试脚本或删减旧测试。

## 仅运行离线 Mock

使用三个新终端，每个终端先执行同样的环境隔离。91是示例 domain，必须确认
没有其他应用占用；不得在已有 Python Server 或真实驱动所在 ROS 图中启动。
以下变量只影响当前终端，不改系统配置：

```bash
source /opt/ros/jazzy/setup.bash
source /home/robot/meituan_challenge_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=91
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS ROS_LOCALHOST_ONLY
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
export ROS_LOG_DIR=/tmp/mtc_phase2a_logs
ros2 node list --no-daemon --spin-time 2
```

启动前图应为空。终端1启动 PickPlace，保持运行：

```bash
ros2 run mtc_ros2_action_cpp mock_pick_place_server
```

终端2启动 Task，保持运行：

```bash
ros2 run mtc_ros2_action_cpp mock_task_server
```

终端3先核对节点、类型及唯一 Server，再提交基础任务：

```bash
ros2 node list --no-daemon --spin-time 2
ros2 action list -t
ros2 action info /mtc/manipulation/pick_place -t
ros2 action info /mtc/task/execute -t
ros2 action send_goal /mtc/task/execute mtc_interfaces/action/ExecuteTask \
  '{task_id: "mock_basic_001", mode: 1, ordered_colors: [1], timeout_ms: 20000, placement_stability_sec: 3.0}' \
  --feedback
```

预期只有本包的 `/mtc_mock_pick_place` 和 `/mtc_mock_task`，每个 Action 一个
Server；Feedback 包含 Task 状态和 PickPlace 子状态，最终 `success=true`、
`completed_count=1`、`completed_slots=[T0]`。颜色1为 red，合法颜色为1–4；
MODE_SEQUENCE 必须恰好三个不同颜色，按原顺序映射 P1/P2/P3。稳定时长必须
有限且大于0，示例3.0秒为工程默认值，不是比赛官方最低时间。

Motion Action 可以在**另一套空 domain** 中独立运行：

```bash
ros2 run mtc_ros2_action_cpp mock_motion_server
```

它只支持一个六关节目标，默认缩放上限0.2，使用核心校验和独立停稳证据。
`float32(0.2)` 在转换层规范化到核心 `double(0.2)`；紧邻上限的更大浮点值仍拒绝。
本包没有轨迹入口；`execute_trajectory` 明确拒绝，绝不拆成多个单点调用。
配置若试图启用 timed trajectory 或 Cartesian capability，构造阶段直接拒绝。

结束前等待当前 Mock Goal 终结；分别对 Task 和 PickPlace 程序 Ctrl+C，Motion
独立程序同样 Ctrl+C，再检查本次节点已退出。程序拥有并回收 worker，无 detached
thread；正常退出不是安全急停。不要自动重启 FAULT 节点来宣称完成人工恢复。

## API 与线程所有权

公共头文件位于 [include/mtc_ros2_action_cpp](include/mtc_ros2_action_cpp/)。
三个 Node 构造函数可接收 `rclcpp::NodeOptions`；PickPlace/Motion 还可注入
`MockOptions`，Task 可注入 `FsmsConfig`。这些是 C++ 离线测试配置，不提供后端
选择参数，不能通过 ROS 参数开启真实模式。

```text
TaskServer ── ROS PickPlace Action Client ──→ PickPlaceServer
                                              ↓ PickPlaceFsm
                                MockPerception / MockPlanner / MockTool
                                              ↓ AsyncMockMotion
                                   MotionExecutorCore → MockMotionAdapter

MotionServer（独立验证） → AsyncMockMotion → MotionExecutorCore → MockMotionAdapter
```

* 每个 Node 的 Server、Timer、Task 子 Client 属于同一个 MutuallyExclusive
  CallbackGroup。Goal/Cancel 回调只操作 admission slot；只有 Timer 调用 FSM。
  在4线程 Executor 下仍保持事件串行，不在 ROS 回调中 wait/get 未完成 future。
* 在返回 ACCEPT 前按 UUID 预留唯一 owner；非法参数、并发 Goal、已用业务 ID、
  FAULT/未知状态锁定后的请求均 REJECT，不排队。UUID与业务ID用途不同。
* `AsyncMockMotion` 每条命令最多启动一个 execute worker，轮询返回 `nullopt`
  表示在途；关联 Stop 用独立 worker。派发、结果和取消交错时不覆盖执行所有权。
  Stop 结束之前冻结取消路径的 FSM 派发，不把 pending 当作停稳证据。
* Mock 执行器/Adapter 共享同一 ManualClock。Mock 运动瞬时推进虚拟时间；默认
  100ms仅延迟结果交付，方便观察/取消，并不是物理运动耗时。每个正常 FSM tick
  推进0.02虚拟秒；新感知样本必须有更新的时间戳，稳定窗口仍由原 FSM 核验。
  Task 截止时间和 ROS 结果交付期限使用 SteadyClock，不随 `/clock` 跳变。
* 规划显式生成非零模拟关节输出，保留 intent 的完整6D Pose、frame、stamp、
  接触约束；`simulation_only=true`。没有真实逆运动学/碰撞规划能力。
* MockTool 返回几何挂载、落座、卸载和释放证据，跟踪 holding 状态；只有成功
  release 后才宣称无载荷。工具方法不会访问任何实体 IO。
  Tool 实例跨 Goal 保留；Action 结束后仍 holding/unknown 时继续拒绝新 Goal，
  运动取消确认不等于负载已释放，不能通过新建 FSM 清除负载上下文。
* PickPlace 在本地复用执行器，没有调用 Motion ROS Action；独立 Motion Server
  是另一套 Mock 状态。后续硬件集成必须确立唯一执行权，不能把两套实例连接到
  同一机器人。关闭 Executor 并移除 Node 后才能销毁实例。

## 取消、超时与接口边界

取消只关联当前 ROS UUID；Motion worker 进一步使用原 `command_id` 调用
`cancel_goal`，不调用 `supervisory_stop`。旧 UUID、错 ID 不升级为全局停止。
原始 `CALL_PICK_PLACE.request_id` 保存在子 inbox，每次 ROS 子 Goal 追加唯一
attempt编号；`CANCEL_CHILD` 核对原 ID，再取消保存的那个子 Goal handle。
迟到回调只写入它原来的 inbox，不能推进新任务。

`PickPlace.action` 没有 `stop_confirmed` 字段，所以本包定义加强契约：只有
FSM=`CANCELLED` 且 `stop_confirmed=true` 才返回 ROS `CANCELED/OK`。
Task 只从关联子 Goal 的这个最终结果调用 `confirm_child_stopped()`；ACK、
子 SUCCEEDED、缺少子句柄、缺少结果均不能代替停止确认。首次运动尚未产生
证据或历史证据已过期且未取得新观测时，取消仍得到 `ABORTED/520`。
2026-10-13 C1 修复在 `AsyncMockMotion` 的关联 stop worker 内显式采样
同一条已知停稳的虚拟命令，再由未修改的 Core 验证。该采样保留原停稳
区间/命令身份；状态未知、缺少证据、错误 ID、时钟倒退均不能补造证据。
`stop_evidence()` 仍返回可过期的快照，不自动刷新；详见
[C1 修复与回归](../../docs/migration/C1_DELAYED_CANCEL_FIX_20261013.md)。

**这个契约只适用于可信的本包 Mock Server。公共 Result 没有足够字段独立核验
第三方的停稳证据。连接未知第三方/实机 Server 的通路未实现、不得验收；未来
接口/证据方案须单独审议，本轮不改 ROSIDL。**

超时和状态未知保留核心错误码（例如220、230、520），FAULT 返回 ABORTED 并
永久保留 admission lock，后台仍完成原命令的 Stop 清理。没有自动重发、自动
clear_unknown、人工复位 Service 或重新构造 FAULT FSM。Task 仅按核心契约
复位正常 FINISHED/TASK_FAILED/CANCELLED，FAULT 不执行 RESET。

## 测试场景与证据

[契约单元测试](test/test_action_contracts.cpp) 和三个 Action 测试使用真实 ROS
Client/Server 与4线程 Executor；测试进程选择本机隔离 domain，日志写 `/tmp`。
包含正向端口证据、非法参数、两线程同时发送 Goal、Feedback、关联取消、无
停稳证据、旧 UUID 零后端 Stop、子失败/伪造成功防护、ACK-only、Task deadline、
FSM watchdog、基础/序列任务及轨迹拒绝。Stop 与 execute 调用数来自实际
MockMotionAdapter override，不能只凭 `stop_confirmed=false` 判定取消隔离。

实际构建/测试结果和未运行边界见
[Phase 2A 交付报告](../../docs/migration/PHASE2A_ROS2_ACTION_REPORT.md)。
