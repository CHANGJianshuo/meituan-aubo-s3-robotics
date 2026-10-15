# 第二阶段：ROS 2 / MoveIt 2 集成计划

| 项目 | 值 |
|---|---|
| 前置阶段 | 第一阶段（C++ 纯逻辑 FSM 迁移）— 见 [`CPP_MIGRATION_TEST_REPORT.md`](CPP_MIGRATION_TEST_REPORT.md) |
| 前提 | 第一阶段审计通过 |
| 分支 | 继续在 `feature/jazzy-cpp-fsm`（或审计指定的新分支） |
| 编写时间 | 2026-10-11 |
| 状态 | **计划（未执行）** —— 本文件中的一切均为设计，不代表已实现或已验证 |

> **范围声明**：第二阶段仍**不实现实机连接**，全部验证在 Mock 环境下完成。
> 实机能力在本阶段结束时仍为 **NOT RUN**。

---

## 1. 第二阶段目标与非目标

### 1.1 目标

| # | 目标 | 依据 |
|---|---|---|
| G1 | 为 Task FSM / PickPlace FSM 建立 **ROS 2 Action 外壳**（本阶段**新增** `mtc_task_cpp`/`mtc_manipulation_cpp` 的 ROS 依赖选项或独立节点包） | 需求 6 "ROS 2 Action 外壳" |
| G2 | 用 **MoveIt 2 C++ API** 实现 `PlannerPort` / `MotionPort`（`MoveGroupInterface` + `FollowJointTrajectory`） | 需求 3 |
| G3 | 补齐**多点时间参数化轨迹接口缺口**（见 §3 设计提案） | 需求 4 |
| G4 | 修复接口审计发现的**取消链路断裂**与**状态话题冲突**（见 §5） | 接口审计 R 系列 |
| G5 | 建立 **Mock 集成测试**：C++ FSM ↔ `ros2_control` ↔ `mock_components/GenericSystem` | 需求 6 |
| G6 | 端到端 0.05 rad 单点轨迹 + 多点轨迹的 Plan→Execute 闭环（Mock） | 需求 6 |

### 1.2 非目标（明确排除）

* ❌ 实机 AUBO S3 连接、实机运动指令、实体工具触发
* ❌ 更改设备安全设置
* ❌ 重写 `mtc_safety` / `mtc_tool` / `mtc_aubo_bridge` / `mtc_bringup`（不在迁移范围）
* ❌ 修改官方 `aubo_driver_validation_ws`（保持只读）

---

## 2. 前置条件检查表

| # | 前置条件 | 状态 |
|---|---|---|
| P1 | 第一阶段 257 个 C++ 用例全部 PASS | ✔ 已满足 |
| P2 | 既有 Python 基线 255 例 PASS，既有文件未被修改 | ✔ 已满足 |
| P3 | 行为映射表 §9 的 9 项差异经审计确认 | ⏳ 待审计 |
| P4 | `mtc_interfaces` xmllint FAIL 的处理决定（是否单独修复） | ⏳ 待审计 |
| P5 | 差异 D-2（超时后 `StepResult.state` 与内部状态不一致）是否统一 | ⏳ 待审计 |
| P6 | 差异 D-6（C++ 6D 位姿保真）是否反向回补 Python | ⏳ 待审计 |

> **P3–P6 未决时不应开始 G1**：这些差异会影响 Action 外壳的错误码映射与
> `frame_id` 契约，先定契约后写外壳。

---

## 3. 接口缺口 G1：多点时间参数化轨迹（设计提案）

### 3.1 缺口事实（引自接口审计）

| 证据 | 内容 |
|---|---|
| `src/mtc_interfaces/CMakeLists.txt:37-41` | 仅注册 3 个 action：`ExecuteTask`、`PickPlace`、`ExecuteJointMove` |
| `src/mtc_interfaces/action/ExecuteJointMove.action:4` | 原文："不承诺中间点逐点时间同步；**禁止把轨迹拆成高频调用本接口来伪装轨迹控制**" |
| `src/mtc_interfaces/action/ExecuteJointMove.action` 字段 | 仅 `float64[6] target_rad` 单点目标，**无法承载多点+时间戳** |
| `src/mtc_interfaces/package.xml:16-18` | 无 `trajectory_msgs` / `control_msgs` 依赖 |
| `src/mtc_interfaces/srv/GetMotionCapabilities.srv:8` | 已声明 `timed_trajectory_supported` 字段，但**没有任何接口能承载该能力** |

**结论**：缺口**存在**。`GetMotionCapabilities` 声明了一个无法兑现的能力位，
MoveIt 2 输出的 `trajectory_msgs/JointTrajectory` 在当前接口下无处安放。

### 3.2 设计原则

1. **一条轨迹 = 一个 Goal**：绝不把 N 个路点拆成 N 次 `ExecuteJointMove` 调用。
   第一条已落地的约束见
   `mtc_core_cpp::TimedTrajectory` + `MotionAdapter::execute_trajectory()` 的
   **显式拒绝默认实现**（测试 `TrajectoryIsRejectedAndNotSplitIntoJointMoves`
   断言 `call_log().empty()`）。
2. **不破坏现有接口**：现有 3 action / 6 msg / 3 srv 的字段与常量**逐字节不变**，
   只做**新增**。
3. **能力如实声明**：`timed_trajectory_supported` 只有在后端实测启用后才置 true；
   Mock 后端默认 false（已在第一阶段实现）。
4. **保留 TF 信息**：轨迹携带 `frame_id`。

### 3.3 提案：新增 `ExecuteTrajectory.action`

> **状态**：待审计批准。**本阶段不实现**，不改动 `mtc_interfaces`。

```
# ExecuteTrajectory.action —— 多点时间参数化关节轨迹（提案 v1，待审计批准）
# 提供方：mtc_motion_execution（新增 action server）
# 使用方：mtc_manipulation（搬运/接合/撤离段）、外部规划器
#
# 语义：一次 Goal 承载**一条完整的时间参数化关节轨迹**。
#       禁止把同一轨迹拆成多次 ExecuteJointMove 调用来伪装轨迹控制。
#       后端不支持时必须明确拒绝（EXECUTION_REJECTED=210），不得降级执行。

# --- Goal ---
string command_id
trajectory_msgs/JointTrajectory trajectory   # joint_names + points[]（每点含 time_from_start）
string frame_id                              # 轨迹参考坐标系；空字符串非法
float32 velocity_scaling                     # 相对已审核上限的缩放，0 < x <= 1
float32 acceleration_scaling                 # 同上
uint32 timeout_ms                            # 必须 > 0，且应 >= 轨迹总时长
bool allow_partial_execution                 # 恒为 false；保留字段以便未来审计（见下）
---
# --- Result ---
bool success
bool stop_confirmed          # 仅在后端确认静止后为 true
float64 final_error_rad      # 末点最大关节误差
uint16 error_code
string message
---
# --- Feedback ---
trajectory_msgs/JointTrajectoryPoint current_point  # 含 time_from_start 的当前路点
float32 progress
string execution_state
bool tracking_error_exceeded
```

### 3.4 设计要点说明

| 设计点 | 理由 |
|---|---|
| 直接使用 `trajectory_msgs/JointTrajectory` | 与 MoveIt 2 / `FollowJointTrajectory` 零转换，避免"翻译层引入的语义漂移" |
| `frame_id` 独立字段 | `JointTrajectory.header.frame_id` 存在但常被忽略；独立字段 + 校验可强制保留 TF 语义 |
| `stop_confirmed` 与 `success` 分离 | 延续 `ExecuteJointMove.action` 的既有语义：取消不等于已停稳 |
| `final_error_rad` | 到位判定的**可量化**证据（当前 `ExecuteJointMove` 只给 `final_position_rad`） |
| `tracking_error_exceeded` | 轨迹跟踪超差是独立于"超时/取消"的失败类别，需要单独可观测 |
| `allow_partial_execution` 恒 false | 显式占位：防止未来有人用"部分执行"绕过完整性要求；审计应确认该字段是否有必要保留 |
| 不新增 `FollowJointTrajectory` 转发 | `control_msgs/FollowJointTrajectory` 是**控制器**接口；本层是**执行适配层**，两者职责不同，不应合并 |

### 3.5 依赖与兼容性影响

| 项 | 变更 | 兼容性 |
|---|---|---|
| `mtc_interfaces/package.xml` | 新增 `<depend>trajectory_msgs</depend>` | **向后兼容**（仅新增依赖） |
| `mtc_interfaces/CMakeLists.txt` | `rosidl_generate_interfaces` 的 action 列表追加 `action/ExecuteTrajectory.action` | **向后兼容**（仅新增） |
| 现有 3 action / 6 msg / 3 srv | **不变** | **无影响** |
| `GetMotionCapabilities.srv` | 不变；其 `timed_trajectory_supported` 从此有对应能力 | **语义兑现** |
| C++ 侧 | `mtc_core_cpp::TimedTrajectory` **已经就绪**（第一阶段交付），新增 ROS 转换函数即可 | **无破坏** |

### 3.6 若审计**否决**新增接口的替代方案

若审计认为不应新增 action，则必须满足以下**下界**，否则 MoveIt 2 集成不可行：

1. 在 `mtc_motion_execution` 内部实现 `FollowJointTrajectory` action client，
   **不对外暴露**新接口；
2. `GetMotionCapabilities.timed_trajectory_supported` 必须如实为 `false`
   （因为没有可用的对外通路）；
3. 内部 client 必须仍以"一条轨迹一个 goal"为单位，
   **不得**退化为逐点 `ExecuteJointMove`；
4. 该内部路径必须被 Mock 集成测试覆盖（断言"单次轨迹提交"）。

> 本次**倾向方案 3.3**（新增 action）：
> 内部隐藏通路会让"能力声明"与"实际可得能力"再次脱节，
> 与 `GetMotionCapabilities.srv` 的设计意图相悖。

---

## 4. 工作分解（Work Breakdown）

### P2-1 ROS 2 Action 外壳

**新增包**（建议命名，待定）：`mtc_task_ros`、`mtc_manipulation_ros`、`mtc_motion_execution_ros`
—— 或直接在现有 `_cpp` 包内以 `BUILD_ROS_SHELL` 选项引入。
**推荐独立包**，理由：保持"核心零 ROS 依赖"这一已被测试固化的性质。

| 步骤 | 内容 | 关键约束 |
|---|---|---|
| P2-1.1 | `ExecuteTask` Action Server | `goal_callback` → `TaskFsm::submit_goal`；`goal_rejected` 时 **reject** 而非 accept 后失败 |
| P2-1.2 | 并发 Goal 防护 | reject 依据 `TaskFsm::has_active_task()`；**不得排队** |
| P2-1.3 | `cancel_callback` | 触发 `request_cancel()`；**不得立即返回 canceled**，必须等 FSM 进入 `CANCELLED` |
| P2-1.4 | Result 映射 | `build_outcome()` → `ExecuteTask.Result`；`FAULT` 时 result 必须含具体 `error_code` |
| P2-1.5 | Feedback 周期发布 | `snapshot()` + 子状态 → `feedback.substate` |
| P2-1.6 | 看门狗驱动 | 定时器调用 `check_timeouts()` 与 `check_task_deadline()` |
| P2-1.7 | `PickPlace` Action Server | 同上，映射 `PickPlaceFsm` |
| P2-1.8 | `ExecuteJointMove` Action Server | 映射 `MotionExecutorCore`；`feedback.execution_state` 取 `ExecState` 字符串 |
| P2-1.9 | `RequestStop` Service | 映射 `MotionExecutorCore::request_stop`；响应必须区分 `request_delivered` 与 `stop_confirmed` |
| P2-1.10 | `GetMotionCapabilities` Service | 如实转发 `MotionAdapter::capabilities()` |

**必须修复的既有缺陷**（接口审计发现）：

| 缺陷 | 证据 | 第二阶段要求 |
|---|---|---|
| 取消链路断裂 | `motion_executor_node.py:124-126` 的 `cancel_callback` 只返回 ACCEPT；`_on_execute`（128-171）全程不检查 `goal_handle.is_cancel_requested`、从不调用 `core.request_stop()` | `ExecuteJointMove` 的 `cancel_callback` **必须**调用 `core.request_stop()`；执行循环**必须**检查 `is_cancel_requested`；`CANCEL_NOT_CONFIRMED=520` 必须可达 |
| `ErrorCodes` 形同虚设 | Python 侧 5 处镜像常量；`motion_executor_node.py:24` 导入后未使用 | 新外壳必须统一引用 `mtc_core_cpp::ErrorCodes`，不得再镜像 |

### P2-2 MoveIt 2 C++ 集成

| 步骤 | 内容 | 对应端口 |
|---|---|---|
| P2-2.1 | `MoveGroupInterface` 封装：`plan()` → `PlanResult` | `PlannerPort` |
| P2-2.2 | 规划失败必须映射 `PLANNING_FAILED=200`，**不得**静默重试 | `PlannerPort` |
| P2-2.3 | `ExecuteJointMove` → `FollowJointTrajectory` action client | `MotionPort` |
| P2-2.4 | 到位判定：位置误差 + 持续静止 + 队列清空（三者同时满足） | `MotionPort` |
| P2-2.5 | 取消：`cancelGoal()` 后必须**独立确认** `stop_confirmed`，否则返回 520 | `MotionPort` |
| P2-2.6 | 反馈断流 → `status_unknown=true`（触发 FSM 锁定重发） | `MotionPort` |
| P2-2.7 | 轨迹执行：`FollowJointTrajectory` 直接提交**整条** `JointTrajectory` | `MotionAdapter::execute_trajectory` |
| P2-2.8 | `capabilities()` 必须**实测**后声明（不得因方法存在即置 true） | `MotionAdapter` |

**注意**：MoveIt 2 的 `PlanningScene`/`MoveGroupInterface` 需要 `moveit_core`、`moveit_ros_planning_interface`
依赖；这些**只应出现在 ROS 外壳包**，不得污染 `mtc_core_cpp` / `mtc_task_cpp` /
`mtc_manipulation_cpp`（否则第一阶段"零 ROS 依赖"的性质被破坏，且 GTest 不再能独立运行）。

### P2-3 轨迹接口落地

1. 按 §3.3 提案新增 `ExecuteTrajectory.action`（**需审计批准**）；
2. 实现 `mtc_interfaces` ↔ `mtc_core_cpp::TimedTrajectory` 的双向转换函数
   （含 `frame_id`、`time_from_start`、速度/加速度可选字段）；
3. `MotionAdapter::execute_trajectory` 的 ROS 实现：**整条提交**给
   `FollowJointTrajectory` client；
4. **回归断言**：Mock 集成测试必须断言"轨迹只被提交一次"，
   且 `ExecuteJointMove` 的调用次数为 0（沿用第一阶段的
   `TrajectoryIsRejectedAndNotSplitIntoJointMoves` 断言思路）。

### P2-4 修复状态话题冲突（接口审计发现）

| 问题 | 证据 | 后果 | 建议 |
|---|---|---|---|
| 两个节点同时发布 `/mtc/robot/state` | `motion_executor_node.py:33/105` 与 `bridge_node.py:26/75` | 安全监听的输入被污染 | 拆分话题或增加 `source` 字段（**需审计决定**，属接口变更） |
| `bridge_node.py:129` 硬编码 `motion_active=False` | 同上 | 安全判定失真 | 必须改为真实状态或删除该发布者 |

> 这两项属于**接口/架构变更**，不在第一阶段范围，且**不应**在未经审计时擅自修改。

### P2-5 Mock 集成测试

| 层级 | 内容 | 断言要点 |
|---|---|---|
| I1 | 单点：FSM → `ExecuteJointMove` → ros2_control → `mock_components` | 0.05 rad 轨迹到位；`stop_confirmed=true` |
| I2 | 多点：`ExecuteTrajectory` → `FollowJointTrajectory` | **单次**轨迹提交；路点时间戳保持 |
| I3 | 取消（后端确认） | `CANCELLED`；`stop_confirmed=true`；`CANCEL_PENDING` 出现在历史中 |
| I4 | 取消（后端未确认） | `FAULT(520)`；**绝不**出现 `CANCELLED` |
| I5 | 并发 Goal | 第二个 goal 被 **reject**；第一个继续；任务不被切换 |
| I6 | 反馈断流 | `MOTION_STATUS_UNKNOWN=230`；进入 FAULT 后运动调用次数**不再增长** |
| I7 | 超时 | `EXECUTE_ITEM` 超时 → `FAULT(220)`；`MOTION_STATUS_UNKNOWN` 情形下 `motion_resend_allowed=false` |
| I8 | 完整抓放（Mock 感知/工具/规划） | 状态序列严格等于 `pipeline_states()`；`placement_verified` 仅在 `VERIFY_PLACED` 后为 true |
| I9 | V1/V2 切换 | V1 路径 `unlock` 调用次数 = 0；V2 解锁仅在落座+卸载确认后发生 |

### P2-6 端到端验收（Mock）

| 步骤 | 内容 | 判定 |
|---|---|---|
| E1 | 启动 `mock_components/GenericSystem`（**复用官方驱动已验证配置，只读**） | 控制器加载成功 |
| E2 | 200 Hz `JointTrajectoryController` | 控制器处于 active |
| E3 | 单点 0.05 rad Plan→Execute | 到位误差在容差内 |
| E4 | 多点轨迹 Plan→Execute | 时间戳保持；**未**拆分调用 |
| E5 | 取消 + 停稳确认 | `stop_confirmed` 由后端独立给出 |
| E6 | 全链路 `ExecuteTask`（MODE_BASIC / MODE_SEQUENCE） | 与 Python 基线状态序列一致 |

---

## 5. 接口变更申请清单（需审计批准）

| # | 变更 | 类型 | 影响面 | 提案位置 |
|---|---|---|---|---|
| C1 | 新增 `ExecuteTrajectory.action` | **新增**（向后兼容） | `mtc_interfaces` + 新 action server | §3.3 |
| C2 | `mtc_interfaces` 新增 `trajectory_msgs` 依赖 | **新增**（向后兼容） | 构建依赖 | §3.5 |
| C3 | `/mtc/robot/state` 双发布冲突的处置（拆话题或加 `source` 字段） | **可能破坏兼容** | 安全监听、UI | §P2-4，**需先设计后批准** |
| C4 | `ErrorCodes.msg` 是否需要 `EXECUTION_REJECTED` 之外的轨迹专用码 | 待评估 | 接口 | 建议**暂不新增**：现有 `210/220/230/520` 足够 |

> **未经批准不得实施 C1–C4。** 第一阶段已确保：即使不新增任何接口，
> C++ 核心也能通过 `TimedTrajectory` + "显式拒绝"如实表达能力边界。

---

## 6. 风险清单

| # | 风险 | 等级 | 缓解 |
|---|---|---|---|
| R1 | 差异 D-2（超时返回值与内部状态不一致）被外壳误读为"已转移" | **高** | P3 审计先定契约；外壳**只信 `fsm->state()`**，不信 `StepResult.state` 作为状态转移依据 |
| R2 | 差异 D-1（`timeout_ms` 无 null）导致 Action 默认值 0 被判非法 | 中 | 外壳必须提供有效默认并校验；已记录 |
| R3 | 差异 D-6（6D 保真）使真实感知不填 `frame_id` 时拒绝运动 | 中 | 第二阶段感知接入时必须填 `frame_id`；这是**有意的保守行为** |
| R4 | MoveIt 依赖污染核心包，破坏"零 ROS 依赖"性质 | **高** | 强约束：MoveIt/rclcpp 只出现在 ROS 外壳包；CI 用 grep 断言核心包源码无 `rclcpp` |
| R5 | 轨迹被悄悄拆分为多次单点调用 | **高** | `execute_trajectory` 默认显式拒绝 + Mock 断言 `call_log().empty()` |
| R6 | 取消后未确认停稳却被宣告安全停止 | **高** | `stop_confirmed` 与 `request_delivered` 严格分离；I4 断言 |
| R7 | 实机误连 | **极高** | 继续只用 `mock_components`；`DisabledMotionAdapter` 为默认回退；`make_adapter` 未知名称一律 disabled |
| R8 | 官方驱动工作空间被误改 | 中 | 保持只读；补丁已归档，可反向校验 |
| R9 | `mtc_interfaces` xmllint 既有 FAIL 掩盖新增问题 | 低 | 报告已区分；建议单独修复，避免与新变更混淆 |

---

## 7. 验收标准（第二阶段完成判定）

| # | 标准 | 判定方式 |
|---|---|---|
| A1 | 第一阶段 257 个 C++ 用例仍全部 PASS（无回归） | `colcon test` |
| A2 | 新增 ROS 外壳测试全部 PASS，且**实机相关项为 NOT RUN** | `colcon test-result --all` |
| A3 | Mock 集成 I1–I9 全部 PASS | 端到端测试日志 |
| A4 | 轨迹能力：断言"单次提交、0 次单点拆分" | 测试断言 |
| A5 | 取消链路：`520` 可达且"未确认不得 CANCELLED" | 测试断言 |
| A6 | 核心包仍零 `rclcpp` 依赖 | `grep -rn "include <rclcpp" src/mtc_core_cpp src/mtc_task_cpp src/mtc_manipulation_cpp` 为空 |
| A7 | 报告明确区分 PASS / FAIL / NOT RUN，无未执行项被声称为通过 | 报告审查 |
| A8 | 未修改既有 Python 实现与官方驱动 | `git status --short` 无 `M`/`D` |

---

## 8. 建议执行顺序

```
P3 审计（阻塞）→ 契约冻结（D-1/D-2/D-6 决议）
  → P2-1.1–P2-1.6（Task Action 外壳 + 测试）
  → P2-1.7–P2-1.10（PickPlace / ExecuteJointMove / 服务）
  → P2-2.1–P2-2.6（MoveIt 单点集成）+ I1/I3/I4/I5/I6/I7
  → C1/C2 获批 → P2-3 轨迹接口 + P2-2.7 + I2
  → P2-5 完整 Mock 集成 I8/I9
  → P2-6 端到端 E1–E6
  → 报告与提交 → 审计
```

**任何一步未通过测试时不得进入下一步**，且**不得**把未执行的步骤标注为通过。
