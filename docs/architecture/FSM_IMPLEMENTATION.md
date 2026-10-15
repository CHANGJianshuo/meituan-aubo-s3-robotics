# FSM_IMPLEMENTATION —— 双层状态机实现说明

> 目标架构来源：[`AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md`](./AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md) §3
> 本文记录**落地实现**（源码入口、状态名、事件、守卫、错误码），与设计文档的差异逐条登记。

---

## 0. 实现原则（V3 P4 强制项）

| 要求 | 落实方式 |
|---|---|
| 不允许用巨大的顺序 `sleep` 脚本充当状态机 | 核心为**类型明确的状态枚举 + 集中转移表 + 守卫函数**，无顺序 sleep 流程 |
| 可注入的 Fake 依赖 | `fakes.py` 提供 FakePerception / FakePlanner / FakeMotion / FakeTool / FakeClock |
| 核心逻辑与 ROS 解耦 | FSM 核心模块**不 import rclpy**，因此可在无 ROS 2 环境的机器上离线单测 |
| 每状态有进入条件、事件、取消、超时、错误码与日志 | 状态表逐项定义；`task_id` / `request_id` / `command_id` 全链路关联 |
| 视觉目标不存在、不可达、规划失败、挂载失败、运动状态未知处理各不相同 | 分别对应 `TARGET_NOT_FOUND(110)`、`PLANNING_FAILED(200)`、`ENGAGE_FAILED(300)`、`ATTACH_NOT_VERIFIED(310)`、`MOTION_STATUS_UNKNOWN(230)` |

---

## 1. 第一层：整轮 Task FSM（`src/mtc_task`）

### 1.1 状态

`INIT` → `SELF_CHECK` → `WAIT_TASK` → `VALIDATE_TASK` → `SCAN_SCENE` → `EXECUTE_ITEM` → `RECORD_RESULT` →（`SCAN_SCENE` | `FINISHED`）

异常分支：`RECOVERY` / `CANCEL_PENDING` / `TASK_FAILED` / `FAULT` / `CANCELLED`

### 1.2 关键守卫

| 守卫 | 规则 | 违反结果 |
|---|---|---|
| 任务合法性 | `MODE_BASIC` 恰好 1 色；`MODE_SEQUENCE` 恰好 3 色且互异；颜色属于 `{1,2,3,4}`；`timeout_ms > 0`；`task_id` 非空 | `TASK_FAILED`，`INVALID_TASK=100` |
| 并发 Goal | 同一时刻只允许一个进行中 Goal，第二个**明确拒绝** | 拒绝并记录原因 |
| 取消 | 收到取消先进 `CANCEL_PENDING`，仅在子任务确认停稳后才 `CANCELLED` | 超时未确认 → `FAULT`，`CANCEL_NOT_CONFIRMED=520` |
| 重试 | 仅“明确安全可重试”的感知/规划错误允许重试，默认上限 1 次 | 运动状态未知**不重试**，直接 `FAULT` |
| 顺序映射 | `MODE_SEQUENCE` 按 `ordered_colors` 次序固定映射 `P1/P2/P3` | 不写死颜色 |

### 1.3 源码入口

- 核心（无 rclpy）：`src/mtc_task/mtc_task/task_fsm.py`
- ROS 2 外壳：`src/mtc_task/mtc_task/task_executor.py`，Action `/mtc/task/execute`，Topic `/mtc/task/state`

---

## 2. 第二层：单块 PickPlace FSM（`src/mtc_manipulation`）

### 2.1 状态

`RESOLVE_TARGET` → `PLAN_ACQUIRE` → `MOVE_PRE_ALIGN` → `ENGAGE` → `VERIFY_ENGAGEMENT` → `TEST_LIFT` → `VERIFY_ATTACHED` → `LIFT` → `TRANSPORT` → `MOVE_PRE_SEAT` → `SEAT` → `VERIFY_SEATED` → `UNLOAD` → `DISENGAGE` → `RETREAT` → `VERIFY_PLACED` → `SUCCESS`

异常分支：`RECOVERY` / `FAILED` / `FAULT` / `CANCEL_PENDING` / `CANCELLED`

### 2.2 V1/V2 共用与差异

共用全部状态与转移；**仅** `ENGAGE`、`VERIFY_ENGAGEMENT`、`DISENGAGE` 内部由工具策略区分：

| 统一阶段 | V1 被动舌规 | V2 锁止 + 电磁 |
|---|---|---|
| `ENGAGE` | 横向穿环 + 上提挂钩 | 上方插入 + 触发机械锁止 |
| `VERIFY_ENGAGEMENT` | 机构姿态与几何证据 | 可叠加锁止开关反馈 |
| `DISENGAGE` | 沿标定反向路径退钩 | 解锁脉冲 → 确认解锁 → 按标定路径退出 |

### 2.3 安全语义（逐条有测试）

| 语义 | 行为 | 错误码 |
|---|---|---|
| `placement_verified` 仅由 `VERIFY_PLACED` 置位 | 其他任何状态不得为 `true` | — |
| 未验证挂载不得运输 | `VERIFY_ATTACHED` 证据不足即终止 | `ATTACH_NOT_VERIFIED=310` |
| 穿环/锁止失败 | 仅在**确认无载荷**时允许退回重试 | `ENGAGE_FAILED=300` |
| 未确认落座 | **禁止**进入 `UNLOAD`/`DISENGAGE`，V2 尤其禁止解锁 | `SEAT_NOT_VERIFIED=400` |
| 解锁失败或结果未知 | 进入 `FAULT`，**禁止**强行抽出/上抬 | `UNLOCK_FAILED=410` / `RELEASE_NOT_VERIFIED=420` |
| 运动状态未知 | `FAULT`，**禁止**自动重发 | `MOTION_STATUS_UNKNOWN=230` |
| 取消 | 先 `CANCEL_PENDING`，等 `stop_confirmed` | 未确认 → `FAULT`，`CANCEL_NOT_CONFIRMED=520` |
| 稳定性 | 目标区域内连续稳定 `placement_stability_sec`（默认 3.0 s）后才 `SUCCESS` | 否则 `PLACEMENT_FAILED=430` |

### 2.4 源码入口

- 核心（无 rclpy）：`src/mtc_manipulation/mtc_manipulation/pick_place_fsm.py`
- 端口契约与 Fake：`src/mtc_manipulation/mtc_manipulation/fakes.py`
- ROS 2 外壳：`pick_place_server.py`，Action `/mtc/manipulation/pick_place`

---

## 3. 工具策略层（`src/mtc_tool`）

统一接口 `ToolStrategy`（`strategy.py`）+ 运动原语 `MotionPrimitive`；两个实现：`PassiveHookV1`、`MagneticLatchV2`。

**统一语义：**

- 工具只产生**运动原语**，不产生关节命令、不直接调用 SDK；
- `ToolState.state`（当前估计）与 `verified`（是否被独立证据确认）是**两个独立维度**；
- 解锁必须同时满足「已确认落座」且「载荷已卸除」，否则拒绝且**不触发 IO**；
- `accepted=true` 只表示脉冲请求受理，**不代表已解锁**；
- V1 路径**零电磁 IO 依赖**（测试断言 IO 调用次数为 0）；
- 真实 IO 默认使用 `DisabledUnlockIo`，任何调用被明确拒绝。

---

## 4. 运动执行层（`src/mtc_motion_execution`）

### 4.1 执行状态

`IDLE` / `ACCEPTED` / `MOVING` / `SETTLED` / `CANCELLING` / `STOPPED` / `STOP_UNKNOWN` / `TIMEOUT` / `REJECTED` / `FAULT`

**严格区分“已接受、正在运动、已到位、取消中、停止确认、运动状态未知”**，不把 `ACCEPTED` 当作 `SETTLED`。

### 4.2 七步握手

`PREPARE → ACCEPT → VALIDATE → SEND → MONITOR → CONFIRM → STOP/FAULT`，每次执行留下可追溯记录（`handshake(command_id)`）。

### 4.3 到位判定与不确定状态

- 只有**到位 + 持续静止 + 执行队列清空**同时满足才返回 `success=true`；
- 超时、反馈断流、后端异常一律置 `motion_state_unknown=true`，并**锁住后续运动请求**；
- 该类失败**禁止自动重发**；仅“从未开始运动”的 `REJECTED` 允许立即重发；
- 人工出口 `clear_unknown_after_manual_confirmation()` 只能由现场人员调用。

---

## 5. 安全层（`src/mtc_safety`）

| 条件 | 禁止行为 | 错误码 |
|---|---|---|
| 未 `READY` / 模式不符 / 状态过期 | 任何运动 | `SAFETY_INTERLOCK=500` |
| 未验证挂载 | 进入正常运输 | `ATTACH_NOT_VERIFIED=310` |
| 电池尚未安全落座 | 发送电磁解锁脉冲 | `SEAT_NOT_VERIFIED=400` |
| 解锁/脱离状态不确定 | 水平硬拉、直接上抬、进入下一项 | `RELEASE_NOT_VERIFIED=420` |
| 正在执行运动 | 第二个运动 Goal / 其他客户端抢占 | `EXECUTION_REJECTED=210` |
| 取消或断线后未确认停稳 | 返回成功、启动下一目标、自动 Home | `CANCEL_NOT_CONFIRMED=520` |
| 运动状态未知 | 自动重发 | `MOTION_STATUS_UNKNOWN=230` |

**FAULT 为保持态**：进入后锁定后续自动动作，只有显式人工 `clear_by_human(note)` 才能解除，且记录 note 与时间戳。

**边界声明：** 安全层只发**停止请求**，真实停稳由执行层确认；**ROS 软件停止请求不替代实体急停**。

---

## 6. 设备状态与任务状态分离

设备状态：`DISCONNECTED` / `NOT_READY` / `READY` / `MOVING` / `STOP_REQUESTED` / `STOPPED` / `FAULT`（`mtc_safety.interlocks.DeviceState`）

- 设备 `READY` **不代表** `PickPlace` 成功；
- 设备 `STOPPED` **也不代表**任务安全可重试；
- 切换仿真/真实后必须**重新执行设备准备检查**。

---

## 7. 全链路 ID 关联

| ID | 产生方 | 用途 |
|---|---|---|
| `task_id` | 任务客户端 | 关联整轮任务的所有日志与子目标 |
| `request_id` | `mtc_task` → `PickPlace` | 关联单项抓放 |
| `command_id` | `mtc_manipulation` → 执行层 | 关联单次运动命令；Bridge 侧做唯一性检查 |
| `object_id` | 感知 | 目标电池身份；执行中不得静默切换 |

---

## 8. 与设计文档的差异登记

| 差异 | 内容 | 依据 |
|---|---|---|
| D1 | `PlanMotion.action` / `Latch.srv` 未纳入（源缺失，不重定义） | `INTERFACE_CHANGELOG.md` §3 |
| D2 | 停止服务绑定为 `/mtc/motion/request_stop`（V0.1 写作 `/mtc/safety/stop_motion`） | `INTERFACE_CHANGELOG.md` §4.1 |
| D3 | 后端仅声明关节目标能力；`ExecuteTrajectory` 未实现 | V0.1 §4.2/§4.8 阶段二 |
| D4 | 感知节点（`mtc_perception`）未实现，FSM 通过注入端口消费感知结果 | 本次范围与源缺失 |
| D5 | 工具策略的具体几何参数为**未标定占位值** | `TF_CONVENTIONS.md` §3 |
