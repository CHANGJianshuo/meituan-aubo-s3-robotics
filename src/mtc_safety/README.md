# mtc_safety —— 软件安全联锁与故障监控

> 状态：**Mock / 未编译验证**。本包在本机**未执行 `colcon build`**，也**未在真机/仿真
> 上运行过**；rclpy 外壳（`safety_supervisor.py`）仅通过 `python3 -m py_compile` 语法检查。
> 已实际验证的部分是**纯 Python 联锁核心**（`interlocks.py`）与 70 个离线单元测试。

## 1. 分层与文件

| 文件 | 职责 | 是否依赖 rclpy |
|---|---|---|
| `mtc_safety/interlocks.py` | 联锁判定、载荷状态机、FAULT 锁存、看门狗核心 | **否**（纯 Python 3） |
| `mtc_safety/safety_supervisor.py` | 订阅/发布/服务调用的 rclpy 外壳，入口点 `safety_supervisor` | 是 |
| `test/test_interlocks.py` | 70 个离线单元测试（覆盖 §7.2 全部联锁） | 否 |

离线运行（无需 ROS，`ROS_DISTRO=humble` 也不影响）：

```bash
cd /home/robot/meituan_challenge_ws/src/mtc_safety
python3 -m pytest test/ -v            # 首选
python3 -m unittest discover -s test -t .   # 无 pytest 时的备用方式
python3 -m py_compile mtc_safety/interlocks.py mtc_safety/safety_supervisor.py
```

编译后运行节点（**本机未执行，仅作参考**）：

```bash
ros2 run mtc_safety safety_supervisor
```

## 2. 联锁表（条件 → 禁止行为 → 错误码）

代码位置：`InterlockGuard`；每条守卫返回 `(allowed, error_code, reason)`。

| 当前条件 | 禁止行为 | 错误码 |
|---|---|---|
| 设备非 READY / 通信断开 / 状态过期 / 控制器模式或真机-仿真模式不符 | 任何实机运动 | `SAFETY_INTERLOCK=500` |
| 运动状态未知 | 任何实机运动 | `MOTION_STATUS_UNKNOWN=230` |
| 未验证挂载（或载荷非 carrying） | 进入正常运输阶段 | `ATTACH_NOT_VERIFIED=310` |
| 电池尚未安全落座 | V2 电磁解锁脉冲 | `SEAT_NOT_VERIFIED=400` |
| 载荷状态未知 / 运动状态未知 / 运动中 / 载荷非 carrying | 解锁脉冲 | `SAFETY_INTERLOCK=500` |
| 解锁/脱离状态不确定 | 水平硬拉、直接上抬、撤离 | `RELEASE_NOT_VERIFIED=420` |
| 运动状态未知 | 撤离动作 | `MOTION_STATUS_UNKNOWN=230` |
| 正在执行运动（在途命令或设备 MOVING） | 第二个运动 Goal / 抢占 | `EXECUTION_REJECTED=210` |
| 取消或断线后未确认停稳 | 返回成功、启动下一目标、自动 Home | `CANCEL_NOT_CONFIRMED=520` |
| 载荷状态未知 | 进入下一任务项、无条件 Home | `SAFETY_INTERLOCK=500` |
| 仍携带电池 | 无条件 Home、进入下一任务项 | `SAFETY_INTERLOCK=500` |
| 运动状态未知 | 启动下一任务项 | `MOTION_STATUS_UNKNOWN=230` |
| 运动状态未知，或上一条指令**可能已执行运动** | 自动重发原目标（**最关键**） | `MOTION_STATUS_UNKNOWN=230` |
| 缺少「上次指令从未开始运动」的执行层证据 | 重发 | `EXECUTION_REJECTED=210` |
| FAULT 已锁存 / 设备 FAULT | 以上全部 | `SAFETY_INTERLOCK=500` |

**错误码选择约定（写死，勿随意改）**

- 「未落座」统一使用 `SEAT_NOT_VERIFIED=400`，不使用 500；载荷未知等解锁前置条件不满足用 500。
- 「运动状态未知」在 `allow_motion` / `allow_resend_motion` / `allow_force_withdraw` /
  `allow_second_motion_goal` / `allow_next_task_item` 中统一为 `230`。
- `allow_resend_motion` 的 230 判定**优先于** FAULT 锁存与其他一切标志：即使人为伪造
  「已停稳 / 无未知」标志，只要 `motion_state_unknown` 为真，仍必须 230 拒绝。
- 本包额外提供 `allow_unconditional_home()`（§6.4 的「禁止无条件 Home」），非 §7.2 原文。

## 3. 设备状态图（§3.3）

```
                 ┌──────────────┐
  启动 ─────────▶│ DISCONNECTED │◀──────────── 通信断开（link_lost）
                 └──────┬───────┘
                        │ communication_ok
                        ▼
   ┌────────────┐  ready  ┌────────┐  motion_active  ┌────────┐
   │ NOT_READY  │◀───────▶│ READY  │────────────────▶│ MOVING │
   └────────────┘         └────────┘                 └───┬────┘
        ▲                     ▲                         │ 请求停止
        │                     │ 已停稳且新状态到达        ▼
        │                ┌────┴─────┐             ┌────────────────┐
        │                │ STOPPED  │◀────────────│ STOP_REQUESTED │
        │                └──────────┘ stop_confirmed└───────────────┘
        │                                                 │ 安全状态不满足 / 工具 FAULT
        │                                                 ▼
        │                                            ┌────────┐
        └──────── 人工 clear_by_human + 重新准备 ────│ FAULT  │（锁存，自动路径不可清除）
                                                     └────────┘
```

由 `device_state_from_robot_state()` 纯函数推导，优先级：DISCONNECTED > FAULT >
STOP_REQUESTED > MOVING > STOPPED > READY/NOT_READY。设备状态与任务状态严格分离：
设备 `READY` 不代表 `PickPlace` 成功，设备 `STOPPED` 也不代表任务可安全重试。

## 4. 载荷状态语义

`unknown / none / carrying`（`PayloadTracker`）：

- **正向确认必须给出证据 note**（`mark_carrying(note)` / `mark_none(note)`），空 note 被拒。
- **任意状态都可以回到 `unknown`**（`mark_unknown`），这是保守方向，随时允许。
- `unknown` 时：`automatic_unlock_allowed()` / `unconditional_home_allowed()` /
  `next_task_item_allowed()` 全部为 `False` → **禁止自动解锁、禁止无条件 Home、
  禁止进入下一任务项**（§6.4）。
- 载荷由 `ToolState` 推断（`payload_from_tool_state()`）：`state` 是估计，`verified` 是
  独立维度——只有 `verified=True` 且证据等级 ≥ `EVIDENCE_GEOMETRY` 的
  `ATTACHED/DETACHED` 才能给出 `carrying/none`，仅凭命令回执
  （`EVIDENCE_COMMAND_ONLY`）一律回落 `unknown`。

## 5. FAULT 人工解除流程

1. 触发锁存：停止请求送达后超时未确认停稳（520）、`ToolState=FAULT`（510）、
   `safety_normal=false` / 设备 FAULT 等。
2. **保持锁定**：`FaultLatch.try_auto_clear(origin)` 供任何自动恢复代码调用，它**永远返回
   失败**并把尝试记入 `clear_attempts`；本包不存在任何自动清除路径。
3. 现场人工核验（断电检查、目视确认无干涉、确认机械臂静止）后调用：

   ```python
   guard.fault_latch.clear_by_human('已断电检查，无机械干涉，确认可继续')
   # 或节点入口：node.clear_fault_by_human(note)（刻意不暴露为 ROS 服务）
   ```

   必须提供非空 note；成功后在 `clear_history` 记录 note、时间戳与被清除的故障详情。
4. 解除锁存**不等于**恢复运行：仍需重新执行设备准备检查（模式/仿真切换后必须重做），
   运动状态未知需另行 `clear_unknown_after_manual_confirmation(note)`，且重发依旧要求
   `stop_confirmed` 证据。

## 6. 与 `mtc_motion_execution` 的职责边界

| 事项 | mtc_safety | mtc_motion_execution |
|---|---|---|
| 联锁判定（能否运动/运输/解锁/撤离/重发） | ✅ 唯一实现 | 不重复实现 |
| 七步握手、能力校验、到位确认 | ❌ | ✅ `executor_core.MotionExecutorCore` |
| `motion_state_unknown` / `stop_confirmed` 的产生 | 只**消费**（由执行层/`RobotState` 提供） | ✅ 产生与维护 |
| 停止 | 只发**请求** `/mtc/motion/request_stop` | 真实停止与停稳确认 |
| FAULT 锁存 | ✅ | 执行层故障上报给安全层锁存 |

保持一致：安全层沿用执行层的 `stop_confirmed` / `motion_state_unknown` 语义，
`motion_state_unknown=True` 时**禁止自动重发**；只有执行层明确回报「指令在开始运动前
即被拒绝」才允许重发（`note_command_rejected_before_motion`）。

## 7. 安全声明（务必遵守）

- **ROS 软件停止请求不替代实体急停**。`RequestStop.request_delivered=true` 只表示请求
  送达执行适配层；`stop_confirmed` 必须由执行层独立确认（本包只采信
  `RobotState.stop_confirmed` 与工具状态等证据）。
- 节点崩溃、拔线、`Ctrl+C`、网络失效都**不等价于**实体急停，也不能保证机械臂停下。
- 本包只做软件联锁；电气急停回路、工具 DO 通道与保护电路属硬件范畴，待电气实测。

## 8. rclpy 外壳要点

- 订阅 `/mtc/robot/state`（`RobotState`）、`/mtc/tool/state`（`ToolState`，RELIABLE /
  KEEP_LAST(10)）。
- 发布 `/mtc/safety/interlock_state`（复用 `RobotState`，`detail` 为 JSON：设备状态、载荷
  状态、FAULT 锁存、`stop_requested` / `stop_confirmed` / `stop_delivered`、所用超时参数）。
- 超时/失联/状态未知 → 调用 `/mtc/motion/request_stop`（`RequestStop`）。
  节点日志逐条包含 `stop_requested`、`stop_confirmed` 与全部超时参数
  （`robot_state_timeout_sec=0.5`、`tool_state_timeout_sec=1.0`、
  `stop_confirm_timeout_sec=2.0`，均可经 ROS 参数覆盖）。
- **停止确认超时从第一次停止请求起算**，后续重发不刷新该时刻；超时升级为
  `CANCEL_NOT_CONFIRMED=520` 故障锁存。
- 支持注入 fake client：`SafetySupervisor(stop_client=<fake>)`，适配逻辑在纯 Python 的
  `adapt_stop_client()`（支持 `callable`、带 `call_async` 的对象、带 `call` /
  `request_stop` / `stop` 的同步 fake），离线集成测试无需真实 ROS 服务。
