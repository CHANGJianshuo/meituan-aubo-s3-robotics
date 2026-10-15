# mtc_aubo_bridge

ROS 2 与 **AUBO S3 SDK**（`pyaubo-sdk` / RPC / RTDE）之间的适配层。

> ## ⚠️ 安全默认值：**disabled**
> 本包默认**不连接控制柜、不登录 SDK、不发送任何运动命令、不触发任何电磁 IO**。
> 启用真实通路需要**现场明确授权**并完成实测验证。参见第 5 节。

---

## 1. 为什么需要这一层

交接记录中的实机链路是 `RPC → moveJoint/stopJoint`，`RTDE → q/qd/target_q/target_qd`。该链路证明了**目标运动与反馈可用**，但**不能推出**支持完整的 ROS 2 时间参数化轨迹。

同时存在运行环境冲突：

| 侧 | Python | 说明 |
|---|---|---|
| 旧 SDK | 3.10 | `pyaubo-sdk 0.24.1` |
| ROS 2 Jazzy 系统 Python | 3.12 | 通常 |

**不能默认在同一进程混装 SDK 与 `rclpy`。** 设计文档 §6.2 给出两条路线：

- **路线 A**：使用经实测验证的原生 ROS 2 驱动；
- **路线 B（本包形态）**：把 Python 3.10 SDK 封装为**独立 Worker 进程**，ROS 2 侧通过本机 IPC（Unix Domain Socket）通信。

本包只落地**契约与安全语义**，不实现真实 SDK 调用。

## 2. 已核实 vs 未核实的能力

| 能力 | 当前证据 | 本包声明 |
|---|---|---|
| 只读连接、身份/安全状态读取 | 有历史记录 | 需实测后由 Worker 声明 |
| 单组六关节目标移动 | 仅 **J6** 历史实测 | 需逐关节验证后才声明 `joint_goal_supported` |
| RTDE 运动反馈订阅 | 有历史记录 | 需重新标定时效/容差 |
| 完整时间参数化轨迹跟踪 | **未验证** | `timed_trajectory_supported=false` |
| 受约束笛卡尔进环/退钩 | **未验证** | `cartesian_motion_supported=false` |
| V2 末端 I/O 与锁止反馈 | **未验证** | `tool_io_supported=false` |

> **能力声明规则：** 不得仅因 SDK 文档声称存在某方法就置 `true`。

## 3. 安全语义（均有单元测试覆盖）

| 语义 | 实现 |
|---|---|
| 身份白名单 | 未配置经核实的期望序列号时，`identity_check()` **一律不通过**（安全默认） |
| 健康度 | RPC / RTDE 任一不可用，或反馈年龄超过新鲜度上限 → 拒绝运动 |
| 命令 ID 唯一 | 重复 `command_id` 被拒绝，防止同一条命令被执行两次 |
| 单在途命令 | Bridge 与 Worker 双侧把关，并发请求被拒 |
| 状态未知锁定 | 一旦反馈断流导致状态未知，**锁定所有新的运动请求** |
| 停止 ≠ 解除锁定 | `stop_confirmed=true` **不足以**自动清除锁定；必须 `state_reconciled=true`（Worker 重新取得有效反馈并确认执行队列为空） |
| 人工出口 | 只有 `clear_unknown_after_manual_confirmation(note)` 能在未对账时解除锁定，且写入审计记录 |
| 解锁前置条件 | `send_tool_unlock_pulse()` 在 Bridge 层**再次校验**「已确认落座」且「载荷已卸除」，否则拒绝且不调用 IO |
| `accepted` ≠ 已解锁 | 解锁脉冲受理只表示请求送达，**不代表机构已解锁、电池已释放** |
| 审计轨迹 | `command_intent` / `sdk_response` / `stop_requested` / `stop_confirmed` / `result` / `unlock_*` / `manual_confirmation`，均带可关联 ID |

## 4. 离线运行

```bash
# 单元测试（25 项，不需要 ROS 2 环境，不连接任何设备）
cd /home/robot/meituan_challenge_ws/src/mtc_aubo_bridge
python3 test/test_bridge_core.py
```

节点入口（需要 ROS 2 与 `mtc_interfaces` 已构建）：

```bash
ros2 run mtc_aubo_bridge aubo_bridge --ros-args -p mode:=disabled
```

`mode` 取值：`disabled`（默认，一切拒绝）/ `fake`（离线测试用内存 Worker）。**任何其他取值都会落到 `disabled`**，不会因拼写或扩展而启用真实通路。

## 5. 启用真实通路的必要条件（**未执行**）

以下每一项都需要用户单独明确授权，并在现场完成：

1. 现场确认设备身份（序列号）并写入白名单；
2. 确认 ARCS / SDK / 接口版本与交接记录一致，或记录差异；
3. 用**小范围、低风险、受监护**的方式逐关节验证目标运动能力（不得照搬 J6 历史数值）；
4. 验证 RTDE 反馈时效与断线行为，标定新鲜度上限与停稳判据；
5. 验证停止请求的实际效果（**软件停止不等于实体急停**）；
6. 单独验证 V2 电磁 I/O 通道（电压/电流/脉冲时长/解锁反馈/断电保持）；
7. 完成工具 TCP、质量、重心、碰撞几何标定后，才允许讨论抓放动作。

> 远程进程结束、拔网线或 `Ctrl+C` **都不等价于实体急停**。

## 6. 验证状态

| 项 | 状态 |
|---|---|
| 离线单元测试（25 项） | **PASS**（Python 3.12.3，2026-10-09） |
| ROS 2 Jazzy `colcon build` | **NOT RUN**（本机无 Jazzy） |
| 真实 SDK 连接 / 运动 / IO | **NOT RUN / 禁止** |
