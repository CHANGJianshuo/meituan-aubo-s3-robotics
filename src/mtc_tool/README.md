# mtc_tool —— V1 舌规 / V2 电磁锁止末端策略统一管理

> 本包是**工具层的权威实现**（设计文档 §5.1 逻辑契约的落地）。并行开发的
> `src/mtc_manipulation/` 若也定义工具策略接口，应以本包的 `ToolStrategy` /
> `MotionPrimitive` 为准；本包不修改任何其它功能包。
>
> **状态：未编译、未上机验证。** 本仓库刻意不执行 `colcon build`（本机仅有 ROS 2
> Humble，目标基线为 Jazzy），全部结论来自纯 Python 离线单元测试 + `py_compile`。
> 所有几何/速度数值均为**未标定示意占位值**，不得用于驱动真实机械臂。

## 1. 文件清单与分层

| 文件 | 职责 | 是否依赖 ROS |
| --- | --- | --- |
| `mtc_tool/codes.py` | `ToolState.msg` / `ErrorCodes.msg` 的纯 Python 数值镜像 | 否 |
| `mtc_tool/strategy.py` | `ToolStrategy` 抽象 + `MotionPrimitive` + `PassiveHookV1` / `MagneticLatchV2` | 否 |
| `mtc_tool/io_port.py` | `UnlockIoPort` 抽象 + `MockUnlockIo` + `DisabledUnlockIo` | 否 |
| `mtc_tool/manager.py` | `ToolManager`：工具状态机 + 证据等级 + 解锁联锁 + 错误码 | 否 |
| `mtc_tool/tool_manager_node.py` | rclpy 外壳：`/mtc/tool/state`、`/mtc/tool/trigger_unlock` | 是（仅此文件） |
| `test/test_tool_manager.py` | 纯 Python 3 离线测试（不 import rclpy） | 否 |

**分层铁律：** 工具层只输出 `MotionPrimitive` 列表，**不产生关节角命令、不调用任何
运动/机器人 SDK、不导入 rclpy**（`tool_manager_node.py` 除外）。执行权属于
`motion_executor`（设计文档 §5.1、§9 图注）。

## 2. V1 / V2 差异矩阵（设计文档 §5.2 的动作矩阵）

| 统一阶段 | `PassiveHookV1`（`passive_hook_v1`） | `MagneticLatchV2`（`magnetic_latch_v2`） |
| --- | --- | --- |
| `PRE_ALIGN` | `side_insert_pre_align`：提环开口侧面准备位 | `top_insert_pre_align`：提环上方对中位 |
| `ENGAGE` | `side_insert`（横向穿环）+ `hook_engage`（上提挂钩） | `top_insert`（下压插入）+ `auto_lock`（触发机械锁止） |
| `TEST_LIFT` | 由上层执行试提并回填独立证据 | 同 V1，可叠加锁止开关反馈 |
| `TRANSPORT` | 维持工具姿态 | 保持机械锁止（不需要持续通电吸持） |
| `SEAT` | 上层下降至有承托并 `mark_seated()` | 同 V1 |
| `UNLOAD` | `seat_unload`：小幅调整使倒钩不承重 | `seat_unload`：小幅调整让锁扣不承重 |
| `DISENGAGE` | `side_withdraw`：沿标定反向路径退钩 | `electromagnetic_unlock`（零位移保持位姿，脉冲由 `ToolManager` 经 IO 端口发送）+ `withdraw` |
| `RETREAT` | `retreat`：净空后离开 | `retreat`：净空后离开 |
| 电磁 IO | **从不使用**（`requires_unlock_pulse() == False`，`uses_unlock_io == False`） | 需要解锁脉冲（`requires_unlock_pulse() == True`） |

设计文档 §5.2 提醒：**不得假设 V2 解锁后必定能垂直向上退出**；`MagneticLatchParams.withdraw_offset_m`
只是未标定占位值，退出方向必须由 V2 CAD 装配与实物测试确定。

## 3. 状态与证据等级语义

`ToolState.msg` 的 `state` 是**当前估计**，`verified` 是**独立维度**，二者不得混为一谈。

| `state` | 含义 | 典型 `evidence_level` |
| --- | --- | --- |
| `STATE_UNKNOWN`(0) | 未装载策略或故障已被人工清除后的初始态 | `EVIDENCE_NONE` |
| `STATE_DETACHED`(1) | 工具与电池已分离且已验证 | `EVIDENCE_GEOMETRY` / `SENSOR_OR_VISION` |
| `STATE_ENGAGING`(2) | 正在接合（计划已生成） | `EVIDENCE_COMMAND_ONLY` |
| `STATE_ATTACHED`(3) | 估计已挂载/锁止（**未必已验证**） | `COMMAND_ONLY` → `verified=False` |
| `STATE_RELEASING`(4) | 正在释放；`verified=True` 表示解锁已由独立证据确认 | `COMMAND_ONLY` → `GEOMETRY`/`SENSOR_OR_VISION` |
| `STATE_FAULT`(5) | 保守故障态，`withdraw_allowed=False`，需人工 `clear_fault(note)` | 视情况 |

| `evidence_level` | 含义 | 是否算独立证据 |
| --- | --- | --- |
| `EVIDENCE_NONE`(0) | 无证据 | 否 |
| `EVIDENCE_COMMAND_ONLY`(1) | 仅运动命令执行完毕、或仅电磁铁输出信号 | **否** |
| `EVIDENCE_GEOMETRY`(2) | 几何/机械反馈（试提随动、锁止开关） | 是（`MIN_INDEPENDENT_EVIDENCE`） |
| `EVIDENCE_SENSOR_OR_VISION`(3) | 传感器或视觉确认 | 是 |

举例：`notify_engage_complete()` 后 `state=STATE_ATTACHED` 且
`evidence_level=EVIDENCE_COMMAND_ONLY`，此时 `verified=False`，**禁止进入运输**；
必须由 `verify_attach(evidence >= EVIDENCE_GEOMETRY)` 才能置 `verified=True`。

## 4. 解锁前置条件与联锁（V2）

`ToolManager.trigger_unlock()` 必须**同时**满足：

1. `strategy.requires_unlock_pulse() == True`（V1 被要求解锁 → 直接拒绝）；
2. `state == STATE_RELEASING`（已进入释放流程）；
3. `seated_verified == True`（已确认电池由桌面承托，对应 `VERIFY_SEATED`）；
4. `unloaded == True`（载荷已卸除，倒钩/锁扣不承重，对应 `UNLOAD`）；
5. `pulse_ms` 为合法正整数。

不满足时返回 `SAFETY_INTERLOCK=500`，**且完全不调用 `UnlockIoPort`**（测试用带调用
计数的替身断言 `call_count == 0`）。

**`accepted=True` 只表示脉冲请求被受理**，绝不表示电池已释放或锁止已解除。工具状态
只有拿到独立证据（`confirm_unlock(evidence >= EVIDENCE_GEOMETRY, source=...)`）才能
进入「解锁已确认」；否则返回 `UNLOCK_FAILED=410` 并保持保守 `STATE_FAULT`、
`withdraw_allowed=False`。

## 5. 错误码选择（一致且文档化）

| 情形 | 错误码 |
| --- | --- |
| 未落座 / 未卸载 / 非 `RELEASING` / V1 被要求解锁 / 故障未清除 | `SAFETY_INTERLOCK=500` |
| 挂载证据不足却请求释放 | `ATTACH_NOT_VERIFIED=310` |
| 解锁端口拒绝 / 超时 / 抛异常 / 被禁用 | `UNLOCK_FAILED=410` |
| 脉冲已受理但无独立解锁证据、未受理就确认解锁 | `UNLOCK_FAILED=410` |
| 分离失败或结果未知、分离成功但无独立证据 | `RELEASE_NOT_VERIFIED=420` |
| 非法 `pulse_ms` 或其他非法入参 | `EXECUTION_REJECTED=210` |

`DISENGAGE` 失败或结果未知 → `STATE_FAULT` + `420` + `withdraw_allowed=False`，
**不得强行抽出**；恢复必须走 `clear_fault(operator_note)`（需人工核验说明）。

## 6. 离线运行方式（无需编译、无需 source ROS）

```bash
cd /home/robot/meituan_challenge_ws/src/mtc_tool
python3 -m pytest test/ -v
# 或无 pytest：
python3 -m unittest discover -s test -v
# 语法检查：
python3 -m py_compile mtc_tool/*.py test/*.py
```

核心模块不 import rclpy，因此本机只有 ROS 2 Humble 也能完整运行上述测试。

ROS 环境内的运行方式（**未在本机验证**，且需要 Jazzy + 已构建的 `mtc_interfaces`）：

```bash
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 run mtc_tool tool_manager --ros-args -p tool_type:=magnetic_latch_v2
ros2 service call /mtc/tool/trigger_unlock mtc_interfaces/srv/TriggerUnlock \
  "{request_id: 'demo', pulse_ms: 300}"
```

节点参数：`tool_type`（默认 `passive_hook_v1`）、`unlock_pulse_ms`(300)、
`unlock_io_backend`（默认 `disabled`，`mock` 仅在 `simulation_mode:=true` 时生效）、
`simulation_mode`(false)、`allow_real_io`(false，**被忽略**)、`state_publish_period_s`(0.2)、
`seated_verified`(false)、`unloaded`(false)。

## 7. 安全说明

- **真实电磁解锁 IO 默认禁用。** 默认使用 `DisabledUnlockIo(strict=True)`：任何调用都抛
  `UnlockIoDisabledError`，绝不触碰 GPIO/CAN/串口/网络设备；启动日志明确告警
  「真实电磁解锁 IO 已禁用」。`allow_real_io:=true` 只记录错误并被忽略。
- `MockUnlockIo` 仅用于离线测试，支持 `accept / reject / timeout / raise` 四种可配置结果。
- 本包不联网、不进行 SSH/硬件操作、不含任何凭据；数值参数均为**未标定占位值**。
- 所有测试结果只能标记为 Mock/离线结论，不能写成 AUBO S3 实机抓放成功。

## 8. 与设计文档的偏差、未实现点（如实记录）

1. **未编译验证**：本机无 Jazzy，未运行 `colcon build` / `colcon test`，也未做 ROS 图级
   验证（话题名、QoS、服务签名仅按 IDL 编写）。
2. **扩展字段**：`MotionPrimitive` 在文档 5 个字段外增加了 `stage`、`note`（供上层 FSM
   对齐阶段与日志）；`PoseTarget` 替代 `geometry_msgs/PoseStamped` 以保持纯 Python 可测。
3. **`electromagnetic_unlock` 是零位移保持原语**：策略不发送脉冲，脉冲统一由 `ToolManager`
   经 `UnlockIoPort` 发送；文档未规定该阶段的表现形式。
4. **证据来源未接线**：`mark_seated` / `mark_unloaded` / `verify_attach` / `confirm_unlock`
   目前由调用方（测试或节点参数）注入，尚未接入真实的落座检测、试提随动与锁止开关反馈；
   节点上的 `seated_verified` / `unloaded` 参数只是离线占位输入。
5. **V2 退出方向未定**：`withdraw` / `retreat` 的几何是占位值，未标定、未仿真验证。
6. **`TriggerUnlock` 服务未做请求去重/`request_id` 幂等**：重复 `request_id` 目前会被再次
   受理（每次都会调用 IO），待与上层 FSM 约定后补齐。
7. **未实现**：Gazebo/仿真联动、`RequestStop` 联动、工具 TCP/负载切换导致计划失效的校验、
   `mtc_manipulation` 的实际对接（本包只提供接口）。
