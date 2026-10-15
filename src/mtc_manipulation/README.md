# mtc_manipulation —— 单块电池抓放状态机（第二层 PickPlace FSM）

> **状态：Mock / 离线可测、未编译验证。**
> 本包面向 ROS 2 Jazzy + `mtc_interfaces`，但开发本机只有 ROS 2 Humble、且本次任务禁止
> `colcon build`，因此：**核心逻辑与测试已在纯 Python 3 下真实运行通过；ROS 侧字段以源码审阅为准，
> 未做任何编译、DDS 或实机验证。** 不得据此声称已接通 AUBO S3 实机。

设计依据：`docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md`
§3.2（PickPlace 状态表）、§5（工具策略与几何）、§6.4（停止与不确定状态）、§7（故障与联锁）、§10.1（必测故障用例）。

---

## 1. 模块边界（硬约束）

| 文件 | 是否导入 rclpy | 职责 |
| --- | --- | --- |
| `pick_place_fsm.py` | **否** | FSM 核心：状态、集中转移表、守卫、每状态超时、取消、安全语义 |
| `tool_strategy.py` | **否** | V1/V2 工具策略与运动原语（`MotionPrimitive`），不掌控关节控制权 |
| `fakes.py` | **否** | Fake 端口：可模拟到位、超时、状态未知、取消已/未确认停稳 |
| `pick_place_server.py` | **是（唯一）** | rclpy Action Server 外壳 + ROS 适配器 |
| `test/*` | **否** | 纯 Python 3 单元测试（pytest / unittest 均可） |

依赖注入端口：`PerceptionPort` / `PlannerPort` / `MotionPort` / `ToolPort` / `ClockPort`。
所有安全判决都在 FSM 内，ROS 适配器只做协议翻译。

---

## 2. 状态图（设计 §3.2）

```
RESOLVE_TARGET → PLAN_ACQUIRE → MOVE_PRE_ALIGN → ENGAGE → VERIFY_ENGAGEMENT
      → TEST_LIFT → VERIFY_ATTACHED → LIFT → TRANSPORT → MOVE_PRE_SEAT → SEAT
      → VERIFY_SEATED → UNLOAD → DISENGAGE → RETREAT → VERIFY_PLACED → SUCCESS

可恢复分支：RESOLVE_TARGET / PLAN_ACQUIRE / MOVE_PRE_ALIGN / ENGAGE / VERIFY_ENGAGEMENT
            → RECOVERY →（仅当确认无载荷且运动状态已知）→ RESOLVE_TARGET，否则 FAILED
危险分支  ：VERIFY_SEATED / DISENGAGE / RETREAT / 运动状态未知 → FAULT
取消分支  ：任意非终态 → CANCEL_PENDING →（MotionPort.stop_confirmed）→ CANCELLED
                                         └（未确认）→ FAULT(CANCEL_NOT_CONFIRMED=520)
```

集中转移表同时以代码常量 `TRANSITION_TABLE` 形式给出（`pick_place_fsm.py`），
转移实际由 `_STATE_ACTIONS` 中每个状态处理器返回的 `StepResult` 决定。

---

## 3. 端口契约

| 端口方法 | 返回 | 失败错误码 |
| --- | --- | --- |
| `PerceptionPort.get_detection(object_id, expected_color, max_age_s)` | `DetectionResult{f found, unique, stale, pose}` | 110 / 120 |
| `PlannerPort.plan(phase, target)` | `PlanResult{ok, error_code, primitives}` | 200 |
| `MotionPort.execute_joint_move(command)` | `Optional[MotionResult]`（`None`=仍在执行） | 210 / 220 / 230 |
| `MotionPort.request_stop(reason)` | `StopResult{request_delivered, stop_confirmed}` | 520 |
| `ToolPort.engage/test_lift/seat_verify/unload/unlock/disengage` | `ToolEvidence{ok, state, evidence_level, verified, seated, unloaded, released}` | 300 / 310 / 400 / 410 / 420 |
| `ToolPort.holding_state()` | `"none" / "holding" / "unknown"` | 决定是否允许自动重试 |
| `ClockPort.now()` | `float` 秒 | 测试用 `FakeClock` 加速 |

`expected_color=None` 表示放置后核验，跳过颜色校验、只看“是否在目标区域且稳定”。

---

## 4. V1 / V2 差异矩阵

| 统一阶段 | V1 `passive_hook_v1` | V2 `magnetic_latch_v2` |
| --- | --- | --- |
| 预对准（`side_insert` / `top_insert`） | 提环开口侧面进环准备位 | 提环上方对中位 |
| **ENGAGE** | `hook_engage`：横向穿环 + 上提挂钩（接触，5 mm/s） | `auto_lock`：上方下降插入触发机械锁止 |
| **VERIFY_ENGAGEMENT** | 需 `evidence_level >= EVIDENCE_GEOMETRY` | 同上；锁止反馈可用时提高证据等级 |
| 试提 / 搬运 / 落座 | 共用 `test_lift` / `lift` / `transport` / `pre_seat` / `seat_contact` | 完全共用 |
| **DISENGAGE** | `side_withdraw`，**无任何电磁/夹爪 IO** | 先 `electromagnetic_unlock` 脉冲，确认解除后再 `withdraw` |
| IO 通道 | `IO_CHANNELS = ()` | `IO_CHANNELS = ("electromagnetic_unlock",)` |
| `requires_unlock_pulse()` | `False` | `True` |
| 解锁前置条件 | 恒拒绝（无解锁机构） | 必须 `seated_verified AND unloaded_verified`，否则 400 / 420 |

**除 ENGAGE / VERIFY_ENGAGEMENT / DISENGAGE 三个阶段内部之外，主流水线与工具型号完全无关。**
V2 解锁失败或解锁结果未知时，FSM 一律 FAULT，**禁止继续强行抽出或上抬**（设计 §3.2 / §10.1）。

---

## 5. 错误码映射（`mtc_interfaces/msg/ErrorCodes.msg`）

| 码 | 来源条件 | FSM 处置 |
| --- | --- | --- |
| 100 INVALID_TASK | 目标工位不在 T0/P1/P2/P3 | FAILED（不运动） |
| 110 TARGET_NOT_FOUND | 未找到 / 颜色不符 / 不唯一 / 提环位姿非法 | RECOVERY（限次）→ FAILED |
| 120 POSE_STALE | 感知帧过期、坐标不可用 | RECOVERY（限次）→ FAILED |
| 200 PLANNING_FAILED | 规划失败 | RECOVERY（限次，未发令）→ FAILED |
| 210 EXECUTION_REJECTED | 后端拒绝目标 | FAILED（不重发） |
| 220 MOTION_TIMEOUT | 运动超时（可能仍在动） | FAULT，禁止自动重发 |
| 230 MOTION_STATUS_UNKNOWN | 反馈断流 / 结果不确定 / 状态看门狗超时 | FAULT，锁定动作、禁止重发 |
| 300 ENGAGE_FAILED | 穿环或锁止未完成 | 确认无载荷 → RECOVERY；否则 FAILED |
| 310 ATTACH_NOT_VERIFIED | 试提后缺少挂载证据 | FAILED，**禁止进入 LIFT/TRANSPORT** |
| 320 OBJECT_DROPPED | 预留给上层感知/任务层 | （本包不自动产生） |
| 400 SEAT_NOT_VERIFIED | 未确认桌面承托 | FAULT，**禁止 UNLOAD/DISENGAGE** |
| 410 UNLOCK_FAILED | V2 解锁失败 / 已受理但锁止未解除 | FAULT，禁止退出与上抬 |
| 420 RELEASE_NOT_VERIFIED | 退钩/释放结果未知 | FAULT，禁止撤离 |
| 430 PLACEMENT_FAILED | 未在判定区或稳定性未达标 / 核验窗口耗尽 | FAILED |
| 500 SAFETY_INTERLOCK | 预留给安全监控层 | — |
| 510 HARDWARE_FAULT | 接合后工具状态异常、工具端口未接通 | FAULT |
| 520 CANCEL_NOT_CONFIRMED | 取消后未确认停稳 | FAULT |

---

## 6. 离线运行方式

**依赖：仅 Python 3.10+ 与 pytest（无 pytest 时用标准库 unittest 亦可）。无需 ROS 环境、无需 colcon build。**

```bash
cd /home/robot/meituan_challenge_ws/src/mtc_manipulation

# 语法检查
python3 -m py_compile mtc_manipulation/*.py test/*.py

# 单元测试（pytest，推荐）
python3 -m pytest test/ -v

# 无 pytest 时的等价方式（标准库 unittest）
python3 -m unittest discover -s test -p 'test*.py' -v
```

两个测试文件都实现了 unittest 的 `load_tests` 协议，因此同一份用例可在
pytest 与 unittest 下运行，无需改动测试代码。

测试分层：
- `test/test_pick_place_fsm.py`：FSM 核心（正常闭环 V1/V2、各错误码、取消、稳定性、联锁）；
- `test/test_server_shell.py`：用替身模块（`rclpy` / `mtc_interfaces`）验证 Action Server 外壳
  可导入、可构造、可用 Fake 端口跑通一次闭环；**不能替代 Jazzy 下的真实 DDS 验证**。

在真实 ROS 2 环境（需先 `colcon build`，本机未做）：

```bash
ros2 run mtc_manipulation pick_place_server --ros-args -p tool_type:=passive_hook_v1
```

---

## 7. 未实现 / 未验证点（如实声明）

1. **未编译验证**：`mtc_interfaces` 消息/Action 未生成，`package.xml`/`setup.py` 未经过
   `colcon build`；ROS 侧字段名按 `.action` / `.msg` 源码人工核对，可能存在 IDL 细节偏差。
2. **工具端口未接通**：工作空间没有 V1/V2 的真实工具管理器接口；
   `RosToolPort` 除 `TriggerUnlock` 脉冲外一律返回“未验证”，FSM 会保守停在 FAULT/FAILED，
   这是刻意行为（不伪装成功）。
3. **规划为透传**：`mtc_interfaces` 中没有 `PlanMotion.action`（设计 §11 提到的是旧工程契约），
   `RosPlannerPort` 只做输入校验与透传，**不提供碰撞检查 / IK 结论**。
4. **运动目标为占位**：`MotionPrimitive.target_pose` 未做 TF 求解，`MotionCommand.target_rad`
   为占位零位；真实关节目标必须由运动执行层按标定结果填充。
5. **未验证项**（设计 §12 待决问题）：V1/V2 的 TCP、入环/退钩方向与容差、V2 电磁 IO 反馈来源、
   停稳阈值与超时数值。当前 `PickPlaceConfig.state_timeouts` 为保守默认值，**未经实机标定**。
6. **未覆盖能力**：不实现急停、不解除安全状态、不在上电前发运动指令（设计 §3.1 边界）。
   软件停止不等于实体急停（设计 §6.4）。
7. `ToolStrategy` 相比设计 §5.1 增加了 `make_engage_plan` 等接触敏感阶段原语方法
   （设计只列了 acquire/release 两个入口），这是为了让“V1/V2 差异仅在 ENGAGE/VERIFY_ENGAGEMENT/DISENGAGE”
   在代码结构上显式可测。
