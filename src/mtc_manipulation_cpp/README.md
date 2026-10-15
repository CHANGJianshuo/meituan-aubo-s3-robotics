# mtc_manipulation_cpp

单块电池抓放状态机（第二层 PickPlace FSM）的 **C++17 迁移实现**。

对既有 Python 参考实现
`src/mtc_manipulation/mtc_manipulation/{pick_place_fsm,tool_strategy}.py`。
**旧 Python 实现保持不变**，作为本实现的行为基线。

## 迁移范围

* **22 个状态**（顺序即设计 §3.2 流水线顺序），字符串值与 Python 逐字一致；
* **端口契约**：`PerceptionPort` / `PlannerPort` / `MotionPort` / `ToolPort`
  （统一约定：返回 `std::optional`，**`nullopt` 表示调用未完成**）；
* **看门狗**：16 个状态的超时；**运动在途状态超时 → `FAULT(230)` 并禁止重发**；
* **取消**：`CANCEL_PENDING` 等待 `stop_confirmed`，未确认 → `FAULT(520)`；
* **恢复预算**：仅"确认无载荷且运动状态已知"时可重试；
* **V1/V2 策略切换**：`PassiveHookV1Strategy` / `MagneticLatchV2Strategy`；
* **运动适配桥接**：`MotionAdapterPort` 把 `mtc_motion_execution_cpp::MotionAdapter`
  适配为 PickPlace 的 `MotionPort`。

## 八条安全语义

| # | 语义 | 实现机制 |
|---|---|---|
| 1 | `placement_verified` 只允许由 `VERIFY_PLACED` 成功路径写入 | 成员 **private**；唯一写入口违反状态即抛 `std::logic_error` |
| 2 | 挂载必须有几何级（或更高）证据，否则 `310`，禁止 `LIFT/TRANSPORT` | `has_geometry_evidence() \|\| vision_follow` |
| 3 | `ENGAGE` 失败 → `300`，只有确认无载荷才可重试 | `is_resumable_context()` 检查 `holding_state()=="none"` |
| 4 | `SEAT` 后未确认承托 → `400`，禁止 `UNLOAD/DISENGAGE` | `make_fault(400, …)` |
| 5 | 解锁失败/结果未知 → `410/420` → `FAULT`，禁止硬拉上抬 | 前置联锁 → `unlock.ok` → `unlock.released` 三道判定 |
| 6 | 取消必须等停稳确认，未确认 → `520` | `h_cancel_pending` 只在 `stop_confirmed` 时返回 `CANCELLED` |
| 7 | 运动状态未知 → `FAULT` 并禁止自动重发 | `motion_resend_allowed_` 置 false |
| 8 | 放置需在目标区域连续稳定 `placement_stability_sec` 才 `SUCCESS` | 逐 step 采样；反例样本清零；窗口耗尽 → `430` |

## V1 / V2 差异

差异**只**体现在 `ENGAGE` / `VERIFY_ENGAGEMENT` / `DISENGAGE` 三个阶段：

| 阶段 | V1 | V2 |
|---|---|---|
| 接近 | `side_insert` | `top_insert` |
| 接合 | `hook_engage` | `auto_lock` |
| 分离 | `side_withdraw` | `withdraw`（+ 电磁解锁） |
| 解锁脉冲 | **无**（任何解锁请求都被拒绝） | 需 `seated && unloaded`，否则 `400`/`420` |
| IO 通道 | 空 | `{"electromagnetic_unlock"}` |

其余阶段（`test_lift` / `lift` / `transport` / `pre_seat` / `seat_contact` /
`retreat`）与工具型号无关。

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash
cd ~/meituan_challenge_ws
colcon build --packages-select mtc_core_cpp mtc_motion_execution_cpp mtc_manipulation_cpp
colcon test  --packages-select mtc_manipulation_cpp
colcon test-result --all
```

期望：**102 个用例 PASS，0 失败，0 跳过**。

| 可执行文件 | 用例数 | 覆盖 |
|---|---|---|
| `test_pick_place_tables` | 20 | 表完整性、状态常量、流水线顺序、启动校验、feedback |
| `test_tool_strategy` | 15 | V1/V2 原语、接触标记、6D 目标、解锁前置条件、工厂 |
| `test_pick_place_fsm_v1` | 13 | V1 全流程、状态序列、命令序列、无电磁动作、稳定性窗口 |
| `test_pick_place_fsm_v2` | 14 | V2 全流程、差异隔离、解锁联锁、策略拒绝 |
| `test_pick_place_safety` | 40 | 感知/规划失败、超时、取消、状态未知、超时锁定、恢复预算 |

测试替身见 [`test/fakes.hpp`](test/fakes.hpp)，支持脚本化的故障注入
（`pending_ticks` 模拟在途、`unknown_primitives` / `timeout_primitives` 注入故障、
`seated` / `unloaded` / `released` 控制机构证据）。

## 依赖

* 构建：`ament_cmake`、`mtc_core_cpp`、`mtc_motion_execution_cpp`
* 测试：`ament_cmake_gtest`
* **不依赖 `rclcpp`** —— ROS 2 Action 外壳属于第二阶段

## 相关文档

* 行为映射：[`docs/migration/FSM_BEHAVIOR_MAPPING.md`](../../docs/migration/FSM_BEHAVIOR_MAPPING.md) §5–§7
* 接口审计：[`docs/migration/INTERFACE_COMPAT_AUDIT_CPP.md`](../../docs/migration/INTERFACE_COMPAT_AUDIT_CPP.md)
