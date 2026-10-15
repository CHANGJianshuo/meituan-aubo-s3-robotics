# mtc_task_cpp

Task FSM（第一层：整轮比赛任务状态机）的 **C++17 迁移实现**。

对既有 Python 参考实现 `src/mtc_task/mtc_task/{task_fsm,task_validation}.py`。
**旧 Python 实现保持不变**，作为本实现的行为基线。

## 迁移范围

* **13 个状态**：`INIT / SELF_CHECK / WAIT_TASK / VALIDATE_TASK / SCAN_SCENE /
  EXECUTE_ITEM / RECORD_RESULT / RECOVERY / CANCEL_PENDING / FINISHED /
  TASK_FAILED / FAULT / CANCELLED`
* **20 个事件**：字符串值与 Python `Event.*` 逐字一致
* **40 条转移**：集中表 + 守卫 + `fallback` / `else_state`
* **每状态超时**与§7.1 默认处置（超时去向同样由转移表描述，不散落 `if`）
* **重试白名单**：仅 `110/120/200/430/210` 可自动重试（未列入者一律不可重试）
* **并发 Goal 拒绝**：每个活动状态显式登记 `goal_accepted` 的拒绝转移，**不排队**
* **任务级截止时间**看门狗（`timeout_ms`）

## 结构约定（与 Python 一致）

* 状态转移集中在 `TaskFsm::transitions()`；
* 进入状态的数据更新与动作产出集中在 `TaskFsm::entry_actions()`；
* 所有外部副作用以 `ActionRequest` 返回（`CALL_PICK_PLACE` / `CANCEL_CHILD` /
  `REQUEST_STOP` / `PUBLISH_STATE` / `LOCK_ACTIONS`），**不做任何真实 IO**。

### 与 Python 的实现差异（不改语义）

| 项 | Python | C++ |
|---|---|---|
| 守卫引用 | 字符串名 + `getattr` | **成员函数指针**（拼写错误编译期即发现） |
| payload | `Dict[str, Any]` | 类型化 `EventPayload` |
| 表自检 | 导入期 `_self_check_tables()` 抛异常 | 可测试的 `verify_tables()` 返回问题清单 |
| 待发校验事件 | 动态属性 | `std::optional` 成员 |

## 安全语义

* **Goal Accepted ≠ 执行成功**；`placement_verified` 未通过时不得记为完成；
* 不可重试错误（如 `230 MOTION_STATUS_UNKNOWN`）→ `FAULT` 并**保留真实错误码**，
  不退化为 `510 HARDWARE_FAULT`；
* 取消必须等 `child_stopped_confirmed` 才进入 `CANCELLED`；
  未确认超时 → `FAULT(520)`，且**迟到的停稳确认不得改写终态**。

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash
cd ~/meituan_challenge_ws
colcon build --packages-select mtc_core_cpp mtc_task_cpp
colcon test  --packages-select mtc_task_cpp
colcon test-result --all
```

期望：**79 个用例 PASS，0 失败，0 跳过**。

| 可执行文件 | 用例数 | 覆盖 |
|---|---|---|
| `test_task_validation` | 19 | 任务请求校验（模式/颜色/工位/超时） |
| `test_task_fsm_core` | 21 | 正常流程、序列流程、复位、快照、表完整性 |
| `test_task_fsm_safety` | 39 | 并发拒绝、超时、取消、恢复预算、状态未知锁定 |

## 依赖

* 构建：`ament_cmake`、`mtc_core_cpp`
* 测试：`ament_cmake_gtest`
* **不依赖 `rclcpp`** —— ROS 2 Action 外壳属于第二阶段

## 相关文档

* 行为映射：[`docs/migration/FSM_BEHAVIOR_MAPPING.md`](../../docs/migration/FSM_BEHAVIOR_MAPPING.md) §2–§4
* 第二阶段计划：[`docs/migration/PHASE2_ROS2_INTEGRATION_PLAN.md`](../../docs/migration/PHASE2_ROS2_INTEGRATION_PLAN.md) §P2-1
