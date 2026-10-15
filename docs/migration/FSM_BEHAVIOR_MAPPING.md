# Python → C++ FSM 行为映射表（第一阶段）

| 项目 | 值 |
|---|---|
| 迁移分支 | `feature/jazzy-cpp-fsm`（基线 `main@bd914fb`） |
| 映射时间 | 2026-10-11 |
| 语言标准 | C++17 |
| Python 基线 | 原迁移保持原样；审计同步 D-2；最终安全验收同步严格正稳定窗口及其默认值/测试 |
| 基线审计报告 | [`PYTHON_BEHAVIOR_BASELINE.md`](PYTHON_BEHAVIOR_BASELINE.md) |
| 接口审计报告 | [`INTERFACE_COMPAT_AUDIT_CPP.md`](INTERFACE_COMPAT_AUDIT_CPP.md) |

> **本表用途**：作为 C++ 实现与 Python 参考实现之间的**逐项对照凭据**。
> 任何"行为不一致"都必须出现在 §9「已知差异清单」中并给出理由；
> 未列入 §9 的差异一律视为迁移缺陷。

---

## 1. 模块映射

| Python 模块 | C++ 包 / 文件 | 说明 |
|---|---|---|
| `mtc_task/task_fsm.py` | `mtc_task_cpp/include/mtc_task_cpp/task_fsm.hpp`<br>`mtc_task_cpp/src/task_fsm.cpp` | Task FSM（第一层） |
| `mtc_task/task_validation.py` | `mtc_task_cpp/include/mtc_task_cpp/task_validation.hpp`<br>`mtc_task_cpp/src/task_validation.cpp` | 任务请求校验 |
| `mtc_task/task_fsm.py`（类型部分） | `mtc_task_cpp/include/mtc_task_cpp/task_types.hpp`<br>`mtc_task_cpp/src/task_types.cpp` | 状态/事件/请求/结果类型 |
| `mtc_manipulation/pick_place_fsm.py` | `mtc_manipulation_cpp/include/mtc_manipulation_cpp/pick_place_fsm.hpp`<br>`mtc_manipulation_cpp/src/pick_place_fsm.cpp` | PickPlace FSM（第二层） |
| `mtc_manipulation/pick_place_fsm.py`（类型与端口） | `mtc_manipulation_cpp/include/mtc_manipulation_cpp/pick_place_types.hpp`<br>`mtc_manipulation_cpp/src/pick_place_types.cpp` | 状态/端口/配置/结果 |
| `mtc_manipulation/tool_strategy.py` | `mtc_manipulation_cpp/include/mtc_manipulation_cpp/tool_strategy.hpp`<br>`mtc_manipulation_cpp/src/tool_strategy.cpp` | V1/V2 工具策略 |
| `mtc_motion_execution/backends.py`（常量与类型） | `mtc_core_cpp/include/mtc_core_cpp/{error_codes,tool_state,results,exec_state}.hpp`<br>`mtc_core_cpp/src/error_codes.cpp` | 错误码、工具状态、端口结果契约 |
| `mtc_motion_execution/backends.py`（端口与 Mock） | `mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/*.hpp`<br>`mtc_motion_execution_cpp/src/*.cpp` | MotionAdapter + Mock/Disabled |
| `mtc_motion_execution/executor_core.py` | `mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/executor_core.hpp`<br>`src/executor_core.cpp` | 七步握手执行核心 |
| —（新增，无 Python 对应） | `mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/motion_adapter.hpp` | **Motion Adapter 抽象接口**（需求 4） |
| —（新增，无 Python 对应） | `mtc_core_cpp/include/mtc_core_cpp/timed_trajectory.hpp` | **时间参数化轨迹类型**（缺口 G1 载体） |
| —（新增，无 Python 对应） | `mtc_manipulation_cpp/include/mtc_manipulation_cpp/motion_adapter_port.hpp` | MotionAdapter → PickPlace MotionPort 桥接 |

### 1.1 为什么新增 `mtc_core_cpp`

Python 侧在 **5 个模块**中各自镜像了同一套错误码
（`task_fsm.py:70-90`、`pick_place_fsm.py:80-100`、`interlocks.py:50-67`、
`backends.py:36-42`、`bridge_core.py:19-23`），存在漂移风险。

C++ 侧收敛为**唯一定义**，并以 `test_error_codes_msg_parity` 直接解析
`mtc_interfaces/msg/ErrorCodes.msg` 做双向一致性校验（不匹配即 FAIL）。

---

## 2. Task FSM 状态映射（13 个）

| # | Python `TaskStateName` | C++ `TaskStateName` | 字符串值 | 状态类型 |
|---|---|---|---|---|
| 1 | `INIT` | `INIT` | `"INIT"` | — |
| 2 | `SELF_CHECK` | `SELF_CHECK` | `"SELF_CHECK"` | — |
| 3 | `WAIT_TASK` | `WAIT_TASK` | `"WAIT_TASK"` | 等待态 |
| 4 | `VALIDATE_TASK` | `VALIDATE_TASK` | `"VALIDATE_TASK"` | 活动 |
| 5 | `SCAN_SCENE` | `SCAN_SCENE` | `"SCAN_SCENE"` | 活动 |
| 6 | `EXECUTE_ITEM` | `EXECUTE_ITEM` | `"EXECUTE_ITEM"` | 活动 |
| 7 | `RECORD_RESULT` | `RECORD_RESULT` | `"RECORD_RESULT"` | 活动 |
| 8 | `RECOVERY` | `RECOVERY` | `"RECOVERY"` | 活动 |
| 9 | `CANCEL_PENDING` | `CANCEL_PENDING` | `"CANCEL_PENDING"` | 活动 |
| 10 | `FINISHED` | `FINISHED` | `"FINISHED"` | 终态 |
| 11 | `TASK_FAILED` | `TASK_FAILED` | `"TASK_FAILED"` | 终态 |
| 12 | `FAULT` | `FAULT` | `"FAULT"` | 终态（锁定） |
| 13 | `CANCELLED` | `CANCELLED` | `"CANCELLED"` | 终态 |

* **活动状态集合**（并发 Goal 拒绝依据）：`VALIDATE_TASK / SCAN_SCENE / EXECUTE_ITEM /
  RECORD_RESULT / RECOVERY / CANCEL_PENDING` —— 两侧一致。
* Python `FAULT` 不在 `TERMINAL_STATES` 内；审计修复后 C++ 将其列为执行终态，
  但继续保持故障锁定，普通 `RESET` 和新 Goal 均不能解锁。详见 D-3。

---

## 3. Task FSM 事件映射（20 个）

字符串值逐字一致，映射为 C++ `Event` 中的 `static constexpr const char *`：

| Python `Event.X` | 字符串值 | C++ | 主要来源 |
|---|---|---|---|
| `SELF_CHECK_OK` | `self_check_ok` | `Event::SELF_CHECK_OK` | 启动/自检 |
| `SELF_CHECK_FAIL` | `self_check_fail` | `Event::SELF_CHECK_FAIL` | 自检 |
| `GOAL_ACCEPTED` | `goal_accepted` | `Event::GOAL_ACCEPTED` | Action goal |
| `GOAL_REJECTED` | `goal_rejected` | `Event::GOAL_REJECTED` | 受理后撤回 |
| `VALIDATED` | `task_validated` | `Event::VALIDATED` | 校验通过 |
| `VALIDATION_FAILED` | `task_validation_failed` | `Event::VALIDATION_FAILED` | 校验失败 |
| `TARGET_FOUND` | `target_found` | `Event::TARGET_FOUND` | 感知 |
| `TARGET_NOT_FOUND` | `target_not_found` | `Event::TARGET_NOT_FOUND` | 感知 |
| `SCAN_FAILED` | `scan_failed` | `Event::SCAN_FAILED` | 重试预算耗尽 |
| `ITEM_SUCCEEDED` | `item_succeeded` | `Event::ITEM_SUCCEEDED` | PickPlace Result |
| `ITEM_FAILED` | `item_failed` | `Event::ITEM_FAILED` | PickPlace Result |
| `ITEM_UNSAFE` | `item_unsafe` | `Event::ITEM_UNSAFE` | 不可重试错误 |
| `RETRY_APPROVED` | `retry_approved` | `Event::RETRY_APPROVED` | 恢复 |
| `RETRY_DENIED` | `retry_denied` | `Event::RETRY_DENIED` | 恢复 |
| `RECORD_OK` | `record_ok` | `Event::RECORD_OK` | 记录结果 |
| `RECORD_FAILED` | `record_failed` | `Event::RECORD_FAILED` | 记录结果 |
| `CANCEL_REQUESTED` | `cancel_requested` | `Event::CANCEL_REQUESTED` | Action cancel |
| `CHILD_STOPPED_CONFIRMED` | `child_stopped_confirmed` | `Event::CHILD_STOPPED_CONFIRMED` | 子任务停稳确认 |
| `RESET` | `reset` | `Event::RESET` | 终态复位 |
| `TIMEOUT` | `timeout` | `Event::TIMEOUT` | 看门狗 |

---

## 4. Task FSM 转移表映射（40 条）

Python `TRANSITIONS` 为 40 条 `(state, event)` 键。C++ `TaskFsm::transitions()`
逐条对应，并以 `(state, event)` 为键建索引。

| 分组 | Python 位置 | C++ | 条数 | 校验方式 |
|---|---|---|---|---|
| 启动与自检 | `task_fsm.py:431-434` | `TaskFsm::transitions()` 第 1-4 条 | 4 | `TaskFsmTables.*` |
| 任务受理 + 并发拒绝 | `:436-444` | 第 5-11 条 | 7 | `EveryActiveStateRegistersConcurrentGoalRejection` |
| 任务校验 | `:446-448` | 第 12-14 条 | 3 | `InvalidSequenceFailsWithoutGuessing` |
| 场景扫描 | `:450-453` | 第 15-18 条 | 4 | `ScanSceneTimeoutGoesToRecovery…` |
| 单项执行 | `:455-465` | 第 19-22 条 | 4 | `NonRecoverableCodesAllGoToFault` |
| 记录结果 | `:468-477` | 第 23-25 条 | 3 | `ThreeItemsCompleteInSlotOrder` |
| 恢复 | `:480-489` | 第 26-28 条 | 3 | `RetryBudgetExhaustionGoesToTaskFailed` |
| 取消 | `:491-493` | 第 29-31 条 | 3 | `UnconfirmedStopTimesOutToFault520` |
| 终态复位 | `:495-497` | 第 32-34 条 | 3 | `ResetFromFinishedClearsTaskContext` |
| 每状态超时去向 | `:499-511` | 第 35-41 条 | 7 | `TaskFsmTimeout.*`（6 例） |

**结构等价性保障**：
* Python 在导入期执行 `_self_check_tables()`（`task_fsm.py:541-564`），任何漏配立即抛异常；
* C++ 改为可测试的 `TaskFsm::verify_tables()` 返回问题清单，
  由 `TaskFsmTables.VerifyTablesReportsNoProblem` 断言为空 ——
  **比 Python 更严格**（问题被显式暴露而非仅崩溃）。

### 4.1 守卫函数映射

| Python 守卫 | C++ 守卫 | 语义 |
|---|---|---|
| `_guard_goal_admissible` | `guard_goal_admissible` | WAIT_TASK 下受理 Goal + task_id 基本检查 |
| `_guard_goal_denied` | `guard_goal_denied` | 恒 false，由 step() 登记并发拒绝 |
| `_guard_target_usable` | `guard_target_usable` | object_id 非空 + 颜色双重校验 |
| `_guard_error_recoverable` | `guard_error_recoverable` | 白名单判定 |
| `_guard_all_items_done` | `guard_all_items_done` | 全部完成判定 |
| `_guard_retry_budget_available` | `guard_retry_budget_available` | 重试预算 |
| `_guard_cancel_timeout` | `guard_cancel_timeout` | 仅真超时放行 |
| `_guard_state_timed_out` | `guard_state_timed_out` | 每状态超时 |

**C++ 增强（不改语义）**：guard 由字符串名改为**成员函数指针**，
拼写错误在编译期即被发现，而 Python 只在运行到该转移时才 `getattr` 失败。

### 4.2 进入动作映射

| Python `ENTRY_ACTIONS` | C++ `entry_actions()` | 动作产出 |
|---|---|---|
| `INIT: None` | `{INIT, nullptr}` | — |
| `SELF_CHECK: "_enter_self_check"` | `&TaskFsm::enter_self_check` | — |
| `WAIT_TASK: "_enter_wait_task"` | `&TaskFsm::enter_wait_task` | 清空任务上下文 |
| `VALIDATE_TASK: "_enter_validate_task"` | `&TaskFsm::enter_validate_task` | 计算待发校验事件 |
| `SCAN_SCENE: "_enter_scan_scene"` | `&TaskFsm::enter_scan_scene` | — |
| `EXECUTE_ITEM: "_enter_execute_item"` | `&TaskFsm::enter_execute_item` | `CALL_PICK_PLACE` |
| `RECORD_RESULT: None` | `{RECORD_RESULT, nullptr}` | — |
| `RECOVERY: "_enter_recovery"` | `&TaskFsm::enter_recovery` | — |
| `CANCEL_PENDING: "_enter_cancel_pending"` | `&TaskFsm::enter_cancel_pending` | `CANCEL_CHILD` |
| `FINISHED: "_enter_terminal_ok"` | `&TaskFsm::enter_terminal_ok` | 生成 outcome |
| `TASK_FAILED: "_enter_task_failed"` | `&TaskFsm::enter_task_failed` | 生成 outcome |
| `FAULT: "_enter_fault"` | `&TaskFsm::enter_fault` | `REQUEST_STOP` + `LOCK_ACTIONS` |
| `CANCELLED: "_enter_cancelled"` | `&TaskFsm::enter_cancelled` | 生成 outcome |

**不变式（两侧一致）**：每次 `enter_state` 最后都追加一个 `PUBLISH_STATE` 动作
（Python `task_fsm.py:790`，C++ `enter_state()` 末尾）。

### 4.3 动作意图映射

| Python `ActionKind` | C++ `ActionKind` | 载荷 |
|---|---|---|
| `CALL_PICK_PLACE` | `ActionKind::CALL_PICK_PLACE` | `PickPlaceGoal`（request_id/object_id/expected_color/target_slot/placement_stability_sec） |
| `CANCEL_CHILD` | `ActionKind::CANCEL_CHILD` | `request_id` |
| `REQUEST_STOP` | `ActionKind::REQUEST_STOP` | — |
| `PUBLISH_STATE` | `ActionKind::PUBLISH_STATE` | `TaskStateSnapshot` |
| `LOCK_ACTIONS` | `ActionKind::LOCK_ACTIONS` | — |

**C++ 差异（类型化，非语义）**：Python 载荷是 `Dict[str, Any]`；
C++ 用固定字段集合，`PUBLISH_STATE` 的 snapshot 为强类型结构。

### 4.4 超时配置映射

| 状态 | Python 秒 | C++ 秒 | 超时去向 |
|---|---|---|---|
| `INIT` | 5.0 | 5.0 | `FAULT`（510） |
| `SELF_CHECK` | 10.0 | 10.0 | `FAULT`（510） |
| `WAIT_TASK` | `None` | `nullopt` | 不设超时 |
| `VALIDATE_TASK` | 2.0 | 2.0 | `TASK_FAILED`（100） |
| `SCAN_SCENE` | 5.0 | 5.0 | `RECOVERY`（110） |
| `EXECUTE_ITEM` | 60.0 | 60.0 | `FAULT`（220，不自动重发） |
| `RECORD_RESULT` | 2.0 | 2.0 | `TASK_FAILED`（430） |
| `RECOVERY` | 10.0 | 10.0 | `TASK_FAILED`（110） |
| `CANCEL_PENDING` | 5.0 | 5.0 | `FAULT`（520） |
| `FINISHED / TASK_FAILED / FAULT / CANCELLED` | `None` | `nullopt` | 不设超时 |

**可恢复错误白名单**（`DEFAULT_RECOVERABLE_ERRORS`）：
`110, 120, 200, 430, 210` —— 两侧一致，且均为**白名单**语义
（未列入者一律不可重试，含未知码）。

**不可重试错误**：`230, 220, 500, 510, 410, 420, 320, 520` —— 两侧一致。

---

## 5. PickPlace FSM 状态映射（22 个）

| # | Python `PickPlaceState` | C++ `PickPlaceState` | 终态 | 运动在途 |
|---|---|---|---|---|
| 1 | `RESOLVE_TARGET` | `RESOLVE_TARGET` | | |
| 2 | `PLAN_ACQUIRE` | `PLAN_ACQUIRE` | | |
| 3 | `MOVE_PRE_ALIGN` | `MOVE_PRE_ALIGN` | | ✔ |
| 4 | `ENGAGE` | `ENGAGE` | | ✔ |
| 5 | `VERIFY_ENGAGEMENT` | `VERIFY_ENGAGEMENT` | | |
| 6 | `TEST_LIFT` | `TEST_LIFT` | | ✔ |
| 7 | `VERIFY_ATTACHED` | `VERIFY_ATTACHED` | | |
| 8 | `LIFT` | `LIFT` | | ✔ |
| 9 | `TRANSPORT` | `TRANSPORT` | | ✔ |
| 10 | `MOVE_PRE_SEAT` | `MOVE_PRE_SEAT` | | ✔ |
| 11 | `SEAT` | `SEAT` | | ✔ |
| 12 | `VERIFY_SEATED` | `VERIFY_SEATED` | | |
| 13 | `UNLOAD` | `UNLOAD` | | ✔ |
| 14 | `DISENGAGE` | `DISENGAGE` | | ✔ |
| 15 | `RETREAT` | `RETREAT` | | ✔ |
| 16 | `VERIFY_PLACED` | `VERIFY_PLACED` | | |
| 17 | `SUCCESS` | `SUCCESS` | ✔ | |
| 18 | `RECOVERY` | `RECOVERY` | | |
| 19 | `FAILED` | `FAILED` | ✔ | |
| 20 | `FAULT` | `FAULT` | ✔ | |
| 21 | `CANCEL_PENDING` | `CANCEL_PENDING` | | |
| 22 | `CANCELLED` | `CANCELLED` | ✔ | |

* 字符串值与 Python `PickPlaceState.value` 逐字一致（`feedback.substate` 直接对外）。
* **流水线顺序**（`_PIPELINE_STATES` / `pipeline_states()`）逐项一致，
  由 `PickPlaceTables.PipelineMatchesDesignOrder` 断言。
* **运动在途集合**由 `motion_in_flight_states()` 承载，被 `VerifyTables` 校验
  "每个运动在途状态都必须有看门狗超时"。

### 5.1 PickPlace 每状态超时映射

| 状态 | Python 秒 | C++ 秒 |
|---|---|---|
| `RESOLVE_TARGET` | 2.0 | 2.0 |
| `PLAN_ACQUIRE` | 2.0 | 2.0 |
| `MOVE_PRE_ALIGN` | 30.0 | 30.0 |
| `ENGAGE` | 15.0 | 15.0 |
| `VERIFY_ENGAGEMENT` | 5.0 | 5.0 |
| `TEST_LIFT` | 15.0 | 15.0 |
| `VERIFY_ATTACHED` | 5.0 | 5.0 |
| `LIFT` | 30.0 | 30.0 |
| `TRANSPORT` | 60.0 | 60.0 |
| `MOVE_PRE_SEAT` | 30.0 | 30.0 |
| `SEAT` | 20.0 | 20.0 |
| `VERIFY_SEATED` | 5.0 | 5.0 |
| `UNLOAD` | 15.0 | 15.0 |
| `DISENGAGE` | 20.0 | 20.0 |
| `RETREAT` | 30.0 | 30.0 |
| `VERIFY_PLACED` | 30.0 | 30.0 |

---

## 6. PickPlace 端口映射

| Python 端口方法 | C++ 签名 | 「在途」表达 |
|---|---|---|
| `PerceptionPort.get_detection(object_id, expected_color, max_age_s) -> DetectionResult` | `std::optional<DetectionResult> get_detection(const std::string&, std::optional<int>, double)` | `nullopt` |
| `PlannerPort.plan(phase, target) -> PlanResult` | `std::optional<PlanResult> plan(const PlanTarget&)` | `nullopt` |
| `MotionPort.execute_joint_move(command) -> Optional[MotionResult]` | `std::optional<MotionResult> execute_joint_move(const MotionCommand&)` | 已经就是 `nullopt` |
| `MotionPort.request_stop(reason) -> StopResult` | `StopResult request_stop(const std::string&)` | 无（同步） |
| `ToolPort.engage/test_lift/unload/unlock/disengage -> Optional[ToolEvidence]` | `std::optional<ToolEvidence>` | `nullopt` |
| `ToolPort.seat_verify() -> ToolEvidence` | `ToolEvidence seat_verify()` | 无（Python 亦非 Optional） |
| `ToolPort.holding_state() -> str` | `std::string holding_state()` | 无 |

**`_do(key, func)` 缓存语义映射**：

Python 用 `self._port_results[(state.value, key)]` 单一字典缓存所有端口结果
（`pick_place_fsm.py:849-864`）。C++ 按结果类型拆为 5 个类型安全缓存
（`detection_cache_` / `plan_cache_` / `motion_cache_` / `tool_cache_` /
`primitives_cache_`）以及 1 个 `stop_cache_`（停止请求）。

| 不变式 | Python | C++ |
|---|---|---|
| 同一状态内同一 key 的副作用只执行一次 | `_do` 命中缓存 | `resolve_cached()` 命中缓存 |
| 端口未完成时**不缓存**结果 | `if value is None: return _RUN` | `if (!value.has_value()) return false;`（不 emplace） |
| `_clear_port_results()` 清空并释放在途命令 | `:866-869` | `clear_port_results()` |
| 同一状态内复用同一条 `MotionCommand`（绝不重发） | `_cached_command()` `:871-885` | `cached_command()` |
| 离开状态释放在途命令引用 | `_enter_state` `:758-759` | `enter_state()` |

审计修复的例外：C++ `VERIFY_PLACED` 每次取得新观测，不缓存已完成的感知结果。
`nullopt` 仅表示请求进行中；重复时间戳不能扩展稳定窗口。Python 的缓存逻辑除
D-2 外按本次授权范围保持原样，不能把 Python Mock 通过当作 C++ 新证据契约验证。

---

## 7. 八条安全语义映射

| # | Python 语义（`pick_place_fsm.py:13-26`） | C++ 实现机制 | 覆盖测试 |
|---|---|---|---|
| 1 | `placement_verified` 只允许由 `VERIFY_PLACED` 成功路径写入，且仅在该处写 | 成员 **private**；唯一写入口 `set_placement_verified()`，违反状态即抛 `std::logic_error`——**从类型层面杜绝外部写入** | `PlacementVerifiedCannotBeWrittenOutsideVerifyPlaced`<br>`PlacementVerifiedOnlyTrueAfterSuccessfulVerifyPlaced` |
| 2 | `VERIFY_ATTACHED` 必须有几何级（或更高）挂载证据，否则 310，禁止进入 LIFT/TRANSPORT | `evidence.has_geometry_evidence() \|\| vision_follow` 双重判定 | `AttachNotVerifiedBlocksLiftAndTransport`<br>`VisionFollowCanCompensateForWeakEvidence` |
| 3 | `ENGAGE` 失败 → 300，且只有确认"无载荷"时才允许退回重试 | `make_error(300, …, is_resumable_context())`；`is_resumable_context()` 要求 `holding_state()=="none"` 且重发允许 | `EngageFailureGoesToEngageFailed300`<br>`HoldingStateBlocksAutomaticRetry`<br>`UnknownHoldingStateIsConservative` |
| 4 | `SEAT` 后未确认桌面承托 → 400，禁止进入 UNLOAD/DISENGAGE | `make_fault(400, …)` 且 UNLOAD handler 不可达 | `SeatNotVerifiedBlocksUnload` |
| 5 | `DISENGAGE` 解锁失败或结果未知 → 410/420 → FAULT，禁止强行抽出/上抬 | 三道判定：前置联锁 → `unlock.ok` → `unlock.released`；`disengage` 结果未知亦 FAULT | `UnlockFailureGoesToFaultAndDoesNotRetreat`<br>`AcceptedPulseWithoutReleaseIsStillFailure`<br>`DisengageReleaseUnknownGoesToFaultAndDoesNotRetreat` |
| 6 | 取消 → `CANCEL_PENDING`，等待 `stop_confirmed`；未确认 → 520，绝不因收到 cancel 就声称已停 | `h_cancel_pending` 只在 `stop_confirmed` 为真时返回 CANCELLED，否则 `make_fault(520, …, resend_allowed=false)` | `CancelWithoutStopConfirmationGoesToFault520`<br>`CancelNeverReachesCancelledWithoutConfirmation` |
| 7 | 运动状态未知 → 230 → FAULT，并禁止自动重发（进入后即使人工 step 也不重发） | `motion_resend_allowed_` 置 false，且 FAULT 为终态无转移 | `MotionStatusUnknownLocksAndDoesNotResend`<br>`MotionInFlightTimeoutGoesToFault230AndLocksResend`<br>`RecoveryIsRefusedWhenMotionResendDisallowed` |
| 8 | `VERIFY_PLACED` 必须在目标区域内连续稳定 `placement_stability_sec`（默认 3.0s）才允许 SUCCESS，否则 430 → FAILED | 新鲜有效且时间戳严格递增的观测跨度累计；无效、回退、丢失或超龄采样间隔清零；重复帧/空轮询不累计 | `PlacementAudit.NewFramesMustCoverFullThreeSeconds`<br>`PlacementAudit.FrozenTimestampCannotSucceedForFiveSeconds`<br>`PlacementAudit.InvalidOrStaleFrameInterruptsWindow`<br>`PlacementAudit.PerceptionStopsUpdatingCannotAccumulateTime` |

### 7.1 V1 / V2 差异映射

| 阶段 | V1 `PassiveHookV1Strategy` | V2 `MagneticLatchV2Strategy` |
|---|---|---|
| `make_acquire_plan` | `side_insert` | `top_insert` |
| `make_engage_plan` | `hook_engage`（contact, 0.005 m/s） | `auto_lock`（contact, 0.005 m/s） |
| `make_release_plan` | `unload`（contact） | `unload`（contact） |
| `make_disengage_plan` | `side_withdraw` | `withdraw` |
| `requires_unlock_pulse()` | `false` | `true` |
| `io_channels()` / `IO_CHANNELS` | 空 | `{"electromagnetic_unlock"}` |
| `evaluate_unlock` | 恒拒绝（`UNLOCK_FAILED`） | 需 `seated && unloaded`，否则 400 / 420 |

**共享阶段**（与型号无关）：`test_lift` / `lift` / `transport` / `pre_seat` /
`seat_contact` / `retreat` 及全部速度上限常量
（`DEFAULT_LINEAR_SPEED=0.02`、`DEFAULT_ANGULAR_SPEED=0.1`、
`CONTACT_LINEAR_SPEED=0.005`）。

**C++ 增强**：`MotionPrimitive` 增加 `std::optional<Pose6D> target_pose`，
使原语携带完整 6D 位姿与 `frame_id`（Python 的 `target_pose` 是 `Any` 占位）。
由 `PrimitivePreservesSixDofTargetPose` 断言。

---

## 8. 错误码映射（18 项）

两侧数值逐项一致（并另有 `ErrorCodesMsgParity` 测试直接对照 `.msg` 原文）：

| 常量 | 值 | Task FSM 用途 | PickPlace 用途 |
|---|---|---|---|
| `OK` | 0 | 成功 | 成功 |
| `INVALID_TASK` | 100 | 任务非法 | 非法目标工位 |
| `TARGET_NOT_FOUND` | 110 | 感知未找到 | 目标丢失/颜色不符 |
| `POSE_STALE` | 120 | 感知过期 | 感知/TF 过期 |
| `PLANNING_FAILED` | 200 | 规划失败 | 规划失败/无原语 |
| `EXECUTION_REJECTED` | 210 | 并发拒绝 / 后端拒绝 | — |
| `MOTION_TIMEOUT` | 220 | 执行超时 | 运动未到位 / 非运动态超时 |
| `MOTION_STATUS_UNKNOWN` | 230 | 状态未知 → FAULT | 反馈断流 → FAULT 并锁定 |
| `ENGAGE_FAILED` | 300 | 单项失败 | 接合失败 |
| `ATTACH_NOT_VERIFIED` | 310 | 单项失败 | 挂载证据不足 |
| `OBJECT_DROPPED` | 320 | 不可重试 | 物体脱落 |
| `SEAT_NOT_VERIFIED` | 400 | 单项失败 | 未确认桌面承托 |
| `UNLOCK_FAILED` | 410 | 不可重试 | 解锁失败/未解除 |
| `RELEASE_NOT_VERIFIED` | 420 | 不可重试 | 卸载/分离未确认 |
| `PLACEMENT_FAILED` | 430 | 可重试 | 放置核验失败 |
| `SAFETY_INTERLOCK` | 500 | 不可重试 | 速度/限位联锁 |
| `HARDWARE_FAULT` | 510 | 不可重试 | 工具状态异常 |
| `CANCEL_NOT_CONFIRMED` | 520 | 取消超时 → FAULT | 取消未确认 → FAULT |

---

## 9. 已知差异清单（**必须逐项审计**）

以下差异包含迁移时的实现差异和本次审计明确强化的安全契约。
未列入本节的任何差异都应被视为迁移缺陷。

### D-1 不可达的"类型错误"校验分支

Python 因弱类型而必须处理的分支，在 C++ 强类型下**编译期即不可表达**：

| Python 分支（`task_validation.py`） | C++ 处理 |
|---|---|
| `bad_mode`（`mode` 非整数） | `mode` 为 `int`，不可表达 |
| `bad_color_type`（颜色非整数） | `std::vector<int>`，不可表达 |
| `bad_timeout`（`timeout_ms` 非整数） | `uint32_t`，不可表达 |
| `bad_stability`（稳定性非数值） | `double`，不可表达 |
| `missing_colors`（`ordered_colors` 为 `None`） | `std::vector`，空即 `empty_colors` |
| `unparsable_goal`（goal 字段读取失败） | 由 ROS 外壳在构造 `TaskRequest` 前处理；FSM 保留 `EventPayload::goal_parse_error` 通道以维持拒绝语义 |
| `timeout_ms` 缺省（`None`） | `uint32_t` 无 null；**0 一律判为非法**（`timeout_not_positive`） |

**影响**：C++ 侧错误原因字符串集合是 Python 的子集。
`test_task_validation.cpp` 覆盖了 C++ 侧全部可达原因（19 个用例）。

> **审计提示**：若第二阶段 ROS 外壳直接把 `ExecuteTask.action` 的
> `timeout_ms` 映射为 `uint32_t`，则"缺省即 0"会被判非法 —— 需外壳显式提供
> 有效默认值。此约束已记录在 [`PHASE2_ROS2_INTEGRATION_PLAN.md`](PHASE2_ROS2_INTEGRATION_PLAN.md)。

### D-2 看门狗返回状态与内部状态不一致（审计已同步修复）

基线两侧非运动状态超时仅返回 `make_error/_error`，没有进入返回状态，导致
下一次 `step()` 继续超时。原 C++ 测试甚至断言了这个缺陷。

修复后两侧都保存重试上下文并实际进入返回状态，更新时间和状态历史。
仅无载荷且允许重发时进入 `RECOVERY`，下一步消耗一次恢复预算并重新观察；
载荷存在或未知时不自动恢复。在途运动超时继续进入 `FAULT(230)` 并锁定重发。
C++ 放置观测断流额外按新的核验契约进入 `FAILED(430)`。

验证：原 `PickPlaceSafety.NonMotionStateTimeoutReportsMotionTimeout` 已改断言实际
`RECOVERY`、历史、时间与下一步恢复；`WatchdogAudit.LoadedContextCannotRecoverAndFaultCannotRestart`
以及 Python 新增的三个 `test_watchdog_*` 均实际执行通过。

### D-3 Task FSM：执行结束与故障锁定分离（审计明确契约）

复核确认基线 Python 与 C++ 都没有 `(FAULT, RESET)` 转移；旧文档中
“reset 回到 WAIT_TASK”的描述错误。原 `FAULT` 不在执行终态集合，容易令 Action
外壳持续等待。现在 C++ 将 `FAULT` 加入执行终态集合并提供 `action_finished()`、
`fault_locked()` 和 `accepts_tasks()` 查询。结束/失败结果不解除锁定；普通 RESET、
迟到结果、停止受理与新 Goal 均不能恢复任务。

现有 IDL 无法表达受监督的人工复位，因此本阶段保留故障锁定，不增加自动恢复。
第二阶段必须定义人工授权、独立停稳证据、载荷/工具确认及重新自检后受理任务的
协议。测试：`TaskFaultAudit.ActionFinishesButOrdinaryResetAndNewTasksStayLocked`。

### D-4 未迁移的 Python 死代码

| Python 位置 | 内容 | 处理 |
|---|---|---|
| `pick_place_fsm.py:1401-1410` | `_placement_pose_changed()` | **未迁移**：全仓库无调用点（经 grep 确认），仅用于日志/回放且未接入计时 |
| `pick_place_fsm.py:1405` | `self._placement_last_pose` | C++ 保留字段 `placement_last_position_` 但在 `start()` 中复位，不参与判定 |
| `task_fsm.py:1074` | `except Exception: pass`（防御式） | C++ 侧无对应（`EventPayload` 强类型，无可抛点） |

### D-5 `MotionResult` 载荷统一

Python 存在两个近似类型：
* `pick_place_fsm.MotionResult`（含 `arrived` / `status_unknown` / `timed_out`）
* `backends.JointMoveResult`（含 `exec_state` / `motion_state_unknown`）

C++ 合并为**单一** `mtc::core::MotionResult`，同时携带
`success / arrived / stop_confirmed / status_unknown / timed_out / exec_state`。

**语义保持**：
* `ok()` = `success && arrived`（对应 Python `MotionResult.ok`）
* 审计后 `retry_allowed_without_reobservation()` =
  `!success && exec_state == REJECTED && !status_unknown && !timed_out`，
  仅明确从未执行的拒绝允许按契约重试。
* `status_unknown` 对应 Python `motion_state_unknown`（**字段改名，语义不变**）

### D-6 6D 位姿保真（**C++ 侧修正 Python 的退化**）

Python `PoseEstimate.ring_pose` / `body_pose` 是 `Tuple[float, float, float]`，
丢弃了姿态四元数、`frame_id` 与时间戳；`pick_place_server.py:138-157` 读取
`BatteryDetection.msg` 的 `PoseStamped` 时也只取 `position`。

C++ `mtc::core::Pose6D` 强制携带 `position + orientation + frame_id + stamp_s`，
且 `is_usable()` 要求 `frame_id` 非空、四元数归一化。

* **依据**：需求 5「必须保留完整 6D Pose、frame_id 和 TF 信息，
  不得退化成只有 XYZ 的目标位置」。
* **影响**：C++ FSM 的规划目标比 Python 更完整；
  `PlannerPort::plan` 的 `PlanTarget` 携带 `Pose6D`。
* **风险**：第二阶段接入真实 `PlannerPort` 实现时，必须提供真实 `frame_id`；
  若上游感知不填 `frame_id`，`has_ring_pose()` 为 false 将**拒绝运动**
  （这是有意的保守行为，不是缺陷）。

### D-7 新增能力：时间参数化轨迹（缺口 G1 的第一阶段载体）

`mtc_core_cpp::TimedTrajectory` 与 `MotionAdapter::execute_trajectory()` 为新增，
Python 无对应。

* **默认实现显式拒绝**（`EXECUTION_REJECTED`），**绝不允许**降级为多次
  `execute_joint_move`（需求 4 明文禁止）。
* 由 `TrajectoryIsRejectedAndNotSplitIntoJointMoves` 断言
  `adapter.call_log().empty()`（未产生任何单点调用）。
* **本阶段不新增任何 ROS 接口**；`trajectory_msgs` / `control_msgs` 绑定属于
  第二阶段（见 [`PHASE2_ROS2_INTEGRATION_PLAN.md`](PHASE2_ROS2_INTEGRATION_PLAN.md)）。

### D-8 Python 的 `_error()` 消息模板差异

C++ 的错误消息文本与 Python **不逐字相同**（Python 使用 `%` 格式化中文模板，
C++ 使用字符串拼接）。**错误码与状态转移逐项一致**，仅人类可读文本不同。

> 审计提示：任何依赖消息文本的断言都应改为依赖 `error_code`。
> C++ 测试已遵循此原则（唯一的文本断言是
> `FaultWithoutSpecificCodeDegradesToHardwareFault` 对固定英文模板的断言，
> 该模板在两侧均存在）。

### D-9 Task FSM `check_task_deadline` 的 `enter_state` 直调

Python `check_task_deadline()` 直接调用 `self._enter(TaskStateName.FAULT, ...)`
（`task_fsm.py:713`），绕过 `step()`。C++ 同样直接调用 `enter_state()`。

**差异**：Python 的 `_enter` 与 `step()` 内的 `_enter` 是同一方法 ✔；
C++ 亦然 ✔。**语义一致**，仅记录以说明此路径不经过转移表。

### D-10 审计新增的 C++ 安全契约

停止送达与关联反馈停稳证据分离；执行超时/异常保守锁定；执行器和在途守卫
使用短锁及命令世代保护，后端执行时不持执行器状态锁。FSM 本体仍由单一事件
线程所有，ROS 回调须串行入队。规划必须返回显式关节目标或完整轨迹，Fake
规划标识不可进入可执行后端，缺失规划不产生回零目标。位姿、时间、关节值和
动力学参数拒绝不适用的非有限值；长请求 ID 不截断且不同调用不重复命令 ID。
完整证据、测试名称、范围限制见 [审计修复报告](CODEX_CPP_FSM_AUDIT_FIX_REPORT.md)。

### D-11 最终安全验收的取消、时基与稳定窗口契约

2026-10-11 最终验收进一步修复了三处缺口：

* `cancel_goal(command_id, reason)` 仅取消关联 Goal，ID 不匹配在同一个状态锁内
  直接拒绝，不调用后端、不变更状态/取消标记/owner/锁定/停稳/停止计数。
  `supervisory_stop(reason)` 明确表示监督停止；旧 `request_stop` 按是否传 ID 路由。
  后端默认关联取消不再降级为全局停止。已接受的停止阻止新命令受理，至后端
  返回后才解除 pending 门禁；普通确认仍不能自动解除未知状态锁定。
* Mock 默认创建 ManualClock 并通过 execution_clock 暴露；执行器未显式给时钟时
  共享该实例。不同 ManualClock 即使起点相同也拒绝执行；显式 SteadyClock
  不能用于 Mock，真实后端不能使用 ManualClock 授权执行。静止区间和反馈
  新鲜度的严格校验不放宽，不睡眠、不伪造真实时间流逝。实际静止跨度必须
  严格大于零，比较容差限制为 min(1e-9, settle_duration_s * 1e-9)，极小正配置
  也不能接受零静止跨度。
* Python/C++ Task 与 PickPlace 均要求稳定窗口有限且严格大于零；显式0不被
  替换为默认值。内部 Task 请求缺省与 PickPlace 默认均为3.0秒，这是当前工程
  默认参数，不是比赛官方最低时间。校验 reason 统一为 stability_not_positive，
  错误码仍为100。极小正数也必须有正的实际累计时间，时间比较容差限制为
  min(1e-9, window * 1e-9)，不能令首帧的零累计时间通过。没有修改ROSIDL。

Python本轮仅同步上述稳定参数安全语义，历史缓存/规划差异仍见D-10及审计报告；
Python回归不能替代新的C++可执行契约。全部证据与准入限制见
[最终安全验收报告](CPP_FSM_FINAL_SAFETY_GATE.md)。

---

## 10. 测试用例映射

### 10.1 规模对照

下表保留原迁移报告的历史执行规模。上一轮审计为 C++ **290**、Python **258**、
显式离线端到端 **15** 项通过。本轮最终验收为 C++ **318**、Python **269**、
E2E **15** 项通过；原290个C++身份均在结果XML中核验，详见最终验收报告。

| 层 | Python 基线（本次实际执行） | C++ 新增 GTest | 状态 |
|---|---|---|---|
| Task FSM | `mtc_task` 56 例 PASS | `mtc_task_cpp` 79 例 PASS | ✔ |
| PickPlace FSM | `mtc_manipulation` 45 例 PASS | `mtc_manipulation_cpp` 102 例 PASS | ✔ |
| 运动执行 | `mtc_motion_execution` 20 例 PASS | `mtc_motion_execution_cpp` 55 例 PASS | ✔ |
| 共享核心 | —（无对应层） | `mtc_core_cpp` 21 例 PASS | ✔ |
| 安全联锁 | `mtc_safety` 70 例 PASS | 未迁移（不在第一阶段范围） | NOT RUN |
| 工具管理 | `mtc_tool` 39 例 PASS | 未迁移（不在第一阶段范围） | NOT RUN |
| AUBO 桥接 | `mtc_aubo_bridge` 25 例 PASS | 未迁移（不在第一阶段范围） | NOT RUN |
| 端到端 Mock | `tests/test_mock_end_to_end.py` 15 例 | 未迁移 | **NOT RUN**（未被任何包注册，未执行） |
| **合计** | **255 例 PASS / 15 例 NOT RUN** | **257 例 PASS** | |

### 10.2 关键场景对照表

| 场景（需求 6 列出） | Python 测试 | C++ 测试 | 结果 |
|---|---|---|---|
| 正常抓放流程 | `test_v1_happy_path_reaches_success` | `PickPlaceV1.HappyPathReachesSuccess`<br>`PickPlaceV1.StateSequenceMatchesDesignOrder` | ✔ |
| 正常任务流程 | `test_task_fsm.py`（正常路径组） | `TaskFsmBasic.FullHappyPathReachesFinished`<br>`TaskFsmSequence.ThreeItemsCompleteInSlotOrder` | ✔ |
| V1/V2 策略 | `test_v2_happy_path_reaches_success_and_unlocks_once` | `PickPlaceV2.DifferencesAreConfinedToEngageAndDisengage`<br>`PickPlaceV2.MotionPrimitivesDifferFromV1OnlyInAcquireAndEngage`<br>`ToolStrategyV1.*` / `ToolStrategyV2.*` | ✔ |
| 目标丢失 | `test_target_not_found` | `PickPlaceSafety.TargetNotFoundIsRecoverableThenFails`<br>`TargetNotUniqueIsRejected` / `StalePoseIsRejectedWithPoseStale`<br>`InvalidRingPoseBlocksMotion` / `ColorMismatchIsRejectedToPreventWrongPick` | ✔ |
| 规划失败 | `test_planning_failure*` | `PickPlaceSafety.PlanningFailureIsRecoverable`<br>`TransportPlanningFailureGoesToFaultAfterAttachment` | ✔ |
| 动作超时 | `test_motion_timeout*` | `PickPlaceSafety.MotionInFlightTimeoutGoesToFault230AndLocksResend`<br>`MotionTimeoutLocksAndDoesNotResend`<br>`NonMotionStateTimeoutReportsMotionTimeout`<br>`TaskFsmTimeout.ExecuteItemTimeoutGoesToFaultAndDoesNotAutoResend` | ✔ |
| 取消 | `test_cancel*` | `PickPlaceSafety.CancelWithConfirmedStopReachesCancelled`<br>`CancelWithoutStopConfirmationGoesToFault520`<br>`TaskFsmCancel.*`（7 例） | ✔ |
| 停止确认失败 | `test_cancel_without_stop_confirmation` | `CancelNeverReachesCancelledWithoutConfirmation`<br>`RequestStopIsCalledAtMostOnce`<br>`ExecutorCore.StopDeliveryIsNotConfirmationWhenBackendCannotConfirm`<br>`ExecutorCore.UnsupportedCancelNeverFabricatesStopConfirmed` | ✔ |
| 重复 Goal | `test_second_goal_rejected*` | `TaskFsmConcurrency.SecondGoalRejectedInEveryActiveState`（6 个活动状态遍历）<br>`RejectionDoesNotQueueSecondGoal`<br>`ExecutorCore.ConcurrentGoalDuringMotionIsRejected`<br>`InFlightGuard.SecondAcquireIsRejectedWithInFlightId` | ✔ |
| 状态未知锁定与恢复 | `test_motion_status_unknown*` | `PickPlaceSafety.MotionStatusUnknownLocksAndDoesNotResend`<br>`RecoveryIsRefusedWhenMotionResendDisallowed`<br>`TaskFsmUnsafeErrors.MotionStatusUnknownGoesToFaultWithRealCode`<br>`ExecutorCore.StatusUnknownLocksSubsequentRequests`<br>`ExecutorCore.ManualConfirmationClearsUnknownLock` | ✔ |
| `placement_verified` 唯一写入口 | `test_placement_verified_*` | `PlacementVerifiedCannotBeWrittenOutsideVerifyPlaced`<br>`PlacementVerifiedOnlyTrueAfterSuccessfulVerifyPlaced`<br>`PlacementVerifiedStartsFalseAndIsReadOnly` | ✔ |
| 稳定性窗口 | `test_placement_stability*` | `SuccessRequiresFullStabilityWindow`<br>`PlacementNotAtTargetFailsAfterWindow`<br>`PlacementStaleSampleResetsStabilityTimer`<br>`PickPlaceV1.StabilityWindowIsActuallyElapsed` | ✔ |
| V2 解锁前置条件 | `test_v2_unlock_requires_seated*` | `ToolStrategyV2.UnlockRejectedWhenSeatNotVerified`<br>`ToolStrategyV2.UnlockRejectedWhenNotUnloaded`<br>`ToolStrategyV2.UnlockAllowedOnlyWhenBothConditionsHold`<br>`PickPlaceV2Interlock.NoUnlockWhenSeatNotVerified`<br>`NoUnlockWhenUnloadNotVerified`<br>`StrategyRefusalBlocksUnlockIoEntirely` | ✔ |
| 在途命令不重发 | `test_no_resend_while_inflight` | `PickPlaceSafety.InFlightCommandIsReusedNeverResent`<br>`ProgressIsNotBlockedByInFlightPorts` | ✔ |
| 轨迹不得拆分为单点调用 | —（Python 无此能力） | `MockAdapter.TrajectoryIsRejectedAndNotSplitIntoJointMoves`<br>`MockAdapter.BaseClassDefaultTrajectoryRejectsWithoutSplitting` | ✔ |

### 10.3 C++ 独有的新增测试（无 Python 对应）

| 测试 | 验证内容 |
|---|---|
| `ErrorCodesMsgParity.ConstantsMatchInterfacesMsg` | C++ 错误码与 `ErrorCodes.msg` 原文**双向**一致（直接解析文件） |
| `CorePose.*`（6 例） | 6D 位姿保真：`frame_id`、四元数归一化、不可用即拒绝 |
| `CoreTimedTrajectory.*`（4 例） | 轨迹结构自检：时间单调、维度一致、关节名唯一 |
| `PickPlaceTables.VerifyTablesReportsNoProblem` | 处理器表覆盖全部 22 状态 + 运动在途状态必有超时 |
| `TaskFsmTables.EveryActiveStateRegistersConcurrentGoalRejection` | 每个活动状态都显式登记并发拒绝转移 |
| `ToolStrategyV1.NoUnlockPulseAndNoIoChannels` | V1 路径不触碰任何电磁 IO |
| `PickPlaceV2Interlock.RedundantInterlocksAreUnreachableButPresent` | 记录 DISENGAGE 前置联锁在正常流转下不可达这一事实 |
| `MockAdapter.*`（17 例） | Mock 适配器行为可控性（到位/超时/未知/取消确认/不确认） |

---

## 11. 逐项复核清单（供审计使用）

| 复核项 | 方法 | 期望 |
|---|---|---|
| 状态枚举数量与名称 | `git diff main -- src/mtc_task`（应为空）+ 对照 §2/§5 | 13 / 22 |
| 事件字符串 | 对照 §3 | 20 项逐字一致 |
| 转移表完整性 | 运行 `TaskFsmTables.VerifyTablesReportsNoProblem` | PASS |
| 错误码数值 | 运行 `ErrorCodesMsgParity` | PASS |
| 超时数值 | 对照 §4.4 / §5.1 | 逐项一致 |
| 安全语义 | 对照 §7 表格中的测试名并运行 | 8/8 PASS |
| 既有实现改动范围 | 相对 `feature/jazzy-cpp-fsm` 检查差异 | Python 仅 D-2、严格正稳定窗口及测试；接口仅 XML 依赖顺序，本轮接口无改动 |
| 实机连接 | 检查所有 C++ 源文件 | 无 SDK、无设备访问、无网络调用 |
