# Python 测试「行为基线」审计报告（供 C++ 迁移对照）

```yaml
审计时间: 2026-10-11
审计方式: 只读通读源码（read/grep/glob），未执行任何测试命令
执行状态: NOT RUN —— 本次审计未运行 pytest / unittest / python3，所有 Python 测试的当前运行状态一律为 NOT RUN，不得声称通过
审计者: DSH 子代理（delegated subagent）
审计对象:
  - src/mtc_task/test/test_task_fsm.py
  - src/mtc_manipulation/test/test_pick_place_fsm.py
  - src/mtc_manipulation/test/test_server_shell.py
  - src/mtc_manipulation/test/conftest.py
  - src/mtc_motion_execution/test/test_backends.py
  - src/mtc_safety/test/test_interlocks.py
  - src/mtc_tool/test/test_tool_manager.py
  - src/mtc_aubo_bridge/test/test_bridge_core.py
  - tests/test_mock_end_to_end.py
对照实现（仅用于确认断言所依赖的默认值与端口语义，未修改）:
  - src/mtc_task/mtc_task/task_fsm.py
  - src/mtc_task/mtc_task/task_validation.py
  - src/mtc_manipulation/mtc_manipulation/pick_place_fsm.py
  - src/mtc_manipulation/mtc_manipulation/fakes.py
  - src/mtc_manipulation/mtc_manipulation/tool_strategy.py
  - src/mtc_motion_execution/mtc_motion_execution/backends.py
  - src/mtc_motion_execution/mtc_motion_execution/executor_core.py
  - src/mtc_safety/mtc_safety/interlocks.py
  - src/mtc_tool/mtc_tool/{codes,io_port,manager,strategy}.py
  - src/mtc_aubo_bridge/mtc_aubo_bridge/bridge_core.py
  - src/mtc_bringup/mtc_bringup/mock_pipeline.py
变更影响: 本文件为新增文档，未修改任何源码或测试文件
```

> **运行状态声明（必读）**：本次任务仅做**静态只读审计**。所有 270 条用例的"当前运行状态"均为 **NOT RUN**。本报告中的"验证的行为"是该用例**代码所断言**的内容，不代表该断言在当前环境中已实际通过。任何引用本报告的迁移验收，必须另行实际运行测试并记录真实结果。

---

## 0. 规模总览

| 测试文件 | 行数 | 用例数 | 风格 |
|---|---|---|---|
| [test_task_fsm.py](src/mtc_task/test/test_task_fsm.py) | 833 | 56 | unittest.TestCase |
| [test_pick_place_fsm.py](src/mtc_manipulation/test/test_pick_place_fsm.py) | 893 | 40 | pytest 模块级函数 + `load_tests` 兼容层 |
| [test_server_shell.py](src/mtc_manipulation/test/test_server_shell.py) | 417 | 5 | pytest 模块级函数 + 替身模块注入 |
| [conftest.py](src/mtc_manipulation/test/conftest.py) | 18 | 0 | **纯 sys.path 助手，无 fixture** |
| [test_backends.py](src/mtc_motion_execution/test/test_backends.py) | 268 | 20 | unittest.TestCase |
| [test_interlocks.py](src/mtc_safety/test/test_interlocks.py) | 775 | 70 | unittest.TestCase |
| [test_tool_manager.py](src/mtc_tool/test/test_tool_manager.py) | 661 | 39 | unittest.TestCase |
| [test_bridge_core.py](src/mtc_aubo_bridge/test/test_bridge_core.py) | 237 | 25 | unittest.TestCase |
| [test_mock_end_to_end.py](tests/test_mock_end_to_end.py) | 919 | 15（case_01–case_15） | 自建 Harness（PASS/FAIL/SKIP） |
| **合计** | 5081 | **270** | — |

---

## 1. 测试用例总表（逐用例，共 270 条）

### 1.1 `src/mtc_task/test/test_task_fsm.py`（56 例）

| 测试文件 | 测试函数 | 验证行为 | 涉及状态/错误码 |
|---|---|---|---|
| test_task_fsm.py | TestNoRosDependency.test_ros_modules_not_imported | `rclpy`/`mtc_interfaces`/`rcl_interfaces` 均不在 `sys.modules` | — |
| test_task_fsm.py | TestNoRosDependency.test_core_import_does_not_need_ros | 核心模块属性中无 `rclpy`/`Node`/`ActionServer` | — |
| test_task_fsm.py | TestTableIntegrity.test_every_state_has_entry_handler_and_timeout | 每个状态都在 `ENTRY_ACTIONS` 与 `DEFAULT_STATE_TIMEOUTS` 中 | ALL_STATES |
| test_task_fsm.py | TestTableIntegrity.test_no_duplicate_transition_keys | 转移表无重复 `(state,event)` | TRANSITIONS |
| test_task_fsm.py | TestTableIntegrity.test_guard_names_are_centralised | 守卫名以 `_guard_` 开头且方法真实存在 | — |
| test_task_fsm.py | TestTableIntegrity.test_all_guards_referenced_by_table | 表中守卫集合 == 实现守卫集合（双向无遗漏） | — |
| test_task_fsm.py | TestTableIntegrity.test_required_transitions_present | 9 条必需转移存在且目标状态正确 | WAIT_TASK→VALIDATE_TASK、VALIDATE_TASK→TASK_FAILED、SCAN_SCENE→EXECUTE_ITEM/RECOVERY、EXECUTE_ITEM→RECORD_RESULT、RECORD_RESULT→FINISHED、CANCEL_PENDING→CANCELLED/FAULT、RECOVERY→TASK_FAILED |
| test_task_fsm.py | TestTableIntegrity.test_lifecycle_init_to_wait_task | 生命周期 INIT→start()→SELF_CHECK→SELF_CHECK_OK→WAIT_TASK | INIT/SELF_CHECK/WAIT_TASK |
| test_task_fsm.py | TestTableIntegrity.test_self_check_failure_goes_fault | 自检失败 → FAULT，且动作含 REQUEST_STOP + LOCK_ACTIONS | SELF_CHECK_FAIL → FAULT |
| test_task_fsm.py | TestBasicTaskSuccess.test_basic_single_color_to_t0 | 单色基础任务全流程：VALIDATE→SCAN→EXECUTE→RECORD→FINISHED，slot=T0，completed_count=1 | OK(0)，completed_slots==["T0"] |
| test_task_fsm.py | TestBasicTaskSuccess.test_pick_place_request_fields_match_action_contract | 进入 EXECUTE_ITEM 时 `call_pick_place` payload 字段与 PickPlace.action Goal 完全一致 | 字段集合 == {request_id, object_id, expected_color, target_slot, placement_stability_sec}；request_id=="pp-1#0" |
| test_task_fsm.py | TestBasicTaskSuccess.test_reset_after_finished_allows_next_task | FINISHED 后 reset() 回 WAIT_TASK，清空任务与 completed_slots，可再受理新 Goal | FINISHED/WAIT_TASK/VALIDATE_TASK |
| test_task_fsm.py | TestSequenceTaskSuccess.test_sequence_three_colors_to_p1_p2_p3 | 三色序列 slot 固定 P1/P2/P3，逐项 current_index 0/1/2，最终 completed_count=3 | FINISHED，OK(0) |
| test_task_fsm.py | TestSequenceTaskSuccess.test_sequence_index_advances_only_on_record | 索引只在 `record_completed_item` 后推进（完成前保持 0） | RECORD_RESULT |
| test_task_fsm.py | TestSequenceTaskSuccess.test_slot_mapping_is_derived_from_mode | slot 由 mode 推导：SEQUENCE→[P1,P2,P3]，BASIC→[T0] | MODE_SEQUENCE/MODE_BASIC |
| test_task_fsm.py | TestInvalidTaskRejection.test_empty_colors | 空颜色 → TASK_FAILED，reason=empty_colors | INVALID_TASK(100) |
| test_task_fsm.py | TestInvalidTaskRejection.test_four_colors_rejected | 序列任务 4 色 → TASK_FAILED，reason=sequence_count | INVALID_TASK(100) |
| test_task_fsm.py | TestInvalidTaskRejection.test_basic_with_three_colors_rejected | 基础任务 3 色 → TASK_FAILED，reason=basic_count | INVALID_TASK(100) |
| test_task_fsm.py | TestInvalidTaskRejection.test_duplicate_colors_rejected | 重复颜色 → reason=duplicate_color | INVALID_TASK(100) |
| test_task_fsm.py | TestInvalidTaskRejection.test_illegal_color_value_rejected | 颜色值 0/5/99 → reason=illegal_color | INVALID_TASK(100) |
| test_task_fsm.py | TestInvalidTaskRejection.test_timeout_ms_zero_rejected | timeout_ms=0 → reason=timeout_not_positive | INVALID_TASK(100) |
| test_task_fsm.py | TestInvalidTaskRejection.test_illegal_target_slot_rejected | 7 种非法 target_slot（T1/P1/t0/SOMEWHERE/P4/序列 T0/序列错序 P2）→ reason ∈ {illegal_target_slot, target_slot_mismatch} | INVALID_TASK(100) |
| test_task_fsm.py | TestInvalidTaskRejection.test_legal_target_slot_accepted | (BASIC,[BLUE],"T0") 与 (SEQUENCE,[BLUE,RED,YELLOW],"P1") 被接受 | SCAN_SCENE |
| test_task_fsm.py | TestInvalidTaskRejection.test_empty_task_id_rejected_without_state_change | 空白 task_id 在校验前即拒，状态不变，reason=empty_task_id | WAIT_TASK，INVALID_TASK(100) |
| test_task_fsm.py | TestInvalidTaskRejection.test_unknown_mode_rejected | mode=7 → `validate_task_request` 返回 ok=False，reason=unknown_mode | INVALID_TASK(100) |
| test_task_fsm.py | TestInvalidTaskRejection.test_validation_does_not_guess_or_complete | 非法口令不猜测/不补全：无 `call_pick_place`，`has_active_task` False | TASK_FAILED |
| test_task_fsm.py | TestInvalidTaskRejection.test_valid_task_passes_validation | 合法序列校验通过且 `normalized_slots==["P1","P2","P3"]` | ok=True |
| test_task_fsm.py | TestConcurrentGoalRejection.test_second_goal_rejected_while_executing | 执行中第二个 Goal 被拒：goal_rejected=True、状态不变、reason=task_already_in_progress、message 含 "not queued"、原任务继续 | EXECUTE_ITEM，last_rejection 含 active/rejected_task_id |
| test_task_fsm.py | TestConcurrentGoalRejection.test_second_goal_rejected_in_scan_and_record_and_cancel | SCAN_SCENE / RECORD_RESULT / CANCEL_PENDING 三个阶段均拒绝第二个 Goal | reason=task_already_in_progress |
| test_task_fsm.py | TestConcurrentGoalRejection.test_goal_accepted_after_previous_task_reset | FINISHED 不算进行中（`has_active_task` False），reset 后可受理 | FINISHED→WAIT_TASK→VALIDATE_TASK |
| test_task_fsm.py | TestCancelSemantics.test_cancel_confirmed_leads_to_cancelled | 取消先 CANCEL_PENDING + CANCEL_CHILD，确认停稳才 CANCELLED；取消不算成功 | CANCEL_PENDING→CANCELLED，error_code OK(0)，success=False |
| test_task_fsm.py | TestCancelSemantics.test_cancel_not_confirmed_times_out_to_fault_520 | 覆盖超时 5.0s：4.0s 不转移，5.5s 转移 FAULT，动作 REQUEST_STOP+LOCK_ACTIONS | FAULT，CANCEL_NOT_CONFIRMED(520) |
| test_task_fsm.py | TestCancelSemantics.test_cancel_from_scan_scene_and_record_result | SCAN_SCENE 与 RECORD_RESULT 阶段均可进入 CANCEL_PENDING | CANCEL_PENDING |
| test_task_fsm.py | TestCancelSemantics.test_cancel_while_waiting_is_ignored | 空闲 WAIT_TASK 时取消不引入 CANCEL_PENDING | WAIT_TASK |
| test_task_fsm.py | TestCancelSemantics.test_late_stop_confirmation_after_fault_does_not_cancel | FAULT 后迟到的停稳确认不得改成 CANCELLED（覆盖 1.0s，推进 2.0s） | FAULT，last_error_code=520 |
| test_task_fsm.py | TestScanRecoveryBudget.test_target_not_found_retries_then_fails | 进入 RECOVERY 不消耗预算；request_retry 获批后 attempts=1；耗尽后 TASK_FAILED | TARGET_NOT_FOUND(110)，recovery_max_attempts=1 |
| test_task_fsm.py | TestScanRecoveryBudget.test_recovery_max_attempts_zero_fails_immediately | `recovery_max_attempts=0` 时首次 request_retry 立即失败 | RECOVERY→TASK_FAILED |
| test_task_fsm.py | TestScanRecoveryBudget.test_recovery_budget_is_reset_per_item | 每项重置重试预算（第 2 项仍有独立预算） | recovery_attempts 归 0 后 +1 |
| test_task_fsm.py | TestScanRecoveryBudget.test_failed_placement_placement_not_verified_is_not_success | `success=True` 但 `placement_verified=False` 不得记完成 | RECOVERY，PLACEMENT_FAILED(430)，completed_count=0 |
| test_task_fsm.py | TestScanRecoveryBudget.test_target_found_without_object_id_is_rejected | 空 object_id 不得进入 EXECUTE_ITEM | SCAN_SCENE，TARGET_NOT_FOUND(110) |
| test_task_fsm.py | TestScanRecoveryBudget.test_expected_color_mismatch_is_rejected | 期望色与实际色不符 → 留在 SCAN_SCENE | INVALID_TASK(100) |
| test_task_fsm.py | TestStageTimeout.test_cancel_pending_timeout_already_covered | 占位（指向 TestCancelSemantics） | — |
| test_task_fsm.py | TestStageTimeout.test_scan_scene_timeout_goes_to_recovery | SCAN_SCENE 覆盖 5.0s，推进 5.5s → RECOVERY | TARGET_NOT_FOUND(110) |
| test_task_fsm.py | TestStageTimeout.test_execute_item_timeout_goes_to_fault_with_motion_timeout | EXECUTE_ITEM 覆盖 10.0s：9.0s 不转移，10.5s → FAULT，禁止自动重发 | FAULT，MOTION_TIMEOUT(220) |
| test_task_fsm.py | TestStageTimeout.test_validate_task_timeout_goes_to_task_failed | VALIDATE_TASK 覆盖 2.0s，2.5s → TASK_FAILED | TASK_FAILED |
| test_task_fsm.py | TestStageTimeout.test_no_timeout_configured_never_fires | WAIT_TASK 无超时：推进 10000.0s 不转移 | WAIT_TASK |
| test_task_fsm.py | TestStageTimeout.test_task_level_deadline_goes_to_fault | 任务级 timeout_ms=3000：2.0s 不触发，3.5s → FAULT | FAULT，MOTION_TIMEOUT(220) |
| test_task_fsm.py | TestStageTimeout.test_timeout_does_not_fire_after_leaving_state | 离开状态后计时重置（SCAN 4.0s 后进 EXECUTE_ITEM，check_timeouts 返回 None） | EXECUTE_ITEM |
| test_task_fsm.py | TestMotionStatusUnknown.test_motion_status_unknown_goes_fault_without_retry | 230 → FAULT，不消耗重试预算、不自动重发（无 CALL_PICK_PLACE），动作含 REQUEST_STOP+LOCK_ACTIONS | FAULT，MOTION_STATUS_UNKNOWN(230) |
| test_task_fsm.py | TestMotionStatusUnknown.test_recoverable_error_does_enter_recovery | PLANNING_FAILED 可恢复：进 RECOVERY 不消耗预算，重试获批后 +1 | RECOVERY→SCAN_SCENE，PLANNING_FAILED(200) |
| test_task_fsm.py | TestMotionStatusUnknown.test_other_non_recoverable_errors_go_fault | 220/500/510/410/320 五类错误全部 → FAULT 且错误码原样保留 | FAULT，MOTION_TIMEOUT/SAFETY_INTERLOCK/HARDWARE_FAULT/UNLOCK_FAILED/OBJECT_DROPPED |
| test_task_fsm.py | TestMotionStatusUnknown.test_error_code_names_match_interface | 错误码数值与名称映射 | 100/110/230/520，`name_of(520)=="CANCEL_NOT_CONFIRMED"` |
| test_task_fsm.py | TestStateSnapshotAndEvents.test_snapshot_tracks_progress | snapshot 记录 task_id/state/total_count/completed_count | state=="VALIDATE_TASK"，3/0 |
| test_task_fsm.py | TestStateSnapshotAndEvents.test_publish_state_emitted_on_every_transition | 每次转移产生 PUBLISH_STATE，payload.snapshot.state 与当前状态一致 | VALIDATE_TASK |
| test_task_fsm.py | TestStateSnapshotAndEvents.test_transition_log_records_history | transition_log 记录 WAIT_TASK/VALIDATE_TASK/SCAN_SCENE | — |
| test_task_fsm.py | TestStateSnapshotAndEvents.test_unknown_event_is_ignored_safely | 未知事件被安全忽略：状态不变、actions 为空 | WAIT_TASK，actions==[] |

### 1.2 `src/mtc_manipulation/test/test_pick_place_fsm.py`（40 例）

| 测试文件 | 测试函数 | 验证行为 | 涉及状态/错误码 |
|---|---|---|---|
| test_pick_place_fsm.py | test_v1_happy_path_reaches_success | V1 全流水线状态序列严格相等；7 条原语；无 unlock；command_id 唯一 | SUCCESS，OK(0) |
| test_pick_place_fsm.py | test_v2_happy_path_reaches_success_and_unlocks_once | V2 差异仅在 ENGAGE/DISENGAGE：engage 1 次、unlock 1 次、auto_lock、electromagnetic_unlock 事件、withdraw | SUCCESS，OK(0) |
| test_pick_place_fsm.py | test_target_not_found | 感知未找到 → 失败，零运动指令，RECOVERY 被访问一次 | FAILED，TARGET_NOT_FOUND(110) |
| test_pick_place_fsm.py | test_pose_stale | 位姿过期 → 失败，零运动 | FAILED，POSE_STALE(120) |
| test_pick_place_fsm.py | test_planning_failed | 规划失败 → 失败，零运动，未进入 MOVE_PRE_ALIGN | FAILED，PLANNING_FAILED(200) |
| test_pick_place_fsm.py | test_invalid_slot_rejected | slot="X9" → 失败，零运动 | FAILED，INVALID_TASK(100) |
| test_pick_place_fsm.py | test_engage_failed_without_load_is_retried_then_failed | 确认无载荷时可退回重试一次：ENGAGE 进入 2 次、recovery_attempts==2，禁止进入 LIFT/TRANSPORT/MOVE_PRE_SEAT/SEAT | FAILED，ENGAGE_FAILED(300) |
| test_pick_place_fsm.py | test_engage_low_evidence_is_failure_not_attach | 仅 COMMAND_ONLY 证据 → 失败，且未进入 TEST_LIFT | ENGAGE_FAILED(300) |
| test_pick_place_fsm.py | test_attach_not_verified_blocks_transport | 挂载证据不足 → 失败，禁止 LIFT/TRANSPORT，`object_attached_estimated` False | FAILED，ATTACH_NOT_VERIFIED(310) |
| test_pick_place_fsm.py | test_attach_not_verified_with_uncertain_load_never_retries | 试提失败（载荷不确定）：仅 1 条运动指令 ["side_insert"]，无 RECOVERY，attempts==0 | ATTACH_NOT_VERIFIED(310) |
| test_pick_place_fsm.py | test_vision_follow_can_confirm_attachment | 视觉随动可独立确认挂载 → 成功并进入 LIFT | SUCCESS |
| test_pick_place_fsm.py | test_seat_not_verified_blocks_unload_and_disengage | V2 未落座 → FAULT，禁止 UNLOAD/DISENGAGE/RETREAT/VERIFY_PLACED/SUCCESS，unload/unlock/disengage 调用均为 0 | FAULT，SEAT_NOT_VERIFIED(400) |
| test_pick_place_fsm.py | test_seat_not_verified_v1_also_faults | V1 未落座同样 FAULT 且无 DISENGAGE | FAULT，400 |
| test_pick_place_fsm.py | test_unlock_failed_faults_without_retreat | 解锁失败 → FAULT，unlock 调用 1 次，禁止 RETREAT/VERIFY_PLACED | FAULT，UNLOCK_FAILED(410) |
| test_pick_place_fsm.py | test_unlock_accepted_but_not_released_faults | 受理但锁止状态未知（`unlock_released=None`）→ FAULT，不撤离 | FAULT，410 |
| test_pick_place_fsm.py | test_release_not_verified_faults | 分离未验证 → FAULT，disengage 1 次，禁止 RETREAT/SUCCESS | FAULT，RELEASE_NOT_VERIFIED(420) |
| test_pick_place_fsm.py | test_motion_status_unknown_faults_and_does_not_resend | 230 → FAULT，仅 1 次调用且 command_id 相同，`motion_resend_allowed` False，再 step 5 次不重发 | FAULT，MOTION_STATUS_UNKNOWN(230) |
| test_pick_place_fsm.py | test_motion_timeout_mid_transport_faults | 搬运中 transport 超时 → FAULT，不重发、不自动解锁 | FAULT，MOTION_TIMEOUT(220) |
| test_pick_place_fsm.py | test_execution_rejected_maps_to_210_without_resend | 执行被拒 → 210，lift 仅 1 次，未进入 TRANSPORT | EXECUTION_REJECTED(210) |
| test_pick_place_fsm.py | test_cancel_confirmed_becomes_cancelled | 取消确认 → CANCELLED，无 SUCCESS，无自动解锁，arrive_calls 与 calls 等长 | CANCELLED，OK(0)，placement_verified False |
| test_pick_place_fsm.py | test_cancel_not_confirmed_faults_with_520 | 永不确认停稳 → FAULT；不得立即声称安全停止；CANCELLED 不出现在 CANCEL_PENDING 之后 | FAULT，CANCEL_NOT_CONFIRMED(520) |
| test_pick_place_fsm.py | test_cancel_before_motion_does_not_claim_success | 运动前取消 → CANCELLED，success/placement_verified 均 False | CANCELLED |
| test_pick_place_fsm.py | test_placement_stability_requires_full_window | 稳定窗口 3.0s：成功时 3.0 <= stable_for <= 3.2 | SUCCESS，placement_verified True |
| test_pick_place_fsm.py | test_placement_stability_longer_window_is_respected | 窗口 6.0s：6.0 <= stable_for <= 6.2（max_steps 2500） | SUCCESS |
| test_pick_place_fsm.py | test_placement_inside_window_does_not_declare_success_too_early | 逐步检查（4000 步）未满窗口前绝不出现 SUCCESS，首次成功时 3.0<=observed<=3.2 | VERIFY_PLACED→SUCCESS |
| test_pick_place_fsm.py | test_placement_not_at_target_fails_with_430 | 未落到目标区 → FAILED | PLACEMENT_FAILED(430) |
| test_pick_place_fsm.py | test_placement_observation_reset_clears_stability_timer | 反例样本到达时 `_stability_started_at` 清空、累计归 0，且已累计的 1.0s 不被沿用 | VERIFY_PLACED，`stability_verified_for_s==0.0` |
| test_pick_place_fsm.py | test_placement_verified_flag_written_only_in_verify_placed | 2000 步内 `placement_verified is False or state is VERIFY_PLACED`；成功时 True 且路径含 VERIFY_PLACED | VERIFY_PLACED/SUCCESS |
| test_pick_place_fsm.py | test_v1_never_touches_electromagnetic_io | V1 无 unlock 调用、io_events 无 electromagnetic/unlock；策略 `IO_CHANNELS==()`、`requires_unlock_pulse()` False、`evaluate_unlock(True,True).allowed` False | SUCCESS |
| test_pick_place_fsm.py | test_v1_unlock_request_is_always_rejected | V1 策略级解锁恒被拒，理由含 "无解锁机构" | allowed=False |
| test_pick_place_fsm.py | test_v2_unlock_rejected_when_not_seated | 未落座 → 无 unlock 调用、无 electromagnetic 事件 | SEAT_NOT_VERIFIED(400) |
| test_pick_place_fsm.py | test_v2_unlock_rejected_when_not_unloaded | 未卸载 → FAULT，无 unlock 调用，无 DISENGAGE | RELEASE_NOT_VERIFIED(420) |
| test_pick_place_fsm.py | test_v2_strategy_rejects_unlock_without_preconditions | 策略层三段判定：未落座 400、未卸载 420、满足时 allowed 且 pulse_ms>0 | 400/420/OK |
| test_pick_place_fsm.py | test_fsm_defensive_interlock_rejects_unlock_without_seated_evidence | FSM 级防御联锁：人为破坏落座证据后即便在 DISENGAGE 也不调用解锁 IO | FAULT，400 |
| test_pick_place_fsm.py | test_tool_type_mismatch_is_rejected | 策略与 config.tool_type 不一致时构造抛 ValueError，消息含 "不一致" | — |
| test_pick_place_fsm.py | test_fake_motion_supports_delay_and_inflight_results | `delay_ticks=2` 制造"未完成"（端口返回 None）仍可达 SUCCESS | SUCCESS |
| test_pick_place_fsm.py | test_force_fault_locks_actions_and_never_resends | 外部监护 force_fault → FAULT、resend False；锁定后 5 次 step 不发运动/不接合；终态 force_fault 不覆盖错误码 | FAULT，230（不被 510 覆盖） |
| test_pick_place_fsm.py | test_color_mismatch_blocks_motion_and_reports_110 | 期望 BLUE 实际 RED → 110，零运动，未进入 MOVE_PRE_ALIGN/PLAN_ACQUIRE | TARGET_NOT_FOUND(110) |
| test_pick_place_fsm.py | test_unknown_color_blocks_motion | 感知颜色 0（COLOR_UNKNOWN）→ 110，零运动，未进入 PLAN_ACQUIRE | 110 |
| test_pick_place_fsm.py | test_matching_color_still_reaches_planning | 颜色匹配时新增校验不误伤，可到达 PLAN_ACQUIRE | PLAN_ACQUIRE |

### 1.3 `src/mtc_manipulation/test/test_server_shell.py`（5 例）

| 测试文件 | 测试函数 | 验证行为 | 涉及状态/错误码 |
|---|---|---|---|
| test_server_shell.py | test_server_module_imports_and_constructs | Action Server 名 `"/mtc/manipulation/pick_place"`；构造期零发布；`stop()` 后 `destroyed is True` | — |
| test_server_shell.py | test_server_executes_full_pick_place_cycle | 完整抓放闭环：success/placement_verified True、error_code 0、outcome "succeeded"、feedback[-1].substate=="SUCCESS" 且 progress==1.0；末条发布 topic `"/mtc/tool/state"`、tool_type `"passive_hook_v1"`、detail 含 "SUCCESS" | SUCCESS，OK(0) |
| test_server_shell.py | test_server_cancel_path_never_claims_success | 收到 cancel 即请求取消：success/placement_verified False，outcome "canceled" | CANCELLED 路径 |
| test_server_shell.py | test_server_invalid_goal_is_aborted | target_slot="ZZZ" → 中止，error_code 100 | INVALID_TASK(100)，outcome "aborted" |
| test_server_shell.py | test_ros_adapters_are_instantiable_with_stubs | 真实 ROS 适配器类可用替身实例化；空缓存保守返回未找到；`plan("ACQUIRE",{})` ok；`wait_for_server=False` 判为状态未知而非成功；未接真实 tool_manager 时 engage 保守失败、`holding_state()=="unknown"` | `status_unknown=True`，detection.found False |

### 1.4 `src/mtc_manipulation/test/conftest.py`（0 例）

无任何 pytest fixture。仅在导入期把"含 `mtc_manipulation/pick_place_fsm.py` 的目录"插入 `sys.path`，使包可在未 colcon build 情况下以源码导入，且不导入任何 ROS 模块。

### 1.5 `src/mtc_motion_execution/test/test_backends.py`（20 例）

| 测试文件 | 测试函数 | 验证行为 | 涉及状态/错误码 |
|---|---|---|---|
| test_backends.py | TestValidation.test_joint_order_mismatch_rejected | 关节顺序颠倒 → REJECTED，message 含 "顺序" | EXECUTION_REJECTED(210) |
| test_backends.py | TestValidation.test_length_mismatch_rejected | 关节数 5 个 → 拒绝 | 210 |
| test_backends.py | TestValidation.test_velocity_scaling_over_limit_is_interlock | velocity 0.9 > 上限 0.2 → 联锁，message 含 "上限" | 500 |
| test_backends.py | TestValidation.test_joint_limit_violation_is_interlock | 目标 99.0 rad 越界 → 联锁 | 500 |
| test_backends.py | TestValidation.test_zero_timeout_rejected | timeout_ms=0 → 拒绝 | 210 |
| test_backends.py | TestExecution.test_success_path_settles | 成功路径 SETTLED，stop_confirmed True，final_position_rad 精准等于目标 | SETTLED，error 0 |
| test_backends.py | TestExecution.test_concurrent_command_rejected | InFlightGuard 第二次 acquire 失败，理由含 "在途命令" | — |
| test_backends.py | TestExecution.test_second_command_rejected_while_in_flight | 有在途命令时第二请求 → REJECTED，message 含 "并发" | REJECTED |
| test_backends.py | TestExecution.test_trajectory_not_split_into_multiple_calls | 一次请求 == 恰好一次后端调用（`backend.call_log` 长度 1） | — |
| test_backends.py | TestStopSemantics.test_stop_when_idle_confirms | 空闲停止：delivered True 且 confirmed True | — |
| test_backends.py | TestStopSemantics.test_cancel_without_backend_confirmation_is_not_confirmed | 后端不确认停稳：delivered True、confirmed False、message 含 "未"；结果 `motion_state_unknown True`、success False | 未知状态标记 |
| test_backends.py | TestStopSemantics.test_cancel_with_backend_confirmation | 后端确认停稳：confirmed True 且结果 `stop_confirmed True` | — |
| test_backends.py | TestUnknownState.test_status_unknown_locks_new_commands | 230 → 锁定：`retry_allowed_without_reobservation` False；后续命令仍 230；后端仅被调用 1 次 | MOTION_STATUS_UNKNOWN(230) |
| test_backends.py | TestUnknownState.test_timeout_is_unknown_and_not_retryable | 超时 → TIMEOUT、230、运动状态未知、不可重试（travel_time_s=10.0，timeout=1000ms） | TIMEOUT，220/230 |
| test_backends.py | TestUnknownState.test_manual_clear_unlocks | 人工确认清除后锁解除（后端调用数 1→2） | `clear_unknown_after_manual_confirmation` |
| test_backends.py | TestUnknownState.test_backend_exception_becomes_unknown | 后端抛 RuntimeError → 230，message 含 "RuntimeError" | 230 |
| test_backends.py | TestDisabledBackend.test_disabled_backend_refuses_everything | Disabled 后端三项能力全 False | — |
| test_backends.py | TestDisabledBackend.test_unknown_backend_name_falls_back_to_disabled | 'real'/'aubo'/''/'aubo_sdk'/'gazebo' 一律回落到 DisabledMotionBackend | — |
| test_backends.py | TestDisabledBackend.test_mock_backend_declares_no_trajectory_capability | mock：joint_goal True；timed_trajectory/cartesian 均 False | — |
| test_backends.py | TestHandshakeLog.test_handshake_records_steps | 握手步骤序列含 PREPARE/ACCEPT/VALIDATE/SEND/MONITOR:SETTLED/CONFIRM:settled | — |

### 1.6 `src/mtc_safety/test/test_interlocks.py`（70 例）

| 测试文件 | 测试函数 | 验证行为 | 涉及状态/错误码 |
|---|---|---|---|
| test_interlocks.py | TestAllowMotion.test_not_ready_rejected | 非 READY 拒绝运动 | SAFETY_INTERLOCK(500) |
| test_interlocks.py | TestAllowMotion.test_disconnected_rejected | DISCONNECTED + link_ok False 拒绝 | 500 |
| test_interlocks.py | TestAllowMotion.test_stop_requested_and_stopped_rejected | STOP_REQUESTED / STOPPED 均拒绝 | 500 |
| test_interlocks.py | TestAllowMotion.test_controller_mode_mismatch_rejected | 控制器模式不符，理由含 "模式不符" | 500 |
| test_interlocks.py | TestAllowMotion.test_simulation_mode_mismatch_rejected | 仿真标志与期望不符拒绝 | 500 |
| test_interlocks.py | TestAllowMotion.test_state_expired_rejected | 状态过期（NOW-5.0，ttl 0.5），理由含 "过期" | 500 |
| test_interlocks.py | TestAllowMotion.test_state_never_received_rejected | 从未收到状态（last_state_ts=None）拒绝 | 500 |
| test_interlocks.py | TestAllowMotion.test_motion_status_unknown_rejected | 运动状态未知优先返回 230 | MOTION_STATUS_UNKNOWN(230) |
| test_interlocks.py | TestAllowMotion.test_fault_device_rejected | 设备 FAULT 拒绝 | 500 |
| test_interlocks.py | TestAllowMotion.test_ready_allows | 一切就绪允许，错误码 0 | OK(0) |
| test_interlocks.py | TestAllowMotion.test_expired_state_invalidates_ready | READY 不代表安全：ttl 置 0.0 后 mark_state(NOW-1.0) 即被拒 | 500 |
| test_interlocks.py | TestTransport.test_attach_not_verified_rejected | 未验证挂载拒绝运输 | ATTACH_NOT_VERIFIED(310) |
| test_interlocks.py | TestTransport.test_verified_but_payload_unknown_rejected | 已验证但载荷未知仍拒绝 | 310 |
| test_interlocks.py | TestTransport.test_verified_carrying_allows | 已验证 + CARRYING 允许 | allowed |
| test_interlocks.py | TestUnlock.test_payload_unknown_rejected_with_500 | 载荷未知 → 统一 500 | 500 |
| test_interlocks.py | TestUnlock.test_seat_not_verified_rejected_with_400 | 未落座 → 400（与载荷未知区分） | SEAT_NOT_VERIFIED(400) |
| test_interlocks.py | TestUnlock.test_unlock_without_payload_rejected | 无载荷时解锁拒绝 | 500 |
| test_interlocks.py | TestUnlock.test_unlock_while_moving_rejected | 运动在途（cmd-7）拒绝解锁 | 500 |
| test_interlocks.py | TestUnlock.test_unlock_with_unknown_motion_rejected | 运动状态未知拒绝解锁 | 500 |
| test_interlocks.py | TestUnlock.test_seated_carrying_allows | 已落座 + CARRYING 允许 | allowed |
| test_interlocks.py | TestForceWithdraw.test_release_not_verified_rejected | 释放未验证拒绝硬拉 | RELEASE_NOT_VERIFIED(420) |
| test_interlocks.py | TestForceWithdraw.test_motion_unknown_rejected | 运动未知优先 230 | 230 |
| test_interlocks.py | TestForceWithdraw.test_release_verified_allows | 已验证允许 | allowed |
| test_interlocks.py | TestSecondMotionGoal.test_in_flight_goal_rejected | 在途命令拒绝第二个运动 Goal | EXECUTION_REJECTED(210) |
| test_interlocks.py | TestSecondMotionGoal.test_device_moving_rejected | 设备 MOVING 拒绝 | 210 |
| test_interlocks.py | TestSecondMotionGoal.test_unknown_motion_state_rejected | 运动未知返回 230 | 230 |
| test_interlocks.py | TestSecondMotionGoal.test_idle_allows | 空闲 READY 允许 | allowed |
| test_interlocks.py | TestNextTaskItem.test_cancel_not_confirmed_rejected | 未确认停稳禁止进入下一项 | CANCEL_NOT_CONFIRMED(520) |
| test_interlocks.py | TestNextTaskItem.test_unknown_motion_state_rejected | 运动未知优先 230 | 230 |
| test_interlocks.py | TestNextTaskItem.test_unknown_payload_rejected | 载荷未知禁止下一项 | 500 |
| test_interlocks.py | TestNextTaskItem.test_still_carrying_rejected | 仍携带电池禁止下一项 | 500 |
| test_interlocks.py | TestNextTaskItem.test_stopped_and_empty_allows | 已停稳且空载允许 | allowed |
| test_interlocks.py | TestNextTaskItem.test_unconditional_home_blocked_after_cancel | 未确认停稳时禁止无条件 Home | 520 |
| test_interlocks.py | TestResendMotion.test_no_evidence_rejected_by_default | 默认无"从未执行"证据 → 禁止重发 | 210 |
| test_interlocks.py | TestResendMotion.test_motion_status_unknown_rejected | 运动未知禁止重发 | 230 |
| test_interlocks.py | TestResendMotion.test_may_have_executed_rejected | 可能已执行运动禁止重发 | 230 |
| test_interlocks.py | TestResendMotion.test_unknown_takes_precedence_over_other_evidence | 即使伪造 retry_evidence/stop_confirmed，未知仍优先 230 | 230 |
| test_interlocks.py | TestResendMotion.test_stop_not_confirmed_rejected | 已记录"发送前被拒"但停稳未确认 → 仍拒绝 | 520 |
| test_interlocks.py | TestResendMotion.test_explicit_reject_before_motion_allows | 唯一放行条件：明确"发送前被拒、从未开始运动" | allowed，OK(0) |
| test_interlocks.py | TestResendMotion.test_exception_path_locks_resend | 异常路径锁定重发；人工核验解除未知后仍不允许重发 | 230 → 仍拒绝 |
| test_interlocks.py | TestFaultLatch.test_latch_and_block | 锁存后 `fault_latched` True，运动被拒 | 500 |
| test_interlocks.py | TestFaultLatch.test_first_cause_retained | 保留首因，history 长度 2 | — |
| test_interlocks.py | TestFaultLatch.test_auto_path_cannot_clear | 自动路径不得清除；clear_attempts 记录 accepted False | — |
| test_interlocks.py | TestFaultLatch.test_human_clear_requires_note | 空白备注不得清除 | — |
| test_interlocks.py | TestFaultLatch.test_human_clear_records_note_and_timestamp | 记录 note、时间戳 NOW+7.0、cleared_fault.error_code | HARDWARE_FAULT(510) |
| test_interlocks.py | TestFaultLatch.test_guard_allows_after_human_clear | 人工清除后恢复允许 | 520→allowed |
| test_interlocks.py | TestFaultLatch.test_all_guards_blocked_while_latched | 7 个守卫方法（含 home/next/resend）在锁存下全部拒绝 | 全部 500 |
| test_interlocks.py | TestFaultLatch.test_resend_blocked_while_latched | 即使有 retry_evidence，锁存仍禁止重发 | 500 |
| test_interlocks.py | TestPayloadStateMachine.test_unknown_is_most_conservative | UNKNOWN 下解锁/Home/下一项全部 False | — |
| test_interlocks.py | TestPayloadStateMachine.test_confirm_requires_evidence | 空/空白证据不得改变状态，保持 UNKNOWN | — |
| test_interlocks.py | TestPayloadStateMachine.test_transitions | CARRYING→可解锁/可下一项/不可 Home；NONE→不可解锁/可 Home；mark_unknown→全 False | CARRYING/NONE/UNKNOWN |
| test_interlocks.py | TestPayloadStateMachine.test_unknown_payload_blocks_unlock_and_home_on_guard | 载荷未知时守卫层解锁与 Home 均 500 | 500 |
| test_interlocks.py | TestPureMappings.test_device_state_precedence | 7 组输入的状态推导优先级 | DISCONNECTED/FAULT/STOP_REQUESTED/MOVING/STOPPED/READY/NOT_READY |
| test_interlocks.py | TestPureMappings.test_payload_from_tool_state | 5 组映射：state 是估计、verified 独立维度、COMMAND_ONLY 不算证据 | CARRYING/NONE/UNKNOWN |
| test_interlocks.py | TestWatchdogCore.test_missing_states_trigger_stop_requests | 从未收到状态 → 两条停止请求原因；送达 True 绝不等于停稳；每事件含 timeouts 字典 | 0.5/1.0/2.0s |
| test_interlocks.py | TestWatchdogCore.test_repeated_reason_is_idempotent | 相同原因不重复发停止请求 | — |
| test_interlocks.py | TestWatchdogCore.test_stop_confirmed_only_from_evidence | 仅由 `stop_confirmed=True` 证据置位，置位后 tick 无事件 | — |
| test_interlocks.py | TestWatchdogCore.test_stale_state_resets_stop_confirmed | 状态超时后 stop_confirmed 复位为 False | robot_state_timeout / tool_state_timeout |
| test_interlocks.py | TestWatchdogCore.test_communication_lost_and_safety_false | 通信丢失与安全标志 False 各自产生原因 | robot_communication_lost / safety_normal_false |
| test_interlocks.py | TestWatchdogCore.test_stop_confirm_timeout_escalates_to_fault_latch | 3.0s > 2.0s 升级：恰 1 条 stop_confirm_timeout 事件、error_code 520、锁存、守卫拒绝 | CANCEL_NOT_CONFIRMED(520) |
| test_interlocks.py | TestWatchdogCore.test_tool_fault_latches_guard | 工具 FAULT 锁存守卫，原因码 510 | HARDWARE_FAULT(510) |
| test_interlocks.py | TestWatchdogCore.test_motion_unknown_reason_and_guard_sync | 未知状态同步到守卫并禁止重发 | 230 |
| test_interlocks.py | TestWatchdogCore.test_log_fields_contain_required_keys | log_fields 含 stop_requested/stop_confirmed/timeouts（3 个键名排序） | — |
| test_interlocks.py | TestWatchdogCore.test_note_stop_delivery_does_not_confirm_stop | 记录送达不产生停稳确认 | — |
| test_interlocks.py | TestStopClientAdapter.test_plain_callable | 普通可调用返回 (True,'ok:timeout') | — |
| test_interlocks.py | TestStopClientAdapter.test_callable_returning_object | 返回对象取 request_delivered/message | — |
| test_interlocks.py | TestStopClientAdapter.test_object_with_sync_call | 同步 client `.call` → (True,'ok')，记录原因 | — |
| test_interlocks.py | TestStopClientAdapter.test_object_with_call_async_is_not_treated_as_confirmed | 异步 client：送达未知（None），message 含 'unknown' | — |
| test_interlocks.py | TestStopClientAdapter.test_none_client | None → None | — |
| test_interlocks.py | TestStopClientAdapter.test_watchdog_with_fake_client_end_to_end | fake client 注入看门狗：请求已送达但未停稳 | stop_requested True / stop_confirmed False |

### 1.7 `src/mtc_tool/test/test_tool_manager.py`（39 例）

| 测试文件 | 测试函数 | 验证行为 | 涉及状态/错误码 |
|---|---|---|---|
| test_tool_manager.py | TestUnlockPreconditions.test_seated_and_unloaded_required | 三种组合（仅落座/仅卸载/都无）→ 拒绝且 IO 端口零调用 | SAFETY_INTERLOCK(500) |
| test_tool_manager.py | TestUnlockPreconditions.test_seated_evidence_flag_must_be_explicitly_true | 证据标志复位后必须先重新确认 | 500，io.call_count 0 |
| test_tool_manager.py | TestUnlockPreconditions.test_unlock_requires_releasing_stage | 处于 ATTACHED（未 begin_release）即使置位两标志也拒绝 | 500 |
| test_tool_manager.py | TestUnlockPreconditions.test_invalid_pulse_ms_rejected_without_io | pulse_ms ∈ {0,-1,1.5,"300",None} → 拒绝且无 IO | EXECUTION_REJECTED(210) |
| test_tool_manager.py | TestAcceptedIsNotUnlock.test_accepted_true_does_not_confirm_unlock | accepted≠解锁：状态 RELEASING、证据 COMMAND_ONLY、unlock_confirmed/withdraw_allowed False | STATE_RELEASING(4)，EVIDENCE_COMMAND_ONLY(1) |
| test_tool_manager.py | TestAcceptedIsNotUnlock.test_command_only_evidence_cannot_confirm_unlock | 仅命令级证据 → 保守 FAULT，之后仍不得抽出 | UNLOCK_FAILED(410)、FAULT(5)、RELEASE_NOT_VERIFIED(420) |
| test_tool_manager.py | TestAcceptedIsNotUnlock.test_geometry_evidence_confirms_unlock | GEOMETRY 证据确认解锁，但分离未验证前仍禁止抽出；分离确认后 DETACHED | OK(0)→DETACHED(1) |
| test_tool_manager.py | TestAcceptedIsNotUnlock.test_confirm_unlock_without_accepted_pulse_is_unlock_failed | 未受理脉冲即确认 → 410 + FAULT | 410/FAULT |
| test_tool_manager.py | TestAcceptedIsNotUnlock.test_service_contract_uses_accepted_not_success | `TriggerUnlock.srv` 响应字段恰为 `["accepted","message"]`，无 success/unlocked/released | — |
| test_tool_manager.py | TestV1NeverUsesUnlockIo.test_full_v1_cycle_touches_no_io | V1 全流程 IO 零调用、零脉冲历史 | STATE_DETACHED(1) |
| test_tool_manager.py | TestV1NeverUsesUnlockIo.test_v1_unlock_request_rejected_by_interlock | V1 解锁请求被拒且不调用 IO | 500 |
| test_tool_manager.py | TestV1NeverUsesUnlockIo.test_v1_strategy_declares_no_unlock_io | V1 策略不要求脉冲、不使用解锁 IO | tool_type=="passive_hook_v1" |
| test_tool_manager.py | TestDisengageFailureBlocksWithdraw.test_withdraw_before_disengage_resolution_is_blocked | 分离未决前抽出被拒 | 420，FAULT |
| test_tool_manager.py | TestDisengageFailureBlocksWithdraw.test_disengage_failed_blocks_withdraw | 分离失败禁止抽出 | 420，FAULT |
| test_tool_manager.py | TestDisengageFailureBlocksWithdraw.test_disengage_unknown_blocks_withdraw | 分离未知禁止抽出 | 420，FAULT |
| test_tool_manager.py | TestDisengageFailureBlocksWithdraw.test_disengage_confirmed_without_evidence_is_rejected | 确认但无独立证据 → 拒绝 | 420，FAULT |
| test_tool_manager.py | TestDisengageFailureBlocksWithdraw.test_v2_cannot_claim_release_without_confirmed_unlock | 未确认解锁不得声称释放 | UNLOCK_FAILED(410)，FAULT |
| test_tool_manager.py | TestDisengageFailureBlocksWithdraw.test_fault_requires_explicit_operator_clear | FAULT 后禁止新任务；空备注抛 ValueError；人工清除后回 UNKNOWN | STATE_UNKNOWN(0) |
| test_tool_manager.py | TestVerifiedIsIndependentDimension.test_attached_with_command_only_is_not_verified | V1/V2 均：state 估计为 ATTACHED 但 verified False | STATE_ATTACHED(3)，COMMAND_ONLY |
| test_tool_manager.py | TestVerifiedIsIndependentDimension.test_geometry_evidence_sets_verified | GEOMETRY 证据置 verified True | EVIDENCE_GEOMETRY(2) |
| test_tool_manager.py | TestVerifiedIsIndependentDimension.test_verified_and_state_are_separate_fields | 同一 state 对应两种 verified，证明字段独立 | ATTACHED |
| test_tool_manager.py | TestVerifiedIsIndependentDimension.test_manual_attach_evidence_below_threshold_is_310 | 证据不足 → 310 且禁止进入释放/运输 | ATTACH_NOT_VERIFIED(310) |
| test_tool_manager.py | TestNoJointLevelOutput.test_plans_contain_only_motion_primitives | 计划仅含 MotionPrimitive；原语无 joints/joint_names/target_rad/torque/sdk_handle | — |
| test_tool_manager.py | TestNoJointLevelOutput.test_manager_and_strategy_expose_no_joint_api | manager/strategy 不暴露关节级 API（正则 move_joint/send_joint/set_joint/servo_j/follow_joint/move_linear） | — |
| test_tool_manager.py | TestNoJointLevelOutput.test_core_modules_have_no_joint_or_sdk_tokens | 5 个核心模块源码不含 15 个禁止关键字（关节/SDK/网络） | — |
| test_tool_manager.py | TestNoJointLevelOutput.test_plans_generated_from_pose_only | V2 抓取原语序列与阶段、释放原语序列、解锁原语零速且需接触 | PRE_ALIGN/ENGAGE |
| test_tool_manager.py | TestNormalFlows.test_v1_normal_cycle | V1 正常闭环：前两条原语、末态 DETACHED、verified、error 0、IO 零调用 | DETACHED，OK(0) |
| test_tool_manager.py | TestNormalFlows.test_v2_normal_cycle | V2 正常闭环：受理 300ms 脉冲、确认解锁、分离确认、withdraw_allowed、IO 历史 (300,) | DETACHED，OK(0)，call_count 1 |
| test_tool_manager.py | TestNormalFlows.test_state_machine_sequence_is_monotonic | 状态单调推进至 DETACHED，stage=="DISENGAGE" | RELEASING→DETACHED |
| test_tool_manager.py | TestUnlockPortSafety.test_mock_reject_and_timeout_are_not_acceptance | reject/timeout 均不视为受理 → 410 + FAULT，端口被调用 1 次 | 410，FAULT |
| test_tool_manager.py | TestUnlockPortSafety.test_mock_raise_is_mapped_to_unlock_failed | 端口抛异常映射为 410 + FAULT | 410，FAULT |
| test_tool_manager.py | TestUnlockPortSafety.test_disabled_io_strict_never_touches_hardware | strict 端口 → 410、io_status "DISABLED"、FAULT；直接调用抛 UnlockIoDisabledError | 410，FAULT |
| test_tool_manager.py | TestUnlockPortSafety.test_disabled_io_lenient_returns_explicit_refusal | lenient 端口明确返回 False、last_status DISABLED、call_count 1 | — |
| test_tool_manager.py | TestUnlockPortSafety.test_default_manager_uses_disabled_io | 默认 ToolManager 使用 DisabledUnlockIo 且 strict=True | — |
| test_tool_manager.py | TestIdlFidelity.test_tool_state_constants_match | `ToolState.msg` 的 STATE_*/EVIDENCE_* 常量与 codes.py 逐一相等，且数量一致 | — |
| test_tool_manager.py | TestIdlFidelity.test_error_codes_match | `ErrorCodes.msg` 常量与 `codes.ERROR_*` 逐一相等 | — |
| test_tool_manager.py | TestIdlFidelity.test_tool_state_fields_are_covered | `to_tool_state_fields()` 是 msg 字段子集；接合后字段值正确 | ATTACHED，COMMAND_ONLY，object_id |
| test_tool_manager.py | TestIdlFidelity.test_package_metadata_exists | package.xml/setup.py/setup.cfg/README.md/resource/mtc_tool 齐备 | — |
| test_tool_manager.py | TestIdlFidelity.test_node_shell_fields_and_safety_defaults | 静态核对节点源码：msg./request./response. 字段名与 IDL 一致；默认禁用真实 IO 并告警；无其它 IO 后端名 | — |

### 1.8 `src/mtc_aubo_bridge/test/test_bridge_core.py`（25 例）

| 测试文件 | 测试函数 | 验证行为 | 涉及状态/错误码 |
|---|---|---|---|
| test_bridge_core.py | TestDefaultDisabled.test_default_mode_is_disabled | 7 种模式名（含大写 REAL、gazebo）一律回落 DisabledSdkWorker | — |
| test_bridge_core.py | TestDefaultDisabled.test_disabled_worker_declares_no_capability | disabled worker 四项能力全 False | — |
| test_bridge_core.py | TestDefaultDisabled.test_disabled_bridge_refuses_motion | disabled 桥接拒绝运动 | EXECUTION_REJECTED(210) |
| test_bridge_core.py | TestDefaultDisabled.test_disabled_bridge_refuses_unlock | disabled 桥接拒绝解锁，消息含 "电磁铁" | SAFETY_INTERLOCK(500) |
| test_bridge_core.py | TestDefaultDisabled.test_disabled_worker_never_reports_healthy | rpc_ok/rtde_ok/usable_for_motion 全 False | — |
| test_bridge_core.py | TestIdentityAndHealth.test_identity_requires_verified_whitelist | 未配置白名单必须拒绝，理由含 "白名单" | — |
| test_bridge_core.py | TestIdentityAndHealth.test_identity_mismatch_rejected | 序列号不符拒绝，理由含 "不匹配" | — |
| test_bridge_core.py | TestIdentityAndHealth.test_identity_match_passes | 白名单匹配通过 | — |
| test_bridge_core.py | TestIdentityAndHealth.test_unverified_actual_identity_rejected | 实际身份 `verified=False` 不得通过 | — |
| test_bridge_core.py | TestIdentityAndHealth.test_stale_feedback_blocks_motion | 反馈年龄 5.0s（>0.5s 上限）拒绝，理由含 "反馈不新鲜" | — |
| test_bridge_core.py | TestIdentityAndHealth.test_rtde_down_blocks_motion | RTDE 断链拒绝，理由含 "RTDE" | — |
| test_bridge_core.py | TestIdentityAndHealth.test_motion_blocked_when_feedback_stale | 反馈 10.0s 陈旧时运动被拒，消息含 "不新鲜" | — |
| test_bridge_core.py | TestCommandSemantics.test_successful_move | 正常运动受理且 error_code 0 | OK(0) |
| test_bridge_core.py | TestCommandSemantics.test_duplicate_command_id_rejected | 重复 command_id 拒绝，消息含 "重复" | — |
| test_bridge_core.py | TestCommandSemantics.test_empty_command_id_rejected | 空 command_id 拒绝 | — |
| test_bridge_core.py | TestCommandSemantics.test_unsupported_capability_rejected | 能力不支持时拒绝，消息含 "joint_goal_supported" | — |
| test_bridge_core.py | TestCommandSemantics.test_unknown_state_locks_further_commands | 未知状态锁定后续命令，消息含 "未确认" | `motion_state_unknown=True` |
| test_bridge_core.py | TestCommandSemantics.test_stop_not_confirmed_keeps_lock | 未确认停稳时不得解除不确定锁定 | — |
| test_bridge_core.py | TestCommandSemantics.test_stop_confirmed_alone_cannot_clear_lock | **仅确认停稳不足以解除锁定**，必须完成设备侧状态对账 | `state_reconciled is False` |
| test_bridge_core.py | TestCommandSemantics.test_stop_clears_lock_only_when_reconciled | 无未知且确认停稳 → `state_reconciled True` | — |
| test_bridge_core.py | TestCommandSemantics.test_manual_confirmation_is_the_only_automatic_escape | 人工确认是唯一出路，审计轨迹含 "manual_confirmation" | — |
| test_bridge_core.py | TestUnlockSemantics.test_unlock_requires_seated_and_unloaded | 未落座（消息含 "落座"）/未卸载（含 "载荷"）拒绝；成功消息含 "不代表已解锁" | — |
| test_bridge_core.py | TestUnlockSemantics.test_unlock_rejected_without_tool_io_capability | 无 tool_io 能力拒绝 | 500 |
| test_bridge_core.py | TestUnlockSemantics.test_zero_pulse_rejected | pulse_ms=0 拒绝 | — |
| test_bridge_core.py | TestAuditTrail.test_audit_records_linkable_ids | 审计轨迹含 command_intent/sdk_response/result，且可回溯 command_id | — |

### 1.9 `tests/test_mock_end_to_end.py`（15 例）

| 测试文件 | 测试函数 | 验证行为 | 涉及状态/错误码 |
|---|---|---|---|
| test_mock_end_to_end.py | case_01_sequence_three_colors | 蓝→红→黄：slots 严格 == ["P1","P2","P3"]；每步 SUCCESS + placement_verified；outcome.completed_slots 一致；V1 解锁 IO 0 次 | SUCCESS，P1/P2/P3 |
| test_mock_end_to_end.py | case_02_concurrent_goals_rejected | Task Goal 拒绝 reason=task_already_in_progress、error_code=210、message 含 "not queued"、状态不变；Motion Goal 被并发守卫拒绝 | 210 |
| test_mock_end_to_end.py | case_03_bad_target_never_moves | 4 类感知缺陷（颜色错/重复 ID/位姿过期/ring_pose 不可用）在 `recovery_max_attempts=0` 下后端调用恒为 0；重试对照同样 0 次 | 颜色错∈(100,110)、重复 ID=110、过期=120、无 ring_pose∈(110,120) |
| test_mock_end_to_end.py | case_04_execution_failure_not_success | 注入规划成功但底层执行失败：PickPlace 不得成功、不得 placement_verified、终态 ∈ (FAILED,FAULT)、整轮 success False、completed_count 0 | HARDWARE_FAULT(510) 注入 |
| test_mock_end_to_end.py | case_05_timeout_no_resend | 超时 → FAULT(220)、resend 禁止；再推进 40 次 step 后端调用数不变且 `_step_count` 增长；`snapshot().stop_confirmed is False` | FAULT，MOTION_TIMEOUT(220) |
| test_mock_end_to_end.py | case_06_test_lift_not_followed_blocks_transport | 试提未随动 → 310；未进入 TRANSPORT/LIFT；`object_attached_estimated` False | ATTACH_NOT_VERIFIED(310) |
| test_mock_end_to_end.py | case_07_not_seated_no_unlock | 未落座：解锁 IO 0 次、tool.unlock 0 次、未进入 UNLOAD/DISENGAGE；ToolManager 联锁亦拒绝 | SEAT_NOT_VERIFIED(400) |
| test_mock_end_to_end.py | case_08_unlock_accepted_not_released_no_retreat | 解锁脉冲受理恰 1 次但锁止未解除 → FAULT(410)、disengage 0 次、禁止 RETREAT、ToolManager unlock_confirmed/withdraw_allowed 均 False | FAULT，UNLOCK_FAILED(410) |
| test_mock_end_to_end.py | case_09_cancel_while_carrying | 搬运中取消：CANCEL_PENDING→CANCELLED、success False、stop_confirmed True、保留 carrying 估计、holding_state()=="holding"、无自动解锁、无 RETREAT；对照组停止未确认 → 520、停止请求恰 1 次、不得 CANCELLED/SUCCESS | CANCELLED，CANCEL_NOT_CONFIRMED(520) |
| test_mock_end_to_end.py | case_10_capability_change_invalidates_plan | 控制器模式/TCP/负载变化 → 旧规划作废(200)，且不得悄悄写回新指纹；能力查询 joint_goal True、轨迹/笛卡尔 False；作废通路不得声明运动能力 | PLANNING_FAILED(200) |
| test_mock_end_to_end.py | case_11_non_target_in_sweep_zone | 非目标进入扫掠区 → 500 且后端 0 次，规划器记录 sweep_zone_occupied*；目标自身不误伤（对照 SUCCESS） | SAFETY_INTERLOCK(500) |
| test_mock_end_to_end.py | case_12_stability_window_not_met | 逐帧观测：1.0s/2.0s 均停留 VERIFY_PLACED 且 placement_verified False；距阈值 0.05s 仍不判成功；再 +0.1s 才 SUCCESS | VERIFY_PLACED→SUCCESS，阈值 3.0s |
| test_mock_end_to_end.py | case_13_v1_has_no_unlock_io | V1 三色序列完成，解锁 IO 0 次、脉冲历史空、tool.unlock 0 次；误调用亦被 DISABLED 端口拒绝 | last_status=="DISABLED" |
| test_mock_end_to_end.py | case_14_config_cross_check | 读取真实 `config/*.yaml` 交叉校验：tool_type 属两代之一、stability>0、perception_max_age>0、设备白名单留空 | §8 配置约定 |
| test_mock_end_to_end.py | case_15_interlock_guard | 联锁：未验证挂载禁运输、载荷未知禁解锁、缺"从未执行"证据禁重发、状态不确定禁撤离；在途时禁第二个运动的 Goal；未知时禁实机运动 | 见 §2.4/§2.7 |

---

## 2. 行为基线清单（必须由 C++ 实现保持的可观察行为）

**优先级定义**
- **P0（必须保持）**：安全语义或对外契约。C++ 实现若改变这些行为，视为安全回归或接口破坏，迁移验收不得放行。
- **P1（建议保持）**：可观察但非安全关键的实现细节（日志字段、消息文案、内部计数、测试钩子）。若确有必要变更，必须同步更新测试与接口变更记录。

### 2.1 正常抓放流程的确定性状态序列

| 编号 | 行为 | 优先级 | 来源 |
|---|---|---|---|
| B-1.1 | PickPlace 成功路径状态序列**逐元素严格相等**：RESOLVE_TARGET→PLAN_ACQUIRE→MOVE_PRE_ALIGN→ENGAGE→VERIFY_ENGAGEMENT→TEST_LIFT→VERIFY_ATTACHED→LIFT→TRANSPORT→MOVE_PRE_SEAT→SEAT→VERIFY_SEATED→UNLOAD→DISENGAGE→RETREAT→VERIFY_PLACED→SUCCESS | **P0** | test_pick_place_fsm.py:197-216 |
| B-1.2 | `states_history` 完整记录实际访问序列，不得跳变或漏记；`VERIFY_PLACED` 必然出现 | P0 | 同上 |
| B-1.3 | 每条运动指令 `command_id` 全局唯一 | P1 | test_pick_place_fsm.py:230 |
| B-1.4 | Task 层：WAIT_TASK→VALIDATE_TASK→SCAN_SCENE→EXECUTE_ITEM→RECORD_RESULT→FINISHED；多项时 RECORD_RESULT 回 SCAN_SCENE，最后一项才 FINISHED | **P0** | test_task_fsm.py:222-245 |
| B-1.5 | `call_pick_place` payload 字段集合精确等于 `{request_id, object_id, expected_color, target_slot, placement_stability_sec}` | **P0** | test_task_fsm.py:256-259 |
| B-1.6 | `request_id` 格式 `<task_id>#<index>`（如 `pp-1#0`） | P1 | test_task_fsm.py:263 |
| B-1.7 | slot 由 mode 推导：BASIC→`["T0"]`，SEQUENCE→`["P1","P2","P3"]`，且与颜色次序一一对应 | **P0** | test_task_fsm.py:328-331, 290-296 |
| B-1.8 | `current_index` 只在 `record_completed_item` 后推进；completed_* 只在真正完成时增加 | P0 | test_task_fsm.py:316-325 |
| B-1.9 | snapshot：VALIDATE_TASK 时 total_count==3、completed_count==0；PUBLISH_STATE 每次转移产生且 state 与转移后一致 | P1 | test_task_fsm.py:801-814 |
| B-1.10 | 未知事件被安全忽略：状态不变、actions 为空 | P1 | test_task_fsm.py:825-829 |
| B-1.11 | FINISHED 后 reset 回 WAIT_TASK 并清空 current_task/completed_slots，可受理新 Goal | P0 | test_task_fsm.py:265-278, 501-512 |
| B-1.12 | 生命周期：INIT→start()→SELF_CHECK→SELF_CHECK_OK→WAIT_TASK | P0 | test_task_fsm.py:201-208 |
| B-1.13 | SELF_CHECK_FAIL → FAULT，且必须带 REQUEST_STOP + LOCK_ACTIONS | **P0** | test_task_fsm.py:210-216 |
| B-1.14 | Action Server 名 `"/mtc/manipulation/pick_place"`；构造阶段零发布；stop() 释放 | **P0** | test_server_shell.py:289-293 |
| B-1.15 | 成功时 outcome=="succeeded"、feedback 末帧 substate=="SUCCESS"、progress==1.0 | P1 | test_server_shell.py:307-314 |
| B-1.16 | 工具状态发布 topic `"/mtc/tool/state"`，tool_type 与配置一致，detail 含终态名 | P1 | test_server_shell.py:316-320 |

### 2.2 V1 / V2 策略差异点

| 编号 | 行为 | 优先级 | 来源 |
|---|---|---|---|
| B-2.1 | **正常流水线状态序列两代完全相同**，差异只体现在 ENGAGE/DISENGAGE 内部 | **P0** | test_pick_place_fsm.py:241 |
| B-2.2 | V1 抓取原语前两条为 `side_insert_pre_align`, `side_insert`；接合原语 `hook_engage` | **P0** | test_tool_manager.py:479 |
| B-2.3 | V1 实测全流程 7 条原语：`side_insert, test_lift, lift, transport, pre_seat, seat_contact, retreat` | **P0** | test_pick_place_fsm.py:218-226 |
| B-2.4 | V1 释放原语不含任何 unlock：`seat_unload, withdraw, retreat` | **P0** | test_tool_manager.py:461-465 |
| B-2.5 | V1 端口 IO 事件恰为 `["ENGAGE:hook_engage", "disengage:side_withdraw"]` | P1 | test_pick_place_fsm.py:228 |
| B-2.6 | V2 抓取原语：`top_insert_pre_align, top_insert, auto_lock`（阶段 PRE_ALIGN, ENGAGE, ENGAGE） | **P0** | test_tool_manager.py:458-460 |
| B-2.7 | V2 释放原语：`seat_unload, electromagnetic_unlock, withdraw, retreat` | **P0** | test_tool_manager.py:462-465 |
| B-2.8 | V2 解锁原语 `max_linear_speed_m_s == 0.0` 且 `requires_contact is True` | **P0** | test_tool_manager.py:466-468 |
| B-2.9 | V2 正常流程 engage_calls==1、unlock_calls==1、首个原语=="auto_lock"、含 "withdraw"、不含 "side_withdraw" | **P0** | test_pick_place_fsm.py:242-247 |
| B-2.10 | V1 `unlock_calls == 0`（全流程与误调用均为 0）；`requires_unlock_pulse()` False；`uses_unlock_io` False；`IO_CHANNELS == ()` | **P0** | test_pick_place_fsm.py:227,682-684；test_tool_manager.py:292-296 |
| B-2.11 | V1 策略 `evaluate_unlock(seated,unloaded)` 恒 `allowed=False`，理由含 `"无解锁机构"` | **P0** | test_pick_place_fsm.py:689-693 |
| B-2.12 | V2 策略 `evaluate_unlock`：未落座→400、未卸载→420、满足→allowed 且 pulse_ms>0 | **P0** | test_pick_place_fsm.py:724-738 |
| B-2.13 | 策略与 config.tool_type 不一致时构造必须抛错（Python 为 ValueError，消息含 `"不一致"`） | **P0** | test_pick_place_fsm.py:768-782 |
| B-2.14 | 工具层只产出 MotionPrimitive，绝不携带 joints/joint_names/target_rad/torque/sdk_handle；manager/strategy 不暴露关节级 API | **P0** | test_tool_manager.py:409-451 |
| B-2.15 | V1 默认 IO 端口为 DisabledUnlockIo(strict=False)：误调用返回 False 且 `last_status=="DISABLED"` | P0 | mock_pipeline.py:780；E2E case_13 |
| B-2.16 | V2 正常流程 IO 历史 `(300,)`（pulse 300ms），`call_count==1` | P1 | test_tool_manager.py:512-513 |

### 2.3 目标丢失 / 规划失败 / 动作超时 / 取消 / 停止确认失败

| 编号 | 行为 | 优先级 | 来源 |
|---|---|---|---|
| B-3.1 | 感知缺陷（found=False / 重复 ID / 颜色不匹配 / ring_pose 不可用）→ 一律失败且**零运动指令** | **P0** | test_pick_place_fsm.py:255-301, 836-859；E2E case_03 |
| B-3.2 | 感知拒绝必须先于规划：未访问 PLAN_ACQUIRE/MOVE_PRE_ALIGN | **P0** | test_pick_place_fsm.py:267,292,847 |
| B-3.3 | 错误码映射：found/重复/无 ring_pose→110；stale→120；颜色不匹配→100 | **P0** | 同上；mock_pipeline.py:379-413 |
| B-3.4 | Task 层 `report_target("")` 留 SCAN_SCENE + 110；期望色不符留 SCAN_SCENE + 100 | P0 | test_task_fsm.py:652-666 |
| B-3.5 | RECOVERY 进入**不消耗**预算（recovery_attempts 保持 0）；request_retry 获批后 +1；耗尽→TASK_FAILED(110) 且 has_active_task False | **P0** | test_task_fsm.py:589-622 |
| B-3.6 | 重试预算按项重置 | **P0** | test_task_fsm.py:624-639 |
| B-3.7 | PLANNING_FAILED(200) 可恢复 → RECOVERY；重试获批回 SCAN_SCENE | P0 | test_task_fsm.py:762-771 |
| B-3.8 | 规划指纹（控制器模式/TCP/负载）变化 → 旧规划作废报 200，且不得把新指纹写回 | **P0** | E2E case_10 |
| B-3.9 | 运动在途状态超时 → FAULT(220)，禁止重发、禁止自动解锁 | **P0** | test_pick_place_fsm.py:482-493 |
| B-3.10 | Task 层 EXECUTE_ITEM 超时 → FAULT(220)；任务级 timeout_ms 到期 → FAULT(220) | **P0** | test_task_fsm.py:685-699, 716-726 |
| B-3.11 | 运动执行层超时 → ExecState.TIMEOUT + 220 + motion_state_unknown True + 不可重试 | **P0** | test_backends.py:197-206 |
| B-3.12 | 超时后进入 FAULT，继续 step 不得产生任何新后端调用；`snapshot().stop_confirmed` 仍 False | **P0** | E2E case_05 |
| B-3.13 | 取消必须**先** CANCEL_PENDING 并发 CANCEL_CHILD，不得立即返回成功 | **P0** | test_task_fsm.py:518-527；test_pick_place_fsm.py:541-544 |
| B-3.14 | CANCELLED 不得出现在 CANCEL_PENDING 之后的位置（语义：不能"边取消边成功"） | P0 | test_pick_place_fsm.py:543-544 |
| B-3.15 | SCAN_SCENE/EXECUTE_ITEM/RECORD_RESULT 可取消；空闲 WAIT_TASK 取消被忽略 | P0 | test_task_fsm.py:557-571 |
| B-3.16 | 确认停稳 → CANCELLED，error_code==0 但 success=False、placement_verified=False | **P0** | test_task_fsm.py:532-536 |
| B-3.17 | 取消路径不得自动解锁、不得自动 Home、不得继续 RETREAT；arrive_calls 与 calls 等长 | **P0** | test_pick_place_fsm.py:524-526；E2E case_09 |
| B-3.18 | 不确定负载保护：取消后保留 object_attached_estimated=True / holding_state()=="holding" | **P0** | E2E case_09 |
| B-3.19 | 停止确认失败 → FAULT(520)，stop_confirmed False，不出现 CANCELLED | **P0** | test_pick_place_fsm.py:529-544 |
| B-3.20 | Task 层 CANCEL_PENDING 超时 → FAULT(520) + REQUEST_STOP + LOCK_ACTIONS | **P0** | test_task_fsm.py:538-555 |
| B-3.21 | FAULT 后迟到的停稳确认不得改回 CANCELLED，last_error_code 保持 520 | **P0** | test_task_fsm.py:573-583 |
| B-3.22 | 安全层 allow_next_task_item / allow_unconditional_home 在 stop_confirmed False 时返回 520 | **P0** | test_interlocks.py:264-297 |
| B-3.23 | 看门狗 stop_confirm_timeout 升级为故障锁存，error_code=520；送达 True 绝不等于停稳 | **P0** | test_interlocks.py:631-649, 686-692 |
| B-3.24 | 已记录"发送前被拒"但停稳未确认 → 仍拒绝重发，错误码 520（优先于 210） | P0 | test_interlocks.py:331-337 |
| B-3.25 | 分离失败/未知/确认但无证据 → 一律 420 且 FAULT，禁止抽出 | **P0** | test_tool_manager.py:309-347 |
| B-3.26 | 解锁失败/受理但未释放 → 410 且 FAULT，禁止 RETREAT/VERIFY_PLACED | **P0** | test_pick_place_fsm.py:420-442 |
| B-3.27 | 未落座 → 400 且 FAULT，禁止 UNLOAD/DISENGAGE/RETREAT/VERIFY_PLACED/SUCCESS | **P0** | test_pick_place_fsm.py:388-412 |

### 2.4 重复 Goal 并发拒绝

| 编号 | 行为 | 优先级 | 来源 |
|---|---|---|---|
| B-4.1 | Task 层第二个 Goal：goal_rejected=True、**状态不变**、不排队 | **P0** | test_task_fsm.py:461-475 |
| B-4.2 | `last_rejection["reason"]=="task_already_in_progress"`、`error_code==EXECUTION_REJECTED(210)` | **P0** | test_task_fsm.py:472；E2E case_02 |
| B-4.3 | 拒绝消息必须含 `"not queued"` | P1 | test_task_fsm.py:475；E2E case_02 |
| B-4.4 | 拒绝记录含 active_task_id / rejected_task_id / active_state | P1 | test_task_fsm.py:473-474 |
| B-4.5 | SCAN_SCENE / EXECUTE_ITEM / RECORD_RESULT / CANCEL_PENDING 四阶段行为一致 | **P0** | test_task_fsm.py:483-499 |
| B-4.6 | FINISHED 不算进行中；reset 后可受理 | **P0** | test_task_fsm.py:501-512 |
| B-4.7 | 运动执行层：在途命令存在时第二请求 → ExecState.REJECTED，错误码 210，消息含"并发"/"在途" | **P0** | test_backends.py:120-127；E2E case_02 |
| B-4.8 | 单请求 == 恰好一次后端调用（禁止把轨迹拆成高频调用） | **P0** | test_backends.py:129-134 |
| B-4.9 | 桥接层：重复 command_id 拒绝（含"重复"）；空 command_id 拒绝；未知状态未对账时新命令拒绝（含"未确认"） | **P0** | test_bridge_core.py:126-153 |
| B-4.10 | 安全层 allow_second_motion_goal：在途/设备 MOVING → 210；运动未知 → 230；空闲 READY → 允许 | **P0** | test_interlocks.py:240-258 |

### 2.5 状态未知锁定与恢复

| 编号 | 行为 | 优先级 | 来源 |
|---|---|---|---|
| B-5.1 | status_unknown 或 timed_out → 230/220；运动执行层 motion_state_unknown=True；后端异常同样映射为 230 | **P0** | test_backends.py:181-229 |
| B-5.2 | 230 → FAULT，recovery_attempts 保持 0；不得进入 RECOVERY 消耗预算 | **P0** | test_task_fsm.py:741-757 |
| B-5.3 | 锁定后任何新命令一律 230，且后端调用次数不增加 | **P0** | test_backends.py:191-195 |
| B-5.4 | 恢复唯一出口：人工确认（note 必须非空白，空白抛 ValueError / 返回 False） | **P0** | test_backends.py:208-216；test_tool_manager.py:349-356 |
| B-5.5 | **仅确认停稳不足以解除锁定**：bridge 必须 state_reconciled==False，motion_state_unknown 保持 True | **P0** | test_bridge_core.py:163-171 |
| B-5.6 | 只有"无未知且确认停稳"才 state_reconciled==True | **P0** | test_bridge_core.py:173-178 |
| B-5.7 | 解除未知后**仍不允许重发**（缺"从未开始运动"证据） | **P0** | test_interlocks.py:348-357 |
| B-5.8 | 唯一放行重发条件：note_command_rejected_before_motion（明确发送前被拒） | **P0** | test_interlocks.py:339-346 |
| B-5.9 | 未知优先级最高：即使伪造 retry_evidence/stop_confirmed，allow_resend_motion 仍 230 | **P0** | test_interlocks.py:320-329 |
| B-5.10 | 故障锁存只能人工清除；自动路径 try_auto_clear 必须失败并记录 accepted=False | **P0** | test_interlocks.py:378-385 |
| B-5.11 | 人工清除需非空白 note，记录 note/timestamp/cleared_fault.error_code | P1 | test_interlocks.py:387-404 |
| B-5.12 | 锁存期间全部 7 个守卫方法（motion/transport/unlock/force_withdraw/second_motion_goal/next_task_item/unconditional_home）返回 500 | **P0** | test_interlocks.py:414-423 |
| B-5.13 | 工具 FAULT → 看门狗锁存 510；停稳超时 → 锁存 520 | **P0** | test_interlocks.py:631-657 |
| B-5.14 | 载荷状态 UNKNOWN 为最保守态：禁止解锁、无条件 Home、下一项 | **P0** | test_interlocks.py:437-441, 465-469 |
| B-5.15 | payload 推导：state 是估计、verified 独立维度、COMMAND_ONLY（以及 FAULT）一律 UNKNOWN | **P0** | test_interlocks.py:508-530 |
| B-5.16 | 看门狗 log_fields 含 stop_requested/stop_confirmed/timeouts | P1 | test_interlocks.py:673-684 |
| B-5.17 | 相同原因的停止请求幂等（不重复发） | P1 | test_interlocks.py:588-595 |

### 2.6 `placement_verified` 唯一写入口约束

| 编号 | 行为 | 优先级 | 来源 |
|---|---|---|---|
| B-6.1 | 唯一写入口方法内含断言：仅 `state is VERIFY_PLACED`（或等值字符串）可写入 | **P0** | pick_place_fsm.py:792-798 |
| B-6.2 | 唯一调用点在 VERIFY_PLACED 分支 `stable_for + 1e-9 >= placement_stability_sec` 之后 | **P0** | pick_place_fsm.py:1371-1378 |
| B-6.3 | 2000 步内恒有 `placement_verified is False or state is VERIFY_PLACED`；成功时 True 且路径含 VERIFY_PLACED | **P0** | test_pick_place_fsm.py:649-663 |
| B-6.4 | PickPlace 报 success=True 但 placement_verified=False → Task 层必须失败（430），completed_count 不增加 | **P0** | test_task_fsm.py:641-650 |
| B-6.5 | 稳定性窗口默认 3.0s；成功时 stable_for ∈ [阈值, 阈值+0.2s] | **P0** | test_pick_place_fsm.py:561-584 |
| B-6.6 | 反例样本（过期/不在目标区/未静止）必须把累计计时清零（起点置空、stability_verified_for_s 归 0） | **P0** | test_pick_place_fsm.py:625-646 |
| B-6.7 | VERIFY_PLACED 窗口（默认 30.0s）耗尽仍未稳定 → 430 PLACEMENT_FAILED → FAILED | **P0** | test_pick_place_fsm.py:613-622 |
| B-6.8 | 每次 step 视为一次独立观测；须支持"清缓存后重新观测"以体现累计语义 | P0 | E2E case_12 |
| B-6.9 | 未落座/未卸载导致的 430 之外，320 OBJECT_DROPPED 亦不得写 placement_verified | **P0** | test_task_fsm.py:773-788 |

### 2.7 运动重发禁止（`_motion_resend_allowed`）语义

| 编号 | 行为 | 优先级 | 来源 |
|---|---|---|---|
| B-7.1 | 对外只读属性 `motion_resend_allowed`；初值 True | **P0** | pick_place_fsm.py:568,780-782 |
| B-7.2 | 置 False 路径 1：status_unknown → False + FAULT(230) | **P0** | pick_place_fsm.py:1475-1482 |
| B-7.3 | 置 False 路径 2：timed_out → False + FAULT(220) | **P0** | pick_place_fsm.py:1483-1489 |
| B-7.4 | 置 False 路径 3：取消未确认停稳 → FAULT(520) | **P0** | pick_place_fsm.py:1457-1463 |
| B-7.5 | 置 False 路径 4：force_fault 默认 resend_allowed=False（外部监护/步数上限） | **P0** | pick_place_fsm.py:824-833 |
| B-7.6 | 已为 False 时任何 not-ok 结果直接 FAULT，不得降级为可重试 | **P0** | pick_place_fsm.py:1490-1495 |
| B-7.7 | `_is_resumable_context()`：resend_allowed False 即禁止退回重试；另需 holding_state()=="none"（确认无载荷） | **P0** | pick_place_fsm.py:835-847 |
| B-7.8 | 单步可观察：len(motion.calls)==1 且 command_id == command_ids[0]，再 step 5 次调用数不变，history 无 RECOVERY | **P0** | test_pick_place_fsm.py:464-479 |
| B-7.9 | 长跑可观察：E2E 再推进 40 次 step（每次 +1.0s），backend_call_count 不变而 _step_count 增长 | **P0** | E2E case_05 |
| B-7.10 | force_fault 后 5 次 step 内 motion.calls 与 tool.engage_calls 均不增长 | **P0** | test_pick_place_fsm.py:800-823 |
| B-7.11 | 端口返回 None（在途）时不缓存结果，但**必须复用同一条 MotionCommand（含同一 command_id）**，绝不在等待期间生成新指令 | **P0** | pick_place_fsm.py:849-885 |

### 2.8 其余跨层基线（安全守卫与执行层）

| 编号 | 行为 | 优先级 | 来源 |
|---|---|---|---|
| B-8.1 | 未 READY / 模式不符 / 状态过期 / 从未收到状态 → 禁止运动（500） | **P0** | test_interlocks.py:81-147 |
| B-8.2 | READY 不代表安全：状态过期后 READY 也必须被拒 | **P0** | test_interlocks.py:139-147 |
| B-8.3 | 未验证挂载 → 310；已挂载但载荷 UNKNOWN → 310 | **P0** | test_interlocks.py:153-169 |
| B-8.4 | 解锁联锁：载荷 UNKNOWN/NONE → 500；未落座 → 400；运动在途/未知 → 500 | **P0** | test_interlocks.py:178-214 |
| B-8.5 | 释放未验证 → 420；运动未知优先 230 | **P0** | test_interlocks.py:220-234 |
| B-8.6 | ToolManager：解锁前置（已落座+已卸载+处于 releasing 阶段）否则 500 且 IO 零调用 | **P0** | test_tool_manager.py:148-189 |
| B-8.7 | `accepted=True` ≠ 解锁成功：需独立证据（≥GEOMETRY）；仅 COMMAND_ONLY → 410 + FAULT | **P0** | test_tool_manager.py:204-231 |
| B-8.8 | 成功契约用 accepted 而非 success/unlocked（TriggerUnlock.srv 响应恰为 ["accepted","message"]） | **P0** | test_tool_manager.py:253-261 |
| B-8.9 | 无效 pulse_ms（0/-1/1.5/"300"/None）→ 210 且 IO 零调用 | P0 | test_tool_manager.py:190-197 |
| B-8.10 | 桥接默认禁用：所有模式名（含 real/aubo/sdk/gazebo/大写）回落 DisabledSdkWorker | **P0** | test_bridge_core.py:39-43 |
| B-8.11 | 身份白名单为空 → 拒绝建立运动通路；序列号不匹配 → 拒绝；未核实身份 → 拒绝 | **P0** | test_bridge_core.py:73-95；E2E case_14 |
| B-8.12 | 反馈新鲜度上限 0.5s；RTDE 断链或反馈陈旧 → 禁止运动 | **P0** | test_bridge_core.py:97-116 |
| B-8.13 | 运动执行层参数校验：关节顺序/数量、velocity/accel ≤0.2、关节限位 ±3.0rad、timeout>0 | **P0** | test_backends.py:68-100 |
| B-8.14 | 能力声明保守：mock 只声明 joint_goal；disabled 全 False；不得声明时间参数化轨迹/笛卡尔 | **P0** | test_backends.py:232-251；E2E case_10 |
| B-8.15 | 握手日志步骤 PREPARE/ACCEPT/VALIDATE/SEND/MONITOR:SETTLED/CONFIRM:settled | P1 | test_backends.py:254-264 |
| B-8.16 | 审计轨迹含 command_intent/sdk_response/result 且可按 command_id 回溯 | P1 | test_bridge_core.py:224-233 |
| B-8.17 | 非目标进入扫掠区 → 500 且零运动；目标自身不算障碍 | **P0** | E2E case_11 |

### 2.9 迁移时必须一并迁移的"测试可观察钩子"

这些字段/方法被测试直接读写，属于**事实上的接口**；若 C++ 采用不同命名，必须同步改写测试或提供等价访问器（否则用例无法编译/收集）。

| 钩子 | 用途 | 优先级 |
|---|---|---|
| `PickPlaceFSM.states_history` / `result()` | 序列断言与终态结果 | P0 |
| `PickPlaceFSM._port_results`、`_clear_port_results()` | 制造"新一帧观测"、注入缺陷 | P1 |
| `PickPlaceFSM._stability_started_at`、`stability_verified_for_s` | 稳定性计时断言 | P1 |
| `PickPlaceFSM.motion_resend_allowed`、`command_ids`、`recovery_attempts` | 重发/唯一性/预算断言 | P0 |
| `PickPlaceFSM._seat_evidence`（不可变对象整体替换） | FSM 级防御联锁测试 | P1 |
| `PickPlaceFSM._step_count` | 反证"确实在推进" | P1 |
| `PickPlaceFSM.object_attached_estimated`、`stop_confirmed`、`error_code` | 负载估计与停稳证据 | P0 |
| `TaskFsm.last_rejection`、`last_validation_error`、`transition_log`、`has_active_task` | 拒绝原因与校验原因断言 | P0（last_rejection/last_validation_error）、P1（transition_log） |
| `TaskFsm.config.recovery_max_attempts` | 预算注入 | P1 |
| `MotionExecutorCore._active_command_id`、`clear_unknown_after_manual_confirmation()`、`snapshot()`、`handshake(id).steps` | 并发/恢复/日志断言 | P0（前者）、P1 |
| `ToolManager.status().to_tool_state_fields()`、`check_unlock_preconditions()` | IDL 一致性与前置联锁 | P1 |
| `bridge._expected_identity`、`bridge.motion_state_unknown`、`audit_trail()` | 白名单注入与审计 | P1 |
| `MockPipeline.backend_call_count`、`unlock_io_call_count`、`unlock_io_history` | E2E 零运动/零 IO 断言 | P0（作为"零调用"证据） |

---

## 3. 测试脚手架模式

### 3.1 假时钟

| 文件 | 机制 | 关键行为 | 优先级 |
|---|---|---|---|
| test_task_fsm.py | `FakeClock(start=1000.0)`，`__call__` 返回 now，`advance(s)` | 注入 `TaskFsm(clock=...)`；`make_fsm()` 用 `state_<名>_timeout_s=` 覆盖超时表（宽松前缀匹配 `scan`→`SCAN_SCENE`），非法键抛 ValueError | P1 |
| fakes.FakeClock | `start=0.0, auto_advance=0.0`；`now()` 自增 auto_advance；`advance()` / `set()` | test_server_shell 用 `auto_advance=0.05`，因 server 循环无外部时钟推进者，否则看门狗永不触发 | P1 |
| test_backends.py | 自定义 `class FakeClock(Clock)`，`sleep(s)` 累加 | 注入 `MotionExecutorCore(clock=FakeClock())`；真实在途用真实 `Clock()` + 后台线程 + `time.sleep(0.15)` | P1 |
| test_interlocks.py | `now_box=[0.0]` + `clock=lambda: now_box[0]`，改变量推进 | 守卫用 `NOW=100.0`；看门狗用可变量推进 | P1 |
| mock_pipeline.ManualClock | `now/sleep/advance/set`，同时满足 `ClockPort` 与 `backends.Clock` | E2E 每 tick 显式 `advance(tick_seconds)` → 完全确定性 | P0（确定性来源） |

### 3.2 fake 端口"返回 None = 未完成"语义

- **FSM 侧契约**：`PickPlaceFSM._do(key, func)` 调用端口；返回 `None` → 返回哨兵 `_RUN`，**不写入 `_port_results` 缓存**，故 `func` 会被再次调用（表示"仍在执行"）；返回非 None → 缓存到 `self._port_results[(state, key)]`，保证每状态副作用只发生一次。
- **配套命令缓存**：`_cached_command(label, primitive)` 按 `"<state>:<label>"` 复用同一条 `MotionCommand`，因此"在途重入"绝不产生新 `command_id`。
- **FakeMotion**：`delay_ticks=N` 时前 N 次 `execute_joint_move` 返回 `None`（按 `command_id` 计数 `_inflight`），之后返回到位/故障结果；`request_stop` 在 `stop_confirm_after_ticks` 内返回 `None`（等待确认），`stop_confirm_after_ticks=None` 表示永不确认并直接返回 `stop_confirmed=False`。
- **FakeTool**：`_gate(key)` 用 `delay_ticks` 计数，未达阈值返回 `None`；`engage/test_lift/unload/unlock/disengage` 全部支持。
- **计数语义**：`FakeMotion.calls` 只在首次见到 command_id 时追加，故"重发"体现为 `_inflight` 增长而 `calls` 不增长；测试同时断言 `len(calls)` 与 `command_id`。

### 3.3 端口故障/超时注入清单

| Fake 字段 | 注入效果 | 目标错误码 |
|---|---|---|
| `FakeMotion.delay_ticks` | 制造在途（返回 None） | — |
| `FakeMotion.timeout_primitives` / `timeout_after_n_commands` | 运动超时 | 220 |
| `FakeMotion.unknown_primitives` / `unknown_after_n_commands` | 运动状态未知 | 230 |
| `FakeMotion.rejected_primitives` | 执行被拒（且 stop_confirmed=True） | 210 |
| `FakeMotion.stop_confirm_after_ticks=0` | 第 1 次 request_stop 即确认（判定为 `_stop_ticks > N`） | — |
| `FakeMotion.stop_confirm_after_ticks=None` | 永不确认停稳 | 520 |
| `FakeTool.engage_ok/engage_evidence/test_lift_ok/test_lift_evidence/seated/unloaded/unlock_ok/unlock_released/disengage_ok/disengage_released/delay_ticks` | 覆盖接合/挂载/落座/解锁/分离全谱 | 300/310/400/410/420 |
| `FakeTool.object_present`（经 `holding_state()`） | `"holding"` 禁止自动重试；`"none"` 允许 | — |
| `FakePerception.found/stale/unique/color/ring_pose_valid/followed/placement_at_target/placement_settled/placement_found/placement_stale/detection` | 覆盖感知全谱 | 110/120/100/430 |
| `FakePlanner.ok/fail_phases/error_code/message` | 规划失败 | 200 |
| `MockMotionBackend(status_unknown=True / timeout=True / cancel_confirms=False, travel_time_s)` | 执行层未知/超时/取消不确认 | 230/220/未确认 |
| `MockUnlockIo(mode=accept/reject/timeout/raise)` | 解锁端口四态 | 410 |
| `DisabledSdkWorker / DisabledMotionBackend / DisabledUnlockIo(strict)` | 默认禁用通路 | 210/500/410 |

### 3.4 Harness 与驱动

- **test_pick_place_fsm.Harness（dataclass）**：聚合 `perception/planner/motion/tool/clock/config/fsm/strategy`，注册 `fsm.on_event` 收集 `events: List[(event, state)]`、`state_entries`（`setdefault` 记录**首次**进入时间，避免"再次进入"覆盖首次）、`stability_hits`（每次成功累计的 `(累计稳定秒, 进入后秒)`）。
  - `TICK_SECONDS = 0.1`；`RUN_TICK_SECONDS = 0.2`（使 3.0s 稳定窗口与 30s 看门狗在少量迭代内可判定）。
  - `run(max_steps=1200, cancel_after_events=None, cancel_reason="test cancel", tick_seconds=0.2)`：事件数达阈值时**只发一次** cancel；到 SUCCESS/FAILED/FAULT/CANCELLED 即 break；否则 `clock.advance(tick)`。
  - `run_to(state, max_steps=1200)`：在"进入目标状态"时立即返回（处理器尚未执行），用于构造防御性联锁。
  - 断言助手：`history`（`states_history` 值列表）、`visited(state)`。
  - **unittest 兼容层**：`load_tests()` / `_module_test_suite()` / `_make_case()` 把模块级 `test_*` 函数动态包成 TestCase，使 `python3 -m unittest discover -s test` 在无 pytest 环境亦可收集。test_server_shell.py 有同样的兼容层。
- **test_server_shell 替身模块**：`_install_stub_modules()` 向 `sys.modules` 注入 `rclpy`（spin/spin_once/init/shutdown/create_node）、`rclpy.action.ActionClient`（`wait_for_server` 恒 False）、`rclpy.qos`、`mtc_interfaces.{action,msg,srv}`；`_FakeNode` 记录 `published/subscriptions/clients` 并提供假时钟与 logger；`_GoalHandle` 记录 `feedback` 与 `outcome`（succeeded/aborted/canceled）；`_ActionTypeStub` 提供 Goal/Result/Feedback/Server。`_make_server` 默认 `clock=FakeClock(auto_advance=0.05)`、`max_execution_steps=20000`。
- **test_backends 真实在途**：`_make_in_flight(cancel_confirms)` 用 `MockMotionBackend(travel_time_s=5.0)` + 后台线程 + `time.sleep(0.15)` 令后端处于 MOVING，再主线程 `request_stop(in_flight_command_id='in-flight')`。
- **test_interlocks**：`make_guard(**overrides)` 先构造"一切就绪"守卫（READY、AUTO、ttl 1.0、NOW、各 verified True、stop_confirmed True）再单点覆盖；`adapt_stop_client` 支持普通可调用 / 返回对象的可调用 / 带 `.call` 的同步 client / 带 `.call_async` 的异步 client / None。
- **test_tool_manager**：`_UnlockIoSentinel(MockUnlockIo)` 覆写 `send_unlock_pulse` 记录 `observed_pulses`，用于断言"IO 端口一次都没被调用"；`v2_ready_for_unlock()` / `v1_with_verified_attach()` 构造前置状态；只读解析 `mtc_interfaces/msg|srv` 做 IDL 一致性核对（`parse_msg_constants` / `parse_idl_fields`）。
- **test_bridge_core**：`fake_bridge()` = `make_bridge('fake', identity=VERIFIED_ID, ...)`；`with_whitelist(bridge, serial)` 直接替换 `bridge._expected_identity` 以显式配置白名单。
- **E2E Harness**：`ensure_package_paths(_WS_ROOT)` + `resolve_dependencies()`（13 个模块请求，含占位包 `mtc_motion_planning`）+ `require_pipeline_dependencies`；`CaseSkipped` → SKIP（**绝不计 PASS**）；`must(cond, msg)` 附带调用点文件名与行号；`MockPipeline` 提供 `backend_call_count`、`unlock_io_call_count`、`unlock_io_history`、`new_pick_place()`；`drive(fsm, pipeline, max_ticks=600, tick_seconds=1.0)` 推到终态；`case_04` 用 `FaultingBackend` 包装真实 mock 后端并替换 `pipeline.executor._backend`；`case_09` 直接 monkeypatch `fsm2.motion.request_stop = _unconfirmed_stop`；`case_12` 用 `fsm._clear_port_results()` 模拟新视觉帧；`main()` 若检测到 `rclpy`/`mtc_interfaces`/`rcl_interfaces` 被导入则直接 FAIL（exit 1），返回码 `0 if failed==0 and passed>0 else 1`。

---

## 4. 断言硬数字清单

### 4.1 错误码（跨文件一致，必须逐值冻结）

`OK=0`、`INVALID_TASK=100`、`TARGET_NOT_FOUND=110`、`POSE_STALE=120`、`PLANNING_FAILED=200`、`EXECUTION_REJECTED=210`、`MOTION_TIMEOUT=220`、`MOTION_STATUS_UNKNOWN=230`、`ENGAGE_FAILED=300`、`ATTACH_NOT_VERIFIED=310`、`OBJECT_DROPPED=320`、`SEAT_NOT_VERIFIED=400`、`UNLOCK_FAILED=410`、`RELEASE_NOT_VERIFIED=420`、`PLACEMENT_FAILED=430`、`SAFETY_INTERLOCK=500`、`HARDWARE_FAULT=510`、`CANCEL_NOT_CONFIRMED=520`。

按文件出现的数值断言：

| 文件 | 出现的错误码数值 |
|---|---|
| test_task_fsm.py | 100、110、230、520；`name_of(520)=="CANCEL_NOT_CONFIRMED"` |
| test_pick_place_fsm.py | 0、100、110、120、200、210、220、230、300、310、400、410、420、430（均为 `ErrorCode.X == 数值` 双重断言） |
| test_server_shell.py | 0（成功）、100（INVALID_TASK） |
| test_backends.py | 0、210、220、230、500 |
| test_interlocks.py | 0、210、230、310、400、420、500、510、520 |
| test_tool_manager.py | 210、310、410、420、500；状态码 0/1/2/3/4/5；证据级 0/1/2/3（`MIN_INDEPENDENT_EVIDENCE=EVIDENCE_GEOMETRY=2`） |
| test_bridge_core.py | 0、210、500 |
| test_mock_end_to_end.py | 100/110/120/200/210/220/230/310/400/410/420/430/500/520（多数以符号断言，case_02 显式 210） |

### 4.2 超时与时间常量

| 来源 | 数值 |
|---|---|
| Task 层 `DEFAULT_STATE_TIMEOUTS` | INIT 5.0、SELF_CHECK 10.0、WAIT_TASK None、VALIDATE_TASK 2.0、SCAN_SCENE 5.0、EXECUTE_ITEM 60.0、RECORD_RESULT 2.0、RECOVERY 10.0、CANCEL_PENDING 5.0、FINISHED/TASK_FAILED/FAULT/CANCELLED None |
| test_task_fsm 覆盖值 | CANCEL_PENDING 5.0 与 1.0；SCAN_SCENE 5.0；EXECUTE_ITEM 10.0；VALIDATE_TASK 2.0 |
| test_task_fsm 时间推进点 | 4.0（不触发）/1.5（累计 5.5 触发）；9.0/1.5（10.5）；2.5；2.0/1.5（3.5，任务级 timeout_ms=3000）；10000.0（WAIT_TASK 恒不触发） |
| PickPlace `state_timeouts` 默认 | RESOLVE_TARGET 2.0、PLAN_ACQUIRE 2.0、MOVE_PRE_ALIGN 30.0、ENGAGE 15.0、VERIFY_ENGAGEMENT 5.0、TEST_LIFT 15.0、VERIFY_ATTACHED 5.0、LIFT 30.0、TRANSPORT 60.0、MOVE_PRE_SEAT 30.0、SEAT 20.0、VERIFY_SEATED 5.0、UNLOAD 15.0、DISENGAGE 20.0、RETREAT 30.0、VERIFY_PLACED 30.0 |
| PickPlace 默认配置 | `placement_stability_sec=3.0`、`perception_max_age_s=0.3`、`recovery_max_attempts=1`、`default_velocity_scaling=0.2`、`default_acceleration_scaling=0.2`、`default_timeout_ms=5000` |
| 稳定性测试窗口 | 3.0s（容差 +0.2s）、6.0s（容差 +0.2s）；E2E 阈值逼近用 0.05s 欠量 + 0.1s 越过 |
| 测试 tick | PickPlace `TICK_SECONDS=0.1`、`RUN_TICK_SECONDS=0.2`；E2E `tick_seconds=1.0`；server FakeClock `auto_advance=0.05` |
| motion_execution | `move()` 默认 velocity/accel 0.1、timeout 30_000 ms；`ValidationConfig` 上限 0.2/0.2、关节限位 ±3.0 rad；超时用例 timeout=1000ms、travel_time_s=10.0；在途用例 travel_time_s=5.0、线程 sleep 0.15s；mock `settle_position_tolerance_rad=0.001`、`settle_velocity_tolerance_rad_s=0.001`、`settle_duration_s=0.2` |
| safety 看门狗 | `robot_state_timeout_sec=0.5`、`tool_state_timeout_sec=1.0`、`stop_confirm_timeout_sec=2.0`（推进到 now=3.0 升级）；`state_ttl_sec` 默认 0.5、测试用 1.0 与 0.0；`NOW=100.0`；过期用 NOW-5.0；人工清除时间戳 NOW+7.0 |
| bridge | `feedback_freshness_limit_s=0.5`（测试用 age 0.01/5.0/10.0） |

### 4.3 重试次数 / 稳定性 / 计数硬数字

- `recovery_max_attempts`：默认 **1**；测试显式用 **1** 与 **0**。断言：ENGAGE 重试场景 `recovery_attempts == 2`（首次 + 重试后再入 RECOVERY）、`history.count("ENGAGE") == 2`；正常重试获批后 `recovery_attempts == 1`；不确定载荷场景 `recovery_attempts == 0` 且无 RECOVERY。
- 运动指令条数：V1 happy path 恰 **7** 条（`side_insert, test_lift, lift, transport, pre_seat, seat_contact, retreat`）；`side_insert` 未知后 `len(motion.calls) == 1`；`transport` 超时后 transport 计数 **1**；`lift` 被拒后 lift 计数 **1**；`test_lift_ok=False` 时运动序列恰为 `["side_insert"]`。
- 工具调用计数：V1 `unlock_calls == 0`；V2 `engage_calls == 1`、`unlock_calls == 1`；未落座时 `unload_calls/disengage_calls == 0`；解锁失败 `unlock_calls == 1`；420 场景 `disengage_calls == 1`。
- IO 端口计数：联锁未过 `io.call_count == 0`；受理 `== 1`；V2 正常流程 `io.history == (300,)`（pulse 300ms）；`DisabledUnlockIo` lenient `call_count == 1`。
- 稳定性：`stable_for ∈ [3.0, 3.2]` / `[6.0, 6.2]`；反例注入前累计 `1.0 <= x < 3.0`，重置后 `stability_verified_for_s == 0.0`。
- Task 计数：`completed_count`/`completed_slots` 长度 1（基础）与 3（序列）；`total_count == 3`。
- E2E 计数：`backend_call_count == 0`（4 类感知缺陷 + 重试对照 + 扫掠区场景）；超时后 40 次 step 内 `backend_call_count` 不变；`stop_calls` 恰 **1** 次；`unlock_io_call_count` 为 0（V1/未落座/取消）或 1（V2 解锁）。
- 循环上限（有界推进）：`max_steps` 300/400/600/1000/1200/1500/2000/2500/4000；E2E `max_ticks` 默认 600、case_05 用 80；`max_execution_steps=20000`（server）。
- 断言循环次数：placement_verified 唯一写入口检查 **2000** 步；稳定性提前判成功检查 **4000** 步；FAULT 后不重发检查 **5** 次 step（PickPlace 与 force_fault 各一处）；E2E 超时后 **40** 次 step；防御联锁破坏后最多 **20** 次 step。
- 其它硬数字：Action Server 名 `"/mtc/manipulation/pick_place"`；topic `"/mtc/perception/batteries"`、`"/mtc/tool/state"`；`_ActionTypeStub` 默认 `placement_stability_sec=3.0`、`target_slot="T0"`；`JOINTS` 恰 6 个（`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`）；目标关节位 `(0.1,0,0,0,0,0)`；越界值 `99.0`；速度越限 `0.9`；V2 正常脉冲 `300ms`；无效脉冲集合 `{0, -1, 1.5, "300", None}`；非法颜色 `{0, 5, 99}`；Task 层 `FakeGoal.timeout_ms=120000`、`placement_stability_sec=3.0`、E2E `timeout_ms=60000`。

---

## 5. 审计边界与后续建议

1. **执行状态**：本次审计**未执行** pytest / unittest / python3，全部 270 条用例状态为 **NOT RUN**；本报告不构成"测试通过"的证据。
2. **C++ 迁移等价性检查建议**：优先为 §2.6（`placement_verified` 唯一写入口）与 §2.7（`motion_resend_allowed`）构造负向单测。这两个语义一旦在移植中退化为"普通布尔标志"，多数故障注入用例会以**静默变成可重试**的方式失效——这是最危险的一类回归。
3. **装配方式差异**：`conftest.py` 不是 fixture 源；两个 pytest 风格文件各自在模块顶插入 `sys.path`。迁移到 C++（gtest / colcon）时这部分装配逻辑需重新设计，不能假定存在 Python 式 fixture 注入。
4. **文案耦合**：多处断言匹配中文/英文消息子串（`"不一致"`、`"无解锁机构"`、`"顺序"`、`"上限"`、`"并发"`、`"在途命令"`、`"not queued"`、`"落座"`、`"载荷"`、`"白名单"`、`"不匹配"`、`"反馈不新鲜"`、`"RTDE"`、`"重复"`、`"未确认"`、`"旧规划作废"`、`"不代表已解锁"`、`"电磁铁"`、`"无解锁机构"`）。C++ 侧若改用结构化错误码替代字符串匹配，需同步改写这些断言（建议保留错误码断言，弱化文案断言）。
5. **未修改声明**：本文件为本次审计唯一新增产物，未修改任何源码或测试文件。
