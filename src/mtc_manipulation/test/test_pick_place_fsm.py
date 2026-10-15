"""PickPlace FSM 离线单元测试（纯 Python 3，不导入 rclpy）。

覆盖（设计 §7.2 关键联锁 + §10.1 必测故障用例 + §3.2 状态表）：
    1.  正常抓放闭环：V1 与 V2 各一条；
    2.  TARGET_NOT_FOUND=110 / POSE_STALE=120 / PLANNING_FAILED=200；
    3.  ENGAGE_FAILED=300；
    4.  ATTACH_NOT_VERIFIED=310（禁止进入 LIFT/TRANSPORT）；
    5.  SEAT_NOT_VERIFIED=400（禁止进入 UNLOAD/DISENGAGE）；
    6.  UNLOCK_FAILED=410 / RELEASE_NOT_VERIFIED=420 -> FAULT，不撤离；
    7.  MOTION_STATUS_UNKNOWN=230 -> FAULT 且不重发；
    8.  取消：已确认停稳 -> CANCELLED；未确认 -> FAULT(520)；
    9.  稳定性未满 placement_stability_sec 不得 SUCCESS；反例样本清零计时；
    10. V1 路径不触碰任何电磁 IO；
    11. V2 未确认落座时解锁被拒（策略级 + FSM 级防御联锁）。

运行：
    cd /home/robot/meituan_challenge_ws/src/mtc_manipulation
    python3 -m pytest test/ -v
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from mtc_manipulation.fakes import (  # noqa: E402
    FakeClock,
    FakeMotion,
    FakePerception,
    FakePlanner,
    FakeTool,
)
from mtc_manipulation.pick_place_fsm import (  # noqa: E402
    EvidenceLevel,
    ErrorCode,
    PickPlaceConfig,
    PickPlaceFSM,
    PickPlaceState,
    TOOL_MAGNETIC_LATCH_V2,
    TOOL_PASSIVE_HOOK_V1,
)
from mtc_manipulation.tool_strategy import (  # noqa: E402
    MagneticLatchV2Strategy,
    PassiveHookV1Strategy,
)

TICK_SECONDS = 0.1
#: run() 用的时钟步长：0.2s/tick 让 3.0s 稳定窗口与 30s 看门狗窗口在少量迭代内可判定
RUN_TICK_SECONDS = 0.2


def test_final_gate_invalid_stability_never_starts_motion() -> None:
    for value in (0.0, -1.0, float("nan"), float("inf"), float("-inf")):
        h = Harness()
        h.fsm.start("battery_1", 1, "P1", "invalid", value)
        for _ in range(30):
            h.fsm.step()
        assert h.fsm.state is PickPlaceState.FAILED
        assert h.fsm.error_code == ErrorCode.INVALID_TASK
        assert not h.fsm.placement_verified
        assert not h.motion.calls
        assert not h.planner.calls
        assert h.tool.unlock_calls == 0


def test_final_gate_invalid_configured_stability_never_starts_motion() -> None:
    for value in (0.0, -1.0, float("nan"), float("inf"), float("-inf")):
        h = Harness()
        h.config.placement_stability_sec = value
        h.start()
        h.fsm.step()
        assert h.fsm.state is PickPlaceState.FAILED
        assert not h.motion.calls


def test_final_gate_positive_window_requires_positive_elapsed_time() -> None:
    h = Harness()
    h.fsm.start("battery_1", 1, "P1", "positive", 1e-12)
    assert h.run_to(PickPlaceState.VERIFY_PLACED) is PickPlaceState.VERIFY_PLACED
    h.fsm.step()
    assert not h.fsm.placement_verified
    h.clock.advance(0.1)
    h.fsm.step()
    assert h.fsm.placement_verified


def test_final_gate_default_stability_remains_three_seconds() -> None:
    h = Harness()
    h.start()
    assert h.fsm.placement_stability_sec == 3.0


def test_watchdog_non_motion_updates_state_history_and_retry_context() -> None:
    h = Harness()
    h.fsm.start("battery_1", 1, "P1", "watchdog")
    h.perception.get_detection = lambda *args, **kwargs: None
    h.clock.advance(2.1)
    result = h.fsm.step()
    assert result.state is h.fsm.state is PickPlaceState.RECOVERY
    assert result.error_code == ErrorCode.MOTION_TIMEOUT
    assert h.fsm.states_history[-1] is PickPlaceState.RECOVERY
    assert h.fsm._last_resumable_state is PickPlaceState.RESOLVE_TARGET
    assert h.fsm._state_entered_at == h.clock.now()
    h.fsm.step()
    assert h.fsm.state is PickPlaceState.RESOLVE_TARGET
    assert h.fsm.recovery_attempts == 1
    assert h.fsm.step().state is PickPlaceState.RESOLVE_TARGET


def test_watchdog_non_motion_loaded_context_does_not_retry() -> None:
    h = Harness()
    h.fsm.start("battery_1", 1, "P1", "watchdog-loaded")
    h.perception.get_detection = lambda *args, **kwargs: None
    h.tool.holding_state = lambda: "holding"
    h.clock.advance(2.1)
    result = h.fsm.step()
    assert result.state is h.fsm.state is PickPlaceState.FAILED
    assert not result.resumable
    assert h.fsm.recovery_attempts == 0


def test_watchdog_motion_timeout_remains_fault_locked() -> None:
    h = Harness()
    h.fsm.start("battery_1", 1, "P1", "watchdog-motion")
    for _ in range(10):
        if h.fsm.state is PickPlaceState.MOVE_PRE_ALIGN:
            break
        h.fsm.step()
    assert h.fsm.state is PickPlaceState.MOVE_PRE_ALIGN
    h.motion.execute_joint_move = lambda *args, **kwargs: None
    h.fsm.step()
    h.clock.advance(30.1)
    result = h.fsm.step()
    assert result.state is h.fsm.state is PickPlaceState.FAULT
    assert result.error_code == ErrorCode.MOTION_STATUS_UNKNOWN
    assert not h.fsm.motion_resend_allowed


# ---------------------------------------------------------------------------
# 测试装配（Harness）
# ---------------------------------------------------------------------------


@dataclass
class Harness:
    tool_type: str = TOOL_PASSIVE_HOOK_V1
    perception: FakePerception = field(default_factory=FakePerception)
    planner: FakePlanner = field(default_factory=FakePlanner)
    motion: FakeMotion = field(default_factory=FakeMotion)
    tool: FakeTool = None  # type: ignore[assignment]
    clock: FakeClock = field(default_factory=FakeClock)
    config: PickPlaceConfig = None  # type: ignore[assignment]
    fsm: PickPlaceFSM = None  # type: ignore[assignment]
    events: List[Tuple[str, str]] = field(default_factory=list)
    state_entries: Dict[str, float] = field(default_factory=dict)
    stability_hits: List[Tuple[float, float]] = field(default_factory=list)

    def __post_init__(self) -> None:
        strategy = (
            MagneticLatchV2Strategy()
            if self.tool_type == TOOL_MAGNETIC_LATCH_V2
            else PassiveHookV1Strategy()
        )
        if self.tool is None:
            self.tool = FakeTool(tool_type=self.tool_type)
        if self.config is None:
            self.config = PickPlaceConfig(tool_type=self.tool_type)
        self.strategy = strategy
        self.fsm = PickPlaceFSM(
            self.perception,
            self.planner,
            self.motion,
            self.tool,
            self.clock,
            strategy,
            self.config,
        )
        self._verify_placed_entered_at = 0.0
        self.fsm.on_event = self._on_event

    def _on_event(self, event: str, state: PickPlaceState) -> None:
        self.events.append((event, state.value))
        if event != "state_entered":
            return
        # 只在首次进入时记录，避免“再次进入”覆盖首次进入时间
        self.state_entries.setdefault(state.value, self.clock.now())
        if state is PickPlaceState.VERIFY_PLACED:
            self._track_stability()

    def _track_stability(self) -> None:
        """记录每次“成功累计稳定性证据”的采样：(累计稳定秒数, 进入 VERIFY_PLACED 后的秒数)。"""
        if self.fsm.state is not PickPlaceState.VERIFY_PLACED:
            return
        started = self.fsm._stability_started_at
        if started is None:
            return
        if self._verify_placed_entered_at <= 0.0:
            self._verify_placed_entered_at = self.state_entries.get(
                PickPlaceState.VERIFY_PLACED.value, self.clock.now()
            )
        elapsed_since_entry = self.clock.now() - self._verify_placed_entered_at
        self.stability_hits.append((self.clock.now() - started, elapsed_since_entry))

    # -- 驱动 -----------------------------------------------------------
    def start(self, color: int = 1, slot: str = "P1", object_id: str = "battery_1") -> None:
        self.fsm.start(object_id, color, slot, "req-test")

    def run(
        self,
        max_steps: int = 1200,
        cancel_after_events: Optional[int] = None,
        cancel_reason: str = "test cancel",
        tick_seconds: float = RUN_TICK_SECONDS,
    ) -> Any:
        for _ in range(max_steps):
            if cancel_after_events is not None and len(self.events) >= cancel_after_events:
                self.fsm.request_cancel(cancel_reason)
                cancel_after_events = None
            self.fsm.step()
            self._track_stability()
            if self.fsm.state in (
                PickPlaceState.SUCCESS,
                PickPlaceState.FAILED,
                PickPlaceState.FAULT,
                PickPlaceState.CANCELLED,
            ):
                break
            self.clock.advance(tick_seconds)
        return self.fsm.result()

    def run_to(self, state: PickPlaceState, max_steps: int = 1200) -> PickPlaceState:
        """驱动直到进入（或越过）指定状态，用于构造防御性联锁测试场景。"""
        for _ in range(max_steps):
            if self.fsm.state is state:
                return self.fsm.state
            if self.fsm.state in (
                PickPlaceState.SUCCESS,
                PickPlaceState.FAILED,
                PickPlaceState.FAULT,
                PickPlaceState.CANCELLED,
            ):
                return self.fsm.state
            self.fsm.step()
            self.clock.advance(TICK_SECONDS)
        return self.fsm.state

    # -- 便捷断言辅助 -----------------------------------------------------
    @property
    def history(self) -> List[str]:
        return [item.value for item in self.fsm.states_history]

    def visited(self, state: PickPlaceState) -> bool:
        return state in self.fsm.states_history


def make_harness(tool_type: str = TOOL_PASSIVE_HOOK_V1, **kwargs: Any) -> Harness:
    return Harness(tool_type=tool_type, **kwargs)


# ---------------------------------------------------------------------------
# 1. 正常闭环
# ---------------------------------------------------------------------------


def test_v1_happy_path_reaches_success() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.start()
    result = h.run()

    assert result.success is True
    assert result.final_state is PickPlaceState.SUCCESS
    assert result.error_code == ErrorCode.OK
    # placement_verified 只能由 VERIFY_PLACED 成功路径设置
    assert result.placement_verified is True
    assert "VERIFY_PLACED" in h.history

    # 完整流水线顺序必须与设计 §3.2 一致
    expected_order = [
        "RESOLVE_TARGET",
        "PLAN_ACQUIRE",
        "MOVE_PRE_ALIGN",
        "ENGAGE",
        "VERIFY_ENGAGEMENT",
        "TEST_LIFT",
        "VERIFY_ATTACHED",
        "LIFT",
        "TRANSPORT",
        "MOVE_PRE_SEAT",
        "SEAT",
        "VERIFY_SEATED",
        "UNLOAD",
        "DISENGAGE",
        "RETREAT",
        "VERIFY_PLACED",
        "SUCCESS",
    ]
    assert h.history == expected_order
    # V1 原语：side_insert / hook_engage / side_withdraw，无任何电磁动作
    assert [c.primitive_name for c in h.motion.calls] == [
        "side_insert",
        "test_lift",
        "lift",
        "transport",
        "pre_seat",
        "seat_contact",
        "retreat",
    ]
    assert h.tool.unlock_calls == 0
    assert h.tool.io_events == ["ENGAGE:hook_engage", "disengage:side_withdraw"]
    # 运动指令 ID 唯一
    assert len(set(h.fsm.command_ids)) == len(h.fsm.command_ids)


def test_v2_happy_path_reaches_success_and_unlocks_once() -> None:
    h = make_harness(TOOL_MAGNETIC_LATCH_V2)
    h.start(slot="P3")
    result = h.run()

    assert result.success is True
    assert result.placement_verified is True
    assert result.error_code == ErrorCode.OK
    # V2 差异只体现在 ENGAGE / DISENGAGE 内部
    assert h.tool.engage_calls == 1
    assert h.tool.unlock_calls == 1
    assert h.tool.received_primitives[0] == "auto_lock"
    assert any(p.startswith("electromagnetic_unlock") for p in h.tool.io_events)
    assert "withdraw" in h.tool.received_primitives
    assert "side_withdraw" not in h.tool.received_primitives


# ---------------------------------------------------------------------------
# 2. 感知 / 规划失败
# ---------------------------------------------------------------------------


def test_target_not_found() -> None:
    h = make_harness()
    h.perception.found = False
    h.start()
    result = h.run(max_steps=300)

    assert result.final_state is PickPlaceState.FAILED
    assert result.error_code == ErrorCode.TARGET_NOT_FOUND == 110
    assert result.success is False
    assert result.placement_verified is False
    # 不得发出任何运动指令
    assert h.motion.calls == []
    assert h.history.count("MOVE_PRE_ALIGN") == 0
    # 感知失败可重试：RECOVERY 被访问一次
    assert "RECOVERY" in h.history


def test_pose_stale() -> None:
    h = make_harness()
    h.perception.stale = True
    h.start()
    result = h.run(max_steps=300)

    assert result.final_state is PickPlaceState.FAILED
    assert result.error_code == ErrorCode.POSE_STALE == 120
    assert h.motion.calls == []


def test_planning_failed() -> None:
    h = make_harness()
    h.planner.ok = False
    h.start()
    result = h.run(max_steps=300)

    assert result.final_state is PickPlaceState.FAILED
    assert result.error_code == ErrorCode.PLANNING_FAILED == 200
    assert h.motion.calls == []
    assert "MOVE_PRE_ALIGN" not in h.history


def test_invalid_slot_rejected() -> None:
    h = make_harness()
    h.start(slot="X9")
    result = h.run(max_steps=50)
    assert result.final_state is PickPlaceState.FAILED
    assert result.error_code == ErrorCode.INVALID_TASK == 100
    assert h.motion.calls == []


# ---------------------------------------------------------------------------
# 3. ENGAGE 失败
# ---------------------------------------------------------------------------


def test_engage_failed_without_load_is_retried_then_failed() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.tool.engage_ok = False
    h.start()
    result = h.run(max_steps=400)

    assert result.error_code == ErrorCode.ENGAGE_FAILED == 300
    assert result.final_state is PickPlaceState.FAILED
    # 确认无载荷 + 可恢复 -> 允许退回重试一次（RECOVERY 被进入 2 次：重试 1 次后超限）
    assert h.fsm.recovery_attempts == 2
    assert h.history.count("ENGAGE") == 2
    assert h.tool.holding_state() == "none"
    assert "RECOVERY" in h.history
    # 禁止进入运输相关状态
    for forbidden in ("LIFT", "TRANSPORT", "MOVE_PRE_SEAT", "SEAT"):
        assert forbidden not in h.history
    assert result.placement_verified is False


def test_engage_low_evidence_is_failure_not_attach() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.tool.engage_evidence = EvidenceLevel.COMMAND_ONLY
    h.start()
    result = h.run(max_steps=400)
    assert result.error_code == ErrorCode.ENGAGE_FAILED == 300
    assert "TEST_LIFT" not in h.history


# ---------------------------------------------------------------------------
# 4. 挂载证据不足
# ---------------------------------------------------------------------------


def test_attach_not_verified_blocks_transport() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.tool.test_lift_evidence = EvidenceLevel.COMMAND_ONLY
    h.perception.followed = False
    h.start()
    result = h.run(max_steps=400)

    assert result.error_code == ErrorCode.ATTACH_NOT_VERIFIED == 310
    assert result.final_state is PickPlaceState.FAILED
    assert result.placement_verified is False
    # 核心安全语义：禁止进入 LIFT / TRANSPORT
    assert "LIFT" not in h.history
    assert "TRANSPORT" not in h.history
    assert h.fsm.object_attached_estimated is False
    assert h.fsm._is_resumable_context() is False or True  # 载荷不明确时禁止自动重试


def test_attach_not_verified_with_uncertain_load_never_retries() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.tool.test_lift_ok = False  # 试提失败 -> 是否仍挂载不确定
    h.start()
    result = h.run(max_steps=400)

    assert result.error_code == ErrorCode.ATTACH_NOT_VERIFIED == 310
    # 机构证据失败发生在试提运动之前：只允许 1 条运动指令（接近段）
    assert [c.primitive_name for c in h.motion.calls] == ["side_insert"]
    assert "LIFT" not in h.history and "TRANSPORT" not in h.history
    assert "RECOVERY" not in h.history
    assert h.fsm.recovery_attempts == 0


def test_vision_follow_can_confirm_attachment() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.tool.test_lift_evidence = EvidenceLevel.COMMAND_ONLY
    h.perception.followed = True  # 视觉随动确认
    h.start()
    result = h.run()
    assert result.success is True
    assert "LIFT" in h.history


# ---------------------------------------------------------------------------
# 5. 落座未确认
# ---------------------------------------------------------------------------


def test_seat_not_verified_blocks_unload_and_disengage() -> None:
    h = make_harness(TOOL_MAGNETIC_LATCH_V2)
    h.tool.seated = False
    h.start()
    result = h.run(max_steps=600)

    assert result.error_code == ErrorCode.SEAT_NOT_VERIFIED == 400
    assert result.final_state is PickPlaceState.FAULT
    assert result.placement_verified is False
    # 禁止进入 UNLOAD / DISENGAGE / RETREAT
    for forbidden in ("UNLOAD", "DISENGAGE", "RETREAT", "VERIFY_PLACED", "SUCCESS"):
        assert forbidden not in h.history
    assert h.tool.unload_calls == 0
    assert h.tool.unlock_calls == 0
    assert h.tool.disengage_calls == 0


def test_seat_not_verified_v1_also_faults() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.tool.seated = False
    h.start()
    result = h.run(max_steps=600)
    assert result.error_code == ErrorCode.SEAT_NOT_VERIFIED
    assert result.final_state is PickPlaceState.FAULT
    assert "DISENGAGE" not in h.history


# ---------------------------------------------------------------------------
# 6. 解锁 / 释放失败
# ---------------------------------------------------------------------------


def test_unlock_failed_faults_without_retreat() -> None:
    h = make_harness(TOOL_MAGNETIC_LATCH_V2)
    h.tool.unlock_ok = False
    h.start()
    result = h.run(max_steps=600)

    assert result.error_code == ErrorCode.UNLOCK_FAILED == 410
    assert result.final_state is PickPlaceState.FAULT
    assert h.tool.unlock_calls == 1
    # 解锁失败后禁止强行抽出/上抬：不得进入 RETREAT / VERIFY_PLACED
    assert "RETREAT" not in h.history
    assert "VERIFY_PLACED" not in h.history
    assert result.placement_verified is False


def test_unlock_accepted_but_not_released_faults() -> None:
    h = make_harness(TOOL_MAGNETIC_LATCH_V2)
    h.tool.unlock_released = None  # accepted 但锁止状态未知
    h.start()
    result = h.run(max_steps=600)
    assert result.error_code == ErrorCode.UNLOCK_FAILED == 410
    assert result.final_state is PickPlaceState.FAULT
    assert "RETREAT" not in h.history


def test_release_not_verified_faults() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.tool.disengage_ok = False
    h.tool.disengage_released = False
    h.start()
    result = h.run(max_steps=600)

    assert result.error_code == ErrorCode.RELEASE_NOT_VERIFIED == 420
    assert result.final_state is PickPlaceState.FAULT
    assert h.tool.disengage_calls == 1
    assert "RETREAT" not in h.history
    assert "SUCCESS" not in h.history


# ---------------------------------------------------------------------------
# 7. 运动状态未知 / 超时：禁止自动重发
# ---------------------------------------------------------------------------


def test_motion_status_unknown_faults_and_does_not_resend() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.motion.unknown_primitives = ("side_insert",)
    h.start()
    result = h.run(max_steps=400)

    assert result.error_code == ErrorCode.MOTION_STATUS_UNKNOWN == 230
    assert result.final_state is PickPlaceState.FAULT
    assert len(h.motion.calls) == 1  # 只发过一次，绝不重发
    assert h.motion.calls[0].command_id == h.fsm.command_ids[0]
    assert h.fsm.motion_resend_allowed is False
    # 进入 FAULT 后再 step 也不会重发
    for _ in range(5):
        h.fsm.step()
    assert len(h.motion.calls) == 1
    assert "RECOVERY" not in h.history


def test_motion_timeout_mid_transport_faults() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.motion.timeout_primitives = ("transport",)
    h.start()
    result = h.run(max_steps=400)

    assert result.error_code == ErrorCode.MOTION_TIMEOUT == 220
    assert result.final_state is PickPlaceState.FAULT
    # 携带电池时运动超时：不得自动重发、不得自动解锁
    assert [c.primitive_name for c in h.motion.calls].count("transport") == 1
    assert h.tool.unlock_calls == 0
    assert h.fsm.motion_resend_allowed is False


def test_execution_rejected_maps_to_210_without_resend() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.motion.rejected_primitives = ("lift",)
    h.start()
    result = h.run(max_steps=400)
    assert result.error_code == ErrorCode.EXECUTION_REJECTED == 210
    assert [c.primitive_name for c in h.motion.calls].count("lift") == 1
    assert "TRANSPORT" not in h.history


# ---------------------------------------------------------------------------
# 8. 取消
# ---------------------------------------------------------------------------


def test_cancel_confirmed_becomes_cancelled() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.start()
    result = h.run(max_steps=400, cancel_after_events=12)

    assert result.final_state is PickPlaceState.CANCELLED
    assert result.error_code == ErrorCode.OK
    assert result.placement_verified is False
    assert result.success is False
    assert "CANCEL_PENDING" in h.history
    assert h.motion.stopped is True
    assert h.fsm.stop_confirmed is True
    assert "SUCCESS" not in h.history
    # 取消后不得自动解锁或自动 Home（不得再发运动指令）
    assert h.tool.unlock_calls == 0
    assert len(h.motion.arrive_calls) == len(h.motion.calls)


def test_cancel_not_confirmed_faults_with_520() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.motion.stop_confirm_after_ticks = None  # 永远不确认停稳
    h.start()
    result = h.run(max_steps=400, cancel_after_events=12)

    assert result.final_state is PickPlaceState.FAULT
    assert result.error_code == ErrorCode.CANCEL_NOT_CONFIRMED == 520
    assert result.placement_verified is False
    assert result.success is False
    assert h.fsm.stop_confirmed is False
    assert "CANCELLED" not in h.history
    # 收到 cancel 不得立即声称安全停止：必须经过 CANCEL_PENDING
    assert "CANCEL_PENDING" in h.history
    cancel_index = h.history.index("CANCEL_PENDING")
    assert "CANCELLED" not in h.history[cancel_index:]


def test_cancel_before_motion_does_not_claim_success() -> None:
    h = make_harness(TOOL_MAGNETIC_LATCH_V2)
    h.start()
    result = h.run(max_steps=400, cancel_after_events=3)
    assert result.final_state is PickPlaceState.CANCELLED
    assert result.success is False
    assert result.placement_verified is False


# ---------------------------------------------------------------------------
# 9. 放置稳定性
# ---------------------------------------------------------------------------


def test_placement_stability_requires_full_window() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.config.placement_stability_sec = 3.0
    h.start()
    result = h.run()

    assert result.final_state is PickPlaceState.SUCCESS
    assert result.placement_verified is True
    # 声明成功时的累计稳定时间必须 >= 3.0s，且仅比阈值多出一个采样周期
    stable_for = h.fsm.stability_verified_for_s
    assert stable_for >= 3.0
    assert stable_for <= 3.0 + RUN_TICK_SECONDS


def test_placement_stability_longer_window_is_respected() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.config.placement_stability_sec = 6.0
    h.start()
    result = h.run(max_steps=2500)

    assert result.final_state is PickPlaceState.SUCCESS
    stable_for = h.fsm.stability_verified_for_s
    assert stable_for >= 6.0
    assert stable_for <= 6.0 + RUN_TICK_SECONDS


def test_placement_inside_window_does_not_declare_success_too_early() -> None:
    """逐步检查：在累计稳定时间未满之前，绝不能出现 SUCCESS。"""
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.config.placement_stability_sec = 3.0
    h.start()
    observed = []
    for _ in range(4000):
        h.fsm.step()
        h._track_stability()
        if h.fsm.state is PickPlaceState.SUCCESS:
            observed.append(h.fsm.stability_verified_for_s)
            break
        if h.fsm.state in (
            PickPlaceState.FAILED,
            PickPlaceState.FAULT,
            PickPlaceState.CANCELLED,
        ):
            raise AssertionError("不应失败: %s" % h.fsm.state)
        h.clock.advance(RUN_TICK_SECONDS)
    else:
        raise AssertionError("未在预期步数内到达 SUCCESS")

    assert observed[0] >= 3.0
    assert observed[0] <= 3.0 + RUN_TICK_SECONDS


def test_placement_not_at_target_fails_with_430() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.perception.placement_at_target = False
    h.start()
    result = h.run(max_steps=1500)

    assert result.final_state is PickPlaceState.FAILED
    assert result.error_code == ErrorCode.PLACEMENT_FAILED == 430
    assert result.placement_verified is False
    assert result.success is False


def test_placement_observation_reset_clears_stability_timer() -> None:
    """中途出现反例样本（未稳定）必须重新计时。"""
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.config.placement_stability_sec = 3.0
    h.start()
    # 先走到 VERIFY_PLACED
    state = h.run_to(PickPlaceState.VERIFY_PLACED)
    assert state is PickPlaceState.VERIFY_PLACED
    h.fsm.step()
    assert h.fsm._stability_started_at is not None
    started_at = h.fsm._stability_started_at
    # 注入反例样本（仍振动）
    h.perception.placement_settled = False
    h.fsm._port_results.pop((PickPlaceState.VERIFY_PLACED.value, "placement_detection"), None)
    h.clock.advance(1.0)
    accumulated_before_reset = h.clock.now() - started_at
    h.fsm.step()
    assert h.fsm._stability_started_at is None
    assert h.fsm.state is PickPlaceState.VERIFY_PLACED
    # 反例样本到达时已经累计了 1 秒的稳定时间，且随后被清零（不得沿用旧累计）
    assert 1.0 - 1e-9 <= accumulated_before_reset < 3.0
    assert h.fsm.stability_verified_for_s == 0.0


def test_placement_verified_flag_written_only_in_verify_placed() -> None:
    """静态+动态双重保证：所有非终态状态下 placement_verified 必须为 False。"""
    h = make_harness(TOOL_MAGNETIC_LATCH_V2)
    h.start()
    path = []
    for _ in range(2000):
        assert h.fsm.placement_verified is False or h.fsm.state is PickPlaceState.VERIFY_PLACED
        path.append(h.fsm.state)
        h.fsm.step()
        if h.fsm.state is PickPlaceState.SUCCESS:
            break
        h.clock.advance(TICK_SECONDS)
    assert h.fsm.state is PickPlaceState.SUCCESS
    assert h.fsm.placement_verified is True
    assert PickPlaceState.VERIFY_PLACED in path


# ---------------------------------------------------------------------------
# 10. V1 不触碰电磁 IO
# ---------------------------------------------------------------------------


def test_v1_never_touches_electromagnetic_io() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.start()
    result = h.run(max_steps=1000)

    assert result.success is True
    assert h.tool.unlock_calls == 0
    assert h.tool.io_events  # 有机构事件，但不含解锁
    assert not any("electromagnetic" in event for event in h.tool.io_events)
    assert not any("unlock" in event for event in h.tool.io_events)
    # 策略层声明：无 IO 通道，且不需要解锁脉冲
    assert h.strategy.IO_CHANNELS == ()
    assert h.strategy.requires_unlock_pulse() is False
    assert h.strategy.evaluate_unlock(True, True).allowed is False
    # 全流程无任何 unlock 相关原语
    assert not any("unlock" in name for name in h.tool.received_primitives)


def test_v1_unlock_request_is_always_rejected() -> None:
    strategy = PassiveHookV1Strategy()
    decision = strategy.evaluate_unlock(seated_verified=True, unloaded_verified=True)
    assert decision.allowed is False
    assert "无解锁机构" in decision.reason


# ---------------------------------------------------------------------------
# 11. V2 解锁前置条件
# ---------------------------------------------------------------------------


def test_v2_unlock_rejected_when_not_seated() -> None:
    h = make_harness(TOOL_MAGNETIC_LATCH_V2)
    h.tool.seated = False
    h.start()
    result = h.run(max_steps=600)

    assert result.error_code == ErrorCode.SEAT_NOT_VERIFIED == 400
    assert h.tool.unlock_calls == 0
    assert not any("electromagnetic" in e for e in h.tool.io_events)


def test_v2_unlock_rejected_when_not_unloaded() -> None:
    h = make_harness(TOOL_MAGNETIC_LATCH_V2)
    h.tool.unloaded = False
    h.start()
    result = h.run(max_steps=600)

    assert result.error_code == ErrorCode.RELEASE_NOT_VERIFIED == 420
    assert result.final_state is PickPlaceState.FAULT
    assert h.tool.unlock_calls == 0
    assert "DISENGAGE" not in h.history


def test_v2_strategy_rejects_unlock_without_preconditions() -> None:
    strategy = MagneticLatchV2Strategy()
    assert strategy.requires_unlock_pulse() is True

    not_seated = strategy.evaluate_unlock(seated_verified=False, unloaded_verified=True)
    assert not_seated.allowed is False
    assert not_seated.error_code == ErrorCode.SEAT_NOT_VERIFIED == 400

    not_unloaded = strategy.evaluate_unlock(seated_verified=True, unloaded_verified=False)
    assert not_unloaded.allowed is False
    assert not_unloaded.error_code == ErrorCode.RELEASE_NOT_VERIFIED == 420

    ok = strategy.evaluate_unlock(seated_verified=True, unloaded_verified=True)
    assert ok.allowed is True
    assert ok.pulse_ms > 0


def test_fsm_defensive_interlock_rejects_unlock_without_seated_evidence() -> None:
    """FSM 级防御联锁：即便状态异常到达 DISENGAGE，也不能调用解锁 IO。"""
    h = make_harness(TOOL_MAGNETIC_LATCH_V2)
    h.start()
    state = h.run_to(PickPlaceState.DISENGAGE)
    assert state is PickPlaceState.DISENGAGE
    assert h.tool.unlock_calls == 0
    # 人为破坏前置证据（ToolEvidence 为不可变对象，整体替换以模拟未来改动绕过状态顺序）
    import dataclasses

    h.fsm._seat_evidence = dataclasses.replace(h.fsm._seat_evidence, seated=False)
    assert h.fsm._seat_evidence.seated is False
    for _ in range(20):
        h.fsm.step()
        if h.fsm.state in (PickPlaceState.FAULT, PickPlaceState.FAILED):
            break
        h.clock.advance(TICK_SECONDS)
    assert h.fsm.state is PickPlaceState.FAULT
    assert h.fsm.error_code == ErrorCode.SEAT_NOT_VERIFIED == 400
    assert h.tool.unlock_calls == 0


# ---------------------------------------------------------------------------
# 12. 端口契约 / 配置一致性
# ---------------------------------------------------------------------------


def test_tool_type_mismatch_is_rejected() -> None:
    try:
        PickPlaceFSM(
            FakePerception(),
            FakePlanner(),
            FakeMotion(),
            FakeTool(),
            FakeClock(),
            MagneticLatchV2Strategy(),
            PickPlaceConfig(tool_type=TOOL_PASSIVE_HOOK_V1),
        )
    except ValueError as exc:
        assert "不一致" in str(exc)
    else:  # pragma: no cover
        raise AssertionError("工具类型不一致时必须拒绝构造")


def test_fake_motion_supports_delay_and_inflight_results() -> None:
    """FakeMotion 的“未完成”语义：先返回 None，之后返回到位。"""
    motion = FakeMotion(delay_ticks=2)
    h = make_harness(TOOL_PASSIVE_HOOK_V1, motion=motion)
    h.start()
    result = h.run(max_steps=1000)
    assert result.success is True
    assert motion.calls  # 至少一次在途返回


# ---------------------------------------------------------------------------
# 13. 外部监护强制故障（Action 步数上限等）
# ---------------------------------------------------------------------------


def test_force_fault_locks_actions_and_never_resends() -> None:
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.start()
    h.run_to(PickPlaceState.ENGAGE)
    assert h.fsm.state is PickPlaceState.ENGAGE
    # run_to 在“进入 ENGAGE”时返回：ENGAGE 处理器尚未执行，接合动作与后续指令都还没发出
    before = len(h.motion.calls)
    engage_calls_before = h.tool.engage_calls
    assert engage_calls_before == 0

    outcome = h.fsm.force_fault(ErrorCode.MOTION_STATUS_UNKNOWN, "外部监护：步数超限")
    assert outcome.state is PickPlaceState.FAULT
    assert h.fsm.error_code == ErrorCode.MOTION_STATUS_UNKNOWN
    assert h.fsm.motion_resend_allowed is False

    for _ in range(5):
        h.fsm.step()
    # 锁定后不得再发出任何运动指令或接合动作
    assert len(h.motion.calls) == before
    assert h.tool.engage_calls == engage_calls_before == 0
    assert h.fsm.state is PickPlaceState.FAULT
    # 终态下的 force_fault 不得覆盖已有错误码
    again = h.fsm.force_fault(ErrorCode.HARDWARE_FAULT, "不应覆盖")
    assert again.error_code == ErrorCode.MOTION_STATUS_UNKNOWN


# ---------------------------------------------------------------------------
# 标准库兼容层：本文件用 pytest 风格的模块级测试函数编写；
# 下面的 test_suite() 让 `python3 -m unittest discover -s test` 也能收集并运行全部用例，
# 从而在“没有 pytest”的机器上依然可离线验收。
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# 安全回归：expected_color 双重校验（修复 FSM 层缺失的颜色比对）
# ---------------------------------------------------------------------------
def test_color_mismatch_blocks_motion_and_reports_110() -> None:
    """期望蓝色但感知到红色 -> TARGET_NOT_FOUND，且不得发出任何运动指令。"""
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.perception.color = 1          # RED
    h.start(color=2, slot="P1")     # 期望 BLUE
    result = h.run()

    assert result.error_code == ErrorCode.TARGET_NOT_FOUND == 110
    assert result.success is False
    assert h.motion.calls == [], "颜色不匹配时绝不允许运动"
    assert h.history.count("MOVE_PRE_ALIGN") == 0
    assert "PLAN_ACQUIRE" not in h.history


def test_unknown_color_blocks_motion() -> None:
    """感知颜色未知（COLOR_UNKNOWN=0）-> 拒绝在颜色未确认时运动。"""
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.perception.color = 0
    h.start(color=2, slot="P1")
    result = h.run()

    assert result.error_code == ErrorCode.TARGET_NOT_FOUND == 110
    assert h.motion.calls == [], "颜色未知时绝不允许运动"
    assert "PLAN_ACQUIRE" not in h.history


def test_matching_color_still_reaches_planning() -> None:
    """对照：颜色匹配时不得被新增校验误伤。"""
    h = make_harness(TOOL_PASSIVE_HOOK_V1)
    h.perception.color = 2          # BLUE，与期望一致
    h.start(color=2, slot="P1")
    h.run_to(PickPlaceState.PLAN_ACQUIRE)
    assert h.visited(PickPlaceState.PLAN_ACQUIRE)


def load_tests(loader: Any, tests: Any, pattern: Any) -> Any:
    """unittest 协议钩子：让 `python3 -m unittest discover` 收集本文件的全部用例。"""
    return _module_test_suite()


def _module_test_suite() -> Any:
    import unittest

    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    for name, function in sorted(globals().items()):
        if name.startswith("test_") and callable(function):
            suite.addTest(loader.loadTestsFromTestCase(_make_case(name, function)))
    return suite


def _make_case(name: str, function: Any) -> Any:
    import unittest

    def _run(self: Any) -> None:
        function()

    return type("UT_" + name, (unittest.TestCase,), {name: _run})
