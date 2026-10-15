"""``ToolManager``：工具层状态机 + 证据等级 + 解锁联锁的权威实现。

关键设计（与状态机设计文档 §4.6 / §4.10 / §7.2 对齐）：

- ``state`` 与 ``verified`` 是**两个独立维度**：``STATE_ATTACHED`` 可以同时
  ``evidence_level == EVIDENCE_COMMAND_ONLY``、``verified is False``。
- 只有 ``evidence_level >= EVIDENCE_GEOMETRY`` 才算独立证据；运动命令结束
  （``COMMAND_ONLY``）与电磁铁输出信号都不算。
- V2 解锁必须同时满足「已确认落座」+「载荷已卸除」，否则返回
  ``SAFETY_INTERLOCK=500`` 且**不调用 IO 端口**。
- ``send_unlock_pulse`` 返回 ``True`` 只是「请求受理」；未取得独立证据时
  ``confirm_unlock`` 返回 ``UNLOCK_FAILED=410`` 并进入保守的 ``STATE_FAULT``。
- 分离失败或结果未知一律 ``RELEASE_NOT_VERIFIED=420`` + ``STATE_FAULT`` +
  ``withdraw_allowed=False``，绝不允许强行抽出。
- 本模块只产出 :class:`~mtc_tool.strategy.MotionPrimitive`；不含关节命令，
  不导入任何机器人 SDK。

错误码选择（一致且文档化）：

=================================  ============================================
情形                               错误码
=================================  ============================================
前置条件/联锁不满足（未落座、未卸
载、非 RELEASING、V1 被要求解锁、
接合未验证即请求释放、故障未清除）  ``SAFETY_INTERLOCK=500``
挂载证据不足却请求释放              ``ATTACH_NOT_VERIFIED=310``
解锁端口拒绝 / 超时 / 异常          ``UNLOCK_FAILED=410``
脉冲已受理但无独立解锁证据          ``UNLOCK_FAILED=410``
分离失败或结果未知                  ``RELEASE_NOT_VERIFIED=420``
非法 ``pulse_ms`` 或其他非法入参    ``EXECUTION_REJECTED=210``
=================================  ============================================
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from .codes import (
    DISENGAGE_CONFIRMED,
    DISENGAGE_FAILED,
    DISENGAGE_OUTCOMES,
    DISENGAGE_UNKNOWN,
    ERROR_ATTACH_NOT_VERIFIED,
    ERROR_EXECUTION_REJECTED,
    ERROR_OK,
    ERROR_RELEASE_NOT_VERIFIED,
    ERROR_SAFETY_INTERLOCK,
    ERROR_UNLOCK_FAILED,
    ERROR_NAMES,
    EVIDENCE_COMMAND_ONLY,
    EVIDENCE_NAMES,
    EVIDENCE_NONE,
    MIN_INDEPENDENT_EVIDENCE,
    STATE_ATTACHED,
    STATE_DETACHED,
    STATE_ENGAGING,
    STATE_FAULT,
    STATE_NAMES,
    STATE_RELEASING,
    STATE_UNKNOWN,
    evidence_at_least,
)
from .io_port import DisabledUnlockIo, UnlockIoPort
from .strategy import MotionPrimitive, PoseTarget, ToolStrategy


def is_primitive_sequence(value: object) -> bool:
    """工具层输出类型守卫：``value`` 必须是 ``MotionPrimitive`` 序列。"""

    if isinstance(value, (str, bytes)):
        return False
    if not isinstance(value, (list, tuple)):
        return False
    return all(isinstance(item, MotionPrimitive) for item in value)


def _names(mapping: Dict[int, str], code: int) -> str:
    return mapping.get(int(code), "UNKNOWN_%d" % int(code))


@dataclass(frozen=True)
class ToolStatus:
    """``ToolState.msg`` 的纯 Python 快照（含联锁审计字段）。"""

    state: int = STATE_UNKNOWN
    evidence_level: int = EVIDENCE_NONE
    verified: bool = False
    tool_type: str = "unknown"
    object_id: str = ""
    detail: str = ""
    error_code: int = ERROR_OK
    seated_verified: bool = False
    unloaded: bool = False
    unlock_request_accepted: bool = False
    unlock_confirmed: bool = False
    withdraw_allowed: bool = False

    @property
    def state_name(self) -> str:
        return _names(STATE_NAMES, self.state)

    @property
    def evidence_name(self) -> str:
        return _names(EVIDENCE_NAMES, self.evidence_level)

    @property
    def error_name(self) -> str:
        return ERROR_NAMES.get(int(self.error_code), "UNKNOWN_%d" % int(self.error_code))

    def to_tool_state_fields(self) -> Dict[str, object]:
        """映射到 ``ToolState.msg`` 的同名字段（不含 ``header``）。"""

        return {
            "state": self.state,
            "evidence_level": self.evidence_level,
            "verified": self.verified,
            "tool_type": self.tool_type,
            "object_id": self.object_id,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class PlanResult:
    """计划生成结果：只包含运动原语。"""

    ok: bool
    primitives: Tuple[MotionPrimitive, ...] = ()
    error_code: int = ERROR_OK
    detail: str = ""

    @property
    def stages(self) -> Tuple[str, ...]:
        return tuple(p.stage for p in self.primitives)


@dataclass(frozen=True)
class UnlockOutcome:
    """解锁脉冲请求结果。

    ``accepted=True`` **只表示脉冲请求已受理**，不表示电池已释放。
    """

    accepted: bool
    unlock_confirmed: bool = False
    error_code: int = ERROR_OK
    message: str = ""
    io_status: str = ""
    state: int = STATE_UNKNOWN
    evidence_level: int = EVIDENCE_NONE
    verified: bool = False


@dataclass(frozen=True)
class WithdrawDecision:
    """是否允许执行抽出/撤离路径的判决。"""

    allowed: bool
    error_code: int = ERROR_OK
    detail: str = ""
    state: int = STATE_UNKNOWN


class ToolManager:
    """统一工具策略管理（V1 被动舌规 / V2 电磁锁止）。"""

    def __init__(
        self,
        strategy: Optional[ToolStrategy] = None,
        unlock_io: Optional[UnlockIoPort] = None,
    ) -> None:
        self._lock = threading.RLock()
        self._strategy: Optional[ToolStrategy] = None
        self._io: UnlockIoPort = unlock_io if unlock_io is not None else DisabledUnlockIo(strict=True)
        self._reset_locked_state(object_id="")
        if strategy is not None:
            self.set_strategy(strategy)

    # ------------------------------------------------------------------
    # 配置
    # ------------------------------------------------------------------
    def set_strategy(self, strategy: ToolStrategy) -> None:
        if not isinstance(strategy, ToolStrategy):
            raise TypeError("strategy must be a ToolStrategy instance")
        with self._lock:
            self._strategy = strategy
            self._state = STATE_DETACHED
            self._evidence = EVIDENCE_NONE
            self._verified = False
            self._detail = "工具策略已装载：%s" % strategy.describe()
            self._error = ERROR_OK
            self._object_id = ""
            self._seated_verified = False
            self._unloaded = False
            self._unlock_accepted = False
            self._unlock_confirmed = False
            self._withdraw_allowed = False

    @property
    def strategy(self) -> Optional[ToolStrategy]:
        return self._strategy

    @property
    def unlock_io(self) -> UnlockIoPort:
        return self._io

    def _reset_locked_state(self, object_id: str) -> None:
        self._state = STATE_UNKNOWN
        self._evidence = EVIDENCE_NONE
        self._verified = False
        self._object_id = object_id
        self._detail = "未装载工具策略"
        self._error = ERROR_OK
        self._seated_verified = False
        self._unloaded = False
        self._unlock_accepted = False
        self._unlock_confirmed = False
        self._withdraw_allowed = False
        self._stage = ""

    # ------------------------------------------------------------------
    # 观测
    # ------------------------------------------------------------------
    def status(self) -> ToolStatus:
        with self._lock:
            return ToolStatus(
                state=self._state,
                evidence_level=self._evidence,
                verified=self._verified,
                tool_type=self._strategy.tool_type if self._strategy else "unknown",
                object_id=self._object_id,
                detail=self._detail,
                error_code=self._error,
                seated_verified=self._seated_verified,
                unloaded=self._unloaded,
                unlock_request_accepted=self._unlock_accepted,
                unlock_confirmed=self._unlock_confirmed,
                withdraw_allowed=self._withdraw_allowed,
            )

    @property
    def stage(self) -> str:
        with self._lock:
            return self._stage

    # ------------------------------------------------------------------
    # 内部状态迁移
    # ------------------------------------------------------------------
    def _fault(self, code: int, detail: str) -> None:
        self._state = STATE_FAULT
        self._error = int(code)
        self._detail = detail
        self._withdraw_allowed = False

    def _require_strategy(self) -> Optional[PlanResult]:
        if self._strategy is None:
            return PlanResult(False, (), ERROR_SAFETY_INTERLOCK, "未装载工具策略，拒绝生成计划")
        return None

    # ------------------------------------------------------------------
    # 抓取（PRE_ALIGN → ENGAGE → TEST_LIFT 验证）
    # ------------------------------------------------------------------
    def begin_acquire(self, object_id: str, ring_pose: PoseTarget) -> PlanResult:
        """生成接合计划并进入 ``STATE_ENGAGING``。"""

        with self._lock:
            missing = self._require_strategy()
            if missing is not None:
                return missing
            if self._state == STATE_ENGAGING:
                return PlanResult(False, (), ERROR_SAFETY_INTERLOCK, "工具已在接合过程中，拒绝并发计划")
            if self._state == STATE_FAULT:
                return PlanResult(
                    False, (), ERROR_SAFETY_INTERLOCK, "工具处于 FAULT，需人工 clear_fault 后方可继续"
                )
            if not isinstance(ring_pose, PoseTarget):
                self._fault(ERROR_EXECUTION_REJECTED, "ring_pose 非法：%r" % (ring_pose,))
                return PlanResult(False, (), ERROR_EXECUTION_REJECTED, self._detail)

            assert self._strategy is not None
            primitives = list(self._strategy.make_acquire_plan(ring_pose))
            if not is_primitive_sequence(primitives):
                self._fault(ERROR_EXECUTION_REJECTED, "策略返回了非 MotionPrimitive 输出，拒绝执行")
                return PlanResult(False, (), ERROR_EXECUTION_REJECTED, self._detail)
            self._object_id = str(object_id)
            self._state = STATE_ENGAGING
            self._evidence = EVIDENCE_COMMAND_ONLY
            self._verified = False
            self._error = ERROR_OK
            self._withdraw_allowed = False
            self._unlock_accepted = False
            self._unlock_confirmed = False
            self._seated_verified = False
            self._unloaded = False
            self._stage = primitives[-1].stage if primitives else ""
            self._detail = "接合计划已生成（%d 个运动原语），等待独立证据验证" % len(primitives)
            return PlanResult(True, tuple(primitives), ERROR_OK, self._detail)

    def notify_engage_complete(self) -> ToolStatus:
        """运动命令结束：只能得到 ``EVIDENCE_COMMAND_ONLY`` 的估计状态。"""

        with self._lock:
            self._state = STATE_ATTACHED
            self._evidence = EVIDENCE_COMMAND_ONLY
            self._verified = False
            self._error = ERROR_OK
            self._detail = "接合运动命令结束：仅命令级证据，verified=False（不得据此运输）"
            return self.status()

    def verify_attach(self, evidence_level: int, source: str = "") -> ToolStatus:
        """提交独立挂载证据（试提随动 / 锁止开关 / 视觉）。"""

        with self._lock:
            if evidence_at_least(evidence_level, MIN_INDEPENDENT_EVIDENCE):
                self._state = STATE_ATTACHED
                self._evidence = int(evidence_level)
                self._verified = True
                self._error = ERROR_OK
                self._detail = "挂载已由独立证据确认（source=%s, evidence=%s）" % (
                    source or "unspecified",
                    _names(EVIDENCE_NAMES, evidence_level),
                )
            else:
                self._evidence = int(evidence_level)
                self._verified = False
                self._error = ERROR_ATTACH_NOT_VERIFIED
                self._detail = "挂载未验证（source=%s, evidence=%s）：禁止运输" % (
                    source or "unspecified",
                    _names(EVIDENCE_NAMES, evidence_level),
                )
            return self.status()

    # ------------------------------------------------------------------
    # 释放（SEAT → UNLOAD → DISENGAGE）
    # ------------------------------------------------------------------
    def begin_release(self, seat_pose: PoseTarget) -> PlanResult:
        """生成释放计划并进入 ``STATE_RELEASING``（重置落座/卸载标志）。"""

        with self._lock:
            missing = self._require_strategy()
            if missing is not None:
                return missing
            if not self._verified or self._state != STATE_ATTACHED:
                self._error = ERROR_ATTACH_NOT_VERIFIED
                self._detail = "挂载未验证，拒绝进入释放/运输阶段"
                return PlanResult(False, (), ERROR_ATTACH_NOT_VERIFIED, self._detail)
            if not isinstance(seat_pose, PoseTarget):
                self._fault(ERROR_EXECUTION_REJECTED, "seat_pose 非法：%r" % (seat_pose,))
                return PlanResult(False, (), ERROR_EXECUTION_REJECTED, self._detail)

            assert self._strategy is not None
            primitives = list(self._strategy.make_release_plan(seat_pose))
            if not is_primitive_sequence(primitives):
                self._fault(ERROR_EXECUTION_REJECTED, "策略返回了非 MotionPrimitive 输出，拒绝执行")
                return PlanResult(False, (), ERROR_EXECUTION_REJECTED, self._detail)
            self._state = STATE_RELEASING
            self._evidence = EVIDENCE_COMMAND_ONLY
            self._verified = False
            self._seated_verified = False
            self._unloaded = False
            self._unlock_accepted = False
            self._unlock_confirmed = False
            self._withdraw_allowed = False
            self._error = ERROR_OK
            self._stage = primitives[0].stage if primitives else ""
            self._detail = "释放计划已生成（%d 个运动原语）；解锁前必须确认落座与卸载" % len(primitives)
            return PlanResult(True, tuple(primitives), ERROR_OK, self._detail)

    def mark_seated(self, seated_verified: bool, evidence_level: int = EVIDENCE_NONE, source: str = "") -> ToolStatus:
        """记录「电池已由桌面承托」的确认结果。"""

        with self._lock:
            self._seated_verified = bool(seated_verified)
            if self._seated_verified:
                self._evidence = max(self._evidence, int(evidence_level))
            self._detail = "落座确认 seated_verified=%s（source=%s, evidence=%s）" % (
                self._seated_verified,
                source or "unspecified",
                _names(EVIDENCE_NAMES, evidence_level),
            )
            return self.status()

    def mark_unloaded(self, unloaded: bool, evidence_level: int = EVIDENCE_NONE, source: str = "") -> ToolStatus:
        """记录「倒钩/锁扣已不承重」的确认结果。"""

        with self._lock:
            self._unloaded = bool(unloaded)
            if self._unloaded:
                self._evidence = max(self._evidence, int(evidence_level))
            self._detail = "卸载确认 unloaded=%s（source=%s, evidence=%s）" % (
                self._unloaded,
                source or "unspecified",
                _names(EVIDENCE_NAMES, evidence_level),
            )
            return self.status()

    # ------------------------------------------------------------------
    # 解锁脉冲（V2）
    # ------------------------------------------------------------------
    def check_unlock_preconditions(self) -> Optional[Tuple[int, str]]:
        """返回 ``None`` 表示联锁通过；否则返回 ``(error_code, detail)``。"""

        with self._lock:
            if self._strategy is None:
                return (ERROR_SAFETY_INTERLOCK, "未装载工具策略")
            if not self._strategy.requires_unlock_pulse():
                return (
                    ERROR_SAFETY_INTERLOCK,
                    "工具 %s 不需要解锁脉冲：V1 不得依赖电磁 IO" % self._strategy.tool_type,
                )
            if self._state != STATE_RELEASING:
                return (
                    ERROR_SAFETY_INTERLOCK,
                    "工具未处于 RELEASING 阶段（当前 %s）" % _names(STATE_NAMES, self._state),
                )
            if not self._seated_verified:
                return (ERROR_SAFETY_INTERLOCK, "未确认落座（seated_verified=False）：禁止发送解锁脉冲")
            if not self._unloaded:
                return (ERROR_SAFETY_INTERLOCK, "载荷未卸除（unloaded=False）：禁止发送解锁脉冲")
            return None

    def trigger_unlock(
        self,
        request_id: str,
        pulse_ms: int,
        seated_verified: Optional[bool] = None,
        unloaded: Optional[bool] = None,
    ) -> UnlockOutcome:
        """请求解锁脉冲。

        前置条件不满足时返回 ``SAFETY_INTERLOCK=500`` 且不触碰 IO 端口；
        ``accepted=True`` 仅表示脉冲请求被受理，绝不代表解锁成功。
        """

        with self._lock:
            if seated_verified is not None:
                self._seated_verified = bool(seated_verified)
            if unloaded is not None:
                self._unloaded = bool(unloaded)

            if not isinstance(pulse_ms, int) or isinstance(pulse_ms, bool) or pulse_ms <= 0:
                # 非法入参不属于安全联锁，按执行层拒绝处理；不调用 IO。
                self._error = ERROR_EXECUTION_REJECTED
                self._detail = "非法 pulse_ms=%r（request_id=%s）：拒绝发送" % (pulse_ms, request_id)
                return UnlockOutcome(
                    False, False, ERROR_EXECUTION_REJECTED, self._detail, "", self._state,
                    self._evidence, self._verified,
                )

            failed = self.check_unlock_preconditions()
            if failed is not None:
                code, detail = failed
                # 联锁拒绝：仅记录，不改变保守状态，不调用 IO。
                self._error = code
                self._detail = "%s（request_id=%s）" % (detail, request_id)
                return UnlockOutcome(
                    False, False, code, self._detail, "", self._state, self._evidence, self._verified
                )

            try:
                accepted = bool(self._io.send_unlock_pulse(int(pulse_ms)))
                io_status = self._io.last_status
            except Exception as exc:  # 端口未实现/被禁用/驱动异常：不能当作受理
                self._fault(ERROR_UNLOCK_FAILED, "解锁 IO 异常（%s）：%s" % (type(exc).__name__, exc))
                return UnlockOutcome(
                    False, False, ERROR_UNLOCK_FAILED, self._detail, self._io.last_status,
                    self._state, self._evidence, self._verified,
                )

            if not accepted:
                self._fault(
                    ERROR_UNLOCK_FAILED,
                    "解锁脉冲未被受理（io_status=%s, request_id=%s）：进入 FAULT" % (io_status, request_id),
                )
                return UnlockOutcome(
                    False, False, ERROR_UNLOCK_FAILED, self._detail, io_status,
                    self._state, self._evidence, self._verified,
                )

            # 受理 ≠ 解锁。保持 RELEASING 且 verified=False，等待独立证据。
            self._unlock_accepted = True
            self._unlock_confirmed = False
            self._withdraw_allowed = False
            self._evidence = EVIDENCE_COMMAND_ONLY
            self._verified = False
            self._error = ERROR_OK
            self._detail = (
                "解锁脉冲请求已受理（io_status=%s, pulse_ms=%d, request_id=%s）："
                "尚未解锁，等待独立证据" % (io_status, int(pulse_ms), request_id)
            )
            return UnlockOutcome(
                True, False, ERROR_OK, self._detail, io_status, self._state,
                self._evidence, self._verified,
            )

    def confirm_unlock(self, evidence_level: int, source: str = "") -> UnlockOutcome:
        """提交解锁独立证据（锁止开关反馈 / 视觉确认）。"""

        with self._lock:
            if not self._unlock_accepted:
                self._fault(ERROR_UNLOCK_FAILED, "未受理过解锁脉冲请求，无法确认解锁")
                return UnlockOutcome(
                    False, False, ERROR_UNLOCK_FAILED, self._detail, self._io.last_status,
                    self._state, self._evidence, self._verified,
                )
            if not evidence_at_least(evidence_level, MIN_INDEPENDENT_EVIDENCE):
                self._unlock_confirmed = False
                self._withdraw_allowed = False
                self._verified = False
                self._fault(
                    ERROR_UNLOCK_FAILED,
                    "解锁缺少独立证据（source=%s, evidence=%s）：保持保守 FAULT，不宣布解锁成功"
                    % (source or "unspecified", _names(EVIDENCE_NAMES, evidence_level)),
                )
                return UnlockOutcome(
                    False, False, ERROR_UNLOCK_FAILED, self._detail, self._io.last_status,
                    self._state, self._evidence, self._verified,
                )
            self._unlock_confirmed = True
            self._evidence = int(evidence_level)
            self._verified = True
            self._error = ERROR_OK
            self._detail = "解锁已由独立证据确认（source=%s, evidence=%s），可执行退出路径" % (
                source or "unspecified",
                _names(EVIDENCE_NAMES, evidence_level),
            )
            return UnlockOutcome(
                True, True, ERROR_OK, self._detail, self._io.last_status, self._state,
                self._evidence, self._verified,
            )

    # ------------------------------------------------------------------
    # 分离与撤离
    # ------------------------------------------------------------------
    def request_withdraw(self) -> WithdrawDecision:
        """在执行任何抽出动作前调用；未验证一律拒绝。"""

        with self._lock:
            if self._strategy is None:
                return WithdrawDecision(False, ERROR_SAFETY_INTERLOCK, "未装载工具策略", self._state)
            if self._state == STATE_DETACHED and self._verified:
                self._withdraw_allowed = True
                self._detail = "分离已验证，允许撤离"
                return WithdrawDecision(True, ERROR_OK, self._detail, self._state)
            if self._strategy.requires_unlock_pulse() and not self._unlock_confirmed:
                self._fault(
                    ERROR_RELEASE_NOT_VERIFIED,
                    "解锁未确认（unlock_confirmed=False）：禁止强行抽出，进入 FAULT",
                )
            else:
                self._fault(
                    ERROR_RELEASE_NOT_VERIFIED,
                    "分离结果未知（state=%s）：禁止强行抽出，进入 FAULT" % _names(STATE_NAMES, self._state),
                )
            return WithdrawDecision(False, ERROR_RELEASE_NOT_VERIFIED, self._detail, self._state)

    def resolve_disengage(
        self,
        outcome: str,
        evidence_level: int = EVIDENCE_NONE,
        source: str = "",
    ) -> WithdrawDecision:
        """汇总 DISENGAGE 结果并给出撤离判决。

        - ``CONFIRMED`` 且证据达到 ``EVIDENCE_GEOMETRY``（V2 还需解锁已确认）
          → ``STATE_DETACHED`` + ``verified=True`` + ``withdraw_allowed=True``。
        - ``FAILED`` / ``UNKNOWN`` / 证据不足 → ``RELEASE_NOT_VERIFIED=420`` +
          ``STATE_FAULT`` + ``withdraw_allowed=False``。
        """

        if outcome not in DISENGAGE_OUTCOMES:
            raise ValueError("unsupported disengage outcome: %r" % (outcome,))
        with self._lock:
            self._stage = "DISENGAGE"
            if outcome == DISENGAGE_CONFIRMED:
                if self._strategy is not None and self._strategy.requires_unlock_pulse() and not self._unlock_confirmed:
                    self._fault(ERROR_UNLOCK_FAILED, "解锁未确认，不能接受分离成功结论：进入 FAULT")
                    return WithdrawDecision(False, ERROR_UNLOCK_FAILED, self._detail, self._state)
                if not evidence_at_least(evidence_level, MIN_INDEPENDENT_EVIDENCE):
                    self._fault(
                        ERROR_RELEASE_NOT_VERIFIED,
                        "分离结果无独立证据（source=%s, evidence=%s）：禁止抽出"
                        % (source or "unspecified", _names(EVIDENCE_NAMES, evidence_level)),
                    )
                    return WithdrawDecision(False, ERROR_RELEASE_NOT_VERIFIED, self._detail, self._state)
                self._state = STATE_DETACHED
                self._evidence = int(evidence_level)
                self._verified = True
                self._withdraw_allowed = True
                self._error = ERROR_OK
                self._detail = "分离已确认（source=%s, evidence=%s）：允许撤离" % (
                    source or "unspecified",
                    _names(EVIDENCE_NAMES, evidence_level),
                )
                return WithdrawDecision(True, ERROR_OK, self._detail, self._state)

            if outcome == DISENGAGE_FAILED:
                self._fault(
                    ERROR_RELEASE_NOT_VERIFIED,
                    "DISENGAGE 明确失败（source=%s）：禁止强行抽出，进入 FAULT" % (source or "unspecified",),
                )
            else:  # DISENGAGE_UNKNOWN
                self._fault(
                    ERROR_RELEASE_NOT_VERIFIED,
                    "DISENGAGE 结果未知（source=%s）：不得强行抽出，进入 FAULT" % (source or "unspecified",),
                )
            return WithdrawDecision(False, ERROR_RELEASE_NOT_VERIFIED, self._detail, self._state)

    # ------------------------------------------------------------------
    # 故障处理
    # ------------------------------------------------------------------
    def clear_fault(self, operator_note: str) -> bool:
        """人工清除 FAULT：必须提供说明，清除后回到 ``STATE_UNKNOWN``。"""

        with self._lock:
            if self._state != STATE_FAULT:
                return False
            if not operator_note or not str(operator_note).strip():
                raise ValueError("clear_fault 需要人工核验说明（operator_note）")
            self._state = STATE_UNKNOWN
            self._verified = False
            self._evidence = EVIDENCE_NONE
            self._seated_verified = False
            self._unloaded = False
            self._unlock_accepted = False
            self._unlock_confirmed = False
            self._withdraw_allowed = False
            self._stage = ""
            self._error = ERROR_OK
            self._detail = "FAULT 已由人工清除：%s" % str(operator_note)
            return True
