"""电磁解锁 IO 端口抽象与离线替身实现。

安全边界（必须保持）：

- **唯一**的对外副作用出口是 :meth:`UnlockIoPort.send_unlock_pulse`。
- 真实硬件路径尚未实现，也**默认禁用**：:class:`DisabledUnlockIo` 用于一切真实运行，
  它绝不触碰任何 GPIO/CAN/串口/网络设备。
- :class:`MockUnlockIo` 只用于离线测试，可配置受理 / 拒绝 / 超时 / 抛错。

语义提醒：``send_unlock_pulse`` 返回 ``True`` **只表示脉冲请求被受理**，
不表示电池已释放、也不表示机械锁止已解除（设计文档 §4.10）。
"""

from __future__ import annotations

import abc
from typing import List, Sequence


class UnlockIoDisabledError(RuntimeError):
    """真实（或未实现）解锁 IO 被显式禁用时抛出。"""


class UnlockIoPort(abc.ABC):
    """解锁脉冲端口抽象。

    实现者必须做到：

    1. 记录 ``call_count`` / ``last_status`` / ``history``，便于测试与日志审计。
    2. 不在内部推断解锁结果——受理与解锁成功是两个概念。
    """

    STATUS_IDLE = "IDLE"
    STATUS_ACCEPTED = "ACCEPTED"
    STATUS_REJECTED = "REJECTED"
    STATUS_TIMEOUT = "TIMEOUT"
    STATUS_DISABLED = "DISABLED"
    STATUS_ERROR = "ERROR"

    def __init__(self, name: str) -> None:
        self._name = name
        self._call_count = 0
        self._last_status = self.STATUS_IDLE
        self._history: List[int] = []

    # -- 只读审计信息 ------------------------------------------------------
    @property
    def name(self) -> str:
        return self._name

    @property
    def call_count(self) -> int:
        """``send_unlock_pulse`` 被调用的次数（含被拒绝/超时/抛错的调用）。"""

        return self._call_count

    @property
    def last_status(self) -> str:
        return self._last_status

    @property
    def history(self) -> Sequence[int]:
        """按顺序记录的 ``pulse_ms`` 请求。"""

        return tuple(self._history)

    def reset_audit(self) -> None:
        self._call_count = 0
        self._last_status = self.STATUS_IDLE
        self._history = []

    # -- 记录辅助 ----------------------------------------------------------
    def _record(self, pulse_ms: int, status: str) -> None:
        self._call_count += 1
        self._history.append(int(pulse_ms))
        self._last_status = status

    @abc.abstractmethod
    def send_unlock_pulse(self, pulse_ms: int) -> bool:
        """请求发送一个 ``pulse_ms`` 毫秒的解锁脉冲。

        Returns:
            bool: ``True`` 表示脉冲请求被受理（**不代表解锁成功**）。
        """

        raise NotImplementedError


class MockUnlockIo(UnlockIoPort):
    """离线替身：可配置成功 / 失败 / 超时 / 抛错。"""

    MODE_ACCEPT = "accept"
    MODE_REJECT = "reject"
    MODE_TIMEOUT = "timeout"
    MODE_RAISE = "raise"

    MODES = (MODE_ACCEPT, MODE_REJECT, MODE_TIMEOUT, MODE_RAISE)

    def __init__(
        self,
        mode: str = MODE_ACCEPT,
        script: Sequence[str] | None = None,
        name: str = "mock_unlock_io",
    ) -> None:
        super().__init__(name)
        if mode not in self.MODES:
            raise ValueError("unsupported mock mode: %r" % (mode,))
        self._default_mode = mode
        self._script = [str(m) for m in (script or [])]
        for m in self._script:
            if m not in self.MODES:
                raise ValueError("unsupported scripted mock mode: %r" % (m,))

    @property
    def mode(self) -> str:
        return self._default_mode

    def _mode_for_call(self, index: int) -> str:
        if index < len(self._script):
            return self._script[index]
        return self._default_mode

    def send_unlock_pulse(self, pulse_ms: int) -> bool:
        mode = self._mode_for_call(self._call_count)
        if mode == self.MODE_ACCEPT:
            self._record(pulse_ms, self.STATUS_ACCEPTED)
            return True
        if mode == self.MODE_REJECT:
            self._record(pulse_ms, self.STATUS_REJECTED)
            return False
        if mode == self.MODE_TIMEOUT:
            # 超时：未收到明确受理确认 → 不能当作受理成功。
            self._record(pulse_ms, self.STATUS_TIMEOUT)
            return False
        # MODE_RAISE：驱动异常，端口本身无法给出结论。
        self._record(pulse_ms, self.STATUS_ERROR)
        raise UnlockIoDisabledError("mock unlock io raised for pulse_ms=%s" % (pulse_ms,))


class DisabledUnlockIo(UnlockIoPort):
    """真实 IO 的**默认禁用**实现。

    :param strict: ``True``（默认）时任何调用都抛 :class:`UnlockIoDisabledError`；
        ``False`` 时返回明确拒绝 ``False``。两种模式都不会触碰真实设备。
    """

    def __init__(self, strict: bool = True, name: str = "disabled_unlock_io") -> None:
        super().__init__(name)
        self._strict = bool(strict)

    @property
    def strict(self) -> bool:
        return self._strict

    def send_unlock_pulse(self, pulse_ms: int) -> bool:
        self._record(pulse_ms, self.STATUS_DISABLED)
        if self._strict:
            raise UnlockIoDisabledError(
                "真实电磁解锁 IO 已被禁用（DisabledUnlockIo），拒绝 pulse_ms=%s" % (pulse_ms,)
            )
        return False
