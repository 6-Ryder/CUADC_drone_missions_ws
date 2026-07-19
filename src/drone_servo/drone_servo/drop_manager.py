"""
投放逻辑管理 — 多位置步进投放.

投放序列:
  positions = [1150, 1700, 2150]  (μs)
  初始:    舵机 = 1150μs (闭合)
  投放 #1: 舵机 → 1700μs → 等待 drop_delay_ms
  投放 #2: 舵机 → 2150μs → 等待 drop_delay_ms

与 C++ DropManager 逻辑完全一致.
"""

import time
from enum import Enum
from typing import Callable, List, Optional


class DropState(Enum):
    IDLE = "idle"
    MOVING = "moving"
    DONE = "done"
    FAILED = "failed"


class DropManager:
    """
    多位置步进投放管理器.

    使用方式:
      mgr = DropManager(servo, positions=[1150, 1700, 2150], delay_ms=1000)
      mgr.set_on_drop_done(lambda success, cyl_id: print(f"drop={success}"))
      success = mgr.execute_drop(cylinder_id=1)
    """

    def __init__(self, servo, positions: List[float],
                 delay_ms: float = 1000.0, timeout_s: float = 3.0):
        """
        Parameters
        ----------
        servo : ServoMock or PX4CommandServo
            舵机控制实例 (必须有 set_pwm(pwm_us) 方法)
        positions : list[float]
            PWM 位置序列 (μs), 长度 = 投放次数 + 1
        delay_ms : float
            投放后保持延时 (ms)
        timeout_s : float
            单次投放超时 (s)
        """
        if len(positions) < 2:
            raise ValueError("positions 至少需要 2 个值")

        self._servo = servo
        self._positions = list(positions)
        self._max_drops = len(positions) - 1
        self._delay_s = delay_ms / 1000.0
        self._timeout = timeout_s

        self._state = DropState.IDLE
        self._drop_count: int = 0
        self._last_result: Optional[bool] = None

        self._on_drop_done: Optional[Callable[[bool, int], None]] = None

    def set_on_drop_done(self, cb: Callable[[bool, int], None]):
        """设置投放完成回调."""
        self._on_drop_done = cb

    def execute_drop(self, cylinder_id: int = -1) -> bool:
        """
        执行一次投放: 舵机移动到 positions[drop_count + 1].

        Returns
        -------
        bool: 投放是否成功
        """
        if self._drop_count >= self._max_drops:
            self._state = DropState.FAILED
            self._notify_done(False, cylinder_id)
            return False

        if not self._servo.is_ready():
            self._state = DropState.FAILED
            self._notify_done(False, cylinder_id)
            return False

        self._state = DropState.MOVING
        target_pwm = self._positions[self._drop_count + 1]

        try:
            success = self._servo.set_pwm(target_pwm)
            if success:
                time.sleep(self._delay_s)
        except Exception:
            success = False

        if success:
            self._drop_count += 1
            self._state = DropState.DONE
        else:
            self._state = DropState.FAILED

        self._last_result = success
        self._notify_done(success, cylinder_id)
        return success

    def can_drop(self) -> bool:
        """是否还可以执行投放."""
        return (
            self._drop_count < self._max_drops
            and self._servo.is_ready()
            and self._state != DropState.MOVING
        )

    def reset(self):
        """重置投放管理器, 舵机回到初始位置."""
        self._state = DropState.IDLE
        self._drop_count = 0
        self._last_result = None
        self._servo.set_pwm(self._positions[0])

    # ---- 属性 ----

    @property
    def state(self) -> DropState:
        return self._state

    @property
    def drop_count(self) -> int:
        return self._drop_count

    @property
    def max_drops(self) -> int:
        return self._max_drops

    @property
    def drops_remaining(self) -> int:
        return max(0, self._max_drops - self._drop_count)

    @property
    def last_result(self) -> Optional[bool]:
        return self._last_result

    @property
    def current_pwm(self) -> float:
        return self._positions[self._drop_count]

    @property
    def positions(self) -> List[float]:
        return list(self._positions)

    # ---- 内部 ----

    def _notify_done(self, success: bool, cylinder_id: int):
        if self._on_drop_done:
            try:
                self._on_drop_done(success, cylinder_id)
            except Exception:
                pass
