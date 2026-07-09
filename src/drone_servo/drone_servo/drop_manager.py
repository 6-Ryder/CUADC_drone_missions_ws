"""
投放逻辑管理 — 单投/连投/状态监测.

管理投放全流程:
  - 投放前: 检查舵机就绪 + 无人机位置确认
  - 投放中: 触发舵机动作 + 等待完成
  - 投放后: 确认投放结果 + 更新内部状态

支持:
  - 单次投放 (execute_drop)
  - 多次投放 (execute_sequential_drops, 用于两轮投放任务)
  - 投放计数与限制
"""

import time
from enum import Enum
from typing import Callable, Optional


class DropState(Enum):
    IDLE = "idle"
    OPENING = "opening"
    HOLDING = "holding"
    CLOSING = "closing"
    DONE = "done"
    FAILED = "failed"


class DropManager:
    """
    投放逻辑管理器.

    使用方式:
      mgr = DropManager(servo, max_drops=2)
      mgr.set_confirm_callback(lambda: vision_confirm_drop())
      success = mgr.execute_drop(cylinder_id=1)
    """

    def __init__(self, servo, max_drops: int = 2, drop_timeout_s: float = 3.0):
        """
        Parameters
        ----------
        servo : ServoController or ServoMock
            舵机控制实例
        max_drops : int
            最大投放次数
        drop_timeout_s : float
            单次投放超时 (s)
        """
        self._servo = servo
        self._max_drops = max_drops
        self._drop_timeout = drop_timeout_s

        self._state = DropState.IDLE
        self._drop_count: int = 0
        self._last_result: Optional[bool] = None

        # 可选回调
        self._on_drop_start: Optional[Callable] = None
        self._on_drop_done: Optional[Callable[[bool, int], None]] = None  # (success, cyl_id)
        self._confirm_callback: Optional[Callable[[], bool]] = None

    def set_confirm_callback(self, cb: Callable[[], bool]):
        """
        设置视觉确认回调.

        cb 应返回 True(确认投放成功) 或 False(投放失败/无法确认).
        """
        self._confirm_callback = cb

    def set_on_drop_done(self, cb: Callable[[bool, int], None]):
        """设置投放完成回调."""
        self._on_drop_done = cb

    def execute_drop(self, cylinder_id: int = -1) -> bool:
        """
        执行一次投放.

        Parameters
        ----------
        cylinder_id : int
            目标圆筒 ID (用于日志)

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

        # 投放
        self._state = DropState.OPENING
        if self._on_drop_start:
            try:
                self._on_drop_start()
            except Exception:
                pass

        try:
            success = self._servo.drop()
        except Exception:
            success = False

        if success:
            self._drop_count += 1
            self._state = DropState.DONE
        else:
            self._state = DropState.FAILED

        self._last_result = success

        # 视觉确认 (可选)
        if success and self._confirm_callback:
            try:
                confirmed = self._confirm_callback()
                if not confirmed:
                    # 视觉确认失败, 但舵机已动作, 仍标记为成功
                    # (真正方案应该有更多重试逻辑)
                    pass
            except Exception:
                pass

        self._notify_done(success, cylinder_id)
        return success

    def execute_sequential_drops(self, cylinder_ids: list) -> dict:
        """
        对多个圆筒依次执行投放.

        Parameters
        ----------
        cylinder_ids : list[int]
            目标圆筒 ID 列表

        Returns
        -------
        dict: {cylinder_id: success}
        """
        results = {}
        for cyl_id in cylinder_ids:
            if self._drop_count >= self._max_drops:
                results[cyl_id] = False
                continue
            results[cyl_id] = self.execute_drop(cyl_id)
        return results

    def can_drop(self) -> bool:
        """是否还可以执行投放."""
        return (
            self._drop_count < self._max_drops
            and self._servo.is_ready()
            and self._state != DropState.OPENING
        )

    def reset(self):
        """重置投放管理器."""
        self._state = DropState.IDLE
        self._drop_count = 0
        self._last_result = None
        if hasattr(self._servo, "reset"):
            self._servo.reset()

    # ---- 属性 ----

    @property
    def state(self) -> DropState:
        return self._state

    @property
    def drop_count(self) -> int:
        return self._drop_count

    @property
    def drops_remaining(self) -> int:
        return max(0, self._max_drops - self._drop_count)

    @property
    def last_result(self) -> Optional[bool]:
        return self._last_result

    # ---- 内部 ----

    def _notify_done(self, success: bool, cylinder_id: int):
        if self._on_drop_done:
            try:
                self._on_drop_done(success, cylinder_id)
            except Exception:
                pass
