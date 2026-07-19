"""
SITL 模拟舵机 — 记录投放事件到日志, 发布模拟状态.

在 SITL 模式下替代真实 GPIO/PWM 硬件操作,
所有投放指令仅做日志记录和状态模拟.
"""

import time
from typing import Optional


class ServoMock:
    """
    SITL 模拟舵机.

    行为:
      - drop(): 延时 drop_delay_ms 后返回 True (模拟成功)
      - 记录每条操作的日志
      - 通过回调向外通知状态变化
    """

    def __init__(self, drop_delay_ms: float = 1000.0, drop_timeout_s: float = 3.0):
        self._drop_delay = drop_delay_ms / 1000.0  # 转为秒
        self._drop_timeout = drop_timeout_s
        self._is_open = False
        self._last_drop_time: float = 0.0
        self._drop_count: int = 0
        self._on_state_change: Optional[callable] = None

    def set_state_callback(self, callback):
        """设置状态变化回调: callback(is_open, drop_count)."""
        self._on_state_change = callback

    def open(self) -> bool:
        """打开投放机构."""
        self._is_open = True
        self._notify()
        return True

    def close(self) -> bool:
        """关闭投放机构."""
        self._is_open = False
        self._notify()
        return True

    def drop(self) -> bool:
        """
        执行一次完整投放动作: open → delay → close.

        Returns
        -------
        bool: True 表示投放成功 (模拟总是成功)
        """
        self._drop_count += 1
        self._last_drop_time = time.time()

        # 模拟机械动作延迟
        self.open()
        time.sleep(self._drop_delay)
        self.close()

        return True

    def is_ready(self) -> bool:
        """检查舵机是否就绪 (SITL 中总是就绪)."""
        return True

    def is_open(self) -> bool:
        return self._is_open

    def reset(self):
        """重置舵机状态."""
        self._is_open = False
        self._drop_count = 0
        self._notify()

    @property
    def drop_count(self) -> int:
        return self._drop_count

    @property
    def status_dict(self) -> dict:
        return {
            "is_open": self._is_open,
            "drop_count": self._drop_count,
            "last_drop_time": self._last_drop_time,
            "ready": self.is_ready(),
        }

    def _notify(self):
        if self._on_state_change:
            try:
                self._on_state_change(self._is_open, self._drop_count)
            except Exception:
                pass
