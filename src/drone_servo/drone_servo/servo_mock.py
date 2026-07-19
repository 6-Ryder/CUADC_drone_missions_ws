"""
SITL 模拟舵机 — 多位置步进投放.

与 C++ drone_servo2 逻辑一致:
  - set_pwm(): 设置 PWM 脉宽 (μs)
  - 投放序列: [1150, 1700, 2150] → 三次位置, 两次投放
  - 舵机只进不退
"""

from typing import Optional


class ServoMock:
    """SITL 模拟舵机 — 与 PX4CommandServo 接口对齐."""

    def __init__(self, init_pwm_us: float = 1150.0):
        self._current_pwm = init_pwm_us
        self._init_pwm = init_pwm_us
        self._on_pwm_change: Optional[callable] = None

    def set_pwm_change_callback(self, callback):
        """设置 PWM 变化回调: callback(pwm_us)."""
        self._on_pwm_change = callback

    def set_pwm(self, pwm_us: float) -> bool:
        """设置 PWM 输出."""
        self._current_pwm = pwm_us
        if self._on_pwm_change:
            try:
                self._on_pwm_change(pwm_us)
            except Exception:
                pass
        return True

    def is_ready(self) -> bool:
        return True

    def reset(self):
        """回到初始位置."""
        self._current_pwm = self._init_pwm

    @property
    def current_pwm(self) -> float:
        return self._current_pwm

    @property
    def status_dict(self) -> dict:
        return {
            "pwm_us": self._current_pwm,
            "ready": 1.0,
        }
