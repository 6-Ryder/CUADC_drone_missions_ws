"""
紧急处理 — 失控保护, 低电量返航, 通信中断, 视觉丢失.

监测条件:
  - BATTERY_LOW: 电量 < warning 阈值 → 记录告警
  - BATTERY_CRITICAL: 电量 < critical 阈值 → 强制返航
  - OFFBOARD_LOST: Offboard 心跳超时 → 切换到 LOITER (PX4 自动处理)
  - VISION_LOST: 视觉连续 N 秒无有效检测 → 切换到盲降模式
  - TIMEOUT: 单状态超时 → 重试一次 → 仍超时则中止

返回:
  "ok"      — 正常继续
  "warn"    — 需要关注但可继续
  "rtl"     — 立即返航降落
  "land"    — 原地紧急降落
  "abort"   — 任务中止
"""

import time
from enum import Enum
from typing import Optional


class EmergencyLevel(Enum):
    OK = "ok"
    WARN = "warn"
    RTL = "rtl"
    LAND = "land"
    ABORT = "abort"


class EmergencyHandler:
    """
    紧急情况分级处理器.

    使用方式:
      handler = EmergencyHandler(battery_warning=0.25, battery_critical=0.15)
      # 每循环:
      handler.update_battery(0.35)
      handler.update_vision(has_detection=True)
      level = handler.check()
      if level != EmergencyLevel.OK:
          ...
    """

    def __init__(
        self,
        battery_warning: float = 0.25,
        battery_critical: float = 0.15,
        vision_loss_timeout: float = 3.0,
        state_max_duration: float = 180.0,
    ):
        # 电池阈值 (比例, 0~1)
        self.battery_warning = battery_warning
        self.battery_critical = battery_critical

        # 视觉丢失超时
        self.vision_loss_timeout = vision_loss_timeout

        # 单状态最大持续时间
        self.state_max_duration = state_max_duration

        # ---- 实时状态 ----
        self._battery_remaining: float = 1.0
        self._battery_voltage: float = 16.8  # 4S LiPo 满电

        self._vision_last_seen: float = time.time()
        self._vision_has_detection: bool = False

        self._offboard_alive: bool = True
        self._offboard_last_heartbeat: float = time.time()

        self._state_start_time: float = time.time()
        self._current_state: str = "INIT"
        self._state_timeout: Optional[float] = None  # 每状态可单独设置超时

        self._gps_fix: bool = True
        self._gps_satellites: int = 10

        # ---- 统计 ----
        self._warning_count: int = 0
        self._error_count: int = 0
        self._max_errors: int = 5
        self._last_level: EmergencyLevel = EmergencyLevel.OK

        # 每个条件 "已经从 OK 转为告警" 的标记, 防止 check() 高频重复计数
        self._offboard_active: bool = False
        self._vision_active: bool = False
        self._timeout_active: bool = False
        self._gps_active: bool = False

        # 电池 debounce: 防止瞬时电流波动触发误报
        self._battery_critical_count: int = 0
        self._battery_critical_debounce_ticks: int = 10  # 20Hz 下=0.5s
        self._battery_warning_active: bool = False  # 只在首次触发时计数

    # ================================================================
    # 更新接口
    # ================================================================

    def update_battery(self, remaining: float, voltage: float = 0.0):
        """更新电池状态."""
        self._battery_remaining = max(0.0, min(1.0, remaining))
        if voltage > 0:
            self._battery_voltage = voltage

    def update_vision(self, has_detection: bool):
        """更新视觉检测状态."""
        if has_detection:
            self._vision_last_seen = time.time()
        self._vision_has_detection = has_detection

    def update_offboard(self, alive: bool):
        """更新 Offboard 连接状态."""
        self._offboard_alive = alive
        if alive:
            self._offboard_last_heartbeat = time.time()

    def update_gps(self, fix: bool, satellites: int = 0):
        """更新 GPS 状态."""
        self._gps_fix = fix
        self._gps_satellites = satellites

    def reset_state_timer(self, timeout: Optional[float] = None):
        """重置状态计时器 (状态切换时调用)."""
        self._state_start_time = time.time()
        self._state_timeout = timeout

    def set_state(self, state_name: str, timeout: Optional[float] = None):
        """记录当前状态并重置计时."""
        self._current_state = state_name
        self.reset_state_timer(timeout)

    # ================================================================
    # 检查
    # ================================================================

    def check(self) -> EmergencyLevel:
        """
        执行完整紧急检查.

        核心原则: 每个条件只在第一次触发时计一次数, 条件解除后重置标记,
        避免 20Hz 循环导致瞬间累积到 max_errors.

        Returns
        -------
        EmergencyLevel: 当前紧急等级
        """
        now = time.time()

        # 1. 严重低电量 → debounce 后再触发 RTL, 防止瞬时电流波动误判
        if self._battery_remaining <= self.battery_critical:
            self._battery_critical_count += 1
            if self._battery_critical_count >= self._battery_critical_debounce_ticks:
                self._last_level = EmergencyLevel.RTL
                return EmergencyLevel.RTL
            self._last_level = EmergencyLevel.WARN
            return EmergencyLevel.WARN
        else:
            self._battery_critical_count = max(0, self._battery_critical_count - 1)

        # 2. 低电量告警 — 仅在首次触发时计一次, 防止 20Hz 循环炸穿计数器
        if self._battery_remaining <= self.battery_warning:
            if not self._battery_warning_active:
                self._battery_warning_active = True
                self._warning_count += 1
            self._last_level = EmergencyLevel.WARN
            return EmergencyLevel.WARN
        else:
            self._battery_warning_active = False

        # 3. Offboard 丢失 → 只在首次触发时计一次 error
        if not self._offboard_alive:
            if not self._offboard_active:
                self._offboard_active = True
                self._error_count += 1
            if self._error_count > self._max_errors:
                self._last_level = EmergencyLevel.ABORT
                return EmergencyLevel.ABORT
            self._last_level = EmergencyLevel.WARN
            return EmergencyLevel.WARN
        else:
            self._offboard_active = False

        # 4. 视觉长时间丢失
        vision_lost_duration = now - self._vision_last_seen
        if vision_lost_duration > self.vision_loss_timeout:
            if not self._vision_active:
                self._vision_active = True
                self._warning_count += 1
            self._last_level = EmergencyLevel.WARN
            return EmergencyLevel.WARN
        else:
            self._vision_active = False

        # 5. 状态超时
        effective_timeout = self._state_timeout if self._state_timeout is not None else self.state_max_duration
        state_duration = now - self._state_start_time
        if effective_timeout > 0 and state_duration > effective_timeout:
            if not self._timeout_active:
                self._timeout_active = True
                self._error_count += 1
            if self._error_count > self._max_errors:
                self._last_level = EmergencyLevel.ABORT
                return EmergencyLevel.ABORT
            self._last_level = EmergencyLevel.WARN
            return EmergencyLevel.WARN
        else:
            self._timeout_active = False

        # 6. GPS 丢失
        if not self._gps_fix:
            if not self._gps_active:
                self._gps_active = True
                self._warning_count += 1
            self._last_level = EmergencyLevel.WARN
            return EmergencyLevel.WARN
        else:
            self._gps_active = False

        # 正常 → 重置错误计数 (所有条件都已恢复正常)
        self._error_count = 0
        self._last_level = EmergencyLevel.OK
        return EmergencyLevel.OK

    # ================================================================
    # 便捷方法
    # ================================================================

    def is_critical(self) -> bool:
        """是否需要立即采取行动 (使用缓存值, 避免重复计数)."""
        return self._last_level in (EmergencyLevel.RTL, EmergencyLevel.LAND, EmergencyLevel.ABORT)

    def should_rtl(self) -> bool:
        """是否应该返航 (使用缓存值)."""
        return self._last_level == EmergencyLevel.RTL

    def should_abort(self) -> bool:
        """是否应该中止 (使用缓存值)."""
        return self._last_level == EmergencyLevel.ABORT

    @property
    def battery_remaining(self) -> float:
        return self._battery_remaining

    @property
    def vision_lost_duration(self) -> float:
        return time.time() - self._vision_last_seen

    @property
    def error_count(self) -> int:
        return self._error_count

    @property
    def state_duration(self) -> float:
        return time.time() - self._state_start_time
