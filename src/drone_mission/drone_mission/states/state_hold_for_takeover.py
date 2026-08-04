"""Latched zero-velocity hold for a human operator takeover."""

import time
from typing import Optional


class HoldForTakeoverState:
    """Hold in Offboard until the operator takes control or PX4 leaves it."""

    def __init__(self, fsm):
        self._fsm = fsm
        self._last_log_time = 0.0

    def enter(self) -> None:
        self._last_log_time = time.monotonic()
        if self._fsm.bridge.is_armed() and self._fsm.bridge.is_offboard():
            self._fsm.bridge.hold_for_takeover()
        else:
            self._fsm.bridge.stop_autonomous_output()

    def execute(self) -> Optional[str]:
        if not self._fsm.bridge.is_armed() or not self._fsm.bridge.is_offboard():
            self._fsm.bridge.stop_autonomous_output()
            return "COMPLETE"

        self._fsm.bridge.hold_for_takeover()
        now = time.monotonic()
        if now - self._last_log_time >= 5.0:
            self._fsm.get_logger().error(
                "HOLD_FOR_TAKEOVER: 零速度悬停，等待 RC 接管"
            )
            self._last_log_time = now
        return None

    def exit(self) -> None:
        pass
