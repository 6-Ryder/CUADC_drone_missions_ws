"""
RETURN 状态 — 返航至 H 点上方 5m.

流程:
  1. 从当前位置飞向起飞点 H (World: 10, 4, 5)
  2. 以 return_speed (3 m/s) 飞行
  3. 到达 H 点上方 → PRECISION_LAND
"""

import time
from typing import Optional


class ReturnState:
    """返航状态处理器."""

    # 起飞点 H (World 坐标)
    HOME_X = 10.0
    HOME_Y = 4.0
    HOME_Z = 5.0  # 巡航高度

    def __init__(self, fsm):
        self._fsm = fsm
        self._start_time: float = 0.0
        self._at_home_stable: int = 0

    def enter(self):
        self._fsm.logger.log_state("RECON_PATROL", "RETURN_TO_HOME",
                                   "开始返航")
        self._start_time = time.time()
        self._at_home_stable = 0

        self._fsm.get_logger().info(
            f"RETURN: 返航至 H 点 ({self.HOME_X}, {self.HOME_Y}, {self.HOME_Z})"
        )

        # 设置返航航点
        self._fsm.fc.goto_world(
            self.HOME_X, self.HOME_Y, self.HOME_Z,
            speed=self._fsm.get_param("return_speed", 3.0),
        )

    def execute(self) -> Optional[str]:
        # 获取当前位置
        from px4_control.telemetry import ned_to_world
        pos = self._fsm.bridge.get_current_position()
        drone_wx, drone_wy, drone_wz = ned_to_world(pos.north, pos.east, pos.down)

        import math
        dist_xy = math.hypot(drone_wx - self.HOME_X, drone_wy - self.HOME_Y)
        dist_z = abs(drone_wz - self.HOME_Z)

        tolerance = self._fsm.get_param("waypoint_tolerance", 0.5)

        if dist_xy < tolerance and dist_z < tolerance:
            self._at_home_stable += 1
        else:
            self._at_home_stable = 0

        elapsed = time.time() - self._start_time

        if elapsed > 1.0 and elapsed % 5.0 < 0.2:
            self._fsm.get_logger().info(
                f"RETURN: dist_xy={dist_xy:.2f}m dist_z={dist_z:.2f}m "
                f"stable={self._at_home_stable}"
            )

        # 稳定在 H 点 2s
        if self._at_home_stable >= 40:
            self._fsm.get_logger().info("RETURN: 到达 H 点, 开始精准降落")
            return "PRECISION_LAND"

        # 超时
        if elapsed > 60.0:
            self._fsm.get_logger().warn("RETURN: 超时, 强制进入精准降落")
            return "PRECISION_LAND"

        return None

    def exit(self):
        self._fsm.logger.log_state("RETURN_TO_HOME", "PRECISION_LAND",
                                   "返航完成")
