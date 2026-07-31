"""
RETURN 状态 — 返航至 H 点上方 5m.

流程:
  1. 从当前位置飞向起飞点 H (World: 4, 10, 5)
  2. 以 return_speed (3 m/s) 飞行
  3. 到达 H 点上方 → PRECISION_LAND
"""

import time
from typing import Optional


class ReturnState:
    """返航状态处理器."""

    HOME_Z = 5.0  # 巡航高度（仿真/真机一致）

    def __init__(self, fsm):
        self._fsm = fsm
        self._start_time: float = 0.0
        self._at_home_stable: int = 0

    def enter(self):
        self._fsm.logger.log_state("RECON_PATROL", "RETURN_TO_HOME",
                                   "开始返航")
        self._start_time = time.time()
        self._at_home_stable = 0

        home_x = self._fsm.get_param("home_x", 4.0)
        home_y = self._fsm.get_param("home_y", 10.0)

        self._fsm.get_logger().info(
            f"RETURN: 返航至 H 点 ({home_x}, {home_y}, {self.HOME_Z})"
        )

        # 设置返航航点
        from px4_control.telemetry import world_to_ned
        tn, te, _ = world_to_ned(home_x, home_y, 0)
        td = self._fsm.fc.altitude_to_ned(self.HOME_Z)
        self._fsm.fc.goto_ned(tn, te, td, yaw=0.0,
                              speed=self._fsm.get_param("return_speed", 3.0))

    def execute(self) -> Optional[str]:
        # 获取当前位置
        from px4_control.telemetry import ned_to_world
        pos = self._fsm.bridge.get_current_position()
        drone_wx, drone_wy, _ = ned_to_world(pos.north, pos.east, pos.down)

        import math
        home_x = self._fsm.get_param("home_x", 4.0)
        home_y = self._fsm.get_param("home_y", 10.0)
        dist_xy = math.hypot(drone_wx - home_x, drone_wy - home_y)
        # 用相对高度比较, 不受 GPS/EKF 原点漂移影响
        current_alt = self._fsm.fc.ned_to_altitude(pos.down)
        dist_z = abs(current_alt - self.HOME_Z)

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
            self._fsm.get_logger().error("RETURN: 未到达 H 点, 请求 PX4 RTL")
            self._fsm.bridge.return_to_launch()
            return "ERROR"

        return None

    def exit(self):
        self._fsm.logger.log_state("RETURN_TO_HOME", "PRECISION_LAND",
                                   "返航完成")
