"""
RETURN 状态 — 返航至 H 点上方 5m.

流程:
  1. 从当前位置飞向起飞点 H (World: 4, 10, 5)
  2. 以 return_speed (3 m/s) 飞行
  3. 到达 H 点上方 → PRECISION_LAND
"""

import time
from typing import Optional

from drone_utils.progress_monitor import ProgressMonitor

class ReturnState:
    """返航状态处理器."""

    def __init__(self, fsm):
        self._fsm = fsm
        self._start_time: float = 0.0
        self._at_home_stable: int = 0
        self._target_altitude: float = 5.0
        self._target_ned = None
        self._progress: Optional[ProgressMonitor] = None

    def enter(self):
        self._fsm.logger.log_state("RECON_PATROL", "RETURN_TO_HOME",
                                   "开始返航")
        self._start_time = time.time()
        self._at_home_stable = 0

        home_x = self._fsm.get_param("home_x", 4.0)
        home_y = self._fsm.get_param("home_y", 10.0)
        self._target_altitude = float(
            self._fsm.get_param("return_altitude", 5.0)
        )

        self._fsm.get_logger().info(
            "RETURN: 从最后侦察航点直接返航至 H 点 "
            f"({home_x}, {home_y}, {self._target_altitude})"
        )

        # One 3-D target begins the return at the last scan waypoint while
        # climbing toward the verified Home altitude; no separate climb state.
        from px4_control.telemetry import world_to_ned
        tn, te, _ = world_to_ned(home_x, home_y, 0)
        td = self._fsm.fc.altitude_to_ned(self._target_altitude)
        self._target_ned = (tn, te, td)
        self._progress = ProgressMonitor(
            window_s=float(self._fsm.get_param("navigation_progress_window", 30.0)),
            min_improvement=float(
                self._fsm.get_param("navigation_progress_min_improvement", 0.25)
            ),
        )
        self._command_home()

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
        dist_z = abs(current_alt - self._target_altitude)

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

        if self._progress is not None:
            distance = math.hypot(dist_xy, dist_z)
            now = time.monotonic()
            if self._progress.observe(distance, now=now):
                if self._progress.mark_retry(now=now):
                    self._fsm.get_logger().warn("RETURN: 无进展, 重发 Home+高度目标一次")
                    self._command_home()
                else:
                    self._fsm.request_takeover(
                        "RETURN: Home target made no progress after retry"
                    )

        return None

    def _command_home(self) -> None:
        if self._target_ned is None:
            return
        north, east, down = self._target_ned
        self._fsm.fc.goto_ned(
            north,
            east,
            down,
            yaw=0.0,
            speed=self._fsm.get_param("return_speed", 3.0),
        )

    def exit(self):
        self._fsm.logger.log_state("RETURN_TO_HOME", "PRECISION_LAND",
                                   "返航完成")
