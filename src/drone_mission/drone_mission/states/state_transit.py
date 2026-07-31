"""
TRANSIT 状态 — 区域间飞行.

支持两种具体的飞行场景:
  TRANSIT_TO_DROP_ZONE:  从起飞点飞向投放区
  TRANSIT_TO_RECON_ZONE: 从投放区飞向侦察区

统一逻辑:
  1. 设置目标航点 (World → NED 转换)
  2. 以 transit_speed (2 m/s) 飞行
  3. 到达目标区域 → 返回下一状态名
"""

import time
from typing import Optional, Tuple



class TransitState:
    """
    过渡飞行状态.

    可以配置目标坐标和目标状态名.
    """

    def __init__(self, fsm):
        self._fsm = fsm
        self._start_time: float = 0.0
        self._target_world: Optional[Tuple[float, float, float]] = None
        self._target_state: str = ""
        self._label: str = "TRANSIT"

    def configure(self, target_world_xyz: Tuple[float, float, float],
                  target_state: str, label: str = "TRANSIT"):
        """
        配置过渡飞行的目标.

        Parameters
        ----------
        target_world_xyz : (x, y, z)
            World 坐标系目标 (Gazebo)
        target_state : str
            到达后跳转的状态名
        label : str
            日志标签

        """
        self._target_world = target_world_xyz
        self._target_state = target_state
        self._label = label

    def enter(self):
        if self._target_world is None:
            self._fsm.get_logger().error(f"{self._label}: 未配置目标!")
            return

        from_state = self._fsm.current_state
        self._start_time = time.time()

        tx, ty, tz = self._target_world
        self._fsm.get_logger().info(
            f"{self._label}: 飞向 World({tx:.1f}, {ty:.1f}, {tz:.1f}) "
            f"→ {self._target_state}"
        )
        self._fsm.logger.log_state(from_state, self._label, f"飞向 ({tx:.1f},{ty:.1f})")

        # 设置航点
        speed = self._fsm.get_param("transit_speed", 2.0)
        from px4_control.telemetry import world_to_ned
        tn, te, _ = world_to_ned(tx, ty, 0)
        td = self._fsm.fc.altitude_to_ned(tz)
        self._fsm.fc.goto_ned(tn, te, td, yaw=0.0, speed=speed)

    def execute(self) -> Optional[str]:
        if self._target_world is None:
            return self._target_state

        tx, ty, tz = self._target_world
        pos = self._fsm.fc.get_position()

        # 导入坐标变换
        from px4_control.telemetry import world_to_ned
        tn, te, _ = world_to_ned(tx, ty, 0)
        td = self._fsm.fc.altitude_to_ned(tz)

        dist = pos.distance_to(type(pos)(tn, te, td))
        tolerance = self._fsm.get_param("waypoint_tolerance", 0.5)

        elapsed = time.time() - self._start_time

        if elapsed > 1.0 and elapsed % 5.0 < 0.2:
            self._fsm.get_logger().info(
                f"{self._label}: dist={dist:.2f}m tol={tolerance}m"
            )

        if dist < tolerance:
            self._fsm.get_logger().info(
                f"{self._label}: 到达目标区域 → {self._target_state}"
            )
            return self._target_state

        # 超时
        timeout = self._fsm.get_param("waypoint_timeout", 120.0)
        if elapsed > timeout:
            self._fsm.get_logger().error(f"{self._label}: 未到达目标航点")
            return "ERROR"

        return None

    def exit(self):
        self._fsm.logger.log_state(self._label, self._target_state, "过渡完成")
