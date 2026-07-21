"""
飞行控制高级接口 — 面向任务的 Python API.

封装 px4_bridge 的 setpoint 发布为直观的飞行指令:
  takeoff(altitude)         垂直起飞
  goto_ned(n,e,d,yaw)       飞到 NED 航点
  goto_world(x,y,z,yaw)     飞到 World 航点
  loiter(duration)           悬停
  land()                     原地降落
  set_velocity_ned(vn,ve,vd) 速度控制
  set_velocity_body(fwd,right,down) 机体系速度控制
  force_disarm()            紧急上锁
  reached(target, tol)      航点到达判断
"""

import math
import time
from typing import Optional, Tuple

from px4_control.telemetry import Position
from px4_control.px4_bridge import PX4Bridge


class FlightController:
    """
    PX4 Offboard 飞行控制高级接口.

    所有位置使用 NED 坐标系 (north, east, down).
    所有速度使用 NED 坐标系 (vn, ve, vd).
    """

    def __init__(self, bridge: PX4Bridge):
        """
        初始化飞行控制接口.

        Parameters
        ----------
        bridge : PX4Bridge
            PX4 通信桥接实例

        """
        self._bridge = bridge
        self._node = bridge  # PX4Bridge 本身是 Node

        # 参数缓存
        self._takeoff_height = bridge.get_parameter("takeoff_height").value
        self._waypoint_tolerance = bridge.get_parameter("waypoint_tolerance").value
        self._takeoff_tolerance = bridge.get_parameter("takeoff_tolerance").value

        # 状态
        self._current_waypoint: Optional[Position] = None
        self._waypoint_start_time: float = 0.0
        self._waypoint_timeout: float = bridge.get_parameter("waypoint_timeout").value
        self._target_speed: float = 1.0

        self._landing = False
        self._land_start_time: float = 0.0
        self._land_mode_requested = False

    # ================================================================
    # 起飞
    # ================================================================

    def takeoff(self, altitude: Optional[float] = None) -> bool:
        """
        垂直起飞至指定高度.

        策略:
          1. 切换到 Offboard 模式
          2. 解锁
          3. 在当前位置上方设置目标高度, 水平位置保持不变
          4. 等待到达目标高度

        Parameters
        ----------
        altitude : float or None
            目标高度 (NED down, 负值=上升). None 则使用默认值.

        Returns
        -------
        bool: 是否已到达目标高度

        """
        if altitude is None:
            altitude = self._takeoff_height

        # 确保 Offboard 模式
        if not self._bridge.is_offboard():
            self._bridge.set_offboard_mode()

        # 解锁
        if not self._bridge.is_armed():
            self._bridge.arm()

        # 从当前位置起飞
        pos = self._bridge.get_current_position()
        target_down = -altitude  # NED: 负值=上升
        self._bridge.set_position_target(pos.north, pos.east, target_down)
        self._current_waypoint = Position(pos.north, pos.east, target_down)
        self._waypoint_start_time = time.time()

        return self.reached((pos.north, pos.east, target_down), self._takeoff_tolerance)

    # ================================================================
    # 航点飞行
    # ================================================================

    def goto_ned(self, north: float, east: float, down: float,
                 yaw: float = 0.0, speed: float = 1.0):
        """
        设置 NED 航点.

        Parameters
        ----------
        north, east, down : float
            NED 坐标 (m)
        yaw : float
            目标偏航角 (rad)
        speed : float
            飞行速度 (m/s)

        """
        self._target_speed = speed
        self._bridge.set_position_target(north, east, down, yaw)
        self._current_waypoint = Position(north, east, down)
        self._waypoint_start_time = time.time()

    def goto_world(self, x: float, y: float, z: float,
                   yaw: float = 0.0, speed: float = 1.0):
        """
        设置 World 坐标航点, 自动转换为 NED.

        Parameters
        ----------
        x, y, z : float
            World 坐标 (Gazebo: x=east, y=north, z=up)
        yaw : float
            目标偏航角 (rad)
        speed : float
            飞行速度 (m/s)

        """
        from px4_control.telemetry import world_to_ned
        north, east, down = world_to_ned(x, y, z)
        self.goto_ned(north, east, down, yaw, speed)

    def loiter(self, duration: float = 0.0):
        """
        悬停在当前位置.

        Parameters
        ----------
        duration : float
            悬停时间 (s). 0 表示无限 (由外部停止).

        """
        pos = self._bridge.get_current_position()
        self._bridge.set_position_target(pos.north, pos.east, pos.down)
        self._current_waypoint = pos
        self._waypoint_start_time = time.time()
        self._waypoint_timeout = duration if duration > 0 else 3600.0

    def set_velocity_ned(self, vn: float, ve: float, vd: float, yaw_rate: float = 0.0):
        """
        设置速度控制指令 (NED).

        Parameters
        ----------
        vn, ve, vd : float
            NED 速度 (m/s). vd 正值=下降.
        yaw_rate : float
            偏航速率 (rad/s).

        """
        self._bridge.set_velocity_target(vn, ve, vd, yaw_rate)

    def set_velocity_body(
        self,
        forward: float,
        right: float,
        down: float = 0.0,
        yaw_rate: float = 0.0,
    ):
        """
        设置机体系速度指令, 内部转换为 NED 速度发布.

        forward/right/down 使用 FRD 机体系: forward=机头方向, right=机体右侧,
        down=向下。当前 px4_msgs/TrajectorySetpoint 仅支持 NED velocity, 因此
        这里用当前 yaw 做 body→NED 旋转, 不直接假设 PX4 ROS2 支持 body frame。
        """
        yaw = self._current_yaw()
        vn = float(forward) * math.cos(yaw) - float(right) * math.sin(yaw)
        ve = float(forward) * math.sin(yaw) + float(right) * math.cos(yaw)
        self.set_velocity_ned(vn, ve, float(down), yaw_rate)

    def stop_velocity(self):
        """停止速度控制, 保持当前位置悬停."""
        self.set_velocity_ned(0.0, 0.0, 0.0)

    def _current_yaw(self) -> float:
        """Return current PX4 yaw in NED if available."""
        if hasattr(self._bridge, "_current_yaw"):
            return float(getattr(self._bridge, "_current_yaw"))
        attitude = self._bridge.get_current_attitude()
        return float(getattr(attitude, "yaw", 0.0))

    # ================================================================
    # 着陆
    # ================================================================

    def land(self) -> bool:
        """
        原地垂直降落.

        使用速度控制缓慢下降, 直到 land_detected 为 True.

        Returns
        -------
        bool: 是否已着陆

        """
        if self._bridge.is_landed():
            self._landing = False
            return True

        if not self._landing:
            self._landing = True
            self._land_start_time = time.time()
            self._land_mode_requested = False
            # 水平悬停, 缓慢下降
            self._bridge.set_velocity_target(0.0, 0.0, 0.3)  # 0.3 m/s 下降
            return False

        # 下降中
        if self._bridge.is_landed():
            self._landing = False
            # 着陆后上锁
            self._bridge.disarm()
            return True

        # 超时保护
        if time.time() - self._land_start_time > 30.0:
            if not self._land_mode_requested:
                self._bridge.land_at_current()
                self._land_mode_requested = True

        return False

    def force_disarm(self):
        """紧急上锁."""
        self._bridge.force_disarm()
        self._landing = False

    # ================================================================
    # 辅助
    # ================================================================

    def reached(self, target: Tuple[float, float, float], tolerance: float = 0.0) -> bool:
        """
        判断是否已到达目标航点.

        Parameters
        ----------
        target : (north, east, down)
            目标位置 (NED)
        tolerance : float
            容差 (m). 0 表示使用默认 waypoint_tolerance.

        Returns
        -------
        bool

        """
        if tolerance <= 0:
            tolerance = self._waypoint_tolerance

        pos = self._bridge.get_current_position()
        dist = math.sqrt(
            (pos.north - target[0]) ** 2 +
            (pos.east - target[1]) ** 2 +
            (pos.down - target[2]) ** 2
        )
        return dist < tolerance

    def reached_xy(self, target: Tuple[float, float], tolerance: float = 0.0) -> bool:
        """
        判断是否已到达目标水平位置.

        Parameters
        ----------
        target : (north, east)
            目标水平位置 (NED).
        tolerance : float
            水平距离容差 (m).

        """
        if tolerance <= 0:
            tolerance = self._waypoint_tolerance

        pos = self._bridge.get_current_position()
        dist = math.hypot(pos.north - target[0], pos.east - target[1])
        return dist < tolerance

    def reached_altitude(self, target_down: float, tolerance: float = 0.0) -> bool:
        """
        判断是否已到达目标高度.

        Parameters
        ----------
        target_down : float
            NED z (负=上)
        tolerance : float
            高度容差 (m).

        """
        if tolerance <= 0:
            tolerance = self._takeoff_tolerance

        pos = self._bridge.get_current_position()
        return abs(pos.down - target_down) < tolerance

    def distance_to(self, target: Tuple[float, float, float]) -> float:
        """计算到目标的三维距离 (m)."""
        pos = self._bridge.get_current_position()
        return math.sqrt(
            (pos.north - target[0]) ** 2 +
            (pos.east - target[1]) ** 2 +
            (pos.down - target[2]) ** 2
        )

    def is_waypoint_timed_out(self) -> bool:
        """当前航点是否超时."""
        return (time.time() - self._waypoint_start_time) > self._waypoint_timeout

    def get_position(self) -> Position:
        return self._bridge.get_current_position()

    @property
    def current_waypoint(self) -> Optional[Position]:
        return self._current_waypoint

    @property
    def waypoint_elapsed(self) -> float:
        return time.time() - self._waypoint_start_time
