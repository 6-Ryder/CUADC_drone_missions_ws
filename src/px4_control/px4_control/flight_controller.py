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

    def __init__(self, bridge):
        """
        初始化飞行控制接口.

        Parameters
        ----------
        bridge : PX4Bridge or StandaloneBridge
            PX4 通信桥接实例. 两种实现共享相同接口:
            is_armed / arm / set_offboard_mode / get_current_position /
            set_position_target / set_velocity_target / is_landed / force_disarm /
            land_at_current / get_current_attitude / get_parameter / get_telemetry

        """
        self._bridge = bridge

        # 参数缓存 (bridge 通过 get_parameter 委托到所属 ROS2 Node)
        self._takeoff_height = bridge.get_parameter("takeoff_height").value
        self._waypoint_tolerance = bridge.get_parameter("waypoint_tolerance").value
        self._takeoff_tolerance = bridge.get_parameter("takeoff_tolerance").value

        # 状态
        self._current_waypoint: Optional[Position] = None
        self._waypoint_start_time: float = 0.0
        self._default_waypoint_timeout: float = bridge.get_parameter("waypoint_timeout").value
        self._waypoint_timeout: float = self._default_waypoint_timeout
        self._target_speed: float = 1.0

        self._landing = False
        self._land_start_time: float = 0.0
        self._land_mode_requested = False

        self._origin_down: Optional[float] = None

    # ================================================================
    # 获取初始相对限制
    # ================================================================

    @property
    def origin_down(self) -> float:
        """起飞原点的 NED down 值 (首次访问时自动记录)"""
        if self._origin_down is None:
            self._origin_down = self._bridge.get_current_position().down
        return self._origin_down

    def reset_origin(self) -> None:
        """着陆后重置原点，下次起飞重新标定"""
        self._origin_down = None

    def altitude_to_ned(self, altitude_m: float) -> float:
        """物理高度(m, 距离起飞点) -> NED down"""
        return self.origin_down - altitude_m

    def ned_to_altitude(self, ned_down: float) -> float:
        """NED down -> 物理高度（m, 离起飞点)"""
        return self.origin_down - ned_down

    # ================================================================
    # 起飞
    # ================================================================

    def takeoff(self, altitude: Optional[float] = None) -> bool:
        """
        垂直起飞至指定高度.

        策略:
          1. 切换到 Offboard 模式
          2. 解锁 (仅当 is_armed()=False 时)
          3. 在当前位置上方设置目标高度, 水平位置保持不变
          4. 等待到达目标高度

        .. warning::
           在 RC 模式 (rc_require_arm_signal=True) 下, 调用方必须在
           调用本方法前确保 PX4 已被遥控器解锁. 本方法的内部 arm()
           调用仅在 is_armed()=False 时触发, 正常的 RC 模式流程中
           这个 guard 为 True, arm() 不会被调用.

        Parameters
        ----------
        altitude : float or None
            目标高度 (NED down, 相对于起飞点). None 则使用默认值.

        Returns
        -------
        bool: 是否已到达目标高度

        """
        if altitude is None:
            altitude = self._takeoff_height

        # 确保 Offboard 模式
        if not self._bridge.is_offboard():
            self._bridge.set_offboard_mode()

        # 从当前位置起飞
        pos = self._bridge.get_current_position()
        target_down = self.altitude_to_ned(altitude)  # NED: 负值=上升
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
        self._waypoint_timeout = self._default_waypoint_timeout

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
        # 使用 loiter 专用超时, 不影响后续航点的默认超时
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
        # Fallback: compute yaw from attitude quaternion
        attitude = self._bridge.get_current_attitude()
        try:
            w, x, y, z = attitude.q_w, attitude.q_x, attitude.q_y, attitude.q_z
            return float(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
        except Exception:
            return 0.0

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
            # PX4 LNDMC_Z_VEL_MAX=0.25, 下降速度必须 < 此阈值
            # 才能提前进入 maybe_landed 状态, 避免单靠触地瞬间 IMU 信号
            self._bridge.set_velocity_target(0.0, 0.0, 0.2)  # 0.2 m/s (< 0.25)
            return False

        # 下降中
        if self._bridge.is_landed():
            self._landing = False
            self.reset_origin()
            # 着陆确认后用 force_disarm 避免被"非手动 disarm"拒绝
            self._bridge.force_disarm()
            return True

        # 超时保护: 15s 后切 LAND mode (原来 30s 太长)
        if time.time() - self._land_start_time > 15.0:
            if not self._land_mode_requested:
                self._bridge.land_at_current()
                self._land_mode_requested = True

        return False

    def force_disarm(self):
        """紧急上锁."""
        self.reset_origin()
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
