"""
PX4 DDS 通信桥接 — PX4 ↔ ROS2 核心层.

核心职责:
  1. 通过 micro-XRCE-DDS 与 PX4 飞控通信
  2. 订阅 /fmu/out/* 话题 (VehicleOdometry, VehicleStatus, VehicleAttitude, VehicleLandDetected)
  3. 发布 /fmu/in/* 话题 (OffboardControlMode, TrajectorySetpoint, VehicleCommand)
  4. 转换 NED → World 坐标, 发布 /drone/pose (供 vision_pipeline 使用)
  5. 10Hz Offboard 心跳 + 20Hz 控制循环
  6. 斜坡 Setpoint 平滑 (ramped setpoint)

参考: Task_flight_ws/src/px4_control_dds/px4_control_dds/mission_control_node_v2.py
"""

import math
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool
from drone_interfaces.msg import NedPositionSetpoint

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
    VehicleStatus,
    VehicleAttitude,
    VehicleLandDetected,
    BatteryStatus,
    SensorGps,
)

from px4_control.telemetry import (
    NAV_STATE_OFFBOARD,
    VEHICLE_CMD_COMPONENT_ARM_DISARM,
    VEHICLE_CMD_DO_SET_MODE,
    VEHICLE_CMD_NAV_LAND,
    VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,
    Position, Attitude, Velocity, Telemetry, ned_to_world_pose,
    configure_home,
)
from drone_utils.logger import TaskLogger
from drone_utils.watchdog import Watchdog


# ============================================================
# QoS 配置
# ============================================================

def px4_qos() -> QoSProfile:
    """PX4 micro-XRCE-DDS 通信所需的 QoS profile."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )


def autonomy_lease_qos() -> QoSProfile:
    """QoS for the fail-closed mission-to-DDS autonomy lease."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


# ============================================================
# PX4Bridge 节点
# ============================================================

class PX4Bridge(Node):
    """
    PX4 DDS ↔ ROS2 通信桥接节点.

    订阅 (from PX4):
      /fmu/out/vehicle_odometry       → 位置/速度 (50Hz, NED)
      /fmu/out/vehicle_status         → 解锁/导航状态 (5Hz)
      /fmu/out/vehicle_attitude       → 姿态四元数 (50Hz)
      /fmu/out/vehicle_land_detected  → 着陆检测 (5Hz)

    发布 (to PX4):
      /fmu/in/offboard_control_mode   → Offboard 模式声明 (10Hz)
      /fmu/in/trajectory_setpoint     → 位置/速度指令 (10-20Hz)
      /fmu/in/vehicle_command         → 模式切换/解锁/Land 指令

    发布 (to ROS2):
      /drone/pose                     → World 位姿 (50Hz, PoseStamped)
      /drone/telemetry                → 综合遥测 (10Hz)
    """

    def __init__(self):
        super().__init__("px4_bridge")

        # ---- 参数 ----
        self._declare_params()

        # ---- 坐标变换: 配置 Home 点偏移 ----
        configure_home(
            float(self.get_parameter("home_x").value),
            float(self.get_parameter("home_y").value),
            float(self.get_parameter("home_z").value),
        )

        # ---- QoS ----
        qos = px4_qos()

        # ---- 订阅 PX4 → ROS2 ----
        self._odom_sub = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._odom_cb, qos,
        )
        status_topic = str(self.get_parameter("vehicle_status_topic").value)
        self._status_sub = self.create_subscription(
            VehicleStatus, status_topic, self._status_cb, qos,
        )
        self._att_sub = self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude", self._attitude_cb, qos,
        )
        self._land_sub = self.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected", self._land_cb, qos,
        )
        self._battery_sub = self.create_subscription(
            BatteryStatus, "/fmu/out/battery_status_v1", self._battery_cb, qos,
        )
        self._gps_sub = self.create_subscription(
            SensorGps, "/fmu/out/vehicle_gps_position", self._gps_cb, qos,
        )

        # ---- 发布 ROS2 → PX4 ----
        self._offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos,
        )
        self._trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos,
        )
        self._cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos,
        )

        # ---- 发布 PX4 → ROS2 (处理后) ----
        # 使用 VOLATILE QoS 避免 DURABILITY 不兼容
        ros_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pose_pub = self.create_publisher(
            PoseStamped, "/drone/pose", ros_qos,
        )
        self._vel_pub = self.create_publisher(
            TwistStamped, "/drone/servo/velocity_cmd", ros_qos,
        )

        # ---- 内部状态 ----
        self._odom: Optional[VehicleOdometry] = None
        self._status: Optional[VehicleStatus] = None
        self._attitude: Optional[VehicleAttitude] = None
        self._land_detected: Optional[VehicleLandDetected] = None

        self._current_telemetry = Telemetry()

        # Setpoint 平滑
        self._target_setpoint = Position(0.0, 0.0, -5.0)  # NED
        self._current_setpoint = Position(0.0, 0.0, -5.0)
        self._target_yaw: float = 0.0
        self._target_speed: Optional[float] = None
        self._current_yaw: float = 0.0
        self._last_attitude_time: Optional[float] = None
        self._smoothed_yaw: float = 0.0  # 平滑后的 yaw setpoint, 避免与物理 yaw 冲突
        self._setpoint_lock = threading.Lock()

        # 组件ID (注册用)
        self._component_id: int = 1

        # 控制标志
        self._armed: bool = False
        self._offboard_active: bool = False
        self._velocity_mode: bool = False  # True=速度控制, False=位置控制
        self._last_velocity_cmd = (0.0, 0.0, 0.0)  # 缓存速度指令, 避免心跳覆写
        # StandaloneBridge owns the mission-side lease.  Fail closed until a
        # live lease arrives, so a stopped/restarted FSM cannot leave this DDS
        # node replaying an old relay setpoint.
        self._autonomy_lease_enabled: bool = False
        self._autonomy_lease_last_seen: Optional[float] = None
        self._autonomy_lease_timeout_s = float(
            self.get_parameter("autonomy_lease_timeout_s").value
        )
        self._autonomy_target_received: bool = False

        # ---- 订阅 Mission FSM 的 setpoint 指令 (NED) ----
        self._cmd_sp_sub = self.create_subscription(
            NedPositionSetpoint, "/drone/cmd_setpoint_ned", self._cmd_setpoint_cb, 10,
        )
        # 订阅 Mission FSM 的速度指令 (NED velocity)
        from geometry_msgs.msg import Twist
        self._cmd_vel_sub = self.create_subscription(
            Twist, "/drone/cmd_velocity_ned", self._cmd_velocity_cb, 10,
        )
        self._vision_vel_sub = self.create_subscription(
            TwistStamped, "/drone/vision/servo_cmd", self._vision_velocity_cb, 10,
        )
        # 注意: 此路径与 _cmd_velocity_cb 共享 _last_velocity_cmd 缓存, 心跳只发
        # 最后写入的值 —— 双写入者会互相覆盖。B4 (2026-08-07) 起发布端已禁用
        # (vision_pipeline SetVisionMode(2) 不再激活内嵌伺服), 保留订阅仅为兼容。
        # 新发布者接入前必须确认与 FSM 速度指令的独占关系。
        # 订阅 VehicleCommand 中继 — FSM 等节点通过此话题发送命令,
        # px4_bridge 转发到 /fmu/in/vehicle_command (已验证的 DDS 路径)
        self._cmd_relay_sub = self.create_subscription(
            VehicleCommand, "/drone/cmd/vehicle_command_relay",
            self._cmd_relay_cb, 10,
        )
        self._autonomy_lease_sub = self.create_subscription(
            Bool, "/drone/cmd/autonomy_lease", self._autonomy_lease_cb,
            autonomy_lease_qos(),
        )

        # ---- 定时器 ----
        self._offboard_timer = self.create_timer(0.1, self._offboard_heartbeat)   # 10Hz
        self._control_timer = self.create_timer(0.05, self._control_loop)         # 20Hz
        # ---- 工具 ----
        self._task_logger = TaskLogger(node_name="px4_bridge")
        self._watchdog = Watchdog()
        self._watchdog.register("px4_odom", timeout=3.0,
                                on_timeout=lambda n: self.get_logger().warn(f"{n} 超时"))
        self._watchdog.register("px4_status", timeout=3.0,
                                on_timeout=lambda n: self.get_logger().warn(f"{n} 超时"))

        self.get_logger().info("PX4 Bridge 已启动 (等待 DDS 连接...)")

    def _declare_params(self):
        self.declare_parameter("quick_drop_profile", "blue")
        self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status_v1")
        self.declare_parameter("takeoff_height", 5.0)
        self.declare_parameter("waypoint_tolerance", 0.5)
        self.declare_parameter("takeoff_tolerance", 0.3)
        self.declare_parameter("control_frequency", 20.0)
        self.declare_parameter("waypoint_timeout", 120.0)
        self.declare_parameter("max_setpoint_speed_limit", 5.0)
        self.declare_parameter("autonomy_lease_timeout_s", 0.30)
        self.declare_parameter("home_x", 4.0)
        self.declare_parameter("home_y", 10.0)
        self.declare_parameter("home_z", 0.0)

    # ================================================================
    # 订阅回调
    # ================================================================

    def _odom_cb(self, msg: VehicleOdometry):
        self._odom = msg
        self._watchdog.feed("px4_odom")

        # 更新遥测
        self._current_telemetry.timestamp = time.time()
        self._current_telemetry.position = Position(
            north=float(msg.position[0]),
            east=float(msg.position[1]),
            down=float(msg.position[2]),
        )
        self._current_telemetry.velocity = Velocity(
            vn=float(msg.velocity[0]),
            ve=float(msg.velocity[1]),
            vd=float(msg.velocity[2]),
        )

        # 发布 /drone/pose (World 坐标系, 供 vision_pipeline 使用)
        self._publish_drone_pose()

    def _status_cb(self, msg: VehicleStatus):
        self._status = msg
        self._watchdog.feed("px4_status")

        self._current_telemetry.status.armed = (msg.arming_state == 2)
        self._current_telemetry.status.offboard_mode = (msg.nav_state == NAV_STATE_OFFBOARD)
        self._current_telemetry.status.navigation_state = msg.nav_state

        self._armed = (msg.arming_state == 2)

    def _attitude_cb(self, msg: VehicleAttitude):
        self._attitude = msg
        q = msg.q  # [w, x, y, z] Hamilton convention
        if len(q) >= 4:
            if (not all(math.isfinite(float(v)) for v in q[:4])
                    or sum(float(v) * float(v) for v in q[:4]) <= 1e-12):
                self._last_attitude_time = None
                return
            self._current_telemetry.attitude = Attitude(
                q_w=float(q[0]), q_x=float(q[1]), q_y=float(q[2]), q_z=float(q[3]),
            )
            # 从四元数计算 yaw (VehicleAttitude 消息没有单独的 yaw 字段)
            w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
            self._current_yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            self._last_attitude_time = time.monotonic()

    def _land_cb(self, msg: VehicleLandDetected):
        self._land_detected = msg
        self._current_telemetry.status.landed = msg.landed

    def _battery_cb(self, msg: BatteryStatus):
        """Update telemetry from a connected battery with a valid state of charge."""
        if msg.connected and 0.0 <= msg.remaining <= 1.0:
            self._current_telemetry.status.battery_remaining = float(msg.remaining)

    def _gps_cb(self, msg: SensorGps):
        """订阅真实 GPS 状态."""
        self._current_telemetry.status.gps_fix_type = msg.fix_type
        self._current_telemetry.status.gps_satellites = msg.satellites_used

    def _cmd_relay_cb(self, msg: VehicleCommand):
        """中继 VehicleCommand — 从 /drone/cmd/vehicle_command_relay 转发到 PX4."""
        if not self._autonomy_lease_valid():
            return
        self._cmd_pub.publish(msg)
        self.get_logger().debug(
            f"VehicleCommand relay: cmd={msg.command} "
            f"p1={msg.param1} p2={msg.param2}"
        )

    def _cmd_setpoint_cb(self, msg: NedPositionSetpoint):
        """接收 Mission FSM 的 setpoint 指令并更新目标."""
        with self._setpoint_lock:
            if not self._autonomy_lease_valid_locked():
                return
            target = Position(
                north=float(msg.north),
                east=float(msg.east),
                down=float(msg.down),
            )
            if not self._autonomy_target_received:
                self._current_setpoint = target
                self._smoothed_yaw = float(msg.yaw)
            self._velocity_mode = False
            self._target_setpoint = target
            self._target_yaw = float(msg.yaw)
            speed = float(msg.speed)
            self._target_speed = (
                speed if msg.use_speed and math.isfinite(speed) and speed > 0.0 else None
            )
            self._autonomy_target_received = True

    def _cmd_velocity_cb(self, msg):
        """接收 Mission FSM 的速度指令 (NED velocity)."""
        with self._setpoint_lock:
            if not self._autonomy_lease_valid_locked():
                return
            self._velocity_mode = True
            self._target_setpoint = None
            self._target_speed = None
            self._autonomy_target_received = True
        self.publish_velocity_cmd(
            float(msg.linear.x),   # vn
            float(msg.linear.y),   # ve
            float(msg.linear.z),   # vd
            float(msg.angular.z),  # yaw_rate
        )

    def _vision_velocity_cb(self, msg: TwistStamped):
        """Forward visual-servo velocity, which is expressed in NED."""
        with self._setpoint_lock:
            if not self._autonomy_lease_valid_locked():
                return
            self._velocity_mode = True
            self._target_setpoint = None
            self._target_speed = None
            self._autonomy_target_received = True
        self.publish_velocity_cmd(
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            float(msg.twist.linear.z),
            float(msg.twist.angular.z),
        )

    # ================================================================
    # 发布方法
    # ================================================================

    def _publish_drone_pose(self):
        """发布 World 坐标系下的无人机位姿."""
        if self._odom is None:
            return
        if self.get_parameter("quick_drop_profile").value == "bucket":
            attitude_time = self._last_attitude_time
            if attitude_time is None or time.monotonic() - attitude_time > 0.10:
                return
        pos = self._odom.position
        att = self._attitude
        q = att.q if att and len(att.q) >= 4 else [1.0, 0.0, 0.0, 0.0]

        world_x, world_y, world_z, qx, qy, qz, qw = ned_to_world_pose(
            float(pos[0]), float(pos[1]), float(pos[2]),
            float(q[1]) if att else 0.0,
            float(q[2]) if att else 0.0,
            float(q[3]) if att else 0.0,
            float(q[0]) if att else 1.0,
        )

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "world"
        pose.pose.position.x = world_x
        pose.pose.position.y = world_y
        pose.pose.position.z = world_z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self._pose_pub.publish(pose)

    def publish_velocity_cmd(self, vn: float, ve: float, vd: float, yaw_rate: float = 0.0):
        """
        发布速度指令 — 同时发布到 ROS2 话题和 PX4 TrajectorySetpoint.

        PX4 velocity setpoint: vx=north, vy=east, vz=down (NED).

        缓存速度指令以防止 _offboard_heartbeat 覆写为 NAN.
        """
        with self._setpoint_lock:
            if not self._autonomy_lease_valid_locked():
                return
            # 缓存速度指令, 供心跳持续发布
            self._last_velocity_cmd = (float(vn), float(ve), float(vd))
            self._autonomy_target_received = True

            # 发布到 ROS2 话题 (供监控/日志)
            vel = TwistStamped()
            vel.header.stamp = self.get_clock().now().to_msg()
            vel.header.frame_id = "ned"
            vel.twist.linear.x = vn
            vel.twist.linear.y = ve
            vel.twist.linear.z = vd
            vel.twist.angular.z = yaw_rate
            self._vel_pub.publish(vel)

            # 直接作为 TrajectorySetpoint 的 velocity 域发布到 PX4
            setpoint = TrajectorySetpoint()
            setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            setpoint.position = [float('nan'), float('nan'), float('nan')]
            setpoint.velocity = [float(vn), float(ve), float(vd)]
            setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
            setpoint.yaw = float(self._target_yaw)
            setpoint.yawspeed = float('nan')
            self._trajectory_pub.publish(setpoint)

    # ================================================================
    # 心跳 + 控制循环
    # ================================================================

    def _offboard_heartbeat(self):
        """
        10Hz Offboard 心跳.

        必须持续发布 OffboardControlMode + TrajectorySetpoint,
        否则 PX4 会自动退出 Offboard 模式 (>0.5s 无信号).
        """
        with self._setpoint_lock:
            if not self._autonomy_lease_valid_locked() or not self._autonomy_target_received:
                return
            sp = self._current_setpoint
            yaw = self._smoothed_yaw
            is_vel_mode = self._velocity_mode

            # OffboardControlMode: 声明控制类型
            offboard_mode = OffboardControlMode()
            offboard_mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            offboard_mode.position = not is_vel_mode
            offboard_mode.velocity = is_vel_mode
            offboard_mode.acceleration = False
            offboard_mode.attitude = False
            offboard_mode.body_rate = False
            self._offboard_mode_pub.publish(offboard_mode)

            # TrajectorySetpoint: 位置/速度指令
            setpoint = TrajectorySetpoint()
            setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)

            if is_vel_mode:
                # 速度模式: 位置=NAN, 速度=缓存的上次指令 (避免覆写为 NAN)
                setpoint.position = [float('nan'), float('nan'), float('nan')]
                vn, ve, vd = self._last_velocity_cmd
                setpoint.velocity = [float(vn), float(ve), float(vd)]
                setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
                setpoint.yaw = float(self._target_yaw)
                setpoint.yawspeed = float('nan')
            else:
                setpoint.position = [float(sp.north), float(sp.east), float(sp.down)]
                setpoint.yaw = float(yaw)
                # NAN 表示"不控制此轴"
                setpoint.velocity = [float('nan'), float('nan'), float('nan')]
                setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
                setpoint.yawspeed = float('nan')

            self._trajectory_pub.publish(setpoint)

    def _control_loop(self):
        """
        20Hz 控制循环 — 斜坡平滑 Setpoint.

        逐步将 _current_setpoint 向 _target_setpoint 移动,
        实现平滑的位置过渡, 避免突然跳变.
        """
        with self._setpoint_lock:
            if not self._autonomy_lease_valid_locked() or not self._autonomy_target_received:
                return
            if self._velocity_mode:
                return
            target = self._target_setpoint
            current = self._current_setpoint
            target_yaw = self._target_yaw
            target_speed = self._target_speed

        safety_limit = float(self.get_parameter("max_setpoint_speed_limit").value)
        safety_limit = max(0.0, safety_limit)
        requested_speed = target_speed if target_speed is not None else safety_limit
        max_step = min(float(requested_speed), safety_limit) * 0.05  # per 20Hz tick

        # 平滑位置
        dx = target.north - current.north
        dy = target.east - current.east
        dz = target.down - current.down
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist > 0.001:
            step = min(max_step, dist)
            ratio = step / dist
            new_north = current.north + dx * ratio
            new_east = current.east + dy * ratio
            new_down = current.down + dz * ratio
        else:
            new_north = target.north
            new_east = target.east
            new_down = target.down

        # 平滑偏航 (使用独立的 smoothed 变量, 不与物理 _current_yaw 冲突)
        dyaw = target_yaw - self._smoothed_yaw
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))  # normalize
        new_yaw = self._smoothed_yaw + dyaw * 0.3  # low-pass

        with self._setpoint_lock:
            self._current_setpoint = Position(new_north, new_east, new_down)
            self._smoothed_yaw = new_yaw

    def _autonomy_lease_cb(self, msg: Bool) -> None:
        """Accept a live mission relay lease or fail closed on revocation."""
        with self._setpoint_lock:
            if bool(msg.data):
                self._autonomy_lease_enabled = True
                self._autonomy_lease_last_seen = time.monotonic()
                return
            self._clear_autonomy_output_locked()

    def _autonomy_lease_valid(self) -> bool:
        """Return whether the source lease is present and within its deadline."""
        with self._setpoint_lock:
            return self._autonomy_lease_valid_locked()

    def _autonomy_lease_valid_locked(self) -> bool:
        """Lease check for callers already holding ``_setpoint_lock``."""
        if not self._autonomy_lease_enabled or self._autonomy_lease_last_seen is None:
            return False
        now = time.monotonic()
        if (
            now < self._autonomy_lease_last_seen
            or now - self._autonomy_lease_last_seen > self._autonomy_lease_timeout_s
        ):
            self._clear_autonomy_output_locked()
            return False
        return True

    def _clear_autonomy_output_locked(self) -> None:
        """Invalidate every cached autonomous setpoint while holding the lock."""
        self._autonomy_lease_enabled = False
        self._autonomy_lease_last_seen = None
        self._autonomy_target_received = False
        self._velocity_mode = False
        self._target_setpoint = None
        self._current_setpoint = None
        self._target_speed = None
        self._last_velocity_cmd = (0.0, 0.0, 0.0)

    # ================================================================
    # 公开接口 (供 FlightController 和 Mission 层调用)
    # ================================================================

    def set_position_target(self, north: float, east: float, down: float,
                            yaw: float = 0.0,
                            speed: Optional[float] = None):
        """
        设置位置控制目标 (NED 坐标系).

        Parameters
        ----------
        north, east, down : float
            NED 位置 (m). down 为正值=下降, 负值=上升.
        yaw : float
            目标偏航角 (rad).

        """
        with self._setpoint_lock:
            self._velocity_mode = False
            self._target_setpoint = Position(north, east, down)
            self._target_yaw = yaw
            self._target_speed = (
                float(speed)
                if speed is not None and math.isfinite(speed) and speed > 0.0
                else None
            )

    def set_velocity_target(self, vn: float, ve: float, vd: float, yaw_rate: float = 0.0):
        """
        设置速度控制目标 (NED 坐标系).

        Parameters
        ----------
        vn, ve, vd : float
            NED 速度 (m/s). vd 正值=下降.
        yaw_rate : float
            偏航速率 (rad/s).

        """
        with self._setpoint_lock:
            self._velocity_mode = True
        # 速度模式: 直接发布 TrajectorySetpoint 的 velocity 域
        self.publish_velocity_cmd(vn, ve, vd, yaw_rate)

    def get_current_position(self) -> Position:
        """获取当前无人机位置 (NED)."""
        return self._current_telemetry.position

    def get_current_attitude(self) -> Attitude:
        """获取当前姿态."""
        return self._current_telemetry.attitude

    def get_telemetry(self) -> Telemetry:
        """获取完整遥测快照."""
        return self._current_telemetry

    def is_armed(self) -> bool:
        return self._armed

    def is_offboard(self) -> bool:
        return self._current_telemetry.status.offboard_mode

    def is_landed(self) -> bool:
        return self._current_telemetry.status.landed

    # ================================================================
    # 指令发送 (VehicleCommand)
    # ================================================================

    def send_command(self, command: int, param1: float = 0.0, param2: float = 0.0,
                     param3: float = 0.0, param4: float = 0.0,
                     param5: float = 0.0, param6: float = 0.0, param7: float = 0.0):
        """发送通用 VehicleCommand."""
        cmd = VehicleCommand()
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        cmd.param1 = param1
        cmd.param2 = param2
        cmd.param3 = param3
        cmd.param4 = param4
        cmd.param5 = param5
        cmd.param6 = param6
        cmd.param7 = param7
        cmd.command = command
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = self._component_id
        cmd.from_external = True
        self._cmd_pub.publish(cmd)

    def set_offboard_mode(self):
        """切换到 Offboard 模式."""
        self.get_logger().info("请求切换到 Offboard 模式...")
        self.send_command(
            VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # custom mode
            param2=6.0,  # offboard
        )

    def arm(self):
        """解锁无人机."""
        self.get_logger().info("发送解锁指令...")
        self.send_command(
            VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,  # arm
        )

    def disarm(self):
        """上锁无人机."""
        self.get_logger().info("发送上锁指令...")
        self.send_command(
            VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,  # disarm
        )

    def force_disarm(self):
        """强制上锁 (紧急).

        PX4 Offboard 模式下会拒绝非手动来源的 disarm, 必须传 magic=21196
        才能绕过此限制, 仿真和真机都是如此.
        """
        self.get_logger().warn("强制上锁!")
        self.send_command(
            VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
            param2=21196.0,
        )

    def land_at_current(self):
        """原地降落."""
        self.get_logger().info("请求着陆...")
        self.send_command(VEHICLE_CMD_NAV_LAND)

    def return_to_launch(self):
        """RTL 返航."""
        self.get_logger().info("请求 RTL 返航...")
        self.send_command(VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)


# ============================================================
# 入口
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = PX4Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
