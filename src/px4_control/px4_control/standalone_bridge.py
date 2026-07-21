"""
Standalone bridge — PX4Bridge-compatible interface over ROS2 relay topics.

Use this when px4_bridge runs as a separate node and you want to control
the drone through relay topics rather than direct PX4 DDS communication.

与 PX4Bridge 的区别:
  - PX4Bridge: 直接通过 DDS 发布 /fmu/in/* 话题 (需要同进程)
  - StandaloneBridge: 通过 /drone/cmd/* 中继话题间接控制
    (px4_bridge 订阅中继话题并转发到 PX4)

用法:
    bridge = StandaloneBridge(node)
    fc = FlightController(bridge)
    fc.takeoff(5.0)
    fc.goto_world(42.5, 4.0, 5.0)
"""

import math
import time
from typing import Optional, Tuple

from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from geometry_msgs.msg import Point, Twist
from px4_msgs.msg import (
    BatteryStatus,
    VehicleAttitude,
    VehicleCommand,
    VehicleLandDetected,
    VehicleOdometry,
    VehicleStatus,
)

from px4_control.telemetry import (
    Attitude,
    Position,
    Telemetry,
    Velocity,
)


def _px4_qos() -> QoSProfile:
    """PX4 micro-XRCE-DDS QoS profile."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )


# ============================================================
# StandaloneBridge
# ============================================================


class StandaloneBridge:
    """
    PX4Bridge-compatible interface over px4_bridge relay topics.

    Implements the same interface as PX4Bridge so FlightController can
    work transparently in both standalone and direct-DDS modes.
    ...
    Publishers:
      /drone/cmd_setpoint_ned           (Point)   — position setpoints
      /drone/cmd_velocity_ned           (Twist)   — velocity setpoints
      /drone/cmd/vehicle_command_relay  (VehicleCommand) — mode/arm/land

    Subscriptions:
      /fmu/out/vehicle_odometry
      /fmu/out/vehicle_status_v4
      /fmu/out/vehicle_land_detected
      /fmu/out/vehicle_attitude
      /fmu/out/battery_status_v1
    """

    NAV_STATE_OFFBOARD = 14
    VEHICLE_CMD_DO_SET_MODE = 176
    VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
    VEHICLE_CMD_NAV_LAND = 189
    VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 93

    def __init__(self, node: Node):
        """
        Create a standalone bridge for the given ROS2 node.

        Parameters
        ----------
        node : rclpy.node.Node
            The ROS2 node that owns publishers/subscriptions/timers.

        """
        self._node = node
        qos = _px4_qos()

        # ---- Telemetry state ----
        self._position = Position(0.0, 0.0, 0.0)
        self._attitude = Attitude()
        self._velocity = Velocity()
        self._armed: bool = False
        self._offboard: bool = False
        self._landed: bool = True
        self._current_yaw: float = 0.0
        self._battery_remaining: float = 1.0
        self._battery_ready: bool = False

        # ---- Setpoint state ----
        self._target_ned: Optional[Tuple[float, float, float]] = None
        self._velocity_mode: bool = False
        self._vel_target: Tuple[float, float, float] = (0.0, 0.0, 0.0)

        # ---- Publishers (relay topics → px4_bridge) ----
        self._sp_pub = node.create_publisher(Point, "/drone/cmd_setpoint_ned", 10)
        self._vel_pub = node.create_publisher(Twist, "/drone/cmd_velocity_ned", 10)
        self._cmd_pub = node.create_publisher(
            VehicleCommand, "/drone/cmd/vehicle_command_relay", 10,
        )

        # ---- Subscriptions (PX4 telemetry) ----
        node.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._odom_cb, qos,
        )
        node.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v4", self._status_cb, qos,
        )
        node.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected", self._land_cb, qos,
        )
        node.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude", self._attitude_cb, qos,
        )
        node.create_subscription(
            BatteryStatus, "/fmu/out/battery_status_v1", self._battery_cb, qos,
        )

        # ---- Heartbeat timer (10Hz) ----
        self._hb_timer = node.create_timer(0.1, self._heartbeat)

    # ----------------------------------------------------------------
    # Heartbeat
    # ----------------------------------------------------------------

    def _heartbeat(self) -> None:
        """10Hz — publish current setpoint to maintain offboard mode."""
        if self._velocity_mode:
            msg = Twist()
            msg.linear.x = float(self._vel_target[0])
            msg.linear.y = float(self._vel_target[1])
            msg.linear.z = float(self._vel_target[2])
            self._vel_pub.publish(msg)
        elif self._target_ned is not None:
            msg = Point()
            msg.x = float(self._target_ned[0])
            msg.y = float(self._target_ned[1])
            msg.z = float(self._target_ned[2])
            self._sp_pub.publish(msg)

    # ----------------------------------------------------------------
    # Telemetry callbacks
    # ----------------------------------------------------------------

    def _odom_cb(self, msg: VehicleOdometry) -> None:
        self._position = Position(
            north=float(msg.position[0]),
            east=float(msg.position[1]),
            down=float(msg.position[2]),
        )
        self._velocity = Velocity(
            vn=float(msg.velocity[0]),
            ve=float(msg.velocity[1]),
            vd=float(msg.velocity[2]),
        )

    def _status_cb(self, msg: VehicleStatus) -> None:
        self._armed = (msg.arming_state == 2)
        self._offboard = (msg.nav_state == self.NAV_STATE_OFFBOARD)

    def _land_cb(self, msg: VehicleLandDetected) -> None:
        self._landed = msg.landed

    def _attitude_cb(self, msg: VehicleAttitude) -> None:
        q = msg.q  # [w, x, y, z]
        if len(q) >= 4:
            w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
            self._attitude = Attitude(q_w=w, q_x=x, q_y=y, q_z=z)
            self._current_yaw = math.atan2(
                2.0 * (w * z + x * y),
                1.0 - 2.0 * (y * y + z * z),
            )

    def _battery_cb(self, msg: BatteryStatus) -> None:
        if msg.connected and 0.0 <= msg.remaining <= 1.0:
            self._battery_remaining = float(msg.remaining)
            self._battery_ready = True

    # ----------------------------------------------------------------
    # PX4Bridge-compatible interface
    # ----------------------------------------------------------------

    def get_current_position(self) -> Position:
        return self._position

    def get_current_attitude(self) -> Attitude:
        return self._attitude

    def get_telemetry(self) -> Telemetry:
        t = Telemetry()
        t.timestamp = time.time()
        t.position = self._position
        t.attitude = self._attitude
        t.velocity = self._velocity
        t.status.armed = self._armed
        t.status.offboard_mode = self._offboard
        t.status.landed = self._landed
        t.status.battery_remaining = self._battery_remaining
        return t

    def is_armed(self) -> bool:
        return self._armed

    def is_offboard(self) -> bool:
        return self._offboard

    def is_landed(self) -> bool:
        return self._landed

    @property
    def battery_remaining(self) -> float:
        return self._battery_remaining

    @property
    def battery_ready(self) -> bool:
        return self._battery_ready

    # -- Setpoints --

    def set_position_target(self, north: float, east: float, down: float,
                            yaw: float = 0.0) -> None:
        """Set position control target (NED)."""
        self._velocity_mode = False
        self._target_ned = (float(north), float(east), float(down))

    def set_velocity_target(self, vn: float, ve: float, vd: float,
                            yaw_rate: float = 0.0) -> None:
        """Set velocity control target (NED)."""
        self._velocity_mode = True
        self._vel_target = (float(vn), float(ve), float(vd))

    # -- Commands --

    def set_offboard_mode(self) -> None:
        self._node.get_logger().info("[StandaloneBridge] 请求 Offboard 模式")
        self._send_command(self.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def arm(self) -> None:
        self._node.get_logger().info("[StandaloneBridge] 解锁请求 (SITL force)")
        self._send_command(self.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)

    def disarm(self) -> None:
        self._node.get_logger().info("[StandaloneBridge] 上锁请求")
        self._send_command(self.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)

    def force_disarm(self) -> None:
        self._node.get_logger().warn("[StandaloneBridge] 强制上锁!")
        self._send_command(self.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)

    def land_at_current(self) -> None:
        """原地降落 (MAV_CMD_NAV_LAND = 189)."""
        self._node.get_logger().info("[StandaloneBridge] 请求着陆")
        self._send_command(self.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)

    def return_to_launch(self) -> None:
        """RTL 返航."""
        self._node.get_logger().info("[StandaloneBridge] 请求 RTL")
        self._send_command(self.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH, 0.0, 0.0)

    # ----------------------------------------------------------------
    # Internal
    # ----------------------------------------------------------------

    def _send_command(self, command: int, param1: float = 0.0,
                      param2: float = 0.0) -> None:
        """Publish VehicleCommand through relay topic."""
        msg = VehicleCommand()
        msg.timestamp = int(time.time() * 1e6)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._cmd_pub.publish(msg)

    # -- Parameter delegation (to the owning Node) --

    def get_parameter(self, name: str):
        """Delegate to the owning ROS2 Node."""
        return self._node.get_parameter(name)

    def get_logger(self):
        return self._node.get_logger()

    def get_clock(self):
        return self._node.get_clock()
