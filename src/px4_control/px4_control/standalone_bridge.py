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
    fc.goto_world(4.0, 42.5, 5.0)
"""

import math
import threading
import time
from contextlib import nullcontext
from typing import Optional, Tuple

from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from drone_interfaces.msg import NedPositionSetpoint
from px4_msgs.msg import (
    BatteryStatus,
    VehicleAttitude,
    VehicleCommand,
    VehicleLandDetected,
    VehicleOdometry,
    VehicleStatus,
)

from px4_control.telemetry import (
    NAV_STATE_OFFBOARD,
    VEHICLE_CMD_COMPONENT_ARM_DISARM,
    VEHICLE_CMD_DO_SET_MODE,
    VEHICLE_CMD_NAV_LAND,
    VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,
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


def _px4_latest_telemetry_qos() -> QoSProfile:
    """Keep only the latest PX4 status/odometry sample for readiness."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _autonomy_lease_qos() -> QoSProfile:
    """Reliable, latched lease state shared with the DDS-owning bridge."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
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
      /drone/cmd_setpoint_ned           (NedPositionSetpoint) — position setpoints
      /drone/cmd_velocity_ned           (Twist)   — velocity setpoints
      /drone/cmd/vehicle_command_relay  (VehicleCommand) — mode/arm/land

    Subscriptions:
      /fmu/out/vehicle_odometry
      /fmu/out/vehicle_status_vN (selected by vehicle_status_topic)
      /fmu/out/vehicle_land_detected
      /fmu/out/vehicle_attitude
      /fmu/out/battery_status_v1
    """

    def __init__(self, node: Node, sim_mode: bool = True):
        """
        Create a standalone bridge for the given ROS2 node.

        Parameters
        ----------
        node : rclpy.node.Node
            The ROS2 node that owns publishers/subscriptions/timers.
        sim_mode : bool, optional
            Whether to run in simulation mode, by default True.

        """
        self._node = node
        self._sim_mode = sim_mode
        qos = _px4_qos()

        # ---- Telemetry state ----
        self._position = Position(0.0, 0.0, 0.0)
        self._attitude = Attitude()
        self._velocity = Velocity()
        self._armed: bool = False
        self._offboard: bool = False
        self._landed: bool = True
        self._land_ready: bool = False
        self._preflight_ok: bool = False
        self._current_yaw: float = 0.0
        self._battery_remaining: float = 1.0
        self._battery_ready: bool = False
        self._last_status_time: Optional[float] = None
        self._last_odom_time: Optional[float] = None
        self._last_status_source_timestamp: Optional[int] = None
        self._last_odom_source_timestamp: Optional[int] = None

        # ---- Setpoint state ----
        self._output_lock = threading.RLock()
        self._autonomous_output_enabled: bool = True
        self._target_ned: Optional[Tuple[float, float, float]] = None
        self._target_yaw: float = 0.0
        self._target_speed: Optional[float] = None
        self._velocity_mode: bool = False
        self._vel_target: Tuple[float, float, float] = (0.0, 0.0, 0.0)

        # ---- 指令限流: 防止 DDS 丢包/节点重启时以 20Hz 轰炸飞控 ----
        self._last_arm_cmd_time: float = 0.0
        self._last_offboard_cmd_time: float = 0.0
        self._telemetry_ready: bool = False  # 收到第一条 VehicleStatus 后才允许发指令

        # ---- Publishers (relay topics → px4_bridge) ----
        self._sp_pub = node.create_publisher(
            NedPositionSetpoint, "/drone/cmd_setpoint_ned", 10,
        )
        self._vel_pub = node.create_publisher(Twist, "/drone/cmd_velocity_ned", 10)
        self._cmd_pub = node.create_publisher(
            VehicleCommand, "/drone/cmd/vehicle_command_relay", 10,
        )
        self._lease_pub = node.create_publisher(
            Bool, "/drone/cmd/autonomy_lease", _autonomy_lease_qos(),
        )

        # ---- Subscriptions (PX4 telemetry) — keep refs for shutdown ----
        latest_telemetry_qos = _px4_latest_telemetry_qos()
        self._odom_sub = node.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._odom_cb,
            latest_telemetry_qos,
        )
        status_topic = str(node.get_parameter("vehicle_status_topic").value)
        self._status_sub = node.create_subscription(
            VehicleStatus, status_topic, self._status_cb, latest_telemetry_qos,
        )
        self._land_sub = node.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected", self._land_cb, qos,
        )
        self._att_sub = node.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude", self._attitude_cb, qos,
        )
        self._battery_sub = node.create_subscription(
            BatteryStatus, "/fmu/out/battery_status_v1", self._battery_cb, qos,
        )

        # ---- Heartbeat timer (10Hz) ----
        self._hb_timer = node.create_timer(0.1, self._heartbeat)

        # ---- Shutdown guard ----
        self._shutdown: bool = False

    # ----------------------------------------------------------------
    # Heartbeat
    # ----------------------------------------------------------------

    def _heartbeat(self) -> None:
        """10Hz — publish current setpoint to maintain offboard mode."""
        with self._output_guard():
            if self._shutdown:
                return
            self._publish_autonomy_lease_locked()
            if not self._outputs_enabled():
                return
            if self._velocity_mode:
                msg = Twist()
                msg.linear.x = float(self._vel_target[0])
                msg.linear.y = float(self._vel_target[1])
                msg.linear.z = float(self._vel_target[2])
                self._vel_pub.publish(msg)
            elif self._target_ned is not None:
                msg = NedPositionSetpoint()
                msg.north = float(self._target_ned[0])
                msg.east = float(self._target_ned[1])
                msg.down = float(self._target_ned[2])
                msg.yaw = float(self._target_yaw)
                msg.use_speed = self._target_speed is not None
                msg.speed = float(self._target_speed) if msg.use_speed else 0.0
                self._sp_pub.publish(msg)

    # ----------------------------------------------------------------
    # Lifecycle
    # ----------------------------------------------------------------

    def shutdown(self) -> None:
        """
        Stop periodic work and release resources.

        Call this when the bridge is being replaced by a direct PX4Bridge
        to avoid a dangling timer continuing to publish stale setpoints
        on the relay topics after the owning FSM has switched control paths.

        Idempotent: repeated calls are safe.
        """
        if self._shutdown:
            return
        self._shutdown = True
        self.stop_autonomous_output()
        if self._hb_timer is not None:
            self._hb_timer.cancel()
            self._hb_timer = None
        # Destroy subscriptions and publishers to stop callbacks from
        # writing to stale state and to release topic resources.
        for attr in ("_odom_sub", "_status_sub", "_land_sub",
                     "_att_sub", "_battery_sub"):
            sub = getattr(self, attr, None)
            if sub is not None:
                try:
                    self._node.destroy_subscription(sub)
                except AttributeError:
                    pass  # ROS2 pre-Humble compatibility
                setattr(self, attr, None)
        for attr in ("_sp_pub", "_vel_pub", "_cmd_pub"):
            pub = getattr(self, attr, None)
            if pub is not None:
                try:
                    self._node.destroy_publisher(pub)
                except AttributeError:
                    pass
                setattr(self, attr, None)
        self._node.get_logger().info("[StandaloneBridge] shutdown complete")

    # ----------------------------------------------------------------
    # Telemetry callbacks
    # ----------------------------------------------------------------

    def _odom_cb(self, msg: VehicleOdometry) -> None:
        if not self._accept_new_source_timestamp(
            "_last_odom_source_timestamp",
            getattr(msg, "timestamp_sample", getattr(msg, "timestamp", None)),
        ):
            return
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
        self._last_odom_time = time.monotonic()

    def _status_cb(self, msg: VehicleStatus) -> None:
        if not self._accept_new_source_timestamp(
            "_last_status_source_timestamp", getattr(msg, "timestamp", None),
        ):
            return
        self._armed = (msg.arming_state == 2)
        self._offboard = (msg.nav_state == NAV_STATE_OFFBOARD)
        self._preflight_ok = msg.pre_flight_checks_pass
        self._telemetry_ready = True  # 收到状态后允许发送指令
        self._last_status_time = time.monotonic()

    def _accept_new_source_timestamp(self, attribute: str, timestamp) -> bool:
        """Reject a replayed or out-of-order PX4 sample when it has a clock."""
        if timestamp is None:
            return True
        try:
            source_timestamp = int(timestamp)
        except (TypeError, ValueError):
            return True
        if source_timestamp <= 0:
            return True
        previous = getattr(self, attribute)
        if previous is not None and source_timestamp <= previous:
            return False
        setattr(self, attribute, source_timestamp)
        return True

    def _land_cb(self, msg: VehicleLandDetected) -> None:
        self._landed = msg.landed
        self._land_ready = True

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
    def land_ready(self) -> bool:
        """Return whether a PX4 land-detection sample has arrived."""
        return self._land_ready

    @property
    def battery_remaining(self) -> float:
        return self._battery_remaining

    @property
    def battery_ready(self) -> bool:
        return self._battery_ready

    @property
    def preflight_ok(self) -> bool:
        """PX4 飞控自检通过 (含 RC/GPS/传感器等所有 arming check)."""
        return self._preflight_ok

    @property
    def autonomous_output_enabled(self) -> bool:
        """Return whether autonomous position/velocity output is enabled."""
        with self._output_guard():
            return self._outputs_enabled()

    def telemetry_fresh(self, now: float, max_age_s: float = 2.0) -> bool:
        """Return whether status and odometry were both received recently."""
        now = float(now)
        max_age_s = float(max_age_s)
        if not math.isfinite(now) or not math.isfinite(max_age_s):
            return False
        if max_age_s < 0.0:
            return False
        if self._last_status_time is None or self._last_odom_time is None:
            return False
        status_age = now - self._last_status_time
        odom_age = now - self._last_odom_time
        return (
            0.0 <= status_age <= max_age_s
            and 0.0 <= odom_age <= max_age_s
        )

    # -- Setpoints --

    def set_position_target(self, north: float, east: float, down: float,
                            yaw: float = 0.0,
                            speed: Optional[float] = None) -> None:
        """Set position control target (NED)."""
        with self._output_guard():
            if not self._outputs_enabled():
                return
            self._velocity_mode = False
            self._target_ned = (float(north), float(east), float(down))
            self._target_yaw = float(yaw)
            self._target_speed = (
                float(speed)
                if speed is not None and math.isfinite(speed) and speed > 0.0
                else None
            )

    def set_velocity_target(self, vn: float, ve: float, vd: float,
                            yaw_rate: float = 0.0) -> None:
        """Set velocity control target (NED)."""
        with self._output_guard():
            if not self._outputs_enabled():
                return
            self._velocity_mode = True
            self._target_ned = None
            self._target_speed = None
            self._vel_target = (float(vn), float(ve), float(vd))

    def hold_for_takeover(self) -> None:
        """Atomically replace the active target with a zero-velocity hold."""
        with self._output_guard():
            if not self._outputs_enabled():
                return
            self._target_ned = None
            self._target_speed = None
            self._velocity_mode = True
            self._vel_target = (0.0, 0.0, 0.0)

    def stop_autonomous_output(self) -> None:
        """Latch setpoint output off and clear every position/velocity target."""
        with self._output_guard():
            self._autonomous_output_enabled = False
            self._target_ned = None
            self._target_speed = None
            self._velocity_mode = False
            self._vel_target = (0.0, 0.0, 0.0)
            self._publish_autonomy_lease_locked()

    def _publish_autonomy_lease_locked(self) -> None:
        """Publish the current relay lease while holding the output lock."""
        message = Bool()
        message.data = self._outputs_enabled() and not self._shutdown
        self._lease_pub.publish(message)

    def _outputs_enabled(self) -> bool:
        """Support legacy test doubles while real instances use the latch."""
        return getattr(self, "_autonomous_output_enabled", True)

    def _output_guard(self):
        """Return the output-state lock, or a no-op guard for old test doubles."""
        return getattr(self, "_output_lock", nullcontext())

    # -- Commands --

    def set_offboard_mode(self) -> None:
        """请求 Offboard 模式 (限流: 1Hz, 遥测就绪后)."""
        if not self._telemetry_ready:
            self._node.get_logger().warn(
                "[StandaloneBridge] 跳过 set_offboard: 遥测未就绪"
            )
            return
        if self._offboard:
            return  # 已在 Offboard, 不重复发
        now = time.time()
        if now - self._last_offboard_cmd_time < 1.0:
            return  # 1Hz 限流
        self._last_offboard_cmd_time = now
        self._node.get_logger().info("[StandaloneBridge] 请求 Offboard 模式")
        self._send_command(VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def arm(self) -> None:
        """解锁请求 (限流: 1Hz, 遥测就绪后, 已解锁则跳过)."""
        if not self._telemetry_ready:
            self._node.get_logger().warn(
                "[StandaloneBridge] 跳过 arm: 遥测未就绪"
            )
            return
        if self._armed:
            return  # 已解锁, 不重复发
        now = time.time()
        if now - self._last_arm_cmd_time < 1.0:
            return  # 1Hz 限流
        self._last_arm_cmd_time = now
        if self._sim_mode:
            self._node.get_logger().info("[StandaloneBridge] 解锁请求 (SITL force)")
            self._send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
        else:
            self._node.get_logger().info("[StandaloneBridge] 解锁请求 (标准)")
            self._send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)

    def disarm(self) -> None:
        self._node.get_logger().info("[StandaloneBridge] 上锁请求")
        self._send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)

    def force_disarm(self) -> None:
        """强制上锁 (紧急).

        PX4 Offboard 模式下会拒绝非手动来源的 disarm, 必须传 magic=21196
        才能绕过此限制, 仿真和真机都是如此.
        """
        self._node.get_logger().warn("[StandaloneBridge] 强制上锁!")
        self._send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)

    def land_at_current(self) -> None:
        """原地降落 (MAV_CMD_NAV_LAND = 21)."""
        self._node.get_logger().info("[StandaloneBridge] 请求着陆")
        self._send_command(VEHICLE_CMD_NAV_LAND, 0.0, 0.0)

    def return_to_launch(self) -> None:
        """RTL 返航."""
        self._node.get_logger().info("[StandaloneBridge] 请求 RTL")
        self._send_command(VEHICLE_CMD_NAV_RETURN_TO_LAUNCH, 0.0, 0.0)

    # ----------------------------------------------------------------
    # Internal
    # ----------------------------------------------------------------

    def _send_command(self, command: int, param1: float = 0.0,
                      param2: float = 0.0) -> None:
        """Publish VehicleCommand through relay topic."""
        msg = VehicleCommand()
        msg.timestamp = int(self._node.get_clock().now().nanoseconds / 1000)
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
