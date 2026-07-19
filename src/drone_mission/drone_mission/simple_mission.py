"""
简单任务 — 起飞 → 飞往投放区 → 返航 → 降落.

状态流:
  INIT → ARM → TAKEOFF → TRANSIT_TO_DROP → LOITER → RETURN → LAND → COMPLETE

通过 px4_bridge 的中继话题工作, 不直接与 PX4 DDS 通信:
  - 发布 /drone/cmd_setpoint_ned (geometry_msgs/Point) → px4_bridge 转发到 PX4
  - 发布 /drone/cmd/vehicle_command_relay → px4_bridge 转发到 /fmu/in/vehicle_command
  - 订阅 /fmu/out/* 获取遥测数据

坐标系:
  - World (Gazebo): x=east, y=north, z=up. 起降点 H 位于 (10, 4, 0).
  - NED: north=World_y-4, east=World_x-10, down=-World_z

用法:
  ros2 launch drone_bringup simple_mission.launch.py
  ros2 launch drone_bringup simple_mission.launch.py drop_x:=42.5 drop_y:=4.0 cruise_altitude:=5.0
"""

import math
import time
from enum import Enum
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleCommand, VehicleOdometry, VehicleStatus, VehicleLandDetected


# ============================================================
# QoS
# ============================================================

def _px4_qos() -> QoSProfile:
    """PX4 micro-XRCE-DDS QoS."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )


# ============================================================
# 坐标变换 (与 px4_control/telemetry.py 保持一致)
# ============================================================

H_WORLD_X = 10.0
H_WORLD_Y = 4.0


def world_to_ned(wx: float, wy: float, wz: float) -> Tuple[float, float, float]:
    """Gazebo World → PX4 NED."""
    return (wy - H_WORLD_Y, wx - H_WORLD_X, -wz)


def ned_to_world(n: float, e: float, d: float) -> Tuple[float, float, float]:
    """PX4 NED → Gazebo World."""
    return (e + H_WORLD_X, n + H_WORLD_Y, -d)


# ============================================================
# 任务状态
# ============================================================

class MissionState(Enum):
    INIT = "INIT"
    ARM = "ARM"
    TAKEOFF = "TAKEOFF"
    TRANSIT_TO_DROP = "TRANSIT_TO_DROP"
    LOITER = "LOITER"
    RETURN = "RETURN"
    LAND = "LAND"
    COMPLETE = "COMPLETE"


# ============================================================
# SimpleMission 节点
# ============================================================

class SimpleMission(Node):
    """
    简化版无人机任务 — 起飞 → 飞往投放区 → 悬停 → 返航 → 降落.

    不依赖 mission_state_machine / vision / servo.
    仅需 PX4 SITL + px4_bridge.
    """

    # ---- 常量 ----
    NAV_STATE_OFFBOARD = 14
    VEHICLE_CMD_DO_SET_MODE = 176
    VEHICLE_CMD_COMPONENT_ARM_DISARM = 400

    def __init__(self):
        super().__init__("simple_mission")

        # ---- 参数 ----
        self.declare_parameter("cruise_altitude", 5.0)       # 巡航高度 (m)
        self.declare_parameter("drop_x", 42.5)               # 投放区 World X
        self.declare_parameter("drop_y", 4.0)                # 投放区 World Y
        self.declare_parameter("home_x", 10.0)               # 起降点 World X
        self.declare_parameter("home_y", 4.0)                # 起降点 World Y
        self.declare_parameter("waypoint_tolerance", 0.5)    # 航点容差 (m)
        self.declare_parameter("takeoff_tolerance", 0.3)     # 起飞容差 (m)
        self.declare_parameter("loiter_time", 5.0)           # 投放区悬停时间 (s)
        self.declare_parameter("transit_timeout", 120.0)     # 航点超时 (s)
        self.declare_parameter("state_timeout", 180.0)       # 单状态超时 (s)

        # ---- QoS ----
        qos = _px4_qos()

        # ---- 订阅 PX4 遥测 ----
        self._odom_sub = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._odom_cb, qos,
        )
        self._status_sub = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v4", self._status_cb, qos,
        )
        self._land_sub = self.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected", self._land_cb, qos,
        )

        # ---- 发布指令 ----
        self._cmd_pub = self.create_publisher(
            VehicleCommand, "/drone/cmd/vehicle_command_relay", 10,
        )
        self._sp_pub = self.create_publisher(
            Point, "/drone/cmd_setpoint_ned", 10,
        )

        # ---- 内部状态 ----
        self._state: MissionState = MissionState.INIT
        self._state_start: float = 0.0
        self._sub_phase_start: float = 0.0
        self._stable_count: int = 0
        self._command_sent: bool = False

        # 遥测缓存
        self._odom: Optional[VehicleOdometry] = None
        self._armed: bool = False
        self._offboard: bool = False
        self._landed: bool = True

        # 航点
        self._target_ned: Optional[Tuple[float, float, float]] = None
        self._target_set_time: float = 0.0

        # ---- 定时器 ----
        self._setpoint_timer = self.create_timer(0.1, self._publish_setpoint)  # 10Hz
        self._ctrl_timer = self.create_timer(0.05, self._control_loop)         # 20Hz

        self.get_logger().info("=" * 60)
        self.get_logger().info("  SimpleMission — 简单任务节点已启动")
        self.get_logger().info(f"  起降点:   World({self.param('home_x')}, {self.param('home_y')})")
        self.get_logger().info(f"  投放区:   World({self.param('drop_x')}, {self.param('drop_y')})")
        self.get_logger().info(f"  巡航高度: {self.param('cruise_altitude')}m")
        self.get_logger().info("=" * 60)

    # ================================================================
    # 参数快捷方法
    # ================================================================

    def param(self, key: str):
        try:
            return self.get_parameter(key).value
        except Exception:
            return None

    # ================================================================
    # 遥测回调
    # ================================================================

    def _odom_cb(self, msg: VehicleOdometry):
        self._odom = msg

    def _status_cb(self, msg: VehicleStatus):
        self._armed = (msg.arming_state == 2)
        self._offboard = (msg.nav_state == self.NAV_STATE_OFFBOARD)

    def _land_cb(self, msg: VehicleLandDetected):
        self._landed = msg.landed

    # ================================================================
    # 位置获取
    # ================================================================

    def _get_ned(self) -> Tuple[float, float, float]:
        """获取当前 NED 位置."""
        if self._odom is None:
            return (0.0, 0.0, 0.0)
        p = self._odom.position
        return (float(p[0]), float(p[1]), float(p[2]))

    @staticmethod
    def _dist(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

    # ================================================================
    # 指令发送
    # ================================================================

    def _send_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        """通过 /drone/cmd/vehicle_command_relay 发送 PX4 指令."""
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

    def _go_to_world(self, wx: float, wy: float, wz: float):
        """设置 World 坐标航点."""
        n, e, d = world_to_ned(wx, wy, wz)
        self._target_ned = (n, e, d)
        self._target_set_time = time.time()
        wn, we, wd = ned_to_world(n, e, d)
        self.get_logger().info(
            f"  航点 → World({wn:.1f}, {we:.1f}, {wd:.1f}) NED({n:.1f}, {e:.1f}, {d:.1f})"
        )

    def _go_to_current_xy(self, altitude: float):
        """保持当前水平位置, 修改目标高度."""
        n, e, _ = self._get_ned()
        self._target_ned = (n, e, -altitude)
        self._target_set_time = time.time()

    def _publish_setpoint(self):
        """10Hz 持续发布 setpoint (维持 Offboard 模式)."""
        if self._target_ned is None:
            return
        msg = Point()
        msg.x = float(self._target_ned[0])
        msg.y = float(self._target_ned[1])
        msg.z = float(self._target_ned[2])
        self._sp_pub.publish(msg)

    # ================================================================
    # 主控制循环 (20Hz)
    # ================================================================

    def _control_loop(self):
        """20Hz FSM 控制循环."""
        if self._state == MissionState.COMPLETE:
            return

        elapsed = time.time() - self._state_start
        tol = self.param("waypoint_tolerance")

        # ---- INIT: 等待 PX4 连接 ----
        if self._state == MissionState.INIT:
            if self._odom is not None:
                self.get_logger().info("INIT: PX4 遥测已连接 → ARM")
                self._transition(MissionState.ARM)

        # ---- ARM: 解锁 ----
        elif self._state == MissionState.ARM:
            if not self._command_sent:
                self.get_logger().info("ARM: 发送解锁指令 (SITL force)...")
                self._send_command(self.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
                self._command_sent = True
                self._sub_phase_start = time.time()

            if self._armed:
                self.get_logger().info("ARM: 解锁成功 → TAKEOFF")
                self._transition(MissionState.TAKEOFF)
            elif time.time() - self._sub_phase_start > 15.0:
                self.get_logger().warn("ARM: 解锁超时, 重试")
                self._command_sent = False

        # ---- TAKEOFF: 切换到 Offboard + 起飞 ----
        elif self._state == MissionState.TAKEOFF:
            # 子阶段: 0=切Offboard, 1=爬升
            if not hasattr(self, '_takeoff_phase'):
                self._takeoff_phase = 0
                self._command_sent = False

            if self._takeoff_phase == 0:
                if not self._command_sent:
                    self.get_logger().info("TAKEOFF: 切换 Offboard 模式...")
                    self._send_command(self.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self._command_sent = True
                    self._sub_phase_start = time.time()

                if time.time() - self._sub_phase_start > 1.5:
                    cruise = self.param("cruise_altitude")
                    self.get_logger().info(f"TAKEOFF: 爬升至 {cruise}m")
                    self._go_to_current_xy(cruise)
                    self._takeoff_phase = 1
                    self._sub_phase_start = time.time()
                    self._stable_count = 0

            else:
                _, _, d = self._get_ned()
                alt_error = abs(d - (-self.param("cruise_altitude")))
                tol_takeoff = self.param("takeoff_tolerance")

                if alt_error < tol_takeoff:
                    self._stable_count += 1
                else:
                    self._stable_count = 0

                if self._stable_count >= 40:  # 2s 稳定
                    self.get_logger().info("TAKEOFF: 到达巡航高度 → TRANSIT_TO_DROP")
                    del self._takeoff_phase
                    self._transition(MissionState.TRANSIT_TO_DROP)
                elif time.time() - self._sub_phase_start > 30.0:
                    self.get_logger().warn("TAKEOFF: 爬升超时, 强制下一状态")
                    del self._takeoff_phase
                    self._transition(MissionState.TRANSIT_TO_DROP)

        # ---- TRANSIT_TO_DROP: 飞往投放区 ----
        elif self._state == MissionState.TRANSIT_TO_DROP:
            if not self._command_sent:
                dx = self.param("drop_x")
                dy = self.param("drop_y")
                alt = self.param("cruise_altitude")
                self.get_logger().info(f"TRANSIT: 飞往投放区 World({dx}, {dy}, {alt})")
                self._go_to_world(dx, dy, alt)
                self._command_sent = True
                self._stable_count = 0

            pos = self._get_ned()
            dist = self._dist(pos, self._target_ned) if self._target_ned else 0.0

            if dist < tol:
                self._stable_count += 1
            else:
                self._stable_count = 0

            if elapsed > 1.0 and int(elapsed * 4) % 20 == 0:
                self.get_logger().info(f"TRANSIT: dist={dist:.2f}m stable={self._stable_count}")

            if self._stable_count >= 20:  # 1s 稳定
                self.get_logger().info(f"TRANSIT: 到达投放区 → LOITER ({self.param('loiter_time')}s)")
                self._transition(MissionState.LOITER)
            elif elapsed > self.param("transit_timeout"):
                self.get_logger().warn("TRANSIT: 超时, 强制进入 LOITER")
                self._transition(MissionState.LOITER)

        # ---- LOITER: 悬停 ----
        elif self._state == MissionState.LOITER:
            if elapsed > self.param("loiter_time"):
                self.get_logger().info("LOITER: 悬停完成 → RETURN")
                self._transition(MissionState.RETURN)

        # ---- RETURN: 返航 ----
        elif self._state == MissionState.RETURN:
            if not self._command_sent:
                hx = self.param("home_x")
                hy = self.param("home_y")
                alt = self.param("cruise_altitude")
                self.get_logger().info(f"RETURN: 返航至 World({hx}, {hy}, {alt})")
                self._go_to_world(hx, hy, alt)
                self._command_sent = True
                self._stable_count = 0

            pos = self._get_ned()
            dist = self._dist(pos, self._target_ned) if self._target_ned else 0.0

            if dist < tol:
                self._stable_count += 1
            else:
                self._stable_count = 0

            if self._stable_count >= 20:
                self.get_logger().info("RETURN: 到达起降点 → LAND")
                self._transition(MissionState.LAND)
            elif elapsed > self.param("transit_timeout"):
                self.get_logger().warn("RETURN: 超时, 强制进入 LAND")
                self._transition(MissionState.LAND)

        # ---- LAND: 降落 ----
        elif self._state == MissionState.LAND:
            if not self._command_sent:
                self.get_logger().info("LAND: 发送着陆指令...")
                self._send_command(189, 0.0, 0.0)  # VEHICLE_CMD_NAV_LAND = 189
                self._command_sent = True
                self._sub_phase_start = time.time()

            if self._landed:
                self.get_logger().info("LAND: 已着陆, 上锁")
                self._send_command(self.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
                self._transition(MissionState.COMPLETE)
            elif time.time() - self._sub_phase_start > 30.0:
                self.get_logger().warn("LAND: 超时, 强制上锁")
                self._send_command(self.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
                self._transition(MissionState.COMPLETE)

    # ================================================================
    # 状态转移
    # ================================================================

    def _transition(self, state: MissionState):
        old = self._state
        self._state = state
        self._state_start = time.time()
        self._command_sent = False
        self._stable_count = 0
        self.get_logger().info(f"[FSM] {old.value} → {state.value}")


# ============================================================
# 入口
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
