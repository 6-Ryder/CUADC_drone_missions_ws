"""
简单任务 — 起飞前遥控器确认 → 飞往投放区 → 悬停 → 返航 → 降落.

状态流:
  INIT → WAIT_ARMED → TAKEOFF → TRANSIT_TO_DROP → LOITER → RETURN → LAND → COMPLETE

安全 (RC 硬件解锁模式):
  - 遥控器 Arm Switch → SBus → PX4 自检 → PX4 解锁
  - ROS 侧不发送 arm 指令, 只检测 is_armed()
  - COM_RC_IN_MODE=0 保证无物理 RC 信号时 PX4 拒绝解锁
  - rc_require_arm_signal:=false 可跳过等待 (无遥控器场景)

通过 StandaloneBridge + FlightController 控制无人机,
不直接与 PX4 DDS 通信, 通过 px4_bridge 中继话题间接控制.

坐标系:
  - World(ENU): x=east, y=north, z=up. 仿真以 Gazebo 世界为准,
    真机以 Home(解锁点) 为原点.
  - NED: north=World_y-Home_y, east=World_x-Home_x, down=-World_z

用法:
  仿真: ros2 launch drone_bringup simple_mission.launch.py
  真机: ros2 launch drone_bringup simple_mission.launch.py \
          sim_mode:=false home_x:=0.0 home_y:=0.0 drop_x:=2.0 drop_y:=20.0
"""

import time
from enum import Enum

import rclpy
from rclpy.node import Node

from px4_control.standalone_bridge import StandaloneBridge
from px4_control.flight_controller import FlightController
from px4_control.telemetry import configure_home, world_to_ned


# ============================================================
# 任务状态
# ============================================================

class MissionState(Enum):
    INIT = "INIT"
    WAIT_ARMED = "WAIT_ARMED"         # 等待遥控器解锁 (人拨动 Arm Switch)
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
    """简化版无人机任务 — 起飞 → 飞往投放区 → 悬停 → 返航 → 降落."""

    def __init__(self):
        super().__init__("simple_mission")

        # ---- 参数 ----
        self.declare_parameter("cruise_altitude", 5.0)
        self.declare_parameter("takeoff_height", 5.0)
        self.declare_parameter("drop_x", 10.0)
        self.declare_parameter("drop_y", 36.5)
        self.declare_parameter("home_x", 4.0)
        self.declare_parameter("home_y", 10.0)
        self.declare_parameter("waypoint_tolerance", 0.5)
        self.declare_parameter("takeoff_tolerance", 0.3)
        self.declare_parameter("waypoint_timeout", 120.0)
        self.declare_parameter("loiter_time", 5.0)
        self.declare_parameter("state_timeout", 180.0)
        self.declare_parameter("sim_mode", True)

        # 遥控器安全 — True=等待人用遥控器解锁, False=代码 arm 跳过
        self.declare_parameter("rc_require_arm_signal", True)

        # ---- 坐标变换 ----
        configure_home(
            self.get_param("home_x", 4.0),
            self.get_param("home_y", 10.0),
            0.0,
        )

        # ---- 控制层 ----
        sim_mode = self.get_param("sim_mode", True)
        self._bridge = StandaloneBridge(self, sim_mode=sim_mode)
        self._fc = FlightController(self._bridge)

        # ---- 状态机 ----
        self._state: MissionState = MissionState.INIT
        self._state_start: float = time.time()
        self._command_sent: bool = False
        self._stable_count: int = 0

        self._last_progress_log: float = 0.0

        # ---- arming 状态 (由 StandaloneBridge vehicle_status 回调驱动) ----
        # self._bridge.is_armed()  → PX4 解锁态 (RC Arm Switch 触发)
        # self._bridge.preflight_ok → PX4 自检通过 (含 RC/GPS/传感器)

        # ---- 控制循环 (20Hz) ----
        self._ctrl_timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info("=" * 60)
        self.get_logger().info("  SimpleMission — 简单任务节点已启动")
        self.get_logger().info(f"  sim_mode:  {sim_mode}")
        self.get_logger().info(f"  Home:      World({self.get_param('home_x')},"
                               f" {self.get_param('home_y')})")
        self.get_logger().info(f"  投放区:    World({self.get_param('drop_x')},"
                               f" {self.get_param('drop_y')})")
        self.get_logger().info(f"  巡航高度:  {self.get_param('cruise_altitude')}m")
        self.get_logger().info("  解锁: 遥控器 Arm Switch → PX4 → is_armed() → TAKEOFF")
        self.get_logger().info("  流程: INIT → WAIT_ARMED → TAKEOFF → TRANSIT")
        self.get_logger().info("         → LOITER → RETURN → LAND → COMPLETE")
        self.get_logger().info("=" * 60)

    # ================================================================
    # 参数快捷方法
    # ================================================================

    def get_param(self, key: str, default=None):
        try:
            return self.get_parameter(key).value
        except Exception:
            if default is None:
                self.get_logger().warn(
                    f"参数 '{key}' 未声明且无默认值, 返回 None, 可能导致运行时错误"
                )
                return None
            self.get_logger().warn(
                f"参数 '{key}' 读取失败, 使用默认值 {default}"
            )
            return default

    # ================================================================
    # 主控制循环 (20Hz)
    # ================================================================

    def _control_loop(self):
        """20Hz FSM 控制循环."""
        if self._state == MissionState.COMPLETE:
            return

        elapsed = time.time() - self._state_start
        tol = self.get_param("waypoint_tolerance", 0.5)
        cruise_alt = self.get_param("cruise_altitude", 5.0)

        # ---- INIT: 等待遥测就绪 → WAIT_ARMED ----
        if self._state == MissionState.INIT:
            pos = self._bridge.get_current_position()
            if pos.norm_xy() > 0.01 or abs(pos.down) > 0.01 or elapsed > 2.0:
                self.get_logger().info("INIT: 遥测就绪 → WAIT_ARMED")
                self._transition(MissionState.WAIT_ARMED)

        # ---- WAIT_ARMED: 等待人用遥控器解锁 ----
        elif self._state == MissionState.WAIT_ARMED:
            require_rc = self.get_param("rc_require_arm_signal", True)
            if not require_rc:
                self.get_logger().info("WAIT_ARMED: RC 检查已禁用 → TAKEOFF")
                self._transition(MissionState.TAKEOFF)
                return

            # PX4 已由人通过遥控器 Arm Switch 解锁
            if self._bridge.is_armed():
                self.get_logger().info(
                    "WAIT_ARMED: ✓ 已解锁 (遥控器 Arm Switch) → TAKEOFF"
                )
                self._transition(MissionState.TAKEOFF)
                return

            # 每 3s 输出状态
            if elapsed - self._last_progress_log > 3.0:
                self._last_progress_log = elapsed
                preflight = self._bridge.preflight_ok
                if preflight:
                    hint = "自检通过! 请拨动遥控器解锁开关..."
                else:
                    hint = "等待飞控自检 (请检查遥控器/GPS/传感器)..."
                self.get_logger().info(
                    f"WAIT_ARMED: {hint}"
                    f" armed=False preflight_ok={preflight}"
                )

        # ---- TAKEOFF: 起飞至巡航高度 ----
        elif self._state == MissionState.TAKEOFF:
            self._fc.takeoff(cruise_alt)
            target_down = -cruise_alt
            pos = self._bridge.get_current_position()

            if abs(pos.down - target_down) < self.get_param("takeoff_tolerance", 0.3):
                self._stable_count += 1
            else:
                self._stable_count = 0

            if self._stable_count >= 40:  # 2s 稳定 @ 20Hz
                self.get_logger().info(f"TAKEOFF: 到达巡航高度 {cruise_alt}m → TRANSIT_TO_DROP")
                self._transition(MissionState.TRANSIT_TO_DROP)
            elif elapsed > 30.0:
                self.get_logger().warn("TAKEOFF: 爬升超时 (30s), 强制下一状态")
                self._transition(MissionState.TRANSIT_TO_DROP)

        # ---- TRANSIT_TO_DROP: 飞往投放区 ----
        elif self._state == MissionState.TRANSIT_TO_DROP:
            if not self._command_sent:
                dx = self.get_param("drop_x", 10.0)
                dy = self.get_param("drop_y", 36.5)
                self.get_logger().info(f"TRANSIT: 飞往投放区 World({dx}, {dy}, {cruise_alt})")
                self._fc.goto_world(dx, dy, cruise_alt, speed=2.0)
                self._command_sent = True

            n, e, d = world_to_ned(
                self.get_param("drop_x", 10.0),
                self.get_param("drop_y", 36.5),
                cruise_alt,
            )
            if self._fc.reached((n, e, d), tol):
                loiter_t = self.get_param("loiter_time", 5.0)
                self.get_logger().info(f"TRANSIT: 到达投放区 → LOITER ({loiter_t}s)")
                self._transition(MissionState.LOITER)
            elif elapsed > self.get_param("waypoint_timeout", 120.0):
                self.get_logger().warn("TRANSIT: 超时, 强制进入 LOITER")
                self._transition(MissionState.LOITER)

        # ---- LOITER: 悬停 ----
        elif self._state == MissionState.LOITER:
            self._fc.loiter()
            if elapsed > self.get_param("loiter_time", 5.0):
                self.get_logger().info("LOITER: 悬停完成 → RETURN")
                self._transition(MissionState.RETURN)

        # ---- RETURN: 返航 ----
        elif self._state == MissionState.RETURN:
            if not self._command_sent:
                hx = self.get_param("home_x", 4.0)
                hy = self.get_param("home_y", 10.0)
                self.get_logger().info(f"RETURN: 返航至 World({hx}, {hy}, {cruise_alt})")
                self._fc.goto_world(hx, hy, cruise_alt, speed=3.0)
                self._command_sent = True

            n, e, d = world_to_ned(
                self.get_param("home_x", 4.0),
                self.get_param("home_y", 10.0),
                cruise_alt,
            )
            if self._fc.reached((n, e, d), tol):
                self.get_logger().info("RETURN: 到达起降点上方 → LAND")
                self._transition(MissionState.LAND)
            elif elapsed > self.get_param("waypoint_timeout", 120.0):
                self.get_logger().warn("RETURN: 超时, 强制进入 LAND")
                self._transition(MissionState.LAND)

        # ---- LAND: 降落 ----
        elif self._state == MissionState.LAND:
            landed = self._fc.land()
            if landed:
                self.get_logger().info("LAND: 着陆完成 → COMPLETE")
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
        self._last_progress_log = 0.0
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
