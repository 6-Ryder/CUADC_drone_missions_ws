"""
分阶段真机实飞测试 — 向前直飞 20m → 侦察区任务.

Phase 1: 起飞 → 向北直飞 20m → 验证航点到达
Phase 2: 降至侦察高度 → 弓字形全覆盖扫描 → 直接返航(不爬升) → 降落

状态流:
  INIT → WAIT_ARMED → TAKEOFF → TRANSIT_FORWARD → DESCEND_TO_RECON
  → RECON_SWEEP → RETURN → LAND → COMPLETE

安全 (RC 硬件解锁模式):
  - 遥控器 Arm Switch → SBus → PX4 自检 → PX4 解锁
  - ROS 侧不发送 arm 指令, 只检测 preflight_ok + is_armed()
  - COM_RC_IN_MODE=0 保证无物理 RC 信号时 PX4 拒绝解锁
  - rc_require_arm_signal:=false 可跳过等待 (无遥控器场景)

坐标系:
  - World(ENU): x=east, y=north, z=up. 真机以 Home(解锁点) 为原点.
  - NED: north=World_y-Home_y, east=World_x-Home_x, down=-World_z
  - 飞行方向: 正北 (World y+) 20m

用法:
  真机: ros2 launch test_mission test_forward_recon.launch.py sim_mode:=false
"""

import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node

from px4_control.standalone_bridge import StandaloneBridge
from px4_control.flight_controller import FlightController
from px4_control.telemetry import configure_home, world_to_ned

from drone_mission.path_planner import PathPlanner

from drone_interfaces.msg import LandingTarget
from drone_interfaces.srv import SetVisionMode
from std_msgs.msg import String


# ============================================================
# 任务状态
# ============================================================

class MissionState(Enum):
    INIT = "INIT"
    WAIT_ARMED = "WAIT_ARMED"                 # 等待遥控器解锁 (人拨动 Arm Switch)
    TAKEOFF = "TAKEOFF"
    TRANSIT_FORWARD = "TRANSIT_FORWARD"       # Phase 1: 向北直飞 20m
    DESCEND_TO_RECON = "DESCEND_TO_RECON"      # Phase 2: 降至侦察高度
    RECON_SWEEP = "RECON_SWEEP"                # Phase 2: 弓字形全覆盖
    RETURN = "RETURN"                          # 保持侦察高度直接返航, 不爬升
    LAND = "LAND"
    COMPLETE = "COMPLETE"


# ============================================================
# TestForwardRecon 节点
# ============================================================

class TestForwardRecon(Node):
    """
    分阶段真机测试 — 向前直飞 20m + 侦察区弓字形全覆盖扫描.

    RC 解锁模式:
      - 遥控器 Arm Switch → SBus → PX4 解锁 → ROS 检测 is_armed()
      - 代码不发送 arm 指令, 由人通过硬件遥控器完成解锁
      - preflight_ok (含 RC/GPS/传感器自检) 作为辅助状态提示

    Phase 1 — 基础飞行验证:
      - 起飞至巡航高度 (5m)
      - 向北水平直飞 20m
      - 到达确认 (位置 + 连续稳定帧数)

    Phase 2 — 侦察区任务:
      - 降至侦察扫描高度 (默认 3m)
      - 弓字形 (Boustrophedon) 全覆盖路径
      - 保持侦察高度直接返航 (不爬升, 省时)
      - 精准降落
    """

    STABLE_TICKS = 40       # 高度/航点稳定确认帧数 (2s @ 20Hz)
    WP_STABLE_TICKS = 8     # 侦察航点稳定确认帧数 (0.4s @ 20Hz)

    def __init__(self):
        super().__init__("test_forward_recon")

        # ---- 参数声明 ----
        # 通用
        self.declare_parameter("sim_mode", True)
        self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status_v1")
        self.declare_parameter("cruise_altitude", 5.0)
        self.declare_parameter("takeoff_height", 5.0)
        self.declare_parameter("waypoint_tolerance", 0.5)
        self.declare_parameter("takeoff_tolerance", 0.3)
        self.declare_parameter("waypoint_timeout", 120.0)
        self.declare_parameter("state_timeout", 180.0)
        self.declare_parameter("control_frequency", 20.0)

        # Home 点 (真机通常为解锁点即原点)
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_z", 0.0)

        # Phase 1: 前飞参数
        self.declare_parameter("forward_distance", 20.0)   # 向北飞行距离 (m)
        self.declare_parameter("transit_speed", 2.0)       # 过渡飞行速度 (m/s)

        # Phase 2: 侦察区参数
        self.declare_parameter("recon_altitude", 3.0)          # 侦察扫描高度 (m)
        self.declare_parameter("recon_area_half_north", 2.5)   # 侦察区 north 半宽 (m), 对齐主任务 Area2 5m
        self.declare_parameter("recon_area_half_east", 4.0)    # 侦察区 east 半宽 (m), 对齐主任务 Area2 8m
        self.declare_parameter("sweep_speed", 0.8)             # 扫描飞行速度 (m/s)
        self.declare_parameter("grid_spacing", 1.67)           # 条带间距 (m), 对齐 PathPlanner 默认
        self.declare_parameter("sweep_margin_north", 0.5)      # north 方向边距 (m)
        self.declare_parameter("sweep_margin_east", 0.5)       # east 方向边距 (m)

        # 返航
        self.declare_parameter("return_speed", 3.0)

        # 遥控器安全 — True=等待人用遥控器解锁, False=代码 arm 跳过
        self.declare_parameter("rc_require_arm_signal", True)

        # ---- 精准降落参数 (与 state_precision_land.py 对齐) ----
        self.declare_parameter("camera_cx", 320.0)
        self.declare_parameter("camera_cy", 240.0)
        self.declare_parameter("landing_search_timeout", 15.0)
        self.declare_parameter("landing_servo_kp_pixel", 0.004)
        self.declare_parameter("landing_servo_max_horizontal_vel", 0.6)
        self.declare_parameter("landing_servo_deadzone_px", 12.0)
        self.declare_parameter("landing_servo_x_to_right_sign", 1.0)
        self.declare_parameter("landing_servo_y_to_forward_sign", -1.0)
        self.declare_parameter("landing_descent_fast", 0.4)
        self.declare_parameter("landing_descent_medium", 0.25)
        self.declare_parameter("landing_descent_slow", 0.12)
        self.declare_parameter("landing_blind_altitude", 0.5)
        self.declare_parameter("vision_loss_timeout", 3.0)

        # ---- 坐标变换 ----
        configure_home(
            self.get_param("home_x", 0.0),
            self.get_param("home_y", 0.0),
            self.get_param("home_z", 0.0),
        )

        # ---- 控制层 ----
        sim_mode = self.get_param("sim_mode", True)
        self._bridge = StandaloneBridge(self, sim_mode=sim_mode)
        self._fc = FlightController(self._bridge)

        # ---- 路径规划器 ----
        self._planner = PathPlanner(
            sweep_speed=self.get_param("sweep_speed", 0.8),
            transit_speed=self.get_param("transit_speed", 2.0),
            return_speed=self.get_param("return_speed", 3.0),
            grid_spacing=self.get_param("grid_spacing", 1.67),
            margin_north=self.get_param("sweep_margin_north", 0.5),
            margin_east=self.get_param("sweep_margin_east", 0.5),
        )

        # ---- 状态机 ----
        self._state: MissionState = MissionState.INIT
        self._state_start: float = time.time()
        self._command_sent: bool = False
        self._stable_count: int = 0

        # ---- arming 状态 (由 StandaloneBridge vehicle_status 回调驱动) ----
        # self._bridge.is_armed()  → PX4 解锁态 (RC Arm Switch 触发)
        # self._bridge.preflight_ok → PX4 自检通过 (含 RC/GPS/传感器)

        # ---- 侦察区航点 ----
        self._recon_waypoints: list = []
        self._recon_wp_idx: int = 0
        self._recon_wp_stable: int = 0

        # ---- 进度日志 ----
        self._last_progress_log: float = 0.0

        # ---- 返航高度 (RETURN 进入时捕获当前相对高度) ----
        self._return_altitude: float = 0.0

        # ---- 精准降落状态 ----
        self._LAND_PHASE_SEARCH = 0
        self._LAND_PHASE_SERVO = 1
        self._LAND_PHASE_BLIND = 2
        self._land_phase: int = 0
        self._landing_target: dict | None = None
        self._land_vision_lost_start: float = 0.0
        self._land_has_target: bool = False
        self._land_servo_converged: int = 0
        self._land_servo_last_log: float = 0.0
        self._land_px4_requested: bool = False

        # ---- 视觉订阅 ----
        try:
            self._landing_sub = self.create_subscription(
                LandingTarget, "/drone/vision/landing_target",
                self._landing_target_cb, 10,
            )
            self._vis_status_sub = self.create_subscription(
                String, "/drone/vision/status", self._vision_status_cb, 10,
            )
            self.get_logger().info("已订阅 /drone/vision/landing_target")
        except Exception as e:
            self.get_logger().warn(f"视觉话题订阅失败: {e}")

        # ---- 视觉模式切换服务 ----
        self._vision_mode_client = self.create_client(
            SetVisionMode, "/drone/vision/set_mode",
        )

        # ---- 控制循环 ----
        freq = self.get_param("control_frequency", 20.0)
        self._ctrl_timer = self.create_timer(1.0 / freq, self._control_loop)

        # ---- 启动日志 ----
        fwd = self.get_param("forward_distance", 20.0)
        ralt = self.get_param("recon_altitude", 3.0)
        half_n = self.get_param("recon_area_half_north", 2.5)
        half_e = self.get_param("recon_area_half_east", 4.0)

        self.get_logger().info("=" * 60)
        self.get_logger().info("  TestForwardRecon — 分阶段真机测试")
        self.get_logger().info(f"  sim_mode:     {sim_mode}")
        self.get_logger().info(f"  Home:         World({self.get_param('home_x')},"
                               f" {self.get_param('home_y')})")
        self.get_logger().info(f"  Phase 1: 向北直飞 {fwd}m"
                               f" @ {self.get_param('transit_speed')}m/s")
        self.get_logger().info(f"  Phase 2: 侦察区 "
                               f"N[{fwd - half_n}:{fwd + half_n}] ×"
                               f" E[{-half_e}:{half_e}] @ {ralt}m")
        self.get_logger().info("  解锁: 遥控器 Arm Switch → PX4 → is_armed() → TAKEOFF")
        self.get_logger().info("  流程: INIT → WAIT_ARMED → TAKEOFF → TRANSIT_FWD")
        self.get_logger().info("         → DESCEND → RECON_SWEEP → RETURN → LAND")
        self.get_logger().info("=" * 60)

    # ================================================================
    # 参数快捷方法
    # ================================================================

    def get_param(self, key: str, default=None):
        try:
            return self.get_parameter(key).value
        except Exception:
            if default is None:
                self.get_logger().warn(f"参数 '{key}' 未声明且无默认值, 返回 None")
            return default

    # ================================================================
    # 主控制循环 (20Hz)
    # ================================================================

    _FLIGHT_STATES = {
        MissionState.TAKEOFF,
        MissionState.TRANSIT_FORWARD,
        MissionState.DESCEND_TO_RECON,
        MissionState.RECON_SWEEP,
        MissionState.RETURN,
        MissionState.LAND,
    }

    def _control_loop(self):
        """20Hz FSM 控制循环."""
        if self._state == MissionState.COMPLETE:
            return

        elapsed = time.time() - self._state_start
        tol = self.get_param("waypoint_tolerance", 0.5)
        cruise_alt = self.get_param("cruise_altitude", 5.0)

        # ---- 飞行中遥控器上锁 → 立刻中止 ----
        if self._state in self._FLIGHT_STATES and not self._bridge.is_armed():
            self.get_logger().warn(
                f"{self._state.value}: 检测到遥控器上锁 (disarm), 中止任务"
            )
            self._transition(MissionState.COMPLETE)
            return

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
            pos = self._bridge.get_current_position()

            if abs(pos.down - self._fc.altitude_to_ned(cruise_alt)) < self.get_param("takeoff_tolerance", 0.3):
                self._stable_count += 1
            else:
                self._stable_count = 0

            if self._stable_count >= self.STABLE_TICKS:
                self.get_logger().info(
                    f"TAKEOFF: 到达巡航高度 {cruise_alt}m → TRANSIT_FORWARD"
                )
                self._transition(MissionState.TRANSIT_FORWARD)
            elif elapsed > 30.0:
                self.get_logger().warn("TAKEOFF: 爬升超时 (30s), 强制下一状态")
                self._transition(MissionState.TRANSIT_FORWARD)

        # ================================================================
        # Phase 1: TRANSIT_FORWARD — 向北直飞
        # ================================================================
        elif self._state == MissionState.TRANSIT_FORWARD:
            if not self._command_sent:
                fwd = self.get_param("forward_distance", 20.0)
                hx = self.get_param("home_x", 0.0)
                hy = self.get_param("home_y", 0.0)
                speed = self.get_param("transit_speed", 2.0)

                self.get_logger().info(
                    f"TRANSIT_FORWARD: 向北 {fwd}m → "
                    f"World({hx:.1f}, {hy + fwd:.1f}, {cruise_alt:.1f})"
                    f" @ {speed:.1f}m/s"
                )
                tn, te, _ = world_to_ned(hx, hy + fwd, 0)
                td = self._fc.altitude_to_ned(cruise_alt)
                self._fc.goto_ned(tn, te, td, yaw=0.0, speed=speed)
                self._command_sent = True

            # 到达检测: World → NED 转换 (XY 用 world_to_ned, Z 用相对高度)
            fwd = self.get_param("forward_distance", 20.0)
            hx = self.get_param("home_x", 0.0)
            hy = self.get_param("home_y", 0.0)
            tn, te, _ = world_to_ned(hx, hy + fwd, 0)
            td = self._fc.altitude_to_ned(cruise_alt)

            if self._fc.reached((tn, te, td), tol):
                self._stable_count += 1
            else:
                self._stable_count = max(0, self._stable_count - 1)

            if self._stable_count >= self.STABLE_TICKS:
                self.get_logger().info(
                    f"TRANSIT_FORWARD: ✓ Phase 1 完成"
                    f" — 到达 {fwd}m 点 → DESCEND_TO_RECON"
                )
                self._transition(MissionState.DESCEND_TO_RECON)
            elif elapsed > self.get_param("waypoint_timeout", 120.0):
                self.get_logger().warn("TRANSIT_FORWARD: 超时, 强制进入 Phase 2")
                self._transition(MissionState.DESCEND_TO_RECON)

            # 每 5s 输出剩余距离
            if elapsed - self._last_progress_log > 5.0:
                self._last_progress_log = elapsed
                pos = self._bridge.get_current_position()
                dist = math.hypot(pos.north - tn, pos.east - te)
                self.get_logger().info(
                    f"TRANSIT_FORWARD: dist_to_target={dist:.2f}m"
                    f" stable={self._stable_count}/{self.STABLE_TICKS}"
                )

        # ================================================================
        # Phase 2: DESCEND_TO_RECON — 降至侦察高度
        # ================================================================
        elif self._state == MissionState.DESCEND_TO_RECON:
            if not self._command_sent:
                fwd = self.get_param("forward_distance", 20.0)
                hx = self.get_param("home_x", 0.0)
                hy = self.get_param("home_y", 0.0)
                ralt = self.get_param("recon_altitude", 3.0)

                self.get_logger().info(
                    f"DESCEND: 降至侦察高度 {ralt}m → "
                    f"World({hx:.1f}, {hy + fwd:.1f}, {ralt:.1f})"
                )
                tn, te, _ = world_to_ned(hx, hy + fwd, 0)
                td = self._fc.altitude_to_ned(ralt)
                self._fc.goto_ned(tn, te, td, yaw=0.0, speed=1.0)
                self._command_sent = True

            target_down = self._fc.altitude_to_ned(self.get_param("recon_altitude", 3.0))
            pos = self._bridge.get_current_position()

            if abs(pos.down - target_down) < self.get_param("takeoff_tolerance", 0.3):
                self._stable_count += 1
            else:
                self._stable_count = max(0, self._stable_count - 1)

            if self._stable_count >= self.STABLE_TICKS:
                self.get_logger().info(
                    "DESCEND: 到达侦察高度 → RECON_SWEEP"
                )
                self._transition(MissionState.RECON_SWEEP)
            elif elapsed > 30.0:
                self.get_logger().warn("DESCEND: 降高超时 (30s), 强制进入 RECON_SWEEP")
                self._transition(MissionState.RECON_SWEEP)

        # ================================================================
        # Phase 2: RECON_SWEEP — 弓字形全覆盖扫描
        # ================================================================
        elif self._state == MissionState.RECON_SWEEP:
            if not self._command_sent:
                self._generate_recon_waypoints()
                self._command_sent = True

            if not self._recon_waypoints:
                self.get_logger().warn("RECON_SWEEP: 无航点, 跳过 → RETURN")
                self._transition(MissionState.RETURN)
                return

            # 所有航点完成 → 保持侦察高度直接返航
            if self._recon_wp_idx >= len(self._recon_waypoints):
                ralt = self.get_param("recon_altitude", 3.0)
                self.get_logger().info(
                    f"RECON_SWEEP: ✓ Phase 2 扫描完成"
                    f" — {len(self._recon_waypoints)} 个航点全部到达"
                    f" → RETURN (保持 {ralt}m, 不爬升)"
                )
                self._transition(MissionState.RETURN)
                return

            current_wp = self._recon_waypoints[self._recon_wp_idx]
            wp_target = (current_wp["north"], current_wp["east"], current_wp["down"])

            # 稳定到达检测 (防抖)
            if self._fc.reached(wp_target, tol):
                self._recon_wp_stable += 1
            else:
                self._recon_wp_stable = max(0, self._recon_wp_stable - 1)

            if self._recon_wp_stable >= self.WP_STABLE_TICKS:
                self._recon_wp_stable = 0
                self._recon_wp_idx += 1
                if self._recon_wp_idx < len(self._recon_waypoints):
                    self._fly_recon_wp(self._recon_wp_idx)

            # 航点超时保护
            if self._fc.is_waypoint_timed_out():
                self.get_logger().error(
                    f"RECON_SWEEP: 航点超时 wp={self._recon_wp_idx}, 跳过"
                )
                self._recon_wp_idx += 1
                self._recon_wp_stable = 0
                if self._recon_wp_idx < len(self._recon_waypoints):
                    self._fly_recon_wp(self._recon_wp_idx)

            # 全局超时 → PX4 RTL
            if elapsed > self.get_param("state_timeout", 180.0):
                self.get_logger().error(
                    f"RECON_SWEEP: 超时 ({elapsed:.0f}s), 触发 PX4 RTL"
                )
                self._bridge.return_to_launch()
                self._transition(MissionState.COMPLETE)
                return

            # 每 10s 输出进度
            if elapsed - self._last_progress_log > 10.0:
                self._last_progress_log = elapsed
                self.get_logger().info(
                    f"RECON_SWEEP: wp={self._recon_wp_idx}/{len(self._recon_waypoints)}"
                    f" elapsed={elapsed:.0f}s"
                )

        # ================================================================
        # RETURN: 保持侦察高度直接返航至 Home 点上方 (不爬升)
        # ================================================================
        elif self._state == MissionState.RETURN:
            if not self._command_sent:
                hx = self.get_param("home_x", 0.0)
                hy = self.get_param("home_y", 0.0)
                speed = self.get_param("return_speed", 3.0)
                # 捕获当前实际相对高度, 真正不爬升
                pos = self._bridge.get_current_position()
                self._return_altitude = self._fc.ned_to_altitude(pos.down)
                self.get_logger().info(
                    f"RETURN: 保持当前高度 {self._return_altitude:.1f}m 返航至"
                    f" Home World({hx:.1f}, {hy:.1f}, {self._return_altitude:.1f})"
                )
                tn, te, _ = world_to_ned(hx, hy, 0)
                td = self._fc.altitude_to_ned(self._return_altitude)
                self._fc.goto_ned(tn, te, td, yaw=0.0, speed=speed)
                self._command_sent = True

            hx = self.get_param("home_x", 0.0)
            hy = self.get_param("home_y", 0.0)
            tn, te, _ = world_to_ned(hx, hy, 0)
            td = self._fc.altitude_to_ned(self._return_altitude)

            if self._fc.reached((tn, te, td), tol):
                self._stable_count += 1
            else:
                self._stable_count = max(0, self._stable_count - 1)

            if self._stable_count >= self.STABLE_TICKS:
                self.get_logger().info("RETURN: 到达 Home 点上方 → LAND")
                self._transition(MissionState.LAND)
            elif elapsed > self.get_param("waypoint_timeout", 120.0):
                self.get_logger().warn("RETURN: 超时, 强制着陆")
                self._transition(MissionState.LAND)

        # ================================================================
        # LAND: H 标识精准降落 (三阶段: SEARCH → SERVO → BLIND)
        # ================================================================
        elif self._state == MissionState.LAND:
            self._handle_precision_land(elapsed)

    # ================================================================
    # 侦察区航点生成
    # ================================================================

    def _generate_recon_waypoints(self):
        """生成侦察区弓字形全覆盖航点 (NED 坐标系)."""
        fwd = self.get_param("forward_distance", 20.0)
        half_n = self.get_param("recon_area_half_north", 2.5)
        half_e = self.get_param("recon_area_half_east", 4.0)
        ralt = self.get_param("recon_altitude", 3.0)

        area_north_min = fwd - half_n
        area_north_max = fwd + half_n
        area_east_min = -half_e
        area_east_max = half_e

        self._recon_waypoints = self._planner.plan_coverage_ned(
            area_north_min=area_north_min,
            area_north_max=area_north_max,
            area_east_min=area_east_min,
            area_east_max=area_east_max,
            altitude=self._fc.altitude_to_ned(ralt),
        )

        num_strips = len(self._recon_waypoints) // 2
        self.get_logger().info(
            f"RECON_SWEEP: 生成 {len(self._recon_waypoints)} 个航点"
            f" ({num_strips} 条带),"
            f" 区域 NED N[{area_north_min:.1f}:{area_north_max:.1f}]"
            f" E[{area_east_min:.1f}:{area_east_max:.1f}],"
            f" 高度 {ralt:.1f}m,"
            f" 间距 {self.get_param('grid_spacing', 1.67)}m"
        )

        if self._recon_waypoints:
            self._fly_recon_wp(0)

    def _fly_recon_wp(self, idx: int):
        """发送侦察航点飞行指令."""
        wp = self._recon_waypoints[idx]
        self._fc.goto_ned(
            wp["north"], wp["east"], wp["down"],
            yaw=wp.get("yaw", 0.0),
            speed=wp.get("speed", self.get_param("sweep_speed", 0.8)),
        )

    # ================================================================
    # 精准降落 (H 标识视觉伺服, 与 state_precision_land.py 对齐)
    # ================================================================

    # ---- 回调 ----

    def _landing_target_cb(self, msg):
        """缓存最新的 H 标识检测结果."""
        self._landing_target = {
            "center_u": msg.center_u,
            "center_v": msg.center_v,
            "world_x": msg.world_x,
            "world_y": msg.world_y,
            "confidence": msg.confidence,
            "yaw_correction": msg.yaw_correction,
        }

    def _vision_status_cb(self, msg):
        """vision pipeline heartbeat (仅日志确认)."""
        pass

    # ---- 视觉模式切换 ----

    _VISION_MODE_NONE = 0
    _VISION_MODE_LANDING = 2
    _VISION_MODE_ALL = 3

    def _set_vision_mode(self, mode: str):
        """通过 ROS 服务切换视觉管线工作模式."""
        if not self._vision_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("set_vision_mode: 服务不可用")
            return
        try:
            req = SetVisionMode.Request()
            if mode == "landing":
                req.mode = self._VISION_MODE_LANDING
                req.enable = True
            elif mode == "none":
                req.mode = self._VISION_MODE_NONE
                req.enable = False
            else:
                req.mode = self._VISION_MODE_ALL
                req.enable = True
            self._vision_mode_client.call_async(req)
        except Exception as e:
            self.get_logger().warn(f"set_vision_mode 调用失败: {e}")

    # ---- 降落状态管理 ----

    def _reset_landing_state(self):
        """重置精准降落内部状态."""
        self._land_phase = self._LAND_PHASE_SEARCH
        self._landing_target = None
        self._land_vision_lost_start = 0.0
        self._land_has_target = False
        self._land_servo_converged = 0
        self._land_servo_last_log = 0.0
        self._land_px4_requested = False

    def _switch_to_blind(self):
        """转到盲降阶段并关闭视觉检测."""
        self._land_phase = self._LAND_PHASE_BLIND
        self.get_logger().info("PRECISION_LAND: Phase 3 - 盲降模式")
        self._set_vision_mode("none")

    def _request_px4_land(self):
        """兜底: 请求 PX4 自主着陆 (仅发送一次)."""
        if not self._land_px4_requested:
            self.get_logger().warn("PRECISION_LAND: 请求 PX4 自主着陆")
            self._bridge.land_at_current()
            self._land_px4_requested = True

    # ---- 主入口 ----

    def _handle_precision_land(self, elapsed: float):
        """LAND 状态: 三阶段精准降落调度."""
        # PX4 已检测到着陆 → 完成
        if self._bridge.is_landed():
            self.get_logger().info("PRECISION_LAND: 已着陆 → COMPLETE")
            self._transition(MissionState.COMPLETE)
            return

        # 检查当前目标
        lt = self._landing_target
        has_target = lt is not None and lt.get("confidence", 0) > 0.3

        # 视觉丢失计时
        if has_target:
            self._land_has_target = True
            self._land_vision_lost_start = 0.0
        elif self._land_has_target:
            if self._land_vision_lost_start == 0.0:
                self._land_vision_lost_start = time.time()

        # 视觉丢失超时 → PX4 LAND 兜底
        vis_loss_to = self.get_param("vision_loss_timeout", 3.0)
        if self._land_has_target and self._land_vision_lost_start > 0:
            if time.time() - self._land_vision_lost_start > vis_loss_to:
                self.get_logger().error("PRECISION_LAND: H 目标持续丢失")
                self._request_px4_land()
                self._transition(MissionState.COMPLETE)
                return

        # 分阶段执行
        if self._land_phase == self._LAND_PHASE_SEARCH:
            self._land_phase_search(elapsed, has_target)
        elif self._land_phase == self._LAND_PHASE_SERVO:
            self._land_phase_servo(has_target)
        elif self._land_phase == self._LAND_PHASE_BLIND:
            self._land_phase_blind(elapsed)

    # ---- Phase 1: 搜索 H 标识 ----

    def _land_phase_search(self, elapsed: float, has_target: bool):
        """在巡航高度悬停, 等待 H 标识出现."""
        if has_target:
            self.get_logger().info(
                "PRECISION_LAND: 检测到 H 标识, 开始伺服下降"
            )
            self._land_phase = self._LAND_PHASE_SERVO
            self._land_servo_converged = 0
            return

        timeout = self.get_param("landing_search_timeout", 15.0)
        if elapsed > timeout:
            self.get_logger().error("PRECISION_LAND: H 标识搜索超时")
            self._request_px4_land()
            self._transition(MissionState.COMPLETE)

    # ---- Phase 2: 像素空间 P 控制伺服下降 ----

    def _land_phase_servo(self, has_target: bool):
        """图像空间 P 控制器: 像素误差 → 机体速度, 对心下降."""
        pos = self._bridge.get_current_position()
        altitude = abs(self._fc.ned_to_altitude(pos.down))

        if not has_target or self._landing_target is None:
            # 目标丢失 → 悬停等待恢复
            self._fc.set_velocity_body(0.0, 0.0, 0.0)
            self._land_servo_converged = 0
        else:
            u = self._landing_target.get("center_u", 0.0)
            v = self._landing_target.get("center_v", 0.0)
            conf = self._landing_target.get("confidence", 0)

            # 像素误差
            img_cx = float(self.get_param("camera_cx", 320.0))
            img_cy = float(self.get_param("camera_cy", 240.0))
            du = u - img_cx
            dv = v - img_cy
            pixel_error = math.hypot(du, dv)

            # P 控制
            kp = float(self.get_param("landing_servo_kp_pixel", 0.004))
            max_vel = float(self.get_param("landing_servo_max_horizontal_vel", 0.6))
            deadzone = float(self.get_param("landing_servo_deadzone_px", 12.0))

            if pixel_error < deadzone:
                forward = 0.0
                right = 0.0
                self._land_servo_converged += 1
            else:
                x_sign = float(self.get_param("landing_servo_x_to_right_sign", 1.0))
                y_sign = float(self.get_param("landing_servo_y_to_forward_sign", -1.0))
                forward = max(-max_vel, min(max_vel, y_sign * kp * dv))
                right = max(-max_vel, min(max_vel, x_sign * kp * du))
                self._land_servo_converged = 0

            # 高度分层下降速度
            if altitude > 3.0:
                descent = float(self.get_param("landing_descent_fast", 0.4))
            elif altitude > 1.5:
                descent = float(self.get_param("landing_descent_medium", 0.25))
            else:
                descent = float(self.get_param("landing_descent_slow", 0.12))

            self._fc.set_velocity_body(forward, right, descent)

            # 1Hz 日志
            now = time.time()
            if now - self._land_servo_last_log > 1.0:
                self._land_servo_last_log = now
                self.get_logger().info(
                    f"PRECISION_LAND: alt={altitude:.1f}m "
                    f"px_err={pixel_error:.0f}px du={du:+.0f} dv={dv:+.0f} "
                    f"vel=(fwd:{forward:.2f},r:{right:.2f},d:{descent:.2f}) "
                    f"conf={conf:.2f} converged={self._land_servo_converged}"
                )

        # 到达盲降高度 → Phase 3
        blind_alt = float(self.get_param("landing_blind_altitude", 0.5))
        if altitude < blind_alt:
            self.get_logger().info(
                f"PRECISION_LAND: 到达盲降高度 ({altitude:.2f}m)"
            )
            self._switch_to_blind()

    # ---- Phase 3: 近地盲降 ----

    def _land_phase_blind(self, elapsed: float):
        """近地低速下降, 等待 PX4 着陆检测."""
        self._fc.set_velocity_body(0.0, 0.0, 0.15)

        if elapsed > 60.0:
            self._request_px4_land()

    # ================================================================
    # 状态转移
    # ================================================================

    def _transition(self, state: MissionState):
        old = self._state

        # ---- 退出 LAND → 关闭视觉 ----
        if old == MissionState.LAND:
            self._set_vision_mode("none")

        self._state = state
        self._state_start = time.time()
        self._command_sent = False
        self._stable_count = 0
        self._last_progress_log = 0.0

        if state == MissionState.RECON_SWEEP:
            self._recon_wp_idx = 0
            self._recon_wp_stable = 0

        # ---- 进入 LAND → 初始化精准降落 ----
        if state == MissionState.LAND:
            self._reset_landing_state()
            self.get_logger().info(
                "LAND: 开始 H 标识精准降落"
                f" (search={self.get_param('landing_search_timeout', 15.0)}s"
                f" blind_alt={self.get_param('landing_blind_altitude', 0.5)}m)"
            )
            self._set_vision_mode("landing")
            self._fc.loiter()

        self.get_logger().info(f"[FSM] {old.value} → {state.value}")


# ============================================================
# 入口
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = TestForwardRecon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
