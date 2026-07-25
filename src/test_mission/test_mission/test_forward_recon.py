"""
分阶段真机实飞测试 — 向前直飞 20m → 侦察区任务.

Phase 1: 起飞 → 向北直飞 20m → 验证航点到达
Phase 2: 降至侦察高度 → 弓字形全覆盖扫描 → 回升 → 返航 → 降落

状态流:
  INIT → WAIT_RC_ARM → TAKEOFF → TRANSIT_FORWARD → DESCEND_TO_RECON
  → RECON_SWEEP → ASCEND_TO_CRUISE → RETURN → LAND → COMPLETE

安全:
  - WAIT_RC_ARM: 起飞前必须确认遥控器已连接且信号正常
  - 遥控器通电 = 操作员已就位; PX4 解锁时还会再次校验
  - 无超时跳过 — 硬性安全要求

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

from px4_msgs.msg import InputRc
from px4_control.standalone_bridge import StandaloneBridge
from px4_control.flight_controller import FlightController
from px4_control.telemetry import configure_home, world_to_ned

from drone_mission.path_planner import PathPlanner


# ============================================================
# 任务状态
# ============================================================

class MissionState(Enum):
    INIT = "INIT"
    WAIT_RC_ARM = "WAIT_RC_ARM"               # 等待遥控器解锁信号 (安全硬性要求)
    TAKEOFF = "TAKEOFF"
    TRANSIT_FORWARD = "TRANSIT_FORWARD"       # Phase 1: 向北直飞 20m
    DESCEND_TO_RECON = "DESCEND_TO_RECON"      # Phase 2: 降至侦察高度
    RECON_SWEEP = "RECON_SWEEP"                # Phase 2: 弓字形全覆盖
    ASCEND_TO_CRUISE = "ASCEND_TO_CRUISE"      # Phase 2: 回升至巡航高度
    RETURN = "RETURN"
    LAND = "LAND"
    COMPLETE = "COMPLETE"


# ============================================================
# TestForwardRecon 节点
# ============================================================

class TestForwardRecon(Node):
    """
    分阶段真机测试 — 向前直飞 20m + 侦察区弓字形全覆盖扫描.

    Phase 1 — 基础飞行验证:
      - 起飞至巡航高度 (5m)
      - 向北水平直飞 20m
      - 到达确认 (位置 + 连续稳定帧数)

    Phase 2 — 侦察区任务:
      - 降至侦察扫描高度 (默认 3m)
      - 弓字形 (Boustrophedon) 全覆盖路径
      - 回升至巡航高度
      - 返航 + 降落
    """

    STABLE_TICKS = 40       # 高度/航点稳定确认帧数 (2s @ 20Hz)
    WP_STABLE_TICKS = 8     # 侦察航点稳定确认帧数 (0.4s @ 20Hz)

    def __init__(self):
        super().__init__("test_forward_recon")

        # ---- 参数声明 ----
        # 通用
        self.declare_parameter("sim_mode", True)
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
        self.declare_parameter("recon_area_half_north", 4.0)   # 侦察区 north 半宽 (m)
        self.declare_parameter("recon_area_half_east", 3.0)    # 侦察区 east 半宽 (m)
        self.declare_parameter("sweep_speed", 0.8)             # 扫描飞行速度 (m/s)
        self.declare_parameter("grid_spacing", 1.0)            # 条带间距 (m)
        self.declare_parameter("sweep_margin_north", 0.5)      # north 方向边距 (m)
        self.declare_parameter("sweep_margin_east", 0.5)       # east 方向边距 (m)

        # 返航
        self.declare_parameter("return_speed", 3.0)

        # 遥控器安全 — 起飞前必须确认 RC 连接正常
        self.declare_parameter("rc_require_arm_signal", True)  # 是否强制等待 RC
        self.declare_parameter("rc_min_channels", 4)           # 最少有效通道数

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
            grid_spacing=self.get_param("grid_spacing", 1.0),
            margin_north=self.get_param("sweep_margin_north", 0.5),
            margin_east=self.get_param("sweep_margin_east", 0.5),
        )

        # ---- 状态机 ----
        self._state: MissionState = MissionState.INIT
        self._state_start: float = time.time()
        self._command_sent: bool = False
        self._stable_count: int = 0

        # ---- 遥控器状态缓存 ----
        self._rc_lost: bool = True          # 默认认为丢失, 直到收到有效消息
        self._rc_failsafe: bool = True
        self._rc_channel_count: int = 0
        self._rc_rssi: int = -1
        self._rc_values: list = []          # 18 通道 PWM 值 (μs)
        self._rc_last_msg_time: float = 0.0

        # ---- RC 状态订阅 ----
        from rclpy.qos import (
            QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
        )
        rc_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._rc_sub = self.create_subscription(
            InputRc, "/fmu/out/input_rc", self._rc_cb, rc_qos,
        )

        # ---- 侦察区航点 ----
        self._recon_waypoints: list = []
        self._recon_wp_idx: int = 0
        self._recon_wp_stable: int = 0

        # ---- 进度日志 ----
        self._last_progress_log: float = 0.0

        # ---- 控制循环 ----
        freq = self.get_param("control_frequency", 20.0)
        self._ctrl_timer = self.create_timer(1.0 / freq, self._control_loop)

        # ---- 启动日志 ----
        fwd = self.get_param("forward_distance", 20.0)
        ralt = self.get_param("recon_altitude", 3.0)
        half_n = self.get_param("recon_area_half_north", 4.0)
        half_e = self.get_param("recon_area_half_east", 3.0)

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
        self.get_logger().info("  流程: INIT → WAIT_RC_ARM → TAKEOFF → TRANSIT_FWD")
        self.get_logger().info("         → DESCEND → RECON_SWEEP → ASCEND → RETURN → LAND")
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

    def _control_loop(self):
        """20Hz FSM 控制循环."""
        if self._state == MissionState.COMPLETE:
            return

        elapsed = time.time() - self._state_start
        tol = self.get_param("waypoint_tolerance", 0.5)
        cruise_alt = self.get_param("cruise_altitude", 5.0)

        # ---- INIT: 等待遥测就绪 → WAIT_RC_ARM ----
        if self._state == MissionState.INIT:
            pos = self._bridge.get_current_position()
            if pos.norm_xy() > 0.01 or abs(pos.down) > 0.01 or elapsed > 2.0:
                self.get_logger().info("INIT: 遥测就绪 → WAIT_RC_ARM")
                self._transition(MissionState.WAIT_RC_ARM)

        # ---- WAIT_RC_ARM: 等待遥控器连接 (安全硬性要求) ----
        elif self._state == MissionState.WAIT_RC_ARM:
            require_rc = self.get_param("rc_require_arm_signal", True)
            if not require_rc:
                self.get_logger().info("WAIT_RC_ARM: RC 检查已禁用 → TAKEOFF")
                self._transition(MissionState.TAKEOFF)
                return

            if self._check_rc_ready():
                self.get_logger().info(
                    f"WAIT_RC_ARM: ✓ RC 连接正常"
                    f" (ch={self._rc_channel_count} rssi={self._rc_rssi})"
                    f" → TAKEOFF"
                )
                self._transition(MissionState.TAKEOFF)
                return

            # 每 3s 输出 RC 状态
            if elapsed - self._last_progress_log > 3.0:
                self._last_progress_log = elapsed
                min_ch = self.get_param("rc_min_channels", 4)
                self.get_logger().info(
                    f"WAIT_RC_ARM: 等待 RC 连接..."
                    f" rc_lost={self._rc_lost}"
                    f" failsafe={self._rc_failsafe}"
                    f" channels={self._rc_channel_count}/{min_ch}"
                    f" rssi={self._rc_rssi}"
                )

        # ---- TAKEOFF: 起飞至巡航高度 ----
        elif self._state == MissionState.TAKEOFF:
            self._fc.takeoff(cruise_alt)
            pos = self._bridge.get_current_position()

            if abs(pos.down - (-cruise_alt)) < self.get_param("takeoff_tolerance", 0.3):
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
                self._fc.goto_world(hx, hy + fwd, cruise_alt, speed=speed)
                self._command_sent = True

            # 到达检测: World → NED 转换
            fwd = self.get_param("forward_distance", 20.0)
            hx = self.get_param("home_x", 0.0)
            hy = self.get_param("home_y", 0.0)
            tn, te, td = world_to_ned(hx, hy + fwd, cruise_alt)

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
                self._fc.goto_world(hx, hy + fwd, ralt, speed=1.0)
                self._command_sent = True

            target_down = -self.get_param("recon_altitude", 3.0)
            pos = self._bridge.get_current_position()

            if abs(pos.down - target_down) < self.get_param("takeoff_tolerance", 0.5):
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
                self.get_logger().warn("RECON_SWEEP: 无航点, 跳过 → ASCEND")
                self._transition(MissionState.ASCEND_TO_CRUISE)
                return

            # 所有航点完成
            if self._recon_wp_idx >= len(self._recon_waypoints):
                self.get_logger().info(
                    f"RECON_SWEEP: ✓ Phase 2 扫描完成"
                    f" — {len(self._recon_waypoints)} 个航点全部到达 → ASCEND"
                )
                self._transition(MissionState.ASCEND_TO_CRUISE)
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
        # Phase 2: ASCEND_TO_CRUISE — 回升至巡航高度
        # ================================================================
        elif self._state == MissionState.ASCEND_TO_CRUISE:
            if not self._command_sent:
                fwd = self.get_param("forward_distance", 20.0)
                hx = self.get_param("home_x", 0.0)
                hy = self.get_param("home_y", 0.0)

                self.get_logger().info(
                    f"ASCEND: 回升至巡航高度 {cruise_alt}m"
                )
                self._fc.goto_world(hx, hy + fwd, cruise_alt, speed=1.0)
                self._command_sent = True

            pos = self._bridge.get_current_position()
            if abs(pos.down - (-cruise_alt)) < self.get_param("takeoff_tolerance", 0.5):
                self._stable_count += 1
            else:
                self._stable_count = max(0, self._stable_count - 1)

            if self._stable_count >= self.STABLE_TICKS:
                self.get_logger().info(
                    f"ASCEND: 回到巡航高度 {cruise_alt}m → RETURN"
                )
                self._transition(MissionState.RETURN)
            elif elapsed > 30.0:
                self.get_logger().warn("ASCEND: 回升超时 (30s), 强制返航")
                self._transition(MissionState.RETURN)

        # ================================================================
        # RETURN: 返航至 Home 点上方
        # ================================================================
        elif self._state == MissionState.RETURN:
            if not self._command_sent:
                hx = self.get_param("home_x", 0.0)
                hy = self.get_param("home_y", 0.0)
                speed = self.get_param("return_speed", 3.0)
                self.get_logger().info(
                    f"RETURN: 返航至 Home World({hx:.1f}, {hy:.1f}, {cruise_alt:.1f})"
                )
                self._fc.goto_world(hx, hy, cruise_alt, speed=speed)
                self._command_sent = True

            hx = self.get_param("home_x", 0.0)
            hy = self.get_param("home_y", 0.0)
            tn, te, td = world_to_ned(hx, hy, cruise_alt)

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
        # LAND: 降落
        # ================================================================
        elif self._state == MissionState.LAND:
            landed = self._fc.land()
            if landed:
                self.get_logger().info("LAND: 着陆完成 → COMPLETE")
                self._transition(MissionState.COMPLETE)
            elif elapsed > 30.0:
                self.get_logger().error("LAND: 着陆超时 (30s), 强制上锁")
                self._fc.force_disarm()
                self._transition(MissionState.COMPLETE)

    # ================================================================
    # 遥控器状态
    # ================================================================

    def _rc_cb(self, msg: InputRc) -> None:
        """缓存最新的 RC 接收机状态."""
        self._rc_lost = msg.rc_lost
        self._rc_failsafe = msg.rc_failsafe
        self._rc_channel_count = msg.channel_count
        self._rc_rssi = msg.rssi
        self._rc_values = list(msg.values)
        self._rc_last_msg_time = time.time()

    def _check_rc_ready(self) -> bool:
        """
        验证遥控器状态是否满足起飞条件.

        条件:
          1. RC 接收机未丢失 (rc_lost=False)
          2. 未触发失控保护 (rc_failsafe=False)
          3. 通道数 >= rc_min_channels (接收机正常工作)
          4. 最近 2s 内收到过 RC 消息 (数据新鲜)

        遥控器通电且信号正常 = 操作员已就位确认.
        PX4 自身也会在解锁时再次校验 RC 状态.
        """
        if self._rc_lost:
            return False
        if self._rc_failsafe:
            return False
        if self._rc_channel_count < self.get_param("rc_min_channels", 4):
            return False
        if time.time() - self._rc_last_msg_time > 2.0:
            return False
        return True

    # ================================================================
    # 侦察区航点生成
    # ================================================================

    def _generate_recon_waypoints(self):
        """生成侦察区弓字形全覆盖航点 (NED 坐标系)."""
        fwd = self.get_param("forward_distance", 20.0)
        half_n = self.get_param("recon_area_half_north", 4.0)
        half_e = self.get_param("recon_area_half_east", 3.0)
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
            altitude=-ralt,
        )

        num_strips = len(self._recon_waypoints) // 2
        self.get_logger().info(
            f"RECON_SWEEP: 生成 {len(self._recon_waypoints)} 个航点"
            f" ({num_strips} 条带),"
            f" 区域 NED N[{area_north_min:.1f}:{area_north_max:.1f}]"
            f" E[{area_east_min:.1f}:{area_east_max:.1f}],"
            f" 高度 {ralt:.1f}m,"
            f" 间距 {self.get_param('grid_spacing', 1.0)}m"
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
    # 状态转移
    # ================================================================

    def _transition(self, state: MissionState):
        old = self._state
        self._state = state
        self._state_start = time.time()
        self._command_sent = False
        self._stable_count = 0
        self._last_progress_log = 0.0

        if state == MissionState.RECON_SWEEP:
            self._recon_wp_idx = 0
            self._recon_wp_stable = 0

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
