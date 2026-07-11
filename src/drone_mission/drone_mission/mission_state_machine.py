"""
任务状态机 — FSM 驱动的自主飞行编排.

核心职责:
  1. 管理全状态的生命周期 (enter → execute → exit)
  2. 协调 vision / servo / px4_control 各层
  3. 异常处理: 超时重试、低电量返航、视觉丢失盲降
  4. 记录完整任务日志

状态:
  INIT → ARM → TAKEOFF → TRANSIT_TO_DROP_ZONE → CYLINDER_SEARCH
  → SELECT_TARGET → CYLINDER_APPROACH → DESCEND_TO_DROP
  → EXECUTE_DROP → ASCEND → (drops<2: 回到 SELECT_TARGET)
  → TRANSIT_TO_RECON_ZONE → RECON_PATROL (全覆盖弓字形巡视)
  → RETURN_TO_HOME → PRECISION_LAND → COMPLETE

注意: 侦察区仅执行全覆盖路径巡视, 不进行危险品识别分类.

运行频率: 20Hz (0.05s 控制周期)
"""

import time
from enum import Enum
from typing import Any, Dict, List, Optional, Type

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

# 状态处理器
from drone_mission.states.state_init import InitState
from drone_mission.states.state_takeoff import TakeoffState
from drone_mission.states.state_transit import TransitState
from drone_mission.states.state_cylinder_search import CylinderSearchState
from drone_mission.states.state_cylinder_approach import CylinderApproachState
from drone_mission.states.state_drop import DropState
from drone_mission.states.state_recon_patrol import ReconPatrolState
from drone_mission.states.state_return import ReturnState
from drone_mission.states.state_precision_land import PrecisionLandState

# 依赖层
from px4_control.px4_bridge import PX4Bridge
from px4_control.flight_controller import FlightController
from px4_control.telemetry import Position, ned_to_world, world_to_ned

from drone_mission.cylinder_selector import CylinderSelector
from drone_mission.path_planner import PathPlanner

from drone_utils.logger import TaskLogger
from drone_utils.watchdog import Watchdog
from drone_utils.emergency import EmergencyHandler, EmergencyLevel


# ============================================================
# 状态枚举
# ============================================================

class MissionState(Enum):
    INIT = "INIT"
    ARM = "ARM"
    TAKEOFF = "TAKEOFF"
    TRANSIT_TO_DROP_ZONE = "TRANSIT_TO_DROP_ZONE"
    CYLINDER_SEARCH = "CYLINDER_SEARCH"
    SELECT_TARGET = "SELECT_TARGET"
    CYLINDER_APPROACH = "CYLINDER_APPROACH"
    DESCEND_TO_DROP = "DESCEND_TO_DROP"
    EXECUTE_DROP = "EXECUTE_DROP"
    ASCEND = "ASCEND"
    TRANSIT_TO_RECON_ZONE = "TRANSIT_TO_RECON_ZONE"
    RECON_PATROL = "RECON_PATROL"
    RETURN_TO_HOME = "RETURN_TO_HOME"
    PRECISION_LAND = "PRECISION_LAND"
    COMPLETE = "COMPLETE"
    ERROR = "ERROR"


# ============================================================
# 状态转移表
# ============================================================

STATE_TRANSITIONS: Dict[MissionState, MissionState] = {
    MissionState.INIT: MissionState.ARM,
    MissionState.ARM: MissionState.TAKEOFF,
    MissionState.TAKEOFF: MissionState.TRANSIT_TO_DROP_ZONE,
    MissionState.TRANSIT_TO_DROP_ZONE: MissionState.CYLINDER_SEARCH,
    MissionState.CYLINDER_SEARCH: MissionState.SELECT_TARGET,
    MissionState.SELECT_TARGET: MissionState.CYLINDER_APPROACH,
    MissionState.CYLINDER_APPROACH: MissionState.DESCEND_TO_DROP,
    MissionState.DESCEND_TO_DROP: MissionState.EXECUTE_DROP,
    MissionState.EXECUTE_DROP: MissionState.ASCEND,
    # ASCEND → SELECT_TARGET (if drops < 2) or TRANSIT_TO_RECON_ZONE
    MissionState.TRANSIT_TO_RECON_ZONE: MissionState.RECON_PATROL,
    MissionState.RECON_PATROL: MissionState.RETURN_TO_HOME,
    MissionState.RETURN_TO_HOME: MissionState.PRECISION_LAND,
    MissionState.PRECISION_LAND: MissionState.COMPLETE,
}


# ============================================================
# 主状态机
# ============================================================

class MissionFSM(Node):
    """
    无人机自主任务状态机 — ROS2 Node.

    整合 px4_control / drone_vision / drone_servo 三层,
    以 20Hz 频率驱动状态执行.
    """

    # 投放区坐标 (World)
    DROP_ZONE = (42.5, 4.0, 5.0)   # (x, y, z) — 投放区中心
    # 侦察区坐标 (World)
    RECON_ZONE = (67.5, 4.0, 5.0)  # (x, y, z) — 侦察区中心

    def __init__(self):
        super().__init__("mission_state_machine")

        self._declare_params()

        # ---- 核心组件 ----
        self._bridge: Optional[PX4Bridge] = None
        self._fc: Optional[FlightController] = None
        self._vision_node = None  # VisionPipelineNode 引用 (外部注入)
        # Facade caches (created lazily in standalone mode)
        self._standalone_bridge_facade = None
        self._standalone_flight_facade = None

        # ---- 状态机 ----
        self._current_state: MissionState = MissionState.INIT
        self._prev_state: Optional[MissionState] = None
        self._state_handlers: Dict[MissionState, Any] = {}
        self._state_enter_time: float = 0.0

        # ---- 策略 ----
        self._cylinder_selector = CylinderSelector(min_radius_first=True)
        self._path_planner: Optional[PathPlanner] = None

        # ---- 工具 ----
        self._task_logger = TaskLogger(node_name="mission")
        self._task_logger.set_ros_logger(self.get_logger())
        self._watchdog = Watchdog()
        self._emergency = EmergencyHandler(
            battery_warning=self.get_param("battery_warning", 0.25),
            battery_critical=self.get_param("battery_critical", 0.15),
            vision_loss_timeout=self.get_param("vision_loss_timeout", 3.0),
            state_max_duration=self.get_param("state_max_duration", 180.0),
        )

        # ---- 任务统计 ----
        self._drop_count: int = 0
        self._drop_success_count: int = 0
        self._current_drop_cyl_id: int = -1
        self._drop_future = None
        self._drop_pending_cylinder_id: int = -1
        self._drop_result_consumed: bool = True

        # ---- 视觉数据缓存 ----
        self._cylinders: List[Any] = []
        self._markers: List[Any] = []
        self._known_cylinders: List[Dict[str, Any]] = []
        self._known_markers: List[Dict[str, Any]] = []
        self._next_known_cylinder_id: int = 1
        self._next_known_marker_id: int = 1001
        self._hazards: List[Any] = []
        self._landing_target: Optional[Dict] = None
        self._coverage_status: Optional[Dict] = None
        self._cylinders_timestamp: float = 0.0
        self._markers_timestamp: float = 0.0
        self._hazards_timestamp: float = 0.0

        # ---- 遥测缓存 (standalone mode, 通过 ROS2 话题获取) ----
        self._standalone_pose = None  # (x, y, z, qx, qy, qz, qw)
        self._standalone_odom_ready = False
        self._standalone_armed = False
        self._standalone_landed = True
        self._standalone_battery = 1.0

        # ---- 初始化延迟 ----
        self._init_timer = self.create_timer(1.0, self._lazy_init)
        self._initialized = False

        self.get_logger().info("=" * 60)
        self.get_logger().info("  无人机自主任务状态机 v2.0 已启动")
        self.get_logger().info("=" * 60)

    def _declare_params(self):
        """声明所有 YAML 参数."""
        # 全局
        self.declare_parameter("takeoff_height", 5.0)
        self.declare_parameter("waypoint_tolerance", 0.5)
        self.declare_parameter("takeoff_tolerance", 0.3)
        self.declare_parameter("control_frequency", 20.0)
        self.declare_parameter("max_error_count", 5)
        self.declare_parameter("waypoint_timeout", 120.0)
        self.declare_parameter("land_altitude", 0.15)
        # 速度
        self.declare_parameter("sweep_speed", 0.8)
        self.declare_parameter("transit_speed", 2.0)
        self.declare_parameter("return_speed", 3.0)
        self.declare_parameter("max_acceleration", 0.2)
        # Area 1
        self.declare_parameter("area1_north_min", -4.0)
        self.declare_parameter("area1_north_max", 4.0)
        self.declare_parameter("area1_east_min", 30.0)
        self.declare_parameter("area1_east_max", 35.0)
        self.declare_parameter("area1_altitude", 2.0)
        self.declare_parameter("area1_search_altitude", 5.0)
        # Area 2
        self.declare_parameter("area2_north_min", -4.0)
        self.declare_parameter("area2_north_max", 4.0)
        self.declare_parameter("area2_east_min", 55.0)
        self.declare_parameter("area2_east_max", 60.0)
        self.declare_parameter("area2_altitude", 5.0)
        # 覆盖
        self.declare_parameter("grid_spacing", 1.0)
        self.declare_parameter("sweep_passes", 2)
        self.declare_parameter("sweep_margin_north", 1.0)
        self.declare_parameter("sweep_margin_east", 0.5)
        self.declare_parameter("recon_require_full_path", True)
        # 投放
        self.declare_parameter("min_radius_first", True)
        self.declare_parameter("min_drop_targets_required", 2)
        self.declare_parameter("cylinder_memory_merge_distance", 0.75)
        self.declare_parameter("cylinder_memory_min_observations", 1)
        self.declare_parameter("drop_marker_min_confidence", 0.35)
        self.declare_parameter("drop_marker_fallback_delay", 15.0)
        self.declare_parameter("marker_memory_merge_distance", 0.75)
        self.declare_parameter("marker_memory_min_observations", 1)
        self.declare_parameter("drop_descend_height", 2.0)
        self.declare_parameter("drop_confirmation_timeout", 3.0)
        self.declare_parameter("drop_align_pixel_tolerance", 35.0)
        self.declare_parameter("drop_visual_align_probe_step", 0.40)
        self.declare_parameter("drop_visual_align_probe_speed", 0.30)
        self.declare_parameter("drop_visual_align_probe_settle", 1.0)
        self.declare_parameter("drop_visual_align_world_gate", 1.5)
        self.declare_parameter("drop_visual_align_stable_frames", 6)
        self.declare_parameter("drop_visual_align_timeout", 18.0)
        self.declare_parameter("drop_visual_align_brake_threshold", 80.0)
        self.declare_parameter("drop_visual_align_brake_timeout", 4.0)
        self.declare_parameter("drop_visual_align_hover_time", 2.5)
        self.declare_parameter("cylinder_approach_fast_speed", 0.30)
        self.declare_parameter("cylinder_approach_slow_speed", 0.15)
        self.declare_parameter("cylinder_approach_memory_speed", 0.25)
        self.declare_parameter("cylinder_approach_pixel_gain", 0.65)
        self.declare_parameter("cylinder_approach_max_step", 0.45)
        self.declare_parameter("camera_fx", 539.9)
        self.declare_parameter("camera_fy", 539.9)
        self.declare_parameter("camera_cx", 640.0)
        self.declare_parameter("camera_cy", 480.0)
        # 降落
        self.declare_parameter("landing_search_altitude", 5.0)
        self.declare_parameter("landing_approach_speed", 0.5)
        self.declare_parameter("landing_blind_altitude", 0.5)
        self.declare_parameter("landing_precision_target", 0.1)
        # 异常
        self.declare_parameter("battery_warning", 0.25)
        self.declare_parameter("battery_critical", 0.15)
        self.declare_parameter("vision_loss_timeout", 3.0)
        self.declare_parameter("state_max_duration", 180.0)

    # ================================================================
    # 延迟初始化
    # ================================================================

    def _lazy_init(self):
        """等待 PX4 Bridge 就绪后初始化."""
        if self._initialized:
            return

        # 检查 PX4 Bridge 是否可以获取
        # 在真实部署中, PX4Bridge 是独立节点; 此处通过 ros2 话题检查
        # 为简化 integration, 此处创建桥接的引用 (实际需通过 DDS)
        # 在 SITL 场景中, 桥接和 FSM 在不同进程中通过 ROS2 话题通信

        # 我们假设 PX4Bridge 已在另一个进程中运行
        # 此时仅初始化内部组件
        self._path_planner = PathPlanner(
            sweep_speed=self.get_param("sweep_speed", 0.8),
            transit_speed=self.get_param("transit_speed", 2.0),
            return_speed=self.get_param("return_speed", 3.0),
            grid_spacing=self.get_param("grid_spacing", 1.0),
            margin_north=self.get_param("sweep_margin_north", 1.0),
            margin_east=self.get_param("sweep_margin_east", 0.5),
        )

        # 创建状态处理器
        self._state_handlers = {
            MissionState.INIT: InitState(self),
            MissionState.TAKEOFF: TakeoffState(self),
            MissionState.TRANSIT_TO_DROP_ZONE: TransitState(self),
            MissionState.TRANSIT_TO_RECON_ZONE: TransitState(self),
            MissionState.CYLINDER_SEARCH: CylinderSearchState(self),
            MissionState.CYLINDER_APPROACH: CylinderApproachState(self),
            # DropState 管理 DESCEND_TO_DROP / EXECUTE_DROP / ASCEND 三个子状态
            MissionState.DESCEND_TO_DROP: DropState(self),
            MissionState.EXECUTE_DROP: DropState(self),
            MissionState.ASCEND: DropState(self),
            MissionState.RECON_PATROL: ReconPatrolState(self),
            MissionState.RETURN_TO_HOME: ReturnState(self),
            MissionState.PRECISION_LAND: PrecisionLandState(self),
        }

        # 配置 TransitState 的目标
        transit_drop = self._state_handlers.get(MissionState.TRANSIT_TO_DROP_ZONE)
        if isinstance(transit_drop, TransitState):
            transit_drop.configure(
                target_world_xyz=self.DROP_ZONE,
                target_state="CYLINDER_SEARCH",
                label="TRANSIT_TO_DROP_ZONE",
            )
        transit_recon = self._state_handlers.get(MissionState.TRANSIT_TO_RECON_ZONE)
        if isinstance(transit_recon, TransitState):
            transit_recon.configure(
                target_world_xyz=self.RECON_ZONE,
                target_state="RECON_PATROL",
                label="TRANSIT_TO_RECON_ZONE",
            )

        # 配置 DropState 子状态
        for drop_state_name in [MissionState.DESCEND_TO_DROP,
                                 MissionState.EXECUTE_DROP,
                                 MissionState.ASCEND]:
            handler = self._state_handlers.get(drop_state_name)
            if isinstance(handler, DropState):
                handler.set_sub_state(drop_state_name.value)

        # 注册看门狗通道
        self._watchdog.register("mission_fsm", timeout=5.0,
                                on_timeout=lambda n: self.get_logger().warn(f"{n} 无心跳"))
        self._watchdog.register("vision", timeout=5.0)
        self._watchdog.register("servo", timeout=10.0)

        # 启动控制循环
        freq = self.get_param("control_frequency", 20.0)
        self._ctrl_timer = self.create_timer(1.0 / freq, self._control_loop)

        self._initialized = True
        self._init_timer.cancel()

        # 初始化视觉数据订阅
        self._init_vision_subscriptions()

        self.get_logger().info("Mission FSM 初始化完成, 开始任务")
        self._transition_to(MissionState.INIT)

    def _init_vision_subscriptions(self):
        """订阅视觉检测话题 + 遥测 (standalone mode)."""
        try:
            from drone_interfaces.msg import CylinderArray, HazardArray, LandingTarget, CoverageStatus

            self._cyl_sub = self.create_subscription(
                CylinderArray, "/drone/vision/cylinders", self._cyl_cb, 10,
            )
            self._marker_sub = self.create_subscription(
                CylinderArray, "/drone/vision/markers", self._marker_cb, 10,
            )
            self._hazard_sub = self.create_subscription(
                HazardArray, "/drone/vision/hazards", self._hazard_cb, 10,
            )
            self._landing_sub = self.create_subscription(
                LandingTarget, "/drone/vision/landing_target", self._landing_target_cb, 10,
            )
            self._coverage_sub = self.create_subscription(
                CoverageStatus, "/drone/vision/coverage", self._coverage_cb, 10,
            )
            self.get_logger().info("已订阅视觉检测话题")
        except Exception as e:
            self.get_logger().warn(f"视觉话题订阅失败 (视觉节点可能未启动): {e}")

        # 订阅 /drone/pose 用于 standalone 模式遥测
        try:
            from geometry_msgs.msg import PoseStamped
            self._pose_sub = self.create_subscription(
                PoseStamped, "/drone/pose", self._pose_cb, 10,
            )
            self.get_logger().info("已订阅 /drone/pose (standalone 遥测)")
        except Exception as e:
            self.get_logger().warn(f"/drone/pose 订阅失败: {e}")

    # ================================================================
    # 视觉数据回调
    # ================================================================

    def _cyl_cb(self, msg):
        self._cylinders = list(msg.cylinders)
        self._cylinders_timestamp = time.time()
        if self._current_state == MissionState.CYLINDER_SEARCH:
            self._remember_cylinders(self._cylinders)
        self._watchdog.feed("vision")
        self._emergency.update_vision(len(self._cylinders) > 0)

    def _marker_cb(self, msg):
        self._markers = list(msg.cylinders)
        self._markers_timestamp = time.time()
        if self._current_state == MissionState.CYLINDER_SEARCH:
            self._remember_markers(self._markers)
        self._watchdog.feed("vision")

    @staticmethod
    def _cyl_field(cyl, key: str, default=0.0):
        return getattr(cyl, key, cyl.get(key, default) if isinstance(cyl, dict) else default)

    def _remember_cylinders(self, cylinders: list) -> None:
        """Accumulate drop-zone cylinder positions across the search scan."""
        min_conf = 0.35
        merge_dist = float(self.get_param("cylinder_memory_merge_distance", 0.75))
        now = time.time()

        for cyl in cylinders:
            conf = float(self._cyl_field(cyl, "confidence", 0.0))
            if conf < min_conf:
                continue

            raw_x = float(self._cyl_field(cyl, "center_x", 0.0))
            raw_y = float(self._cyl_field(cyl, "center_y", 0.0))
            target_xy = self._normalize_drop_target_xy(raw_x, raw_y)
            if target_xy is None:
                continue
            cx, cy = target_xy
            radius = float(self._cyl_field(cyl, "radius", 0.0))

            best = None
            best_dist = float("inf")
            for known in self._known_cylinders:
                dist = ((known["center_x"] - cx) ** 2 + (known["center_y"] - cy) ** 2) ** 0.5
                if dist < best_dist:
                    best = known
                    best_dist = dist

            if best is not None and best_dist <= merge_dist:
                obs = best["observations"]
                weight = min(conf, 1.0)
                total = obs + weight
                best["center_x"] = (best["center_x"] * obs + cx * weight) / total
                best["center_y"] = (best["center_y"] * obs + cy * weight) / total
                best["radius"] = (best["radius"] * obs + radius * weight) / total
                best["confidence"] = max(best["confidence"], conf)
                best["observations"] = obs + 1
                best["last_seen"] = now
            else:
                self._known_cylinders.append({
                    "id": self._next_known_cylinder_id,
                    "center_x": cx,
                    "center_y": cy,
                    "radius": radius,
                    "confidence": conf,
                    "observations": 1,
                    "last_seen": now,
                })
                self._next_known_cylinder_id += 1

    def _remember_markers(self, markers: list) -> None:
        """Accumulate blue drop-marker positions as fallback drop targets."""
        min_conf = float(self.get_param("drop_marker_min_confidence", 0.35))
        merge_dist = float(self.get_param("marker_memory_merge_distance", 0.75))
        now = time.time()

        for marker in markers:
            conf = float(self._cyl_field(marker, "confidence", 0.0))
            if conf < min_conf:
                continue

            raw_x = float(self._cyl_field(marker, "center_x", 0.0))
            raw_y = float(self._cyl_field(marker, "center_y", 0.0))
            target_xy = self._normalize_drop_target_xy(raw_x, raw_y)
            if target_xy is None:
                continue
            mx, my = target_xy

            best = None
            best_dist = float("inf")
            for known in self._known_markers:
                dist = ((known["center_x"] - mx) ** 2 + (known["center_y"] - my) ** 2) ** 0.5
                if dist < best_dist:
                    best = known
                    best_dist = dist

            if best is not None and best_dist <= merge_dist:
                obs = best["observations"]
                weight = min(conf, 1.0)
                total = obs + weight
                best["center_x"] = (best["center_x"] * obs + mx * weight) / total
                best["center_y"] = (best["center_y"] * obs + my * weight) / total
                best["confidence"] = max(best["confidence"], conf)
                best["observations"] = obs + 1
                best["last_seen"] = now
            else:
                self._known_markers.append({
                    "id": self._next_known_marker_id,
                    "center_x": mx,
                    "center_y": my,
                    "radius": 0.5,
                    "confidence": conf,
                    "observations": 1,
                    "last_seen": now,
                    "source": "marker",
                })
                self._next_known_marker_id += 1

    def _drop_zone_world_bounds(self, margin: float = 1.0):
        n_min = float(self.get_param("area1_north_min", -4.0))
        n_max = float(self.get_param("area1_north_max", 4.0))
        e_min = float(self.get_param("area1_east_min", 30.0))
        e_max = float(self.get_param("area1_east_max", 35.0))
        corners = [
            ned_to_world(n_min, e_min, 0.0),
            ned_to_world(n_min, e_max, 0.0),
            ned_to_world(n_max, e_min, 0.0),
            ned_to_world(n_max, e_max, 0.0),
        ]
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin

    def _normalize_drop_target_xy(self, raw_x: float, raw_y: float):
        """
        Return a bounded world XY target for a cylinder detection.

        Some detector outputs can be in world XY, some older nodes publish
        NED east/north, and bad pose projections can jump outside the drop
        area. The mission may use memory targets for descent, so never store
        an out-of-zone coordinate.
        """
        wx_min, wx_max, wy_min, wy_max = self._drop_zone_world_bounds(margin=1.0)
        if wx_min <= raw_x <= wx_max and wy_min <= raw_y <= wy_max:
            return raw_x, raw_y

        n_min = float(self.get_param("area1_north_min", -4.0))
        n_max = float(self.get_param("area1_north_max", 4.0))
        e_min = float(self.get_param("area1_east_min", 30.0))
        e_max = float(self.get_param("area1_east_max", 35.0))
        if e_min - 1.0 <= raw_x <= e_max + 1.0 and n_min - 1.0 <= raw_y <= n_max + 1.0:
            wx, wy, _ = ned_to_world(raw_y, raw_x, 0.0)
            if wx_min <= wx <= wx_max and wy_min <= wy <= wy_max:
                return wx, wy

        self.get_logger().warn(
            f"CYL_SEARCH: 丢弃投放区外圆筒坐标 ({raw_x:.2f}, {raw_y:.2f})"
        )
        return None

    def _hazard_cb(self, msg):
        self._hazards = list(msg.hazards)
        self._hazards_timestamp = time.time()
        self._watchdog.feed("vision")

    def _landing_target_cb(self, msg):
        self._landing_target = {
            "center_u": msg.center_u,
            "center_v": msg.center_v,
            "world_x": msg.world_x,
            "world_y": msg.world_y,
            "confidence": msg.confidence,
            "yaw_correction": msg.yaw_correction,
        }
        self._watchdog.feed("vision")

    def _coverage_cb(self, msg):
        self._coverage_status = {
            "coverage_ratio": msg.coverage_ratio,
            "is_complete": msg.is_complete,
        }

    def _pose_cb(self, msg):
        """Standalone mode: receive /drone/pose from px4_bridge."""
        p = msg.pose.position
        q = msg.pose.orientation
        self._standalone_pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        self._standalone_odom_ready = True
        self._watchdog.feed("px4_odom")

    # ================================================================
    # 主控制循环 (20Hz)
    # ================================================================

    def _control_loop(self):
        """20Hz 状态机主循环."""
        if not self._initialized:
            return

        self._watchdog.feed("mission_fsm")

        # 紧急检查
        emergency_level = self._emergency.check()
        if emergency_level == EmergencyLevel.RTL:
            self.get_logger().warn("紧急: 触发 RTL 返航!")
            self._transition_to(MissionState.RETURN_TO_HOME)
            return
        elif emergency_level == EmergencyLevel.ABORT:
            self.get_logger().error("紧急: 任务中止!")
            self._transition_to(MissionState.COMPLETE)
            return

        # 执行当前状态
        if self._current_state == MissionState.COMPLETE:
            return

        handler = self._state_handlers.get(self._current_state)
        if handler is None:
            self.get_logger().warn(f"无处理器: {self._current_state.value}")
            return

        try:
            next_state_name = handler.execute()
        except Exception as e:
            self.get_logger().error(f"状态执行异常 {self._current_state.value}: {e}")
            self._emergency._error_count += 1
            next_state_name = None

        if next_state_name:
            # 特殊处理: ASCEND 的下一状态取决于投放次数
            if self._current_state == MissionState.ASCEND:
                if next_state_name == "SELECT_TARGET":
                    self._transition_to(MissionState.SELECT_TARGET)
                elif next_state_name == "TRANSIT_TO_RECON_ZONE":
                    self._transition_to(MissionState.TRANSIT_TO_RECON_ZONE)
                else:
                    self._transition_to_str(next_state_name)
            else:
                self._transition_to_str(next_state_name)

    def _transition_to_str(self, state_name: str):
        """从字符串状态名转换为枚举并跳转."""
        try:
            state = MissionState(state_name)
            self._transition_to(state)
        except ValueError:
            self.get_logger().error(f"未知状态: {state_name}")

    def _transition_to(self, state: MissionState):
        """执行状态转换."""
        old_state = self._current_state
        self._prev_state = old_state
        self._current_state = state
        self._state_enter_time = time.time()

        # 退出旧状态
        if old_state in self._state_handlers:
            try:
                self._state_handlers[old_state].exit()
            except Exception as e:
                self.get_logger().warn(f"状态退出异常 {old_state.value}: {e}")

        # 特殊处理: ARM 状态 (从 INIT 自动完成)
        if state == MissionState.ARM:
            self.get_logger().info("ARM: 准备起飞")
            self._transition_to(MissionState.TAKEOFF)
            return

        # 特殊处理: SELECT_TARGET → 直接运行选择逻辑
        if state == MissionState.SELECT_TARGET:
            self._handle_select_target()
            return

        # 进入新状态
        if state in self._state_handlers:
            handler = self._state_handlers[state]
            try:
                handler.enter()
            except Exception as e:
                self.get_logger().error(f"状态进入异常 {state.value}: {e}")
            self._emergency.set_state(state.value)
        else:
            # COMPLETE / ERROR 无处理器
            if state == MissionState.COMPLETE:
                self._handle_complete()
            elif state == MissionState.ERROR:
                self._handle_error()

        self.get_logger().info(
            f"[FSM] {old_state.value} → {state.value}"
        )

    def _handle_select_target(self):
        """处理圆筒选择."""
        targets = self.get_known_drop_targets()
        if not targets:
            self.get_logger().warn("SELECT_TARGET: 暂无可投目标, 返回投放区搜索")
            self._transition_to(MissionState.CYLINDER_SEARCH)
            return

        target_id = self._cylinder_selector.select(targets)
        if target_id is None:
            max_drops = int(self.get_param("max_drops", 2))
            if self._drop_success_count < max_drops:
                self.get_logger().warn("SELECT_TARGET: 候选均被占用, 返回投放区重新搜索")
                self._transition_to(MissionState.CYLINDER_SEARCH)
            else:
                self._transition_to(MissionState.TRANSIT_TO_RECON_ZONE)
            return

        self._current_drop_cyl_id = target_id
        self.get_logger().info(f"SELECT_TARGET: 选中圆筒 ID={target_id}")
        self._task_logger.log("target_selected", {
            "cylinder_id": target_id,
            "drop_number": self._drop_success_count + 1,
        })
        self._transition_to(MissionState.CYLINDER_APPROACH)

    def _handle_complete(self):
        """任务完成处理."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("  任务完成!")
        self.get_logger().info(f"  投放: {self._drop_success_count}/{self._drop_count}")
        coverage = self.get_coverage()
        if coverage:
            self.get_logger().info(f"  侦察覆盖: {coverage.get('coverage_ratio', 0):.1%}")
        self.get_logger().info("=" * 60)
        self._task_logger.write_summary({
            "success": True,
            "drops_completed": self._drop_success_count,
            "drops_total": self._drop_count,
            "coverage_ratio": coverage.get("coverage_ratio", 0) if coverage else 0,
            "final_state": "COMPLETE",
        })
        # 停止控制循环
        if hasattr(self, '_ctrl_timer'):
            self._ctrl_timer.cancel()

    def _handle_error(self):
        """错误处理."""
        self.get_logger().error(f"任务进入 ERROR 状态")
        self._task_logger.log_error("mission_error", "FSM entered ERROR state")
        self._transition_to(MissionState.COMPLETE)

    # ================================================================
    # 面向状态处理器的接口 (Context)
    # ================================================================

    # -- 飞行控制 --
    @property
    def bridge(self):
        """
        PX4Bridge instance (may be None in standalone mode).

        In standalone mode, we use /drone/pose + /fmu/out/* directly.
        Returns a cached minimal facade when bridge is None.
        """
        if self._bridge is not None:
            return self._bridge
        if self._standalone_bridge_facade is None:
            self._standalone_bridge_facade = _StandaloneBridgeFacade(self)
        return self._standalone_bridge_facade

    @bridge.setter
    def bridge(self, b: PX4Bridge):
        self._bridge = b
        self._fc = FlightController(b)

    @property
    def fc(self):
        if self._fc is not None:
            return self._fc
        if self._standalone_flight_facade is None:
            self._standalone_flight_facade = _StandaloneFlightFacade(self)
        return self._standalone_flight_facade

    @property
    def vision_node(self):
        """VisionPipelineNode 实例 (外部注入)."""
        return self._vision_node

    @vision_node.setter
    def vision_node(self, vn):
        self._vision_node = vn

    # -- 工具 --
    @property
    def logger(self) -> TaskLogger:
        return self._task_logger

    @property
    def watchdog(self) -> Watchdog:
        return self._watchdog

    @property
    def emergency(self) -> EmergencyHandler:
        return self._emergency

    @property
    def cylinder_selector(self) -> CylinderSelector:
        return self._cylinder_selector

    @property
    def path_planner(self) -> Optional[PathPlanner]:
        return self._path_planner

    @property
    def current_state(self) -> str:
        return self._current_state.value

    @property
    def drop_count(self) -> int:
        return self._drop_count

    @property
    def drop_success_count(self) -> int:
        return self._drop_success_count

    # -- 参数获取 --
    def get_param(self, key: str, default=None):
        try:
            return self.get_parameter(key).value
        except Exception:
            return default

    # -- 视觉数据获取 --
    def get_cylinders(self) -> list:
        """获取最新的圆筒检测结果 (过去 2s 内)."""
        if time.time() - self._cylinders_timestamp > 2.0:
            return []
        return self._cylinders

    def get_markers(self) -> list:
        """获取最新的蓝色投放区标识检测结果 (过去 2s 内)."""
        if time.time() - self._markers_timestamp > 2.0:
            return []
        return self._markers

    def get_known_cylinders(self) -> list:
        """获取搜索阶段累计的圆筒目标库."""
        min_obs = int(self.get_param("cylinder_memory_min_observations", 1))
        return [
            c for c in self._known_cylinders
            if int(c.get("observations", 0)) >= min_obs
        ]

    def get_known_markers(self) -> list:
        """获取搜索阶段累计的蓝色标识目标库."""
        min_obs = int(self.get_param("marker_memory_min_observations", 1))
        return [
            m for m in self._known_markers
            if int(m.get("observations", 0)) >= min_obs
        ]

    def get_known_drop_targets(self) -> list:
        """
        获取投放候选.

        圆筒目标优先。只有完全没有圆筒候选时，才用蓝色圆形标识兜底。
        """
        cylinders = self.get_known_cylinders()
        if cylinders:
            return cylinders

        markers = self.get_known_markers()
        targets = []
        for marker in markers:
            mx = float(marker.get("center_x", 0.0))
            my = float(marker.get("center_y", 0.0))
            duplicate = any(
                ((float(t.get("center_x", 0.0)) - mx) ** 2
                 + (float(t.get("center_y", 0.0)) - my) ** 2) ** 0.5 < 0.6
                for t in targets
            )
            if not duplicate:
                targets.append(marker)
        return targets

    def clear_known_cylinders(self) -> None:
        """开始/重试投放区搜索前重置视觉目标库, 保留已投位置去重."""
        self._known_cylinders.clear()
        self._known_markers.clear()
        self._next_known_cylinder_id = 1
        self._next_known_marker_id = 1001

    def release_current_drop_target(self) -> None:
        """投放未执行或失败时释放当前选择, 允许重新搜索/重试."""
        self._cylinder_selector.release_last_selection()
        self._current_drop_cyl_id = -1

    def is_in_drop_zone_world(self, world_x: float, world_y: float, margin: float = 0.5) -> bool:
        wx_min, wx_max, wy_min, wy_max = self._drop_zone_world_bounds(margin=margin)
        return wx_min <= world_x <= wx_max and wy_min <= world_y <= wy_max

    def get_hazards(self) -> list:
        """获取最新的危险品检测结果."""
        if time.time() - self._hazards_timestamp > 2.0:
            return []
        return self._hazards

    def get_landing_target(self) -> Optional[Dict]:
        return self._landing_target

    def get_coverage(self) -> Optional[Dict]:
        return self._coverage_status

    # -- 服务调用 --
    def start_drop_service(self, cylinder_id: int) -> None:
        """
        异步触发舵机投放服务.

        不能在控制循环回调里同步 spin 等待同一节点的 service future,
        否则单线程执行器可能无法处理响应, 导致舵机实际成功但 Mission 统计失败.
        """
        try:
            from drone_interfaces.srv import TriggerDrop

            if not hasattr(self, '_drop_client'):
                self._drop_client = self.create_client(
                    TriggerDrop, "/drone/servo/trigger_drop"
                )

            self._drop_pending_cylinder_id = cylinder_id
            self._drop_result_consumed = False

            if not self._drop_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().warn("舵机服务不可用, 模拟投放")
                self._drop_future = None
                return

            req = TriggerDrop.Request()
            req.cylinder_id = cylinder_id
            self._drop_future = self._drop_client.call_async(req)

        except Exception as e:
            self.get_logger().warn(f"投放服务启动失败: {e}, 模拟投放")
            self._drop_future = None
            self._drop_pending_cylinder_id = cylinder_id
            self._drop_result_consumed = False

    def check_drop_service_result(self) -> Optional[bool]:
        """轮询异步投放结果. None 表示仍在等待."""
        if self._drop_result_consumed:
            return None

        # 服务不可用或启动失败时按 SITL 模拟成功处理, 保持任务可继续.
        if self._drop_future is None:
            self._drop_count += 1
            self._drop_success_count += 1
            self._drop_result_consumed = True
            return True

        if not self._drop_future.done():
            return None

        success = False
        try:
            resp = self._drop_future.result()
            success = bool(resp and resp.success)
        except Exception as e:
            self.get_logger().warn(f"投放服务响应异常: {e}")

        self._drop_count += 1
        if success:
            self._drop_success_count += 1

        self._drop_result_consumed = True
        self._drop_future = None
        return success

    def call_drop_service(self, cylinder_id: int) -> bool:
        """
        调用舵机投放服务.

        通过 ROS2 Service Client 调用 /drone/servo/trigger_drop.
        """
        try:
            from drone_interfaces.srv import TriggerDrop

            if not hasattr(self, '_drop_client'):
                self._drop_client = self.create_client(
                    TriggerDrop, "/drone/servo/trigger_drop"
                )

            if not self._drop_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn("舵机服务不可用, 模拟投放")
                self._drop_count += 1
                self._drop_success_count += 1
                return True

            req = TriggerDrop.Request()
            req.cylinder_id = cylinder_id

            future = self._drop_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

            if future.done() and future.result() is not None:
                resp = future.result()
                if resp.success:
                    self._drop_count += 1
                    self._drop_success_count += 1
                else:
                    self._drop_count += 1
                return resp.success
            else:
                self._drop_count += 1
                return False

        except Exception as e:
            self.get_logger().warn(f"投放服务调用失败: {e}")
            # 模拟投放
            self._drop_count += 1
            self._drop_success_count += 1
            return True

    # -- 悬停 --
    def loiter(self, duration: float = 0.0):
        """在当前位悬停."""
        self.fc.loiter(duration)


# ================================================================
# Standalone Bridge Facade (当 PX4Bridge 独立进程时使用)
# ================================================================

class _StandaloneBridgeFacade:
    """
    PX4Bridge-compatible interface used when MissionFSM runs separately.

    It reads telemetry from /drone/pose and PX4 status topics. Commands are
    relayed through px4_bridge so only one node writes /fmu/in/* control topics.
    """

    # MAVLink command constants
    VEHICLE_CMD_DO_SET_MODE = 176
    VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
    VEHICLE_CMD_NAV_TAKEOFF = 22
    NAV_STATE_OFFBOARD = 14

    def __init__(self, fsm: "MissionFSM"):
        self._fsm = fsm
        self._position = Position(0, 0, 0)
        self._current_yaw = 0.0
        self._px4_armed = False
        self._px4_offboard = False
        self._px4_landed = True

        # ---- Publishers ----
        # VehicleCommand 通过 /drone/cmd/vehicle_command_relay 中继,
        # 由 px4_bridge 转发到 /fmu/in/vehicle_command (已验证的 DDS 路径)
        from px4_msgs.msg import VehicleCommand
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._cmd_pub = self._fsm.create_publisher(
            VehicleCommand, "/drone/cmd/vehicle_command_relay", 10,
        )

        # ---- Subscriptions to PX4 telemetry ----
        # PX4 发布 vehicle_status_v4 (不是 vehicle_status)
        from px4_msgs.msg import VehicleStatus, VehicleLandDetected
        self._fsm.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v4", self._status_cb, px4_qos,
        )
        self._fsm.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected", self._land_cb, px4_qos,
        )

    # ---- PX4 telemetry callbacks ----
    def _status_cb(self, msg):
        self._px4_armed = (msg.arming_state == 2)
        self._px4_offboard = (msg.nav_state == self.NAV_STATE_OFFBOARD)

    def _land_cb(self, msg):
        self._px4_landed = msg.landed

    # ---- Publish VehicleCommand helper (借鉴 Task_flight_ws) ----
    def _send_command(self, command, param1=0.0, param2=0.0):
        from px4_msgs.msg import VehicleCommand
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

    def _set_target(self, north, east, down):
        pass  # No-op; FlightFacade publishes TrajectorySetpoint directly

    def get_telemetry(self):
        from px4_control.telemetry import Telemetry, Attitude

        # Use real /drone/pose from px4_bridge if available
        if self._fsm._standalone_pose:
            px, py, pz = self._fsm._standalone_pose[:3]
            n, e, d = world_to_ned(px, py, pz)
            self._position = Position(north=n, east=e, down=d)

        t = Telemetry()
        t.position = self._position
        t.attitude = Attitude()
        t.status.armed = self._px4_armed
        t.status.landed = self._px4_landed
        t.status.battery_remaining = self._fsm._standalone_battery
        return t

    def get_current_position(self):
        self.get_telemetry()
        return self._position

    def get_current_attitude(self):
        return self.get_telemetry().attitude

    def is_armed(self):
        return self._px4_armed

    def is_offboard(self):
        return self._px4_offboard

    def is_landed(self):
        return self._px4_landed

    def set_position_target(self, north, east, down, yaw=0.0):
        """在 standalone 模式, 位置控制由 _StandaloneFlightFacade.goto_ned 处理."""
        self._fsm.get_logger().debug(
            f"BridgeFacade.set_position_target({north:.1f}, {east:.1f}, {down:.1f}) — "
            f"使用 _StandaloneFlightFacade 代替"
        )

    def set_velocity_target(self, vn, ve, vd, yaw_rate=0.0):
        """在 standalone 模式, 速度控制由 _StandaloneFlightFacade.set_velocity_ned 处理."""
        self._fsm.get_logger().debug(
            f"BridgeFacade.set_velocity_target({vn:.1f}, {ve:.1f}, {vd:.1f}) — "
            f"使用 _StandaloneFlightFacade 代替"
        )

    def set_offboard_mode(self):
        self._fsm.get_logger().info("[PX4] 切换 Offboard 模式")
        # DO_SET_MODE(176): param1=1.0, param2=6.0 → OFFBOARD (借鉴 Task_flight_ws)
        self._send_command(176, 1.0, 6.0)

    def arm(self):
        self._fsm.get_logger().info("[PX4] 解锁请求 (SITL force)")
        # COMPONENT_ARM_DISARM(400): param2=21196 → SITL force arm magic
        self._send_command(400, 1.0, 21196.0)
        self._fsm._standalone_armed = True

    def disarm(self):
        self._fsm.get_logger().info("[PX4] 上锁请求")
        self._send_command(400, 0.0, 0.0)
        self._fsm._standalone_armed = False
        self._fsm._standalone_landed = True

    def force_disarm(self):
        self._fsm.get_logger().warn("[PX4] 强制上锁!")
        self._send_command(400, 0.0, 21196.0)
        self._fsm._standalone_armed = False

    def get_parameter(self, name):
        return self._fsm.get_parameter(name)

    def get_logger(self):
        return self._fsm.get_logger()

    def get_clock(self):
        return self._fsm.get_clock()


class _StandaloneFlightFacade:
    """
    FlightController-compatible interface that publishes target setpoints
    to /drone/cmd_setpoint (picked up by px4_bridge for PX4 forwarding).
    """

    def __init__(self, fsm: "MissionFSM"):
        self._fsm = fsm
        self._target_ned = None  # (north, east, down)
        self._target_set_time = 0.0
        self._loiter_start = 0.0
        self._loiter_duration = 0.0
        self._landing = False
        self._waypoint_timeout = 120.0
        self._waypoint_tol = 0.5
        self._takeoff_tol = 0.3
        self._velocity_mode = False
        self._vel_target = (0.0, 0.0, 0.0)  # (vn, ve, vd)

        # ---- Publish target setpoint for px4_bridge to forward ----
        from geometry_msgs.msg import Point, Twist
        self._cmd_pub = self._fsm.create_publisher(
            Point, "/drone/cmd_setpoint_ned", 10,
        )
        self._vel_pub = self._fsm.create_publisher(
            Twist, "/drone/cmd_velocity_ned", 10,
        )
        # Keep publishing at 10Hz
        self._pub_timer = self._fsm.create_timer(0.1, self._publish_cmd)

    def _publish_cmd(self):
        """Publish current NED target or velocity for px4_bridge at 10Hz."""
        if self._velocity_mode:
            from geometry_msgs.msg import Twist
            msg = Twist()
            msg.linear.x = float(self._vel_target[0])   # vn
            msg.linear.y = float(self._vel_target[1])   # ve
            msg.linear.z = float(self._vel_target[2])   # vd
            self._vel_pub.publish(msg)
        else:
            if self._target_ned is None:
                return
            from geometry_msgs.msg import Point
            msg = Point()
            msg.x = float(self._target_ned[0])  # north
            msg.y = float(self._target_ned[1])  # east
            msg.z = float(self._target_ned[2])  # down
            self._cmd_pub.publish(msg)

    def takeoff(self, altitude=None):
        if altitude is None:
            altitude = self._fsm.get_param("takeoff_height", 5.0)
        pos = self._fsm.bridge.get_current_position()
        new_target = (float(pos.north), float(pos.east), -float(altitude))
        if new_target != self._target_ned:
            self._target_ned = new_target
            self._target_set_time = time.time()
            self._fsm.get_logger().info(
                f"[PX4] takeoff → NED({self._target_ned[0]:.1f}, "
                f"{self._target_ned[1]:.1f}, {self._target_ned[2]:.1f})"
            )

    def goto_ned(self, north, east, down, yaw=0.0, speed=1.0):
        self._velocity_mode = False  # 切回位置模式
        new_target = (float(north), float(east), float(down))
        if new_target != self._target_ned:
            self._target_ned = new_target
            self._target_set_time = time.time()
            self._fsm.get_logger().info(
                f"[PX4] goto_ned → ({north:.1f}, {east:.1f}, {down:.1f}) @{speed}m/s"
            )

    def goto_world(self, x, y, z, yaw=0.0, speed=1.0):
        n, e, d = world_to_ned(x, y, z)
        self.goto_ned(n, e, d, yaw, speed)

    def loiter(self, duration=0.0):
        self._velocity_mode = False  # 切回位置模式
        # 在当前位置悬停: 读取当前位置并设为 setpoint
        pos = self._fsm.bridge.get_current_position()
        self._target_ned = (pos.north, pos.east, pos.down)
        self._target_set_time = time.time()
        self._loiter_start = time.time()
        self._loiter_duration = duration

    def set_velocity_ned(self, vn, ve, vd, yaw_rate=0.0):
        self._velocity_mode = True
        self._vel_target = (float(vn), float(ve), float(vd))

    def land(self):
        self._fsm.bridge._send_command(21)  # VEHICLE_CMD_NAV_LAND
        self._landing = True
        return True

    def reached(self, target, tolerance=0.0):
        """Check real position against target using /drone/pose."""
        pos = self._fsm.bridge.get_current_position()
        if hasattr(target, '__len__') and len(target) == 3:
            tn, te, td = target
        else:
            # Assume target is already NED and we're checking against _target_ned
            if self._target_ned is None:
                return True
            tn, te, td = self._target_ned
        tol = tolerance if tolerance > 0 else self._waypoint_tol
        dist = ((pos.north - tn)**2 + (pos.east - te)**2 + (pos.down - td)**2)**0.5
        return dist < tol

    def reached_xy(self, target, tolerance=0.0):
        pos = self._fsm.bridge.get_current_position()
        if hasattr(target, '__len__') and len(target) >= 2:
            tn, te = target[0], target[1]
        else:
            if self._target_ned is None:
                return True
            tn, te = self._target_ned[0], self._target_ned[1]
        tol = tolerance if tolerance > 0 else self._waypoint_tol
        dist = ((pos.north - tn)**2 + (pos.east - te)**2)**0.5
        return dist < tol

    def reached_altitude(self, target_down, tolerance=0.0):
        pos = self._fsm.bridge.get_current_position()
        tol = tolerance if tolerance > 0 else self._takeoff_tol
        return abs(pos.down - target_down) < tol

    def get_position(self):
        return self._fsm.bridge.get_current_position()

    def distance_to(self, target):
        pos = self._fsm.bridge.get_current_position()
        if hasattr(target, '__len__') and len(target) >= 2:
            return ((pos.north - target[0])**2 + (pos.east - target[1])**2)**0.5
        return 0.0

    def is_waypoint_timed_out(self):
        return (time.time() - self._target_set_time) > self._waypoint_timeout

    def force_disarm(self):
        self._fsm._standalone_armed = False


# ================================================================
# 入口
# ================================================================

def main(args=None):
    rclpy.init(args=args)
    node = MissionFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
