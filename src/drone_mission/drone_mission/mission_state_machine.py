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
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
from px4_control.standalone_bridge import StandaloneBridge
from px4_control.telemetry import configure_home, ned_to_world

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

    # 投放区/侦察区 NED 相对偏移 (从 Home 点出发, 仿真/真机一致)
    #  投放区: Home 东 32.5m, 北 0m
    #  侦察区: Home 东 57.5m, 北 0m
    DROP_NED_OFFSET = (0.0, 32.5)    # (north, east)
    RECON_NED_OFFSET = (0.0, 57.5)   # (north, east)
    TARGET_ALTITUDE = 5.0            # 巡航高度 (m)

    def __init__(self):
        super().__init__("mission_state_machine")

        self._declare_params()

        # ---- 核心组件 ----
        self._bridge: Optional[PX4Bridge] = None
        self._fc: Optional[FlightController] = None
        self._vision_node = None  # VisionPipelineNode 引用 (外部注入)
        # Facade caches (created lazily in standalone mode)
        self._standalone_bridge: Optional[StandaloneBridge] = None
        self._standalone_fc: Optional[FlightController] = None

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
        self._servo_ready: bool = False
        self._vision_ready: bool = False
        self._mission_failed: bool = False

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

        self._standalone_odom_ready = False

        # ---- 初始化延迟 ----
        self._init_timer = self.create_timer(1.0, self._lazy_init)
        self._initialized = False

        self.get_logger().info("=" * 60)
        self.get_logger().info("  无人机自主任务状态机 v2.0 已启动")
        self.get_logger().info("=" * 60)

    def _declare_params(self):
        """声明所有 YAML 参数."""
        # 全局
        self.declare_parameter("sim_mode", True)
        self.declare_parameter("home_x", 10.0)
        self.declare_parameter("home_y", 4.0)
        self.declare_parameter("home_z", 0.0)
        self.declare_parameter("takeoff_height", 5.0)
        self.declare_parameter("waypoint_tolerance", 0.5)
        self.declare_parameter("takeoff_tolerance", 0.3)
        self.declare_parameter("control_frequency", 20.0)
        self.declare_parameter("max_error_count", 5)
        self.declare_parameter("waypoint_timeout", 120.0)
        self.declare_parameter("land_altitude", 0.15)
        self.declare_parameter("max_drops", 2)
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
        self.declare_parameter("marker_memory_min_observations", 3)
        self.declare_parameter("drop_descend_height", 2.0)
        self.declare_parameter("drop_confirmation_timeout", 5.0)
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
        self.declare_parameter("drop_visual_align_lock_tolerance", 2.0)
        self.declare_parameter("precision_align_enable", True)
        self.declare_parameter("precision_align_target_source", "cylinder")
        self.declare_parameter("precision_align_image_width", 1280.0)
        self.declare_parameter("precision_align_image_height", 960.0)
        self.declare_parameter("precision_align_alpha", 0.55)
        self.declare_parameter("precision_align_gain", 1.25)
        self.declare_parameter("precision_align_max_speed", 0.25)
        self.declare_parameter("precision_align_fine_pixel_error", 90.0)
        self.declare_parameter("precision_align_fine_max_speed", 0.065)
        self.declare_parameter("precision_align_min_correction_speed", 0.012)
        self.declare_parameter("precision_align_step_s", 0.18)
        self.declare_parameter("precision_align_fine_step_s", 0.12)
        self.declare_parameter("precision_align_hover_s", 0.04)
        self.declare_parameter("precision_align_timeout", 25.0)
        self.declare_parameter("precision_align_max_retries", 2)
        self.declare_parameter("precision_align_image_x_to_right_sign", 1.0)
        self.declare_parameter("precision_align_image_y_to_forward_sign", -1.0)
        self.declare_parameter("precision_align_threshold_norm_x", 0.008)
        self.declare_parameter("precision_align_threshold_norm_y", 0.008)
        self.declare_parameter("precision_align_raw_pixel_tolerance", 9.0)
        self.declare_parameter("precision_align_smooth_pixel_tolerance", 7.0)
        self.declare_parameter("precision_align_stable_frames", 5)
        self.declare_parameter("precision_align_final_brake_s", 0.5)
        self.declare_parameter("precision_align_release_max_horizontal_speed", 0.04)
        self.declare_parameter("precision_align_min_confidence", 0.25)
        self.declare_parameter("precision_align_target_world_gate", 1.5)
        self.declare_parameter("precision_align_max_jump_diag_ratio", 0.55)
        self.declare_parameter("precision_align_reject_center_ratio", 0.28)
        self.declare_parameter("precision_align_reacquire_after_lost", 12)
        self.declare_parameter("precision_align_max_lost_frames", 48)
        self.declare_parameter("precision_align_drop_height_tolerance", 0.50)
        self.declare_parameter("precision_align_final_descent_enable", True)
        self.declare_parameter("precision_align_final_drop_height", 1.25)
        self.declare_parameter("precision_align_final_descent_speed", 0.25)
        self.declare_parameter("precision_align_log_interval", 0.75)
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

        hx = self.get_param("home_x", 10.0)
        hy = self.get_param("home_y", 4.0)
        hz = self.get_param("home_z", 0.0)
        configure_home(hx, hy, hz)

        # 从 Home + NED 偏移计算投放区/侦察区的 World 坐标
        # NED east → World x, NED north → World y（仿真/真机相对关系一致）
        drop_world_x = hx + self.DROP_NED_OFFSET[1]
        drop_world_y = hy + self.DROP_NED_OFFSET[0]
        recon_world_x = hx + self.RECON_NED_OFFSET[1]
        recon_world_y = hy + self.RECON_NED_OFFSET[0]
        drop_zone = (drop_world_x, drop_world_y, self.TARGET_ALTITUDE)
        recon_zone = (recon_world_x, recon_world_y, self.TARGET_ALTITUDE)

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
                target_world_xyz=drop_zone,
                target_state="CYLINDER_SEARCH",
                label="TRANSIT_TO_DROP_ZONE",
            )
        transit_recon = self._state_handlers.get(MissionState.TRANSIT_TO_RECON_ZONE)
        if isinstance(transit_recon, TransitState):
            transit_recon.configure(
                target_world_xyz=recon_zone,
                target_state="RECON_PATROL",
                label="TRANSIT_TO_RECON_ZONE",
            )

        # 配置 DropState 子状态
        for drop_state_name in [
            MissionState.DESCEND_TO_DROP,
            MissionState.EXECUTE_DROP,
            MissionState.ASCEND,
        ]:
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
        # INIT 依赖 PX4 电池/状态消息，必须先创建 standalone 遥测订阅。
        _ = self.bridge

        self.get_logger().info("Mission FSM 初始化完成, 开始任务")
        self._transition_to(MissionState.INIT)

    def _init_vision_subscriptions(self):
        """订阅视觉检测话题 + 遥测 (standalone mode)."""
        try:
            from drone_interfaces.msg import (
                CoverageStatus,
                CylinderArray,
                HazardArray,
                LandingTarget,
            )

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

        self._servo_status_sub = self.create_subscription(
            String, "/drone/servo/status", self._servo_status_cb, 10,
        )
        self._vision_status_sub = self.create_subscription(
            String, "/drone/vision/status", self._vision_status_cb, 10,
        )

        from drone_interfaces.srv import SetVisionMode
        self._vision_mode_client = self.create_client(
            SetVisionMode, "/drone/vision/set_mode",
        )

    def _servo_status_cb(self, msg: String):
        """记录舵机节点心跳和硬件就绪状态."""
        self._watchdog.feed("servo")
        self._servo_ready = "ready=1" in msg.data

    def _vision_status_cb(self, msg: String):
        """Accept vision heartbeat only after models and a camera frame are ready."""
        self._vision_ready = "ready=1" in msg.data
        if self._vision_ready:
            self._watchdog.feed("vision")

    # ================================================================
    # 视觉数据回调
    # ================================================================

    def _cyl_cb(self, msg):
        self._cylinders = list(msg.cylinders)
        self._cylinders_timestamp = time.time()
        if self._current_state == MissionState.CYLINDER_SEARCH:
            self._remember_cylinders(self._cylinders)
        self._emergency.update_vision(len(self._cylinders) > 0)

    def _marker_cb(self, msg):
        self._markers = list(msg.cylinders)
        self._markers_timestamp = time.time()
        if self._current_state == MissionState.CYLINDER_SEARCH:
            self._remember_markers(self._markers)

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

        if self.bridge.battery_ready:
            self._emergency.update_battery(self.bridge.battery_remaining)

        # 紧急检查
        emergency_level = self._emergency.check()
        if emergency_level == EmergencyLevel.RTL:
            self.get_logger().warn("紧急: 触发 RTL 返航!")
            self._transition_to(MissionState.RETURN_TO_HOME)
            return
        elif emergency_level == EmergencyLevel.ABORT:
            self.get_logger().error("紧急: 任务中止!")
            self._transition_to(MissionState.ERROR)
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
            self._transition_to(MissionState.ERROR)
            return

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
                self._transition_to(MissionState.ERROR)
                return
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
        success = not self._mission_failed
        self.get_logger().info("=" * 60)
        self.get_logger().info("  任务完成" if success else "  任务失败")
        self.get_logger().info(f"  投放: {self._drop_success_count}/{self._drop_count}")
        coverage = self.get_coverage()
        if coverage:
            self.get_logger().info(f"  侦察覆盖: {coverage.get('coverage_ratio', 0):.1%}")
        self.get_logger().info("=" * 60)
        self._task_logger.write_summary({
            "success": success,
            "drops_completed": self._drop_success_count,
            "drops_total": self._drop_count,
            "coverage_ratio": coverage.get("coverage_ratio", 0) if coverage else 0,
            "final_state": "COMPLETE" if success else "ERROR",
        })
        # 停止控制循环
        if hasattr(self, '_ctrl_timer'):
            self._ctrl_timer.cancel()

    def _handle_error(self):
        """错误处理."""
        self._mission_failed = True
        self.get_logger().error("任务进入 ERROR 状态")
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

        In standalone mode, we use StandaloneBridge which communicates
        through /drone/cmd/* relay topics instead of direct PX4 DDS.
        """
        if self._bridge is not None:
            return self._bridge
        if self._standalone_bridge is None:
            self._standalone_bridge = StandaloneBridge(
                self, sim_mode=self.get_param("sim_mode", True)
            )
        return self._standalone_bridge

    @bridge.setter
    def bridge(self, b: PX4Bridge):
        self._bridge = b
        self._fc = FlightController(b)

    @property
    def fc(self):
        if self._fc is not None:
            return self._fc
        if self._standalone_fc is None:
            self._standalone_fc = FlightController(self.bridge)
        return self._standalone_fc

    @property
    def vision_node(self):
        """Return the externally injected vision pipeline node."""
        return self._vision_node

    @vision_node.setter
    def vision_node(self, vn):
        self._vision_node = vn

    # Vision mode mapping (must match vision_pipeline service handler)
    _VISION_MODE_NONE = 0
    _VISION_MODE_LANDING = 2
    _VISION_MODE_ALL = 3

    def set_vision_mode(self, mode: str) -> None:
        """Switch the separately running vision pipeline through its ROS service."""
        if self._vision_node is not None:
            self._vision_node.set_servo_mode(mode)
            return

        if not self._vision_mode_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("视觉模式服务不可用")

        from drone_interfaces.srv import SetVisionMode
        request = SetVisionMode.Request()

        if mode == "landing":
            request.mode = self._VISION_MODE_LANDING
            request.enable = True
        elif mode == "none":
            request.mode = self._VISION_MODE_NONE
            request.enable = False
        else:
            # "all" / default — full detection mode
            request.mode = self._VISION_MODE_ALL
            request.enable = True

        self._vision_mode_client.call_async(request)

    # -- 工具 --
    @property
    def logger(self) -> TaskLogger:
        return self._task_logger

    @property
    def watchdog(self) -> Watchdog:
        return self._watchdog

    @property
    def servo_ready(self) -> bool:
        return self._servo_ready

    @property
    def odom_ready(self) -> bool:
        return self._standalone_odom_ready

    @property
    def battery_ready(self) -> bool:
        return self.bridge.battery_ready

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
            if default is None:
                self.get_logger().warn(
                    f"参数 '{key}' 未声明且无默认值, 返回 None, 可能导致运行时错误"
                )
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
        min_obs = int(self.get_param("marker_memory_min_observations", 3))
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

            if not self._drop_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("舵机服务不可用, 投放失败")
                self._drop_future = None
                return

            req = TriggerDrop.Request()
            req.cylinder_id = cylinder_id
            self._drop_future = self._drop_client.call_async(req)

        except Exception as e:
            self.get_logger().error(f"投放服务启动失败: {e}")
            self._drop_future = None
            self._drop_pending_cylinder_id = cylinder_id
            self._drop_result_consumed = False

    def check_drop_service_result(self) -> Optional[bool]:
        """轮询异步投放结果. None 表示仍在等待."""
        if self._drop_result_consumed:
            return None

        # 服务不可用或启动失败是真实投放失败, 绝不能增加成功计数.
        if self._drop_future is None:
            self._drop_count += 1
            self._drop_result_consumed = True
            return False

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

    # -- 悬停 --
    def loiter(self, duration: float = 0.0):
        """在当前位悬停."""
        self.fc.loiter(duration)


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
