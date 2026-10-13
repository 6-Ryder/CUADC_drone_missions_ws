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

import math
import statistics
import time
from enum import Enum
from typing import Any, Dict, List, Optional, Protocol

import rclpy
from rcl_interfaces.msg import SetParametersResult
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
from drone_mission.states.state_precision_land_simple import SimplePrecisionLandState
from drone_mission.states.state_hold_for_takeover import HoldForTakeoverState

# 依赖层
from px4_control.px4_bridge import PX4Bridge
from px4_control.flight_controller import FlightController
from px4_control.standalone_bridge import StandaloneBridge
from px4_control.telemetry import configure_home, ned_to_world, world_to_ned

from drone_mission.cylinder_selector import CylinderSelector
from drone_mission.quick_bucket_gate import (
    BucketReleaseGate, is_bucket_search_candidate,
)
from drone_mission.path_planner import PathPlanner

from drone_utils.logger import TaskLogger
from drone_utils.layout_frame import LayoutFrame
from drone_utils.watchdog import Watchdog
from drone_utils.emergency import EmergencyHandler, EmergencyLevel


FROZEN_LAYOUT_PARAMETERS = frozenset({
    "layout_heading_deg",
    "area1_forward_min", "area1_forward_max",
    "area1_right_min", "area1_right_max",
    "area2_forward_min", "area2_forward_max",
    "area2_right_min", "area2_right_max",
    "sweep_margin_forward", "sweep_margin_right",
    "quick_drop_profile", "bucket_calibration_valid", "bucket_rim_height",
    "bucket_home_body_height", "bucket_camera_down",
    "bucket_front_forward", "bucket_rear_forward",
    "bucket_front_right", "bucket_rear_right",
    "bucket_front_down", "bucket_rear_down",
    "bucket_front_delay", "bucket_rear_delay",
    "bucket_front_uncertainty", "bucket_rear_uncertainty",
    "bucket_disc_uncertainty",
    "bucket_front_bias_north", "bucket_front_bias_east",
    "bucket_rear_bias_north", "bucket_rear_bias_east",
})


# ============================================================
# 状态处理器接口
# ============================================================

class StateHandler(Protocol):
    """状态处理器协议 — 每个 Mission 状态实现 enter/execute/exit."""

    def enter(self) -> None: ...

    def execute(self) -> Optional[str]: ...

    def exit(self) -> None: ...


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
    HOLD_FOR_TAKEOVER = "HOLD_FOR_TAKEOVER"
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

    TARGET_ALTITUDE = 5.0            # 巡航高度 (m)

    def __init__(self):
        super().__init__("mission_state_machine")

        self._declare_params()
        self._layout_frame = LayoutFrame(float(self.get_param("layout_heading_deg", 0.0)))
        self.add_on_set_parameters_callback(self._reject_layout_heading_update)

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
        self._state_handlers: Dict[MissionState, StateHandler] = {}
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
        # 投放确认: 固定延时, 不依赖 ROS2 服务应答或 topic 回调
        self._drop_service_failed: bool = False
        self._drop_result_consumed: bool = True
        self._drop_sent_ok: bool = False
        self._drop_request_time: float = 0.0
        self._drop_pending_cylinder_id: int = -1
        # 状态机重入防护
        self._state_machine_busy: bool = False
        self._servo_ready: bool = False
        self._vision_ready: bool = False
        self._mission_failed: bool = False
        self._takeover_reason: Optional[str] = None
        self._takeover_requested: bool = False
        self._recon_path_completed: bool = False
        # Set only after TAKEOFF has received an actual PX4 Offboard confirmation.
        # Before that point, TAKEOFF itself is responsible for requesting Offboard.
        self._offboard_confirmed: bool = False

        # ---- 视觉数据缓存 ----
        self._cylinders: List[Any] = []
        self._markers: List[Any] = []
        self._known_cylinders: List[Dict[str, Any]] = []
        self._known_markers: List[Dict[str, Any]] = []
        self._next_known_cylinder_id: int = 1
        self._next_known_marker_id: int = 1001
        self._hazards: List[Any] = []
        self._landing_target: Optional[Dict] = None
        # 每个视觉源各记两个时刻 (秒, 单调时钟):
        #   *_capture_stamp  — 消息 header 的采集时刻 (相机帧时刻), 用于控制:
        #                      新鲜度门控、积分/微分真实步长、按帧去重
        #   *_timestamp      — 消息到达 FSM 的时刻, 用于"这个源还在不在"的
        #                      存活判断 (watchdog / 到达时刻龄门)
        # 统一出口是 vision_measurement_stamp(source), 它优先返回采集时刻.
        self._landing_target_stamp: float = 0.0
        # 降落目标的采集时刻 (消息 header, 相机帧时刻). 与 _landing_target_stamp
        # (到达 FSM 的时刻) 分开, 与另两源的 *_capture_stamp 同形.
        self._landing_target_capture_stamp: float = 0.0
        # 最后一次"含真实检测"的降落目标消息时刻 (confidence > 0). 与
        # _landing_target_stamp (每条消息都刷新, 含 miss 帧) 分开, 因为二者
        # 回答不同问题: "这个源还在发消息吗" vs "还有可用的测量吗".
        self._landing_target_detection_stamp: float = 0.0
        self._coverage_status: Optional[Dict] = None
        self._cylinders_timestamp: float = 0.0
        self._markers_timestamp: float = 0.0
        self._hazards_timestamp: float = 0.0
        self._cylinder_capture_stamp: float = 0.0
        self._marker_capture_stamp: float = 0.0
        self._quick_bucket_gate = None
        self._quick_release_intent = None

        self._standalone_odom_ready = False

        # ---- 初始化延迟 ----
        self._init_timer = self.create_timer(1.0, self._lazy_init)
        self._initialized = False

        self.get_logger().info("=" * 60)
        self.get_logger().info("  无人机自主任务状态机 v2.0 已启动")
        self.get_logger().info("=" * 60)

        # 伺服 PID 生效参数回显 —— 请勿当作日志噪音删除.
        # config/pid_tuning.yaml 的顶层 node key 若写错 (缺前导斜杠 "/mission_state_machine"),
        # ROS 2 会整份静默忽略该文件, 不报错也不警告; 本行是判断该文件是否真的被加载的
        # 唯一手段. 真机地面调试必须先核对此行, 数值与预期不符时不要起飞调参.
        self.get_logger().info(
            "SERVO_PID 生效参数: "
            f"kp={self.get_param('servo_kp', 0.40):.4f} "
            f"ki={self.get_param('servo_ki', 0.08):.4f} "
            f"kd={self.get_param('servo_kd', 0.05):.4f} "
            f"max={self.get_param('servo_max_speed', 0.25):.4f} "
            f"min={self.get_param('servo_min_speed', 0.012):.4f} | "
            f"norm kp={self.get_param('servo_norm_kp', 1.25):.4f} "
            f"ki={self.get_param('servo_norm_ki', 0.25):.4f} "
            f"kd={self.get_param('servo_norm_kd', 0.15):.4f} | "
            f"px kp={self.get_param('servo_px_kp', 0.0025):.4f} "
            f"ki={self.get_param('servo_px_ki', 0.0005):.4f} "
            f"kd={self.get_param('servo_px_kd', 0.0003):.4f} "
            f"px_max={self.get_param('servo_px_max_speed', 0.35):.4f} | "
            f"ilim={self.get_param('servo_integral_limit', 0.10):.4f} "
            f"dalpha={self.get_param('servo_derivative_alpha', 0.30):.4f} "
            f"band={self.get_param('servo_integral_band', 0.0):.4f}"
        )

    def _declare_params(self):
        """声明所有 YAML 参数."""
        # 全局
        self.declare_parameter("sim_mode", True)
        self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status_v1")
        self.declare_parameter("home_x", 4.0)
        self.declare_parameter("home_y", 10.0)
        self.declare_parameter("home_z", 0.0)
        self.declare_parameter("layout_heading_deg", 0.0)
        self.declare_parameter("takeoff_height", 5.0)
        self.declare_parameter("waypoint_tolerance", 0.5)
        self.declare_parameter("takeoff_tolerance", 0.3)
        self.declare_parameter("takeoff_control_loop_gap_warn_s", 1.0)
        # 遥控器安全 — True=等待人用遥控器解锁, False=代码 arm 跳过
        self.declare_parameter("rc_require_arm_signal", True)
        self.declare_parameter("control_frequency", 20.0)
        self.declare_parameter("max_error_count", 5)
        self.declare_parameter("waypoint_timeout", 120.0)
        self.declare_parameter("land_altitude", 0.15)
        self.declare_parameter("max_drops", 2)
        # 速度
        self.declare_parameter("sweep_speed", 0.8)
        self.declare_parameter("transit_speed", 2.0)
        self.declare_parameter("return_speed", 3.0)
        self.declare_parameter("return_altitude", 5.0)
        self.declare_parameter("navigation_progress_window", 30.0)
        self.declare_parameter("navigation_progress_min_improvement", 0.25)
        self.declare_parameter("search_progress_window", 30.0)
        self.declare_parameter("visual_progress_min_improvement", 2.0)
        # Area 1
        self.declare_parameter("area1_forward_min", 30.0)
        self.declare_parameter("area1_forward_max", 35.0)
        self.declare_parameter("area1_right_min", -4.0)
        self.declare_parameter("area1_right_max", 4.0)
        self.declare_parameter("area1_altitude", 2.0)
        self.declare_parameter("area1_search_altitude", 5.0)
        # Area 2
        self.declare_parameter("area2_forward_min", 55.0)
        self.declare_parameter("area2_forward_max", 60.0)
        self.declare_parameter("area2_right_min", -4.0)
        self.declare_parameter("area2_right_max", 4.0)
        self.declare_parameter("area2_altitude", 2.5)   # 侦察区低空扫描高度, 与 YAML 一致
        # 覆盖
        self.declare_parameter("grid_spacing", 1.67)
        self.declare_parameter("sweep_margin_forward", 1.0)
        self.declare_parameter("sweep_margin_right", 0.5)
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
        self.declare_parameter("precision_align_continuous_enable", False)
        self.declare_parameter("precision_align_ground_kp", 0.40)
        self.declare_parameter("precision_align_max_speed", 0.25)
        # ── 伺服 PID 增益 (三误差域) ──
        # 契约文件: config/pid_tuning.yaml —— 真机 PID 标定只改那一个文件.
        # 米制域    → 原 precision_align_ground_kp / max_speed / min_correction_speed
        # 归一化域  → 原 precision_align_gain (precision_align_continuous_enable=false 回退路径)
        # 像素域    → 原 landing_servo_kp_pixel / landing_servo_max_horizontal_vel
        # 默认值必须与 pid_tuning.yaml 逐项一致; 全部写成 Python float (0.0 而非 0):
        # YAML 传整数而此处按 double 声明时, rclpy 会抛
        # InvalidParameterTypeException 导致 mission_fsm 启动即崩溃.
        self.declare_parameter("servo_kp", 0.40)
        self.declare_parameter("servo_ki", 0.02)
        self.declare_parameter("servo_kd", 0.0)
        self.declare_parameter("servo_max_speed", 0.25)
        self.declare_parameter("servo_min_speed", 0.012)
        self.declare_parameter("servo_norm_kp", 1.25)
        self.declare_parameter("servo_norm_ki", 0.0625)
        self.declare_parameter("servo_norm_kd", 0.0)
        self.declare_parameter("servo_px_kp", 0.0025)
        self.declare_parameter("servo_px_ki", 0.000125)
        self.declare_parameter("servo_px_kd", 0.0)
        self.declare_parameter("servo_px_max_speed", 0.35)
        # 两域共用: 抗饱和与微分滤波
        self.declare_parameter("servo_integral_limit", 0.03)
        self.declare_parameter("servo_derivative_alpha", 0.30)
        # 积分分离带, 0.0 = 不启用 (保持 float, 勿写成 0)
        self.declare_parameter("servo_integral_band", 0.0)
        self.declare_parameter("bucket_servo_mid_max_speed", 0.14)
        self.declare_parameter("bucket_blue_predescent_enable", True)
        self.declare_parameter("precision_align_fine_pixel_error", 90.0)
        self.declare_parameter("precision_align_fine_max_speed", 0.065)
        self.declare_parameter("precision_align_min_correction_speed", 0.012)
        self.declare_parameter("precision_align_step_s", 0.18)
        self.declare_parameter("precision_align_fine_step_s", 0.12)
        self.declare_parameter("precision_align_hover_s", 0.04)
        self.declare_parameter("precision_align_timeout", 25.0)
        self.declare_parameter("precision_align_max_retries", 1)
        self.declare_parameter("precision_align_image_x_to_right_sign", 1.0)
        self.declare_parameter("precision_align_image_y_to_forward_sign", -1.0)
        self.declare_parameter("precision_align_threshold_norm_x", 0.008)
        self.declare_parameter("precision_align_threshold_norm_y", 0.008)
        self.declare_parameter("precision_align_raw_pixel_tolerance", 9.0)
        self.declare_parameter("precision_align_smooth_pixel_tolerance", 7.0)
        self.declare_parameter("precision_align_stable_frames", 5)
        self.declare_parameter("precision_align_final_brake_s", 0.5)
        self.declare_parameter("precision_align_release_max_horizontal_speed", 0.04)
        # 对心满足后投放许可宽限秒数: 短暂丢帧不撤销已满足的对心, 避免卡在投放区
        self.declare_parameter("precision_align_release_latch_s", 1.5)
        # 两级对心窗口: 仅真机显式配置 tight_window_height>0 时启用.
        # SITL 不配置 → 切换高度 0.0 → 始终用 raw_pixel_tolerance, 行为不变.
        self.declare_parameter("precision_align_final_raw_pixel_tolerance", 30.0)
        self.declare_parameter("precision_align_tight_window_height", 0.0)
        self.declare_parameter("precision_align_min_confidence", 0.25)
        # 记忆世界坐标门控开关 (默认关闭 = 候选选择一切以视觉像素为准).
        # 开启后 precision_align_target_world_gate 才被读取: 未锁定时拒绝距
        # 搜索期记忆点过远的候选, 并把距离作为 0.08 权重的弱项参与评分.
        self.declare_parameter("precision_align_world_gate_enable", False)
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
        self.declare_parameter("camera_fx", 539.9)
        self.declare_parameter("camera_fy", 539.9)
        self.declare_parameter("camera_cx", 640.0)
        self.declare_parameter("camera_cy", 480.0)
        # True=画面顶朝机头 (真机), False=gz SITL 相机 (画面底朝机头)
        self.declare_parameter("image_top_faces_forward", True)
        # 降落
        self.declare_parameter("landing_search_altitude", 5.0)
        self.declare_parameter("landing_approach_speed", 0.5)
        self.declare_parameter("landing_blind_altitude", 1.5)
        self.declare_parameter("landing_blind_align_tolerance_px", 30.0)
        self.declare_parameter("landing_blind_stable_frames", 10)
        self.declare_parameter("landing_alignment_latch_max_altitude", 2.5)
        self.declare_parameter("landing_precision_target", 0.1)
        # 降落伺服 (图像空间 P 控制器, 与投放区精校同方法)
        self.declare_parameter("landing_servo_kp_pixel", 0.004)
        self.declare_parameter("landing_servo_max_horizontal_vel", 0.6)
        self.declare_parameter("landing_servo_deadzone_px", 12.0)
        self.declare_parameter("landing_servo_x_to_right_sign", 1.0)
        self.declare_parameter("landing_servo_y_to_forward_sign", -1.0)
        self.declare_parameter("landing_target_min_confidence", 0.60)
        self.declare_parameter("landing_target_max_jump_px", 80.0)
        self.declare_parameter("landing_descent_fast", 0.4)
        self.declare_parameter("landing_descent_medium", 0.25)
        self.declare_parameter("landing_descent_slow", 0.12)
        self.declare_parameter("landing_blind_descent_speed", 0.30)
        # 丢失目标后继续飞向"最后已知位置" (PX4 precland 水平接近段语义).
        # 速度取保守值: 这段是在没有测量的情况下飞行.
        # 降落目标状态估计器 (drone_utils.landing_estimator). 权威的"微分项"
        # 来源是滤波器状态而非对测量求差; 参数含义见该模块 docstring.
        self.declare_parameter("landing_estimator_acc_unc", 0.5)
        self.declare_parameter("landing_estimator_pos_unc_init", 0.25)
        self.declare_parameter("landing_estimator_vel_unc_init", 0.25)
        # 角噪声方差 (tan²). σ_tan≈0.022 ≈ f=400 时 9px 抖动; 位置方差 = 该值×高度².
        # ⚠️ 必须按真机探测器抖动实测标定, 默认值只是保守起点.
        self.declare_parameter("landing_estimator_meas_unc_tan2", 5e-4)
        self.declare_parameter("landing_lost_transit_max_speed", 0.15)
        self.declare_parameter("landing_lost_transit_tolerance", 0.15)
        # 低于此高度不再横向接近, 只保持垂直 (对齐 ArduPilot PLND_ALT_MIN=0.75:
        # 近地丢目标不再重试, 避免在离地很近时做未知的横向机动).
        self.declare_parameter("landing_lost_transit_min_altitude", 0.75)
        self.declare_parameter("landing_blind_timeout", 45.0)
        self.declare_parameter("landing_search_progress_window", 15.0)
        self.declare_parameter("landing_touchdown_max_altitude", 0.10)
        self.declare_parameter("landing_touchdown_max_horizontal_speed", 0.10)
        self.declare_parameter("landing_touchdown_max_vertical_speed", 0.05)
        self.declare_parameter("landing_touchdown_hold_time", 2.0)
        # 异常
        self.declare_parameter("battery_warning", 0.25)
        self.declare_parameter("battery_critical", 0.15)
        self.declare_parameter("vision_loss_timeout", 3.0)
        self.declare_parameter("state_max_duration", 180.0)
        # 快速投放策略 (quick_drop_mode 默认关闭, 保持现状行为)
        self.declare_parameter("quick_drop_mode", False)
        self.declare_parameter("quick_drop_profile", "blue")
        self.declare_parameter("bucket_calibration_valid", False)
        self.declare_parameter("bucket_rim_height", 0.3)
        self.declare_parameter("bucket_home_body_height", 0.0)
        self.declare_parameter("bucket_camera_down", 0.0)
        self.declare_parameter("bucket_front_forward", 0.04)
        self.declare_parameter("bucket_rear_forward", -0.04)
        self.declare_parameter("bucket_front_right", 0.0)
        self.declare_parameter("bucket_rear_right", 0.0)
        self.declare_parameter("bucket_front_down", 0.0)
        self.declare_parameter("bucket_rear_down", 0.0)
        self.declare_parameter("bucket_front_delay", 0.0)
        self.declare_parameter("bucket_rear_delay", 0.0)
        self.declare_parameter("bucket_front_uncertainty", 0.03)
        self.declare_parameter("bucket_rear_uncertainty", 0.10)
        self.declare_parameter("bucket_disc_uncertainty", 0.10)
        self.declare_parameter("bucket_front_bias_north", 0.0)
        self.declare_parameter("bucket_front_bias_east", 0.0)
        self.declare_parameter("bucket_rear_bias_north", 0.0)
        self.declare_parameter("bucket_rear_bias_east", 0.0)
        self.declare_parameter("drop_target_source", "cylinder")  # "cylinder" | "marker"
        self.declare_parameter("drop_align_stall_threshold", 40.0)
        self.declare_parameter("drop_align_stall_stable_band", 5.0)
        self.declare_parameter("drop_align_stall_duration", 2.0)
        self.declare_parameter("precision_align_fallback_max_pixel_error", 0.0)  # 0=禁用兜底
        # A 方案: 下降全程曾对心 (最佳地面误差 ≤ 阈值) 兜底投放, 0=禁用
        self.declare_parameter("precision_align_fallback_max_ground_error", 0.4)
        self.declare_parameter("drop_descend_speed", 0.5)
        # 降落区 simple 模式 (landing_simple_mode 默认关闭 = 主任务逻辑)
        self.declare_parameter("landing_simple_mode", False)
        self.declare_parameter("landing_search_timeout", 15.0)  # simple 搜索 H 超时
        self.declare_parameter("landing_simple_blind_timeout", 60.0)  # simple 盲降超时
        self.declare_parameter("landing_target_reacquire_timeout", 3.0)
        self.declare_parameter("landing_visual_handoff_altitude", 0.0)
        self.declare_parameter("landing_visual_handoff_max_ground_error_m", 0.20)
        self.declare_parameter("landing_visual_handoff_stable_frames", 10)
        # 快速模式边伺服边下降: 降高中像素 P 控制水平对心, 分层垂直速度
        self.declare_parameter("drop_servo_descend_fast", 0.5)
        self.declare_parameter("drop_servo_descend_slow", 0.3)

    # ================================================================
    # 延迟初始化
    # ================================================================

    @property
    def layout_frame(self) -> LayoutFrame:
        """Return the immutable field layout captured at mission startup."""
        return self._layout_frame

    @property
    def layout_yaw_rad(self) -> float:
        """Return the fixed absolute yaw used for mission setpoints."""
        return self._layout_frame.yaw_rad

    def zone_bounds(self, prefix: str) -> tuple[float, float, float, float]:
        """Return a named area's local forward/right bounds."""
        return (
            float(self.get_param(f"{prefix}_forward_min")),
            float(self.get_param(f"{prefix}_forward_max")),
            float(self.get_param(f"{prefix}_right_min")),
            float(self.get_param(f"{prefix}_right_max")),
        )

    def _validate_zone_bounds(self, prefix: str) -> tuple[float, float, float, float]:
        """Fail closed before arming when an area cannot define a rectangle."""
        bounds = self.zone_bounds(prefix)
        if not all(math.isfinite(value) for value in bounds):
            raise ValueError(f"{prefix} bounds must be finite")
        forward_min, forward_max, right_min, right_max = bounds
        if forward_min >= forward_max or right_min >= right_max:
            raise ValueError(f"{prefix} bounds must have min < max")
        return bounds

    def _reject_layout_heading_update(self, params):
        """Keep all launch-time layout geometry immutable for one mission."""
        if any(param.name in FROZEN_LAYOUT_PARAMETERS for param in params):
            return SetParametersResult(
                successful=False,
                reason="mission geometry and drop profile are fixed at startup",
            )
        return SetParametersResult(successful=True)

    def _lazy_init(self):
        """等待 PX4 Bridge 就绪后初始化."""
        if self._initialized:
            return

        profile = self.get_param("quick_drop_profile", "blue")
        if profile not in ("blue", "bucket"):
            self.get_logger().error(f"quick_drop_profile 非法: {profile}")
            self._mission_failed = True
            self._init_timer.cancel()
            return

        drop_bounds = self._validate_zone_bounds("area1")
        recon_bounds = self._validate_zone_bounds("area2")
        drop_north, drop_east = self._layout_frame.bounds_center_ned(*drop_bounds)
        recon_north, recon_east = self._layout_frame.bounds_center_ned(*recon_bounds)

        hx = self.get_param("home_x", 4.0)
        hy = self.get_param("home_y", 10.0)
        hz = self.get_param("home_z", 0.0)
        configure_home(hx, hy, hz)

        drop_world_x, drop_world_y, _ = ned_to_world(drop_north, drop_east, 0.0)
        recon_world_x, recon_world_y, _ = ned_to_world(recon_north, recon_east, 0.0)
        self.get_logger().info(
            f"MISSION_LAYOUT: heading={self._layout_frame.heading_deg:.1f}deg "
            f"home=({hx:.2f}, {hy:.2f}, {hz:.2f}) "
            f"drop_ned=({drop_north:.2f}, {drop_east:.2f}) "
            f"recon_ned=({recon_north:.2f}, {recon_east:.2f})"
        )
        drop_zone = (drop_world_x, drop_world_y, self.TARGET_ALTITUDE)
        recon_zone = (recon_world_x, recon_world_y, self.TARGET_ALTITUDE)

        # 检查 PX4 Bridge 是否可以获取
        # 在真实部署中, PX4Bridge 是独立节点; 此处通过 ros2 话题检查
        # 为简化 integration, 此处创建桥接的引用 (实际需通过 DDS)
        # 在 SITL 场景中, 桥接和 FSM 在不同进程中通过 ROS2 话题通信

        # 我们假设 PX4Bridge 已在另一个进程中运行
        # 此时仅初始化内部组件
        self._path_planner = PathPlanner(
            layout_frame=self._layout_frame,
            sweep_speed=self.get_param("sweep_speed", 0.8),
            transit_speed=self.get_param("transit_speed", 2.0),
            return_speed=self.get_param("return_speed", 3.0),
            grid_spacing=self.get_param("grid_spacing", 1.67),
            margin_forward=self.get_param("sweep_margin_forward", 1.0),
            margin_right=self.get_param("sweep_margin_right", 0.5),
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
            # 降落区: landing_simple_mode=True 时用 test_mission 方法 (quick 策略),
            # 默认 False 保持主任务精准降落逻辑, 两套互不影响
            MissionState.PRECISION_LAND: (
                SimplePrecisionLandState(self)
                if self.get_param("landing_simple_mode", False)
                else PrecisionLandState(self)
            ),
            MissionState.HOLD_FOR_TAKEOVER: HoldForTakeoverState(self),
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

        # 关键参数验证: 非法值会在日志警告, 不会阻止启动
        self._validate_critical_params()

        # 初始化视觉数据订阅
        self._init_vision_subscriptions()
        # INIT 依赖 PX4 电池/状态消息，必须先创建 standalone 遥测订阅。
        _ = self.bridge

        # 提前创建投放服务客户端, 避免首次投放时 wait_for_service 阻塞控制循环
        try:
            from drone_interfaces.srv import TriggerDrop
            self._drop_client = self.create_client(
                TriggerDrop, "/drone/servo/trigger_drop"
            )
        except Exception as e:
            self.get_logger().warn(f"投放服务客户端创建失败: {e}")

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
        capture_stamp = self._capture_seconds(msg)
        if (self.get_param("quick_drop_profile", "blue") == "bucket"
                and capture_stamp <= self._cylinder_capture_stamp):
            return
        self._cylinders = list(msg.cylinders)
        self._cylinders_timestamp = time.time()
        self._cylinder_capture_stamp = capture_stamp
        if self._current_state == MissionState.CYLINDER_SEARCH:
            self._remember_cylinders(self._cylinders)
        self._emergency.update_vision(len(self._cylinders) > 0)

    def _marker_cb(self, msg):
        capture_stamp = self._capture_seconds(msg)
        if (self.get_param("quick_drop_profile", "blue") == "bucket"
                and capture_stamp <= self._marker_capture_stamp):
            return
        self._markers = list(msg.cylinders)
        self._markers_timestamp = time.time()
        self._marker_capture_stamp = capture_stamp
        if self._current_state == MissionState.CYLINDER_SEARCH:
            self._remember_markers(self._markers)
        # marker_only 模式下 cylinders 恒空, 应急视觉信号必须由 marker 喂给
        if self.get_param("drop_target_source", "cylinder") == "marker":
            self._emergency.update_vision(len(self._markers) > 0)

    def vision_measurement_stamp(self, source: str) -> float:
        """
        返回指定视觉源最近一次测量的**采集时刻** (秒), 无测量时返回 0.0.

        这里的"采集时刻"取消息 header 的时刻 (相机帧时刻), 而不是消息到达
        FSM 的时刻 —— 与权威约定一致: MAVLink LANDING_TARGET.time_usec 与
        PX4 的 FiducialMarkerPosReport.timestamp_sample 都是"这次测量是什么
        时候拍的". 两者在 3-5Hz 下相差约一个推理周期, 但按到达时刻求差会把
        每帧的推理耗时抖动带进积分/微分步长.

        消费方用它做三件事:
          - 新鲜度门控: 过期测量不得继续驱动控制 (stale -> 视为无目标).
          - 真实步长: 积分/微分按相邻两次测量的时间差推进, 而不是按 FSM
            tick 周期. 真机视觉 3-5Hz 而 tick 20Hz, 按 tick 推进会把同一份
            误差重复累加数倍, 微分也会被放大.
          - 按帧去重: 同一份测量在多个 tick 上被复用时传入相同 stamp,
            ServoPid 只在 stamp 前进时推进积分/微分.

        source 取 "cylinder" | "marker" | "landing".

        降级路径 (与 PX4 sync_stamp 未收敛时退回接收时刻同构, 但必须能被
        察觉): 三个源的 header 缺失时 _capture_seconds 返回 0.0, 此时退回
        到达时刻 —— 步长会重新带上推理耗时抖动, 是降级不是等价物.
        """
        if source == "cylinder":
            return self._cylinder_capture_stamp or self._cylinders_timestamp
        if source == "marker":
            return self._marker_capture_stamp or self._markers_timestamp
        if source == "landing":
            return (self._landing_target_capture_stamp
                    or self._landing_target_stamp)
        return 0.0

    @staticmethod
    def _capture_seconds(msg) -> float:
        try:
            stamp = msg.header.stamp
            seconds = stamp.sec + stamp.nanosec * 1e-9
            return seconds if math.isfinite(seconds) and seconds > 0 else 0.0
        except (AttributeError, TypeError, ValueError):
            return 0.0

    @staticmethod
    def _cyl_field(cyl, key: str, default=0.0):
        return getattr(cyl, key, cyl.get(key, default) if isinstance(cyl, dict) else default)

    def _remember_cylinders(self, cylinders: list) -> None:
        """Accumulate drop-zone cylinder positions across the search scan."""
        if self.quick_bucket_enabled():
            self._remember_bucket_cylinders(cylinders)
            return
        min_conf = 0.35
        merge_dist = float(self.get_param("cylinder_memory_merge_distance", 0.75))
        now = time.time()

        for cyl in cylinders:
            conf = float(self._cyl_field(cyl, "confidence", 0.0))
            if conf < min_conf:
                continue

            if not bool(self._cyl_field(cyl, "world_position_valid", False)):
                continue
            raw_x = float(self._cyl_field(cyl, "world_x", math.nan))
            raw_y = float(self._cyl_field(cyl, "world_y", math.nan))
            if not (math.isfinite(raw_x) and math.isfinite(raw_y)):
                continue
            target_xy = self.validate_drop_target_world_xy(raw_x, raw_y)
            if target_xy is None:
                continue
            cx, cy = target_xy
            radius = float(self._cyl_field(cyl, "radius", 0.0))

            best = None
            best_dist = float("inf")
            for known in self._known_cylinders:
                dist = ((known["world_x"] - cx) ** 2 + (known["world_y"] - cy) ** 2) ** 0.5
                if dist < best_dist:
                    best = known
                    best_dist = dist

            if best is not None and best_dist <= merge_dist:
                obs = best["observations"]
                weight = min(conf, 1.0)
                total = obs + weight
                best["world_position_valid"] = True
                best["world_x"] = (best["world_x"] * obs + cx * weight) / total
                best["world_y"] = (best["world_y"] * obs + cy * weight) / total
                best["radius"] = (best["radius"] * obs + radius * weight) / total
                best["confidence"] = max(best["confidence"], conf)
                best["observations"] = obs + 1
                best["last_seen"] = now
            else:
                best = {
                    "id": self._next_known_cylinder_id,
                    "world_position_valid": True,
                    "world_x": cx,
                    "world_y": cy,
                    "radius": radius,
                    "confidence": conf,
                    "observations": 1,
                    "last_seen": now,
                }
                self._known_cylinders.append(best)
                self._next_known_cylinder_id += 1

            # The rim refiner can alternate between a real 7.5 cm rim and
            # nearby blue-disc edges. Keep independent small-radius evidence
            # instead of diluting it into the all-sample mean above.
            samples = [(stamp, value) for stamp, value in
                       best.get("small_radius_samples", []) if now - stamp <= 1.0]
            if math.isfinite(radius) and 0.06 <= radius <= 0.09:
                samples.append((now, radius))
            best["small_radius_samples"] = samples[-30:]
            best["small_radius_confirmed"] = len(samples) >= 5

    def _remember_bucket_cylinders(self, cylinders: list) -> None:
        """Track recent image targets; high-altitude world projection is advisory."""
        now = time.time()
        capture = self._cylinder_capture_stamp or now
        for cyl in cylinders:
            conf = float(self._cyl_field(cyl, "confidence", 0.0))
            radius = float(self._cyl_field(cyl, "radius", math.nan))
            u = float(self._cyl_field(cyl, "center_u", math.nan))
            v = float(self._cyl_field(cyl, "center_v", math.nan))
            if (not math.isfinite(conf) or conf < 0.35
                    or not math.isfinite(radius) or not 0.03 <= radius <= 0.30
                    or not math.isfinite(u) or not math.isfinite(v)):
                continue
            tracker_id = int(self._cyl_field(cyl, "id", -1))
            best = None
            best_distance = math.inf
            for known in self._known_cylinders:
                distance = math.hypot(known["center_u"] - u, known["center_v"] - v)
                limit = 90.0 if known["tracker_id"] == tracker_id else 35.0
                if distance <= limit and distance < best_distance:
                    best, best_distance = known, distance
            if best is None:
                best = {
                    "id": self._next_known_cylinder_id,
                    "tracker_id": tracker_id,
                    "center_u": u, "center_v": v,
                    "world_position_valid": False,
                    "world_x": math.nan, "world_y": math.nan,
                    "radius": radius, "confidence": conf,
                    "observations": 0, "last_seen": now,
                }
                self._known_cylinders.append(best)
                self._next_known_cylinder_id += 1
            best["center_u"], best["center_v"] = u, v
            best["tracker_id"] = tracker_id
            best["confidence"] = max(best["confidence"], conf)
            best["observations"] += 1
            best["last_seen"] = now
            if bool(self._cyl_field(cyl, "world_position_valid", False)):
                wx = float(self._cyl_field(cyl, "world_x", math.nan))
                wy = float(self._cyl_field(cyl, "world_y", math.nan))
                if math.isfinite(wx) and math.isfinite(wy):
                    best["world_position_valid"] = True
                    best["world_x"], best["world_y"] = wx, wy
            samples = {stamp: value for stamp, value in
                       best.get("bucket_search_samples", {}).items()
                       if 0 <= now - stamp <= 2.0}
            if 0 < capture <= now:
                samples.setdefault(capture, radius)
            best["bucket_search_samples"] = samples
            if samples:
                best["bucket_search_radius"] = statistics.median(samples.values())
                best["radius"] = best["bucket_search_radius"]

    def _remember_markers(self, markers: list) -> None:
        """Accumulate blue drop-marker positions as fallback drop targets."""
        min_conf = float(self.get_param("drop_marker_min_confidence", 0.35))
        merge_dist = float(self.get_param("marker_memory_merge_distance", 0.75))
        now = time.time()

        for marker in markers:
            conf = float(self._cyl_field(marker, "confidence", 0.0))
            if conf < min_conf:
                continue

            if not bool(self._cyl_field(marker, "world_position_valid", False)):
                continue
            raw_x = float(self._cyl_field(marker, "world_x", math.nan))
            raw_y = float(self._cyl_field(marker, "world_y", math.nan))
            if not (math.isfinite(raw_x) and math.isfinite(raw_y)):
                continue
            target_xy = ((raw_x, raw_y) if self.quick_bucket_enabled() else
                         self.validate_drop_target_world_xy(raw_x, raw_y))
            if target_xy is None:
                continue
            mx, my = target_xy

            best = None
            best_dist = float("inf")
            for known in self._known_markers:
                dist = ((known["world_x"] - mx) ** 2 + (known["world_y"] - my) ** 2) ** 0.5
                if dist < best_dist:
                    best = known
                    best_dist = dist

            if best is not None and best_dist <= merge_dist:
                obs = best["observations"]
                weight = min(conf, 1.0)
                total = obs + weight
                best["world_position_valid"] = True
                best["world_x"] = (best["world_x"] * obs + mx * weight) / total
                best["world_y"] = (best["world_y"] * obs + my * weight) / total
                best["confidence"] = max(best["confidence"], conf)
                best["observations"] = obs + 1
                best["last_seen"] = now
            else:
                self._known_markers.append({
                    "id": self._next_known_marker_id,
                    "world_position_valid": True,
                    "world_x": mx,
                    "world_y": my,
                    "radius": 0.5,
                    "confidence": conf,
                    "observations": 1,
                    "last_seen": now,
                    "source": "marker",
                })
                self._next_known_marker_id += 1

    def validate_drop_target_world_xy(
        self, world_x: float, world_y: float, margin: float = 1.0,
    ) -> Optional[tuple[float, float]]:
        """Return a world target only when it lies in the local drop rectangle."""
        if self._contains_drop_target_world_xy(world_x, world_y, margin=margin):
            return world_x, world_y
        self.get_logger().warn(
            f"CYL_SEARCH: discard out-of-zone world target ({world_x:.2f}, {world_y:.2f})"
        )
        return None

    def _contains_drop_target_world_xy(
        self, world_x: float, world_y: float, margin: float,
    ) -> bool:
        """Check exact local drop bounds without emitting a rejection warning."""
        north, east, _ = world_to_ned(world_x, world_y, 0.0)
        return self.layout_frame.contains_ned(
            north, east, *self.zone_bounds("area1"), margin=margin,
        )

    def _hazard_cb(self, msg):
        self._hazards = list(msg.hazards)
        self._hazards_timestamp = time.time()
        self._watchdog.feed("vision")

    def _landing_target_cb(self, msg):
        now = time.time()
        self._landing_target_stamp = now
        capture_stamp = self._capture_seconds(msg)
        if capture_stamp > 0.0:
            self._landing_target_capture_stamp = capture_stamp
        # "有测量"的判据: 视觉节点对"本帧没看到 H 标识"发布的是 confidence
        # 恰为 0.0 的合成样本 (vision_pipeline._process_landing_v2 三个发布
        # 分支), 所以 >0 就是"这一帧真的检测到了", 与置信度门槛 (消费侧的
        # 0.60) 是两件事, 不能互相替代.
        if float(getattr(msg, "confidence", 0.0)) > 0.0:
            self._landing_target_detection_stamp = now
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

        # 重入防护: spin_once 期间定时器可能再次触发, 跳过避免状态撕裂
        if self._state_machine_busy:
            return
        self._state_machine_busy = True
        try:
            self._control_loop_impl()
        finally:
            self._state_machine_busy = False

    def _control_loop_impl(self):

        self._watchdog.feed("mission_fsm")

        if self.bridge.battery_ready:
            self._emergency.update_battery(self.bridge.battery_remaining)

        # 紧急检查
        emergency_level = self._emergency.check()
        if emergency_level == EmergencyLevel.RTL:
            self.request_takeover("emergency battery/vision warning")
            return
        elif emergency_level == EmergencyLevel.LAND:
            self.request_takeover("emergency land")
            return
        elif emergency_level == EmergencyLevel.ABORT:
            self.request_takeover("emergency abort")
            return

        # TAKEOFF owns the initial transition into Offboard.  Once PX4 has
        # explicitly confirmed that transition, any later mode loss invalidates
        # the active task trajectory and must stop autonomous output at once.
        if (
            self._offboard_confirmed
            and self._current_state not in (
                MissionState.COMPLETE,
                MissionState.ERROR,
                MissionState.HOLD_FOR_TAKEOVER,
            )
            and not self.bridge.is_offboard()
        ):
            self.request_takeover("PX4 left Offboard")
            return

        # 飞行中遥控器上锁 → 立刻中止 (仅 RC 模式)
        if self.get_param("rc_require_arm_signal", True):
            if self._current_state not in (
                MissionState.INIT, MissionState.COMPLETE, MissionState.ERROR,
                MissionState.HOLD_FOR_TAKEOVER,
            ) and not self.bridge.is_armed():
                handler = self._state_handlers.get(self._current_state)
                final_ground_disarm = (
                    self._current_state == MissionState.PRECISION_LAND
                    and bool(getattr(handler, "final_disarm_pending", False))
                )
                if not final_ground_disarm:
                    self.get_logger().error(
                        "遥控器上锁 (disarm) — 飞行中检测到，停止自主输出"
                    )
                    self.request_takeover("vehicle disarmed")
                    return
                # PrecisionLandState issued force_disarm only after PX4
                # touchdown (or the strict touchdown fallback).  Let it
                # observe the expected disarm and complete the mission.

        # 执行当前状态
        if self._current_state == MissionState.COMPLETE:
            return

        handler = self._state_handlers.get(self._current_state)
        if handler is None:
            self.request_takeover(f"no handler for {self._current_state.value}")
            return

        try:
            next_state_name = handler.execute()
        except Exception as e:
            self.get_logger().error(f"状态执行异常 {self._current_state.value}: {e}")
            self.request_takeover(f"state execute exception: {e}")
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
            if state == MissionState.ERROR:
                self.request_takeover("legacy ERROR transition")
                return
            self._transition_to(state)
        except ValueError:
            self.get_logger().error(f"未知状态: {state_name}")
            self.request_takeover(f"unknown state: {state_name}")

    def _transition_to(self, state: MissionState):
        """执行状态转换."""
        if state == MissionState.ERROR:
            self.request_takeover("legacy ERROR transition")
            return
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
                self.request_takeover(f"state enter exception: {e}")
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
        if self.quick_bucket_enabled():
            cylinders = self.get_known_cylinders()
            targets = [c for c in cylinders
                       if is_bucket_search_candidate(c, time.time())]
            if not targets:
                self.request_takeover("SELECT_TARGET: no recently confirmed bucket")
                return
        if not targets:
            self.get_logger().warn("SELECT_TARGET: 暂无可投目标, 返回投放区搜索")
            self._transition_to(MissionState.CYLINDER_SEARCH)
            return

        target_id = self._cylinder_selector.select(
            targets, preferred_radius_window=(0.06, 0.09)
            if self.quick_bucket_enabled() else None,
            allow_pixel_only=self.quick_bucket_enabled())
        if target_id is None:
            max_drops = int(self.get_param("max_drops", 2))
            if self._drop_success_count < max_drops:
                self.get_logger().warn("SELECT_TARGET: 候选均被占用, 返回投放区重新搜索")
                self._transition_to(MissionState.CYLINDER_SEARCH)
            else:
                self._transition_to(MissionState.TRANSIT_TO_RECON_ZONE)
            return

        self._current_drop_cyl_id = target_id
        if self.quick_bucket_enabled():
            bucket = next(c for c in targets if int(c["id"]) == target_id)
            self._quick_bucket_gate = BucketReleaseGate()
            self._quick_bucket_gate.bucket_pixel_uv = (
                bucket["center_u"], bucket["center_v"])
            self._quick_release_intent = None
        self.get_logger().info(f"SELECT_TARGET: 选中圆筒 ID={target_id}")
        self._task_logger.log("target_selected", {
            "cylinder_id": target_id,
            "drop_number": self._drop_success_count + 1,
        })
        self._transition_to(MissionState.CYLINDER_APPROACH)

    def _handle_complete(self):
        """任务完成处理."""
        max_drops = int(self.get_param("max_drops", 2))
        success = (
            not self._mission_failed
            and self._drop_success_count >= max_drops
            and bool(getattr(self, "_recon_path_completed", False))
        )
        self.get_logger().info("=" * 60)
        self.get_logger().info("  任务完成" if success else "  任务失败")
        self.get_logger().info(f"  投放: {self._drop_success_count}/{self._drop_count}")
        coverage = self.get_coverage()
        if coverage:
            self.get_logger().info(f"  侦察覆盖: {coverage.get('coverage_ratio', 0):.1%}")
        self.get_logger().info("=" * 60)
        self._task_logger.write_summary({
            "success": success,
            "layout_heading_deg": self.layout_frame.heading_deg,
            "strategy": "quick" if self.get_param("quick_drop_mode", False) else "main",
            "drops_completed": self._drop_success_count,
            "drops_total": self._drop_count,
            "coverage_ratio": coverage.get("coverage_ratio", 0) if coverage else 0,
            "final_state": "COMPLETE" if success else "HOLD_FOR_TAKEOVER",
        })
        # The standalone bridge owns its own heartbeat timer.  Stopping only
        # this FSM timer would leave a live lease and the last Offboard target
        # on the real PX4 DDS path after a completed mission.
        bridge = getattr(self, "bridge", None)
        if bridge is not None:
            bridge.stop_autonomous_output()
        # 停止控制循环
        if hasattr(self, '_ctrl_timer'):
            self._ctrl_timer.cancel()

    def _handle_error(self):
        """错误处理."""
        self.request_takeover("legacy ERROR handler")

    def request_takeover(self, reason: str) -> None:
        """Stop task progress and retain a zero-velocity hold for RC takeover."""
        if (
            getattr(self, "_takeover_requested", False)
            or getattr(self, "_current_state", None) == MissionState.HOLD_FOR_TAKEOVER
        ):
            return
        self._takeover_requested = True
        self._mission_failed = True
        if self._takeover_reason is None:
            self._takeover_reason = str(reason)
            self.get_logger().error(
                f"任务等待人工接管: {self._takeover_reason}"
            )

        bridge = self.bridge
        if bridge.is_armed() and bridge.is_offboard():
            bridge.hold_for_takeover()
        else:
            bridge.stop_autonomous_output()
        self._transition_to(MissionState.HOLD_FOR_TAKEOVER)

    def mark_offboard_confirmed(self) -> None:
        """Record the one successful PX4 Offboard confirmation for this mission."""
        self._offboard_confirmed = True

    def mark_recon_path_completed(self) -> None:
        """Record completion of every planned reconnaissance waypoint."""
        self._recon_path_completed = True

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
        # 清理旧的 StandaloneBridge, 避免悬空 timer 继续发布过期 setpoint
        if self._standalone_bridge is not None:
            self._standalone_bridge.shutdown()
            self._standalone_bridge = None
            self._standalone_fc = None
        self._bridge = b
        self._fc = FlightController(b, task_yaw=self._layout_frame.yaw_rad)

    @property
    def fc(self):
        if self._fc is not None:
            return self._fc
        if self._standalone_fc is None:
            self._standalone_fc = FlightController(
                self.bridge, task_yaw=self._layout_frame.yaw_rad
            )
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
    _VISION_MODE_COVERAGE_RESET = 4
    _VISION_MODE_COVERAGE = 5
    _VISION_MODE_MARKER_ONLY = 6

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
        elif mode == "coverage_reset":
            request.mode = self._VISION_MODE_COVERAGE_RESET
            request.enable = True
        elif mode == "coverage":
            request.mode = self._VISION_MODE_COVERAGE
            request.enable = True
        elif mode == "none":
            request.mode = self._VISION_MODE_NONE
            request.enable = False
        elif mode == "marker_only":
            request.mode = self._VISION_MODE_MARKER_ONLY
            request.enable = True
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
    def land_ready(self) -> bool:
        return bool(getattr(self.bridge, "land_ready", False))

    @property
    def vision_ready(self) -> bool:
        return self._vision_ready

    @property
    def rc_ready(self) -> bool:
        """遥控器已解锁 → 才允许进入自动任务.

        rc_require_arm_signal=True (默认/真机):
          PX4 必须已被 RC Arm Switch 解锁 (is_armed=True).
          代码绝不发送 arm 指令.
        rc_require_arm_signal=False (仿真/无遥控器):
          只需 preflight_ok, arm 由 TakeoffState 发送.
        """
        if not self.get_param("rc_require_arm_signal", True):
            return True
        return self.bridge.preflight_ok and self.bridge.is_armed()

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
                return None
            self.get_logger().warn(
                f"参数 '{key}' 读取失败, 使用默认值 {default}"
            )
            return default

    def _validate_critical_params(self) -> None:
        """验证关键参数的取值范围, 越界时记录告警."""
        checks = [
            ("takeoff_height", 1.0, 50.0),
            ("control_frequency", 5.0, 100.0),
            ("battery_warning", 0.05, 1.0),
            ("battery_critical", 0.01, 0.9),
            ("vision_loss_timeout", 0.5, 60.0),
            ("state_max_duration", 0.0, 600.0),
            ("drop_descend_height", 0.3, 10.0),
            ("precision_align_alpha", 0.01, 1.0),
            ("precision_align_gain", 0.1, 10.0),
            ("precision_align_ground_kp", 0.01, 3.0),
            ("servo_kp", 0.0, 10.0),
            ("servo_ki", 0.0, 5.0),
            ("servo_kd", 0.0, 5.0),
            ("servo_max_speed", 0.0, 2.0),
            ("servo_min_speed", 0.0, 0.5),
            ("servo_norm_kp", 0.0, 10.0),
            ("servo_norm_ki", 0.0, 5.0),
            ("servo_norm_kd", 0.0, 5.0),
            ("servo_px_kp", 0.0, 0.1),
            ("servo_px_ki", 0.0, 0.05),
            ("servo_px_kd", 0.0, 0.05),
            ("servo_px_max_speed", 0.0, 2.0),
            ("servo_integral_limit", 0.0, 1.0),
            ("servo_derivative_alpha", 0.01, 1.0),
            ("servo_integral_band", 0.0, 100.0),
            ("precision_align_max_speed", 0.01, 2.0),
            ("precision_align_timeout", 2.0, 120.0),
            ("precision_align_stable_frames", 1, 50),
            ("precision_align_final_drop_height", 0.2, 8.0),
            ("grid_spacing", 0.2, 10.0),
            ("waypoint_tolerance", 0.05, 5.0),
            ("sweep_speed", 0.1, 15.0),
            ("transit_speed", 0.1, 15.0),
            ("return_speed", 0.1, 15.0),
        ]
        for key, lo, hi in checks:
            val = self.get_param(key)
            if val is None:
                continue
            try:
                fval = float(val)
                if not (lo <= fval <= hi):
                    self.get_logger().warn(
                        f"参数 '{key}'={fval} 超出推荐范围 [{lo}, {hi}]"
                    )
            except (TypeError, ValueError):
                self.get_logger().warn(f"参数 '{key}' 类型无效: {val}")

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

        默认圆筒目标优先。只有完全没有圆筒候选时，才用蓝色圆形标识兜底。
        quick_drop_mode 下由 drop_target_source="marker" 门控：只认蓝色标识。
        """
        if self.quick_bucket_enabled():
            return self.get_known_cylinders()
        if self.get_param("drop_target_source", "cylinder") == "marker":
            return self._dedupe_markers(self.get_known_markers())

        cylinders = self.get_known_cylinders()
        if cylinders:
            return cylinders

        return self._dedupe_markers(self.get_known_markers())

    def quick_bucket_enabled(self) -> bool:
        return (bool(self.get_param("quick_drop_mode", False))
                and self.get_param("quick_drop_profile", "blue") == "bucket")

    def _dedupe_markers(self, markers: list) -> list:
        """同一蓝色标识按世界坐标 0.6m 去重后返回."""
        targets = []
        for marker in markers:
            mx = float(marker.get("world_x", 0.0))
            my = float(marker.get("world_y", 0.0))
            duplicate = any(
                ((float(t.get("world_x", 0.0)) - mx) ** 2
                 + (float(t.get("world_y", 0.0)) - my) ** 2) ** 0.5 < 0.6
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
        self._quick_bucket_gate = None
        self._quick_release_intent = None

    def release_current_drop_target(self) -> None:
        """投放未执行或失败时释放当前选择, 允许重新搜索/重试."""
        self._cylinder_selector.release_last_selection()
        self._current_drop_cyl_id = -1
        self._quick_bucket_gate = None
        self._quick_release_intent = None

    def is_in_drop_zone_world(self, world_x: float, world_y: float, margin: float = 0.5) -> bool:
        return self._contains_drop_target_world_xy(
            world_x, world_y, margin=margin,
        )

    def get_hazards(self) -> list:
        """获取最新的危险品检测结果."""
        if time.time() - self._hazards_timestamp > 2.0:
            return []
        return self._hazards

    def get_landing_target(self) -> Optional[Dict]:
        """获取最新的 H 降落目标 (过去 2s 内有过一次真实检测).

        与 get_cylinders/get_markers/get_hazards 同形的龄期门. 判据取"距最后
        一次**含检测**的消息", 而不是"距最后一条消息" —— 后者会让相机/推理
        链路死亡时旧目标永不失效 (视觉节点对 miss 帧也发消息), 这正是记录在
        案的失效路径: 目标永久冻结, 而 watchdog 因帧流仍在而被喂活.

        权威口径同此: PX4 precland 以 landing_target_pose 的时间戳判龄
        (PLD_BTOUT), 且其估计器的 _last_update 只在成功融合/发布时刷新 ——
        "一串持续被门拒绝的测量"同样会走到复位 (LandingTargetEstimator.cpp
        :76-80,120,180).
        """
        if time.time() - self._landing_target_detection_stamp > 2.0:
            return None
        return self._landing_target

    def get_coverage(self) -> Optional[Dict]:
        return self._coverage_status

    # -- 服务调用 --
    def start_drop_service(self, cylinder_id: int) -> None:
        """
        发送投放请求, 通过 Future 回调获取真实结果.

        与 check_drop_service_result 配合:
          - Future 完成 → 读取真实 TriggerDrop.Response.success
          - Future 超时 → drop_confirmation_timeout 后视为失败
        """
        try:
            if not hasattr(self, '_drop_client') or self._drop_client is None:
                self.get_logger().error("舵机服务客户端未初始化")
                self._drop_service_failed = True
                return

            if not self._drop_client.service_is_ready():
                self.get_logger().error("舵机服务不可用")
                self._drop_service_failed = True
                return

            from drone_interfaces.srv import TriggerDrop

            self._drop_pending_cylinder_id = cylinder_id
            self._drop_result_consumed = False
            self._drop_service_failed = False
            self._drop_future_result: Optional[bool] = None

            req = TriggerDrop.Request()
            req.cylinder_id = cylinder_id
            self._drop_future = self._drop_client.call_async(req)
            self._drop_request_time = time.time()
            self._drop_sent_ok = True

        except Exception as e:
            self.get_logger().error(f"投放请求发送失败: {e}")
            self._drop_service_failed = True

    def check_drop_service_result(self) -> Optional[bool]:
        """
        确认投放结果 — Future 优先, 超时回退.

        Returns:
            True   — 投放成功 (servo 确认)
            False  — 投放失败 (servo 拒绝 / 硬件故障 / 超时)
            None   — 等待中, 下个 tick 再查
        """
        if self._drop_result_consumed:
            return None

        if self._drop_service_failed:
            self._drop_count += 1
            self._drop_result_consumed = True
            return False

        if not getattr(self, '_drop_sent_ok', False):
            return None

        # ── 优先读取真实 Future 结果 ──
        future = getattr(self, '_drop_future', None)
        if future is not None and future.done():
            self._drop_count += 1
            try:
                response = future.result()
                if response is not None and getattr(response, 'success', False):
                    self._drop_success_count += 1
                    self._drop_result_consumed = True
                    self._drop_sent_ok = False
                    self._drop_future = None
                    return True
            except Exception as e:
                self.get_logger().warn(f"投放 Future 异常: {e}")
            # 任何非成功路径 → 失败
            self._drop_result_consumed = True
            self._drop_sent_ok = False
            self._drop_future = None
            return False

        # ── Future 未完成 → 超时回退 ──
        timeout = self.get_param("drop_confirmation_timeout", 5.0)
        if time.time() - self._drop_request_time < timeout:
            return None

        self.get_logger().warn(
            f"投放确认超时 ({timeout}s), servo 未响应, 视为失败"
        )
        self._drop_count += 1
        self._drop_result_consumed = True
        self._drop_sent_ok = False
        return False

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
