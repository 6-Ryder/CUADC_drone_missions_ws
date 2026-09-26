"""
统一视觉管线 v2 — 单 YOLO 模型 + 姿态补偿 + 高度自适应.

核心改进 (v1 → v2):
  - 姿态补偿坐标变换: 用完整 6-DoF 位姿替代简化高度/焦距缩放
  - 高度自适应检测阈值: 根据飞行高度动态调整 conf/min_radius
  - 统一伺服接口: 同时支持降落和圆筒逼近模式
  - 帧率自适应: Jetson 上自动跳帧以保证实时性

发布:
  /drone/vision/cylinders      (CylinderArray)
  /drone/vision/hazards         (HazardArray)
  /drone/vision/landing_target  (LandingTarget)
  /drone/vision/coverage        (CoverageStatus)
  /drone/vision/diagnostics     (VisionDiagnostics) — 调试信息
"""

import os
import gc
import math
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from cv_bridge import CvBridge

from drone_interfaces.msg import (
    CylinderDetection, CylinderArray,
    HazardArray,
    LandingTarget, CoverageStatus,
)
from drone_interfaces.srv import SetVisionMode

from drone_vision.models.yolo_engine import Detection, create_engine, YOLOEngine
from drone_vision.cylinder_detector import (
    CylinderRefiner,
    CylinderTracker,
)
from drone_vision.marker_detector import BlueMarkerRefiner
from drone_vision.landing_detector import LandingCircleTracker, LandingPadRefiner
from drone_vision.coverage_monitor import CoverageMonitor
from drone_utils.layout_frame import LayoutFrame
from drone_vision.coordinate_transforms import CoordinateTransformer
from drone_vision.capture_pose import interpolate_pose
from collections import deque
from drone_vision.visual_servoing import (
    VisualServoController, CylinderApproachController, ServoState,
)


CAMERA_INFO_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

IMAGE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


FROZEN_LAYOUT_PARAMETERS = frozenset({
    "layout_heading_deg", "quick_drop_profile", "bucket_rim_height",
    "bucket_camera_forward", "bucket_camera_right", "bucket_camera_down",
    "bucket_camera_roll", "bucket_camera_pitch", "bucket_camera_yaw",
})


class VisionPipelineNode(Node):
    """
    统一视觉管线 v2 — 姿态补偿 + 高度自适应.

    关键状态:
      - _drone_pose: 实时位姿 (x, y, z, qx, qy, qz, qw)
      - _servo_mode: "none" | "landing" | "cylinder_approach"
      - _target_cylinder_id: 圆筒逼近时的目标 ID
    """

    def __init__(self):
        super().__init__("vision_pipeline")

        self._declare_params()
        self._bucket_projection = (
            self.get_parameter("quick_drop_profile").value == "bucket")
        self._layout_frame = LayoutFrame(
            float(self.get_parameter("layout_heading_deg").value)
        )
        self.add_on_set_parameters_callback(self._reject_layout_heading_update)

        # ---- 核心组件 (延迟加载) ----
        self._engine: Optional[YOLOEngine] = None
        self._drop_marker_engine: Optional[YOLOEngine] = None
        self._landing_engine: Optional[YOLOEngine] = None
        self._landing_model_loaded: bool = False
        self._bridge = CvBridge()

        # 坐标变换 + 姿态补偿
        self._coord_xform: Optional[CoordinateTransformer] = None

        # 后处理器
        self._cyl_refiner: Optional[CylinderRefiner] = None
        self._cyl_tracker: Optional[CylinderTracker] = None
        self._marker_refiner: Optional[BlueMarkerRefiner] = None
        self._landing_refiner: Optional[LandingPadRefiner] = None
        self._landing_circle_tracker: Optional[LandingCircleTracker] = None
        self._landing_tracker_diagnostic = None
        self._coverage: Optional[CoverageMonitor] = None

        # 伺服控制器
        self._landing_servo: Optional[VisualServoController] = None
        self._cylinder_servo: Optional[CylinderApproachController] = None
        self._servo_mode: str = "none"  # "landing" | "cylinder_approach" | "none"
        self._detection_mode: str = "all"  # "all" | "landing" | "idle"
        self._target_cylinder_id: int = -1
        self._models_validated = False
        self._detection_models_released = False

        # ---- 实时状态 ----
        self._drone_pose: Tuple[float, float, float, float, float, float, float] = (
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        )  # (x, y, z, qx, qy, qz, qw)
        self._pose_received = False
        self._last_pose_monotonic: Optional[float] = None
        self._current_altitude = 5.0
        self._current_yaw_ned = 0.0
        self._pose_history = deque(maxlen=100)
        self._frame_pose = None
        self._frame_stamp = None
        self._last_frame_time = time.time()
        self._landing_last_yolo_inference_monotonic = 0.0
        self._frame_count = 0
        self._skip_count = 0

        # ---- 订阅 ----
        if self.get_parameter("camera_compressed").value:
            self._img_sub = self.create_subscription(
                CompressedImage, "/drone/camera/image_compressed",
                self._image_cb, IMAGE_QOS)
        else:
            self._img_sub = self.create_subscription(
                Image, "/drone/camera/image_raw", self._image_cb,
                IMAGE_QOS)
        self._caminfo_sub = self.create_subscription(
            CameraInfo, "/drone/camera/camera_info", self._caminfo_cb,
            CAMERA_INFO_QOS,
        )
        # 订阅无人机位姿 (用于姿态补偿)
        self._pose_sub = self.create_subscription(
            PoseStamped, "/drone/pose", self._pose_cb, 10,
        )

        # ---- 发布 ----
        self._cyl_pub = self.create_publisher(
            CylinderArray, "/drone/vision/cylinders", 10,
        )
        self._hazard_pub = self.create_publisher(
            HazardArray, "/drone/vision/hazards", 10,
        )
        self._landing_pub = self.create_publisher(
            LandingTarget, "/drone/vision/landing_target", 10,
        )
        self._cov_pub = self.create_publisher(
            CoverageStatus, "/drone/vision/coverage", 10,
        )
        # 投放区标识检测 (V1 蓝色标识模型)
        self._marker_pub = self.create_publisher(
            CylinderArray, "/drone/vision/markers", 10,
        )
        # 视觉伺服速度指令 (供 px4_bridge 或外部节点消费, 转发至 PX4)
        self._servo_vel_pub = self.create_publisher(
            TwistStamped, "/drone/vision/servo_cmd", 10,
        )
        self._status_pub = self.create_publisher(
            String, "/drone/vision/status", CAMERA_INFO_QOS,
        )
        self._mode_service = self.create_service(
            SetVisionMode, "/drone/vision/set_mode", self._set_mode_cb,
        )

        # ---- 初始化 ----
        self._init_timer = self.create_timer(1.0, self._lazy_init)
        self._status_timer = self.create_timer(1.0, self._publish_status)
        self._initialized = False

        self.get_logger().info("统一视觉管线 v2 已创建 (姿态补偿 + 高度自适应)")

    # ================================================================
    # 参数声明
    # ================================================================

    def _declare_params(self):
        # YOLO
        self.declare_parameter("model_path", "")
        self.declare_parameter("drop_marker_model_path", "")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("nms_iou_threshold", 0.45)
        self.declare_parameter("model_width", 640)
        self.declare_parameter("model_height", 640)
        self.declare_parameter("device", "cuda")
        self.declare_parameter("max_inference_fps", 30.0)
        self.declare_parameter("camera_compressed", False)
        self.declare_parameter("quick_drop_profile", "blue")
        self.declare_parameter("bucket_rim_height", 0.3)
        self.declare_parameter("bucket_camera_forward", 0.0)
        self.declare_parameter("bucket_camera_right", 0.0)
        self.declare_parameter("bucket_camera_down", 0.0)
        self.declare_parameter("bucket_camera_roll", 0.0)
        self.declare_parameter("bucket_camera_pitch", 0.0)
        self.declare_parameter("bucket_camera_yaw", 0.0)
        # 类别名 (YAML 配置, 必须与模型输出类别数一致)
        self.declare_parameter("class_names", ["cylinder"])
        self.declare_parameter("drop_marker_class_names", ["drop_zone_marker"])
        # 投放区蓝色标识 OpenCV 精修
        self.declare_parameter("drop_marker_refine_enable", True)
        self.declare_parameter("drop_marker_min_blue_ratio", 0.4)
        self.declare_parameter("drop_marker_min_area", 80)
        self.declare_parameter("drop_marker_min_aspect_ratio", 0.4)
        self.declare_parameter("drop_marker_min_circularity", 0.35)
        # B1: 低空下降段放宽门槛 (运动模糊误杀 → 目标断裂)
        self.declare_parameter("drop_marker_descend_min_blue_ratio", 0.3)
        self.declare_parameter("drop_marker_descend_min_circularity", 0.25)
        self.declare_parameter("drop_marker_descend_alt_threshold", 4.5)
        self.declare_parameter("drop_marker_min_diameter", 30)
        # 尺寸一致性过滤开关 (默认关闭 = 只信 yolo+blue 高置信目标).
        # 开启后按当前高度推算 1m 标识理论投影直径, 用下面两个比例窗口拒绝
        # 尺寸异常的目标; 两者仅在开关为 True 时被读取.
        self.declare_parameter("drop_marker_filter_by_diameter_enable", False)
        self.declare_parameter("drop_marker_min_scale_ratio", 0.45)
        self.declare_parameter("drop_marker_max_scale_ratio", 1.8)
        # 相机
        self.declare_parameter("fx", 400.0)
        self.declare_parameter("fy", 400.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        # True=画面顶朝机头 (真机), False=gz SITL 相机 (画面底朝机头)
        self.declare_parameter("image_top_faces_forward", True)
        self.declare_parameter("vision_pose_max_age_s", 0.5)
        # 圆筒
        self.declare_parameter("cylinder_refine_enable", True)
        self.declare_parameter("cylinder_edge_blur", 5)
        self.declare_parameter("cylinder_canny_low", 50)
        self.declare_parameter("cylinder_canny_high", 150)
        self.declare_parameter("cylinder_min_radius_ratio", 0.3)
        self.declare_parameter("cylinder_max_radius_ratio", 1.2)
        self.declare_parameter("cylinder_white_brightness", 120)
        self.declare_parameter("cylinder_tracking_max_age", 10)
        self.declare_parameter("cylinder_tracking_iou_thresh", 0.3)
        self.declare_parameter("cylinder_tracking_min_hits", 3)
        # 降落
        self.declare_parameter("landing_refine_enable", True)
        self.declare_parameter("landing_adaptive_thresh_block", 11)
        self.declare_parameter("landing_circle_min_area", 80)
        self.declare_parameter("landing_H_orientation_enable", True)
        self.declare_parameter("landing_model_path", "landing_h_yolo.onnx")
        self.declare_parameter("landing_class_names", ["landing_H"])
        self.declare_parameter("landing_circle_track_enable", False)
        self.declare_parameter("landing_circle_tracker_roi_padding_px", 48)
        self.declare_parameter("landing_circle_tracker_min_edge_margin_px", 12.0)
        self.declare_parameter("landing_circle_tracker_max_center_jump_px", 80.0)
        self.declare_parameter("landing_circle_tracker_max_radius_change_ratio", 0.35)
        self.declare_parameter("landing_yolo_guard_rate_hz", 2.0)
        # 覆盖 — 侦察区场地局部坐标 (forward/right)
        self.declare_parameter("coverage_zone_forward_min", 55.0)
        self.declare_parameter("coverage_zone_forward_max", 60.0)
        self.declare_parameter("coverage_zone_right_min", -4.0)
        self.declare_parameter("coverage_zone_right_max", 4.0)
        self.declare_parameter("home_x", 4.0)
        self.declare_parameter("home_y", 10.0)
        self.declare_parameter("layout_heading_deg", 0.0)
        self.declare_parameter("coverage_grid_resolution", 0.5)
        self.declare_parameter("coverage_fov_half_angle", 35.0)
        self.declare_parameter("coverage_complete_ratio", 0.95)
        # 伺服
        self.declare_parameter("servo_kp", 0.5)
        self.declare_parameter("servo_ki", 0.02)
        self.declare_parameter("servo_kd", 0.1)
        self.declare_parameter("servo_max_horizontal_vel", 1.0)
        self.declare_parameter("servo_descent_speed", 0.3)
        self.declare_parameter("servo_integral_max", 0.5)
        self.declare_parameter("servo_target_deadzone", 0.03)

    def _reject_layout_heading_update(self, params):
        """Keep coverage and mission geometry on one launch-time heading."""
        if any(param.name in FROZEN_LAYOUT_PARAMETERS for param in params):
            return SetParametersResult(
                successful=False,
                reason="layout geometry is fixed at mission startup",
            )
        return SetParametersResult(successful=True)

    # ================================================================
    # 初始化
    # ================================================================

    def _lazy_init(self):
        if self._initialized:
            return

        # --- 坐标变换 ---
        fx = self.get_parameter("fx").value
        fy = self.get_parameter("fy").value
        cx = self.get_parameter("cx").value
        cy = self.get_parameter("cy").value
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        self._coord_xform = CoordinateTransformer(
            K,
            image_top_faces_forward=self.get_parameter(
                "image_top_faces_forward"
            ).value,
            camera_body_offset=(tuple(float(self.get_parameter(name).value)
                                      for name in ("bucket_camera_forward",
                                                   "bucket_camera_right",
                                                   "bucket_camera_down"))
                                if self._bucket_projection else (0.0, 0.0, 0.0)),
            camera_mount_rpy=(tuple(float(self.get_parameter(name).value)
                                    for name in ("bucket_camera_roll",
                                                 "bucket_camera_pitch",
                                                 "bucket_camera_yaw"))
                              if self._bucket_projection else (0.0, 0.0, 0.0)),
        )

        # --- 后处理器 ---
        self._cyl_refiner = CylinderRefiner(
            blur_ksize=self.get_parameter("cylinder_edge_blur").value,
            canny_low=self.get_parameter("cylinder_canny_low").value,
            canny_high=self.get_parameter("cylinder_canny_high").value,
            min_radius_ratio=self.get_parameter("cylinder_min_radius_ratio").value,
            max_radius_ratio=self.get_parameter("cylinder_max_radius_ratio").value,
            white_brightness=self.get_parameter("cylinder_white_brightness").value,
        )
        self._cyl_tracker = CylinderTracker(
            max_age=self.get_parameter("cylinder_tracking_max_age").value,
            iou_thresh=self.get_parameter("cylinder_tracking_iou_thresh").value,
            min_hits=self.get_parameter("cylinder_tracking_min_hits").value,
        )
        self._marker_refiner = BlueMarkerRefiner(
            min_component_area=self.get_parameter(
                "drop_marker_min_area"
            ).value,
            min_detection_blue_ratio=self.get_parameter(
                "drop_marker_min_blue_ratio"
            ).value,
            min_aspect_ratio=self.get_parameter(
                "drop_marker_min_aspect_ratio"
            ).value,
            min_circularity=self.get_parameter(
                "drop_marker_min_circularity"
            ).value,
            min_component_diameter=self.get_parameter(
                "drop_marker_min_diameter"
            ).value,
        )
        self._marker_refine_thresholds_applied: Optional[tuple[float, float]] = None
        self._landing_refiner = LandingPadRefiner(
            adaptive_thresh_block=self.get_parameter("landing_adaptive_thresh_block").value,
            circle_min_area=self.get_parameter("landing_circle_min_area").value,
        )
        self._landing_circle_tracker = LandingCircleTracker(
            adaptive_thresh_block=self.get_parameter(
                "landing_adaptive_thresh_block"
            ).value,
            circle_min_area=self.get_parameter("landing_circle_min_area").value,
            roi_padding_px=self.get_parameter(
                "landing_circle_tracker_roi_padding_px"
            ).value,
            min_edge_margin_px=self.get_parameter(
                "landing_circle_tracker_min_edge_margin_px"
            ).value,
            max_center_jump_px=self.get_parameter(
                "landing_circle_tracker_max_center_jump_px"
            ).value,
            max_radius_change_ratio=self.get_parameter(
                "landing_circle_tracker_max_radius_change_ratio"
            ).value,
        )
        self._coverage = CoverageMonitor(
            zone_forward_range=(
                self.get_parameter("coverage_zone_forward_min").value,
                self.get_parameter("coverage_zone_forward_max").value,
            ),
            zone_right_range=(
                self.get_parameter("coverage_zone_right_min").value,
                self.get_parameter("coverage_zone_right_max").value,
            ),
            layout_frame=self._layout_frame,
            home_world_xy=(
                self.get_parameter("home_x").value,
                self.get_parameter("home_y").value,
            ),
            grid_resolution=self.get_parameter("coverage_grid_resolution").value,
            fov_half_angle_deg=self.get_parameter("coverage_fov_half_angle").value,
            complete_ratio=self.get_parameter("coverage_complete_ratio").value,
        )

        # --- 伺服控制器 ---
        self._landing_servo = VisualServoController(
            kp=self.get_parameter("servo_kp").value,
            ki=self.get_parameter("servo_ki").value,
            kd=self.get_parameter("servo_kd").value,
            max_horizontal_vel=self.get_parameter("servo_max_horizontal_vel").value,
            descent_speed=self.get_parameter("servo_descent_speed").value,
            integral_max=self.get_parameter("servo_integral_max").value,
            deadzone=self.get_parameter("servo_target_deadzone").value,
        )
        self._cylinder_servo = CylinderApproachController(
            kp=0.8, ki=0.03, kd=0.15,
            max_horizontal_vel=1.5, deadzone=0.05,
        )

        # Both task models are mandatory. A live process without them is not healthy.
        model_path = self._resolve_model_path(
            self.get_parameter("model_path").value,
        )
        marker_model_path = self._resolve_model_path(
            self.get_parameter("drop_marker_model_path").value,
        )
        common = {
            "conf_threshold": self.get_parameter("conf_threshold").value,
            "nms_iou_threshold": self.get_parameter("nms_iou_threshold").value,
            "input_size": (
                self.get_parameter("model_width").value,
                self.get_parameter("model_height").value,
            ),
            "device": self.get_parameter("device").value,
        }
        self._engine = create_engine(
            model_path=model_path,
            class_names=list(self.get_parameter("class_names").value),
            **common,
        )
        self._drop_marker_engine = create_engine(
            model_path=marker_model_path,
            class_names=list(self.get_parameter("drop_marker_class_names").value),
            **common,
        )
        if not self._engine.load():
            raise RuntimeError(f"圆筒 YOLO 模型加载失败: {model_path}")
        if not self._drop_marker_engine.load():
            raise RuntimeError(f"蓝色标识 YOLO 模型加载失败: {marker_model_path}")
        self._models_validated = True
        self.get_logger().info(f"圆筒 YOLO 已加载: {model_path}")
        self.get_logger().info(f"蓝色标识 YOLO 已加载: {marker_model_path}")

        self._initialized = True
        self._init_timer.cancel()
        self.get_logger().info(
            "统一视觉管线 v2 初始化完成 (姿态补偿已启用)"
        )

    @staticmethod
    def _resolve_model_path(configured_path: str) -> str:
        if not configured_path:
            raise RuntimeError("视觉模型路径为空")
        path = configured_path
        if not os.path.isabs(path):
            path = os.path.join(os.path.dirname(__file__), "models", path)
        if not os.path.isfile(path):
            raise RuntimeError(f"视觉模型不存在: {path}")
        return path

    # ================================================================
    # 伺服模式切换 (供 mission 节点通过 Service 调用)
    # ================================================================

    def _set_mode_cb(self, request, response):
        if not request.enable:
            self._detection_mode = "idle"
            self.set_servo_mode("none")
            if self._coverage:
                self._coverage.end_scan()
            self._release_detection_models()
        elif request.mode == 2:
            self._detection_mode = "landing"
            # B4 (2026-08-07): 不再激活内嵌伺服。视觉管线伺服与 FSM/test_mission
            # 的像素伺服会同时写入 px4_bridge 速度缓存互相抵消 —— 8/7 真机
            # H 降落漂移丢失的直接根因(视觉伺服按"图像下方=机头"的下视相机
            # 约定, 前视下倾相机下纵向符号反向, 见 PLAN_visual_servo_optimization.md B4)。
            # 伺服由任务节点独占, 本管线只负责检测与 landing_target 发布。
            self.set_servo_mode("none")
            self._ensure_landing_engine()
            self._release_detection_models()
        elif request.mode in (0, 3):
            if self._detection_models_released:
                response.success = False
                response.message = "投放检测模型已在侦察阶段释放"
                return response
            self._detection_mode = "all"
            self.set_servo_mode("none")
            if self._coverage:
                self._coverage.end_scan()
        elif request.mode == 6:
            # quick 模式: 只跑蓝底标识模型, 投放区检测 FPS 翻倍
            if self._detection_models_released:
                response.success = False
                response.message = "投放检测模型已在侦察阶段释放"
                return response
            self._detection_mode = "marker_only"
            self.set_servo_mode("none")
            if self._coverage:
                self._coverage.end_scan()
        elif request.mode == 4:
            self._detection_mode = "idle"
            self.set_servo_mode("none")
            self._release_detection_models()
            if self._coverage:
                self._coverage.reset()
        elif request.mode == 5:
            self._detection_mode = "idle"
            self.set_servo_mode("none")
            if self._coverage:
                self._coverage.begin_scan()
        else:
            response.success = False
            response.message = f"不支持的视觉模式: {request.mode}"
            return response
        response.success = True
        response.message = self._detection_mode
        return response

    def _ensure_landing_engine(self) -> bool:

        if self._landing_model_loaded:
            return self._landing_engine is not None and self._landing_engine.is_loaded

        self._landing_model_loaded = True

        try:
            landing_path = self._resolve_model_path(
                self.get_parameter("landing_model_path").value,
            )
        except RuntimeError as e:
            self.get_logger().warn(
                f"降落模型路径无效: {e}; 回退到OpenCV检测"
            )
            return False

        try:
            self._landing_engine = create_engine(
                model_path=landing_path,
                conf_threshold=self.get_parameter("conf_threshold").value,
                nms_iou_threshold=self.get_parameter("nms_iou_threshold").value,
                input_size=(
                    self.get_parameter("model_width").value,
                    self.get_parameter("model_height").value,
                ),
                class_names=list(
                    self.get_parameter("landing_class_names").value
                ),
                device=self.get_parameter("device").value,
            )
        except Exception as e:
            self.get_logger().error(f"降落引擎创建失败: {e}")
            self._landing_engine = None
            return False

        if not self._landing_engine.load():
            self.get_logger().warn(
                "降落模型加载失败; 回退到OpenCV全图H标识搜索"
            )
            self._landing_engine = None
            return False

        self.get_logger().info(f"降落H标识YOLO已加载: {landing_path}")
        return True

    def _infer_landing(self, image: np.ndarray) -> List[Detection]:

        if self._landing_engine is None:
            self._ensure_landing_engine()

        if self._landing_engine is not None and self._landing_engine.is_loaded:
            try:
                return self._landing_engine.infer(image)
            except Exception as e:
                self.get_logger().error(f"降落引擎推理异常: {e}")
        return []

    def _release_detection_models(self):
        if self._detection_models_released:
            return
        for engine in (self._engine, self._drop_marker_engine):
            if engine is not None:
                engine.close()
        self._engine = None
        self._drop_marker_engine = None
        self._detection_models_released = True
        gc.collect()
        self.get_logger().info("投放检测模型已释放")

    def _publish_status(self):
        ready = self._initialized and self._models_validated and self._frame_count > 0
        msg = String()
        msg.data = (
            f"ready={int(ready)} models={int(self._models_validated)} "
            f"frames={self._frame_count} mode={self._detection_mode}"
        )
        self._status_pub.publish(msg)

    def set_servo_mode(self, mode: str, cylinder_id: int = -1):
        """
        切换伺服模式.

        Parameters
        ----------
        mode : str
            "landing" | "cylinder_approach" | "none".
        cylinder_id : int
            圆筒逼近时的目标 ID.

        """
        if mode not in ("landing", "cylinder_approach", "none"):
            self.get_logger().warn(f"未知伺服模式: {mode}")
            return

        old_mode = self._servo_mode
        self._servo_mode = mode
        self._target_cylinder_id = cylinder_id

        if mode == "landing" and self._landing_servo:
            self._landing_servo.reset()
        elif mode == "cylinder_approach" and self._cylinder_servo:
            self._cylinder_servo.activate(cylinder_id)
        elif mode == "none":
            if self._landing_servo:
                self._landing_servo.reset()
            if self._cylinder_servo:
                self._cylinder_servo.deactivate()

        self.get_logger().info(f"伺服模式: {old_mode} → {mode} (cyl_id={cylinder_id})")

    def get_servo_state(self) -> Optional[ServoState]:
        """获取当前伺服状态."""
        if self._servo_mode == "landing" and self._landing_servo:
            # 返回最后计算的状态
            return ServoState(
                horizontal_error=self._landing_servo._error_history[-1]
                if self._landing_servo._error_history else 0.0,
                is_converged=self._landing_servo._is_stable(),
            )
        elif self._servo_mode == "cylinder_approach" and self._cylinder_servo:
            return ServoState(
                horizontal_error=self._cylinder_servo.horizontal_error,
                is_converged=self._cylinder_servo.is_converged,
            )
        return None

    def _caminfo_cb(self, msg: CameraInfo):
        """从 CameraInfo 更新内参."""
        if msg.k[0] > 0 and self._coord_xform is not None:
            self._coord_xform.update_intrinsics(msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _pose_cb(self, msg: PoseStamped):
        """接收无人机位姿 (用于姿态补偿坐标变换)."""
        pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        quaternion_norm_sq = sum(value * value for value in pose[3:])
        if (
            not all(math.isfinite(value) for value in pose)
            or quaternion_norm_sq <= 1e-12
        ):
            self._pose_received = False
            self._last_pose_monotonic = None
            if hasattr(self, "_pose_history"):
                self._pose_history.clear()
            self.get_logger().warn("VISION: 忽略无效无人机位姿")
            return

        self._drone_pose = pose
        self._pose_received = True
        self._last_pose_monotonic = time.monotonic()
        pose_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (math.isfinite(pose_stamp) and pose_stamp > 0
                and hasattr(self, "_pose_history")
                and (not self._pose_history or pose_stamp > self._pose_history[-1][0])):
            self._pose_history.append((pose_stamp, pose))
        self._current_altitude = msg.pose.position.z
        q = msg.pose.orientation
        world_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._current_yaw_ned = math.atan2(
            math.sin(math.pi / 2.0 - world_yaw),
            math.cos(math.pi / 2.0 - world_yaw),
        )

    def _pose_is_fresh(self, now_monotonic: Optional[float] = None) -> bool:
        """Whether a valid 6-DoF pose is available for World projection."""
        if not self._pose_received or self._last_pose_monotonic is None:
            return False
        now = time.monotonic() if now_monotonic is None else now_monotonic
        max_age = float(self.get_parameter("vision_pose_max_age_s").value)
        return 0.0 <= now - self._last_pose_monotonic <= max_age

    def _apply_marker_refine_thresholds(self) -> None:
        """B1: 低空下降段放宽蓝验证门槛, 避免运动模糊误杀致目标断裂.

        触发条件: 0 < 高度 < drop_marker_descend_alt_threshold (默认 4.5m).
        8/14 晚卡死架次丢失高度 3.15-3.75m, 阈值须覆盖该区间.
        """
        alt = self._current_altitude
        descend_thr = float(self.get_parameter(
            "drop_marker_descend_alt_threshold").value)
        relaxed = 0.0 < alt < descend_thr
        blue_ratio = float(self.get_parameter(
            "drop_marker_descend_min_blue_ratio" if relaxed
            else "drop_marker_min_blue_ratio").value)
        circularity = float(self.get_parameter(
            "drop_marker_descend_min_circularity" if relaxed
            else "drop_marker_min_circularity").value)
        applied = (blue_ratio, circularity)
        if applied != self._marker_refine_thresholds_applied:
            self._marker_refiner.set_thresholds(blue_ratio, circularity)
            self._marker_refine_thresholds_applied = applied
            self.get_logger().info(
                "DROP_MARKER: refine 门槛切换 -> "
                f"{'放宽 (下降段)' if relaxed else '标定'} "
                f"blue_ratio={blue_ratio} circularity={circularity} "
                f"alt={alt:.1f}m"
            )

    def _project_world_xy(
        self, u: float, v: float, ground_z: float = 0.0,
    ) -> Optional[tuple[float, float]]:
        """Project a pixel using only a fresh full 6-DoF pose."""
        if self._coord_xform is None:
            return None
        if getattr(self, "_bucket_projection", False):
            pose = getattr(self, "_frame_pose", None)
            if pose is None:
                return None
        else:
            if not self._pose_is_fresh():
                return None
            pose = self._drone_pose
        result = self._coord_xform.pixel_to_world(
            u, v, pose[:3], pose[3:7],
            ground_z=ground_z, q_convention="xyzw",
        )
        if result is None:
            return None
        offset = math.hypot(
            result[0] - pose[0], result[1] - pose[1],
        )
        return result if offset <= max(pose[2] * 1.5, 3.0) else None

    def _new_cylinder_array(self) -> CylinderArray:
        msg = CylinderArray()
        frame_stamp = getattr(self, "_frame_stamp", None)
        msg.header.stamp = (frame_stamp if frame_stamp is not None
                            else self.get_clock().now().to_msg())
        msg.header.frame_id = "world"
        return msg

    # ================================================================
    # 主回调
    # ================================================================

    def _image_cb(self, msg: Image):
        if not self._initialized:
            return

        # Mission 控制周期是 20Hz；处理更快只会增加 GPU 压力。
        self._frame_count += 1
        t_elapsed = time.time() - self._last_frame_time
        max_fps = max(float(self.get_parameter("max_inference_fps").value), 1.0)
        min_interval = 1.0 / max_fps
        if t_elapsed < min_interval:
            self._skip_count += 1
            if self._skip_count % 100 == 0:
                self.get_logger().debug(f"跳帧 {self._skip_count}/{self._frame_count}")
            return
        self._last_frame_time = time.time()
        self._frame_stamp = msg.header.stamp
        if getattr(self, "_bucket_projection", False):
            image_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._frame_pose = interpolate_pose(self._pose_history, image_stamp)
        else:
            self._frame_pose = None

        t0 = time.time()
        try:
            image = (self._bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                     if isinstance(msg, CompressedImage)
                     else self._bridge.imgmsg_to_cv2(msg, "bgr8"))
        except Exception:
            return

        # 目标模型只在投放阶段运行。两模型顺序执行，避免无收益的 GPU 并发峰值。
        # marker_only (quick 模式): 只跑蓝底标识模型, 检测 FPS 翻倍。
        all_dets: List[Detection] = []
        marker_dets: List[Detection] = []
        marker_source = "none"
        if self._detection_mode in ("all", "marker_only"):
            if self._detection_mode == "all":
                all_dets = self._engine.infer(image)
            if self._drop_marker_engine is not None:
                marker_dets = self._drop_marker_engine.infer(image)
                marker_source = "yolo"
                if (
                    self.get_parameter("drop_marker_refine_enable").value
                    and self._marker_refiner
                ):
                    self._apply_marker_refine_thresholds()
                    marker_dets = self._marker_refiner.refine(image, marker_dets)
                    marker_source = "yolo+blue"
                    if self.get_parameter(
                        "drop_marker_filter_by_diameter_enable"
                    ).value:
                        # 尺寸一致性过滤: 按当前高度推算 1m 标识的理论投影直径,
                        # 拒绝比例超出窗口的蓝色连通域 (地面杂色/尺度异常误检).
                        # 关闭 (默认): 只信 yolo+blue 高置信目标.
                        # 开启: 额外要求尺寸与高度假设一致.
                        expected_diameter = (
                            (
                                self.get_parameter("fx").value
                                + self.get_parameter("fy").value
                            )
                            * 0.5
                            / max(self._current_altitude, 0.5)
                        )
                        marker_dets = (
                            self._marker_refiner.filter_by_expected_diameter(
                                marker_dets,
                                expected_diameter,
                                self.get_parameter(
                                    "drop_marker_min_scale_ratio"
                                ).value,
                                self.get_parameter(
                                    "drop_marker_max_scale_ratio"
                                ).value,
                            )
                        )

        # 发布投放区标识检测
        if self._marker_refiner:
            marker_array = self._new_cylinder_array()
            for i, md in enumerate(marker_dets):
                det = CylinderDetection()
                det.id = i
                det.confidence = float(md.confidence)
                det.center_u = float(md.cx)
                det.center_v = float(md.cy)
                det.world_position_valid = False
                det.world_x = math.nan
                det.world_y = math.nan
                det.radius = 0.5  # 蓝色标识固定 1m 直径, 半径 0.5m
                det.bbox_x1 = int(md.bbox[0])
                det.bbox_y1 = int(md.bbox[1])
                det.bbox_x2 = int(md.bbox[2])
                det.bbox_y2 = int(md.bbox[3])

                selected_xy = self._project_world_xy(md.cx, md.cy)

                if selected_xy is not None and all(
                    math.isfinite(value) for value in selected_xy
                ):
                    det.world_x = float(selected_xy[0])
                    det.world_y = float(selected_xy[1])
                    det.world_position_valid = True
                marker_array.cylinders.append(det)
            self._marker_pub.publish(marker_array)
            if marker_dets and self._frame_count % 30 == 0:
                self.get_logger().info(
                    "DROP_MARKER: "
                    f"n={len(marker_dets)} "
                    f"source={marker_source} "
                    f"conf={marker_dets[0].confidence:.2f}"
                )
            all_dets.extend(marker_dets)

        # 根据任务阶段筛选检测。
        if self._detection_mode == "landing":
            cyl_dets = []
            now_monotonic = time.monotonic()
            guard_rate_hz = float(self.get_parameter(
                "landing_yolo_guard_rate_hz"
            ).value)
            if (
                not self._circle_tracking_active()
                or self._landing_yolo_inference_due(now_monotonic, guard_rate_hz)
            ):
                landing_dets = self._infer_landing(image)
                self._landing_last_yolo_inference_monotonic = now_monotonic
            else:
                landing_dets = []
        elif self._detection_mode == "all":
            cyl_dets = [
                d for d in all_dets
                if d.class_name in ("cylinder", "empty_cylinder")
            ]
            landing_dets = [d for d in all_dets if d.class_name == "landing_H"]
        elif self._detection_mode == "marker_only":
            # quick 模式: 只发布蓝底标识, 圆筒/H 检测全关
            cyl_dets = []
            landing_dets = []
        else:
            cyl_dets = []
            landing_dets = []

        # ---- 2. 圆筒检测 + OpenCV 精修 (姿态补偿坐标) ----
        self._process_cylinders_v2(image, cyl_dets)

        # ---- 3. H 降落标识 + 姿态补偿 + 伺服 ----
        self._process_landing_v2(image, landing_dets)

        # ---- 4. 覆盖监控 ----
        self._process_coverage()

        # FPS 监控
        dt = time.time() - t0
        if self._frame_count % 100 == 0:
            self.get_logger().info(
                f"FPS: {1/dt:.0f} | 耗时: {dt*1000:.0f}ms | "
                f"高度: {self._current_altitude:.1f}m | 模式: {self._servo_mode}"
            )

    # ================================================================
    # 圆筒处理 v2 (姿态补偿坐标)
    # ================================================================

    def _process_cylinders_v2(self, image: np.ndarray, dets: List[Detection]):
        refined: List[tuple] = []
        enable_refine = self.get_parameter("cylinder_refine_enable").value

        for d in dets:
            bbox_px = (
                int(d.bbox[0]), int(d.bbox[1]),
                int(d.bbox[2]), int(d.bbox[3]),
            )
            if enable_refine and self._cyl_refiner:
                result = self._cyl_refiner.refine(image, d.bbox)
                if result:
                    refined_cx, refined_cy, radius_px, _ = result
                    refined.append((
                        refined_cx,
                        refined_cy,
                        radius_px,
                        d.confidence,
                        bbox_px,
                    ))
                elif d.confidence > 0.7:
                    refined.append((d.cx, d.cy,
                                    min(d.bbox[2] - d.bbox[0], d.bbox[3] - d.bbox[1]) / 4,
                                    d.confidence * 0.7, bbox_px))
            else:
                refined.append((d.cx, d.cy,
                                min(d.bbox[2] - d.bbox[0], d.bbox[3] - d.bbox[1]) / 4,
                                d.confidence, bbox_px))

        if not refined:
            self._cyl_pub.publish(self._new_cylinder_array())
            return

        self._cyl_tracker.update(refined)
        self._cyl_tracker.prune()
        active = self._cyl_tracker.get_active()

        bucket_mode = getattr(self, "_bucket_projection", False)
        fx = self.get_parameter("fx").value
        fy = self.get_parameter("fy").value
        if bucket_mode and self._coord_xform is not None:
            fx = float(self._coord_xform.K[0, 0])
            fy = float(self._coord_xform.K[1, 1])
        rim_height = (float(self.get_parameter("bucket_rim_height").value)
                      if bucket_mode else 0.0)
        pose_altitude = (self._frame_pose[2]
                         if bucket_mode and self._frame_pose
                         else self._current_altitude)
        alt = max(pose_altitude - rim_height, 0.5)

        msg_array = self._new_cylinder_array()

        for t in active:
            det = CylinderDetection()
            det.id = t["id"]
            det.confidence = float(t["confidence"])
            det.center_u = float(t["cx"])
            det.center_v = float(t["cy"])
            det.world_position_valid = False
            det.world_x = math.nan
            det.world_y = math.nan
            # YOLO bbox (用于调试可视化)
            bbox = t.get("bbox")
            if bbox and len(bbox) == 4:
                det.bbox_x1, det.bbox_y1, det.bbox_x2, det.bbox_y2 = bbox

            # World coordinates require a fresh full 6-DoF pose; pixels stay usable.
            selected_xy = self._project_world_xy(t["cx"], t["cy"], rim_height)

            if selected_xy is not None and all(
                math.isfinite(value) for value in selected_xy
            ):
                det.world_x = float(selected_xy[0])
                det.world_y = float(selected_xy[1])
                det.world_position_valid = True

            # 真实半径 (像素→米)
            det.radius = t["radius"] * alt / max(fx, fy)

            msg_array.cylinders.append(det)

        self._cyl_pub.publish(msg_array)

    # ================================================================
    # 降落处理 v2 (姿态补偿 + 伺服)
    # ================================================================

    def _landing_yolo_inference_due(
        self,
        now_monotonic: float,
        guard_rate_hz: float,
    ) -> bool:
        """Keep full-frame YOLO active until a track exists, then use a guard rate."""
        if (
            self._landing_circle_tracker is None
            or not self._landing_circle_tracker.has_track
        ):
            return True
        guard_period = 1.0 / max(guard_rate_hz, 0.1)
        return (
            now_monotonic - self._landing_last_yolo_inference_monotonic
            >= guard_period
        )

    def _circle_tracking_active(self) -> bool:
        """Limit the quick circle tracker strictly to the landing phase."""
        return (
            self._detection_mode == "landing"
            and bool(self.get_parameter("landing_circle_track_enable").value)
        )

    def _select_landing_circle_track(
        self,
        image: np.ndarray,
        dets: List[Detection],
    ):
        """Choose a refined YOLO seed or a continuous bounded CV circle track."""
        if self._landing_refiner is None or self._landing_circle_tracker is None:
            return None, "none"

        if dets:
            best = max(dets, key=lambda detection: detection.confidence)
            refined = self._landing_refiner.refine(image, best.bbox)
            if refined is not None:
                track = self._landing_circle_tracker.seed(image, best.bbox)
                if track is not None:
                    return track, "yolo_refined"

        track = self._landing_circle_tracker.track(image)
        if track is not None:
            return track, "cv_tracked"
        return None, "none"

    def _select_landing_legacy_target(
        self,
        image: np.ndarray,
        dets: List[Detection],
    ):
        """Return the pre-existing YOLO/OpenCV target as a safe fallback.
        Circle tracking is an enhancement, never evidence that the landing
        target disappeared.  This preserves the established quick landing
        behaviour whenever the new geometric tracker cannot fit a frame.
        """
        if not dets:
            result = self._landing_refiner.detect(image) if self._landing_refiner else None
            if result is None:
                return None, "none"
            return result, "opencv_global_fallback"

        best = max(dets, key=lambda detection: detection.confidence)
        enable_refine = self.get_parameter("landing_refine_enable").value
        if enable_refine and self._landing_refiner:
            result = self._landing_refiner.refine(image, best.bbox)
            if result is not None:
                cx_px, cy_px, confidence, yaw = result
                return (
                    cx_px,
                    cy_px,
                    max(best.confidence, confidence),
                    yaw,
                ), "yolo_refined_fallback"
        return (best.cx, best.cy, best.confidence, 0.0), "yolo_bbox_fallback"

    def _process_landing_v2(self, image: np.ndarray, dets: List[Detection]):
        target = LandingTarget()
        target.confidence = 0.0

        if self._circle_tracking_active():
            track, source = self._select_landing_circle_track(image, dets)
            if track is None:
                reason = (
                    self._landing_circle_tracker.last_rejection_reason
                    if self._landing_circle_tracker is not None
                    else "uninitialized"
                )
                # Tracking may fail for a perspective-distorted ring.  Retry
                # YOLO immediately and retain the original landing detector
                # instead of publishing a false "target lost" sample.
                if not dets:
                    dets = self._infer_landing(image)
                    self._landing_last_yolo_inference_monotonic = time.monotonic()
                fallback, source = self._select_landing_legacy_target(image, dets)
                if fallback is None:
                    diagnostic = ("rejected", reason, len(dets))
                    if diagnostic != self._landing_tracker_diagnostic:
                        self.get_logger().warn(
                            "LANDING_CIRCLE: rejected "
                            f"reason={reason} yolo_detections={len(dets)}"
                        )
                    self._landing_tracker_diagnostic = diagnostic
                    self._landing_pub.publish(target)
                    return
                target.center_u, target.center_v, target.confidence, target.yaw_correction = fallback
                diagnostic = ("fallback", reason, source)
                if diagnostic != self._landing_tracker_diagnostic:
                    self.get_logger().warn(
                        "LANDING_CIRCLE: rejected "
                        f"reason={reason}, fallback={source}"
                    )
                self._landing_tracker_diagnostic = diagnostic
            else:
                target.center_u = track.center_u
                target.center_v = track.center_v
                target.confidence = track.confidence
                target.yaw_correction = 0.0
                diagnostic = ("tracking", source)
                if diagnostic != self._landing_tracker_diagnostic:
                    self.get_logger().info(
                        "LANDING_CIRCLE: "
                        f"source={source} conf={track.confidence:.2f} "
                        f"r={track.radius_px:.0f}px margin={track.edge_margin_px:.0f}px "
                        f"residual={track.fit_residual:.3f}"
                    )
                self._landing_tracker_diagnostic = diagnostic
        else:
            fallback, source = self._select_landing_legacy_target(image, dets)
            if fallback is None:
                self._landing_pub.publish(target)
                return
            target.center_u, target.center_v, target.confidence, target.yaw_correction = fallback

        # ---- 姿态补偿坐标变换 ----
        if self._coord_xform:
            world_xy = self._coord_xform.pixel_to_world(
                target.center_u, target.center_v,
                (self._drone_pose[0], self._drone_pose[1], self._drone_pose[2]),
                self._drone_pose[3:7],
                ground_z=0.0, q_convention="xyzw",
            )
            if world_xy:
                target.world_x = world_xy[0]
                target.world_y = world_xy[1]

        self._landing_pub.publish(target)

        # ---- 伺服模式: 降落 ----
        if self._servo_mode == "landing" and self._landing_servo and target.confidence > 0.3:
            fx = self.get_parameter("fx").value
            fy = self.get_parameter("fy").value
            cx = self.get_parameter("cx").value
            cy = self.get_parameter("cy").value

            state = self._landing_servo.update(
                target_u=target.center_u,
                target_v=target.center_v,
                image_center_u=cx,
                image_center_v=cy,
                altitude=max(self._current_altitude, 0.3),
                focal_length_u=fx,
                focal_length_v=fy,
                current_yaw=self._current_yaw_ned,
            )
            # 发布伺服速度指令
            vel = TwistStamped()
            vel.header.stamp = self.get_clock().now().to_msg()
            vel.header.frame_id = "ned"
            vel.twist.linear.x = state.velocity_north
            vel.twist.linear.y = state.velocity_east
            vel.twist.linear.z = state.velocity_down
            vel.twist.angular.z = state.yaw_rate
            self._servo_vel_pub.publish(vel)
            # 伺服状态可通过 diagnostics 话题发布
            self.get_logger().debug(
                f"伺服: err={state.horizontal_error:.3f}m "
                f"vel=({state.velocity_north:.2f},{state.velocity_east:.2f}) "
                f"conv={state.is_converged}"
            )

    # ================================================================
    # 覆盖监控
    # ================================================================

    def _process_coverage(self):
        # 每帧更新覆盖网格 (用当前无人机 World 位姿)
        if self._coverage:
            self._coverage.update(
                self._drone_pose[0],  # drone_world_x
                self._drone_pose[1],  # drone_world_y
                max(self._drone_pose[2], 0.1),  # drone_world_z (altitude, +up)
            )
        ratio = self._coverage.coverage_ratio if self._coverage else 0.0
        msg = CoverageStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.coverage_ratio = ratio
        msg.is_complete = self._coverage.is_complete if self._coverage else False
        self._cov_pub.publish(msg)

    # ================================================================
    # 外部接口
    # ================================================================

    def update_altitude(self, altitude_m: float):
        """外部更新无人机高度."""
        self._current_altitude = altitude_m

    def update_pose(self, world_x: float, world_y: float, world_z: float):
        """外部更新无人机位姿 (用于覆盖监控)."""
        if self._coverage:
            self._coverage.update(world_x, world_y, world_z)

    def destroy_node(self):
        self._release_detection_models()
        if self._landing_engine is not None:
            self._landing_engine.close()
            self._landing_engine = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionPipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
