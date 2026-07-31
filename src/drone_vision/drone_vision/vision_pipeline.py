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
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image, CameraInfo
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
from drone_vision.landing_detector import LandingPadRefiner
from drone_vision.coverage_monitor import CoverageMonitor
from drone_vision.coordinate_transforms import CoordinateTransformer
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
        self._current_altitude = 5.0
        self._current_yaw_ned = 0.0
        self._last_frame_time = time.time()
        self._frame_count = 0
        self._skip_count = 0

        # ---- 订阅 ----
        # gz_camera_bridge.py (Python) 发布到 /drone/camera/image_raw, QoS 匹配
        self._img_sub = self.create_subscription(
            Image, "/drone/camera/image_raw", self._image_cb,
            IMAGE_QOS,
        )
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
        self.declare_parameter("max_inference_fps", 20.0)
        # 类别名 (YAML 配置, 必须与模型输出类别数一致)
        self.declare_parameter("class_names", ["cylinder"])
        self.declare_parameter("drop_marker_class_names", ["drop_zone_marker"])
        # 投放区蓝色标识 OpenCV 精修
        self.declare_parameter("drop_marker_refine_enable", True)
        self.declare_parameter("drop_marker_min_blue_ratio", 0.4)
        self.declare_parameter("drop_marker_min_area", 80)
        self.declare_parameter("drop_marker_min_aspect_ratio", 0.4)
        self.declare_parameter("drop_marker_min_circularity", 0.35)
        self.declare_parameter("drop_marker_min_diameter", 30)
        self.declare_parameter("drop_marker_min_scale_ratio", 0.45)
        self.declare_parameter("drop_marker_max_scale_ratio", 1.8)
        # 相机
        self.declare_parameter("fx", 400.0)
        self.declare_parameter("fy", 400.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
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
        # 覆盖 — 侦察区 World 坐标 (北向布局: Home X=4,Y=10; Recon NED north=55-60 east=±4)
        self.declare_parameter("coverage_zone_x_min", 0.0)   # World X = east_min + home_x = -4+4
        self.declare_parameter("coverage_zone_x_max", 8.0)   # World X = east_max + home_x = 4+4
        self.declare_parameter("coverage_zone_y_min", 65.0)  # World Y = north_min + home_y = 55+10
        self.declare_parameter("coverage_zone_y_max", 70.0)  # World Y = north_max + home_y = 60+10
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
        self._coord_xform = CoordinateTransformer(K)

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
        self._landing_refiner = LandingPadRefiner(
            adaptive_thresh_block=self.get_parameter("landing_adaptive_thresh_block").value,
            circle_min_area=self.get_parameter("landing_circle_min_area").value,
        )
        self._coverage = CoverageMonitor(
            zone_x_range=(
                self.get_parameter("coverage_zone_x_min").value,
                self.get_parameter("coverage_zone_x_max").value,
            ),
            zone_y_range=(
                self.get_parameter("coverage_zone_y_min").value,
                self.get_parameter("coverage_zone_y_max").value,
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
            self.set_servo_mode("landing")
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
        self._drone_pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
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

        t0 = time.time()
        try:
            image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        # 目标模型只在投放阶段运行。两模型顺序执行，避免无收益的 GPU 并发峰值。
        all_dets: List[Detection] = []
        marker_dets: List[Detection] = []
        marker_source = "none"
        if self._detection_mode == "all":
            all_dets = self._engine.infer(image)
            marker_dets = self._drop_marker_engine.infer(image)
            marker_source = "yolo"
            if (
                self.get_parameter("drop_marker_refine_enable").value
                and self._marker_refiner
            ):
                marker_dets = self._marker_refiner.refine(image, marker_dets)
                marker_source = "yolo+blue"
            expected_diameter = (
                (self.get_parameter("fx").value + self.get_parameter("fy").value)
                * 0.5 / max(self._current_altitude, 0.5)
            )
            marker_dets = self._marker_refiner.filter_by_expected_diameter(
                marker_dets,
                expected_diameter,
                self.get_parameter("drop_marker_min_scale_ratio").value,
                self.get_parameter("drop_marker_max_scale_ratio").value,
            )

        # 发布投放区标识检测
        if self._marker_refiner:
            marker_array = CylinderArray()
            marker_array.header.stamp = self.get_clock().now().to_msg()
            marker_array.header.frame_id = "world"
            for i, md in enumerate(marker_dets):
                det = CylinderDetection()
                det.id = i
                det.confidence = float(md.confidence)
                det.center_u = float(md.cx)
                det.center_v = float(md.cy)
                det.radius = 0.5  # 蓝色标识固定 1m 直径, 半径 0.5m
                det.bbox_x1 = int(md.bbox[0])
                det.bbox_y1 = int(md.bbox[1])
                det.bbox_x2 = int(md.bbox[2])
                det.bbox_y2 = int(md.bbox[3])

                fx = self.get_parameter("fx").value
                fy = self.get_parameter("fy").value
                cx = self.get_parameter("cx").value
                cy = self.get_parameter("cy").value
                alt = max(self._current_altitude, 0.5)
                selected_xy = CoordinateTransformer.pixel_to_world_simple(
                    md.cx, md.cy, alt, fx, fy, cx, cy,
                    drone_world_x=self._drone_pose[0],
                    drone_world_y=self._drone_pose[1],
                )
                if self._coord_xform:
                    world_xy = self._coord_xform.pixel_to_world(
                        md.cx, md.cy,
                        (self._drone_pose[0], self._drone_pose[1], self._drone_pose[2]),
                        (self._drone_pose[3], self._drone_pose[4],
                         self._drone_pose[5], self._drone_pose[6]),
                        ground_z=0.0, q_convention="xyzw",
                    )
                    if world_xy:
                        dx = world_xy[0] - self._drone_pose[0]
                        dy = world_xy[1] - self._drone_pose[1]
                        max_reasonable_offset = max(alt * 1.5, 3.0)
                        if (dx * dx + dy * dy) ** 0.5 <= max_reasonable_offset:
                            selected_xy = world_xy

                det.center_x = selected_xy[0]
                det.center_y = selected_xy[1]
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
            landing_dets = self._infer_landing(image)
        elif self._detection_mode == "all":
            cyl_dets = [
                d for d in all_dets
                if d.class_name in ("cylinder", "empty_cylinder")
            ]
            landing_dets = [d for d in all_dets if d.class_name == "landing_H"]
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
            self._cyl_pub.publish(CylinderArray())
            return

        self._cyl_tracker.update(refined)
        self._cyl_tracker.prune()
        active = self._cyl_tracker.get_active()

        fx = self.get_parameter("fx").value
        fy = self.get_parameter("fy").value
        alt = max(self._current_altitude, 0.5)

        msg_array = CylinderArray()
        msg_array.header.stamp = self.get_clock().now().to_msg()
        msg_array.header.frame_id = "world"

        for t in active:
            det = CylinderDetection()
            det.id = t["id"]
            det.confidence = float(t["confidence"])
            det.center_u = float(t["cx"])
            det.center_v = float(t["cy"])
            # YOLO bbox (用于调试可视化)
            bbox = t.get("bbox")
            if bbox and len(bbox) == 4:
                det.bbox_x1, det.bbox_y1, det.bbox_x2, det.bbox_y2 = bbox

            # ---- 姿态补偿坐标变换 ----
            # 优先使用 6-DoF 精确投影；debug 模式无位姿时回退 simple 投影
            cx = self.get_parameter("cx").value
            cy = self.get_parameter("cy").value
            selected_xy = CoordinateTransformer.pixel_to_world_simple(
                t["cx"], t["cy"], alt, fx, fy, cx, cy,
                drone_world_x=self._drone_pose[0],
                drone_world_y=self._drone_pose[1],
            )
            if self._coord_xform:
                world_xy = self._coord_xform.pixel_to_world(
                    t["cx"], t["cy"],
                    (self._drone_pose[0], self._drone_pose[1], self._drone_pose[2]),
                    self._drone_pose[3:7],
                    ground_z=0.0, q_convention="xyzw",
                )
                if world_xy is not None:
                    dx = world_xy[0] - self._drone_pose[0]
                    dy = world_xy[1] - self._drone_pose[1]
                    max_reasonable_offset = max(alt * 1.5, 3.0)
                    if (dx * dx + dy * dy) ** 0.5 <= max_reasonable_offset:
                        selected_xy = world_xy

            det.center_x = selected_xy[0]
            det.center_y = selected_xy[1]

            # 真实半径 (像素→米)
            det.radius = t["radius"] * alt / max(fx, fy)

            msg_array.cylinders.append(det)

        self._cyl_pub.publish(msg_array)

    # ================================================================
    # 降落处理 v2 (姿态补偿 + 伺服)
    # ================================================================

    def _process_landing_v2(self, image: np.ndarray, dets: List[Detection]):
        target = LandingTarget()
        target.confidence = 0.0

        if not dets:
            result = None
            if self._landing_refiner:
                result = self._landing_refiner.detect(image)
            if result is None:
                self._landing_pub.publish(target)
                return
            cx_px, cy_px, confidence, yaw = result
            target.center_u = cx_px
            target.center_v = cy_px
            target.confidence = confidence
            target.yaw_correction = yaw
            best = None
        else:
            best = max(dets, key=lambda d: d.confidence)

        enable_refine = self.get_parameter("landing_refine_enable").value

        if best is not None and enable_refine and self._landing_refiner:
            result = self._landing_refiner.refine(image, best.bbox)
            if result:
                cx_px, cy_px, conf, yaw = result
                target.center_u = cx_px
                target.center_v = cy_px
                target.confidence = max(best.confidence, conf)
                target.yaw_correction = yaw
            else:
                target.center_u = best.cx
                target.center_v = best.cy
                target.confidence = best.confidence
                target.yaw_correction = 0.0
        elif best is not None:
            target.center_u = best.cx
            target.center_v = best.cy
            target.confidence = best.confidence
            target.yaw_correction = 0.0

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
