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
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge

from drone_interfaces.msg import (
    CylinderDetection, CylinderArray,
    HazardDetection, HazardArray,
    LandingTarget, CoverageStatus,
)

from drone_vision.models.yolo_engine import Detection, create_engine, YOLOEngine
from drone_vision.cylinder_detector import CylinderRefiner, CylinderTracker
from drone_vision.hazard_classifier import ColorVerifier, CLASS_TO_HAZARD
from drone_vision.landing_detector import LandingPadRefiner
from drone_vision.coverage_monitor import CoverageMonitor
from drone_vision.coordinate_transforms import (
    CoordinateTransformer, AltitudeAdaptiveParams,
)
from drone_vision.visual_servoing import (
    VisualServoController, CylinderApproachController, ServoState,
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
        self._bridge = CvBridge()

        # 坐标变换 + 姿态补偿
        self._coord_xform: Optional[CoordinateTransformer] = None
        self._adaptive_params: Optional[AltitudeAdaptiveParams] = None

        # 后处理器
        self._cyl_refiner: Optional[CylinderRefiner] = None
        self._cyl_tracker: Optional[CylinderTracker] = None
        self._color_verifier: Optional[ColorVerifier] = None
        self._landing_refiner: Optional[LandingPadRefiner] = None
        self._coverage: Optional[CoverageMonitor] = None

        # 伺服控制器
        self._landing_servo: Optional[VisualServoController] = None
        self._cylinder_servo: Optional[CylinderApproachController] = None
        self._servo_mode: str = "none"  # "landing" | "cylinder_approach" | "none"
        self._target_cylinder_id: int = -1

        # ---- 实时状态 ----
        self._drone_pose: Tuple[float, float, float, float, float, float, float] = (
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        )  # (x, y, z, qx, qy, qz, qw)
        self._current_altitude = 5.0
        self._last_frame_time = time.time()
        self._frame_count = 0
        self._skip_count = 0

        # ---- 订阅 ----
        # gz_camera_bridge.py (Python) 发布到 /drone/camera/image_raw, QoS 匹配
        self._img_sub = self.create_subscription(
            Image, "/drone/camera/image_raw", self._image_cb, 10,
        )
        self._caminfo_sub = self.create_subscription(
            CameraInfo, "/drone/camera/camera_info", self._caminfo_cb, 10,
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
        # 视觉伺服速度指令 (供 px4_bridge 或外部节点消费, 转发至 PX4)
        self._servo_vel_pub = self.create_publisher(
            TwistStamped, "/drone/vision/servo_cmd", 10,
        )

        # ---- 初始化 ----
        self._init_timer = self.create_timer(1.0, self._lazy_init)
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
        # 危险品
        self.declare_parameter("hazard_enable_color_fallback", True)
        self.declare_parameter("hazard_color_confirm_ratio", 0.3)
        # 降落
        self.declare_parameter("landing_refine_enable", True)
        self.declare_parameter("landing_adaptive_thresh_block", 11)
        self.declare_parameter("landing_circle_min_area", 300)
        self.declare_parameter("landing_H_orientation_enable", True)
        # 覆盖
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

        # --- 坐标变换 + 高度自适应 ---
        fx = self.get_parameter("fx").value
        fy = self.get_parameter("fy").value
        cx = self.get_parameter("cx").value
        cy = self.get_parameter("cy").value
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        self._coord_xform = CoordinateTransformer(K)
        self._adaptive_params = AltitudeAdaptiveParams(
            focal_length_px=max(fx, fy),
            base_altitude=5.0,
            base_conf_threshold=self.get_parameter("conf_threshold").value,
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
        )
        self._color_verifier = ColorVerifier(
            confirm_ratio=self.get_parameter("hazard_color_confirm_ratio").value,
        )
        self._landing_refiner = LandingPadRefiner(
            adaptive_thresh_block=self.get_parameter("landing_adaptive_thresh_block").value,
            circle_min_area=self.get_parameter("landing_circle_min_area").value,
        )
        self._coverage = CoverageMonitor(
            zone_x_range=(65.0, 70.0), zone_y_range=(0.0, 8.0),
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

        # --- 加载 YOLO ---
        model_path = self.get_parameter("model_path").value
        if not model_path:
            default = os.path.join(
                os.path.dirname(__file__), "..", "models", "drone_yolo.onnx",
            )
            if os.path.exists(default):
                model_path = default

        if model_path and os.path.exists(model_path):
            try:
                class_names = [
                    "cylinder", "flammable", "health_hazard",
                    "danger", "landing_H", "empty_cylinder",
                ]
                self._engine = create_engine(
                    model_path=model_path,
                    conf_threshold=self.get_parameter("conf_threshold").value,
                    nms_iou_threshold=self.get_parameter("nms_iou_threshold").value,
                    input_size=(
                        self.get_parameter("model_width").value,
                        self.get_parameter("model_height").value,
                    ),
                    class_names=class_names,
                    device=self.get_parameter("device").value,
                )
                ok = self._engine.load()
                if ok:
                    self.get_logger().info(f"YOLO 模型已加载: {model_path}")
                else:
                    self.get_logger().warn("YOLO 加载失败, 将使用纯 OpenCV 回退")
            except Exception as e:
                self.get_logger().error(f"YOLO 引擎创建失败: {e}")
        else:
            self.get_logger().warn("未找到 YOLO 模型, 使用纯 OpenCV 回退模式")

        # --- 加载投放区蓝色标识模型 (可选) ---
        marker_model_path = self.get_parameter("drop_marker_model_path").value
        if marker_model_path and os.path.exists(marker_model_path):
            try:
                self._drop_marker_engine = create_engine(
                    model_path=marker_model_path,
                    conf_threshold=self.get_parameter("conf_threshold").value,
                    nms_iou_threshold=self.get_parameter("nms_iou_threshold").value,
                    input_size=(
                        self.get_parameter("model_width").value,
                        self.get_parameter("model_height").value,
                    ),
                    class_names=["drop_zone_marker"],
                    device=self.get_parameter("device").value,
                )
                ok = self._drop_marker_engine.load()
                if ok:
                    self.get_logger().info(f"投放区蓝色标识模型已加载: {marker_model_path}")
                else:
                    self.get_logger().warn("投放区蓝色标识模型加载失败")
                    self._drop_marker_engine = None
            except Exception as e:
                self.get_logger().error(f"投放区蓝色标识模型创建失败: {e}")

        self._initialized = True
        self._init_timer.cancel()
        self.get_logger().info(
            "统一视觉管线 v2 初始化完成 "
            f"(姿态补偿={'✓' if self._coord_xform else '✗'}, "
            f"高度自适应={'✓' if self._adaptive_params else '✗'})"
        )

    # ================================================================
    # 伺服模式切换 (供 mission 节点通过 Service 调用)
    # ================================================================

    def set_servo_mode(self, mode: str, cylinder_id: int = -1):
        """
        切换伺服模式.

        Parameters
        ----------
        mode : "landing" | "cylinder_approach" | "none"
        cylinder_id : 圆筒逼近时的目标 ID
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

    # ================================================================
    # 主回调
    # ================================================================

    def _image_cb(self, msg: Image):
        if not self._initialized:
            return

        # ---- 帧率自适应: Jetson 上跟不上时自动跳帧 ----
        self._frame_count += 1
        t_elapsed = time.time() - self._last_frame_time
        min_interval = 0.03  # 最大 ~33 FPS
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

        h, w = image.shape[:2]

        # ---- 高度自适应置信度阈值 ----
        if self._adaptive_params:
            cyl_params = self._adaptive_params.get_cylinder_params(self._current_altitude)
            effective_conf = cyl_params["effective_conf_threshold"]
        else:
            effective_conf = self.get_parameter("conf_threshold").value

        # ---- 1. YOLO 推理 ----
        if self._engine and self._engine.is_loaded:
            all_dets = self._engine.infer(image)
        else:
            all_dets = self._fallback_all(image)

        if self._drop_marker_engine and self._drop_marker_engine.is_loaded:
            marker_dets = self._drop_marker_engine.infer(image)
            if marker_dets and self._frame_count % 30 == 0:
                best_marker = marker_dets[0]
                self.get_logger().info(
                    "DROP_MARKER: detected "
                    f"conf={best_marker.confidence:.2f} "
                    f"bbox={best_marker.bbox}"
                )
            all_dets.extend(marker_dets)

        # 根据当前伺服模式筛选检测
        if self._servo_mode == "landing":
            # 降落模式: 仅关注 landing_H
            cyl_dets = []
            hazard_dets = []
            landing_dets = [d for d in all_dets if d.class_name == "landing_H"]
        elif self._servo_mode == "cylinder_approach":
            # 圆筒逼近: 仅关注 cylinder
            cyl_dets = [d for d in all_dets if d.class_name in ("cylinder", "empty_cylinder")]
            hazard_dets = []
            landing_dets = []
        else:
            # 全模式
            cyl_dets = [d for d in all_dets if d.class_name in ("cylinder", "empty_cylinder")]
            hazard_dets = [d for d in all_dets if d.class_name in ("flammable", "health_hazard", "danger", "empty_cylinder")]
            landing_dets = [d for d in all_dets if d.class_name == "landing_H"]

        # ---- 2. 圆筒检测 + OpenCV 精修 (姿态补偿坐标) ----
        self._process_cylinders_v2(image, cyl_dets)

        # ---- 3. 危险品分类 — 已禁用 (侦察区仅执行全覆盖路径巡视) ----
        # self._process_hazards(image, hazard_dets)

        # ---- 4. H 降落标识 + 姿态补偿 + 伺服 ----
        self._process_landing_v2(image, landing_dets)

        # ---- 5. 覆盖监控 ----
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
        refined: List[Tuple[float, float, float, float]] = []
        enable_refine = self.get_parameter("cylinder_refine_enable").value

        # 高度自适应半径范围
        if self._adaptive_params:
            cyl_params = self._adaptive_params.get_cylinder_params(self._current_altitude)
        else:
            cyl_params = {"min_radius_px": 5, "max_radius_px": 100}

        for d in dets:
            if enable_refine and self._cyl_refiner:
                result = self._cyl_refiner.refine(image, d.bbox)
                if result:
                    refined.append(result)
                elif d.confidence > 0.7:
                    refined.append((d.cx, d.cy,
                                    min(d.bbox[2] - d.bbox[0], d.bbox[3] - d.bbox[1]) / 4,
                                    d.confidence * 0.7))
            else:
                refined.append((d.cx, d.cy,
                                min(d.bbox[2] - d.bbox[0], d.bbox[3] - d.bbox[1]) / 4,
                                d.confidence))

        if not refined:
            self._cyl_pub.publish(CylinderArray())
            return

        self._cyl_tracker.update(refined)
        self._cyl_tracker.prune()
        active = self._cyl_tracker.get_active()

        fx = self.get_parameter("fx").value
        fy = self.get_parameter("fy").value
        cx = self.get_parameter("cx").value
        cy = self.get_parameter("cy").value
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

            simple_xy = CoordinateTransformer.pixel_to_world_simple(
                t["cx"], t["cy"], alt, fx, fy, cx, cy,
                drone_world_x=self._drone_pose[0],
                drone_world_y=self._drone_pose[1],
            )
            selected_xy = None

            # ---- 姿态补偿坐标变换 ----
            if self._coord_xform:
                world_xy = self._coord_xform.pixel_to_world(
                    t["cx"], t["cy"],
                    (self._drone_pose[0], self._drone_pose[1], self._drone_pose[2]),
                    (self._drone_pose[3], self._drone_pose[4], self._drone_pose[5], self._drone_pose[6]),
                    ground_z=0.0, q_convention="xyzw",
                )
                if world_xy:
                    dx = world_xy[0] - self._drone_pose[0]
                    dy = world_xy[1] - self._drone_pose[1]
                    max_reasonable_offset = max(alt * 1.5, 3.0)
                    if (dx * dx + dy * dy) ** 0.5 <= max_reasonable_offset:
                        selected_xy = world_xy
                    else:
                        self.get_logger().warn(
                            "CYL_COORD: 姿态投影异常, 使用简化投影 "
                            f"raw=({world_xy[0]:.2f},{world_xy[1]:.2f}) "
                            f"drone=({self._drone_pose[0]:.2f},{self._drone_pose[1]:.2f})"
                        )
            else:
                selected_xy = simple_xy

            if selected_xy is None:
                selected_xy = simple_xy

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
            self._landing_pub.publish(target)
            return

        best = max(dets, key=lambda d: d.confidence)
        enable_refine = self.get_parameter("landing_refine_enable").value

        if enable_refine and self._landing_refiner:
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
        else:
            target.center_u = best.cx
            target.center_v = best.cy
            target.confidence = best.confidence
            target.yaw_correction = 0.0

        # ---- 姿态补偿坐标变换 ----
        if self._coord_xform:
            world_xy = self._coord_xform.pixel_to_world(
                target.center_u, target.center_v,
                (self._drone_pose[0], self._drone_pose[1], self._drone_pose[2]),
                (self._drone_pose[3], self._drone_pose[4], self._drone_pose[5], self._drone_pose[6]),
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
            )
            # 发布伺服速度指令
            vel = TwistStamped()
            vel.header.stamp = self.get_clock().now().to_msg()
            vel.header.frame_id = "base_link"
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
    # 旧方法保留 (向后兼容, v1 节点单独运行时使用)
    # ================================================================

    # ================================================================
    # 危险品处理
    # ================================================================

    def _process_hazards(self, image: np.ndarray, dets: List[Detection]):
        hazards = []
        enable_color = self.get_parameter("hazard_enable_color_fallback").value

        for d in dets:
            confirmed_type = CLASS_TO_HAZARD.get(d.class_name, 0)
            final_conf = d.confidence

            if enable_color and d.confidence < 0.7:
                is_match, color_ratio = self._color_verifier.verify(
                    image, d.bbox, confirmed_type,
                )
                if not is_match:
                    for alt_type in [1, 2, 3]:
                        if alt_type == confirmed_type:
                            continue
                        is_alt, alt_ratio = self._color_verifier.verify(
                            image, d.bbox, alt_type,
                        )
                        if is_alt and alt_ratio > color_ratio:
                            confirmed_type = alt_type
                            final_conf = max(final_conf, alt_ratio)
                            break
                    else:
                        continue
                final_conf = max(final_conf, color_ratio)

            h = HazardDetection()
            h.id = -1
            h.hazard_type = confirmed_type
            h.confidence = float(final_conf)
            hazards.append(h)

        msg_array = HazardArray()
        msg_array.header.stamp = self.get_clock().now().to_msg()
        msg_array.header.frame_id = "world"
        msg_array.hazards = hazards
        self._hazard_pub.publish(msg_array)

    # ================================================================
    # 降落处理
    # ================================================================

    def _process_landing(self, image: np.ndarray, dets: List[Detection]):
        target = LandingTarget()
        target.confidence = 0.0

        if not dets:
            self._landing_pub.publish(target)
            return

        best = max(dets, key=lambda d: d.confidence)
        enable_refine = self.get_parameter("landing_refine_enable").value

        if enable_refine:
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
        else:
            target.center_u = best.cx
            target.center_v = best.cy
            target.confidence = best.confidence

        fx = self.get_parameter("fx").value
        fy = self.get_parameter("fy").value
        cx = self.get_parameter("cx").value
        cy = self.get_parameter("cy").value
        alt = max(self._current_altitude, 0.3)

        target.world_x = (target.center_u - cx) * alt / fx
        target.world_y = (target.center_v - cy) * alt / fy

        self._landing_pub.publish(target)

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
    # OpenCV 回退 (无 YOLO 模型时的全图搜索)
    # ================================================================

    def _fallback_all(self, image: np.ndarray) -> List[Detection]:
        """纯 OpenCV 全图搜索, 返回简单 Detection 列表."""
        detections = []

        # 圆筒: HoughCircles
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, dp=1.0, minDist=30,
            param1=100, param2=30, minRadius=10, maxRadius=100,
        )
        if circles is not None:
            for c in circles[0]:
                cx, cy, r = c
                bbox = (
                    max(0, int(cx - r * 1.3)), max(0, int(cy - r * 1.3)),
                    min(image.shape[1], int(cx + r * 1.3)), min(image.shape[0], int(cy + r * 1.3)),
                )
                detections.append(Detection(
                    class_id=0, class_name="cylinder", confidence=0.5,
                    bbox=bbox, cx=float(cx), cy=float(cy),
                ))

        # 危险品: 颜色分割 (简化)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        color_configs = [
            (1, [np.array([5, 80, 80]), np.array([25, 255, 255])]),
            (2, [np.array([100, 80, 80]), np.array([130, 255, 255])]),
            (3, [np.array([0, 80, 80]), np.array([10, 255, 255])]),
            (3, [np.array([160, 80, 80]), np.array([180, 255, 255])]),
        ]
        class_names = {1: "flammable", 2: "health_hazard", 3: "danger"}
        for cls_id, (lower, upper) in color_configs:
            mask = cv2.inRange(hsv, lower, upper)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 100:
                    continue
                x, y, w, h_box = cv2.boundingRect(cnt)
                detections.append(Detection(
                    class_id=cls_id,
                    class_name=class_names.get(cls_id, "unknown"),
                    confidence=min(area / 500.0, 0.8),
                    bbox=(x, y, x + w, y + h_box),
                    cx=float(x + w / 2), cy=float(y + h_box / 2),
                ))

        return detections

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


def main(args=None):
    rclpy.init(args=args)
    node = VisionPipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
