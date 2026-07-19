"""
圆筒检测节点 — YOLO 粗定位 + OpenCV 精确拟合.

流程:
  1. 订阅 /drone/camera/image_raw
  2. YOLO 推理 → 检出所有 "cylinder" bbox
  3. 每个 bbox 内: Canny 边缘 → 椭圆拟合 → 精确圆心 + 像素半径
  4. 像素坐标 → 世界坐标 (利用高度 + 相机内参)
  5. 多帧追踪 (IoU 匹配) → 分配稳定 ID
  6. 发布 CylinderArray 话题

发布:
  /drone/vision/cylinders  (CylinderArray)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, CameraInfo
from drone_interfaces.msg import CylinderDetection, CylinderArray
from cv_bridge import CvBridge
import numpy as np
import cv2
from typing import List, Optional, Tuple
import time

from drone_vision.models.yolo_engine import Detection


CAMERA_INFO_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


# ============================================================
# 圆筒精确拟合器 (OpenCV 后处理)
# ============================================================

class CylinderRefiner:
    """在 YOLO bbox 内用 OpenCV 精确拟合圆筒圆心和半径."""

    def __init__(
        self,
        blur_ksize: int = 5,
        canny_low: int = 50,
        canny_high: int = 150,
        min_radius_ratio: float = 0.3,
        max_radius_ratio: float = 1.2,
        white_brightness: int = 120,
    ):
        self.blur_ksize = blur_ksize
        self.canny_low = canny_low
        self.canny_high = canny_high
        self.min_radius_ratio = min_radius_ratio
        self.max_radius_ratio = max_radius_ratio
        self.white_brightness = white_brightness

    def refine(
        self, image: np.ndarray, bbox: Tuple[int, int, int, int],
    ) -> Optional[Tuple[float, float, float, float]]:
        """
        在 bbox 区域内精确拟合椭圆形圆筒开口.

        Parameters
        ----------
        image : BGR uint8 (H, W, 3)
        bbox : (x1, y1, x2, y2)

        Returns
        -------
        (cx, cy, radius_px, confidence) 或 None
            cx, cy 为原图像素坐标; radius_px 为椭圆平均半径.
        """
        x1, y1, x2, y2 = bbox
        # 扩展 ROI 边距
        margin = 5
        x1c = max(0, x1 - margin)
        y1c = max(0, y1 - margin)
        x2c = min(image.shape[1], x2 + margin)
        y2c = min(image.shape[0], y2 + margin)

        if x2c <= x1c or y2c <= y1c:
            return None

        roi = image[y1c:y2c, x1c:x2c]
        if roi.size == 0:
            return None

        # 灰度 + 模糊
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (self.blur_ksize, self.blur_ksize), 0)

        # Canny 边缘
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)

        # 找轮廓 → 椭圆拟合
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        bbox_w = x2 - x1
        bbox_h = y2 - y1
        if bbox_w <= 0 or bbox_h <= 0:
            return None
        half_min = min(bbox_w, bbox_h) * self.min_radius_ratio / 2
        half_max = max(bbox_w, bbox_h) * self.max_radius_ratio / 2

        best_ellipse = None
        best_score = -1.0

        for cnt in contours:
            if len(cnt) < 5:
                continue
            ellipse = cv2.fitEllipse(cnt)
            (ecx, ecy), (ea, eb), angle = ellipse
            if not np.isfinite((ecx, ecy, ea, eb, angle)).all():
                continue
            if ea <= 0 or eb <= 0:
                continue
            if not (0 <= ecx < roi.shape[1] and 0 <= ecy < roi.shape[0]):
                continue
            avg_r = (ea + eb) / 4  # 直径→半径 (/2), 椭圆平均 (/2)
            r_min, r_max = min(ea, eb) / 2, max(ea, eb) / 2

            # 半径范围检查
            if avg_r < half_min or avg_r > half_max:
                continue
            # 椭圆偏心率不能太大 (接近圆形)
            if r_min < r_max * 0.5:
                continue

            # 评分: 边缘点数 × 圆形度
            circularity = r_min / (r_max + 1e-6)
            area = cv2.contourArea(cnt)
            score = area * circularity

            # 白色验证: 在椭圆中心采样
            sample_x = int(round(ecx))
            sample_y = int(round(ecy))
            if 0 <= sample_y < roi.shape[0] and 0 <= sample_x < roi.shape[1]:
                roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                v_center = roi_hsv[sample_y, sample_x, 2]
                if v_center < self.white_brightness:
                    score *= 0.3  # 中心不够白, 惩罚

            if score > best_score:
                best_score = score
                # 映射回原图坐标
                best_ellipse = (ecx + x1c, ecy + y1c, avg_r)

        if best_ellipse is None:
            return None

        ecx_abs, ecy_abs, avg_r = best_ellipse
        confidence = min(best_score / 500.0, 1.0)
        return (ecx_abs, ecy_abs, avg_r, confidence)

    def estimate_real_radius(
        self, radius_px: float, altitude: float, focal_length: float,
    ) -> float:
        """
        像素半径 → 真实半径 (m).

        r_real = r_px * altitude / f
        假设相机垂直向下.
        """
        if altitude <= 0:
            return 0.0
        return radius_px * altitude / focal_length


# ============================================================
# 多帧追踪器
# ============================================================

class CylinderTracker:
    """基于 IoU 的简单多目标追踪器.

    通过 min_hits 过滤闪烁假目标: 新 track 需连续命中 min_hits 帧后才输出.
    """

    def __init__(self, max_age: int = 10, iou_thresh: float = 0.3,
                 min_hits: int = 3):
        self.max_age = max_age
        self.iou_thresh = iou_thresh
        self.min_hits = min_hits
        self.tracks: List[dict] = []  # {id, cx, cy, radius, hits, last_seen, age}
        self._next_id = 0

    def update(self, detections: List[Tuple[float, float, float, float, tuple]]):
        """
        用新检测更新追踪.

        Parameters
        ----------
        detections : [(cx, cy, radius, confidence, bbox), ...]
            bbox : (x1, y1, x2, y2) 或 None
        """
        # 所有 track 年龄 +1
        for t in self.tracks:
            t["age"] += 1

        matched_track_indices: set = set()
        matched_det_indices: set = set()

        if self.tracks and detections:
            for det_idx, item in enumerate(detections):
                dcx, dcy, dr, dconf = item[0], item[1], item[2], item[3]
                bbox = item[4] if len(item) > 4 else None
                best_track = None
                best_dist = float("inf")
                for ti, t in enumerate(self.tracks):
                    if ti in matched_track_indices:
                        continue
                    dist = np.hypot(dcx - t["cx"], dcy - t["cy"])
                    threshold = max(dr, t["radius"]) * 2
                    if dist < threshold and dist < best_dist:
                        best_dist = dist
                        best_track = ti

                if best_track is not None and best_dist < max(dr, self.tracks[best_track]["radius"]) * 2:
                    self.tracks[best_track]["cx"] = dcx
                    self.tracks[best_track]["cy"] = dcy
                    self.tracks[best_track]["radius"] = dr
                    self.tracks[best_track]["confidence"] = dconf
                    self.tracks[best_track]["hits"] = self.tracks[best_track].get("hits", 0) + 1
                    if bbox:
                        self.tracks[best_track]["bbox"] = bbox
                    self.tracks[best_track]["last_seen"] = time.time()
                    self.tracks[best_track]["age"] = 0
                    matched_track_indices.add(best_track)
                    matched_det_indices.add(det_idx)

        # 未匹配的检测 → 新建 track
        for det_idx, item in enumerate(detections):
            if det_idx not in matched_det_indices:
                dcx, dcy, dr, dconf = item[0], item[1], item[2], item[3]
                bbox = item[4] if len(item) > 4 else None
                track = {
                    "id": self._next_id,
                    "cx": dcx,
                    "cy": dcy,
                    "radius": dr,
                    "confidence": dconf,
                    "hits": 0,
                    "last_seen": time.time(),
                    "age": 0,
                }
                if bbox:
                    track["bbox"] = bbox
                self.tracks.append(track)
                self._next_id += 1

    def prune(self):
        """移除过期 track."""
        self.tracks = [t for t in self.tracks if t["age"] < self.max_age]

    def get_active(self) -> List[dict]:
        """返回已确认的活跃 track (当前帧命中 + 连续命中 >= min_hits)."""
        return [t for t in self.tracks
                if t["age"] == 0 and t.get("hits", 0) >= self.min_hits]

    def get_pending(self) -> List[dict]:
        """返回待确认的 track (活跃但未达 min_hits, 用于调试)."""
        return [t for t in self.tracks
                if t["age"] == 0 and t.get("hits", 0) < self.min_hits]


def detect_white_cylinders(image: np.ndarray) -> List[Detection]:
    """Detect circular white cylinder openings when no YOLO model is loaded."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    max_radius = max(20, int(min(image.shape[:2]) * 0.25))
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.0,
        minDist=30,
        param1=100,
        param2=30,
        minRadius=10,
        maxRadius=max_radius,
    )
    if circles is None:
        return []

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    candidates = []
    yy, xx = np.ogrid[:image.shape[0], :image.shape[1]]
    for cx, cy, radius in circles[0]:
        inner = (xx - cx) ** 2 + (yy - cy) ** 2 <= (radius * 0.85) ** 2
        white = (hsv[:, :, 1] < 100) & (hsv[:, :, 2] > 100) & inner
        white_ratio = float(np.count_nonzero(white) / max(1, inner.sum()))
        if white_ratio < 0.25:
            continue
        confidence = min(0.9, 0.45 + white_ratio * 0.5)
        candidates.append((confidence, float(cx), float(cy), float(radius)))

    selected = []
    for candidate in sorted(candidates, reverse=True):
        _, cx, cy, radius = candidate
        duplicate = any(
            np.hypot(cx - kept[1], cy - kept[2])
            < max(radius, kept[3]) * 0.8
            for kept in selected
        )
        if not duplicate:
            selected.append(candidate)

    detections = []
    height, width = image.shape[:2]
    for confidence, cx, cy, radius in selected:
        detections.append(Detection(
            class_id=0,
            class_name="cylinder",
            confidence=confidence,
            bbox=(
                max(0, int(round(cx - radius * 1.15))),
                max(0, int(round(cy - radius * 1.15))),
                min(width, int(round(cx + radius * 1.15))),
                min(height, int(round(cy + radius * 1.15))),
            ),
            cx=cx,
            cy=cy,
        ))
    return detections


# ============================================================
# ROS2 节点
# ============================================================

class CylinderDetectorNode(Node):
    """圆筒检测 ROS2 节点."""

    def __init__(self):
        super().__init__("cylinder_detector")

        # 参数
        self._declare_params()

        # YOLO 引擎 (延迟加载)
        self._engine = None
        self._refiner: Optional[CylinderRefiner] = None
        self._tracker: Optional[CylinderTracker] = None
        self._bridge = CvBridge()

        # 当前无人机高度 (m), 从遥测更新
        self._current_altitude = 5.0

        # 订阅
        self._img_sub = self.create_subscription(
            Image, "/drone/camera/image_raw", self._image_cb,
            qos_profile_sensor_data,
        )
        self._caminfo_sub = self.create_subscription(
            CameraInfo, "/drone/camera/camera_info", self._caminfo_cb,
            CAMERA_INFO_QOS,
        )

        # 发布
        self._cyl_pub = self.create_publisher(
            CylinderArray, "/drone/vision/cylinders", 10,
        )

        # 初始化定时器 — 模型延迟加载
        self._init_timer = self.create_timer(1.0, self._lazy_init)
        self._initialized = False

        self.get_logger().info("圆筒检测节点已启动 (等待模型加载...)")

    def _declare_params(self):
        self.declare_parameter("model_path", "")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("nms_iou_threshold", 0.45)
        self.declare_parameter("model_width", 640)
        self.declare_parameter("model_height", 640)
        self.declare_parameter("device", "cuda")
        self.declare_parameter("class_names", ["cylinder"])

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

        self.declare_parameter("fx", 400.0)
        self.declare_parameter("fy", 400.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)

    def _lazy_init(self):
        """延迟初始化 YOLO 引擎 (等待参数就绪)."""
        if self._initialized:
            return

        model_path = self.get_parameter("model_path").value
        if not model_path:
            # 尝试默认路径
            import os
            default = os.path.join(
                os.path.dirname(__file__), "models", "cylinder_yolo.pt",
            )
            if os.path.exists(default):
                model_path = default
            else:
                self.get_logger().warn(
                    "未指定 model_path 且默认模型不存在, "
                    "将使用纯 OpenCV 回退模式"
                )
                self._init_refiner_and_tracker()
                self._initialized = True
                self._init_timer.cancel()
                return
        elif not os.path.isabs(model_path):
            resolved = os.path.join(
                os.path.dirname(__file__), "models", model_path,
            )
            if os.path.exists(resolved):
                model_path = resolved

        try:
            from drone_vision.models.yolo_engine import create_engine

            class_names = self.get_parameter("class_names").get_parameter_value().string_array_value
            self._engine = create_engine(
                model_path=model_path,
                conf_threshold=self.get_parameter("conf_threshold").value,
                nms_iou_threshold=self.get_parameter("nms_iou_threshold").value,
                input_size=(
                    self.get_parameter("model_width").value,
                    self.get_parameter("model_height").value,
                ),
                class_names=list(class_names) if class_names else None,
                device=self.get_parameter("device").value,
            )
            if not self._engine.load():
                self.get_logger().error("YOLO 模型加载失败")
        except Exception as e:
            self.get_logger().error(f"YOLO 引擎初始化失败: {e}")

        self._init_refiner_and_tracker()
        self._initialized = True
        self._init_timer.cancel()
        self.get_logger().info("圆筒检测节点初始化完成")

    def _init_refiner_and_tracker(self):
        self._refiner = CylinderRefiner(
            blur_ksize=self.get_parameter("cylinder_edge_blur").value,
            canny_low=self.get_parameter("cylinder_canny_low").value,
            canny_high=self.get_parameter("cylinder_canny_high").value,
            min_radius_ratio=self.get_parameter("cylinder_min_radius_ratio").value,
            max_radius_ratio=self.get_parameter("cylinder_max_radius_ratio").value,
            white_brightness=self.get_parameter("cylinder_white_brightness").value,
        )
        self._tracker = CylinderTracker(
            max_age=self.get_parameter("cylinder_tracking_max_age").value,
            iou_thresh=self.get_parameter("cylinder_tracking_iou_thresh").value,
            min_hits=self.get_parameter("cylinder_tracking_min_hits").value,
        )

    def _caminfo_cb(self, msg: CameraInfo):
        """更新相机内参."""
        # 从 CameraInfo 更新内参 (优先级高于 YAML 参数)
        pass  # 使用 YAML 参数作为回退

    def _image_cb(self, msg: Image):
        """图像回调: YOLO 检测 → OpenCV 精修 → 发布."""
        if not self._initialized:
            return

        try:
            image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        # ---- YOLO 推理 (或纯 OpenCV 回退) ----
        raw_detections = []
        if self._engine and self._engine.is_loaded:
            dets = self._engine.infer(image)
            raw_detections = [d for d in dets if d.class_name == "cylinder"]
        else:
            # 纯 OpenCV 回退: 全局 HoughCircles + 白色验证
            raw_detections = self._fallback_detect(image)

        # ---- OpenCV 精确拟合 ----
        refined: List[tuple] = []
        enable_refine = self.get_parameter("cylinder_refine_enable").value

        for det in raw_detections:
            bbox_px = (
                int(det.bbox[0]), int(det.bbox[1]),
                int(det.bbox[2]), int(det.bbox[3]),
            )
            if enable_refine and self._refiner:
                result = self._refiner.refine(image, det.bbox)
                if result:
                    refined_cx, refined_cy, radius_px, _ = result
                    refined.append((
                        refined_cx,
                        refined_cy,
                        radius_px,
                        det.confidence,
                        bbox_px,
                    ))
                elif det.confidence > 0.7:
                    refined.append((det.cx, det.cy, min(
                        det.bbox[2] - det.bbox[0],
                        det.bbox[3] - det.bbox[1],
                    ) / 4, det.confidence * 0.7, bbox_px))
            else:
                refined.append((det.cx, det.cy, min(
                    det.bbox[2] - det.bbox[0],
                    det.bbox[3] - det.bbox[1],
                ) / 4, det.confidence, bbox_px))

        if not refined:
            # 发布空数组
            self._cyl_pub.publish(CylinderArray())
            return

        # ---- 多帧追踪 ----
        self._tracker.update(refined)
        self._tracker.prune()
        active = self._tracker.get_active()

        # ---- 像素 → 世界坐标 ----
        fx = self.get_parameter("fx").value
        fy = self.get_parameter("fy").value
        cx = self.get_parameter("cx").value
        cy = self.get_parameter("cy").value
        alt = max(self._current_altitude, 0.5)

        # 无人机当前世界坐标 (从遥测获取, 这里用占位)
        # 实际应该从 px4_control 的 telemetry 话题获取
        drone_ned_x = 0.0  # placeholder
        drone_ned_y = 0.0

        msg_array = CylinderArray()
        msg_array.header.stamp = self.get_clock().now().to_msg()
        msg_array.header.frame_id = "world"

        for t in active:
            det_msg = CylinderDetection()
            det_msg.id = t["id"]
            det_msg.confidence = float(t["confidence"])
            det_msg.center_u = float(t["cx"])
            det_msg.center_v = float(t["cy"])
            # YOLO bbox
            bbox = t.get("bbox")
            if bbox and len(bbox) == 4:
                det_msg.bbox_x1, det_msg.bbox_y1 = bbox[0], bbox[1]
                det_msg.bbox_x2, det_msg.bbox_y2 = bbox[2], bbox[3]

            # 像素 → World:
            # dx_world = (cx_px - cx) * altitude / fx   (相机向下, x对应right=east)
            # dy_world = (cy_px - cy) * altitude / fy   (y对应down=forward=north)
            # 这里简化: 相机 x→world east, y→world north (取决于安装方向)
            dx_world = -(t["cy"] - cy) * alt / fy   # 像素行 → world north
            dy_world = -(t["cx"] - cx) * alt / fx   # 像素列 → world east

            det_msg.center_x = drone_ned_x + dy_world  # NED east
            det_msg.center_y = drone_ned_y + dx_world  # NED north

            # 真实半径估计
            pixel_radius = t["radius"]
            det_msg.radius = pixel_radius * alt / max(fx, fy)

            msg_array.cylinders.append(det_msg)

        self._cyl_pub.publish(msg_array)

    def _fallback_detect(self, image: np.ndarray) -> list:
        """
        纯 OpenCV 回退检测 (无 YOLO 模型时使用).

        返回格式与 YOLO 检测一致的简单对象列表.
        """
        return detect_white_cylinders(image)

    def update_altitude(self, altitude_m: float):
        """由外部调用: 更新当前无人机高度."""
        self._current_altitude = altitude_m


def main(args=None):
    rclpy.init(args=args)
    node = CylinderDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
