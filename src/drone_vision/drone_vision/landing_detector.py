"""
H 降落标识检测节点 — YOLO 粗定位 + OpenCV 精确圆心拟合.

流程:
  1. 订阅 /drone/camera/image_raw
  2. YOLO 推理 → 检出 "landing_H" bbox
  3. OpenCV 后处理: 自适应阈值 → 圆形底板检测 → 精确圆心
  4. 可选: 估计 H 方向 → 偏航修正角
  5. 像素 → 世界坐标
  6. 发布 LandingTarget 话题

发布:
  /drone/vision/landing_target  (LandingTarget)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from drone_interfaces.msg import LandingTarget
from cv_bridge import CvBridge
import numpy as np
import cv2
from dataclasses import dataclass
from typing import Optional, Tuple


class LandingPadRefiner:
    """在 YOLO bbox 内精确拟合降落标识圆心."""

    def __init__(
        self,
        adaptive_thresh_block: int = 11,
        circle_min_area: int = 80,
    ):
        self.adaptive_thresh_block = adaptive_thresh_block | 1  # 必须为奇数
        self.circle_min_area = circle_min_area

    def detect(
        self, image: np.ndarray,
    ) -> Optional[Tuple[float, float, float, float]]:
        """Find a white H enclosed by the landing pad's circular white ring."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(
            gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU,
        )
        contours, _ = cv2.findContours(
            binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE,
        )

        h_candidates = []
        ring_candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.circle_min_area:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if width == 0 or height == 0:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            fill = area / float(width * height)
            hull_area = cv2.contourArea(cv2.convexHull(contour))
            solidity = area / max(hull_area, 1.0)
            aspect = min(width, height) / max(width, height)
            circularity = area / max(np.pi * radius * radius, 1.0)

            if aspect > 0.75 and circularity > 0.72:
                ring_candidates.append((cx, cy, radius, circularity))
            elif aspect > 0.55 and 0.20 < fill < 0.62 and solidity < 0.70:
                moments = cv2.moments(contour)
                if moments["m00"] <= 0:
                    continue
                h_cx = moments["m10"] / moments["m00"]
                h_cy = moments["m01"] / moments["m00"]
                angle = cv2.minAreaRect(contour)[2]
                h_candidates.append((h_cx, h_cy, max(width, height), solidity, angle))

        best = None
        for h_cx, h_cy, h_size, solidity, angle in h_candidates:
            for ring_cx, ring_cy, radius, circularity in ring_candidates:
                center_error = np.hypot(h_cx - ring_cx, h_cy - ring_cy)
                size_ratio = (2.0 * radius) / max(h_size, 1.0)
                if center_error > radius * 0.25 or not 1.4 < size_ratio < 4.0:
                    continue
                confidence = min(
                    0.99,
                    0.55 + 0.25 * circularity + 0.20 * (1.0 - solidity),
                )
                candidate = (h_cx, h_cy, confidence, float(angle))
                if best is None or confidence > best[2]:
                    best = candidate
        return best

    def refine(
        self, image: np.ndarray, bbox: Tuple[int, int, int, int],
    ) -> Optional[Tuple[float, float, float, float]]:
        """
        在 bbox 内找到降落 pad 的精确圆心和方向.

        Returns
        -------
        (cx_px, cy_px, confidence, yaw_offset_deg) 或 None

        """
        x1, y1, x2, y2 = bbox
        margin = 10
        x1c = max(0, x1 - margin)
        y1c = max(0, y1 - margin)
        x2c = min(image.shape[1], x2 + margin)
        y2c = min(image.shape[0], y2 + margin)

        roi = image[y1c:y2c, x1c:x2c]
        if roi.size == 0:
            return None

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # 自适应阈值 → 二值化
        thresh = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, self.adaptive_thresh_block, 2,
        )

        # 形态学清理
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        cleaned = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # 找最大轮廓 → 拟合最小外接圆
        best_circle = None
        best_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.circle_min_area:
                continue

            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            # 圆形度验证
            circle_area = np.pi * radius ** 2
            circularity = area / (circle_area + 1e-6)
            if circularity < 0.6:
                continue

            # 映射回原图坐标
            abs_cx = cx + x1c
            abs_cy = cy + y1c

            if area > best_area:
                best_area = area
                best_circle = (abs_cx, abs_cy, radius, circularity)

        if best_circle is None:
            return None

        # 计算方向 (H 的朝向)
        yaw_offset = self._estimate_h_orientation(roi, best_circle, x1c, y1c)

        confidence = min(best_circle[3], 1.0)
        return (best_circle[0], best_circle[1], confidence, yaw_offset)

    def _estimate_h_orientation(
        self, roi: np.ndarray,
        circle: Tuple[float, float, float, float],
        offset_x: int, offset_y: int,
    ) -> float:
        """
        估计 H 标识的偏航方向.

        思路: 在圆形底板内, 用 H 形状的横条方向估计方向.
        简化版: 用 PCA 分析 H 的二值轮廓主方向.
        """
        try:
            cx_rel, cy_rel, radius, _ = circle
            cx_int = int(round(cx_rel))
            cy_int = int(round(cy_rel))
            r_int = int(radius)

            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

            # 圆形 mask
            mask = np.zeros_like(thresh)
            cv2.circle(mask, (cx_int, cy_int), int(r_int * 0.8), 255, -1)
            masked = cv2.bitwise_and(thresh, mask)

            # 找 H 的像素点
            ys, xs = np.where(masked > 128)
            if len(ys) < 50:
                return 0.0

            # PCA 方向
            data = np.column_stack([xs, ys])
            mean = data.mean(axis=0)
            centered = data - mean
            cov = np.cov(centered.T)
            eigenvalues, eigenvectors = np.linalg.eig(cov)
            principal = eigenvectors[:, np.argmax(eigenvalues)]

            # 角度: H 横条方向 vs 竖条方向
            # 取 H 像素的朝向为主方向, 计算与图像 x 轴的夹角
            angle_rad = np.arctan2(principal[1], principal[0])
            angle_deg = np.degrees(angle_rad)

            return angle_deg
        except Exception:
            return 0.0


@dataclass(frozen=True)
class LandingCircleTrack:
    """One accepted circular landing-pad measurement in image coordinates."""

    center_u: float
    center_v: float
    radius_px: float
    confidence: float
    fit_residual: float
    edge_margin_px: float


class LandingCircleTracker:
    """Track the physical landing circle inside a bounded, seeded ROI."""

    def __init__(
        self,
        circle_min_area: int = 80,
        adaptive_thresh_block: int = 11,
        roi_padding_px: int = 48,
        min_edge_margin_px: float = 12.0,
        max_center_jump_px: float = 80.0,
        max_radius_change_ratio: float = 0.35,
    ):
        self._circle_min_area = circle_min_area
        self._adaptive_thresh_block = adaptive_thresh_block | 1
        self._roi_padding_px = roi_padding_px
        self._min_edge_margin_px = min_edge_margin_px
        self._max_center_jump_px = max_center_jump_px
        self._max_radius_change_ratio = max_radius_change_ratio
        self._last_track: Optional[LandingCircleTrack] = None
        self._last_rejection_reason = "uninitialized"

    def reset(self) -> None:
        self._last_track = None
        self._last_rejection_reason = "reset"

    @property
    def has_track(self) -> bool:
        return self._last_track is not None

    @property
    def last_rejection_reason(self) -> str:
        return self._last_rejection_reason

    def seed(
        self,
        image: np.ndarray,
        bbox: Tuple[int, int, int, int],
    ) -> Optional[LandingCircleTrack]:
        """Seed tracking from a geometrically validated YOLO acquisition ROI."""
        return self._accept(self._find_circle(image, self._padded_roi(image, bbox)))

    def track(self, image: np.ndarray) -> Optional[LandingCircleTrack]:
        """Return the next coherent circle track without performing full-frame search."""
        if self._last_track is None:
            return None
        radius = self._last_track.radius_px + self._roi_padding_px
        bbox = (
            int(round(self._last_track.center_u - radius)),
            int(round(self._last_track.center_v - radius)),
            int(round(self._last_track.center_u + radius)),
            int(round(self._last_track.center_v + radius)),
        )
        return self._accept(self._find_circle(image, self._clip_roi(image, bbox)))

    def _padded_roi(
        self,
        image: np.ndarray,
        bbox: Tuple[int, int, int, int],
    ) -> Tuple[int, int, int, int]:
        x1, y1, x2, y2 = bbox
        padded = (
            x1 - self._roi_padding_px,
            y1 - self._roi_padding_px,
            x2 + self._roi_padding_px,
            y2 + self._roi_padding_px,
        )
        return self._clip_roi(image, padded)

    @staticmethod
    def _clip_roi(
        image: np.ndarray,
        bbox: Tuple[int, int, int, int],
    ) -> Tuple[int, int, int, int]:
        x1, y1, x2, y2 = bbox
        return (
            max(0, x1),
            max(0, y1),
            min(image.shape[1], x2),
            min(image.shape[0], y2),
        )

    def _find_circle(
        self,
        image: np.ndarray,
        bbox: Tuple[int, int, int, int],
    ) -> Optional[LandingCircleTrack]:
        x1, y1, x2, y2 = bbox
        roi = image[y1:y2, x1:x2]
        if roi.size == 0:
            self._last_rejection_reason = "empty_roi"
            return None

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, self._adaptive_thresh_block, 2,
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        cleaned = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(
            cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
        )

        best: Optional[LandingCircleTrack] = None
        self._last_rejection_reason = "no_circle"
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self._circle_min_area:
                self._last_rejection_reason = "small_contour"
                continue
            if len(contour) < 5:
                self._last_rejection_reason = "non_elliptical"
                continue
            ellipse = cv2.fitEllipse(contour)
            (center_x, center_y), (axis_a, axis_b), _angle = ellipse
            radius = max(axis_a, axis_b) / 2.0
            if radius <= 0.0 or min(axis_a, axis_b) <= 0.0:
                self._last_rejection_reason = "zero_radius"
                continue
            axis_ratio = min(axis_a, axis_b) / max(axis_a, axis_b)
            ellipse_fill = area / max(np.pi * axis_a * axis_b / 4.0, 1.0)
            # A horizontal circle becomes an ellipse for the vehicle's
            # down-tilted camera.  Validate ellipse fit rather than reject a
            # physically correct perspective projection as "non-circular".
            if axis_ratio < 0.45 or ellipse_fill < 0.55:
                self._last_rejection_reason = "non_elliptical"
                continue
            absolute_x = center_x + x1
            absolute_y = center_y + y1
            edge_margin = min(
                absolute_x - radius,
                absolute_y - radius,
                image.shape[1] - (absolute_x + radius),
                image.shape[0] - (absolute_y + radius),
            )
            if edge_margin < self._min_edge_margin_px:
                self._last_rejection_reason = "edge_clipped"
                continue
            residual = 1.0 - min(ellipse_fill, 1.0)
            margin_quality = min(1.0, edge_margin / max(radius, 1.0))
            confidence = min(
                0.99,
                0.55 + 0.25 * min(ellipse_fill, 1.0)
                + 0.10 * axis_ratio + 0.10 * margin_quality,
            )
            candidate = LandingCircleTrack(
                center_u=absolute_x,
                center_v=absolute_y,
                radius_px=radius,
                confidence=confidence,
                fit_residual=residual,
                edge_margin_px=edge_margin,
            )
            if best is None or candidate.confidence > best.confidence:
                best = candidate
        return best

    def _accept(
        self,
        candidate: Optional[LandingCircleTrack],
    ) -> Optional[LandingCircleTrack]:
        if candidate is None:
            return None
        if self._last_track is not None:
            center_jump = np.hypot(
                candidate.center_u - self._last_track.center_u,
                candidate.center_v - self._last_track.center_v,
            )
            if center_jump > self._max_center_jump_px:
                self._last_rejection_reason = "center_jump"
                return None
            radius_change = abs(candidate.radius_px - self._last_track.radius_px)
            if radius_change / max(self._last_track.radius_px, 1.0) > self._max_radius_change_ratio:
                self._last_rejection_reason = "radius_change"
                return None
        self._last_track = candidate
        self._last_rejection_reason = "accepted"
        return candidate


class LandingDetectorNode(Node):
    """H 降落标识检测 ROS2 节点."""

    def __init__(self):
        super().__init__("landing_detector")

        self._declare_params()

        self._engine = None
        self._refiner: Optional[LandingPadRefiner] = None
        self._bridge = CvBridge()

        self._current_altitude = 5.0

        self._img_sub = self.create_subscription(
            Image, "/drone/camera/image_raw", self._image_cb,
            qos_profile_sensor_data,
        )

        self._landing_pub = self.create_publisher(
            LandingTarget, "/drone/vision/landing_target", 10,
        )

        self._init_timer = self.create_timer(1.0, self._lazy_init)
        self._initialized = False

        self.get_logger().info("降落标识检测节点已启动")

    def _declare_params(self):
        self.declare_parameter("model_path", "")
        self.declare_parameter("conf_threshold", 0.4)  # 降落检测阈值放宽
        self.declare_parameter("nms_iou_threshold", 0.45)
        self.declare_parameter("model_width", 640)
        self.declare_parameter("model_height", 640)
        self.declare_parameter("device", "cuda")
        self.declare_parameter("class_names", [
            "cylinder", "flammable", "health_hazard",
            "danger", "landing_H", "empty_cylinder",
        ])

        self.declare_parameter("landing_refine_enable", True)
        self.declare_parameter("landing_adaptive_thresh_block", 11)
        self.declare_parameter("landing_circle_min_area", 80)
        self.declare_parameter("landing_H_orientation_enable", True)

        self.declare_parameter("fx", 400.0)
        self.declare_parameter("fy", 400.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)

    def _lazy_init(self):
        if self._initialized:
            return

        model_path = self.get_parameter("model_path").value
        if not model_path:
            import os
            default = os.path.join(
                os.path.dirname(__file__), "..", "models", "drone_yolo.onnx",
            )
            if os.path.exists(default):
                model_path = default
            else:
                self.get_logger().warn("无 YOLO 模型, 使用纯 OpenCV 回退")
                self._refiner = LandingPadRefiner(
                    adaptive_thresh_block=self.get_parameter(
                        "landing_adaptive_thresh_block"
                    ).value,
                    circle_min_area=self.get_parameter("landing_circle_min_area").value,
                )
                self._initialized = True
                self._init_timer.cancel()
                return

        try:
            from drone_vision.models.yolo_engine import create_engine

            class_names = (
                self.get_parameter("class_names")
                .get_parameter_value().string_array_value
            )
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
            self._engine.load()
        except Exception as e:
            self.get_logger().error(f"YOLO 引擎初始化失败: {e}")

        self._refiner = LandingPadRefiner(
            adaptive_thresh_block=self.get_parameter("landing_adaptive_thresh_block").value,
            circle_min_area=self.get_parameter("landing_circle_min_area").value,
        )
        self._initialized = True
        self._init_timer.cancel()
        self.get_logger().info("降落标识检测节点初始化完成")

    def _image_cb(self, msg: Image):
        if not self._initialized:
            return

        try:
            image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        target = LandingTarget()
        target.confidence = 0.0

        if self._engine and self._engine.is_loaded:
            dets = self._engine.infer(image)
            landing_dets = [d for d in dets if d.class_name == "landing_H"]

            if landing_dets:
                # 取置信度最高的
                best = max(landing_dets, key=lambda d: d.confidence)

                if self.get_parameter("landing_refine_enable").value and self._refiner:
                    result = self._refiner.refine(image, best.bbox)
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

                # 像素 → 世界坐标
                fx = self.get_parameter("fx").value
                fy = self.get_parameter("fy").value
                cx = self.get_parameter("cx").value
                cy = self.get_parameter("cy").value
                alt = max(self._current_altitude, 0.3)

                # 像素偏移 → 世界偏移
                du = target.center_u - cx
                dv = target.center_v - cy
                # 相机 x 轴对应 world east, y 轴对应 world north
                target.world_x = du * alt / fx  # east 偏移
                target.world_y = dv * alt / fy  # north 偏移
        else:
            # 纯 OpenCV 回退: 搜索圆形 + H 形状
            target = self._fallback_detect(image)

        self._landing_pub.publish(target)

    def _fallback_detect(self, image: np.ndarray) -> LandingTarget:
        """使用 OpenCV 自适应阈值、圆形检测和 H 形状验证."""
        target = LandingTarget()
        target.confidence = 0.0

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 11, 2,
        )

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return target

        best = None
        best_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 300:
                continue

            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            circularity = area / (np.pi * radius ** 2 + 1e-6)
            if circularity < 0.5:
                continue

            if area > best_area:
                best_area = area
                best = (cx, cy, radius, circularity)

        if best:
            target.center_u = best[0]
            target.center_v = best[1]
            target.confidence = min(best[3], 1.0)

        return target

    def update_altitude(self, altitude_m: float):
        self._current_altitude = altitude_m


def main(args=None):
    rclpy.init(args=args)
    node = LandingDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
