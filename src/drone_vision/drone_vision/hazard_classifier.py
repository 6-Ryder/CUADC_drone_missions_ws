"""
危险品分类节点 — YOLO 检测 + 颜色回退验证.

流程:
  1. 订阅 /drone/camera/image_raw
  2. YOLO 推理 → 检出 flammable / health_hazard / danger / empty_cylinder
  3. 对低置信度检测启用颜色直方图回退验证
  4. 关联危险品到最近的圆筒 (通过空间距离)
  5. 发布 HazardArray 话题

发布:
  /drone/vision/hazards  (HazardArray)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from drone_interfaces.msg import HazardDetection, HazardArray
from cv_bridge import CvBridge
import numpy as np
import cv2
from typing import List, Tuple, Optional


# 危险品类型常量
HAZARD_EMPTY = 0
HAZARD_FLAMMABLE = 1
HAZARD_HEALTH = 2
HAZARD_DANGER = 3

# 类别名 → 类型映射
CLASS_TO_HAZARD = {
    "empty_cylinder": HAZARD_EMPTY,
    "flammable": HAZARD_FLAMMABLE,
    "health_hazard": HAZARD_HEALTH,
    "danger": HAZARD_DANGER,
}


# ============================================================
# 颜色回退验证器
# ============================================================

class ColorVerifier:
    """
    YOLO 低置信度时的颜色验证回退.

    危险品标识颜色特征:
      - 易燃 (flammable): 橙色底, 主色调 H ∈ [5, 25]
      - 健康危害 (health_hazard): 蓝色底, H ∈ [100, 130]
      - 危险 (danger): 红色底, H ∈ [0, 10] ∪ [160, 180]
    """

    # HSV 颜色范围
    COLOR_RANGES = {
        HAZARD_FLAMMABLE: {
            "lower": np.array([5, 80, 80]),
            "upper": np.array([25, 255, 255]),
        },
        HAZARD_HEALTH: {
            "lower": np.array([100, 80, 80]),
            "upper": np.array([130, 255, 255]),
        },
        HAZARD_DANGER: {
            "lower1": np.array([0, 80, 80]),
            "upper1": np.array([10, 255, 255]),
            "lower2": np.array([160, 80, 80]),
            "upper2": np.array([180, 255, 255]),
        },
    }

    def __init__(self, confirm_ratio: float = 0.3):
        self.confirm_ratio = confirm_ratio

    def verify(
        self, image: np.ndarray, bbox: Tuple[int, int, int, int],
        hazard_type: int,
    ) -> Tuple[bool, float]:
        """
        验证 bbox 区域是否符合指定危险品颜色.

        Returns
        -------
        (is_match, color_ratio) — color_ratio 为匹配像素占比.
        """
        x1, y1, x2, y2 = bbox
        roi = image[y1:y2, x1:x2]
        if roi.size == 0:
            return (False, 0.0)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        if hazard_type == HAZARD_DANGER:
            # 红色需要两段 HSV 范围
            mask1 = cv2.inRange(hsv, self.COLOR_RANGES[HAZARD_DANGER]["lower1"],
                                self.COLOR_RANGES[HAZARD_DANGER]["upper1"])
            mask2 = cv2.inRange(hsv, self.COLOR_RANGES[HAZARD_DANGER]["lower2"],
                                self.COLOR_RANGES[HAZARD_DANGER]["upper2"])
            mask = cv2.bitwise_or(mask1, mask2)
        elif hazard_type in self.COLOR_RANGES:
            mask = cv2.inRange(hsv, self.COLOR_RANGES[hazard_type]["lower"],
                               self.COLOR_RANGES[hazard_type]["upper"])
        else:
            return (False, 0.0)

        ratio = np.count_nonzero(mask) / mask.size
        return (ratio >= self.confirm_ratio, ratio)


# ============================================================
# ROS2 节点
# ============================================================

class HazardClassifierNode(Node):
    """危险品分类 ROS2 节点."""

    def __init__(self):
        super().__init__("hazard_classifier")

        self._declare_params()

        self._engine = None
        self._verifier: Optional[ColorVerifier] = None
        self._bridge = CvBridge()

        # 已知圆筒位置 (从 cylinder_detector 话题更新)
        self._known_cylinders: List[Tuple[float, float]] = []

        # 订阅
        self._img_sub = self.create_subscription(
            Image, "/drone/camera/image_raw", self._image_cb, 10,
        )

        # 发布
        self._hazard_pub = self.create_publisher(
            HazardArray, "/drone/vision/hazards", 10,
        )

        self._init_timer = self.create_timer(1.0, self._lazy_init)
        self._initialized = False

        self.get_logger().info("危险品分类节点已启动 (等待模型加载...)")

    def _declare_params(self):
        self.declare_parameter("model_path", "")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("nms_iou_threshold", 0.45)
        self.declare_parameter("model_width", 640)
        self.declare_parameter("model_height", 640)
        self.declare_parameter("device", "cuda")
        self.declare_parameter("class_names", [
            "cylinder", "flammable", "health_hazard",
            "danger", "landing_H", "empty_cylinder",
        ])
        self.declare_parameter("hazard_enable_color_fallback", True)
        self.declare_parameter("hazard_color_confirm_ratio", 0.3)

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
                self.get_logger().warn("无 YOLO 模型, 将使用纯颜色分割回退")
                self._verifier = ColorVerifier(
                    confirm_ratio=self.get_parameter("hazard_color_confirm_ratio").value,
                )
                self._initialized = True
                self._init_timer.cancel()
                return

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
            self._engine.load()
        except Exception as e:
            self.get_logger().error(f"YOLO 引擎初始化失败: {e}")

        self._verifier = ColorVerifier(
            confirm_ratio=self.get_parameter("hazard_color_confirm_ratio").value,
        )
        self._initialized = True
        self._init_timer.cancel()
        self.get_logger().info("危险品分类节点初始化完成")

    def _image_cb(self, msg: Image):
        if not self._initialized:
            return

        try:
            image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        hazards: List[HazardDetection] = []

        if self._engine and self._engine.is_loaded:
            dets = self._engine.infer(image)
            # 筛选危险品类
            hazard_classes = {"flammable", "health_hazard", "danger", "empty_cylinder"}

            for d in dets:
                if d.class_name not in hazard_classes:
                    continue

                confirmed_type = CLASS_TO_HAZARD.get(d.class_name, HAZARD_EMPTY)
                final_conf = d.confidence

                # 低置信度 → 颜色回退验证
                enable_color = self.get_parameter("hazard_enable_color_fallback").value
                if enable_color and d.confidence < 0.7 and self._verifier:
                    is_match, color_ratio = self._verifier.verify(
                        image, d.bbox, confirmed_type,
                    )
                    if not is_match:
                        # 尝试其他危险品颜色
                        for alt_type in [HAZARD_FLAMMABLE, HAZARD_HEALTH, HAZARD_DANGER]:
                            if alt_type == confirmed_type:
                                continue
                            is_alt, alt_ratio = self._verifier.verify(
                                image, d.bbox, alt_type,
                            )
                            if is_alt and alt_ratio > color_ratio:
                                confirmed_type = alt_type
                                final_conf = max(final_conf, alt_ratio)
                                break
                        else:
                            continue  # 颜色也不匹配, 丢弃
                    final_conf = max(final_conf, color_ratio)

                h = HazardDetection()
                h.id = -1  # 后续由 mission 节点关联圆筒
                h.hazard_type = confirmed_type
                h.confidence = float(final_conf)
                h.center_x = 0.0  # 像素→世界 (需要高度+内参)
                h.center_y = 0.0
                h.world_x = 0.0
                h.world_y = 0.0
                hazards.append(h)
        else:
            # 纯颜色分割回退 (无 YOLO)
            hazards = self._fallback_color_detect(image)

        # 发布
        msg_array = HazardArray()
        msg_array.header.stamp = self.get_clock().now().to_msg()
        msg_array.header.frame_id = "world"
        msg_array.hazards = hazards
        self._hazard_pub.publish(msg_array)

    def _fallback_color_detect(self, image: np.ndarray) -> List[HazardDetection]:
        """
        纯颜色分割回退检测.

        对整个图像搜索橙色/蓝色/红色区域,
        然后用轮廓找到标识板.
        """
        hazards = []
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        color_configs = [
            (HAZARD_FLAMMABLE, [
                (np.array([5, 80, 80]), np.array([25, 255, 255])),
            ]),
            (HAZARD_HEALTH, [
                (np.array([100, 80, 80]), np.array([130, 255, 255])),
            ]),
            (HAZARD_DANGER, [
                (np.array([0, 80, 80]), np.array([10, 255, 255])),
                (np.array([160, 80, 80]), np.array([180, 255, 255])),
            ]),
        ]

        for hazard_type, ranges in color_configs:
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                m = cv2.inRange(hsv, lower, upper)
                mask = cv2.bitwise_or(mask, m)

            # 形态学清理
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 100:  # 最小面积
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                # 宽高比接近 1 (标识是正方形 12cm×12cm)
                aspect = w / (h + 1e-6)
                if aspect < 0.5 or aspect > 2.0:
                    continue

                det = HazardDetection()
                det.id = -1
                det.hazard_type = hazard_type
                det.confidence = min(area / 500.0, 1.0)
                det.center_x = float(x + w / 2)
                det.center_y = float(y + h / 2)
                hazards.append(det)

        return hazards


def main(args=None):
    rclpy.init(args=args)
    node = HazardClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
