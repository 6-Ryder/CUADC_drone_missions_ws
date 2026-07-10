#!/usr/bin/env python3
"""
实时视觉检测画面查看器 — 订阅 /drone/camera/image_raw 和检测话题, 显示标注画面.

订阅:
  /drone/camera/image_raw      原始图像
  /drone/vision/cylinders      圆筒检测
  /drone/vision/landing_target H降落标识
  /drone/vision/coverage       覆盖状态

按 q 退出 | 按 s 截图保存
"""

import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from drone_interfaces.msg import CylinderArray, LandingTarget, CoverageStatus


class VisionViewer(Node):
    """订阅视觉管线话题, 渲染标注画面."""

    def __init__(self):
        super().__init__("vision_viewer")

        # 最新帧
        self._image: np.ndarray | None = None
        self._cylinders: list = []
        self._markers: list = []
        self._landing: LandingTarget | None = None
        self._coverage: float = 0.0
        self._fps_counter = []
        self._frame_count = 0
        self._last_fps_print = time.time()

        # 订阅 (qos=1 只保留最新帧, 避免旧帧延迟)
        self._img_sub = self.create_subscription(
            Image, "/drone/camera/image_raw", self._on_image, 1,
        )
        self._cyl_sub = self.create_subscription(
            CylinderArray, "/drone/vision/cylinders", self._on_cylinders, 1,
        )
        self._marker_sub = self.create_subscription(
            CylinderArray, "/drone/vision/markers", self._on_markers, 1,
        )
        self._land_sub = self.create_subscription(
            LandingTarget, "/drone/vision/landing_target", self._on_landing, 1,
        )
        self._cov_sub = self.create_subscription(
            CoverageStatus, "/drone/vision/coverage", self._on_coverage, 1,
        )

        self.get_logger().info("画面查看器已就绪")

    def _on_image(self, msg: Image):
        try:
            h, w = msg.height, msg.width
            self._image = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, w, 3)
        except Exception:
            pass

    def _on_cylinders(self, msg: CylinderArray):
        self._cylinders = msg.cylinders

    def _on_markers(self, msg: CylinderArray):
        self._markers = msg.cylinders

    def _on_landing(self, msg: LandingTarget):
        self._landing = msg

    def _on_coverage(self, msg: CoverageStatus):
        self._coverage = msg.coverage_ratio

    @property
    def has_image(self) -> bool:
        return self._image is not None

    def render(self) -> np.ndarray:
        """返回标注后的画面 (BGR)."""
        if self._image is None:
            return np.zeros((480, 640, 3), dtype=np.uint8)

        frame = self._image.copy()
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        t = time.time()

        # ---- FPS ----
        self._fps_counter.append(t)
        self._fps_counter = [x for x in self._fps_counter if t - x < 2.0]
        fps = len(self._fps_counter) / 2.0 if len(self._fps_counter) > 1 else 0

        # ---- 画面中心十字 ----
        cv2.circle(frame, (cx, cy), 6, (255, 0, 0), -1)
        cv2.line(frame, (cx, 0), (cx, h), (255, 0, 0), 1)
        cv2.line(frame, (0, cy), (w, cy), (255, 0, 0), 1)

        # ---- 圆筒检测 (绿色 bbox + 黄色椭圆) ----
        for c in self._cylinders:
            u, v = int(c.center_u), int(c.center_v)
            # YOLO 绿色边界框
            if c.bbox_x2 > c.bbox_x1 and c.bbox_y2 > c.bbox_y1:
                cv2.rectangle(frame, (c.bbox_x1, c.bbox_y1),
                              (c.bbox_x2, c.bbox_y2), (0, 255, 0), 2)
            # OpenCV 精修圆心 (红点) + 拟合圆/椭圆 (黄色)
            cv2.circle(frame, (u, v), 8, (0, 0, 255), -1)
            r_px = int(c.radius * max(w, h) / 5.0)  # 近似的像素半径
            if r_px > 2:
                cv2.circle(frame, (u, v), r_px, (0, 255, 255), 2)
            label = f"CYL#{c.id} r={c.radius:.2f}m {c.confidence:.2f}"
            cv2.putText(frame, label, (u + 15, v - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

        # ---- 投放区蓝色标识 (橙色框 + 橙色圆心) ----
        for m in self._markers:
            if m.bbox_x2 > m.bbox_x1 and m.bbox_y2 > m.bbox_y1:
                cv2.rectangle(frame, (m.bbox_x1, m.bbox_y1),
                              (m.bbox_x2, m.bbox_y2), (0, 165, 255), 2)
            mu, mv = int(m.center_u), int(m.center_v)
            cv2.circle(frame, (mu, mv), 6, (0, 165, 255), -1)
            label = f"MARKER {m.confidence:.2f}"
            cv2.putText(frame, label, (mu + 10, mv - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 2)

        # ---- H 降落标识 (青色) ----
        if self._landing and self._landing.confidence > 0.1:
            lu, lv = int(self._landing.center_u), int(self._landing.center_v)
            cv2.circle(frame, (lu, lv), 15, (255, 255, 0), 2)
            cv2.putText(frame, f"H yaw={self._landing.yaw_correction:.1f}deg {self._landing.confidence:.2f}",
                        (lu + 20, lv), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 2)

        # ---- HUD 面板 ----
        # 半透明底栏
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, h - 60), (w, h), (0, 0, 0), -1)
        frame = cv2.addWeighted(frame, 0.7, overlay, 0.3, 0)

        line_h = h - 40
        cv2.putText(frame, f"FPS:{fps:.0f}  {w}x{h}",
                    (10, line_h), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"CYL:{len(self._cylinders)}  |  MKR:{len(self._markers)}  |  H:{'YES' if self._landing and self._landing.confidence > 0.1 else 'no'}  |  COV:{self._coverage:.1%}",
                    (200, line_h), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # 无检测提示
        if not self._cylinders and not (self._landing and self._landing.confidence > 0.1):
            cv2.putText(frame, "NO TARGET — point camera at white cylinder or blue marker",
                        (cx - 220, cy - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 100, 255), 1)

        return frame


def main():
    rclpy.init(args=sys.argv)
    viewer = VisionViewer()

    print("=" * 60)
    print("  视觉检测实时查看器")
    print("  按键: q=退出  s=截图")
    print("  绿色圆=圆筒  青色圆=H降落标识")
    print("=" * 60)

    screenshot_dir = os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "log", "vision_screenshots",
    )
    os.makedirs(screenshot_dir, exist_ok=True)
    screenshot_count = 0

    try:
        while rclpy.ok():
            # 一次性消费所有积压回调, 只保留最新帧
            for _ in range(20):
                rclpy.spin_once(viewer, timeout_sec=0.001)

            if viewer.has_image:
                frame = viewer.render()
                cv2.imshow("Vision Debug — Drone Detection", frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord("q"):
                    break
                elif key == ord("s"):
                    screenshot_count += 1
                    path = os.path.join(screenshot_dir, f"screenshot_{screenshot_count:04d}.png")
                    cv2.imwrite(path, frame)
                    print(f"  截图已保存: {path}")

            else:
                # 等待图像到来
                rclpy.spin_once(viewer, timeout_sec=0.1)
                cv2.waitKey(1)

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()
        print("\n查看器已关闭")


if __name__ == "__main__":
    main()
