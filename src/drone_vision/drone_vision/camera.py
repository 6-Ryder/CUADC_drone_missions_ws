"""
相机驱动节点.

SITL 模式: 通过 ros_gz_bridge 订阅 Gazebo 相机话题
实飞模式: 通过 GStreamer pipeline 读取 Jetson 相机 (CSI/USB)

参数:
  sim_mode: true → 订阅 /camera/image (Gazebo)
            false → 打开 GStreamer pipeline

发布:
  /drone/camera/image_raw   (sensor_msgs/Image)
  /drone/camera/camera_info (sensor_msgs/CameraInfo)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import time
import threading

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


class CameraDriverNode(Node):
    """相机驱动 — SITL 或实飞."""

    def __init__(self):
        super().__init__("camera_driver")

        self._declare_params()
        self._bridge = CvBridge()

        self.sim_mode = self.get_parameter("sim_mode").value

        # 发布
        self._img_pub = self.create_publisher(
            Image, "/drone/camera/image_raw", 10,
        )
        self._info_pub = self.create_publisher(
            CameraInfo, "/drone/camera/camera_info", 10,
        )
        # 发送一次 camera_info
        self._publish_camera_info()

        if self.sim_mode:
            self._init_sitl()
        else:
            self._init_real()

        self.get_logger().info(f"相机驱动已启动 (sim_mode={self.sim_mode})")

    def _declare_params(self):
        self.declare_parameter("sim_mode", True)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 30.0)
        self.declare_parameter("fx", 400.0)
        self.declare_parameter("fy", 400.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)

    # ----------------------------------------------------------
    # SITL 模式
    # ----------------------------------------------------------

    def _init_sitl(self):
        """SITL: 订阅 Gazebo 相机话题 (通过 ros_gz_bridge)."""
        self.get_logger().info("SITL 模式: 等待 /camera/image 话题...")
        self._gz_sub = self.create_subscription(
            Image, "/camera/image", self._sitl_image_cb, 10,
        )

    def _sitl_image_cb(self, msg: Image):
        """Gazebo 相机回调 — 直接转发."""
        # 修改 frame_id 和时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_frame"
        self._img_pub.publish(msg)

    # ----------------------------------------------------------
    # 实飞模式
    # ----------------------------------------------------------

    def _init_real(self):
        """实飞: GStreamer pipeline 读取 Jetson 相机."""
        self.get_logger().info("实飞模式: 初始化 GStreamer 相机...")
        self._cap = None
        self._running = False
        self._thread = None

        # 尝试打开相机
        try:
            self._open_camera()
            self._running = True
            self._thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._thread.start()
        except Exception as e:
            self.get_logger().error(f"相机初始化失败: {e}")

    def _open_camera(self):
        """打开 GStreamer 相机."""
        # 尝试多个 pipeline:
        #   - nvarguscamerasrc (Jetson MIPI CSI)
        #   - v4l2src (USB camera)
        pipelines = [
            # Jetson MIPI CSI (Raspberry Pi Camera v2)
            (
                "nvarguscamerasrc sensor-id=0 ! "
                "video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1 ! "
                "nvvidconv flip-method=2 ! "
                "video/x-raw,format=BGRx ! "
                "videoconvert ! video/x-raw,format=BGR ! appsink"
            ),
            # USB camera
            (
                "v4l2src device=/dev/video0 ! "
                "video/x-raw,width=640,height=480,framerate=30/1 ! "
                "videoconvert ! video/x-raw,format=BGR ! appsink"
            ),
        ]

        for i, pipeline in enumerate(pipelines):
            self.get_logger().info(f"尝试 pipeline {i}: {pipeline[:80]}...")
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                self._cap = cap
                self.get_logger().info(f"相机已打开 (pipeline {i})")
                return

        # 回退: 尝试简单设备ID
        for dev_id in [0, 1]:
            cap = cv2.VideoCapture(dev_id)
            if cap.isOpened():
                self._cap = cap
                self.get_logger().info(f"相机已打开 (device /dev/video{dev_id})")
                return

        raise RuntimeError("无法打开任何相机设备")

    def _capture_loop(self):
        """实飞采集循环."""
        fps = self.get_parameter("camera_fps").value
        period = 1.0 / max(fps, 1.0)

        while self._running and rclpy.ok():
            t0 = time.time()
            if self._cap is None:
                time.sleep(0.1)
                continue

            ret, frame = self._cap.read()
            if not ret:
                self.get_logger().warn("相机读取失败, 重试...")
                time.sleep(0.5)
                continue

            # 发布
            msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_optical_frame"
            self._img_pub.publish(msg)

            elapsed = time.time() - t0
            if elapsed < period:
                time.sleep(period - elapsed)

    def _publish_camera_info(self):
        """发布相机内参."""
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_frame"
        msg.height = self.get_parameter("camera_height").value
        msg.width = self.get_parameter("camera_width").value
        msg.k = [
            self.get_parameter("fx").value, 0.0, self.get_parameter("cx").value,
            0.0, self.get_parameter("fy").value, self.get_parameter("cy").value,
            0.0, 0.0, 1.0,
        ]
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._info_pub.publish(msg)

    # ----------------------------------------------------------
    # 生命周期
    # ----------------------------------------------------------

    def destroy_node(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cap:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
