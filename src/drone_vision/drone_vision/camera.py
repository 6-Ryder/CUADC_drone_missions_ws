"""
相机驱动节点.

SITL 模式: 通过 ros_gz_bridge 订阅 Gazebo 相机话题
实飞模式: 通过 V4L2 读取指定的 USB 相机

参数:
  sim_mode: true → 订阅 /camera/image (Gazebo)
            false → 打开指定的 V4L2 设备

发布:
  /drone/camera/image_raw   (sensor_msgs/Image)
  /drone/camera/camera_info (sensor_msgs/CameraInfo)
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
from cv_bridge import CvBridge
import time
import threading

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


CAMERA_INFO_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class CameraDriverNode(Node):
    """相机驱动 — SITL 或实飞."""

    def __init__(self):
        super().__init__("camera_driver")

        self._declare_params()
        self._bridge = CvBridge()
        self._cap = None
        self._running = False
        self._thread = None

        self.sim_mode = self.get_parameter("sim_mode").value

        # 发布
        self._img_pub = self.create_publisher(
            Image, "/drone/camera/image_raw", qos_profile_sensor_data,
        )
        self._info_pub = self.create_publisher(
            CameraInfo, "/drone/camera/camera_info", CAMERA_INFO_QOS,
        )
        if self.sim_mode:
            self._init_sitl()
        else:
            self._init_real()
        self._publish_camera_info()

        self.get_logger().info(f"相机驱动已启动 (sim_mode={self.sim_mode})")

    def _declare_params(self):
        self.declare_parameter("sim_mode", True)
        self.declare_parameter("camera_device", "/dev/video0")  # 摄像头设备路径
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
            Image, "/camera/image", self._sitl_image_cb,
            qos_profile_sensor_data,
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
        """实飞: 通过 V4L2 读取指定相机."""
        self.get_logger().info("实飞模式: 初始化 V4L2 相机...")
        # 初始化失败必须终止节点，否则启动脚本会把无图像的进程误判为健康。
        self._open_camera()
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _open_camera(self):
        """通过 V4L2 打开配置指定的 USB 相机."""
        if not HAS_CV2:
            raise RuntimeError("OpenCV 未安装, 无法启动 V4L2 相机")
        device = self.get_parameter("camera_device").value
        w = self.get_parameter("camera_width").value
        h = self.get_parameter("camera_height").value
        fps = self.get_parameter("camera_fps").value

        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"无法通过 V4L2 打开相机设备: {device}")

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        actual_w = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
        actual_h = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        actual_fps = float(cap.get(cv2.CAP_PROP_FPS))
        if (actual_w, actual_h) != (w, h):
            cap.release()
            raise RuntimeError(
                f"相机不支持任务分辨率 {w}x{h}, 实际协商为 "
                f"{actual_w}x{actual_h}"
            )
        self._actual_width = actual_w
        self._actual_height = actual_h
        self._actual_fps = actual_fps
        self._cap = cap
        self.get_logger().info(
            f"V4L2 相机已打开: {device} "
            f"({actual_w}x{actual_h} @ {actual_fps:.1f}Hz)"
        )

    def _capture_loop(self):
        """实飞采集循环 — 丢弃积压帧, 始终取最新."""
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

            # 手动构造 Image, 避免不同 cv_bridge 构建之间的类型兼容问题。
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_optical_frame"
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = frame.shape[1] * 3
            msg.data = frame.tobytes()
            self._img_pub.publish(msg)

            elapsed = time.time() - t0
            if elapsed < period:
                time.sleep(period - elapsed)

    def _publish_camera_info(self):
        """发布相机内参."""
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_frame"
        msg.height = getattr(
            self, "_actual_height", self.get_parameter("camera_height").value,
        )
        msg.width = getattr(
            self, "_actual_width", self.get_parameter("camera_width").value,
        )
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
        if self._cap is not None:
            self._cap.release()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
