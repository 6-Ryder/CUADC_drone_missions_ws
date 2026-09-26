#!/usr/bin/env python3
"""Bridge Gazebo camera frames to ROS without JSON/Base64 conversion.

Gazebo delivers frames on a transport callback thread. The ROS timer publishes
only the newest frame, so image processing cannot build an old-frame backlog.
Run with ``python3 -s``: system Gazebo protobuf bindings must take priority
over a user's Python packages.
"""

import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node as RosNode
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from gz.transport13 import Node as GazeboNode
from gz.msgs10.image_pb2 import Image as GazeboImage


GZ_TOPIC = sys.argv[1] if len(sys.argv) > 1 else (
    "/world/drone_field/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"
)
ROS_TOPIC = sys.argv[2] if len(sys.argv) > 2 else "/drone/camera/image_compressed"
PUBLISH_HZ = 15.0


class GzCameraBridge(RosNode):
    def __init__(self):
        super().__init__("gz_camera_bridge")
        self._pub = self.create_publisher(
            CompressedImage, ROS_TOPIC, qos_profile_sensor_data)
        self._gz_node = GazeboNode()
        self._lock = threading.Lock()
        self._latest = None
        self._received = 0
        self._published = 0
        self._last_log = time.monotonic()
        self._gz_node.subscribe(GazeboImage, GZ_TOPIC, self._capture)
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_latest)
        self.get_logger().info(
            f"Gazebo 原生相机桥接: {GZ_TOPIC} → {ROS_TOPIC}, "
            f"发布上限 {PUBLISH_HZ:.0f} Hz"
        )

    def _capture(self, frame):
        if frame.width <= 0 or frame.height <= 0 or not frame.data:
            return
        if frame.pixel_format_type not in (3, 8):  # RGB_INT8 / BGR_INT8
            return
        stamp_ns = time.time_ns()
        with self._lock:
            self._latest = (frame, stamp_ns)
            self._received += 1

    def _publish_latest(self):
        with self._lock:
            latest = self._latest
            self._latest = None
        if latest is None:
            return

        frame, stamp_ns = latest
        if frame.step < frame.width * 3 or len(frame.data) < frame.step * frame.height:
            return
        pixels = np.frombuffer(frame.data, dtype=np.uint8).reshape(
            frame.height, frame.step)[:, :frame.width * 3].reshape(
            frame.height, frame.width, 3)
        bgr = (cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
               if frame.pixel_format_type == 3 else pixels)
        encoded, jpeg = cv2.imencode(
            ".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not encoded:
            return

        msg = CompressedImage()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 10**9)
        msg.header.frame_id = "camera_link"
        msg.format = "jpeg"
        msg.data = jpeg.tobytes()
        self._pub.publish(msg)
        self._published += 1

        now = time.monotonic()
        if now - self._last_log >= 10.0:
            self.get_logger().info(
                f"相机帧: 收到 {self._received}, 发布 {self._published} "
                f"({frame.width}x{frame.height} jpeg {len(msg.data)//1024}KB)"
            )
            self._last_log = now

    def close(self):
        self._gz_node.unsubscribe(GZ_TOPIC)


def main():
    os.environ.setdefault("GZ_IP", "127.0.0.1")
    rclpy.init()
    node = GzCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
