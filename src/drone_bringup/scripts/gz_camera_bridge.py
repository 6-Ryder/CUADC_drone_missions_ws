#!/usr/bin/env python3
"""
Gazebo Camera → ROS2 Image Bridge.

解决 ros_gz_image C++ bridge 的 DDS 大消息传输问题。
通过 gz topic --json-output 抓取单帧，base64 解码后发布为 ROS2 Image。

用法: python3 gz_camera_bridge.py [gz_camera_topic] [ros_image_topic]
默认: gz camera → /drone/camera/image_raw @ 10 FPS
"""
import subprocess
import os
import json
import base64
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

GZ_TOPIC = sys.argv[1] if len(sys.argv) > 1 else (
    "/world/drone_field/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"
)
ROS_TOPIC = sys.argv[2] if len(sys.argv) > 2 else "/drone/camera/image_raw"
GZ_ENV = {**os.environ, "GZ_IP": "127.0.0.1"}


class GzCameraBridge(Node):
    """Gazebo Transport → ROS2 Image 桥接节点."""

    def __init__(self):
        super().__init__("gz_camera_bridge")
        self._pub = self.create_publisher(Image, ROS_TOPIC, 10)
        self._count = 0
        # 10 FPS — 平衡实时性和 CPU 开销
        self._timer = self.create_timer(0.1, self._capture)
        self.get_logger().info(
            f"Gazebo 相机桥接已启动: {GZ_TOPIC} → {ROS_TOPIC} @10Hz"
        )

    def _capture(self):
        """抓取一帧并发布."""
        try:
            r = subprocess.run(
                ["gz", "topic", "-t", GZ_TOPIC, "-e", "--num", "1", "--json-output"],
                capture_output=True, timeout=3, env=GZ_ENV,
            )
            if r.returncode != 0 or len(r.stdout) < 100:
                return

            frame = json.loads(r.stdout)
            w = frame.get("width", 0)
            h = frame.get("height", 0)
            b64 = frame.get("data", "")
            if not w or not h or not b64:
                return

            img_bytes = base64.b64decode(b64)
            step = frame.get("step", w * 3)
            enc = "rgb8" if step == w * 3 else "bgr8"

            msg = Image()
            msg.header = Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id="camera_link",
            )
            msg.width = w
            msg.height = h
            msg.encoding = enc
            msg.is_bigendian = 0
            msg.step = step
            msg.data = img_bytes
            self._pub.publish(msg)

            self._count += 1
            if self._count == 1 or self._count % 50 == 0:
                self.get_logger().info(
                    f"已发布 {self._count} 帧 ({w}x{h} {enc} {len(img_bytes)//1024}KB)"
                )
        except subprocess.TimeoutExpired:
            pass
        except Exception as e:
            if self._count == 0:
                self.get_logger().warn(f"相机抓取异常: {e}")


def main():
    rclpy.init()
    node = GzCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
