"""
仅视觉模块启动 — 调试用.

模型路径由 drone_vision/config/vision_params.yaml 配置.
命令行可覆盖 camera 参数.

使用方法:
  ros2 launch drone_bringup vision_only.launch.py                 # 仅视觉管线 (需单独启动相机)
  ros2 launch drone_bringup vision_only.launch.py start_camera:=true  # + 摄像头
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_vision = get_package_share_directory("drone_vision")
    vision_param_file = os.path.join(pkg_vision, "config", "vision_params.yaml")

    start_camera_arg = DeclareLaunchArgument(
        "start_camera", default_value="false",
        description="是否同时启动摄像头 (true/false)",
    )

    # 调试用摄像头为 640x480; 覆盖 YAML 中 SITL 的 1280x960 内参
    vision_node = Node(
        package="drone_vision",
        executable="vision_pipeline",
        name="vision_pipeline",
        output="screen",
        emulate_tty=True,
        parameters=[
            vision_param_file,
            {
                "camera_width": 640,
                "camera_height": 480,
                "cx": 320.0,
                "cy": 240.0,
            },
        ],
    )

    camera_node = Node(
        package="drone_vision",
        executable="camera_node",
        name="camera_driver",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_camera")),
        parameters=[{
            "sim_mode": False,
            "camera_width": 640,
            "camera_height": 480,
            "camera_fps": 30.0,
        }],
    )

    return LaunchDescription([
        start_camera_arg,
        LogInfo(msg="=== 视觉模块独立调试模式 ==="),
        vision_node,
        camera_node,
    ])
