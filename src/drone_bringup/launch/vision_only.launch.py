"""
仅视觉模块启动 — 调试用.

启动:
  - vision_pipeline (统一视觉管线)
  - camera_node (可选)

使用方法:
  ros2 launch drone_bringup vision_only.launch.py
  ros2 bag play recorded_field.bag   # 回放录制数据
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_vision = get_package_share_directory("drone_vision")
    vision_param_file = os.path.join(pkg_vision, "config", "vision_params.yaml")

    model_path_arg = DeclareLaunchArgument(
        "model_path", default_value="",
        description="YOLO ONNX 模型路径",
    )
    drop_marker_model_path_arg = DeclareLaunchArgument(
        "drop_marker_model_path", default_value="",
        description="投放区蓝色标识 YOLO 模型路径 (可选)",
    )

    vision_node = Node(
        package="drone_vision",
        executable="vision_pipeline",
        name="vision_pipeline",
        output="screen",
        emulate_tty=True,
        parameters=[
            vision_param_file,
            {
                "model_path": LaunchConfiguration("model_path"),
                "drop_marker_model_path": LaunchConfiguration("drop_marker_model_path"),
            },
        ],
        arguments=[
            "--ros-args",
        ],
    )

    return LaunchDescription([
        model_path_arg,
        drop_marker_model_path_arg,
        LogInfo(msg="=== 视觉模块独立调试模式 ==="),
        vision_node,
    ])
