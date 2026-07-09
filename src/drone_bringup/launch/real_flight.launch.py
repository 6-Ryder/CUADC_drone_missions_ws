"""
实飞启动 — Jetson Orin Nano 实飞部署.

与 SITL 的区别:
  - platform:=jetson (舵机用真实 GPIO)
  - sim_mode:=false (相机用 GStreamer)
  - 启动独立 GStreamer 节点

使用方法:
  ros2 launch drone_bringup real_flight.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_control = get_package_share_directory("px4_control")
    pkg_vision = get_package_share_directory("drone_vision")
    pkg_servo = get_package_share_directory("drone_servo")
    pkg_mission = get_package_share_directory("drone_mission")

    px4_param_file = os.path.join(pkg_control, "config", "px4_params.yaml")
    vision_param_file = os.path.join(pkg_vision, "config", "vision_params.yaml")
    servo_param_file = os.path.join(pkg_servo, "config", "servo_params.yaml")
    mission_param_file = os.path.join(pkg_mission, "config", "mission_params.yaml")

    model_path_arg = DeclareLaunchArgument(
        "model_path", default_value="",
        description="YOLO ONNX 模型路径",
    )
    drop_marker_model_path_arg = DeclareLaunchArgument(
        "drop_marker_model_path", default_value="",
        description="投放区蓝色标识 YOLO 模型路径 (可选)",
    )

    # PX4 桥接
    px4_bridge_node = Node(
        package="px4_control",
        executable="px4_bridge",
        name="px4_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[px4_param_file],
    )

    # 视觉 — 使用实飞配置
    vision_node = Node(
        package="drone_vision",
        executable="vision_pipeline",
        name="vision_pipeline",
        output="screen",
        parameters=[
            vision_param_file,
            {
                "model_path": LaunchConfiguration("model_path"),
                "drop_marker_model_path": LaunchConfiguration("drop_marker_model_path"),
                "sim_mode": False,
                "device": "tensorrt",
            },
        ],
        arguments=[
            "--ros-args",
        ],
    )

    # 舵机 — 真实 GPIO
    servo_node = Node(
        package="drone_servo",
        executable="servo_controller",
        name="servo_controller",
        output="screen",
        parameters=[servo_param_file],
        arguments=[
            "--ros-args",
            "-p", "platform:=jetson",
        ],
    )

    # 任务状态机
    mission_node = Node(
        package="drone_mission",
        executable="mission_fsm",
        name="mission_state_machine",
        output="screen",
        emulate_tty=True,
        parameters=[mission_param_file],
    )

    # 顺序启动
    vision_delayed = TimerAction(period=1.0, actions=[vision_node])
    servo_delayed = TimerAction(period=1.0, actions=[servo_node])
    mission_delayed = TimerAction(period=3.0, actions=[mission_node])

    return LaunchDescription([
        model_path_arg,
        drop_marker_model_path_arg,
        LogInfo(msg="=== 无人机实飞全系统启动 (Jetson) ==="),
        px4_bridge_node,
        vision_delayed,
        servo_delayed,
        mission_delayed,
    ])
