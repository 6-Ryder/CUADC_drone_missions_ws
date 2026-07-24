"""
SITL 全系统启动 — 一键启动仿真环境中所有节点.

启动节点:
  1. px4_bridge       — PX4 DDS 通信桥接 (订阅/发布 /fmu/* 话题)
  2. vision_pipeline  — 统一视觉管线 (YOLO + OpenCV + 伺服)
  3. servo_controller — 舵机投放控制 (SITL Mock)
  4. mission_fsm      — 任务状态机 (FSM 编排)

使用方法:
  ros2 launch drone_bringup sitl.launch.py
  ros2 launch drone_bringup sitl.launch.py model_path:=/path/to/model.onnx

依赖:
  - MicroXRCEAgent 已在运行 (udp4 -p 8888)
  - PX4 SITL + Gazebo 已在运行
  - ros_gz_image bridge 已在运行
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """生成 SITL 全系统启动配置."""
    
    home_x_args = DeclareLaunchArgument(
        "home_x",
        default_value="10.0",
        description="Home position X coordinate (meters, world frame)",
    )

    home_y_args = DeclareLaunchArgument(
        "home_y",
        default_value="4.0",
        description="Home position Y coordinate (meters, world frame)",
    )

    home_z_args = DeclareLaunchArgument(
        "home_z",
        default_value="0.0",
        description="Home position Z coordinate (meters, world frame)",
    )

    flight_axis_arg = DeclareLaunchArgument(
        "flight_axis",
        default_value="north",
        description="主飞行轴朝向: 'east' | 'north'",
    )
    drop_distance_arg = DeclareLaunchArgument(
        "drop_distance",
        default_value="32.5",
        description="Home → 投放区距离 (m)",
    )
    recon_distance_arg = DeclareLaunchArgument(
        "recon_distance",
        default_value="57.5",
        description="Home → 侦察区距离 (m)",
    )

    # ---- 包路径 ----
    pkg_control = get_package_share_directory("px4_control")
    pkg_vision = get_package_share_directory("drone_vision")
    pkg_servo = get_package_share_directory("drone_servo2")
    pkg_mission = get_package_share_directory("drone_mission")

    # ---- 参数文件路径 ----
    px4_param_file = os.path.join(pkg_control, "config", "px4_params.yaml")
    vision_param_file = os.path.join(pkg_vision, "config", "vision_params.yaml")
    servo_param_file = os.path.join(pkg_servo, "config", "servo_params.yaml")
    mission_param_file = os.path.join(pkg_mission, "config", "mission_params.yaml")

    # ---- 启动参数 (模型路径由 vision_params.yaml 配置) ----

    # ---- 节点 ----

    # 1. PX4 通信桥接 — 必须先启动
    px4_bridge_node = Node(
        package="px4_control",
        executable="px4_bridge",
        name="px4_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[px4_param_file, {
            "home_x": LaunchConfiguration("home_x"),
            "home_y": LaunchConfiguration("home_y"),
            "home_z": LaunchConfiguration("home_z"),
        }],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # 2. 统一视觉管线 (模型路径由 YAML 配置)
    vision_node = Node(
        package="drone_vision",
        executable="vision_pipeline",
        name="vision_pipeline",
        output="screen",
        parameters=[vision_param_file],
        arguments=[
            "--ros-args",
            "--log-level", "info",
        ],
    )

    # 3. 舵机投放控制器 (SITL Mock — 多位置步进投放)
    servo_node = Node(
        package="drone_servo2",
        executable="servo_controller",
        name="servo_controller",
        output="screen",
        parameters=[servo_param_file],
        arguments=[
            "--ros-args",
            "-p", "platform:=sitl",
            "--log-level", "info",
        ],
    )

    # 4. 任务状态机 — 最后启动 (依赖以上所有, 内含低空投放视觉校准)
    mission_node = Node(
        package="drone_mission",
        executable="mission_fsm",
        name="mission_state_machine",
        output="screen",
        emulate_tty=True,
        parameters=[mission_param_file, {
            "home_x": LaunchConfiguration("home_x"),
            "home_y": LaunchConfiguration("home_y"),
            "home_z": LaunchConfiguration("home_z"),
            "flight_axis": LaunchConfiguration("flight_axis"),
            "drop_distance": LaunchConfiguration("drop_distance"),
            "recon_distance": LaunchConfiguration("recon_distance"),
        }],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # ---- 启动顺序 (延迟确保依赖就绪) ----
    # PX4 bridge 立即启动
    # Vision + Servo 延迟 1s
    # Mission FSM 延迟 3s
    vision_delayed = TimerAction(
        period=1.0,
        actions=[vision_node],
    )
    servo_delayed = TimerAction(
        period=1.0,
        actions=[servo_node],
    )
    mission_delayed = TimerAction(
        period=3.0,
        actions=[mission_node],
    )

    return LaunchDescription([
        LogInfo(msg="=== 无人机 SITL 全系统启动 ==="),
        LogInfo(msg="  1. PX4 Bridge (DDS → ROS2)"),
        LogInfo(msg="  2. Vision Pipeline (YOLO + OpenCV)"),
        LogInfo(msg="  3. Servo Controller (SITL Mock)"),
        LogInfo(msg="  4. Mission FSM (任务状态机 + 低空投放视觉校准)"),
        home_x_args,
        home_y_args,
        home_z_args,
        flight_axis_arg,
        drop_distance_arg,
        recon_distance_arg,
        px4_bridge_node,
        vision_delayed,
        servo_delayed,
        mission_delayed,
    ])
