"""
实飞启动 — Jetson Orin Nano 实飞部署.

与 SITL 的区别:
  - platform:=jetson (舵机通过 PX4 VehicleCommand 控制 MAIN5)
  - 启动独立 V4L2 相机节点
  - PX4 通过以太网 UDP 连接 Jetson 上的 MicroXRCEAgent

使用方法:
  ros2 launch drone_bringup real_flight.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_control = get_package_share_directory("px4_control")
    pkg_vision = get_package_share_directory("drone_vision")
    pkg_servo = get_package_share_directory("drone_servo2")
    pkg_mission = get_package_share_directory("drone_mission")

    px4_param_file = os.path.join(pkg_control, "config", "px4_params.yaml")
    vision_param_file = os.path.join(pkg_vision, "config", "vision_params.yaml")
    servo_param_file = os.path.join(pkg_servo, "config", "servo_params.yaml")
    mission_param_file = os.path.join(pkg_mission, "config", "mission_params.yaml")

    camera_device_arg = DeclareLaunchArgument(
        "camera_device",
        default_value="/dev/v4l/by-id/usb-D-VITEC_U3344W-video-index0",
        description="任务相机的稳定 V4L2 设备路径",
    )
    dds_port_arg = DeclareLaunchArgument(
        "dds_port", default_value="8888",
        description="Jetson 上 MicroXRCEAgent 的 UDP 监听端口",
    )

    home_x_arg = DeclareLaunchArgument(
        "home_x", default_value="0.0",
        description="Home 点 World x (真机解锁点即原点)",
    )
    home_y_arg = DeclareLaunchArgument(
        "home_y", default_value="0.0",
        description="Home 点 World y",
    )
    home_z_arg = DeclareLaunchArgument(
        "home_z", default_value="0.0",
        description="Home 点 World z",
    )

    flight_axis_arg = DeclareLaunchArgument(
        "flight_axis", default_value="north",
        description="主飞行轴朝向: 'east' | 'north'",
    )
    drop_distance_arg = DeclareLaunchArgument(
        "drop_distance", default_value="32.5",
        description="Home → 投放区距离 (m)",
    )
    recon_distance_arg = DeclareLaunchArgument(
        "recon_distance", default_value="57.5",
        description="Home → 侦察区距离 (m)",
    )
    rc_require_arg = DeclareLaunchArgument(
        "rc_require_arm_signal", default_value="true",
        description="true=等待人用遥控器Arm Switch解锁, false=代码arm直接起飞",
    )

    xrce_agent = ExecuteProcess(
        cmd=[
            "MicroXRCEAgent", "udp4", "-p",
            LaunchConfiguration("dds_port"),
        ],
        output="screen",
    )

    # 同一 UDP 端口只能由一个 Agent 监听。若端口被其他 Agent 占用，
    # MicroXRCEAgent 会立即退出；此时继续运行任务节点将导致“ROS 在跑、
    # 但 PX4 DDS 已断开”的危险假象，因此直接结束整套 launch。
    agent_exit_guard = RegisterEventHandler(
        OnProcessExit(
            target_action=xrce_agent,
            on_exit=[
                LogInfo(
                    msg=(
                        "MicroXRCEAgent 已退出，停止真机任务，"
                        "避免在 PX4 DDS 未连接时继续执行。"
                    )
                ),
                EmitEvent(event=Shutdown(reason="MicroXRCEAgent exited")),
            ],
        )
    )

    # PX4 桥接
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
    )

    camera_node = Node(
        package="drone_vision",
        executable="camera_node",
        name="camera_driver",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "sim_mode": False,
            "camera_device": LaunchConfiguration("camera_device"),
            "camera_width": 640,
            "camera_height": 480,
            "camera_fps": 30.0,
            "fx": 400.0,
            "fy": 400.0,
            "cx": 320.0,
            "cy": 240.0,
        }],
    )

    # 视觉 — 实飞模式 (ONNX Runtime CUDA)
    # 真机 Home 为解锁点(0,0), 侦察区 = NED north[55,60] east[-4,4] → World
    vision_node = Node(
        package="drone_vision",
        executable="vision_pipeline",
        name="vision_pipeline",
        output="screen",
        parameters=[
            vision_param_file,
            {
                "camera_width": 640,
                "camera_height": 480,
                "fx": 400.0,
                "fy": 400.0,
                "cx": 320.0,
                "cy": 240.0,
                "device": "cuda",
                "coverage_zone_x_min": -4.0,
                "coverage_zone_x_max": 4.0,
                "coverage_zone_y_min": 55.0,
                "coverage_zone_y_max": 60.0,
            },
        ],
    )

    # 舵机 — CUAV 7-Nano MAIN5, PX4 PWM 50Hz
    servo_node = Node(
        package="drone_servo2",
        executable="servo_controller",
        name="servo_controller",
        output="screen",
        parameters=[servo_param_file, {"platform": "jetson"}],
    )

    # 任务状态机
    mission_node = Node(
        package="drone_mission",
        executable="mission_fsm",
        name="mission_state_machine",
        output="screen",
        emulate_tty=True,
        parameters=[
            mission_param_file,
            {
                "sim_mode": False,
                "precision_align_image_width": 640.0,
                "precision_align_image_height": 480.0,
                "camera_fx": 400.0,
                "camera_fy": 400.0,
                "camera_cx": 320.0,
                "camera_cy": 240.0,
                "home_x": LaunchConfiguration("home_x"),
                "home_y": LaunchConfiguration("home_y"),
                "home_z": LaunchConfiguration("home_z"),
                "flight_axis": LaunchConfiguration("flight_axis"),
                "drop_distance": LaunchConfiguration("drop_distance"),
                "recon_distance": LaunchConfiguration("recon_distance"),
                "rc_require_arm_signal": LaunchConfiguration("rc_require_arm_signal"),
            },
        ],
        additional_env={"RCUTILS_COLORIZED_OUTPUT": "1"},
    )

    # 顺序启动
    vision_delayed = TimerAction(period=1.0, actions=[vision_node])
    servo_delayed = TimerAction(period=1.0, actions=[servo_node])
    mission_delayed = TimerAction(period=3.0, actions=[mission_node])

    return LaunchDescription([
        LogInfo(msg="=== 无人机实飞全系统启动 (Jetson) ==="),
        camera_device_arg,
        dds_port_arg,
        home_x_arg,
        home_y_arg,
        home_z_arg,
        flight_axis_arg,
        drop_distance_arg,
        recon_distance_arg,
        rc_require_arg,
        xrce_agent,
        agent_exit_guard,
        camera_node,
        px4_bridge_node,
        vision_delayed,
        servo_delayed,
        mission_delayed,
    ])
