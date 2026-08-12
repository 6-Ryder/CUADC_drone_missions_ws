"""
实飞快速投放启动 — Jetson Orin Nano (独立于原 real_flight.launch.py).

与 real_flight.launch.py 的区别:
  - mission 参数换 mission_params_quick.yaml (quick 策略)
  - 舵机参数换 servo_params_quick.yaml (一次全开双投)
  其余节点、话题、平台约定与原实飞 launch 完全一致.

MicroXRCEAgent 由启动脚本 scripts/start_quick_mission_real.sh 持有:
脚本先清理旧进程、启动 Agent 并等待 DDS 就绪, 再调用本 launch;
Agent 意外退出时由脚本检测并停止本 launch (等价于旧的进程内 exit guard)。

使用方法:
  ros2 launch drone_bringup quick_flight.launch.py       # 需 Agent 已在运行
  ./scripts/start_quick_mission_real.sh                  # 推荐入口 (持有 Agent)
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
    pkg_servo = get_package_share_directory("drone_servo2")
    pkg_mission = get_package_share_directory("drone_mission")

    px4_param_file = os.path.join(pkg_control, "config", "px4_params.yaml")
    vision_param_file = os.path.join(pkg_vision, "config", "vision_params.yaml")
    servo_param_file = os.path.join(pkg_servo, "config", "servo_params_quick.yaml")
    mission_param_file = os.path.join(
        pkg_mission, "config", "mission_params_quick.yaml"
    )

    camera_device_arg = DeclareLaunchArgument(
        "camera_device",
        default_value="/dev/v4l/by-id/usb-D-VITEC_U3344W-video-index0",
        description="任务相机的稳定 V4L2 设备路径",
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

    layout_heading_arg = DeclareLaunchArgument(
        "layout_heading_deg",
        default_value="0.0",
        description="场地前方绝对方位角: 0=正北, 顺时针为正 (deg)",
    )
    rc_require_arg = DeclareLaunchArgument(
        "rc_require_arm_signal", default_value="true",
        description="true=等待人用遥控器Arm Switch解锁, false=代码arm直接起飞",
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
                "home_x": LaunchConfiguration("home_x"),
                "home_y": LaunchConfiguration("home_y"),
                "layout_heading_deg": LaunchConfiguration("layout_heading_deg"),
                "image_top_faces_forward": True,  # 真机: 画面顶朝机头
            },
        ],
    )

    # 舵机 — CUAV 7-Nano MAIN5, PX4 PWM 50Hz (quick 参数: 一次全开双投)
    servo_node = Node(
        package="drone_servo2",
        executable="servo_controller",
        name="servo_controller",
        output="screen",
        parameters=[servo_param_file, {"platform": "jetson"}],
    )

    # 任务状态机 (quick 参数: 单次降高 2.4m + marker_only + 放宽对心 + 单次双投)
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
                "home_x": LaunchConfiguration("home_x"),
                "home_y": LaunchConfiguration("home_y"),
                "home_z": LaunchConfiguration("home_z"),
                "layout_heading_deg": LaunchConfiguration("layout_heading_deg"),
                "rc_require_arm_signal": LaunchConfiguration("rc_require_arm_signal"),
                "image_top_faces_forward": True,  # 真机: 画面顶朝机头
            },
        ],
        additional_env={"RCUTILS_COLORIZED_OUTPUT": "1"},
    )

    # 顺序启动
    vision_delayed = TimerAction(period=1.0, actions=[vision_node])
    servo_delayed = TimerAction(period=1.0, actions=[servo_node])
    mission_delayed = TimerAction(period=3.0, actions=[mission_node])

    return LaunchDescription([
        LogInfo(msg="=== 无人机实飞快速投放启动 (Jetson) ==="),
        camera_device_arg,
        home_x_arg,
        home_y_arg,
        home_z_arg,
        layout_heading_arg,
        rc_require_arg,
        camera_node,
        px4_bridge_node,
        vision_delayed,
        servo_delayed,
        mission_delayed,
    ])
