"""
test_forward_recon 启动 — px4_bridge + test_forward_recon.

任务流程:
  Phase 1: TAKEOFF → TRANSIT_FORWARD (向北直飞 20m)
  Phase 2: DESCEND → RECON_SWEEP → ASCEND → RETURN → LAND

使用方法:
  # 仿真 (SITL)
  ros2 launch test_mission test_forward_recon.launch.py

  # 真机 (Home=解锁点原点)
  ros2 launch test_mission test_forward_recon.launch.py sim_mode:=false
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

    px4_param_file = os.path.join(pkg_control, "config", "px4_params.yaml")
    vision_param_file = os.path.join(pkg_vision, "config", "vision_params.yaml")

    # ---- 启动参数 ----
    sim_mode_arg = DeclareLaunchArgument(
        "sim_mode", default_value="true",
        description="true=SITL / false=真机",
    )
    cruise_altitude_arg = DeclareLaunchArgument(
        "cruise_altitude", default_value="5.0",
        description="巡航飞行高度 (m)",
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
    forward_distance_arg = DeclareLaunchArgument(
        "forward_distance", default_value="20.0",
        description="Phase 1: 向北直飞距离 (m)",
    )
    transit_speed_arg = DeclareLaunchArgument(
        "transit_speed", default_value="2.0",
        description="过渡飞行速度 (m/s)",
    )
    recon_altitude_arg = DeclareLaunchArgument(
        "recon_altitude", default_value="3.0",
        description="Phase 2: 侦察扫描高度 (m)",
    )
    recon_half_north_arg = DeclareLaunchArgument(
        "recon_area_half_north", default_value="4.0",
        description="侦察区 north 半宽 (m)",
    )
    recon_half_east_arg = DeclareLaunchArgument(
        "recon_area_half_east", default_value="3.0",
        description="侦察区 east 半宽 (m)",
    )
    sweep_speed_arg = DeclareLaunchArgument(
        "sweep_speed", default_value="0.8",
        description="扫描飞行速度 (m/s)",
    )
    grid_spacing_arg = DeclareLaunchArgument(
        "grid_spacing", default_value="1.0",
        description="弓字形条带间距 (m)",
    )
    rc_require_arg = DeclareLaunchArgument(
        "rc_require_arm_signal", default_value="true",
        description="true=等待人用遥控器Arm Switch解锁, false=代码arm直接起飞",
    )
    vision_params_arg = DeclareLaunchArgument(
        "vision_params", default_value=vision_param_file,
        description="视觉管线参数文件路径",
    )
    camera_device_arg = DeclareLaunchArgument(
        "camera_device",
        default_value="/dev/v4l/by-id/usb-D-VITEC_U3344W-video-index0",
        description="V4L2 相机设备路径 (仅 sim_mode:=false 时使用)",
    )

    # ---- 节点 ----

    # 1. PX4 通信桥接
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

    # 2. 相机驱动 (V4L2 USB 相机 → /drone/camera/image_raw)
    camera_node = Node(
        package="drone_vision",
        executable="camera_node",
        name="camera_driver",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "sim_mode": LaunchConfiguration("sim_mode"),
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

    # 3. 视觉管线 (H 标识精准降落, ONNX Runtime CUDA)
    vision_pipeline_node = Node(
        package="drone_vision",
        executable="vision_pipeline",
        name="vision_pipeline",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("vision_params"),
            {
                "camera_width": 640,
                "camera_height": 480,
                "camera_fx": 400.0,
                "camera_fy": 400.0,
                "camera_cx": 320.0,
                "camera_cy": 240.0,
            },
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # 4. 分阶段测试节点
    test_node = Node(
        package="test_mission",
        executable="test_forward_recon",
        name="test_forward_recon",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "sim_mode": LaunchConfiguration("sim_mode"),
            "cruise_altitude": LaunchConfiguration("cruise_altitude"),
            "takeoff_height": LaunchConfiguration("cruise_altitude"),
            "home_x": LaunchConfiguration("home_x"),
            "home_y": LaunchConfiguration("home_y"),
            "home_z": LaunchConfiguration("home_z"),
            "forward_distance": LaunchConfiguration("forward_distance"),
            "transit_speed": LaunchConfiguration("transit_speed"),
            "recon_altitude": LaunchConfiguration("recon_altitude"),
            "recon_area_half_north": LaunchConfiguration("recon_area_half_north"),
            "recon_area_half_east": LaunchConfiguration("recon_area_half_east"),
            "sweep_speed": LaunchConfiguration("sweep_speed"),
            "grid_spacing": LaunchConfiguration("grid_spacing"),
            "rc_require_arm_signal": LaunchConfiguration("rc_require_arm_signal"),
            "camera_cx": 320.0,
            "camera_cy": 240.0,
        }],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # 延迟 3s 启动测试节点 (等 px4_bridge DDS 连接就绪)
    test_delayed = TimerAction(
        period=3.0,
        actions=[test_node],
    )

    return LaunchDescription([
        LogInfo(msg="=== 分阶段真机测试: 前飞 20m + 侦察区 ==="),
        LogInfo(msg="  安全: WAIT_ARMED → 遥控器 Arm Switch 解锁 → TAKEOFF"),
        LogInfo(msg="  Phase 1: TAKEOFF → TRANSIT_FORWARD (向北 20m)"),
        LogInfo(msg="  Phase 2: DESCEND → RECON_SWEEP → ASCEND → RETURN"),
        LogInfo(msg="  LAND: H 标识视觉伺服精准降落"),
        sim_mode_arg,
        cruise_altitude_arg,
        home_x_arg,
        home_y_arg,
        home_z_arg,
        forward_distance_arg,
        transit_speed_arg,
        recon_altitude_arg,
        recon_half_north_arg,
        recon_half_east_arg,
        sweep_speed_arg,
        grid_spacing_arg,
        rc_require_arg,
        vision_params_arg,
        camera_device_arg,
        px4_bridge_node,
        camera_node,
        vision_pipeline_node,
        test_delayed,
    ])
