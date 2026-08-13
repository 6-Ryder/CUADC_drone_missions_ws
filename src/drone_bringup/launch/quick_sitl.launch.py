"""
SITL 快速投放全系统启动 — 独立于原 sitl.launch.py.

与 sitl.launch.py 的区别:
  - mission 参数换 mission_params_quick_sim.yaml (quick 策略)
  - 舵机参数换 servo_params_quick.yaml (一次全开双投)
  其余节点、话题 (vehicle_status_v4)、平台约定与原 SITL launch 完全一致.

使用方法:
  ros2 launch drone_bringup quick_sitl.launch.py
  ./scripts/start_quick_simulation.sh                     # 推荐入口

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
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成 SITL 快速投放全系统启动配置."""
    home_x_args = DeclareLaunchArgument(
        "home_x",
        default_value="4.0",
        description="Home position X coordinate (meters, world frame, East offset)",
    )

    home_y_args = DeclareLaunchArgument(
        "home_y",
        default_value="10.0",
        description="Home position Y coordinate (meters, world frame, North offset)",
    )

    home_z_args = DeclareLaunchArgument(
        "home_z",
        default_value="0.0",
        description="Home position Z coordinate (meters, world frame)",
    )

    layout_heading_arg = DeclareLaunchArgument(
        "layout_heading_deg",
        default_value="0.0",
        description="场地前方绝对方位角: 0=正北, 顺时针为正 (deg)",
    )
    vehicle_status_topic_arg = DeclareLaunchArgument(
        "vehicle_status_topic",
        default_value="/fmu/out/vehicle_status_v4",
        description="SITL PX4 VehicleStatus DDS topic",
    )

    # ---- 包路径 ----
    pkg_control = get_package_share_directory("px4_control")
    pkg_vision = get_package_share_directory("drone_vision")
    pkg_servo = get_package_share_directory("drone_servo2")
    pkg_mission = get_package_share_directory("drone_mission")

    # ---- 参数文件路径 (quick: 独立参数, 不影响原 SITL) ----
    px4_param_file = os.path.join(pkg_control, "config", "px4_params.yaml")
    vision_param_file = os.path.join(pkg_vision, "config", "vision_params.yaml")
    vision_quick_param_file = os.path.join(
        pkg_vision, "config", "vision_params_quick.yaml"
    )
    servo_param_file = os.path.join(pkg_servo, "config", "servo_params_quick.yaml")
    mission_param_file = os.path.join(
        pkg_mission, "config", "mission_params_quick_sim.yaml"
    )

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
            "vehicle_status_topic": LaunchConfiguration("vehicle_status_topic"),
        }],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # 2. 统一视觉管线 (模型路径由 YAML 配置)
    vision_node = Node(
        package="drone_vision",
        executable="vision_pipeline",
        name="vision_pipeline",
        output="screen",
        parameters=[vision_param_file, vision_quick_param_file, {
            "image_top_faces_forward": False,  # gz 相机: 画面底朝机头
        }],
        arguments=[
            "--ros-args",
            "--log-level", "info",
        ],
    )

    # 3. 舵机投放控制器 (SITL Mock — quick: 一次全开双投)
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

    # 4. 任务状态机 — 最后启动 (依赖以上所有, quick 参数)
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
            "layout_heading_deg": ParameterValue(
                LaunchConfiguration("layout_heading_deg"), value_type=float,
            ),
            "vehicle_status_topic": LaunchConfiguration("vehicle_status_topic"),
            "rc_require_arm_signal": False,  # SITL 无物理遥控器, 代码强制解锁
            "image_top_faces_forward": False,  # gz 相机: 画面底朝机头
        }],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # ---- 启动顺序 (延迟确保依赖就绪) ----
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
        LogInfo(msg="=== 无人机 SITL 快速投放启动 ==="),
        LogInfo(msg="  1. PX4 Bridge (DDS → ROS2)"),
        LogInfo(msg="  2. Vision Pipeline (YOLO + OpenCV, quick marker_only)"),
        LogInfo(msg="  3. Servo Controller (SITL Mock, 一次全开双投)"),
        LogInfo(msg="  4. Mission FSM (quick 参数: 2.4m 单次降高)"),
        home_x_args,
        home_y_args,
        home_z_args,
        layout_heading_arg,
        vehicle_status_topic_arg,
        px4_bridge_node,
        vision_delayed,
        servo_delayed,
        mission_delayed,
    ])
