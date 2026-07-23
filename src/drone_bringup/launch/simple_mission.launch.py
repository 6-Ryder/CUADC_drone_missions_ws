"""
简化任务启动 — 仅启动 px4_bridge + simple_mission.

任务流程:
  INIT → TAKEOFF → TRANSIT_TO_DROP → LOITER → RETURN → LAND → COMPLETE

使用方法:
  # 仿真 (默认参数, Gazebo drone_field)
  ros2 launch drone_bringup simple_mission.launch.py

  # 真机 (以解锁点为原点, 投放区在正前方 20m)
  ros2 launch drone_bringup simple_mission.launch.py \
      sim_mode:=false home_x:=0.0 home_y:=0.0 drop_x:=2.0 drop_y:=20.0

依赖:
  - MicroXRCEAgent 已在运行 (udp4 -p 8888)
  - PX4 (SITL 或真机) 已上电/启动
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_control = get_package_share_directory("px4_control")
    pkg_mission = get_package_share_directory("drone_mission")

    px4_param_file = os.path.join(pkg_control, "config", "px4_params.yaml")

    # ---- 启动参数 ----
    cruise_altitude_arg = DeclareLaunchArgument(
        "cruise_altitude", default_value="5.0",
        description="巡航飞行高度 (m)",
    )
    drop_x_arg = DeclareLaunchArgument(
        "drop_x", default_value="10.0",
        description="投放区 World X 坐标 (m)",
    )
    drop_y_arg = DeclareLaunchArgument(
        "drop_y", default_value="36.5",
        description="投放区 World Y 坐标 (m)",
    )
    home_x_arg = DeclareLaunchArgument(
        "home_x", default_value="10.0",
        description="起降点 World X 坐标 (m)",
    )
    home_y_arg = DeclareLaunchArgument(
        "home_y", default_value="4.0",
        description="起降点 World Y 坐标 (m)",
    )
    loiter_time_arg = DeclareLaunchArgument(
        "loiter_time", default_value="5.0",
        description="投放区悬停时间 (s)",
    )
    sim_mode_arg = DeclareLaunchArgument(
        "sim_mode", default_value="true",
        description="true=SITL force arm(21196) / false=真机标准解锁(0)",
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
            "home_z": 0.0,
        }],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # 2. 简化任务节点
    simple_mission_node = Node(
        package="drone_mission",
        executable="simple_mission",
        name="simple_mission",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "cruise_altitude": LaunchConfiguration("cruise_altitude"),
            "takeoff_height": LaunchConfiguration("cruise_altitude"),
            "drop_x": LaunchConfiguration("drop_x"),
            "drop_y": LaunchConfiguration("drop_y"),
            "home_x": LaunchConfiguration("home_x"),
            "home_y": LaunchConfiguration("home_y"),
            "loiter_time": LaunchConfiguration("loiter_time"),
            "sim_mode": LaunchConfiguration("sim_mode"),
        }],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # 延迟 3s 启动任务 (等 px4_bridge DDS 连接就绪)
    mission_delayed = TimerAction(
        period=3.0,
        actions=[simple_mission_node],
    )

    return LaunchDescription([
        LogInfo(msg="=== 无人机简化任务启动 ==="),
        LogInfo(msg="  1. PX4 Bridge (DDS ↔ ROS2)"),
        LogInfo(msg="  2. SimpleMission (起飞 → 投放区 → 返航 → 降落)"),
        LogInfo(msg=""),
        LogInfo(msg="  任务流程:"),
        LogInfo(msg="    INIT → TAKEOFF → TRANSIT_TO_DROP"),
        LogInfo(msg="    → LOITER → RETURN → LAND → COMPLETE"),
        cruise_altitude_arg,
        drop_x_arg,
        drop_y_arg,
        home_x_arg,
        home_y_arg,
        loiter_time_arg,
        sim_mode_arg,
        px4_bridge_node,
        mission_delayed,
    ])
