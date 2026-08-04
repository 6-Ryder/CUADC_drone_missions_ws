#!/usr/bin/env bash
# Single safe entry point for the complete real-hardware mission.
# This script does not arm, change PX4 mode, publish setpoints, call services,
# control the servo, kill processes, or start a second DDS Agent.  The launch
# file owns the Agent and all flight nodes.
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WORKSPACE_DIR/log/real_mission_$RUN_ID"
DEFAULT_CAMERA_DEVICE="/dev/v4l/by-id/usb-D-VITEC_U3344W-video-index0"
CAMERA_DEVICE="${CAMERA_DEVICE:-$DEFAULT_CAMERA_DEVICE}"

for launch_arg in "$@"; do
  case "$launch_arg" in
    camera_device:=*) CAMERA_DEVICE="${launch_arg#camera_device:=}" ;;
  esac
done

fail() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

mkdir -p "$LOG_DIR/ros"
export ROS_LOG_DIR="$LOG_DIR/ros"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  fail "ROS 2 Humble 未安装：缺少 /opt/ros/humble/setup.bash"
fi
source /opt/ros/humble/setup.bash

if [[ ! -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
  fail "工作空间未构建：请先在 $WORKSPACE_DIR 运行 colcon build"
fi
source "$WORKSPACE_DIR/install/setup.bash"

if [[ "${AMENT_PREFIX_PATH:-}" == *"/sitl_px4_msgs"* ]] || \
   [[ "${CMAKE_PREFIX_PATH:-}" == *"/sitl_px4_msgs"* ]] || \
   [[ "${PYTHONPATH:-}" == *"/sitl_px4_msgs"* ]]; then
  fail "检测到 SITL px4_msgs overlay；请使用干净终端启动真机任务"
fi

command -v ros2 >/dev/null || fail "未找到 ros2"
command -v MicroXRCEAgent >/dev/null || fail "未找到 MicroXRCEAgent"
ros2 pkg prefix drone_bringup >/dev/null || fail "drone_bringup 未安装到工作空间"

if [[ ! -f "$WORKSPACE_DIR/install/drone_bringup/share/drone_bringup/launch/real_flight.launch.py" ]]; then
  fail "未找到已安装的 real_flight.launch.py；请重新构建 drone_bringup"
fi

if [[ ! -e "$CAMERA_DEVICE" ]]; then
  fail "相机设备不存在：$CAMERA_DEVICE（可通过 camera_device:=... 指定）"
fi
if [[ ! -r "$CAMERA_DEVICE" ]]; then
  fail "当前用户无权读取相机：$CAMERA_DEVICE"
fi

printf '%q ' "$@" > "$LOG_DIR/launch_arguments.txt"
printf '\n' >> "$LOG_DIR/launch_arguments.txt"

cat <<EOF
=== 全流程真机任务 ===
日志目录: $LOG_DIR
PX4 接口: /fmu/out/vehicle_status_v1（真机 v1）
相机设备: $CAMERA_DEVICE
Home: 解锁点；投放区 32.5m、侦察区 57.5m（可通过 launch 参数修改）
操作要求: 保持桨叶与场地安全；由操作者在遥控器上 Arm；随时可切出 Offboard 接管。
脚本不会自动 Arm、切模式、投放、降落或返航；MicroXRCEAgent 仅由 launch 管理。
EOF

ros2 launch drone_bringup real_flight.launch.py "$@" 2>&1 | tee "$LOG_DIR/mission.log"
