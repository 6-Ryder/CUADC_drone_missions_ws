#!/usr/bin/env bash
# Read-only preflight verifier for a running real_flight.launch.py session.
# It only queries ROS graph data and bounded topic samples.  It never publishes,
# calls a service, starts an Agent, or changes vehicle/servo state.
# 不能用 nounset 选项 (-u): /opt/ros/humble/setup.bash 读取 $AMENT_TRACE_SETUP_FILES
# 无 :- 保护, 变量未赋值时 -u 直接报错退出 (与 start_full_real_mission.sh 同因).
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SAMPLE_TIMEOUT="${SAMPLE_TIMEOUT:-8s}"
DROP_SERVICE="/drone/servo/trigger_drop"

fail() {
  printf 'NOT READY: %s\n' "$*" >&2
  exit 1
}

if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
  fail "ROS 2 Humble 或已构建工作空间缺失"
fi
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

if [[ "${AMENT_PREFIX_PATH:-}" == *"/sitl_px4_msgs"* ]] || \
   [[ "${CMAKE_PREFIX_PATH:-}" == *"/sitl_px4_msgs"* ]] || \
   [[ "${PYTHONPATH:-}" == *"/sitl_px4_msgs"* ]]; then
  fail "检测到 SITL px4_msgs overlay；本检查仅适用于真机 v1 环境"
fi

command -v ros2 >/dev/null || fail "未找到 ros2"
command -v timeout >/dev/null || fail "未找到 timeout"

sample_topic() {
  local topic="$1"
  local info sample
  info="$(ros2 topic info "$topic" 2>&1)" || fail "无法查询话题 $topic"
  grep -Eq 'Publisher count: [1-9][0-9]*' <<< "$info" || \
    fail "话题没有发布者：$topic"
  sample="$(timeout "$SAMPLE_TIMEOUT" ros2 topic echo "$topic" --once \
    --qos-reliability best_effort 2>/dev/null)" || \
    fail "在 $SAMPLE_TIMEOUT 内未收到话题样本：$topic"
  printf '%s\n' "$sample"
}

for topic in \
  "/fmu/out/vehicle_status_v1" \
  "/fmu/out/vehicle_odometry" \
  "/fmu/out/battery_status_v1" \
  "/fmu/out/vehicle_land_detected"; do
  sample_topic "$topic" >/dev/null
  printf 'OK: %s\n' "$topic"
done

# VehicleCommandAck is emitted only when PX4 processes a command, so requiring
# a new sample here would make a healthy, already-initialized system fail this
# read-only check.  The servo's periodic ready=1 status below proves its own
# initial DO_SET_ACTUATOR command was accepted.
ack_info="$(ros2 topic info "/fmu/out/vehicle_command_ack" 2>&1)" || \
  fail "无法查询话题 /fmu/out/vehicle_command_ack"
grep -Eq 'Publisher count: [1-9][0-9]*' <<< "$ack_info" || \
  fail "话题没有发布者：/fmu/out/vehicle_command_ack"
printf 'OK: /fmu/out/vehicle_command_ack 发布者存在\n'

camera_info="$(sample_topic "/drone/camera/camera_info")"
grep -Fqx 'width: 640' <<< "$camera_info" || fail "CameraInfo width 不是 640"
grep -Fqx 'height: 480' <<< "$camera_info" || fail "CameraInfo height 不是 480"
camera_k="$(awk '/^k:/,/^r:/' <<< "$camera_info")"
grep -Fq -- '- 400.0' <<< "$camera_k" || fail "CameraInfo fx/fy 不是 400.0"
grep -Fq -- '- 320.0' <<< "$camera_k" || fail "CameraInfo cx 不是 320.0"
grep -Fq -- '- 240.0' <<< "$camera_k" || fail "CameraInfo cy 不是 240.0"
printf 'OK: /drone/camera/camera_info (640x480, fx=fy=400, cx=320, cy=240)\n'

vision_status="$(sample_topic "/drone/vision/status")"
grep -Fq 'ready=1' <<< "$vision_status" || fail "视觉节点尚未 ready=1"
printf 'OK: /drone/vision/status ready=1\n'

servo_status="$(sample_topic "/drone/servo/status")"
grep -Fq 'ready=1' <<< "$servo_status" || fail "舵机节点尚未 ready=1"
printf 'OK: /drone/servo/status ready=1（初始 PWM 已获 PX4 ACK）\n'

service_list="$(ros2 service list)"
grep -Fqx "$DROP_SERVICE" <<< "$service_list" || \
  fail "投放服务不存在：$DROP_SERVICE"
printf 'OK: 投放服务已存在（本检查不会调用它）\n'

printf 'READY: 真机 v1 遥测、相机、视觉与舵机 ACK 初始化检查通过。\n'
