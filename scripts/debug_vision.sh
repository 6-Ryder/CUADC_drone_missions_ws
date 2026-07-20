#!/usr/bin/env bash
# ================================================================
# 视觉调试一键启动 — 摄像头 + 视觉管线 + 实时画面
#
# 用法:
#   ./scripts/debug_vision.sh                      # 默认外接任务相机
#   ./scripts/debug_vision.sh /dev/video0          # 临时使用内置相机
#
# 停止: 在画面窗口按 q
# ================================================================
set -Eeo pipefail

CAM_DEVICE="${1:-/dev/v4l/by-id/usb-D-VITEC_U3344W-video-index0}"

WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS_DIR"

LOG_DIR="$WS_DIR/log/vision_debug_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR/ros"
export ROS_LOG_DIR="$LOG_DIR/ros"

CAM_PID=""
VIS_PID=""

# 确保 workspace 已构建
if [ ! -f install/setup.bash ]; then
    echo "[ERROR] 请先 colcon build"
    exit 1
fi

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
source install/setup.bash
set -u

# 清理旧进程
pkill -f camera_node 2>/dev/null || true
pkill -f vision_pipeline 2>/dev/null || true
fuser -k "$CAM_DEVICE" 2>/dev/null || true
sleep 1

cleanup() {
    echo ""
    echo "[CLEANUP] 正在停止..."
    if [ -n "$VIS_PID" ]; then
        kill "$VIS_PID" 2>/dev/null || true
    fi
    if [ -n "$CAM_PID" ]; then
        kill "$CAM_PID" 2>/dev/null || true
    fi
    echo "[CLEANUP] 完成"
}
trap cleanup EXIT INT TERM

echo "============================================"
echo "  视觉调试模式"
echo "  摄像头: $CAM_DEVICE"
echo "  YOLO(圆筒+投放区标识) + 实时画面"
echo "============================================"

# 1. 摄像头
echo "[1/3] 启动摄像头 ($CAM_DEVICE)..."
ros2 run drone_vision camera_node --ros-args \
    -p sim_mode:=false \
    -p camera_device:="$CAM_DEVICE" \
    >"$LOG_DIR/camera.log" 2>&1 &
CAM_PID=$!
sleep 2

if ! kill -0 "$CAM_PID" 2>/dev/null; then
    echo "[ERROR] 相机节点启动失败: $CAM_DEVICE"
    tail -40 "$LOG_DIR/camera.log"
    exit 1
fi

echo "[1/3] 等待首帧图像..."
if ! timeout 12s ros2 topic echo --once \
    --qos-profile sensor_data /drone/camera/image_raw >/dev/null 2>&1; then
    echo "[ERROR] 12 秒内未收到 /drone/camera/image_raw"
    tail -40 "$LOG_DIR/camera.log"
    exit 1
fi

# 2. 视觉管线
echo "[2/3] 启动视觉管线..."
ros2 launch drone_bringup vision_only.launch.py \
    >"$LOG_DIR/vision_pipeline.log" 2>&1 &
VIS_PID=$!
sleep 4

if ! kill -0 "$VIS_PID" 2>/dev/null; then
    echo "[ERROR] 视觉管线启动失败"
    tail -60 "$LOG_DIR/vision_pipeline.log"
    exit 1
fi

echo "[2/3] 等待 CUDA 模型和视觉首帧就绪..."
if ! timeout 20s bash -c '
    until ros2 topic echo --once /drone/vision/status 2>/dev/null | grep -q "ready=1"; do
        sleep 0.5
    done
'; then
    echo "[ERROR] 视觉模型或首帧未就绪"
    tail -80 "$LOG_DIR/vision_pipeline.log"
    exit 1
fi

# 3. 实时查看器
echo "[3/3] 启动查看器 (按 q 退出)..."
if [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    echo "[ERROR] 未检测到图形会话 (DISPLAY/WAYLAND_DISPLAY)，无法打开查看器"
    exit 1
fi

echo "[INFO] 日志目录: $LOG_DIR"
ros2 run drone_vision vision_viewer
