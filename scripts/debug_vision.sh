#!/bin/bash
# ================================================================
# 视觉调试一键启动 — 摄像头 + 视觉管线 + 实时画面
#
# 用法:
#   ./scripts/debug_vision.sh                      # 默认 /dev/video0
#   ./scripts/debug_vision.sh /dev/video1          # 使用第二个摄像头
#
# 停止: 在画面窗口按 q
# ================================================================
set -e

CAM_DEVICE="${1:-/dev/video0}"

WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS_DIR"

# 确保 workspace 已构建
if [ ! -f install/setup.bash ]; then
    echo "[ERROR] 请先 colcon build"
    exit 1
fi

source install/setup.bash

# 清理旧进程
pkill -f camera_node 2>/dev/null || true
pkill -f vision_pipeline 2>/dev/null || true
fuser -k "$CAM_DEVICE" 2>/dev/null || true
sleep 1

cleanup() {
    echo ""
    echo "[CLEANUP] 正在停止..."
    pkill -f camera_node 2>/dev/null || true
    pkill -f vision_pipeline 2>/dev/null || true
    fuser -k "$CAM_DEVICE" 2>/dev/null || true
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
    -p camera_device:="$CAM_DEVICE" &
CAM_PID=$!
sleep 2

# 2. 视觉管线
echo "[2/3] 启动视觉管线..."
ros2 launch drone_bringup vision_only.launch.py &
VIS_PID=$!
sleep 6

# 3. 实时查看器
echo "[3/3] 启动查看器 (按 q 退出)..."
python3 "$WS_DIR/src/drone_vision/scripts/vision_viewer.py"
