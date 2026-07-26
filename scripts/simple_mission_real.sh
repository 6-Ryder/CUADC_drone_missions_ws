#!/usr/bin/env bash
# ==============================================================================
# simple_mission 真机启动 — Jetson Orin Nano + CUAV 7 Nano
#
# 与仿真 (simple_mission.sh) 的区别:
#   - sim_mode:=false (标准解锁, 非 SITL force arm)
#   - home_x/home_y:=0.0 (以解锁点为原点)
#   - 无需 Gazebo / PX4 SITL / ros_gz_bridge
#   - 仅需 MicroXRCEAgent + px4_bridge + simple_mission
#
# 硬件连接:
#   - CUAV 7 Nano 通过网线直连 Jetson Orin Nano (UDP 模式)
#   - 或通过串口 TELEM2 -> Jetson UART (Serial 模式)
#
# 用法:
#   # 以太网连接 (默认)
#   ./scripts/simple_mission_real.sh
#
#   # 串口连接
#   DDS_TRANSPORT=serial SERIAL_DEV=/dev/ttyTHS0 ./scripts/simple_mission_real.sh
#
#   # 自定义航点
#   ./scripts/simple_mission_real.sh drop_x:=25.0 drop_y:=3.0 cruise_altitude:=10.0
# ==============================================================================
set -eo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $(date '+%H:%M:%S') $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $(date '+%H:%M:%S') $*"; }
error() { echo -e "${RED}[ERROR]${NC} $(date '+%H:%M:%S') $*"; }
step()  { echo -e "\n${BLUE}==== $(date '+%H:%M:%S') $* ====${NC}"; }

PROJ="$(cd "$(dirname "$0")/.." && pwd)"
LOG_DIR="$PROJ/log/real_flight_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

# ---- 环境 ----
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    error "ROS2 Humble 未找到"
    exit 1
fi

if [ ! -f "$PROJ/install/setup.bash" ]; then
    error "CUADC_drone_missions_ws 未编译, 请先运行: colcon build --symlink-install"
    exit 1
fi
source "$PROJ/install/setup.bash"
export ROS_DOMAIN_ID=0

# ---- DDS 通信方式 ----
# CUAV 7 Nano 两种模式:
#   1. UDP (PX4 参数: UXRCE_DDS_CFG=2, UXRCE_DDS_AG_IP=<Jetson IP>) — 网线直连
#   2. Serial (PX4 参数: UXRCE_DDS_CFG=1, SER_TEL2_BAUD=921600) — TELEM2 -> Jetson UART
DDS_TRANSPORT="${DDS_TRANSPORT:-udp}"
DDS_PORT="${DDS_PORT:-8888}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyTHS0}"
SERIAL_BAUD="${SERIAL_BAUD:-921600}"

# ---- Step 0: 清理旧进程 ----
step "Step 0/3 — 清理旧进程"
pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
pkill -9 -f "px4_bridge" 2>/dev/null || true
#pkill -9 -f "simple_mission" 2>/dev/null || true
sleep 1
info "旧进程已清理"

# ---- Step 1: MicroXRCEAgent ----
if [ "$DDS_TRANSPORT" = "udp" ]; then
    step "Step 1/3 — 启动 MicroXRCEAgent (UDP :$DDS_PORT)"
    MicroXRCEAgent udp4 -p "$DDS_PORT" > "$LOG_DIR/agent.log" 2>&1 &
    AGENT_PID=$!
else
    step "Step 1/3 — 启动 MicroXRCEAgent (Serial: $SERIAL_DEV @ $SERIAL_BAUD)"
    if [ ! -e "$SERIAL_DEV" ]; then
        error "串口设备不存在: $SERIAL_DEV"
        ls -la /dev/tty* 2>/dev/null | head -10 || true
        exit 1
    fi
    MicroXRCEAgent serial --dev "$SERIAL_DEV" --baudrate "$SERIAL_BAUD" \
        > "$LOG_DIR/agent.log" 2>&1 &
    AGENT_PID=$!
fi

echo "$AGENT_PID" > "$LOG_DIR/agent.pid"
sleep 3

if kill -0 "$AGENT_PID" 2>/dev/null; then
    info "MicroXRCEAgent 已启动 (PID: $AGENT_PID, transport: $DDS_TRANSPORT)"
else
    error "MicroXRCEAgent 启动失败! 检查: $LOG_DIR/agent.log"
    exit 1
fi

# ---- Step 2: 等待 PX4 DDS 通信就绪 ----
step "Step 2/3 — 等待 PX4 DDS 通信就绪"
DDS_READY=false
for i in $(seq 1 60); do
    if ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status"; then
        info "PX4 DDS 通信就绪 (耗时 ${i}s)"
        DDS_READY=true
        break
    fi
    if [ $((i % 10)) -eq 0 ]; then
        info "  等待 PX4 连接... (${i}s)"
    fi
    sleep 1
done

if [ "$DDS_READY" = false ]; then
    error "PX4 DDS 通信超时 (60s)"
    error ""
    error "硬件检查:"
    error "  1. 飞控是否上电?"
    if [ "$DDS_TRANSPORT" = "udp" ]; then
        error "  2. 网线: Jetson <-> CUAV 7 Nano?"
        error "  3. ping 飞控 IP?"
        error "  4. PX4 参数: UXRCE_DDS_CFG=2, UXRCE_DDS_AG_IP=<Jetson IP>"
    else
        error "  2. 串口线: $SERIAL_DEV?"
        error "  3. PX4 参数: UXRCE_DDS_CFG=1, SER_TEL2_BAUD=$SERIAL_BAUD"
    fi
    kill $AGENT_PID 2>/dev/null
    exit 1
fi

# ---- Step 3: 启动 simple_mission ----
step "Step 3/3 — 启动 simple_mission (真机模式)"

echo ""
echo "┌──────────────────────────────────────────────────────────┐"
echo "│           simple_mission 真机任务                         │"
echo "├──────────────────────────────────────────────────────────┤"
echo "│  通信:     MicroXRCEAgent $DDS_TRANSPORT                  │"
echo "│  解锁:     标准 ARM (非 SITL force)                      │"
echo "│  Home:     解锁点 (0, 0)                                 │"
echo "│  投放区:   (20.0, 2.0)                                   │"
echo "│  巡航高度: 5.0m                                          │"
echo "│  安全:     遥控器 Arm Switch 解锁 → is_armed() → 起飞       │"
echo "│  任务:     TAKEOFF -> 航点 -> LOITER -> RETURN -> LAND   │"
echo "├──────────────────────────────────────────────────────────┤"
echo "│  日志:     $LOG_DIR                                      │"
echo "│  紧急停止: kill $AGENT_PID && 遥控器切 MANUAL              │"
echo "└──────────────────────────────────────────────────────────┘"
echo ""

ros2 launch drone_bringup simple_mission.launch.py \
    sim_mode:=false \
    home_x:=0.0 \
    home_y:=0.0 \
    home_z:=0.0 \
    drop_x:=2.0 \
    drop_y:=20.0 \
    cruise_altitude:=5.0 \
    loiter_time:=5.0 \
    rc_require_arm_signal:=true \
    "$@" 2>&1 | tee "$LOG_DIR/mission.log"

# ---- 清理 ----
info "任务结束, 清理..."
kill $AGENT_PID 2>/dev/null || true
info "完成."
