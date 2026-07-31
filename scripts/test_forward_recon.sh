#!/usr/bin/env bash
# ==============================================================================
# test_forward_recon 真机启动 — 分阶段测试: 前飞 20m + 侦察区
#
# Phase 1: 起飞 → 向北直飞 20m → 验证航点到达
# Phase 2: 降至侦察高度 → 弓字形全覆盖扫描 → 直接返航(不爬升) → 降落
#
# 硬件连接:
#   - CUAV 7 Nano 通过网线直连 Jetson Orin Nano (UDP 模式)
#   - 或通过串口 TELEM2 -> Jetson UART (Serial 模式)
#
# 用法:
#   # 以太网连接 (默认)
#   ./scripts/test_forward_recon.sh
#
#   # 串口连接
#   DDS_TRANSPORT=serial SERIAL_DEV=/dev/ttyTHS0 ./scripts/test_forward_recon.sh
#
#   # 自定义参数
#   ./scripts/test_forward_recon.sh \
#       forward_distance:=25.0 \
#       recon_altitude:=2.5 \
#       recon_area_half_north:=5.0
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
    error "工作空间未编译, 请先运行: colcon build --symlink-install"
    exit 1
fi
source "$PROJ/install/setup.bash"
export ROS_DOMAIN_ID=0

# ---- DDS 通信方式 ----
DDS_TRANSPORT="${DDS_TRANSPORT:-udp}"
DDS_PORT="${DDS_PORT:-8888}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyTHS0}"
SERIAL_BAUD="${SERIAL_BAUD:-921600}"

# ---- Step 0: 清理旧进程 ----
step "Step 0/3 — 清理旧进程"
pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
pkill -9 -f "px4_bridge" 2>/dev/null || true
pkill -9 -f "vision_pipeline" 2>/dev/null || true
pkill -9 -f "camera_node" 2>/dev/null || true
#pkill -9 -f "test_forward_recon" 2>/dev/null || true
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

# ---- Step 3: 启动 test_forward_recon ----
step "Step 3/3 — 启动 test_forward_recon (真机模式)"

echo ""
echo "┌──────────────────────────────────────────────────────────┐"
echo "│        test_forward_recon 分阶段真机测试                   │"
echo "├──────────────────────────────────────────────────────────┤"
echo "│  安全:     遥控器 Arm Switch 解锁 → is_armed() → 起飞       │"
echo "│  通信:     MicroXRCEAgent $DDS_TRANSPORT                  │"
echo "│  解锁:     标准 ARM (非 SITL force)                      │"
echo "│  Home:     解锁点 (0, 0)                                 │"
echo "├──────────────────────────────────────────────────────────┤"
echo "│  Phase 1:  起飞 → 向北直飞 ${forward_distance:-20.0}m         │"
echo "│  Phase 2:  侦察区扫描 → 返航                               │"
echo "│  LAND:     H 标识视觉伺服精准降落 (vision_pipeline)         │"
echo "├──────────────────────────────────────────────────────────┤"
echo "│  日志:     $LOG_DIR                                      │"
echo "│  紧急停止: 关闭遥控器 或 kill $AGENT_PID                    │"
echo "└──────────────────────────────────────────────────────────┘"
echo ""

ros2 launch test_mission test_forward_recon.launch.py \
    sim_mode:=false \
    rc_require_arm_signal:=true \
    home_x:=0.0 \
    home_y:=0.0 \
    home_z:=0.0 \
    forward_distance:=20.0 \
    cruise_altitude:=5.0 \
    transit_speed:=2.0 \
    recon_altitude:=3.0 \
    recon_area_half_north:=4.0 \
    recon_area_half_east:=3.0 \
    sweep_speed:=0.8 \
    grid_spacing:=1.0 \
    "$@" 2>&1 | tee "$LOG_DIR/mission.log"

# ---- 清理 ----
info "任务结束, 清理..."
kill $AGENT_PID 2>/dev/null || true
info "完成."
