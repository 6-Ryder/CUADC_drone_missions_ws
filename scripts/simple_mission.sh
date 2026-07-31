#!/usr/bin/env bash
# ==============================================================================
# simple_mission 仿真一键启动
#
# 与 start_full_simulation.sh 的区别:
#   - 不启动 vision_pipeline / servo_controller / mission_fsm
#   - 只启动 px4_bridge + simple_mission
#   - 适合首次验证飞行控制链路
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
LOG_DIR="$PROJ/log/simple_mission_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
export ROS_LOG_DIR="$LOG_DIR/ros"
mkdir -p "$ROS_LOG_DIR"

# ============================================================================
# 环境检测
# ============================================================================
PX4_DIR="/home/nvidia/PX4-Autopilot"
PX4_BLD="$PX4_DIR/build/px4_sitl_default"
PX4_WORLD="$PX4_DIR/Tools/simulation/gz/worlds/drone_field.sdf"

if [ ! -f "$PX4_BLD/bin/px4" ]; then
    error "PX4 SITL 未编译: $PX4_BLD/bin/px4"
    error "请先编译: cd $PX4_DIR && make px4_sitl_default"
    exit 1
fi

if [ ! -d "$(dirname "$PX4_WORLD")" ]; then
    error "PX4 Gazebo 世界目录不存在: $(dirname "$PX4_WORLD")"
    exit 1
fi

# 工作区场景是唯一来源
cp "$PROJ/field/drone_field.sdf" "$PX4_WORLD"

# 清理 PX4 持久化参数
rm -f "$PX4_BLD/rootfs/0/parameters.bson" "$PX4_BLD/rootfs/0/parameters_backup.bson"
rm -rf "$PX4_BLD/rootfs/0/dataman"
rm -f "$PX4_BLD/rootfs/parameters.bson" "$PX4_BLD/rootfs/parameters_backup.bson"

if ! source "$PROJ/scripts/sitl_env.sh"; then
    error "SITL ROS2 环境初始化失败"
    exit 1
fi

if [ ! -f "$PROJ/install/setup.bash" ]; then
    error "CUADC_drone_missions_ws 未编译, 请先运行: colcon build --symlink-install"
    exit 1
fi
source "$PROJ/install/setup.bash"

if ! command -v MicroXRCEAgent &>/dev/null; then
    error "MicroXRCEAgent 未安装"
    exit 1
fi

# ============================================================================
# Step 0: 清理旧进程
# ============================================================================
step "Step 0/4 — 清理旧进程"
pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gzserver" 2>/dev/null || true
pkill -9 -f "bin/px4" 2>/dev/null || true
pkill -9 -f "px4_bridge" 2>/dev/null || true
pkill -9 -f "simple_mission" 2>/dev/null || true
sleep 3
info "旧进程已清理"

# ============================================================================
# Step 1: MicroXRCEAgent
# ============================================================================
step "Step 1/4 — 启动 MicroXRCEAgent (UDP 8888)"
MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/agent.log" 2>&1 &
AGENT_PID=$!
echo "$AGENT_PID" > "$LOG_DIR/agent.pid"
sleep 2

if kill -0 "$AGENT_PID" 2>/dev/null; then
    info "MicroXRCEAgent 已启动 (PID: $AGENT_PID, UDP:8888)"
else
    error "MicroXRCEAgent 启动失败, 检查: $LOG_DIR/agent.log"
    exit 1
fi

# ============================================================================
# Step 2: PX4 SITL + Gazebo Harmonic
# ============================================================================
step "Step 2/4 — 启动 PX4 SITL + Gazebo (drone_field 世界)"

cd "$PX4_BLD"
export PX4_SIM_MODEL="gz_x500_mono_cam_down"
export PX4_GZ_WORLD="drone_field"
export PX4_GZ_MODEL_POSE="4,10,0.3,0,0,0"
export GZ_IP="127.0.0.1"
export ROS_DOMAIN_ID=0
export HEADLESS=1

./bin/px4 -d -i 0 > "$LOG_DIR/px4.log" 2>&1 &
PX4_PID=$!
echo "$PX4_PID" > "$LOG_DIR/px4.pid"
info "PX4 已启动 (PID: $PX4_PID)"
info "等待 Gazebo 世界就绪..."

WORLD_READY=false
for i in $(seq 1 90); do
    if ! kill -0 "$PX4_PID" 2>/dev/null; then
        error "PX4 意外退出! 查看日志: tail -60 $LOG_DIR/px4.log"
        tail -60 "$LOG_DIR/px4.log"
        exit 1
    fi
    if gz topic -l 2>/dev/null | grep -q "/world/drone_field/clock"; then
        info "Gazebo world [drone_field] 就绪 (耗时 ${i}s)"
        WORLD_READY=true
        break
    fi
    if [ $((i % 10)) -eq 0 ]; then
        info "  等待中... (${i}s)"
    fi
    sleep 1
done

if [ "$WORLD_READY" = false ]; then
    error "Gazebo world 超时未就绪 (90s)"
    exit 1
fi

# 等待 DDS 话题
info "等待 PX4 DDS 话题..."
DDS_READY=false
for i in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -qx "/fmu/out/vehicle_status_v4"; then
        info "PX4 DDS 话题就绪 (${i}s)"
        DDS_READY=true
        break
    fi
    sleep 1
done

if [ "$DDS_READY" = false ]; then
    error "DDS 话题未就绪 (30s)"
    exit 1
fi

# 初始化 SITL 参数
info "初始化 PX4 参数..."
for i in $(seq 1 10); do
    if "$PX4_BLD/bin/px4-param" --instance 0 set CBRK_SUPPLY_CHK 894281 2>/dev/null; then
        break
    fi
    sleep 1
done
"$PX4_BLD/bin/px4-param" --instance 0 set COM_ARM_WO_GPS 1
"$PX4_BLD/bin/px4-param" --instance 0 set COM_DISARM_PRFLT -1
"$PX4_BLD/bin/px4-param" --instance 0 set NAV_RCL_ACT 0
"$PX4_BLD/bin/px4-param" --instance 0 set NAV_DLL_ACT 0
info "PX4 参数已初始化"

# ============================================================================
# Step 3: ros_gz_image bridge
# ============================================================================
step "Step 3/4 — 启动 ros_gz_image bridge"
python3 "$PROJ/src/drone_bringup/scripts/gz_camera_bridge.py" \
    > "$LOG_DIR/camera_bridge.log" 2>&1 &
BRIDGE_PID=$!
echo "$BRIDGE_PID" > "$LOG_DIR/image_bridge.pid"
sleep 2

if kill -0 "$BRIDGE_PID" 2>/dev/null; then
    info "ros_gz_image bridge 已启动 (PID: $BRIDGE_PID)"
else
    warn "ros_gz_image bridge 启动失败, 继续..."
fi

# ============================================================================
# Step 4: ROS2 simple_mission
# ============================================================================
step "Step 4/4 — 启动 simple_mission"

echo ""
echo "┌──────────────────────────────────────────────────────────────────┐"
echo "│              simple_mission 简化任务仿真                          │"
echo "├──────────────────────────────────────────────────────────────────┤"
echo "│  场地:      drone_field (70m × 8m)                              │"
echo "│  飞行方向:  向北 (NED north)                                    │"
echo "│  任务:      TAKEOFF → TRANSIT → LOITER → RETURN → LAND          │"
echo "├──────────────────────────────────────────────────────────────────┤"
echo "│  ✓ MicroXRCEAgent    UDP:8888  (PID: $AGENT_PID)                │"
echo "│  ✓ PX4 SITL + Gazebo drone_field  (PID: $PX4_PID)               │"
echo "│  ✓ ros_gz_image bridge          (PID: $BRIDGE_PID)              │"
echo "│  → 正在启动 simple_mission...                                    │"
echo "├──────────────────────────────────────────────────────────────────┤"
echo "│  日志: $LOG_DIR                                                  │"
echo "│  停止: kill $AGENT_PID $PX4_PID $BRIDGE_PID                     │"
echo "└──────────────────────────────────────────────────────────────────┘"
echo ""

ros2 launch drone_bringup simple_mission.launch.py "$@" 2>&1 | tee "$LOG_DIR/mission.log"
