#!/usr/bin/env bash
# ==============================================================================
# CUADC_drone_missions_ws — 无人机自主任务全仿真启动脚本
#
# 启动顺序:
#   1. MicroXRCEAgent  (PX4 ↔ ROS2 DDS 桥梁, UDP 8888)
#   2. PX4 SITL + Gazebo Harmonic (drone_field 世界, gz_x500)
#   3. ros_gz_image bridge (Gazebo 相机 → ROS2 Image 话题)
#   4. ROS2 全系统节点 (px4_bridge + vision_pipeline + servo_controller + mission_fsm)
#
# 场地: 70m×8m (北向), H起降点(4,10), 投放区(Gazebo y~42.5), 侦察区(Gazebo y~67.5)
# 飞行方向: 向北 (NED north)
# 任务: 投放区2次精准投放 → 侦察区全覆盖巡逻 → 视觉精准返航降落
# ==============================================================================
set -eo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $(date '+%H:%M:%S') $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $(date '+%H:%M:%S') $*"; }
error() { echo -e "${RED}[ERROR]${NC} $(date '+%H:%M:%S') $*"; }
step()  { echo -e "\n${BLUE}==== $(date '+%H:%M:%S') $* ====${NC}"; }

PROJ="$(cd "$(dirname "$0")/.." && pwd)"
LOG_DIR="$PROJ/log/simulation_$(date +%Y%m%d_%H%M%S)"
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

# 从原始 ROS launch 参数解析场地朝向；原始参数随后完整转发给 ROS2。
LAYOUT_HEADING_DEG="0.0"
for task_arg in "$@"; do
    case "$task_arg" in
        layout_heading_deg:=*)
            LAYOUT_HEADING_DEG="${task_arg#layout_heading_deg:=}"
            ;;
    esac
done

# 每次运行生成独立场地，确保 Gazebo 场地、机体初始 yaw 与 ROS 任务朝向一致。
RUNTIME_FIELD="$LOG_DIR/drone_field.sdf"
GAZEBO_YAW_RAD="$(python3 "$PROJ/field/layout_field.py" \
    "$PROJ/field/drone_field.sdf" "$RUNTIME_FIELD" \
    --heading-deg "$LAYOUT_HEADING_DEG" --home-x 4.0 --home-y 10.0 \
    --print-gazebo-yaw-rad)"
cp "$RUNTIME_FIELD" "$PX4_WORLD"
printf 'layout_heading_deg=%s\nhome_x=4.0\nhome_y=10.0\nruntime_field=%s\ngazebo_yaw_rad=%s\n' \
    "$LAYOUT_HEADING_DEG" "$RUNTIME_FIELD" "$GAZEBO_YAW_RAD" \
    > "$LOG_DIR/layout_heading.log"
info "场地朝向: ${LAYOUT_HEADING_DEG}° (NED 顺时针); Gazebo yaw: ${GAZEBO_YAW_RAD} rad"
info "运行时场地: $RUNTIME_FIELD"

# 清理 PX4 持久化参数 (避免旧参数覆盖启动设置)
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
pkill -9 -f "vision_pipeline" 2>/dev/null || true
pkill -9 -f "servo_controller" 2>/dev/null || true
pkill -9 -f "mission_state_machine" 2>/dev/null || true
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
export PX4_GZ_MODEL_POSE="4,10,0.3,0,0,$GAZEBO_YAW_RAD"
export GZ_IP="127.0.0.1"
export ROS_DOMAIN_ID=0
export HEADLESS=1

./bin/px4 -d -i 0 > "$LOG_DIR/px4.log" 2>&1 &
PX4_PID=$!
echo "$PX4_PID" > "$LOG_DIR/px4.pid"
info "PX4 已启动 (PID: $PX4_PID)"
info "等待 Gazebo 世界就绪..."

# 等待 Gazebo world 加载
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
    error "DDS 话题未就绪 (30s), 检查 MicroXRCEAgent 连接"
    exit 1
fi

# 初始化 SITL 参数；解锁和 Offboard 切换由任务状态机统一负责。
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
step "Step 3/4 — 启动 ros_gz_image bridge (Gazebo 相机 → ROS2)"

# 启动 Python Gazebo 相机桥接 (替代 ros_gz_image, 绕过 DDS 大消息问题)
info "启动 Gazebo 相机桥接 (Python)..."
python3 "$PROJ/src/drone_bringup/scripts/gz_camera_bridge.py" \
    > "$LOG_DIR/camera_bridge.log" 2>&1 &
BRIDGE_PID=$!
echo "$BRIDGE_PID" > "$LOG_DIR/image_bridge.pid"
sleep 2

if kill -0 "$BRIDGE_PID" 2>/dev/null; then
    info "ros_gz_image bridge 已启动 (PID: $BRIDGE_PID)"
else
    error "ros_gz_image bridge 启动失败, 检查: $LOG_DIR/image_bridge.log"
    exit 1
fi

# ============================================================================
# Step 4: ROS2 全系统节点
# ============================================================================
step "Step 4/4 — 启动 ROS2 全系统节点"

echo ""
echo "┌──────────────────────────────────────────────────────────────────┐"
echo "│                    无画面全仿真系统已启动                         │"
echo "├──────────────────────────────────────────────────────────────────┤"
echo "│  场地:      drone_field (70m × 8m)                              │"
echo "│  场地朝向:  ${LAYOUT_HEADING_DEG}° NED 顺时针                         │"
echo "│  无人机:    gz_x500 @ H起降点 (4, 10, 0.3)                      │"
echo "│  飞行方向:  向北 (NED north)                                    │"
echo "│  投放区:    y=40~45m (3个圆筒: Ø15/Ø20/Ø25cm)                   │"
echo "│  侦察区:    y=65~70m (5个圆筒, 3个含危险品标识)                  │"
echo "│  起降点:    (4, 10) — 含 H 标识                                 │"
echo "├──────────────────────────────────────────────────────────────────┤"
echo "│  组件状态:                                                       │"
echo "│    ✓ MicroXRCEAgent    UDP:8888  (PID: $AGENT_PID)              │"
echo "│    ✓ PX4 SITL + Gazebo server      (PID: $PX4_PID)             │"
echo "│    ✓ ros_gz_image bridge          (PID: $BRIDGE_PID)            │"
echo "│    → 正在启动 ROS2 节点...                                       │"
echo "├──────────────────────────────────────────────────────────────────┤"
echo "│  任务流程 (FSM):                                                 │"
echo "│    INIT → ARM → TAKEOFF → TRANSIT → CYLINDER_SEARCH             │"
echo "│    → SELECT → APPROACH → DESCEND → DROP(×2) → ASCEND            │"
echo "│    → RECON_PATROL → CLASSIFY → RETURN → PRECISION_LAND          │"
echo "│    → COMPLETE                                                    │"
echo "├──────────────────────────────────────────────────────────────────┤"
echo "│  日志目录: $LOG_DIR                                              │"
echo "│  停止: kill $AGENT_PID $PX4_PID $BRIDGE_PID                     │"
echo "└──────────────────────────────────────────────────────────────────┘"
echo ""

# 启动 ROS2 系统
ros2 launch drone_bringup sitl.launch.py "$@" 2>&1 | tee "$LOG_DIR/ros2_system.log"
