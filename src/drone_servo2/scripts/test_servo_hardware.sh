#!/bin/bash
# ============================================================================
# 真实舵机硬件测试 — 最小链路独立测试
#
# 使用方式 (在 Jetson 上):
#   chmod +x test_servo_hardware.sh
#   ./test_servo_hardware.sh
#
# 可选参数:
#   ./test_servo_hardware.sh --udp-port 8888
#   ./test_servo_hardware.sh --init-only     # 仅初始闭合, 不执行投放
#   ./test_servo_hardware.sh --dry-run       # 仅检查环境, 不启动任何节点
#
# 链路:
#   PX4 Ethernet → MicroXRCEAgent (UDP→DDS) → px4_bridge (DDS↔ROS2)
#     → servo_controller (jetson) → /drone/cmd/vehicle_command_relay
#       → px4_bridge relay → /fmu/in/vehicle_command
#         → PX4 → MAIN5 PWM 50Hz → 舵机
#
# 投放序列:
#   1150μs 闭合 → 1700μs 投放#1 → 2150μs 投放#2
# ============================================================================

set -e

# ---- 默认参数 ----
DDS_PORT=8888
DRY_RUN=false
INIT_ONLY=false
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ---- 解析参数 ----
while [[ $# -gt 0 ]]; do
    case "$1" in
        --udp-port) DDS_PORT="$2"; shift 2 ;;
        --dry-run) DRY_RUN=true; shift ;;
        --init-only) INIT_ONLY=true; shift ;;
        -h|--help)
            echo "用法: $0 [--udp-port PORT] [--dry-run] [--init-only]"
            echo ""
            echo "  --udp-port PORT Agent UDP 监听端口 (默认 8888)"
            echo "  --dry-run       仅检查环境, 不启动节点"
            echo "  --init-only     仅初始闭合舵机, 不执行投放"
            echo "  -h, --help      显示帮助"
            exit 0
            ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

ok()   { echo -e "${GREEN}✅ $1${NC}"; }
warn() { echo -e "${YELLOW}⚠️  $1${NC}"; }
err()  { echo -e "${RED}❌ $1${NC}"; }
info() { echo -e "${CYAN}ℹ️  $1${NC}"; }

# ---- 环境检查 ----
echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║     舵机硬件独立测试 — 环境检查              ║"
echo "╚══════════════════════════════════════════════╝"
echo ""

# 1. ROS2
if command -v ros2 &>/dev/null; then
    ok "ros2: $(which ros2)"
else
    err "ros2 未找到! 先 source /opt/ros/humble/setup.bash"
fi

# 2. Workspace
if [ -f ~/CUADC_drone_missions_ws/install/setup.bash ]; then
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    source ~/CUADC_drone_missions_ws/install/setup.bash 2>/dev/null || true
    ok "workspace sourced"
else
    err "~/CUADC_drone_missions_ws/install/setup.bash 不存在"
    exit 1
fi

# 3. 可执行文件
if ros2 pkg executables drone_servo2 2>/dev/null | grep -q servo_controller; then
    ok "drone_servo2 servo_controller 可用"
else
    err "drone_servo2 未编译, 先 colcon build"
    exit 1
fi

if ros2 pkg executables px4_control 2>/dev/null | grep -q px4_bridge; then
    ok "px4_control px4_bridge 可用"
else
    err "px4_control 未编译"
    exit 1
fi

# 4. MicroXRCEAgent
if command -v MicroXRCEAgent &>/dev/null; then
    ok "MicroXRCEAgent: $(which MicroXRCEAgent)"
else
    err "MicroXRCEAgent 未安装! 安装: sudo apt install micro-xrce-dds-agent"
    exit 1
fi

echo ""
info "环境检查完成"
echo ""

if $DRY_RUN; then
    echo "dry-run 模式 — 到此为止"
    exit 0
fi

# ========================================================================
# 启动最小链路
# ========================================================================

echo "╔══════════════════════════════════════════════╗"
echo "║     启动舵机控制最小链路                     ║"
echo "╚══════════════════════════════════════════════╝"
echo ""

cleanup() {
    echo ""
    info "清理进程..."
    kill $SERVO_PID 2>/dev/null || true
    wait $SERVO_PID 2>/dev/null || true
    kill $BRIDGE_PID 2>/dev/null || true
    wait $BRIDGE_PID 2>/dev/null || true
    kill $XRCE_PID 2>/dev/null || true
    wait $XRCE_PID 2>/dev/null || true
    echo "done."
}
trap cleanup EXIT INT TERM

# 1. MicroXRCEAgent — Ethernet UDP ↔ DDS
info "启动 MicroXRCEAgent (Ethernet UDP :$DDS_PORT)..."
MicroXRCEAgent udp4 -p "$DDS_PORT" &
XRCE_PID=$!
sleep 2

# 检查 Agent 是否存活
if ! kill -0 $XRCE_PID 2>/dev/null; then
    err "MicroXRCEAgent 启动失败"
    err "检查: 1) UDP 端口是否已被占用 2) CUAV 7-Nano 是否上电 3) 网线和 PX4 Ethernet 参数"
    exit 1
fi
ok "MicroXRCEAgent 已启动 (PID=$XRCE_PID)"

# 2. px4_bridge — DDS ↔ ROS2
info "启动 px4_bridge..."
ros2 run px4_control px4_bridge --ros-args --log-level info &
BRIDGE_PID=$!
sleep 2

if ! kill -0 $BRIDGE_PID 2>/dev/null; then
    err "px4_bridge 启动失败"
    exit 1
fi
ok "px4_bridge 已启动 (PID=$BRIDGE_PID)"

# 等待 DDS 建连 + PX4 心跳
info "等待 PX4 连接 (最多 15s)..."
for i in $(seq 1 15); do
    if ros2 topic list 2>/dev/null | grep -qx "/fmu/out/vehicle_status_v1"; then
        ok "PX4 已连接 (${i}s)"
        break
    fi
    sleep 1
    if [ $i -eq 15 ]; then
        err "PX4 连接超时 — 检查: 1) CUAV 7-Nano 是否已上电 2) 网线 3) UXRCE_DDS_CFG=1000 4) Jetson IP/UDP:$DDS_PORT"
        exit 1
    fi
done

# 3. servo_controller — jetson 模式
info "启动 servo_controller (jetson 模式)..."
ros2 run drone_servo2 servo_controller --ros-args -p platform:=jetson --log-level info &
SERVO_PID=$!
sleep 2

if ! kill -0 $SERVO_PID 2>/dev/null; then
    err "servo_controller 启动失败"
    exit 1
fi
ok "servo_controller 已启动 (PID=$SERVO_PID)"

# 等待舵机服务就绪
info "等待舵机服务就绪..."
for i in $(seq 1 10); do
    if ros2 service list 2>/dev/null | grep -q "/drone/servo/trigger_drop"; then
        ok "舵机服务已就绪"
        break
    fi
    sleep 1
    if [ $i -eq 10 ]; then
        err "舵机服务超时"
        exit 1
    fi
done

# 等待 MAIN5 初始化 ACK (servo_controller 会重试 init)
info "等待 MAIN5 初始闭合位置确认..."
sleep 5

echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║     链路就绪 — 舵机已闭合 (1150μs)           ║"
echo "╚══════════════════════════════════════════════╝"
echo ""

# ========================================================================
# 测试投放
# ========================================================================

if $INIT_ONLY; then
    echo ""
    info "--init-only 模式: 舵机已处于初始闭合位置, 跳过投放测试"
    info "要测试投放, 去掉 --init-only 参数重新运行"
    echo ""
    info "释放前请按 Ctrl-C 退出, 舵机保持在闭合位置"
    # 保持运行直到用户中断
    wait $SERVO_PID 2>/dev/null || true
    exit 0
fi

echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║     开始舵机投放测试                          ║"
echo "╚══════════════════════════════════════════════╝"
echo ""
echo -e "${YELLOW}⚠️  确保: 舵机线已连接 CUAV 7-Nano MAIN5${NC}"
echo -e "${YELLOW}⚠️  确保: 投放物已正确挂载${NC}"
echo ""
echo -n "按 Enter 继续, Ctrl-C 取消..."
read -r

# 监听 VehicleCommand relay (后台记录)
info "开始记录 VehicleCommand 输出..."
stdbuf -oL ros2 topic echo /drone/cmd/vehicle_command_relay --field param1 \
    > /tmp/servo_test_pwm.log 2>&1 &
ECHO_PID=$!
sleep 0.5

# ---- 投放 #1: 1150→1700 μs ----
echo ""
echo "=== 投放 #1: 1150 → 1700 μs ==="
echo -n "按 Enter 触发..."
read -r
RESULT1=$(ros2 service call /drone/servo/trigger_drop \
    drone_interfaces/srv/TriggerDrop "cylinder_id: 1" 2>&1)
echo "$RESULT1"
if echo "$RESULT1" | grep -q "success=True"; then
    ok "投放 #1 成功 — 舵机应已转到 1700μs 位置"
else
    warn "投放 #1 返回非成功 — 检查 PX4 日志"
fi
sleep 1.5

# ---- 投放 #2: 1700→2150 μs ----
echo ""
echo "=== 投放 #2: 1700 → 2150 μs ==="
echo -n "按 Enter 触发..."
read -r
RESULT2=$(ros2 service call /drone/servo/trigger_drop \
    drone_interfaces/srv/TriggerDrop "cylinder_id: 2" 2>&1)
echo "$RESULT2"
if echo "$RESULT2" | grep -q "success=True"; then
    ok "投放 #2 成功 — 舵机应已转到 2150μs 位置"
else
    warn "投放 #2 返回非成功 — 检查 PX4 日志"
fi
sleep 1.5

# ---- 验证: 第三次应被拒绝 ----
echo ""
echo "=== 验证: 第3次应被拒绝 (max_drops=2) ==="
RESULT3=$(ros2 service call /drone/servo/trigger_drop \
    drone_interfaces/srv/TriggerDrop "cylinder_id: 3" 2>&1)
echo "$RESULT3"
if echo "$RESULT3" | grep -q "success=False"; then
    ok "正确拒绝第3次投放 — 溢出保护生效"
else
    warn "可能允许了超限投放, 检查 DropManager max_drops"
fi

# ---- 汇总 ----
echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║     测试完成 — PWM 指令记录                   ║"
echo "╚══════════════════════════════════════════════╝"
echo ""
echo "发往 PX4 的 PWM 归一化值序列:"
echo "  (1150μs→-1.0, 1650μs→0.0, 2150μs→+1.0)"
echo ""
kill $ECHO_PID 2>/dev/null; wait $ECHO_PID 2>/dev/null || true
grep -v "^$" /tmp/servo_test_pwm.log 2>/dev/null | head -10
echo ""
echo "预期序列: -1.0(初始) → 0.0476(1700μs) → 1.0(2150μs)"
echo ""

# 检查状态
ros2 topic echo /drone/servo/status --once 2>/dev/null || true
echo ""

info "测试结束, 清理中..."
rm -f /tmp/servo_test_pwm.log
