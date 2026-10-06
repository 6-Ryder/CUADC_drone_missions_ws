#!/usr/bin/env bash
# ==============================================================================
# 全流程真机快速投放 — Jetson 启动入口 (独立于 start_full_real_mission.sh)
#
# 与 start_full_real_mission.sh 相同的结构: 脚本持有唯一 MicroXRCEAgent,
# launch 只启动任务节点。区别仅是 launch 换 quick_flight.launch.py
# (mission_params_quick.yaml + servo_params_quick.yaml)。默认 bucket 桶盘配对投放；
# 可选择 blue 使用原有 2.4m 蓝盘投放；两者均一次全开双投。
# 默认场地方向为北偏东 54°；真机不会生成 SDF，必须确认实体场地方向一致.
#
# 用法:
#   # 现场终端直接运行快速投放全流程
#   ./scripts/start_quick_mission_real.sh
#   ./scripts/start_quick_mission_real.sh camera_device:=/dev/video0
#   ./scripts/start_quick_mission_real.sh quick_drop_profile:=blue
#
#   # 仅启动 Agent (可选: 飞控晚于 Jetson 上电超过 60s 时用于自启)
#   ./scripts/start_quick_mission_real.sh --agent-only
#   DDS_PORT=8888 ./scripts/start_quick_mission_real.sh --agent-only
#
# 环境变量: DDS_PORT=8888 (Agent 端口), DDS_WAIT_SECONDS=60 (等 DDS 上限).
# 每次运行的完整过程写入 <log 目录>/startup.log (自启下也可见).
#
# 本脚本不会 Arm、切模式、投放、降落或返航; 不会发送任何飞行命令。
# ==============================================================================
# 注意: 不能用 nounset 选项 (-u)! /opt/ros/humble/setup.bash 第 8 行读取
# $AMENT_TRACE_SETUP_FILES (无 :- 保护), 变量未赋值时 -u 直接报错退出,
# 这正是此前"开机自启空日志 + disconnected"的根因。与 test_mission 同为
# set -eo pipefail (仓库测试 ..._sources_ros_before_enabling_nounset 已记录此约束).
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="$(date +%Y%m%d_%H%M%S_%N)_$$"
LOG_DIR="$WORKSPACE_DIR/log/quick_real_mission_$RUN_ID"
DDS_PORT="${DDS_PORT:-8888}"
DDS_WAIT_SECONDS="${DDS_WAIT_SECONDS:-60}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m'

info() { echo -e "${GREEN}[INFO]${NC}  $(date '+%H:%M:%S') $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $(date '+%H:%M:%S') $*"; }
error() { echo -e "${RED}[ERROR]${NC} $(date '+%H:%M:%S') $*"; }
step()  { echo -e "\n${BLUE}==== $(date '+%H:%M:%S') $* ====${NC}"; }
fail()  { error "$*"; exit 1; }

MODE="mission"
LAUNCH_ARGS=()
LAYOUT_HEADING_DEG="54.0"
HAS_LAYOUT_HEADING=0
# 可改为 blue；命令行 quick_drop_profile:=... 优先。小桶精投仍需有效标定。
QUICK_DROP_PROFILE="bucket"
HAS_QUICK_PROFILE=0
for arg in "$@"; do
  case "$arg" in
    --agent-only) MODE="agent" ;;
    layout_heading_deg:=*)
      LAYOUT_HEADING_DEG="${arg#layout_heading_deg:=}"
      HAS_LAYOUT_HEADING=1
      LAUNCH_ARGS+=("$arg")
      ;;
    quick_drop_profile:=*)
      QUICK_DROP_PROFILE="${arg#quick_drop_profile:=}"
      HAS_QUICK_PROFILE=1
      LAUNCH_ARGS+=("$arg")
      ;;
    *) LAUNCH_ARGS+=("$arg") ;;
  esac
done
case "$QUICK_DROP_PROFILE" in
  blue|bucket) ;;
  *) fail "quick_drop_profile 只能是 blue 或 bucket: $QUICK_DROP_PROFILE" ;;
esac
# 未显式传参时，将默认方向一并传给 quick_flight.launch.py。
if [ "$HAS_LAYOUT_HEADING" -eq 0 ]; then
  LAUNCH_ARGS+=("layout_heading_deg:=${LAYOUT_HEADING_DEG}")
fi
if [ "$HAS_QUICK_PROFILE" -eq 0 ]; then
  LAUNCH_ARGS+=("quick_drop_profile:=${QUICK_DROP_PROFILE}")
fi

# 自启和手动启动不得相互清理在用节点；在任何清理/DDS 操作前拒绝第二实例。
# 子进程关闭 fd 9，防止 ROS daemon 等长期进程延长锁的生命周期。
command -v flock >/dev/null || fail "未找到 flock，无法保证 quick 单实例运行"
exec 9> "${TMPDIR:-/tmp}/drone_quick_real_${UID}.lock"
flock -n 9 || fail "已有 quick 实例运行；请先停止原任务后再启动，避免中断在用节点"

mkdir -p "$LOG_DIR/ros"
export ROS_LOG_DIR="$LOG_DIR/ros"
printf '%q ' "${LAUNCH_ARGS[@]}" > "$LOG_DIR/launch_arguments.txt"
printf '\n' >> "$LOG_DIR/launch_arguments.txt"

# 自启环境下 stdout 不落盘 (cron→邮件 / systemd→journal), 这里把脚本全部
# 输出镜像到 startup.log: 每次运行都在日志目录留下完整过程, 不再有"日志空"
# 的盲区。终端行为不变 (tee 仍输出到原 stdout)。
exec > >(tee -a "$LOG_DIR/startup.log" 9>&-) 2>&1
# 等待 tee 就绪 (fork 竞态: 极早期输出可能先进管道缓冲); 上限 1s.
i=0
until [ -f "$LOG_DIR/startup.log" ] || [ $i -ge 20 ]; do sleep 0.05; i=$((i + 1)); done

AGENT_PID=""
LAUNCH_PID=""
TAIL_PID=""
WATCHDOG_PID=""

cleanup() {
  [ -n "$WATCHDOG_PID" ] && kill "$WATCHDOG_PID" 2>/dev/null || true
  [ -n "$TAIL_PID" ] && kill "$TAIL_PID" 2>/dev/null || true
  if [ -n "$AGENT_PID" ] && kill -0 "$AGENT_PID" 2>/dev/null; then
    kill "$AGENT_PID" 2>/dev/null || true
    info "MicroXRCEAgent 已清理 (PID $AGENT_PID)"
  fi
}
trap cleanup EXIT

# ---- 环境自愈 (自动启动环境适配) ----
# systemd 默认 PATH 不含 /opt/ros, cron @reboot 默认 PATH=/usr/bin:/bin。
for _p in /opt/ros/humble/bin /usr/local/bin; do
  case ":$PATH:" in
    *":$_p:"*) ;;
    *) PATH="$_p:$PATH" ;;
  esac
done

# SITL v4 overlay 与真机 v1 消息运行时冲突 (WORKLOG "UnsupportedTypeSupport").
# 不硬失败退出 (会阻断自动启动), 而是把相关环境变量中的条目剥离掉。
overlay_stripped=false
for v in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH COLCON_PREFIX_PATH; do
  case "${!v:-}" in
    *sitl_px4_msgs*)
      filtered=""
      old_ifs="$IFS"; IFS=':'
      for entry in ${!v:-}; do
        case "$entry" in
          *sitl_px4_msgs*) ;;
          *) filtered="${filtered:+$filtered:}$entry" ;;
        esac
      done
      IFS="$old_ifs"
      export "$v=$filtered"
      overlay_stripped=true
      ;;
  esac
done
if [ "$overlay_stripped" = true ]; then
  warn "检测到 SITL px4_msgs overlay, 已从环境剥离 (防止真机 v1/v4 消息冲突)"
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
  fail "ROS 2 Humble 未安装: 缺少 /opt/ros/humble/setup.bash"
fi
source /opt/ros/humble/setup.bash

if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  fail "工作空间未构建: 请先在 $WORKSPACE_DIR 运行 colcon build --symlink-install"
fi
source "$WORKSPACE_DIR/install/setup.bash"
export ROS_DOMAIN_ID=0

# ---- Step 0: 清理旧进程 ----
# 同一 UDP 端口只能由一个 Agent 监听; 先清旧进程保证端口唯一。
# 单实例锁已生效；这里只清理此前退出后残留的旧进程。
step "Step 0/3 — 清理旧进程 (保证 UDP :$DDS_PORT 唯一)"
pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
pkill -9 -f "px4_bridge" 2>/dev/null || true
pkill -9 -f "mission_fsm" 2>/dev/null || true
pkill -9 -f "vision_pipeline" 2>/dev/null || true
pkill -9 -f "camera_node" 2>/dev/null || true
sleep 1
info "旧进程已清理"

# ---- Step 1: MicroXRCEAgent (先于一切任务闸门, 保证 DDS 必然就绪) ----
step "Step 1/3 — 启动 MicroXRCEAgent (Ethernet UDP :$DDS_PORT)"
command -v MicroXRCEAgent >/dev/null || fail "未找到 MicroXRCEAgent (安装 micro-ros-agent 或放入 PATH)"
MicroXRCEAgent udp4 -p "$DDS_PORT" > "$LOG_DIR/agent.log" 2>&1 9>&- &
AGENT_PID=$!
echo "$AGENT_PID" > "$LOG_DIR/agent.pid"
sleep 2

if kill -0 "$AGENT_PID" 2>/dev/null; then
  info "MicroXRCEAgent 已启动 (PID $AGENT_PID)"
else
  error "MicroXRCEAgent 启动失败, 日志尾部:"
  tail -5 "$LOG_DIR/agent.log" 2>/dev/null || true
  fail "见 $LOG_DIR/agent.log"
fi

if [ "$MODE" = "agent" ]; then
  info "agent-only 模式: 前台持有 Agent, PX4 会自动重试连接 :$DDS_PORT"
  info "日志: $LOG_DIR/agent.log ; 退出后 Agent 随之停止"
  wait "$AGENT_PID"
fi

# ---- Step 2: 等待 PX4 DDS 通信就绪 ----
step "Step 2/3 — 等待 PX4 DDS 通信就绪"
DDS_READY=false
for i in $(seq 1 "$DDS_WAIT_SECONDS"); do
  if ros2 topic list 9>&- 2>/dev/null | grep -qx "/fmu/out/vehicle_status_v1"; then
    info "PX4 DDS 通信就绪 (耗时 ${i}s)"
    DDS_READY=true
    break
  fi
  [ $((i % 10)) -eq 0 ] && info "  等待 PX4 连接... (${i}s)"
  sleep 1
done

if [ "$DDS_READY" = false ]; then
  error "PX4 DDS 通信超时 (${DDS_WAIT_SECONDS}s, 可用 DDS_WAIT_SECONDS 调大)"
  error ""
  error "硬件检查:"
  error "  1. 飞控是否上电?"
  error "  2. 网线: Jetson <-> CUAV 7 Nano?"
  error "  3. PX4 Ethernet IP 是否可达? (Agent 日志: $LOG_DIR/agent.log)"
  error "  4. PX4 参数: UXRCE_DDS_CFG=1000, UXRCE_DDS_AG_IP=<Jetson IP>, UXRCE_DDS_PRT=$DDS_PORT"
  fail "DDS 未就绪"
fi

# ---- Step 3: 启动任务节点 ----
step "Step 3/3 — 启动任务节点 (quick_flight.launch.py)"

# 部署一致性: 若 install/ 仍是旧版 launch (自带 Agent), 会与脚本的 Agent
# 同端口冲突 → 旧 guard 关掉整套任务。必须重建后才能执行。
if grep -q '"MicroXRCEAgent", "udp4"' \
    "$WORKSPACE_DIR/install/drone_bringup/share/drone_bringup/launch/quick_flight.launch.py" 2>/dev/null; then
  fail "install/ 中的 quick_flight.launch.py 仍是旧版 (自带 Agent); 请先重建: colcon build --packages-select drone_bringup --symlink-install"
fi

echo ""
echo "┌──────────────────────────────────────────────────────────┐"
echo "│           全流程真机快速投放 (Jetson)                      │"
echo "├──────────────────────────────────────────────────────────┤"
echo "│  通信:     PX4 Ethernet → Agent UDP:$DDS_PORT (脚本持有)   │"
echo "│  解锁:     遥控器 Arm Switch → is_armed() → 起飞           │"
echo "│  Home:     解锁点 (0, 0)                                 │"
echo "│  模式:     ${QUICK_DROP_PROFILE}                                             │"
echo "│  投放:     配置搜索高度 → 模式目标高度 → 一次全开双投      │"
echo "│  安全:     Agent 退出自动停止任务; 随时遥控器切 Offboard    │"
echo "├──────────────────────────────────────────────────────────┤"
echo "│  日志:     $LOG_DIR                                      │"
echo "└──────────────────────────────────────────────────────────┘"
echo ""

ros2 launch drone_bringup quick_flight.launch.py "${LAUNCH_ARGS[@]}" \
  > "$LOG_DIR/mission.log" 2>&1 9>&- &
LAUNCH_PID=$!
tail -f "$LOG_DIR/mission.log" 9>&- & TAIL_PID=$!

# Agent 是任务唯一数据源: 意外退出时停止任务 (与旧 launch exit guard 等价).
# 使用精确 PID 而非 pkill 模式匹配, 避免误伤并发 launch 实例或其他进程.
( while kill -0 "$AGENT_PID" 2>/dev/null; do sleep 2; done
  if kill -0 "$$" 2>/dev/null && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    warn "MicroXRCEAgent 已退出, 停止任务节点 (避免 DDS 断开后继续飞行)"
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
  fi ) 9>&- &
WATCHDOG_PID=$!

set +e
wait "$LAUNCH_PID"
LAUNCH_RC=$?
set -e
kill "$TAIL_PID" 2>/dev/null || true

info "任务结束 (launch 退出码 $LAUNCH_RC)"
exit "$LAUNCH_RC"
