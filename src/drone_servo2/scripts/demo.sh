#!/bin/bash
# 舵机多位置步进投放演示 — 一条命令看完整投放序列
set -e
source ~/CUADC_drone_missions_ws/install/setup.bash

echo "╔══════════════════════════════════════════════╗"
echo "║  投放序列: 1150 → 1700 → 2150 μs            ║"
echo "║  初始闭合 → 投放1 → 投放2                    ║"
echo "╚══════════════════════════════════════════════╝"
echo ""

pkill -f servo_controller 2>/dev/null || true
sleep 0.3

# 启动节点
ros2 run drone_servo2 servo_controller --ros-args -p platform:=jetson &
NODE=$!
sleep 2

# 后台记录 VehicleCommand
stdbuf -oL ros2 topic echo /drone/cmd/vehicle_command_relay --field param1 \
  > /tmp/servo_pwm.txt 2>&1 &
ECHO=$!
sleep 0.5

echo "=== 投放 #1: 1150 → 1700 μs ==="
ros2 service call /drone/servo/trigger_drop drone_interfaces/srv/TriggerDrop "cylinder_id: 1"
sleep 1.5
echo ""

echo "=== 投放 #2: 1700 → 2150 μs ==="
ros2 service call /drone/servo/trigger_drop drone_interfaces/srv/TriggerDrop "cylinder_id: 2"
sleep 1.5
echo ""

echo "=== 投放 #3 (应被拒绝) ==="
ros2 service call /drone/servo/trigger_drop drone_interfaces/srv/TriggerDrop "cylinder_id: 3"
sleep 0.5
echo ""

echo "=== 发往 PX4 的 PWM 指令序列 ==="
echo "   (normalized: -1.0=1150μs, 0.0=1650μs, +1.0=2150μs)"
echo ""
kill $ECHO 2>/dev/null; wait $ECHO 2>/dev/null || true
grep -v "^$" /tmp/servo_pwm.txt 2>/dev/null | head -10
echo ""

kill $NODE 2>/dev/null; wait $NODE 2>/dev/null || true
rm -f /tmp/servo_pwm.txt

echo "╔══════════════════════════════════════════════╗"
echo "║              演示完成                        ║"
echo "╚══════════════════════════════════════════════╝"
