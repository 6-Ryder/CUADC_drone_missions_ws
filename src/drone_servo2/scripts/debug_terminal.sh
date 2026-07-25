#!/bin/bash
echo "SHELL=$SHELL"
echo "ROS2=$(which ros2)"
echo "alias ros2=$(alias ros2 2>/dev/null || echo none)"
echo ""
echo "=== 不用 ros2 命令, 直接用二进制 ==="
timeout 2 ~/CUADC_drone_missions_ws/install/drone_servo2/lib/drone_servo2/servo_controller --ros-args -p platform:=sitl 2>&1 || true
echo ""
echo "=== 用绝对路径 ros2 ==="
timeout 2 /opt/ros/humble/bin/ros2 run drone_servo2 servo_controller --ros-args -p platform:=sitl 2>&1 || true
echo ""
echo "=== 用 ros2 (PATH 里的) ==="
source /opt/ros/humble/setup.bash
source ~/CUADC_drone_missions_ws/install/setup.bash
timeout 2 ros2 run drone_servo2 servo_controller --ros-args -p platform:=sitl 2>&1 || true
echo ""
echo "=== done ==="
