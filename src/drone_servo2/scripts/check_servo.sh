#!/bin/bash
echo "=== 1. ROS2 环境 ==="
which ros2 || echo "❌ ros2 未找到！先运行: source /opt/ros/humble/setup.bash"

echo ""
echo "=== 2. 工作空间 ==="
if [ -f ~/drone_tasks_ws/install/setup.bash ]; then
    source ~/drone_tasks_ws/install/setup.bash
    echo "✅ workspace sourced"
else
    echo "❌ ~/drone_tasks_ws/install/setup.bash 不存在"
fi

echo ""
echo "=== 3. 可执行文件 ==="
ros2 pkg executables drone_servo2 2>&1

echo ""
echo "=== 4. 尝试运行 (3秒超时) ==="
timeout 3 ros2 run drone_servo2 servo_controller --ros-args -p platform:=jetson 2>&1
echo "(exit: $?)"

echo ""
echo "=== 5. 如果上面没有输出, 试试直接执行二进制 ==="
~/drone_tasks_ws/install/drone_servo2/lib/drone_servo2/servo_controller --ros-args -p platform:=jetson &
PID=$!
sleep 2
kill $PID 2>/dev/null
wait $PID 2>/dev/null
