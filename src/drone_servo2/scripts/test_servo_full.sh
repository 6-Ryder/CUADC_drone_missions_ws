#!/bin/bash
source ~/drone_tasks_ws/install/setup.bash

echo "=== 启动节点 ==="
ros2 run drone_servo2 servo_controller --ros-args -p platform:=jetson &
NODE_PID=$!
sleep 1.5

echo ""
echo "=== 触发投放 #1 ==="
ros2 service call /drone/servo/trigger_drop drone_interfaces/srv/TriggerDrop "cylinder_id: 1"

echo ""
echo "=== 触发投放 #2 ==="
ros2 service call /drone/servo/trigger_drop drone_interfaces/srv/TriggerDrop "cylinder_id: 2"

echo ""
echo "=== 第三次应该被拒绝 (max_drops=2) ==="
ros2 service call /drone/servo/trigger_drop drone_interfaces/srv/TriggerDrop "cylinder_id: 3"

echo ""
kill $NODE_PID 2>/dev/null
wait $NODE_PID 2>/dev/null
echo "=== 完成 ==="
