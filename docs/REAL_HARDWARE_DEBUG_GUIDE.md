# 真机舵机调试指南

带此清单去现场, 按顺序做. 每一步失败即停, 排查后再继续.

---

## 第一步: 通电检查

Jetson 上电 → 飞控上电 → QGC 连接飞控 → 确认 MAIN5 配置:

- 执行器页面 → MAIN5 = "Peripheral via Actuator Set 1"
- MAIN5 的 PWM Min = 1150, PWM Max = 2150
- 确认 Disarmed 状态下 MAIN5 输出状态

---

## 第二步: 验证 px4_bridge 通信

```bash
ssh jetson
cd ~/drone_tasks_ws
source install/setup.bash
ros2 run px4_control px4_bridge
```

等待 "注册完成" 日志.

另开终端验证:
```bash
source install/setup.bash
ros2 topic echo /drone/pose --once    # 应看到位姿数据
```

不通 → 检查飞控与 Jetson 串口连接.

---

## 第三步: 启动舵机节点

```bash
# 保持 px4_bridge 运行
# 新终端:
source install/setup.bash
ros2 run drone_servo2 servo_controller --ros-args -p platform:=jetson
```

预期输出:
```
舵机: PX4 VehicleCommand 模式 (set=1 idx=0 pwm=[1150..2150] init=1150μs)
投放序列 (2 次):
  [0] 1150 μs ← 初始/闭合
  [1] 1700 μs
  [2] 2150 μs
舵机投放服务已就绪
```

舵机应保持 1150μs (闭合), 不动作.

---

## 第四步: 触发投放

```bash
# 新终端:
source install/setup.bash
ros2 service call /drone/servo/trigger_drop drone_interfaces/srv/TriggerDrop "cylinder_id: 1"
```

观察舵机是否从 1150 → 1700μs.

**不动时检查 QGC 输出映射:**

MAIN5 必须配置为 `Peripheral via Actuator Set 1`。控制器固定发送
`DO_SET_ACTUATOR.param1`，并将 `param7` 设为 0；不要通过尝试其他索引绕过映射错误。

---

## 第五步: 抓取 VehicleCommand 确认链路

```bash
ros2 topic echo /drone/cmd/vehicle_command_relay
```

触发投放后应看到 `command: 187`. 有 → 软件通; 无 → servo_controller 问题.

---

## 第六步: 解锁后测试

部分 PX4 配置锁外围执行器, 需解锁后才输出 PWM:

```bash
# QGC 里手动解锁 (遥控器或按钮)
# 或:
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "value: true"
```

**注意: 解锁后电机也转, 确保不装桨!**

解锁后重试投放.

---

## 快速排查表

| 现象 | 原因 | 动作 |
|---|---|---|
| 舵机不动, VehicleCommand 有输出 | MAIN5 功能或 PWM 范围错误 | QGC 设置 MAIN5 为 Peripheral via Actuator Set 1, 范围 1150-2150μs |
| 舵机不动, VehicleCommand 无输出 | servo_controller 没发布 | 检查节点日志 |
| 舵机动了但角度不对 | PWM 范围不匹配 | QGC 调 PWM Min/Max |
| 飞控不响应 VehicleCommand | 未解锁或 PX4 配置锁 | 解锁后重试 |
| px4_bridge 没输出 | DDS 连接问题 | 检查串口/USB |
