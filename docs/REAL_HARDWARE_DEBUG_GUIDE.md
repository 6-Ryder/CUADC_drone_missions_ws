# 真机现场调试指南

这是现场操作清单，不替代操作者的遥控器和硬件急停。每一步失败都应停在地面排查。

## 1. 上电与安全

- 螺旋桨、舵机和载荷安装状态已确认，测试区域无人员和障碍物。
- 遥控器已连接，Arm Switch 处于禁止解锁位置，手动接管可用。
- QGroundControl 能连接 CUAV 7-Nano，电池和传感器状态正常。
- 确认 PX4 v1.17 参数记录与当前飞控一致，尤其是 RC、接地检测和速度限制。

参数参考：[飞控参数修改记录](../real_flight_logs/飞控参数修改记录.md)。

## 2. PX4 DDS 与桥接

先在地面启动环境：

```bash
source /opt/ros/humble/setup.bash
source /home/ryder/drone_tasks_ws/install/setup.bash
ros2 launch drone_bringup real_flight.launch.py
```

确认真机使用 v1 话题：

```bash
ros2 topic list | rg '/fmu/out/vehicle_status_v1|/fmu/out/vehicle_land_detected'
ros2 topic echo /drone/pose --once
```

如果没有状态或位姿，停止任务，检查 MicroXRCEAgent 的串口/UDP 传输、PX4 `UXRCE_DDS_CFG`、串口权限和网络连接。

## 3. 相机与视觉

确认设备路径和图像流：

```bash
ls -l /dev/v4l/by-id/
ros2 topic list | rg 'image|vision/status'
ros2 topic echo /drone/vision/status --once
```

必须看到视觉 `ready=1`，并使用真机标定的 `fx/fy/cx/cy`。若模型加载失败、provider 不可用、相机无帧或 ready 未置位，不得解锁。

## 4. 舵机与 MAIN5

地面无载荷测试投放服务：

```bash
ros2 service list | rg '/drone/servo/trigger_drop'
ros2 topic echo /drone/servo/status --once
```

确认 MAIN5 配置为 Peripheral via Actuator Set 1，PWM 范围为 1150–2150 μs。舵机序列只能前进：1150（闭合）→ 1700（第一次释放）→ 2150（第二次释放）。

## 5. 起飞前任务检查

- `vehicle_status_v1`、`/drone/pose`、视觉和舵机均 ready；
- 遥控器允许解锁但仍保留手动接管；
- 任务区域坐标、相机朝向和目标载荷与配置一致；
- 操作者知道如何切换手动/安全模式；
- 记录本次日志目录和飞控参数版本。

## 6. 降落与异常

精准降落必须先由 H 视觉完成对心锁存，再进入 1.5 m 盲降。视觉丢失、盲降超时或水平误差未稳定时，系统应停止运动并进入错误处理；不要以 PX4 LAND 代替视觉流程。

接地后确认 `/fmu/out/vehicle_land_detected`，再检查电机停转、舵机状态和任务摘要。任何异常先由遥控器接管，再停止 ROS 2 节点。
