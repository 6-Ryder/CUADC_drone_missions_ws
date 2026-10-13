# 真机现场调试指南

这是现场操作清单，不替代操作者的遥控器和硬件急停。每一步失败都应停在地面排查。

## 1. 上电与安全

- 螺旋桨、舵机和载荷安装状态已确认，测试区域无人员和障碍物。
- 遥控器已连接，Arm Switch 处于禁止解锁位置，手动接管可用。
- QGroundControl 能连接 CUAV 7-Nano，电池和传感器状态正常。
- 确认 PX4 v1.17 参数记录与当前飞控一致，尤其是 RC、接地检测和速度限制。

参数参考：[飞控参数修改记录](../../real_flight_logs/飞控参数修改记录.md)。

## 2. PX4 DDS 与桥接

先在地面启动环境（脚本会先启动唯一 MicroXRCEAgent、等待 DDS 就绪，再启动任务节点）：

```bash
cd /home/nvidia/CUADC_drone_missions_ws
./scripts/start_full_real_mission.sh
```

确认真机使用 v1 话题：

```bash
ros2 topic list | rg '/fmu/out/vehicle_status_v1|/fmu/out/vehicle_land_detected'
ros2 topic echo /drone/pose --once
```

PX4 与 Jetson 使用以太网 UDP，不使用 Jetson UART。若没有状态或位姿，停止任务，按以下顺序检查：Jetson 网卡 IP、UDP `8888` 监听、PX4 `UXRCE_DDS_CFG=1000`、`UXRCE_DDS_AG_IP=192.168.1.11`、`UXRCE_DDS_PRT=8888` 和网线连接。

## 3. 相机与视觉

确认设备路径和图像流：

```bash
ls -l /dev/v4l/by-id/
ros2 topic list | rg 'image|vision/status'
ros2 topic echo /drone/vision/status --once
```

必须看到视觉 `ready=1`，并核对实际 `fx/fy/cx/cy` 与相机标定；当前真机配置中的 400/400/320/240 仍是待标定值。`ready=1` 不能代替帧龄检查。若模型加载失败、provider 不可用、相机无帧或 ready 未置位，不得解锁。

## 4. 舵机与 MAIN5

地面无载荷测试投放服务：

```bash
ros2 service list | rg '/drone/servo/trigger_drop'
ros2 topic echo /drone/servo/status --once
```

确认 MAIN5 配置为 Peripheral via Actuator Set 1，PWM 范围为 1150–2150 μs。舵机序列只能前进：1150（闭合）→ 1700（第一次释放）→ 2150（第二次释放）。

### ACK 话题名版本差异（易踩坑）

`VehicleCommandAck` 的 DDS 话题名随消息版本不同，**真机与 SITL 不一致**：

| 环境 | MESSAGE_VERSION | ACK 话题名 | servo 订阅 |
|---|---|---|---|
| 真机 (v1) | 0 | `/fmu/out/vehicle_command_ack`（无后缀） | ✅ 无后缀 |
| SITL (v4 overlay) | 1 | `/fmu/out/vehicle_command_ack_v1` | SITL 用 ServoMock，不订阅 |

`servo_controller.cpp` 订阅的是**无后缀**话题，仅真机正确。若 PX4 固件升级改变 `VehicleCommandAck.MESSAGE_VERSION`，话题名会变，servo 会持续报 `PX4 舵机命令 ACK 超时` 而命令实际已被 PX4 接收（`/fmu/in/vehicle_command` 有数据但 ACK 无回）。

排查时三层差分：

```bash
ros2 topic echo /drone/cmd/vehicle_command_relay --field param1   # servo 命令发出 (-1.0)
ros2 topic echo /fmu/in/vehicle_command --field param1            # px4_bridge 中继
ros2 topic echo /fmu/out/vehicle_command_ack --once               # PX4 ACK (result=0)
```

如果前两层有数据、第三层为空，优先怀疑话题名版本不匹配（用 `ros2 topic list | grep ack` 核对真实话题名）。

## 5. 起飞前任务检查

- `vehicle_status_v1`、`/drone/pose`、视觉和舵机均 ready；
- 遥控器允许解锁但仍保留手动接管；
- 任务区域坐标、相机朝向和目标载荷与配置一致；
- 核对 `airframe_params_real.yaml` 中相机下偏移、机体原点及投放点，与实体一致；
- 运行 `python3 -m pytest -q src/drone_mission/test/test_mission_params_contract.py`，确认 YAML double 参数和 launch 浮点覆盖类型正确；
- 操作者知道如何切换手动/安全模式；
- 记录本次日志目录和飞控参数版本。

## 6. 降落与异常

主任务 full 必须先由 H 视觉完成对心锁存，再于约 1.5 m 切入盲降。查看 H 目标采集时刻、2 s 龄期、估计器拒帧和对心锁存日志；盲降默认 45 s 超时后停止并请求人工接管。短时丢失时若有成熟目标且高于近地横移下限，状态会有界地向最后已知位置靠近。quick/simple 的 1.6 m 交接与 PX4 LAND 兜底是另一条路径，不能套用 full 的故障预期。

接地后确认 `/fmu/out/vehicle_land_detected`，并检查是否使用了严格近地静止备用判据，再核对电机停转、舵机状态和任务摘要。备用判据在真机的可靠性仍待实测。任何异常先由遥控器接管，再停止 ROS 2 节点。
