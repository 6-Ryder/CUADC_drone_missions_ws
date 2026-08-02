# 构建与部署

本文档只记录当前可执行的启动方式。真机飞行前必须由现场操作者完成遥控器、PX4 参数和硬件急停检查。

## 1. 构建

```bash
cd /home/nvidia/CUADC_drone_missions_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

验证：

```bash
pytest -q src/drone_mission/test src/drone_vision/test src/px4_control/test
```

视觉配置引用的模型位于 `src/drone_vision/drone_vision/models/`，当前运行模型为 ONNX。模型、ROS 2 Humble、PX4 消息和 Python 依赖缺失时不要开始飞行。

## 2. 完整 SITL

前置条件：

- PX4-Autopilot 已构建 `px4_sitl_default`；
- Gazebo Harmonic、`gz` 命令和 MicroXRCEAgent 可用；
- `field/drone_field.sdf` 存在；
- 模型文件和工作区已构建。

执行：

```bash
cd /home/nvidia/CUADC_drone_missions_ws
./scripts/start_full_simulation.sh
```

脚本会将工作区场景同步到 PX4，并创建 `log/simulation_YYYYMMDD_HHMMSS/`。常用日志：

| 文件 | 内容 |
|---|---|
| `px4.log` | PX4 启动、DDS 和飞控状态 |
| `agent.log` | MicroXRCEAgent |
| `camera_bridge.log` | Gazebo 图像桥 |
| `ros2_system.log` | launch、视觉、舵机和 FSM 合并日志 |
| `ros/` | 各 ROS 2 节点独立日志 |

SITL 脚本内部执行 `source scripts/sitl_env.sh`。该脚本先加载工作区，再将隔离构建的 v4 `px4_msgs` overlay 提升到环境前端，使 `vehicle_status_v4` 与 PX4 SITL 匹配。

## 3. SITL 环境切换

不要在已经加载真机 v1 消息的终端中直接运行 SITL 节点。需要手动启动节点时：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/CUADC_drone_missions_ws/scripts/sitl_env.sh
ros2 launch drone_bringup sitl.launch.py
```

SITL launch 默认 Home 为 World `(4,10,0)`、飞行轴为 north、投放距离 32.5 m、侦察距离 57.5 m。

## 4. 真机全系统

真机使用 `real_flight.launch.py`，默认设备参数为：

- V4L2 相机：`/dev/v4l/by-id/usb-D-VITEC_U3344W-video-index0`；
- 串口 MicroXRCEAgent：`/dev/ttyTHS1`、921600 baud；
- PX4 状态话题：`/fmu/out/vehicle_status_v1`；
- 相机分辨率：640×480，内参必须替换为实测标定值；
- `rc_require_arm_signal=true`，必须由遥控器 Arm Switch 允许解锁。

启动：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/CUADC_drone_missions_ws/install/setup.bash
ros2 launch drone_bringup real_flight.launch.py
```

如果使用 UDP Agent 或不同设备路径，可覆盖 launch 参数：

```bash
ros2 launch drone_bringup real_flight.launch.py \
  camera_device:=/dev/video0 \
  serial_device:=/dev/ttyTHS1 \
  serial_baud:=921600
```

启动后先观察 `/drone/pose`、`/drone/vision/status` 和 `/drone/servo/status`，确认就绪再允许任务进入 ARM。现场检查顺序见 [REAL_HARDWARE_DEBUG_GUIDE.md](REAL_HARDWARE_DEBUG_GUIDE.md)。

## 5. 分阶段 smoke test

`scripts/simple_mission_real.sh` 是旧的简化航点/投放测试入口，不等价于完整 FSM。它适合验证通信、起飞、简单航点和返航链路；完整任务必须使用 `real_flight.launch.py`。

## 6. 停止与故障定位

- SITL 正常结束后，脚本会结束 ROS 2 launch；若异常退出，先读取当前日志目录中的 PID 文件，再只停止该次运行创建的进程。
- 真机异常时优先由操作者切换遥控器安全模式/手动模式，再停止上位机节点；不要在飞行中依赖终端清理命令代替接管。
- 视觉未就绪：检查模型路径、ONNX Runtime provider、相机话题和 `/drone/vision/status`。
- PX4 无状态：检查 Agent 传输方式、串口/UDP 参数和 v1 消息环境。
- 舵机无动作：检查 `/drone/servo/trigger_drop`、MAIN5 配置和 [飞控参数修改记录](../real_flight_logs/飞控参数修改记录.md)。
- 降落失败：检查 H 检测置信度、视觉对心日志、盲降计时和 `vehicle_land_detected`；不得把问题掩盖为 PX4 LAND。
