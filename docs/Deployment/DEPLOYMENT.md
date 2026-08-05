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

> ⚠️ 版本切换或消息接口变更后**必须全量清理重建**，增量构建会残留旧版本产物导致启动即崩溃，见 [§2 构建产物清理](#2-构建产物清理)。

## 2. 构建产物清理

构建系统（colcon + rosidl）的增量构建原则是**只增不删**：源码有变化就重新生成产物，但不会删除"上次生成、这次源码中已不存在"的旧产物。消息接口（`.msg`/`.srv`）变更尤其危险：

- 生成代码是消息定义的"快照"；C typesupport 库与 Python 静态 typesupport 扩展（`*_s__rosidl_typesupport_c*.so`）各自独立判断是否重跑，极易出现**新库 + 旧扩展**的混搭；
- 编译期不报错（编译只针对本次生成的代码），运行时 import 才炸：`undefined symbol: ..._convert_to_py` / `UnsupportedTypeSupport`，且**每次启动必现**，不是偶发。

### 什么时候必须清理后重建

| 场景 | 说明 |
|---|---|
| 消息接口变更 | 新增/删除 `.msg`/`.srv`、修改字段、改 CMakeLists 消息列表 |
| 版本切换 | git pull / checkout / 回退 / 分支切换，尤其涉及接口与依赖时 |
| 环境变化 | 换机器或架构（x86_64 ↔ aarch64 的 `install/` 不可互拷）、升级 Python / ROS / 系统库 |
| 诡异症状 | `undefined symbol`、`UnsupportedTypeSupport`、改了代码行为不变、报错引用已删除文件 |

### 什么时候不需要清理

- 只改 `.py` 源码：`--symlink-install` 构建下直接生效，无需编译；
- 只改 yaml 配置；
- 同一版本内、消息接口未变的 C++ 改动：`colcon build --packages-select <pkg>` 即可。

### 操作方法

```bash
# 全量清理（版本切换后默认走这个，最保险）
rm -rf build install log
colcon build --symlink-install

# 局部清理（只动接口时）
rm -rf build/drone_interfaces install/drone_interfaces
colcon build --packages-up-to drone_vision test_mission px4_control
# --packages-up-to = 该包 + 它依赖的所有包
```

实飞事故实例（版本回退后两次任务启动即崩溃）见 [WORKLOG.md](WORKLOG.md) 2026-08-02 复盘。

## 3. 完整 SITL

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

## 4. SITL 环境切换

不要在已经加载真机 v1 消息的终端中直接运行 SITL 节点。需要手动启动节点时：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/CUADC_drone_missions_ws/scripts/sitl_env.sh
ros2 launch drone_bringup sitl.launch.py
```

SITL launch 默认 Home 为 World `(4,10,0)`、飞行轴为 north、投放距离 32.5 m、侦察距离 57.5 m。

## 5. 真机全系统

真机使用 `real_flight.launch.py`。物理连接和默认参数为：

- 相机：USB → Jetson V4L2，`/dev/v4l/by-id/usb-D-VITEC_U3344W-video-index0`；
- PX4：以太网 UDP → Jetson MicroXRCEAgent，监听端口 `8888`；
- PX4 当前参数：`UXRCE_DDS_CFG=1000`、`UXRCE_DDS_AG_IP=192.168.1.11`、`UXRCE_DDS_PRT=8888`；
- PX4 状态话题：`/fmu/out/vehicle_status_v1`；
- 相机分辨率：640×480，内参必须替换为实测标定值；
- `rc_require_arm_signal=true`，必须由遥控器 Arm Switch 允许解锁。

推荐入口是启动脚本，脚本持有唯一 MicroXRCEAgent（test_mission 模式）：

```bash
cd /home/ryder/drone_tasks_ws
./scripts/start_full_real_mission.sh
```

脚本先清理旧进程、启动 `MicroXRCEAgent udp4 -p ${DDS_PORT:-8888}` 并等待
`/fmu/out/vehicle_status_v1` 就绪，再由 launch 启动任务节点。
如需覆盖 Agent 端口或相机设备：

```bash
DDS_PORT=8888 ./scripts/start_full_real_mission.sh camera_device:=/dev/video0
```

开机自启直接运行全流程脚本（与 test_mission 相同的启动逻辑）：

```bash
@reboot /home/ryder/drone_tasks_ws/scripts/start_full_real_mission.sh
```

仅当飞控晚于 Jetson 上电超过 60s（脚本 DDS 等待上限）时，改用 `--agent-only`
模式（仅持有 Agent、无限重试连接、不启动任务）。

脚本会自动补齐 PATH（/opt/ros/humble/bin、/usr/local/bin）并把环境中残留的
SITL v4 overlay 剥离，因此自启环境下也能可靠拉起 Agent。

同一 UDP 端口只能运行一个 Agent；脚本 Step 0 会清理旧 Agent 保证端口唯一。
Agent 意外退出时脚本检测并停止任务节点（等价旧的 launch exit guard），不能忽略该报错继续飞行。启动后先观察 `/fmu/out/vehicle_status_v1`、`/drone/pose`、`/drone/vision/status` 和 `/drone/servo/status`，确认就绪再允许任务进入 ARM。现场检查顺序见 [REAL_HARDWARE_DEBUG_GUIDE.md](REAL_HARDWARE_DEBUG_GUIDE.md)。

## 6. 分阶段 smoke test

`scripts/simple_mission_real.sh` 是旧的简化航点/投放测试入口，不等价于完整 FSM。它适合验证通信、起飞、简单航点和返航链路；完整任务必须使用 `real_flight.launch.py`。

## 7. 停止与故障定位

- SITL 正常结束后，脚本会结束 ROS 2 launch；若异常退出，先读取当前日志目录中的 PID 文件，再只停止该次运行创建的进程。
- 真机异常时优先由操作者切换遥控器安全模式/手动模式，再停止上位机节点；不要在飞行中依赖终端清理命令代替接管。
- 视觉未就绪：检查模型路径、ONNX Runtime provider、相机话题和 `/drone/vision/status`。
- 启动即崩溃 `undefined symbol` / `UnsupportedTypeSupport`：版本切换或接口变更后增量构建产物残留，执行全量清理重建（见 [§2 构建产物清理](#2-构建产物清理)）。
- PX4 无状态：检查网线、Jetson IP、UDP `8888` 监听、PX4 Ethernet uXRCE 参数和 v1 消息环境。
- 舵机无动作：检查 `/drone/servo/trigger_drop`、MAIN5 配置和 [飞控参数修改记录](../real_flight_logs/飞控参数修改记录.md)。
- 降落失败：检查 H 检测置信度、视觉对心日志、盲降计时和 `vehicle_land_detected`；不得把问题掩盖为 PX4 LAND。
