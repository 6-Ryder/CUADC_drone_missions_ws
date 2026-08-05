# 全流程真机飞行指南

本文档对应当前唯一的全流程真机入口。它描述真实 PX4 v1、相机、视觉、舵机和 MissionFSM 的联调方式。

本文档不替代现场飞手的安全判断。代码没有自动解锁，也不会代替操作者切换手动模式或执行急停。

## 1. 适用范围

全流程真机使用：

- `scripts/start_full_real_mission.sh`
- `drone_bringup/real_flight.launch.py`
- `src/drone_mission/config/mission_params_real.yaml`

`test_mission` 和 `scripts/test_forward_recon.sh` 是独立的分阶段测试入口。
它们可作为硬件通信和视觉降落的参考，但不会被全流程脚本隐式启动。

真机使用工作空间默认的 PX4 v1 消息环境。不要在同一个终端中加载 SITL 的 v4 `px4_msgs` overlay。

## 2. 任务行为

任务状态顺序如下：

```text
INIT → ARM → TAKEOFF → TRANSIT_TO_DROP_ZONE → CYLINDER_SEARCH
 → SELECT_TARGET → CYLINDER_APPROACH → DESCEND_TO_DROP
 → EXECUTE_DROP → ASCEND
 → TRANSIT_TO_RECON_ZONE → RECON_PATROL
 → RETURN_TO_HOME → PRECISION_LAND → COMPLETE
```

默认任务参数为：

| 项目 | 真机默认值 |
|---|---:|
| 起飞高度 | 5 m |
| 投放巡航高度 | 5 m |
| 投放区距离 | 32.5 m |
| 侦察区距离 | 57.5 m |
| 投放接近高度 | 3 m |
| 侦察高度 | 3 m |
| 返航高度 | 5 m |
| 位置设定点频率 | 20 Hz |

侦察区扫描完成后，FSM 从最后一个扫描航点直接进入返航。
返航航段同时向 Home 方向移动并回升至 5 m，不单独插入一次爬升任务。

精准降落先等待 H 标识和视觉对心稳定，再执行垂直盲降。
接地后等待 PX4 明确报告已解锁，随后才释放自主控制租约。

## 3. 硬件与 PX4 前置条件

起飞前由现场人员逐项确认：

- Jetson 与 PX4 以太网链路正常，PX4 uXRCE-DDS 参数指向 Jetson 的 UDP `8888` 端口。
- PX4 的 `UXRCE_DDS_CFG=1000`，真实状态话题为 `/fmu/out/vehicle_status_v1`。
- 相机设备存在且可读，默认路径为 `/dev/v4l/by-id/usb-D-VITEC_U3344W-video-index0`。
- 相机输出为 640×480，当前运行内参为 `fx=fy=400`、`cx=320`、`cy=240`。
- 舵机输出接在 PX4 MAIN5/Peripheral 或对应的 Actuator Set 1 通道。
- 舵机 PWM 方向已在无桨、地面安全状态确认：关闭、释放和中间位置符合现场机构。
- 遥控器可切出 Offboard，Arm 开关和手动接管开关均能正常工作。
- 电池、定位、飞控预检、地面范围和桨叶安全区域全部满足飞行条件。

舵机的 `ready=1` 只会在初始化 PWM 获得 PX4 命令确认后出现。
没有 `ready=1` 时不得解锁起飞。

## 4. 构建与启动

使用干净的真机终端，不要预先加载 SITL 环境（若已加载，脚本会自动剥离 v4 overlay）：

```bash
cd /home/ryder/drone_tasks_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如修改过消息接口、切换过分支或更换过机器，先按部署文档执行全量清理后重建。

启动唯一的全流程真机入口：

```bash
cd /home/ryder/drone_tasks_ws
./scripts/start_full_real_mission.sh
```

脚本会创建 `log/real_mission_YYYYMMDD_HHMMSS/`，先清理旧进程并启动唯一
`MicroXRCEAgent`（UDP `8888`），等待 PX4 DDS 就绪后再由 launch 启动任务节点。
不要在另一终端再启动同端口 Agent；Agent 意外退出时脚本会自动停止任务。

如需覆盖相机设备，只覆盖已确认存在的设备路径：

```bash
./scripts/start_full_real_mission.sh \
  camera_device:=/dev/video0
```

脚本不会自动 Arm、切换飞行模式、调用投放服务、执行降落或发送额外的飞行命令。

### 4.1 开机自启

自启直接运行全流程脚本（与 test_mission 相同的启动逻辑：清理 → Agent → 等 DDS
→ 任务节点），Jetson 与飞控同时上电时开机即可完成 DDS 连接。

```bash
# systemd Type=simple:
ExecStart=/home/ryder/drone_tasks_ws/scripts/start_full_real_mission.sh
# 或 cron @reboot:
@reboot /home/ryder/drone_tasks_ws/scripts/start_full_real_mission.sh
```

同端口 Agent 唯一性由脚本 Step 0 清理保证（自启与现场终端并发时，后启动者拥有
端口）。

若飞控晚于 Jetson 上电超过 60s（脚本的 DDS 等待上限），可改用 `--agent-only`
模式：仅持有 Agent、无限重试连接、不启动任务，任务由飞控就绪后的现场终端启动。

## 5. 就绪检查与解锁

保持启动终端运行，在第二个干净终端执行只读检查：

```bash
cd /home/ryder/drone_tasks_ws
./scripts/verify_real_flight_ready.sh
```

必须看到最后一行：

```text
READY: 真机 v1 遥测、相机、视觉与舵机 ACK 初始化检查通过。
```

检查内容包括：

- PX4 状态、里程计、电池和接地检测均有新样本；
- `/fmu/out/vehicle_command_ack` 存在发布者；
- CameraInfo 为 640×480，内参为当前真机配置；
- `/drone/vision/status` 为 `ready=1`；
- `/drone/servo/status` 为 `ready=1`；
- 投放服务存在，但检查脚本不会调用服务。

只有在检查通过、现场区域再次确认安全后，操作者才允许通过遥控器 Arm。
代码会先持续发送安全当前位置保持设定点至少 1 秒，再请求 Offboard；确认飞控进入 Offboard 后才发送起飞目标。

## 6. 飞行中应观察的现象

起飞后，正常位置控制设定点会持续发送。任务状态会依次经过投放搜索、目标接近、下降投放、爬升和第二次投放。

视觉识别只在需要的状态中参与决策：圆筒目标用于投放定位，覆盖监控用于侦察完成判定，H 标识和对心用于精准降落。

侦察区扫描完成后，最后一个航点不会停留等待独立返航指令。
系统会从该点直接向 Home 飞行，同时将目标高度回升到 5 m。

降落阶段不得把暂时丢失的单帧视觉结果当成已接地。
最终完成条件是 PX4 `vehicle_land_detected` 接地并确认 `armed=false`。

## 7. 故障与人工接管

任务采用有限重试策略：同一动作优先进行一次重试；仍无法完成时进入 B 方案，即保持零速度的 Offboard 悬停，并等待操作者接管。

任务级故障不会自动调用 RTL 或 LAND。
这保留现场人员对飞机位置、障碍物和风况的判断，也避免在视觉任务中强行切换到不符合现场的自动模式。

发现飞机姿态、位置、通信、动力或障碍物异常时，立即使用遥控器切出 Offboard 并接管。
接管后再停止上位机进程；不要用终端杀进程代替空中的人工控制。

若飞机已接地但任务仍未结束，先确认 PX4 已报告 `armed=false`，再检查日志。
在确认解锁之前，MissionFSM 会继续保持零速度设定点，不会释放控制租约后再盲目继续下降。

## 8. 日志与停止

每次启动的日志目录打印在启动终端中，主要文件为：

| 文件 | 用途 |
|---|---|
| `mission.log` | launch、FSM、视觉和舵机合并输出 |
| `launch_arguments.txt` | 本次启动参数 |
| `ros/` | ROS 2 节点独立日志 |

正常完成后，任务会在确认最终解锁后停止自主输出并结束相关控制流程。

异常飞行中，先由操作者完成遥控器接管和安全降落，再停止 launch 终端。
停止后保留本次完整日志，不要覆盖下一次飞行的日志目录。

## 9. 飞行前一页检查单

```text
[ ] 使用干净终端，未加载 SITL v4 overlay
[ ] PX4 uXRCE-DDS、Jetson IP、UDP 8888 已确认
[ ] 相机设备可读，640×480 和方向正确
[ ] 舵机 MAIN5/PWM 方向已在地面确认
[ ] 遥控器 Arm、手动接管和急停有效
[ ] 电池、定位、预检和场地安全通过
[ ] start_full_real_mission.sh 已启动且仅有一个 Agent
[ ] verify_real_flight_ready.sh 输出 READY
[ ] 现场人员明确：READY 后才 Arm，异常立即切手动
```

本指南只记录当前全流程真机实现。若修改真实相机、PX4 通道、任务距离或 Home，必须同步重新执行硬件检查和就绪验证。
