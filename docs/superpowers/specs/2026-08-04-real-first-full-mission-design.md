# 真机优先完整任务设计

**状态：已确认，待书面复核**

## 1. 目标与范围

目标是在真实飞行器上通过唯一完整任务入口完成：两次确认投放、侦察区完整扫描、从最后侦察航点直接返航并同步升至 5 m、H 标识视觉微调降落和接地确认。

完整真机入口固定为：

```text
scripts/start_full_real_mission.sh
  -> drone_bringup/real_flight.launch.py
  -> drone_mission/MissionFSM
```

真机成功的含义是完整满足上述任务条件。SITL 只用于任务语义回归和发现代码缺陷；真机的速度、视觉控制增益、容差、恢复窗口和实际航迹不要求与 SITL 相同。

`test_mission` 已完成实飞验证。从本规格开始它不再修改：不改其状态机、视觉微调、参数、返航高度或启动脚本。它继续作为独立的已验证测试流程，但不是完整真机任务的实现目标。

相机和视觉节点保持已实飞验证的既有实现与参数路径，不新建标定文件、运行时标定契约或 SITL 相机桥。完整任务仅使用该既有真机基线（640×480，`fx=fy=400`，`cx=320`，`cy=240`）；这组值同时写入完整任务真机参数文件，确保 `MissionFSM` 的视觉微调与启动的相机/视觉节点一致。

## 2. 部署边界

真机是权威部署：

| 用途 | 真机默认 | SITL 专用 |
|---|---|---|
| `px4_msgs` | v1 | 隔离的 v4 overlay |
| 飞行状态 | `/fmu/out/vehicle_status_v1` | `/fmu/out/vehicle_status_v4` |
| 电池 | `/fmu/out/battery_status_v1` | SITL 对应消息 |
| 舵机 ACK | `/fmu/out/vehicle_command_ack` | 舵机 mock，不改变真机 ACK |
| 应用话题 | `/drone/*` | 与真机保持同名 |

源码和真机 launch 的默认值必须保持真机 v1。只有 `scripts/sitl_env.sh` 及 SITL launch 可以加载 v4 并覆盖状态话题。不得为了通过 SITL 回归修改真机默认类型、话题或启动脚本。

## 3. 唯一完整真机任务路径

### 3.1 参数与启动

新增 `src/drone_mission/config/mission_params_real.yaml`，承载完整任务的真机专用速度、容差、视觉微调和恢复参数。`real_flight.launch.py` 必须加载该文件；SITL 保持自己的参数文件和启动流程。

完整任务以已实飞成功的 `scripts/test_forward_recon.sh` 所传真机值作为所有同义参数的唯一初始基线。`mission_params_real.yaml` 必须明确记录并由 `MissionFSM` 直接消费下表值：

| 参数组 | 完整真机初值 | 来源 |
|---|---:|---|
| 起飞/巡航高度 | 5.0 m | `cruise_altitude=5.0` |
| 航点/起飞容差 | 0.5 m / 0.3 m | 已实飞测试默认 |
| 控制频率 | 20 Hz | 已实飞测试默认 |
| 中转速度 | 2.0 m/s | 实飞测试脚本 `transit_speed=2.0` |
| 侦察高度 | 3.0 m | 实飞测试脚本 `recon_altitude=3.0` |
| 扫描速度/间距 | 0.8 m/s / 1.67 m | 实飞测试脚本 |
| 返航速度 | 3.8 m/s | 已实飞测试默认 |
| H 伺服 | `kp=0.004`、最大 0.6 m/s、死区 12 px、右移/前移符号 `+1/-1` | 已实飞测试默认 |
| H 分层下降 | 0.4 / 0.25 / 0.12 m/s | 已实飞测试默认 |
| 视觉丢失窗口 | 3.0 s | 已实飞测试默认 |
| 真机相机 | 640×480，`fx=fy=400`，`cx=320`，`cy=240` | 已实飞测试相机基线 |

完整任务专属的投放区/侦察区边界、两次投放参数、圆筒搜索高度和投放精校参数保留为完整任务配置；它们没有 `test_mission` 的同义项。完整任务返航高度固定为 5.0 m，这是已确认的完整任务要求，不采用 `test_mission` 当前旧流程的 4.0 m。

这些值是完整 `MissionFSM` 的参数，不通过导入、共享状态或调用 `test_mission` 实现来复用。未来只允许修改 `mission_params_real.yaml` 中的真机值并以实飞日志说明原因；真机 launch 不得以第二套数值覆盖该文件，Python 参数声明只作为 ROS 类型/缺省兼容，不能取代已加载的真机配置。

`real_flight.launch.py` 必须：

- 保持既有相机节点和视觉节点的 640×480、`400/400/320/240` 参数，不修改相机或视觉包；
- 加载 `mission_params_real.yaml`，并且不再为 `MissionFSM` 额外覆盖任务运动、视觉微调或相机几何参数；
- 只将部署拓扑参数（相机设备、DDS 端口、Home、飞行轴、投放/侦察距离、RC 解锁要求）作为 launch 参数传入；所有 `MissionFSM` 运动、视觉和恢复数值只由 `mission_params_real.yaml` 提供；
- 继续由 launch 内的 `MicroXRCEAgent` 管理真机 DDS，防止启动脚本额外启动同端口 Agent。

新增 `scripts/start_full_real_mission.sh`。它只做下列工作：

1. 定位工作区，加载 `/opt/ros/humble/setup.bash` 与 `install/setup.bash`；拒绝使用 SITL 环境。
2. 为一次执行创建 `log/real_mission_YYYYMMDD_HHMMSS/`，设置 ROS 日志目录，并保存启动参数和控制台输出。
3. 检查 `ros2`、`MicroXRCEAgent`、工作区安装文件、`real_flight.launch.py` 和配置的相机设备路径；失败时退出并给出可操作诊断。
4. 打印真机 v1 话题、相机设备、Home、投放和侦察距离、人工 RC 解锁及接管说明。
5. 通过 `ros2 launch drone_bringup real_flight.launch.py "$@"` 启动唯一完整真机任务。

脚本不得执行 `pkill`，不得加载 `sitl_env.sh`，不得自动 Arm、切换 Offboard、发布位置或速度指令、调用投放服务、切换 LAND/RTL 或驱动舵机。人工 RC 解锁和接管始终由操作者完成。

### 3.2 正常任务状态语义

完整 `MissionFSM` 的唯一成功路径为：

```text
INIT (真机遥测、相机、视觉、舵机就绪)
  -> WAIT_ARMED (人工 RC 解锁)
  -> TAKEOFF 5 m
  -> TRANSIT_TO_DROP_ZONE
  -> CYLINDER_SEARCH / SELECT / APPROACH / DESCEND / DROP (两次)
  -> TRANSIT_TO_RECON_ZONE
  -> RECON_PATROL (完整弓字形路径)
  -> RETURN (从最后侦察航点向 Home 同时爬升至 5 m)
  -> PRECISION_LAND (H 搜索、微调、垂直下降)
  -> COMPLETE
```

侦察区的每个计划航点都是必需工作，不能因状态时长或普通超时跳过。返航状态在一次三维 `goto_ned()` 中发送 Home 的 North/East 与 `takeoff_height=5.0`，不得增加“先原地爬升”的独立状态。

既有视觉节点是完整任务的唯一视觉来源。它继续使用当前已实飞的相机参数与图像流，输出圆筒、标记、覆盖状态和 H 降落目标。完整任务使用这些输出及真机参数文件中的阈值做投放定位和 H 微调；不改动相机驱动、视觉处理或 SITL 相机桥。

### 3.3 最小恢复与真值摘要

正常持续进展的动作不会因固定总时长被中止。系统只监测必要条件：真机遥测/视觉新鲜度、位置或视觉误差的无进展窗口、舵机 ACK/服务结果。

- 航点或高度无进展：重发当前同一目标一次；
- 视觉短暂丢失：在当前阶段重捕获一次；
- 两个投放目标不足：重复同一搜索路径一次；
- 舵机 ACK 超时：重发同一个绝对 PWM 一次；
- H 识别在已开始对心后丢失：清空本次视觉锁定、重捕获一次；
- 同一动作的恢复耗尽：进入 `HOLD_FOR_TAKEOVER`，持续发布零速度 Offboard；RC 接管、解锁状态变化或 PX4 离开 Offboard 后停止自主输出。

任务级恢复不得自动 RTL 或 LAND，不得跳过侦察航点、投放次数或接地确认。PX4 原生电池、RC 丢失和通信 failsafe 保留；一旦 PX4 不再处于 Offboard，任务不与其竞争。

降落以 PX4 `vehicle_land_detected.landed` 为主。仅在已经完成视觉对心、已进入垂直盲降，且近地高度、水平/垂直速度连续满足严格阈值时，才允许备用接地确认。未确认接地、等待接管、未完成两次投放或未完成侦察时，摘要必须为 `success=false`。

## 4. 验证和部署顺序

1. 单元和接口测试验证：真机 v1 默认、既有 640×480 相机参数与 `MissionFSM` 参数文件一致、启动脚本无危险命令、完整 FSM 的航点/投放/恢复/成功摘要契约。
2. SITL 只回归完整任务结果：两次确认投放、完整扫描、5 m 同步返航、H 对心、接地确认和 `success=true`。SITL 失败必须先有可复现测试；其调参不得覆盖真机参数文件。
3. 真机地面检查：桨叶拆除时启动完整入口，确认 v1 DDS、相机 `CameraInfo`、视觉 ready、舵机 ACK 和投放服务；启动检查只读，不发飞行或舵机命令。
4. 真机分阶段验证：视觉目标检测、单次投放、两次投放、完整任务。每次保存启动、Agent、MissionFSM、视觉和 PX4 日志。

只有日志同时证明两次投放、全部侦察航点、从最后航点开始的 Home+5 m 三维返航、H 降落、接地确认和真值成功摘要时，才标记为完整真机任务成功。

## 5. 非目标

- 不修改 `test_mission`。
- 不要求真机和 SITL 的航迹、速度、视觉控制参数或内部恢复过程逐项一致。
- 不建立第二套完整真机状态机或新的 `full_real_mission` 节点。
- 不修改相机驱动、视觉处理、相机标定文件或 SITL 相机桥。
- 不在启动脚本中实施无人监管的解锁、模式切换、投放、降落、RTL 或进程清理。
