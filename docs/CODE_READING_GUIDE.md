# 代码导读与学习路径

这份文档回答两个问题：每个目录里的代码负责什么，以及一次任务从传感器输入到飞控输出是怎样经过这些文件的。建议先按“接口和调用链”阅读，再深入单个算法。

## 1. 先建立整体心智模型

系统不是一个脚本，而是几个通过 ROS 2 话题、服务和 PX4 DDS 连接的模块。每个模块都有自己的接口和实现：

```text
相机/遥测输入
      ↓
drone_vision ── 检测、坐标换算、覆盖、视觉伺服
      ↓ 视觉结果/速度指令
drone_mission ── FSM 决定“现在做什么、何时切换状态”
      ↓ NED 航点/速度、投放服务
px4_control ── 把 ROS 指令转换成 PX4 DDS
      ↓                         ↓
PX4 飞控                    drone_servo2 舵机投放
```

关键理解：`drone_mission` 不直接处理图像，也不直接操作 PWM；`drone_vision` 不决定任务状态；`px4_control` 不决定任务策略。模块之间的接口就是系统的 seam，排查问题时先确认接口两端是否都在工作。

## 2. 从启动入口开始

### 2.1 SITL 启动链

按下面顺序阅读：

1. `scripts/start_full_simulation.sh`：准备场景、清理旧运行、启动 PX4、Gazebo 图像桥和 ROS 2 launch，并建立日志目录。
2. `scripts/sitl_env.sh`：加载 ROS 2 和工作区，再把隔离构建的 v4 `px4_msgs` overlay 放到环境前端。
3. `src/drone_bringup/launch/sitl.launch.py`：组合 `px4_bridge`、`vision_pipeline`、`servo_controller` 和 `mission_fsm`，并覆盖 SITL 的 Home、方向和 v4 状态话题。
4. `src/drone_bringup/scripts/gz_camera_bridge.py`：把 Gazebo 图像转换成 `/drone/camera/image_raw` 和相机信息。

### 2.2 真机启动链

1. `src/drone_bringup/launch/real_flight.launch.py`：启动 MicroXRCEAgent、V4L2 相机、PX4 bridge、视觉、舵机和完整任务 FSM。
2. `src/drone_vision/drone_vision/camera.py`：读取 V4L2 设备并发布标准 ROS 图像话题。
3. PX4 通过以太网 UDP 向 Jetson 的 `MicroXRCEAgent udp4 -p 8888` 建立 DDS；相机则独立通过 USB 接入 Jetson。
4. 真机使用工作区默认 v1 `px4_msgs` 和 `/fmu/out/vehicle_status_v1`；任务逻辑仍与 SITL 共用。

## 3. 按模块阅读文件

### 3.1 `drone_interfaces`：先看协议

阅读顺序：

1. `msg/NedPositionSetpoint.msg`：任务位置航段如何原子传递 NED 目标、yaw 和可选速度给 `px4_bridge`。
2. `msg/CylinderDetection.msg`、`CylinderArray.msg`：投放目标怎样表达。
3. `msg/LandingTarget.msg`：H 目标中心、世界坐标和置信度怎样表达。
4. `msg/CoverageStatus.msg`：覆盖率怎样从视觉传给任务。
5. `srv/SetVisionMode.srv`：FSM 如何切换视觉模式。
6. `srv/TriggerDrop.srv`：FSM 如何请求一次投放。
7. `action/PrecisionLand.action`：历史/独立精准降落接口；当前完整 FSM 主要通过话题和状态类完成降落。

阅读目标不是记住消息字段，而是掌握生产者、消费者、时间新鲜度和失败含义。比如 `LandingTarget.confidence` 只是视觉输出，是否允许盲降由 `state_precision_land.py` 决定。

### 3.2 `px4_control`：理解飞控适配 seam

推荐顺序：

1. `telemetry.py`：先理解 NED、World、Home 和位置/速度数据结构。
2. `flight_controller.py`：看任务状态使用的高层动作，例如 arm、takeoff、goto、set velocity、force disarm。
3. `standalone_bridge.py`：看任务 FSM 如何通过 `/drone/cmd_*` relay 话题发送指令，并接收 PX4 遥测；位置航段使用 `NedPositionSetpoint`，视觉精校/降落仍使用独立速度话题。
4. `px4_bridge.py`：看 relay 话题如何转换成 `/fmu/in/*`，以及 PX4 输出如何变成 `/drone/pose`、状态和接地信号。

这里最容易误解的是：任务 FSM 和 `px4_bridge` 是不同 ROS 2 进程。FSM 通常使用 `StandaloneBridge`，`px4_bridge` 再把 ROS 命令转发给 PX4；不要把两者当作一个类调用链。

### 3.3 `drone_vision`：从管线入口向内读

推荐顺序：

1. `vision_pipeline.py`：先看订阅、发布、模式服务和 `_image_cb` 主循环。
2. `models/yolo_engine.py`：看 ONNX 模型解析、provider 选择、推理和资源关闭。
3. `cylinder_detector.py`、`marker_detector.py`：看投放目标和蓝色标识的检测/精修。
4. `landing_detector.py`：看 H 标识精修和 `LandingTarget` 生成。
5. `coordinate_transforms.py`：看像素、相机、机体、NED 和 World 的变换假设。
6. `visual_servoing.py`：看像素误差如何转换成 `/drone/vision/servo_cmd`。
7. `coverage_monitor.py`：看 FOV 网格、扫描生命周期和 `/drone/vision/coverage`。
8. `camera.py`：最后阅读相机输入，因为它是输入 Adapter，不决定检测策略。

视觉管线的主线是：

```text
/drone/camera/image_raw + /drone/pose
  → YOLO/ONNX
  → bbox 原图坐标
  → OpenCV 精修/置信度/跳变过滤
  → CylinderArray / LandingTarget / CoverageStatus
  → 视觉伺服速度
```

进入侦察阶段后由模式服务停止不需要的投放检测并释放模型。阅读性能或显存问题时，应沿 `set_mode → _release_detection_models → engine.close()` 跟踪，而不是只看模型加载函数。

### 3.4 `drone_mission`：从状态机向状态实现展开

先读 `mission_state_machine.py` 的四部分：

1. `MissionState` 和 `STATE_TRANSITIONS`：允许哪些状态以及默认顺序。
2. `_init_vision_subscriptions`：FSM 接收哪些视觉、遥测和状态接口。
3. `_control_loop_impl`：每个 20 Hz 周期如何执行当前状态、处理紧急事件和完成状态转移。
4. `set_vision_mode`、`bridge`、`fc` 等 Context 接口：状态类通过哪些 seam 使用外部能力。

然后按任务顺序阅读 `states/`：

| 阶段 | 文件 | 重点 |
|---|---|---|
| 初始化/起飞 | `state_init.py`、`state_takeoff.py` | 等待遥测、视觉和舵机 ready，完成 Offboard 起飞 |
| 区域中转 | `state_transit.py` | NED 航点、速度、容差和超时 |
| 投放搜索 | `state_cylinder_search.py` | 目标记忆、覆盖搜索和搜索结束条件 |
| 接近/投放 | `state_cylinder_approach.py`、`state_drop.py` | 目标追踪、低空精校、服务确认和两次投放分支 |
| 侦察 | `state_recon_patrol.py` | coverage reset、弓字形路径和完整路径条件 |
| 返航 | `state_return.py` | 回到 Home 并交给降落状态 |
| 降落 | `state_precision_land.py` | H 对心、锁存、盲降、接地和错误处理 |

支撑策略文件单独阅读：

- `cylinder_selector.py`：候选目标排序和已投目标排除；
- `path_planner.py`：区域航线和弓字形航点；
- `precision_align_controller.py`：投放区像素闭环精校；
- `drop_visual_aligner.py`：旧的梯度探测式对齐实现，主要作为回退/独立实现理解；
- `simple_mission.py`：简化 smoke test，不是完整任务 FSM。

## 4. 一次完整任务的调用链

### 4.1 起飞

`state_init.py` 等待 `px4_control` 的遥测、视觉状态和舵机状态 → `state_takeoff.py` 调用 `fsm.fc` → `StandaloneBridge` 发布 `/drone/cmd_setpoint_ned` 或车辆命令 → `px4_bridge.py` 发布 PX4 Offboard/DDS → PX4 更新 `/fmu/out/*` → 状态回调确认高度和模式。

### 4.2 一次投放

`state_cylinder_search.py` 接收 `/drone/vision/cylinders` → FSM 记忆世界坐标 → `cylinder_selector.py` 选择 ID → `state_cylinder_approach.py` 发送 NED 速度 → `state_drop.py` 调用 `/drone/vision/set_mode` 和精校控制器 → 目标连续稳定后调用 `/drone/servo/trigger_drop` → `drone_servo2` 移动一个 PWM 位置并返回成功 → FSM 进入 ASCEND。

### 4.3 侦察

`state_recon_patrol.py` 调用 coverage reset → `vision_pipeline.py` 清空覆盖网格 → `path_planner.py` 给出弓字形航点 → `/drone/pose` 驱动 `CoverageMonitor` 更新网格 → `/drone/vision/coverage` 回到 FSM → 完整路径结束后返航。

### 4.4 视觉降落

返航到 Home → FSM 切换 landing 模式 → `vision_pipeline.py` 加载 `landing_h_yolo.onnx` 并发布 `LandingTarget` → `state_precision_land.py` 检查置信度、跳变、像素误差和稳定帧 → 对心锁存后下降到 1.5 m → 只发布垂直 Offboard 速度 → 真机等待 `vehicle_land_detected`，SITL 评估严格近地备用条件 → 零速度、force disarm、写入 `mission_end`。

## 5. 遇到问题时的定位顺序

按数据流从上游到下游排查，不要直接修改状态机参数：

1. **节点是否启动**：检查 launch、进程和 `/drone/vision/status`、`/drone/servo/status`。
2. **输入是否存在**：检查 `/drone/camera/image_raw`、`/drone/pose` 和 PX4 状态话题。
3. **接口类型是否匹配**：检查话题类型、v1/v4 消息环境和服务是否可用。
4. **数据是否新鲜**：查看视觉时间戳、目标置信度、目标跳变、覆盖更新时间。
5. **策略是否做出决定**：查看 FSM 当前状态和状态转移事件。
6. **输出是否到达执行器**：检查 `/drone/cmd_velocity_ned`、`/fmu/in/*`、舵机服务响应和 PX4 ACK。
7. **状态是否正确收尾**：检查接地信号、force disarm、视觉模型释放和 `mission_end`。

## 6. 建议的代码学习顺序

### 第一阶段：只读接口

阅读本文件、`docs/ARCHITECTURE.md`、全部 `drone_interfaces` 文件，以及每个 launch 文件。目标是能画出节点、话题和服务图，不深入算法。

### 第二阶段：跑通简化链路

阅读 `simple_mission.py`、`flight_controller.py` 和 `px4_bridge.py`，理解起飞、航点、返航和接地。然后阅读 `scripts/simple_mission.sh` 作为最小运行入口。

### 第三阶段：读完整 FSM

按 `mission_state_machine.py → state_init/takeoff/transit → cylinder_search → approach/drop → recon → return → precision_land` 的顺序阅读。每读一个状态，记录：进入条件、执行动作、成功转移、失败转移、超时和输出接口。

### 第四阶段：读视觉闭环

按 `vision_pipeline.py → yolo_engine.py → detector/refiner → coordinate_transforms.py → visual_servoing.py` 阅读。重点跟踪一帧图像如何变成目标坐标和速度，而不是先调阈值。

### 第五阶段：用日志反向验证

打开 [log/simulation_20260730_192807/ros2_system.log](../log/simulation_20260730_192807/ros2_system.log)，按 `[FSM]`、`投放成功`、`覆盖`、`盲降`、`mission_end` 搜索。把每条日志对应回状态类和接口，再用测试文件验证局部算法。

### 第六阶段：再看真机差异

最后阅读 `real_flight.launch.py`、`scripts/sitl_env.sh`、`standalone_bridge.py` 和 [REAL_HARDWARE_DEBUG_GUIDE.md](REAL_HARDWARE_DEBUG_GUIDE.md)，专门比较 USB 相机、PX4 Ethernet/UDP DDS、v1/v4 消息、MAIN5 和接地确认，不要把 SITL Adapter 的行为误认为真机实现。

## 7. 阅读时容易混淆的文件

| 文件/目录 | 当前定位 |
|---|---|
| `drone_servo2` | 当前完整任务使用的 C++ 舵机实现 |
| `drone_servo` | Python 旧实现/参考代码，不是完整任务主链路 |
| `simple_mission.py` | 简化任务 smoke test，不等于 `MissionFSM` |
| `state_recon_inspect.py` | 遗留的危险品检查状态，当前侦察主线使用 `state_recon_patrol.py` |
| `hazard_classifier.py` | 可独立运行的危险品分类模块，当前完整任务只做覆盖巡查 |
| `src/*/build`、`src/*/install`、根目录 `install` | 构建产物，不是源码阅读入口 |

## 8. 读懂一个模块的检查表

对任意模块，先回答以下问题：

1. 它的接口是什么，调用者必须提供哪些参数和配置？
2. 它发布/订阅哪些 ROS 2 话题，服务调用的成功和失败含义是什么？
3. 它是否隐藏了复杂实现，还是只是浅层转发？
4. 它的时间、坐标、线程和资源生命周期约束是什么？
5. 哪个测试跨过同一个接口验证它？
6. 如果替换一个 Adapter，任务逻辑是否仍然保持不变？

如果答不出这些问题，先回到接口和调用链，不要直接修改内部算法。
