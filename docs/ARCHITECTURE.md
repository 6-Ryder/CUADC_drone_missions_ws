# 当前系统架构

## 工作空间总览：`CUADC_drone_missions_ws`

这是一个**基于 ROS 2 Humble + PX4 Offboard + Gazebo SITL 的无人机自主任务系统**，部署在 Jetson Orin Nano 真机上，SITL 仿真在 x86 开发机上。

### 核心任务

无人机自主执行**全流程任务**：
1. **起飞** → 飞行至投放区 → 搜索圆筒目标 → **两次精准投放**（舵机 PWM 序列 `1150→1700→2150 μs`）
2. 飞至侦察区 → **弓字形全覆盖扫描**（相机 FOV 覆盖网格）
3. 返航 → **H 标识视觉伺服** → 对心后近地盲降 → 接地并强制解锁

### 已跑通状态（2026-07-30 SITL 全流程）

| 指标 | 结果 |
|---|---|
| 两次投放 | ✅ 成功 |
| 侦察覆盖率 | ✅ 100% |
| H 标识对心盲降 | ✅ 1.5m 切入，接地确认 |
| 任务完成 | ✅ `mission_end.success=True, ~200s` |

### 源码包（10 个包，`src/`）

```
drone_interfaces  ← ROS 2 自定义 msg/srv/action (接口协议)
    ├── msg: NedPositionSetpoint, CylinderArray, LandingTarget, CoverageStatus...
    └── srv: SetVisionMode, TriggerDrop

px4_control       ← PX4 DDS 适配层
    ├── px4_bridge.py        — PX4 ↔ ROS2 DDS 桥 (发布 /fmu/in/*, 中转 /drone/cmd/*)
    ├── standalone_bridge.py — FSM 不直连 DDS 时的轻量转发器 (同一接口)
    ├── flight_controller.py — 高层飞控动作 (arm/takeoff/goto/velocity/disarm)
    └── telemetry.py         — NED/World 坐标、位置/速度/姿态模型

drone_vision      ← 统一视觉管线 v2
    ├── vision_pipeline.py   — 主节点: 订阅 /drone/camera/image_raw, 运行 YOLO/ONNX
    ├── models/yolo_engine.py — ONNX Runtime 推理, provider 选择, 模型生命周期
    ├── cylinder_detector.py  — 圆筒 YOLO + OpenCV 精修 (边缘、白色、半径)
    ├── landing_detector.py   — H 标识检测 + OpenCV 精修
    ├── marker_detector.py    — 蓝色投放标识精修 (蓝色占比、圆度、直径过滤)
    ├── coordinate_transforms.py — pixel_to_world (6-DoF 姿态补偿 + simple fallback)
    ├── visual_servoing.py    — 像素误差 → NED 速度 (PI-D 控制器)
    ├── coverage_monitor.py   — 相机 FOV 网格覆盖 → /drone/vision/coverage
    └── camera.py             — V4L2 相机驱动 (真机)

drone_servo2      ← C++ 多位置舵机投放（当前主用）
    ├── servo_controller.cpp  — 订阅 TriggerDrop, 步进 PWM 位置序列
    └── SITL Mock / Jetson PX4 DO_SET_ACTUATOR → MAIN5

drone_servo       ← Python 旧舵机实现 (参考/独立, 不在主链路)

drone_mission     ← 核心任务调度
    ├── mission_state_machine.py — 20Hz FSM 主循环, 参数声明, 服务/话题桥接
    ├── states/
    │   ├── state_init.py, state_takeoff.py   — 初始化, 起飞
    │   ├── state_transit.py                  — 区域中转 (NED 航点 + 速度)
    │   ├── state_cylinder_search.py          — 投放区搜索 (目标记忆/去重)
    │   ├── state_drop.py                     — DESCEND→EXECUTE→ASCEND 三阶段
    │   ├── state_recon_patrol.py             — 弓字形全覆盖扫描
    │   ├── state_return.py                   — 返航
    │   ├── state_precision_land.py           — H 搜索→视觉伺服→盲降→接地
    │   └── state_hold_for_takeover.py        — 异常接管悬停
    ├── path_planner.py           — 弓字形航点生成
    ├── cylinder_selector.py      — 已投排除 + min_radius_first 策略
    ├── precision_align_controller.py — CUADC 风格像素闭环精校 (替代旧梯度探测)
    └── simple_mission.py         — Smoke test (非完整 FSM)

drone_bringup     ← Launch 文件
    ├── sitl.launch.py       — SITL: px4_bridge + vision + servo + mission
    └── real_flight.launch.py — 真机: +V4L2 camera + Jetson 参数 (Agent 由启动脚本持有)

test_mission      ← 分阶段真机测试 (test_forward_recon: 直飞20m+侦察扫描)
drone_utils       ← 共享工具 (TaskLogger, Watchdog, EmergencyHandler, ProgressMonitor)
px4_msgs          ← PX4 DDS 消息定义 (v1/v4 隔离)
```

### 架构数据流

```
相机 (Gazebo/V4L2) ──Image──► drone_vision ──检测结果──► drone_mission (FSM)
                                  │                          │
                          视觉伺服速度               NED 航点/速度/投放
                                  │                          │
                          px4_control ◄──────────────────────┘
                                  │
                          /fmu/in/* (PX4 DDS)
                                  │
                               PX4 飞控
                                  │
                          drone_servo2 ←── TriggerDrop 服务
```

### 坐标系统

| 坐标系 | 方向 | 用途 |
|---|---|---|
| **NED** | x=North, y=East, z=Down | PX4 飞控指令与遥测 |
| **World (ENU)** | x=East, y=North, z=Up | Gazebo/任务逻辑内部 |
| 转换 | `World x = home_x + NED east`, `World y = home_y + NED north` | `telemetry.py` |

### 关键设计决策

1. **精准降落只走视觉 H 对心**，视觉丢失不 fallback 到 PX4 LAND
2. **FSM 与 PX4 在不同进程**：FSM → StandaloneBridge → ROS relay 话题 → px4_bridge → PX4 DDS
3. **v1/v4 消息隔离**：真机 v1，SITL v4；`sitl_env.sh` 控制 overlay 环境
4. **视觉模型按需加载**：投放模型在进入侦察时释放，降落模型按需创建
5. **舵机步进投放**：每次只前进一步 PWM 位置，不跳位
6. **遥控器安全**：真机 `rc_require_arm_signal=true`，必须人工 Arm Switch 解锁

### 辅助资源

| 目录 | 内容 |
|---|---|
| `docs/` | ARCHITECTURE, CODE_READING_GUIDE, DEPLOYMENT, RTK 指南, WORKLOG |
| `scripts/` | start_full_simulation.sh, start_full_real_mission.sh, sitl_env.sh, debug_vision.sh |
| `field/` | Gazebo 场景 (drone_field.sdf, 70m×8m 北向) |
| `real_flight_logs/` | 真机飞行日志 (按日期) + 飞控参数修改记录 |
| `log/` | SITL 仿真日志 (209 个子目录) |
| `controller_params_v3.params` | 37KB PX4 控制器参数文件 |

---

本文档描述仓库当前实现，不描述已经废弃的方案。代码事实优先级高于本文档；参数以各包 `config/*.yaml` 为准。

## 1. 设计不变量

1. 任务由 `drone_mission` 的单一 FSM 编排，飞控、视觉和舵机通过 ROS 2 接口协作。
2. 飞行指令使用 PX4 NED：`x=north`、`y=east`、`z=down`；向上高度对应负 z。
3. 当前场地沿北向布置，SITL Home World 为 `(4,10,0)`，真机 Home 为解锁点 `(0,0,0)`。
4. 精准降落必须使用视觉 H 对心；视觉丢失或盲降超时进入错误处理，不自动改用 PX4 LAND。
5. 任务结束停止 FSM 控制定时器；视觉在进入侦察阶段释放投放模型，避免无用显存和句柄继续占用。

## 2. 包职责

| 包 | 职责 | 主要实现 |
|---|---|---|
| `drone_interfaces` | ROS 2 自定义接口 | `NedPositionSetpoint`、`CylinderArray`、`LandingTarget`、`CoverageStatus`、`SetVisionMode`、`TriggerDrop` |
| `px4_control` | PX4 DDS 适配和指令输出 | `px4_bridge.py`、`standalone_bridge.py`、`flight_controller.py` |
| `drone_vision` | 相机、ONNX 推理、目标精修、覆盖监控、视觉伺服 | `vision_pipeline.py` 及检测器/控制器 |
| `drone_servo2` | 多位置投放控制 | C++ `servo_controller.cpp`，SITL Mock 或 Jetson PX4 执行器 |
| `drone_mission` | 任务状态机和策略 | `mission_state_machine.py`、`states/`、`path_planner.py` |
| `drone_bringup` | 启动组合 | `sitl.launch.py`、`real_flight.launch.py` |
| `test_mission` | 分阶段真机测试 | `test_forward_recon` 等测试节点 |

## 3. 运行拓扑

```text
Gazebo 相机 / V4L2 相机
          │ Image
          ▼
   drone_vision
  ├─ cylinders ───────────────┐
  ├─ markers                   │
  ├─ landing_target            │
  ├─ coverage                  │
  ├─ servo_cmd ──► px4_control│
  └─ set_mode ◄───────────────┘
                                ▼
                         drone_mission FSM
                          │          │
          TriggerDrop ◄───┘          └── NED setpoint/velocity
                │                              │
          drone_servo2                    px4_control
                │                              │
             MAIN5/PWM                         PX4
```

运行时由 launch 启动四类节点：PX4 bridge、视觉管线、舵机控制器、任务 FSM。真机由启动脚本 `scripts/start_full_real_mission.sh` 先启动唯一 Ethernet/UDP MicroXRCEAgent 并等待 DDS 就绪，launch 再启动 USB V4L2 相机等节点；SITL 额外启动 Gazebo 图像桥。

## 4. 坐标与场地

### 4.1 坐标链

PX4 遥测和指令采用 NED。Gazebo World 使用 `x=east`、`y=north`。任务通过 `telemetry.py` 的转换和 `home_x/home_y` 将两者关联：

```text
World x = home_x + NED east
World y = home_y + NED north
```

SITL 默认 Home 为 `(4,10)`。因此侦察区 NED `north=55..60, east=-4..4` 对应 World `x=0..8, y=65..70`；投放区 NED `north=30..35, east=-4..4` 对应 World `x=0..8, y=40..45`。

### 4.2 关键区域

| 区域 | NED 范围 | 高度 |
|---|---|---|
| 投放区 | north `30..35`，east `-4..4` | 搜索 5 m，投放低空 2.8 m 精校后降至 1.7 m |
| 侦察区 | north `55..60`，east `-4..4` | 2.5 m |
| H 起降点 | Home | 搜索 5 m，稳定对心后 1.5 m 盲降 |

## 5. 任务 FSM

状态定义在 `src/drone_mission/drone_mission/mission_state_machine.py`，状态行为在 `states/` 中拆分：

```text
INIT → ARM → TAKEOFF → TRANSIT_TO_DROP_ZONE → CYLINDER_SEARCH
 → SELECT_TARGET → CYLINDER_APPROACH → DESCEND_TO_DROP
 → EXECUTE_DROP → ASCEND
        ├─ drops < max_drops → SELECT_TARGET
        └─ drops complete → TRANSIT_TO_RECON_ZONE
 → RECON_PATROL → RETURN_TO_HOME → PRECISION_LAND → COMPLETE
```

任一状态异常、遥控器在飞行中解锁丢失、严重电量事件或关键视觉超时都会进入 `ERROR`，随后停止任务并写入失败摘要。`COMPLETE` 和 `ERROR` 都会写 `mission_end` 事件。

## 6. 视觉管线

`vision_pipeline` 是独立 ROS 2 节点，加载 ONNX 模型并按 FSM 的服务请求切换检测模式：

| 模式值 | 名称 | 作用 |
|---:|---|---|
| 0 | none | 停止检测/伺服输出 |
| 2 | landing | H 标识检测和降落伺服 |
| 3 | all | 圆筒、投放标识和可用检测 |
| 4 | coverage_reset | 重置覆盖网格 |
| 5 | coverage | 更新侦察覆盖 |

核心输出：

| 话题 | 类型 | 消费者 |
|---|---|---|
| `/drone/vision/cylinders` | `CylinderArray` | 投放搜索、目标记忆和选择 |
| `/drone/vision/markers` | `CylinderArray` | 投放标识兜底 |
| `/drone/vision/landing_target` | `LandingTarget` | 精准降落状态 |
| `/drone/vision/coverage` | `CoverageStatus` | 侦察状态和任务摘要 |
| `/drone/vision/servo_cmd` | `TwistStamped` | `px4_control` 速度转发 |
| `/drone/vision/status` | `String` | FSM 看门狗和就绪判断 |

视觉处理包含 YOLO/ONNX 检测、原图坐标恢复、OpenCV 精修、置信度和跳变过滤。投放模型在进入侦察阶段通过 `close()`、解除引用和垃圾回收释放；降落模型按需加载。

## 7. 两次投放

1. `CYLINDER_SEARCH` 在搜索高度收集目标，按世界坐标合并观测。
2. `SELECT_TARGET` 使用 `CylinderSelector` 避免已投目标，并按 `min_radius_first` 选择策略排序。
3. `CYLINDER_APPROACH` 以目标记忆和最新检测进行中转、慢速接近。
4. `DESCEND_TO_DROP` 在低空启用视觉精校：EMA 平滑像素误差、速度脉冲、短暂停稳、稳定帧和最终刹车复查。
5. `EXECUTE_DROP` 调用 `/drone/servo/trigger_drop`。舵机位置序列为 `1150 → 1700 → 2150 μs`，每次只前进一个位置。
6. 服务成功且确认时间满足后进入 `ASCEND`。达到 `max_drops=2` 才转入侦察。

## 8. 侦察覆盖

`RECON_PATROL` 启用 `coverage_reset` 后切换到 `coverage`，按 `PathPlanner` 生成的弓字形航线执行完整路径。`CoverageMonitor` 按相机 FOV 将网格单元标记为已覆盖并发布比例。

默认 `recon_require_full_path=true`，因此完整路径是任务条件，覆盖率是实时质量指标；仿真成功日志中最终覆盖率为 1.0。侦察主线不执行危险品分类。

## 9. 视觉精准降落

`PRECISION_LAND` 分为三段：

1. 在约 5 m 搜索 H，过滤置信度低和帧间跳变大的目标。
2. 视觉伺服根据图像中心误差发布水平速度；低于 `2.5 m` 且连续稳定帧满足条件后锁存对心。
3. 下降到 `1.5 m` 后停止水平修正，使用约 `0.30 m/s` 的纯垂直 Offboard 速度盲降。

盲降只在已锁存对心后发生。高空视觉丢失、对心失败或盲降超过 45 s 会停止运动并进入错误处理，不发送 PX4 LAND。真机接地依据 `/fmu/out/vehicle_land_detected`；SITL 只有在高度、水平速度、垂直速度和保持时间同时满足时才使用本地备用判定。确认接地后发送零速度并 force-disarm。

## 10. PX4 与舵机控制

`px4_control` 接收 `/drone/cmd_setpoint_ned`（`NedPositionSetpoint`：NED 位置、yaw 和可选航段速度）、`/drone/cmd_velocity_ned` 和 `/drone/vision/servo_cmd`，向 PX4 发布 OffboardControlMode、TrajectorySetpoint 和 VehicleCommand；同时输出 `/drone/pose`、速度和状态供任务与视觉使用。位置航段速度由任务 YAML 配置，并受 `max_setpoint_speed_limit` 和 PX4 自身速度上限约束；视觉精校/降落的速度模式保持独立。

`drone_servo2` 通过 `/drone/servo/trigger_drop` 接收投放请求。SITL 使用 Mock，仅记录和模拟位置；Jetson 使用 `VehicleCommand` 的 `DO_SET_ACTUATOR` 经 PX4 MAIN5 输出 PWM。

## 11. SITL 与真机消息边界

源码默认参数面向真机：`/fmu/out/vehicle_status_v1`。`sitl.launch.py` 显式覆盖为 v4；`scripts/sitl_env.sh` 将隔离构建的 v4 `px4_msgs` overlay 提升到环境前端。两端保持同一任务接口，差异集中在消息类型和启动传输层。

```text
真机：USB 相机 → Jetson；ROS Humble + workspace(v1) → UDP Agent:8888 ← PX4 Ethernet
SITL：ROS Humble + workspace(v1) + isolated overlay(v4) → UDP Agent → PX4 SITL v4
```

不要在同一运行环境中混用 v1/v4 生成的 Python 消息类型；切换 SITL 前必须执行 `source scripts/sitl_env.sh`。

## 12. 安全、异常和资源生命周期

- FSM 看门狗监控任务、视觉和舵机心跳。
- 飞行中检测到需要 RC 解锁但实际已解锁丢失时立即错误退出。
- 电池严重告警可触发返航或中止。
- 视觉节点只在需要阶段加载对应模型，侦察阶段释放投放模型。
- 任务结束取消 20 Hz 控制定时器，视觉模式切回 none，记录结果摘要。
- 真机操作仍必须由操作者保留遥控器接管和硬件急停能力。

## 13. 验证证据

最近一次完整仿真：`log/simulation_20260730_192807/ros2_system.log`。
关键证据包括两条“投放成功”、覆盖率最终 `1.0`、视觉盲降、SITL 近地停滞确认接地和 `mission_end: success=True`。

代码测试与构建命令见 [README.md](README.md) 和 [DEPLOYMENT.md](DEPLOYMENT.md)。
逐文件调用链和循序阅读路径见 [CODE_READING_GUIDE.md](CODE_READING_GUIDE.md)。

## 14. 已知限制

- SITL 的相机、动力学和接地检测不能替代真机飞行验证。
- 真机必须重新核对相机内参、V4L2 设备路径、PX4 v1 消息、MAIN5 输出和遥控器策略。
- `drone_servo` Python 包和危险品分类模块属于遗留/独立代码，不在当前完整任务主链路中。
