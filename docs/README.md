# 无人机自主任务飞行系统 — 全仿真

基于 ROS2 Humble + PX4 SITL + Gazebo Harmonic 的四旋翼无人机自主任务仿真系统。

**场地**: 70m × 8m | **机型**: gz_x500 | **任务**: 精准投放 + 侦察巡视 + 视觉返航降落

---

## 修改参数

所有飞行参数在 YAML 文件中，**改数值 → 重启仿真即可，无需编译**。

### 一步修改法

```bash
# 1. 用任意编辑器打开主配置文件
vim src/drone_mission/config/mission_params.yaml
# 或
code src/drone_mission/config/mission_params.yaml

# 2. 改数值，保存

# 3. 重启仿真
./scripts/start_full_simulation.sh
```

### 常用调参示例

```yaml
# 想让无人机飞得更快
transit_speed: 3.0     # 原 2.0 → 3.0 m/s，区域间中转更快

# 想让扫描更密集 (侦察更彻底)
sweep_speed: 0.5       # 原 0.8 → 0.5 m/s，慢速扫描
grid_spacing: 0.5      # 原 1.0 → 0.5 m，扫描线更密

# 投放高度调整
area1_altitude: 3.0    # 原 2.0 → 3.0 m，投放区飞高一点
drop_descend_height: 1.5  # 原 2.0 → 1.5 m，投放时降得更低

# 降落更精确 (更慢但更准)
landing_approach_speed: 0.3   # 原 0.5 → 0.3 m/s
landing_precision_target: 0.05  # 原 0.1 → 0.05 m

# 航点容差放宽 (避免卡在某个航点)
waypoint_tolerance: 1.0  # 原 0.5 → 1.0 m
waypoint_timeout: 60.0   # 原 120 → 60 s，超时更快跳过
```

### 四个配置文件

| 文件 | 改什么 | 频率 |
|------|--------|------|
| `src/drone_mission/config/mission_params.yaml` | 速度、高度、区域、容差、投放策略 | **最常用** |
| `src/px4_control/config/px4_params.yaml` | 底层控制频率、加速度 | 偶尔 |
| `src/drone_vision/config/vision_params.yaml` | YOLO 阈值、相机内参、视觉伺服 PID | 调试视觉 |
| `src/drone_servo2/config/servo_params.yaml` | 舵机 PWM 序列、投放延时 | 实飞前 |

### 参数生效机制

```
YAML 文件 → ROS2 Node 启动时加载 → Node.get_param() 读取 → 运行时使用
                                              ↑
                                    重启节点即重新读取
```

**不需要 `colcon build`**，因为 YAML 通过 ROS2 parameter 机制在运行时加载，不参与编译。

---

---

## 快速启动

```bash
cd /home/bridge/CUADC_drone_missions_ws
./scripts/start_full_simulation.sh
```

脚本自动完成：清理旧进程 → 启动 MicroXRCEAgent → PX4 SITL + Gazebo → ros_gz_image → 全系统 ROS2 节点

**停止仿真**:
```bash
pkill -f "bin/px4"; pkill -f "MicroXRCEAgent"; pkill -f "ros_gz_image"
```

---

## 系统架构

```
┌──────────────────────────────────────────────────────┐
│  Layer 4: Mission Orchestrator (drone_mission)       │
│  - 任务状态机 (FSM)   - PrecisionAlign 精校   - 路径规划 │
├──────────────────────────────────────────────────────┤
│  Layer 3: Skill Modules                              │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ │
│  │ drone_vision │ │drone_servo2  │ │ px4_control   │ │
│  │ YOLO+OpenCV  │ │ PWM/投放     │ │ 飞行控制+遥测 │ │
│  └──────────────┘ └──────────────┘ └──────────────┘ │
├──────────────────────────────────────────────────────┤
│  Layer 2: Flight Control Abstraction (px4_bridge)    │
│  - Offboard 管理  - 坐标变换(NED↔World)  - 遥测收集 │
├──────────────────────────────────────────────────────┤
│  Layer 1: Communication (MicroXRCEAgent / DDS)       │
│  - /fmu/out/* (PX4→ROS2)  - /fmu/in/* (ROS2→PX4)    │
└──────────────────────────────────────────────────────┘
```

### 任务状态机

```
INIT → ARM → TAKEOFF → TRANSIT_TO_DROP_ZONE → CYLINDER_SEARCH
→ SELECT_TARGET → APPROACH_CYLINDER → DESCEND_TO_DROP
→ PRECISION_ALIGN (像素闭环精校, 双速 + 两级降高, 2m→1.25m 热重启复查) → EXECUTE_DROP(×2)
→ ASCEND → TRANSIT_TO_RECON_ZONE → RECON_PATROL (全覆盖弓字形)
→ RETURN_TO_HOME → PRECISION_LAND → COMPLETE
```

### 飞行场地

```
  0m      7m   10m           40m   45m           65m   70m
  ├───────┼─────┼─────────────┼─────┼─────────────┼─────┤
  │ 准备区 │起降区             │投放区│   间隔区    │侦察区│
  │       │ H(10,4)           │3圆筒│             │5圆筒│
  └───────┴─────┴─────────────┴─────┴─────────────┴─────┘
```

---

## 飞行参数配置

所有参数在 YAML 文件中定义，**修改后重启仿真即可生效，无需重新编译**。

### 参数文件总览

| 文件 | 用途 |
|------|------|
| `src/drone_mission/config/mission_params.yaml` | **任务级参数**（主配置文件） |
| `src/px4_control/config/px4_params.yaml` | 飞行控制基础参数 |
| `src/drone_vision/config/vision_params.yaml` | 视觉检测参数 |
| `src/drone_servo2/config/servo_params.yaml` | 舵机投放参数 |

---

### 主配置文件: `mission_params.yaml`

```yaml
/mission_state_machine:
  ros__parameters:

    # ============================================================
    # 速度参数
    # ============================================================
    sweep_speed: 0.8              # 区域内部慢速扫描速度 (m/s)
    transit_speed: 2.0            # 区域之间中转飞行速度 (m/s)
    return_speed: 3.0             # 返航速度 (m/s)
    max_acceleration: 0.2         # 最大加速度 (m/s²)，控制轨迹平滑度

    # ============================================================
    # 高度参数
    # ============================================================
    takeoff_height: 5.0           # 起飞/巡航飞行高度 (m)
    area1_altitude: 2.0           # 投放区飞行高度 (m) — 低于巡航以便视觉检测
    area2_altitude: 5.0           # 侦察区飞行高度 (m)
    drop_descend_height: 2.0      # 投放时降高高度 (m)
    precision_align_drop_height_tolerance: 0.40  # 进入精校的高度误差阈值 (SITL 稳态~0.3m)
    landing_search_altitude: 5.0  # 降落前搜索H标识的高度 (m)
    landing_blind_altitude: 0.5   # 盲降切换高度 (m, 视觉丢失时启用)

    # ============================================================
    # 精准降落参数
    # ============================================================
    landing_approach_speed: 0.5   # 降落逼近速度 (m/s)
    landing_precision_target: 0.1 # 目标水平精度 (m)

    # ============================================================
    # Area 1: 投放区 (Gazebo x:40~45, y:0~8)
    # NED 坐标: north=[-4,4], east=[30,35]
    # ============================================================
    area1_north_min: -4.0
    area1_north_max: 4.0
    area1_east_min: 30.0
    area1_east_max: 35.0

    # ============================================================
    # Area 2: 侦察区 (Gazebo x:65~70, y:0~8)
    # NED 坐标: north=[-4,4], east=[55,60]
    # ============================================================
    area2_north_min: -4.0
    area2_north_max: 4.0
    area2_east_min: 55.0
    area2_east_max: 60.0

    # ============================================================
    # 弓字形覆盖参数 (侦察/搜索时使用)
    # ============================================================
    grid_spacing: 1.0             # 扫描线间距 (m)
    sweep_passes: 2               # 每区域折返次数
    sweep_margin_north: 1.0       # north 边界内缩 (m)
    sweep_margin_east: 0.5        # east 边界内缩 (m)

    # ============================================================
    # 容差与超时
    # ============================================================
    waypoint_tolerance: 0.5       # 航点到达判定距离 (m)
    takeoff_tolerance: 0.3        # 起飞到达判定距离 (m)
    waypoint_timeout: 120.0       # 单航点超时 (s), 超时则跳过
    state_max_duration: 180.0     # 单状态最大持续时间 (s)

    # ============================================================
    # 投放策略
    # ============================================================
    min_radius_first: true        # 优先选择最小半径圆筒
    drop_confirmation_timeout: 3.0 # 投放确认超时 (s)
    max_drops: 2                  # 最大投放次数

    # ============================================================
    # 异常处理
    # ============================================================
    battery_warning: 0.25         # 低电量警告 (比例)
    battery_critical: 0.15        # 严重低电量 (比例)
    vision_loss_timeout: 3.0      # 视觉连续丢失超时 (s)
    max_error_count: 5            # 最大错误次数
    control_frequency: 20.0       # 控制循环频率 (Hz)
```

---

### 速度参数使用场景

| 参数 | 使用阶段 | 说明 |
|------|---------|------|
| `sweep_speed` | CYLINDER_SEARCH, RECON_PATROL | 投放区慢速扫描、侦察区弓字形遍历 |
| `transit_speed` | TRANSIT_TO_DROP_ZONE, TRANSIT_TO_RECON_ZONE | 区域间中转飞行 |
| `return_speed` | RETURN_TO_HOME | 任务完成返航 |
| `max_acceleration` | 所有阶段 | px4_bridge 20Hz 控制循环平滑过渡 |

### 高度参数使用场景

| 参数 | 使用阶段 | 说明 |
|------|---------|------|
| `takeoff_height` | TAKEOFF, RETURN_TO_HOME | 起降和返航巡航高度 |
| `area1_altitude` | CYLINDER_SEARCH, APPROACH, DESCEND | 投放区降低高度便于视觉 |
| `area2_altitude` | RECON_PATROL | 侦察区降低高度后的相机扫描高度 |
| `drop_descend_height` | DESCEND_TO_DROP | 投放前降高 |

---

### 参数流向

```
mission_params.yaml            px4_params.yaml
       │                            │
       ▼                            ▼
  MissionFSM.get_param()     px4_bridge.get_parameter()
       │                            │
       ▼                            ▼
  fc.takeoff(height)         max_acceleration
  fc.goto_ned(n,e,d,speed)        │
       │                           ▼
       ▼                    20Hz 控制循环平滑
  /drone/cmd_setpoint_ned         │
       │                          ▼
       ▼                  /fmu/in/trajectory_setpoint
    px4_bridge                     │
       │                          ▼
       ▼                       PX4 飞控
  /fmu/in/trajectory_setpoint
       │
       ▼
    PX4 飞控执行
```

---

## 坐标系

| 坐标系 | 原点 | X | Y | Z |
|--------|------|---|---|---|
| Gazebo World | 场地左下角 | 东 (长边 70m) | 北 (宽边 8m) | 上 |
| PX4 NED | H 点 (10,4,0) | North (北) | East (东) | Down (下) |
| Body FRD | 无人机质心 | 前 (机头) | 右 | 下 |

**转换公式**: `NED_x = world_y - 4`, `NED_y = world_x - 10`, `NED_z = -world_z`

---

## 日志

仿真日志保存在 `log/simulation_<timestamp>/`:

| 文件 | 内容 |
|------|------|
| `ros2_system.log` | 全系统 ROS2 节点日志 (FSM 状态转换、视觉检测等) |
| `px4.log` | PX4 SITL 飞控日志 |
| `agent.log` | MicroXRCEAgent 日志 |
| `image_bridge.log` | ros_gz_image 桥接日志 |

实时查看:
```bash
tail -f log/simulation_*/ros2_system.log
```

---

## 项目结构

```
CUADC_drone_missions_ws/
├── src/
│   ├── drone_mission/       # 任务状态机 + 路径规划 + 精校控制
│   │   ├── config/mission_params.yaml
│   │   ├── drone_mission/mission_state_machine.py  # FSM 核心 (1381行)
│   │   ├── drone_mission/precision_align_controller.py  # 低空精校 (CUADC 像素闭环, 默认)
│   │   ├── drone_mission/drop_visual_aligner.py     # 低空视觉校准 (梯度下降备选)
│   │   ├── drone_mission/path_planner.py            # 弓字形路径规划
│   │   ├── drone_mission/cylinder_selector.py       # 圆筒选择策略
│   │   └── drone_mission/states/        # 11 个任务状态处理器
│   │   ├── drone_mission/simple_mission.py         # 简化航点测试 (自包含, 不依赖其他模块)
│   ├── px4_control/         # PX4 DDS 通信桥接 (657行) + 飞行控制 API (328行)
│   ├── drone_vision/        # YOLO(.pt) + OpenCV 视觉管线 (982行)
│   │   ├── drone_vision/models/  # YOLO 引擎 + 模型权重 (.pt)
│   │   └── scripts/vision_viewer.py  # 实时标注查看器
│   ├── drone_servo2/        # 舵机投放控制 (C++, 多位置步进)
│   ├── drone_servo/         # 舵机投放控制 (Python, 参考)
│   ├── drone_interfaces/    # 自定义 ROS2 接口 (msg/srv/action)
│   ├── drone_utils/         # 日志/看门狗/紧急处理/配置加载
│   ├── drone_bringup/       # 启动脚本 (launch: SITL + 实飞 + 视觉调试 + 简化航点)
│   └── px4_msgs/            # PX4 uORB → ROS2 消息定义
├── field/                   # Gazebo 场地描述 + 随机场地生成器
├── PX4-Autopilot -> /home/bridge/PX4-Autopilot
├── scripts/
│   ├── start_full_simulation.sh  # 一键 SITL 启动
│   └── debug_vision.sh           # 一键视觉调试 (摄像头 + 双YOLO模型 + viewer)
│   │   ├── simple_mission.sh         # 一键 SITL 启动 (简化航点测试)
├── cv_trainer -> /home/bridge/cv_trainer  # 模型训练来源
├── docs/                    # 架构文档 + 工作日志
└── log/                     # 构建/仿真/视觉截图日志
```

---

## 代码通读路径

从头全面理解该工作空间的推荐阅读路径。按依赖关系和信息流从底层到顶层排列，共 **7 个阶段、约 25 个文件**。

### 阅读原则

1. **从数据到控制**：先理解"有什么数据、怎么流动"，再理解"怎么用数据做决策"
2. **从底层到顶层**：先读基础设施，再读业务逻辑
3. **每次只聚焦一个问题**：读完一个模块后问自己"这段代码的输入是什么、输出是什么、做了哪些关键假设"

---

### 阶段 1：大局观（~30 min）

**目标**：理解系统全貌，知道有哪些组件、它们怎么连在一起

| 顺序 | 文件 | 重点关注 |
|------|------|---------|
| 1.1 | `docs/README.md`（本文件） | 场地布局、任务流程、启动方式、参数体系 |
| 1.2 | `docs/ARCHITECTURE.md` | 四层架构、ROS2 包结构、状态转换图、数据流话题表、坐标变换链 |
| 1.3 | `docs/DEBUG_2026-07-09.md` | 已知控制链分析和已知坑点 |
| 1.4 | `docs/CHANGELOG_2026-07-10.md` | 最新变更：模型融合、bbox 消息、launch 修复、视觉调试工具 |
| 1.5 | `scripts/start_full_simulation.sh` | 启动顺序（Agent→PX4→Bridge→ROS2节点），环境变量 |

**读完应能回答**：系统有几个包？每个包干什么？数据在话题间怎么流？无人机要完成什么任务？

---

### 阶段 2：基础设施 — 坐标、配置、工具（~45 min）

**目标**：理解整系统依赖的"语言"——坐标系、参数体系、日志/看门狗

| 顺序 | 文件 | 行数 | 重点关注 |
|------|------|------|---------|
| 2.1 | `src/px4_control/px4_control/telemetry.py` | ~150 | **核心**：`Position`/`Attitude`/`Telemetry` 数据类；`world_to_ned()` 和 `ned_to_world()` 变换公式；H 点偏移常量 (10, 4, 0) |
| 2.2 | `src/drone_vision/drone_vision/coordinate_transforms.py` | ~330 | **核心**：`CoordinateTransformer.pixel_to_world()` 6-DoF 射线求交；`R_cam_to_body` 旋转矩阵；运行管线不接受简化投影替代无效姿态 |
| 2.3 | `src/drone_utils/drone_utils/config_loader.py` | ~40 | 统一参数加载模式 |
| 2.4 | `src/drone_utils/drone_utils/logger.py` | ~40 | TaskLogger 接口 |
| 2.5 | `src/drone_utils/drone_utils/watchdog.py` | ~40 | Watchdog 心跳/超时机制 |
| 2.6 | `src/drone_utils/drone_utils/emergency.py` | ~40 | EmergencyHandler 紧急等级判定 |

**读完应能回答**：World 坐标 (42, 4, 5) 对应的 NED 坐标是什么？像素 (300, 200) 在 5m 高度对应哪个地面点？roll/pitch 为什么影响这个计算？

---

### 阶段 3：通信层 — PX4 ↔ ROS2 桥接（~60 min）

**目标**：理解无人机和 ROS2 之间每一条消息怎么传输

| 顺序 | 文件 | 行数 | 重点关注 |
|------|------|------|---------|
| 3.1 | `src/px4_control/px4_control/px4_bridge.py` | ~657 | **全系统最关键的单文件**。PX4 DDS QoS；里程计/状态/电池/着陆遥测；NED→World + `/drone/pose`；10Hz Offboard 心跳；20Hz 位置/速度控制；视觉速度和 VehicleCommand 中继 |
| 3.2 | `src/px4_control/px4_control/flight_controller.py` | ~300 | `takeoff()`/`goto_ned()`/`goto_world()`/`land()`/`set_velocity_ned()` 高级 API；`reached()` 航点到达判断；超时保护 |
| 3.3 | `src/drone_bringup/scripts/gz_camera_bridge.py` | ~100 | Gazebo 相机的实际分辨率（`gz topic --json-output`）——直接决定了 cx/cy 参数对不对 |

**读完应能回答**：`MissionFSM.fc.goto_ned(0, 32, -5)` 这一行代码触发了多少层调用，最终 PX4 收到了什么？位置模式和速度模式的切换机制是什么？如果 Offboard 心跳断了会发生什么？

---

### 阶段 4：视觉管线 — 感知系统（~90 min）

**目标**：理解从相机图像到检测结果的全过程

| 顺序 | 文件 | 行数 | 重点关注 |
|------|------|------|---------|
| 4.1 | `src/drone_vision/drone_vision/models/yolo_engine.py` | ~420 | ONNX Runtime CUDA/CPU 和 PyTorch 推理；严格验证请求的 provider；letterbox→推理→bbox 回原图；显式释放模型资源 |
| 4.2 | `src/drone_vision/drone_vision/vision_pipeline.py` | ~900 | **全系统第二关键的单文件**。强制加载圆筒与蓝色标识 ONNX 模型；20FPS 上限下串行推理；蓝色 HSV/形状/物理尺寸验证；H 几何检测和 NED 视觉伺服；进入侦察后释放两套模型 |
| 4.3 | `src/drone_vision/drone_vision/cylinder_detector.py` | ~530 | `CylinderRefiner.refine()`: YOLO bbox→ROI→Canny→椭圆拟合→白度评分；`CylinderTracker`: IoU 多帧匹配→min_hits过滤闪烁→稳定ID (支持 bbox) |
| 4.4 | `src/drone_vision/drone_vision/landing_detector.py` | ~250 | `LandingPadRefiner.refine()`: 自适应阈值→圆形检测→PCA 方向估计 |
| 4.5 | `src/drone_vision/drone_vision/coverage_monitor.py` | ~180 | 网格离散化 + FOV 投影覆盖标记 + 覆盖率计算 |
| 4.6 | `src/drone_vision/drone_vision/visual_servoing.py` | ~280 | `VisualServoController` 降落模式 PID；`CylinderApproachController` 圆筒逼近 PID |
| 4.7 | `src/drone_vision/drone_vision/camera.py` | ~200 | SITL/V4L2 双模式；稳定设备路径；强制校验实际协商的分辨率；每发布周期只读取一帧 |
| 4.8 | `src/drone_vision/scripts/vision_viewer.py` | ~200 | 实时标注画面：绿色YOLO框+红点精修圆心+橙色投放区标识；按 q 退出 / s 截图 |
| 4.9 | `scripts/debug_vision.sh` | ~70 | 一键视觉调试：摄像头 + 双模型 + 标注画面 |

**读完应能回答**：实飞 640×480 图像中的圆筒怎样变成 `CylinderDetection`？姿态补偿为什么是硬要求？模型或 CUDA provider 不可用时节点为什么必须启动失败？

---

### 阶段 5：任务编排 — 状态机（~90 min）

**目标**：理解无人机每个状态的决策逻辑

| 顺序 | 文件 | 行数 | 重点关注 |
|------|------|------|---------|
| 5.1 | `src/drone_mission/drone_mission/mission_state_machine.py` | ~1381 | **全系统第三关键的单文件**。`MissionFSM` 主类：参数声明、`_lazy_init`、20Hz `_control_loop`；`STATE_TRANSITIONS` 转移表；`_remember_cylinders` 记忆库积累+合并；`_normalize_drop_target_xy` 投放区边界验证；`_StandaloneBridgeFacade` PX4 桥接门面；`_StandaloneFlightFacade` 飞行控制门面（`/drone/cmd_setpoint_ned` 和 `/drone/cmd_velocity_ned` 发布）；`start_drop_service`/`check_drop_service_result` 异步投放 |
| 5.2 | `src/drone_mission/drone_mission/states/state_init.py` | ~83 | INIT：等待 odom + 30s 超时 |
| 5.3 | `src/drone_mission/drone_mission/states/state_takeoff.py` | ~127 | TAKEOFF：ARM→OFFBOARD→CLIMB 三子阶段；15s/30s 超时 |
| 5.4 | `src/drone_mission/drone_mission/states/state_transit.py` | ~105 | TRANSIT：通用过渡飞行，120s 超时 |
| 5.5 | `src/drone_mission/drone_mission/states/state_cylinder_search.py` | ~178 | CYLINDER_SEARCH：悬停→累积记忆→慢速扫描→超时策略 |
| 5.6 | `src/drone_mission/drone_mission/cylinder_selector.py` | ~145 | 半径排序+位置去重+已选排除 |
| 5.7 | `src/drone_mission/drone_mission/states/state_cylinder_approach.py` | ~229 | **最复杂的单状态**。像素伺服 vs 记忆导航双模式；`_goto_pixel_centered_target` vs `_goto_target_xy`；位置匹配（不依赖ID）；20帧对心稳定性检查 |
| 5.8 | `src/drone_mission/drone_mission/states/state_drop.py` | ~465 | 降高→精校→投放许可检查→异步投放→回升→判断是否再投；使用 PrecisionAlignController (CUADC 像素闭环, 双速+两级降高) 作为默认精校方案，DropVisualAligner (梯度下降) 备选；超时原地重试 |
| 5.9 | `src/drone_mission/drone_mission/precision_align_controller.py` | ~426 | **默认精校方案**。CUADC 风格像素闭环：像素误差→归一化→EMA平滑→P增益→机体速度；双速模式(粗调≤0.075m/s, 精调≤0.022m/s)；步进-悬停-观察周期；收敛→0.8s 刹车复查→投放许可；两级投放高度(2.0m→1.25m)。关键参数在 `mission_params.yaml` 中以 `precision_align_` 前缀 |
| 5.10 | `src/drone_mission/drone_mission/drop_visual_aligner.py` | ~489 | **备选方案** (`precision_align_enable: false`)。端点比较法梯度下降：锁定目标→±探测→三方比较→提交→悬停观察→刹车。在线目标身份锁定 + 自适应步长。marker 中心已改用 OpenCV HSV 精修 |
| 5.11 | `src/drone_mission/drone_mission/states/state_recon_patrol.py` | ~148 | 弓字形路径生成→逐航点飞行→覆盖监控→超时/完成返航 |
| 5.12 | `src/drone_mission/drone_mission/states/state_return.py` | ~82 | 返航 H 点→2s 稳定性确认 |
| 5.13 | `src/drone_mission/drone_mission/states/state_precision_land.py` | ~176 | 三阶段降落：搜索→伺服→盲降；视觉丢失退化策略 |
| 5.14 | `src/drone_mission/drone_mission/path_planner.py` | ~366 | 自动最优条带方向；`plan_coverage_ned` NED 弓字形；`plan_transit` 过渡路径 |

**读完应能回答**：状态机怎么从 INIT 一步步走到 COMPLETE？每个状态的 enter/execute/exit 三阶段做了什么决策？异常怎么处理？

---

### 阶段 6：执行与接口 — 舵机、自定义消息、启动（~30 min）

**目标**：理解投放执行和系统集成

| 顺序 | 文件 | 行数 | 重点关注 |
|------|------|------|---------|
| 6.1 | `src/drone_servo2/src/servo_controller.cpp` | ~400 | 舵机多位置步进投放 (C++ 主力) |
| 6.2 | `src/drone_servo2/include/drone_servo2/drop_manager.hpp` | ~104 | 投放状态机 (C++) |
| 6.3 | `src/drone_servo/drone_servo/servo_controller.py` | ~220 | Python 参考实现 |
| 6.4 | `src/drone_interfaces/msg/` 目录 | — | CylinderDetection、LandingTarget、CoverageStatus 消息定义 |
| 6.5 | `src/drone_interfaces/srv/` 目录 | — | TriggerDrop 服务定义 |
| 6.6 | `src/drone_bringup/launch/sitl.launch.py` | ~139 | 4 节点启动顺序 + 延迟 |

---

### 阶段 7：参数与配置 — 运行时调参（~15 min）

**目标**：知道改什么参数会影响什么行为

| 顺序 | 文件 | 重点关注 |
|------|------|---------|
| 7.1 | `src/drone_mission/config/mission_params.yaml` | 速度/高度/区域/容差/投放/降落/异常 — **最常改的文件** |
| 7.2 | `src/px4_control/config/px4_params.yaml` | 底层控制频率/加速度 |
| 7.3 | `src/drone_vision/config/vision_params.yaml` | 相机内参、YOLO 阈值、伺服 PID |
| 7.4 | `src/drone_servo2/config/servo_params.yaml` | PWM 位置序列 |

---

### 阅读时间估算

| 阶段 | 预估时间 | 累计 |
|------|---------|------|
| 1. 大局观 | 30 min | 30 min |
| 2. 基础设施 | 45 min | 1h 15min |
| 3. 通信层 | 60 min | 2h 15min |
| 4. 视觉管线 | 100 min | 3h 55min |
| 5. 状态机 | 90 min | 5h 25min |
| 6. 执行与接口 | 30 min | 5h 55min |
| 7. 参数配置 | 15 min | 6h 10min |

> **总计**: ~25 个关键文件，约 8000 行 Python 代码

---

### 核心心智模型

通读完以上所有文件后，应能在脑中建立这条完整的控制链：

```
Gazebo 相机 (分辨率取决于 PX4 模型)
  → gz_camera_bridge → /drone/camera/image_raw
    → VisionPipelineNode._image_cb()
      → YOLO 推理 → letterbox rescale → 后处理
        → 姿态补偿 / 简化投影 → center_x, center_y (World)
          → /drone/vision/cylinders
            → MissionFSM._cyl_cb() → _remember_cylinders()
              → CylinderSelector.select() → 选筒
                → CylinderApproachState._goto_pixel_centered_target()
                  → 像素误差 du/dv → du*alt/fx (注意: 无姿态补偿! 无yaw旋转!)
                    → target_wx, target_wy → world_to_ned()
                      → fc.goto_ned(tn, te, td)
                        → _StandaloneFlightFacade._publish_cmd()
                          → /drone/cmd_setpoint_ned
                            → px4_bridge._cmd_setpoint_cb()
                              → _control_loop 斜坡平滑 (max_acceleration)
                                → _offboard_heartbeat
                                  → /fmu/in/trajectory_setpoint
                                    → PX4 飞控 → 电机
                                      → /fmu/out/vehicle_odometry
                                        → px4_bridge → /drone/pose
                                          → VisionPipeline._pose_cb (闭环!)
```

**关键注意点**（已知坑）：
- 相机分辨率与 cx/cy 参数的匹配（`DEBUG_2026-07-09.md` 根因 1）
- 像素伺服忽略无人机 yaw（`DEBUG_2026-07-09.md` 根因 2）
- 两套独立的世界坐标计算（`DEBUG_2026-07-09.md` 根因 3）
- YOLO bbox 必须 rescale 回原图（`DEBUG_2026-07-09.md` 提及）
- coverage monitor 必须每帧 update（`DEBUG_2026-07-09.md` 提及）
- OpenCV VideoCapture 内部缓冲导致显示延迟 → grab/retrieve 替代 read（`CHANGELOG_2026-07-10.md`）
- launch 文件空字符串覆盖 YAML 参数 → 移除 model_path 覆盖（`CHANGELOG_2026-07-10.md`）
- class_names 必须与模型 nc 一致，否则后处理列偏移错误 → YAML 驱动

---

## SITL 与真机双路线

SITL 仿真已跑通。真机通过独立 launch/config 覆盖差异, 核心代码共用。

> **当前状态 (2026-07-20)**: ONNX 模型随包安装；视觉节点同时验证两套模型和指定执行 provider，任一失败都会终止启动，不再静默进入慢速回退模式。
>
> 审计报告: `docs/2026-07-14-sitl-audit-and-real-hardware-gaps.md`
> 关键发现: `precision_align_drop_height_tolerance: 0.30→0.40` 修复 SITL 稳态高度偏差 (PX4 MPC_THR_HOVER 估计偏差)
> 任务解锁使用标准 `param2=0`；`21196` 仅保留在显式紧急强制上锁接口，不参与正常任务流程。
> **真机开发方案**: `docs/2026-07-17-real-hardware-workspace-plan.md` — 工作空间布局、4 阶段开发顺序、文件改动清单
> 真机 ARM force-magic (21196) 在 SITL 侧保留, 真机侧需新建标准解锁流程

---

## 简化航点飞行测试 (`simple_mission`)

全任务 FSM 涉及 14 个状态、视觉检测、舵机投放等模块，首次在真机上调试时变量太多。`simple_mission` 是一个**自包含的单文件简化任务节点**，只做最基础的飞行：

```
INIT → ARM → TAKEOFF → TRANSIT_TO_DROP → LOITER → RETURN → LAND → COMPLETE
```

### 编译

```bash
cd ~/CUADC_drone_missions_ws
colcon build --symlink-install
source install/setup.bash
```

### 启动

**快速启动**
```bash
./scripts/simple_mission.sh
```

**仿真环境**（需先启动 PX4 SITL + Gazebo + MicroXRCEAgent）：

```bash
# 使用默认参数：飞往投放区 (42.5, 4.0)，高度 5m，悬停 5s
ros2 launch drone_bringup simple_mission.launch.py

# 自定义航点和高度
ros2 launch drone_bringup simple_mission.launch.py \
    drop_x:=42.5 drop_y:=4.0 \
    cruise_altitude:=5.0 \
    loiter_time:=3.0

# 无 GUI (headless) 模式, 减少内存占用
HEADLESS=1 ros2 launch drone_bringup simple_mission.launch.py
```

**真机环境**（仅需 MicroXRCEAgent + px4_bridge + simple_mission）：

```bash
ros2 launch drone_bringup simple_mission.launch.py \
    sim_mode:=false \
    home_x:=0.0 home_y:=0.0 \
    drop_x:=<目标点X> drop_y:=<目标点Y> \
    cruise_altitude:=5.0
```

### 可配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `cruise_altitude` | 5.0 | 巡航飞行高度 (m) |
| `drop_x` | 42.5 | 目标航点 World X |
| `drop_y` | 4.0 | 目标航点 World Y |
| `home_x` | 10.0 | 起飞点 World X |
| `home_y` | 4.0 | 起飞点 World Y |
| `loiter_time` | 5.0 | 目标点上空悬停时间 (s) |
| `sim_mode` | true | true=SITL force arm, false=真机标准解锁 |

### 与全任务代码的对应关系

`simple_mission` 与 `mission_fsm` **共用相同的底层模块和通信路径**：

```
simple_mission ── StandaloneBridge ──→ /drone/cmd_setpoint_ned            ←── mission_fsm 同
                                   ──→ /drone/cmd/vehicle_command_relay    ←── mission_fsm 同
                                   ←── /fmu/out/vehicle_odometry           ←── mission_fsm 同
                                   ←── /fmu/out/vehicle_status_v4          ←── mission_fsm 同
                                   ←── /fmu/out/vehicle_land_detected      ←── mission_fsm 同

坐标变换:  simple_mission ── telemetry.configure_home() → world_to_ned()  ←── mission_fsm 同
飞行控制:  simple_mission ── FlightController.takeoff/goto_world/land     ←── mission_fsm 同
```

各阶段与全任务 FSM 的异同：

| 阶段 | 与全任务对比 | 说明 |
|------|:-----------:|------|
| 坐标变换 | **完全相同** | 共用 `telemetry.configure_home()` + `world_to_ned()` |
| 通信层 | **完全相同** | 共用 `StandaloneBridge` (10Hz 心跳 + 遥测 + 指令中继) |
| 解锁/Offboard | **完全相同** | `StandaloneBridge.arm()` / `set_offboard_mode()` |
| 起飞 (TAKEOFF) | **逻辑相似** | `fc.takeoff()` 内联实现, 全任务用 `TakeoffState` 分阶段 |
| 航点飞行 (TRANSIT) | **完全相同** | `world_to_ned() → StandaloneBridge.set_position_target()` |
| 返航 (RETURN) | **完全相同** | 同上 |
| 降落 (LAND) | **策略不同** | simple: `StandaloneBridge.land_at_current()` 托管 PX4; 全任务: `FlightController.land()` 速度下降 |

### 这个测试能验证什么

**能验证（与全任务共享的底层链路）：**

| 验证项 | 对应全任务代码 |
|--------|--------------|
| PX4 DDS ↔ ROS2 通信链路 | `px4_bridge.py` — DDS 订阅/发布 + Offboard 心跳 |
| StandaloneBridge 中继通信 | `standalone_bridge.py` — 遥控测 + 10Hz 心跳 + 指令转发 |
| 坐标变换 (World ↔ NED) | `telemetry.py` — `configure_home()` + `world_to_ned()` / `ned_to_world()` |
| FlightController 飞行控制 | `flight_controller.py` — `takeoff()` / `goto_world()` / `land()` |
| 位置控制指令 → 飞控执行 | `/drone/cmd_setpoint_ned` → `px4_bridge` → PX4 |
| 真机 GPS/里程计定位精度 | 全任务所有依赖位置的状态 |

**不能验证（视觉和投放模块需单独测试）：**

| 模块 | 涉及文件 | 为何不覆盖 |
|------|---------|-----------|
| 视觉检测管线 | `vision_pipeline.py`, `cylinder_detector.py`, YOLO 模型 | simple_mission 不启动 vision 节点 |
| 精准投放对准 | `precision_align_controller.py`, `drop_visual_aligner.py` | 需要视觉输入 |
| 舵机投放控制 | `servo_controller.py`, `drop_manager.py` | 不启动 servo 节点 |
| 精准视觉降落 | `state_precision_land.py`, `landing_detector.py` | 需要视觉 + H 标识检测 |
| 侦察巡逻 | `state_recon_patrol.py`, `path_planner.py` | 不涉及 |

### 建议的测试顺序

```
1. SITL 仿真跑通 simple_mission     ← 验证代码本身正确
2. 真机跑通 simple_mission           ← 验证 PX4 通信 + 飞行控制链路
3. 真机跑全任务（不加视觉）          ← 验证完整 FSM 编排（舵机用 mock）
4. 真机加视觉（先调试 YOLO 模型）   ← 逐步集成感知模块
5. 真机全任务联调                     ← 完整系统验证
```

这一步一步来，每一步出问题都能精确定位到新增的模块，而不是在 14 个状态中猜测根因。
