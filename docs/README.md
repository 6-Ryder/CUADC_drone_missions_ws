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
| `src/drone_servo/config/servo_params.yaml` | 舵机 PWM、投放延时 | 实飞前 |

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
cd /home/ryder/drone_tasks_ws
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
│  - 任务状态机 (FSM)   - 圆筒选择   - 路径规划        │
├──────────────────────────────────────────────────────┤
│  Layer 3: Skill Modules                              │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ │
│  │ drone_vision │ │ drone_servo  │ │ px4_control   │ │
│  │ YOLO+OpenCV  │ │ PWM/投放     │ │ 飞行控制抽象  │ │
│  └──────────────┘ └──────────────┘ └──────────────┘ │
├──────────────────────────────────────────────────────┤
│  Layer 2: Flight Control Abstraction (px4_bridge)    │
│  - Offboard 管理  - 坐标变换  - 遥测收集            │
├──────────────────────────────────────────────────────┤
│  Layer 1: Communication (MicroXRCEAgent / DDS)       │
│  - /fmu/out/* (PX4→ROS2)  - /fmu/in/* (ROS2→PX4)    │
└──────────────────────────────────────────────────────┘
```

### 任务状态机

```
INIT → ARM → TAKEOFF → TRANSIT_TO_DROP_ZONE → CYLINDER_SEARCH
→ SELECT_TARGET → APPROACH_CYLINDER → DESCEND_TO_DROP → EXECUTE_DROP(×2)
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
| `src/drone_servo/config/servo_params.yaml` | 舵机投放参数 |

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
drone_tasks_ws/
├── src/
│   ├── drone_mission/       # 任务状态机 + 路径规划
│   │   ├── config/mission_params.yaml
│   │   └── drone_mission/states/   # 各任务状态处理器
│   ├── px4_control/         # PX4 DDS 通信桥接
│   ├── drone_vision/        # YOLO + OpenCV 视觉管线
│   ├── drone_servo/         # 舵机投放控制
│   ├── drone_interfaces/    # 自定义 ROS2 接口
│   ├── drone_utils/         # 日志/看门狗/紧急处理
│   ├── drone_bringup/       # 启动脚本 (launch files)
│   └── px4_msgs/            # PX4 uORB → ROS2 消息定义
├── field/
│   └── drone_field.sdf      # Gazebo 场地描述文件
├── PX4-Autopilot -> /home/ryder/PX4-Autopilot
├── scripts/
│   └── start_full_simulation.sh # 一键启动脚本
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
| 1.4 | `scripts/start_full_simulation.sh` | 启动顺序（Agent→PX4→Bridge→ROS2节点），环境变量 |

**读完应能回答**：系统有几个包？每个包干什么？数据在话题间怎么流？无人机要完成什么任务？

---

### 阶段 2：基础设施 — 坐标、配置、工具（~45 min）

**目标**：理解整系统依赖的"语言"——坐标系、参数体系、日志/看门狗

| 顺序 | 文件 | 行数 | 重点关注 |
|------|------|------|---------|
| 2.1 | `src/px4_control/px4_control/telemetry.py` | ~150 | **核心**：`Position`/`Attitude`/`Telemetry` 数据类；`world_to_ned()` 和 `ned_to_world()` 变换公式；H 点偏移常量 (10, 4, 0) |
| 2.2 | `src/drone_vision/drone_vision/coordinate_transforms.py` | ~330 | **核心**：`CoordinateTransformer.pixel_to_world()` 6-DoF 射线求交；`pixel_to_world_simple()` 简化回退；`AltitudeAdaptiveParams` 高度自适应；`R_cam_to_body` 旋转矩阵 |
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
| 3.1 | `src/px4_control/px4_control/px4_bridge.py` | ~660 | **全系统最关键的单文件**。4 步注册握手；QoS 配置；`_odom_cb` 中 NED→World 变换 + `/drone/pose` 发布；`_offboard_heartbeat` 10Hz Offboard 模式声明；`_control_loop` 20Hz 斜坡平滑；位置/速度双模式切换；VehicleCommand 中继 |
| 3.2 | `src/px4_control/px4_control/flight_controller.py` | ~300 | `takeoff()`/`goto_ned()`/`goto_world()`/`land()`/`set_velocity_ned()` 高级 API；`reached()` 航点到达判断；超时保护 |
| 3.3 | `src/drone_bringup/scripts/gz_camera_bridge.py` | ~100 | Gazebo 相机的实际分辨率（`gz topic --json-output`）——直接决定了 cx/cy 参数对不对 |

**读完应能回答**：`MissionFSM.fc.goto_ned(0, 32, -5)` 这一行代码触发了多少层调用，最终 PX4 收到了什么？位置模式和速度模式的切换机制是什么？如果 Offboard 心跳断了会发生什么？

---

### 阶段 4：视觉管线 — 感知系统（~90 min）

**目标**：理解从相机图像到检测结果的全过程

| 顺序 | 文件 | 行数 | 重点关注 |
|------|------|------|---------|
| 4.1 | `src/drone_vision/drone_vision/models/yolo_engine.py` | ~320 | YOLO ONNX/TensorRT 推理引擎；letterbox→推理→`_rescale_bbox()` 回原图（**关键修复点**）；`Detection` 数据类 |
| 4.2 | `src/drone_vision/drone_vision/vision_pipeline.py` | ~840 | **全系统第二关键的单文件**。`_lazy_init` 加载引擎+创建后处理器；`_image_cb` 主回调（帧率自适应→YOLO推理→三路后处理）；`_process_cylinders_v2` 姿态补偿坐标变换链；`_process_landing_v2` H 标识+视觉伺服；`_process_coverage` 覆盖网格更新；伺服模式切换 |
| 4.3 | `src/drone_vision/drone_vision/cylinder_detector.py` | ~320 | `CylinderRefiner.refine()`: YOLO bbox→ROI→Canny→椭圆拟合→白度评分；`CylinderTracker`: IoU 多帧匹配→稳定ID |
| 4.4 | `src/drone_vision/drone_vision/landing_detector.py` | ~250 | `LandingPadRefiner.refine()`: 自适应阈值→圆形检测→PCA 方向估计 |
| 4.5 | `src/drone_vision/drone_vision/coverage_monitor.py` | ~180 | 网格离散化 + FOV 投影覆盖标记 + 覆盖率计算 |
| 4.6 | `src/drone_vision/drone_vision/visual_servoing.py` | ~280 | `VisualServoController` 降落模式 PID；`CylinderApproachController` 圆筒逼近 PID |
| 4.7 | `src/drone_vision/drone_vision/camera.py` | ~180 | SITL/GStreamer 双模式相机驱动 |

**读完应能回答**：一个 1280×960 图像中的圆筒经过了哪些处理才变成 `CylinderDetection(center_x, center_y, confidence)`？姿态补偿和简化投影的区别在哪里？YOLO 模型不可用时系统如何回退？

---

### 阶段 5：任务编排 — 状态机（~90 min）

**目标**：理解无人机每个状态的决策逻辑

| 顺序 | 文件 | 行数 | 重点关注 |
|------|------|------|---------|
| 5.1 | `src/drone_mission/drone_mission/mission_state_machine.py` | ~1186 | **全系统第三关键的单文件**。`MissionFSM` 主类：参数声明、`_lazy_init`、20Hz `_control_loop`；`STATE_TRANSITIONS` 转移表；`_remember_cylinders` 记忆库积累+合并；`_normalize_drop_target_xy` 投放区边界验证；`_StandaloneBridgeFacade` PX4 桥接门面；`_StandaloneFlightFacade` 飞行控制门面（`/drone/cmd_setpoint_ned` 和 `/drone/cmd_velocity_ned` 发布）；`start_drop_service`/`check_drop_service_result` 异步投放 |
| 5.2 | `src/drone_mission/drone_mission/states/state_init.py` | ~83 | INIT：等待 odom + 30s 超时 |
| 5.3 | `src/drone_mission/drone_mission/states/state_takeoff.py` | ~127 | TAKEOFF：ARM→OFFBOARD→CLIMB 三子阶段；15s/30s 超时 |
| 5.4 | `src/drone_mission/drone_mission/states/state_transit.py` | ~105 | TRANSIT：通用过渡飞行，120s 超时 |
| 5.5 | `src/drone_mission/drone_mission/states/state_cylinder_search.py` | ~161 | CYLINDER_SEARCH：悬停→累积记忆→慢速扫描→超时策略 |
| 5.6 | `src/drone_mission/drone_mission/cylinder_selector.py` | ~135 | 半径排序+位置去重+已选排除 |
| 5.7 | `src/drone_mission/drone_mission/states/state_cylinder_approach.py` | ~196 | **最复杂的单状态**。像素伺服 vs 记忆导航双模式；`_goto_pixel_centered_target` vs `_goto_target_xy`；位置匹配（不依赖ID）；20帧对心稳定性检查 |
| 5.8 | `src/drone_mission/drone_mission/states/state_drop.py` | ~204 | 降高→投放许可检查（`_drop_target_is_centered`）→异步投放→回升→判断是否再投 |
| 5.9 | `src/drone_mission/drone_mission/states/state_recon_patrol.py` | ~149 | 弓字形路径生成→逐航点飞行→覆盖监控→超时/完成返航 |
| 5.10 | `src/drone_mission/drone_mission/states/state_return.py` | ~82 | 返航 H 点→2s 稳定性确认 |
| 5.11 | `src/drone_mission/drone_mission/states/state_precision_land.py` | ~177 | 三阶段降落：搜索→伺服→盲降；视觉丢失退化策略 |
| 5.12 | `src/drone_mission/drone_mission/path_planner.py` | ~367 | 自动最优条带方向；`plan_coverage_ned` NED 弓字形；`plan_transit` 过渡路径 |

**读完应能回答**：状态机怎么从 INIT 一步步走到 COMPLETE？每个状态的 enter/execute/exit 三阶段做了什么决策？异常怎么处理？

---

### 阶段 6：执行与接口 — 舵机、自定义消息、启动（~30 min）

**目标**：理解投放执行和系统集成

| 顺序 | 文件 | 行数 | 重点关注 |
|------|------|------|---------|
| 6.1 | `src/drone_servo/drone_servo/servo_controller.py` | ~30 | 舵机控制（桩） |
| 6.2 | `src/drone_servo/drone_servo/servo_mock.py` | ~10 | SITL 模拟投放 |
| 6.3 | `src/drone_servo/drone_servo/drop_manager.py` | ~10 | 投放逻辑管理 |
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
| 7.4 | `src/drone_servo/config/servo_params.yaml` | PWM 参数 |

---

### 阅读时间估算

| 阶段 | 预估时间 | 累计 |
|------|---------|------|
| 1. 大局观 | 30 min | 30 min |
| 2. 基础设施 | 45 min | 1h 15min |
| 3. 通信层 | 60 min | 2h 15min |
| 4. 视觉管线 | 90 min | 3h 45min |
| 5. 状态机 | 90 min | 5h 15min |
| 6. 执行与接口 | 30 min | 5h 45min |
| 7. 参数配置 | 15 min | 6h |

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
