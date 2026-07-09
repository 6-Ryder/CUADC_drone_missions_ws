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
