# 无人机自主任务飞行系统 — 架构设计方案

## 一、背景与目标

在已有的 `drone_fields_ws` 场地基础上，构建一套运行于 Jetson Orin Nano 上的无人机自主任务系统。系统控制一架搭载下视摄像头、舵机投放机构的四旋翼无人机，在 70m×8m 的飞行场地中完成四项任务：

1. **精准投放 (Area 1)**：视觉识别圆筒 → 选择半径最小者 → 飞至正上方 → 舵机投放 → 再选另一圆筒重复
2. **侦察巡视 (Area 2)**：全覆盖弓字形路径巡视 → 实时覆盖监控 (不进行危险品识别)
3. **精准返航降落**：返回起降点 → 视觉识别 "H" 标识 → 对心精准降落

---

## 二、系统架构总览

### 2.1 硬件拓扑

```
┌──────────────────────────────────────────────────────┐
│                  Jetson Orin Nano                     │
│                                                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐   │
│  │ Mission   │  │ Vision   │  │ Servo            │   │
│  │ Controller│  │ Pipeline │  │ Controller       │   │
│  │ (Python)  │  │ (C++/Py) │  │ (GPIO/PWM)       │   │
│  └─────┬─────┘  └────┬─────┘  └────────┬─────────┘   │
│        │              │                │              │
│        └──────────────┼────────────────┘              │
│                       │ DDS/RTPS                      │
│                       │ (micro-XRCE-DDS)               │
│                       │ via Serial/UART               │
└───────────────────────┼──────────────────────────────┘
                        │
┌───────────────────────┼──────────────────────────────┐
│                PX4 Flight Controller                  │
│  uxrce_dds_client ←→ uORB ←→ EKF2/Commander/Control  │
│   ⇅                                                  │
│  IMU / GPS / Baro / Mag (sensors)                     │
│  Motor PWM outputs                                    │
└──────────────────────────────────────────────────────┘
```

### 2.2 软件分层架构

采用**四层解耦架构**，层与层之间通过明确的 ROS2 接口通信：

```
┌─────────────────────────────────────────────────────────┐
│  Layer 4: Mission Orchestrator (drone_mission)          │
│  - 任务状态机 (FSM)                                      │
│  - 任务级决策 (选筒策略、侦察完成判定)                     │
│  - 异常处理与降级策略                                     │
├─────────────────────────────────────────────────────────┤
│  Layer 3: Skill Modules                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ Vision       │  │ Servo        │  │ Path Planner  │  │
│  │ (drone_vision)│  │ (drone_servo)│  │ (内嵌)        │  │
│  │              │  │              │  │              │  │
│  │ - YOLO检测   │  │ - PWM控制    │  │ - 覆盖路径   │  │
│  │ - 椭圆精修   │  │ - 投放逻辑   │  │ - 过渡路径   │  │
│  │ - 覆盖监控   │  │ - SITL Mock  │  │ - 对心降落   │  │
│  │ - 姿态补偿   │  │              │  │              │  │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  │
├─────────┼─────────────────┼─────────────────┼───────────┤
│  Layer 2: Flight Control Abstraction (px4_control)      │
│  - PX4 Offboard 模式管理                                │
│  - 位置/速度/偏航角 setpoint 发布                        │
│  - 组件注册、解锁检查、模式切换                           │
│  - 遥测数据收集与分发 (Odometry, Status, LandDetected)   │
│  - 坐标系转换 (World ↔ NED Local ↔ Body)                 │
├─────────────────────────────────────────────────────────┤
│  Layer 1: Communication (micro-XRCE-DDS)                │
│  - /fmu/out/* 订阅 (PX4→ROS2)                           │
│  - /fmu/in/*  发布 (ROS2→PX4)                           │
│  - QoS: BEST_EFFORT + TRANSIENT_LOCAL                   │
└─────────────────────────────────────────────────────────┘
```

---

## 三、ROS2 包结构

```
drone_tasks_ws/src/
├── px4_msgs/                       # PX4 uORB → ROS2 消息定义 (从Task_flight_ws复用)
│   └── msg/                        # 150+ .msg 文件
│
├── drone_interfaces/               # 本系统自定义接口
│   ├── msg/
│   │   ├── CylinderDetection.msg    # 圆筒检测结果 (id, 圆心xy, 半径, 置信度)
│   │   ├── CylinderArray.msg        # 多个圆筒检测结果数组
│   │   ├── HazardDetection.msg      # 危险品检测结果 (类型, 位置, 置信度)
│   │   ├── HazardArray.msg
│   │   └── LandingTarget.msg        # 降落目标 (圆心, 方向, 置信度)
│   ├── srv/
│   │   ├── TriggerDrop.srv          # 触发投放服务 (request: cylinder_id, response: 成功/失败)
│   │   └── ReportHazards.srv        # 危险品上报服务
│   └── action/
│       └── PrecisionLand.action     # 精准降落Action (feedback: 偏移量)
│
├── px4_control/                     # 飞行控制抽象层
│   ├── px4_control/
│   │   ├── __init__.py
│   │   ├── px4_bridge.py            # PX4 DDS 通信桥接 (注册/心跳/遥测/NED→World)
│   │   ├── flight_controller.py     # 飞行控制指令接口 (takeoff, goto_ned, land)
│   │   └── telemetry.py             # 遥测数据模型 (Position, Status, Attitude)
│   ├── config/
│   │   └── px4_params.yaml
│   └── package.xml
│
├── drone_vision/                    # 视觉处理管线 (YOLO + OpenCV 混合)
│   ├── drone_vision/
│   │   ├── __init__.py              # 包文档与版本
│   │   ├── vision_pipeline.py       # ★ 统一管线: 单YOLO模型多任务推理 (推荐部署)
│   │   ├── cylinder_detector.py     # 圆筒检测: YOLO粗定位→OpenCV椭圆拟合
│   │   ├── hazard_classifier.py     # 危险品分类 (已禁用, 保留供独立使用)
│   │   ├── landing_detector.py      # H标识检测: YOLO→圆形底板精修
│   │   ├── visual_servoing.py       # IBVS视觉伺服: 降落 + 圆筒逼近双模式
│   │   ├── coverage_monitor.py      # 覆盖监控: 网格离散化 + 覆盖率计算
│   │   ├── coordinate_transforms.py # 姿态补偿坐标变换 (6-DoF射线-地面求交)
│   │   ├── camera.py                # 相机驱动 (SITL/GStreamer)
│   │   ├── models/                  # YOLO推理引擎 + 训练权重
│   │   │   ├── yolo_engine.py       # ONNX/TensorRT 推理后端
│   │   │   └── *.onnx               # 训练好的模型文件
│   │   └── core/                    # 纯算法 (无ROS依赖, 可独立测试)
│   │       └── __init__.py
│   ├── training/                    # 训练管线 (独立于ROS, PC上运行)
│   │   ├── dataset_config.yaml      # 训练数据配置
│   │   ├── train.py                 # YOLOv8 训练脚本
│   │   ├── export.py                # ONNX/TensorRT 导出脚本
│   │   └── data_generation/
│   │       └── generate_labels.py   # SDF→YOLO标签 自动生成
│   ├── config/
│   │   └── vision_params.yaml       # YOLO参数 + OpenCV后处理 + 伺服PID
│   └── package.xml
│
├── drone_servo/                     # 舵机投放控制
│   ├── drone_servo/
│   │   ├── __init__.py
│   │   ├── servo_controller.py      # GPIO/PWM 舵机控制接口
│   │   ├── drop_manager.py          # 投放逻辑管理 (单投/连投/状态监测)
│   │   └── servo_mock.py            # SITL 模拟舵机
│   ├── config/
│   │   └── servo_params.yaml
│   └── package.xml
│
├── drone_mission/                   # 任务编排层
│   ├── drone_mission/
│   │   ├── __init__.py
│   │   ├── mission_state_machine.py # 任务状态机核心
│   │   ├── states/                  # 各状态处理逻辑
│   │   │   ├── __init__.py
│   │   │   ├── state_init.py        # 初始化 & 注册 & 预热
│   │   │   ├── state_takeoff.py     # 起飞
│   │   │   ├── state_transit.py     # 跨区域飞行
│   │   │   ├── state_cylinder_search.py    # 投放区搜索 & 选筒
│   │   │   ├── state_cylinder_approach.py  # 逼近圆筒圆心
│   │   │   ├── state_drop.py        # 触发投放
│   │   │   ├── state_recon_patrol.py       # 侦察区全覆盖巡视
│   │   │   ├── state_recon_inspect.py      # 危险品分类确认 (已废弃, 不再使用)
│   │   │   ├── state_return.py      # 返航
│   │   │   └── state_precision_land.py    # 视觉精准降落
│   │   ├── path_planner.py          # 路径规划 (覆盖路径+过渡路径)
│   │   └── cylinder_selector.py     # 圆筒选择策略
│   ├── config/
│   │   └── mission_params.yaml
│   ├── launch/
│   │   └── mission.launch.py        # 主launch文件
│   └── package.xml
│
├── drone_bringup/                   # 启动与集成
│   ├── launch/
│   │   ├── sitl.launch.py           # SITL 全系统启动
│   │   ├── real_flight.launch.py    # 实飞启动
│   │   └── vision_only.launch.py    # 仅视觉模块 (调试用)
│   ├── scripts/
│   │   ├── start_sitl.sh            # SITL 一键启动
│   │   └── calibrate_camera.sh      # 相机标定脚本
│   └── package.xml
│
└── drone_utils/                     # 通用工具
    ├── drone_utils/
    │   ├── __init__.py
    │   ├── logger.py                # 任务日志 (记录每次飞行)
    │   ├── watchdog.py              # 看门狗/心跳监测
    │   ├── emergency.py             # 紧急处理 (失控保护)
    │   └── config_loader.py         # 统一配置加载
    └── package.xml
```

---

## 四、坐标变换体系

### 4.1 坐标系定义

| 坐标系 | 原点 | X轴 | Y轴 | Z轴 | 用途 |
|--------|------|-----|-----|-----|------|
| World (Gazebo) | 场地左下(0,0,0) | 东 (70m) | 北 (8m) | 上 | 仿真世界 |
| NED Global | 同World | 北 (World +y) | 东 (World +x) | 下 (World -z) | PX4导航 |
| NED Local | H点 (10,4,0) | 北 (相对H) | 东 (相对H) | 下 (-z=向上) | 控制器航点 |
| Body (FRD) | 无人机质心 | 前 (机头) | 右 | 下 | 机体 |
| Camera | 相机光心 | 右 | 下 | 前 (光轴) | 视觉 |

### 4.2 变换链

```
World(x_east, y_north, z_up)
    ↕
    NED_x = World_y - 4.0              # 相对H点
    NED_y = World_x - 10.0
    NED_z = -World_z
    ↕
PX4 NED (vehicle_odometry.position)
    ↕
    Body_FRD = R_NED_to_Body * (NED - NED_drone)
    其中 R_NED_to_Body 由 vehicle_attitude.q 构造
    ↕
    Pixel(u,v) = K * [R_cam_to_body | t_cam_to_body] * [X_body; 1]
    其中 K 为相机内参矩阵 (需标定)
```

### 4.3 姿态补偿像素→世界变换 (v2)

视觉检测得到的像素坐标 → 世界坐标，使用完整的 6-DoF 姿态补偿（`coordinate_transforms.py` 中 `CoordinateTransformer.pixel_to_world()`）：

```
1. Pixel(u,v) + 相机内参K⁻¹ → 相机坐标系射线方向 ray_cam
2. ray_cam × R_cam_to_body → 机体坐标系射线 ray_body
   (相机安装: 垂直向下, 相机x→body右, 相机y→body前, 光轴→body下)
3. ray_body × R_body_to_world(drone_attitude_q) → 世界坐标系射线 ray_world
4. 射线与地面平面(z=0)求交: t = -drone_z / ray_world_z
   → P_ground = drone_pos + t * ray_world
```

**与简化版的精度对比**:

| 方法 | 5m高度+5°倾斜误差 | 适用场景 |
|------|-------------------|---------|
| 简化版 `dx = pixel * alt / f` | ~0.44m | 无姿态数据时回退 |
| 姿态补偿 `pixel_to_world()` | <0.05m | 正常飞行 (推荐) |

**回退策略**: 当 `/drone/pose` 话题不可用时，自动回退到简化版 `pixel_to_world_simple()`.

---

## 五、任务状态机设计

### 5.1 状态转换图

```
                      ┌─────────┐
                      │  INIT   │ 初始化 & 等待数据
                      └────┬────┘
                           │ odom稳定 + 注册完成
                      ┌────▼────┐
                      │  ARM    │ 预热setpoint流 + 切换Offboard + 解锁
                      └────┬────┘
                           │ 解锁成功
                      ┌────▼────┐
                      │ TAKEOFF │ 垂直起飞至巡航高度(5m)
                      └────┬────┘
                           │ 到达巡航高度
              ┌────────────▼────────────┐
              │  TRANSIT_TO_DROP_ZONE   │ 飞向投放区上方
              └────────────┬────────────┘
                           │ 到达投放区 (40~45, y=4)
              ┌────────────▼────────────┐
              │  CYLINDER_SEARCH        │ 悬停/慢速飞行 + 下视搜索圆筒
              └────────────┬────────────┘
                           │ 检测到3个圆筒
              ┌────────────▼────────────┐
              │  SELECT_TARGET          │ 选择半径最小的圆筒 (C_min)
              └────────────┬────────────┘
                           │ 选定目标筒
              ┌────────────▼────────────┐
              │  APPROACH_CYLINDER       │ 水平移动至目标圆筒正上方
              └────────────┬────────────┘
                           │ 到达正上方 (误差 < tolerance)
              ┌────────────▼────────────┐
              │  DESCEND_TO_DROP        │ 降高至投放高度 (约2m)
              └────────────┬────────────┘
                           │ 到达投放高度
              ┌────────────▼────────────┐
              │  EXECUTE_DROP           │ 触发舵机投放 + 确认投放完成
              └────────────┬────────────┘
                           │ 投放完成
              ┌────────────▼────────────┐
              │  ASCEND                 │ 回升至巡航高度
              └────────────┬────────────┘
                           │ 到达巡航高度
                           ├── 投放次数 < 2: 回到 SELECT_TARGET (选另一个)
                           │
                           │ 投放次数 = 2:
              ┌────────────▼────────────┐
              │  TRANSIT_TO_RECON_ZONE  │ 飞向侦察区
              └────────────┬────────────┘
                           │ 到达侦察区 (65~70, y=4)
              ┌────────────▼────────────┐
              │  RECON_PATROL           │ 弓字形全覆盖巡视
              │                         │ 降低高度扫描 + 实时覆盖监控
              └────────────┬────────────┘
                           │ 默认航点完成; 覆盖率用于验证相机观察
              ┌────────────▼────────────┐
              │  RETURN_TO_HOME         │ 返航至H点上方 (5m)
              └────────────┬────────────┘
                           │ 到达H点上方
              ┌────────────▼────────────┐
              │  PRECISION_LAND         │ 视觉识别H标识 + 视觉伺服对心 + 下降
              └────────────┬────────────┘
                           │ 着陆完成 + 上锁
                      ┌────▼────┐
                      │COMPLETE │ 任务完成，等待指令
                      └─────────┘
```

### 5.2 异常处理状态

每个飞行状态均可因以下条件跳转到异常处理：

- **TIMEOUT**: 单状态超时 → 重试一次 → 仍超时则任务中止
- **BATTERY_LOW**: 电量低于阈值 → 立即返航降落
- **VISION_FAIL**: 视觉连续N帧无结果 → 切换到盲降模式
- **OFFBOARD_LOST**: Offboard连接断开 → PX4自动进入故障保护 (LOITER)

---

## 六、模块详细设计

### 6.1 px4_control: 飞行控制抽象层

**核心类: `PX4Bridge`**  
- 管理外部组件注册 (4步握手)  
- 10Hz 心跳定时器 (OffboardControlMode + TrajectorySetpoint)  
- 20Hz 控制循环定时器  
- 提供 `set_position_target(north, east, down, yaw)` 接口  
- 提供 `set_velocity_target(vx, vy, vz, yaw_rate)` 接口（用于视觉伺服）  
- 注入 setpoint 斜坡平滑 (ramped setpoint)

**核心类: `FlightController`**  
```python
class FlightController:
    """飞行控制高级接口"""

    def takeoff(altitude: float) -> bool: ...
    def goto_ned(north: float, east: float, down: float, yaw: float, speed: float) -> bool: ...
    def goto_world(x: float, y: float, z: float, yaw: float, speed: float) -> bool: ...
    def loiter(duration: float) -> bool: ...
    def set_velocity_ned(vn: float, ve: float, vd: float, yaw_rate: float) -> None: ...
    def land() -> bool: ...
    def force_disarm() -> None: ...
    def reached(target: tuple, tolerance: float) -> bool: ...
```

**QoS 配置**: Best-effort + Transient-local + depth=10  
**关键**: `OffboardControlMode + TrajectorySetpoint` 必须以 >2Hz 持续发布

### 6.2 drone_vision: 视觉处理管线 (YOLO + OpenCV 混合)

#### 6.2.0 架构决策: 为什么使用 YOLO + OpenCV

| 维度 | 传统 CV (HoughCircles + HSV) | YOLO + OpenCV (当前方案) |
|------|------------------------------|--------------------------|
| 检测鲁棒性 | 光照一变就崩, 参数需逐场景调整 | YOLO 泛化强, 仿真→实飞几乎不需调参 |
| 分类能力 | 颜色分割受阴影干扰 | YOLO 直接输出类别 + 置信度 |
| 多目标 | 三个独立 pipeline | 单模型同时输出 6 类目标 |
| Jetson 推理 | ~30ms (CPU) | ~8-15ms (TensorRT FP16) |
| 开发效率 | 每个检测器手工设计特征 | 标注数据 → 训练 → 导出 → 部署 |

**核心思路**: YOLO 负责鲁棒的粗定位 (bbox)，OpenCV 在 bbox 内做亚像素级精确拟合 (圆筒椭圆/H 圆形底板)。YOLO 置信度低时用 HSV 颜色回退验证。

#### 6.2.1 统一视觉管线 (`vision_pipeline.py`)

**推荐部署模式**: 单一 ROS2 节点，一次 YOLO 前向传播，多任务后处理。

```
YOLOv8n (640×640, 6类)
    │ ONNX / TensorRT
    ▼
┌─────────────────────────────────────────────┐
│              vision_pipeline.py              │
│                                              │
│  cylinder dets  → CylinderRefiner (椭圆拟合) │
│                  → CylinderTracker (多帧ID)  │
│                  → 姿态补偿 World 坐标        │
│                  → /drone/vision/cylinders   │
│                                              │
│  landing_H dets → LandingPadRefiner (圆拟合) │
│                  → 姿态补偿 World 坐标        │
│                  → /drone/vision/landing_target│
│                                              │
│  无人机位姿      → CoverageMonitor            │
│                  → /drone/vision/coverage    │
└─────────────────────────────────────────────┘
```

**YOLO 6 类输出**:

> **注意**: 当前侦察任务仅使用 cylinder(0) 和 landing_H(4) 两类。危险品类别 (1-3,5) 在主线视觉管线中已禁用，仅在 `hazard_classifier` 独立节点中可选使用。

| ID | 类名 | 含义 | 检测高度范围 | 状态 |
|----|------|------|-------------|------|
| 0 | cylinder | 白色圆筒 | 1m-10m | ✅ 使用中 |
| 1 | flammable | 易燃标识 (橙色) | 1m-3m | ⚠️ 已禁用 |
| 2 | health_hazard | 健康危害标识 (蓝色) | 1m-3m | ⚠️ 已禁用 |
| 3 | danger | 危险标识 (红色) | 1m-3m | ⚠️ 已禁用 |
| 4 | landing_H | H 降落标识 | 0.5m-8m | ✅ 使用中 |
| 5 | empty_cylinder | 空圆筒 (无危险品) | 1m-3m | ⚠️ 已禁用 |

#### 6.2.2 高度自适应

`coordinate_transforms.py` 中的 `AltitudeAdaptiveParams` 根据飞行高度动态调整参数:

```python
adaptive = AltitudeAdaptiveParams(focal_length_px=400, base_altitude=5.0)

# 高度↑ → 目标像素变小 → 降低置信度阈值
params = adaptive.get_cylinder_params(altitude_m=10.0)
# → {"min_radius_px": 3, "conf_scale": 0.5, "effective_conf_threshold": 1.0}

params = adaptive.get_cylinder_params(altitude_m=2.0)
# → {"min_radius_px": 15, "conf_scale": 2.5, "effective_conf_threshold": 0.2}
```

#### 6.2.3 圆筒检测 + OpenCV 精修 (`cylinder_detector.py`)

```
YOLO 检出 "cylinder" bbox
    ↓
CylinderRefiner.refine(image, bbox):
  ROI 提取 → GaussianBlur → Canny Edge
  → findContours → fitEllipse (每个轮廓)
  → 评分: 面积 × 圆形度 × 白色验证
  → 最优椭圆: (cx, cy, radius_px, confidence)
    ↓
像素半径 → 真实半径: r_real = r_px * altitude / focal_length
    ↓
CylinderTracker: IoU 多帧匹配 → 稳定 ID
```

#### 6.2.4 危险品分类 (`hazard_classifier.py`) — ⚠️ 已废弃

> **状态变更 (2026-07-07)**: 侦察区任务简化为纯全覆盖路径巡视。危险品识别分类功能已从主线视觉管线中移除。`hazard_classifier.py` 保留供独立使用，`vision_pipeline.py` 中的 `_process_hazards()` 调用已禁用。

<details>
<summary>原始设计 (点击展开)</summary>

```
YOLO 检出 flammable/health_hazard/danger/empty_cylinder
    ↓
if confidence < 0.7:
    ColorVerifier.verify(image, bbox, hazard_type):
      ROI → HSV → color_mask
      → 颜色匹配像素占比 > 阈值? → 确认/拒绝/纠错
    ↓
发布 HazardDetection (关联到最近圆筒)
```

</details>

#### 6.2.5 H 降落标识 (`landing_detector.py`)

```
YOLO 检出 "landing_H" bbox
    ↓
LandingPadRefiner.refine(image, bbox):
  ROI → 自适应阈值 → 形态学清理
  → 找最大圆形轮廓 → minEnclosingCircle
  → 圆形度验证 > 0.6
  → PCA 分析 H 形状主方向 → 偏航修正角
    ↓
姿态补偿 → World 坐标
```

#### 6.2.6 视觉伺服 (`visual_servoing.py`)

双模式 PID 控制器:

| 模式 | 用途 | 参数特点 |
|------|------|---------|
| `LandingApproach` | H 标识对心 + 下降 | 死区 3cm, 下降速度 0.3m/s |
| `CylinderApproach` | 圆筒对心 (不下滑) | 死区 5cm, KP 更激进 |

```python
# 降落模式
servo = VisualServoController(kp=0.5, ki=0.02, kd=0.1)
state = servo.update(target_u, target_v, img_cx, img_cy, altitude, fx, fy)
if state.is_converged:
    # 视觉对心完成

# 圆筒逼近模式
approach = CylinderApproachController(kp=0.8, ki=0.03, kd=0.15)
approach.activate(cylinder_id=target_id)
state = approach.update(cylinder_u, cylinder_v, img_cx, img_cy, altitude, fx, fy)
```

#### 6.2.7 覆盖监控 (`coverage_monitor.py`)

不变——侦察区离散化网格 + FOV 覆盖标记 + 覆盖率阈值判定。

#### 6.2.8 无 YOLO 回退模式

当 `model_path` 参数为空或模型加载失败时，自动启用纯 OpenCV 回退:
- 圆筒: `HoughCircles` + 白色 HSV 验证
- 危险品: HSV 颜色分割 + 轮廓筛选
- 降落: 自适应阈值 + 圆形检测

回退模式延迟 ~30ms，精度比 YOLO 低但保证系统可运行。

### 6.3 drone_servo: 舵机控制

```python
class ServoController:
    """舵机投放控制器"""
    def __init__(self, pwm_pin: int, platform: str = 'jetson'):
        # Jetson: 使用 Jetson.GPIO 或 PCA9685 PWM驱动板
        # SITL: 使用 ServoMock (仅记录日志)
        ...

    def configure(self, open_pulse_us: int, close_pulse_us: int): ...
    def open(self) -> bool: ...       # 打开投放机构
    def close(self) -> bool: ...      # 关闭投放机构
    def drop(self) -> bool: ...       # 一次完整投放动作 (open → 延时 → close)
    def is_ready(self) -> bool: ...   # 检查伺服状态
```

**SITL Mock**: 在SITL模式下，`ServoMock` 将所有操作记录到日志，并通过ROS2话题发布投放事件。

**投放确认策略**:
- 发送投放指令后，等待 1s (机械动作完成)
- 可选: 通过视觉确认 (投放前后图像对比, 判断圆筒内是否有变化)
- 投放成功/失败通过 `TriggerDrop.srv` 的服务响应返回

### 6.4 drone_mission: 任务编排

#### 6.4.1 圆筒选择策略 (`cylinder_selector.py`)

```
输入: CylinderArray (3个检测到的圆筒, 各有半径估计)
输出: 选中的目标圆筒

策略优先级:
  1. 按半径升序排序: [Ø15cm (最小), Ø20cm, Ø25cm (最大)]
  2. 第一轮选择: 半径最小者 (Ø15cm)
  3. 第二轮选择: 半径次小者 (Ø20cm)
  4. 约束: 不得重复选择同一圆筒
  5. 异常: 若半径估计置信度不足, 降高至2m重新检测确认
```

#### 6.4.2 覆盖路径规划 (`path_planner.py`)

**区域2侦察覆盖** (NED: north 8m × east 5m = 40m²):

使用**优化后的弓字形 (Boustrophedon) 全覆盖路径**，自动选择最优条带方向：

```
算法: 自动方向选择
  1. 比较区域长宽: north_range(8m) vs east_range(5m)
  2. 沿长边飞行 (north, 8m/条), 条带沿短边分布 (east, 5条带 @1m间距)
  3. 转弯次数: 5条带 × 2 = 10 端点 (vs 沿短边飞: 8×2=16)
  4. 总飞行距离: 5条带 × 8m = 40m

弓字形参数 (可配置):
  - grid_spacing: 1.0m (条间距)
  - sweep_speed: 0.8 m/s (巡航速度)
  - 高度 2.5m + FOV 35°半角 → 地面覆盖半径 ~1.75m → 1m间距确保充分重叠
  - 预计巡视时间: ~50s (不含起降和中转)

路径示例 (NED坐标, Area 2: N[-4,4] E[55,60]):
  Strip 0: (north=-4, east=55) → (north= 4, east=55)  // 向北
  Strip 1: (north= 4, east=56) → (north=-4, east=56)  // 向南
  Strip 2: (north=-4, east=57) → (north= 4, east=57)  // 向北
  Strip 3: (north= 4, east=58) → (north=-4, east=58)  // 向南
  Strip 4: (north=-4, east=59) → (north= 4, east=59)  // 向北
```

**关键设计决策**: 纯路径覆盖, 不进行圆筒悬停或危险品检测。侦察区默认必须完成弓字形路径, 确保下视相机按路径观察完整区域；覆盖率由 `CoverageMonitor` 实时跟踪, 作为相机视场覆盖验证指标。若设置 `recon_require_full_path=false`, 可恢复覆盖率≥95%提前返航。

#### 6.4.3 状态机核心框架

```python
class MissionStateMachine:
    """
    使用分层状态机:
    - Super State: 任务阶段 (PHASE_DROP, PHASE_RECON, PHASE_RETURN)
    - Sub State: 阶段内具体步骤
    - 每个状态实现 enter() / execute() / exit() / check_timeout()
    - 状态转换通过 transition table 明确声明
    """

    # 状态转移表: (当前状态, 条件) → 下一状态
    TRANSITIONS = {
        State.INIT:              [(Condition.ODOM_READY, State.ARM)],
        State.ARM:               [(Condition.ARMED, State.TAKEOFF)],
        State.TAKEOFF:           [(Condition.AT_ALTITUDE, State.TRANSIT_DROP)],
        State.TRANSIT_DROP:      [(Condition.AT_ZONE, State.CYL_SEARCH)],
        State.CYL_SEARCH:        [(Condition.CYLINDERS_DETECTED, State.SELECT_TARGET)],
        State.SELECT_TARGET:     [(Condition.TARGET_SET, State.APPROACH)],
        State.APPROACH:          [(Condition.OVER_TARGET, State.DESCEND)],
        State.DESCEND:           [(Condition.AT_DROP_ALT, State.EXECUTE_DROP)],
        State.EXECUTE_DROP:      [(Condition.DROP_DONE, State.ASCEND)],
        State.ASCEND:            [(Condition.AT_CRUISE,
                                   State.SELECT_TARGET if drops < 2
                                   else State.TRANSIT_RECON)],
        State.TRANSIT_RECON:     [(Condition.AT_ZONE, State.RECON_PATROL)],
        State.RECON_PATROL:      [(Condition.COVERAGE_COMPLETE, State.RETURN_HOME)],
        State.RETURN_HOME:       [(Condition.AT_HOME, State.PRECISION_LAND)],
        State.PRECISION_LAND:    [(Condition.LANDED, State.COMPLETE)],
        # 异常转移 (任意状态)
        '*': {
            Condition.BATTERY_LOW:    State.RETURN_HOME,
            Condition.TIMEOUT:        State.ERROR_RECOVERY,
            Condition.CRITICAL_ERROR: State.COMPLETE,
        }
    }
```

---

## 七、数据流设计

### 7.1 ROS2 话题一览

| 话题 | 类型 | 方向 | 频率 | 说明 |
|------|------|------|------|------|
| `/fmu/out/vehicle_odometry` | VehicleOdometry | PX4→Ctrl | 50Hz | 位置速度姿态 (NED) |
| `/fmu/out/vehicle_status` | VehicleStatus | PX4→Ctrl | 5Hz | 解锁/导航状态 |
| `/fmu/out/vehicle_land_detected` | VehicleLandDetected | PX4→Ctrl | 5Hz | 着陆检测 |
| `/fmu/out/vehicle_attitude` | VehicleAttitude | PX4→Ctrl | 50Hz | 姿态四元数 |
| `/fmu/in/offboard_control_mode` | OffboardControlMode | Ctrl→PX4 | 10Hz | 控制模式声明 |
| `/fmu/in/trajectory_setpoint` | TrajectorySetpoint | Ctrl→PX4 | 10Hz | 位置/速度指令 |
| `/fmu/in/vehicle_command` | VehicleCommand | Ctrl→PX4 | 按需 | 模式/解锁/Land |
| `/drone/pose` | PoseStamped | Ctrl→Vision | 50Hz | **v2新增**: 姿态补偿用 World 位姿 |
| `/drone/camera/image_raw` | Image | Camera→Vision | 30Hz | 原始图像 |
| `/drone/camera/camera_info` | CameraInfo | Camera→Vision | 静态 | 相机内参 |
| `/drone/vision/cylinders` | CylinderArray | Vision→Mission | 10Hz | 圆筒检测结果 |
| `/drone/vision/hazards` | HazardArray | Vision→Mission | (已禁用) | 危险品检测结果 (主线已停用) |
| `/drone/vision/landing_target` | LandingTarget | Vision→Mission | 20Hz | 降落目标定位 |
| `/drone/vision/coverage` | CoverageStatus | Vision→Mission | 2Hz | 侦察区覆盖进度 |
| `/drone/servo/velocity_cmd` | TwistStamped | Vision→Ctrl | 20Hz | 视觉伺服速度指令 |
| `/drone/servo/status` | ServoStatus | Servo→Mission | 1Hz | 舵机状态 |

### 7.2 ROS2 服务

| 服务 | 调用方 | 提供方 | 说明 |
|------|--------|--------|------|
| `/drone/servo/trigger_drop` | Mission | Servo | 触发投放 |
| `/drone/vision/start_cylinder_detect` | Mission | Vision | 启用圆筒检测 |
| `/drone/vision/start_landing_detect` | Mission | Vision | 启用H检测 |

### 7.3 节点图

```
┌─────────────────┐     ┌─────────────────┐
│  Camera Node    │────→│ Vision Node     │
│  (camera_node)  │ Img │ (detector)      │
└─────────────────┘     └────────┬────────┘
                                 │ CylinderArray
                                 │ LandingTarget
                                 │ CoverageStatus
                                 ▼
┌─────────────────┐     ┌─────────────────┐
│  PX4 Bridge     │←───→│ Mission FSM     │───→ Servo Node
│  (px4_bridge)   │     │ (state_machine) │
└────────┬────────┘     └─────────────────┘
         │ Odometry
         │ Status
         ▼
   PX4 (via DDS)
```

---

## 八、精准降落算法详细

### 8.1 阶段划分

**Phase 1: 粗定位** (H点上方 5m)
- 无人机已飞至 H 点 (NED: 0, 0, -5)
- 启动下视相机，搜索 landing pad 圆形轮廓
- 若检测到，获取初步圆心估计

**Phase 2: 对心下降** (5m → 0.5m)  ← 视觉伺服核心
```
Loop (每 0.05s):
  img = camera.capture()
  target = landing_detector.detect(img)
  if target.confidence > threshold:
      error_px = (target.center_u - img_center_u,
                  target.center_v - img_center_v)
      error_world = pixel_to_world_delta(error_px, altitude)
      set_velocity_ned( -k * error_world.north,
                        -k * error_world.east,
                        -descent_speed,
                        0.0 )
  else:
      # 视觉丢失: 保持水平位置, 继续下降
      set_velocity_ned(0.0, 0.0, -descent_speed, 0.0)
```

**Phase 3: 盲降** (< 0.5m)
- 视觉可能因过近而失效
- 保持水平位置不变 (惯性)
- 以最低速率下降
- land_detected → 停止Offboard → disarm

### 8.2 精度预期与退化策略

| 场景 | 预期精度 | 策略 |
|------|---------|------|
| 视觉正常, 光照良好 | <10cm | 视觉伺服全流程 |
| 视觉偶尔丢帧 | <30cm | 保持上次修正量, 持续下降 |
| 视觉长时间丢失 | <50cm | GPS辅助 + H点已知坐标 |
| 视觉完全不可用 | ~1m | 纯GPS降落 (PX4 LAND模式备选) |

---

## 九、训练管线

### 9.1 数据来源

| 来源 | 方法 | 标注方式 | 数量建议 |
|------|------|---------|---------|
| Gazebo 仿真 | `generate_labels.py` 从 SDF 自动生成 YOLO 标签 | 自动 (已知世界坐标→投影) | 500-1000 张 |
| Gazebo 截图 | 批量放置虚拟相机 + `gz camera` 截图 | 自动 | 配合 labels |
| 实飞录像 | `ros2 bag record /drone/camera/image_raw` | LabelImg 手动标注 | 200-500 张 |
| 混合 | 仿真预训练 + 真实微调 | 自动 + 手动 | **推荐** |

### 9.2 训练流程

```bash
cd src/drone_vision/training/

# Step 1: 从仿真生成标签 (自动, 无需图像)
python data_generation/generate_labels.py \
    --sdf ../../field/drone_field_random.sdf \
    --output dataset_sim/ --samples 1000

# Step 2: 在 Gazebo 中批量截图 (放入 dataset_sim/images/)

# Step 3: 训练
python train.py --data dataset_sim/data.yaml --model yolov8n.pt --epochs 200

# Step 4: (可选) 真实数据微调
# 先用 LabelImg 标注 300+ 张真实图像
python train.py --data dataset_real/data.yaml \
    --model runs/drone/train/weights/best.pt \
    --epochs 50 --lr0 0.0005

# Step 5: 导出 ONNX → 复制到 models/
python export.py --weights runs/drone/train/weights/best.pt --format onnx --half
cp runs/drone/train/weights/best.onnx ../models/drone_yolo.onnx
```

### 9.3 数据增强

`train.py` 已内置适合无人机场景的增强参数:

| 增强 | 参数 | 理由 |
|------|------|------|
| 缩放 `scale=0.3` | 模拟 1m-10m 不同高度 |
| 旋转 `degrees=5` | 模拟 ±5° 姿态变化 |
| 亮度 `hsv_v=0.2` | 模拟不同时间/天气 |
| 禁止上下翻转 `flipud=0` | 无人机不会倒飞 |
| Mosaic `mosaic=0.5` | 小目标检测增强 |

---

## 十、SITL 仿真设计

### 10.1 Gazebo 相机仿真

在 PX4 x500 模型上挂载下视相机传感器：
```xml
<!-- 附加到 x500 机身底部 -->
<plugin name="downward_camera" filename="libgz-sim8-camera-system.so">
    <topic>/camera/image</topic>
    <width>640</width><height>480</height>
    <update_rate>30</update_rate>
    <pose>0 0 0 0 1.5708 0</pose>   <!-- 垂直向下 -->
</plugin>
```

通过 `ros_gz_bridge` 将 Gazebo 相机话题桥接到 ROS2:
```
ros2 run ros_gz_image image_bridge /camera/image /drone/camera/image_raw
```

### 10.2 投放机构仿真

在 Gazebo 中不需要物理仿真投放过程。`ServoMock` 记录投放事件并发布成功消息。

### 10.3 SITL 测试流程 (v2)

```bash
# 终端1: MicroXRCEAgent
MicroXRCEAgent udp4 -p 8888

# 终端2: PX4 + Gazebo (with camera sensor)
PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=drone_field_random \
PX4_GZ_MODEL_POSE="10,4,0.3,0,0,0" \
make px4_sitl gz_x500

# 终端3: ros_gz_image bridge
ros2 run ros_gz_image image_bridge /camera/image /drone/camera/image_raw

# 终端4: 全系统启动 (使用统一视觉管线)
ros2 launch drone_bringup sitl.launch.py

# 可选: 指定自定义 YOLO 模型
ros2 launch drone_bringup sitl.launch.py \
    model_path:=src/drone_vision/models/drone_yolo.onnx
```

---

## 十一、YAML 参数配置体系

> **核心设计原则**: 飞行参数与代码分离。所有可调参数集中在各包的 `config/*.yaml` 中，通过 launch 文件加载，无需重新编译即可调整飞行行为。这完全参考了 `Task_flight_ws/src/px4_control_dds/config/mission_params.yaml` 的成熟模式。

### 11.1 工作原理

```
config/*.yaml               launch/*.launch.py              Node 代码
─────────────────           ─────────────────────           ─────────
/node_name:                 parameters=[param_file]   →    self.declare_parameter('key', default)
  ros__parameters:                                         self.get_parameter('key').value
    key: value
```

三步走机制：

**Step 1 — YAML 定义** (`config/xxx_params.yaml`)：
```yaml
/node_name:
  ros__parameters:
    takeoff_height: 5.0
    sweep_speed: 0.8
    waypoint_tolerance: 0.5
```

**Step 2 — launch 加载** (`launch/*.launch.py`)：
```python
pkg_share = get_package_share_directory('package_name')
param_file = os.path.join(pkg_share, 'config', 'xxx_params.yaml')

Node(
    package='...', executable='...', name='node_name',
    parameters=[param_file],
)
```

**Step 3 — 节点读取**：
```python
self.declare_parameter('takeoff_height', 5.0)    # 声明 + 设默认值
h = self.get_parameter('takeoff_height').value    # 读取 (YAML覆盖默认值)
```

### 11.2 配置 YAML 文件清单

每个需要运行时调整的包配备一个 YAML：

| 包 | YAML 文件 | 命名空间 | 内容 |
|----|-----------|----------|------|
| `px4_control` | `config/px4_params.yaml` | `/px4_bridge` | 飞行控制基础参数 (高度/速度/容差) |
| `drone_vision` | `config/vision_params.yaml` | `/vision_pipeline` | 相机内参、检测阈值、视觉伺服增益 |
| `drone_servo` | `config/servo_params.yaml` | `/servo_controller` | PWM参数、投放时序、平台选择 |
| `drone_mission` | `config/mission_params.yaml` | `/mission_state_machine` | 区域坐标、弓字形参数、任务策略 |

### 11.3 setup.py 安装 config 和 launch

关键点：`data_files` 中必须声明 config 和 launch 目录，否则 `colcon build` 不会将它们复制到 `install/` 目录：

```python
from glob import glob

setup(
    name='package_name',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ↓ 参数文件 — 必须声明否则 install/ 里没有
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # ↓ launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
)
```

### 11.4 使用方式

```bash
# 一键启动 (YAML 参数自动加载)
ros2 launch drone_bringup sitl.launch.py

# 调整参数无需重新编译 — 直接改 YAML 再启动
vim src/drone_mission/config/mission_params.yaml  # 改 sweep_speed: 0.8 → 1.5
ros2 launch drone_bringup sitl.launch.py           # 速度变为 1.5m/s

# 命令行临时覆盖单个参数
ros2 run drone_mission mission_fsm --ros-args -p sweep_speed:=1.5
```

---

## 十二、实飞部署考量

### 12.1 Jetson Orin Nano 软件环境
- OS: JetPack 6.x (Ubuntu 22.04 based)
- ROS2 Humble (ARM64)
- OpenCV 4.5+ (CUDA accelerated)
- 相机驱动: GStreamer + nvarguscamerasrc (MIPI CSI) 或 v4l2 (USB)

### 12.2 通信方式选择
- **首选**: micro-XRCE-DDS over Serial (UART) — 与SITL架构一致
- **备选**: MAVLink over Serial + pymavlink — 若DDS在Jetson上有兼容性问题

### 12.3 从SITL到实飞的切换点
- `px4_bridge.py`: 初始化时检测 `platform` 参数，切换QoS配置
- `camera_node.py`: 根据 `sim_mode` 参数选择 Gazebo bridge 或 GStreamer pipeline
- `servo_controller.py`: 根据 `platform` 参数选择真实GPIO或Mock
- 其余模块代码**完全不变**

---

## 十三、测试策略

### 13.1 分级测试

| Level | 环境 | 测试内容 | 通过标准 |
|-------|------|---------|---------|
| L0: 单元测试 | pytest | 各模块独立功能 (坐标变换、检测算法、状态转移) | >80% 覆盖率 |
| L1: 组件测试 | ROS2 + 录制数据 | 视觉检测回放 (rosbag) | 检测率>90% |
| L2: SITL集成 | Gazebo + PX4 | 完整任务飞行 | 连续3次成功 |
| L3: HITL | 飞控硬件 + 仿真视觉 | 飞行控制+伺服 | 投放精度<15cm |
| L4: 实飞 | 实际场地 | 完整任务 | 任务完成率>95% |

### 13.2 SITL 关键验证项

1. 圆筒随机位置下视觉正确检测3个筒并排序
2. 最小半径选择+投放 → 次小半径选择+投放
3. 侦察区弓字形全覆盖巡视 (覆盖率≥95%)
4. H标识视觉识别+精准降落 (<30cm 精度)
5. 异常注入测试: 视觉丢失、GPS漂移、超时

---

## 十四、项目文件映射 (v2 更新)

| 文件 (drone_tasks_ws/src/) | 状态 | 行数 | 对应功能 |
|---------------------------|------|------|---------|
| `px4_control/px4_bridge.py` | 桩代码 | ~30 | PX4 DDS通信桥接 + NED→World pose发布 |
| `px4_control/flight_controller.py` | 桩代码 | ~20 | 飞行控制接口 |
| `px4_control/telemetry.py` | 桩代码 | ~10 | 遥测数据模型 |
| `drone_vision/models/yolo_engine.py` | ✅ 已实现 | ~320 | ONNX/TensorRT YOLO推理引擎 |
| `drone_vision/vision_pipeline.py` | ✅ 已实现 | ~480 | 统一视觉管线 (单模型多任务) |
| `drone_vision/cylinder_detector.py` | ✅ 已实现 | ~320 | 圆筒检测: YOLO→椭圆拟合→追踪 |
| `drone_vision/hazard_classifier.py` | ⚠️ 已废弃 | ~220 | 危险品分类 (已从主线禁用, 可独立使用) |
| `drone_vision/landing_detector.py` | ✅ 已实现 | ~250 | H降落: YOLO→圆形底板→PCA方向 |
| `drone_vision/visual_servoing.py` | ✅ 已实现 | ~280 | IBVS: 降落 + 圆筒逼近双模式 |
| `drone_vision/coverage_monitor.py` | ✅ 已实现 | ~180 | 覆盖监控 |
| `drone_vision/coordinate_transforms.py` | ✅ 已实现 | ~250 | 姿态补偿坐标变换 + 高度自适应 |
| `drone_vision/camera.py` | ✅ 已实现 | ~180 | 相机驱动 (SITL/GStreamer) |
| `drone_vision/training/generate_labels.py` | ✅ 已实现 | ~200 | SDF→YOLO标签自动生成 |
| `drone_vision/training/train.py` | ✅ 已实现 | ~80 | YOLOv8 训练 |
| `drone_vision/training/export.py` | ✅ 已实现 | ~50 | ONNX/TensorRT 导出 |
| `drone_servo/servo_controller.py` | 桩代码 | ~30 | 舵机控制 |
| `drone_servo/servo_mock.py` | 桩代码 | ~10 | SITL模拟 |
| `drone_servo/drop_manager.py` | 桩代码 | ~10 | 投放管理 |
| `drone_mission/mission_state_machine.py` | 桩代码 | ~30 | FSM核心 |
| `drone_mission/states/*.py` | 桩代码 | ~10×10 | 10个状态 |
| `drone_mission/path_planner.py` | 桩代码 | ~15 | 路径规划 |
| `drone_mission/cylinder_selector.py` | 桩代码 | ~10 | 选筒策略 |
| `drone_utils/*.py` | 桩代码 | ~40 | 日志/看门狗/配置/紧急 |
| **总计** | | **~3000** |

---

## 十五、验证方案

### SITL 验证步骤

1. `colcon build && source install/setup.bash`  
2. `ros2 launch drone_bringup sitl.launch.py`  
3. 观察 Gazebo 中无人机自主完成任务全流程  
4. 检查 `/tmp/drone_mission_logs/` 下的任务日志  
5. 验证: 投放精度 < 15cm, 侦察覆盖率 > 95%, 降落精度 < 30cm

### 视觉模块独立验证

```bash
# 使用rosbag回放验证视觉检测
ros2 launch drone_bringup vision_only.launch.py
ros2 bag play recorded_field.bag
ros2 topic echo /drone/vision/cylinders
ros2 topic echo /drone/vision/hazards
```

---

## 十六、实现顺序 (v2 更新)

遵循依赖链的严格构建顺序：

| 序号 | 包名 | 状态 | 依赖 | 关键产出 |
|------|------|------|------|---------|
| 1 | `drone_interfaces` | ✅ | 无 | 编译通过的自定义 msg/srv/action |
| 2 | `px4_msgs` | ✅ | 无 (软链接) | PX4 uORB 消息可用 |
| 3 | `drone_utils` | ⏳ | `drone_interfaces` | 配置加载/日志/看门狗/紧急处理 |
| 4 | `px4_control` | ⏳ | `px4_msgs`, `drone_utils` | DDS桥接 + /drone/pose 发布 + 飞行控制 |
| 5 | `drone_servo` | ⏳ | `drone_interfaces`, `drone_utils` | 舵机控制 + SITL Mock |
| 6 | `drone_vision` | ✅ | `drone_interfaces` | YOLO检测 + OpenCV精修 + IBVS伺服 (完整) |
| 7 | `drone_mission` | ⏳ | 以上全部 | FSM + 路径规划 + 视觉管线集成 |
| 8 | `drone_bringup` | ✅ | 以上全部 | `sitl.launch.py` 一键启动全系统 |

**图例**: ✅=已实现, ⏳=桩代码待实现

---

## 十七、关键设计决策速览 (v2)

| 决策 | 理由 |
|------|------|
| 7包非单体架构 | 独立测试、清晰职责、并行开发 |
| 视觉为独立节点 | 计算密集，需独立进程；Jetson上利用GPU不阻塞飞控 |
| YOLO + OpenCV 混合检测 | YOLO提供鲁棒粗定位，OpenCV做亚像素精修；泛化强于纯传统CV |
| 单模型6类多任务 | 一次推理 vs 三个独立模型 → 延迟降3x, 显存省3x |
| 姿态补偿坐标变换 | 5m+5°倾斜时误差从0.44m→<0.05m；/drone/pose不可用时回退简化版 |
| 高度自适应阈值 | 10m高空目标仅~7px, 必须降conf阈值；避免固定参数导致的漏检/误检 |
| 仿真预训练+真实微调 | 仿真零标注成本快速迭代；200-500张真实数据微调即可实飞 |
| 无YOLO回退模式 | HoughCircles+HSV保证模型不可用时系统仍可运行 |
| 指令用Service, 数据用Topic | 指令需要确认响应；视觉数据是持续流 |
| 抽象舵机硬件接口 | SITL(mock)与实飞(GPIO)使用完全相同的任务代码 |
| 斜坡Setpoint速度控制 | 已有验证模式，精确速度控制无需调PX4参数 |
| 坐标变换统一在drone_vision | 姿态补偿需相机内参，属视觉范畴；drone_utils提供基础工具 |
| 自动最优条带方向 | 沿区域长边飞行使转弯最少; Area 2 从16次转弯降至10次 |
| 覆盖率驱动返航 | 覆盖率≥95%触发返航, 不等候全部航点, 比固定超时更智能 |
| 侦察区纯路径覆盖 | 不执行危险品识别, 聚焦全覆盖巡视的路径算法 |
| 单模型6类 → 实际使用2类 | 训练6类保持模型通用性, 主线仅用 cylinder+landing_H |
| PID视觉伺服(双模式) | 积分项修正漂移；降落/圆筒逼近不同参数 |

---

## 十八、关键参考文件

本方案设计参考了以下文件中的已有模式和接口：

- `Task_flight_ws/src/px4_control_dds/px4_control_dds/mission_control_node_v2.py` — DDS 通信模式 (QoS/注册/Offboard心跳/斜坡setpoint/强制解锁/着陆策略)
- `PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml` — DDS 话题映射 (所有 /fmu/in/* 和 /fmu/out/* 话题)
- `drone_tasks_ws/field/drone_field.sdf` — Gazebo 世界定义 (场地布局/圆筒模型/危险品标识)
- `drone_tasks_ws/field/generate_random_field.py` — 随机场地生成器 (SITL 测试时作为视觉检测的 ground truth)
