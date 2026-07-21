# 真机飞行开发 — 工作空间组织方案

日期: 2026-07-17 | 状态: 规划中
前置文档: [SITL 审计 & 真机差距分析](2026-07-14-sitl-audit-and-real-hardware-gaps.md)

---

## 一、当前状态

7 包 ROS2 工作空间，约 8700 行 Python，SITL 全流程已验证通过。SITL → 真机"双路线"策略已有框架但未完成：

- `real_flight.launch.py` ✅ 存在（但过于简化）
- `camera.py` / `servo_controller.py` 通过 `sim_mode`/`platform` 参数做了双模式抽象 ✅
- **6 个硬阻塞项未解决** ❌（详见前置文档）

---

## 二、整体原则：不污染 SITL 主线

核心代码（状态机、视觉管线、飞控接口）**完全不变**。真机差异通过以下方式隔离：

1. **配置层覆盖** — 独立 YAML 参数文件，launch 时覆盖 SITL 默认值
2. **独立启动脚本** — `start_real_flight.sh`，不修改 `start_full_simulation.sh`
3. **独立日志目录** — 不混入 `log/simulation_*`
4. **平台分支代码** — 仅在已有 `platform`/`sim_mode` 分支处补齐缺失实现

---

## 三、推荐的工作空间布局

```
drone_tasks_ws/
├── src/                            # 不变：核心代码共用
│   └── (7 个 ROS2 包)
├── docs/                           # 不变
├── field/                          # 不变 (SITL 用)
├── scripts/
│   ├── start_full_simulation.sh    # 不变
│   ├── debug_vision.sh             # 不变
│   └── start_real_flight.sh        # ⬅ 新建：真机一键启动
├── config_real/                    # ⬅ 新建：真机专属配置文件目录
│   ├── vision_params_real.yaml     #   真机相机内参 (标定值)
│   ├── servo_params_real.yaml      #   platform: jetson + GPIO 引脚
│   └── px4_params_real.yaml        #   (可选) 真机飞控参数覆盖
├── logs_real/                      # ⬅ 新建：真机飞行日志 (gitignore)
├── rosbags/                        # ⬅ 可选：真机录制的 rosbag
├── launch_real/                    # ⬅ 新建：真机 launch 文件
│   └── real_flight.launch.py       #   完善版 (或直接放在 drone_bringup/launch/)
├── PX4-Autopilot -> /home/ryder/PX4-Autopilot  # 需要真机 airframe 配置
└── cv_trainer -> /home/ryder/cv_trainer
```

### 为什么不建议大规模重构

现有架构的 `platform`/`sim_mode` 模式已经能处理 90% 的 SITL/真机差异。真正需要的是**补齐缺失模块**，而非改动现有代码结构。

---

## 四、开发顺序：按依赖链分 4 个阶段

### 阶段 1：硬件驱动验证（最底层，先跑通）

| 序号 | 任务 | 涉及文件 | 说明 |
|------|------|---------|------|
| 1.1 | **相机标定** | `config_real/vision_params_real.yaml` | 真机跑棋盘格标定，拿到真实的 `fx/fy/cx/cy`，写入单独 YAML |
| 1.2 | **GStreamer 相机验证** | `camera.py`（小改） | `sim_mode: false` 路径已存在，需真机验证 GStreamer pipeline 正常读 CSI/USB 相机 |
| 1.3 | **舵机 GPIO 驱动实现** | `servo_controller.py:50-52` | `_create_real_servo()` 方法需**实现**：PCA9685 PWM 驱动板 或 Jetson.GPIO。当前仅有日志输出，无实际驱动代码 |
| 1.4 | **Serial 通信验证** | 启动脚本 | MicroXRCEAgent 从 UDP `-p 8888` 改为 Serial `/dev/ttyTHS1`。验证 `/fmu/out/vehicle_odometry` 等话题正常到达 |

**里程碑**: 真机上图像正常采集、舵机能动、PX4 通信正常。

---

### 阶段 2：飞控安全链路（最关键的 blocker）

| 序号 | 任务 | 涉及文件 | 说明 |
|------|------|---------|------|
| 2.1 | **标准 ARM 解锁** | `mission_state_machine.py:1190` | 当前硬编码 SITL force-magic `param2=21196`。真机需标准流程：arming check → preflight check → arm。在 `_StandaloneBridgeFacade.arm()` 中根据 `platform` 参数分支 |
| 2.2 | **GPS/电池遥测** | `px4_bridge.py:251-253` | 当前 `battery_remaining=1.0`, `gps_fix_type=0` 硬编码假值。需订阅 `/fmu/out/battery_status` 和 `/fmu/out/sensor_gps` |
| 2.3 | **PX4 Airframe 配置** | PX4-Autopilot 目录 | 为真机四旋翼创建 airframe 配置文件（电机混控、PID 参数、传感器朝向），禁用 SITL bypass 参数 |
| 2.4 | **紧急处理真机化** | `emergency.py` | 低电量 RTL、GPS 丢失 LOITER、Offboard 连接丢失降级策略。SITL 中宽松，真机必须严格 |

**里程碑**: 遥控器起飞、切 Offboard 模式、QGC 显示正确电池/GPS 数据。

---

### 阶段 3：任务逻辑真机适配

| 序号 | 任务 | 涉及文件 | 说明 |
|------|------|---------|------|
| 3.1 | **投放失败处理** | `mission_state_machine.py:965-970` | 当前服务不可用时**静默模拟成功**。真机需：真实等待舵机反馈 → 超时重试 → 视觉确认圆筒是否有变化 → 失败时跳过当前目标而非静默通过 |
| 3.2 | **视觉模型 TensorRT 转换** | `vision_pipeline.py` + `yolo_engine.py` | `.pt` → `.engine` (TensorRT FP16)。YAML 中 `device: "tensorrt"` 已声明，需确认 `yolo_engine.py` 的 TensorRT 后端在 Jetson 上正确加载 `.engine` 文件 |
| 3.3 | **高度自适应参数验证** | `coordinate_transforms.py` | `AltitudeAdaptiveParams` 基于仿真推导，真机需验证并微调 |
| 3.4 | **真机飞行参数标定** | `config_real/mission_params_real.yaml` | PX4 真机高度跟踪精度远好于 SITL（无 ~0.3m 稳态误差），可将 `precision_align_drop_height_tolerance` 从 0.40 调回 0.15-0.20。其他 `precision_align_*` 参数也需实测微调 |

**里程碑**: 地面测试完成投放完整流程，视觉精校在真机上收敛。

---

### 阶段 4：真机一键启动 & 日志

| 序号 | 任务 | 涉及文件 | 说明 |
|------|------|---------|------|
| 4.1 | **start_real_flight.sh** | `scripts/start_real_flight.sh` | 参考 `start_full_simulation.sh` 结构：清理旧进程 → Serial MicroXRCEAgent → GStreamer 相机 → ros2 launch。增加安全确认（"确认已上电?" 交互提示） |
| 4.2 | **real_flight.launch.py 完善** | `launch_real/real_flight.launch.py` | 现有版本过于简化（4 节点 + TimerAction）。需加入：日志目录创建、`config_real/` 路径、相机内参参数文件加载 |
| 4.3 | **真机日志系统** | `logs_real/` | 参考 `log/simulation_<timestamp>/` 模式。飞行后自动分析脚本 |
| 4.4 | **rosbag 自动录制** | launch 或脚本 | 自动录制 `/drone/camera/image_raw` + `/drone/vision/cylinders` + `/drone/pose`，便于离线回放调试 |

**里程碑**: 一条命令 `./scripts/start_real_flight.sh` 启动全系统，完成完整任务飞行。

---

## 五、配置文件双轨策略

### SITL 默认配置（不改动）

```
config/
├── mission_params.yaml      # 任务级参数
├── px4_params.yaml          # 飞控基础参数
├── vision_params.yaml       # 视觉检测参数（SITL 内参）
└── servo_params.yaml        # platform: sitl
```

### 真机覆盖配置（新建目录）

```
config_real/
├── vision_params.yaml       # 相机内参（标定后）+ device: tensorrt
├── servo_params.yaml        # platform: jetson + GPIO 引脚
└── px4_params.yaml          # (可选) 真机飞控参数覆盖
```

**加载顺序**: `real_flight.launch.py` 先加载 `config/` 默认值，再加载 `config_real/` 覆盖 — ROS2 parameter 机制后加载的覆盖先加载的。

### mission_params.yaml 注意事项

大部分任务参数（速度、区域、容差）在 SITL 和真机间**保持一致**以保持行为可重复。需要区分的少数参数（如 `precision_align_drop_height_tolerance`）可通过 launch 的 `--ros-args -p` 单独覆盖，不必拆分整个文件。

---

## 六、改动范围总览

| 类别 | 文件 | 变更程度 |
|------|------|---------|
| **新建** | `scripts/start_real_flight.sh` | 全新 |
| **新建** | `config_real/vision_params.yaml` | 全新 |
| **新建** | `config_real/servo_params.yaml` | 全新 |
| **新建** | `launch_real/real_flight.launch.py` (完善版) | 全新 |
| **新建** | `logs_real/` | 目录 |
| **新建** | `rosbags/` | 目录（可选） |
| **修改** | `mission_state_machine.py:1190` | ARM 分支（~5 行） |
| **修改** | `mission_state_machine.py:965-970` | 投放失败处理（~10 行） |
| **修改** | `px4_bridge.py:251-253` | GPS/电池遥测（~15 行） |
| **修改** | `servo_controller.py:50-52` | `_create_real_servo()` 实现 |
| **修改** | `emergency.py` | 电池/GPS 阈值真机化 |
| **修改** | `camera.py` | GStreamer 路径验证（小改或无） |
| **不改动** | 其他所有核心代码 | 状态机、视觉管线、路径规划、坐标变换等 |

---

## 七、与 SITL 的关系

```
                    ┌─ 状态机 (共用, 不改动)
                    ├─ 视觉管线 (共用, 不改动)
                    ├─ 飞行控制器 (共用, 不改动)
                    ├─ PX4 Bridge (共用, ⚠ 小改: 遥测)
                    └─ 舵机逻辑 (共用, 不改动)
                           │
           ┌───────────────┴───────────────┐
           ▼                               ▼
    SITL 路线 (已跑通)              真机路线 (本方案)
    ├ sitl.launch.py              ├ real_flight.launch.py (完善)
    ├ start_full_simulation.sh    ├ start_real_flight.sh (新建)
    ├ config/                     ├ config_real/ (新建, 覆盖)
    ├ ARM: param2=21196 (force)   ├ ARM: 标准解锁流程
    ├ 舵机: ServoMock             ├ 舵机: PCA9685/GPIO
    ├ 相机: Gazebo bridge         ├ 相机: GStreamer
    ├ 通信: UDP MicroXRCEAgent    ├ 通信: Serial MicroXRCEAgent
    ├ 模型: .pt (PyTorch)         ├ 模型: .engine (TensorRT)
    ├ 遥测: 硬编码假值             ├ 遥测: 真实 GPS/电池
    └ 投放失败: 静默模拟           └ 投放失败: 真实反馈+重试
```

SITL 随时可回归测试，真机开发不影响仿真验证。

---

## 八、风险与缓解

| 风险 | 影响 | 缓解措施 |
|------|------|---------|
| Jetson TensorRT 兼容性问题 | `.engine` 模型加载失败 | 保留 `.pt` (PyTorch) 回退，QoS 降级但可飞 |
| Serial 通信丢包/断连 | Offboard 失控 | PX4 自带 Offboard 丢失 → LOITER 保护 |
| 真实相机 FOV/畸变与仿真差异大 | 视觉检测率下降 | 标定后先在地面用手持相机验证检测效果 |
| 真机 PID 参数不匹配 | 飞行振荡或不稳 | 先在 QGC 手动调参，稳定后再切 Offboard |
| 投放机构机械故障 | 任务无法完成 | 状态机已有 `max_drops` 和 timeout 保护 |
