# 2026-07-21 重构: StandaloneBridge + 坐标变换统一 + 内存泄漏修复

## 概述

本次重构解决三个核心问题:
1. **代码重复**: `simple_mission` 和 `mission_fsm` 各自实现了相同的 PX4 中继通信逻辑
2. **坐标变换硬编码**: `telemetry.py` 中 `H_WORLD_X/Y` 写死为仿真值, 真机无法切换
3. **内存泄漏**: `gz_camera_bridge.py` 中 `list(img_bytes)` 导致仿真内存持续增长至崩溃

---

## 改动清单

### 1. `telemetry.py` — 可配置 Home 偏移

- 新增 `configure_home(world_x, world_y, world_z)` 函数
- `world_to_ned` / `ned_to_world` 使用模块级变量, 默认 `(0, 0, 0)`
- **仿真**: `configure_home(10, 4, 0)` (Home 在 Gazebo 世界坐标 (10,4))
- **真机**: `configure_home(0, 0, 0)` (Home=解锁点, World=本地 ENU)

### 2. 新增 `standalone_bridge.py` — 可复用独立桥接层

位置: `src/px4_control/px4_control/standalone_bridge.py`

从 `mission_state_machine.py` 提取为公共类 `StandaloneBridge`:

```
功能:
  - 订阅 /fmu/out/vehicle_odometry 等 4 个 PX4 遥测话题
  - 发布 /drone/cmd_setpoint_ned (位置指令)
  - 发布 /drone/cmd/vehicle_command_relay (模式/解锁指令)
  - 10Hz 心跳维持 Offboard 模式
  - sim_mode 参数: True=SITL force arm, False=真机标准解锁

接口: 完全兼容 PX4Bridge, 可直接传给 FlightController
```

**为什么需要它**: PX4 micro-XRCE-DDS 只允许一个外部组件直接写 `/fmu/in/*` 话题。
其他节点必须通过 `px4_bridge` 中继。`StandaloneBridge` 封装了中继逻辑,
对外暴露 `PX4Bridge` 兼容接口。

### 3. `mission_state_machine.py` — 移除 326 行冗余

- 删除私有类 `_StandaloneBridgeFacade` (~170 行)
- 删除私有类 `_StandaloneFlightFacade` (~150 行)
- `bridge` 属性 → `StandaloneBridge(self)`
- `fc` 属性 → `FlightController(self.bridge)` (直接使用真正的 FlightController)

### 4. `simple_mission.py` — 430→228 行

| 改动 | 之前 | 之后 |
|------|------|------|
| 坐标变换 | 自己实现 `world_to_ned` + 硬编码常量 | `from px4_control.telemetry import configure_home, world_to_ned` |
| 遥测订阅 | 手动订阅 3 个 PX4 话题 | `StandaloneBridge` 内部处理 |
| 指令发送 | 手动发布 Point + VehicleCommand + 10Hz 心跳 | `StandaloneBridge` 内部处理 |
| 飞行控制 | 手动 `_go_to_world()` + `_publish_setpoint()` | `FlightController.takeoff()` / `.goto_world()` / `.loiter()` / `.land()` |
| 解锁 | 硬编码 SITL force arm | `StandaloneBridge(sim_mode=True/False)` 自动选择 |

### 5. `gz_camera_bridge.py` — 修复内存泄漏 🔴

```python
# 之前 (每帧 ~25MB Python 对象垃圾)
msg.data = list(img_bytes)   # 921,600 字节 → 921,600 个 Python int (每个 28 字节)

# 之后
msg.data = img_bytes         # bytes 对象, 0 额外开销
```

### 6. PX4 仿真相机降分辨率

文件: `PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf`

```
1280×960 @ 30fps  →  640×480 @ 15fps
```

与 ROS2 代码默认参数一致 (`camera_width=640, camera_height=480`)。

### 7. `px4_bridge.py` — 电池/GPS 改用真实话题

- 新增订阅 `/fmu/out/battery_status` 和 `/fmu/out/vehicle_gps_position`
- 删除 SITL 硬编码假值 `battery_remaining=1.0`

---

## 架构变化

```
之前:
  simple_mission          mission_fsm
  ├─ 自己实现 world_to_ned  ├─ _StandaloneBridgeFacade (私有)
  ├─ 手动 PX4 话题订阅      ├─ _StandaloneFlightFacade (私有)
  ├─ 手动 10Hz 心跳         ├─ FlightController (仅 DDS 模式)
  └─ 手动 VehicleCommand    └─ ...

之后:
  simple_mission          mission_fsm
  ├─ configure_home()      ├─ configure_home()
  ├─ StandaloneBridge      ├─ StandaloneBridge      ← 同一个类
  └─ FlightController      └─ FlightController       ← 同一个类
         ↑                        ↑
         └──────── 共享 ──────────┘
```

---

## 真机部署

```bash
# 仿真 (默认)
ros2 launch drone_bringup simple_mission.launch.py

# 真机 (Home=解锁点, sim_mode=false 使用标准解锁)
ros2 launch drone_bringup simple_mission.launch.py \
  sim_mode:=false \
  home_x:=0.0 home_y:=0.0 \
  drop_x:=20.0 drop_y:=2.0
```

---

## 内存占用估算

| 模式 | 进程 | 内存 |
|------|------|------|
| SITL (headless) | PX4 + Gazebo + px4_bridge + simple_mission | ~4 GB |
| SITL (headless, 全流程) | + vision_pipeline + servo | ~6 GB |
| 真机 (Jetson 8GB) | px4_bridge + vision + mission + servo | ~2.5 GB |
