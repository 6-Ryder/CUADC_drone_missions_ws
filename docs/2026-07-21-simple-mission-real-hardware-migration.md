# 2026-07-21 — simple_mission 真机移植 (Jetson Orin Nano)

## 概述

从 SITL 仿真向 Jetson Orin Nano 真机迁移的第一步：打通 `simple_mission`
（起飞 → 航点 → 悬停 → 返航 → 降落）在真机上的运行链路。

`simple_mission.py` 本身代码**无需修改** — 它通过 `StandaloneBridge` +
`FlightController` 工作，已经是平台无关的。修改集中在底层桥接层和启动脚本。

---

## 代码变更

### 1. `src/px4_control/px4_control/standalone_bridge.py` — 🔴 核心修复

**问题**: `arm()` 写死 `param2=21196`（SITL force arm magic number），真机 PX4 会拒绝此值。

**修改**:
- `__init__` 新增 `sim_mode: bool = True` 参数
- `arm()`: `sim_mode=True` → `param2=21196` (SITL force)；`sim_mode=False` → `param2=0` (标准解锁)
- `force_disarm()`: 同样区分 SITL/真机参数

```python
# 修改后
def __init__(self, node: Node, sim_mode: bool = True):
    ...
    self._sim_mode = sim_mode

def arm(self) -> None:
    if self._sim_mode:
        self._send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)  # SITL
    else:
        self._send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)      # 真机
```

### 2. `src/px4_control/px4_control/telemetry.py` — 默认值修正

**问题**: Home 点默认 `(10, 4, 0)` 是 SITL 场地的 H 起降点坐标。

**修改**: 默认值改为 `(0, 0, 0)`（真机以解锁点为原点）。
`configure_home()` 仍然可以在 SITL 场景下被调用覆盖。

### 3. `src/px4_control/px4_control/px4_bridge.py` — 电池/GPS 真实数据

**问题**: `_status_cb` 中电池电量、GPS 状态硬编码为假值 (`battery_remaining=1.0, gps_fix_type=0`)。

**修改**:
- 新增 `BatteryStatus`、`SensorGps` import
- 新增 `/fmu/out/battery_status` 和 `/fmu/out/vehicle_gps_position` 订阅
- 新增 `_battery_cb`、`_gps_cb` 回调，从真实 PX4 话题更新遥测
- 删除 `_status_cb` 中的硬编码假值

### 4. `src/drone_mission/drone_mission/simple_mission.py` — 新增 sim_mode 参数

**修改**: 新增 `sim_mode` ROS2 参数（默认 `True` = SITL），传递给 `StandaloneBridge(sim_mode=...)`。

### 5. `src/drone_bringup/launch/simple_mission.launch.py` — 暴露 sim_mode

**修改**: 新增 `sim_mode` launch argument，默认 `true`。真机启动时传入 `sim_mode:=false`。

### 6. `scripts/simple_mission_real.sh` — 🆕 新建

真机一键启动脚本，对标 SITL 的 `simple_mission.sh`。

**特性**:
- 支持两种 DDS 通信方式（环境变量 `DDS_TRANSPORT`）:
  - `udp` (默认): 以太网连接 (CUAV 7 Nano / Pixhawk 6X)
  - `serial`: 串口连接 (TELEM2)
- 自动等待 PX4 DDS 话题就绪（60s 超时 + 诊断提示）
- 标准解锁 (`param2=0`)
- 以解锁点为原点 (`home_x=0.0, home_y=0.0`)

```bash
# 以太网 (CUAV 7 Nano)
./scripts/simple_mission_real.sh

# 串口
DDS_TRANSPORT=serial SERIAL_DEV=/dev/ttyTHS0 ./scripts/simple_mission_real.sh

# 自定义航点
./scripts/simple_mission_real.sh drop_x:=25.0 drop_y:=3.0 cruise_altitude:=10.0
```

---

## 真机部署步骤

### 环境准备

| 组件 | 要求 |
|------|------|
| OS | JetPack 6.x (Ubuntu 22.04) |
| ROS2 | Humble (ARM64) |
| MicroXRCEAgent | 从源码编译或预编译 |
| 飞控 | CUAV 7 Nano (或兼容 PX4 飞控) |

### CUAV 7 Nano 以太网连接

```
Jetson Orin Nano Eth0 ──── 网线 ──── CUAV 7 Nano Ethernet
```

**Jetson 端**:
```bash
sudo ip addr add 192.168.1.100/24 dev eth0
ping 192.168.1.1   # 确认飞控可达
```

**PX4 飞控参数**:
| 参数 | 值 | 说明 |
|------|-----|------|
| `UXRCE_DDS_CFG` | `2` | Ethernet/UDP |
| `UXRCE_DDS_AG_IP` | `192.168.1.100` | Jetson IP |

### 验证通信

```bash
# 终端1
MicroXRCEAgent udp4 -p 8888

# 终端2
source /opt/ros/humble/setup.bash
source ~/CUADC_drone_missions_ws/install/setup.bash
ros2 topic list | grep fmu
# 预期: /fmu/out/vehicle_status, /fmu/out/vehicle_odometry ...
```

### 编译 & 运行

```bash
cd ~/CUADC_drone_missions_ws
git pull origin bridge
colcon build --symlink-install
source install/setup.bash

# 首飞（建议先不装桨）
./scripts/simple_mission_real.sh
```

---

## simple_mission 任务流程

```
INIT → TAKEOFF → TRANSIT_TO_DROP → LOITER → RETURN → LAND → COMPLETE
  │        │            │              │         │        │
  │    垂直起飞      飞往航点       悬停N秒    返航Home  视觉/盲降
  │    至巡航高度   (drop_x,drop_y)                        着陆
  └─ 等待遥测就绪
```

---

## 后续步骤

1. **验证通信**: 确认 `/fmu/out/*` 话题可达
2. **地面测试**: 不装桨，确认 ARM → Offboard → 指令流正常
3. **首飞**: 低高度 (3m)、近航点 (10m) 首飞
4. **相机**: GStreamer 适配 + 标定
5. **模型**: PyTorch `.pt` → TensorRT `.engine`
6. **舵机**: PCA9685 驱动
7. **完整任务**: `mission_state_machine` 全流程

---

## 相关文件

- SITL 审计: `docs/2026-07-14-sitl-audit-and-real-hardware-gaps.md`
- 架构文档: `docs/ARCHITECTURE.md`
- SITL 启动: `scripts/simple_mission.sh`
- 真机启动: `scripts/simple_mission_real.sh` 🆕
