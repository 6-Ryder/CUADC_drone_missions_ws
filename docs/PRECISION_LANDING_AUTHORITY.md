# 视觉精准降落的权威方法调研

> **状态**：第一批（本地 PX4 源码取证）已完成并核实，见 §2。第二批（MAVLink 规范、
> ArduPilot、IBVS 控制论经典、PX4-ROS2 实践）的并行调研仍在进行，落地后追加。
> 本文只写**已亲手核实**的内容；每条结论附一手来源，未取证的不写。

调研问题：本项目的视觉伺服应改用哪种权威控制方法，或能复用哪个权威实现。

---

## 1. 结论摘要

**"直接复用 PX4 官方 precision landing 流水线"这条路，在本项目当前的集成架构下走不通。**
不是因为它不好，而是因为三处结构性阻断（§2）。可行路径是**复用其算法结构**，在 ROS2 侧实现。

同时发现一项**未被使用的现成资产**：工程已经在算目标的世界坐标（`world_x`/`world_y`），
但降落链路完全没用它（§3）。这是改造所需的关键输入，已经存在。

---

## 2. 已核实的事实（本地 PX4 源码）

源码树：`/home/ryder/PX4-Autopilot`（工作区内软链，可直接逐行对照）。

### 2.1 权威实现确实存在

- `src/modules/landing_target_estimator/` 存在，含 `KalmanFilter.cpp` / `KalmanFilter.h` /
  `LandingTargetEstimator.cpp` / `landing_target_estimator_params.yaml`。
  → PX4 对"低频、有噪、间歇的落地目标测量"给出的权威答案是**卡尔曼滤波**，不是直接对
  像素误差做控制。

- 相关 uORB 消息在 `msg/` 中：`LandingTargetPose.msg`、`LandingTargetInnovations.msg`、
  `PositionControllerLandingStatus.msg`。

### 2.2 但 precision land 已重构为 mission 工作项

- `src/modules/navigator/mission_base.h:94` 定义 `WORK_ITEM_TYPE_PRECISION_LAND`。
- 使用点：`mission_base.cpp:375`、`mission_base.cpp:712`、`mission.cpp:208`、
  `rtl_mission_fast_reverse.cpp:293`。
- `src/modules/flight_mode_manager/tasks/` 下**没有** `PrecisionLand` 任务
  （该目录只有 AltitudeCruise / Auto / AutoFollowTarget / Descend / Failsafe /
  Manual* / Orbit / Transition / Utility）。

**阻断 1**：precision land 由 **MAVLink 任务协议**驱动，而本项目是**纯 DDS offboard**，
不使用任务协议。要走上这条路等于重构整个任务下发方式。

### 2.3 估计器输出未暴露给 DDS

`src/modules/uxrce_dds_client/dds_topics.yaml` 中与 landing/target 相关的条目**只有**：

```yaml
  - topic: /fmu/in/landing_gear
    type: px4_msgs::msg::LandingGear
```

**阻断 2**：`LandingTargetPose`（估计器输出）没有 DDS 桥接 —— ROS2 侧**读不到**；
也没有 `/fmu/in/landing_target` —— ROS2 侧**也喂不进**。要打通必须改
`dds_topics.yaml` 并**重刷飞控固件**。

### 2.4 板级构建中未见该模块

`grep -rln 'landing_target_estimator' boards/` 仅命中
`boards/modalai/voxl2/scripts/install-voxl.sh`（一个安装脚本），未出现在任何
`*.px4board` 中。

**阻断 3（待确认）**：该模块在本项目所用固件里**可能根本没编译进去**。需按实际板型
（CUAV 7-Nano）核对 px4board 后才能定论 —— 列为待办，不作结论。

### 2.5 本项目实际使用的 DDS 话题

`src/px4_control/px4_control/px4_bridge.py` 中共 7 个：

```
进: /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint, /fmu/in/vehicle_command
出: /fmu/out/vehicle_odometry, /fmu/out/vehicle_attitude,
    /fmu/out/vehicle_land_detected, /fmu/out/vehicle_gps_position
```

即：**位置/速度 setpoint + 命令入，遥测出**。没有任何目标级接口。

---

## 3. 未被使用的现成资产

- `src/drone_interfaces/msg/LandingTarget.msg` 已含 `world_x` / `world_y` 字段。
- `src/drone_vision/drone_vision/coordinate_transforms.py` 有完整的
  `pixel_to_world`：针孔模型 → 相机→机体（安装旋转 + `image_top_faces_forward`）
  → 机体→World（四元数）→ 与地面平面求交。
- `src/drone_vision/drone_vision/visual_servoing.py` 里有 `WorldCoordinateServo`（世界坐标
  P 控制）。

**但降落链路（`state_precision_land.py` / `state_precision_land_simple.py`）走的是
像素误差 → 机体速度**，不使用世界坐标，也不经 PX4 的位置控制器。

这与 PX4/ArduPilot 的共同结构相反：**权威做法是视觉提供"目标在哪"的测量，由飞控的
位置控制器负责"怎么飞过去"**；本项目让 ROS2 自己写了一个速度环外环。

---

## 4. 由此得出的方向（待第二批调研确认后定稿）

1. **视觉侧应退化为测量源**，输出目标位置（估计值 + 质量 + 时间戳），而不是速度指令。
2. **位置式外环**（PBVS 类）交给 PX4 已整定的位置控制器，避免在外环再叠一个积分器。
3. **测量需经估计器**（卡尔曼/α-β）而非直接对像素误差做控制——这是 PX4 与 ArduPilot
   的一致做法，也直接针对本项目已记录的"低空追误检振荡"。
4. 三处 DDS/固件阻断（§2.2–2.4）需明确取舍：是接受"ROS2 侧复用算法结构"，还是投入
   改固件以复用官方实现。

---

## 5. 待办与未决

- [ ] 第二批调研：MAVLink `LANDING_TARGET` 字段语义与时间戳/延迟补偿规定
- [ ] 第二批调研：ArduPilot `AC_PrecLand` 的估计器类型、超时与门限设计
- [ ] 第二批调研：IBVS vs PBVS 的已知病态，以及微分噪声的标准处理
- [ ] 核对 CUAV 7-Nano 的 px4board 是否包含 `landing_target_estimator`（§2.4）
- [ ] 定稿分阶段实施方案

## 6. 本文档的方法与限度

- 所有结论均来自**本地 PX4 源码的直接阅读**，附文件:行号。未经实测。
- **不构成飞行就绪结论**。按 ROS 2 工程规范的 L0–L6 口径，本文属 **L0（静态审阅）**：
  证明了命名模块/参数在已安装版本中存在，不证明任何执行行为。
- 未取证的推测一律标注为待办，不写进结论。
