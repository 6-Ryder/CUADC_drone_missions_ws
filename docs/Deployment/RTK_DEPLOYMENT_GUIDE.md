# C-RTK 在当前工作空间的部署与使用指南

> 适用基线：PX4 v1.17、CUAV 7-Nano、ROS 2 Humble、Jetson 上的本工作空间。
> 
> 硬件假设：CUAV C-RTK 9P/9Ps **基站端 + 机载 Rover 端**完整套件。9P 已是旧型号；若采购时可选，优先确认卖家提供仍受支持的 9Ps 套件、对应端角色和线缆。本文不适用于仅购买一个 RTK 模块、NTRIP 或双天线航向方案。

本指南的目标是让 RTK 改善当前任务的水平定位，同时保持既有 ROS 任务逻辑不变。RTK 不是避障系统，也不替代视觉精准降落或人工遥控器接管。

## 0. 你们拟采用的分阶段路线

### 阶段 A：已落地（实际为 TFmini，不是 TF02‑Pro）

这一步只改善垂直高度估计，不会解决 GPS 的水平漂移。**该阶段已随 `params_v4.params` 完成**，但实际装的是 Benewake **TFmini**、接在 **GPS 2 口**，并非本文原先设想的 TF02‑Pro。相对 v3 的实际改动是：

```text
SENS_TFMINI_CFG:  0   → 202          # 0=禁用；202 = GPS 2 口（波特率 Auto）
SENS_TFMINI_HW:   新增 → 1           # 1 = TFMINI
EKF2_HGT_REF:     1   → 2            # 1=GPS → 2=测距仪
EKF2_GPS_CTRL:    7   → 5            # 关掉 bit 1（高度融合），高度改由测距仪主导
EKF2_RNG_POS_X/Y: 0 / 0 → 0.33 / −0.32   # 测距仪安装偏移
EKF2_IMU_POS_Z:   0   → −0.2         # IMU 安装位置
```

`SENS_EN_TF02PRO` 保持 `0`，`EKF2_RNG_CTRL` 保持 `1`（Conditional Range Aid）。验证仍走 PX4 的 `distance_sensor`、`vehicle_local_position.dist_bottom_valid` 和 `estimator_status_flags.cs_rng_hgt`。不要为了启用测距仪修改任务的 `home_x/home_y`、航点或速度。

**GPS 2 口已被测距仪占用**，因此 `GPS_2_CONFIG` 必须保持 `0`；将来若要增加第二块 GPS，需先另找接口。

若改用 TF02‑Pro，PX4 v1.17 已包含 `tf02pro` 驱动；具体 I2C/UART 接口、地址和线缆必须按照 7‑Nano 与传感器版本的配件说明核对。[PX4 距离传感器驱动](https://docs.px4.io/v1.17/en/modules/modules_driver_distance_sensor)

### 阶段 B：再部署 C‑RTK

RTK 部署时保留已装好的测距仪（TFmini），不需要为了两种传感器增加 ROS 节点。RTK 收敛期间让 ROS 提前运行，但任务必须停在等待状态；只有 RTK 质量满足条件才允许起飞。这样既能保留测距仪的低空高度收益，也不会让普通 GPS 的平面误差进入任务航点闭环。

## 1. 最终方案与不变量

本仓库的控制位置来自 PX4 EKF2 输出的本地 NED 位姿，而不是 ROS 侧直接计算的经纬度：

```text
C-RTK 基站 ─USB─> QGroundControl ─MAVLink 2 / 数传─> PX4 ─GPS 口─> Rover
                                             │ RTCM 注入      │
                                             ▼                ▼
                                      EKF2 本地 NED ─> vehicle_odometry
                                                               │
  Jetson <── Ethernet / MicroXRCE-DDS ────────────────────────┤
                                                               ▼
                       px4_control ─> /drone/pose ─> 测试任务 / 主任务 FSM
```

PX4 会把 QGroundControl 传来的 `GPS_RTCM_DATA` 转发给 GPS/Rover；基站与 Rover、以及稳定的地面—机载 MAVLink 数传三者缺一不可。低带宽数传必须端到端使用 MAVLink 2，PX4 官方给出的典型上行需求约为 300 B/s。[PX4 v1.17 RTK 集成](https://docs.px4.io/v1.17/en/advanced/rtk_gps)

本指南使用的逐项官方事实、当前参数基线和 7-Nano 接口核查记录见 [RTK_OFFICIAL_RESEARCH.md](RTK_OFFICIAL_RESEARCH.md)。

以下不变量必须保持：

- 真机的 NED 原点仍是 PX4/EKF 起飞时建立的局部原点；`home_x/home_y/home_z` 仍是 ROS World 映射偏移，**不要因为加 RTK 而改动**。
- Jetson 与 PX4 的 Ethernet/uXRCE-DDS 连接仍只承担 ROS 2 话题；它不能替代基站到 QGC 再到飞控的 RTCM/MAVLink 通道。
- 现有航点、`waypoint_tolerance`、视觉目标坐标和任务状态机均不需为“接入 RTK”修改。
- 必须保留 RC 接管、硬件急停和人工监控；RTK Fixed 不是飞行安全的充分条件。

## 2. 当前代码与 RTK 的关系

### 2.1 无代码改动即可运行的部分

`px4_bridge` 已订阅 PX4 的 `/fmu/out/vehicle_odometry`，把 NED 位姿变换为 `/drone/pose`；任务和视觉据此工作。它也已订阅 `/fmu/out/vehicle_gps_position`，但目前只记录 `fix_type` 和卫星数。

| 工作空间位置 | 当前职责 | RTK 接入后的变化 |
|---|---|---|
| `src/px4_control/px4_control/px4_bridge.py` | PX4 DDS、EKF 位姿、NED 指令 | 不改控制逻辑；RTK 让输入 odometry 更准确 |
| `src/px4_control/px4_control/telemetry.py` | NED/World 变换和遥测模型 | 不改坐标变换 |
| `src/drone_mission/` | 完整任务 FSM、航点与视觉投放 | 不改任务参数/航点 |
| `src/test_mission/` | 20 m 前飞 + 侦察分阶段试飞 | 不改试飞路线 |
| `src/drone_bringup/launch/real_flight.launch.py` | 真机的 DDS、相机、视觉、舵机与主 FSM 启动 | 不改启动命令 |

因此，**基础 RTK 部署的 ROS/Python 改动量为 0 行**。所需变化是飞控参数、物理接线、QGC 与试飞流程。

### 2.2 当前代码的安全缺口

完整主任务只依赖 PX4 `preflight_ok` 和 RC 解锁；分阶段 `test_mission` 也只检查相同状态。PX4 v1.17 的 `SensorGps` 消息虽然包含 `fix_type`、`eph`、`rtcm_msg_used`、`rtcm_injection_rate` 和 CRC 状态，但当前测试桥没有订阅它，主桥也没有将它变成任务准入条件。

这意味着：RTK 失锁为普通 3D GPS 时，当前任务会继续飞行。基础部署先依赖操作者在 QGC 确认 RTK Fixed；若项目要求“没有 RTK Fixed 绝不自动起飞/继续任务”，应实施第 8 节的质量门代码。

## 3. 硬件、连接与安装

### 3.1 采购与检查清单

- 一套明确标识为 **Base（基站）+ Rover（机载）** 的 C-RTK 9P/9Ps；不能用两个相同角色的单机替代。
- 基站端的测量天线、Rover 端天线、机载固定支架、GPS 数据线和基站 USB 数据线。
- 能与 QGroundControl 双向通信的机载数传/Wi-Fi；飞行时不能依赖系留 USB。其上行余量必须覆盖 RTCM，且必须确认 MAVLink 2。
- 运行 QGroundControl 的电脑（RTK 基站设置不使用移动端 QGC）。
- 备用的当前 `params_v4.params` 导出和一块已验证的 SD 卡。

CUAV 文档说明 C-RTK 9P/9Ps 与 PX4 可即插即用；CUAV 7-Nano 的 GPS 接口也说明可连接 C-RTK 系列。仍须以实物接口和随件线缆为准，禁止以外观相似的线缆盲插。[CUAV C-RTK 配置](https://doc.cuav.net/gps/c-rtk-series/zh-hans/c-rtk-9ps/parameter.html) [7-Nano 手册](https://manual.cuav.net/controller/7-Nano_manual_en.pdf)

### 3.2 机载接线和安装顺序

1. 卸桨、断开动力电池，只保留 USB 或安全供电。
2. 用随 Rover 提供的 GPS 线连接 **7-Nano 的 GPS/GPS&SAFETY 口**；不要接 Jetson、TELEM 或以太网口。
3. 将 Rover 天线装在机架最高处、朝上，远离电调、电源线、数传天线和 Jetson；线缆固定但不可拉紧天线接头。
4. 若替换了原 GPS/罗盘组件，必须重新执行 PX4 传感器与罗盘校准；不得复用旧标定结果。
5. 上电后先在 QGC 检查 GPS、罗盘、安全开关（如适用）和校准状态；任何一项异常均不得进入基站配置或解锁。

不要把基站放在车顶、移动桌面或可能被人移动的位置。基站应放在开阔、固定、抬高的位置，天线具有无遮挡天空视野，并远离金属结构和强射频源。

## 4. QGroundControl 与 RTK 修正链路

### 4.1 建链顺序

1. 在户外开阔场地启动 QGroundControl（PC/macOS/Linux 版）。
2. 用 USB 将 **Base** 连接到运行 QGC 的电脑；确认 QGC 的 RTK 图标出现。
3. 打开机载 PX4，并通过数传/Wi-Fi 使同一 QGC 同时连接飞控；确认遥测链路是双向且使用 MAVLink 2。
4. 等待 QGC 完成 Base 的 Survey-In；首次 Survey-In 需要数分钟。已验证位置后可以保存并复用固定基站坐标，但不可把移动后的基站继续当作旧坐标使用。
5. Survey-In 完成后，QGC 会开始把修正发送到飞机。只有 QGC 显示 RTK 正常、飞控 GPS 状态为 **3D RTK Fixed**，才算链路完成。

PX4 会自动配置其支持的 GPS 模块并转发 RTCM，通常不需要 u-center。不要在 QGC 自动配置已成功时再导入不明配置文件。[PX4 v1.17 RTK 自动配置](https://docs.px4.io/v1.17/en/advanced/rtk_gps)

### 4.2 必须确认的状态

在 QGC 中确认：

- Base 已完成 Survey-In 或已加载经验证的固定位置；
- 飞机图标显示 **3D RTK Fixed**，不是 `RTK Float`、`DGPS` 或普通 `3D GPS`；
- MAVLink Inspector 中 `GPS_RAW_INT` / `GPS2_RAW` 的 fix 类型与 QGC 状态一致；
- 若信号不断从 Fixed 切到 Float，先查天线、基站遮挡、数传上行和 RTCM，不要通过放宽 EKF 参数掩盖。

飞控 NuttX 控制台可辅助检查：

```sh
listener vehicle_gps_position 5
listener estimator_status_flags 5
mavlink status
```

期望 `vehicle_gps_position` 中有 `fix_type=6`（RTK Fixed）、`rtcm_msg_used=2`、`rtcm_crc_failed=false`，并且 `rtcm_injection_rate` 为正。`estimator_status_flags` 中应当有 `cs_gnss_pos=true`、`cs_gnss_vel=true`、`cs_gnss_fault=false`。字段定义见本工作空间的 [SensorGps.msg](../../src/px4_msgs/msg/SensorGps.msg)；其含义也见 [PX4 SensorGps 文档](https://docs.px4.io/v1.17/en/msg_docs/SensorGps)。

### 4.3 收敛时间优化：不要每次从零开始 Survey-In

需要区分两个时间：

| 阶段 | 慢的原因 | 对当前任务的处理 |
|---|---|---|
| Base Survey-In | 基站在估计自己的绝对坐标；要求绝对误差越小，等待越久 | 只在首次部署/基站移动后做；完成一次后保存固定坐标并保持 Base 不动 |
| Rover RTK Fixed | Rover 等待 RTCM、卫星载波相位和稳定链路 | 每次开机先建立修正链路；保持天线开阔、数传 MAVLink2 和模块供电稳定 |

CUAV 对 9Ps 标称 RTK Survey-In 小于 60 s、热启动小于 1 s、冷启动小于 24 s，但这些是设备规格，不是任何场地的保证。CUAV 另指出，基站绝对坐标达到约 2 m 可能只需几分钟，达到 30 cm 约需 1 小时，达到 10 cm 可能需要数小时。[C-RTK 9Ps 规格](https://doc.cuav.net/gps/c-rtk-series/en/c-rtk-9ps/) [CUAV Base Survey-In 说明](https://doc.cuav.net/gps/c-rtk-series/en/rtkbase-for-mp.html)

本任务的坐标链路是“起飞点建立局部 NED 原点 → 飞 20–60 m 的相对航线”。只要 Base 在整个任务期间不移动，Base 的绝对坐标误差主要表现为整个局部坐标系的共同平移，并不会按同样大小破坏起飞点到航点的相对距离。因此，不要为了每次都达到 10 cm 的 Base 绝对 Survey-In 而延迟任务数小时；如果任务以后需要跨天复用绝对经纬度地图或地理围栏，再单独提高 Base 绝对坐标精度。

推荐运行策略：

1. 首次部署时在固定、开阔位置完成一次可接受的 Survey-In，并在 QGC 保存 Base position。
2. 以后每天开机前只检查 Base 没有被移动、天线没有改变、QGC 能读取保存的 Base position；不要重复执行高精度 Survey-In。
3. Base、地面 QGC 和机载 Rover 按固定顺序提前上电，先让 RTCM 链路建立，再启动/允许任务。
4. 让 ROS 节点可以提前启动，但把任务保持在 `INIT/WAIT_ARMED`；代码只在 `fix_type=6`、RTCM 正在使用、数据未超时且 EKF XY 有效时放行起飞。这样收敛期间不会阻塞 Jetson，也不会误起飞。
5. 设置一个现场超时，例如 90–120 s；超时仍是 `RTK Float` 或普通 GPS 就取消本架次自主任务，转人工检查，不放宽到 Float 执行高精度航线。

不要通过反复断电、移动 Base、换到遮挡位置或把 `EKF2_REQ_FIX`/质量门放宽来“加快收敛”。如果每次都必须从冷启动等待，优先检查供电、天线天空视野、MAVLink2 上行和 RTCM 使用状态，而不是修改任务速度或坐标。

## 5. PX4 参数：当前值、首飞设置与禁止项

先在 QGC 导出参数，并保存为带日期的新文件；绝不覆盖仓库中的 `params_v4.params`，它是当前基线记录。

```sh
# NuttX/QGC MAVLink Console：只用于读取和记录
param show GPS_1_CONFIG
param show GPS_1_PROTOCOL
param show SER_GPS1_BAUD
param show EKF2_GPS_CTRL
param show EKF2_GPS_P_NOISE
param show EKF2_GPS_V_NOISE
param show COM_ARM_WO_GPS
param show MAV_PROTO_VER
```

### 5.1 首次接入的参数矩阵

| 参数 | 当前基线 | RTK 首飞建议 | 原因 |
|---|---:|---|---|
| `GPS_1_CONFIG` | `201`（GPS 1） | 保持 `201` | 主 GPS 端口的正确串口映射 |
| `GPS_1_PROTOCOL` | `1`（u-blox） | 保持 | C-RTK/F9P 路径由 PX4 GPS 栈自动配置 |
| `SER_GPS1_BAUD` | `0`（Auto） | 保持 | 避免未验证时强制错误波特率 |
| `GPS_1_GNSS` | `0`（模块默认） | 保持 | 先让 Rover/PX4 使用已验证默认多星配置 |
| `GPS_2_CONFIG` | `0` | 保持 `0` | 首次 RTK 只接一个主 GPS；第二接收机冗余另立验证任务 |
| `GPS_UBX_MODE` | `0`（Default） | 保持 `0` | QGC → 飞控 → Rover 的标准 RTCM 链路不使用模式 5；模式 5 是 RTCM 直接接 Rover UART2 的另一种接法 |
| `EKF2_GPS_CTRL` | `7` | 保持 `7` | 继续融合 GNSS 位置、高度、速度；不启用未安装的双天线航向 |
| `EKF2_REQ_FIX` | `3`（3D fix） | 基础部署保持；严格 RTK 策略经失锁评审后可设为 `6` | GPS 进入 EKF 的最低 fix 类型；`6` 才是 RTK Fixed |
| `EKF2_GPS_P_NOISE` | `0.5` m | 首飞保持；日志验证后再评估 `0.2` | PX4 提醒 RTK 可使用更低噪声，但过早调低会使创新量拒绝更频繁 |
| `EKF2_GPS_V_NOISE` | `0.3` m/s | 首飞保持；日志验证后再评估 `0.2` | 同上 |
| `EKF2_REQ_EPH` | `3.0` m | 保持或最多收紧到固件允许的 `2.0` m | **不能用于要求厘米级 RTK**；v1.17 下限为 2 m |
| `COM_ARM_WO_GPS` | `1` | 完成地面验证后改为 `0` | 禁止无有效全球位置估计时解锁；仍不能区分 RTK Fixed 与普通 GPS |
| `MAV_PROTO_VER` | `2` | 保持 `2` | RTK 修正数传要求 MAVLink 2 |
| `SENS_GPS_PRIME` | `0` | 保持 | 首飞不用 GPS blending 复杂化问题 |
| `NAV_ACC_RAD` | `2.0` m | 不改 | 本项目 Offboard 到达由任务 `waypoint_tolerance=0.5 m` 判断，不由此参数决定 |

执行须重启的参数修改后必须完整断电重启飞控，再重新检查 QGC 和 RTK 状态。例如在确认 RTK 地面链路有效后：

```sh
param set COM_ARM_WO_GPS 0
param save
reboot
```

`COM_ARM_WO_GPS=0` 只要求 EKF 的全球位置检查通过；它不是 “RTK Fixed 必须存在” 的开关。PX4 v1.17 中 `EKF2_REQ_EPH` 的可设下限是 2 m，因此也不能把它设置为 0.1 m 来模拟 RTK 门限。[PX4 参数参考](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference)

如果任务明确规定“GPS 只能在 RTK Fixed 下参与 EKF”，可以在完成第 7 节的失锁试验、并确定人为接管和 Offboard 失效策略后设置：

```sh
param set EKF2_REQ_FIX 6
param save
reboot
```

`EKF2_REQ_FIX=6` 会在 RTK 从 Fixed 退化时阻止普通 GPS 进入 EKF；这不是免费的安全增强：它也可能导致飞行中全局/本地定位状态退化。因此不得在首次链路验证前设置，也不能替代第 8 节所述的项目级质量门。

### 5.2 首飞时不要改的项

- 不要将 `EKF2_GPS_CTRL` 改为双天线航向模式，除非实际安装并验证了双天线基线；单 Rover RTK 不提供该能力。
- 不要改变 `GPS_1_CONFIG`、`GPS_1_PROTOCOL` 或 `SER_GPS1_BAUD` 来“强制 RTK”；这会破坏自动 GPS 配置。
- 不要把 `GPS_UBX_MODE` 改为 `5`；那是“RTCM 直接注入 Rover UART2”的不同布线方案，不适用于本指南的 QGC/MAVLink 方案。
- 不要因为 RTK 而立即把任务 `waypoint_tolerance` 从 `0.5 m` 调小。先用真机数据验证飞控动态、风和机架振动。
- 不要把 `COM_OBL_RC_ACT` 当作 RTK 失锁保护；它处理的是 Offboard 链路丢失。当前值为 `0`，其失效策略必须作为独立飞行安全决策评审。

### 5.3 参数回滚

如果 RTK 无法获得 Fixed：卸桨后恢复前一次 QGC 参数导出，或至少恢复 `COM_ARM_WO_GPS=1` 后再做室内/无 GPS 台架调试。恢复该参数不等于允许执行自主任务；自主任务仍应在有有效定位的户外环境测试。

## 6. 工作空间启动与日常使用

RTK 不需要额外 ROS 节点，也不需要重建 `px4_msgs`。仅在改变本工作空间源码/接口后才按部署文档构建。

### 6.1 分阶段 RTK 验证（首选）

1. 完成第 4 节，确保 QGC 已是 RTK Fixed。
2. 启动 ROS：

```bash
cd /home/nvidia/CUADC_drone_missions_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch test_mission test_forward_recon.launch.py \
  sim_mode:=false rc_require_arm_signal:=true
```

3. 确认 DDS、视觉、GPS 和 PX4 状态，保留遥控器手动接管：

```bash
ros2 topic echo /fmu/out/vehicle_gps_position --once
ros2 topic echo /fmu/out/vehicle_odometry --once
ros2 topic echo /drone/pose --once
ros2 topic echo /fmu/out/vehicle_status_v1 --once
```

4. 无桨确认完毕后，依次做：静态悬停 → 20 m 前飞 → 20 m 返回 → 小范围侦察。每一步单独落地、导出日志、比较与基准的终点偏差；禁止第一次就运行投放或完整任务。

`test_forward_recon` 有独立航线和参数，不读取完整 FSM 的区域配置；RTK 不改变二者的 NED 坐标约定。

### 6.2 完整任务使用

只有分阶段验证通过后，按既有真机入口启动：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/CUADC_drone_missions_ws/install/setup.bash
ros2 launch drone_bringup real_flight.launch.py
```

每次完整任务的飞行前顺序：

1. 固定 Base 并完成 Survey-In/加载已验证基站坐标；
2. 检查 QGC 为 RTK Fixed，且保持至少 30 秒；
3. 检查 `rtcm_msg_used=2`、无 CRC 失败、EKF 正在融合 GNSS；
4. 启动 ROS，确认 `/drone/pose` 连续稳定；
5. 确认相机、舵机、电池、遥控器、地面人员和接管策略；
6. 人工 RC 解锁后才允许 FSM 进入起飞。

## 7. 验收、日志与故障定位

### 7.1 量化验收

每项至少重复 5 次，记录风、Base 位置、卫星状况、QGC 固定状态和机架载荷：

| 项目 | 通过条件 | 记录位置 |
|---|---|---|
| 静态 RTK | 连续 30 s Fixed，无 Float/3D 回退 | QGC 截图、ULog |
| 修正链路 | `rtcm_msg_used=2`、`rtcm_crc_failed=false` | `vehicle_gps_position` |
| EKF 融合 | `cs_gnss_pos=true`、`cs_gnss_vel=true`、无 GNSS fault | `estimator_status_flags` |
| 前飞/返航 | 20 m 往返端点误差和重复性显著优于普通 GNSS 基线 | ULog + 场地测量 |
| 侦察 | 全部航点完成，无位置重置、无任务超时 | 任务日志 + ULog |

`eph`/`epv` 是接收机的估计误差，不是地面真值；用已测量的起降点和重复往返偏差做最终评价。PX4 对 EPH/EPV 的定义见 [GNSS 数据说明](https://docs.px4.io/v1.17/en/gps_compass/index)。

可从 ULog 导出关键话题：

```bash
ulog2csv real_flight_logs/<date>/qgc_log/<flight>.ulg \
  -m vehicle_gps_position estimator_status estimator_status_flags vehicle_local_position
```

重点检查 `vehicle_gps_position.eph`、`fix_type`、`rtcm_msg_used`、`rtcm_crc_failed`，以及 `vehicle_local_position.eph`、`xy_reset_counter` 和 `estimator_status.pos_test_ratio`。

### 7.2 常见故障

| 现象 | 首先检查 | 处理 |
|---|---|---|
| QGC 没有 RTK 图标 | Base USB、驱动、QGC 版本 | 先只让 QGC 识别 Base |
| Base 正常但飞机仍是 3D GPS | 飞机到 QGC 数传、MAVLink 2、`mavlink status` | 检查双向上行，不能只看下行遥测 |
| 在 Fixed/Float 间反复切换 | 天线天空视野、Base 是否移动、射频/电源干扰 | 停飞并重布设；不调松 EKF 掩盖 |
| ROS 正常但定位不准 | QGC Fixed、`rtcm_msg_used`、EKF flags | ROS 侧不产生 RTK；回到 PX4 链路排查 |
| 任务在普通 GPS 下仍可解锁 | `COM_ARM_WO_GPS` 和现有任务逻辑 | 设置 `COM_ARM_WO_GPS=0`，并实施第 8 节质量门 |
| 加装 RTK 后罗盘异常 | 组件方向、校准和安装干扰 | 卸桨重新校准；检查磁罗盘健康 |

## 8. 可选但推荐：RTK 质量门代码

### 8.1 何时需要

若操作者始终在 QGC 确认 Fixed 后才手动解锁，基础部署无需代码。`EKF2_REQ_FIX=6` 能进一步约束 EKF 的 GNSS 接纳，但没有提供任务级预热、RTCM 新鲜度、状态原因和飞行中失锁动作。以下代码只在项目有明确要求时实施：

> 仅当 RTK Fixed、RTCM 已被 Rover 使用、数据未超时且 EKF 具有有效 XY 定位时，允许自动任务从 INIT 进入 ARM；飞行中持续失去条件则停止发布新航点并进入预先评审的安全动作。

### 8.2 模块设计

在 `px4_control` 新建一个深模块 `PositionQualityGate`。它的对外 interface 只有：

```python
observe_gps(sensor_gps: SensorGps) -> None
observe_local_position(local_position: VehicleLocalPosition) -> None
is_ready(now_s: float) -> bool
reason(now_s: float) -> str
```

它在内部处理 `fix_type == FIX_TYPE_RTK_FIXED (6)`、`rtcm_msg_used == RTCM_MSG_USED_USED (2)`、无 CRC、最新数据年龄、`eph` 门限和 `xy_valid`。任务 FSM 只读取 `bridge.position_ready`，不读取 GNSS 原始字段。这样同一个 interface 同时服务主任务和 `test_mission`，避免把 RTK 判定散落在各状态。

建议的首版运行参数：

```yaml
require_rtk_fixed: true
rtk_max_eph_m: 0.15
rtk_warmup_s: 10.0
rtk_data_timeout_s: 1.0
```

这些是项目安全门限，不是 PX4 或 CUAV 的保证精度；应依据第 7 节日志调整。

### 8.3 精确改动范围

| 文件 | 改动 |
|---|---|
| `src/px4_control/px4_control/position_quality.py` | 新建纯 Python 质量模块及单元测试 |
| `src/px4_control/px4_control/px4_bridge.py` | 喂入已有 GPS 话题，并新增 `VehicleLocalPosition` 订阅与 `position_ready` 属性 |
| `src/px4_control/px4_control/standalone_bridge.py` | 同样订阅 GPS/本地位置，以便 `test_mission` 使用同一 interface |
| `src/drone_mission/.../state_init.py` | 将 `position_ready` 纳入 INIT 条件和超时诊断 |
| `src/test_mission/.../test_forward_recon.py` | 在 `WAIT_ARMED`/起飞前拒绝非 RTK 就绪 |
| `src/px4_control/config/px4_params.yaml` 与 launch | 增加上述项目级门限，真机启用、SITL 默认关闭 |
| 各包 `test/` | 覆盖 Fixed、Float、RTCM 未使用、过期、EKF XY 无效和恢复场景 |

实现前须先明确飞行中失锁策略（建议至少悬停并通知操作者）；不要在未经现场安全评审时自动执行 `LAND`、RTL 或 disarm。这个质量门是后续软件工作，不属于本指南的“零代码部署”。

## 9. 与其他文档的关系

- [ARCHITECTURE.md](../ARCHITECTURE.md)：任务数据流、NED/World 坐标和 PX4/Jetson 边界。
- [DEPLOYMENT.md](DEPLOYMENT.md)：工作空间构建、SITL 和现有真机启动入口。
- [REAL_HARDWARE_DEBUG_GUIDE.md](../Debug/REAL_HARDWARE_DEBUG_GUIDE.md)：遥控器、DDS、相机、舵机和接地现场清单。
- [PARAMETER_USAGE_GUIDE.md](../Debug/PARAMETER_USAGE_GUIDE.md)：任务航线、速度、相机与状态机参数；RTK 不改变它们的所有权。
- `params_v4.params`：当前真机基线。每次 RTK 参数变化都应新建带日期的导出，不要覆写。

## 10. 首飞签字清单

- [ ] Base 与 Rover 角色、线缆、天线和安装方向已核验；
- [ ] Base 固定且天空无遮挡，Survey-In/固定坐标已确认；
- [ ] QGC 显示连续 RTK Fixed；
- [ ] `rtcm_msg_used=2` 且 `rtcm_crc_failed=false`；
- [ ] `MAV_PROTO_VER=2`，数传上行没有饱和；
- [ ] GPS/罗盘重新校准与飞控自检通过；
- [ ] `COM_ARM_WO_GPS=0` 已在户外地面测试后启用；
- [ ] RC 手动接管、飞行区域和硬件急停已就绪；
- [ ] 已完成无桨检查、悬停与 20 m 往返，且日志已保存；
- [ ] 未在第一次 RTK 飞行中启用投放或完整任务。
