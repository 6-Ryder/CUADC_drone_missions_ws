# 视觉精准降落的权威方法调研

> **状态**：第一批（本地 PX4 源码取证）与第二批（MAVLink 规范、PX4 precland/VTE、
> ArduPilot AC_PrecLand、IBVS/PBVS 控制论经典）均已完成并逐条核实；差距表（§5）由
> 一次六路并行对照调查产出，其中五路完成、六路对抗性核查因 API 余额中断（见 §9）。
> 本文只写**已亲手核实**的内容；每条结论附一手来源（`文件:行号` 或 URL+章节）。
> 未经核实的一律进 §8 待办，不写进结论。

调研问题：本项目的视觉伺服应改用哪种权威控制方法，或能复用哪个权威实现。

---

## 1. 结论摘要

**1. "直接复用 PX4 官方 precision landing 流水线"在本项目当前的集成架构下走不通**，
原因是三处结构性阻断（§2.2–2.4）。可行路径是**复用其算法结构**，在 ROS2 侧实现。

**2. 四份权威（MAVLink 测量契约、PX4 估计器+VTE、PX4 precland 时序、ArduPilot
AC_PrecLand）与 IBVS/PBVS 正典在五点上完全一致**（§4.6），且其中一条是**禁令**：

> 正典中**不存在对原始像素/测量误差的 D 项**。误差导数只作为"被滤波的外部目标运动
> 估计"以**前馈**形式注入；速度信息来自**估计器的状态**，不是对测量求差。
> （Chaumette & Hutchinson, IEEE RAM 2006 Part I pp.82-83 eqs (1)-(5)；Part II p.115
> finding 14；PX4 与 ArduPilot 的实现同此。）

**3. 本项目与这五点的差距不是"缺一个滤波器"，而是三种形态的问题**（§5）：
   - **缺**：测量与控制器之间没有任何状态估计（§5.2）；无不确定度通道（§5.1）。
   - **反了**：龄期语义与权威相反 —— 对"消息没来"无限宽容，对"来了但被拒"立即判丢失
     （§5.3）。最危险的情形（相机/推理链路死亡）反而是最沉默的失效路径。
   - **空转**：机制已存在但生产路径未接线 —— `ServoPid` 的多速率接口、
     `vision_measurement_stamp()`、`_locked_world_xy`、`yaw_correction` 全是零消费方
     （§5.3、§5.4、§5.5）。

**4. 第二阶段边界据此调整为三件事**（详见 §7），其中一条与候选方案不同：

| | 候选方案 | 本调研调整后 |
|---|---|---|
| 边界 | 加 Header + 三源 stamp 接入 `ServoPid`，**恢复微分项** | 加 Header + 三源 stamp 接入 `ServoPid`，**微分项保持归零** |
| 依据 | 按真实测量间隔求差即可用 D | D-on-raw-error 不在任何权威控制律中；导数信息应由估计器提供（阶段 3） |
| 附带 | —— | ① 修正 Stage 1 的一处缺陷（§7 阶段 2a）；② 补降落源唯一缺失的龄期门 |

---

## 2. 第一批：已核实的事实（本地 PX4 源码）

源码树：`/home/ryder/PX4-Autopilot`（工作区内软链，可直接逐行对照），
版本 `v1.18.0-alpha1-544-g3e9841cbb6`。

### 2.1 权威实现确实存在

- `src/modules/landing_target_estimator/` 存在，含 `KalmanFilter.cpp` / `KalmanFilter.h` /
  `LandingTargetEstimator.cpp` / `landing_target_estimator_params.yaml`。
  → PX4 对"低频、有噪、间歇的落地目标测量"给出的权威答案是**卡尔曼滤波**，不是直接对
  像素误差做控制。
- 相关 uORB 消息在 `msg/` 中：`LandingTargetPose.msg`、`LandingTargetInnovations.msg`、
  `PositionControllerLandingStatus.msg`。

### 2.2 阻断 1：precision land 已重构为 navigator 状态机，且由任务协议驱动

- `src/modules/navigator/precland.h:72` —— `class PrecLand : public MissionBlock`；
  `navigator_main.cpp:91` 注册为导航模式索引 5。
- `src/modules/flight_mode_manager/tasks/` 下**没有** `PrecisionLand` 任务：

  ```
  $ git -C /home/ryder/PX4-Autopilot cat-file -t HEAD:src/modules/flight_mode_manager/tasks/PrecisionLanding
  fatal: Not a valid object name
  ```

  任何"复用 PX4 的 PrecisionLand flight task"的计划都指向一个**已不存在的文件**。

**阻断 1**：precision land 由 **MAVLink 任务协议**驱动，而本项目是**纯 DDS offboard**，
不使用任务协议。要走上这条路等于重构整个任务下发方式。

### 2.3 阻断 2：估计器输出未暴露给 DDS

`src/modules/uxrce_dds_client/dds_topics.yaml` 中与 landing/target 相关的条目**只有**：

```yaml
  - topic: /fmu/in/landing_gear
    type: px4_msgs::msg::LandingGear
```

**阻断 2**：`LandingTargetPose`（估计器输出）没有 DDS 桥接 —— ROS2 侧**读不到**；
也没有 `/fmu/in/landing_target` —— ROS2 侧**也喂不进**。要打通必须改
`dds_topics.yaml` 并**重刷飞控固件**。

### 2.4 阻断 3：板级构建中未见该模块（仍未定论）

`grep -rln 'landing_target_estimator' boards/` 仅命中
`boards/modalai/voxl2/scripts/install-voxl.sh`（一个安装脚本），未出现在任何
`*.px4board` 中。**本项目所用具体板型（CUAV 7-Nano）的 px4board 尚未核对** ——
列在 §8 待办，不作结论。

### 2.5 本项目实际使用的 DDS 话题

`src/px4_control/px4_control/px4_bridge.py` 中共 7 个：

```
进: /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint, /fmu/in/vehicle_command
出: /fmu/out/vehicle_odometry, /fmu/out/vehicle_attitude,
    /fmu/out/vehicle_land_detected, /fmu/out/vehicle_gps_position
```

即：**位置/速度 setpoint + 命令入，遥测出**。没有任何目标级接口。
`grep -rn 'LANDING_TARGET|landing_target' src/px4_control/` 在生产代码中 **0 命中**。

`MulticopterPositionControl.cpp:441,467,550` 表明：offboard 下 `trajectory_setpoint`
**直通** `PositionControl`，navigator/precland 的全部逻辑（§4.3）**永远不会执行**。

---

## 3. "未被使用的现成资产"（修正版）

原文（第一批）称工程已经在算目标的世界坐标但降落链路没用它。**结论不变，但三处细节
必须修正**，否则会按错误的心智模型去消费它：

1. **生产者是谁**。`LandingTarget` 在树里有**两个**发布者，但只有一个在跑：
   - `src/drone_vision/drone_vision/vision_pipeline.py:1127-1197`（`_process_landing_v2`）
     —— 三个 `launch` 文件里跑的都是 `vision_pipeline`（`real_flight.launch.py:109`、
     `sitl.launch.py:97`、`quick_flight.launch.py:118`）。
   - `src/drone_vision/drone_vision/landing_detector.py` —— 只有它的
     `LandingCircleTracker` / `LandingPadRefiner` 被 `vision_pipeline.py:51` 导入；
     其**节点类从不启动**（`setup.py:38` 有入口点，但三个 launch 均未引用）。
   - ⚠️ 该未启动节点的 `:556-557` 把 `world_x/world_y` 写成**相对偏移**
     （`du*alt/fx`），与 `.msg` 注释"World frame x/y"矛盾。**这是死代码，但会误导读者**。

2. **用的是哪一时刻的姿态**。活的那条路径
   （`vision_pipeline.py:1186-1196`）调用 `pixel_to_world` 时传的是 `self._drone_pose`
   —— **当前（处理）姿态**；而**同一个进程**里的桶口投影通道传的是
   `self._frame_pose`（采集时刻姿态，`capture_pose.interpolate_pose` 插值得到，
   `vision_pipeline.py:812-817`）。同一进程两套时间基准（§5.3）。

3. **消费方为零**。`get_landing_target()` 的两个消费点
   （`state_precision_land.py:293-300`、`state_precision_land_simple.py:350-382`）
   只读 `center_u/center_v/confidence`；`world_x/world_y` 全项目无读取方。

其余资产不变：`coordinate_transforms.pixel_to_world`（针孔模型 → 相机→机体 → 机体→World
→ 与地面平面求交）与 `visual_servoing.WorldCoordinateServo`（世界坐标 P 律，本项目唯一
的位置域控制器，未被降落链路使用）。

---

## 4. 第二批：四份权威设计的共同结构

### 4.1 测量契约（MAVLink LANDING_TARGET / TARGET_RELATIVE）

来源：`common.xml:6836-6853`（消息 #149，CRC 200，MAVLink2 扩展字段承载全部位置域）；
`development.xml:618-631`（后继 `TARGET_RELATIVE` #511）；`mavlink_receiver.cpp:2782-2818`
与 `:3226-3394`（两条接收路径）。

- 消息**自带 `time_usec`**（μs），规范原文只说"按数值量级判别 epoch 还是 boot"，
  **没有独立的延迟/龄期字段 —— 时间戳本身就是龄期机制**（`common.xml:6838`）。
- 后继 `TARGET_RELATIVE` 把时间拆成 `timestamp`（到达）+ `timestamp_sample`
  （`= sync_stamp(发送端时间)`，`mavlink_receiver.cpp:3345-3348`），并携带
  **显式坐标系**（`TARGET_OBS_FRAME`）、**逐轴不确定度** `pos_std`（接收端以 σ² 填协方差，
  `:3367-3369`）、目标朝向 `q_target`、采集时姿态 `q_sensor`。
- 位置路径 PX4 **只接受 `frame == MAV_FRAME_LOCAL_NED`(=1)**，其它帧打 critical 日志并
  **丢弃**（`:2788-2804`）。角度路径则把 `angle_x/angle_y` 直接当 `irlock_report`
  （注意单位陷阱：MAVLink 规定 rad，而 uORB 字段是 `tan(theta)`，接收端**不做转换**）。
- PX4 **不读** `distance`、`q`、`type`（`grep` 整个 handler 36 行无引用）。

### 4.2 估计器层（PX4 landing_target_estimator 与 VTE）

来源：`landing_target_estimator/{KalmanFilter.cpp,KalmanFilter.h,LandingTargetEstimator.cpp,
landing_target_estimator_params.yaml}`；`vision_target_estimator/`（VTE，非默认编译）。

- **不是**对像素误差做低通，而是**每轴一个 2 状态（位置, 速度）常速卡尔曼**
  （`LandingTargetEstimator.h:172-173`：`_kalman_filter_x` / `_kalman_filter_y`），
  `A=[1 dt;0 1]`、`H=[1 0]`（`KalmanFilter.h:36-47`）。
  → **项目缺的"微分项"在这里是滤波器的一个状态**，不是对测量求差。
- `predict` 以**载机自身加速度**为控制输入（旋转到 NED 后取负号，因为被估计量是
  "目标相对载机"且目标假设不加速）；姿态或加速度无效时输入置零、**照常 predict**
  （`LandingTargetEstimator.cpp:85-97`）。过程噪声 `Q = acc_unc·GGᵀ`，`G=[dt²/2; dt]`。
- 测量噪声**随距离平方缩放**：`R = LTEST_MEAS_UNC · dist_z²`，
  `LTEST_MEAS_UNC = 0.005 tan²`（`:125`）。效果：5 m 处 35 cm σ 的检测被自动怀疑，
  0.5 m 处 3.5 cm σ 的被自动信任 —— **不需要外接置信度**。
- **离群拒绝 = 卡方门**：`beta = r²/(P00+R)`，`beta > 3.84`（5% 误报）则该轴不融合，
  状态与协方差不动、predict 照常跑（`KalmanFilter.cpp:97-103`）。
  关键性质：`P00` 在掉线期间增大，故**重捕获瞬间门最松、收敛后收紧** —— 与固定像素门相反。
- **掉线 = 距最后一次被成功融合的龄期到点即硬复位**（2 s，
  `LandingTargetEstimator.h:94`）。注意 `_last_update` 只在**成功**初始化/发布时刷新，
  因此**一串持续被门拒绝的测量同样会走到复位**。
- 只在被融合时发布；被拒时**不发任何消息**（`:103-106,129-182`），由消费方按时间戳判龄。
- 另发 `landing_target_innovations`（含被拒帧），使门限可事后复算（`:184-194`）。

**VTE（新版，非默认）**：位置滤波改为**时间戳驱动的 OOSM 投影修正**
（`VTEOosm.h:33-52`），并把龄期拆成**四个角色分立的参数**：
`VTE_BTOUT 3.0 s`（硬复位）/ `VTE_TGT_TOUT 2.0 s`（发布有效性）/ `VTE_M_REC_TOUT 1.0 s`
（可否融合）/ `VTE_M_UPD_TOUT 0.1 s`（缓存保留）。零时间戳与**未来时间戳一律视为超时**
（`common.h:127-145`）。它**要求**测量携带 `timestamp_sample`，并按采集时姿态
（`q_sensor`）旋转而非当前姿态（`VTEPosition.cpp:959-961`）。

### 4.3 引导与时序（PX4 precland）

来源：`src/modules/navigator/precland.cpp`（约 750 行）+ `precland_params.yaml`。

七个状态：Start / HorizontalApproach / DescendAboveTarget / FinalApproach / Search /
Fallback / Done，相位发布在 `prec_land_status`。

| 机制 | 位置 | 数值 |
|---|---|---|
| 进入下降需**连续驻留** | `:304-316` | 2 s 不间断，中断即重计 |
| 水平验收盒（**逐轴**，非欧氏半径） | `:617-620` | `PLD_HACC_RAD` 0.2 m/轴 |
| 下降段对水平误差**吸收** | `:603-613` | 注释原文 "only leave it if target becomes unusable" |
| 接近途中丢失**继续飞向最后已知位置** | `:584-597` | 到达后才评估丢失 |
| 近地**主动放弃视觉**转盲降 | `:344-356,:623-625` | `PLD_FAPPR_ALT` 0.1 m |
| 近地龄期**收紧一个量级** | `:609` | 硬编码 0.5 s（正常 `PLD_BTOUT` 5 s） |
| 目标失效 = **纯龄期判据** | `:127-135` | `hrt_elapsed_time(&timestamp) > PLD_BTOUT` |
| 终态什么都不做，直接落 | `:375-379,:413-417` | Fallback 写当前位置为 `SETPOINT_TYPE_LAND` |

Z 轴下降速度由 `FlightTaskAuto.cpp:227-329` 按离地高度**连续插值**调度
（`MPC_LAND_ALT1/2/3 = 10/5/1 m` → `MPC_LAND_SPEED 0.7` / `MPC_LAND_CRWL 0.3` m/s），
XY 是位置 setpoint、Z 是速度 setpoint 的**混合**形式。

⚠️ 权威自身的缺口（移植时不可照抄）：`#define STATE_TIMEOUT` 定义于 `:62` 却**全文件无引用**；
HorizontalApproach 与 DescendAboveTarget **没有时间看门狗**，只有 Search 有。

### 4.4 独立第二意见（ArduPilot AC_PrecLand）

来源：`/home/ryder/ardupilot/libraries/AC_PrecLand/`（commit `549752196e`）。

结构：硬件无关的 backend 只提供（视线单位向量、测量时间戳、可选距离），frontend 把
**每条检测变成米制的、带时间戳的、几何修正过的位置**，再进估计器。控制层从不接触像素误差。

| 机制 | 位置 | 数值 |
|---|---|---|
| 估计器默认即卡尔曼 | `AC_PrecLand.cpp:79` | `PLND_EST_TYPE` 默认 1（非 RAW） |
| 延迟被**承认并补偿** | `:120-128,:226-232,:500-510,:724-756` | `PLND_LAG` 20–250 ms → 惯性环形缓冲，在测量真实时刻融合后前推 |
| 横移与下降**解耦** | `ArduCopter/mode.cpp:736-746` | `PLND_XY_DIST_MAX` 2.5 m 内才允许下降 |
| 近地降速按**水平误差**调度 | `mode.cpp:747-756` | 0.15 m 参考误差，下限 0.1 m/s |
| NIS 门 + **强制融合逃逸** | `AC_PrecLand.cpp:574-583` | `max(NIS) < 3.0`，连续 3 次拒绝后强融 |
| 测量方差随高度与角速率 | `:556` | `(h·(0.01+0.01·|gyro|)+0.02)²` |
| 龄期两级阶梯 | `AC_PrecLand_MAVLink.cpp:19`, `AC_PrecLand.cpp:26-28` | 1 s stale / 2 s lost |
| 记忆失效 | `AC_PrecLand.cpp:29-30` | 180 s 或 30 m → 降级 NEVER_SEEN |
| 重试改变几何 | `AC_PrecLand_StateMachine.cpp:10-12,150-235` | +1.5 m 爬升、0.75 m 到位容差、`PLND_RET_MAX` 4 |
| 兜底策略显式 | `:240-275` | 默认悬停 7 s 给飞手，然后垂直降落 |

⚠️ 权威的域外前提：官方 SITL 传感器默认 **100 Hz**（`SIM_PRECLAND_RATE`），IRLock 硬件
**50 Hz**。任何在这两个速率下验证的结论**不能**外推到本项目的 3–5 Hz 管线。

### 4.5 控制律正典（IBVS/PBVS）

来源：Chaumette & Hutchinson, IEEE RAM 13(4):82-90 (2006) / 14(1):109-118 (2007)；
Handbook of Robotics ch. 24；ViSP（Inria 参考实现）；Bourquardez et al., IEEE T-RO 25(3) (2009)。

- 基本律 `v_c = −λ·L̂_e⁺·e`，**整定旋钮只有一个标量 λ**；闭环 `ė = −λL_eL̂_e⁺e`，
  稳定性归于 `L_e·L̂_e⁺` 的正性条件（Part I p.83 eqs (4)-(5)、p.87 eq (19)）。
  → **本项目的像素误差→机体速度律就是这条正典律把 `L̂_e⁺` 折叠成逐轴标量增益**，
  不是自创控制律；该被批评的是缺失的伪逆结构（深度/增益缩放），不是律本身。
- **对原始误差的 D 项不存在于任何正典控制律中**。误差导数的正典身份是
  `∂e/∂t̂ = ê − L̂_e v̂_c`，其中 `ê` 由有限差分起步、**必须再经滤波/卡尔曼改进**，
  且以**前馈**形式注入，不是反馈（Part II p.115；Handbook §24.8 eq (24.32)）。
- I 项**只能消常速扰动**（原文："only if the target has a constant velocity"）。
- 已知病态：`k>6` 时 `L_eL̂_e⁺` 有零空间 → 局部极小；稳定邻域大小至今是 open issue；
  光学轴方向（本项目即高度轴）与旋转耦合 → "camera retreat"（Part I p.85）。
- 大位移的正典解法是**结构性**的：分区控制（Malis 2.5D，`s_t=(x, log Z)`）、
  特征轨迹规划、切换方案，或**误差范数自适应增益**
  （ViSP `vpAdaptiveGain`：`λ(‖e‖)=(λ0−λ∞)e^{−(λ′0/(λ0−λ∞))‖e‖}+λ∞`，
  默认 1.666 → 0.1666，约 10:1）—— **不是重调常数**。
- 虚假/不可靠检测的正典处理是**控制律内的统计加权**
  `v = −λ(DL̂_s)⁺D(s−s*)`，`D` 由 Tukey M-estimator 给出（Comport et al., T-RO 2006），
  而不是"整帧丢弃"或"整帧全权"。

### 4.6 五条一致结论（四份权威 + 正典的交集）

1. **视觉只提供"目标在哪"的测量**；闭环由飞控的位置控制器完成，companion 不发速度指令。
2. **每条测量必须携带采集时刻**（PX4 `timestamp_sample` / ArduPilot `los_meas_time_ms` /
   MAVLink `time_usec`）；消费方的一切龄期判断以它为准。
3. **测量与控制器之间必须有状态估计器**，速度信息取自滤波器状态，
   而非对测量求差 —— 这正是"微分项"的权威来源。
4. **离群拒绝用归一化新息（卡方/NIS）门**（3.84 / 3.0 + 连续 3 次强制融合），
   其松紧随不确定度自适应；不是固定像素跳变阈值。
5. **丢失是分级、有界、有兜底的策略**（stale/lost/记忆失效 + 重试阶梯 + 终态盲降），
   不是单级"丢失即悬停"。

**外加一条禁令**：不要在原始误差上加 D 项。

### 4.7 权威与"本项目验收口径"的量级对照（诚实预期管理）

| | 权威数值 | 本项目当前口径 |
|---|---|---|
| 水平验收半径 | PX4 `PLD_HACC_RAD` 0.2 m/轴；ArduPilot `PLND_XY_DIST_MAX` 2.5 m | Ø15 圆筒要求系统偏置 < 1 cm |
| 声称落点精度 | PX4 官方文档 **"roughly 10 cm"** | 同上传 |
| 近地放弃视觉高度 | PX4 0.1 m；ArduPilot 0.75 m | `landing_blind_altitude` 1.0–1.6 m |
| 进入下降驻留 | 2 s（PX4）/ 2 s 数据预热（ArduPilot） | 10 帧 tick（≈0.5 s），quick 档降为 **1 帧** |
| 传感器速率前提 | SITL 100 Hz / IRLock 50 Hz | 3–5 Hz |

**复用权威架构买到的是"有界、有兜底、可解释的行为"，不是厘米级精度。** 厘米级必须来自
载机自身定位（RTK / 已上线的测距仪基线）与目标测量的**坐标系正确性**，不是视觉环的增益。

---

## 5. 差距表：权威设计 vs 本项目现状

方法：六路并行对照调查（测量契约 / 估计器 / 延迟与龄期 / 控制律 / 时序状态机 /
集成路径），每路先读 §4 的一手来源、再逐条打开本项目源码核对，并强制记录实际跑过的
搜索式。**五路完成**（集成路径一路与全部对抗性核查因 API 余额中断，见 §9）。

图例：**缺** = 无对应机制；**形异** = 有，但形态/语义与权威不同；**空转** = 机制已存在
但生产路径未接线；**同构** = 与权威等价。

### 5.1 测量契约层

| # | 权威要素 | 现状 | 项目证据 |
|---|---|---|---|
| M1 | 每条测量自带**采集时刻** | **形异**（降落源缺） | `LandingTarget.msg:1-7` 无 header/无时间字段；`vision_pipeline.py:1127-1197` 三个发布分支均不写时间。对照 `CylinderArray.msg:1`、`HazardArray.msg:1` 有 `std_msgs/Header`，`vision_pipeline.py:812→787-789` 填 `_frame_stamp` |
| M2 | **有效性位**（`position_valid` / `world_position_valid`） | **形异** | `CylinderDetection.msg:3` 有 `bool world_position_valid` + NaN 约定 + 消费方先查位（`precision_align_controller.py:157-162`）；`LandingTarget` 无，`world_x/world_y` 投影失败时**保持 0.0**（`vision_pipeline.py:1185-1195` 仅成功分支赋值），与真实 (0,0) 不可区分 |
| M3 | **坐标系标签**（`frame` / `frame_id`） | **形异** | `LandingTarget` 同一条消息并存像素域 `center_u/v` 与 World 域 `world_x/y`，无帧字段、无判别位；数组层有 `frame_id="world"` |
| M4 | **不确定度**（`pos_std`，接收端 σ² 填协方差） | **缺** | 7 个 msg 全无方差字段；`grep -i 'std_dev\|variance\|covariance\|uncertain'` 仅命中投放释放余量常量（`quick_release_geometry.py:53-69`、`mission_state_machine.py:472-474`），与测量无关 |
| M5 | **距离**（度量化的前提） | **形异** | 无 `distance` 字段；尺度取 FSM 侧**当前**位姿高度（`precision_align_controller.py:216-225`）。`px4_control` 全包 `grep -i 'rangefinder\|dist_bottom'` **0 命中** —— params_v4 已上线的测距仪没有进入任何 ROS 侧测量契约 |
| M6 | 类型/身份随测量走 | **形异**（影响小） | 类型由**话题名**承担（`vision_pipeline.py:169-182`）；`CylinderDetection.id` 有跟踪 ID，`LandingTarget` 无 id |

**⚠️ 语义上限（写进验收口径）**：现存的"采集时刻"实为**相机节点的发布时刻**，不是曝光时刻
—— `camera.py:170` 在读到帧时用 `get_clock().now()` 打时间戳；SITL 转发分支
`camera.py:98` 更是直接**覆写** Gazebo 相机的时间戳。因此给 `LandingTarget` 加 header
能拿到的语义上限是**与圆柱流同级的"帧发布时刻"**，不应说成"精确采集时刻"。

### 5.2 估计器层

| # | 权威要素 | 现状 | 项目证据 |
|---|---|---|---|
| E1 | 每轴 2 状态（位置, 速度）常速卡尔曼 | **缺** | 全项目无任何卡尔曼/α-β 实现。控制侧是标量 P/I/D 律（`servo_pid.py:249-324`），视觉侧只有对误差的一阶 EMA（`precision_align_controller.py:325-331`）与对 D 的一阶低通（`servo_pid.py:263-265`）；`LandingCircleTracker` / `CylinderTracker` 只是**帧间关联器**，无状态无速度 |
| E2 | 以载机加速度为 predict 输入（前馈） | **缺** | `grep -i 'accel\|feedforward'` 在 `drone_mission`/`drone_vision` **0 命中**。载机速度只出现在投放弹道落点预测（`quick_release_geometry.py:10-50`，不进对准回路）与释放前速度归零门 |
| E3 | 归一化新息（卡方/NIS）门 | **形异** | 三道**固定阈值**连续性门：`landing_detector.py:381-401`（`center_jump>80px`）、状态侧 `landing_target_max_jump_px=80`（`state_precision_land_simple.py:369-379`，被 `test_flight_safety_contract.py:568-586` 钉死）、`confidence<0.60` 门。没有预测状态就没有新息；阈值是**绝对像素常数**，不随高度缩放（2.5 m 处≈0.5 m 地面偏差才被拒） |
| E4 | 测量方差随距离平方缩放 | **形异** | 误差**信号**已按高度换算（`precision_align_controller.py:207-225` 的 `_ground_error_m`），但**门限全是固定值**（9px/7px/30px、死区 12-40px、近地 0.20 m）——"一个像素值多少米"没有进入判决与定权 |
| E5 | 掉线 = 距**最后一次被接受测量**的龄期 | **形异（方向相反）** | `get_landing_target()` 直接返回缓存 dict、**无任何龄期检查**（`mission_state_machine.py:1613-1614`），而 `get_cylinders/get_markers/get_hazards` 都有 2.0 s 门。更严重：丢失计时只在"被门拒绝"时启动，**消息断流时旧 dict 会持续通过校验**（jump 与自己比较=0）→ 相机死掉反而不触发"失去目标" |
| E6 | 协方差随估计一并发布 | **缺** | `LandingTarget.msg:6` 只有 `confidence` 标量，是启发式几何分（`landing_detector.py:85-88,364-368`），与测量时刻、距离、历史无关 |
| E7 | 新息/拒绝原因可复盘 | **形异** | 有拒因日志（`state_precision_land_simple.py:383-390`）与 P/I/D 状态日志（`precision_align_controller.py:679-698`），但只有标量，无法复算门限 |

### 5.3 延迟与龄期层

| # | 权威要素 | 现状 | 项目证据 |
|---|---|---|---|
| L1 | 延迟被**补偿**而非回避（惯性缓冲 + 真实时刻融合 + 前推） | **缺** | `grep 'predict\|extrapolat\|oosm\|前推\|外推'` 在四个包内仅命中投放几何与日志，**无控制用途**。`capture_pose.interpolate_pose` 是**反向**插值（把位姿退回采集时刻），方向与权威相反 |
| L2 | 测量时刻参与控制的**真实步长** | **空转** | `ServoPid.update(stamp=...)` 已实现"同 stamp 重复调用不推进 P/I/D"（`servo_pid.py:225-232,252-274,284-295`），但**全部 5 个生产调用点传的都是 FSM 20Hz 时钟** `self._now()`：`state_precision_land.py:187-189,260-261`；`state_precision_land_simple.py:182-184,255-256`；`state_drop.py:696-698,755`；`state_cylinder_search.py:243-245,329-330`；`precision_align_controller.py:164-165,447-451,523`。结果 `is_new` 恒为 True、`dt` 恒 0.05 s |
| L3 | 四个角色分立的龄期参数 | **形异** | 有阈值但无角色分离、且散落在各状态里：到达时刻 2.0 s（三处 `get_*` 写死）、采集时刻 0.25 s / 0.40 s、里程计 0.10 s |
| L4 | 消费方按"距上次测量"判失效（`PLD_BTOUT`） | **缺（降落源）** | 见 E5。`_landing_target_stamp` 只被写（`:1001`）与经 `vision_measurement_stamp` 读出（`:781`），**无任何门控消费方** |
| L5 | 近地龄期收紧一个量级 | **形异** | 项目用"连续稳定**帧数**锁存 + 高度"（`landing_blind_altitude` 1.0–1.6 m）代替龄期门，且阈值是权威（0.1 m）的 10–16 倍 |
| L6 | 用**采集时刻姿态**做坐标变换 | **形异** | 机制已实现且很好（`capture_pose.py:6-31` 含四元数最短弧插值，`vision_pipeline.py:812-817`），但**只服务桶口投影**；降落通道用 `self._drone_pose`（当前姿态）且**不检查位姿新鲜度** |
| L7 | 跨节点时钟域显式映射 | **缺（当前靠巧合）** | 采集时刻来自视觉节点 ROS 时钟，FSM 侧与 `time.time()` 做差；全仓无 `use_sim_time` 配置。今天两者同源所以不显现，一旦启用 sim time 或分处两台机器即刻静默失效 |
| L8 | 按帧去重的保留窗口 | **形异** | 三处正确形态已存在（`mission_state_machine.py:910-915` 以采集时刻为 key 的字典、`state_drop.py:364-374`、`state_cylinder_search.py:304-311`、`quick_bucket_gate.py:21-27`）；**降落通道一个都没有** —— 只有按 **tick** 累加的 `_servo_converged_count`，所以 `landing_blind_stable_frames=10` 实际只等价 0.5 s 墙钟，不是 10 帧真实测量 |

### 5.4 控制律层

| # | 权威要素 | 现状 | 项目证据 |
|---|---|---|---|
| C1 | `v=−λL̂⁺e`，单标量 λ | **同构** | `servo_pid.py:56-129,247-250`；四个调用点各构造一个 `ServoPid`，整定唯一入口 `pid_tuning.yaml:85-101` |
| C2 | `L_e` 的**深度依赖**需结构性处理 | **形异** | 米制域已做 `Z/f` 换算（`precision_align_controller.py:207-225`，真机主用路径）；**像素域三个降落调用点直接用原始像素误差配固定 kp**（`state_precision_land.py:260`、`state_precision_land_simple.py:255`、`state_drop.py:755`）→ 同一组 kp 在高/低空之间物理增益差 10× |
| C3 | 导数只能是**带时刻的、被滤波的**目标运动估计 | **形异** | 见 L2；且三域 kd 当前**全部归零**（`pid_tuning.yaml:87,95,101`） |
| C4 | 正典**无**原始误差 D 项 | **形异（合规）** | `ServoPid` 的 D 是原始误差的滤波反馈（`servo_pid.py:252-270`），当前归零 —— 与禁令一致，属"可保持现状" |
| C5 | I 项只消常速扰动 | **同构** | 条件积分 + 硬钳位 + 死区冻结（`servo_pid.py:234-245,283-295`），形状比正典更细 |
| C6 | 误差范数**自适应**增益 / 误差有界化 | **形异** | `grep -i 'adaptive\|gain_schedule'` 0 命中；λ 恒定，只有 `max_speed` 被调用方改写。**但**项目已有"误差有界化"结构（分阶段下降 + 低空锁存 + 盲降），是正典 finding 23 的结构性替代 |
| C7 | 虚假检测用**统计加权**（`D=diag(w_i)`） | **形异** | 坏帧只有"全权驱动/清零速度"两极：`confidence<0.60` 整帧丢弃、`>80px` 整帧丢弃；`ServoPid` 无任何权重入参。检测侧其实已有连续质量量（`confidence`、`radius`、`fit_residual`） |
| C8 | 系统级：视觉只提供测量，外环在飞控 | **形异** | 降落链路发**速度域** TrajectorySetpoint（`px4_bridge.py:419-452`），闭环点在 companion。位置通道成熟但未被降落使用（`px4_bridge.py:594-616` `set_position_target` + 20Hz 斜坡平滑） |
| C9 | 旋转/偏航通道解耦 | **空转** | `LandingTarget.yaw_correction` 检测器已在算，存入 FSM，**全项目无控制消费方**；`set_velocity_body` 支持 `yaw_rate` 形参但所有降落/投放调用点均为 3 参调用 |
| C10 | 末段混合 setpoint（XY 位置 + Z 按离地高度连续调度） | **形异** | 项目同构但**离散三档**且不读测距仪（`state_precision_land.py:263-269`）。历史失败的 3.15–3.75 m 检测断裂带**恰好紧邻 3.0 m 的 fast→medium 切换点**，而 PX4 结构里该高度段的下降速率与视觉完全无关 |

### 5.5 时序与状态机层

| # | 权威要素 | 现状 | 项目证据 |
|---|---|---|---|
| S1 | 进入下降前**连续驻留**（2 s） | **形异** | 用 **20Hz tick 计数**：`_servo_converged_count`（`state_precision_land.py:236-256`）10 帧≈0.5 s，且同一份缓存测量被反复计数；quick 档 `mission_params_quick.yaml:180 landing_blind_stable_frames: 1`（注释：原 10 帧锁存常不达成）—— **与权威方向相反**。正确的"采集时刻键控 N 帧 + 时间窗"已在三处实现，只是没用在降落 |
| S2 | 下降段对水平误差**吸收** | **形异** | full 变体整个下降段每帧重跑水平 PID，无吸收；simple 变体的 `_vertical_only_latched` 是末端交接锁存（更晚、不可回退），不是"进入下降即吸收" |
| S3 | 接近途中丢失**飞向最后已知位置** | **缺** | 所有伺服实现丢失即 `set_velocity_body(0,0,0)`（`state_precision_land.py:215-221`、`state_precision_land_simple.py:206-213`、`precision_align_controller.py:266-303`、`state_drop.py:742-748`）。**最后已知位置锁存 `_locked_world_xy` 存在但从无读取方**（`precision_align_controller.py:59,114,665`） |
| S4 | 近地阈值以下**无条件**放弃视觉 | **形异** | 项目是"低于阈值**且视觉锁存成立**才转盲降"——阈值依赖视觉可信度；锁存不成立时 full 继续用视觉飞、simple 悬停 |
| S5 | 水平误差与下降**耦合**（未对准保持高度） | **形异** | 降落两态均为边对心边下降，下降速度只按高度分档、与水平误差无关。声明好的 `landing_precision_target: 0.1`（`mission_state_machine.py:433`）**无消费者** |
| S6 | 丢目标**分级**策略 | **形异** | 单级超时 + 一次重捕获 + 接管：`state_precision_land.py:110-126`（3.0/5.0 s）、`state_precision_land_simple.py:118-130`（1.0 s）。没有"陈旧但可用"的中间级，也没有记忆过期判决（后者与搬机平移 30–35 m 的失效模式直接相关） |
| S7 | 重试阶梯**改变几何** | **形异** | 重试 = 原地重发同一目标，无爬升几何、无到位容差，次数上限实际为 1（`ProgressMonitor._retry_used`，`progress_monitor.py:47-60`）。权威是 +1.5 m 爬升再降回、上限 4 次 |
| S8 | 终态兜底 = **原地盲降** | **形异（分叉）** | simple 变体等价（`_request_px4_land → land_at_current()`，quick 在用）；**full 变体相反**：`request_takeover` → `HoldForTakeoverState` **无退出超时的无限悬停** |
| S9 | 阶段超时 | **同构（项目更完备）** | 权威自身缺（`STATE_TIMEOUT` 死代码）；项目各阶段普遍有 `ProgressMonitor`。唯一缺口：full 盲降的 `landing_blind_timeout` 与 `_blind_start_time` 是**死代码** |
| S10 | 目标身份切换时重置控制器 | **同构** | `ServoPid.reset()`（`servo_pid.py:150-164`）在重捕获/换相时调用 |

**同批应清理的"死代码/死参数"**（属"以为有其实没有"的实例，会让后续维护者误判）：
`landing_search_altitude`、`landing_approach_speed`、`landing_blind_align_tolerance_px`、
`landing_precision_target`、`landing_blind_timeout`（五个参数全库无 `get_param` 消费者）、
`_blind_start_time`、`_locked_world_xy`、`LandingTarget.yaw_correction`、
`visual_servoing.py` 的整个内嵌伺服（B4 已禁用，入口无 launch）、
`landing_detector.py` 的节点部分。

### 5.6 集成路径层（本路调查中断，以下为本文档自行核实）

| # | 权威路径 | 可行性判定 | 依据 |
|---|---|---|---|
| I1 | MAVLink `LANDING_TARGET`（位置，`MAV_FRAME_LOCAL_NED`）→ PX4 precland | **需新增子系统 + 需离开 offboard** | `grep -rn 'LANDING_TARGET\|mavlink' src/` 生产代码 0 命中（仅 `tools/fetch_ulog.py` 用 pymavlink 连地面站）。onboard 无 MAVLink 链路；且 offboard 下 precland 永不执行（`MulticopterPositionControl.cpp:441,467,550`） |
| I2 | `TARGET_RELATIVE` + VTE | **需改固件 + development dialect** | `docs/en/advanced_features/vision_target_estimator.md` 明示 VTE 为 beta、默认板级配置**不编译**，需 `CONFIG_MODULES_VISION_TARGET_ESTIMATOR=y` 与 `CONFIG_MAVLINK_DIALECT="development"` |
| I3 | 旧 `landing_target_estimator` | **前置未定** | §2.4 板型未核对；且其输入只能来自 MAVLink（I1 的前置） |
| I4 | 坐标系陷阱 | **高危** | PX4 位置路径要求 `LOCAL_NED`，文档原文 "The origin of the local NED frame [0,0] is the home position"，而本项目**投放区坐标系挂的是 PX4 局部 NED 原点 = EKF 初始化点（上电点）**，搬动飞机即整体平移 30–35 m 且不报错 |
| I5 | 精度预期 | **低于本项目目标** | PX4 官方文档自称 "roughly 10 cm"（`docs/en/advanced_features/precland.md:118`） |

---

## 6. 集成路径决策

| | **路径 A：ROS2 侧复用算法结构**（推荐） | **路径 B：MAVLink → PX4 precland** |
|---|---|---|
| 要做什么 | 测量契约 + 估计器 + 时序策略，全部在 ROS2 内实现 | 建立 onboard MAVLink 链路；降落阶段离开 offboard；目标换算到 `LOCAL_NED` |
| 固件改动 | 无 | 视方案而定（VTE 需自定义固件；旧估计器需先确认编译进去） |
| 架构改动 | 无（保持 offboard） | **架构级**：FSM 交出降落控制权给 navigator |
| 坐标系风险 | 自控 | 继承 `LOCAL_NED` 原点语义（已咬过两次） |
| 精度上限 | 自控（取决于估计器与几何正确性） | ~10 cm（官方口径） |
| 收益 | 可增量、每步可回退、可 SITL 验证 | 一次性获得整定好的位置环 + 离群门 + 时序 |
| 判定 | **本阶段推荐** | 作为独立立项，需先回答 §8 的三个前置问题 |

**本文档的立场**：路径 A 不是"退而求其次"，因为 §4.6 的五条结论**全部可以在 ROS2 侧
按权威的算法结构实现**，且 `ServoPid` 的多速率骨架、`capture_pose` 的采集时刻姿态插值、
三处"采集时刻键控 N 帧"的既有实现都已经是正确形态，只是没接上降落链路。
路径 B 的价值在于**免除自己维持一个位置环**，代价是接受它的坐标系与 `~10 cm` 精度口径。

---

## 7. 分阶段方案

原则：每阶段独立可回退、小提交、提交后推送；提交信息标明哪些能地面验证、哪些必须上真机；
验证级别按 L0（静态）/ L1（单元）/ L4（硬件在环）如实标注。

### 阶段 2（本次）：测量时间线贯通 —— 让"测量时刻"真正进入控制

**边界（三件事，三笔提交）**

- **2a｜接线已有采集时刻 + 修正 Stage 1 缺陷**（无需重建 `drone_interfaces`）
  ⚠️ 核查中发现：`vision_measurement_stamp()` 目前返回的是**到达时刻**
  （`mission_state_machine.py:779-781` 映射到 `_cylinders_timestamp`/`_markers_timestamp`/
  `_landing_target_stamp`，全部是 `time.time()`），而圆柱/标识的真正采集时刻
  `_cylinder_capture_stamp`/`_marker_capture_stamp`（`:234-235,747,759`）**没有被暴露**。
  Stage 1 的提交信息承诺的是"测量时刻"，实现给的是到达时刻 —— 本次一并修正。
  然后把 5 个生产调用点的 `self._now()` 换成 `vision_measurement_stamp(source)`。
- **2b｜`LandingTarget.msg` 加 `std_msgs/Header` + 有效性位**（需重建 `drone_interfaces`）
  照抄同目录 `CylinderArray.msg:1` / `CylinderDetection.msg:3` 的既有约定：
  `header.stamp = self._frame_stamp`（`vision_pipeline.py:812` 已在手），
  `world_position_valid` + NaN 语义。**必须当场决定 miss 帧是否推进 stamp。**
- **2c｜降落源的龄期门**（唯一没有任何龄期门的视觉源）
  在 `get_landing_target()` 加与 `get_cylinders`（`:1523`）同形的门。
  注意：龄期门管"消息没来"，现有的"被拒即丢失"计时管"检测死了"，两者互补但都不等于
  权威的"距最后一次**被接受**的测量"——后者要等估计器落地（阶段 3）。

**明确不做**：不恢复微分项（kd 保持 0）。依据 §4.5 与 §4.6 的禁令。`ServoPid` 已有的
`d_term` 上报可用于**观察** Δe/Δt，但不作为反馈项使用。

**验收（L1 可地面验证）**：重复 stamp 不重复推进 I/D；跨源 stamp 不串味；
`stamp==0`（无测量）走 `measurement_valid=False`；miss 帧不冒充新测量；
`LandingTarget.header.stamp == _frame_stamp`（照抄
`test_cylinder_world_contract.py:203-225` 的契约测试形态）。
**必须上真机才能验证的**：3–5 Hz 实际帧率下 I/D 步长的真实收益。

### 阶段 3：估计器层（权威的"微分项"来源）

在 `drone_utils` 新增与 `KalmanFilter.cpp` 同构的 per-axis 2 状态常速卡尔曼
（`init(pos,vel,pos_unc,vel_unc)` / `predict(dt,acc)` / `update` 返回新息与协方差），
消费端沿用现有误差域与参数名。含卡方门（3.84，或 NIS 3.0 + 连续 3 次强制融合）、
`R ∝ range²`、以及"仅在成功融合时发布"的语义。
**可先用 α-β 增益形式落地同一 `predict/update` 签名**，把风险留在调参上而非结构上。
验证级别：结构 L1 可测，**参数必须 L4 真机调**。

### 阶段 4：时序策略（不依赖估计器，可与阶段 3 并行）

- S3 丢失时飞向最后已知位置（`_locked_world_xy` 已有）
- S1 驻留改为"采集时刻键控的 N 帧 + 2 s 时间窗"（照抄 `state_drop.py:364-374`）
- S4 近地阈值以下**无条件**转盲降
- S5 未对准保持高度 + 近地按水平误差调度下降速度
- S8 full 变体补"悬停 N 秒后原地盲降"兜底
- 清理 §5.5 列出的死参数/死代码

> **实施注记（按"不引入错误偏移/漂移/控制逻辑冲突"逐条复核后）**
>
> - **S3 已实施**（simple 变体）。实现上避开三个陷阱：锁存**世界坐标点**而非
>   机体系误差（否则航向一动目标即漂）；偏移量取自 servo 已标定的带符号误差再按
>   高度/焦距折成**米**（`ServoPidStatus` 里是**像素**，直接当米用会差 2–3 个
>   数量级）；指令仍走 `set_velocity_body` 速度域，不切位置 setpoint —— 避开
>   `px4_bridge._velocity_mode` 粘性标志与位置通道互相作废的已知陷阱。近地
>   0.75 m 以下不做横向机动（对齐 ArduPilot `PLND_ALT_MIN`）。
> - **S1 暂缓，且理由与直觉相反**：现有的 tick 计数虽然语义不严，但 quick 档
>   `landing_blind_stable_frames: 1` 是**针对真机观察到的"10 帧锁存常不达成"**
>   而刻意放宽的（`mission_params_quick.yaml:180` 注释）。改成测量键控会让锁存
>   更难达成 → 近地 0.5 m 处更容易悬停等待 → 更可能异地降落。**要改必须先有
>   配套的兜底策略**（S8 + 重试阶梯），否则是拿落点精度换语义纯度。
> - **S4 不能照抄权威的数值**：PX4 的 `PLD_FAPPR_ALT` 是 0.1 m 的"近地丢目标
>   救援"，而本项目的 `landing_blind_altitude` 是 1.0–1.6 m 的"确认对准后才允许
>   盲降"。把 1.5 m 处改成无条件盲降会在**横向未对准**时直接落地 —— 那正是要
>   避免的错误偏移。正确做法是保留现有锁存，**另设一个更低的强制阈值**。
> - **S5 需与重试阶梯同时上**：单独实现"未对准保持高度"会让持续偏心的目标永不
>   下降 → 悬停 → 超时接管，把 8/14 的"卡住"放大。
>
> **S8 经复核后判定"不实施"，因为它是控制逻辑冲突而非改进**：
> ArduPilot 的兜底是"悬停 7 s 给飞手，然后无论如何垂直降落"（`AC_PrecLand_StateMachine
> .cpp:240-275`），PX4 的 Fallback 也是"原地落"。但本项目有一条**显式且被测试钉住**
> 的策略：`request_takeover` 之后只做零速度悬停交给飞手，**永不自主动 RTL 或降落**
> （`test_operator_takeover_contract.py::test_request_takeover_latches_the_first_reason_and_never_commands_rtl_or_land`）。
> 把权威的兜底搬进来等于推翻这条契约。**注意区分**：simple 变体直接调用
> `bridge.land_at_current()` 是自己的降落兜底，不经过 `request_takeover`，两者不冲突
> —— 但 full 变体选择的是"交人"，这属于政策而非缺陷，不由本次改造推翻。
>
> **S5 的已实施形态与诚实评估**：只实施"近地按水平误差调度下降速度"这一半
> （ArduPilot `mode.cpp:747-756` 的结构，含 0.1 m/s 下限，保证偏心也不会停在半空），
> **不实施**"未对准保持高度"那一半（理由同上条）。实现放在
> `drone_utils/landing_profile.py`，**两条分支共用同一份**，并把此前各写一份的
> 高度分层梯子也合并进去 —— 避免继续出现"只改一侧"。同时给 `landing_precision_target`
> 这个此前零消费者的参数接上了用途（作为可接受误差）。
> ⚠️ **但必须说清楚：这条律在当前的整定下作用很小** —— 慢速档名义速度
> `landing_descent_slow = 0.12 m/s` 与下限 0.1 m/s 只差 0.02，所以 1.5 m 以下的
> 调度幅度仅 0.12→0.10。它的价值是**结构就位 + 参数复活 + 消除重复实现**；要让这条律
> 真正有牙齿，需要同时提高 `landing_descent_slow`（例如 0.25–0.30），那是一次真机整定
> 决定，不在地面可验证范围内。
>
> **S7 侧的一个小步（搜索几何**）：`landing_search_altitude` 与 `landing_approach_speed`
> 此前同为死参数 —— 两条分支进入降落时都是裸调 `fc.loiter()`，即"停在上一段（返航）
> 留下的高度"，而代码注释写着"保持在 5m"：那是**假设**而不是**命令**。现已改为
> 命令式（`landing_profile.hold_at_search_altitude`，两条分支共用）。**今天是零行为
> 变更** —— `return_altitude` 与 `landing_search_altitude` 都是 5.0，所以命令值与原
> 实际高度相同；改动只是把这个巧合变成可控参数。它的价值在于：检测质量与高度强相关
> （本项目已记录近地 ~0.9 m 与 3.15–3.75 m 两个失效带），搜索高度可控才谈得上把搜索段
> 稳定放在失效带之外。参数非法时退回裸 `loiter()`，与原行为一致。

### 阶段 5（可选，另立项）：路径 B 评估

需先回答 §8 的三个前置问题。

---

## 8. 待办与未决

- [ ] 核对 CUAV 7-Nano 的 px4board 是否包含 `landing_target_estimator`（§2.4，阻断 3）
- [ ] 真机固件版本与本地 PX4 树的 `LTEST_*` / `VTE_*` 默认值是否一致（本地树为
      `v1.18.0-alpha1-544`，真机基线为 params_v4）
- [ ] 决定 `LandingTarget` 是否承载 `pos_std`（M4 是本层唯一的真"缺"）
- [ ] 时钟域统一方案（L7）：是否全部走 `self._fsm.get_clock().now()` 并在出口做域校验
- [ ] 测距仪数据是否进入 ROS 侧测量契约（M5）
- [ ] `yaw_correction` 是否启用（C9，字段已算但零消费方）
- [ ] 六路差距调查的**对抗性核查未完成**（§9）—— §5 的每条"缺/形异"判定目前只有
      单路调查 + 本文档的选择性复核，未经过独立第二人复核

## 9. 本文档的方法与限度

- **第一批**：本地 PX4 源码直接阅读，附文件:行号。**第二批**：五份并行调研（MAVLink
  规范、PX4 估计器+VTE、PX4 precland、ArduPilot AC_PrecLand、IBVS/PBVS 正典），
  每份均回到一手源码/规范/论文核实并附引用。
- **差距表**：六路并行对照调查（§5），每路强制记录实际跑过的搜索式（"先盘清现状再改"
  的项目约定），每条项目侧陈述附 file:line。
- **⚠️ 核查缺口**：本该对每条"缺/形异"判定做**对抗性复核**（默认假设"项目其实已有"），
  以及"集成路径"一路和"完备性批评"一路，**均因 API 余额耗尽（HTTP 402）中断**。
  因此 §5 的否定性判定尚未经过系统性反证。本文档作者已就其中决定性最强的几条
  （E5 龄期门、L2 调用点、M1 采集时刻、L6 姿态时刻、2a 的 `vision_measurement_stamp`
  语义）自行逐条打开源码复核，并在 §7 记录了其中一处**推翻 Stage 1 前提**的发现。
  其余条目在据以动手前应同样复核。
- **全部结论为 L0（静态审阅）**，未做任何运行、编译或飞行验证。
- **不构成飞行就绪结论**。§4.7 的量级对照表明：复用权威架构买到的是**有界、有兜底、
  可解释的行为**，不是本项目验收口径假设的厘米级精度。
- 未取证的推测一律标注为待办，不写进结论。
