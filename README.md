# 无人机自主任务系统

一个基于 **ROS 2 + PX4 Offboard** 的全自主多旋翼任务系统:单机完成两次精准投放、
侦察区弓字形全覆盖、返航与视觉精准降落。从飞控适配、视觉推理、任务状态机到真机
部署与飞行取证,端到端由一人完成。

*[English →](README.en.md)*

**当前状态**:全流程在 SITL 与真机上均已跑通;可靠性收敛仍在进行中。
13 个飞行日里的每一次失败都已定位到根因,分类记录见
**[飞行失败模式分析](docs/FAILURE_MODES.md)** —— 那份文档比这份更能说明这个项目做了什么。

---

## 30 秒概览

| | |
|---|---|
| **规模** | 33,000 行 Python(其中 12,000 行测试)· 10 个 ROS 2 包 · 510 个测试函数 |
| **技术栈** | ROS 2 Humble · PX4 1.17 Offboard · ONNX Runtime / TensorRT · Gazebo Harmonic |
| **真机平台** | Jetson Orin Nano · CUAV 7-Nano · USB 相机 · PWM 舵机(MAIN5) |
| **飞行经历** | 12 个飞行日的完整日志目录(2026-07-28 – 08-15)+ 9 / 10 月多次复飞;ULog 全程留档(未入库,按需提供) |
| **工程方法** | 契约测试 · ULog 取证 · 权威实现优先(读 PX4/ArduPilot/MAVLink 一手来源,不凭印象调参) |

**任务流程**

```text
INIT → ARM → TAKEOFF → TRANSIT_TO_DROP_ZONE → CYLINDER_SEARCH → SELECT_TARGET
 → CYLINDER_APPROACH → DESCEND_TO_DROP → EXECUTE_DROP → ASCEND
 →(投满 2 次后)TRANSIT_TO_RECON_ZONE → RECON_PATROL → RETURN_TO_HOME
 → PRECISION_LAND → COMPLETE
```

任一状态异常、遥控器失联、严重电量事件或关键视觉超时都会进入 `HOLD_FOR_TAKEOVER`
或 `ERROR`,停止自主输出并交还飞手。

**运行时拓扑**

```text
         相机 (Gazebo 图像桥 / USB V4L2)
                    │ Image
                    ▼
             drone_vision              drone_mission
        ├─ 圆筒检测 (YOLO+形状精修)      │  单一 FSM,20 Hz 编排
        ├─ H 标识检测 (YOLO)            │
        ├─ 覆盖监控                      ├─ 位置/速度设定点 ──► px4_control ──► PX4
        └─ 视觉测量时刻 ────────────────►│                         (Offboard/DDS)
                                         └─ TriggerDrop ──► drone_servo2 ──► MAIN5 PWM
```

---

## 最难的三件事

这一节是这份 README 的重点。系统的规模不难,难在这三处。

### 1. 一个所有单元测试都抓不到的相机外参缺陷

**现象**:首次真机试飞(8/5),投放目标的世界坐标系统性北偏约 1.3–1.7 m。一个架次
因此被投放区世界门拒绝,另一个架次下降到 2 m 后目标已不在视野内。

**为什么它潜伏了这么久**:`image_top_faces_forward=False` 分支的坐标变换矩阵
`[[0,1,0],[1,0,0],[0,0,1]]` 的**行列式是 −1 —— 它是镜像,不是旋转**。而反射矩阵满足
`R² = I`,所以 `pixel_to_world` / `world_to_pixel` 的往返**完全自洽**:
任何"转过去再转回来"式的单元测试都会通过。更巧的是镜像在画面中心处误差为零,
既有的测试用例恰好用了 `u=640.0`(画面中心),精确地绕过了唯一会出错的那根轴。

**怎么定性的**:五条彼此独立的证据收敛到同一结论 —— 行列式符号;离线复算
(正确约定 0.029 m / 错误约定 3.199 m);伺服符号参数的既有约定;1234 帧真值反解
(免标定,前向与右向两轴同时锁定,96% 同号);以及 Gazebo 模型与真机相机是同一挂法。
修复后 quick 路径 SITL 首次全流程 `success: True`,锁定时世界坐标与真值差 **2.8 cm**。

> **可迁移的教训**:相机外参这类"没有物理直觉可依"的矩阵,必须有一个**按物理定义**
> 写的用例 —— 目标在机体右前方,就必须落在画面右上 —— 而不能按当前实现反推期望值。

### 2. 静默失效:所有健康信号都是绿的,飞机只是"不肯动"

三个独立案例,共同形态是 **不报错、不触发看门狗、飞控收到的是一条完全合法的指令**:

| 案例 | 表面现象 | 实际机制 |
|---|---|---|
| 粘性速度模式(10/7) | 起飞后原地悬停 20 s 不向前飞 | 一次零速度指令把 Offboard 模式位永久翻到 velocity,`TrajectorySetpoint.position` 全程 NaN;OCM 心跳 10 Hz 无丢包 |
| 看门狗喂错了信号 | 圆筒检测全死也不报警 | "vision" 通道由 `landing_target` **每帧消息**喂活(miss 帧也发 `confidence=0.0`),相机出帧即绿灯 |
| 观测龄门饿死控制器 | 表现为"检测器坏了" | 门的取值必须大于「一帧间隔 + 管线延迟」;取小了会让连续 3 帧收敛的概率降到 ~2%,结构性不可能收敛 |

诊断这类问题不能看健康信号,要从飞控日志反推:先看 `offboard_control_mode` 的
position/velocity 位在何时单向翻转,再看 `trajectory_setpoint.position` 是否 NaN ——
这条链比直接看飞机位置快得多。

### 3. 一个物理问题穿过整个控制栈,最后杀死着陆检测器

**现象**:飞手报告"位姿很不稳定",最终被迫按 Kill 落锁。

**根因**:机械振动。`gyro_vibration_metric` 达 4.9,而 PX4 阈值是 0.02 —— **245 倍**,
主频 80 Hz,一天之内幅值涨了 8.7 倍。

**后果链**(这才是值得看的部分):80 Hz 振动直接灌进角速率环 → 电机输出在 0 与 1 之间
狂摆(`torque_setpoint_achieved` 未达成率 60.9%)→ 飞控把横滚**设定值**打到 ±40° 反复
左右,而机体实际纹丝不动(跟踪误差 std 23.65°)→ 但遥控杆量 std 只有 0.088,**振荡是控制器
自己产生的,不是飞手在打杆** → 最后打到着陆检测器上:落锁判据 `LNDMC_ROT_MAX = 20 °/s`,
而实际角速度中位数 40.19 °/s、72.8% 时间超标,于是 `maybe_landed` / `landed` 恒为 0 →
**落锁被拒 3 次,飞手只能 Kill**。电机一停,角速度立刻降到 0.30 °/s,落地马上被识别 ——
这一条反证了整条链。

**判读时还踩了一个坑**:`vehicle_angular_velocity` 话题的日志采样率只有 49.45 Hz,会把
80 Hz 的振动**混叠**到 19 Hz,一度被误判为"19.73 Hz 极限环"。改用 197.8 Hz 的
`sensor_combined` 后主峰精确落回 80.04 Hz。**判读频谱前先确认采样率与奈奎斯特频率。**

> 这三件事的共同教训:系统的行为与它的健康信号是两回事。绿色的看门狗、合法的指令、
> 自洽的往返测试,都不能证明系统在工作。

---

## 验证状态

诚实起见,分清楚什么被证明了、什么没有:

| | 状态 | 证据 |
|---|---|---|
| 完整任务流程 (SITL) | ✅ 跑通 | 2026-07-30,`mission_end.success=True`,`drops=2/2`,覆盖率 `1.0`,199.95 s |
| 快速策略全流程 (SITL) | ✅ 跑通 | 2026-09-25,125.75 s,锁定时世界坐标误差 2.8 cm |
| **真机完整任务** | ✅ **成功过** | 2026-08-14 夜间 Run3:135.5 s,投放 1/1 + 侦察 100% + H 降落完成 |
| 真机可靠性 | ⚠️ **未收敛** | 同晚 5 个架次中仅 1 次完整成功;其余卡在投放区(3/5)或侦察段 |
| 精度降落(权威方法改造) | 🚧 进行中 | 阶段 1 / 2a / 2c 已落地,尚无完整 SITL 或真机复飞证据 |

未完成的部分不是"没做",是"正在收敛"。失败模式的分类、证据与修复状态见
[docs/FAILURE_MODES.md](docs/FAILURE_MODES.md)。

---

## 工程实践

**契约测试(510 个函数,非覆盖率数字)**。测的是接口承诺,不是行覆盖:
`test_flight_safety_contract` · `test_px4_safety_contract` · `test_mission_setpoint_contract` ·
`test_operator_takeover_contract` · 参数双向契约。两条参数类型契约**注入真实违规验证过会失败** ——
它们对应两类曾把节点打死的真机故障(rclpy 对参数类型严格匹配,YAML 里把 `35.0` 写成 `35`
就会让 FSM 启动即崩,当天飞不起来)。

**ULog 飞行取证工具链**。MAVLink FTP 直连飞控抓日志(带 ULog magic 与非零字节校验,
因为 QGC 在链路差时会**静默产出全 0 文件**);状态变化时间轴;Offboard 模式位生命周期;
事件解码。9/27 的振动分析、10/7 的粘性速度模式定性都出自这条链。

**权威实现优先**。精度降落改造前先做了一轮一手调研:MAVLink `LANDING_TARGET` 测量契约、
PX4 `landing_target_estimator` 与 VTE、PX4 `navigator/precland`、ArduPilot `AC_PrecLand`,
外加 IBVS/PBVS 正典。产出五条共识与**一条禁令** —— 正典中不存在对原始误差的 D 项,
误差导数的唯一正典身份是"被滤波的外部目标运动估计",以**前馈**注入。这条结论直接推翻了
原计划里"按真实测量间隔求差打开 kd"的整定依据。

**会撤回自己的结论**。`de56e66 撤回 "门降低 70-90%"(仿真时钟假象)` —— 台面仿真比实时
快 1760 倍,导致滤波器 `dt≈2e-5`,报出的收益在接上 `SimClock` 后归零。同期还发现被引用
数月的前提数字"视觉 3–5 Hz / τ 0.2–0.33 s"在真机实测下是 **10.5 Hz / τ 95 ms**,
且日志里的 `FPS` 字段其实是**单帧耗时的倒数**、不是帧率。已在 20 处注释与文档中修正。

**CI**。GitHub Actions + `ros-tooling/action-ros-ci`,Ubuntu 22.04 / ROS 2 Humble,
构建 8 个包并跑全部测试。

---

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 单元测试
python3 -m pytest -q src/drone_mission/test src/drone_vision/test src/px4_control/test
```

**完整仿真**需要 PX4 SITL、Gazebo Harmonic、MicroXRCEAgent,以及
`src/drone_vision/drone_vision/models/` 下配置引用的 ONNX 模型:

```bash
./scripts/start_full_simulation.sh              # 主任务
./scripts/start_quick_simulation.sh             # 快速策略
./scripts/start_full_simulation.sh layout_heading_deg:=50   # 非正北场地
```

**真机**:`./scripts/start_full_real_mission.sh`。起飞前用
`./scripts/verify_real_flight_ready.sh` 做只读预检(只查询 ROS 图与有界话题采样,
从不发布、不调用服务、不改变载具状态)。

**部署纪律**:Jetson 上任何模型或参数改动都必须走分支提交 + 重新构建。
有两次架次因为 build/install 目录里的陈旧参数或引擎格式与加载器不同步而未能起飞 ——
在真机上,这种"改了不起作用"比崩溃更难查。

---

## 仓库结构

| 包 | 职责 |
|---|---|
| `drone_interfaces` | ROS 2 自定义接口(`NedPositionSetpoint` / `CylinderArray` / `LandingTarget` / `CoverageStatus` …) |
| `px4_control` | PX4 DDS 适配与指令输出,Offboard 心跳、模式位、安全门 |
| `drone_vision` | 相机、ONNX/TensorRT 推理、目标精修、覆盖监控 |
| `drone_mission` | 任务 FSM、各状态行为、路径规划、精度降落控制器 |
| `drone_servo2` | C++ 多位置投放控制(SITL Mock / 真机 PX4 执行器) |
| `drone_utils` | 场地坐标系、位姿历史、降落估计器、看门狗、参数加载 |
| `drone_bringup` | SITL / 真机 × 主任务 / 快速策略的启动组合 |

**文档地图**(按阅读顺序建议):

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — 当前实现的结构、坐标链、数据流
- [docs/FAILURE_MODES.md](docs/FAILURE_MODES.md) — **飞行失败模式分类与分析**
- [docs/PRECISION_LANDING_AUTHORITY.md](docs/PRECISION_LANDING_AUTHORITY.md) — 精度降落的权威依据与差距表
- [docs/CODE_READING_GUIDE.md](docs/CODE_READING_GUIDE.md) — 逐文件导读与运行时调用链
- [docs/Debug/REAL_HARDWARE_DEBUG_GUIDE.md](docs/Debug/REAL_HARDWARE_DEBUG_GUIDE.md) — 真机现场检查清单

## 已知边界

- 控制环与视觉推理均为 Python,真机实测视觉 ≈10.5 Hz、端到端延迟 τ≈95 ms。
  实时性是靠参数调稳的,不是靠架构保证的。
- 场地坐标目前挂在 PX4 局部 NED 原点(= EKF 初始化点)上,而 EKF 原点在**上电**时锁存。
  飞机上电后才被搬上起飞点,会整体平移投放区且不报错 —— 这是一个已定位、待修的设计缺陷。
- 未上 CI 的部分:没有自动化 SITL 回归。每次改动的验证成本目前仍是一次实飞。

## 关于

作者:单人开发(仓库 245 次提交中 241 次)。真机使用 `jetson` 分支作为部署基线,
`Optimization` 为开发分支,`scripts/check_jetson_sync.sh` 防止两条线漂移。

> 详细的失败模式分类、证据链与修复状态见 **[docs/FAILURE_MODES.md](docs/FAILURE_MODES.md)**。
