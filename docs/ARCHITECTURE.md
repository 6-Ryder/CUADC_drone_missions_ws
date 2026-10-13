# 当前系统架构

> 截至 2026-10-13，描述 `Optimization` 分支的当前实现；降落改动尚未经过完整 SITL 或真机复飞。2026-07-30 的完整 SITL 成功记录只证明当时版本。

本文档描述仓库当前实现，不描述已经废弃的方案。代码事实优先级高于本文档；参数以各包 `config/*.yaml` 为准。

## 1. 设计不变量

1. 任务由 `drone_mission` 的单一 FSM 编排，飞控、视觉和舵机通过 ROS 2 接口协作。
2. 飞行指令使用 PX4 NED：`x=north`、`y=east`、`z=down`；向上高度对应负 z。
3. `layout_heading_deg` 是唯一的场地方位：`0°` 为 NED 北、顺时针为正；任务启动时冻结。SITL Home World 为 `(4,10,0)`，真机 Home 为解锁点 `(0,0,0)`。
4. full 精准降落使用视觉 H 对心；视觉丢失或盲降超时请求人工接管。quick 的 `landing_simple_mode=true` 有独立的 PX4 LAND 兜底。
5. 任务结束停止 FSM 控制定时器；视觉在进入侦察阶段释放投放模型，避免无用显存和句柄继续占用。

## 2. 包职责

| 包 | 职责 | 主要实现 |
|---|---|---|
| `drone_interfaces` | ROS 2 自定义接口 | `NedPositionSetpoint`、`CylinderArray`、`LandingTarget`、`CoverageStatus`、`SetVisionMode`、`TriggerDrop` |
| `px4_control` | PX4 DDS 适配和指令输出 | `px4_bridge.py`、`standalone_bridge.py`、`flight_controller.py` |
| `drone_vision` | 相机、ONNX 推理、目标精修、覆盖监控、视觉伺服 | `vision_pipeline.py` 及检测器/控制器 |
| `drone_servo2` | 多位置投放控制 | C++ `servo_controller.cpp`，SITL Mock 或 Jetson PX4 执行器 |
| `drone_mission` | 任务状态机和策略 | `mission_state_machine.py`、`states/`、`path_planner.py` |
| `drone_bringup` | 快速/主任务 × SITL/真机启动组合；按平台加载机体几何 | `quick_sitl.launch.py`、`sitl.launch.py`、`quick_flight.launch.py`、`real_flight.launch.py`、`airframe_params_{sitl,real}.yaml` |
| `test_mission` | 分阶段真机测试 | `test_forward_recon` 等测试节点 |

## 3. 运行拓扑

```text
Gazebo 相机 / V4L2 相机
          │ Image
          ▼
   drone_vision
  ├─ cylinders ───────────────┐
  ├─ markers                   │
  ├─ landing_target            │
  ├─ coverage                  │
  ├─ servo_cmd ──► px4_control│
  └─ set_mode ◄───────────────┘
                                ▼
                         drone_mission FSM
                          │          │
          TriggerDrop ◄───┘          └── NED setpoint/velocity
                │                              │
          drone_servo2                    px4_control
                │                              │
             MAIN5/PWM                         PX4
```

运行时由 launch 启动四类节点：PX4 bridge、视觉管线、舵机控制器、任务 FSM。真机由启动脚本 `scripts/start_full_real_mission.sh` 先启动唯一 Ethernet/UDP MicroXRCEAgent 并等待 DDS 就绪，launch 再启动 USB V4L2 相机等节点；SITL 额外启动 Gazebo 图像桥。

## 4. 坐标与场地

### 4.1 坐标链

PX4 遥测和指令采用 NED。Gazebo World 使用 `x=east`、`y=north`。任务通过 `telemetry.py` 的转换和 `home_x/home_y` 将两者关联：

```text
World x = home_x + NED east
World y = home_y + NED north
```

`LayoutFrame` 以 Home 为原点定义场地局部坐标：`forward` 指向
`layout_heading_deg`，`right` 是面对 forward 时的右侧。设 `h` 为 heading 的弧度：

```text
north = forward * cos(h) - right * sin(h)
east  = forward * sin(h) + right * cos(h)
```

反变换用于将 World 目标和飞机位置送回局部区域判定。SITL 默认 Home 为
`(4,10)`；在 `0°` 时，侦察区 local `forward=55..60, right=-4..4` 对应
World `x=0..8, y=65..70`，投放区 local `forward=30..35, right=-4..4` 对应
World `x=0..8, y=40..45`。非零 heading 时不得把这些 World 范围当作固定边界。

### 4.2 关键区域

| 区域 | field-local 范围 | 高度 |
|---|---|---|
| 投放区 | forward `30..35`，right `-4..4` | 搜索 5.6 m，投放低空 2.8 m 精校后降至 1.5 m |
| 侦察区 | forward `55..60`，right `-4..4` | 2.5 m |
| H 起降点 | Home | 搜索 5 m，稳定对心后 1.5 m 盲降 |

### 4.3 方位启动与策略边界

以下命令是完整任务的操作入口；同一个 `layout_heading_deg` 同时传给任务、
视觉和 SITL 运行时场地。快速策略是方向改造的权威基线，但不会替换主任务的
投放、侦察、返航和精准降落策略。

```bash
# Quick authoritative baseline, north
./scripts/start_quick_simulation.sh layout_heading_deg:=0

# Quick authoritative 50-degree validation
./scripts/start_quick_simulation.sh layout_heading_deg:=50

# Main-strategy 50-degree regression
./scripts/start_full_simulation.sh layout_heading_deg:=50

# Real-flight quick/main entry points use the same parameter
./scripts/start_quick_mission_real.sh layout_heading_deg:=50
./scripts/start_full_real_mission.sh layout_heading_deg:=50
```

起飞进入 Offboard 后会在既有爬升过程中转向目标 heading；没有额外的 yaw
对准状态、yaw 容差门或等待超时。位置与速度控制均持续发布同一任务 yaw，
侦察条带只旋转位置，不随往返方向转机头。真机不生成 SDF；操作者必须在解锁
前确认实体场地朝向与启动参数一致。

## 5. 任务 FSM

状态定义在 `src/drone_mission/drone_mission/mission_state_machine.py`，状态行为在 `states/` 中拆分：

```text
INIT → ARM → TAKEOFF → TRANSIT_TO_DROP_ZONE → CYLINDER_SEARCH
 → SELECT_TARGET → CYLINDER_APPROACH → DESCEND_TO_DROP
 → EXECUTE_DROP → ASCEND
        ├─ drops < max_drops → SELECT_TARGET
        └─ drops complete → TRANSIT_TO_RECON_ZONE
 → RECON_PATROL → RETURN_TO_HOME → PRECISION_LAND → COMPLETE
```

任一状态异常、遥控器在飞行中解锁丢失、严重电量事件或关键视觉超时都会进入 `ERROR`，随后停止任务并写入失败摘要。`COMPLETE` 和 `ERROR` 都会写 `mission_end` 事件。

## 6. 视觉管线

`vision_pipeline` 是独立 ROS 2 节点，加载 ONNX 模型并按 FSM 的服务请求切换检测模式。`LandingTarget.header.stamp` 携带采集时刻；FSM 对“最后一次含检测的消息”执行 2 秒龄期门，并保留载机位姿历史供降落估计器按采集时刻对齐：

| 模式值 | 名称 | 作用 |
|---:|---|---|
| 0 | none | 停止检测/伺服输出 |
| 2 | landing | H 标识检测和降落伺服 |
| 3 | all | 圆筒、投放标识和可用检测 |
| 4 | coverage_reset | 重置覆盖网格 |
| 5 | coverage | 更新侦察覆盖 |

核心输出：

| 话题 | 类型 | 消费者 |
|---|---|---|
| `/drone/vision/cylinders` | `CylinderArray` | 投放搜索、目标记忆和选择 |
| `/drone/vision/markers` | `CylinderArray` | 投放标识兜底 |
| `/drone/vision/landing_target` | `LandingTarget` | 精准降落状态 |
| `/drone/vision/coverage` | `CoverageStatus` | 侦察状态和任务摘要 |
| `/drone/vision/servo_cmd` | `TwistStamped` | `px4_control` 速度转发 |
| `/drone/vision/status` | `String` | FSM 看门狗和就绪判断 |

视觉处理包含 YOLO/ONNX 检测、原图坐标恢复、OpenCV 精修、置信度和跳变过滤。投放模型在进入侦察阶段通过 `close()`、解除引用和垃圾回收释放；降落模型按需加载。

## 7. 两次投放

1. `CYLINDER_SEARCH` 在搜索高度收集目标，按世界坐标合并观测。
2. `SELECT_TARGET` 使用 `CylinderSelector` 避免已投目标，并按 `min_radius_first` 选择策略排序。
3. `CYLINDER_APPROACH` 以目标记忆和最新检测进行中转、慢速接近。
4. `DESCEND_TO_DROP` 在低空启用视觉精校：EMA 平滑像素误差、速度脉冲、短暂停稳、稳定帧和最终刹车复查。
5. `EXECUTE_DROP` 调用 `/drone/servo/trigger_drop`。舵机位置序列为 `1150 → 1700 → 2150 μs`，每次只前进一个位置。
6. 服务成功且确认时间满足后进入 `ASCEND`。达到 `max_drops=2` 才转入侦察。

## 8. 侦察覆盖

`RECON_PATROL` 启用 `coverage_reset` 后切换到 `coverage`，按 `PathPlanner` 生成的弓字形航线执行完整路径。`CoverageMonitor` 按相机 FOV 将网格单元标记为已覆盖并发布比例。

默认 `recon_require_full_path=true`，因此完整路径是任务条件，覆盖率是实时质量指标；仿真成功日志中最终覆盖率为 1.0。侦察主线不执行危险品分类。

## 9. 视觉精准降落

`PRECISION_LAND` 有 full（`state_precision_land.py`）与 quick/simple（`state_precision_land_simple.py`）两套状态实现。两者都先搜索 H、再以像素域 PID 对心下降，锁存后才进入纯垂直盲降；伺服增益由 `pid_tuning.yaml` 覆盖，当前 D 项为零。真机视觉日志实测约 10.5 Hz、采集到消费延迟均值约 95 ms，因此 PID 只按新测量推进，估计器融合时使用采集时刻的载机位姿。估计器维护目标世界位置，成熟后可在暂时失去目标或拒帧时继续朝最后已知位置靠近；近地低于配置高度、未锁存目标或超时后仍受状态兜底约束。伺服卡方拒帧开关 `landing_servo_estimator_gate_enable` 默认关闭，待实测标定。

full 在约 5 m 搜索，低于 2.5 m 才允许锁存，默认于 1.5 m 切盲降、以 0.30 m/s 下降。搜索无进展、视觉丢失重捕获失败或盲降超过 `landing_blind_timeout`（默认 45 s）后停止并请求人工接管，不调用 PX4 LAND。quick 真机采用 `landing_simple_mode=true`、1 帧像素死区锁存与 1.6 m 盲降切换；搜索、目标重捕获或盲降超时可请求 PX4 LAND。quick SITL 保留独立交接参数，不能把它的结果直接当作真机路径验证。

两条状态实现都优先读取 PX4 接地信号，也都有“已锁存且已进入盲降”后的高度、速度、保持时间备用判据；它没有 `sim_mode` 门，因此真机也可能使用。真机气压高度下该判据的可靠性尚待验证。细节和待决策项见 [PRECISION_LANDING_PLAN.md](PRECISION_LANDING_PLAN.md)。

## 10. PX4 与舵机控制

`px4_control` 接收 `/drone/cmd_setpoint_ned`（`NedPositionSetpoint`：NED 位置、yaw 和可选航段速度）、`/drone/cmd_velocity_ned` 和 `/drone/vision/servo_cmd`，向 PX4 发布 OffboardControlMode、TrajectorySetpoint 和 VehicleCommand；同时输出 `/drone/pose`、速度和状态供任务与视觉使用。位置航段速度由任务 YAML 配置，并受 `max_setpoint_speed_limit` 和 PX4 自身速度上限约束；视觉精校/降落的速度模式保持独立。

`drone_servo2` 通过 `/drone/servo/trigger_drop` 接收投放请求。SITL 使用 Mock，仅记录和模拟位置；Jetson 使用 `VehicleCommand` 的 `DO_SET_ACTUATOR` 经 PX4 MAIN5 输出 PWM。

## 11. SITL 与真机消息边界

源码默认参数面向真机：`/fmu/out/vehicle_status_v1`。`sitl.launch.py` 显式覆盖为 v4；`scripts/sitl_env.sh` 将隔离构建的 v4 `px4_msgs` overlay 提升到环境前端。两端保持同一任务接口，差异集中在消息类型和启动传输层。

```text
真机：USB 相机 → Jetson；ROS Humble + workspace(v1) → UDP Agent:8888 ← PX4 Ethernet
SITL：ROS Humble + workspace(v1) + isolated overlay(v4) → UDP Agent → PX4 SITL v4
```

不要在同一运行环境中混用 v1/v4 生成的 Python 消息类型；切换 SITL 前必须执行 `source scripts/sitl_env.sh`。

## 12. 安全、异常和资源生命周期

- FSM 看门狗监控任务、视觉和舵机心跳。
- 飞行中检测到需要 RC 解锁但实际已解锁丢失时立即错误退出。
- 电池严重告警可触发返航或中止。
- 视觉节点只在需要阶段加载对应模型，侦察阶段释放投放模型。
- 任务结束取消 20 Hz 控制定时器，视觉模式切回 none，记录结果摘要。
- 真机操作仍必须由操作者保留遥控器接管和硬件急停能力。

## 13. 验证证据

最近一次完整仿真：`log/simulation_20260730_192807/ros2_system.log`。
关键证据包括两条“投放成功”、覆盖率最终 `1.0`、视觉盲降、SITL 近地停滞确认接地和 `mission_end: success=True`。

代码测试与构建命令见 [README.md](README.md) 和 [DEPLOYMENT.md](Deployment/DEPLOYMENT.md)。
逐文件调用链和循序阅读路径见 [CODE_READING_GUIDE.md](CODE_READING_GUIDE.md)。

## 14. 已知限制

- SITL 的相机、动力学和接地检测不能替代真机飞行验证。
- 真机必须重新核对相机内参、V4L2 设备路径、PX4 v1 消息、MAIN5 输出和遥控器策略。
- `drone_servo` Python 包和危险品分类模块属于遗留/独立代码，不在当前完整任务主链路中。
