# 无人机自主任务系统

这是一个基于 ROS 2、PX4 Offboard 和 Gazebo SITL 的自主任务工作空间。主任务由一个 FSM 编排飞控、视觉和舵机：完成两次投放、侦察区全覆盖、返航和视觉精准降落。

## 当前验证状态

2026-07-30 的完整 SITL 流程已跑通：

- 两次投放均成功；
- 侦察区覆盖率从扫描开始持续增长，最终达到 100%；
- H 标识视觉对心稳定锁存，1.50 m 切入盲降；
- SITL 接地判定、停速和强制解锁完成；
- `mission_end.success=True`，总时长约 200 s。

日志目录：[log/simulation_20260730_192807](/home/nvidia/CUADC_drone_missions_ws/log/simulation_20260730_192807)

## 先读什么

- [ARCHITECTURE.md](ARCHITECTURE.md)：当前代码结构、数据流、FSM 和控制算法。
- [CODE_READING_GUIDE.md](CODE_READING_GUIDE.md)：逐文件导读、运行时调用链和代码学习路径。
- [DEPLOYMENT.md](DEPLOYMENT.md)：构建、仿真、真机启动和日志操作。
- [REAL_HARDWARE_DEBUG_GUIDE.md](REAL_HARDWARE_DEBUG_GUIDE.md)：真机现场检查清单。
- [RISKS_AND_TODOS.md](RISKS_AND_TODOS.md)：风险清单和待办事项，真机飞行前的 blocking items。
- [WORKLOG.md](WORKLOG.md)：问题、修改、验证和遗留风险的工作记录。
- [field/README.md](../field/README.md)：Gazebo 场景和随机场地说明。
- [飞控参数修改记录](../real_flight_logs/飞控参数修改记录.md)：实际飞行后的 PX4 参数记录。

## 任务流程

```text
INIT → ARM → TAKEOFF → TRANSIT_TO_DROP_ZONE → CYLINDER_SEARCH
 → SELECT_TARGET → CYLINDER_APPROACH → DESCEND_TO_DROP
 → EXECUTE_DROP → ASCEND
 → (第二次投放前回到 SELECT_TARGET)
 → TRANSIT_TO_RECON_ZONE → RECON_PATROL
 → RETURN_TO_HOME → PRECISION_LAND → COMPLETE
```

投放区按圆筒记忆库和半径策略选目标，低空使用图像闭环精校后调用舵机服务。侦察区执行弓字形全路径扫描，覆盖监控只统计相机视场是否覆盖网格。降落阶段必须先识别 H、视觉对心并连续稳定，再进入纯垂直盲降；不调用 PX4 LAND 作为精准降落路径。

## SITL 与真机

两种部署共享任务逻辑，但通信适配不同：

| 项目 | SITL | 真机 |
|---|---|---|
| 启动 | `scripts/start_full_simulation.sh` | `real_flight.launch.py` |
| PX4 状态消息 | `/fmu/out/vehicle_status_v4` | `/fmu/out/vehicle_status_v1` |
| 消息环境 | `scripts/sitl_env.sh` 提升隔离 v4 `px4_msgs` | 工作区默认 v1 `px4_msgs` |
| 相机 | Gazebo 图像桥 | V4L2 相机节点 |
| 舵机 | `drone_servo2` SITL Mock | PX4 `DO_SET_ACTUATOR` → MAIN5 |
| 接地确认 | 严格的高度/速度保持条件备用判定 | PX4 `vehicle_land_detected` |

因此，“真机发布 v1、仿真使用 v4”是部署适配，不是任务逻辑分叉。详细边界见 [ARCHITECTURE.md](ARCHITECTURE.md)。

## 构建与测试

```bash
cd /home/nvidia/CUADC_drone_missions_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
pytest -q src/drone_mission/test src/drone_vision/test src/px4_control/test
```

完整仿真前需准备 PX4 SITL、Gazebo Harmonic、MicroXRCEAgent，以及 `src/drone_vision/drone_vision/models/` 中配置引用的 ONNX 模型。运行：

```bash
./scripts/start_full_simulation.sh
```

脚本会创建 `log/simulation_YYYYMMDD_HHMMSS/`，包括 PX4、视觉桥、ROS 2 系统和任务事件日志。

## 仓库结构

```text
src/
├── drone_interfaces  # 自定义 msg/srv/action
├── px4_control       # PX4 DDS、NED 指令和遥测
├── drone_vision      # 相机、ONNX 检测、覆盖监控、视觉伺服
├── drone_servo2      # C++ 多位置舵机投放
├── drone_mission     # 任务 FSM、路径规划、投放和降落状态
├── drone_bringup     # SITL/真机 launch
└── test_mission      # 分阶段真机测试工具
field/                # Gazebo 场景
scripts/              # 构建、环境和启动脚本
docs/                 # 当前说明、部署手册和工作记录
```

## 文档维护规则

运行规范只写入 `README.md`、`ARCHITECTURE.md` 和 `DEPLOYMENT.md`。问题复盘写入 `WORKLOG.md`，并标注“已验证”“仅 SITL 验证”或“待真机验证”。旧方案不再作为运行说明保留。
