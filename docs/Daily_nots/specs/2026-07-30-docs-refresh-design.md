# 文档体系整理设计

日期：2026-07-30

## 目标

把 `docs/` 从按日期堆积的开发草稿，整理成能够指导当前 SITL 验证和真机部署的文档入口。
文档必须以当前源码、启动脚本、参数文件和最近一次完整仿真结果为事实来源。

## 读者与使用场景

1. 首次接手项目的人：先看项目做什么、如何构建和运行。
2. 调试任务流程的人：查看状态机、视觉模式、坐标、话题和异常处理。
3. 部署真机的人：明确真机 v1 与 SITL v4 的接口差异、设备连接和安全检查。
4. 复盘任务的人：从工作记录查找已验证结果、已知限制和历史决策。

## 文件布局

### 保留并重写

- `docs/README.md`：唯一入口，提供快速开始、任务流程、SITL/真机差异和文档索引。
- `docs/ARCHITECTURE.md`：当前实现的架构、状态机、数据流、算法和资源生命周期。
- `docs/REAL_HARDWARE_DEBUG_GUIDE.md`：更新为真机现场检查和安全操作清单。

### 新增

- `docs/DEPLOYMENT.md`：构建、SITL 启动、真机启动、日志定位和停止流程。
- `docs/WORKLOG.md`：合并历史文档中仍有价值的技术结论，并记录 2026-07-30 完整仿真证据。

### 删除

以下文件是已被当前实现取代的提案、规划或旧版记录；有价值内容先迁移到 `WORKLOG.md`：

- `docs/2026-07-14-sitl-audit-and-real-hardware-gaps.md`
- `docs/2026-07-15-vision-precision-improvement-proposals.md`
- `docs/2026-07-16-vision-false-positive-analysis-and-solutions.md`
- `docs/2026-07-17-crash-prevention-and-performance-plan.md`
- `docs/2026-07-17-real-hardware-workspace-plan.md`
- `docs/2026-07-17-vision-fps-optimization.md`
- `docs/CHANGELOG_2026-07-19.md`
- `docs/rush-strategy-design.md`
- `docs/superpowers/specs/2026-07-22-flight-direction-design.md`

删除理由：这些文档含有旧的东向布局、`.pt`/TensorRT 尚未实施方案、旧接口、旧参数和“待实施”结论；继续保留会让读者误以为它们是当前运行规范。

### 不删除

- `field/README.md`：场景生成器仍是 SITL 的直接操作说明，后续只修正与当前北向布局不一致的描述。
- `real_flight_logs/飞控参数修改记录.md`：这是实际飞行后的参数记录，不属于过时方案。
- `src/px4_msgs/README.md` 等第三方包文档：属于 vendored 依赖，不纳入本次业务文档重写。

## 内容边界

### README

只回答“项目是什么、怎么运行、当前是否验证通过、应该继续读什么”。不复制完整参数表和代码实现细节。

### ARCHITECTURE

以当前代码为准，覆盖：

- `px4_control`、`drone_vision`、`drone_servo2`、`drone_mission`、`drone_interfaces` 的职责。
- FSM：起飞、两次投放、侦察全覆盖、返航和视觉精准降落。
- ROS2 话题/服务和关键接口。
- NED/World 坐标关系以及北向场地参数。
- 真机 v1 / SITL v4 的消息适配方式。
- 视觉对心、盲降、SITL 接地备用判定、强制停速/解锁和资源释放。
- 测试和当前验证证据。

### DEPLOYMENT

只记录可执行命令和前置条件。仿真使用 `scripts/start_full_simulation.sh` 与 `scripts/sitl_env.sh`；真机使用 `real_flight.launch.py`，并明确相机、串口/UDP、遥控器和 PX4 参数要求。

### WORKLOG

按日期记录“问题—根因—修改—验证—遗留风险”，不保留大段已经失效的设计代码或重复参数表。

## 当前事实基线

- 完整 SITL 日志：`log/simulation_20260730_192807`。
- 结果：两次投放成功，侦察覆盖率达到 100%，H 视觉对心锁存后盲降，SITL 接地确认，任务 `success=True`。
- 精准降落不调用 PX4 LAND；真机以 PX4 `vehicle_land_detected` 为主，SITL 使用严格的近地速度/高度备用判定。
- 视觉进入侦察阶段会释放投放检测模型；任务结束会停止 FSM 控制循环。
- 真机默认订阅 `/fmu/out/vehicle_status_v1`；SITL 通过隔离的 `px4_msgs` v4 overlay 使用 `/fmu/out/vehicle_status_v4`。

## 自检要求

完成后检查：

1. 所有 Markdown 链接都指向存在的文件。
2. 文档中的命令、包名、launch 文件和参数名能在仓库中找到。
3. 不再出现已删除文档路径、旧东向坐标、旧 `.pt` 运行模型或 PX4 LAND 作为精准降落路径。
4. 文档事实与源码和 2026-07-30 日志一致。
