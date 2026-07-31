# 工作记录

本文档记录已经影响当前实现的关键问题、修改和验证。历史方案只保留结论，不再作为运行手册。

## 2026-07-31 — 风险与待办审计

- 审计：对全仓代码、git 历史、WORKLOG 和 SITL 验证结果做系统风险排查。
- 产出：新增 `docs/RISKS_AND_TODOS.md`，分类为 6 项风险（3 高/2 中/1 低）+ 12 项待办（5 必须/4 应该/3 可以）。
- 关键风险：FSM 宽泛异常吞没、真机 land_detected 不可靠无 fallback、test_mission 3m 返航高度碰撞。
- 真机前必须：相机标定、v1 DDS 验证、舵机行程标定、接地检测 fallback。

## 2026-07-31 — 补充代码导读

- 反馈：总览文档压缩后，文件之间如何协作、如何循序阅读代码不够清晰。
- 修改：新增 `docs/CODE_READING_GUIDE.md`，按启动入口、接口协议、飞控适配、视觉管线、任务 FSM、一次任务调用链、故障定位和学习阶段组织内容。
- 入口：在 `docs/README.md` 和 `docs/ARCHITECTURE.md` 增加导读链接。
- 重点：明确 `MissionFSM`、`StandaloneBridge`、`px4_bridge`、`vision_pipeline` 和 `drone_servo2` 的 seam，区分完整任务、simple smoke test、遗留 Python 舵机和构建产物。

## 2026-07-14 — SITL 与真机差异审计

- 问题：仿真和真机使用不同 PX4 消息版本，旧文档把两者描述成同一接口。
- 结论：源码默认面向真机 v1；SITL 使用隔离 `px4_msgs` v4 overlay，并由 launch 覆盖 `vehicle_status_topic`。
- 当前状态：已通过 `scripts/sitl_env.sh` 固化环境边界；真机仍需使用工作区默认 v1。

## 2026-07-17 — 视觉模型和资源占用

- 问题：早期方案使用 `.pt`/PyTorch，并担心 Gazebo 与推理争用显存。
- 修改：当前运行配置使用 ONNX Runtime；视觉管线按模式加载模型，进入侦察阶段释放投放检测模型并执行垃圾回收。
- 结论：旧 TensorRT/`.pt` 迁移方案不再是当前运行步骤，不能作为部署要求。

## 2026-07-22 — 飞行方向切换

- 问题：旧设计文档以东向为默认，而当前场地已旋转为北向。
- 当前参数：Home World `(4,10)`；投放区 north `30..35`；侦察区 north `55..60`、east `-4..4`。
- 结论：文档和场景说明统一以 NED north 为主飞行轴，World 通过 `x=east, y=north` 转换。

## 2026-07-28 — H 标识视觉精准降落

- 问题：降落阶段需要真正使用视觉，不能把 PX4 LAND 当作完成条件。
- 修改：集成 `landing_h_yolo.onnx`、图像空间 P 伺服、置信度和跳变过滤；对心稳定后锁存，再在 1.5 m 进入纯垂直盲降。
- 安全约束：高空丢失、盲降超时或未完成对心时停止并进入错误处理；真机以 PX4 `vehicle_land_detected` 确认接地，SITL 才使用严格近地备用判定。

## 2026-07-30 — 完整 SITL 验证

日志：`log/simulation_20260730_192807`。

- FSM 完成 INIT → ARM → TAKEOFF → 两次投放 → RECON_PATROL → RETURN_TO_HOME → PRECISION_LAND → COMPLETE。
- 两次投放均成功，舵机序列执行到第二个释放位置。
- 侦察覆盖率达到 `1.0`，全路径扫描完成。
- H 视觉对心稳定 10/10，1.50 m 切入盲降。
- SITL 近地高度/速度条件持续满足，确认接地后停速并 force-disarm。
- `mission_end.success=True`，`drops_completed=2`，`coverage_ratio=1.0`，总时长约 199.95 s。

## 当前仍需真机验证

1. 640×480 相机内参、镜头安装方向和 H 目标实际尺度。
2. Jetson 与 CUAV 7-Nano 的 v1 DDS 连接、遥控器 Arm Switch 和 failsafe。
3. MAIN5 的 PWM 范围、舵机机械行程和两次投放实物结果。
4. 真机 `vehicle_land_detected` 的触发时序以及接地后的自动/强制解锁策略。

这些项目不能由 SITL 成功日志替代，现场操作必须遵循 [REAL_HARDWARE_DEBUG_GUIDE.md](REAL_HARDWARE_DEBUG_GUIDE.md)。
