# 工作记录

本文档记录已经影响当前实现的关键问题、修改和验证。历史方案只保留结论，不再作为运行手册。

## 2026-10-13 — 精准降落链与两类启动参数崩溃的现状复核

- 当前 full 与 quick 降落均已将 `LandingTarget.header.stamp` 贯通到伺服和目标估计器；FSM 只在新测量上融合，并用 `pose_history.py` 查询采集时刻的载机位姿。H 目标消费端在最后一次含检测消息超过 2 s 后返回无目标；full 的 `landing_blind_timeout` 已接线。失去目标或估计器拒帧时，有成熟落点且高于近地横移下限才有界地继续靠近。伺服卡方门默认关闭，参数待实测标定。
- 历史真机日志取证：视觉约 10.5 Hz（2175 条样本），采集到消费延迟均值约 95 ms。它们用于核对多速率代码前提，不证明新降落链真机有效。`landing_desk_sim.py` 的定量收益结论因时钟与真实 FSM 路径不一致已撤回，工具只用于定性检查和发现自相矛盾。
- 2026-07-23 `px4_bridge.home_z` 的 STRING→DOUBLE 崩溃与 2026-08-06 `mission_fsm` YAML INTEGER→DOUBLE 崩溃都有真机 exit code 1 日志。当前 5 个 launch 的浮点覆盖值已显式转 double，`test_mission_params_contract.py` 同时检查 YAML double 值与 launch 下发类型；两条守卫已做变异验证。
- 验证级别：上述代码改动为 L1 单元/契约验证，历史日志为已有真机负载和故障证据；本分支尚无完整 SITL 或真机复飞。详细待办见 [PRECISION_LANDING_PLAN.md](../PRECISION_LANDING_PLAN.md)，残余接地与断流风险见 [RISKS_AND_TODOS.md](RISKS_AND_TODOS.md)。

## 2026-08-06 — 像素误差参数去掉小数点 → mission_fsm 启动崩溃，真机无法起飞

- 场景：真机全流程启动后无人机不动作，舵机日志反复"等待任务租约 (autonomy_lease)"，FSM 从未进入任何飞行状态。
- 证据：`mission.log` —— `mission_fsm` 在 `_declare_params` 崩溃（exit 1）：
  `rclpy.exceptions.InvalidParameterTypeException: Trying to set parameter 'precision_align_raw_pixel_tolerance' to '35' of type 'INTEGER', expecting type 'DOUBLE'`。舵机等租约是下游症状（FSM 死亡 → 租约永不发布）。
- 根因：提交 `4f0d8a0` 把三处 double 参数去掉小数点（`9.0→35`、`7.0→35`、`12.0→40`），YAML 解析为 int；`declare_parameter(name, 9.0)` 声明为 double，rclpy 对覆盖值类型严格校验，启动即抛异常。共三处，修掉一个还会接着崩下一个（`precision_align_smooth_pixel_tolerance`、`landing_servo_deadzone_px`）。
- 修改：`mission_params_real.yaml` 三处补小数点（`35.0`/`35.0`/`40.0`）。真机 `--symlink-install` 下改 YAML 无需重新编译，重启 launch 即可。
- 验证：`python3 -c "import yaml; ..."` 确认三值类型为 float；全量交叉核对 114 个 float 声明参数与 yaml 值，无其余不匹配。**待真机复飞确认**。
- 教训：改 YAML 数值时 float 参数必须保留小数点；改完先验类型再上机。完整规范见 [PARAMETER_USAGE_GUIDE.md §1.4](PARAMETER_USAGE_GUIDE.md)。

## 2026-08-05 — 启动架构改为 test_mission 模式（脚本持有 Agent）

- 场景：`start_full_real_mission.sh` 用作 Jetson 开机自启后，地面站 `uxrce_dds_client status` 一致"无法连接"。根因：旧版脚本把 Agent 埋在 launch 里，前面压着 SITL overlay 硬失败、相机存在性、`command -v` 等 fail-fast 闸门；自动启动环境下（cron/systemd 的 PATH、开机早期 USB 相机未枚举、shell 继承 SITL overlay env）任一闸门挂掉 → launch 根本没跑 → 8888 上无 Agent → PX4 客户端永远连不上。
- 修改（对齐 `test_forward_recon.sh` 结构）：
  1. `scripts/start_full_real_mission.sh` 重写：Step 0 清理旧进程（保证 UDP 端口唯一）→ Step 1 **无条件先启动 MicroXRCEAgent**（先于一切任务闸门）→ Step 2 等 DDS 就绪 → Step 3 相机限时等待+警告（不再硬失败）→ Step 4 launch 任务节点；`--agent-only` 模式前台持有 Agent，适配 systemd Type=simple 自启；EXIT trap 回收 Agent（修复 Ctrl+C 残留）。
  2. 环境自愈替代硬失败：自动补齐 PATH（/opt/ros/humble/bin、/usr/local/bin）；检测到 `sitl_px4_msgs` overlay 时从 AMENT_PREFIX_PATH/CMAKE_PREFIX_PATH/LD_LIBRARY_PATH/PYTHONPATH/COLCON_PREFIX_PATH 剥离而不是退出（保留 v1/v4 消息冲突防护，WORKLOG 08-02 "UnsupportedTypeSupport"）。
  3. `real_flight.launch.py` 移除自持 Agent 与 `agent_exit_guard`（双 Agent 会同端口冲突互杀）；等价安全契约由脚本 watchdog 承担：Agent 意外退出 → 自动停止任务 launch。
  4. 契约测试同步：`test_real_mission_scripts.py` 改断言"脚本持有唯一 UDP Agent + watchdog + 剥离逻辑"；`test_px4_interface_deployment_contract.py` 断言 launch 不再含 Agent/ExecuteProcess，真机脚本三件套统一 `udp4 -p "$DDS_PORT"`。
- 验证：`bash -n`、两测试文件 13 passed。**待真机验证**：自启后 QGC `uxrce_dds_client status` 应显示连接；agent.log 应出现来自 `192.168.1.10` 的 UDP 会话。
- 后续收敛（2026-08-05 同日）：按现场反馈（test_mission 开机自启稳定，全流程自启失败），脚本与 test_mission 进一步对齐——**移除相机限时等待**（不设脚本级相机检查，由 launch 与 verify 脚本处理），步骤改为 Step 0-3/3；自启配置定为直接运行全流程脚本（`--agent-only` 降级为飞控晚于 Jetson 上电 >60s 时的替代）；文档 §4.1/DEPLOYMENT §5 同步。断点定位结论：两脚本逻辑同构、环境正常时，全流程自启失败仅剩 install/ 旧版 launch（自带 Agent → 双 Agent → guard 关整机）与 Jetson 未同步两种部署状态，均需 `git pull` + `colcon build --packages-select drone_bringup --symlink-install` 解决。
- 可观测性完善（2026-08-05 同日）：脚本全部输出镜像到 `startup.log`（`exec > >(tee -a ...)` + 就绪屏障 1s，防 fork 竞态丢极早期输出）——自启下不再出现"日志空"盲区；`DDS_WAIT_SECONDS` 环境变量可配置 DDS 等待上限（默认 60，与 test_mission 一致）。隔离测试验证极早期错误可落盘。

## 2026-08-05 — 根因定案：`set -u` 与 ROS setup.bash 的未保护变量

- 现场直接复现：`./scripts/start_full_real_mission.sh` 终端报 `/opt/ros/humble/setup.bash: 行8: AMENT_TRACE_SETUP_FILES: 未绑定的变量`。
- 根因：a777457 起启动脚本用 `set -euo pipefail`（含 `-u`）；`/opt/ros/humble/setup.bash` 第 8 行 `if [ -n "$AMENT_TRACE_SETUP_FILES" ]` **无 `:-` 保护**，变量未赋值时（全新环境 = cron/systemd 自启、干净终端）`-u` 直接报错退出。**脚本死在 source ROS 这一步——早于一切检查、早于 Agent**。旧脚本 mkdir（25 行）在 source（31 行）之前、launch_arguments.txt（59 行）之后 → 目录空；Agent 永不起 → disconnected；进不了任务。
- 为何 test_mission 免疫：`set -eo pipefail`（无 `-u`），实测 `env -i PATH=/usr/bin:/bin bash -c 'source ...'` 可正常加载。
- 为何此前分析错方向：仓库测试 `test_sitl_px4_msgs_builder_sources_ros_before_enabling_nounset` 早已记录"ROS Humble setup reads optional variables and cannot run under set -u"（为 build 脚本写的约束），a777457 在启动脚本里违反了同一约束；"六道闸门"分析未意识到脚本死在闸门之前。
- 修复：`start_full_real_mission.sh` 与 `verify_real_flight_ready.sh` 改为 `set -eo pipefail`（与 test_mission 一致）；契约测试断言同步（`set -eo pipefail` + 不得含 `-u`），防止回归。验证：13 passed；fresh-env 实测 source 通过。
- 教训：凡 source ROS/colcon 生成 setup 的脚本**一律不得启用 nounset**；已有契约测试防的是 build 脚本，启动类脚本需要同样覆盖（已补）。

## 2026-08-05 — 真机链路代码审计：四个缺口

- 场景：对真机飞行全套代码（launch / 启动脚本 / 预检脚本 / bridge / FSM / 视觉 / 舵机）的针对性审计，重点是真机独有的失效模式。
- 发现（修复方向与待办已录入 [RISKS_AND_TODOS.md](RISKS_AND_TODOS.md) 风险 #2/#7-#11、待办 T5/T13-T16）：
  1. **视觉"消息停更"不可检测**：`get_landing_target()`（`mission_state_machine.py:1264`）无新鲜度门，`_landing_target_cb`（:710）不记时间戳；`vision_pipeline.py:484` 的 `ready` 只需 `frame_count>0`，相机死后永久 `ready=1`；emergency 只在收到消息时更新，且视觉丢失的 `WARN` 不被主循环消费（:757-767）→ 相机/视觉进程死亡时，降落伺服拿冻结目标继续下降，视觉丢失保护失效。对照：cylinders/markers/hazards 均有 2s 新鲜度门，landing_target 属遗漏。
  2. **相机内参占位且被固化**：`real_flight.launch.py` 硬编码 `fx=fy=400, cx=320, cy=240`（未标定）；`verify_real_flight_ready.sh:66-68` 断言这些值 → 完成标定后预检必失败，检查脚本与 T1 待办矛盾。
  3. **落地确认链不一致**：静止兜底 `_fallback_touchdown_confirmed()`（`state_precision_land.py:329`）无 sim_mode 门控（代码开），yaml 注释允许，RISKS 文档写"真机没有"（已过时）；真机无距离传感器（`SYS_HAS_NUM_DIST=0`），0.10m 高度容差靠气压计不可靠；`landing_blind_timeout`（`mission_state_machine.py:344`）声明后从未接线，`_blind_start_time` 只写不读 → 盲降无硬性时间上限。
  4. **`COM_OBL_RC_ACT=0` 未追踪**：Jetson/Agent 崩溃时 PX4 无固件级自动处置，仅依赖操作者接管；参数记录自注"建议改为 2=Land"但从未进入待办。
  5. 次要：`state_max_duration=0.0` 魔法值禁用状态超时层（`emergency.py:199`）；PX4Bridge 缺 `stop_autonomous_output`/`hold_for_takeover`（`mission_state_machine.py:947,973`），接口漂移。
- 状态：**尚未修改代码**；本次为审计与待办录入，修复留待真机复飞前逐项清零（T1-T5、T13-T16）。

## 2026-08-02 — 版本回退后构建产物残留导致启动崩溃

- 现象：07:20 实飞成功（`real_flight_20260802_072045`）；git pull 新版本（Full `62a5cdf`，新增 `NedPositionSetpoint.msg` 并重构速度控制）亦成功；回退到初版本（jetson 分支，无该消息）后，09:15 / 09:25 两次任务**启动瞬间全部失败**。
- 证据：两次 `mission.log` 逐字一致 —— `vision_pipeline` 与 `test_forward_recon` 均在 `__init__` 创建话题/服务时崩溃（exit code 1）：
  `ImportError: .../drone_interfaces_s__rosidl_typesupport_c.cpython-310-aarch64-linux-gnu.so: undefined symbol: drone_interfaces__msg__ned_position_setpoint__convert_to_py` → `UnsupportedTypeSupport`。任务从未进入 FSM。
- 根因：回退后增量 `colcon build` **只重建不回收** —— C typesupport 库已按回退后的源码重新生成（该符号被移除），Python 静态 typesupport 扩展仍是新版本构建的残留（引用已删除的符号），运行时 dlopen 符号解析失败。编译期不报错，运行期每次启动必现。
- 教训：修改消息接口或切换 / 回退版本后**必须全量清理重建**（`rm -rf build install log && colcon build --symlink-install`），不要在增量构建上赌依赖判断。规范见 [DEPLOYMENT.md §2 构建产物清理](../Deployment/DEPLOYMENT.md)。
- 修复验证：待真机全量重建后复飞确认。

## 2026-08-02 — 真机通信链路纠正

- 发现：完整真机 launch、舵机硬件测试和部分文档仍把 PX4↔Jetson 通信写成 Jetson UART 串口。
- 证据：PX4 参数为 `UXRCE_DDS_CFG=1000`、Agent IP `192.168.1.11`、端口 `8888`；实飞 Agent 日志显示 UDP 会话来自 PX4 `192.168.1.10`。
- 修改：真机完整 launch、简化真机脚本、前飞侦察脚本和舵机测试统一为 `MicroXRCEAgent udp4 -p 8888`，并把 DDS 就绪检查收紧为 `vehicle_status_v1`。
- 物理拓扑：相机使用 USB 接入 Jetson；PX4 使用以太网 UDP 接入 Jetson；舵机由 PX4 MAIN5 驱动。
- 待现场确认：Jetson UDP 端口唯一占用、PX4 Ethernet 参数、DDS v1 话题与真实图像/舵机状态。

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
