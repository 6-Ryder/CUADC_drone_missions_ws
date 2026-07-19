# 真机分阶段部署 + 可测试性方案

> 日期: 2026-07-19
> 状态: 设计阶段，待实施

---

## 核心思路：两个维度的拆分

| 维度 | 说明 |
|------|------|
| **纵向切分** | 按阶段递进，每一阶段只部署验证一小部分 |
| **横向切分** | 在任一阶段都能单独测试某个模块/某个状态 |

关键设计原则：**在 FSM 中加一个 `mission_mode` 参数，让同一个代码既跑完整任务，也能跑任意子任务，避免维护两套代码。**

---

## 分阶段部署路线图（6 个 Phase）

### Phase 0 — 地面静态验证（不飞）

**目标**：确认全部软硬件链路通畅，不冒任何飞行风险。

```
真机通电 → 启动 ROS2 系统 → 验证所有节点心跳正常
```

需要做的事：
- 在 `real_flight.launch.py` 基础上做一个 `ground_test.launch.py`，启动全部节点但不进入 Offboard
- 验证 PX4 Bridge 注册成功（看 `/drone/pose` 是否有数据）
- 验证相机出图（`/drone/camera/image_raw`）
- 验证视觉 Pipeline 推理正常（看 `/drone/vision/cylinders` 是否有输出，手持目标在镜头前晃）
- 验证舵机响应（`/drone/servo/trigger_drop` 服务调用是否正常动作）
- 验证 TensorRT 引擎加载成功、帧率达标

**产出**：确认 4 个节点（px4_bridge / vision_pipeline / servo_controller / mission_fsm）在 Jetson 上全部正常启动。

---

### Phase 1 — 基础飞行验证（手动 + Offboard 悬停）

**目标**：验证 PX4 Offboard 控制环路在真机上正常工作，不涉及任何视觉/任务。

需要做的事：
- 新增 `basic_flight.launch.py`：只启动 `px4_bridge`，不启动 vision/servo/mission
- 写一个极简的 `test_hover.py` 脚本：Takeoff → 5m 悬停 30s → Land
- 或者直接用 `mission_fsm` 但加 `mission_mode: hover_test`，只走 INIT→ARM→TAKEOFF→悬停→Land

**关键验证点**：
- Offboard 模式切换是否稳定
- 高度控制稳态偏差（SITL 已知 0.3m 偏差，真机可能不同）
- 位置保持精度
- 手动切回 Position/Stabilized 模式的紧急接管流程

---

### Phase 2 — 单项技能验证（不执行完整任务）

**目标**：每个技能模块在真实环境中独立验证。

采用 FSM 的 **单状态运行模式**——`mission_mode` 参数控制：

```yaml
# mission_params.yaml 新增
mission_mode: "full"  # full | hover_test | drop_search_only |
                      # drop_approach_only | drop_descend_only |
                      # drop_full | recon_patrol_only |
                      # precision_land_only | transit_test

# 单状态测试参数
single_state_test:
  target_state: "CYLINDER_SEARCH"  # 直接跳到这个状态
  target_state_params: {}           # 覆盖该状态的参数
  auto_land_after: true             # 测试完自动降落
```

具体测试项：

| 测试项 | mission_mode | 飞行内容 |
|--------|-------------|---------|
| 投放区搜索 | `drop_search_only` | 起飞 → Transit → 投放区搜索（不投放）→ RTL |
| 投放区微调 | `drop_descend_only` | 起飞 → 手动定位到桶上方 → 仅执行下降+精校+投放 |
| 侦察区巡逻 | `recon_patrol_only` | 起飞 → Transit → 侦察区 zigzag → RTL |
| 精确降落 | `precision_land_only` | 起飞 → 悬停 → 搜索 H 标志 → 视觉伺服降落 |

---

### Phase 3 — 分段任务验证

**目标**：验证相邻阶段的衔接逻辑。

| 分段 | 状态链 | 验证重点 |
|------|--------|---------|
| 投放段完整 | TAKEOFF → ... → ASCEND（2轮投放） | 搜索→选择→接近→投放 的完整循环 |
| 侦察段完整 | TAKEOFF → TRANSIT_TO_RECON → RECON_PATROL → RTL | 覆盖率、zigzag 路径执行 |
| 返航+降落段 | TAKEOFF → 任意点 → RTL → PRECISION_LAND | GPS/视觉融合降落 |

---

### Phase 4 — 完整任务（保守参数）

**目标**：首次端到端完整任务，用最保守的参数。

- 降低所有速度 30%
- 加长所有 timeout 2x
- 增加中间悬停检查点
- 全程有安全飞手待命接管

---

### Phase 5 — 性能调优

**目标**：逐步收紧参数，达到任务指标。

这个时候就需要"只测某一小段就能调参"的能力了。

---

## 实现方案：如何在 FSM 中加入可测试性

### 1. 新增 `mission_mode` 参数

在 `mission_params.yaml` 中：

```yaml
mission_mode: "full"  # 完整任务

# 子任务模式定义
sub_mission:
  enabled: false                    # true 时使用子任务模式
  entry_state: "CYLINDER_SEARCH"    # 从哪个状态开始
  exit_state: "ASCEND"              # 到哪个状态结束（完成后 RTL）
  max_drops: 1                      # 覆盖投放次数
  skip_transit_to_target: false     # 跳过 transit，直接从当前位置开始

# 调试开关
debug:
  pause_at_state_entry: false       # 进入每个状态前悬停等待确认
  state_timeout_override: 0         # 0=使用默认, >0=覆盖超时秒数
  log_extra_telemetry: false        # 额外遥测日志
```

### 2. FSM 中注入状态跳转逻辑

在 `mission_state_machine.py` 的初始化中：

```python
# 如果 sub_mission 启用，修改状态转换表
if self.sub_mission_enabled:
    # 修改入口：INIT → ARM → TAKEOFF → TRANSIT_TO_TARGET → entry_state
    # 修改出口：exit_state → RETURN_TO_HOME（而不是原本的下一状态）
    self._patch_transitions_for_sub_mission()
```

关键：**不修改原有状态 handler 的代码**，只修改状态转换表。这样保证测试的代码和正式代码完全相同。

### 3. 各场景的启动配置

为每个 Phase 创建对应的 launch 文件和参数覆盖：

```bash
# 投放区完整测试
ros2 launch drone_bringup real_flight.launch.py \
  mission_mode:=drop_full \
  max_drops:=2

# 侦察区巡逻测试
ros2 launch drone_bringup real_flight.launch.py \
  mission_mode:=recon_patrol_only

# 精确降落测试
ros2 launch drone_bringup real_flight.launch.py \
  mission_mode:=precision_land_only

# 单状态测试：只测投放精校
ros2 launch drone_bringup real_flight.launch.py \
  sub_mission.enabled:=true \
  sub_mission.entry_state:=DESCEND_TO_DROP \
  sub_mission.exit_state:=EXECUTE_DROP
```

---

## 性能调优的快速迭代方案

### 参数热重载

在 FSM 中加入一个 ROS2 service `/drone/mission/reload_params`，调用时重新读取 YAML 并更新运行中参数：

```python
# 在 mission_fsm 中
self.reload_params_srv = self.create_service(
    Trigger, '/drone/mission/reload_params', self._reload_params_cb
)
```

这样调参时不需要重新启动整个系统，改 YAML → 调 service → 立即生效。

### 录包回放（最安全的方式）

对于纯视觉参数调优（目标检测阈值、跟踪参数、伺服 PID），不需要反复飞行：

```bash
# 录制一次飞行
ros2 bag record /drone/camera/image_raw /drone/pose /drone/vision/cylinders

# 地面回放调参
ros2 bag play <bag_file> --loop
# 同时跑 vision_pipeline，调 YAML 参数看效果
```

这对 `CYLINDER_APPROACH` 的像素伺服 PID 调参特别有用——飞一次录下来，地面上反复调。

### 关键性能指标监控

在 FSM 中实时打印每个状态的性能指标，飞行后分析：

```python
# 每个状态退出时记录
self.state_metrics = {
    "CYLINDER_SEARCH": {"duration": 12.3, "targets_found": 5, "avg_confidence": 0.72},
    "DESCEND_TO_DROP": {"duration": 8.1, "align_error_final": 12, "retries": 0},
    "RECON_PATROL": {"duration": 45.2, "coverage_ratio": 0.97, "strips_completed": 4},
    ...
}
```

---

## 推荐的实施优先级

```
第1优先: Phase 0（地面静态验证）
  工作量：~半天
  风险：零

第2优先: Phase 1（基础飞行）+ mission_mode 框架
  工作量：~1天（代码框架）+ ~半天（外场试飞）
  风险：低（只有基础飞行，随时可切回手动）
  产出：mission_mode 框架后续所有阶段都能复用

第3优先: Phase 2 中最关心的模块（比如投放区搜索+微调）
  工作量：~1-2天
  风险：中（需要目标物布置）

第4优先: Phase 3（分段）+ Phase 4（完整保守）
  工作量：~2-3天
  风险：中高

第5优先: Phase 5（性能调优）
  工作量：持续迭代
  配合录包回放大幅减少飞行次数
```

---

## 启动方式汇总

| 启动命令 | 用途 | 风险 |
|---------|------|------|
| `ground_test.launch.py` | 地面静态验证 | 零 |
| `basic_flight.launch.py` + `test_hover.py` | 基础悬停测试 | 低 |
| `real_flight.launch.py mission_mode:=drop_search_only` | 投放区搜索 | 中 |
| `real_flight.launch.py mission_mode:=drop_descend_only` | 投放精校 | 中 |
| `real_flight.launch.py mission_mode:=recon_patrol_only` | 侦察区巡逻 | 中 |
| `real_flight.launch.py mission_mode:=precision_land_only` | 精确降落 | 中 |
| `real_flight.launch.py mission_mode:=drop_full` | 投放段完整 | 中高 |
| `real_flight.launch.py mission_mode:=full` | 完整任务 | 高 |
| `ros2 bag play` + `vision_only.launch.py` | 视觉参数离线调优 | 零 |

---

## 相关文档

- [[2026-07-17-real-hardware-workspace-plan]] — 真机部署工作空间计划
- [[2026-07-14-sitl-audit-and-real-hardware-gaps]] — SITL 与真机差异审计
- [[ARCHITECTURE]] — 系统四层架构总览
