# 侦察区速度参数审计

> 审计日期: 2026-08-02
> 涉及文件: `mission_params.yaml`, `mission_state_machine.py`, `path_planner.py`, `flight_controller.py`, `px4_bridge.py`, `px4_params.yaml`, 所有 launch 文件及启动脚本

---

## 一、`sweep_speed` 是死参数（严重）

`_target_speed` 在整个代码库中只有两处引用，都是写入，从不读取：

```python
# px4_control/flight_controller.py:57   — 初始化
self._target_speed: float = 1.0

# px4_control/flight_controller.py:153  — 写入
self._target_speed = speed
```

**完整链路（断在 `flight_controller.py`）：**

```
mission_params.yaml:sweep_speed (0.8)
  → mission_state_machine.get_param("sweep_speed")
    → PathPlanner(sweep_speed=0.8)
      → waypoint["speed"] = 0.8    （每个覆盖航点都带）
        → state_recon_patrol._fly_to_wp()
          → fc.goto_ned(speed=0.8)
            → self._target_speed = 0.8   ← 死胡同，到此为止
```

**实际驱动飞行的是 `px4_bridge` 位置模式控制循环：**
- 逐航点发送 NED 位置 setpoint（position control）
- `max_acceleration` 斜坡平滑（`px4_bridge.py:445`）限制 setpoint 移动**加速度**
- PX4 内部 MPC 以自身允许的最大速度飞向每个位置 setpoint

**结论：`sweep_speed` 改多少都不会影响实际飞行速度。**

### 真正有效的速度控制杠杆

| 杠杆 | 位置 | 效果 |
|------|------|------|
| `MPC_XY_VEL_MAX` | PX4 参数 | PX4 内部水平速度硬上限 |
| `max_acceleration: 2.0` | px4_params.yaml | setpoint 斜坡加速度 (m/s²) |
| `grid_spacing: 1.67` | mission_params.yaml | 条带越密航点越多，总时间越长（间接） |

### 修复方向

**方案 A（不改代码）**：调 PX4 参数

```bash
px4-param set MPC_XY_VEL_MAX 1.5   # 限制 PX4 内部最大水平速度
```

**方案 B（改代码）**：让 `_target_speed` 真正生效——在 px4_bridge 位置模式下把 setpoint 移动速率 clamp 到 `_target_speed`，或切到速度控制模式。

---

## 二、代码-YAML 默认值不一致（中等）

### 2.1 `grid_spacing`：三处代码写 1.0，YAML 是 1.67

| 位置 | 值 | 说明 |
|------|-----|------|
| `mission_params.yaml:56` | **1.67** | 运行时实际生效 |
| `path_planner.py:41` 构造函数默认 | 1.67 | 与 YAML 一致 |
| `path_planner.py:7` 文档注释 | **1.0** | 过时注释 |
| `mission_state_machine.py:242` `declare_parameter` | **1.0** | 被 YAML 覆盖 |
| `mission_state_machine.py:389` `get_param` fallback | **1.0** | 被 YAML 覆盖 |

运行时链路：`YAML(1.67) → get_param("grid_spacing", 1.0) → PathPlanner(grid_spacing=1.67)`，值正确但代码里的 1.0 会误导读者。

**建议**：统一四处为 1.67。

### 2.2 `max_acceleration`：FSM 声明 0.2 但从未读取

| 位置 | 值 | 说明 |
|------|-----|------|
| `mission_params.yaml:33` | **2.0** | 运行时生效 |
| `px4_params.yaml:23` | **2.0** | px4_bridge 实际使用 |
| `px4_bridge.py:220` | 2.0 | 与 YAML 一致 |
| `mission_state_machine.py:227` | **0.2** | FSM 声明但从未 `get_param` — **死声明** |

`max_acceleration` 唯一的消费者是 `px4_bridge.py:445`：

```python
max_step = self.get_parameter("max_acceleration").value * 0.05  # per 20Hz tick
```

FSM 的声明是冗余的，且默认值 0.2 与 YAML 2.0 不一致。

**建议**：删除 FSM 中的死声明，或至少将默认值改为 2.0。

### 2.3 `sweep_speed`：一致但无效

| 位置 | 值 |
|------|-----|
| `mission_params.yaml:30` | 0.8 |
| `mission_state_machine.py:224` | 0.8 |
| `path_planner.py:38` | 0.8 |

值一致，但参数本身对飞行无影响（见第一条）。

---

## 三、`sweep_speed` 缺少参数校验（轻微）

`_validate_critical_params()`（`mission_state_machine.py:1062-1082`）校验了以下参数但**漏了 `sweep_speed`**：

```python
checks = [
    ...
    ("grid_spacing", 0.2, 10.0),      # ✅ 有
    ("transit_speed", 0.1, 15.0),     # ✅ 有
    ("return_speed", 0.1, 15.0),      # ✅ 有
    # ("sweep_speed", ?, ?)           # ❌ 缺失
]
```

即使该参数目前是死的（第一条），校验也应补上以保持一致性。

**建议**：追加 `("sweep_speed", 0.1, 5.0)`。

---

## 四、`px4_params.yaml` 含冗余死参数（轻微）

`px4_bridge` 声明并加载了 `sweep_speed`、`transit_speed`、`return_speed`，但**从未读取任何一个**。这三个参数的实际消费者是 `mission_state_machine` / `PathPlanner`。

YAML 值还存在不一致：

| 参数 | px4_params.yaml | mission_params.yaml | 实际消费者 |
|------|----------------|---------------------|-----------|
| `sweep_speed` | 0.8 | 0.8 | mission_state_machine |
| `transit_speed` | **3.0** | 2.0 | mission_state_machine |
| `return_speed` | **4.0** | 3.0 | mission_state_machine |
| `max_acceleration` | 2.0 | 2.0 | **px4_bridge**（唯一实际使用者） |

虽然无运行时影响（px4_bridge 不读它们，各自命名空间隔离），但造成代码阅读困惑。

**建议**：从 `px4_bridge.py` 和 `px4_params.yaml` 中删除 `sweep_speed`、`transit_speed`、`return_speed`，只保留 `max_acceleration`（唯一被消费的参数）。

---

## 五、覆盖风险检查（无风险）

### 5.1 主任务（sitl.launch.py / real_flight.launch.py）

```
参数来源优先级: YAML > 代码 declare_parameter 默认值
inline dict 覆盖: 仅 home_x/y/z, flight_axis, drop/recon_distance, rc_require_arm_signal
```

`sweep_speed`、`grid_spacing`、`max_acceleration`、`transit_speed`、`return_speed` **均不在** inline dict 覆盖列表内，也不作为 launch arg 暴露。改 YAML 即生效，不会被覆写。

### 5.2 启动脚本

| 脚本 | 是否覆写这 5 个参数 |
|------|-------------------|
| `start_full_simulation.sh` | ❌ 不涉及，`"$@"` 可传但 launch 未声明对应 arg，会报错 |
| `simple_mission.sh` | ❌ 不涉及 |
| `simple_mission_real.sh` | ❌ 不涉及 |
| `test_forward_recon.sh` | ⚠️ 硬编码 `sweep_speed:=0.8`、`grid_spacing:=1.67`，但走独立测试节点，不影响主任务 |

### 5.3 test_forward_recon 独立参数体系

`test_forward_recon` 是独立 ROS 节点，有自己的一套参数声明且不加载 `mission_params.yaml`。改主任务 YAML 不影响它，改它的参数需同步改 shell 脚本或 launch 文件。详见 [[test_mission_param_conflicts.md]]。

---

## 优先级汇总

| # | 问题 | 严重度 | 建议 |
|---|------|--------|------|
| 1 | `sweep_speed` 不控制实际飞行速度 | **严重** | 方案 A: 调 PX4 `MPC_XY_VEL_MAX`；方案 B: 让代码走速度控制 |
| 2 | `grid_spacing` 代码默认 1.0 vs YAML 1.67（三处不一致） | 中等 | 统一为 1.67 |
| 3 | `max_acceleration` FSM 声明 0.2 vs YAML 2.0，且 FSM 从未读取 | 中等 | 删除 FSM 死声明，或默认改为 2.0 |
| 4 | `sweep_speed` 缺校验 | 轻微 | 补 `("sweep_speed", 0.1, 5.0)` |
| 5 | px4_params.yaml / px4_bridge 含三个冗余速度参数 | 轻微 | 删除 `sweep_speed`、`transit_speed`、`return_speed` |
