# test_mission 参数冲突分析

> 分析日期: 2026-08-01  
> 涉及文件: `test_forward_recon.py`, `test_forward_recon.launch.py`, `test_forward_recon.sh`, `path_planner.py`, `mission_params.yaml`, `real_flight.launch.py`, `simple_mission.launch.py`

---

## 🔴 冲突 1: `recon_area_half_north` 默认值三处不一致

| 位置 | 默认值 |
|---|---|
| `test_forward_recon.py:114` (`declare_parameter`) | **2.5** |
| `test_forward_recon.launch.py:67` (`DeclareLaunchArgument`) | **4.0** |
| `test_forward_recon.sh:163` (shell 脚本传入) | **4.0** |

Python 注释写 "对齐主任务 Area2 5m"，主任务 Area2 的 `north[55,60]` 半宽正是 2.5m。但 launch 文件和 shell 脚本传的是 4.0，**实际运行时以 launch 文件的 4.0 覆盖了 Python 的 2.5**，导致侦察区 north 半宽从 2.5m → 4.0m（总面积从 5m×8m 变成 8m×6m），与注释声明的"对齐主任务"不一致。

---

## 🔴 冲突 2: `recon_area_half_east` 默认值三处不一致

| 位置 | 默认值 |
|---|---|
| `test_forward_recon.py:115` (`declare_parameter`) | **4.0** |
| `test_forward_recon.launch.py:69` (`DeclareLaunchArgument`) | **3.0** |
| `test_forward_recon.sh:164` (shell 脚本传入) | **3.0** |

Python 注释写 "对齐主任务 Area2 8m"（half_east=4.0 即总宽 8m），与主任务 `east[-4,4]` 一致。但 launch/shell 传 3.0（总宽 6m）。

---

## 🔴 冲突 3: `grid_spacing` 默认值不一致

| 位置 | 默认值 |
|---|---|
| `test_forward_recon.py:117` (`declare_parameter`) | **1.67** |
| `test_forward_recon.launch.py:79` | **1.0** |
| `test_forward_recon.sh:166` | **1.0** |
| `PathPlanner.__init__` (`path_planner.py:41`) | **1.67** |
| `mission_params.yaml:56` (主任务) | **1.67** |

Python 注释写 "对齐 PathPlanner 默认"（1.67），但 launch 文件覆盖为 1.0，导致 test_mission 生成更密的条带（比主任务多 ~67% 的航点）。

---

## 🟡 冲突 4: `takeoff_tolerance` 在 DESCEND 阶段 fallback 值不一致

`test_forward_recon.py`:
- **声明** (line 98): `takeoff_tolerance` 默认 `0.3`
- **TAKEOFF 使用** (line 323): `self.get_param("takeoff_tolerance", 0.3)` ✅
- **DESCEND_TO_RECON 使用** (line 411): `self.get_param("takeoff_tolerance", 0.5)` ❌

两个 `get_param` 的 fallback 值不同（0.3 vs 0.5）。虽然因为参数已声明，实际返回都是 0.3，但 fallback 不一致说明意图不明确。

---

## 🟡 冲突 5: `vehicle_status_topic` 无法从 launch 文件覆盖

| 文件 | 是否声明/传递 `vehicle_status_topic` |
|---|---|
| `simple_mission.launch.py` | ✅ 声明参数 + 传递给 px4_bridge + simple_mission |
| `sitl.launch.py` | ✅ 声明参数 + 传递 |
| `test_forward_recon.launch.py` | ❌ 未声明，未传递 |
| `real_flight.launch.py` | ❌ 未声明，未传递 |

`test_forward_recon.launch.py` 不声明也不传递 `vehicle_status_topic`——px4_bridge 和 test_forward_recon 节点各自依赖内部默认值（都是 v1，碰巧一致）。但如果 PX4 固件版本不同需要 v4，用户**无法从命令行或 launch 参数覆盖**，只能改代码。

同样 `real_flight.launch.py` 也未传递，依赖 yaml 配置文件（v1）。

对比 `simple_mission.launch.py` 设计是完整的——声明了参数并传递给两个节点。

---

## 🟡 冲突 6: `sweep_margin_north` / `sweep_margin_east` 无法从 launch 覆盖

Python 声明了这两个参数（lines 118-119）并传给了 `PathPlanner`，但 launch 文件**没有对应的 `DeclareLaunchArgument`**，shell 脚本也无法传入。只能改代码。

---

## 🟢 无冲突的地方（供参考）

| 参数 | Python | Launch | Shell | 状态 |
|---|---|---|---|---|
| `camera_cx` / `camera_cy` | 320 / 240 | 320 / 240 | — | ✅ |
| `forward_distance` | 20.0 | 20.0 | 20.0 | ✅ |
| `cruise_altitude` | 5.0 | 5.0 | 5.0 | ✅ |
| `takeoff_height` | 5.0 | =cruise_altitude | — | ✅ |
| `sim_mode` | true | true | false(真机) | ✅ |
| `rc_require_arm_signal` | true | true | true | ✅ |
| `vehicle_status_topic` 默认 | v1 | v1 (来自 yaml) | — | ✅ (默认一致) |

---

## 建议修复优先级

1. **冲突 1+2**: 统一 `recon_area_half_north/half_east`——决定是要对齐主任务 Area2（2.5/4.0）还是独立定义（当前 launch 的 4.0/3.0），然后三处统一
2. **冲突 3**: 统一 `grid_spacing` 为 1.67（对齐 PathPlanner 和主任务）或 1.0（如果 test_mission 有意做更密扫描）
3. **冲突 5**: 给 `test_forward_recon.launch.py` 加上 `vehicle_status_topic` 参数声明并传递给两个节点，保持与 `simple_mission.launch.py` 一致的架构
4. **冲突 4**: `DESCEND_TO_RECON` 的 `get_param("takeoff_tolerance", 0.5)` fallback 改为 `0.3`
5. **冲突 6**: 可选——暴露 `sweep_margin_*` 到 launch 参数
