# 方案规划：侦察 yaw 参数化 + 任务按任意制定方向运行

> 状态：**已被取代，未实施**（2026-08-12）
>
> 替代方案：[`2026-08-12-arbitrary-layout-heading-design.md`](../superpowers/specs/2026-08-12-arbitrary-layout-heading-design.md)
>
> 本文仅保留为历史设计记录，不再作为实施依据。
> 基线：HEAD `4b5b5db`

---

## 1. 背景与根因

### 1.1 现象

真机侦察扫描时机头在弓字形条带间 ±90° 旋转；路径正确，旋转无功能价值（下视相机）。

### 1.2 根因

1. `62a5cdf`：`px4_bridge.py _cmd_setpoint_cb` 从硬编码 `0.0` 改为透传 `float(msg.yaw)`
2. `path_planner.py plan_coverage_ned`：条带沿 east 飞行时生成 `yaw=±π/2`

### 1.3 临时修复（HEAD `4b5b5db`）

`plan_coverage_ned` east 分支 yaw 置 `0.0` + 测试 + 参数指南。本方案吸收：`yaw_mode="hold"` = 其参数化表达。

### 1.4 需求

1. 消除多余旋转（`hold` 默认）
2. **任务区域可沿任意指定方向布置**——当前场地正北（0°），未来任意方位
3. **所有任务环节在任意方向下完整无损**——转场目标、扫描足迹、覆盖率、投放判定全部自洽

---

## 2. 核心设计

### 2.1 数学语义

**区域定义在制定方向局部坐标系中，从 NED home `(0,0)` 出发**：

- u' 轴 = 制定方向 `flight_heading_deg`（0=正北, 顺时针为正）；v' 轴 = u' + 90°
- `area*_north_min/max` = 沿 u' 轴距 home 的距离（m）；`area*_east_min/max` = 沿 v' 轴距 home 的距离
- 区域中心：局部 = `((n_min+n_max)/2, (e_min+e_max)/2)`；NED = `(57.5·cos h, 57.5·sin h)`
- 半宽与 heading 无关：`u_half = (n_max-n_min)/2 = 2.5m`, `v_half = (e_max-e_min)/2 = 4.0m`

变换（原点始终 home）：

```
local_to_ned(u, v, h):   north = u·cos h - v·sin h
                          east  = u·sin h + v·cos h
ned_to_local(n, e, h):   u = n·cos h + e·sin h
                          v = -n·sin h + e·cos h
```

### 2.2 数学恒等式（16 项验证通过）

| # | 恒等式 | 含义 |
|---|---|---|
| 1 | `local∘ned = 恒等`（∀h） | 可逆变换 |
| 2 | 中心 NED = `(57.5·cos h, 57.5·sin h)` | = `compute_zone_centers` recon 分量 |
| 3 | 对角 = `√89`（∀h） | 范围不变（heading 无关） |
| 4 | h=0 逐位一致 | 兼容保证 |
| 9 | TRANSIT 目标 = 区域中心 | **完整无损核心** |

h=0 时 `u'=north, v'=east`，与所有现有参数/输出逐位一致。

### 2.3 设计决策

| 项 | 选择 |
|---|---|
| 方向参数 | `flight_heading_deg`（度，0=北，顺时针）；`flight_axis` 保留兼容映射 |
| yaw_mode | `hold` 默认（yaw 恒 0）；`follow_path` 可选 |
| 区域参数语义 | 沿制定方向距 home 的距离（h=0 等价原 NED 值） |
| 兼容 | 所有新参数默认值 = 当前行为；h=0 输出**严格相等** |

---

## 3. 参数一览

| 参数名 | 类型 | 默认值 | 语义 |
|---|---|---|---|
| `flight_heading_deg` | float, 度 | `0.0` | 任务制定方向：影响区域中心、足迹、覆盖率 |
| `yaw_mode` | string | `"hold"` | 侦察 yaw：`"hold"`=恒 0，`"follow_path"`=跟随条带 |

配置：`mission_params.yaml` + `vision_params.yaml`（coverage 也读此参数）+ 3 个 launch 透传。

**弃用**：`area1_heading_deg`/`area2_heading_deg`（额外旋转与"区域沿制定方向"语义冲突；区域内调整宽高比即可改变条带方向）。

---

## 4. 旋转工具层（新增 `src/drone_utils/drone_utils/coordinates.py`）

`drone_mission` 与 `drone_vision` 均依赖 `drone_utils`，共享层：

```python
def heading_local_to_ned(u, v, heading_rad):
    """局部绝对坐标 → NED，原点 home=(0,0)"""
    return u*math.cos(heading_rad) - v*math.sin(heading_rad), \
           u*math.sin(heading_rad) + v*math.cos(heading_rad)

def ned_to_heading_local(n, e, heading_rad):
    """NED → 局部 (用于旋转矩形内判定)"""
    return n*math.cos(heading_rad) + e*math.sin(heading_rad), \
           -n*math.sin(heading_rad) + e*math.cos(heading_rad)

def normalize_yaw(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))  # → (-π, π]

def compute_zone_centers(flight_heading_deg, drop_dist, recon_dist):
    """纯函数: heading(度) → ((drop_NED), (recon_NED)).
       内处理 flight_axis 兼容映射 (仅当 heading_deg 未显式配置时使用)."""
    h = math.radians(flight_heading_deg)
    return (drop_dist*math.cos(h), drop_dist*math.sin(h)), \
           (recon_dist*math.cos(h), recon_dist*math.sin(h))
```

配套单测 (`src/drone_utils/test/test_coordinates.py`)：互逆、90° 几何、normalize_yaw 边界、compute_zone_centers 各方向 + flight_axis 映射。

---

## 5. 阶段 A：核心实现

### A1. `path_planner.py` — `plan_coverage_ned`

```python
def plan_coverage_ned(self, area_north_min, area_north_max, area_east_min, area_east_max,
                       altitude=5.0, heading=0.0, yaw_mode="hold"):
    """生成弓字形全覆盖航点 (NED 坐标系).

    area_north_min/max: 沿 heading 方向距 NED home 的距离 (m)
    area_east_min/max:  垂直 heading 方向距 NED home 的距离 (m)
    heading: 制定方向 (rad, 0=NED north, 顺时针为正)
    yaw_mode: "hold"=yaw 恒 0.0 (默认) | "follow_path"=yaw 跟随条带方向
    """
```

实现要点（替换约 :184-261）：
- `yaw_mode` 非法 `raise ValueError`
- 计算 `cos_h`, `sin_h`；半宽 `u_half = (n_max-n_min)/2, v_half = (e_max-e_min)/2`
- 局部绝对坐标下生成弓字形：`u' ∈ [n_min, n_max], v' ∈ [e_min, e_max]`；长边优先选 `fly_along`（"u"/"v" 语义）
- 每航点 `heading_local_to_ned` 变换；yaw：hold → 0.0；follow_path → `h`/`h+π` 或 `h+π/2`/`h-π/2`（条带 enter/exit 同向）
- **无 `center` 参数**：局部坐标原点 = home，中心隐式自洽
- 条带数：`num_strips = ceil(条带方向跨度 / grid_spacing)`

### A2. `state_recon_patrol.py`

```python
flight_heading_deg = float(self._fsm.get_param("flight_heading_deg", 0.0))
yaw_mode = self._fsm.get_param("yaw_mode", "hold")
heading_rad = math.radians(flight_heading_deg)
# area2 参数语义已变为"沿制定方向距 home"，直接原值传入
self._waypoints = self._fsm.path_planner.plan_coverage_ned(
    area_north_min=area2_n_min, area_north_max=area2_n_max,
    area_east_min=area2_e_min, area_east_max=area2_e_max,
    altitude=..., heading=heading_rad, yaw_mode=yaw_mode)
```

日志追加 heading/yaw_mode。`_fly_to_wp` 已透传 yaw，不改。

### A3. `mission_state_machine.py`

- `_declare_params`（:210 后 / :239 后）：`flight_heading_deg`、`yaw_mode`
- `_validate_critical_params`：范围校验 + yaw_mode 值校验
- `_lazy_init`（:356-374）：
  ```python
  (drop_n, drop_e), (recon_n, recon_e) = compute_zone_centers(
      float(self.get_param("flight_heading_deg", 0.0)),
      float(self.get_param("drop_distance", 32.5)),
      float(self.get_param("recon_distance", 57.5)))
  # 缓存: self._recon_center_n/e, self._drop_center_n/e
  ```
  修复 :364 注释不一致。`flight_axis` 仅在 heading_deg 未显式配置时生效。
- 辅助方法：`recon_zone_ned()`/`drop_zone_ned()` = 转场目标 = 区域中心

### A4. 配置与 launch

- `mission_params.yaml`：`flight_heading_deg: 0.0`（:16 后 注"度, 0=正北, 顺时针"）+ `yaw_mode: "hold"`（Area 2 块）+ 更新 area 注释（"沿制定方向距 home"）
- `sitl.launch.py` / `real_flight.launch.py`：声明 + mission_node 参数覆写；`flight_heading_deg` 同时透传 vision_pipeline（coverage 联动）
- `test_forward_recon.py`：`__init__` 声明 `flight_heading_deg`/`yaw_mode`；`TRANSIT_FORWARD` 目标 `(fwd·cos h, fwd·sin h)`；`_generate_recon_waypoints` 透传 heading/yaw_mode
- `test_forward_recon.launch.py`：同模式

### A5. CoverageMonitor 联动

**问题**：CoverageMonitor 网格是 World 系轴对齐矩形（`vision_pipeline.py:222-228`），heading≠0 时足迹斜置 → 覆盖率失真。

**方案**：
1. `coverage_monitor.py`：`CoverageMonitor.__init__` 加 `heading_rad=0.0`；`update()` 开头 drone World → NED → heading 局部 → 盒内判定 → 网格累积（网格尺寸按 u'/v' 范围，heading 无关）
2. `vision_pipeline.py`：declare `flight_heading_deg`（默认 0.0），构造 CoverageMonitor 时传入 `math.radians(...)`
3. launch：同一 `flight_heading_deg` 透传两节点，单一来源

---

## 6. 阶段 B：投放区沿制定方向

### B1. `is_in_drop_zone_world` / `_normalize_drop_target_xy`

World 输入 → `world_to_ned` → `ned_to_heading_local(h)` → u'/v' 范围精确判定。

**禁用 AABB 外扩**：45° 时区外点被放行 ~1.2m。margin（0.5/0.75）原样保留到精确判定。

### B2. `state_cylinder_search._start_scan`（:127-141）

扫描线 = 中心局部 ± v_half 沿 v' 轴 → `heading_local_to_ned`。中心点 :45-48 不改（局部化后中心随 heading 一致）。

### B3. 其余投放环节

`state_drop.py`、`drop_visual_aligner.py`、`precision_align_controller.py`、`cylinder_selector.py`、`state_cylinder_approach.py`（`is_in_drop_zone_world` 内部语义升级自动生效）— 均不改。

---

## 7. 完整任务链审计

| 任务环节 | 方向依赖 | 覆盖 |
|---|---|---|
| TAKEOFF | 无 | — |
| TRANSIT_TO_DROP/RECON | `compute_zone_centers` | §5 A3 |
| CYL_SEARCH 中心 | 局部化后自动一致 | — |
| CYL_SEARCH 扫描线 | `_start_scan` heading 局部 | §6 B2 |
| 目标过滤 | `_normalize_drop_target_xy` 精确判定 | §6 B1 |
| 投放安全判定（×3） | `is_in_drop_zone_world` 精确判定 | §6 B1 |
| DESCEND/DROP/ASCEND | 无（视觉对心） | — |
| 投放精校/目标选择 | 无 | — |
| RECON_PATROL | `plan_coverage_ned` (heading + yaw_mode) | §5 A1-A2 |
| 覆盖率 | CoverageMonitor 局部网格 | §5 A5 |
| RETURN | `goto_ned(..., yaw=0)`, home 固定 | — |
| PRECISION_LAND | 视觉对心 | — |

---

## 8. 测试计划

### 8.1 新增测试

**`src/drone_utils/test/test_coordinates.py`**：互逆（6 角度×4 点）、normalize_yaw 边界、compute_zone_centers 各方向

**`src/drone_mission/test/test_path_planner_coverage_ned.py`**（area2 默认、grid=1.67）：

| # | 用例 | 期望 |
|---|---|---|
| 1 | h=0 + hold 兼容锁定 | 航点 `[(55,-4),(55,4),(57.5,4),(57.5,-4),(60,-4),(60,4)]`，yaw 全 0.0（**严格相等**） |
| 2 | h=π/2 + hold 区域沿东 | 航点 `[(4,55),(-4,55),(-4,57.5),(4,57.5),(4,60),(-4,60)]`，中心 `(0,57.5)`（places=6） |
| 3 | h=π/2 + hold 中心=转场目标 | 航点中心 NED = `(0, 57.5)` = recon_distance 沿 heading 之 recon 分量 |
| 4 | follow_path yaw | h=0: `[π/2, -π/2, π/2]`；h=π/2: `[π, 0, π]`（3 strips, v-fly） |
| 5 | hold 全部 yaw==0.0（h∈{0,π/2,π/4}）|
| 6 | `yaw_mode="bogus"` → ValueError |
| 7 | 逆变换：∀h∈{0,π/6,π/3,-π/4,120°}，航点在局部盒 `[55,60]×[-4,4]` 内（tol 1e-9） |
| 8 | 零范围 → `[]` |

**`src/drone_vision/test/test_coverage_monitor_rotate.py`**：旋转 zone 覆盖率达标；h=0 与原行为一致

### 8.2 现有测试保护

- `test_mission_setpoint_contract.py:204-219`（fixed_heading）原样通过
- `test_recon_coverage_lifecycle.py` mock planner，不受影响
- `test_coverage_monitor_scan_lifecycle.py`：CoverageMonitor 新增 keyword 参数（默认值），零破坏
- flake8/pep257 CI 门禁

### 8.3 运行方式

```bash
source install/setup.bash   # egg-link，纯 Python 改动无需 colcon build
python3 -m pytest src/drone_utils/test src/drone_mission/test src/drone_vision/test src/px4_control/test -q
```

---

## 9. 实施顺序

| 步 | 内容 | 验证 |
|---|---|---|
| 1 | `drone_utils/coordinates.py` + 单测 | test_coordinates 全绿 |
| 2 | path_planner `plan_coverage_ned` 改造 + docstring | test_path_planner_coverage_ned 全绿 |
| 3 | CoverageMonitor heading + 测试 | test_coverage_monitor_rotate 全绿 |
| 4 | 全量回归（4 包） | 全绿（含 fixed_heading、flake8、scan_lifecycle） |
| 5 | mission_state_machine（declare/validate/_lazy_init）+ state_recon_patrol 透传 + mission_params.yaml | SITL 日志含 heading/yaw_mode |
| 6 | 3 个 launch + test_forward_recon.py（TRANSIT_FORWARD 沿 heading） | 参数透传确认 |
| 7 | 阶段 B：投放区判定/扫描线局部化 | SITL `flight_heading_deg:=45` 全链运行 |
| 8 | 文档更新（4 个文档表述同步） | 无旧表述残留 |
| 9 | SITL 端到端：hold 默认全 0；follow_path 序列；`flight_heading_deg:=45` 斜向全链覆盖率 1.0 |
| 10 | 真机：标定 heading → hold → follow_path → heading=30 足迹吻合 |

---

## 10. 风险

1. **follow_path yaw 低通**（px4_bridge.py:471-478）：`dyaw*0.3`，~0.5s 收敛，航点间早已稳定。hold 完全无 yaw 运动
2. **h=0 严格相等**：测试 1 锁定（cos0=1/sin0=0 精确）
3. **coverage 联动**：vision_pipeline 直读 `flight_heading_deg`，单一来源
4. **投放安全**：精确判定（`ned_to_heading_local`），禁用 AABB
5. **参数语义变化**：`area*_north/east_*` 从"NED 绝对坐标"→"沿制定方向距离"（h=0 等价，YAML 注释同步更新）
6. **范围纪律**：条带端点裁剪、平滑提速、GPS 经纬度列为后续项

---

## 11. 关键文件

| 文件 | 改动 |
|---|---|
| `src/drone_utils/drone_utils/coordinates.py` | **新增** |
| `src/drone_mission/drone_mission/path_planner.py` | 核心：`heading_local_to_ned` + heading + yaw_mode（无 center） |
| `src/drone_mission/drone_mission/states/state_recon_patrol.py` | 参数透传 + 日志 |
| `src/drone_mission/drone_mission/mission_state_machine.py` | declare/validate/_lazy_init/投放判定局部化/辅助方法 |
| `src/drone_mission/drone_mission/states/state_cylinder_search.py` | :127-141 扫描线局部化 |
| `src/drone_mission/drone_mission/states/state_cylinder_approach.py` | 不改（判定自动升级） |
| `src/drone_vision/drone_vision/coverage_monitor.py` | heading_rad + 局部累积 |
| `src/drone_vision/drone_vision/vision_pipeline.py` | declare flight_heading_deg + 透传 |
| `src/drone_mission/config/mission_params.yaml` | 新参数 + 注释更新 |
| `src/drone_vision/config/vision_params.yaml` | flight_heading_deg 默认 |
| `src/drone_bringup/launch/sitl.launch.py`、`real_flight.launch.py` | 声明 + 透传 |
| `src/test_mission/test_mission/test_forward_recon.py` + launch | 声明 + 透传 + TRANSIT 沿 heading |
| 测试 ×3 | test_coordinates / test_path_planner_coverage_ned / test_coverage_monitor_rotate |
| 文档 ×4 | PARAMETER_USAGE_GUIDE / recon_speed_audit / ARCHITECTURE / DEPLOYMENT |
