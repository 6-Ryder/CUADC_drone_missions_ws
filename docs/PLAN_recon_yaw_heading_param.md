# 方案规划：侦察 yaw 参数化 + 任务按任意制定方向运行

> 状态：**规划文档，未实施**（2026-08-02）
> 基线：HEAD `4b5b5db`（已包含"侦察区 yaw 固定 0.0"临时修复）
> 关联：`PARAMETER_USAGE_GUIDE.md`、`recon_speed_parameter_audit.md`、`ARCHITECTURE.md`

---

## 1. 背景与根因

### 1.1 现象

真机实飞时，侦察区航点/机头朝向在弓字形扫描条带间发生 ±90° 旋转；飞行路径本身正确。经排查，该行为无实际功能价值（侦察为下视相机覆盖扫描，机头朝向无关），属多余行为。

### 1.2 根因（两个改动叠加）

1. **`62a5cdf`（08-02 08:47）**：`px4_bridge.py _cmd_setpoint_cb` 从硬编码 `self._target_yaw = 0.0` 改为透传 `float(msg.yaw)`（配套新增 `NedPositionSetpoint.msg` 的 `yaw` 字段）。此前 yaw 被 bridge 层丢弃，PX4 永远收到 0。
2. **`path_planner.py plan_coverage_ned`**：条带沿 east 方向飞行时生成 `yaw = ±π/2`（fly_along 由 north/east 范围比较自动选择；侦察区 55-60 × -4~4 中 east 范围 8m > north 范围 5m → 沿 east 飞，yaw=±90°）。

叠加效果：改动前 yaw 被丢弃（恒 0），改动后 yaw 真实透传 → 机头随条带旋转。

### 1.3 已应用的临时修复（HEAD `4b5b5db`）

- `plan_coverage_ned` east 分支 yaw 置 `0.0`（注释"固定朝 NED North"）
- 新增测试 `test_recon_coverage_waypoints_keep_a_fixed_heading`（断言所有侦察航点 yaw==0.0）
- 新增 `docs/PARAMETER_USAGE_GUIDE.md`

**本方案"吸收"该临时修复**：`yaw_mode="hold"` 完全复现其行为，不回退。

### 1.4 需求

1. **消除多余旋转**：侦察扫描机头朝向可配置（默认 `hold` = 固定 NED North，yaw 恒 0）
2. **任意制定方向**：任务（起飞 → 转场 → 投放 → 侦察 → 返航 → 降落）可沿任意指定方向完整无损运行——当前场地正北（0°），未来场地/任务任意方位
3. 机头朝向无所谓（下视相机）→ `hold` 为默认；未来前视相机场景 → `follow_path` 可选

---

## 2. 设计总览（最终方案）

### 2.1 核心设计：单一制定方向参数 `flight_heading_deg`

**所有区域定义统一到"制定方向局部坐标系"，全链路共享同一方向来源**：

- 局部坐标：u' 轴 = 制定方向（NED 方位角 `flight_heading_deg`，顺时针为正），v' 轴 = u'+90°；原点 = 任务原点（home）
- `area*_north_min/max` 语义 = **沿制定方向的 u' 距离范围**；`area*_east_min/max` = 垂直 v' 范围
- 区域中心（局部）`= ((n_min+n_max)/2, (e_min+e_max)/2)` → 与 `drop_distance`/`recon_distance` 计算的转场目标**同一来源、天然一致**
- 弓字形条带方向 = 区域长边自动选择（现算法，不变）
- 制定方向 = 0° 时：u'=north、v'=east，与现状**完全兼容**

**为何不做"区域足迹额外旋转"参数**（审查中推演后弃用）：
- "足迹沿制定方向"与"区域参数沿制定方向定义"是自洽的；若再加一个额外旋转角，YAML 范围值的方向归属产生歧义（沿制定方向还是总方向），需要旋转盒 AABB 映射（45° 时外溢 ~15%，且临界角翻转导致条带方向突变）——复杂且无实际场景支撑
- 区域内需要不同朝向时，直接调整 `area*_min/max` 的宽高比即可（弓字形自动沿长边）

### 2.2 设计决策（已与用户确认 + 审查收敛）

| 决策项 | 选择 |
|---|---|
| 方向参数 | **`flight_heading_deg` 唯一制定方向**（0=正北, 90=正东, NED 顺时针）；`flight_axis` 保留兼容映射（"north"→0°, "east"→90°） |
| heading 单位 | **度 + `_deg` 后缀**（YAML/launch 写度数；调用方 `math.radians` 转弧度；规划器 API 用弧度） |
| yaw_mode 默认 | **hold**（yaw 恒 0.0），可选 `follow_path`（yaw 跟随条带方向） |
| 区域参数语义 | `area*_north_min/max` = 沿制定方向范围，`area*_east_min/max` = 垂直范围（制定方向=0° 时 = 现状 north/east） |
| 兼容原则 | 所有新参数默认值 = 当前行为 → **零 YAML 改动即保持现状**；h=0 输出与现算法**逐位一致**（严格相等测试锁定） |

### 2.3 坐标系与旋转数学（统一工具放 `drone_utils`）

- NED：x=north, y=east, z=down；yaw `0=北, π/2=东`（顺时针为正）
- World(ENU)：x=east, y=north；`telemetry.py world_to_ned` 为平移+轴交换

制定方向局部坐标（原点 = home / zone 中心，见各使用点）：
```
NED → 局部(u'):  u = dn·cos h + de·sin h          (dn = n - origin_n, de = e - origin_e)
                 v = -dn·sin h + de·cos h
局部 → NED:      north = origin_n + u·cos h - v·sin h
                 east  = origin_e + u·sin h + v·cos h
```
关键性质：h=0 时 `cos0=1, sin0=0` 精确 → 输出逐位一致；旋转不改变面积/间距；区域中心不变。

---

## 3. 公共层：旋转工具（新增 `src/drone_utils/drone_utils/coordinates.py`）

`drone_mission` 与 `drone_vision` 均依赖 `drone_utils`（package.xml 已确认），共享层避免跨包依赖：

```python
def local_to_ned(u, v, c_n, c_e, heading_rad):          # 局部 → NED (绕中心)
def ned_to_local(n, e, c_n, c_e, heading_rad):          # NED → 局部 (绕中心; 点在旋转盒内判定)
def world_to_zone_local(wx, wy, zc_x, zc_y, heading_rad):  # World → zone 局部 (CoverageMonitor)
def normalize_yaw(yaw):                                 # 归一化到 (-pi, pi]
def compute_zone_centers(flight_heading_deg, drop_dist, recon_dist):
    # → (drop_north, drop_east, recon_north, recon_east) 纯函数, 便于单测
```

配套单测：互逆性（`local_to_ned`∘`ned_to_local` = 恒等）、90° 旋转几何、`normalize_yaw` 边界（π/−π 等价）、`compute_zone_centers` 0°/90°/任意角/`flight_axis` 兼容映射。

---

## 4. 参数一览（最终）

| 参数名 | 类型/单位 | 默认值 | 语义 |
|---|---|---|---|
| `flight_heading_deg` | float, 度 | `0.0` | **任务制定方向**：区域中心方向 + 区域足迹朝向（0=正北, 90=正东, NED 顺时针） |
| `yaw_mode` | string: `"hold"`/`"follow_path"` | `"hold"` | 侦察航点 yaw 策略 |
| `coverage_zone_heading_deg` | float, 度 | `0.0` | 覆盖监测网格方向（**联动** flight_heading_deg，见 §5 A5） |
| ~~`area1_heading_deg`/`area2_heading_deg`~~ | — | — | **弃用**（见 §2.1 设计权衡：足迹沿制定方向，无需额外旋转） |

配置位置：`mission_params.yaml`（mission 侧）+ `vision_params.yaml`（coverage 侧）+ 3 个 launch（声明与透传）。`flight_axis` 仅作兼容映射，不再作为主方向配置。

---

## 5. 阶段 A：核心（yaw_mode + 制定方向 + coverage 联动）

### A1. `src/drone_mission/drone_mission/path_planner.py`

1. 导入 `drone_utils.coordinates` 的 `normalize_yaw`、`local_to_ned`、`ned_to_local`
2. **`plan_coverage_ned` 新签名**（追加 keyword 参数，现有 3 处调用零破坏）：
   ```python
   def plan_coverage_ned(self, area_north_min, area_north_max, area_east_min, area_east_max,
                         altitude=5.0, heading=0.0, yaw_mode="hold",
                         center_n=None, center_e=None):
       # heading:    区域方位角 (rad, 0=正北, 顺时针); 区域绕旋转中心旋转
       # yaw_mode:   "hold"=yaw 恒 0.0 (默认) | "follow_path"=yaw 跟随条带方向
       # center_n/e: 旋转中心 (NED); None = 区域边界盒中心 (现状语义)
       # area_north_min/max: 沿 heading 方向、相对旋转中心的局部范围
       #                     (heading=0, center=None 时 = 现状 NED 范围)
   ```
3. **函数体改造**（替换约 :184-261）：
   - `yaw_mode` 非法值 `raise ValueError`（fail fast，可单测）
   - 旋转中心 `center_n/center_e`（None → 边界盒中心）；`cos_h/sin_h`；中心在局部坐标的投影 `u_center = center_n·cos h + center_e·sin h`、`v_center = -center_n·sin h + center_e·cos h`
   - 局部范围相对中心：`u ∈ [area_north_min - u_center, area_north_max - u_center]`、`v ∈ [area_east_min - v_center, area_east_max - v_center]`（尺寸不变）
   - 局部坐标系生成弓字形（`fly_along` 换为 "u"/"v" 语义，长边优先算法不变）
   - 每航点 `local_to_ned` 逆变换回 NED（绕 center）；yaw 取值：
     - `hold`：恒 `0.0`
     - `follow_path`：`fly_along=="u"` → `heading`(偶条带)/`normalize_yaw(heading+π)`(奇)；`fly_along=="v"` → `normalize_yaw(heading+π/2)`(偶)/`normalize_yaw(heading-π/2)`(奇)
     - 条带 enter/exit 同 yaw（与 62a5cdf 之前原始行为一致）
   - **中心一致性（"完整无损"关键）**：调用方传入 `center = 转场目标中心`（recon_zone）时，旋转后足迹中心与 TRANSIT_TO_RECON_ZONE 目标重合；默认路径（heading=0, center=None）输出与现状**逐位一致**
4. **docstring 重写**：heading/yaw_mode/center 语义、局部坐标系说明、follow_path yaw 取值表。
5. **`plan_coverage`（World 版）**：死代码（无运行时调用方、无测试），**不改逻辑**，docstring 加"已弃用，请使用 plan_coverage_ned"。

### A2. `src/drone_mission/drone_mission/states/state_recon_patrol.py`

```python
# enter() 参数读取区后:
flight_heading_deg = float(self._fsm.get_param("flight_heading_deg", 0.0))
yaw_mode = self._fsm.get_param("yaw_mode", "hold")
heading_rad = math.radians(flight_heading_deg)
center_n, center_e = self._fsm.recon_zone_ned()   # 新辅助方法 (A3): 侦察区中心 NED = 转场目标

# plan_coverage_ned 调用追加:
heading=heading_rad, yaw_mode=yaw_mode, center_n=center_n, center_e=center_e
# area2 参数直接作为局部范围传入 (u' 范围 = area2_north_min/max 沿制定方向, 语义已局部化)
```

- 日志追加 `yaw_mode/heading/center` 输出（真机可观测性）
- `_fly_to_wp` 已透传 `wp.get("yaw", 0.0)`，**无需改动**
- 默认（制定方向=0°）：`heading=0, center=(57.5,0)` → 输出与现状**逐位一致**（兼容锁定测试）

### A3. `src/drone_mission/drone_mission/mission_state_machine.py`

- `_declare_params`（:210-212 方向块 + Area 块 :228-239）：
  ```python
  self.declare_parameter("flight_heading_deg", 0.0)   # 任务制定方向 (deg, 0=正北, 顺时针)
  self.declare_parameter("yaw_mode", "hold")          # 侦察机头策略: "hold" | "follow_path"
  ```
- `_validate_critical_params`（:1062-1081）：加 `("flight_heading_deg", -180.0, 180.0)`；循环后追加 `yaw_mode not in ("hold","follow_path")` 时 warn
- `_lazy_init`（:356-374）改造：
  ```python
  drop_north, drop_east, recon_north, recon_east = compute_zone_centers(
      float(self.get_param("flight_heading_deg", 0.0)),
      float(self.get_param("drop_distance", 32.5)),
      float(self.get_param("recon_distance", 57.5)))
  ```
  （`compute_zone_centers` 内处理 `flight_axis` 兼容映射：仅当 `flight_heading_deg` 未显式配置时使用；顺带修复 :364 注释与默认值不一致）
- **新增辅助方法**（区域中心 = 转场目标，同一来源，`_lazy_init` 缓存）：
  ```python
  def recon_zone_ned(self): return (self._recon_center_n, self._recon_center_e)
  def drop_zone_ned(self):  return (self._drop_center_n, self._drop_center_e)
  ```
  （`_recon_center_n/e` = `compute_zone_centers` 的 recon 分量 + home 偏移转 NED；`drop_zone` 同理）

### A4. 配置与 launch

- `mission_params.yaml`（:16-18 方向块）：
  ```yaml
  flight_heading_deg: 0.0       # 任务制定方向 (deg, 0=正北, 90=正东, NED 顺时针)
  flight_axis: "north"          # 保留兼容 (仅当 flight_heading_deg 未显式配置时使用)
  ```
  （Area 2 块加 `yaw_mode: "hold"`；`area2_*` 注释更新为"沿制定方向范围"）
- `sitl.launch.py` / `real_flight.launch.py`：声明 `flight_heading_deg_arg`（默认 "0.0"）、`yaw_mode_arg`（默认 "hold"）+ mission_node 参数覆写 + LaunchDescription 列表；`flight_axis` 声明保留
- `test_forward_recon.py`：`__init__` 声明 `flight_heading_deg`/`yaw_mode`；`TRANSIT_FORWARD` 目标沿制定方向（`hx + fwd·sin h, hy + fwd·cos h`）；`_generate_recon_waypoints` 透传 heading/yaw_mode（区域以 (fwd,0) 局部为中心）；`test_forward_recon.launch.py` 同模式声明+透传

### A5. coverage 联动（审查补充，必须配套）

**问题**：CoverageMonitor 网格是世界系轴对齐矩形（`coverage_zone_*`，vision_pipeline.py:222-228 + vision_params.yaml:86-89）；侦察区旋转后足迹斜置，轴对齐网格与足迹不重合 → 覆盖率失真（永远达不到 100% 或提前达标）。

**方案**：CoverageMonitor 支持 zone 旋转（网格在 zone 局部坐标累积）：

1. `src/drone_vision/drone_vision/coverage_monitor.py`：
   - `CoverageMonitor.__init__` 增加 `heading_rad: float = 0.0`；网格尺寸按 u/v 局部范围（`u_range = y_range`, `v_range = x_range`，尺寸与 heading 无关）构建；`update()` 开头把无人机 World 坐标经 `world_to_zone_local` 变换为 (u, v)，后续累积逻辑不变
   - `CoverageMonitorNode` 增加 `coverage_zone_heading_deg` 参数（默认 0.0），转弧度传入
2. `src/drone_vision/drone_vision/vision_pipeline.py`：declare `flight_heading_deg`（:225 后，默认 0.0）+ 构造 `CoverageMonitor` 时传 `heading=math.radians(flight_heading_deg)`（**单一来源：直接读制定方向参数，不新增独立角度**）
3. `vision_params.yaml`：`flight_heading_deg: 0.0`
4. launch（sitl/real_flight）：`flight_heading_deg` 同时透传给 vision_pipeline 与 mission（同一 launch 参数，操作员只改一处）
5. 测试：`drone_vision/test/test_coverage_monitor_rotate.py`——旋转 zone 内扫描覆盖率达标；h=0 与原行为一致（`test_coverage_monitor_scan_lifecycle.py` 保护）

---

## 6. 阶段 B：投放区/转场沿制定方向

### B1. 转场与区域中心（flight_heading_deg，见 A3 已覆盖）

- `compute_zone_centers` 纯函数：`drop/recon` 中心 = `dist·(cos h, sin h)`（NED）
- 配置示例（场地整体旋转 30°）：只需 `flight_heading_deg: 30.0`——区域中心、投放区/侦察区足迹、覆盖率全部自动跟随；`area*_min/max` 数值不变（仍是"沿制定方向 30~35m/55~60m、垂直 ±4m"）

### B2. 投放区沿制定方向（area1 判定局部化）

使用点（已全量排查）：
- `mission_state_machine.py` `_drop_zone_world_bounds`（:648-661）→ `is_in_drop_zone_world`（:1160-1162）
- `mission_state_machine.py` `_normalize_drop_target_xy`（:663-683）→ 圆筒/marker 目标过滤（:564、:613）
- `state_cylinder_approach.py:62-63, 77-78, 117-118`（**安全判定**，3 处调用 `is_in_drop_zone_world`）
- `state_cylinder_search.py:127-141`（慢速扫描线）；:45-48 中心点**天然兼容不改**（中心 = area1 盒中心，局部化后仍为沿制定方向中心）
- `state_drop.py` / `drop_visual_aligner.py` / `precision_align_controller.py` / `cylinder_selector.py` **无区域边界依赖**（视觉对心+世界坐标目标）✓

改造设计（**安全精确判定，禁用 AABB 外扩**）：

1. **`is_in_drop_zone_world`**（安全关键）：World 输入 → `ned_to_world` 逆变换 → `ned_to_local`（制定方向局部坐标，原点=home）→ 判断 `u' ∈ [area1_north_min, max]`、`v' ∈ [area1_east_min, max]`（margin 保留 0.5/0.75）。旋转后 AABB 会让区外点被判为区内（45° 时外扩 ~1.2m），破坏安全语义——**必须精确判定**
2. **`_normalize_drop_target_xy`**：World 输入分支与 NED 输入分支统一走 `ned_to_local` 精确判定（替换轴对齐判断）
3. **`_drop_zone_world_bounds`**：保留，语义改为"制定方向局部盒 → 世界系四角 → AABB"（仅服务诊断/兼容；安全判定走精确路径）
4. **`state_cylinder_search._start_scan`**：扫描线 = 中心 ± 局部半宽沿局部 v' 轴（制定方向+90°）的两个点（局部生成后 `local_to_ned`）
5. 测试：h=0 行为不变；制定方向 90° 时 `is_in_drop_zone_world` 对旋转矩形内外点判定正确（旋转角点 ± 区外点）；`_normalize_drop_target_xy` 旋转过滤

---

## 7. 测试计划

### 7.1 新增测试文件

| 文件 | 用例 |
|---|---|
| `src/drone_utils/test/test_coordinates.py` | 旋转工具互逆、90° 几何、normalize_yaw 边界、`compute_zone_centers`（0°/90°/任意角/flight_axis 映射） |
| `src/drone_mission/test/test_path_planner_coverage_ned.py` | 见下 |
| `src/drone_vision/test/test_coverage_monitor_rotate.py` | 旋转 zone 扫描达标、h=0 兼容 |

`test_path_planner_coverage_ned.py` 用例（默认区 `[55,60]×[-4,4]`、`altitude=-2.5`、`grid_spacing=1.67`）：

1. `test_heading_zero_keeps_current_waypoints_unchanged`：h=0 + hold + center=None，与硬编码期望列表**严格相等**（3 条 east 条带 6 航点：`[(55,-4),(55,4),(57.5,4),(57.5,-4),(60,-4),(60,4)]`，yaw 全 0）——兼容性锁定
2. `test_heading_pi_over_two_rotates_area_about_center`：h=π/2 → 条带 `[(61.5,-2.5),(53.5,-2.5),(53.5,0),(61.5,0),(61.5,2.5),(53.5,2.5)]`，中心 (57.5,0) 不变（assertAlmostEqual places=6）
3. `test_center_matches_transit_target`：**"完整无损"核心用例**——`heading=π/6` + `center=(0, 57.5)`（制定方向 30° 的侦察区中心）时，航点足迹中心 = (0, 57.5)，且第一条条带中点与转场目标重合
4. `test_follow_path_yaw_matches_strip_direction`：east 飞行 ±π/2；north 飞行 0/π；h=π/2 时 π/0
5. `test_hold_mode_always_zero_yaw`：h∈{0, π/2, π/4} 全部 yaw 严格 == 0.0
6. `test_invalid_yaw_mode_raises_value_error`：`pytest.raises(ValueError)`
7. `test_waypoints_stay_inside_rotated_footprint`：h∈{π/6, π/3, -π/4, 120°} 逆变换回局部坐标断言在盒内（tol 1e-9）
8. `test_degenerate_area_returns_empty`：零范围返回 `[]`

### 7.2 现有测试保护

- `test_mission_setpoint_contract.py:204-219`（fixed_heading，默认路径 yaw==0）**必须原样通过**
- `test_recon_coverage_lifecycle.py` mock planner，不受影响
- `test_coverage_monitor_scan_lifecycle.py`（drone_vision）：`CoverageMonitor` 新增 keyword 参数（默认值），现有调用零破坏；网格内部逻辑改局部坐标后需确认仍绿
- `test_flake8.py`/`test_pep257.py` 为 CI 门禁（79 列、docstring 规范）

### 7.3 运行方式

```bash
source install/setup.bash   # egg-link 安装，纯 Python 改动无需 colcon build
python3 -m pytest src/drone_utils/test src/drone_mission/test src/drone_vision/test src/px4_control/test -q
```

---

## 8. 文档更新

| 文件 | 改动 |
|---|---|
| `docs/PARAMETER_USAGE_GUIDE.md` | 基线句改"默认 yaw_mode=hold"；参数表加 `flight_heading_deg`/`yaw_mode`；§4 重写为"侦察 yaw 与任务方向"（yaw_mode 表、制定方向语义、"区域参数沿制定方向"说明、配置示例） |
| `docs/recon_speed_parameter_audit.md` | "yaw 固定 0.0" → "默认 yaw_mode=hold（yaw=0.0），可选 follow_path 经参数启用"，删除"修改规划器固定 yaw"旧指引 |
| `docs/ARCHITECTURE.md` | 弓字形航线描述补 yaw_mode/制定方向语义（§8）；coverage_zone 联动说明 |
| `docs/DEPLOYMENT.md` | SITL 默认值描述追加 `flight_heading_deg=0`、`yaw_mode=hold` |

---

## 9. 实施顺序（每步独立可验证）

| 步骤 | 内容 | 验证 |
|---|---|---|
| 1 | `drone_utils/coordinates.py` 旋转工具 + `compute_zone_centers` + 单测 | test_coordinates 全绿 |
| 2 | path_planner：`plan_coverage_ned` 改造（heading/yaw_mode/center）+ docstring；plan_coverage 弃用注释 | test_path_planner_coverage_ned 全绿 |
| 3 | CoverageMonitor 旋转（coverage_monitor.py + vision_pipeline.py + vision_params.yaml）+ 测试 | test_coverage_monitor_rotate 全绿 |
| 4 | 全量回归（4 包 pytest） | 全绿（含 fixed_heading、flake8/pep257、scan_lifecycle） |
| 5 | mission_state_machine（declare/validate/_lazy_init/recon_zone_ned）；state_recon_patrol 透传 + 日志；mission_params.yaml | drone_mission pytest 绿；SITL 日志含 yaw_mode/heading/center |
| 6 | 3 个 launch + test_forward_recon.py（含 TRANSIT_FORWARD 沿制定方向） | launch 参数透传确认（含 coverage 联动） |
| 7 | 阶段 B：`is_in_drop_zone_world`/`_normalize`/`_start_scan` 局部化 + 测试 | 旋转判定测试绿；SITL `flight_heading_deg:=45` 覆盖率 1.0 |
| 8 | 文档更新 | grep 无旧表述残留 |
| 9 | SITL 端到端 | hold 默认全 0；follow_path 序列 0/π、±π/2；`flight_heading_deg:=45` 全任务链斜向运行（转场 → 投放 → 侦察 → 返航）完整、覆盖率 1.0 |
| 10 | 真机验证 | 场地标定制定方向 → 默认 hold → follow_path 转向无侧滑 → 制定方向 30° 足迹吻合 + 覆盖率 1.0；遥控器随时可接管（RETURN 路径只依赖 yaw=0，不受影响） |

---

## 10. 风险与注意事项

1. **follow_path 下 yaw 低通平滑**（px4_bridge.py:471-478）：条带转向 yaw 跳变，`dyaw` 经 atan2 取最短路径 + `*0.3` 低通，~0.5s 收敛；位置斜坡 0.8m/s 下航点间早已稳定。hold 模式完全无 yaw 运动。真机 follow_path 需目视确认转向无异常侧滑。
2. **NED yaw 归一化**：规划器输出 ∈(-π,π]（如 north 返回条带为 π 而非 -π）；bridge 侧自带 atan2 归一化，两者不冲突；测试断言注意 π/−π 等价（assertAlmostEqual 归一化后比较）。
3. **h=0 位精确一致**：测试 1 用严格相等锁定（防未来重构引入 ±1e-16 漂移）。
4. **coverage 联动是硬要求**：制定方向 ≠ 0 而不联动 coverage_zone → 覆盖率失真（永远 100% 或提前达标）。vision_pipeline 直接声明 `flight_heading_deg`（launch 透传同一参数），单一来源。
5. **投放安全判定**：`is_in_drop_zone_world` 必须用 `ned_to_local` 精确判定（margin 已留 0.5/0.75），**禁用 AABB 外扩**（45° 时区外点被放行 ~1.2m 会改变投放安全边界语义）。
6. **参数管道一致性**：3 个 launch 的声明/透传必须配套完成（步骤 6 完整性），否则 ROS 参数告警。
7. **真机 coverage_zone 坐标系注**：real_flight.launch.py:158-161 用 NED 值（home=原点），vision_params.yaml 用 World 绝对值（SITL home=(4,10)）——两套各自正确，保持现状，文档注明。
8. **区域参数语义变化的影响面**：`area*_north_min/max` 语义从"NED north 范围"变为"沿制定方向范围"——默认 0° 时等价；文档与 YAML 注释必须同步更新，避免操作员按旧语义配置。
9. **范围纪律**：条带端点裁剪优化、follow_path 的 yaw 平滑提速、真实 GPS 经纬度区域定义、区域内足迹独立朝向均列为后续项，本次不做（避免过度设计）。

---

## 11. 完整任务链方向依赖点审计（"完整无损"保障）

| 任务环节 | 坐标依赖点 | 方案覆盖 |
|---|---|---|
| TAKEOFF | 无（原地爬升） | — |
| TRANSIT_TO_DROP / TO_RECON | 区域中心（`compute_zone_centers` + `_lazy_init`） | §5 A3 / §6 B1 |
| CYLINDER_SEARCH 中心 | area1 盒中心（局部化后仍为制定方向中心，天然兼容） | — |
| CYLINDER_SEARCH 慢速扫描线 | `_start_scan` 沿局部 v' 轴 | §6 B2 |
| 圆筒/标记目标过滤 | `_normalize_drop_target_xy` 局部判定 | §6 B2 |
| 投放安全判定 | `is_in_drop_zone_world` 精确判定（3 处调用） | §6 B2 |
| DESCEND / EXECUTE_DROP / ASCEND | 无区域边界依赖（视觉对心） | — |
| 投放精校 / 目标选择 | 无区域边界依赖 | — |
| RECON_PATROL 航点 | `plan_coverage_ned`（heading/center/yaw_mode） | §5 A1-A3 |
| 覆盖率监测 | CoverageMonitor 局部网格（联动制定方向） | §5 A5 |
| RETURN_TO_HOME | `goto_ned(..., yaw=0.0)`，home 固定 | — |
| PRECISION_LAND | H 标识视觉对心，无区域边界依赖 | — |

结论：方向相关坐标依赖点共 **6 处**（区域中心、侦察航点、覆盖率、投放判定 ×3），方案逐一覆盖且**共享同一制定方向来源**；其余环节与方向无关，任意制定方向下自然完整无损。

---

## 12. 关键文件清单

| 文件 | 改动性质 |
|---|---|
| `src/drone_utils/drone_utils/coordinates.py` | **新增**：旋转工具 + `compute_zone_centers`（共享层） |
| `src/drone_mission/drone_mission/path_planner.py` | 核心：`plan_coverage_ned` 签名（heading/yaw_mode/center）与局部坐标+旋转实现 |
| `src/drone_mission/drone_mission/states/state_recon_patrol.py` | 参数读取与透传、日志 |
| `src/drone_mission/drone_mission/mission_state_machine.py` | declare/validate、`_lazy_init` 改造、`recon_zone_ned`/`drop_zone_ned`、投放区判定局部化 |
| `src/drone_mission/drone_mission/states/state_cylinder_search.py` | :127-141 扫描线局部化 |
| `src/drone_mission/drone_mission/states/state_cylinder_approach.py` | 无改动（`is_in_drop_zone_world` 内部语义升级自动生效） |
| `src/drone_vision/drone_vision/coverage_monitor.py` | CoverageMonitor 旋转支持（zone 局部坐标累积） |
| `src/drone_vision/drone_vision/vision_pipeline.py` | declare `flight_heading_deg` + 透传 CoverageMonitor |
| `src/drone_mission/config/mission_params.yaml`、`src/drone_vision/config/vision_params.yaml` | 新参数默认值 + area 语义注释更新 |
| `src/drone_bringup/launch/sitl.launch.py`、`real_flight.launch.py` | 参数声明+透传（含 coverage 联动） |
| `src/test_mission/test_mission/test_forward_recon.py` + launch | 参数声明+透传（含 TRANSIT_FORWARD 沿制定方向） |
| 测试：`test_coordinates.py`、`test_path_planner_coverage_ned.py`、`test_coverage_monitor_rotate.py` | **新增** |
| `docs/PARAMETER_USAGE_GUIDE.md` 等 4 个文档 | 表述同步 |
