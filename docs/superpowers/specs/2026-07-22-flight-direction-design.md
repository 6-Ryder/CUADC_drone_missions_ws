# 飞行方向可配置化 — 方案对比

## 背景

当前系统硬编码飞行方向为**向东**（NED east = 正方向）。

```python
# mission_state_machine.py
DROP_NED_OFFSET = (0.0, 32.5)    # (north, east) — 投放区在 Home 东 32.5m
RECON_NED_OFFSET = (0.0, 57.5)   # 侦察区在 Home 东 57.5m
```

未来场地可能要求**向北飞行**。需要一种方式在东西南北之间切换。

---

## 方案 A：改所有硬编码

### 做法

把项目里所有涉及方向的常量、YAML 默认值、Shell 脚本默认参数**全部改成北向**。

### 涉及文件

| 文件 | 改动 | 说明 |
|------|------|------|
| `mission_state_machine.py` | `DROP_NED_OFFSET = (32.5, 0.0)` | north 32.5m, east 0m |
| `mission_state_machine.py` | `RECON_NED_OFFSET = (57.5, 0.0)` | north 57.5m, east 0m |
| `mission_params.yaml` | `home_x: 4.0, home_y: 10.0` | Home 点坐标旋转 90° |
| `mission_params.yaml` | area1/area2 的 north/east 互换 | 搜索区跟着旋转 |
| `sitl.launch.py` | `home_x=4.0, home_y=10.0` | 仿真默认 Home |
| `real_flight.launch.py` | 不变 | 真机 Home 本来就是 `(0,0)` |
| `px4_bridge.py` | `home_x=4.0, home_y=10.0` | 默认参数值 |
| `start_full_simulation.sh` | `PX4_GZ_MODEL_POSE="4,10,0.3,0,0,0"` + `home` 参数 | 无人机生成点 |
| `simple_mission.sh` | 同上 | |
| `simple_mission.py` | `drop_x=4.0, drop_y=42.5, home_x=4.0, home_y=10.0` | 默认参数值 |
| `simple_mission_real.sh` | `drop_x:=2.0 drop_y:=20.0` | 向北 20m 偏右 2m |
| `field/drone_field.sdf` | 所有 model pose 的 x↔y 互换 | Gazebo 场地旋转 90° |

### 切回东向的代价

把以上 12 处全改回来。或者保留一个 git commit，切方向时 `git revert`。

### 摘要

- **改动文件**：12 个
- **改动形式**：每个文件改具体数值
- **切方向代价**：改 12 个文件（或 revert 一个 commit）
- **运行时开销**：零
- **出错风险**：高 — 漏改一处则坐标不一致，仿真/真机行为异常

---

## 方案 B：设计方向参数

### 做法

新增 3 个 YAML 参数，代码在初始化时读取参数动态计算 NED 偏移。

### 新增 YAML 参数

```yaml
# mission_params.yaml
/mission_state_machine:
  ros__parameters:
    flight_axis: "east"          # 主飞行轴: "east" | "north"
    drop_distance: 32.5          # Home → 投放区距离 (m)
    recon_distance: 57.5         # Home → 侦察区距离 (m)
```

### 代码改动（仅 1 个文件）

`mission_state_machine.py`：

```python
# 删除
DROP_NED_OFFSET = (0.0, 32.5)
RECON_NED_OFFSET = (0.0, 57.5)

# _lazy_init() 中改为
flight_axis = self.get_param("flight_axis", "east")
drop_dist = self.get_param("drop_distance", 32.5)
recon_dist = self.get_param("recon_distance", 57.5)

if flight_axis == "north":
    drop_offset = (drop_dist, 0.0)    # (north, east)
    recon_offset = (recon_dist, 0.0)
else:  # east (default)
    drop_offset = (0.0, drop_dist)
    recon_offset = (0.0, recon_dist)

drop_zone = (hx + drop_offset[1], hy + drop_offset[0], altitude)
recon_zone = (hx + recon_offset[1], hy + recon_offset[0], altitude)
```

### 配合 area1/area2 搜索区

area1/area2 的 NED 坐标 YAML 值取决于飞行方向。切换方向时需要手动改 `mission_params.yaml` 里的 area 参数：

```yaml
# 东向
area1_north_min: -4.0
area1_north_max:  4.0
area1_east_min:  30.0      # 32.5 ± 2.5
area1_east_max:  35.0

# 北向（手动改成）
area1_north_min: 30.0      # 32.5 ± 2.5
area1_north_max: 35.0
area1_east_min:  -4.0
area1_east_max:   4.0
```

> 这部分不自动计算的原因是：area 边界和具体场地形状有关，不应由 `flight_axis` 一个参数决定全部。

### 使用方式

```bash
# 向东（默认）
ros2 launch drone_bringup sitl.launch.py

# 向北 — 命令行覆盖，不需要改任何文件
ros2 launch drone_bringup sitl.launch.py \
    flight_axis:=north drop_distance:=20.0 recon_distance:=50.0
```

### 摘要

- **改动文件**：2 个（`mission_params.yaml` + `mission_state_machine.py`）
- **改动形式**：+3 个参数声明 + 删 2 行硬编码 + ~10 行动态计算
- **切方向代价**：改 1 个 YAML 值或命令行传参
- **运行时开销**：零（计算在 `_lazy_init()` 中，仅执行一次）
- **出错风险**：低 — 单点配置，不一致不可能发生

---

## 对比总结

| 维度 | 方案 A（改硬编码） | 方案 B（方向参数） |
|------|-------------------|-------------------|
| 改动文件数 | 12 | 2 |
| 切换方向改动点 | 12 处 | 1 个参数 |
| 漏改风险 | 高 | 无 |
| 运行时性能 | 无影响 | 无影响 |
| 运行时内存 | 无影响 | 无影响 |
| 扩展性（倾斜飞行等） | 不支持 | 可扩展 `flight_angle` 参数 |
| Git 可维护性 | 方向切换 = 新 commit | 方向切换 = 配置文件修改 |

---

## 推荐

**方案 B**。改动量小、单点控制、不出错。方案 A 适合"永远不会再改方向"的场景，但既然你已经问到了"以后怎么改"，说明方向可能变化，方案 B 更合适。
