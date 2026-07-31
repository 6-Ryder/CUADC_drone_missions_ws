# Drone Field — Gazebo 无人机仿真场地

为 Gazebo 无人机仿真任务生成北向飞行布局、投放区和侦察区的圆筒/标识物。

## 目录结构

```
field/
├── drone_field.sdf              # 场地模板（固定结构：地面、围栏、灯光等）
├── generate_random_field.py     # 随机生成脚本（核心）
├── drone_field_random.sdf       # 生成的随机场地（每次运行覆盖）
├── drone_field_meshes/
│   ├── cyl_r0075_h030.stl       # 圆筒网格：半径 0.075m，高 0.30m
│   ├── cyl_r0100_h015.stl       # 圆筒网格：半径 0.100m，高 0.15m
│   ├── cyl_r0100_h030.stl       # 圆筒网格：半径 0.100m，高 0.30m
│   ├── cyl_r0125_h030.stl       # 圆筒网格：半径 0.125m，高 0.30m
│   └── textures/
│       ├── hazard_flammable.png # 易燃标识
│       ├── hazard_health.png    # 健康危害标识
│       └── hazard_danger.png    # 危险标识
└── README.md
```

## 当前布局

Gazebo World 使用 `x=east`、`y=north`。SITL Home 为 World `(4,10,0)`，任务沿 NED north 正方向飞行：

- 投放区：World `y=40..45`，对应 NED north `30..35`、east `-4..4`；
- 侦察区：World `y=65..70`，对应 NED north `55..60`、east `-4..4`；
- H 起降点：World `(4,10)`。

## 功能说明

`generate_random_field.py` 读取场地模板 `drone_field.sdf`，在以下两个子区域内随机放置障碍物：

### 投放区（Drop Zone）

- 中心 World `(4.0,42.5)`，范围 `x=0..8` × `y=40..45`
- 放置 3 个不同尺寸的圆筒（半径 0.075 / 0.100 / 0.125 m）
- 每个圆筒上方生成蓝色 B 区标识圆盘（半径 0.50m）
- 圆筒之间最小间距 1.1m

### 侦察区（Recon Zone）

- 中心 World `(4.0,67.5)`，范围 `x=0..8` × `y=65..70`
- 放置 5 个圆筒（均为半径 0.100m，高 0.15m）
- 随机选择 3 个圆筒，在其上放置危险品标识板（易燃、健康危害、危险）
- 圆筒之间最小间距 0.60m

### 随机策略

- 以子区域中心为均值，采用截断二维正态分布采样 x、y 坐标
- 拒绝采样：保证圆筒整体位于区域内，且任意两圆筒中心间距不小于阈值
- 同一模板每次运行生成不同的随机布局，适合训练场景多样化

## 依赖

- Python 3.7+
- NumPy

```bash
pip install numpy
```

## 用法

```bash
# 默认：读取 drone_field.sdf，输出 drone_field_random.sdf
python3 generate_random_field.py

# 指定输入/输出文件
python3 generate_random_field.py 模板.sdf 输出.sdf
```

运行后会打印每个圆筒的位置信息：

```
已生成随机场地文件: /path/to/drone_field_random.sdf
投放区圆筒位置:
  筒1: x=42.123, y=3.456, r=0.075
  筒2: x=43.789, y=5.012, r=0.100
  筒3: x=41.567, y=2.345, r=0.125
侦察区圆筒位置:
  筒1: x=67.890, y=3.210
  筒2: x=66.543, y=5.678
  ...
```

## 配置

编辑 `generate_random_field.py` 顶部的 `DROP_ZONE` 和 `RECON_ZONE` 字典可调整：

| 参数 | 说明 |
|------|------|
| `center` | 子区域中心坐标 (x, y) |
| `half_size` | 区域长/宽的一半 |
| `sigma` | 正态分布标准差（越小越集中） |
| `min_gap` | 圆筒中心最小间距 |
| `cylinders` | 圆筒规格列表（半径, 高度, STL 文件名） |
| `MAX_TRIALS` | 拒绝采样最大尝试次数 |

## 在 Gazebo 中使用

将生成的 `drone_field_random.sdf` 作为 Gazebo world 文件加载：

```bash
gazebo drone_field_random.sdf
```
