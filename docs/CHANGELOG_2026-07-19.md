# CHANGELOG 2026-07-19 — 舵机真机控制 C++ 化 & 多位置步进投放

## 架构变更

舵机控制链路从"Jetson 直连 GPIO/PCA9685"改为"飞控中转"标准架构:

```
旧: Jetson → GPIO/PCA9685 → 舵机          ← 实际硬件不存在
新: Jetson → VehicleCommand → PX4 → MAIN5 → 舵机  ← 标准飞控架构
```

## 新增: drone_servo2 (C++)

`src/drone_servo2/` — 舵机多位置步进投放控制器 (C++ 实现).

| 文件 | 行数 | 说明 |
|---|---|---|
| `include/drone_servo2/servo_interface.hpp` | 28 | IServo 抽象接口: `set_pwm(pwm_us)` |
| `include/drone_servo2/drop_manager.hpp` | 104 | 多位置步进投放状态机 |
| `src/servo_controller.cpp` | 400 | ServoMock + PX4CommandServo + ROS2 节点 |
| `config/servo_params.yaml` | 16 | 投放参数配置 |

### 投放逻辑

```
positions = [1150, 1700, 2150] μs  (PWM range: 1150-2150)
  初始:  舵机 = 1150μs (闭合, 挂载两枚)
  投放1: 舵机 → 1700μs (释放第一枚)
  投放2: 舵机 → 2150μs (释放第二枚)
```

舵机只进不退, 每次投放移动到下一个 PWM 位置.

### 硬件适配器

| 模式 | 类 | 行为 |
|---|---|---|
| SITL | ServoMock | 打日志, 模拟位置变化 |
| Jetson | PX4CommandServo | 发布 VehicleCommand(DO_SET_ACTUATOR, cmd=187) → px4_bridge → PX4 → MAIN5 |

### PWM 归一化

```
normalized = (pwm_us - pwm_center) / pwm_half_range
pwm_center = (1150 + 2150) / 2 = 1650
pwm_half_range = (2150 - 1150) / 2 = 500

1150μs → -1.0    1700μs → +0.1    2150μs → +1.0
```

### 依赖

- `rclcpp`, `drone_interfaces`, `px4_msgs`

## 更新: drone_servo (Python)

Python 版 `drone_servo` 同步更新为多位置步进逻辑, 与 C++ 版完全一致.
保持为参考实现, 实际部署使用 C++ 版.

| 文件 | 改动 |
|---|---|
| `servo_mock.py` | open/close → set_pwm(); 增加初始位置 |
| `drop_manager.py` | 开/关循环 → 多位置步进: positions=[1150,1700,2150] |
| `servo_controller.py` | 新增 PX4CommandServo (发布 VehicleCommand) |
| `servo_params.yaml` | 同步 C++ 版参数 |
| `package.xml` | 新增 px4_msgs 依赖 |

## 更新: Launch 文件

| 文件 | 改动 |
|---|---|
| `real_flight.launch.py` | drone_servo → drone_servo2 (C++) |
| `sitl.launch.py` | drone_servo → drone_servo2 (C++), 显式设置 platform:=sitl |

两个 launch 现在统一使用 `drone_servo2`, SITL 和真机共用一套 C++ 代码.

## 新增: 测试脚本

`src/drone_servo2/scripts/`

| 脚本 | 功能 |
|---|---|
| `check_servo.sh` | 环境诊断: 检查 ROS2/编译/运行是否正常 |
| `test_servo_full.sh` | 全链路: 启动节点 + 2次投放 + 越界拒绝 |
| `demo.sh` | 完整演示: 含 VehicleCommand 消息捕获 |

## QGC/PX4 配置

舵机物理连接: 舵机 → CUAV 7-Nano MAIN5 (PWM 50Hz)
QGC 配置: MAIN5 = "Peripheral via Actuator Set 1", PWM range 1150-2150μs

## 验证结果

| 测试项 | 结果 |
|---|---|
| C++ 编译 | ✅ 零 warning |
| Python 编译 | ✅ 通过 |
| SITL 模式启动 | ✅ 序列 [0]=1150 [1]=1700 [2]=2150 |
| Jetson 模式启动 | ✅ PX4 VehicleCommand 模式 |
| 投放 #1 | ✅ 1150→1700μs, success=True |
| 投放 #2 | ✅ 1700→2150μs, success=True |
| 投放 #3 (越界) | ✅ success=False, "Cannot drop: done" |
| VehicleCommand | ✅ command=187, param7=1, 归一化正确 |
