# 2026-07-14 工作日志 — SITL 仿真审计 + 真机部署差距分析

## 一、仿真效果评估

### 最新仿真: `simulation_20260714_111330`

| 指标 | 值 |
|------|-----|
| 总时长 | ~180s (INIT → RECON 过渡) |
| 投放 #1 耗时 | 41s (timeout=40s, 临界) |
| 投放 #2 耗时 | 95s (timeout=40s, 严重超时) |
| height_error WARN 数 | 98 条 |
| 投放结果 | 2/2 成功, 像素误差 30.8px / 24.9px |

### 核心问题: height_error 阻塞

无人机降高至 2.0m 时产生稳定的 0.30-0.37m 高度误差, 恰好等于 `precision_align_drop_height_tolerance: 0.30`。状态机在高度达标和高度回漂之间反复振荡, 精校被频繁中断。

### 根因

PX4 SITL (X500, MPC_THR_HOVER=0.6) 高度位置环只有 P 控制器 (MPC_Z_P=1.0), Gazebo 物理模型与 PX4 推力估计不完全匹配, 导致 ~0.3m 稳态误差。真机上 MPC_THR_HOVER_LEARN 会自动消除。

### 修复

`mission_params.yaml` — `precision_align_drop_height_tolerance: 0.30 → 0.40`

精度不受影响: PrecisionAlignController 是纯像素闭环 (drop_height 参数未被使用)。真机无此问题。

---

## 二、真机部署审计

### 策略

SITL 代码冻结不改, 真机通过独立的 launch/config/script 层覆盖差异。核心状态机、视觉管线、飞行控制器共用。

### SITL 路线 (已跑通)

| 组件 | 配置 |
|------|------|
| Launch | `sitl.launch.py` |
| 脚本 | `start_full_simulation.sh` |
| ARM | `param2=21196` (force arm, 跳过预检) |
| 舵机 | ServoMock, 故障时静默成功 |
| 相机 | Gazebo ros_gz_bridge |
| 通信 | MicroXRCEAgent UDP 8888 |
| 视觉模型 | `.pt` (PyTorch) |

### 真机路线 (待开发)

| 组件 | 需创建/修改 |
|------|------------|
| Launch | `real_flight.launch.py` ✅ 已存在 |
| 脚本 | `start_real_flight.sh` ❌ |
| ARM | 标准 `param2=0` 解锁 |
| 舵机 | PCA9685 / Jetson.GPIO, 故障时报错 |
| 相机 | GStreamer (CSI/USB) |
| 通信 | MicroXRCEAgent serial |
| 视觉模型 | `.engine` (TensorRT) |
| 相机标定 | fx/fy/cx/cy 真机实测值 |
| GPS/电池 | 订阅 `/fmu/out/battery_status` 等真实话题 |
| PX4 airframe | 真机参数, 禁用 SITL bypass |

### 已知 gap (blocker 级别)

1. `mission_state_machine.py:1190` — ARM 用 SITL force magic (21196)
2. `mission_state_machine.py:965-970` — Drop 失败静默模拟成功
3. `px4_bridge.py:251-254` — GPS/电池遥测硬编码假值
4. `vision_params.yaml:10-13` — 相机内参 SITL 默认值
5. 无 `start_real_flight.sh`
6. 无真机 PX4 airframe 配置

---

## 三、配置变更记录

| 文件 | 变更 | 原因 |
|------|------|------|
| `mission_params.yaml:99` | `precision_align_drop_height_tolerance: 0.30 → 0.40` | SITL 稳态偏差卡边界 |

---

## 四、Memory 条目

- `sitl-height-steady-state-error` — 高度偏差根因分析
- `real-hardware-deployment-gaps` — 真机部署审计 + 两条路线策略
