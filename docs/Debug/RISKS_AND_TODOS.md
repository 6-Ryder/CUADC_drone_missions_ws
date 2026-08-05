# 风险与待办

> 2026-07-31 基于代码审计、git 历史、WORKLOG 和 SITL 验证结果整理。
> 2026-08-05 真机链路审计补充：视觉消息停更检测、相机内参固化、落地确认链、PX4 最后防线（见风险 #2/#7-#11）。
> 更新：修改代码或完成验证后同步更新本文档。

---

## 风险

### 1. `mission_state_machine.py` 9 处宽泛 `except Exception` 吞异常 🔴

**位置**: `src/drone_mission/drone_mission/mission_state_machine.py:464,496,506,777,813,832,1051,1210,1246`

**场景**: 状态机主循环中异常被捕获后继续运行。line 1051 `except Exception:` 无日志直接继续。如果视觉回调抛异常或状态转换写错属性名，FSM 静默跳过，表现为"卡住不动"而非"报错退出"——真机上极难定位。

**风险本质**: 静默故障 → 真机悬停或错误行为，操作者不知道发生了什么。

**缓解方向**: 改为具体异常类型，或至少加 `traceback` 日志。

---

### 2. 真机落地确认链：`land_detected` 失效时无可靠自动退出 🔴

**位置**: `state_precision_land.py` — 真机以 PX4 `/fmu/out/vehicle_land_detected` 为主判定接地；已知柔软地面/草地触地不报告（WORKLOG "2026-07-28 — H 标识视觉精准降落"）。

**现状不一致（代码/配置/文档三层）**: 代码 `_fallback_touchdown_confirmed()`（`state_precision_land.py:329`）**无 sim_mode 门控**，真机同样激活；`mission_params_real.yaml` 注释允许（"仅已视觉对心且进入盲降后才允许严格静止备用确认"）；但本文档此前写"真机没有 fallback"——已过时。

**物理上不可靠**: 真机 `SYS_HAS_NUM_DIST=0` 无距离传感器，`altitude ≤ 0.10m` 靠气压计（±0.3m 漂移常见）→ 兜底在 `land_detected` 失效的场景大概率同样失效。（补充：兜底含 `vertical_speed ≤ 0.05` 速度门，盲降持续指令 0.30m/s 下降，故不会在空中误触发 force-disarm——问题是"不可靠"而非"危险"。）

**`landing_blind_timeout` 未接线**: 参数在 `mission_state_machine.py:344` 与 yaml 中声明（45s），但 `state_precision_land.py` 的 `_execute_blind` 从未读取；`_blind_start_time` 只写不读 → 盲降无硬性时间上限，退出只剩 land_detected / 不可靠兜底 / 30s 无进展监测。

**场景**: 触地但 land_detected 不报 → 电机持续指令下降 30s+，期间依赖操作者接管。

**缓解方向**: 三选一并统一三层（① 兜底加 sim_mode 门控，真机只信 land_detected；② 真机保留兜底但高度容差提到 0.2-0.3m；③ 加距离传感器）；接线 `landing_blind_timeout` 并真机验证。

---

### 3. test_mission 取消回升后 3m 返航的障碍物碰撞 🔴

**位置**: 最新提交 `f053f87` — 删除 `ASCEND_TO_CRUISE`，侦察扫描完成后保持侦察高度直接返航。

**场景**: 3m 相对高度在真机场地上可能低于树木、灯杆等未建模障碍物。之前的设计 5m → 3m 侦察 → 5m 回升 → 返航 → 降落，现在去掉了保护层。

**风险本质**: 真机碰撞风险上升。

**缓解方向**: 评估真机场地后决定是否增设安全高度参数。

---

### 4. SITL/真机 v1/v4 消息混用导致 crash 🟡

**环境边界**: `scripts/sitl_env.sh` 控制 overlay 优先级，人工操作。如果忘记 source 或误加载了相反版本的 overlay，Python 消息类型不兼容会直接抛异常。

**风险本质**: 操作失误 → 启动即 crash。脚本已封装（`start_full_simulation.sh` 内部 source），但人类仍可能绕过。

**缓解方向**: 现有封装已基本充分，真机部署文档明确说明 SITL/真机环境切换步骤。

---

### 5. 视觉模型异常退出时泄漏 🟡

**位置**: ARCHITECTURE.md line 109 — 投放模型在进入侦察阶段通过 `close()` + 垃圾回收释放。

**场景**: 如果侦察阶段异常退出（FSM ERROR 路径），释放逻辑可能被跳过。真机 Jetson 内存/显存有限，泄漏累积数次飞行后推理延迟上升 → 投放精校或降落伺服超时。

**风险本质**: 资源累积退化，不是一次性故障。

**缓解方向**: 释放逻辑放入 `finally` 块，或在 FSM ERROR 路径也触发释放。

---

### 6. 0 个 TODO/FIXME 标记 🟢

全源码 Python 文件里没有 TODO、FIXME、HACK 标记。所有临时方案都以"最终方案"提交，代码里没有留下"这里需要重新审视"的痕迹。

**风险本质**: 知识流失，维护者没有路标，不是运行风险。

---

### 7. 视觉"消息停更"在 FSM 消费端不可检测 🔴

**位置**: `mission_state_machine.py` — `_landing_target_cb`（:710）不记录时间戳，`get_landing_target()`（:1264）永不过期；`vision_pipeline.py:484` 的 `ready = initialized && models_validated && frame_count>0`，相机死后永久 `ready=1`；`_cyl_cb`（:554）只在收到消息时调用 `emergency.update_vision`，且主循环（:757-767）只响应 `RTL/LAND/ABORT`，视觉丢失的 `WARN` 被无视。

**场景**: 真机降落/投放阶段相机断流或视觉进程崩溃 → 消息停止，但最后一条高置信度 `LandingTarget` 永远留在缓存 → 降落伺服用冻结的像素误差继续下降，视觉丢失保护与 `vision_loss_timeout` 全部失效；vision watchdog 通道无 `on_timeout` 且主循环不消费 `watchdog.check()` 结果。

**对照**: `get_cylinders`/`get_markers`/`get_hazards` 均有 2s 新鲜度门，唯独 `landing_target` 缺失——属遗漏而非设计。

**SITL 未暴露原因**: 仿真管线持续发布 `confidence=0` 空目标，confidence 字段偶然充当新鲜度信号；SITL 验证从未执行过"杀掉视觉进程"场景。

**缓解方向**: `get_landing_target` 加时间戳门；vision status 增加帧龄字段；emergency 增加消息停更检测（对关键话题到达时间做检测，而非依赖收到空消息）。

---

### 8. 相机内参为占位值，且被预检脚本断言固化 🔴

**位置**: `real_flight.launch.py` camera_node/vision_node 参数块硬编码 `fx=fy=400, cx=320, cy=240`（640×480，未标定）；`verify_real_flight_ready.sh:66-68` grep 断言 `- 400.0 / - 320.0 / - 240.0`。

**矛盾**: 完成 T1（标定内参）后预检脚本必然 fail——检查脚本固化了它自己承认是错误的占位值。

**影响**: 像素闭环（投放精校/降落伺服）对内参鲁棒；但世界坐标记忆（`vision_pipeline.py:768` `pixel_to_world_simple`）与投放记忆目标受线性放大。例：真实 fx=500 vs 猜测 400 → 5m 高、偏 320px 的目标世界坐标偏 0.8m > `cylinder_memory_merge_distance=0.75` → 同筒记两候选，或低空真实目标落出假设视场 → 检测丢失重试。

**缓解方向**: 标定内参；验证脚本改为"有发布者 + 值在合理范围/参数化"断言。

---

### 9. `COM_OBL_RC_ACT=0`：Jetson 失效时 PX4 无固件级自动处置 🔴

**位置**: `飞控参数修改记录.md`"未修改但有影响的参数"表（自注"建议后续改为 2=Land"），未纳入任何待办追踪。

**场景**: lease 与 FSM offboard 检测只保护 FSM 进程；px4_bridge/MicroXRCEAgent/Jetson 整机崩溃时，最后防线在 PX4 固件侧。`COM_OBL_RC_ACT=0` → offboard 丢失无自动动作，完全依赖操作者手持 RC 及时接管。

**缓解方向**: 改 2=Land，真机实测 offboard 丢失行为（确认触发降落而非失控），写入文档。

---

### 10. `state_max_duration: 0.0` 魔法值禁用状态超时层 🟡

**位置**: `mission_params_real.yaml`（注释"不用固定总时长中止真机任务"）；`emergency.py:199` 要求 `effective_timeout > 0` 才检查。

**代价**: 状态级超时整层失效，ProgressMonitor（30s 窗口）成为唯一状态保险；若某状态 ProgressMonitor 逻辑有缺陷（如 `state_drop.py:562` `_alignment_stalled` 对非有限 pixel_error 直接返回 False），该状态可无限挂死。

**缓解方向**: 显式开关参数（如 `state_timeout_enabled`）替代魔法 0，让参数校验可感知。

---

### 11. PX4Bridge/StandaloneBridge 接口漂移 🟡

**位置**: `_handle_complete` 调用 `bridge.stop_autonomous_output()`（`mission_state_machine.py:947`），`request_takeover` 调用 `bridge.hold_for_takeover()`（:973）——PX4Bridge 均无此方法；PX4Bridge 直接接口（`set_position_target` 等）无 lease 门控。

**场景**: 当前部署 FSM 永远走 StandaloneBridge 故未暴露；一旦按"直接模式"接入 PX4Bridge，任务结束/接管即 AttributeError，或 setpoint 绕过 fail-closed。

**缓解方向**: 抽公共契约（Protocol/文档），两边实现同一方法集；或删除未使用的直接模式接口。

---

## 待办

### 必须做（真机飞行前）

| # | 事项 | 依据 |
|---|---|---|
| T1 | 标定 640×480 相机内参 + H 目标实际尺度；**同步将 verify_real_flight_ready.sh 改为参数化/范围断言** | WORKLOG "当前仍需真机验证 #1" + 风险 #8 |
| T2 | 验证 Jetson-CUAV 7-Nano 的 v1 DDS 连接、遥控器 Arm Switch 和 failsafe | WORKLOG #2 |
| T3 | 标定 MAIN5 PWM 舵机行程（1150→1700→2150μs），验证两次投放实物结果 | WORKLOG #3 |
| T4 | 验证真机 `vehicle_land_detected` 触发时序（各种地面材质：水泥/草地/软土） | WORKLOG #4 |
| T5 | 决策真机接地兜底（① sim_mode 门控 ② 提高高度容差 ③ 加距离传感器），统一代码+yaml+文档；**接线 `landing_blind_timeout`** | 风险 #2 |
| T13 | 视觉新鲜度闭环：`get_landing_target` 加时间戳门、vision status 增加帧龄、emergency 增加消息停更检测 | 风险 #7 |
| T14 | `COM_OBL_RC_ACT` 改 2=Land 并真机实测 offboard 丢失行为 | 风险 #9 |

### 应该做（质量提升）

| # | 事项 | 关联风险 |
|---|---|---|
| T6 | 审计 9 处 `except Exception`，按危险度分类：关键路径至少加 `traceback` 并显式决策（重试 or ERROR），不允许"捕获后继续下一个循环" | 风险 #1 |
| T7 | 视觉模型释放放入 `finally` 块，ERROR 路径也触发释放 | 风险 #5 |
| T8 | 评估 test_mission 3m 返航高度是否需要在真机场地增设安全高度参数 | 风险 #3 |
| T9 | 清理 `src/*/install/` `src/*/build/` 磁盘残留（不在 git 跟踪中但目录存在） | — |
| T15 | 消除双 bridge 接口漂移：抽出公共契约（Protocol/文档），PX4Bridge 补齐 `stop_autonomous_output`/`hold_for_takeover`，或删除未使用的直接模式 | 风险 #11 |
| T16 | `state_max_duration` 改为显式开关参数，去掉"0=禁用"魔法语义 | 风险 #10 |

### 可以做（债务清理）

| # | 事项 |
|---|---|
| T10 | 关键路径加 TODO 标记（如 "真机核实后移除此 fallback"） |
| T11 | 决策 `drone_servo`（Python 遗留舵机包）和危险品分类模块：归档 or 删除 |
| T12 | 推进 6 阶段真机部署路线图 + FSM `mission_mode` 可测试性设计 |

---

## 设计不变量（从风险推演的保护规则）

- **接地判定**: 真机不能只信 `vehicle_land_detected`。参考 SITL 的严格条件作为兜底。
- **返航高度**: 任何返航路径必须高于沿途可能障碍物。真机场地测绘完成前，不降低返航高度。
- **异常处理**: FSM 关键路径不允许裸 `except Exception` 吞噬。异常必须记录 traceback 并显式决定 ERROR or 重试。
- **模型生命周期**: 任何模型加载必须有对称的释放路径，覆盖正常退出和异常退出。
- **环境边界**: SITL 和真机的 px4_msgs overlay 切换必须由脚本保证，不能依赖人类记忆。
- **信号模型**: 关键输入必须同时检测"收到空消息"与"消息停更"两种失效；消费端对关键话题（如 `/drone/vision/landing_target`）做时间戳新鲜度门。
- **标定一致性**: 验证脚本不得固化未经标定的占位值；内参标定必须与预检断言联动修改。
- **声明即接线**: 声明为安全保护的参数必须被代码消费并有测试覆盖（`landing_blind_timeout` 教训）；不留"写了没接"的防护。
- **PX4 级最后防线**: Jetson 侧保护全部失效时，PX4 固件必须仍有自动处置（`COM_OBL_RC_ACT` 不得为 0）。
