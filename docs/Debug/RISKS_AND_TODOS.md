# 风险与待办

> 2026-07-31 基于代码审计、git 历史、WORKLOG 和 SITL 验证结果整理。
> 2026-08-05 真机链路审计补充：视觉消息停更检测、相机内参固化、落地确认链、PX4 最后防线（见风险 #2/#7-#11）。
> 2026-10-13 按当前 `Optimization` 分支复核降落链：修订风险 #2/#7 与 T5/T13。历史风险按原审计日期保留，引用行号可能已移动。
> 更新：修改代码或完成验证后同步更新本文档。

---

## 风险

### 1. `mission_state_machine.py` 9 处宽泛 `except Exception` 吞异常 🔴

**位置**: `src/drone_mission/drone_mission/mission_state_machine.py:464,496,506,777,813,832,1051,1210,1246`

**场景**: 状态机主循环中异常被捕获后继续运行。line 1051 `except Exception:` 无日志直接继续。如果视觉回调抛异常或状态转换写错属性名，FSM 静默跳过，表现为"卡住不动"而非"报错退出"——真机上极难定位。

**风险本质**: 静默故障 → 真机悬停或错误行为，操作者不知道发生了什么。

**缓解方向**: 改为具体异常类型，或至少加 `traceback` 日志。

---

### 2. 真机落地确认链：备用判据可靠性待验证 🔴

**位置**：`state_precision_land.py` 与 `state_precision_land_simple.py` 都优先使用 PX4 `/fmu/out/vehicle_land_detected`；两者的 `_fallback_touchdown_confirmed()` 在“视觉已锁存且已进入盲降”后允许严格近地静止备用确认，**没有 `sim_mode` 门**，真机也可能进入。柔软地面/草地触地不报告 `land_detected` 的历史记录见 WORKLOG。

**已修复**：full 的 `landing_blind_timeout` 已接线，默认 45 s，到期停止下降并请求人工接管；不能再把它列为无消费者。quick/simple 使用独立超时和 PX4 LAND 兜底。

**剩余风险**：真机 `SYS_HAS_NUM_DIST=0`，备用判据的高度门 `≤0.10` m 依赖气压高度，触地后未必能满足；虽有速度和保持时间门，不能把它当成经真机验证的接地保证。需在不同地面材质记录 PX4 接地信号、估计高度/速度与实际触地时序，再决定保留、修改或增加测距来源。

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

### 7. H 目标消息停更：消费端已加龄期门，端到端故障仍待验证 🟡

**已修复**：`mission_state_machine.py` 的 `_landing_target_cb()` 记录采集时刻、接收时刻和最后一次**含检测**的时刻；`get_landing_target()` 在真实检测超过 2 s 后返回 `None`。因此不能再说最后一条高置信度 `LandingTarget` 会无限留在 FSM 缓存。相机停流与持续 miss 帧均会使消费端目标过期。

**剩余风险**：视觉状态 `ready` 与消息龄期不是同一信号；对杀掉视觉进程、相机断流、持续空帧的端到端处置尚未在完整 SITL/真机验证。需检查状态转换、目标失效和 quick/full 各自的兜底是否按预期发生。

---

### 8. 相机内参为占位值，且被预检脚本断言固化 🔴

**位置**: `real_flight.launch.py` camera_node/vision_node 参数块硬编码 `fx=fy=400, cx=320, cy=240`（640×480，未标定）；`verify_real_flight_ready.sh:66-68` grep 断言 `- 400.0 / - 320.0 / - 240.0`。

**矛盾**: 完成 T1（标定内参）后预检脚本必然 fail——检查脚本固化了它自己承认是错误的占位值。

**影响**: 像素闭环（投放精校/降落伺服）可在一定程度容忍内参误差，但世界坐标投影与目标记忆依赖当前 `coordinate_transforms.pixel_to_world()` 的内参。目标世界坐标仅在新鲜 6-DoF pose 可用时才标记有效；内参占位值仍会放大投影误差，并可能让同一圆筒被记成多个候选。降落估计器的像素到米换算也使用任务侧 `camera_fx/fy`，须与视觉侧一起标定。

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
| T5 | 实测真机 `land_detected` 与严格近地静止备用判据；按结果决定判据或测距来源。`landing_blind_timeout` 已接线 | 风险 #2 |
| T13 | H 目标 2 s 龄期门已实现；补相机断流/视觉进程退出/持续空帧的端到端 SITL 与真机验证，评估 status 帧龄和 watchdog 联动 | 风险 #7 |
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
- **声明即接线**: 声明为安全保护的参数必须被代码消费并有测试覆盖（`landing_blind_timeout` 已补接线）；不留"写了没接"的防护。
- **PX4 级最后防线**: Jetson 侧保护全部失效时，PX4 固件必须仍有自动处置（`COM_OBL_RC_ACT` 不得为 0）。
