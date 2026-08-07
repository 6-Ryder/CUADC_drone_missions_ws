# 方案规划:投放区"边降边校到点即投" + 降落区"低空直接盲降"(用户终版决策)

> 状态:**设计已确认并落盘**(2026-08-07);代码实施(Phase 1)待用户确认后执行
> 基线:HEAD `b149c95`(分支 Full;master 为主分支)
> 数据源:`real_flight_logs/2026_8_07/real_mission_20260807_190630`(19:06, 本次主分析对象)、`real_mission_20260807_185255`(18:52)
> 姊妹文档:`docs/Tactics/PLAN_drop_zone_align_gate.md`(投放对心判定,已实施)、`docs/Tactics/DROP_STALL_ANALYSIS.md`(停滞分析)
> 决策人:用户(2026-08-07 头脑风暴)

---

## 1. 背景与根因(19:06 架次)

### 1.1 现象

| # | 现象 | 日志证据 |
|---|---|---|
| 1 | **投放两次盲投**:2.8m 精校成功(5.0px)后按两级策略降至 1.5m 复查,复查首帧像素误差跳变 232px,振荡 24s 不收敛,目标全丢时强制盲投 | 19:11:34 "视觉精校稳定, 允许投放 5.0px stable=5/4";19:11:57.9 "最终下降停滞, 直接执行投放 pixel_error=infpx"(盲投#1);19:12:38 第二次同型盲投 |
| 2 | **降落无限悬停**:H 标识 2.6m 以下连续单帧跳变(>80px)→ 跳变帧零速悬停 → 3s 判丢 → 重捕获失败 → HOLD_FOR_TAKEOVER,悬停约 2.2m | 19:14:11-31:px_err 331→181 伺服正常,然后 106/261/401px 单帧跳变被拒;"H target was lost after reacquire" |
| 3 | **只扫到一个目标**:高处搜索只识别 1 个区内圆筒,两次投放同一目标(Phase 2 处理) | ID=2 与 ID=1 为同一圆筒换视角重复登记 |
| 4 | 新 .engine 模型(cylinder_new)识别/推理明显增强 | 本次架次受益,保留现状不再改动 |
| 5 | 扫描时飞机沿投放区来回飞行 | 单线航点路径,Phase 2 改单次横穿 |

### 1.2 根因(按伤害排序)

1. **两级降高策略的"复查死结"**(投放盲投主因):2.8m 对心成功后降高到 1.5m 复查,降高改变视角→检测中心跳变(归一化误差 0.02→0.41,非漂移),精校+停滞检测 24s 不收敛,目标丢失时盲投。**降高本身就是复查失败之源**。舵机延迟 1.02s 非主因(投放时零速悬停)。
2. **跳变拒绝→零速悬停→判丢→接管级联**(降落悬停主因):`_get_stable_landing_target` 拒掉跳变帧(合理,防追跳变),但拒绝后 `target is None` 分支下发 `(0,0,0)` 零速度→实际悬停→3s 无有效目标判视觉丢失→一次重捕获失败→接管。**拒绝"追跳变"是对的,错在不该悬停**。
3. **帧级重复登记**(扫描):世界坐标映射误差+换视角,同一圆筒登记多次,掩盖"只找到一个目标"的真相。
4. **下降中精校被抑制**(投放结构性缺陷):`_execute_descend` 169-174 行,下降期间 `_precision_enabled() and not descent_ready` 直接 `return None`,精校与下降互斥——这正是"两级"设计存在的原因,也是修复的着力点。

## 2. 用户终版决策(2026-08-07 头脑风暴)

| 项 | 决策 |
|---|---|
| **范围** | Phase 1 = 投放+降落(扫描策略保留现状);Phase 2 = 扫描(下轮单独交付,两期均推 GitHub,git 历史保留两种策略) |
| **投放策略** | **边降边校,到点即投**——解除下降中精校抑制,速度合成,到点判定全有界(≤30s 必出结果) |
| **降落策略** | **低空直接盲降**——低空跳变不悬停继续降;曾见 H 且 ≤2.5m 直接盲降 |
| **验证** | 本地 pytest + SITL + 真机试飞 |

---

## 3. Phase 1-A 投放三段式(改 `src/drone_mission/drone_mission/states/state_drop.py`)

### 3.1 调度层(零漂移保障)

`_execute_descend`(现 146-257 行)改为分派器:

```python
def _execute_descend(self):
    align_h = self._active_drop_height()          # 2.8m
    final_h = self._fsm.get_param("precision_align_final_drop_height", 1.5)
    enable = (self._fsm.get_param("descend_align_enable", True)
              and self._fsm.get_param("precision_align_final_descent_enable", True)
              and 0.0 < final_h < align_h - 0.05)
    if enable:
        return self._execute_descend_align()
    return self._execute_descend_legacy()         # 原函数体逐字节保留,改名即得
```

- `_execute_descend_legacy()` = 现 `_execute_descend` 函数体原样改名,零行为变化。
- quick 模式 yaml 显式 `descend_align_enable: false` → 走 legacy → **quick 零漂移**(三重防护:yaml 开关 + final_descent_enable 门控 + 契约测试)。

### 3.2 关键机制 `_compose_aligner_descent()`(速度合成,解决 PX4 setpoint 互斥)

PX4 offboard 下位置/速度 setpoint **最后一条生效**——直接放开 169-174 的 early-return 会让精校脉冲把垂直下降切掉。解法:每 tick 调 `_precision_aligner.step()` 后,**检测本 tick 是否发出新脉冲**(`status.command_forward/right` 相对上次 tick 变化),模拟精校器脉冲宽度(fine 0.12s / 粗 0.18s):

```python
def _compose_aligner_descent(self, descent_speed: float):
    """合成水平精校脉冲与垂直下降为一条速度命令."""
    now = time.monotonic()
    status = self._precision_aligner.status()
    fwd, rgt = status.command_forward, status.command_right
    if (fwd, rgt) != self._last_align_cmd:          # 本 tick 新脉冲
        self._last_align_cmd = (fwd, rgt)
        if (fwd, rgt) != (0.0, 0.0):
            self._align_pulse_until = now + self._align_pulse_window_s   # fine 0.12 / coarse 0.18
    if now < self._align_pulse_until:               # 脉冲窗口: 水平+垂直
        return fwd, rgt, descent_speed
    return 0.0, 0.0, descent_speed                   # 停机/悬停窗口: 纯垂直
```

- **脉冲窗口内**下发 `(fwd, rgt, descent)`,**停机/悬停窗口内**下发 `(0, 0, descent)`——水平修正与垂直下降永不互斥。
- `step()` 的 `drop_h` 一律传 `final_h`(1.5)→ 紧窗口 30px 在下降途中即生效,到点即投。

### 3.3 三段式流程(全有界)

| 阶段 | 高度段 | 行为 | 退出条件(任一) |
|---|---|---|---|
| 1 锁定 | 5m→2.8m | 合成命令持续下降 | ① `pixel_error < 60px`(`descend_align_coarse_lock_px`,真机);② 10s 无锁(`descend_align_lock_timeout_s`);③ 到达 2.8m(align_h)。均只簿记+日志,无缝穿过 |
| 2 边降边校 | 2.8m→1.5m,0.35m/s | 持续合成命令;TARGET_LOST 不阻塞(丢失时精校器命令归零→自然发纯垂直) | 到达 final_h(1.5m) |
| 3 到点判定 | 1.5m | 原地继续合成+判定 | ① `_drop_target_is_centered(final_h)` → 立即投;② 目标丢失停滞 2s(`drop_arrival_lost_drop_s`)→ 投;③ 原地精校 ≤5s(`drop_arrival_align_timeout`)→ **无论误差投放**;④ 外层 `drop_total_timeout` 30s 硬上限(自 DESCEND enter 起)→ **强制投** |

**任何路径 ≤30s 出结果**(解决 24s 振荡 + infpx 盲投:盲投从"丢失才投"改为"限时必投")。

### 3.4 配套改动

- `_drop_target_is_centered(drop_h: Optional[float] = None)`(现 473-511)加可选参数,阶段3 显式传 `final_h` 保证紧窗口生效。
- 进入阶段3 时:`fc.stop_velocity()`;若此刻已 TARGET_LOST,则 `self._precision_aligner.reset()` 冷重置,给一次重新锁定机会。
- `mission_state_machine.py`:**只加 declare_parameter 行**(新参数默认值),不动状态机骨架。

---

## 4. Phase 1-B 降落低空盲降(改 `src/drone_mission/drone_mission/states/state_precision_land.py`)

四处修改,其余(`SimplePrecisionLandState`/`landing_simple_mode`、`_execute_search`、`_reset_for_h_reacquire`)不动:

| # | 改动 | 位置 | 语义 |
|---|---|---|---|
| 1 | **低空不计视觉丢失** | `execute()` 78-128 | `altitude ≤ 3.0m`(`landing_jump_bypass_altitude`)时不启动 `_vision_lost_start` 计时 → 断掉"跳变→3s判丢→重捕获→接管"链路(跳变拒绝本身保留,拒绝的是"追跳变",不再导致悬停) |
| 2 | **跳变不悬停** | `_execute_servo()` 161-265 | `target is None` 分支按高度分流:**≤3m** 下发 `(last_fwd, last_rgt, layered_descent)`(锁存修正速度继续降);**>3m** 维持现状 `(0,0,0)`。有效目标分支记录 `_last_servo_fwd/rgt`(限幅后)。抽 `_layered_descent_speed(altitude)` 复用现有 fast/medium/slow 分层 |
| 3 | **锁存放宽** | 原"10 帧连续死区"锁存块 | 删除 10 帧死区锁存,改为 `_has_landing_target`(本会话曾见 H)且 `altitude ≤ 2.5m`(`landing_alignment_latch_max_altitude`,现有参数)→ `_visual_alignment_latched = True`。安全网自动成立(从未见 H 不锁存) |
| 4 | **盲降水平衰减** | `_switch_to_blind()` 380-387、`_execute_blind()` 298-327 | `_switch_to_blind` 锁存 `_last_servo_fwd/rgt`;`_execute_blind` 内水平速度按 `landing_blind_horiz_decay_s`(1s)线性衰减至 0,垂直 0.3m/s 直降;落地检测 `_fallback_touchdown_confirmed` + `_finish_touchdown`(366-378)沿用 |

---

## 5. 参数变更表

文件:`config/mission_params.yaml`(SITL)、`config/mission_params_real.yaml`(真机)、`config/mission_params_quick.yaml`(quick,含 quick SITL 用到的 quick 变体)+ `mission_state_machine.py` declare_parameter 默认。

| 参数 | 默认(declare) | SITL | 真机 | quick |
|---|---|---|---|---|
| `descend_align_enable`(新) | True | true | true | **false** |
| `descend_align_coarse_lock_px`(新) | 60.0 | 120.0 | 60.0 | — |
| `descend_align_lock_timeout_s`(新) | 10.0 | 10.0 | 10.0 | — |
| `drop_arrival_align_timeout`(新) | 5.0 | 5.0 | 5.0 | — |
| `drop_arrival_lost_drop_s`(新) | 2.0 | 2.0 | 2.0 | — |
| `drop_total_timeout`(新) | 30.0 | 30.0 | 30.0 | — |
| `landing_jump_bypass_altitude`(新) | 3.0 | 3.0 | 3.0 | — |
| `landing_blind_horiz_decay_s`(新) | 1.0 | 1.0 | 1.0 | — |
| `drop_descend_speed`(改) | 0.5→**0.35** | 0.35 | 0.35 | 0.8 不变 |
| `landing_alignment_latch_max_altitude`(复用,现有) | 2.5 | 2.5 | 2.5 | 2.5 |

## 6. 测试计划(沿用现有轻量 mock 模式:`test_precision_align_controller.py` 的 `_Clock/_Fsm/_Bridge`)

**新增 `src/drone_mission/test/test_descend_align_strategy.py`(~15 用例)**:
1. 门控:`descend_align_enable=false` → legacy 路径;2. 门控:`final_descent_enable=false` → legacy;3. 门控:`final_h` 越界 → legacy;4. 调度:新模式下 `_execute_descend` 走 `_execute_descend_align`
5. 阶段1:锁定成功(pixel<60px)→ 阶段2;6. 阶段1:10s 无锁 → 阶段2;7. 阶段1:到达 2.8m → 阶段2
8. 合成:脉冲窗口内 `(fwd, rgt, descent)`;9. 合成:停机窗口 `(0,0,descent)`;10. 合成:TARGET_LOST 时纯垂直不阻塞
11. 阶段2:下降中持续合成,到达 final_h → 阶段3
12. 阶段3:release_ready → EXECUTE_DROP;13. 阶段3:丢失 2s → EXECUTE_DROP;14. 阶段3:5s 未对齐 → EXECUTE_DROP;15. 阶段3:30s 总超时 → EXECUTE_DROP(**全有界**)

**新增 `src/drone_mission/test/test_precision_land_low_alt.py`(~10 用例)**:
1. 3.2m 跳变(target None)→ 悬停 `(0,0,0)`;2. 2.8m 跳变 → `(last_fwd, last_rgt, descent)` 继续降;3. 有效帧记录 `_last_servo_fwd/rgt`;4. 曾见 H + ≤2.5m → 锁存;5. 从未见 H → 不锁存;6. ≤3m 视觉丢失不启动计时;7. >3m 视觉丢失仍计时;8. 盲降水平 1s 线性衰减至 0;9. 盲降垂直 0.3m/s 直降;10. 落地检测 → COMPLETE

**修改现有**:
- `test_flight_safety_contract.py` 3 处(mock 高度边界落入 ≤3m 新语义):`-3.0→-3.2` ×2、`-1.0→-4.0`
- `test_quick_drop_strategy.py`:`drop_descend_speed` 断言 0.5→0.35 + 新参数默认断言(`descend_align_enable: false` 语义)

## 7. 提交计划(3 commits,分支 Full,推 GitHub)

1. **设计文档**(批准后首步):本方案全文 → `docs/Tactics/PLAN_drop_landing_optimization.md`(含 Phase 2 设计章节,不实现)→ `设计文档: 投放边降边校+降落盲降两期方案, Phase 2 扫描3.6m横穿+帧间归并待下轮`
2. **投放侧**:state_drop.py + declare_parameter + 两个 base yaml + quick yaml 开关 + 新测试 + quick 契约更新 → `投放改边降边校三段式: 解除下降中精校抑制, 到点判定全有界 (≤30s 必投)`
3. **降落侧**:state_precision_land.py + declare_parameter + yaml + 新测试 + safety contract 更新 → `降落低空跳变不再悬停: 锁存修正速度继续降, 曾见H≤2.5m直接盲降, 盲降水平1s衰减`

## 8. Phase 2 扫描设计(本期只设计不实现,下轮单独推送)

**用户决策**:高度降至 3.6m;在航点位置相对当前前进方向**从左到右单次横穿**;获取两个目标后**分别**执行边降边校投放。

- **高度 3.6m**(`area1_search_altitude` 5.6→3.6,真机参数):目标像素更大、识别更稳;横穿一次覆盖投放区宽度(8m),扫描时间 ≈ 8/0.8 = 10s + 稳定帧,**不显著拉长投放区时间**(回应"会不会导致投放区时间过长"疑虑)。
- **横穿路径**:改 `state_cylinder_search._start_scan`(167-172 单线两点)为三点序列:`[当前航向左侧起点, 当前航向右侧终点]`(机身相对坐标,经 yaw 转 NED),单次横穿**不再来回**。
- **帧间归并**:`mission_state_machine._remember_cylinders`(按 merge_dist 世界距离合并)增强帧间连续性跟踪——同目标连续帧内位置连续性(像素位移 ≤ 阈值且世界位置接近)合并,解决 ID=2 与 ID=1 同一圆筒重复登记。
- **双目标边降边校**:累计 ≥2 目标后,对每个目标依次执行 Phase 1-A 三段式(目标切换时 `aligner.reset()` 冷重置 + 清除锁定状态)。
- **engine**:Jetson 上 `training/export.py --format engine --device 0` 重导 `cylinder_new.engine` / `drop_marker_new.engine`(本次已换新模型,保留验证)。

## 9. 风险要点

1. **quick 模式零漂移**:三重防护(yaml 开关 + final_descent_enable 门控 + 契约测试);legacy 逐字节保留。
2. **速度合成正确性**:合成后全程 velocity 模式,位置目标仅覆盖首 tick——与 2.8m 段现状一致(8.06 对心 9/9 成功先例);若真机修正占空比不足,调 `_align_pulse_until` 模拟窗口。
3. **SITL/真机参数差异**:SITL 窗口更紧(raw 9px),粗窗口按 120px 等比;SITL 回归先过再谈真机。
4. **盲降偏航**(已批准):从未真正对心也盲降的可能偏航,靠 H 大目标 + 全程跟踪限幅缓解,真机架次验证。
5. **Phase 2 横穿覆盖**:3.6m 更低高度 FOV 变小,单横穿可能漏投放区两端——横穿速度 `sweep_speed` 实飞验证,必要时降速。

## 10. 验证(端到端)

1. `python3 -m pytest src/drone_mission/test/ -q` 全绿
2. SITL 端到端:`colcon build --packages-select drone_mission` + `ros2 launch drone_bringup sitl.launch.py`:投放 2/2 且两次 EXECUTE_DROP 有"阶段1/2/3"日志链、降落盲降→COMPLETE、无 HOLD_FOR_TAKEOVER、`mission_end.success=True`
3. `quick_sitl.launch.py` 回归:无阶段日志,证明 quick 零漂移
4. 实飞(下架次):Jetson 同步 + engine 重导 + 真机参数确认
