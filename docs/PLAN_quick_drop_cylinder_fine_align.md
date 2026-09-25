# quick 投放策略：双通道视觉 + 地面误差口径 + 两级降高 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 quick 策略的投放段用「圆筒 + 蓝盘双通道视觉」定位，并用「地面误差（米）」而非「像素误差」作为释放判据，配合投放高度 2.4 m → 1.2 m，把载荷尽可能投进目标圆筒中心。

**Architecture:** 视觉侧把 quick 的检测模式从 `marker_only` 恢复为 `all`（双模型同时推理，圆筒通道自带实测半径）；决策侧把投放目标源切到「圆筒优先、蓝盘兜底」并选出最小筒；控制侧在既有 `PrecisionAlignController` 上新增 `"fused"` 目标源（按高度调度主/副通道，失效即接管），并把释放门限从像素口径换成物理口径；状态机侧启用已存在但被 quick 关闭的两级降高。

**Tech Stack:** ROS 2 Humble · Python 3 · PX4 Offboard · YOLO TensorRT 引擎 · pytest

**创建日期:** 2026-09-19

---

## Global Constraints

以下约束适用于**每一个** task，不再逐条重复：

1. **代码基线 = git 分支 `jetson`（tip `ec1aa03`）**，不是当前所在的 PlanB 工作树。所有路径均相对仓库根，行号均指该基线的文件。实施前请 `git checkout jetson`（或在该分支的工作树中作业）。
2. **真机部署目录**：Jetson 上为 `/home/nvidia/CUADC_drone_missions_ws`。
3. **不动** 基础策略 `src/drone_mission/config/mission_params.yaml` 与真机全流程 `mission_params_real.yaml`。
4. **不修改** `precision_align_target_source` 的既有取值语义：`"marker"` = 只追蓝盘，`"cylinder"` = 只追圆筒。`test_precision_align_controller.py:232-258`（`test_cylinder_precision_does_not_fallback_to_marker`）必须保持绿色。
5. **不修改** 圆筒与降落模型权重文件。
6. **所有新增参数默认值必须等于「旧行为」**，使既有单元测试零漂移，行为变化只可能来自 quick yaml。
7. **单位与坐标系**：地面误差单位 m；`_ground_error_m` 返回 `(forward, right)`；forward 用 `dv` 与 `fy`，right 用 `du` 与 `fx`。
8. **真机内参**：`camera_fx=fy=400`、`camera_cx=320`、`camera_cy=240`、图像 640×480。
9. **SITL 内参**：`fx=fy=539.9`、`cx=640`、`cy=480`、图像 1280×960。
10. 本文档中标注 **【已核实】** 的行号与代码片段均已逐条读源确认；标注 **【假设】** 的需实测验证。

---

## 1. 为什么做这件事

### 1.1 问题

真机 quick 策略（`scripts/start_quick_mission_real.sh` → `quick_flight.launch.py`）为压缩投放区耗时，在 commit `a908070` 把视觉切成 `marker_only`（只跑蓝色投放标识模型，理由是"推理 FPS 翻倍"）。圆筒模型 `cylinder_new.engine` 仍被强制加载（`vision_pipeline.py:402-415`，加载失败直接 RuntimeError），但**推理路径被关掉了**。结果：

- 投放目标源只剩蓝盘，圆筒的实测半径能力（`CylinderDetection.radius`）完全没被使用；
- 释放判据全在像素域，地面等效随高度漂移，且宽到无法保证落点在筒内；
- 5.6 m 搜索 + 2.4 m 单级降高，投放高度偏高，落体时间长，漂移项吃掉全部误差预算。

### 1.2 两条决定方案的量化结论【已核实】

`地面误差 = 像素误差 × 高度 / fx`（`precision_align_controller.py:152-170`）：

**结论 A —— 现状判据物理上够不着目标。**
`precision_align_raw_pixel_tolerance: 40`（`mission_params_quick.yaml:113`）@ 2.4 m、fx=400
⇒ 地面误差窗口 **0.24 m**，是最小筒半径 7.5 cm 的 **3.2 倍**。

**结论 B —— 只收紧判据也够不着，必须降高。**
2.4 m 自由落体 `t=√(2h/g)=0.70 s`，按实测常态释放速度 0.10~0.12 m/s，
**仅"速度 × 落体时间"一项就是 7.0~8.4 cm**，已吃完全部 7.5 cm 半径预算。

| 投放高度 | 1 px 等效 | 蓝盘 Ø1.0 m | 最小筒 Ø15 cm | 落体时间 | 漂移 @0.10~0.12 m/s |
|---|---|---|---|---|---|
| 2.4 m（现状） | 6.0 mm | 167 px | **25 px** | 0.70 s | **7.0~8.4 cm**（超预算） |
| 1.5 m | 3.75 mm | 267 px | 40 px | 0.55 s | 5.5~6.6 cm |
| **1.2 m（采用）** | 3.0 mm | 333 px | **50 px** | 0.49 s | 4.9~5.9 cm |

推论：降高后圆筒在画面里从 25 px 涨到 50 px，**"低空用圆筒精确对心"在数值上才成立**。

### 1.3 已确认的需求

| # | 决策 | 对设计的影响 |
|---|---|---|
| 1 | 现场蓝盘与圆筒**同心**（SITL 场地由 `field/generate_random_field.py` 生成，蓝盘 r=0.5 m 与筒心同心） | 追蓝盘像素中心 ≈ 追筒心 |
| 2 | 终极验收 = **落进最小筒 Ø15 cm（半径 7.5 cm）** | 决定门限与降高目标值 |
| 3 | **降投放高度纳入一期**：2.4 → 1.2~1.5 m | 启用两级降高 |
| 4 | **双通道全链路参与**：高空两模型都定位；圆筒只在高空选择目标时算半径并选最小筒；选定后两模型只做定位与微调对心、**不放弃目标**；低空蓝盘缺失时用圆筒继续精确对心 | 决定 Task 2/3/4 |

### 1.4 验收分级（比"直接要求 Ø15"诚实，需用户认可）

1.4 节的预算显示 Ø15 只剩 1.6~2.6 cm 余量，而系统偏置一期才刚开始标定。

| 阶段 | 验收 |
|---|---|
| **一期（本计划）** | 落点分布**显著收敛**；Ø20/25 稳定命中；Ø15 命中率**可测量**；系统偏置与释放速度两个量被钉住 |
| 二期 | Ø15 稳定命中（需先完成 Task 7 标定 + 偏移补偿） |

把 Ø15 作为一期硬指标会迫使收紧门限，而 §1.5 的教训正是"收紧 → 卡死"。

### 1.5 历史教训（决定判据松紧的边界）【已核实】

- `docs/Tactics/PLAN_drop_zone_align_gate.md`：**每一次收紧判据都导致空中悬停卡死**。
  9 px 窗口（8/05 架次）、`threshold_norm 0.008`、`release_max_horizontal_speed 0.04`
  （实测常态 0.10-0.12 m/s，**物理不可达**）——三项都是事故项。
- 同一份 8/05 日志证明伺服真收敛到 **2.5 px @2.0 m ≈ 1.5 cm、稳定 5/5**，
  失败发生在刹车 0.5 s 后漂到 9.3 px 越界。
  ⇒ **对心能力不是瓶颈，释放判据的鲁棒性才是。**
- `docs/Tactics/PLAN_visual_servo_optimization.md` §1.2 E3：real 策略当年最终下降用
  `goto_ned` **盲降**，2.8 m 对心完成后到达 1.5 m 时偏差 **104.9 / 162.5 px（39 / 61 cm）**。
  ⇒ **降高过程必须保持连续像素伺服。**
- `docs/投放区卡死-剩余风险抉择.md`：真机 **2.6~3.4 m 检测在视觉源端彻底消失**，
  根因嫌疑是下降 + 侧向伺服的运动模糊导致 refine 门槛（circularity ≥0.35 / blue_ratio ≥0.4）失败。
  ⇒ **圆筒通道不做蓝色验证，是真正独立的通道**——这是接入圆筒的第一价值。

### 1.6 现状盘点：让圆筒在 quick 失效的 9 个开关【已核实】

| # | 位置 | 作用 | 由哪个 Task 处理 |
|---|---|---|---|
| ① | `vision_pipeline.py:751, 822-825` | `marker_only` 下圆筒引擎不推理，`cyl_dets = []` | Task 2 |
| ② | `mission_params_quick.yaml:144` `drop_target_source: "marker"` → `mission_state_machine.py:1314-1315` 早退 | `get_known_cylinders()` 在图 `:1317` 永不可达 | Task 3 |
| ③ | `state_cylinder_search.py:45-53` | 进 CYLINDER_SEARCH 即 `set_vision_mode("marker_only")` | Task 2 |
| ④ | `state_cylinder_search.py:74-92` | quick 搜索期候选计数只数 marker | Task 3 |
| ⑤ | `mission_state_machine.py:641, 649-650` | 应急视觉喂狗：`_marker_cb` 仅在 `drop_target_source == "marker"` 时喂 | Task 3 |
| ⑥ | `mission_params_quick.yaml:93` + `precision_align_controller.py:458-463` | 精校只建 marker 候选 | Task 4 |
| ⑦ | `state_drop.py:374-414` `_servo_descend_tick` | 连续下降伺服只遍历 markers | Task 4 |
| ⑧ | `vision_pipeline.py:459-472, 554-564` `_release_detection_models()` | 一次性闩锁。**时序安全**：释放发生在 RECON_PATROL（投放之后），quick `max_drops=1` 不会再回来 | 无需处理 |
| ⑨ | `config/vision_params_quick.yaml` | 无圆筒参数覆盖，圆筒精修沿用 `vision_params.yaml:61-70` | Task 8（观察后定） |

### 1.7 可直接复用、不必新写的既有能力【已核实】

| 能力 | 位置 |
|---|---|
| 圆筒实测半径 | `CylinderDetection.radius`，发布于 `vision_pipeline.py:919`：`r_px * alt / max(fx,fy)` |
| 圆筒精修 | `cylinder_detector.py:47-167` `CylinderRefiner.refine` |
| 实时检测列表 | `mission_state_machine.py:636-650` `_cyl_cb`/`_marker_cb` ← `:1276-1288`，**无条件**刷新，只做 2 s 新鲜度 |
| 圆筒优先 + 蓝盘兜底 | `mission_state_machine.py:1307-1321` `get_known_drop_targets`——**已实现** |
| 最小半径优先 | `cylinder_selector.py:92-94` 按半径升序排序 |
| 地面误差计算 | `precision_align_controller.py:152-170` `_ground_error_m`，用**实测高度** |
| 最佳地面误差跟踪 | `state_drop.py:177-184` `_descend_best_ground_error` |
| 地面误差兜底投放 | `state_drop.py:784-796` `_ground_error_fallback_allow_drop` |
| 两级降高 | `state_drop.py:533-536` `_active_drop_height`、`:538-569` `_start_final_descent_if_needed` |
| 边伺服边下降 | `state_drop.py:190-196` `_execute_descend` 的 `_servo_descend_active` 分支 |

### 1.8 顺带发现的既有缺陷

1. **`min_radius_first` 是死参数**【已核实】：`CylinderSelector` 在 `mission_state_machine.py:166`
   **硬编码** `min_radius_first=True`；`:274` 声明的同名参数**全仓库无读取点**——
   四个 yaml 里的 `min_radius_first: true` 都不起作用。
   排序本来就在按半径升序选最小筒 ✓，但**无法通过 yaml 关掉**。
   且蓝盘的 `radius` 是硬编码 0.5（`vision_pipeline.py:775`、`mission_state_machine.py:757`），
   一旦蓝盘候选混进同一个池子，0.5 会**永远排在真实圆筒（0.075~0.125）之后**。
   **⇒ 任何半径逻辑必须只在 source 为 cylinder 时启用。**
2. **SITL 判据比真机宽 48%**【已核实】：`mission_params_quick_sim.yaml:110` 用
   `raw_pixel_tolerance: 80`（真机 40 的 2×），注释称"1280 宽 = 真机 40px 等比"；
   但等比因子应是 fx 之比 **539.9/400 = 1.35 而非 2**。
   按地面口径：SITL 80 px @2.4 m ⇒ 0.356 m，真机 ⇒ 0.240 m。
   **SITL 过检不代表真机过检。** Task 5 的地面口径会顺带消除这个不一致。
3. **cx/cy 未标定**【已核实】：`real_flight.launch.py:104-107, 122-125` 写死 320/240。
   像素对心把目标钉在**配置光心**，真实光轴偏 δpx ⇒ 落点存在**像素闭环无法消除**的系统偏移。
   @1.2 m 每 px = 3 mm：cx 偏 40 px ⇒ 落点偏 12 cm，直接推出 Ø15 筒。
4. **相机↔释放点 XY 偏移补偿完全不存在**：全仓库无 `drop_align_offset` 类参数，
   `PLAN_visual_servo_optimization.md` 的 Z4 项标为未实施。
5. `area1_altitude: 2.0` 是死参数（`mission_params_quick.yaml:47`）：全仓库仅声明、无读取点。

---

## 2. 文件结构

| 文件 | 职责 | 本计划中的改动 |
|---|---|---|
| `src/drone_mission/drone_mission/precision_align_controller.py` | 低空精对心控制器：选目标、算误差、判释放 | Task 4 新增 `"fused"` 通道；Task 5 新增地面误差门 |
| `src/drone_mission/drone_mission/mission_state_machine.py` | FSM：参数声明、目标库、视觉模式、投放目标选择 | Task 2 新参数；Task 3 目标源分支 + 喂狗 + 参数声明 |
| `src/drone_mission/drone_mission/states/state_cylinder_search.py` | 投放区搜索状态 | Task 2 模式参数化；Task 3 计数同源 |
| `src/drone_mission/drone_mission/states/state_drop.py` | 降高/对心/投放状态 | Task 4 伺服目标源；Task 6 RELEASE 日志 |
| `src/drone_mission/config/mission_params_quick.yaml` | 真机 quick 参数 | Task 1/3/5/6 |
| `src/drone_mission/config/mission_params_quick_sim.yaml` | SITL quick 参数 | Task 1/3/5/6 |
| `src/drone_mission/test/test_precision_align_controller.py` | 控制器单测 | Task 4/5 |
| `src/drone_mission/test/test_quick_drop_strategy.py` | quick 策略契约测试 | Task 2/3/5/6 |
| `src/drone_mission/test/test_layout_mission_config.py` | 布局配置契约 | Task 3 |
| `src/drone_bringup/test/test_quick_launch_configuration.py` | 启动配置契约 | Task 3 |
| `src/drone_bringup/launch/real_flight.launch.py` | 真机内参 | Task 7 |

---

## 3. 实施顺序与依赖

```
Task 1 (释放速度, 纯 YAML)  ──► 先飞一次拿基线, 其结果决定 Task 6 的最终高度
Task 2 (视觉模式参数化)  ──► Task 3 (目标源+计数同源)  ──► Task 4 (双通道融合)  ──► Task 5 (地面误差门)  ──► Task 6 (两级降高)
                                                                                          │
                                                              Task 7 (相机标定) ◄─────────┘
                                                                          └──► Task 8 (分层验证)
```

**Task 1 排在最前**：它与视觉无关、纯配置、能独立验证，且**实测释放速度直接决定 Task 6 的高度取 1.2 还是 1.5**。

---

## Task 1: 压释放水平速度与延长刹车（WP6）

**为什么先做**：按 §1.2 的误差预算，释放瞬间水平速度**直接乘自由落体时间**，是三项误差里最大的一项，且与视觉改造完全解耦。

**Files:**
- Modify: `src/drone_mission/config/mission_params_quick.yaml:118-119`
- Modify: `src/drone_mission/config/mission_params_quick_sim.yaml`（对应 `final_brake_s` / `release_max_horizontal_speed` 两行）

**Interfaces:**
- Consumes: `precision_align_release_max_horizontal_speed`、`precision_align_final_brake_s`，读取点 `precision_align_controller.py:422`（`_horizontal_speed_ok`）与 `:293-297`（刹车）
- Produces: 投放时刻 `speed_xy` 的实测分布 → 决定 Task 6 的 `precision_align_final_drop_height`

**门限取值的算术约束**（这一步不能凭感觉定）：门限是**最坏情况**上限，不是典型值。1.2 m 落体 0.49 s：

| 门限 | 门限处漂移 = 门限 × 0.49 s | 对 7.5 cm 预算 |
|---|---|---|
| 0.25（现值） | 12.3 cm | 爆 |
| 0.15 | 7.4 cm | 几乎吃满，对心残差与系统偏置无处安放 |
| **0.12（采用）** | **5.9 cm** | 余 1.6 cm ✓ |

- [ ] **Step 1: 修改真机 quick 参数**

`src/drone_mission/config/mission_params_quick.yaml`：

```yaml
    precision_align_final_brake_s: 0.45   # quick: 投放前零速度悬停复查 (原 0.3, 给刹车留时间把速度压到门限下)
    precision_align_release_max_horizontal_speed: 0.12 # quick: 释放时最大水平速度 (原 0.25)
                                                       # 0.12 × 0.49s(1.2m落体) = 5.9cm, 留在 7.5cm 半径预算内
```

- [ ] **Step 2: 同步 SITL quick 参数**

`src/drone_mission/config/mission_params_quick_sim.yaml`：把同名两项改为同样的 `0.45` / `0.12`。

> 注意：SITL 与真机的高度一致（都是 2.4 m），落体时间相同，所以速度门限可以共用同一个值；**不需要**按 fx 比例换算（fx 只影响像素口径，不影响速度门限）。

- [ ] **Step 3: 跑契约测试确认没有回归**

Run: `python3 -m pytest src/drone_mission/test/test_quick_drop_strategy.py -q`
Expected: PASS（若有断言直接钉死这两个数值，见 Task 6 Step 5 一并更新）

- [ ] **Step 4: 真机飞一个架次记录基线**

观测日志中投放时刻的 `PREC_ALIGN: 等待水平速度归零 speed_xy=... max=...`（`precision_align_controller.py:435-438`）与释放时刻的 `speed_xy`。

记录：
- 进入 `_horizontal_speed_ok` 等待的次数（若每架次 >3 次，说明 0.12 偏紧）；
- 实际释放时的 `speed_xy` 分布。

- [ ] **Step 5: 提交**

```bash
git add src/drone_mission/config/mission_params_quick.yaml src/drone_mission/config/mission_params_quick_sim.yaml
git commit -m "tune(quick): 释放速度门 0.25→0.12, 刹车 0.3→0.45s"
```

**决策点**：若 Step 4 显示拦停频繁（每架次 >3 次且耗时 >2 s），放宽到 `0.15` 并接受
"Ø15 命中率下降、Ø20/25 仍稳"——这是可解释的降级，不是失败。

---

## Task 2: 视觉模式参数化，quick 恢复双模型（WP1）

**Files:**
- Modify: `src/drone_mission/drone_mission/mission_state_machine.py`（新增参数声明）
- Modify: `src/drone_mission/drone_mission/states/state_cylinder_search.py:45-53`
- Modify: `src/drone_mission/config/mission_params_quick.yaml`、`mission_params_quick_sim.yaml`
- Test: `src/drone_mission/test/test_quick_drop_strategy.py:201-221`

**Interfaces:**
- Consumes: `mission_state_machine.py:1120-1156` 的 `set_vision_mode(mode: str) -> None`；`vision_pipeline.py:441-494` 的 `_set_mode_cb`
- Produces: 新参数 `quick_search_vision_mode`（字符串，默认 `"marker_only"`，`"all"` 启用双模型）

**背景**【已核实】：`vision_pipeline.py:459-472` 中 `mode in (0, 3)` 与 `mode == 6` 都会检查
`_detection_models_released` 闩锁；该闩锁只在 mode 0/2/4 里置位。
quick 任务在搜索前**没有任何** `set_vision_mode` 调用，所以搜索期模型未释放，mode 3 可用。

- [ ] **Step 1: 写失败测试**

在 `src/drone_mission/test/test_quick_drop_strategy.py` 中，紧跟现有的
`test_quick_search_enters_requests_marker_only_vision`（`:201-221`）之后新增。
**必须复用该用例已有的 fsm stub 搭法**（`_SearchFSM` + 临时挂 `logger`/`fc`/`layout_frame`/
`zone_bounds`/`clear_known_cylinders`，再 `CylinderSearchState(fsm)` + `state.enter()`）——
视觉模式切换发生在 `enter()` 里，不是 `__init__`：

```python
def test_quick_search_vision_mode_follows_param():
    """quick 搜索期视觉模式由 quick_search_vision_mode 决定."""
    fsm = _SearchFSM({
        "quick_drop_mode": True,
        "min_drop_targets_required": 1,
        "drop_marker_fallback_delay": 0.0,
        "quick_search_vision_mode": "all",
    })
    fsm.logger = type("TL", (), {"log_state": staticmethod(lambda *a: None)})()
    fsm.fc = type("FC", (), {
        "goto_ned": staticmethod(lambda *a, **kw: None),
        "altitude_to_ned": staticmethod(lambda a: -a),
    })()
    fsm.clear_known_cylinders = lambda: None
    state = CylinderSearchState(fsm)
    state.enter()

    assert fsm.set_vision_calls == ["all"]


def test_quick_search_vision_mode_defaults_to_marker_only():
    """未配置该参数时保持旧行为."""
    fsm = _SearchFSM({
        "quick_drop_mode": True,
        "min_drop_targets_required": 1,
        "drop_marker_fallback_delay": 0.0,
    })
    fsm.logger = type("TL", (), {"log_state": staticmethod(lambda *a: None)})()
    fsm.fc = type("FC", (), {
        "goto_ned": staticmethod(lambda *a, **kw: None),
        "altitude_to_ned": staticmethod(lambda a: -a),
    })()
    fsm.clear_known_cylinders = lambda: None
    state = CylinderSearchState(fsm)
    state.enter()

    assert fsm.set_vision_calls == ["marker_only"]
```

> `_SearchFSM(params: dict)`（`:126`）**接收显式参数字典，不读 yaml**——
> 所以第二条用例天然钉住"代码默认值"，与 yaml 改成 `"all"` 互不干扰。

- [ ] **Step 2: 跑测试确认失败**

Run: `python3 -m pytest src/drone_mission/test/test_quick_drop_strategy.py::test_quick_search_vision_mode_follows_param -v`
Expected: FAIL —— `assert [] == ['all']`（参数尚未接线）

- [ ] **Step 3: 声明新参数**

`src/drone_mission/drone_mission/mission_state_machine.py`，在 `quick_drop_mode` / `drop_target_source` 附近（`:274` 一带的投放参数块，与 `min_drop_targets_required` 等并列）新增：

```python
        self.declare_parameter("quick_search_vision_mode", "marker_only")
```

- [ ] **Step 4: 让搜索状态读参数**

`src/drone_mission/drone_mission/states/state_cylinder_search.py:45-53`，把：

```python
        if self._fsm.get_param("quick_drop_mode", False):
            # quick 模式: 视觉只跑蓝底识别模型 (mode 6 marker_only)
            # 服务不可用会抛异常并被 _transition_to 升级为 takeover, 必须兜住
            try:
                self._fsm.set_vision_mode("marker_only")
            except Exception as exc:
                self._fsm.get_logger().warn(
                    f"CYL_SEARCH: 切换到 marker_only 失败, 继续用默认检测模式: {exc}"
                )
```

改为：

```python
        if self._fsm.get_param("quick_drop_mode", False):
            # quick 模式视觉档位由参数决定: "all" = 圆筒+蓝底双模型 (双通道定位),
            # "marker_only" = 只跑蓝底 (旧行为). 默认值保持 marker_only, 零漂移.
            # 服务不可用会抛异常并被 _transition_to 升级为 takeover, 必须兜住
            vision_mode = str(
                self._fsm.get_param("quick_search_vision_mode", "marker_only")
            )
            if vision_mode:
                try:
                    self._fsm.set_vision_mode(vision_mode)
                except Exception as exc:
                    self._fsm.get_logger().warn(
                        f"CYL_SEARCH: 切换到 {vision_mode} 失败, 继续用默认检测模式: {exc}"
                    )
```

- [ ] **Step 5: 跑测试确认通过**

Run: `python3 -m pytest src/drone_mission/test/test_quick_drop_strategy.py -q`
Expected: PASS（含既有 `:201-221` 用例）

- [ ] **Step 6: 打开 quick 的双模型**

`mission_params_quick.yaml` 与 `mission_params_quick_sim.yaml` 的 quick 参数块新增：

```yaml
    quick_search_vision_mode: "all"       # 圆筒+蓝底双模型; "marker_only" 回退单模型
```

- [ ] **Step 7: 确认既有用例无需改动**

**结论：`test_quick_drop_strategy.py:201-221` 不需要改。**
该用例用 `_SearchFSM({...})` 传入**显式参数字典**，其中不含 `quick_search_vision_mode`，
走的是代码默认值 `"marker_only"`，因此 `assert fsm.set_vision_calls == ["marker_only"]`
**在 yaml 改成 `"all"` 之后依然成立**。

真正读 yaml 的只有 `test_quick_real_params_enable_strategy`（`:345`）与
`test_quick_sim_params_enable_strategy_with_scaled_pixel_windows`（`:360`），
它们断言的是 yaml 内容——Step 6 之后如需补断言，在此追加：

```python
    assert p["quick_search_vision_mode"] == "all"
```

Run: `python3 -m pytest src/drone_mission/test/test_quick_drop_strategy.py -q`
Expected: PASS

- [ ] **Step 8: SITL 实测 FPS 代价**

Run: `./scripts/start_quick_simulation.sh`

记录搜索段与投放段的视觉推理 FPS（`/drone/vision/status` 或节点日志）。

**决策点**：若搜索段 FPS 下降到影响 `min_drop_targets_required` 的达成速度，
退路是新增一个视觉档（搜索期 marker_only、选择完成后再切 all）——
`vision_pipeline.py:441-494` 的 `_set_mode_cb` 中 **mode 1 未被任何分支处理**
（会落到 `else` 返回"不支持的视觉模式"），是现成的空档位；
`mission_state_machine.py:1131-1133` 的常量表同样没有 mode 1，需两侧同时新增。
**一期先不实现**，等 Step 8 的实测数据说话。

- [ ] **Step 9: 提交**

```bash
git add src/drone_mission/drone_mission/mission_state_machine.py \
        src/drone_mission/drone_mission/states/state_cylinder_search.py \
        src/drone_mission/config/mission_params_quick.yaml \
        src/drone_mission/config/mission_params_quick_sim.yaml \
        src/drone_mission/test/test_quick_drop_strategy.py
git commit -m "feat(quick): 视觉模式参数化, quick 搜索期恢复圆筒+蓝底双模型"
```

---

## Task 3: 投放目标源切到圆筒优先，并让搜索计数同源（WP2）

**Files:**
- Modify: `src/drone_mission/config/mission_params_quick.yaml:144`、`mission_params_quick_sim.yaml`
- Modify: `src/drone_mission/drone_mission/states/state_cylinder_search.py:74-92`
- Modify: `src/drone_mission/drone_mission/mission_state_machine.py:649-650`
- Test: `test_quick_drop_strategy.py:91-119`、`test_layout_mission_config.py:46,56`、`test_quick_launch_configuration.py:113`

**Interfaces:**
- Consumes: `mission_state_machine.py:1307-1321` `get_known_drop_targets()`
- Produces: quick 的 `drop_target_source = "cylinder"`；搜索期 `known` 与选择期同源

**关键事实**【已核实】：`get_known_drop_targets` 的**默认分支已经实现**了
"有圆筒用圆筒、没有才回落蓝盘"：

```python
if self.get_param("drop_target_source", "cylinder") == "marker":
    return self._dedupe_markers(self.get_known_markers())
cylinders = self.get_known_cylinders()
if cylinders:
    return cylinders
return self._dedupe_markers(self.get_known_markers())
```

⇒ 目标源改成 `"cylinder"` 后**不需要改这段代码**。

- [ ] **Step 1: 给测试 stub 补上目标库方法**

**先做这一步，否则改动会被 stub 静默吞掉。**
`_SearchFSM`（`test_quick_drop_strategy.py:126`）目前只有 `get_known_cylinders` /
`get_known_markers`，**没有 `get_known_drop_targets`**。
Step 4 的新代码用 `hasattr(self._fsm, "get_known_drop_targets")` 做守卫，
在 stub 上会是 `False` → 分支落到 `else`，测试**照样通过但什么都没验证**。

在 `_SearchFSM` 中补一个与真实实现（`mission_state_machine.py:1307-1321`）同构的方法：

```python
    def get_known_drop_targets(self):
        if self._params.get("drop_target_source", "cylinder") == "marker":
            return self._known_markers
        return self._known_cylinders or self._known_markers
```

- [ ] **Step 2: 写失败测试（计数同源）**

在现有的 `test_quick_search_counts_only_markers_ignoring_cylinder_memory`（`:185`）
之后新增，复用该用例的驱动方式（`_search_state(fsm)` + `state.execute()` 循环）：

```python
def test_quick_search_counts_cylinders_when_source_is_cylinder():
    """搜索期候选计数必须与 SELECT_TARGET 同源, 否则会反复重搜."""
    fsm = _SearchFSM({
        "quick_drop_mode": True,
        "min_drop_targets_required": 1,
        "drop_marker_fallback_delay": 0.0,
        "drop_target_source": "cylinder",
    })
    fsm._known_markers = []          # 蓝盘为空, 只剩 1 个圆筒候选
    state = _search_state(fsm)

    result = None
    for _ in range(5):
        result = state.execute()

    assert result == "SELECT_TARGET"


def test_quick_search_falls_back_to_markers_when_no_cylinder():
    """圆筒库为空时仍能用蓝盘候选, 与 get_known_drop_targets 的兜底一致."""
    fsm = _SearchFSM({
        "quick_drop_mode": True,
        "min_drop_targets_required": 1,
        "drop_marker_fallback_delay": 0.0,
        "drop_target_source": "cylinder",
    })
    fsm._known_cylinders = []        # 无圆筒 → 回落蓝盘
    state = _search_state(fsm)

    result = None
    for _ in range(5):
        result = state.execute()

    assert result == "SELECT_TARGET"
```

- [ ] **Step 3: 跑测试确认失败（至少第一条）**

Run: `python3 -m pytest src/drone_mission/test/test_quick_drop_strategy.py -k counts_cylinders -v`
Expected: 在 Step 1 补了 stub 方法、但 Step 5 尚未改计数逻辑时，
quick 分支只数 `get_known_markers()`（此时为空）→ 候选数 0 → **FAIL**（不会进 SELECT_TARGET）

- [ ] **Step 4: 搜索计数改为同源**

`src/drone_mission/drone_mission/states/state_cylinder_search.py:74-92`，把：

```python
        if (
            self._fsm.get_param("quick_drop_mode", False)
            and hasattr(self._fsm, "get_known_markers")
        ):
            # quick 模式只认蓝底标识; mode 6 切换是异步服务调用,
            # 生效窗口内圆筒可能仍入记忆, 此处显式排除只数 marker
            known = self._fsm.get_known_markers()
```

改为：

```python
        if (
            self._fsm.get_param("quick_drop_mode", False)
            and hasattr(self._fsm, "get_known_drop_targets")
        ):
            # quick 模式计数必须与 SELECT_TARGET 同源 (get_known_drop_targets 已实现
            # "有圆筒用圆筒、没有才回落蓝盘"), 否则会出现
            # "搜索判定候选足够 → 选目标时却选不出" 的口径裂缝, 表现为反复重搜.
            known = self._fsm.get_known_drop_targets()
```

- [ ] **Step 5: 跑测试确认通过**

Run: `python3 -m pytest src/drone_mission/test/test_quick_drop_strategy.py -q`
Expected: PASS

> **注意既有用例 `test_quick_search_counts_only_markers_ignoring_cylinder_memory`（`:185`）**：
> 改动后它的结论仍成立（stub 里圆筒与蓝盘各有 1 个，两种口径都得到候选数 1 ≥ 1），
> 但**它的用例名与注释已经与新行为不符**（"只数 marker" 不再为真）。
> 请把该用例改名为 `test_quick_search_counts_drop_targets_source` 并更新注释，
> 否则它会变成一条"名字在说谎"的测试。

- [ ] **Step 6: 切换投放目标源**

`mission_params_quick.yaml:144` 与 `mission_params_quick_sim.yaml` 对应行：

```yaml
    drop_target_source: "cylinder"        # 圆筒优先, 无圆筒才回落蓝底 (get_known_drop_targets 默认分支)
```

- [ ] **Step 7: 修应急视觉喂狗**

`mission_state_machine.py:649-650` 现状：

```python
        # marker_only 模式下 cylinders 恒空, 应急视觉信号必须由 marker 喂给
        if self.get_param("drop_target_source", "cylinder") == "marker":
            self._emergency.update_vision(len(self._markers) > 0)
```

方式改为 **两通道任一有检测即算有视觉**（`_cyl_cb:641` 已经无条件喂过一次，
所以这里只需把 marker 也纳入，避免"圆筒漏检而蓝盘可见"时误报视觉丢失）：

```python
        # 双通道任一有检测即视为视觉可用; 目标源切换不应让另一路失去喂狗能力
        if self.get_param("drop_target_source", "cylinder") == "marker":
            self._emergency.update_vision(len(self._markers) > 0)
        elif self._markers:
            self._emergency.update_vision(True)
```

- [ ] **Step 8: 更新受影响的契约测试**

| 文件:行 | 改动 |
|---|---|
| `test_quick_drop_strategy.py:349`、`:364` | quick profile 的 `drop_target_source` 期望值 `"marker"` → `"cylinder"` |
| `test_quick_drop_strategy.py:325` | **AST 默认值契约**，断言的是**代码默认值** `"cylinder"`，`mission_state_machine.py` 默认不动 → **保持** |
| `test_quick_drop_strategy.py:91-119` | marker 门语义用例把 `drop_target_source` 显式设为 `"marker"` 后断言只返回蓝盘 → **保持不变，必须继续绿** |
| `test_layout_mission_config.py:46`、`:56` | 两个 quick profile 的 `"drop_target_source": "marker"` → `"cylinder"` |
| `test_quick_launch_configuration.py:113` | `assert quick["drop_target_source"] == "marker"` → `"cylinder"` |

Run: `python3 -m pytest src/drone_mission/test src/drone_bringup/test -q`
Expected: PASS

- [ ] **Step 9: 提交**

```bash
git add src/drone_mission/config/mission_params_quick.yaml \
        src/drone_mission/config/mission_params_quick_sim.yaml \
        src/drone_mission/drone_mission/states/state_cylinder_search.py \
        src/drone_mission/drone_mission/mission_state_machine.py \
        src/drone_mission/test src/drone_bringup/test
git commit -m "feat(quick): 投放目标源切圆筒优先, 搜索计数与选择同源"
```

---

## Task 4: 双通道融合对心（WP3）

**Files:**
- Modify: `src/drone_mission/drone_mission/precision_align_controller.py:443-524`（`_choose_target`）
- Modify: `src/drone_mission/drone_mission/states/state_drop.py:374-414`（`_servo_descend_tick`）
- Modify: `src/drone_mission/drone_mission/mission_state_machine.py`（参数声明）
- Modify: quick 两个 yaml（`precision_align_target_source`）
- Test: `src/drone_mission/test/test_precision_align_controller.py`

**Interfaces:**
- Consumes: `_choose_target(self, markers: list, cylinders: list)`（`:443`）；`_ground_error_m(du, dv, altitude_m) -> Tuple[float, float]`（`:152-170`）；`_field(obj, name, default)`；`_world_xy(obj)`
- Produces: `precision_align_target_source` 新增合法值 `"fused"`；新参数 `precision_align_fuse_cylinder_below_m`、`precision_align_channel_switch_max_error_m`

**约束**【已核实】：`test_precision_align_controller.py:232-258` 断言 `"cylinder"` 模式在只有 marker 时
返回 `no_candidates` 且不发指令。**不得修改 `"marker"` / `"cylinder"` 的既有语义**，新增独立分支。

**主/副按高度调度**：高空蓝盘像素量压倒性（2.4 m 处 167 px vs 25 px），低空圆筒才追平
（1.2 m 处 50 px vs 333 px）。所以不是"圆筒永远优先"。

**防串筒**：既有跳变门 `precision_align_max_jump_diag_ratio = 0.55` 是**图像对角线比例**——
真机 800 px 对角线 ×0.55 = **440 px**，宽到挡不住"切到旁边另一只筒"。
quick 投放区有 Ø15/20/25 三只筒，低空切错筒 = 投错目标，必须单独加门。

**两通道不做数值平均**：marker refiner 与 cylinder refiner 的系统偏置不同，平均会引入未知偏置。

- [ ] **Step 1: 写失败测试**

`src/drone_mission/test/test_precision_align_controller.py` 新增（沿用文件中已有的 helper，见下方说明）：

**必须复用该文件已有的 helper**（已核实）：
`_Fsm(params: dict)`（`:63`，构造时用 dict 覆盖默认参数）、
`PrecisionAlignController(fsm)` + `controller.reset()`（见 `:106-108` 的既有用例）、
`_window_target(center_u=640.0, center_v=480.0)`（`:451`，**返回一个 list**，
且**不接受 `confidence` / `drop_height` 参数**——置信度固定 0.9）。
`_Fsm` 默认图像为 1280×960、cx=640、cy=480，所以"画面中心"是 640/480。

```python
def test_fused_prefers_cylinder_below_threshold_altitude():
    """低于 fuse_cylinder_below_m 时主通道=圆筒: 蓝盘存在也应选圆筒."""
    fsm = _Fsm({
        "precision_align_target_source": "fused",
        "precision_align_fuse_cylinder_below_m": 1.8,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset()

    controller.step(
        markers=_window_target(center_u=640.0),
        cylinders=_window_target(center_u=640.0 + 22.0),
        drop_height=1.2, altitude_m=1.2,
    )
    assert controller.last_status.source == "cylinder"


def test_fused_prefers_marker_above_threshold_altitude():
    """高于阈值时主通道=蓝盘."""
    fsm = _Fsm({
        "precision_align_target_source": "fused",
        "precision_align_fuse_cylinder_below_m": 1.8,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset()

    controller.step(
        markers=_window_target(center_u=640.0 + 22.0),
        cylinders=_window_target(center_u=640.0),
        drop_height=2.4, altitude_m=2.4,
    )
    assert controller.last_status.source == "marker"


def test_fused_takes_over_when_primary_channel_missing():
    """主通道无目标时副通道立即接管, 不计 lost — 低空蓝盘缺失的兜底."""
    fsm = _Fsm({
        "precision_align_target_source": "fused",
        "precision_align_fuse_cylinder_below_m": 1.8,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset()

    result = controller.step(
        markers=[],
        cylinders=_window_target(center_u=640.0),
        drop_height=1.2, altitude_m=1.2,
    )
    assert result.value == "continue"
    assert controller.last_status.source == "cylinder"


def test_fused_rejects_switch_to_far_cylinder():
    """副通道候选相对锁定像素地面误差过大 → 拒绝, 防低空串到旁边另一只筒."""
    fsm = _Fsm({
        "precision_align_target_source": "fused",
        "precision_align_fuse_cylinder_below_m": 1.8,
        "precision_align_channel_switch_max_error_m": 0.25,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset()

    controller.step(
        markers=[], cylinders=_window_target(center_u=640.0),
        drop_height=1.2, altitude_m=1.2,
    )
    locked_u = controller._locked_pixel_uv[0]

    # 200px @ 1.2m / fx400 = 0.6m 地面误差, 远超 0.25 门限
    controller.step(
        markers=[], cylinders=_window_target(center_u=640.0 + 200.0),
        drop_height=1.2, altitude_m=1.2,
    )
    # 仍锁定原目标, 不接受远处的筒
    assert controller._locked_pixel_uv[0] == locked_u
```

- [ ] **Step 2: 跑测试确认失败**

Run: `python3 -m pytest src/drone_mission/test/test_precision_align_controller.py -k fused -v`
Expected: FAIL —— `"fused"` 现在落到 `else` 分支（`marker_candidates + cylinder_candidates`），
`source` 与预期不符；且 `fuse_cylinder_below_m` 参数未被识别。

- [ ] **Step 3: 声明新参数**

`mission_state_machine.py` 的 `precision_align_target_source`（`:296`）附近新增：

```python
        self.declare_parameter("precision_align_fuse_cylinder_below_m", 0.0)
        self.declare_parameter("precision_align_channel_switch_max_error_m", 0.0)
```

> `0.0 = 关闭` 是本仓库既有约定，保证默认行为等于现状。

- [ ] **Step 4: 实现 fused 通道选择**

`precision_align_controller.py:458-463`，把：

```python
        if source_preference == "marker":
            candidates = marker_candidates
        elif source_preference == "cylinder":
            candidates = cylinder_candidates
        else:
            candidates = marker_candidates + cylinder_candidates
```

改为：

```python
        if source_preference == "marker":
            candidates = marker_candidates
        elif source_preference == "cylinder":
            candidates = cylinder_candidates
        elif source_preference == "fused":
            candidates = self._fused_candidates(
                marker_candidates, cylinder_candidates, altitude_m,
            )
        else:
            candidates = marker_candidates + cylinder_candidates
```

并在 `_choose_target` 签名处增加带回退默认值的关键字参数（**测试全部经由 `step()` 驱动，
已确认无直接调用者**，加默认值零影响）：

```python
    def _choose_target(self, markers: list, cylinders: list, altitude_m: float = 0.0):
```

`step()` 内的调用点（`precision_align_controller.py:209` 附近）改为：

```python
        target, source, reason = self._choose_target(markers, cylinders, altitude)
```

新增方法（放在 `_choose_target` 之前）：

```python
    def _fused_candidates(self, marker_candidates, cylinder_candidates, altitude_m):
        """按高度调度主/副通道, 主通道无候选时副通道立即接管 (不计 lost).

        高空蓝盘像素量远大于圆筒 (2.4m: 167px vs 25px), 低空圆筒才追平
        (1.2m: 50px vs 333px), 所以主通道随高度切换而非固定圆筒优先.
        """
        fuse_below = float(self._param("precision_align_fuse_cylinder_below_m", 0.0))
        if fuse_below <= 0.0:
            # 未启用高度调度: 回退到"合并候选池"的旧行为
            return list(marker_candidates) + list(cylinder_candidates)
        if float(altitude_m) <= fuse_below:
            primary, secondary = cylinder_candidates, marker_candidates
        else:
            primary, secondary = marker_candidates, cylinder_candidates
        return list(primary) or list(secondary)
```

- [ ] **Step 5: 实现防串筒门**

在 `_choose_target` 的候选过滤循环中（`jump_gate` / `center_reject` 判定所在的 `:469-483` 一带），
对**副通道刚接管的候选**追加地面误差门。由于该循环现已同时处理两种来源，最小改动是在
选定 `selected` 之前加一次校验——在 `scored` 生成后、`min(scored, ...)` 之前插入：

```python
        switch_gate = float(
            self._param("precision_align_channel_switch_max_error_m", 0.0)
        )
        if switch_gate > 0.0 and self._locked_pixel_uv is not None and altitude > 0.05:
            cx_, cy_, width_, height_ = self._camera_geometry()
            locked_u, locked_v = self._locked_pixel_uv
            gated = []
            for score, target, source in scored:
                u = float(self._field(target, "center_u", cx_))
                v = float(self._field(target, "center_v", cy_))
                g_fwd, g_right = self._ground_error_m(
                    (u - locked_u), (v - locked_v), altitude,
                )
                if math.hypot(g_fwd, g_right) <= switch_gate:
                    gated.append((score, target, source))
            if gated:
                scored = gated
```

> **注意**：`_ground_error_m` 内部对 `du`/`dv` 是按**像素**单位处理的
> （它自己做 `/fx`、`/fy`），所以这里传的是像素差 `(u - locked_u)`，
> **不要**再乘 `width`/`height` —— 与 `step()` 中 `smooth_du = smooth_dx * width` 的
> 用法一致（那里传的也是像素）。`altitude` 用 `step()` 里已算好的局部变量。

- [ ] **Step 6: 跑测试确认通过**

Run: `python3 -m pytest src/drone_mission/test/test_precision_align_controller.py -q`
Expected: PASS，特别是 `:232-258` 的 `test_cylinder_precision_does_not_fallback_to_marker` **必须仍是绿的**。

- [ ] **Step 7: 下降伺服同步目标源**

`state_drop.py:374-414` `_servo_descend_tick` 目前**只遍历 `get_markers()`**（已核实）。
不同步改的话，下降伺服与精校会追不同的目标。

在方法开头读取目标源，并把候选集合按同一规则取：

```python
        source_pref = str(
            self._fsm.get_param("precision_align_target_source", "cylinder")
        ).lower()
        if source_pref == "fused":
            candidates = list(
                self._fsm.get_cylinders() if hasattr(self._fsm, "get_cylinders") else []
            ) or list(markers)
        else:
            candidates = list(markers)
```

然后遍历 `candidates` 而非 `markers`。

- [ ] **Step 8: 打开 quick 的融合通道**

`mission_params_quick.yaml:93`：

```yaml
    precision_align_target_source: "fused"  # 双通道: 低空圆筒优先, 高空蓝盘优先, 失效互相接管
    precision_align_fuse_cylinder_below_m: 1.8  # 低于此高度主通道=圆筒 (1.2m 处圆筒 50px)
    precision_align_channel_switch_max_error_m: 0.25  # 防低空串到旁边另一只筒
```

`mission_params_quick_sim.yaml` 同步（数值相同；这是**米制**参数，不需按 fx 换算）。

- [ ] **Step 9: 提交**

```bash
git add src/drone_mission/drone_mission/precision_align_controller.py \
        src/drone_mission/drone_mission/states/state_drop.py \
        src/drone_mission/drone_mission/mission_state_machine.py \
        src/drone_mission/config/mission_params_quick.yaml \
        src/drone_mission/config/mission_params_quick_sim.yaml \
        src/drone_mission/test/test_precision_align_controller.py
git commit -m "feat(quick): 精对心新增 fused 双通道, 按高度调度主副并防串筒"
```

---

## Task 5: 释放判据改成地面误差口径（WP4，核心）

**Files:**
- Modify: `src/drone_mission/drone_mission/precision_align_controller.py:256-263` 与 `_is_precise`（`:378-394`）
- Modify: `src/drone_mission/drone_mission/mission_state_machine.py`（参数声明）
- Modify: quick 两个 yaml
- Test: `src/drone_mission/test/test_precision_align_controller.py`

**Interfaces:**
- Consumes: `_ground_error_m(du, dv, altitude_m) -> Tuple[float, float]`；`step()` 内的局部变量 `altitude`、`continuous`、`raw_dx`、`raw_dy`、`width`、`height`
- Produces: 新参数 `precision_align_release_max_ground_error_m`（m，`0.0` = 禁用）

**动机**：现有判据全在像素域，地面等效随高度漂移，SITL 与真机口径还不一致（§1.8 第 2 条）。
改成物理口径后，判据在 SITL / 真机 / 不同高度之间自动一致。

**门限取 0.03 而不是 0.05 —— 预算算术**：7.5 cm 半径预算要同时装下三项：

| 项 | 量级 | 来源 |
|---|---|---|
| 释放漂移（最坏 0.12 m/s × 0.49 s） | 5.9 cm | Task 1 |
| 对心残差（= 本门限） | 3.0 cm | 本 Task |
| 相机↔释放点 + cx/cy 系统偏置 | 待标定 | Task 7 |

按 RMS 合成 √(4.9² + 3.0²) ≈ 5.7 cm（典型速度 0.10 m/s），再叠 1 cm 系统偏置 → **6.7 cm < 7.5 cm ✓**。
取 0.05 则 RMS 已 7.0 cm，加任何偏置都爆。
⇒ **0.03 是"系统偏置压到 1 cm 以内"这个前提下的最大可行值**，
这也是 Task 7 从"可选优化"升级为"一期必需"的算术原因。

- [ ] **Step 1: 写失败测试**

**断言选择说明**：不要断言 `release_ready`——它还要跨过 `stable_frames`（`_Fsm` 默认 8）与
刹车复查，单步断言必然失败。**要断言 `controller._stable_count`**：
`step()` 里 `self._stable_count = self._stable_count + 1 if aligned else 0`，
它精确隔离 `aligned` 的取值，正是本 Task 要验证的量。

**参数隔离说明**：`_Fsm` 默认 `raw_pixel_tolerance=6.0`、`threshold_norm_x/y=0.006`——
39 px 的偏移会先被**像素门**挡掉，测不到地面门。所以测试里要把像素门放宽，
让"像素达标、地面不达标"这个唯一场景成立。

```python
_PIXEL_LOOSE = {
    "precision_align_raw_pixel_tolerance": 40.0,
    "precision_align_smooth_pixel_tolerance": 40.0,
    "precision_align_threshold_norm_x": 0.10,
    "precision_align_threshold_norm_y": 0.10,
}


def test_ground_error_gate_blocks_pixel_aligned_target():
    """像素达标但地面误差超标 → 不计入稳定帧 (不得释放)."""
    fsm = _Fsm({
        **_PIXEL_LOOSE,
        "precision_align_release_max_ground_error_m": 0.03,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset()

    # 39px 通过像素门(39<=40), 但 39*2.4/400 = 0.234m 远超 0.03m 地面门
    controller.step(
        markers=[], cylinders=_window_target(center_u=640.0 + 39.0),
        drop_height=2.4, altitude_m=2.4,
    )
    assert controller._stable_count == 0


def test_ground_error_gate_allows_when_within_budget():
    """地面误差在门限内 → 正常计入稳定帧."""
    fsm = _Fsm({
        **_PIXEL_LOOSE,
        "precision_align_release_max_ground_error_m": 0.03,
    })
    controller = PrecisionAlignController(fsm)
    controller.reset()

    # 3px @ 1.2m / fx400 = 0.009m, 在 0.03m 门限内
    controller.step(
        markers=[], cylinders=_window_target(center_u=640.0 + 3.0),
        drop_height=1.2, altitude_m=1.2,
    )
    assert controller._stable_count == 1


def test_ground_error_gate_disabled_by_default():
    """默认 0.0 = 禁用: 同样的 39px@2.4m 在未配置时应当通过."""
    fsm = _Fsm(_PIXEL_LOOSE)
    controller = PrecisionAlignController(fsm)
    controller.reset()

    controller.step(
        markers=[], cylinders=_window_target(center_u=640.0 + 39.0),
        drop_height=2.4, altitude_m=2.4,
    )
    assert controller._stable_count == 1
```

> 第三条是"默认值不改行为"的反向对照：与第一条**只差一个参数**，
> 结果却从 0 变 1——这直接证明了新门限是唯一变量。

- [ ] **Step 2: 跑测试确认失败**

Run: `python3 -m pytest src/drone_mission/test/test_precision_align_controller.py -k ground_error_gate -v`
Expected: `test_ground_error_gate_blocks_pixel_aligned_target` **FAIL** ——
`assert 1 == 0`（当前只看像素，39 px 通过了 raw_tol=40，`_stable_count` 变成 1）。
另外两条应当已经 PASS（第二条本来就通过，第三条是既有行为）。

- [ ] **Step 3: 声明新参数**

`mission_state_machine.py` 的 `precision_align_target_source`（`:296`）附近：

```python
        self.declare_parameter("precision_align_release_max_ground_error_m", 0.0)
```

- [ ] **Step 4: 在 step() 内插入地面门**

`precision_align_controller.py:256-263`。现状是：

```python
        raw_tol = self._raw_tolerance(drop_height)
        aligned = self._is_precise(pixel_error, smooth_pixel_error, raw_dx, raw_dy, raw_tol)
```

改为：

```python
        raw_tol = self._raw_tolerance(drop_height)
        aligned = self._is_precise(pixel_error, smooth_pixel_error, raw_dx, raw_dy, raw_tol)

        # 地面误差硬门: 像素口径随高度漂移, 米制口径在 SITL/真机/各高度间一致.
        # 用原始 raw 值 (本帧够不够准), 不用 EMA 平滑值 — status 里的地面误差
        # 字段仍用平滑值, 保持既有数值断言不变.
        # 地面误差只在 continuous 路径有意义 (非 continuous 恒为 0.0), 必须同时判 continuous.
        ground_gate = float(
            self._param("precision_align_release_max_ground_error_m", 0.0)
        )
        if continuous and ground_gate > 0.0:
            g_fwd, g_right = self._ground_error_m(
                raw_dx * width, raw_dy * height, altitude,
            )
            if math.hypot(g_fwd, g_right) > ground_gate:
                aligned = False
```

> **为什么 `* width` / `* height`**：`step()` 中计算 status 字段时用的是
> `smooth_du = float(self._smooth_dx) * width`（`:266-269`），
> 即 `_ground_error_m` 接收的是**像素**位移。这里用同样的口径。
> （与 Task 4 Step 5 的防串筒门不同——那里传的已经是像素差，不能再乘。）

- [ ] **Step 5: 跑测试确认通过**

Run: `python3 -m pytest src/drone_mission/test/test_precision_align_controller.py -q`
Expected: PASS。特别确认
`test_ground_error_is_height_invariant_for_same_physical_offset`（`:364`）与
`test_continuous_descent_keeps_correcting_inside_release_pixel_window`（`:413`）
**未受影响**——本 Step 不改 status 的地面误差字段，只改 `aligned`。

- [ ] **Step 6: 打开 quick 的地面门**

`mission_params_quick.yaml` 与 `_sim.yaml`：

```yaml
    precision_align_release_max_ground_error_m: 0.03  # 米制释放门; 0=禁用
```

- [ ] **Step 7: 保留四级防卡死阶梯（不改代码，仅确认）**

| 级 | 机制 | 位置 | 处置 |
|---|---|---|---|
| L1 | 地面门 0.03 m（唯一"正常"放行通道） | 本 Task 新增 | 启用 |
| L2 | 停滞直投：`pixel < 45` 且 band 5 内持续 2 s → 直接投 | `state_drop.py:233-286` | **保留**，但必须 warn 打印当时的 `best_ground_error`，供飞后统计这类投放的实际落点 |
| L3 | 对齐超时 15 s → 原地重置 1 次 → 像素兜底 60 px | `:288-306`、`:767-796` | 保留 |
| L4 | 曾对心兜底：`_descend_best_ground_error ≤ 0.4` → 投放，否则 takeover | `:784-796` | **保留 0.4，一期不收** |

> **L4 为什么不收**：它是"视觉几何彻底失效时还能不能投"的唯一活口。
> 在主要门限已经很紧的同时收紧最后一道兜底，正是 §1.5 里反复出现的卡死配方。
> 一期保留 0.4，靠日志统计走到这条兜底的架次比例，二期再决策。

- [ ] **Step 8: 补 L2 的地面误差触发条件**

现有 `drop_align_stall_threshold` 是纯像素口径且 = raw_tol + 5
（`test_quick_drop_strategy.py:397-405` 有断言），会出现
**"像素达标但地面不达标"时既不投也不触发停滞**的空窗。
在 `state_drop.py:233` 的停滞条件里补一个地面误差分支：

```python
        best_ground = self._descend_best_ground_error
        ground_stall = (
            math.isfinite(best_ground)
            and best_ground > float(
                self._fsm.get_param("precision_align_release_max_ground_error_m", 0.0)
            )
        )
```

并让停滞计时在 `ground_stall` 为真时**也**推进（与像素分支并列，不替代）。

- [ ] **Step 9: 提交**

```bash
git add src/drone_mission/drone_mission/precision_align_controller.py \
        src/drone_mission/drone_mission/states/state_drop.py \
        src/drone_mission/drone_mission/mission_state_machine.py \
        src/drone_mission/config/mission_params_quick.yaml \
        src/drone_mission/config/mission_params_quick_sim.yaml \
        src/drone_mission/test/test_precision_align_controller.py
git commit -m "feat(quick): 释放判据改为地面误差米制门 (0.03m), 补地面侧停滞触发"
```

---

## Task 6: 启用两级降高到 1.2 m（WP5）

**Files:**
- Modify: `src/drone_mission/config/mission_params_quick.yaml:127-129`
- Modify: `src/drone_mission/config/mission_params_quick_sim.yaml`
- Modify: `src/drone_mission/drone_mission/states/state_drop.py:226-231`（RELEASE 日志）
- Test: `test_quick_drop_strategy.py:345-357`

**Interfaces:**
- Consumes: `drop_descend_height`、`precision_align_final_descent_enable`、`precision_align_final_drop_height`、`_active_drop_height()`（`state_drop.py:533-536`）、`_start_final_descent_if_needed()`（`:538-569`）
- Produces: 投放高度 ≈1.2 m

**为什么第一级必须留在 2.4 m**【已核实】：`_check_descent_ready`（`:467-531`）的停滞判定要求

```python
min_height_progress = max(0.3, self._descent_initial_error * 0.25)
if self._descent_initial_error - height_error < min_height_progress:
    return False
```

`_descent_initial_error` 在 `:558` 按 `|当前高度 − final_h|` 重算：

| 第一级高度 | initial_error | 要求 progress | 能否达到 |
|---|---|---|---|
| **2.4 m（本方案）** | ≈1.2 m | max(0.3, 0.30) = 0.30 | ✓ 可达 |
| 1.5 m（**不要这么做**） | ≈0.3 m | max(0.3, 0.075) = **0.30** | ✗ 要求 0.30 m 走完 0.30 m 全程 |

⇒ 第一级降到 1.5 m 会让二阶段**永远判不到位**（叠加 SITL/PX4 稳态偏差后更甚）→ 死等。
**保持 2.4 m 即规避，无需改动 `_check_descent_ready`。**

**"到底是不是盲降"**【已核实，重要】：
`_start_final_descent_if_needed`（`:559-562`）**确实会发一条纯位置下降指令**
`fc.goto_ned(pos.north, pos.east, altitude_to_ned(final_h), speed=speed)`，
日志原文是"定点直降"——字面上正是 §1.5 里让 real 策略偏 39/61 cm 的盲降模式。

**但它不会持续生效**，原因是 `_servo_descend_active`：

| 时刻 | 值 | 位置 |
|---|---|---|
| `_enter_descend` 进入 quick 分支 | → **True** | `:117-119` |
| 整个 DESCEND 期间 | 保持 True | — |
| 仅在 `descent_ready` 为真时 | → False | `:197` |

而 `_start_final_descent_if_needed` **既不置位也不清零**该标志，所以它返回 True 后，
下一 tick 的 `_execute_descend`（`:190-196`）仍走 `_continuous_servo_descend_tick`
→ `_nudge_to_visible_drop_target(self._active_drop_height()=1.2, descent_speed=0.28, allow_release=False)`。
`_active_drop_height()` 已切到 1.2 m，于是**视觉伺服继续接管下降全程**，
那条 `goto_ned` 只存活一个 tick（50 ms）就被速度指令覆盖。

⇒ **quick + continuous 路径下启用两级降高不是盲降**，但成立的依据是
`_servo_descend_active` 的保持，而**不是**"没发位置指令"。

- [ ] **Step 1: 改真机 quick 参数**

`mission_params_quick.yaml`：

```yaml
    precision_align_final_descent_enable: true  # quick: 启用两级降高 (2.4m 对心 → 1.2m 复查)
    precision_align_final_drop_height: 1.2      # 最终投放高度; 圆筒 50px, 落体 0.49s, 漂移 4.9~5.9cm
    precision_align_final_descent_speed: 0.25   # 连续伺服路径不使用, 仅非连续路径生效
```

`drop_descend_height` **保持 2.4 不变**（第一级对心高度，理由见上表）。

- [ ] **Step 2: 同步 SITL**

`mission_params_quick_sim.yaml` 同名三项改为同样的值。

- [ ] **Step 3: 写失败测试**

沿用该文件已有的 `_quick_params(name)`（`:337`，内部调 `_load_parameters`）：

```python
def test_quick_enables_two_stage_descent():
    p = _quick_params("mission_params_quick.yaml")

    assert p["precision_align_final_descent_enable"] is True
    assert p["precision_align_final_drop_height"] == 1.2
    assert p["drop_descend_height"] == 2.4   # 第一级留在 2.4m (见本 Task 的进到位判定分析)
```

Run: `python3 -m pytest src/drone_mission/test/test_quick_drop_strategy.py::test_quick_enables_two_stage_descent -v`
Expected: FAIL（现值 `false` / `1.5`）

- [ ] **Step 4: 跑测试确认通过**

Run: `python3 -m pytest src/drone_mission/test/test_quick_drop_strategy.py -q`
Expected: PASS（`:345-357` 若有钉死旧值的断言，在此一并更新）

- [ ] **Step 5: 加 RELEASE 结构化日志**

`state_drop.py:226-231`（居中 → EXECUTE_DROP）处新增一行：

```python
                self._fsm.get_logger().info(
                    "RELEASE: "
                    f"height={active_h:.2f}m speed_xy={speed_xy:.3f}m/s "
                    f"gate_px={gate_px:.0f} gate_ground_m={gate_ground_m:.3f} "
                    f"source={status.source} "
                    f"ground=({status.ground_forward_error_m:+.3f},"
                    f"{status.ground_right_error_m:+.3f})m "
                    f"best_ground_m={self._descend_best_ground_error:.3f} "
                    f"drift_est_m={fall_time * speed_xy:.3f}"
                )
```

其中 `fall_time = math.sqrt(2.0 * active_h / 9.81)`。

> 现在投放时刻的关键量散在多条日志里，没有这一行就无法逐架次核对误差预算是否兑现
> （`PLAN_visual_servo_optimization.md` D2 项指出的"无落点反馈闭环"在软件侧的第一步）。

- [ ] **Step 6: SITL 观察切换点平滑性**

Run: `./scripts/start_quick_simulation.sh`

观察 2.4→1.2 切换点（日志 `DESCEND: 初次精校完成, 定点直降至最终投放高度后复查`）前后的水平速度曲线。

**残留风险**：那条一 tick 的位置指令会让飞控瞬间切到位置控制，可能造成横向顿挫。
若明显，给 `state_drop.py:559` 加分支——`_continuous_drop_servo_enabled()` 为真时不发 `goto_ned`：

```python
        if self._continuous_drop_servo_enabled():
            # 连续伺服路径: 不发位置指令, 由 _continuous_servo_descend_tick 接管
            # 二阶段直降 (避免一个 tick 的位置控制打断横向伺服)
            self._fsm.get_logger().info(
                "DESCEND: 二阶段直降由连续地面误差伺服接管, 不发定点指令"
            )
        else:
            self._fsm.fc.goto_ned(
                pos.north, pos.east,
                self._fsm.fc.altitude_to_ned(final_h), speed=speed,
            )
```

- [ ] **Step 7: 提交**

```bash
git add src/drone_mission/config/mission_params_quick.yaml \
        src/drone_mission/config/mission_params_quick_sim.yaml \
        src/drone_mission/drone_mission/states/state_drop.py \
        src/drone_mission/test/test_quick_drop_strategy.py
git commit -m "feat(quick): 启用两级降高至 1.2m, 加 RELEASE 结构化日志"
```

---

## Task 7: 相机标定与释放点偏移量测（WP7）

**为什么是一期必需**：Task 5 的预算算术显示，Ø15 只剩 1.6~2.6 cm，
而 cx 偏 10 px @1.2 m 就是 3 cm。系统偏置是**像素闭环无法消除**的：
对心再完美落点也会稳定偏出筒心，且地面误差门会因偏置不可达**频繁触发兜底**
（表现为"投放正常但落点超差"，最容易被误读成"视觉没对准"）。

**关于 `verify_real_flight_ready.sh`（纠正一处易错判断）**【已核实】：
该脚本 `:63-70` 硬断言 fx=fy=400 / cx=320 / cy=240 且 `grep -Fq` 精确匹配，
改标定值后它必失败——**但它只被 `scripts/start_full_real_mission.sh:212` 引用**，
**quick 路径不经过它**，所以改标定不会卡住 quick 起飞。
真正相关的是 `src/drone_bringup/test/test_real_mission_scripts.py` 会断言该脚本内容，
且 `docs/Debug/RISKS_AND_TODOS.md` 的 **T1 项**已登记此事（"标定 640×480 相机内参…
同步将 verify_real_flight_ready.sh 改为参数化/范围断言"）。

**Files:**
- Modify: `src/drone_bringup/launch/real_flight.launch.py:104-107, 122-125`（标定后）
- Modify: `src/drone_mission/config/mission_params_quick.yaml:136-139`（标定后）
- Modify: `scripts/verify_real_flight_ready.sh:63-70`（改为范围/参数化断言，随 T1）

**Interfaces:**
- Consumes: `scripts/debug_vision.sh`（独立视觉调试，注意其 `pixel_to_world` fallback 不可缺失）
- Produces: 标定后的 fx / cx / cy；落点相对筒心的系统偏置实测值

- [ ] **Step 1: 搭静态标定台**

机体水平架高，**拆下桨叶**。从释放钩垂铅垂线到地面做标记。

- [ ] **Step 2: 量测 cx / cy**

Run: `./scripts/debug_vision.sh`

把蓝盘或 Ø15 圆筒放在地面标记上，读取检测中心像素 `(u*, v*)`：

```
cx 误差 = u* - 320        （640 宽图像的真机半宽）
cy 误差 = v* - 240        （480 高图像的真机半高）
```

- [ ] **Step 3: 量测 fx**

量镜头到目标的水平距离 D，读目标像素直径 w：

```
fx ≈ w · D / 真实直径      （蓝盘 1.0 m，圆筒 0.15 m）
```

两个距离各测一次取均值。**交叉验证**：`/drone/vision/cylinders` 的 `radius` 字段
（`vision_pipeline.py:919`）应与实测筒半径吻合——若差得多，说明 fx 或高度估计有问题。

- [ ] **Step 4: 量测释放点偏移**

机体保持水平，记录铅垂线与相机光心投影的像素差，换算成机体系地面偏置：

```
bias_fwd/right = 像素偏置 × 高度 / fx
```

在 1.5 m 与 1.2 m 各测一次。

- [ ] **Step 5: 更新内参与自检脚本**

若标定值与原值（400/320/240）差异显著，更新 `real_flight.launch.py` 与
`mission_params_quick.yaml:136-139`，并把 `verify_real_flight_ready.sh:63-70`
的 `grep -Fq` 精确匹配改为范围断言，避免 full-real 路径被误伤。

Run: `python3 -m pytest src/drone_bringup/test/test_real_mission_scripts.py -q`
Expected: PASS

- [ ] **Step 6: 判定是否补偿**

| 条件 | 动作 |
|---|---|
| `|bias| ≤ 1 cm` 且重复两次一致 | 记录，不改参数 |
| `\|bias\| > 1 cm` 且重复两次一致 | 二期新增 `drop_align_offset_forward_m` / `drop_align_offset_right_m` 参数化补偿 |
| 不一致 | 只记录——可能混入了风/动压等动态偏差，**不要**当机械偏置补偿掉 |

> **一期只量测、先不实现补偿参数**：需先在 ≥2 架次重复验证偏置的稳定性。
> 一期投入代码去补偿一个还没验证的量，会把噪声固化进系统。

- [ ] **Step 7: 提交**

```bash
git add src/drone_bringup/launch/real_flight.launch.py \
        src/drone_mission/config/mission_params_quick.yaml \
        scripts/verify_real_flight_ready.sh
git commit -m "calib: 真机相机内参标定 + verify 脚本改范围断言"
```

---

## Task 8: 分层验证与真机架次

**Files:** 无代码改动，仅执行与记录。

- [ ] **Step 1: L1 地面静态（不飞，成本最低，先做）**

Run: `./scripts/debug_vision.sh`

- 圆筒在 2.4 / 1.5 / 1.2 m 各高度的**检出率与置信度**（Ø15 在 2.4 m 只有 ~25 px）；
- `CylinderRefiner` 是否稳定；`radius` 与实测筒半径是否吻合；
- **蓝盘与圆筒的像素中心是否重合**（验证"同心"前提，需求决策 1）。

**这一层失败 ⇒ 整个方案不成立，不必飞。**

- [ ] **Step 2: L2 SITL 端到端**

Run: `./scripts/start_quick_simulation.sh`

（`quick_sitl.launch.py` + `mission_params_quick_sim.yaml`，`layout_heading_deg` 默认 30°，
场地圆筒 Ø15/20/25 cm 且蓝盘与筒心同心——**同心前提可在 SITL 端到端验证**）

检查清单：

| 项 | 观测点 | 判据 |
|---|---|---|
| 双通道生效 | `/drone/vision/status`、`/drone/vision/cylinders` | 圆筒 n≥1 |
| 选中最小筒 | `SELECT_TARGET` 日志 | 与地面实测最小筒一致 |
| 半径排序正确 | 候选 `radius/observations` | Ø15 < Ø20 < Ø25（5.6 m 处 Ø15 仅 ~14 px，需多帧平均核对） |
| 低空主通道 | `PREC_ALIGN` 日志 | 1.2 m 附近出现 `source=cylinder` |
| 地面门生效 | RELEASE 日志 | `ground_*_m` 在 0.03 内 |
| 投放高度 | RELEASE 日志 | `height≈1.2m` |

> **注意**：SITL 与真机的像素窗口口径不一致（§1.8 第 2 条），
> **只有地面误差可直接对比**——这正是 Task 5 的价值。

- [ ] **Step 3: L3 单元测试**

Run: `python3 -m pytest src/drone_mission/test src/drone_vision/test src/drone_bringup/test -q`
Expected: 全 PASS

- [ ] **Step 4: L4 真机起飞前自检**

Run: `./scripts/verify_real_flight_ready.sh`（若走 full-real 路径；quick 走自己的启动脚本）

- [ ] **Step 5: 真机 2~3 架次，逐项记录**

| 观测 | 判据 |
|---|---|
| 两通道检测数/置信度/圆筒 `radius` | 三档可区分 |
| 精校段 **FPS** | 记录，供 Task 2 的退路决策 |
| 投放时刻 `pixel_error` 与 `ground_error_m` | 进入目标区间 |
| 锁定期间是否跳变 | 无（Task 4 风险点） |
| 蓝盘缺失兜底 | 人为遮挡蓝盘时不出现 lost/TARGET_LOST |
| **人工实测落点相对筒心偏差** | 必须记录 |

> 最后一项是 D2 项指出的"无落点反馈闭环"的唯一数据来源。
> 没有它就无法判断 Task 7 的偏置量，也无法决定二期是否进一步降高。

---

## Task 9: 文档与测试同步收官

- [ ] **Step 1: 全量测试**

Run: `python3 -m pytest src/ -q`
Expected: 全 PASS

- [ ] **Step 2: 检查测试同步清单**

| 测试文件 | 断言 | 状态 |
|---|---|---|
| `test_quick_drop_strategy.py:201-221` | `set_vision_calls == ["marker_only"]` | **保持绿，无需改动**（用显式参数 dict，不含新参数 → 走默认值）。Task 2 Step 7 |
| `test_quick_drop_strategy.py:185` | "只数 marker 不计圆筒" | **需改名并更新注释**（行为已变，结论碰巧仍成立）。Task 3 Step 5 |
| `test_quick_drop_strategy.py:325` | AST 默认值契约 | Task 3 Step 8：`drop_target_source` 默认不变；**新参数须加进这份默认值表** |
| `test_quick_drop_strategy.py:345`、`:364` | quick yaml 的 target_source / final_descent / height | Task 3 Step 8、Task 6 Step 4 |
| `test_quick_drop_strategy.py:360-381` | SITL 像素窗口 = 真机 2× | **保持**（Task 5 后地面口径统一，像素口径差异仍存在，注释可补） |
| `test_quick_drop_strategy.py:397-405` | `stall = raw_tol + 5` | Task 5 Step 8 后需确认仍成立 |
| `test_quick_drop_strategy.py:431-441` | quick 参数不得出现在基础/real yaml | 新参数须只加到 quick 两个 yaml |
| `test_precision_align_controller.py:232-258` | cylinder 不回落 marker | **必须保持绿** |
| `test_precision_align_controller.py:364`、`:413` | status 地面误差字段 / 连续下降 | Task 5 不改这些字段，**应保持绿** |
| `test_vision_modes.py:90-105` | 源码字符串断言 | 本计划**不改** `vision_pipeline.py`，应保持绿 |
| `test_layout_mission_config.py:46,56` | quick profile `drop_target_source` | Task 3 Step 8 |
| `test_quick_launch_configuration.py:113` | `quick["drop_target_source"]` | Task 3 Step 8 |

- [ ] **Step 3: 更新本仓库相关文档**

- `docs/Tactics/PLAN_visual_servo_optimization.md`：X1（mode 1 单模型精校）本次未实现，
  补一句"已在 quick 双通道方案中评估，一期未启用"；Z4（相机↔释放点偏移）标注 Task 7 已量测。
- `docs/Debug/RISKS_AND_TODOS.md` T1：标注 `verify_real_flight_ready.sh` 已改范围断言。

- [ ] **Step 4: 提交**

```bash
git add docs/
git commit -m "docs: quick 投放双通道视觉方案实施记录与文档同步"
```

---

## 4. 风险与回退

### 4.1 最可能失败的四点（按严重度）

1. **系统偏置没压到 1 cm 以内**——最根本的一条。1.2 m 处只剩 1.6~2.6 cm 预算，
   而 cx 偏 10 px 就是 3 cm。偏置是像素闭环无法消除的，且会让地面误差门因不可达
   而频繁触发兜底。⇒ Task 7 从可选变必需的直接原因。
2. **低空动态下圆筒检测连续性不足**。下降段横向机动 + 运动模糊，圆筒只有 40~50 px，
   且 `CylinderRefiner` 的门槛**没有针对低空放宽**（`vision_params.yaml:46-48` 只给了 marker 低空 relax）。
   一旦帧间不连续，`"fused"` 实际退化成单通道，"降高后圆筒更准"就不成立。
   先验观测：`/drone/vision/cylinders` 的连续性、`PREC_ALIGN` 里 `source=cylinder` 的占比。
3. **搜索期双模型的 FPS 代价**吃掉 quick 的提速收益；同时帧数门
   （`reacquire_after_lost 12` / `max_lost_frames 48`）在低 FPS 下等价于"数秒"，鲁棒性随之下降。
4. **判据收紧后悬停卡死**（历史重复模式）。靠 L1~L4 四级阶梯；
   **不用半径自适应来"松绑"**（见下）。

### 4.2 为什么不做半径自适应门限

`radius = r_px × alt / max(fx,fy)`（`vision_pipeline.py:919`）与 `_ground_error_m`
**共用同一个 fx**。fx 未标定时两者同步缩放：fx 若真值 460 而配置 400，
地面误差被**低估**、半径同时被**高估**，于是 `k × radius` 这个门会跟着一起放宽，
恰好把标定缺失抵掉。
⇒ 半径自适应在"标定没做"的阶段会**掩盖**错误而非暴露它，不适合一期。
一期用**固定米制门**；半径自适应留二期，且必须等 fx 标定完成后再评估。
另外 refiner 半径本身有 ±10~20% 噪声，也支持它只做辅助判据、不承担验收。

### 4.3 一句话回退（全部配置级）

| Task | 回退 |
|---|---|
| 1 | 两个数值改回 `0.25` / `0.3` |
| 2 | `quick_search_vision_mode: "marker_only"` 一行 |
| 3 | `drop_target_source: "marker"` 一行 |
| 4 | `precision_align_target_source: "marker"` 一行；或 `precision_align_fuse_cylinder_below_m: 0.0` |
| 5 | `precision_align_release_max_ground_error_m: 0.0` 即禁用（`0` = 关闭是既有约定） |
| 6 | `precision_align_final_descent_enable: false` 一行（回到 2.4 m 单级） |
| 7 | fx/cx/cy 恢复 400/320/240 |

**这是刻意的设计约束**：所有行为变化都由"默认为旧值"的新参数或既有 yaml 键承载，
每一层都能单独、立即、不用重新编译地退回。
**任何需要"改代码才能回退"的改动在这个方案里都不合格。**

### 4.4 二期备选

- `drop_align_offset_forward_m` / `drop_align_offset_right_m` 参数化补偿（依赖 Task 7 实测）；
- 按实测释放速度反推是否进一步降高或延长刹车；
- 圆筒检测参数针对低空单独调优（`vision_params_quick.yaml` 现无圆筒覆盖）；
- 视觉档位新增 mode 1（搜索 marker_only / 投放 all），用于 FPS 与鲁棒性的折中。

---

## 5. 未决事项

1. **搜索期 FPS 的取舍**：`all` 双模型是否可接受，需 Task 2 Step 8 实测后再定。
2. **`goto_ned` 单 tick 顿挫**：Task 6 Step 6 的 SITL 观察结果决定是否加分支。
3. **实测释放速度基线**：Task 1 Step 4 决定 Task 6 的高度取 1.2 还是 1.5，
   以及速度门 0.12 是否需放宽到 0.15。
4. **低空圆筒检测连续性**：是否需要给圆筒参数加一组低空放宽——依赖 Task 8 L1/L2 数据。
5. **一期验收口径的分级**（§1.4）需要用户认可：一期为"分布收敛 + Ø15 可测量"，
   Ø15 稳定命中放二期。

---

## 6. 本计划对初稿的四处逻辑修正（留档）

这四处都是"读起来像对的、但读源码后不成立"的类型，记录以防将来重复推导：

| 初稿说法 | 实际情况 | 修正位置 |
|---|---|---|
| "启用两级降高后全程连续伺服、不是盲降" | `_start_final_descent_if_needed` **确实发了纯位置 `goto_ned`**（日志原文"定点直降"）。不盲降的真正原因是 `_servo_descend_active` 在 DESCEND 期间保持 True，下一 tick 就被连续伺服覆盖。**依据错了，结论对** | Task 6 |
| "`min_radius_first` 因此真正生效" | 它在 `mission_state_machine.py:166` **硬编码 True**，`:274` 声明的参数全仓库无读取点——yaml 里那行是死的。排序本来就在跑，但**无法通过 yaml 关掉** | §1.8 |
| "必须同步改 `verify_real_flight_ready.sh`，否则 quick 起飞前自检失败" | 该脚本**只被 `start_full_real_mission.sh:212` 引用**，quick 不经过它。且此事已登记为 `RISKS_AND_TODOS.md` 的 T1 | Task 7 |
| "必要时只用半径自适应、不用固定紧阈值"（作为防卡死退路） | 半径与地面误差**共用未标定的 fx**，同向缩放 ⇒ 半径门会跟着放宽、**掩盖标定缺失**。不能作为退路 | §4.2 |
