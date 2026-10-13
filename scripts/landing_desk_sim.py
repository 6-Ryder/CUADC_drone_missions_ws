#!/usr/bin/env python3
"""
降落伺服台面仿真 —— 量出"低空误检"造成的实际横向漂移, 以及卡方门能挡掉多少.

**这不是 SITL**: 载机用一阶速度环 + 位置积分近似 (含频率滞后), 检测流是脚本生成的.
它验证的是**控制律**(伺服 + 门 + 估计器), 不是 PX4 的动力学. 结论必须按此口径引用.

为什么值得有: 单测能证明"误检帧会让指令跳到 0.284 m/s"(刻画), 但回答不了
"那到底把飞机带偏多少米". 这个脚本把闭环跑起来, 给出可比较的数字。

用法:
    python3 tools/landing_desk_sim.py            # 默认场景
    python3 tools/landing_desk_sim.py --json     # 机器可读输出

场景: 目标在原点; 载机从 (0.5, 0.5) 起、2.0m 高, 边对心边下降;
在 t=3.0-3.3s 注入 70px 的低空误检 (与 2026-08-08 记录同形态: 能穿过
置信度门与 80px 跳变门 —— 在 2m 高度它只相当于约 0.35m 地面偏差).
"""

import argparse
import json
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
for pkg in ("drone_mission", "drone_utils", "drone_vision", "px4_control"):
    sys.path.insert(0, str(ROOT / "src" / pkg))
sys.path.insert(0, str(ROOT / "src" / "drone_mission" / "test"))

import test_simple_precision_land as harness  # noqa: E402
from drone_mission.states.state_precision_land_simple import (  # noqa: E402
    SimplePrecisionLandState,
)

# ── 台面模型参数 (与真机同量级的近似, 不是标定值) ──────────────────────────
TICK_HZ = 20.0
VISION_HZ = 4.0            # 3-5Hz 量级
VEL_LAG_S = 0.3            # 一阶速度环滞后 (替身: PX4 速度环 + 姿态环的等效)
DURATION_S = 12.0
FALSE_FRAMES = 1           # 误检持续多少个检测帧
DETECT_LATENCY_S = 0.25    # 采集→可用 的延迟 (3-5Hz 实测量级)
FALSE_PX = 70.0
ALTITUDE_M = 2.0
# 已记录的检测失效高度带 (m, 离地): 3.15-3.75m 是 2026-08-14 的断裂带;
# 0.85-1.05m 是近地失效 (~0.9m, 多个 profile 的注释都提到). 带内电机**什么都不发**,
# 模拟"检测在某个高度带整体消失"—— 这是台面仿真此前完全没有的输入。
DROPOUT_BANDS = ((3.15, 3.75), (0.85, 1.05))
DESCENT_START_M = 5.0      # 从 5m 起降, 这样会**穿过**两个失效带
DROP_DURATION_S = 45.0
AGE_GATE_S = 2.0           # 与 landing_target 龄期门同值
FX = FY = 400.0
CX, CY = 320.0, 240.0


class DeskVehicle:
    """一阶速度环 + 位置积分 (NED). 只为了给控制律一个带滞后的被控对象."""

    def __init__(self, north=0.5, east=0.5, down=-ALTITUDE_M):
        self.north, self.east, self.down = north, east, down
        self.vn = self.ve = self.vd = 0.0
        self.yaw = 0.0

    def step(self, cmd_forward, cmd_right, cmd_down, dt):
        # 机体系 → NED (与 set_velocity_body 同约定)
        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        vn_cmd = cmd_forward * cos_yaw - cmd_right * sin_yaw
        ve_cmd = cmd_forward * sin_yaw + cmd_right * cos_yaw
        alpha = min(dt / VEL_LAG_S, 1.0)
        self.vn += alpha * (vn_cmd - self.vn)
        self.ve += alpha * (ve_cmd - self.ve)
        self.vd += alpha * (cmd_down - self.vd)
        self.north += self.vn * dt
        self.east += self.ve * dt
        self.down += self.vd * dt


def _detected_pixel(vehicle, target=(0.0, 0.0)):
    """真值 → 像素 (脚本检测器). 用"镜头到平面"的距离做尺度."""
    plane = max(-vehicle.down, 0.05)
    dn = target[0] - vehicle.north
    de = target[1] - vehicle.east
    cos_yaw, sin_yaw = math.cos(vehicle.yaw), math.sin(vehicle.yaw)
    fwd = dn * cos_yaw + de * sin_yaw
    right = -dn * sin_yaw + de * cos_yaw
    # ⚠️ 前向轴的符号: 真机相机"画面顶朝机头"(image_top_faces_forward), 因此
    # **目标在前方 → 出现在画面上方 → v < CY**。写成 CY + fwd*FY/plane 会与
    # 伺服标定 (landing_servo_y_to_forward_sign = -1.0) 相反, 载机朝反方向飞,
    # 台面回路因此发散 —— 无故障对照组就是被这个抓出来的。
    return (CX + right * FX / plane, CY - fwd * FY / plane)


def run(gate_enabled: bool, inject_fault: bool = True, latency_s=DETECT_LATENCY_S,
        fault_frames=FALSE_FRAMES, fault_px=FALSE_PX, alternate=False):
    params = {
        "servo_px_kp": 0.0025,
        "servo_px_ki": 0.000125,
        "servo_px_max_speed": 0.35,
        "landing_servo_deadzone_px": 12.0,
        "landing_blind_altitude": 0.05,
        "camera_fx": FX, "camera_fy": FY, "camera_cx": CX, "camera_cy": CY,
        "landing_servo_estimator_gate_enable": gate_enabled,
    }
    fsm = harness._SimpleFSM(params)
    state = SimplePrecisionLandState(fsm)
    state._phase = state.PHASE_SERVO
    vehicle = DeskVehicle()

    dt = 1.0 / TICK_HZ
    steps = int(DURATION_S * TICK_HZ)
    vision_every = max(int(round(TICK_HZ / VISION_HZ)), 1)

    trace = []
    excursion = 0.0          # 误检**造成的额外偏移**, 而不是含初始条件的绝对漂移
    baseline = 0.707        # 故障开始时的漂移 (起点到目标距离)
    # 位姿历史: 让 sim 能提供"采集时刻"的位姿 —— 与真机上 FSM 的 pose_at 同角色
    history = []

    def pose_at_time(ts):
        best = None
        for item in history:
            if item[0] <= ts:
                best = item
        return None if best is None else best[1:]

    fault_from = 3.0
    fault_to = fault_from + fault_frames / VISION_HZ
    for k in range(steps):
        t = k * dt
        fsm.bridge._pos = type("P", (), {
            "north": vehicle.north, "east": vehicle.east, "down": vehicle.down})()
        history.append((1.0 + t, vehicle.north, vehicle.east, vehicle.down,
                        vehicle.yaw))
        fsm.set_pose_at(*history[-1][1:])

        if k % vision_every == 0:
            # **延迟的真实性**: 这一帧拍于 latency 之前, 所以用当时的位姿成像
            past = pose_at_time(1.0 + t - latency_s) or history[-1][1:]
            ghost = DeskVehicle(past[0], past[1], past[2])
            ghost.yaw = past[3]
            u, v = _detected_pixel(ghost)
            in_false = inject_fault and fault_from <= t < fault_to
            if in_false:
                # 低空误检: 只扰动 u (等价于 east 轴跳变).
                # alternate=True 逐帧换边 —— 这才是"追着误检左右摆"的形态,
                # 固定偏移只是温和特例 (载机停在幻影处就完事).
                side = -1.0 if (alternate and (k // vision_every) % 2) else 1.0
                u += side * fault_px
            # 状态是从 FSM 取目标的 (_get_stable_landing_target → get_landing_target),
            # 不是从 _execute_servo 的入参取 —— 所以必须写回 fsm._target,
            # 否则 has_target=True 也拿不到东西, 载机原地不动 (踩过一次).
            fsm._target = {"center_u": u, "center_v": v, "confidence": 0.9}
            fsm.set_measurement_stamp(1.0 + t)

        if fsm._target is None:
            continue
        state._execute_servo(has_target=True)
        cmd = fsm.fc.calls[-1]
        vehicle.step(cmd[0], cmd[1], cmd[2], dt)

        drift = math.hypot(vehicle.north, vehicle.east)
        trace.append((t, drift))
        if fault_from - 1e-9 <= t < fault_from + 1e-9:
            baseline = drift
        if fault_from <= t < fault_to + 0.5:
            excursion = max(excursion, drift - baseline)

    return {
        "excursion_m": max(excursion, 0.0),
        "gate_enabled": gate_enabled,
        "inject_fault": inject_fault,
        "latency_s": latency_s,
        "fault_frames": fault_frames,
        "fault_px": fault_px,
        "alternate": alternate,
        "max_drift_m": max(d for _, d in trace),
        "final_drift_m": trace[-1][1],
        "final_altitude_m": -vehicle.down,
        "rejected_frames": state._target_estimator.rejected_count,
        "trace": trace,
    }


def run_dropout(age_gate: bool, transit: bool, duration_s=DROP_DURATION_S):
    """穿过检测失效带的完整降落, 带/不带两项已有修复, 看会不会"卡住".

    两个开关对应本项目已交付的两项改动:
      age_gate  —— 2c: 降落目标龄期门 (超时即视为无目标; 关掉=用冻结的旧目标继续飞)
      transit   —— 4a/4b: 丢失后继续飞向最后已知位置 (关掉=原地悬停)

    台面模型只到控制律这一层: 它没有 PX4 动力学, 也没有真实的检测失效机理
    (怎么失效是脚本写死的), 所以它回答的是"在这条失效序列下, 控制律的表现如何",
    不是"真机为什么在那一段失效"。
    """
    params = {
        "servo_px_kp": 0.0025, "servo_px_ki": 0.000125, "servo_px_max_speed": 0.35,
        "landing_servo_deadzone_px": 12.0,
        "landing_blind_altitude": 0.05,
        "landing_descent_fast": 0.4, "landing_descent_medium": 0.25,
        "landing_descent_slow": 0.12,
        "camera_fx": FX, "camera_fy": FY, "camera_cx": CX, "camera_cy": CY,
    }
    fsm = harness._SimpleFSM(params)
    state = SimplePrecisionLandState(fsm)
    state._phase = state.PHASE_SERVO
    if not transit:
        state._lost_transit_command = lambda altitude: (0.0, 0.0)
    last_detection = [-1e9]
    if age_gate:
        # 2c 的语义: 判据是"距最后一次**含检测**的消息", 超时即视为无目标
        fsm.get_landing_target = lambda: (
            fsm._target if fsm._target is not None
            and (sim_time[0] - last_detection[0]) <= AGE_GATE_S else None)

    # ⚠️ 必须有横向初始偏差: 若从目标正上方起降, 伺服没有任何活要干 → 检测是否
    # 失效对结果毫无影响, 四组配置会给出完全相同的数字 (踩过一次, 那次"跑通了"
    # 但什么都没测到).
    vehicle = DeskVehicle(0.8, -0.6, -DESCENT_START_M)
    sim_time = [0.0]
    dt = 1.0 / TICK_HZ
    steps = int(duration_s * TICK_HZ)
    vision_every = max(int(round(TICK_HZ / VISION_HZ)), 1)

    stuck_s = 0.0          # 带内几乎不下降的时间
    band_s = 0.0           # 处于失效带内的总时间
    loss_ticks = 0         # 走到"目标视为丢失"分支的 tick 数 (见下面的自检)
    landed_at = None
    for k in range(steps):
        t = k * dt
        sim_time[0] = t
        fsm.bridge._pos = type("P", (), {
            "north": vehicle.north, "east": vehicle.east, "down": vehicle.down})()

        in_band = any(lo <= -vehicle.down <= hi for lo, hi in DROPOUT_BANDS)
        if in_band:
            band_s += dt

        if k % vision_every == 0 and not in_band:
            u, v = _detected_pixel(vehicle)
            fsm._target = {"center_u": u, "center_v": v, "confidence": 0.9}
            fsm.set_measurement_stamp(1.0 + t)
            last_detection[0] = t

        if fsm._target is None:
            continue
        before_down = vehicle.down
        if fsm.get_landing_target() is None:
            loss_ticks += 1
        state._execute_servo(has_target=True)
        cmd = fsm.fc.calls[-1]
        vehicle.step(cmd[0], cmd[1], cmd[2], dt)
        if in_band and abs(vehicle.down - before_down) < 0.05 * dt:
            stuck_s += dt
        if landed_at is None and -vehicle.down <= 0.2:
            landed_at = t
            break

    return {
        "loss_ticks": loss_ticks,
        "age_gate": age_gate,
        "transit": transit,
        "landed": landed_at is not None,
        "landed_at_s": landed_at,
        "final_altitude_m": -vehicle.down,
        "final_lateral_m": math.hypot(vehicle.north, vehicle.east),
        "time_in_dropout_band_s": band_s,
        "stuck_in_band_s": stuck_s,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--json", action="store_true", help="输出 JSON")
    ap.add_argument("--frames", type=int, default=FALSE_FRAMES,
                    help="误检持续多少个检测帧")
    ap.add_argument("--px", type=float, default=FALSE_PX, help="误检像素偏移")
    ap.add_argument("--latency", type=float, default=DETECT_LATENCY_S,
                    help="采集→可用延迟 (s)")
    ap.add_argument("--sweep", action="store_true", help="扫故障帧数, 只看关键数字")
    ap.add_argument("--alternate", action="store_true",
                    help="误检逐帧换边 (追误检左右摆的形态)")
    ap.add_argument("--dropout", action="store_true",
                    help="跑穿过检测失效带的场景 (四组对照)")
    args = ap.parse_args()

    if args.dropout:
        print("穿过检测失效带 (3.15-3.75m / 0.85-1.05m) 的完整降落, 从 5m 起")
        print("⚠️ 台面模型: 只到控制律这一层; 失效机理是脚本写死的, 不是真机机理\n")
        rows = [run_dropout(ag, tr) for ag in (True, False) for tr in (True, False)]
        max_loss = max(r["loss_ticks"] for r in rows)
        if max_loss < TICK_HZ * AGE_GATE_S:
            # ⚠️ 自检: "目标视为丢失"这条分支几乎没被走到, 那么 age_gate / transit
            # 两个开关**根本没有作用对象**, 四组数字必然相同 —— 这不是
            # "检测中断无害"的证据, 而是场景没走到被测代码的证据.
            print(f"⚠️ 场景无效, 不给出结论: 四组里最多只有 {max_loss} 个 tick 走到"
                  f"\"目标视为丢失\"分支 (阈值 {int(TICK_HZ * AGE_GATE_S)})。")
            print("   原因: 失效带太窄 —— 0.6m 带在 0.4m/s 下降速度下只要 1.5s, "
                  "短于 2s 龄期门。")
            print("   要让这条场景有意义, 需要**先让下降停下来**(即复现真机那种"
                  "'悬停→检测恢复→再下降→再丢失'的自持循环), 而那属于投放精校"
                  "子系统 (precision_align_controller), 不是本脚本驱动的降落状态。")
            print("   详见 docs/PRECISION_LANDING_PLAN.md §4。")
            return
        print(f"{'龄期门':>7s}{'继续接近':>10s}{'丢失tick':>10s}{'着陆':>6s}"
              f"{'着陆用时s':>10s}{'结束高度m':>10s}{'结束横向m':>10s}{'带内卡住s':>10s}")
        for r in rows:
            landed = "是" if r["landed"] else "否"
            took = f"{r['landed_at_s']:.1f}" if r["landed"] else "—"
            print(f"{('开' if r['age_gate'] else '关'):>7s}"
                  f"{('开' if r['transit'] else '关'):>10s}"
                  f"{r['loss_ticks']:>10d}{landed:>6s}{took:>10s}"
                  f"{r['final_altitude_m']:>10.2f}{r['final_lateral_m']:>10.3f}"
                  f"{r['stuck_in_band_s']:>10.1f}")
        print("\n读法: '带内卡住s' 是处于失效带内且几乎不下降的时间 —— "
              "它大说明飞机卡在检测看不见的高度上, 无法靠下降脱离。")
        return

    if args.sweep:
        mode = "逐帧换边" if args.alternate else "固定偏移"
        print(f"故障帧数 → 误检造成的额外偏移 / 结束偏差 (m)   [{mode}]")
        print(f"{'帧数':>4s}{'时长s':>8s}{'门关偏移':>11s}{'门开偏移':>11s}"
              f"{'门关结束':>11s}{'门开结束':>11s}")
        for n in (1, 2, 4, 8, 16):
            o = run(False, fault_frames=n, latency_s=args.latency,
                    fault_px=args.px, alternate=args.alternate)
            g = run(True, fault_frames=n, latency_s=args.latency,
                    fault_px=args.px, alternate=args.alternate)
            print(f"{n:>4d}{n / VISION_HZ:>8.2f}{o['excursion_m']:>11.3f}"
                  f"{g['excursion_m']:>11.3f}{o['final_drift_m']:>11.3f}"
                  f"{g['final_drift_m']:>11.3f}")
        return

    # 无故障对照: 若这条都不收敛, 说明台面模型本身的回路不稳定,
    # 那么"门挡掉多少漂移"就没有比较意义 (基线是假的).
    clean = run(gate_enabled=False, inject_fault=False)
    off = run(gate_enabled=False, fault_frames=args.frames, latency_s=args.latency)
    on = run(gate_enabled=True, fault_frames=args.frames, latency_s=args.latency)

    if args.json:
        print(json.dumps({"gate_off": off, "gate_on": on}, indent=2))
        return

    print("降落伺服台面仿真 —— 低空误检 (70px, t=3.0-3.3s, 高度 2.0m)")
    print("⚠️ 不是 SITL: 载机是一阶速度环+积分近似, 验证的是控制律不是动力学\n")
    print(f"{'':22s}{'门关':>12s}{'门开':>12s}")
    for key, label, unit in (
        ("excursion_m", "误检造成的额外偏移", "m"),
        ("final_drift_m", "结束横向偏差", "m"),
        ("final_altitude_m", "结束高度", "m"),
        ("rejected_frames", "被门拒绝帧数", ""),
    ):
        print(f"{label:22s}{off[key]:>11.3f}{unit:1s}{on[key]:>11.3f}{unit:1s}")
    if off["excursion_m"] > 1e-6:
        print(f"\n偏移减少: {off['excursion_m'] - on['excursion_m']:+.3f} m "
              f"({(1 - on['excursion_m'] / off['excursion_m']) * 100:.0f}%)")
    print(f"无故障对照 (回路自身是否收敛): 结束漂移 {clean['final_drift_m']:.3f} m")
    if clean["final_drift_m"] > 0.2:
        print("⚠️ 无故障时回路自己就在漂 —— 台面模型不稳定, 上面的对比不成立,")
        print("   需要先修模型 (增益/滞后/采样) 或改用真实 SITL。")


if __name__ == "__main__":
    main()
