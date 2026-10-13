"""
降落剖面: 像素→米换算 与 近地下降速度调度.

为什么放在 drone_utils 而不是某个状态里
--------------------------------------
本项目有两条并行的降落状态 (full 的 PrecisionLandState 与 quick 的
SimplePrecisionLandState), 二者各维护一套逻辑. 历史上多次出现"能力只改了
一条分支" —— 加在某一侧的算法另一侧永远拿不到. 凡是**跨分支共用的控制律**
都放在这里, 让两侧调用同一份实现, 只在一处写测试.

近地下降速度调度的权威来源
--------------------------
ArduPilot `ArduCopter/mode.cpp:747-756` 在离目标 0.35-2.0 m 段把下降速度按
剩余水平误差线性调度:

    precland_acceptable_error_m = 0.15
    land_slowdown_ms = MAX(0, target_error_m * (max_descent_speed_ms / 0.15))
    climb_rate_ms    = MIN(-0.1, -max_descent_speed_ms + land_slowdown_ms)

即 **越对准降得越快, 越偏降得越慢, 但有 0.1 m/s 的下限** (始终在下降, 不会
因为偏心而停在半空). 这一条直接针对本项目记录的低空振荡: 边横移边全速下降时,
横向修正来不及收敛就已经贴地, 于是低空出现"追着误检左右摆"的运动学条件.
"""

import math

# 下降速度下限 (m/s). 取自 ArduPilot mode.cpp:747-756 的硬编码 0.1 ——
# 它的作用是"无论多偏都要保持下降", 因此不能由调用方传 0 关掉.
LAND_DESCENT_FLOOR_MS = 0.1

# 启用误差→速度调度的最高高度 (m). ArduPilot 在离目标 0.35-2.0 m 段做调度;
# 本项目的高度分层下降在 1.5 m 以下进入慢速档, 故取同一个 1.5 作为边界 ——
# 高于它时横向误差本来就由伺服持续修正, 再叠加降速只会拖长低空停留.
NEAR_GROUND_SLOWDOWN_MAX_ALT_M = 1.5


def nominal_descent_speed(
    altitude_m: float,
    fast: float,
    medium: float,
    slow: float,
) -> float:
    """
    离地高度三档下降速度 (高空快降、近地慢降).

    两条降落分支此前各写一份: simple 在 `_descent_speed` 里, full 内联在
    `_execute_servo` 中, 阈值 3.0/1.5 各写一遍. 合并到这里, 免得只改一侧.
    """
    if altitude_m > 3.0:
        return float(fast)
    if altitude_m > NEAR_GROUND_SLOWDOWN_MAX_ALT_M:
        return float(medium)
    return float(slow)


def ground_error_m(
    du: float,
    dv: float,
    altitude_m: float,
    fx: float,
    fy: float,
) -> float:
    """
    像素误差 → 地面米制误差 (相机光轴竖直时的平面近似).

    Parameters
    ----------
    du, dv : float
        图像 x/y 方向的像素偏差 (已含图像→机体标定符号或取其模长皆可,
        本函数只取模长, 符号不影响结果).
    altitude_m : float
        离目标平面的高度 (m).
    fx, fy : float
        焦距 (px).

    Returns
    -------
    float
        地面水平误差 (m). 输入非法时返回 inf —— 调用方据此走"不可用"分支,
        绝不用一个错误的数继续控制.
    """
    values = (du, dv, altitude_m, fx, fy)
    if not all(math.isfinite(v) for v in values):
        return math.inf
    if fx <= 0.0 or fy <= 0.0 or altitude_m < 0.0:
        return math.inf
    return math.hypot(du * altitude_m / fx, dv * altitude_m / fy)


def near_ground_descent_speed(
    nominal: float,
    error_m: float,
    acceptable_error_m: float,
    floor: float = LAND_DESCENT_FLOOR_MS,
) -> float:
    """
    按剩余水平误差调度近地下降速度 (ArduPilot mode.cpp:747-756 的结构).

    error = 0            → nominal (最快)
    error = acceptable   → floor   (最慢, 仍在下)
    error > acceptable   → floor

    退化规则 (失败方向取"不改变原有行为"):
      - acceptable_error_m <= 0  → 返回 nominal, 即调度未启用;
      - error_m 非有限或为负     → 返回 nominal (当作已对准);
      - nominal 非有限或 <= 0    → 原样返回.
    """
    if not math.isfinite(nominal) or nominal <= 0.0:
        return nominal
    if not math.isfinite(acceptable_error_m) or acceptable_error_m <= 0.0:
        return nominal
    if not math.isfinite(error_m) or error_m <= 0.0:
        return nominal

    scale = 1.0 - error_m / acceptable_error_m
    if scale <= 0.0:
        return min(floor, nominal)
    return max(min(floor, nominal), nominal * scale)
