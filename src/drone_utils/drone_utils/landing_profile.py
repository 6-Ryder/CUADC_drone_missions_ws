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


def hold_at_search_altitude(fsm) -> None:
    """
    把载机保持在 (当前位置, `landing_search_altitude`) 悬停, 等待视觉检测.

    两条降落分支此前都是裸调 `fsm.loiter()`, 即"在进入降落那一刻的高度悬停",
    代码注释写着"保持在 5m" —— 那是**假设**上一段(返航)留下 5m, 而不是命令.
    `landing_search_altitude` / `landing_approach_speed` 两个参数因此一直零消费者.

    为什么值得命令而不是假设: 检测质量与高度强相关, 本项目已记录多个失效高度带
    (近地 ~0.9m, 以及 3.15-3.75m 的断裂带). 让搜索高度由参数决定, 才有可能把
    搜索段稳定放在这些带之外; 靠"上一段碰巧停在哪"则不可控.

    速度用 `landing_approach_speed` —— 这是一次纯垂直的、无测量的移动,
    必须慢.
    """
    fc = fsm.fc
    position = fsm.bridge.get_current_position()
    altitude = float(fsm.get_param("landing_search_altitude", 5.0))
    speed = float(fsm.get_param("landing_approach_speed", 0.5))
    if not math.isfinite(altitude) or altitude <= 0.0:
        # 参数不可用时退回"原地悬停" —— 与改动前的行为一致.
        fsm.loiter()
        return
    fc.goto_ned(
        float(position.north), float(position.east),
        fc.altitude_to_ned(altitude),
        speed=speed if math.isfinite(speed) and speed > 0.0 else 0.5,
    )


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


def camera_plane_distance(
    altitude_m: float,
    camera_offset_down_m: float = 0.0,
    min_distance_m: float = 0.05,
) -> float:
    """
    镜头光学中心到目标平面的距离 (m) —— 像素→米换算的**正确**高度基准.

    为什么不是直接用 altitude: `altitude` 是**机体原点**离地高度, 而像素是按
    **镜头**看到的东西成像的. 本项目真机相机装在原点下方 36.5cm
    (airframe_params_real.yaml 的 bucket_camera_down), 于是直接用 altitude 会把
    地面误差系统性放大: 2.5m 处放大约 17%, 1.0m 处约 57%.

    换算公式与投放路径保持一致 (state_drop.py:347 / state_cylinder_search.py:322
    的 plane_distance 同样减掉了相机下偏移), 避免同一物理量在两条路径上用两个
    公式.

    ⚠️ 下界取 0.05 而不是投放路径那个 0.3: 投放路径是**除以**这个距离
    (把机体偏移换算成像素), 小分母会发散, 所以需要一个大一点的保护值; 本函数
    的结果是用来**乘**像素误差的, 小距离只会得到小误差, 不存在发散问题. 若照抄
    0.3, 在 0.2m 高度会把误差凭空放大 50% —— 近地恰恰是最不该放大误差的地方.
    下界只为挡住"高度低于相机安装偏移"导致的零/负值.

    退化方向: 参数非法或算出的距离非正时返回下界, 而不是返回错误值或抛异常 ——
    调用方永远拿到一个可用的正距离.
    """
    if not all(math.isfinite(v) for v in
               (altitude_m, camera_offset_down_m, min_distance_m)):
        return max(min_distance_m, 0.0)
    distance = altitude_m - camera_offset_down_m
    if distance <= min_distance_m:
        return min_distance_m
    return distance


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
