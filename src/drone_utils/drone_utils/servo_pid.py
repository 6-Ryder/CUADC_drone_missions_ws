"""
低频间歇视觉伺服的 PID 控制器.

用途
----
把像素/地面误差映射为机体系水平速度指令 (forward, right), 供任务状态机在
投放精校、圆筒逼近、精准降落等阶段使用.

与常规 PID 的三处关键区别
-------------------------
1. 多速率: 积分与微分按"新测量"推进, 不按控制 tick 推进.
   真机视觉实测约 10.5Hz, 而 FSM tick 为 20Hz. 若按 tick 积分, 同一份误差会被
   重复累加 4-7 次, 积分项以数倍速率冲向上限并在目标丢失时造成大速度踢出.
   本实现用两次新测量之间的真实时间作为积分/微分步长.

2. 死区即冻结: 误差进入死区时输出零速度且冻结积分 (不累加也不清零).
   现有投放/降落判决门 (稳定帧计数、对心锁存) 都假定"进入死区 == 机体已停稳",
   因此死区内必须输出零速度. 冻结而非清零, 使积分保留对抗常值扰动
   (风、重心偏置、相机安装倾角) 所需的偏置量, 目标再次偏出时可立即起作用.

3. 条件积分抗饱和: 仅在测量有效、非死区、且输出未达上限时累加积分.
   这是防止真机低空检测成串丢失 (代码中 lost_frames 最多计到 48 帧) 时
   积分饱和的唯一可靠手段.

坐标系与符号
------------
输入误差已由调用方完成坐标系变换, 本类不做任何坐标转换.
``sign_forward`` / ``sign_right`` 处理"图像方向 → 机体方向"的标定符号,
米制域调用方传入已带符号的误差时保持默认 +1.0 即可.
输出为机体 FRD 水平速度 (m/s), forward=机头方向, right=机体右侧.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class ServoPidStatus:
    """单次 update 的可观测状态, 供日志与测试使用."""

    error_forward: float = 0.0
    error_right: float = 0.0
    p_term: float = 0.0
    i_term: float = 0.0
    d_term: float = 0.0
    output_forward: float = 0.0
    output_right: float = 0.0
    in_deadzone: bool = False
    saturated: bool = False
    is_new_measurement: bool = False
    integrated: bool = False
    dt_measurement: float = 0.0


class ServoPid:
    """
    双轴速度 PID, 面向低频间歇视觉测量.

    Parameters
    ----------
    kp, ki, kd : float
        比例/积分/微分增益. 单位由误差域决定:
          米制域 — kp [ (m/s) per m ], ki [ (m/s) per (m·s) ], kd [ (m/s) per (m/s) ]
          像素域 — kp [ (m/s) per px ], ki [ (m/s) per (px·s) ], kd [ (m/s) per (px/s) ]
        ki=0 即退化为 PD, kd=0 即退化为 PI, 两者为 0 则等价于原有的纯 P 行为.

    max_speed : float
        水平合速度上限 (m/s), 按向量模长限幅.

    min_speed : float
        最小有效修正速度 (m/s). 用于压过飞控速度环的响应死区;
        误差在死区之外但算出的速度低于此值时, 按比例抬升到该值.

    deadzone : float
        误差死区, 单位与误差一致. 进入后输出零速度并冻结积分.

    sign_forward, sign_right : float
        误差 → 机体速度的标定符号, 取 ±1.

    integral_limit : float
        积分项对输出贡献的绝对值上限 (m/s). 这是抗饱和的硬钳位,
        与"条件积分"配合使用: 前者限制最坏情况, 后者阻止其发生.

    derivative_alpha : float
        微分项一阶低通系数, 取值 (0, 1]. 1.0 = 不滤波.
        像素抖动约 30px 时原始微分几乎全是噪声, 必须滤波;
        越小滤波越强但相位滞后越大. 建议 0.2-0.4.

    integral_band : float
        积分分离带, 单位与误差一致. 误差超过该值时停止积分,
        避免大误差段的积分在接近目标时变成过冲. 0 = 不启用.

    """

    def __init__(
        self,
        kp: float,
        ki: float = 0.0,
        kd: float = 0.0,
        max_speed: float = 0.25,
        min_speed: float = 0.0,
        deadzone: float = 0.0,
        sign_forward: float = 1.0,
        sign_right: float = 1.0,
        integral_limit: float = 0.1,
        derivative_alpha: float = 0.3,
        integral_band: float = 0.0,
    ):
        if not math.isfinite(kp) or kp < 0.0:
            raise ValueError("kp 必须为有限非负值")
        if not math.isfinite(ki) or ki < 0.0:
            raise ValueError("ki 必须为有限非负值")
        if not math.isfinite(kd) or kd < 0.0:
            raise ValueError("kd 必须为有限非负值")
        if not math.isfinite(max_speed) or max_speed <= 0.0:
            raise ValueError("max_speed 必须为有限正值")

        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.max_speed = float(max_speed)
        self.min_speed = max(0.0, float(min_speed))
        self.deadzone = max(0.0, float(deadzone))
        self.sign_forward = float(sign_forward)
        self.sign_right = float(sign_right)
        self.integral_limit = max(0.0, float(integral_limit))
        self.derivative_alpha = min(max(float(derivative_alpha), 0.01), 1.0)
        self.integral_band = max(0.0, float(integral_band))

        self._integral_fwd = 0.0
        self._integral_right = 0.0
        self._prev_error_fwd: Optional[float] = None
        self._prev_error_right: Optional[float] = None
        self._prev_stamp: Optional[float] = None
        self._d_fwd = 0.0
        self._d_right = 0.0
        self._last_dt = 0.0
        self._last_status = ServoPidStatus()

    @property
    def last_status(self) -> ServoPidStatus:
        return self._last_status

    @property
    def integral(self) -> Tuple[float, float]:
        """当前积分贡献 (m/s), 供诊断."""
        return (self._integral_fwd, self._integral_right)

    def reset(self) -> None:
        """
        全量重置 — 目标身份切换、重新捕获、状态机换相时调用.

        清空积分与微分历史, 使下一帧从纯 P 起步, 避免把上一个目标的
        积分偏置带到新目标上.
        """
        self._integral_fwd = 0.0
        self._integral_right = 0.0
        self._prev_error_fwd = None
        self._prev_error_right = None
        self._prev_stamp = None
        self._d_fwd = 0.0
        self._d_right = 0.0
        self._last_dt = 0.0

    def update(
        self,
        error_forward: float,
        error_right: float,
        stamp: float,
        *,
        measurement_valid: bool = True,
    ) -> Tuple[float, float]:
        """
        推进一个控制 tick.

        Parameters
        ----------
        error_forward, error_right : float
            当前误差, 单位由增益域决定. 已由调用方完成坐标变换;
            米制域应为已带符号的地面误差.
        stamp : float
            本帧测量的时间戳 (秒, 单调). 同一份测量重复调用时传入相同
            时间戳即可 — 积分与微分只在时间戳前进时推进.
        measurement_valid : bool
            本帧是否有有效测量. False 表示目标丢失: 输出零速度,
            冻结积分, 并重置微分历史 (恢复时不应把丢失前的斜率接上).

        Returns
        -------
        (forward, right) : Tuple[float, float]
            机体系水平速度指令 (m/s).

        """
        if not math.isfinite(error_forward) or not math.isfinite(error_right):
            # 非有限误差视为丢失, 走丢失分支, 绝不把 NaN 传入飞控
            measurement_valid = False
            error_forward = 0.0
            error_right = 0.0

        if not measurement_valid:
            self._prev_error_fwd = None
            self._prev_error_right = None
            self._d_fwd = 0.0
            self._d_right = 0.0
            self._last_status = ServoPidStatus(
                error_forward=error_forward,
                error_right=error_right,
                i_term=math.hypot(self._integral_fwd, self._integral_right),
                output_forward=0.0,
                output_right=0.0,
                in_deadzone=False,
                is_new_measurement=False,
                integrated=False,
            )
            return (0.0, 0.0)

        # 标定符号在这里一次性作用到误差上, 之后 P/I/D 三项都在同一个
        # 带符号误差上计算. 若改为只在 P/I 上乘符号, 当 sign=-1 时微分项
        # 会与比例项反号 —— 微分不再是阻尼而是正反馈, 会主动加剧振荡.
        e_fwd = self.sign_forward * float(error_forward)
        e_right = self.sign_right * float(error_right)

        # ---- 新测量判定与真实步长 ----
        is_new = self._prev_stamp is None or stamp > self._prev_stamp
        if is_new:
            dt = (stamp - self._prev_stamp) if self._prev_stamp is not None else 0.0
            self._prev_stamp = stamp
        else:
            dt = 0.0
        dt = dt if (dt > 0.0 and math.isfinite(dt)) else 0.0
        dt = min(dt, 2.0)  # 异常长间隔不作数, 防止一帧灌入过量积分

        mag = math.hypot(e_fwd, e_right)
        in_deadzone = mag < self.deadzone

        # ---- 死区: 零输出 + 冻结积分 (见模块 docstring 第 2 点) ----
        if in_deadzone:
            self._last_status = ServoPidStatus(
                error_forward=e_fwd, error_right=e_right,
                i_term=abs(self._integral_fwd),
                in_deadzone=True, is_new_measurement=is_new,
                integrated=False, dt_measurement=dt,
            )
            return (0.0, 0.0)

        # ---- 比例项 ----
        # 符号已在上面作用到 e_fwd/e_right, 此处不得再乘 sign_*.
        p_fwd = self.kp * e_fwd
        p_right = self.kp * e_right

        # ---- 微分项: 仅在新测量上求差, 再一阶低通 ----
        d_fwd = 0.0
        d_right = 0.0
        prev_fwd = self._prev_error_fwd
        prev_right = self._prev_error_right
        if (
            self.kd > 0.0 and dt > 0.0
            and prev_fwd is not None and prev_right is not None
        ):
            raw_d_fwd = self.kd * (e_fwd - prev_fwd) / dt
            raw_d_right = self.kd * (e_right - prev_right) / dt
            a = self.derivative_alpha
            self._d_fwd = a * raw_d_fwd + (1.0 - a) * self._d_fwd
            self._d_right = a * raw_d_right + (1.0 - a) * self._d_right
        elif dt == 0.0 and self._prev_error_fwd is None:
            self._d_fwd = 0.0
            self._d_right = 0.0
        d_fwd = self._d_fwd
        d_right = self._d_right

        if is_new:
            self._prev_error_fwd = e_fwd
            self._prev_error_right = e_right

        # ---- 饱和判定: 计入当前积分贡献 ----
        # 用 P+I+D 而非仅 P+D 判断, 使积分一旦把输出顶到上限就停止累加,
        # 即标准的钳位式抗饱和. 仅看 P+D 会让积分继续增长到硬限幅为止.
        unsaturated_fwd = p_fwd + self._integral_fwd + d_fwd
        unsaturated_right = p_right + self._integral_right + d_right
        saturated = math.hypot(unsaturated_fwd, unsaturated_right) >= self.max_speed

        # ---- 条件积分: 新测量 + 未饱和 + 误差在积分分离带内 ----
        integrate = (
            self.ki > 0.0
            and dt > 0.0
            and is_new
            and not saturated
            and (self.integral_band <= 0.0 or mag <= self.integral_band)
        )
        if integrate:
            self._integral_fwd += self.ki * e_fwd * dt
            self._integral_right += self.ki * e_right * dt
            self._integral_fwd = _clamp(self._integral_fwd, self.integral_limit)
            self._integral_right = _clamp(self._integral_right, self.integral_limit)

        out_fwd = p_fwd + self._integral_fwd + d_fwd
        out_right = p_right + self._integral_right + d_right

        # ---- 向量模长限幅 (比逐轴限幅更保守, 对角方向不再超速) ----
        norm = math.hypot(out_fwd, out_right)
        if norm > self.max_speed > 0.0:
            scale = self.max_speed / norm
            out_fwd *= scale
            out_right *= scale
            saturated = True
        elif 0.0 < norm < self.min_speed:
            # 死区补偿: 抬升到最小有效修正速度, 压过飞控响应死区
            scale = self.min_speed / norm
            out_fwd *= scale
            out_right *= scale

        self._last_status = ServoPidStatus(
            error_forward=e_fwd, error_right=e_right,
            p_term=math.hypot(p_fwd, p_right),
            i_term=math.hypot(self._integral_fwd, self._integral_right),
            d_term=math.hypot(d_fwd, d_right),
            output_forward=out_fwd, output_right=out_right,
            in_deadzone=False, saturated=saturated,
            is_new_measurement=is_new, integrated=integrate,
            dt_measurement=dt,
        )
        self._last_dt = dt
        return (out_fwd, out_right)


def _clamp(value: float, limit: float) -> float:
    """对称钳位. limit<=0 表示不限制."""
    if limit <= 0.0:
        return value
    return max(-limit, min(limit, value))
