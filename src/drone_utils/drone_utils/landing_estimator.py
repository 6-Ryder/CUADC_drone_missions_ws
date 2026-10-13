"""
降落目标的状态估计器 —— 权威的"微分项"来源与误检门.

为什么需要它
------------
权威（PX4 `landing_target_estimator` 与 ArduPilot `AC_PrecLand`）都不对原始
测量做低通、也不对像素误差求差:

  - PX4 每轴跑一个 2 状态 (位置, 速度) 常速卡尔曼
    (`LandingTargetEstimator.h:172-173`、`KalmanFilter.h:36-47`), **速度是该
    滤波器的状态**, 而不是两次测量相减 —— 这正是本项目一直缺的"微分项".
  - 离群拒绝用归一化新息平方 (卡方, 1 DOF) 门: `beta = r²/(P00+R)`,
    `beta > 3.84` 即 5% 误报, 该轴不融合 (`KalmanFilter.cpp:97-103`).
  - 测量方差随距离平方缩放 `R = LTEST_MEAS_UNC · dist_z²`
    (`LandingTargetEstimator.cpp:125`).
  - 掉线判据是"距最后一次**被接受**的测量"的龄期, 且持续被门拒绝的测量同样会
    走到复位 (`LandingTargetEstimator.cpp:76-80,120,180`).
  - ArduPilot 补了一条关键细节: 连续 3 次拒绝后**强制融合**, 否则滤波器在目标
    真的移动时会饿死 (`AC_PrecLand.cpp:574-583`).

本项目此前的替代品是"固定像素跳变阈值"(中心跳变 > 80px 即整帧丢弃), 它有两个
结构性缺陷: (a) 阈值不随高度缩放, 同一 80px 在 0.5m 是 10cm、在 5m 是 1m;
(b) 被拒帧被当作"目标丢失", 与权威"拒绝只跳过融合、龄期另行计"的语义相反.

本模块的建模选择
----------------
被估计量取**目标的世界位置 (NED)**, 不是"目标相对载机的机体系偏移". 两个理由:
  1. 降落目标是静止的 —— 世界位置是常量, 而机体系偏移随航向转动, 在后者的
     坐标系里做积分/滤波会把航向变化误读成目标运动.
  2. 载机自身的运动因此从"测量噪声"里被彻底移除: 观测 = 载机位置 + 机体系偏移
     (即 `lost_target_transit` 已经在算的那个量), 载机怎么飞都不影响目标位置.

过程噪声保留一个小量 (`acc_unc`), 使 `P` 在丢帧期间增长 —— 这正是权威"重捕获
瞬间门最松、收敛后收紧"的性质来源; 若取 0, 门会一直很紧, 重新捕获时会把正确
测量当成离群拒掉.
"""

import math
from typing import Optional, Tuple

# 卡方门限 (1 DOF, 5% 误报). 数值取自 PX4 KalmanFilter.cpp:97-103 的 3.84.
CHI2_GATE_95 = 3.84

# 连续拒绝多少次后强制融合. 取自 ArduPilot AC_PrecLand.cpp:574-583 ——
# 没有这条, 目标真的移动/初始估计偏时滤波器会被自己的门饿死.
FORCE_FUSE_AFTER_REJECTS = 3


class TargetAxisFilter:
    """
    单轴 2 状态 (位置, 速度) 常速卡尔曼, 结构与 PX4 `KalmanFilter.cpp` 同构.

    状态 x = [p, v]; A = [[1, dt], [0, 1]]; H = [1, 0].
    predict 支持控制输入 acc (PX4 用它喂载机加速度; 本模块估计的是静止目标的
    世界位置, 调用方传 0 即可, 但保留该入口以便结构和权威一致、将来可复用).
    """

    def __init__(self, gate: float = CHI2_GATE_95):
        self._gate = float(gate)
        self._p = 0.0
        self._v = 0.0
        self._p00 = 0.0
        self._p01 = 0.0
        self._p11 = 0.0
        self._initialized = False
        self._reject_streak = 0
        self._last_residual = 0.0
        self._last_innov_cov = 0.0

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def position(self) -> float:
        return self._p

    @property
    def velocity(self) -> float:
        return self._v

    @property
    def covariance(self) -> Tuple[float, float, float]:
        """(P00, P01, P11) — 供诊断与门限复算."""
        return (self._p00, self._p01, self._p11)

    @property
    def last_innovation(self) -> Tuple[float, float]:
        """(残差, 新息协方差) —— 二者之比即被检验的 beta, 供日志复盘."""
        return (self._last_residual, self._last_innov_cov)

    @property
    def reject_streak(self) -> int:
        return self._reject_streak

    def reset(self) -> None:
        self._initialized = False
        self._p = self._v = 0.0
        self._p00 = self._p01 = self._p11 = 0.0
        self._reject_streak = 0

    def init(
        self,
        position: float,
        velocity: float,
        pos_unc: float,
        vel_unc: float,
    ) -> None:
        """用一次测量播种 (与 PX4 KalmanFilter::init 同形)."""
        self._p = float(position)
        self._v = float(velocity)
        self._p00 = max(float(pos_unc), 0.0)
        self._p01 = 0.0
        self._p11 = max(float(vel_unc), 0.0)
        self._initialized = True
        self._reject_streak = 0

    def predict(self, dt: float, acc: float, acc_psd: float) -> None:
        """
        常速模型 + 白噪声加速度过程噪声.

        与 PX4 `KalmanFilter.cpp:69-86` 同构, 但过程噪声用**速率型 PSD**而不是
        per-step 方差: Q = q·[[dt³/3, dt²/2], [dt²/2, dt]], q 单位 m²/s³。

        为什么不用 PX4 旧式 Q = acc_unc·GGᵀ: 那个 acc_unc 是 per-step 量, 其
        效果随预测频率变化 —— 换个调用频率就等于换了滤波器带宽。PX4 自己在 VTE
        里就是因为这个缺陷改成了 rate-based PSD, 参数文档原文 "This is a variance
        rate, not a per-step variance; the value is independent of the filter
        update rate" (`vision_target_estimator_params.yaml`, VTE_ACC_D_UNC 描述)。
        本项目 predict 频率可能变 (20Hz tick / 未来别的节拍), 所以从一开始就用
        频率无关的形式。

        `acc` 保留为控制输入入口 (PX4 用它喂载机加速度, 那是"相对量"建模的需要);
        本模块估计的是静止目标的**世界位置**, 调用方传 0 即可。
        """
        if not self._initialized:
            return
        if not all(math.isfinite(v) for v in (dt, acc, acc_psd)):
            return
        if dt <= 0.0:
            return
        dt2 = dt * dt

        self._p += self._v * dt + 0.5 * dt2 * acc
        self._v += acc * dt

        # A P Aᵀ, A = [[1, dt], [0, 1]]
        p00 = self._p00 + 2.0 * dt * self._p01 + dt2 * self._p11
        p01 = self._p01 + dt * self._p11
        p11 = self._p11
        # Q = q * [[dt³/3, dt²/2], [dt²/2, dt]] (连续白噪声加速度的离散化)
        q = max(acc_psd, 0.0)
        dt3 = dt2 * dt
        self._p00 = p00 + q * dt3 / 3.0
        self._p01 = p01 + q * dt2 / 2.0
        self._p11 = p11 + q * dt

    def update(self, measurement: float, meas_unc: float) -> bool:
        """
        位置观测融合. 返回 False 表示被卡方门拒绝 (状态与协方差不动).

        连续 `FORCE_FUSE_AFTER_REJECTS` 次拒绝后强制融合 (ArduPilot 的做法).
        """
        if not self._initialized or not math.isfinite(measurement):
            return False
        meas_unc = max(float(meas_unc), 0.0) if math.isfinite(meas_unc) else 0.0

        residual = float(measurement) - self._p
        innov_cov = self._p00 + meas_unc
        self._last_residual = residual
        self._last_innov_cov = innov_cov

        if innov_cov > 0.0:
            beta = residual * residual / innov_cov
            if beta > self._gate and self._reject_streak < FORCE_FUSE_AFTER_REJECTS:
                self._reject_streak += 1
                return False

        self._reject_streak = 0
        if innov_cov <= 0.0:
            self._p = float(measurement)
            return True
        k0 = self._p00 / innov_cov
        k1 = self._p01 / innov_cov
        self._p += k0 * residual
        self._v += k1 * residual
        # P = (I - K H) P, H = [1, 0] → 减去 K · [P00, P01]
        p00, p01 = self._p00, self._p01
        self._p00 = p00 - k0 * p00
        self._p01 = p01 - k0 * p01
        self._p11 = self._p11 - k1 * p01
        return True


class LandingTargetEstimator:
    """
    静止目标的水平位置估计器: 每轴各一个 2 状态滤波器 (NED 北/东).

    观测 = 载机自身 NED 位置 + 机体系偏移 (前/右, 米) 旋转到 NED, 即
    "目标此刻看起来在哪". 载机的运动因此不进入被估计量.
    """

    def __init__(
        self,
        acc_psd: float = 0.01,
        pos_unc_init: float = 0.25,
        vel_unc_init: float = 0.25,
        vel_init: float = 0.0,
        meas_unc_tan2: float = 5e-4,
        min_frames: int = 5,
    ):
        self.acc_psd = float(acc_psd)
        self.pos_unc_init = float(pos_unc_init)
        self.vel_unc_init = float(vel_unc_init)
        self.vel_init = float(vel_init)
        self._meas_unc_tan2 = float(meas_unc_tan2)
        self.min_frames = max(int(min_frames), 0)
        self._north = TargetAxisFilter()
        self._east = TargetAxisFilter()
        self._accepted_count = 0
        self._rejected_count = 0
        self._last_obs_usable = False
        self._last_obs_accepted = False

    @property
    def initialized(self) -> bool:
        return self._north.initialized and self._east.initialized

    @property
    def last_observation_usable(self) -> bool:
        """上一次 observe 的输入是否可用 (非有限值一律不可用)."""
        return self._last_obs_usable

    @property
    def last_observation_accepted(self) -> bool:
        """上一次 observe 是否**被融合** (两轴都过门).

        ⚠️ 消费方要判"这一帧该不该采信", 必须用 (usable and not accepted) 这个
        组合, 而不是去看某一个轴的 reject_streak —— 那是逐轴的, 只看 north 会
        漏掉只影响 east 的误检 (u 方向跳变), 反之亦然。这个坑踩过一次。
        """
        return self._last_obs_accepted

    @property
    def accepted_count(self) -> int:
        """自上次 reset 以来被接受的观测帧数."""
        return self._accepted_count

    @property
    def rejected_count(self) -> int:
        """被卡方门拒绝的帧数 (供复盘: 拒绝对了=挡住误检, 拒错了=看得见却不接受)."""
        return self._rejected_count

    @property
    def is_mature(self) -> bool:
        """
        估计是否已"成熟"到可以驱动动作 (当前用于丢失后继续接近的落点).

        为什么需要这道门: 播种帧**不经过卡方门** (没有先验就无从算新息), 而播种
        协方差又给得较宽 (pos_unc_init), 于是头几帧实质上不过滤 —— 一次误检可以
        直接成为继续接近的飞行目标. 权威侧同样承认这个风险 (PX4 就是单帧播种),
        并把它列为已知缺口; ArduPilot 的对策是要求连续 2s 数据才置
        target_acquired (`AC_PrecLand.cpp:557-572`).

        本项目取**帧数**而非 2s: 一方面 3-5Hz 下 2s 约 6-10 帧, 与下面的默认值同阶;
        另一方面本项目播种用的观测**已经过置信度与跳变门**
        (state_precision_land*._get_stable_landing_target), 不是 PX4 那种裸帧,
        风险本就低一档, 故不必等满 2s.
        """
        return self._accepted_count >= self.min_frames

    @property
    def position(self) -> Optional[Tuple[float, float]]:
        if not self.initialized:
            return None
        return (self._north.position, self._east.position)

    @property
    def velocity(self) -> Tuple[float, float]:
        return (self._north.velocity, self._east.velocity)

    @property
    def filter_north(self) -> TargetAxisFilter:
        return self._north

    @property
    def filter_east(self) -> TargetAxisFilter:
        return self._east

    @classmethod
    def from_params(cls, fsm) -> "LandingTargetEstimator":
        """
        按 FSM 参数构造 —— 两条降落分支共用同一个入口, 免得只调一侧的数值.

        参数 (均在 mission_state_machine 声明):
          landing_estimator_meas_unc_tan2 — 角噪声方差 (tan²). 位置的测量方差
            由它乘高度平方得到 (R ∝ range², 见 PX4 LandingTargetEstimator.cpp:125).
            默认 5e-4 对应 σ_tan≈0.022, 即 f=400 时约 9px 的检测抖动.
          landing_estimator_acc_psd — 过程噪声速率 PSD (m²/s³). 它决定"丢线多久
            之后估计器愿意接受多大的目标位移": 丢线 T 秒后 P00 ≈ q·T³/3, 要接受
            位移 Δ 需 P00 ≥ Δ²/3.84。默认 0.01 由 Δ=0.3m、T=2s (与本项目降落
            目标 2s 龄期门同阶) 反推: q ≥ 3Δ²/(3.84·T³) ≈ 0.0088。**频率无关**
            (rate-based PSD), 改 predict 频率不必重算。
        """
        return cls(
            acc_psd=float(fsm.get_param("landing_estimator_acc_psd", 0.01)),
            pos_unc_init=float(fsm.get_param("landing_estimator_pos_unc_init", 0.25)),
            vel_unc_init=float(fsm.get_param("landing_estimator_vel_unc_init", 0.25)),
            meas_unc_tan2=float(
                fsm.get_param("landing_estimator_meas_unc_tan2", 5e-4)),
            min_frames=int(fsm.get_param("landing_estimator_min_frames", 5)),
        )

    def measurement_uncertainty(self, altitude_m: float) -> float:
        """位置测量方差 = 角噪声方差 × 高度² (权威的 R ∝ range²)."""
        if not math.isfinite(altitude_m) or altitude_m < 0.0:
            return float("inf")
        return self._meas_unc_tan2 * altitude_m * altitude_m

    def reset(self) -> None:
        self._north.reset()
        self._east.reset()
        self._accepted_count = 0
        self._rejected_count = 0

    @staticmethod
    def _to_ned(
        yaw: float, error_forward_m: float, error_right_m: float,
    ) -> Tuple[float, float]:
        """机体系 (前, 右) → NED 偏移, 与 flight_controller.set_velocity_body 同约定."""
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            error_forward_m * cos_yaw - error_right_m * sin_yaw,
            error_forward_m * sin_yaw + error_right_m * cos_yaw,
        )

    def observe(
        self,
        north: float,
        east: float,
        yaw: float,
        error_forward_m: float,
        error_right_m: float,
        meas_unc: float,
    ) -> bool:
        """
        融合一次观测. 返回 True 表示这一帧被接受 (两轴都过门).

        首次观测用于播种 (PX4 同样是一次测量初始化, 见
        `LandingTargetEstimator.cpp:112-121`), 其协方差由 pos_unc_init 给出.
        """
        if not all(math.isfinite(v) for v in
                   (north, east, yaw, error_forward_m, error_right_m)):
            self._last_obs_usable = False
            self._last_obs_accepted = False
            return False
        dn, de = self._to_ned(yaw, error_forward_m, error_right_m)
        target_north = float(north) + dn
        target_east = float(east) + de

        if not self.initialized:
            self._north.init(target_north, self.vel_init,
                             self.pos_unc_init, self.vel_unc_init)
            self._east.init(target_east, self.vel_init,
                            self.pos_unc_init, self.vel_unc_init)
            self._accepted_count += 1
            self._last_obs_usable = True
            self._last_obs_accepted = True
            return True

        ok_north = self._north.update(target_north, meas_unc)
        ok_east = self._east.update(target_east, meas_unc)
        accepted = ok_north and ok_east
        self._last_obs_usable = True
        self._last_obs_accepted = accepted
        if accepted:
            self._accepted_count += 1
        else:
            self._rejected_count += 1
        return accepted

    def predict(self, dt: float) -> None:
        """
        把估计推进到当前时刻 (目标静止, 故无控制输入 acc=0).

        ⚠️ **必须每个控制 tick 调用**. 不调用会有两个隐蔽后果, 都不是性能问题
        而是功能缺失:
          - 速度状态恒等于 vel_init: 单轴增益 k1 = P01/innov_cov, 而 P01 只被
            predict 抬起 (p01 += dt·p11), init 置 0、update 只收缩 —— 没有
            predict 则 k1≡0, 速度永不更新, "权威的微分项"运行时不成立。
          - P00 只降不升: 于是"丢线期间 P 增长 → 重捕获瞬间门变松"这条性质在
            运行时不存在, 长丢线后目标真移动会被连拒。
        """
        self._north.predict(dt, 0.0, self.acc_psd)
        self._east.predict(dt, 0.0, self.acc_psd)


def _observe(
    estimator: LandingTargetEstimator,
    north: float,
    east: float,
    yaw: float,
    error_forward_m: float,
    error_right_m: float,
    meas_unc: float = 0.001,
) -> bool:
    """测试/脚本用的薄包装: 用固定测量方差调 observe.

    生产代码里 meas_unc 由 measurement_uncertainty(altitude) 按高度算出;
    本包装只服务于"给定观测序列"的验证脚本, 不参与飞行路径.
    """
    return estimator.observe(north, east, yaw, error_forward_m, error_right_m,
                             meas_unc)
