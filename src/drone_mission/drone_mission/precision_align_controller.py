"""
CUADC-style low-altitude precision alignment.

This controller aligns the selected target from the current camera image.
It uses normalized pixel error, EMA smoothing, shared ServoPid body-velocity
control (gain contract in config/pid_tuning.yaml), continuous or short
step/stop commands, and stable-frame confirmation.
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional, Tuple

from drone_utils.servo_pid import ServoPid


class PrecisionAlignResult(Enum):
    CONTINUE = "continue"
    CONVERGED = "converged"
    TARGET_LOST = "target_lost"


@dataclass
class PrecisionAlignStatus:
    source: str = "none"
    pixel_error: float = float("inf")
    smooth_pixel_error: float = float("inf")
    raw_dx_norm: float = 0.0
    raw_dy_norm: float = 0.0
    smooth_dx_norm: float = 0.0
    smooth_dy_norm: float = 0.0
    stable_count: int = 0
    lost_frames: int = 0
    target_world_xy: Optional[Tuple[float, float]] = None
    command_forward: float = 0.0
    command_right: float = 0.0
    ground_forward_error_m: float = 0.0
    ground_right_error_m: float = 0.0
    altitude_m: float = 0.0
    continuous: bool = False
    release_ready: bool = False
    reason: str = ""


class PrecisionAlignController:
    """
    Low-altitude visual alignment controller.

    The caller provides current marker/cylinder detections each control tick.
    The controller chooses and tracks one target, computes a smoothed normalized
    image error, and sends short body-frame velocity commands through fsm.fc.
    """

    def __init__(self, fsm):
        self._fsm = fsm
        self._target_hint_xy: Optional[Tuple[float, float]] = None
        self._locked_pixel_uv: Optional[Tuple[float, float]] = None
        self._locked_world_xy: Optional[Tuple[float, float]] = None
        self._locked_source: str = "none"
        self._smooth_dx: Optional[float] = None
        self._smooth_dy: Optional[float] = None
        self._stable_count = 0
        self._lost_frames = 0
        self._active_until = 0.0
        self._hover_until = 0.0
        self._brake_until = 0.0
        self._release_ready = False
        self._last_forward = 0.0
        self._last_right = 0.0
        self._last_log_time = 0.0
        self._last_status = PrecisionAlignStatus()
        self._active_pid: Optional[ServoPid] = None  # 最近一次出速度指令的 PID, 供日志诊断

        # ── 共享伺服 PID (参数契约: config/pid_tuning.yaml) ──
        # 两条输出分支的误差域不同, 各用一套增益, 不再有独立的纯 P 增益:
        #   连续模式 (_pid_continuous): 误差 = _ground_error_m 输出的地面误差
        #                               (m, 已带符号), 米制域增益 servo_kp/ki/kd
        #   步进模式 (_pid_norm):       误差 = _smooth_dx/_smooth_dy
        #                               (归一化图像比例), 归一化域增益 servo_norm_*
        # 两域共用抗饱和与微分滤波参数. deadzone 保持 0: 对心判决由 _is_precise
        # 的多重门槛负责, 死区停发指令会与 stable_frames 计数语义打架.
        pid_common = {
            "min_speed": float(self._init_param("servo_min_speed", 0.012)),
            "integral_limit": float(self._init_param("servo_integral_limit", 0.10)),
            "derivative_alpha": float(self._init_param("servo_derivative_alpha", 0.30)),
            "integral_band": float(self._init_param("servo_integral_band", 0.0)),
        }
        servo_max_speed = float(self._init_param("servo_max_speed", 0.25))
        self._pid_continuous = ServoPid(
            kp=float(self._init_param("servo_kp", 0.40)),
            ki=float(self._init_param("servo_ki", 0.08)),
            kd=float(self._init_param("servo_kd", 0.05)),
            max_speed=servo_max_speed,
            **pid_common,
        )
        self._pid_norm = ServoPid(
            kp=float(self._init_param("servo_norm_kp", 1.25)),
            ki=float(self._init_param("servo_norm_ki", 0.25)),
            kd=float(self._init_param("servo_norm_kd", 0.15)),
            max_speed=servo_max_speed,
            **pid_common,
        )

    @property
    def last_status(self) -> PrecisionAlignStatus:
        return self._last_status

    def reset(self, target_hint_xy: Optional[Tuple[float, float]] = None,
              target_hint_uv: Optional[Tuple[float, float]] = None) -> None:
        self._target_hint_xy = target_hint_xy or self._selected_memory_xy()
        self._locked_pixel_uv = target_hint_uv
        self._locked_world_xy = None
        self._locked_source = "none"
        self._smooth_dx = None
        self._smooth_dy = None
        self._stable_count = 0
        self._lost_frames = 0
        self._active_until = 0.0
        self._hover_until = 0.0
        self._brake_until = 0.0
        self._release_ready = False
        self._last_forward = 0.0
        self._last_right = 0.0
        self._last_log_time = 0.0
        self._last_status = PrecisionAlignStatus(reason="reset")
        # 重新捕获 = 目标身份可能改变, 积分/微分历史必须清空 (见 ServoPid.reset)
        self._pid_continuous.reset()
        self._pid_norm.reset()
        self._active_pid = None

    def reset_confirmation(self) -> None:
        """
        仅重置确认状态 (稳定计数/刹车/投放许可).

        保留目标像素锁定和 EMA 平滑值, 使高度变化后无需冷启动
        重新收敛, 仅需在新高度重新确认像素居中。

        PID 状态同样刻意保留: 积分偏置是机体系速度偏置, 对抗的是风/重心/
        相机安装倾角这类与高度无关的常值扰动 (高度对 P 增益的影响已由
        _ground_error_m 按当前高度换算误差抵消), 清掉反而要在新高度重新
        收敛一遍.
        """
        self._stable_count = 0
        self._release_ready = False
        self._brake_until = 0.0
        self._active_until = 0.0
        self._hover_until = 0.0
        self._last_forward = 0.0
        self._last_right = 0.0
        self._last_status = PrecisionAlignStatus(reason="altitude_change")

    @staticmethod
    def _field(obj: Any, key: str, default=0.0):
        return getattr(obj, key, obj.get(key, default) if isinstance(obj, dict) else default)

    def _world_xy(self, target: Any) -> Optional[Tuple[float, float]]:
        if not bool(self._field(target, "world_position_valid", False)):
            return None
        wx = float(self._field(target, "world_x", math.nan))
        wy = float(self._field(target, "world_y", math.nan))
        return (wx, wy) if math.isfinite(wx) and math.isfinite(wy) else None

    def _now(self) -> float:
        return self._fsm.get_clock().now().nanoseconds / 1e9

    def _param(self, key: str, default):
        return self._fsm.get_param(key, default)

    def _init_param(self, key: str, default):
        """构造期读参: fsm 是缺少 get_param 的桩对象时退回默认值.

        构造函数原本完全不碰 fsm, 有些调用方 (如 CylinderApproachState(object()))
        只传空桩; 只有 step() 才真正依赖 fsm 接口. 这里保持同样的宽容度:
        拿不到参数源就用默认值, 真实 fsm 行为不变.
        """
        getter = getattr(self._fsm, "get_param", None)
        if not callable(getter):
            return default
        return getter(key, default)

    def _selected_memory_xy(self) -> Optional[Tuple[float, float]]:
        selected_positions = getattr(self._fsm.cylinder_selector, "_selected_positions", [])
        if not selected_positions:
            return None
        point = selected_positions[-1]
        return point if point is not None and all(math.isfinite(v) for v in point) else None

    def _camera_geometry(self) -> Tuple[float, float, float, float]:
        cx = float(self._param("camera_cx", 640.0))
        cy = float(self._param("camera_cy", 480.0))
        width = float(self._param("precision_align_image_width", max(cx * 2.0, 1.0)))
        height = float(self._param("precision_align_image_height", max(cy * 2.0, 1.0)))
        return cx, cy, max(width, 1.0), max(height, 1.0)

    def _raw_tolerance(self, drop_height: float) -> float:
        """两级对心窗口: 低于 tight_window_height 用投放段收紧窗口, 否则用精校段窗口.

        SITL 未配置 tight_window_height (默认 0.0) → 切换高度不可达,
        永远用精校段窗口 (raw_pixel_tolerance), 行为不变.
        """
        tight_h = float(self._param("precision_align_tight_window_height", 0.0))
        if drop_height < tight_h:
            return float(self._param("precision_align_final_raw_pixel_tolerance", 30.0))
        return float(self._param("precision_align_raw_pixel_tolerance", 9.0))

    def _ground_error_m(
        self, du: float, dv: float, altitude_m: float,
    ) -> Tuple[float, float]:
        """Return signed (forward, right) target error in metres.

        Pixel offsets grow as the aircraft descends.  Converting them with
        the current altitude keeps the physical P-controller gain constant
        throughout quick drop descent.
        """
        fx = max(float(self._param("camera_fx", 400.0)), 1.0)
        fy = max(float(self._param("camera_fy", 400.0)), 1.0)
        altitude = max(float(altitude_m), 0.05)
        y_to_forward_sign = float(
            self._param("precision_align_image_y_to_forward_sign", -1.0)
        )
        return (
            y_to_forward_sign * float(dv) * altitude / fy,
            float(du) * altitude / fx,
        )

    def step(
        self,
        markers: list,
        cylinders: list,
        drop_height: float,
        *,
        altitude_m: Optional[float] = None,
        descent_speed: float = 0.0,
        allow_release: bool = True,
    ) -> PrecisionAlignResult:
        """Run one alignment tick."""
        now = self._now()
        continuous = (
            bool(self._param("precision_align_continuous_enable", False))
            and altitude_m is not None
        )
        altitude = float(altitude_m) if altitude_m is not None else 0.0

        if not continuous and now < self._active_until:
            self._fsm.fc.set_velocity_body(self._last_forward, self._last_right, 0.0)
            return PrecisionAlignResult.CONTINUE

        if not continuous and self._active_until > 0.0:
            self._fsm.fc.stop_velocity()
            self._active_until = 0.0
            self._hover_until = now + float(self._param("precision_align_hover_s", 0.04))
            return PrecisionAlignResult.CONTINUE

        if not continuous and now < self._hover_until:
            self._fsm.fc.stop_velocity()
            return PrecisionAlignResult.CONTINUE

        if now < self._brake_until:
            if continuous:
                self._fsm.fc.set_velocity_body(0.0, 0.0, 0.0)
            else:
                self._fsm.fc.stop_velocity()
            return PrecisionAlignResult.CONTINUE

        target, source, reason = self._choose_target(markers, cylinders)
        if target is None:
            self._stable_count = 0
            self._brake_until = 0.0
            self._release_ready = False
            self._lost_frames += 1
            self._last_status = PrecisionAlignStatus(
                source=source,
                stable_count=0,
                lost_frames=self._lost_frames,
                reason=reason,
            )
            self._fsm.fc.stop_velocity()
            # 丢帧期间不喂新误差. 本分支已经 stop_velocity 并 return, 天然不经过
            # PID; 这里仍显式传 measurement_valid=False, 作用是 (1) 清空微分历史,
            # 恢复时不会把丢失前的斜率接上 (2) 把"本帧无有效测量"的语义落到 PID
            # 内部. 积分按设计冻结不清零, 保留对抗常值扰动的偏置.
            self._pid_continuous.update(0.0, 0.0, now, measurement_valid=False)
            self._pid_norm.update(0.0, 0.0, now, measurement_valid=False)
            self._maybe_log_missing()

            reacquire_after = int(self._param("precision_align_reacquire_after_lost", 6))
            if self._lost_frames == reacquire_after:
                self._locked_pixel_uv = None
                self._smooth_dx = None
                self._smooth_dy = None
                # 像素锁定已清除 = 下一帧按画面中心重新选目标 (目标身份切换),
                # 积分/微分从头起步, 不把旧目标的偏置带到新目标上.
                self._pid_continuous.reset()
                self._pid_norm.reset()
                self._fsm.get_logger().warn(
                    "PREC_ALIGN: 目标短暂丢失, 清除像素锁定并按画面中心重捕获"
                )

            max_lost = int(self._param("precision_align_max_lost_frames", 32))
            if self._lost_frames > max_lost:
                return PrecisionAlignResult.TARGET_LOST
            return PrecisionAlignResult.CONTINUE

        self._lost_frames = 0
        cx, cy, width, height = self._camera_geometry()
        u = float(self._field(target, "center_u", cx))
        v = float(self._field(target, "center_v", cy))
        if self._param("quick_drop_profile", "blue") == "bucket" and altitude > 0:
            # Aim the body behind the bucket by the measured front-bottle offset.
            plane_distance = max(
                altitude + float(self._param("bucket_home_body_height", 0.0))
                - float(self._param("bucket_camera_down", 0.0))
                - float(self._param("bucket_rim_height", 0.3)), 0.3)
            y_sign = float(self._param("precision_align_image_y_to_forward_sign", -1.0))
            if abs(y_sign) > 1e-6:
                v -= (float(self._param("bucket_front_forward", 0.04))
                      * float(self._param("camera_fy", 400.0)) / plane_distance / y_sign)
            u -= (float(self._param("bucket_front_right", 0.0))
                  * float(self._param("camera_fx", 400.0)) / plane_distance)
        raw_dx = (u - cx) / width
        raw_dy = (v - cy) / height
        pixel_error = math.hypot(u - cx, v - cy)

        alpha = max(0.01, min(float(self._param("precision_align_alpha", 0.55)), 1.0))
        if self._smooth_dx is None or self._smooth_dy is None:
            self._smooth_dx = raw_dx
            self._smooth_dy = raw_dy
        else:
            self._smooth_dx = alpha * raw_dx + (1.0 - alpha) * self._smooth_dx
            self._smooth_dy = alpha * raw_dy + (1.0 - alpha) * self._smooth_dy

        smooth_pixel_error = math.hypot(self._smooth_dx * width, self._smooth_dy * height)
        raw_tol = self._raw_tolerance(drop_height)
        aligned = self._is_precise(pixel_error, smooth_pixel_error, raw_dx, raw_dy, raw_tol)
        if allow_release:
            self._stable_count = self._stable_count + 1 if aligned else 0
        else:
            self._stable_count = 0
        if not aligned or not allow_release:
            self._brake_until = 0.0
            self._release_ready = False

        smooth_du = float(self._smooth_dx) * width
        smooth_dv = float(self._smooth_dy) * height
        plane_altitude = (max(
                            altitude + float(self._param("bucket_home_body_height", 0.0))
                            - float(self._param("bucket_camera_down", 0.0))
                            - float(self._param("bucket_rim_height", 0.3)), 0.3)
                          if self._param("quick_drop_profile", "blue") == "bucket"
                          else altitude)
        ground_forward_error_m, ground_right_error_m = self._ground_error_m(
            smooth_du, smooth_dv, plane_altitude,
        ) if continuous else (0.0, 0.0)

        target_world_xy = self._world_xy(target)
        self._last_status = PrecisionAlignStatus(
            source=source,
            pixel_error=pixel_error,
            smooth_pixel_error=smooth_pixel_error,
            raw_dx_norm=raw_dx,
            raw_dy_norm=raw_dy,
            smooth_dx_norm=float(self._smooth_dx),
            smooth_dy_norm=float(self._smooth_dy),
            stable_count=self._stable_count,
            lost_frames=0,
            target_world_xy=target_world_xy,
            ground_forward_error_m=ground_forward_error_m,
            ground_right_error_m=ground_right_error_m,
            altitude_m=altitude,
            continuous=continuous,
            release_ready=False,
            reason=reason,
        )

        stable_frames = int(self._param("precision_align_stable_frames", 5))
        if allow_release and self._stable_count >= stable_frames:
            self._fsm.fc.stop_velocity()
            if not self._release_ready and self._brake_until <= 0.0:
                brake_s = float(self._param("precision_align_final_brake_s", 0.5))
                self._brake_until = now + max(brake_s, 0.0)
                self._last_status.reason = "final_brake"
                self._fsm.get_logger().info(
                    "PREC_ALIGN: 进入最终刹车复查 "
                    f"source={source} raw={pixel_error:.1f}px smooth={smooth_pixel_error:.1f}px "
                    f"stable={self._stable_count}/{stable_frames}"
                )
                return PrecisionAlignResult.CONTINUE

            if self._horizontal_speed_ok():
                self._release_ready = True
                self._last_status.release_ready = True
                self._last_status.reason = "ready"
                self._fsm.get_logger().info(
                    "PREC_ALIGN: 视觉精校稳定, 允许投放 "
                    f"source={source} raw={pixel_error:.1f}px smooth={smooth_pixel_error:.1f}px "
                    f"stable={self._stable_count}/{stable_frames}"
                )
                return PrecisionAlignResult.CONVERGED

            self._last_status.reason = "waiting_velocity_settle"
            self._maybe_log_tracking()
            return PrecisionAlignResult.CONTINUE

        # The release tolerance is deliberately not a descent deadband.  A
        # 40 px offset can be acceptable at the release height yet represent a
        # much larger ground offset higher up, so keep horizontal correction
        # active until the height handoff.
        if continuous and not allow_release:
            forward, right = self._continuous_command(
                ground_forward_error_m, ground_right_error_m, now, altitude,
            )
            self._last_forward = forward
            self._last_right = right
            self._last_status.command_forward = forward
            self._last_status.command_right = right
            self._last_status.reason = "descending_tracking"
            self._maybe_log_tracking()
            self._fsm.fc.set_velocity_body(forward, right, descent_speed)
            return PrecisionAlignResult.CONTINUE

        if aligned:
            self._fsm.fc.stop_velocity()
            self._last_status.reason = "collecting_stable_frames"
            return PrecisionAlignResult.CONTINUE

        if continuous:
            forward, right = self._continuous_command(
                ground_forward_error_m, ground_right_error_m, now, altitude,
            )
        else:
            # 步进模式: 归一化域 PID. 图像方向→机体方向的标定符号在这里预先
            # 作用到误差上 (与米制域一致: PID 收到的误差已带符号, sign_* 保持
            # +1), 不交给 ServoPid 的 sign_forward/sign_right —— 那两个只乘在
            # P/I 上, 微分项不乘, 符号为 -1 时会把 D 项反号.
            x_to_right_sign = float(self._param("precision_align_image_x_to_right_sign", 1.0))
            y_to_forward_sign = float(self._param("precision_align_image_y_to_forward_sign", -1.0))
            pid = self._pid_norm
            # 归一化域两级限速 (恢复原 _limit_vector 非连续分支的行为):
            # 精调段 (px 误差 ≤ fine 阈值) 用 precision_align_fine_max_speed,
            # 其余用 servo_max_speed (原 precision_align_max_speed). 精调限速是
            # 近地精调的安全限速, 与 profile 无关, 不做 bucket 分层.
            if pixel_error <= float(self._param("precision_align_fine_pixel_error", 90.0)):
                pid.max_speed = float(self._param("precision_align_fine_max_speed", 0.065))
            else:
                pid.max_speed = float(self._param("servo_max_speed", 0.25))
            forward, right = pid.update(
                y_to_forward_sign * float(self._smooth_dy),
                x_to_right_sign * float(self._smooth_dx),
                now,
            )
            self._active_pid = pid

        self._last_forward = forward
        self._last_right = right
        self._last_status.command_forward = forward
        self._last_status.command_right = right
        self._maybe_log_tracking()

        if continuous:
            self._fsm.fc.set_velocity_body(forward, right, descent_speed)
            return PrecisionAlignResult.CONTINUE

        if pixel_error <= float(self._param("precision_align_fine_pixel_error", 90.0)):
            step_s = float(self._param("precision_align_fine_step_s", 0.12))
        else:
            step_s = float(self._param("precision_align_step_s", 0.18))
        self._fsm.fc.set_velocity_body(forward, right, 0.0)
        self._active_until = now + step_s
        return PrecisionAlignResult.CONTINUE

    def _is_precise(
        self,
        raw_pixel_error: float,
        smooth_pixel_error: float,
        raw_dx: float,
        raw_dy: float,
        raw_tol: float,
    ) -> bool:
        smooth_tol = float(self._param("precision_align_smooth_pixel_tolerance", 7.0))
        threshold_x = float(self._param("precision_align_threshold_norm_x", 0.008))
        threshold_y = float(self._param("precision_align_threshold_norm_y", 0.008))
        return (
            raw_pixel_error <= raw_tol
            and smooth_pixel_error <= smooth_tol
            and abs(raw_dx) <= threshold_x
            and abs(raw_dy) <= threshold_y
        )

    def _continuous_max_speed(self, altitude_m: float) -> float:
        """连续模式水平合速度上限 (m/s), 含 bucket 低空分层限速.

        原 _limit_vector 的固定上限 precision_align_max_speed 已由
        servo_max_speed 取代; bucket 分层 (低空降速) 原样保留:
        1.9m 及以下取 precision_align_fine_max_speed,
        3.0m 及以下取 bucket_servo_mid_max_speed, 取两者与 servo_max_speed
        的较小值.
        """
        max_speed = float(self._param("servo_max_speed", 0.25))
        if self._param("quick_drop_profile", "blue") == "bucket":
            if altitude_m <= 1.9:
                max_speed = min(max_speed, float(self._param(
                    "precision_align_fine_max_speed", 0.065)))
            elif altitude_m <= 3.0:
                max_speed = min(max_speed, float(self._param(
                    "bucket_servo_mid_max_speed", 0.14)))
        return max_speed

    def _continuous_command(
        self,
        error_forward_m: float,
        error_right_m: float,
        stamp: float,
        altitude_m: float,
    ) -> Tuple[float, float]:
        """连续模式: 米制域 PID, 每 tick 刷新一次限速上限.

        限幅 (向量模长) 与最小修正速度抬升都在 ServoPid 内部完成, 这里只负责
        把按高度算出的 max_speed 交给它 —— bucket 分层限速因此得以保留.
        """
        pid = self._pid_continuous
        pid.max_speed = self._continuous_max_speed(altitude_m)
        command = pid.update(error_forward_m, error_right_m, stamp)
        self._active_pid = pid
        return command

    def _horizontal_speed_ok(self) -> bool:
        max_speed = float(self._param("precision_align_release_max_horizontal_speed", 0.04))
        bridge = getattr(self._fsm, "bridge", None)
        get_telemetry = getattr(bridge, "get_telemetry", None)
        if not callable(get_telemetry):
            # 无法获取遥测时不能假设速度为零, 延迟此帧等遥测就绪
            return False
        try:
            velocity = get_telemetry().velocity
            speed_xy = math.hypot(float(velocity.vn), float(velocity.ve))
        except Exception:
            return False
        if speed_xy <= max_speed:
            return True
        self._fsm.fc.stop_velocity()
        self._fsm.get_logger().info(
            "PREC_ALIGN: 等待水平速度归零 "
            f"speed_xy={speed_xy:.3f}m/s max={max_speed:.3f}m/s"
        )
        return False

    def _choose_target(self, markers: list, cylinders: list):
        source_preference = str(self._param("precision_align_target_source", "cylinder")).lower()
        bucket_mode = self._param("quick_drop_profile", "blue") == "bucket"
        if bucket_mode:
            source_preference = "bucket"
        min_conf = float(self._param("precision_align_min_confidence", 0.25))

        marker_candidates = [
            (m, "marker")
            for m in markers
            if float(self._field(m, "confidence", 0.0)) >= min_conf
        ]
        cylinder_candidates = [
            (c, "cylinder")
            for c in cylinders
            if float(self._field(c, "confidence", 0.0)) >= min_conf
        ]

        if source_preference == "bucket":
            candidates = cylinder_candidates
        elif source_preference == "marker":
            candidates = marker_candidates
        elif source_preference == "cylinder":
            candidates = cylinder_candidates
        else:
            candidates = marker_candidates + cylinder_candidates
        if not candidates:
            return None, source_preference, "no_candidates"

        cx, cy, width, height = self._camera_geometry()
        diag = max(math.hypot(width, height), 1.0)
        # 记忆世界坐标门控开关 (默认关闭 = 一切以视觉像素为准).
        # 开启后, 未锁定时距搜索期记忆点超过 world_gate 的候选直接拒绝,
        # 距离还作为弱项参与评分 (权重 0.08).
        world_gate_enabled = (not bucket_mode and bool(
            self._param("precision_align_world_gate_enable", False)
        ))
        world_gate = float(
            self._param("precision_align_target_world_gate", 1.5)
        )
        jump_gate = float(self._param("precision_align_max_jump_diag_ratio", 0.45))
        center_reject = float(self._param("precision_align_reject_center_ratio", 0.28))

        scored = []
        for target, source in candidates:
            u = float(self._field(target, "center_u", cx))
            v = float(self._field(target, "center_v", cy))
            center_ratio = math.hypot(u - cx, v - cy) / diag

            # 记忆世界坐标门控 (默认关闭, 见上方 world_gate_enabled).
            # 关闭时 world_ratio 恒为 0, 评分式中的 0.08 项不产生任何影响.
            world_ratio = 0.0
            if (
                world_gate_enabled
                and self._target_hint_xy is not None
                and self._locked_pixel_uv is None
            ):
                world_xy = self._world_xy(target)
                if world_xy is None:
                    world_ratio = 1.0
                else:
                    hint_x, hint_y = self._target_hint_xy
                    world_error = math.hypot(
                        world_xy[0] - hint_x, world_xy[1] - hint_y,
                    )
                    if world_error > world_gate:
                        continue
                    world_ratio = min(
                        world_error / max(world_gate, 0.01), 1.0,
                    )

            continuity_ratio = center_ratio
            if self._locked_pixel_uv is not None:
                lu, lv = self._locked_pixel_uv
                continuity_ratio = math.hypot(u - lu, v - lv) / diag
                if continuity_ratio > jump_gate and center_ratio > center_reject:
                    continue

            # CUADC-style locked tracking: pixel continuity dominates; center
            # distance helps reacquire after altitude changes or short loss.
            #
            # 评分权重设计原则 (越小越好):
            #   0.68 continuity_ratio  — 主项: 同一目标的帧间像素位移, 防止锁定跳变
            #   0.32 center_ratio       — 辅项: 距画面中心的归一化距离, 倾向选居中的目标
            #   0.08 world_ratio        — 弱项: 距预期 World 位置的距离, 仅在门控
            #                             开启且未锁定时非零 (默认关闭 → 恒为 0)
            #  -0.04 confidence         — 微调: 轻微偏好高置信度 (负权重→置信度越高 score 越低)
            #
            # continuity_ratio 占主导 (~65%) 确保锁定的目标不会被画面边缘的新目标"抢走".
            score = (
                0.68 * continuity_ratio
                + 0.32 * center_ratio
                + 0.08 * world_ratio
                - 0.04 * float(self._field(target, "confidence", 0.0))
            )
            scored.append((score, target, source))

        if not scored:
            return None, source_preference, "all_candidates_rejected"

        _, selected, source = min(scored, key=lambda item: item[0])
        u = float(self._field(selected, "center_u", cx))
        v = float(self._field(selected, "center_v", cy))
        selected_world_xy = self._world_xy(selected)

        if self._locked_pixel_uv is None:
            world_label = (
                f"({selected_world_xy[0]:.2f},{selected_world_xy[1]:.2f})"
                if selected_world_xy is not None else "invalid"
            )
            self._fsm.get_logger().info(
                "PREC_ALIGN: 锁定低空精校目标 "
                f"source={source} pixel=({u:.0f},{v:.0f}) world={world_label}"
            )

        self._locked_pixel_uv = (u, v)
        self._locked_world_xy = selected_world_xy
        self._locked_source = source
        return selected, source, "tracking"

    def _maybe_log_tracking(self) -> None:
        now = self._now()
        interval = float(self._param("precision_align_log_interval", 0.5))
        if now - self._last_log_time < interval:
            return
        self._last_log_time = now
        s = self._last_status
        world_label = (
            f"({s.target_world_xy[0]:.2f},{s.target_world_xy[1]:.2f})"
            if s.target_world_xy is not None else "invalid"
        )
        # PID 分项 (P/I/D 各为双轴合矢量模长, 单位 m/s) + 是否饱和:
        # 真机调参时据此判断是比例、积分还是微分在主导当前指令.
        pid_label = ""
        if self._active_pid is not None:
            p = self._active_pid.last_status
            pid_label = (
                f" pid=(p{p.p_term:.3f},i{p.i_term:.3f},d{p.d_term:.3f})"
                f" sat={int(p.saturated)}"
            )
        self._fsm.get_logger().info(
            "PREC_ALIGN: "
            f"source={s.source} pixel_error={s.pixel_error:.1f}px "
            f"raw=({s.raw_dx_norm:+.4f},{s.raw_dy_norm:+.4f}) "
            f"smooth=({s.smooth_dx_norm:+.4f},{s.smooth_dy_norm:+.4f}) "
            f"world={world_label} "
            f"ground=({s.ground_forward_error_m:+.3f},{s.ground_right_error_m:+.3f})m "
            f"alt={s.altitude_m:.2f}m continuous={s.continuous} "
            f"stable={s.stable_count} cmd=({s.command_forward:+.3f},{s.command_right:+.3f})"
            f"{pid_label}"
        )

    def _maybe_log_missing(self) -> None:
        now = self._now()
        interval = float(self._param("precision_align_log_interval", 0.5))
        if now - self._last_log_time < interval:
            return
        self._last_log_time = now
        self._fsm.get_logger().warn(
            "PREC_ALIGN: 未见可用目标 "
            f"lost={self._lost_frames} reason={self._last_status.reason}"
        )
