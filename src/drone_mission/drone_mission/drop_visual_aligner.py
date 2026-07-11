"""低空投放视觉校准.

该模块只负责 DESCEND_TO_DROP 阶段的最后校准:
  不放先算方向、不依赖 fx/fy/alt/gain 参数。
  每一轮在 X/Y 轴上做 ± 探测步, 直接观察 pixel_error 变化,
  选择使像素误差减小的方向移动。本质是 2D 像素空间的梯度下降。

高空搜索/目标选择仍由 MissionFSM + CylinderSelector 负责。
"""

import math
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Optional, Tuple

from px4_control.telemetry import ned_to_world, world_to_ned


class ServoPhase(IntEnum):
    PROBE_X_PLUS = 0    # 探测 +X
    PROBE_X_MINUS = 1   # 探测 -X (从 +X 位置跳过去)
    PROBE_Y_PLUS = 2    # 探测 +Y
    PROBE_Y_MINUS = 3   # 探测 -Y
    COMMIT_AXIS = 4     # 提交当前轴的最佳方向
    BRAKING = 5         # 刹车悬停, 等待稳定对齐


@dataclass
class VisualAlignStatus:
    """低空视觉校准当前状态."""

    source: str = "none"
    pixel_error: float = float("inf")
    pixel_du: float = 0.0
    pixel_dv: float = 0.0
    world_error_to_memory: float = float("inf")
    target_world_xy: Optional[Tuple[float, float]] = None
    stable_count: int = 0


class DropVisualAligner:
    """投放前低空视觉精修控制器.

    每轮迭代:
      1. PROBE_X: 从锚点尝试 ±WorldX, 取 pixel_error 更小的方向提交
      2. PROBE_Y: 从提交后的位置尝试 ±WorldY, 同理
      3. 如果 pixel_error < brake_threshold → BRAKING 悬停等 is_centered
      4. 重复直到居中或超时
    """

    PROBE_MOVE_TIMEOUT = 1.2   # 探测移动 + 稳定等待时间 (s)
    BRAKE_CHECK_INTERVAL = 0.3  # 刹车区像素采样间隔 (s)

    def __init__(self, fsm):
        self._fsm = fsm
        self._stable_count = 0
        self._last_log_time = 0.0
        self._last_status = VisualAlignStatus()

        # 探测状态 — 端点比较法, 不回锚点
        self._phase: ServoPhase = ServoPhase.PROBE_X_PLUS
        self._working_axis_x: bool = True
        self._baseline_n: float = 0.0
        self._baseline_e: float = 0.0
        self._baseline_pixel_error: float = float("inf")
        self._baseline_source: str = "none"   # 锁定目标身份, 防止探测中途切换
        self._probe_start_time: float = 0.0
        self._endpoint_plus_error: float = float("inf")
        self._endpoint_minus_error: float = float("inf")

        # 刹车
        self._brake_enter_time: float = 0.0
        self._brake_last_check: float = 0.0

        # 自适应步长
        self._step_scale: float = 1.0  # 1.0=大步, 0.5=中步, 0.25=小步

    def reset(self) -> None:
        self._stable_count = 0
        self._last_log_time = 0.0
        self._last_status = VisualAlignStatus()
        self._phase = ServoPhase.PROBE_X_PLUS
        self._working_axis_x = True
        self._baseline_n = 0.0
        self._baseline_e = 0.0
        self._baseline_pixel_error = float("inf")
        self._baseline_source = "none"
        self._probe_start_time = 0.0
        self._endpoint_plus_error = float("inf")
        self._endpoint_minus_error = float("inf")
        self._brake_enter_time = 0.0
        self._brake_last_check = 0.0
        self._step_scale = 1.0

    @property
    def last_status(self) -> VisualAlignStatus:
        return self._last_status

    @staticmethod
    def _field(obj: Any, key: str, default=0.0):
        return getattr(obj, key, obj.get(key, default) if isinstance(obj, dict) else default)

    def _selected_memory_xy(self) -> Optional[Tuple[float, float]]:
        selected_positions = getattr(self._fsm.cylinder_selector, "_selected_positions", [])
        if not selected_positions:
            return None
        return selected_positions[-1]

    def _current_targets(self) -> Tuple[list, str]:
        cylinders = self._fsm.get_cylinders()
        if cylinders:
            return cylinders, "cylinder"
        markers = self._fsm.get_markers() if hasattr(self._fsm, "get_markers") else []
        if markers:
            return markers, "marker"
        return [], "none"

    def _pixel_delta(self, target: Any) -> Tuple[float, float]:
        img_cx = float(self._fsm.get_param("camera_cx", 640.0))
        img_cy = float(self._fsm.get_param("camera_cy", 480.0))
        u = float(self._field(target, "center_u", 0.0))
        v = float(self._field(target, "center_v", 0.0))
        return u - img_cx, v - img_cy

    def _choose_target(self) -> Tuple[Optional[Any], VisualAlignStatus]:
        targets, source = self._current_targets()
        memory_xy = self._selected_memory_xy()
        if not targets or memory_xy is None:
            return None, VisualAlignStatus(source=source, stable_count=self._stable_count)

        tx, ty = memory_xy
        world_gate = float(self._fsm.get_param("drop_visual_align_world_gate", 1.5))

        best = None
        best_score = float("inf")
        best_pixel = float("inf")
        best_du = 0.0
        best_dv = 0.0
        best_world = float("inf")

        for target in targets:
            conf = float(self._field(target, "confidence", 0.0))
            if conf < 0.25:
                continue
            wx = float(self._field(target, "center_x", 0.0))
            wy = float(self._field(target, "center_y", 0.0))
            world_error = math.hypot(wx - tx, wy - ty)
            du, dv = self._pixel_delta(target)
            pixel_error = math.hypot(du, dv)

            if (world_error > world_gate
                    and pixel_error > float(self._fsm.get_param("drop_align_pixel_tolerance", 35.0))):
                continue

            score = min(world_error, world_gate) * 1000.0 + pixel_error
            if score < best_score:
                best = target
                best_score = score
                best_pixel = pixel_error
                best_du = du
                best_dv = dv
                best_world = world_error

        if best is None:
            return None, VisualAlignStatus(source=source, stable_count=self._stable_count)

        return best, VisualAlignStatus(
            source=source,
            pixel_error=best_pixel,
            pixel_du=best_du,
            pixel_dv=best_dv,
            world_error_to_memory=best_world,
            target_world_xy=(float(self._field(best, "center_x", tx)),
                             float(self._field(best, "center_y", ty))),
            stable_count=self._stable_count,
        )

    def is_centered(self) -> bool:
        """目标是否已经在画面中心稳定对齐."""
        target, status = self._choose_target()
        tolerance = float(self._fsm.get_param("drop_align_pixel_tolerance", 35.0))
        stable_frames = int(self._fsm.get_param("drop_visual_align_stable_frames", 6))

        if target is None or status.pixel_error > tolerance:
            self._stable_count = 0
            status.stable_count = 0
            self._last_status = status
            return False

        self._stable_count += 1
        status.stable_count = self._stable_count
        self._last_status = status
        if self._stable_count >= stable_frames:
            self._fsm.get_logger().info(
                "DESCEND_ALIGN: 视觉中心稳定, 允许投放 "
                f"source={status.source} pixel_error={status.pixel_error:.1f}px "
                f"stable={self._stable_count}/{stable_frames}"
            )
            return True
        return False

    # ── 像素值采样辅助 ────────────────────────────────────────────

    def _pixel_valid(self, status: VisualAlignStatus) -> bool:
        return (math.isfinite(status.pixel_error)
                and not math.isinf(status.pixel_error)
                and math.isfinite(status.pixel_du)
                and math.isfinite(status.pixel_dv))

    def _current_step(self) -> float:
        """自适应步长: 根据当前误差大小选步长."""
        base = float(self._fsm.get_param("drop_visual_align_probe_step", 0.40))
        return base * self._step_scale

    def _update_step_scale(self, pixel_error: float) -> None:
        """根据当前像素误差调整步长档位."""
        if pixel_error > 200:
            self._step_scale = 1.0    # 大步 0.40m
        elif pixel_error > 100:
            self._step_scale = 0.65   # 中步 0.26m
        elif pixel_error > 50:
            self._step_scale = 0.40   # 小步 0.16m
        else:
            self._step_scale = 0.25   # 微步 0.10m

    def _probe_move(self, n: float, e: float, drop_h: float, label: str) -> None:
        """发送探测移动指令."""
        speed = float(self._fsm.get_param("drop_visual_align_probe_speed", 0.30))
        self._fsm.fc.goto_ned(n, e, -drop_h, speed=speed)
        self._probe_start_time = self._fsm.get_clock().now().nanoseconds / 1e9
        self._fsm.get_logger().info(
            f"DESCEND_PROBE: {label} → NED({n:.2f},{e:.2f}) "
            f"step={self._current_step():.3f}m scale={self._step_scale:.2f}"
        )

    # ── 端点比较法梯度下降 ─────────────────────────────────────────

    def _axis_delta_ned(self, axis_is_x: bool, sign: float, step: float) -> Tuple[float, float]:
        """返回 (dn, de): 指定轴和方向的 NED 偏移."""
        if axis_is_x:
            return (0.0, sign * step)   # WorldX = NED East
        else:
            return (sign * step, 0.0)   # WorldY = NED North

    def command_alignment(self, drop_height: float) -> bool:
        """主入口. 返回是否发送了 setpoint."""
        target, status = self._choose_target()
        self._last_status = status
        if target is None or status.target_world_xy is None:
            return False

        if self._phase == ServoPhase.BRAKING:
            return self._braking(status, drop_height)

        if self._phase == ServoPhase.COMMIT_AXIS:
            # 等待提交移动到位, 然后推进
            settle = float(self._fsm.get_param("drop_visual_align_probe_settle", 1.0))
            if self._fsm.get_clock().now().nanoseconds / 1e9 - self._probe_start_time >= settle:
                self._advance_phase(drop_height)
            return True

        return self._probing(status, drop_height)

    def _target_identity(self, status: VisualAlignStatus) -> str:
        """返回目标身份标识, 用于探测周期内锁定."""
        return f"{status.source}"

    def _probing(self, status: VisualAlignStatus, drop_h: float) -> bool:
        """端点比较法: 锁定目标身份, 分别采样 ±step, 取 pixel_error 小的提交."""
        now = self._fsm.get_clock().now().nanoseconds / 1e9
        settle = float(self._fsm.get_param("drop_visual_align_probe_settle", 1.0))
        step = self._current_step()
        axis_label = "X" if self._working_axis_x else "Y"

        # ── 设定基线 ──
        if self._probe_start_time == 0.0:
            pos = self._fsm.bridge.get_current_position()
            self._baseline_n = pos.north
            self._baseline_e = pos.east
            self._baseline_pixel_error = status.pixel_error
            self._baseline_source = self._target_identity(status)
            self._update_step_scale(status.pixel_error)
            step = self._current_step()

            self._phase = (ServoPhase.PROBE_X_PLUS if self._working_axis_x else ServoPhase.PROBE_Y_PLUS)
            dn, de = self._axis_delta_ned(self._working_axis_x, +1.0, step)
            self._probe_move(
                self._baseline_n + dn, self._baseline_e + de, drop_h,
                f"{axis_label}+ baseline_err={self._baseline_pixel_error:.0f}px "
                f"src={self._baseline_source}"
            )
            return True

        # ── 等待到位 ──
        if now - self._probe_start_time < settle:
            return True

        # ── 目标身份校验 ——
        current_id = self._target_identity(status)
        if current_id != self._baseline_source or not self._pixel_valid(status):
            self._fsm.get_logger().warn(
                f"DESCEND_PROBE: 目标切换/丢失 baseline={self._baseline_source} "
                f"→ current={current_id} valid={self._pixel_valid(status)}, 回基线重试"
            )
            self._goto_baseline(drop_h)
            self._probe_start_time = 0.0
            self._phase = ServoPhase.PROBE_X_PLUS
            self._working_axis_x = True
            return True

        if self._phase in (ServoPhase.PROBE_X_PLUS, ServoPhase.PROBE_Y_PLUS):
            # 刚采了 + 端点
            self._endpoint_plus_error = status.pixel_error
            delta = self._baseline_pixel_error - status.pixel_error
            self._fsm.get_logger().info(
                f"DESCEND_PROBE: +采样 err={status.pixel_error:.0f}px "
                f"delta={delta:+.0f}px src={current_id}"
            )

            self._phase = (ServoPhase.PROBE_X_MINUS if self._working_axis_x else ServoPhase.PROBE_Y_MINUS)
            dn, de = self._axis_delta_ned(self._working_axis_x, -1.0, step)
            self._probe_move(
                self._baseline_n + dn, self._baseline_e + de, drop_h,
                f"{axis_label}- from baseline"
            )
            return True

        else:
            # 刚采了 - 端点 → 三方比较
            self._endpoint_minus_error = status.pixel_error
            delta = self._baseline_pixel_error - status.pixel_error
            self._fsm.get_logger().info(
                f"DESCEND_PROBE: -采样 err={status.pixel_error:.0f}px "
                f"delta={delta:+.0f}px src={current_id}"
            )

            best = min(
                (self._baseline_pixel_error, 0.0),
                (self._endpoint_plus_error, +1.0),
                (self._endpoint_minus_error, -1.0),
                key=lambda x: x[0],
            )
            best_err, best_sign = best

            self._fsm.get_logger().info(
                f"DESCEND_PROBE: 比较 baseline={self._baseline_pixel_error:.0f} "
                f"+={self._endpoint_plus_error:.0f} -={self._endpoint_minus_error:.0f} "
                f"→ 最佳={best_sign:+.0f} err={best_err:.0f}px src={self._baseline_source}"
            )

            if best_sign != 0.0:
                dn, de = self._axis_delta_ned(self._working_axis_x, best_sign, step)
                self._goto_ned(self._baseline_n + dn, self._baseline_e + de, drop_h)
                self._probe_start_time = now
                self._phase = ServoPhase.COMMIT_AXIS
            else:
                self._goto_baseline(drop_h)
                self._probe_start_time = now
                self._phase = ServoPhase.COMMIT_AXIS
                self._step_scale *= 0.5
                self._fsm.get_logger().info(
                    f"DESCEND_PROBE: 探测无改善, 缩半步长→{self._current_step():.3f}m"
                )
            return True

    def _goto_baseline(self, drop_h: float) -> None:
        """回到本轮基线位置."""
        self._goto_ned(self._baseline_n, self._baseline_e, drop_h)

    def _goto_ned(self, n: float, e: float, drop_h: float) -> None:
        speed = float(self._fsm.get_param("drop_visual_align_probe_speed", 0.30))
        self._fsm.fc.goto_ned(n, e, -drop_h, speed=speed)

    def _advance_phase(self, drop_h: float) -> None:
        """提交后推进到下一个探测轴或刹车."""
        self._probe_start_time = 0.0
        brake_threshold = float(self._fsm.get_param("drop_visual_align_brake_threshold", 80.0))

        if self._last_status.pixel_error < brake_threshold:
            self._phase = ServoPhase.BRAKING
            self._brake_enter_time = self._fsm.get_clock().now().nanoseconds / 1e9
            self._brake_last_check = 0.0
            pos = self._fsm.bridge.get_current_position()
            self._goto_ned(pos.north, pos.east, drop_h)
            self._fsm.get_logger().info(
                f"DESCEND_PROBE: err={self._last_status.pixel_error:.0f}px < brake → BRAKING"
            )
        elif self._working_axis_x:
            # X 轴结束 → 开始 Y 轴
            self._working_axis_x = False
            self._phase = ServoPhase.PROBE_Y_PLUS
        else:
            # Y 轴结束 → 回到 X 轴
            self._working_axis_x = True
            self._phase = ServoPhase.PROBE_X_PLUS

    # ── 刹车悬停 ──────────────────────────────────────────────────

    def _braking(self, status: VisualAlignStatus, drop_h: float) -> bool:
        """刹车区: 悬停不动, 只观察. 如果 pixel_error 重新超阈值则退出."""
        now = self._fsm.get_clock().now().nanoseconds / 1e9
        brake_threshold = float(self._fsm.get_param("drop_visual_align_brake_threshold", 80.0))

        # 持续悬停
        pos = self._fsm.bridge.get_current_position()
        self._fsm.fc.goto_ned(pos.north, pos.east, -drop_h, speed=0.0)

        # 刹车超时
        brake_timeout = float(self._fsm.get_param("drop_visual_align_brake_timeout", 4.0))
        if now - self._brake_enter_time > brake_timeout:
            self._fsm.get_logger().info(
                f"DESCEND_BRAKE: 刹车超时 ({brake_timeout}s), 退出重试"
            )
            self._phase = ServoPhase.PROBE_X_PLUS
            self._probe_start_time = 0.0
            return True

        # 目标丢失 → 退出刹车
        if not self._pixel_valid(status):
            self._fsm.get_logger().warn("DESCEND_BRAKE: 目标丢失, 退出刹车")
            self._phase = ServoPhase.PROBE_X_PLUS
            self._probe_start_time = 0.0
            return True

        # pixel_error 重新涨超阈值 → 退出刹车
        if status.pixel_error >= brake_threshold:
            self._fsm.get_logger().info(
                f"DESCEND_BRAKE: pixel_error={status.pixel_error:.0f}px 超阈值, 退出刹车"
            )
            self._phase = ServoPhase.PROBE_X_PLUS
            self._probe_start_time = 0.0
            return True

        # 刹车中, 定期日志
        if now - self._brake_last_check >= self.BRAKE_CHECK_INTERVAL:
            self._brake_last_check = now
            # is_centered 会在 _execute_descend 中调用, 这里只负责悬停

        if now - self._last_log_time >= 1.0:
            self._last_log_time = now
            self._fsm.get_logger().info(
                "DESCEND_BRAKE: 悬停中 "
                f"pixel_error={status.pixel_error:.0f}px "
                f"du={status.pixel_du:.0f} dv={status.pixel_dv:.0f} "
                f"brake_t={now - self._brake_enter_time:.1f}s"
            )
        return True
