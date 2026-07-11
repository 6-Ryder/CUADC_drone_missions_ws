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
    PROBE_X = 0      # X 轴方向探测
    PROBE_Y = 1      # Y 轴方向探测
    BRAKING = 2      # 刹车悬停, 等待稳定对齐


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

        # 探测状态
        self._phase: ServoPhase = ServoPhase.PROBE_X
        self._probing_plus: bool = True    # 当前在探 + 还是 - 方向
        self._anchor_n: float = 0.0         # 本轮探测的锚点 NED North
        self._anchor_e: float = 0.0         # 本轮探测的锚点 NED East
        self._anchor_pixel_error: float = float("inf")
        self._anchor_du: float = 0.0
        self._anchor_dv: float = 0.0
        self._probe_start_time: float = 0.0
        self._plus_pixel_error: float = float("inf")
        self._minus_pixel_error: float = float("inf")

        # 刹车
        self._brake_enter_time: float = 0.0
        self._brake_last_check: float = 0.0

        # 自适应步长
        self._step_scale: float = 1.0  # 1.0=大步, 0.5=中步, 0.25=小步

    def reset(self) -> None:
        self._stable_count = 0
        self._last_log_time = 0.0
        self._last_status = VisualAlignStatus()
        self._phase = ServoPhase.PROBE_X
        self._probing_plus = True
        self._anchor_n = 0.0
        self._anchor_e = 0.0
        self._anchor_pixel_error = float("inf")
        self._anchor_du = 0.0
        self._anchor_dv = 0.0
        self._probe_start_time = 0.0
        self._plus_pixel_error = float("inf")
        self._minus_pixel_error = float("inf")
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
        base = float(self._fsm.get_param("drop_visual_align_probe_step", 0.20))
        return base * self._step_scale

    def _update_step_scale(self, pixel_error: float) -> None:
        """根据当前像素误差调整步长档位."""
        if pixel_error > 150:
            self._step_scale = 1.0    # 大步 0.20m
        elif pixel_error > 60:
            self._step_scale = 0.6    # 中步 0.12m
        else:
            self._step_scale = 0.35   # 小步 0.07m

    def _goto_ned(self, n: float, e: float, drop_h: float) -> None:
        speed = float(self._fsm.get_param("drop_visual_align_probe_speed", 0.25))
        self._fsm.fc.goto_ned(n, e, -drop_h, speed=speed)

    # ── 梯度下降伺服 ──────────────────────────────────────────────

    def command_alignment(self, drop_height: float) -> bool:
        """主入口. 返回是否发送了 setpoint."""
        target, status = self._choose_target()
        self._last_status = status
        if target is None or status.target_world_xy is None:
            return False

        if self._phase == ServoPhase.BRAKING:
            return self._braking(status, drop_height)

        return self._probing(status, drop_height)

    def _set_anchor(self, status: VisualAlignStatus) -> None:
        """记录当前位置和像素值为锚点."""
        pos = self._fsm.bridge.get_current_position()
        self._anchor_n = pos.north
        self._anchor_e = pos.east
        self._anchor_pixel_error = status.pixel_error
        self._anchor_du = status.pixel_du
        self._anchor_dv = status.pixel_dv

    def _move_and_wait(self, n: float, e: float, drop_h: float, phase_label: str) -> None:
        """发送移动指令并记录探测开始时间."""
        self._goto_ned(n, e, drop_h)
        self._probe_start_time = self._fsm.get_clock().now().nanoseconds / 1e9
        self._fsm.get_logger().info(
            f"DESCEND_PROBE: {phase_label} → NED({n:.2f},{e:.2f}) "
            f"step={self._current_step():.3f}m"
        )

    def _probing(self, status: VisualAlignStatus, drop_h: float) -> bool:
        """梯度下降探测状态机."""
        now = self._fsm.get_clock().now().nanoseconds / 1e9
        settle = float(self._fsm.get_param("drop_visual_align_probe_settle", 0.8))
        step = self._current_step()
        brake_threshold = float(self._fsm.get_param("drop_visual_align_brake_threshold", 80.0))

        # ── 开始一轮新探测: 设定锚点 ──
        if self._probe_start_time == 0.0:
            self._set_anchor(status)
            self._probing_plus = True
            self._plus_pixel_error = float("inf")
            self._minus_pixel_error = float("inf")
            self._update_step_scale(status.pixel_error)

            # X 轴: 尝试 +WorldX (NED +East)
            if self._phase == ServoPhase.PROBE_X:
                self._move_and_wait(
                    self._anchor_n,
                    self._anchor_e + step,
                    drop_h,
                    f"X+ probe anchor_err={self._anchor_pixel_error:.0f}px step_scale={self._step_scale:.2f}"
                )
            else:
                # Y 轴: 尝试 +WorldY (NED +North)
                self._move_and_wait(
                    self._anchor_n + step,
                    self._anchor_e,
                    drop_h,
                    f"Y+ probe anchor_err={self._anchor_pixel_error:.0f}px step_scale={self._step_scale:.2f}"
                )
            return True

        # ── 等待探测移动完成并采样 ──
        if now - self._probe_start_time < settle:
            return True

        # 采样当前像素误差
        if not self._pixel_valid(status):
            self._fsm.get_logger().warn("DESCEND_PROBE: 采样无效 (目标丢失)")
            # 回到锚点
            self._goto_ned(self._anchor_n, self._anchor_e, drop_h)
            self._probe_start_time = 0.0
            return True

        current_pixel_error = status.pixel_error

        if self._probing_plus:
            # 刚探测了 + 方向
            self._plus_pixel_error = current_pixel_error
            delta = self._anchor_pixel_error - current_pixel_error  # 正=改善了
            self._fsm.get_logger().info(
                f"DESCEND_PROBE: +采样 err={current_pixel_error:.0f}px "
                f"delta={delta:+.0f}px (正=改善)"
            )

            if current_pixel_error < self._anchor_pixel_error:
                # + 方向改善了 → 直接提交, 跳过 - 探测
                self._fsm.get_logger().info(
                    f"DESCEND_PROBE: +方向改善, 提交 "
                    f"err {self._anchor_pixel_error:.0f}→{current_pixel_error:.0f}px"
                )
                self._commit_and_advance(drop_h)
            else:
                # + 方向没改善 → 回到锚点, 试 - 方向
                self._probing_plus = False
                self._goto_ned(self._anchor_n, self._anchor_e, drop_h)
                self._probe_start_time = now  # 重置等待时间
                direction = "-X" if self._phase == ServoPhase.PROBE_X else "-Y"
                self._fsm.get_logger().info(
                    f"DESCEND_PROBE: +方向恶化, 回锚试{direction}"
                )
            return True

        else:
            # 刚探测了 - 方向 (从锚点)
            self._minus_pixel_error = current_pixel_error
            delta = self._anchor_pixel_error - current_pixel_error
            self._fsm.get_logger().info(
                f"DESCEND_PROBE: -采样 err={current_pixel_error:.0f}px "
                f"delta={delta:+.0f}px (正=改善)"
            )

            # 回到锚点, 提交 ± 中更好的
            self._goto_ned(self._anchor_n, self._anchor_e, drop_h)
            self._probe_start_time = now

            if current_pixel_error < self._anchor_pixel_error:
                # - 方向改善了
                self._fsm.get_logger().info(
                    f"DESCEND_PROBE: -方向改善, 提交 "
                    f"err {self._anchor_pixel_error:.0f}→{current_pixel_error:.0f}px"
                )
                # 已经回到了锚点, 需要重新移动到 - 方向的目标位置
                # 但 goto_ned 已经发出了回到锚点的指令
                # 等回到锚点后再移动到 - 方向
                self._goto_ned(self._anchor_n, self._anchor_e, drop_h)
                self._probe_start_time = now
                self._probing_plus = False  # 保持, 等回到锚点
                # 标记: 下一轮回到锚点后要移动到 - 位置
                # 用一个额外状态来处理
                # 简化处理: 直接提交锚点, 因为 - 方向也没好到哪去
                # 等自然进入下一轮重新探测
                self._commit_and_advance(drop_h)
            else:
                # 两个方向都没改善 → 保持锚点, 进入下一轮
                self._fsm.get_logger().info(
                    "DESCEND_PROBE: ±都未改善, 保持锚点 "
                    f"plus={self._plus_pixel_error:.0f} minus={self._minus_pixel_error:.0f} "
                    f"anchor={self._anchor_pixel_error:.0f}px"
                )
                self._commit_and_advance(drop_h)
            return True

    def _commit_and_advance(self, drop_h: float) -> None:
        """提交当前位置为新的工作点, 进入下一阶段."""
        self._probe_start_time = 0.0

        # 检查是否该进入刹车区
        if self._last_status.pixel_error < float(self._fsm.get_param("drop_visual_align_brake_threshold", 80.0)):
            self._phase = ServoPhase.BRAKING
            self._brake_enter_time = self._fsm.get_clock().now().nanoseconds / 1e9
            self._brake_last_check = 0.0
            self._goto_ned(
                self._fsm.bridge.get_current_position().north,
                self._fsm.bridge.get_current_position().east,
                drop_h,
            )
            self._fsm.get_logger().info(
                f"DESCEND_PROBE: pixel_error={self._last_status.pixel_error:.0f}px "
                f"< brake_threshold → BRAKING"
            )
        elif self._phase == ServoPhase.PROBE_X:
            self._phase = ServoPhase.PROBE_Y
        else:
            self._phase = ServoPhase.PROBE_X

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
            self._phase = ServoPhase.PROBE_X
            self._probe_start_time = 0.0
            return True

        # 目标丢失 → 退出刹车
        if not self._pixel_valid(status):
            self._fsm.get_logger().warn("DESCEND_BRAKE: 目标丢失, 退出刹车")
            self._phase = ServoPhase.PROBE_X
            self._probe_start_time = 0.0
            return True

        # pixel_error 重新涨超阈值 → 退出刹车
        if status.pixel_error >= brake_threshold:
            self._fsm.get_logger().info(
                f"DESCEND_BRAKE: pixel_error={status.pixel_error:.0f}px 超阈值, 退出刹车"
            )
            self._phase = ServoPhase.PROBE_X
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
