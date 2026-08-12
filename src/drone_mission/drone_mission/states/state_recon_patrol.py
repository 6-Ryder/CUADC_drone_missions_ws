"""
RECON_PATROL 状态 — 降低高度后的弓字形全覆盖巡视.

流程:
  1. 降至 area2_altitude 指定的侦察扫描高度
  2. 使用 PathPlanner 生成侦察区弓字形全覆盖航点
  3. 沿 strips 完整飞行, 实时监控相机覆盖进度
  4. 默认所有航点完成后 → RETURN_TO_HOME

注意: 覆盖率用于验证相机观察范围, 默认不提前终止路径扫描.
"""

import time
from typing import Optional, List

from drone_utils.progress_monitor import ProgressMonitor

class ReconPatrolState:
    """侦察区弓字形全覆盖巡视状态 (纯路径覆盖)."""

    def __init__(self, fsm):
        self._fsm = fsm
        self._start_time: float = 0.0
        self._waypoints: List[dict] = []
        self._wp_idx: int = 0
        self._coverage_complete: bool = False
        self._last_progress_log: float = 0.0
        self._waypoint_stable_count: int = 0
        self._waypoint_stable_required: int = 8  # 20Hz 下=0.4s 稳定确认
        self._waypoint_progress: Optional[ProgressMonitor] = None

    def enter(self):
        self._fsm.logger.log_state("TRANSIT_TO_RECON_ZONE", "RECON_PATROL",
                                   "降低高度后开始侦察区全覆盖扫描")
        self._start_time = time.time()
        self._wp_idx = 0
        self._coverage_complete = False
        self._last_progress_log = 0.0
        self._waypoint_stable_count = 0
        self._waypoint_progress = ProgressMonitor(
            window_s=float(self._fsm.get_param("navigation_progress_window", 30.0)),
            min_improvement=float(
                self._fsm.get_param("navigation_progress_min_improvement", 0.25)
            ),
        )

        # 侦察阶段只记录覆盖率，不需要运行两套目标检测模型。
        self._fsm.set_vision_mode("coverage_reset")

        # 生成低空弓字形覆盖路径 (NED 坐标系)
        area2_forward_min, area2_forward_max, area2_right_min, area2_right_max = (
            self._fsm.zone_bounds("area2")
        )
        recon_alt = self._fsm.get_param("area2_altitude", 2.5)

        self._waypoints = self._fsm.path_planner.plan_coverage_ned(
            area_forward_min=area2_forward_min,
            area_forward_max=area2_forward_max,
            area_right_min=area2_right_min,
            area_right_max=area2_right_max,
            altitude=self._fsm.fc.altitude_to_ned(recon_alt),  # NED: 负值=向上
        )

        # 计算预计覆盖参数
        forward_range = area2_forward_max - area2_forward_min
        right_range = area2_right_max - area2_right_min
        total_area = forward_range * right_range
        grid_spacing = self._fsm.get_param("grid_spacing", 1.67)
        num_strips = len(self._waypoints) // 2  # 每 strip 2 个航点

        self._fsm.get_logger().info(
            f"RECON_PATROL: {len(self._waypoints)} 个航点 ({num_strips} 条带), "
            f"区域 local F[{area2_forward_min}:{area2_forward_max}] "
            f"R[{area2_right_min}:{area2_right_max}], "
            f"面积 {total_area:.0f}m², 高度 {recon_alt:.1f}m, 间距 {grid_spacing}m"
        )

        # 飞向第一个航点
        if self._waypoints:
            self._fly_to_wp(0)
        else:
            self._coverage_complete = True

    def execute(self) -> Optional[str]:
        # 覆盖率用于验证相机视场覆盖. 默认必须跑完整条弓字形路径,
        # 避免大 FOV 或历史覆盖状态导致刚进入侦察区就提前返航.
        coverage = self._fsm.get_coverage()
        require_full_path = self._fsm.get_param("recon_require_full_path", True)
        if (not require_full_path) and coverage and coverage.get("is_complete", False):
            self._coverage_complete = True
            self._fsm.get_logger().info(
                f"RECON_PATROL: 覆盖率达标 "
                f"(ratio={coverage.get('coverage_ratio', 0):.1%}), "
                f"wp={self._wp_idx}/{len(self._waypoints)}"
            )
            return "RETURN_TO_HOME"

        # 所有航点完成
        if self._wp_idx >= len(self._waypoints):
            cov_ratio = coverage.get("coverage_ratio", 0) if coverage else 0
            self._fsm.get_logger().info(
                f"RECON_PATROL: 全部扫描航点完成, coverage={cov_ratio:.1%}"
            )
            if hasattr(self._fsm, "mark_recon_path_completed"):
                self._fsm.mark_recon_path_completed()
            return "RETURN_TO_HOME"

        current_wp = self._waypoints[self._wp_idx]

        # 检查是否到达当前航点 (需连续稳定多帧, 防止偶然穿过容差边界后漂出)
        wp_target = (current_wp["north"], current_wp["east"], current_wp["down"])
        at_waypoint = self._fsm.fc.reached(
            wp_target, self._fsm.get_param("waypoint_tolerance", 0.5)
        )
        if at_waypoint:
            self._waypoint_stable_count += 1
        else:
            self._waypoint_stable_count = max(0, self._waypoint_stable_count - 1)

        if self._waypoint_stable_count >= self._waypoint_stable_required:
            self._waypoint_stable_count = 0
            self._wp_idx += 1
            if self._wp_idx == 1:
                # The first waypoint is the strip entry. Start accounting only
                # after reaching scan altitude, before flying the first strip.
                self._fsm.set_vision_mode("coverage")
            if self._wp_idx < len(self._waypoints):
                self._waypoint_progress.reset()
                self._fly_to_wp(self._wp_idx)

        if (
            not at_waypoint
            and self._wp_idx < len(self._waypoints)
            and self._waypoint_progress
        ):
            current_wp = self._waypoints[self._wp_idx]
            pos = self._fsm.fc.get_position()
            error = pos.distance_to(
                type(pos)(current_wp["north"], current_wp["east"], current_wp["down"])
            )
            now = time.monotonic()
            if self._waypoint_progress.observe(error, now=now):
                if self._waypoint_progress.mark_retry(now=now):
                    self._fsm.get_logger().warn(
                        f"RECON_PATROL: 航点 {self._wp_idx} 无进展, 重发一次"
                    )
                    self._fly_to_wp(self._wp_idx)
                else:
                    self._fsm.request_takeover(
                        "RECON_PATROL: waypoint "
                        f"{self._wp_idx} made no progress after retry"
                    )
                    return None

        elapsed = time.time() - self._start_time

        # 每 10s 输出进度
        if elapsed - self._last_progress_log > 10.0:
            self._last_progress_log = elapsed
            cov_ratio = coverage.get("coverage_ratio", 0) if coverage else 0
            self._fsm.get_logger().info(
                f"RECON_PATROL: wp={self._wp_idx}/{len(self._waypoints)} "
                f"coverage={cov_ratio:.1%} elapsed={elapsed:.0f}s"
            )

        return None

    def _fly_to_wp(self, idx: int):
        wp = self._waypoints[idx]
        self._fsm.fc.goto_ned(
            wp["north"], wp["east"], wp["down"],
            yaw=wp.get("yaw", 0.0),
            speed=wp.get("speed", self._fsm.get_param("sweep_speed", 0.8)),
        )
        if self._fsm.watchdog:
            self._fsm.watchdog.feed("mission_fsm")

    def exit(self):
        coverage = self._fsm.get_coverage()
        cov_ratio = coverage.get("coverage_ratio", 0) if coverage else 0
        self._fsm.logger.log_state("RECON_PATROL", "RETURN_TO_HOME",
                                   f"侦察区完整扫描完成, coverage={cov_ratio:.1%}")
