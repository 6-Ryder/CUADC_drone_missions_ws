"""
CYLINDER_SEARCH 状态 — 悬停搜索圆筒 + 视觉检测 + 半径排序.

流程:
  1. 飞到投放区中心, 悬停在较高识别高度
  2. 高处/扫描过程中累计圆筒世界坐标
  3. 累计到可投目标后选择目标 → SELECT_TARGET

回退策略:
  - 若长时间检测不足, 执行慢速扫描 (沿投放区短边飞行)
  - 若超时但已有目标, 使用已累计目标继续投放
"""

import time
from typing import Optional, List

from drone_utils.progress_monitor import ProgressMonitor

class CylinderSearchState:
    """投放区圆筒搜索状态."""

    def __init__(self, fsm):
        self._fsm = fsm
        self._start_time: float = 0.0
        self._scan_phase: int = 0
        self._scan_waypoints: List[dict] = []
        self._scan_wp_idx: int = 0
        self._detection_stable_count: int = 0
        self._last_cylinder_count: int = 0
        self._min_cylinders_required: int = 2
        self._target_progress: Optional[ProgressMonitor] = None

    def enter(self):
        self._fsm.logger.log_state("TRANSIT_TO_DROP_ZONE", "CYL_SEARCH", "开始搜索圆筒")
        self._start_time = time.time()
        self._scan_phase = 0
        self._scan_wp_idx = 0
        self._detection_stable_count = 0
        self._last_cylinder_count = 0
        if hasattr(self._fsm, "clear_known_cylinders"):
            self._fsm.clear_known_cylinders()
        self._min_cylinders_required = int(
            self._fsm.get_param("min_drop_targets_required", 2)
        )
        self._target_progress = ProgressMonitor(
            window_s=float(self._fsm.get_param("search_progress_window", 30.0)),
            min_improvement=1.0,
        )

        # 飞到投放区中心
        area1_x = (self._fsm.get_param("area1_east_min", -4.0) +
                   self._fsm.get_param("area1_east_max", 4.0)) / 2
        area1_y = (self._fsm.get_param("area1_north_min", 30.0) +
                   self._fsm.get_param("area1_north_max", 35.0)) / 2

        # NED 坐标
        drop_alt = self._fsm.fc.altitude_to_ned(self._fsm.get_param("area1_search_altitude", 5.0))
        self._fsm.fc.goto_ned(area1_y, area1_x, drop_alt,
                              speed=self._fsm.get_param("sweep_speed", 0.8))

        self._fsm.get_logger().info(
            f"CYL_SEARCH: 投放区中心 NED({area1_y:.1f}, {area1_x:.1f}, {drop_alt:.1f})"
        )

    def execute(self) -> Optional[str]:
        elapsed = time.time() - self._start_time

        # 获取视觉检测结果, 过滤低置信度假阳性
        cylinders = self._fsm.get_cylinders()
        markers = self._fsm.get_markers() if hasattr(self._fsm, "get_markers") else []
        min_conf = 0.4  # 最低置信度, 过滤 OpenCV 噪声
        valid = [c for c in cylinders if c.confidence >= min_conf]
        marker_delay = float(self._fsm.get_param("drop_marker_fallback_delay", 8.0))
        allow_marker_fallback = elapsed >= marker_delay
        if hasattr(self._fsm, "get_known_cylinders"):
            known_cylinders = self._fsm.get_known_cylinders()
        else:
            known_cylinders = valid

        if (
            allow_marker_fallback
            and not known_cylinders
            and hasattr(self._fsm, "get_known_drop_targets")
        ):
            known = self._fsm.get_known_drop_targets()
        else:
            known = known_cylinders
        cyl_count = len(valid)
        marker_count = len([m for m in markers if getattr(m, "confidence", 0.0) >= 0.35])
        known_count = len(known)

        # 稳定检测计数
        if known_count >= self._min_cylinders_required:
            self._detection_stable_count += 1
        else:
            self._detection_stable_count = max(0, self._detection_stable_count - 1)

        if known_count != self._last_cylinder_count:
            self._fsm.get_logger().info(
                f"CYL_SEARCH: 当前圆筒 {cyl_count} 个, marker {marker_count} 个, "
                f"累计投放候选 {known_count} 个 "
                f"(稳定={self._detection_stable_count}, marker_fallback={allow_marker_fallback})"
            )
            self._last_cylinder_count = known_count

        # 累计目标稳定 ≥ 0.25s (5 帧 @ 20Hz)
        if self._detection_stable_count >= 5 and known_count >= self._min_cylinders_required:
            self._fsm.get_logger().info(
                f"CYL_SEARCH: 投放候选累计稳定 ({known_count} 个), 进入选择"
            )
            return "SELECT_TARGET"

        if self._target_progress is not None:
            missing_targets = float(max(0, self._min_cylinders_required - known_count))
            now = time.monotonic()
            if self._target_progress.observe(missing_targets, now=now):
                if self._target_progress.mark_retry(now=now):
                    self._fsm.get_logger().warn(
                        "CYL_SEARCH: 候选不足且无进展, 重走同一搜索路径一次"
                    )
                    self._start_time = time.time()
                    self._start_scan()
                else:
                    self._fsm.request_takeover(
                        "CYL_SEARCH: insufficient targets after one search retry"
                    )
                    return None

        # 扫描策略: 如果 5s 后还没检测到足够圆筒, 进行慢速扫描
        if elapsed > 10.0 and self._scan_phase == 0:
            self._scan_phase = 1
            self._start_scan()
            self._fsm.get_logger().info("CYL_SEARCH: 启动慢速扫描")

        # 扫描阶段
        if self._scan_phase == 1:
            return self._execute_scan()

        return None

    def _start_scan(self):
        """启动投放区慢速扫描."""
        self._scan_phase = 1
        area1_n_min = self._fsm.get_param("area1_north_min", 30.0)
        area1_n_max = self._fsm.get_param("area1_north_max", 35.0)
        area1_e_min = self._fsm.get_param("area1_east_min", -4.0)
        area1_e_max = self._fsm.get_param("area1_east_max", 4.0)
        alt = self._fsm.get_param("area1_search_altitude", 5.0)

        # 沿 east 方向慢速扫描
        self._scan_waypoints = [
            {"north": (area1_n_min + area1_n_max) / 2, "east": area1_e_min, "down": self._fsm.fc.altitude_to_ned(alt),
             "label": "scan_w"},
            {"north": (area1_n_min + area1_n_max) / 2, "east": area1_e_max, "down": self._fsm.fc.altitude_to_ned(alt),
             "label": "scan_e"},
        ]
        self._scan_wp_idx = 0
        wp = self._scan_waypoints[0]
        self._fsm.fc.goto_ned(wp["north"], wp["east"], wp["down"],
                              speed=self._fsm.get_param("sweep_speed", 0.8))

    def _execute_scan(self) -> Optional[str]:
        """执行扫描."""
        if self._scan_wp_idx >= len(self._scan_waypoints):
            # 扫描完成
            self._scan_phase = 2
            if hasattr(self._fsm, "get_known_drop_targets"):
                targets = self._fsm.get_known_drop_targets()
            else:
                targets = self._fsm.get_cylinders()
            if len(targets) >= self._min_cylinders_required:
                return "SELECT_TARGET"
            self._fsm.get_logger().warn(
                "CYL_SEARCH: 扫描完成但候选不足, 等待同一路径重试"
            )
            return None

        wp = self._scan_waypoints[self._scan_wp_idx]
        if self._fsm.fc.reached((wp["north"], wp["east"], wp["down"]),
                                self._fsm.get_param("waypoint_tolerance", 0.5)):
            self._scan_wp_idx += 1
            if self._scan_wp_idx < len(self._scan_waypoints):
                next_wp = self._scan_waypoints[self._scan_wp_idx]
                self._fsm.fc.goto_ned(
                    next_wp["north"], next_wp["east"], next_wp["down"],
                    speed=self._fsm.get_param("sweep_speed", 0.8),
                )

        return None

    def exit(self):
        if hasattr(self._fsm, "get_known_cylinders"):
            cyl_count = len(self._fsm.get_known_drop_targets())
        else:
            cyl_count = len(self._fsm.get_cylinders())
        self._fsm.logger.log_state("CYL_SEARCH", "SELECT_TARGET",
                                   f"搜索完成, 累计 {cyl_count} 个投放候选")
