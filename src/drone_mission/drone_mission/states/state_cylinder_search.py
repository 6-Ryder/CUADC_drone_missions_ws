"""
CYLINDER_SEARCH 状态 — 悬停搜索圆筒 + 视觉检测 + 半径排序.

流程:
  1. 飞到投放区中心, 悬停在较高识别高度
  2. 高处/扫描过程中累计圆筒图像观测；世界坐标仅供粗定位
  3. 累计到可投目标后选择目标 → SELECT_TARGET

回退策略:
  - 若长时间检测不足, 执行慢速扫描 (沿投放区短边飞行)
  - 若超时但已有目标, 使用已累计目标继续投放
"""

import math
import time
from typing import Optional, List

from drone_utils.progress_monitor import ProgressMonitor
from drone_utils.servo_pid import ServoPid
from drone_mission.quick_bucket_gate import is_bucket_search_candidate

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
        self._last_bucket_diagnostic = 0.0
        self._blue_descent_active = False
        self._blue_descent_done = False
        self._blue_pixel_uv = None
        self._blue_recent_stamps = []
        self._blue_last_stamp = 0.0
        self._blue_last_seen = 0.0
        self._blue_descent_started_at = 0.0
        self._search_altitude = None
        # 蓝盘预降高: 米制域 PID, 跨帧持久; enter() 时 reset 清积分/微分历史
        self._blue_servo_pid = self._new_blue_servo_pid()

    def enter(self):
        self._fsm.logger.log_state("TRANSIT_TO_DROP_ZONE", "CYL_SEARCH", "开始搜索圆筒")
        self._start_time = time.time()
        self._scan_phase = 0
        self._scan_wp_idx = 0
        self._detection_stable_count = 0
        self._last_cylinder_count = 0
        self._last_bucket_diagnostic = 0.0
        self._blue_descent_active = False
        self._blue_descent_done = False
        self._blue_pixel_uv = None
        self._blue_recent_stamps = []
        self._blue_last_stamp = 0.0
        self._blue_last_seen = 0.0
        self._blue_descent_started_at = 0.0
        self._search_altitude = None
        # 每次进入搜索从纯 P 起步, 不把上一次蓝盘逼近的积分偏置带进来
        self._blue_servo_pid.reset()
        if hasattr(self._fsm, "clear_known_cylinders"):
            self._fsm.clear_known_cylinders()
        self._min_cylinders_required = int(
            self._fsm.get_param("min_drop_targets_required", 2)
        )
        if self._fsm.get_param("quick_drop_mode", False):
            # 蓝盘基线保留单模型；小桶配置同时运行圆筒和蓝盘模型。
            # 服务不可用会抛异常并被 _transition_to 升级为 takeover, 必须兜住
            try:
                profile = self._fsm.get_param("quick_drop_profile", "blue")
                self._fsm.set_vision_mode("all" if profile == "bucket" else "marker_only")
            except Exception as exc:
                self._fsm.get_logger().warn(
                    f"CYL_SEARCH: 切换到 marker_only 失败, 继续用默认检测模式: {exc}"
                )
        self._target_progress = ProgressMonitor(
            window_s=float(self._fsm.get_param("search_progress_window", 30.0)),
            min_improvement=1.0,
        )

        # 飞到投放区中心
        forward_min, forward_max, right_min, right_max = self._fsm.zone_bounds("area1")
        center_forward = (forward_min + forward_max) / 2.0
        center_right = (right_min + right_max) / 2.0
        center_north, center_east = self._fsm.layout_frame.local_to_ned(
            center_forward, center_right
        )

        # NED 坐标
        drop_alt = self._fsm.fc.altitude_to_ned(self._fsm.get_param("area1_search_altitude", 5.0))
        self._fsm.fc.goto_ned(center_north, center_east, drop_alt,
                              speed=self._fsm.get_param("sweep_speed", 0.8))

        self._fsm.get_logger().info(
            f"CYL_SEARCH: 投放区中心 NED({center_north:.1f}, "
            f"{center_east:.1f}, {drop_alt:.1f})"
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

        if self._fsm.get_param("quick_drop_mode", False) and self._fsm.get_param("quick_drop_profile", "blue") == "bucket":
            remembered_markers = self._fsm.get_known_markers()
            known = [c for c in known_cylinders
                     if is_bucket_search_candidate(c, time.time())]
            now_monotonic = time.monotonic()
            if now_monotonic - self._last_bucket_diagnostic >= 2.0:
                details = []
                for c in known_cylinders:
                    details.append(
                        f"id={c.get('id', '?')} r={c.get('radius', math.nan):.3f} "
                        f"search_frames={len(c.get('bucket_search_samples', {}))} "
                        f"eligible={int(is_bucket_search_candidate(c, time.time()))} "
                        f"pixel=({c.get('center_u', math.nan):.0f},"
                        f"{c.get('center_v', math.nan):.0f})")
                visible_radii = [round(float(getattr(c, "radius", math.nan)), 3)
                                 for c in valid]
                self._fsm.get_logger().info(
                    f"BUCKET_SEARCH: visible_cyl={len(valid)} "
                    f"world_cyl={sum(bool(getattr(c, 'world_position_valid', False)) for c in valid)} "
                    f"visible_disc={len(markers)} "
                    f"world_disc={sum(bool(getattr(m, 'world_position_valid', False)) for m in markers)} "
                    f"memory_cyl={len(known_cylinders)} memory_disc={len(remembered_markers)} "
                    f"eligible={len(known)} required={self._min_cylinders_required} "
                    f"world_reference_only=1 "
                    f"visible_r={visible_radii} targets=[{'; '.join(details)}]")
                self._last_bucket_diagnostic = now_monotonic
        elif (
            self._fsm.get_param("quick_drop_mode", False)
            and self._fsm.get_param("quick_drop_profile", "blue") != "bucket"
            and hasattr(self._fsm, "get_known_markers")
        ):
            # quick 模式只认蓝底标识; mode 6 切换是异步服务调用,
            # 生效窗口内圆筒可能仍入记忆, 此处显式排除只数 marker
            known = self._fsm.get_known_markers()
        elif (
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
            if self._blue_descent_active:
                self._fsm.fc.stop_velocity()
            return "SELECT_TARGET"

        if (self._fsm.get_param("quick_drop_mode", False)
                and self._fsm.get_param("quick_drop_profile", "blue") == "bucket"
                and self._fsm.get_param("bucket_blue_predescent_enable", True)
                and self._blue_predescent_tick(markers)):
            return None

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

    def _new_blue_servo_pid(self) -> ServoPid:
        """构造蓝盘预降高的米制域伺服 PID (增益契约见 config/pid_tuning.yaml).

        与 PrecisionAlignController 连续模式同域: 误差单位 m, 由平面距离把
        像素偏差换算成地面偏差. 符号沿用图像→机体标定.
        """
        return ServoPid(
            kp=float(self._fsm.get_param("servo_kp", 0.40)),
            ki=float(self._fsm.get_param("servo_ki", 0.08)),
            kd=float(self._fsm.get_param("servo_kd", 0.05)),
            max_speed=float(self._fsm.get_param("servo_max_speed", 0.25)),
            min_speed=float(self._fsm.get_param("servo_min_speed", 0.012)),
            sign_forward=float(self._fsm.get_param(
                "precision_align_image_y_to_forward_sign", -1.0)),
            sign_right=float(self._fsm.get_param(
                "precision_align_image_x_to_right_sign", 1.0)),
            integral_limit=float(self._fsm.get_param("servo_integral_limit", 0.10)),
            derivative_alpha=float(
                self._fsm.get_param("servo_derivative_alpha", 0.30)),
            integral_band=float(self._fsm.get_param("servo_integral_band", 0.0)),
        )

    def _now(self) -> float:
        """FSM 时钟秒 — PID 用它判定新测量与真实积分/微分步长."""
        return self._fsm.get_clock().now().nanoseconds / 1e9

    def _blue_predescent_tick(self, markers: list) -> bool:
        """Use fresh blue pixels to approach a better rim viewing height."""
        if self._blue_descent_done:
            return False
        now = time.time()
        stamp = float(getattr(self._fsm, "_marker_capture_stamp", 0.0))
        fresh = 0 < stamp <= now and now - stamp <= 0.40
        cx = float(self._fsm.get_param("camera_cx", 320.0))
        cy = float(self._fsm.get_param("camera_cy", 240.0))
        width = float(self._fsm.get_param("precision_align_image_width", 640.0))
        height = float(self._fsm.get_param("precision_align_image_height", 480.0))
        candidates = []
        if fresh:
            for marker in markers:
                confidence = float(getattr(marker, "confidence", 0.0))
                u = float(getattr(marker, "center_u", math.nan))
                v = float(getattr(marker, "center_v", math.nan))
                if (math.isfinite(confidence) and confidence >= 0.35
                        and math.isfinite(u) and math.isfinite(v)
                        and 0 <= u < width and 0 <= v < height):
                    candidates.append((u, v))
        hint = self._blue_pixel_uv or (cx, cy)
        target = min(candidates, key=lambda uv: math.dist(uv, hint), default=None)
        if (target is not None and self._blue_pixel_uv is not None
                and math.dist(target, self._blue_pixel_uv) > 120.0):
            target = None

        if target is None and not self._blue_descent_active:
            self._blue_recent_stamps = []
            self._blue_pixel_uv = None
            return False
        pos = self._fsm.bridge.get_current_position()
        altitude = float(self._fsm.fc.ned_to_altitude(pos.down))
        floor = 3.0
        if self._blue_descent_active and altitude <= floor:
            self._finish_blue_predescent(now, altitude)
            return True
        if (self._blue_descent_active
                and now - self._blue_descent_started_at >= 15.0):
            self._finish_blue_predescent(now, altitude)
            return True
        if (not self._blue_descent_active
                and abs(altitude - float(self._fsm.get_param(
                    "area1_search_altitude", 4.5))) > 0.25):
            return False

        if target is None:
            # 目标丢失: 通知 PID 冻结积分/清微分历史, 悬停等待恢复
            self._blue_servo_pid.update(
                0.0, 0.0, self._now(), measurement_valid=False)
            self._fsm.fc.set_velocity_body(0.0, 0.0, 0.0)
            if now - self._blue_last_seen > 2.0:
                self._finish_blue_predescent(now, altitude)
            return True

        self._blue_pixel_uv = target
        self._blue_last_seen = now
        if stamp > self._blue_last_stamp:
            self._blue_last_stamp = stamp
            self._blue_recent_stamps = [s for s in self._blue_recent_stamps
                                        if stamp - s <= 1.0]
            self._blue_recent_stamps.append(stamp)
        if not self._blue_descent_active:
            if len(self._blue_recent_stamps) < 3:
                return False
            self._blue_descent_active = True
            self._blue_descent_started_at = now
            self._scan_phase = 0
            self._fsm.get_logger().info(
                "CYL_SEARCH: 蓝盘三帧稳定，开始视觉引导预降高至 3.0 m")

        # The blue disc is on the ground; bucket rim height and front-bottle
        # offset apply only after a cylinder is selected.
        plane_distance = max(
            altitude + float(self._fsm.get_param("bucket_home_body_height", 0.0))
            - float(self._fsm.get_param("bucket_camera_down", 0.0)), 0.3)
        fx = max(float(self._fsm.get_param("camera_fx", 400.0)), 1.0)
        fy = max(float(self._fsm.get_param("camera_fy", 400.0)), 1.0)
        # 米制域误差 (m): 蓝盘在地面, 用平面距离把像素偏差换算成地面偏差
        error_forward_m = (target[1] - cy) * plane_distance / fy
        error_right_m = (target[0] - cx) * plane_distance / fx
        # 米制域 PID: 符号标定与合速度(向量模长)限幅均由 PID 承担.
        # 蓝盘走 markers 话题, 故测量时刻取 "marker" (采集时刻优先).
        forward, right = self._blue_servo_pid.update(
            error_forward_m, error_right_m,
            self._fsm.vision_measurement_stamp("marker"))
        in_view = (abs(target[0] - cx) <= 0.35 * width
                   and abs(target[1] - cy) <= 0.35 * height)
        descent = (float(self._fsm.get_param("drop_servo_descend_slow", 0.28))
                   if in_view else 0.0)
        self._fsm.fc.set_velocity_body(forward, right, descent)
        return True

    def _finish_blue_predescent(self, now: float, altitude: float) -> None:
        self._blue_descent_active = False
        self._blue_descent_done = True
        self._search_altitude = altitude
        self._start_time = now
        self._scan_phase = 0
        self._target_progress = ProgressMonitor(
            window_s=float(self._fsm.get_param("search_progress_window", 30.0)),
            min_improvement=1.0,
        )
        self._fsm.get_logger().info(
            f"CYL_SEARCH: 蓝盘预降高结束 alt={altitude:.2f}m，继续搜索圆筒")
        self._fsm.fc.stop_velocity()

    def _start_scan(self):
        """启动投放区慢速扫描."""
        self._scan_phase = 1
        forward_min, forward_max, right_min, right_max = self._fsm.zone_bounds("area1")
        alt = (self._search_altitude if self._search_altitude is not None
               else self._fsm.get_param("area1_search_altitude", 5.0))
        center_forward = (forward_min + forward_max) / 2.0
        start_north, start_east = self._fsm.layout_frame.local_to_ned(
            center_forward, right_min
        )
        end_north, end_east = self._fsm.layout_frame.local_to_ned(
            center_forward, right_max
        )
        down = self._fsm.fc.altitude_to_ned(alt)

        # 沿 field-local right 方向慢速扫描
        self._scan_waypoints = [
            {"north": start_north, "east": start_east, "down": down,
             "label": "scan_w"},
            {"north": end_north, "east": end_east, "down": down,
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
            if (self._fsm.get_param("quick_drop_mode", False)
                    and self._fsm.get_param("quick_drop_profile", "blue") == "bucket"):
                cylinders = self._fsm.get_known_cylinders()
                targets = [c for c in cylinders
                           if is_bucket_search_candidate(c, time.time())]
            elif hasattr(self._fsm, "get_known_drop_targets"):
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
        if hasattr(self._fsm, "get_known_drop_targets"):
            cyl_count = len(self._fsm.get_known_drop_targets())
        elif hasattr(self._fsm, "get_known_cylinders"):
            cyl_count = len(self._fsm.get_known_cylinders())
        else:
            cyl_count = len(self._fsm.get_cylinders())
        # The FSM owns the real transition (selection, retry or takeover).
        self._fsm.get_logger().info(
            f"CYL_SEARCH: 退出搜索, 原始目标记忆 {cyl_count} 个, "
            f"合格投放候选 {self._last_cylinder_count} 个")
