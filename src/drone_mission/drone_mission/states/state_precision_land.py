"""
PRECISION_LAND 状态 — 视觉伺服 + H 标识检测 + 对心降落.

三阶段:
  Phase 1: 粗定位 (5m 高度, 搜索 H 标识)
  Phase 2: 视觉伺服对心下降 (5m → 0.5m)
  Phase 3: 盲降 (< 0.5m, 视觉可能失效, 惯性下降)

退出条件:
  - land_detected: 已着陆 → COMPLETE
  - vision lost > 3s: 盲降模式 (保持水平, 持续下降)
  - timeout: 强制 Land 模式
"""

import time
from typing import Optional


class PrecisionLandState:
    """精准降落状态处理器."""

    PHASE_SEARCH = 0     # 粗定位搜索
    PHASE_SERVO = 1      # 视觉伺服下降
    PHASE_BLIND = 2      # 盲降

    def __init__(self, fsm):
        self._fsm = fsm
        self._phase: int = self.PHASE_SEARCH
        self._start_time: float = 0.0
        self._vision_lost_start: float = 0.0
        self._has_landing_target: bool = False
        self._landing_done: bool = False

    def enter(self):
        self._fsm.logger.log_state("RETURN_TO_HOME", "PRECISION_LAND",
                                   "开始精准降落")
        self._phase = self.PHASE_SEARCH
        self._start_time = time.time()
        self._vision_lost_start = 0.0
        self._has_landing_target = False
        self._landing_done = False

        # 激活视觉伺服: landing 模式
        if self._fsm.vision_node:
            self._fsm.vision_node.set_servo_mode("landing")

        self._fsm.get_logger().info("PRECISION_LAND: Phase 1 - 搜索 H 标识")

        # 保持在 5m 悬停等待视觉检测
        self._fsm.loiter()

    def execute(self) -> Optional[str]:
        # 检查是否已着陆
        if self._fsm.bridge.is_landed():
            self._fsm.get_logger().info("PRECISION_LAND: 已着陆")
            return "COMPLETE"

        if self._landing_done:
            return "COMPLETE"

        # 获取降落目标
        landing_target = self._fsm.get_landing_target()
        has_target = landing_target and landing_target.get("confidence", 0) > 0.3

        if has_target:
            self._has_landing_target = True
            self._vision_lost_start = 0.0
        else:
            if self._has_landing_target:
                if self._vision_lost_start == 0.0:
                    self._vision_lost_start = time.time()

        # 视觉丢失超时 → 盲降
        vision_lost_to = self._fsm.get_param("vision_loss_timeout", 3.0)
        if self._has_landing_target and self._vision_lost_start > 0:
            lost_duration = time.time() - self._vision_lost_start
            if lost_duration > vision_lost_to:
                self._switch_to_blind()

        # 阶段逻辑
        if self._phase == self.PHASE_SEARCH:
            return self._execute_search(has_target)
        elif self._phase == self.PHASE_SERVO:
            return self._execute_servo(has_target)
        elif self._phase == self.PHASE_BLIND:
            return self._execute_blind()

        return None

    def _execute_search(self, has_target: bool) -> Optional[str]:
        """Phase 1: 搜索 H 标识."""
        if has_target:
            self._fsm.get_logger().info("PRECISION_LAND: 检测到 H 标识, 开始伺服下降")
            self._phase = self.PHASE_SERVO
            return None

        # 搜索超时 → 盲降
        elapsed = time.time() - self._start_time
        if elapsed > 15.0:
            self._fsm.get_logger().warn("PRECISION_LAND: H 标识搜索超时, 切换到盲降")
            self._switch_to_blind()
        return None

    def _execute_servo(self, has_target: bool) -> Optional[str]:
        """Phase 2: 视觉伺服对心下降."""
        pos = self._fsm.bridge.get_current_position()
        altitude = abs(pos.down)  # NED z 的绝对值 = 高度

        if not has_target:
            # 视觉丢失, 保持水平继续下降
            self._fsm.fc.set_velocity_ned(0.0, 0.0, 0.2)  # 缓慢下降
        else:
            # 视觉伺服会通过 vision_pipeline 计算速度
            # 同时飞行控制也缓慢下降
            target = self._fsm.get_landing_target()
            conf = target.get("confidence", 0)

            landing_speed = self._fsm.get_param("landing_approach_speed", 0.5)
            self._fsm.fc.set_velocity_ned(0.0, 0.0, landing_speed)

            if altitude < 2.0:
                self._fsm.get_logger().info(
                    f"PRECISION_LAND: 高度={altitude:.1f}m conf={conf:.2f}"
                )

        # 下降到盲降高度
        blind_alt = self._fsm.get_param("landing_blind_altitude", 0.5)
        if altitude < blind_alt:
            self._fsm.get_logger().info(
                f"PRECISION_LAND: 到达盲降高度 ({altitude:.2f}m)"
            )
            self._switch_to_blind()

        return None

    def _execute_blind(self) -> Optional[str]:
        """Phase 3: 盲降."""
        pos = self._fsm.bridge.get_current_position()
        altitude = abs(pos.down)

        # 最低下降速率
        self._fsm.fc.set_velocity_ned(0.0, 0.0, 0.15)

        if altitude < 0.15:
            self._fsm.get_logger().info("PRECISION_LAND: 着陆确认")
            self._fsm.bridge.disarm()
            self._landing_done = True
            return "COMPLETE"

        # 超时保护
        elapsed = time.time() - self._start_time
        if elapsed > 60.0:
            self._fsm.get_logger().warn("PRECISION_LAND: 超时, 强制着陆")
            self._fsm.bridge.force_disarm()
            self._landing_done = True
            return "COMPLETE"

        return None

    def _switch_to_blind(self):
        """切换到盲降模式."""
        self._phase = self.PHASE_BLIND
        self._fsm.get_logger().info("PRECISION_LAND: Phase 3 - 盲降模式")
        # 停用视觉伺服
        if self._fsm.vision_node:
            self._fsm.vision_node.set_servo_mode("none")

    def exit(self):
        if self._fsm.vision_node:
            self._fsm.vision_node.set_servo_mode("none")
        # 确保上锁
        try:
            self._fsm.bridge.disarm()
        except Exception:
            pass
        self._fsm.logger.log_state("PRECISION_LAND", "COMPLETE", "降落完成")
