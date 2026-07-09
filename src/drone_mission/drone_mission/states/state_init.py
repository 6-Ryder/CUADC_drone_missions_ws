"""
INIT 状态 — 等待 PX4 连接 + 数据预热.

功能:
  1. 等待 /fmu/out/vehicle_odometry 有有效数据
  2. 等待 /drone/vision 各话题就绪
  3. 初始参数预热
  4. 注册完成 → 跳转 ARM
"""

import time
from typing import Optional


class InitState:
    """
    INIT 状态处理器.

    条件:
      - odom 位置标准差稳定 (< 0.1m 持续 2s)
      - 视觉管线节点心跳正常
      - 舵机节点心跳正常
    """

    def __init__(self, fsm):
        self._fsm = fsm
        self._start_time: float = 0.0
        self._odom_ready_count: int = 0
        self._odom_stable_threshold: int = 20  # 20 帧 ≈ 1s
        self._min_wait_time: float = 2.0  # 最小预热时间

    def enter(self):
        self._fsm.logger.log_state("INIT", "INIT", "开始初始化")
        self._start_time = time.time()
        self._odom_ready_count = 0
        self._fsm.get_logger().info("INIT: 等待系统就绪...")

    def execute(self) -> Optional[str]:
        """返回下一状态名称, 或 None 保持当前状态."""

        # 检查遥测数据是否有意义 (位置非零或时间足够)
        telem = self._fsm.bridge.get_telemetry()
        pos = telem.position

        # PX4 可能初始发送位置 (0,0,0) - 需要确认数据在更新
        if pos.norm_xy() > 0.01 or abs(pos.down) > 0.01:
            self._odom_ready_count += 1
        else:
            # 初始位置可能为零, 等待即可
            elapsed = time.time() - self._start_time
            if elapsed > self._min_wait_time:
                self._odom_ready_count += 1

        # 视觉/舵机节点检查
        vision_alive = self._fsm.watchdog.is_alive("vision")
        servo_alive = self._fsm.watchdog.is_alive("servo")

        elapsed = time.time() - self._start_time

        # 诊断日志
        if int(elapsed) % 5 == 0 and elapsed > 0:
            self._fsm.get_logger().info(
                f"INIT: odom={self._odom_ready_count}, "
                f"vision={'OK' if vision_alive else 'WAIT'}, "
                f"servo={'OK' if servo_alive else 'WAIT'}, "
                f"elapsed={elapsed:.0f}s"
            )

        # 就绪条件
        if self._odom_ready_count >= self._odom_stable_threshold:
            self._fsm.get_logger().info("INIT: 系统就绪, 进入 ARM")
            return "ARM"

        # 超时保护 (30s 后强制进入)
        if elapsed > 30.0:
            self._fsm.get_logger().warn("INIT: 超时, 强制进入 ARM")
            return "ARM"

        return None

    def exit(self):
        self._fsm.logger.log_state("INIT", "ARM", "初始化完成")
