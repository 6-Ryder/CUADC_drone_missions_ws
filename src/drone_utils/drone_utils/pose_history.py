"""
按**采集时刻**取载机位姿的小环形缓冲.

为什么需要它
------------
视觉测量有 0.2-0.33s (3-5Hz) 的采集→使用延迟. 用**处理时刻**的载机位置与航向
去解析一帧旧图, 会引入两类误差:

  - 平移: 误差 ≈ v·τ. 横移 0.25 m/s、τ=0.3s ⇒ 每帧 **7.5cm** —— 对 Ø15 的
    1cm 级验收是数量级差距.
  - 旋转: 机体系偏移要按**拍摄时**的航向旋转到 NED; 用当前航向去转, 误差
    ≈ |偏移|·Δψ, 且方向随航向变化 —— 这正是本项目"固定方向偏置"那类问题的
    来源, 而且**卡方门抓不到它** (门只拒零均值离群, 不拒系统偏差).

权威对应物: PX4 VTE 要求测量携带 `timestamp_sample` 与**采集时刻姿态四元数**
(`FiducialMarkerPosReport.msg:8-14`); ArduPilot 把 Tbn 存进惯性环形缓冲并在
测量真实时刻融合 (`AC_PrecLand.cpp:224-236,500-510`); EKF2 用 `EKF2_EV_DELAY`
把外部视觉量测搬到 IMU 时间轴上. 本项目没有惯性输入可用, 所以退一步: 用**载机
自身位姿的历史**做同样的时间对齐 (位置线性插值, 航向按最短弧).

与 drone_vision.capture_pose.interpolate_pose 的关系: 那个服务于桶口投影, 吃的是
四元数历史; 本模块服务于降落链路, 吃的是 NED 位置 + 偏航, 且**同样**在找不到
足够近的样本时返回 None (失败方向必须是"当作拿不到", 而不是硬用一个错的值).
"""

import math
from collections import deque
from typing import Optional, Tuple


class PoseHistory:
    """
    环形缓冲: (时刻, north, east, down, yaw) → 任意时刻的插值位姿.

    Parameters
    ----------
    maxlen : int
        缓冲长度. 20Hz 控制下 100 条约 5s, 足够覆盖 3-5Hz 视觉的观测龄.
    max_gap : float
        允许的"最近样本"距离 (s). 超出即返回 None —— 宁可让调用方退回到
        "当作拿不到采集时刻位姿", 也不要拿一个几秒前的位姿冒充.
    """

    def __init__(self, maxlen: int = 100, max_gap: float = 0.15):
        self.max_gap = float(max_gap)
        self._items = deque(maxlen=max(int(maxlen), 2))

    def __len__(self) -> int:
        return len(self._items)

    def reset(self) -> None:
        self._items.clear()

    def add(self, stamp: float, north: float, east: float, down: float,
            yaw: float) -> None:
        """采样一次位姿. 时刻非单调或非有限则丢弃 (乱序样本会破坏插值)."""
        if not all(math.isfinite(v) for v in (stamp, north, east, down, yaw)):
            return
        if self._items and stamp <= self._items[-1][0]:
            return
        self._items.append((float(stamp), float(north), float(east),
                            float(down), float(yaw)))

    def at(self, stamp: float) -> Optional[Tuple[float, float, float, float]]:
        """
        取 `stamp` 时刻的位姿 (north, east, down, yaw); 取不到返回 None.

        取不到的情形: 时刻非法 / 缓冲为空 / 前后最近的样本都超过 max_gap.
        """
        if not math.isfinite(stamp) or stamp <= 0.0 or not self._items:
            return None

        before = after = None
        for item in self._items:
            if item[0] <= stamp:
                before = item
            if item[0] >= stamp:
                after = item
                break

        if before is None:
            if after is None:
                return None
            return after[1:] if after[0] - stamp <= self.max_gap else None
        if after is None:
            return before[1:] if stamp - before[0] <= self.max_gap else None
        if stamp - before[0] > self.max_gap + 1e-9:
            return None
        if after[0] - stamp > self.max_gap + 1e-9:
            return None
        if before[0] == after[0]:
            return before[1:]

        alpha = (stamp - before[0]) / (after[0] - before[0])
        north = before[1] + alpha * (after[1] - before[1])
        east = before[2] + alpha * (after[2] - before[2])
        down = before[3] + alpha * (after[3] - before[3])
        yaw = _interpolate_angle(before[4], after[4], alpha)
        return (north, east, down, yaw)


def _interpolate_angle(a: float, b: float, alpha: float) -> float:
    """角度按**最短弧**插值 —— 直接线性插值在 ±π 附近会绕远路."""
    diff = math.atan2(math.sin(b - a), math.cos(b - a))
    return a + alpha * diff
