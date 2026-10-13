"""
丢失视觉目标后, 继续飞向"最后已知目标位置"的横向指令生成器.

为什么需要它
------------
权威做法 (PX4 `navigator/precland.cpp:584-597`): 水平接近途中丢失目标**不悬停、
不判失败**, 继续飞向最后目击位置, 到达该点仍看不到才评估丢失. 代码注释原文是
"Even if we don't see it as we're moving towards it, continue approaching last
known location"。

本项目此前一律悬停并等丢失计时器, 于是"拒绝一帧 → 悬停 → 超时 → 接管/异地
降落"成为唯一路径 (2026-08-12/14 架次). 本类补上"继续接近"这一半。

三条不引入新误差来源的设计约束
------------------------------
1. **锁存世界坐标点, 不锁存机体系误差**: 机体系误差随航向变化会失效, 世界点
   与航向无关. 换航向时机体系指令每 tick 重算, 锁存点不动。
2. **偏移量直接取 ServoPid 已算好的带符号误差** (`error_forward`/`error_right`),
   不再做一次像素→米换算 —— 避免出现第二套符号/标定约定, 这是本项目历史上
   "固定方向偏置"类问题的来源。
3. **指令仍走速度域** (`set_velocity_body`), 不切换到位置 setpoint —— 避开
   `px4_bridge._velocity_mode` 粘性标志与位置通道互相作废的已知陷阱。

到点后输出零速度 (不是继续按最后方向飞), 因此不会退化成持续漂移; 横向移动
期间纵轴由调用方决定 (降落链路保持不下降, 与权威的水平接近段一致)。
"""

import math
from typing import Optional, Tuple


class LostTargetTransit:
    """
    最后已知目标位置的锁存与追踪 (纯逻辑, 不依赖 ROS).

    Parameters
    ----------
    max_speed : float
        横向接近速度上限 (m/s). 应显著低于正常伺服上限 —— 这是在**没有测量**
        的情况下飞行, 保守优先。
    tolerance : float
        到点容差 (m). 剩余距离小于它即输出零速度。
    """

    def __init__(self, max_speed: float = 0.15, tolerance: float = 0.15):
        if not math.isfinite(max_speed) or max_speed <= 0.0:
            raise ValueError("max_speed 必须为有限正值")
        if not math.isfinite(tolerance) or tolerance < 0.0:
            raise ValueError("tolerance 必须为有限非负值")
        self.max_speed = float(max_speed)
        self.tolerance = float(tolerance)
        self._north: Optional[float] = None
        self._east: Optional[float] = None

    @property
    def has_target(self) -> bool:
        """是否已锁存过最后已知位置."""
        return self._north is not None and self._east is not None

    @property
    def target(self) -> Optional[Tuple[float, float]]:
        """锁存点 (north, east); 未锁存时为 None."""
        north, east = self._north, self._east
        if north is None or east is None:
            return None
        return (north, east)

    def reset(self) -> None:
        """清除锁存 —— 换目标/重新捕获/状态换相时调用.

        注意: 与 ServoPid.reset 同寿命. 跨相保留锁存会让"上一段看到的点"
        在换相后继续驱动, 属于陈旧目标。
        """
        self._north = None
        self._east = None

    def latch(
        self,
        north: float,
        east: float,
        yaw: float,
        error_forward: float,
        error_right: float,
    ) -> None:
        """
        用一次有效测量刷新"最后已知目标位置".

        Parameters
        ----------
        north, east : float
            本次测量时刻载机自身的 NED 位置 (m).
        yaw : float
            本次测量时刻载机航向 (rad, NED). 必须与该次测量的误差同时刻 ——
            用当前航向去解析旧误差会引入与转动速率成正比的固定方向偏差。
        error_forward, error_right : float
            ServoPid 已算好的**带符号**机体系误差 (m), 即"从载机指向目标"
            的横向偏移 (前/右为正). 直接取自 ServoPidStatus, 不再自行换算。
        """
        if not all(math.isfinite(v) for v in
                   (north, east, yaw, error_forward, error_right)):
            return
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        # 机体系 → NED, 与 flight_controller.set_velocity_body 同一约定:
        #   vn = f*cos - r*sin ; ve = f*sin + r*cos
        self._north = float(north) + error_forward * cos_yaw - error_right * sin_yaw
        self._east = float(east) + error_forward * sin_yaw + error_right * cos_yaw

    def command(self, north: float, east: float, yaw: float) -> Tuple[float, float]:
        """
        计算(tick)飞向最后已知位置的机体系横向速度 (forward, right) m/s.

        未锁存过目标时返回 (0, 0) —— 失败方向是"不动作", 与调用方原有的
        悬停行为一致, 不会凭空产生指令。
        """
        target_north, target_east = self._north, self._east
        if target_north is None or target_east is None:
            return (0.0, 0.0)
        if not all(math.isfinite(v) for v in (north, east, yaw)):
            return (0.0, 0.0)

        dn = target_north - float(north)
        de = target_east - float(east)
        distance = math.hypot(dn, de)
        if distance <= self.tolerance:
            return (0.0, 0.0)

        # NED 误差 → 机体系 (上式的逆, 即旋转矩阵转置):
        #   f = dn*cos + de*sin ; r = -dn*sin + de*cos
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        forward = dn * cos_yaw + de * sin_yaw
        right = -dn * sin_yaw + de * cos_yaw

        norm = math.hypot(forward, right)
        if norm <= 1e-9:
            return (0.0, 0.0)
        # 按向量模长限幅, 且按剩余距离收尾, 避免到点附近来回冲
        speed = min(self.max_speed, distance)
        scale = speed / norm
        return (forward * scale, right * scale)
