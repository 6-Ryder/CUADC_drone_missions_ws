"""
看门狗/心跳监测 — 监测关键节点和话题的存活状态.

监测项:
  - PX4 Bridge 心跳 → /fmu/out/vehicle_status 有更新
  - Vision Pipeline 心跳 → /drone/vision/cylinders 有更新
  - Servo Controller 心跳 → /drone/servo/status 有更新
  - Offboard 模式心跳 → setpoint 持续发布

若某项超时未更新, 触发告警回调.
"""

import time
from typing import Callable, Dict, Optional


class Watchdog:
    """
    多通道心跳监测器.

    使用方式:
      wd = Watchdog()
      wd.register("px4_status", timeout=3.0, on_timeout=lambda name: print(f"{name} lost"))
      wd.register("vision_cylinders", timeout=5.0)
      # 在主循环中:
      wd.feed("px4_status")
      alerts = wd.check()
    """

    def __init__(self):
        self._channels: Dict[str, dict] = {}
        # channel: {timeout, last_feed, on_timeout, alert_active}

    def register(
        self,
        name: str,
        timeout: float = 3.0,
        on_timeout: Optional[Callable[[str], None]] = None,
    ):
        """
        注册一个监测通道.

        Parameters
        ----------
        name : str
            通道名称 (用于日志和回调)
        timeout : float
            超时秒数, 超过则认为异常
        on_timeout : callable or None
            超时回调, 参数为通道名称

        """
        self._channels[name] = {
            "timeout": timeout,
            "last_feed": None,
            "on_timeout": on_timeout,
            "alert_active": False,
        }

    def unregister(self, name: str):
        """移除监测通道."""
        self._channels.pop(name, None)

    def feed(self, name: str):
        """喂狗 — 通知该通道仍活跃."""
        ch = self._channels.get(name)
        if ch:
            ch["last_feed"] = time.time()
            ch["alert_active"] = False

    def check(self) -> Dict[str, bool]:
        """
        检查所有通道状态.

        Returns
        -------
        dict: {channel_name: is_alive}

        """
        now = time.time()
        status = {}
        for name, ch in self._channels.items():
            last_feed = ch["last_feed"]
            alive = last_feed is not None and now - last_feed < ch["timeout"]
            status[name] = alive

            if not alive and not ch["alert_active"]:
                ch["alert_active"] = True
                if ch["on_timeout"]:
                    try:
                        ch["on_timeout"](name)
                    except Exception:
                        pass

        return status

    def is_alive(self, name: str) -> bool:
        """检查单个通道是否存活."""
        ch = self._channels.get(name)
        if not ch:
            return False
        last_feed = ch["last_feed"]
        return last_feed is not None and time.time() - last_feed < ch["timeout"]

    def all_alive(self) -> bool:
        """所有注册通道是否都存活."""
        return all(self.is_alive(name) for name in self._channels)

    def dead_channels(self) -> list:
        """返回所有超时的通道名称."""
        return [name for name in self._channels if not self.is_alive(name)]
