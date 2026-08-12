"""
路径规划 — 弓字形覆盖路径 + 区域间过渡路径.

侦察区弓字形 (Boustrophedon) 全覆盖:
  - 自动选择最优条带方向: 沿区域长边飞行, 条带沿短边分布
    这样转弯次数最少, 飞行效率最高
  - 条间距: grid_spacing (默认 1.67m), 配合相机 FOV 确保全覆盖
  - 飞行速度: sweep_speed (0.5-1.5 m/s)

过渡路径:
  - 投放区 → 侦察区 → 返航点
  - 直线飞行 + 巡航高度

坐标系: World (Gazebo: x=east, y=north, z=up)
         NED (PX4: north, east, down)
"""

import math
from typing import List, Tuple, Optional

from drone_utils.layout_frame import LayoutFrame


# ============================================================
# 路径规划器
# ============================================================

class PathPlanner:
    """
    路径规划器.

    使用方式:
      planner = PathPlanner(LayoutFrame(0.0), sweep_speed=1.2, transit_speed=2.5)
      waypoints = planner.plan_coverage(zone_bounds, altitude=5.0)
      waypoints = planner.plan_transit(from_xyz, to_xyz)
    """

    def __init__(
        self,
        layout_frame: LayoutFrame,
        sweep_speed: float = 0.8,
        transit_speed: float = 2.0,
        return_speed: float = 3.0,
        grid_spacing: float = 1.67,
        margin_forward: float = 1.0,
        margin_right: float = 0.5,
    ):
        self.layout_frame = layout_frame
        self.sweep_speed = sweep_speed
        self.transit_speed = transit_speed
        self.return_speed = return_speed
        self.grid_spacing = grid_spacing
        self.margin_forward = margin_forward
        self.margin_right = margin_right

    # ================================================================
    # 覆盖路径
    # ================================================================

    def plan_coverage(
        self,
        zone_bounds: dict,
        altitude: float = 5.0,
    ) -> List[dict]:
        """
        生成弓字形全覆盖航点 (World/Gazebo 坐标).

        自动选择最优条带方向: 沿区域长边飞行 → 转弯次数最少.

        Parameters
        ----------
        zone_bounds : dict
            {x_min, x_max, y_min, y_max}  — World 坐标 (Gazebo)
        altitude : float
            飞行高度 (World z, m)

        Returns
        -------
        list of dict: 每个航点为 {x, y, z, yaw, speed, hover_s, label}

        """
        x_min = zone_bounds["x_min"] + self.margin_right
        x_max = zone_bounds["x_max"] - self.margin_right
        y_min = zone_bounds["y_min"] + self.margin_forward
        y_max = zone_bounds["y_max"] - self.margin_forward

        x_range = x_max - x_min  # east 方向
        y_range = y_max - y_min  # north 方向

        if x_range <= 0 or y_range <= 0:
            return []

        # ---- 自动选择最优方向: 沿长边飞行, 条带沿短边分布 ----
        if y_range >= x_range:
            # 沿 y (north, 长边) 飞行, 条带沿 x (east, 短边) 分布
            fly_along = "y"
            strip_range = x_range
            strip_min = x_min
            strip_max = x_max
            flight_min = y_min
            flight_max = y_max
        else:
            # 沿 x (east, 长边) 飞行, 条带沿 y (north, 短边) 分布
            fly_along = "x"
            strip_range = y_range
            strip_min = y_min
            strip_max = y_max
            flight_min = x_min
            flight_max = x_max

        num_strips = max(1, math.ceil(strip_range / self.grid_spacing))
        actual_spacing = strip_range / max(1, num_strips - 1) if num_strips > 1 else 0.0

        waypoints = []

        for i in range(num_strips):
            strip_pos = strip_min + i * actual_spacing if num_strips > 1 else (
                strip_min + strip_max) / 2

            if i % 2 == 0:
                start_flight = flight_min
                end_flight = flight_max
            else:
                start_flight = flight_max
                end_flight = flight_min

            if fly_along == "y":
                waypoints.append({
                    "x": strip_pos, "y": start_flight, "z": altitude,
                    "yaw": 0.0 if i % 2 == 0 else math.pi,
                    "speed": self.sweep_speed,
                    "hover_s": 0.0,
                    "label": f"strip_{i}_enter",
                })
                waypoints.append({
                    "x": strip_pos, "y": end_flight, "z": altitude,
                    "yaw": 0.0 if i % 2 == 0 else math.pi,
                    "speed": self.sweep_speed,
                    "hover_s": 0.0,
                    "label": f"strip_{i}_exit",
                })
            else:
                waypoints.append({
                    "x": start_flight, "y": strip_pos, "z": altitude,
                    "yaw": math.pi / 2 if i % 2 == 0 else -math.pi / 2,
                    "speed": self.sweep_speed,
                    "hover_s": 0.0,
                    "label": f"strip_{i}_enter",
                })
                waypoints.append({
                    "x": end_flight, "y": strip_pos, "z": altitude,
                    "yaw": math.pi / 2 if i % 2 == 0 else -math.pi / 2,
                    "speed": self.sweep_speed,
                    "hover_s": 0.0,
                    "label": f"strip_{i}_exit",
                })

        return waypoints

    def plan_coverage_ned(
        self,
        area_forward_min: float,
        area_forward_max: float,
        area_right_min: float,
        area_right_max: float,
        altitude: float = 5.0,
    ) -> List[dict]:
        """
        生成 NED 坐标系下的弓字形全覆盖航点 (Boustrophedon 分解).

        自动选择最优条带方向: 沿区域长边飞行 → 转弯次数最少 → 效率最高.

        Parameters
        ----------
        area_forward_min, area_forward_max : float
            Field-local forward range (m).
        area_right_min, area_right_max : float
            Field-local right range (m).
        altitude : float
            NED down (负值=上升), e.g. -5.0 = 飞行高度 5m

        Returns
        -------
        list of dict: {north, east, down, yaw, speed, hover_s, label}
            沿长边飞行的弓字形航点序列

        """
        forward_range = area_forward_max - area_forward_min
        right_range = area_right_max - area_right_min

        if forward_range <= 0 or right_range <= 0:
            return []

        # ---- 自动选择最优方向: 沿长边飞行, 条带沿短边分布 ----
        if forward_range >= right_range:
            fly_along = "forward"
            strip_range = right_range
            strip_min = area_right_min
            strip_max = area_right_max
            flight_min = area_forward_min
            flight_max = area_forward_max
        else:
            fly_along = "right"
            strip_range = forward_range
            strip_min = area_forward_min
            strip_max = area_forward_max
            flight_min = area_right_min
            flight_max = area_right_max

        num_strips = max(1, math.ceil(strip_range / self.grid_spacing))
        actual_spacing = strip_range / max(1, num_strips - 1) if num_strips > 1 else 0.0

        waypoints = []

        def waypoint(forward: float, right: float, label: str) -> dict:
            north, east = self.layout_frame.local_to_ned(forward, right)
            return {
                "north": north,
                "east": east,
                "down": altitude,
                "yaw": self.layout_frame.yaw_rad,
                "speed": self.sweep_speed,
                "hover_s": 0.0,
                "label": label,
            }

        for i in range(num_strips):
            strip_pos = strip_min + i * actual_spacing if num_strips > 1 else (
                strip_min + strip_max) / 2

            # 偶数条带正向飞行, 奇数条带反向飞行 (zigzag)
            if i % 2 == 0:
                start_flight = flight_min
                end_flight = flight_max
            else:
                start_flight = flight_max
                end_flight = flight_min

            if fly_along == "forward":
                waypoints.append(waypoint(
                    start_flight, strip_pos, f"strip_{i}_enter"
                ))
                waypoints.append(waypoint(
                    end_flight, strip_pos, f"strip_{i}_exit"
                ))
            else:
                waypoints.append(waypoint(
                    strip_pos, start_flight, f"strip_{i}_enter"
                ))
                waypoints.append(waypoint(
                    strip_pos, end_flight, f"strip_{i}_exit"
                ))

        return waypoints

    # ================================================================
    # 过渡路径
    # ================================================================

    def plan_transit(
        self,
        from_xyz: Tuple[float, float, float],
        to_xyz: Tuple[float, float, float],
        speed: Optional[float] = None,
        cruise_altitude: Optional[float] = None,
    ) -> List[dict]:
        """
        生成两点间过渡航点 (World 坐标).

        策略: 先升/降至巡航高度 → 水平移动 → 再到目标高度 (如果需要).

        Parameters
        ----------
        from_xyz : (x, y, z)
            起始位置 (World)
        to_xyz : (x, y, z)
            目标位置 (World)
        speed : float or None
            飞行速度, None 使用 transit_speed
        cruise_altitude : float or None
            巡航高度 (World z). None 则使用目标高度.

        Returns
        -------
        list of dict: 1-3 个航点

        """
        if speed is None:
            speed = self.transit_speed

        waypoints = []

        # 如果起始和目标高度不同, 先水平移动再调高度
        from_z = from_xyz[2]
        to_z = to_xyz[2]

        if abs(from_z - to_z) > 0.5:
            # 中间航点: 目标水平位置 + 起始高度
            waypoints.append({
                "x": to_xyz[0], "y": to_xyz[1], "z": from_z,
                "yaw": 0.0, "speed": speed, "hover_s": 0.0,
                "label": "transit_horizontal",
            })

        # 最终航点
        waypoints.append({
            "x": to_xyz[0], "y": to_xyz[1], "z": to_z,
            "yaw": 0.0, "speed": speed, "hover_s": 0.0,
            "label": "transit_final",
        })

        return waypoints

    def plan_transit_ned(
        self,
        from_ned: Tuple[float, float, float],
        to_ned: Tuple[float, float, float],
        speed: Optional[float] = None,
    ) -> List[dict]:
        """
        生成 NED 坐标系下的过渡航点.

        Returns
        -------
        list of dict: {north, east, down, yaw, speed, hover_s, label}

        """
        if speed is None:
            speed = self.transit_speed

        return [{
            "north": to_ned[0],
            "east": to_ned[1],
            "down": to_ned[2],
            "yaw": 0.0,
            "speed": speed,
            "hover_s": 0.0,
            "label": "transit",
        }]

    # ================================================================
    # 返航路径
    # ================================================================

    def plan_return(
        self,
        from_xyz: Tuple[float, float, float],
        home_xyz: Tuple[float, float, float] = (0.0, 0.0, 5.0),  # 真机默认原点; SITL 由 caller 传入
    ) -> List[dict]:
        """
        生成返航航点 (World 坐标).

        先爬升至巡航高度, 再水平飞回家点.

        Parameters
        ----------
        from_xyz : (x, y, z)
            当前 World 坐标.
        home_xyz : (x, y, z)
            返航目标 World 坐标.

        Returns
        -------
        list of dict

        """
        return self.plan_transit(from_xyz, home_xyz, speed=self.return_speed)
