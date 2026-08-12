"""
覆盖监控 — 侦察区覆盖范围跟踪.

原理: 将侦察区离散化为网格, 每个控制周期根据无人机位姿 + 相机 FOV
标记已覆盖的网格单元. 当覆盖率超过阈值时触发侦察完成.

发布:
  /drone/vision/coverage  (CoverageStatus)
"""

import rclpy
from rclpy.node import Node
from drone_interfaces.msg import CoverageStatus
from drone_utils.layout_frame import LayoutFrame
import numpy as np
from typing import Tuple, Set


class CoverageMonitor:
    """
    侦察区网格覆盖监控.

    Parameters
    ----------
    zone_forward_range : (float, float)
        侦察区场地局部 forward 范围 (min, max).
    zone_right_range : (float, float)
        侦察区场地局部 right 范围 (min, max).
    layout_frame : LayoutFrame
        不可变的场地方位坐标系.
    home_world_xy : (float, float)
        World 坐标中场地局部原点的位置.
    grid_resolution : float
        网格分辨率 (m).
    fov_half_angle_deg : float
        相机 FOV 半角 (度).
    complete_ratio : float
        全覆盖判定阈值.

    """

    def __init__(
        self,
        zone_forward_range: Tuple[float, float] = (55.0, 60.0),
        zone_right_range: Tuple[float, float] = (-4.0, 4.0),
        layout_frame: LayoutFrame = LayoutFrame(0.0),
        home_world_xy: Tuple[float, float] = (4.0, 10.0),
        grid_resolution: float = 0.5,
        fov_half_angle_deg: float = 35.0,
        complete_ratio: float = 0.95,
    ):
        self.zone_forward_min, self.zone_forward_max = zone_forward_range
        self.zone_right_min, self.zone_right_max = zone_right_range
        self.layout_frame = layout_frame
        self.home_world_x, self.home_world_y = home_world_xy
        self.grid_resolution = grid_resolution
        self.fov_half_angle = np.radians(fov_half_angle_deg)
        self.complete_ratio = complete_ratio

        # 构建网格
        self.nx = int(np.ceil(
            (self.zone_forward_max - self.zone_forward_min) / grid_resolution
        ))
        self.ny = int(np.ceil(
            (self.zone_right_max - self.zone_right_min) / grid_resolution
        ))
        self.grid = np.zeros((self.nx, self.ny), dtype=bool)
        self.total_cells = self.nx * self.ny
        self.covered_set: Set[Tuple[int, int]] = set()
        self._scan_active = False

    def reset(self):
        """重置覆盖状态."""
        self.grid.fill(False)
        self.covered_set.clear()
        self._scan_active = False

    def begin_scan(self):
        """Start a new reconnaissance scan and discard pre-scan observations."""
        self.reset()
        self._scan_active = True

    def end_scan(self):
        """Freeze the completed scan result while the aircraft leaves the zone."""
        self._scan_active = False

    def update(
        self,
        drone_x: float,
        drone_y: float,
        drone_z: float,  # 高度 (正=向上)
    ) -> float:
        """
        更新覆盖状态.

        Parameters
        ----------
        drone_x : float
            无人机 World x 坐标.
        drone_y : float
            无人机 World y 坐标.
        drone_z : float
            无人机高度 (m, 正值).

        Returns
        -------
        coverage_ratio : 当前覆盖率 [0, 1].

        """
        if not self._scan_active:
            return self.coverage_ratio

        altitude = drone_z
        if altitude <= 0.1:
            return self.coverage_ratio

        north = drone_y - self.home_world_y
        east = drone_x - self.home_world_x
        drone_forward, drone_right = self.layout_frame.ned_to_local(
            north, east,
        )

        # 相机在地面的覆盖半径
        ground_radius = altitude * np.tan(self.fov_half_angle)

        # 覆盖的网格范围
        forward_min_idx = int(np.floor(
            (drone_forward - ground_radius - self.zone_forward_min)
            / self.grid_resolution
        ))
        forward_max_idx = int(np.ceil(
            (drone_forward + ground_radius - self.zone_forward_min)
            / self.grid_resolution
        ))
        right_min_idx = int(np.floor(
            (drone_right - ground_radius - self.zone_right_min)
            / self.grid_resolution
        ))
        right_max_idx = int(np.ceil(
            (drone_right + ground_radius - self.zone_right_min)
            / self.grid_resolution
        ))

        # Clamp
        forward_min_idx = max(0, forward_min_idx)
        forward_max_idx = min(self.nx - 1, forward_max_idx)
        right_min_idx = max(0, right_min_idx)
        right_max_idx = min(self.ny - 1, right_max_idx)

        # 标记覆盖 (圆形区域内的网格)
        for ix in range(forward_min_idx, forward_max_idx + 1):
            for iy in range(right_min_idx, right_max_idx + 1):
                if self.grid[ix, iy]:
                    continue
                center_forward = (
                    self.zone_forward_min + (ix + 0.5) * self.grid_resolution
                )
                center_right = (
                    self.zone_right_min + (iy + 0.5) * self.grid_resolution
                )
                dist = np.hypot(
                    center_forward - drone_forward,
                    center_right - drone_right,
                )
                if dist <= ground_radius:
                    self.grid[ix, iy] = True
                    self.covered_set.add((ix, iy))

        return self.coverage_ratio

    @property
    def coverage_ratio(self) -> float:
        if self.total_cells == 0:
            return 1.0
        return len(self.covered_set) / self.total_cells

    @property
    def is_complete(self) -> bool:
        return self.coverage_ratio >= self.complete_ratio

    def uncovered_cells(self) -> np.ndarray:
        """返回未覆盖网格的世界坐标列表 (nx2)."""
        indices = np.argwhere(~self.grid)
        world_coords = np.zeros((len(indices), 2))
        for i, (ix, iy) in enumerate(indices):
            forward = self.zone_forward_min + (ix + 0.5) * self.grid_resolution
            right = self.zone_right_min + (iy + 0.5) * self.grid_resolution
            north, east = self.layout_frame.local_to_ned(forward, right)
            world_coords[i, 0] = self.home_world_x + east
            world_coords[i, 1] = self.home_world_y + north
        return world_coords


class CoverageMonitorNode(Node):
    """覆盖监控 ROS2 节点."""

    def __init__(self):
        super().__init__("coverage_monitor")

        self.declare_parameter("coverage_zone_forward_min", 55.0)
        self.declare_parameter("coverage_zone_forward_max", 60.0)
        self.declare_parameter("coverage_zone_right_min", -4.0)
        self.declare_parameter("coverage_zone_right_max", 4.0)
        self.declare_parameter("home_x", 4.0)
        self.declare_parameter("home_y", 10.0)
        self.declare_parameter("layout_heading_deg", 0.0)
        self.declare_parameter("coverage_grid_resolution", 0.5)
        self.declare_parameter("coverage_fov_half_angle", 35.0)
        self.declare_parameter("coverage_complete_ratio", 0.95)

        self._monitor = CoverageMonitor(
            zone_forward_range=(
                self.get_parameter("coverage_zone_forward_min").value,
                self.get_parameter("coverage_zone_forward_max").value,
            ),
            zone_right_range=(
                self.get_parameter("coverage_zone_right_min").value,
                self.get_parameter("coverage_zone_right_max").value,
            ),
            layout_frame=LayoutFrame(
                float(self.get_parameter("layout_heading_deg").value)
            ),
            home_world_xy=(
                self.get_parameter("home_x").value,
                self.get_parameter("home_y").value,
            ),
            grid_resolution=self.get_parameter(
                "coverage_grid_resolution"
            ).value,
            fov_half_angle_deg=self.get_parameter(
                "coverage_fov_half_angle"
            ).value,
            complete_ratio=self.get_parameter("coverage_complete_ratio").value,
        )

        # 发布
        self._cov_pub = self.create_publisher(
            CoverageStatus, "/drone/vision/coverage", 10,
        )

        # 2Hz 发布
        self._timer = self.create_timer(0.5, self._publish)

        self._current_pose = (0.0, 0.0, 0.0)  # (x, y, z) world

        self.get_logger().info("覆盖监控节点已启动")

    def _publish(self):
        drone_x, drone_y, drone_z = self._current_pose
        ratio = self._monitor.update(drone_x, drone_y, drone_z)

        msg = CoverageStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.coverage_ratio = ratio
        msg.is_complete = self._monitor.is_complete
        self._cov_pub.publish(msg)

    def update_pose(self, world_x: float, world_y: float, world_z: float):
        """由外部更新无人机位姿."""
        self._current_pose = (world_x, world_y, world_z)

    def reset(self):
        self._monitor.reset()


def main(args=None):
    rclpy.init(args=args)
    node = CoverageMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
