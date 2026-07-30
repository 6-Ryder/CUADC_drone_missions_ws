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
import numpy as np
from typing import Tuple, Set


class CoverageMonitor:
    """
    侦察区网格覆盖监控.

    Parameters
    ----------
    zone_x_range : (float, float)
        侦察区 World x 范围 (min, max).
    zone_y_range : (float, float)
        侦察区 World y 范围 (min, max).
    grid_resolution : float
        网格分辨率 (m).
    fov_half_angle_deg : float
        相机 FOV 半角 (度).
    complete_ratio : float
        全覆盖判定阈值.

    """

    def __init__(
        self,
        zone_x_range: Tuple[float, float] = (0.0, 8.0),
        zone_y_range: Tuple[float, float] = (65.0, 70.0),
        grid_resolution: float = 0.5,
        fov_half_angle_deg: float = 35.0,
        complete_ratio: float = 0.95,
    ):
        self.zone_x_min, self.zone_x_max = zone_x_range
        self.zone_y_min, self.zone_y_max = zone_y_range
        self.grid_resolution = grid_resolution
        self.fov_half_angle = np.radians(fov_half_angle_deg)
        self.complete_ratio = complete_ratio

        # 构建网格
        self.nx = int(np.ceil((self.zone_x_max - self.zone_x_min) / grid_resolution))
        self.ny = int(np.ceil((self.zone_y_max - self.zone_y_min) / grid_resolution))
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

        # 相机在地面的覆盖半径
        ground_radius = altitude * np.tan(self.fov_half_angle)

        # 覆盖的网格范围
        x_min_idx = int(np.floor(
            (drone_x - ground_radius - self.zone_x_min) / self.grid_resolution
        ))
        x_max_idx = int(np.ceil(
            (drone_x + ground_radius - self.zone_x_min) / self.grid_resolution
        ))
        y_min_idx = int(np.floor(
            (drone_y - ground_radius - self.zone_y_min) / self.grid_resolution
        ))
        y_max_idx = int(np.ceil(
            (drone_y + ground_radius - self.zone_y_min) / self.grid_resolution
        ))

        # Clamp
        x_min_idx = max(0, x_min_idx)
        x_max_idx = min(self.nx - 1, x_max_idx)
        y_min_idx = max(0, y_min_idx)
        y_max_idx = min(self.ny - 1, y_max_idx)

        # 标记覆盖 (圆形区域内的网格)
        for ix in range(x_min_idx, x_max_idx + 1):
            for iy in range(y_min_idx, y_max_idx + 1):
                if self.grid[ix, iy]:
                    continue
                # 网格中心世界坐标
                cx = self.zone_x_min + (ix + 0.5) * self.grid_resolution
                cy = self.zone_y_min + (iy + 0.5) * self.grid_resolution
                dist = np.hypot(cx - drone_x, cy - drone_y)
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
            world_coords[i, 0] = self.zone_x_min + (ix + 0.5) * self.grid_resolution
            world_coords[i, 1] = self.zone_y_min + (iy + 0.5) * self.grid_resolution
        return world_coords


class CoverageMonitorNode(Node):
    """覆盖监控 ROS2 节点."""

    def __init__(self):
        super().__init__("coverage_monitor")

        # 侦察区 World 坐标 (北向布局: Home X=4,Y=10; Recon NED north=55-60 east=±4)
        self.declare_parameter("coverage_zone_x_min", 0.0)
        self.declare_parameter("coverage_zone_x_max", 8.0)
        self.declare_parameter("coverage_zone_y_min", 65.0)
        self.declare_parameter("coverage_zone_y_max", 70.0)
        self.declare_parameter("coverage_grid_resolution", 0.5)
        self.declare_parameter("coverage_fov_half_angle", 35.0)
        self.declare_parameter("coverage_complete_ratio", 0.95)

        self._monitor = CoverageMonitor(
            zone_x_range=(
                self.get_parameter("coverage_zone_x_min").value,
                self.get_parameter("coverage_zone_x_max").value,
            ),
            zone_y_range=(
                self.get_parameter("coverage_zone_y_min").value,
                self.get_parameter("coverage_zone_y_max").value,
            ),
            grid_resolution=self.get_parameter("coverage_grid_resolution").value,
            fov_half_angle_deg=self.get_parameter("coverage_fov_half_angle").value,
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
