#!/usr/bin/env python3
"""Build and publish a simple accumulated terrain point cloud.

This node intentionally uses the already-available Parrot LiDAR point cloud and
odometry topics instead of the Nav2 costmap. The costmap is a navigation aid for
obstacle avoidance; it is not the right source for reconstructing a terrain
height map from previous scans. This node builds a lightweight map by
accumulating transformed LiDAR points into a fixed frame while the drone moves.
"""

from __future__ import annotations

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class TerrainMapBuilder:
    """Store a limited history of LiDAR points and render them as a cloud."""

    def __init__(
        self,
        max_points: int = 200000,
        resolution: float = 0.5,
        map_size_x: float = 10.0,
        map_size_y: float = 10.0,
    ) -> None:
        self.max_points = max_points
        self.resolution = max(float(resolution), 1e-6)
        self.map_size_x = float(map_size_x)
        self.map_size_y = float(map_size_y)
        self.grid_width = max(1, int(round(self.map_size_x / self.resolution)))
        self.grid_height = max(1, int(round(self.map_size_y / self.resolution)))
        self.grid = np.full((self.grid_height, self.grid_width), -1.0, dtype=np.float32)
        self._history: list[np.ndarray] = []

    def add_points(self, points: np.ndarray) -> None:
        if points is None or len(points) == 0:
            return

        arr = np.asarray(points, dtype=np.float64)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        if arr.shape[1] < 3:
            return

        arr = arr[:, :3]
        self._history.append(arr)
        merged = np.concatenate(self._history, axis=0) if len(self._history) > 1 else arr

        if merged.shape[0] > self.max_points:
            keep = merged[-self.max_points:]
            self._history = [keep]
            merged = keep

        x_min = -self.map_size_x / 2.0
        y_min = -self.map_size_y / 2.0
        xs = merged[:, 0]
        ys = merged[:, 1]
        zs = merged[:, 2]

        x_idx = np.floor((xs - x_min) / self.resolution).astype(int)
        y_idx = np.floor((ys - y_min) / self.resolution).astype(int)

        valid = (x_idx >= 0) & (x_idx < self.grid_width) & (y_idx >= 0) & (y_idx < self.grid_height)
        x_idx = x_idx[valid]
        y_idx = y_idx[valid]
        zs = zs[valid]

        for xi, yi, z in zip(x_idx, y_idx, zs):
            if self.grid[yi, xi] < 0.0 or z > self.grid[yi, xi]:
                self.grid[yi, xi] = float(z)

    @property
    def xyz(self) -> np.ndarray:
        if not self._history:
            return np.empty((0, 3), dtype=np.float64)
        return np.concatenate(self._history, axis=0)

    def as_cloud(self, frame_id: str, stamp) -> PointCloud2 | None:
        points = self.xyz
        if points.size == 0:
            return None

        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        cloud_points = [(float(x), float(y), float(z)) for x, y, z in points]
        return point_cloud2.create_cloud_xyz32(header, cloud_points)

    def reset(self) -> None:
        self._history = []
        self.grid.fill(-1.0)


class TerrainMapNode(Node):
    def __init__(self) -> None:
        super().__init__('terrain_map_builder')

        self.declare_parameter('robot_name', 'parrot1')
        self.declare_parameter('map_frame', 'parrot1_odom')
        self.declare_parameter('odometry_topic', 'odometry')
        self.declare_parameter('pointcloud_topic', 'scan/points')
        self.declare_parameter('output_topic', 'terrain_points')
        self.declare_parameter('history_limit', 200000)

        self.robot_name = self.get_parameter('robot_name').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_topic = self.get_parameter('odometry_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.history_limit = int(self.get_parameter('history_limit').value)

        if not self.map_frame:
            self.map_frame = f'{self.robot_name}_odom'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.terrain_builder = TerrainMapBuilder(max_points=self.history_limit)
        self.latest_odom_msg: Odometry | None = None
        self._current_accumulation_frame: str | None = None

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_callback, qos)
        self.lidar_sub = self.create_subscription(PointCloud2, self.pointcloud_topic, self._pointcloud_callback, qos)
        self.terrain_pub = self.create_publisher(PointCloud2, self.output_topic, qos)

        self.get_logger().info(
            f'Terrain accumulation started for robot={self.robot_name}; '
            f'using map frame={self.map_frame}, odom topic={self.odom_topic}, '
            f'point cloud topic={self.pointcloud_topic}, output={self.output_topic}'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom_msg = msg

    def _extract_xyz(self, msg: PointCloud2) -> np.ndarray:
        raw_points = point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        )

        if isinstance(raw_points, np.ndarray):
            arr = raw_points
        else:
            arr = np.asarray(list(raw_points))

        if arr.size == 0:
            return np.empty((0, 3), dtype=np.float64)

        if getattr(arr.dtype, 'names', None):
            return np.column_stack((arr['x'], arr['y'], arr['z'])).astype(np.float64, copy=False)

        arr = np.asarray(arr, dtype=np.float64)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        if arr.shape[1] < 3:
            return np.empty((0, 3), dtype=np.float64)
        return arr[:, :3]

    def _transform_to_accumulation_frame(self, msg: PointCloud2) -> tuple[PointCloud2, str]:
        target_frames = [self.map_frame]
        if self.latest_odom_msg is not None and self.latest_odom_msg.header.frame_id:
            odom_frame = self.latest_odom_msg.header.frame_id
            if odom_frame not in target_frames:
                target_frames.append(odom_frame)

        for target_frame in target_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2),
                )
                return do_transform_cloud(msg, transform), target_frame
            except Exception:
                continue

        fallback_frame = msg.header.frame_id or self.map_frame
        self.get_logger().warn(
            f'No TF available yet from {msg.header.frame_id} to {self.map_frame}; '
            f'using raw cloud frame {fallback_frame} for terrain accumulation.',
            throttle_duration_sec=5.0,
        )
        return msg, fallback_frame

    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        cloud_in_map, output_frame = self._transform_to_accumulation_frame(msg)
        try:
            xyz = self._extract_xyz(cloud_in_map)
        except Exception as exc:  # pragma: no cover - runtime interface issue
            self.get_logger().warn(f'Could not read transformed point cloud: {exc}', throttle_duration_sec=5.0)
            return

        if xyz.size == 0:
            return

        if xyz.ndim == 1:
            xyz = xyz.reshape(1, -1)

        # Avoid mixing point histories that were accumulated in different frames.
        if self._current_accumulation_frame is None:
            self._current_accumulation_frame = output_frame
        elif self._current_accumulation_frame != output_frame:
            self.get_logger().warn(
                f'Accumulation frame changed from {self._current_accumulation_frame} to {output_frame}; '
                'resetting terrain history to keep frame-consistent data.',
                throttle_duration_sec=5.0,
            )
            self.terrain_builder.reset()
            self._current_accumulation_frame = output_frame

        self.terrain_builder.add_points(xyz)

        accumulated = self.terrain_builder.as_cloud(output_frame, self.get_clock().now().to_msg())
        if accumulated is not None:
            self.terrain_pub.publish(accumulated)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
