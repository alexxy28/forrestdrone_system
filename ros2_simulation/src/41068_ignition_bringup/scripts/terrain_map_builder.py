#!/usr/bin/env python3
"""Build and publish an accumulated elevation grid as a point cloud.

Each Parrot LiDAR cloud is transformed into a fixed map frame and its highest
return is accumulated in a 0.5 m elevation cell. The Nav2 costmap is not used
because it is an obstacle-avoidance aid rather than a terrain height map.
"""

from __future__ import annotations

import numpy as np

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener


class TerrainMapBuilder:
    """Keep the highest observed LiDAR return in each fixed-frame map cell."""

    def __init__(
        self,
        resolution: float = 0.5,
        map_size_x: float = 40.0,
        map_size_y: float = 40.0,
    ) -> None:
        self.resolution = max(float(resolution), 1e-6)
        self.map_size_x = float(map_size_x)
        self.map_size_y = float(map_size_y)

        # Use an odd cell count so the cell at (0, 0) is centered exactly on
        # world/map origin. The requested size is the nominal full span.
        self.half_cells_x = max(
            0, int(np.ceil(self.map_size_x / (2.0 * self.resolution)))
        )
        self.half_cells_y = max(
            0, int(np.ceil(self.map_size_y / (2.0 * self.resolution)))
        )
        self.grid_width = 2 * self.half_cells_x + 1
        self.grid_height = 2 * self.half_cells_y + 1

        # NaN is the unknown-cell marker; valid elevations may be negative.
        self.grid = np.full(
            (self.grid_height, self.grid_width),
            np.nan,
            dtype=np.float32,
        )

    def add_points(self, points: np.ndarray) -> int:
        if points is None or len(points) == 0:
            return 0

        arr = np.asarray(points, dtype=np.float64)

        if arr.ndim == 1:
            arr = arr.reshape(1, -1)

        if arr.shape[1] < 3:
            return 0

        arr = arr[:, :3]
        arr = arr[np.isfinite(arr).all(axis=1)]
        if arr.size == 0:
            return 0

        xs = arr[:, 0]
        ys = arr[:, 1]
        zs = arr[:, 2]

        # Index cells by their centers; (0, 0) maps to the center cell rather
        # than sitting on a boundary between four cells.
        x_idx = np.floor(
            xs / self.resolution + self.half_cells_x + 0.5
        ).astype(int)
        y_idx = np.floor(
            ys / self.resolution + self.half_cells_y + 0.5
        ).astype(int)

        valid = (
            (x_idx >= 0)
            & (x_idx < self.grid_width)
            & (y_idx >= 0)
            & (y_idx < self.grid_height)
        )

        x_idx = x_idx[valid]
        y_idx = y_idx[valid]
        zs = zs[valid]

        for xi, yi, z in zip(x_idx, y_idx, zs):
            cell = self.grid[yi, xi]
            if not np.isfinite(cell) or z > cell:
                self.grid[yi, xi] = float(z)

        return int(zs.size)

    @property
    def xyz(self) -> np.ndarray:
        valid = np.isfinite(self.grid)
        if not valid.any():
            return np.empty((0, 3), dtype=np.float64)

        y_idx, x_idx = np.nonzero(valid)
        xs = (x_idx - self.half_cells_x) * self.resolution
        ys = (y_idx - self.half_cells_y) * self.resolution
        zs = self.grid[y_idx, x_idx].astype(np.float64)
        return np.column_stack((xs, ys, zs)).astype(np.float64, copy=False)

    def as_cloud(
        self,
        frame_id: str,
        stamp,
    ) -> PointCloud2 | None:

        points = self.xyz

        if points.size == 0:
            return None

        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id

        cloud_points = [
            (float(x), float(y), float(z))
            for x, y, z in points
        ]

        return point_cloud2.create_cloud_xyz32(
            header,
            cloud_points,
        )

    def reset(self) -> None:
        self.grid.fill(np.nan)


class TerrainMapNode(Node):
    SELF_RETURN_MIN_RANGE_M = 0.6
    LOW_OUTLIER_MIN_DEVIATION_M = 1.0
    HIGH_OUTLIER_MIN_DEVIATION_M = 4.0
    OUTLIER_ROBUST_SIGMAS = 6.0

    def __init__(self) -> None:
        super().__init__('terrain_map_builder')

        self.declare_parameter('robot_name', 'parrot1')
        self.declare_parameter('map_frame', 'parrot1_odom')
        self.declare_parameter('pointcloud_topic', 'scan/points')
        self.declare_parameter('output_topic', 'terrain_points')
        self.declare_parameter('odometry_topic', '/parrot1/odometry')

        self.robot_name = self.get_parameter(
            'robot_name'
        ).value

        self.map_frame = self.get_parameter(
            'map_frame'
        ).value

        self.pointcloud_topic = self.get_parameter(
            'pointcloud_topic'
        ).value

        self.output_topic = self.get_parameter(
            'output_topic'
        ).value
        self.odometry_topic = self.get_parameter(
            'odometry_topic'
        ).value
        self.latest_odometry: Odometry | None = None

        if not self.map_frame:
            self.map_frame = f'{self.robot_name}_odom'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.terrain_builder = TerrainMapBuilder()

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self._pointcloud_callback,
            qos,
        )
        self.odometry_sub = self.create_subscription(
            Odometry,
            self.odometry_topic,
            self._odometry_callback,
            qos,
        )

        self.terrain_pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos,
        )

        self.get_logger().info(
            f'Terrain accumulation started for robot={self.robot_name}; '
            f'map frame={self.map_frame}, '
            f'point cloud topic={self.lidar_sub.topic_name}, '
            f'odometry topic={self.odometry_sub.topic_name}, '
            f'output topic={self.terrain_pub.topic_name}'
        )

    def _odometry_callback(self, msg: Odometry) -> None:
        self.latest_odometry = msg

    def _extract_xyz(
        self,
        msg: PointCloud2,
    ) -> np.ndarray:

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
            return np.empty(
                (0, 3),
                dtype=np.float64,
            )

        if getattr(arr.dtype, 'names', None):
            return np.column_stack(
                (
                    arr['x'],
                    arr['y'],
                    arr['z'],
                )
            ).astype(
                np.float64,
                copy=False,
            )

        arr = np.asarray(
            arr,
            dtype=np.float64,
        )

        if arr.ndim == 1:
            arr = arr.reshape(1, -1)

        if arr.shape[1] < 3:
            return np.empty(
                (0, 3),
                dtype=np.float64,
            )

        return arr[:, :3]

    @staticmethod
    def _rotation_matrix(quaternion) -> np.ndarray:
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        return np.array(
            [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w),
                ],
                [
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w),
                ],
                [
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y),
                ],
            ],
            dtype=np.float64,
        )

    def _transform_xyz_cloud(
        self,
        msg: PointCloud2,
        rotation_matrix: np.ndarray,
        translation: np.ndarray,
        target_frame: str,
    ) -> PointCloud2:
        """
        Transform only XYZ fields from the incoming cloud.

        The Parrot LiDAR PointCloud2 contains additional fields whose layout
        is incompatible with tf2_sensor_msgs.do_transform_cloud() in this
        environment. Terrain mapping only requires XYZ, so those fields are
        intentionally discarded here.
        """

        xyz = self._extract_xyz(msg)

        # LiDAR is mounted above the airframe. Remove very near returns in
        # the sensor's own frame before transforming/accumulating: these are
        # within the body envelope and otherwise paint a high streak that
        # follows the drone. Terrain at the configured flight altitude is
        # several metres away and is unaffected.
        if xyz.size:
            ranges = np.linalg.norm(xyz, axis=1)
            xyz = xyz[ranges >= self.SELF_RETURN_MIN_RANGE_M]

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = target_frame

        if xyz.size == 0:
            return point_cloud2.create_cloud_xyz32(
                header,
                [],
            )

        transformed_points = xyz @ rotation_matrix.T + translation

        return point_cloud2.create_cloud_xyz32(
            header,
            transformed_points.tolist(),
        )

    def _filter_height_outliers(
        self,
        xyz: np.ndarray,
    ) -> tuple[np.ndarray, int]:
        """Reject isolated extreme scan heights without clipping tree returns.

        The lower side has a 1 m minimum allowance to reject isolated returns
        from the ground plane beyond the terrain near the map edge. The upper
        side keeps a 4 m allowance so tree canopy returns remain visible.
        """
        if xyz.shape[0] < 12:
            return xyz, 0

        heights = xyz[:, 2]
        median = float(np.median(heights))
        mad = float(np.median(np.abs(heights - median)))
        robust_sigma = 1.4826 * mad
        deviation_limit = self.OUTLIER_ROBUST_SIGMAS * robust_sigma
        lower_limit = median - max(
            self.LOW_OUTLIER_MIN_DEVIATION_M,
            deviation_limit,
        )
        upper_limit = median + max(
            self.HIGH_OUTLIER_MIN_DEVIATION_M,
            deviation_limit,
        )
        inlier_mask = (heights >= lower_limit) & (heights <= upper_limit)
        return xyz[inlier_mask], int(xyz.shape[0] - inlier_mask.sum())

    def _transform_to_accumulation_frame(
        self,
        msg: PointCloud2,
    ) -> PointCloud2 | None:
        if not msg.header.frame_id:
            self.get_logger().warning(
                'Dropping LiDAR cloud with an empty frame_id.',
                throttle_duration_sec=5.0,
            )
            return None

        odometry = self.latest_odometry
        if odometry is None:
            self.get_logger().warning(
                'Dropping LiDAR cloud: waiting for raw Parrot odometry.',
                throttle_duration_sec=5.0,
            )
            return None

        if odometry.header.frame_id != self.map_frame:
            self.get_logger().error(
                f'Dropping LiDAR cloud: raw odometry frame '
                f'{odometry.header.frame_id!r} does not match map frame '
                f'{self.map_frame!r}.',
                throttle_duration_sec=5.0,
            )
            return None

        if not odometry.child_frame_id:
            self.get_logger().warning(
                'Dropping LiDAR cloud: raw odometry has an empty child_frame_id.',
                throttle_duration_sec=5.0,
            )
            return None

        try:
            cloud_stamp = rclpy.time.Time.from_msg(
                msg.header.stamp,
                clock_type=self.get_clock().clock_type,
            )
            odometry_stamp = rclpy.time.Time.from_msg(
                odometry.header.stamp,
                clock_type=self.get_clock().clock_type,
            )
            if abs((odometry_stamp - cloud_stamp).nanoseconds) > 250_000_000:
                self.get_logger().warning(
                    'Dropping LiDAR cloud: raw odometry is more than 0.25 s '
                    'from the scan timestamp.',
                    throttle_duration_sec=5.0,
                )
                return None

            # Use only the static base_link -> LiDAR mount from TF. The
            # dynamic robot pose comes from Gazebo's raw odometry: the EKF
            # TF in this setup can report a tilted/offset pose, which adds
            # flight altitude and attitude error into terrain cell heights.
            sensor_mount = self.tf_buffer.lookup_transform(
                odometry.child_frame_id,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except Exception as exc:
            self.get_logger().warning(
                f'Dropping LiDAR cloud: static transform '
                f'{msg.header.frame_id} -> {odometry.child_frame_id} '
                f'unavailable: {exc}',
                throttle_duration_sec=5.0,
            )
            return None

        base_pose = odometry.pose.pose
        base_rotation = self._rotation_matrix(base_pose.orientation)
        mount_rotation = self._rotation_matrix(
            sensor_mount.transform.rotation
        )
        mount_translation = np.array(
            [
                sensor_mount.transform.translation.x,
                sensor_mount.transform.translation.y,
                sensor_mount.transform.translation.z,
            ],
            dtype=np.float64,
        )
        base_translation = np.array(
            [
                base_pose.position.x,
                base_pose.position.y,
                base_pose.position.z,
            ],
            dtype=np.float64,
        )

        # Compose map->base odometry with the static base->sensor mount.
        rotation = base_rotation @ mount_rotation
        translation = base_translation + base_rotation @ mount_translation
        return self._transform_xyz_cloud(
            msg, rotation, translation, self.map_frame
        )

    def _pointcloud_callback(
        self,
        msg: PointCloud2,
    ) -> None:

        cloud_in_map = self._transform_to_accumulation_frame(msg)
        if cloud_in_map is None:
            return

        try:
            xyz = self._extract_xyz(
                cloud_in_map
            )

        except Exception as exc:
            self.get_logger().warn(
                f'Could not read transformed '
                f'point cloud: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        if xyz.size == 0:
            return

        if xyz.ndim == 1:
            xyz = xyz.reshape(1, -1)

        finite = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite]
        if xyz.size == 0:
            return
        xyz, outliers_removed = self._filter_height_outliers(xyz)
        if xyz.size == 0:
            self.get_logger().warning(
                'All LiDAR returns were rejected by the height outlier filter.',
                throttle_duration_sec=5.0,
            )
            return
        mapped_returns = self.terrain_builder.add_points(xyz)

        known = np.isfinite(self.terrain_builder.grid)
        heights = self.terrain_builder.grid[known]
        if mapped_returns == 0:
            self.get_logger().warning(
                f'Received {xyz.shape[0]} finite points in frame '
                f'{cloud_in_map.header.frame_id}, but none fell inside '
                f'the map bounds X=[{-((self.terrain_builder.half_cells_x + 0.5) * self.terrain_builder.resolution):.1f}, '
                f'{((self.terrain_builder.half_cells_x + 0.5) * self.terrain_builder.resolution):.1f}] m, '
                f'Y=[{-((self.terrain_builder.half_cells_y + 0.5) * self.terrain_builder.resolution):.1f}, '
                f'{((self.terrain_builder.half_cells_y + 0.5) * self.terrain_builder.resolution):.1f}] m.',
                throttle_duration_sec=5.0,
            )
        elif heights.size:
            self.get_logger().info(
                f'Accumulated {xyz.shape[0]} finite LiDAR returns '
                f'({mapped_returns} within map); {heights.size} mapped cells, '
                f'height range {float(np.min(heights)):.3f}..'
                f'{float(np.max(heights)):.3f} m; '
                f'rejected {outliers_removed} height outliers',
                throttle_duration_sec=5.0,
            )

        accumulated = (
            self.terrain_builder.as_cloud(
                self.map_frame,
                self.get_clock().now().to_msg(),
            )
        )

        if accumulated is not None:
            self.terrain_pub.publish(accumulated)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = TerrainMapNode()

    try:
        rclpy.spin(node)

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
