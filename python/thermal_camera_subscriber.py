#!/usr/bin/env python3

"""Display Parrot thermal and depth ROS 2 image topics in separate windows."""

import cv2
import math
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry


THERMAL_TOPIC = "/parrot1/thermal_camera"
DEPTH_TOPIC = "/parrot1/camera/depth/image"
ODOMETRY_TOPIC = "/parrot1/odometry"
HEAT_THRESHOLD_C = 30.0
HORIZONTAL_FOV_DEG = 120.0
CAMERA_HEIGHT_M = 0.25
CAMERA_PITCH_DEG = 20.0
SYNC_TOLERANCE_S = 0.5


class CameraViewer(Node):
    def __init__(self) -> None:
        super().__init__("thermal_depth_viewer")
        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_stamp = None
        self.latest_odom = None
        self.latest_odom_stamp = None
        self.last_heat_log_time = 0.0
        self.last_heat_pixel = None
        self.create_subscription(Image, THERMAL_TOPIC, self.thermal_callback, 1)
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 1)
        self.create_subscription(
            Odometry, ODOMETRY_TOPIC, self.odometry_callback, 1
        )
        self.thermal_window = "Thermal Camera"
        self.depth_window = "Depth Camera"
        cv2.namedWindow(self.thermal_window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.depth_window, cv2.WINDOW_NORMAL)
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(
            placeholder,
            "Waiting for thermal frames...",
            (120, 240),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(self.thermal_window, placeholder)
        cv2.putText(
            placeholder,
            "Waiting for depth frames...",
            (120, 280),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(self.depth_window, placeholder)
        self.create_timer(0.03, lambda: cv2.waitKey(1))
        self.get_logger().info(
            f"Subscribed to {THERMAL_TOPIC}, {DEPTH_TOPIC}, and {ODOMETRY_TOPIC}"
        )

    def odometry_callback(self, message: Odometry) -> None:
        self.latest_odom = message
        self.latest_odom_stamp = self._stamp_seconds(message)
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        roll, pitch, yaw = self._euler_from_quaternion(orientation)
        self.get_logger().info(
            f"Pose {message.header.frame_id} -> {message.child_frame_id}: "
            f"xyz=({position.x:.2f}, {position.y:.2f}, {position.z:.2f}), "
            f"rpy=({math.degrees(roll):.1f}, {math.degrees(pitch):.1f}, "
            f"{math.degrees(yaw):.1f})deg, "
            f"quaternion=({orientation.x:.3f}, {orientation.y:.3f}, "
            f"{orientation.z:.3f}, {orientation.w:.3f})",
            throttle_duration_sec=1.0,
        )

    @staticmethod
    def _stamp_seconds(message) -> float:
        return float(message.header.stamp.sec) + message.header.stamp.nanosec * 1e-9

    @staticmethod
    def _euler_from_quaternion(quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        roll = math.atan2(
            2.0 * (w * x + y * z),
            1.0 - 2.0 * (x * x + y * y),
        )
        pitch_argument = 2.0 * (w * y - z * x)
        pitch = math.asin(np.clip(pitch_argument, -1.0, 1.0))
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        return roll, pitch, yaw

    @staticmethod
    def _rotation_matrix_from_euler(roll, pitch, yaw) -> np.ndarray:
        cos_roll, sin_roll = math.cos(roll), math.sin(roll)
        cos_pitch, sin_pitch = math.cos(pitch), math.sin(pitch)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        return np.array([
            [
                cos_yaw * cos_pitch,
                cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll,
                cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll,
            ],
            [
                sin_yaw * cos_pitch,
                sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll,
                sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll,
            ],
            [-sin_pitch, cos_pitch * sin_roll, cos_pitch * cos_roll],
        ])

    def _log_heat_signature(self, temperatures, thermal_message) -> None:
        if self.latest_depth is None or self.latest_depth_stamp is None:
            return
        if self.latest_odom is None or self.latest_odom_stamp is None:
            return

        thermal_stamp = self._stamp_seconds(thermal_message)
        if abs(thermal_stamp - self.latest_depth_stamp) > SYNC_TOLERANCE_S:
            return
        if abs(thermal_stamp - self.latest_odom_stamp) > SYNC_TOLERANCE_S:
            return

        adaptive_threshold = max(
            HEAT_THRESHOLD_C,
            float(np.percentile(temperatures, 98.0)),
        )
        hot_mask = (temperatures >= adaptive_threshold).astype(np.uint8)
        kernel = np.ones((5, 5), dtype=np.uint8)
        hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_OPEN, kernel)
        hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_CLOSE, kernel)
        component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
            hot_mask, connectivity=8
        )
        if component_count <= 1:
            return

        depth = self.latest_depth
        if depth.shape != temperatures.shape:
            depth = cv2.resize(
                depth,
                (temperatures.shape[1], temperatures.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )

        candidates = []
        for component_id in range(1, component_count):
            area = int(stats[component_id, cv2.CC_STAT_AREA])
            if area < 4:
                continue
            ys, xs = np.where(labels == component_id)
            valid_depth = np.isfinite(depth[ys, xs]) & (depth[ys, xs] > 0.1)
            valid_ratio = float(np.mean(valid_depth))
            if valid_ratio < 0.25:
                continue
            component_temperatures = temperatures[ys, xs]
            maximum_temperature = float(np.max(component_temperatures))
            average_temperature = float(np.mean(component_temperatures))
            pixel = (float(np.median(xs)), float(np.median(ys)))
            score = (
                2.0 * maximum_temperature
                + average_temperature
                + 8.0 * math.log1p(area)
                + 20.0 * valid_ratio
            )
            if self.last_heat_pixel is not None:
                pixel_distance = math.hypot(
                    pixel[0] - self.last_heat_pixel[0],
                    pixel[1] - self.last_heat_pixel[1],
                )
                score -= min(pixel_distance / 10.0, 20.0)
            candidates.append((score, component_id, pixel, valid_depth))

        if not candidates:
            return

        _, component, _, _ = max(candidates, key=lambda candidate: candidate[0])
        ys, xs = np.where(labels == component)
        valid_depth = np.isfinite(depth[ys, xs]) & (depth[ys, xs] > 0.1)
        xs = xs[valid_depth]
        ys = ys[valid_depth]
        distance = float(np.median(depth[ys, xs]))
        pixel_x = float(np.median(xs))
        pixel_y = float(np.median(ys))
        self.last_heat_pixel = (pixel_x, pixel_y)

        height, width = temperatures.shape
        horizontal_fov = math.radians(HORIZONTAL_FOV_DEG)
        focal_x = width / (2.0 * math.tan(horizontal_fov / 2.0))
        focal_y = focal_x
        center_x = (width - 1) / 2.0
        center_y = (height - 1) / 2.0
        camera_point = np.array([
            distance,
            -(pixel_x - center_x) * distance / focal_x,
            -(pixel_y - center_y) * distance / focal_y,
        ])

        pitch = math.radians(CAMERA_PITCH_DEG)
        camera_to_base = np.array([
            [math.cos(pitch), 0.0, math.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-math.sin(pitch), 0.0, math.cos(pitch)],
        ])
        base_point = camera_to_base @ camera_point
        base_point += np.array([0.25, 0.0, CAMERA_HEIGHT_M])

        odometry = self.latest_odom.pose.pose
        roll, pitch, yaw = self._euler_from_quaternion(odometry.orientation)
        odom_rotation = self._rotation_matrix_from_euler(roll, pitch, yaw)
        absolute_point = (
            odom_rotation @ base_point
            + np.array([
                odometry.position.x,
                odometry.position.y,
                odometry.position.z,
            ])
        )

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_heat_log_time < 1.0:
            return
        self.last_heat_log_time = now
        maximum_temperature = float(np.max(temperatures[labels == component]))
        self.get_logger().info(
            f"Heat signature: {maximum_temperature:.1f}C, "
            f"pixel=({pixel_x:.0f}, {pixel_y:.0f}), range={distance:.2f}m, "
            f"absolute pose in {self.latest_odom.header.frame_id}: "
            f"({absolute_point[0]:.2f}, {absolute_point[1]:.2f}, "
            f"{absolute_point[2]:.2f})m"
        )

    @staticmethod
    def add_scale(image, minimum, maximum, colormap, unit):
        height = image.shape[0]
        bar_width = 36
        label_width = 86
        gradient = np.linspace(255, 0, height, dtype=np.uint8)
        gradient = np.repeat(gradient[:, None], bar_width, axis=1)
        bar = cv2.applyColorMap(gradient, colormap)
        panel = np.zeros((height, bar_width + label_width, 3), dtype=np.uint8)
        panel[:, :bar_width] = bar

        for index in range(5):
            fraction = index / 4.0
            y = min(max(int(fraction * (height - 1)), 14), height - 4)
            value = maximum - fraction * (maximum - minimum)
            cv2.putText(
                panel,
                f"{value:.1f}{unit}",
                (bar_width + 4, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        return np.hstack((image, panel))

    def thermal_callback(self, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="passthrough"
            )
            if len(frame.shape) == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            if frame.dtype == np.uint16:
                temperature_c = frame.astype(np.float32) / 100.0 - 273.15
                valid = np.isfinite(temperature_c)
                if np.any(valid):
                    minimum = float(temperature_c[valid].min())
                    maximum = float(temperature_c[valid].max())
                    if maximum > minimum:
                        display = cv2.normalize(
                            temperature_c, None, 0, 255, cv2.NORM_MINMAX
                        ).astype(np.uint8)
                    else:
                        display = np.zeros(temperature_c.shape, dtype=np.uint8)
                    cv2.putText(
                        display,
                        f"{minimum:.1f}C to {maximum:.1f}C",
                        (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        255,
                        2,
                        cv2.LINE_AA,
                    )
                else:
                    display = np.zeros(frame.shape, dtype=np.uint8)
            else:
                display = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX)
                display = display.astype(np.uint8)

            thermal_color = cv2.applyColorMap(display, cv2.COLORMAP_INFERNO)
            if frame.dtype == np.uint16:
                self._log_heat_signature(temperature_c, message)
            if frame.dtype == np.uint16 and np.any(valid):
                thermal_color = self.add_scale(
                    thermal_color,
                    minimum,
                    maximum,
                    cv2.COLORMAP_INFERNO,
                    "C",
                )
            cv2.imshow(self.thermal_window, thermal_color)
            cv2.waitKey(1)
        except Exception as error:
            self.get_logger().error(f"Failed to display thermal frame: {error}")

    def depth_callback(self, message: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="passthrough"
            ).astype(np.float32, copy=False)
            self.latest_depth = depth
            self.latest_depth_stamp = self._stamp_seconds(message)
            valid = np.isfinite(depth) & (depth > 0.1)
            display = np.zeros(depth.shape, dtype=np.uint8)

            if np.any(valid):
                minimum, maximum = np.percentile(depth[valid], [5, 95])
                if maximum > minimum:
                    display[valid] = np.clip(
                        (depth[valid] - minimum) * 255.0 / (maximum - minimum),
                        0,
                        255,
                    ).astype(np.uint8)

            depth_color = cv2.applyColorMap(display, cv2.COLORMAP_TURBO)
            if np.any(valid):
                depth_color = self.add_scale(
                    depth_color,
                    float(minimum),
                    float(maximum),
                    cv2.COLORMAP_TURBO,
                    "m",
                )
            cv2.imshow(self.depth_window, depth_color)
            cv2.waitKey(1)
        except Exception as error:
            self.get_logger().error(f"Failed to display depth frame: {error}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()