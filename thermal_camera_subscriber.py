#!/usr/bin/env python3

"""Subscribe to the Parrot thermal camera topic and print frame info (ROS2).

This is a ROS2 `rclpy` conversion of the original ROS1 `rospy` script.
"""

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2  # pyright: ignore[reportMissingImports]
import numpy as np
from sensor_msgs.msg import Image


TOPIC_NAME = "/parrot1/thermal_camera"


class ThermalCameraSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("thermal_camera_subscriber")
        self.bridge = CvBridge()
        self.window_name = "Thermal Camera"
        self.latest_frame_2d = None
        self.latest_temp_celsius = None
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.subscription = self.create_subscription(
            Image, TOPIC_NAME, self.thermal_camera_callback, 1
        )
        # keep reference
        self.subscription
        self.get_logger().info(f"Subscribed to {TOPIC_NAME}")

    def _build_scale_bar(self, height, min_c, max_c, colormap=cv2.COLORMAP_INFERNO):
        """Build a vertical temperature colorbar with tick labels (in C)."""
        bar_w = 40
        # Vertical gradient: top = max temp, bottom = min temp.
        gradient = np.linspace(255, 0, height, dtype=np.uint8).reshape(-1, 1)
        gradient = np.repeat(gradient, bar_w, axis=1)
        bar = cv2.applyColorMap(gradient, colormap)

        # Pad an area on the right for text labels.
        label_w = 70
        panel = np.zeros((height, bar_w + label_w, 3), dtype=np.uint8)
        panel[:, :bar_w] = bar

        # Draw tick labels evenly from max (top) to min (bottom).
        num_ticks = 5
        for i in range(num_ticks):
            frac = i / (num_ticks - 1)
            y = int(frac * (height - 1))
            temp = max_c - frac * (max_c - min_c)
            y_text = min(max(y, 12), height - 4)
            cv2.putText(
                panel,
                f"{temp:.1f}C",
                (bar_w + 4, y_text),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        return panel

    def thermal_camera_callback(self, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        except Exception as error:
            self.get_logger().error(f"Failed to decode thermal frame: {error}")
            return

        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Preserve the radiometric (temperature) data before any normalization.
        # Gazebo's thermal sensor publishes mono16 where each pixel value is
        # temperature in Kelvin * 100 (e.g. 30000 -> 300.00 K -> 26.85 C).
        self.latest_temp_celsius = None
        if str(frame.dtype) == "uint16":
            temp_celsius = frame.astype("float32") / 100.0 - 273.15
            self.latest_temp_celsius = temp_celsius
            self.latest_frame_2d = temp_celsius.tolist()
        else:
            # Non-radiometric (8-bit) feed: no temperature data available.
            self.latest_frame_2d = frame.tolist()

        # Build a separate display-only image so normalization never touches
        # the stored radiometric data.
        if str(frame.dtype) != "uint8":
            display = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX)
            display = display.astype("uint8")
        else:
            display = frame

        # Colorize the display and attach a temperature scale bar when we have
        # real radiometric data.
        display_color = cv2.applyColorMap(display, cv2.COLORMAP_INFERNO)
        if self.latest_temp_celsius is not None:
            h = display_color.shape[0]
            min_c = float(self.latest_temp_celsius.min())
            max_c = float(self.latest_temp_celsius.max())
            scale_bar = self._build_scale_bar(h, min_c, max_c)
            display_color = np.hstack([display_color, scale_bar])

        cv2.imshow(self.window_name, display_color)
        cv2.waitKey(1)

        if self.latest_temp_celsius is not None:
            h, w = self.latest_temp_celsius.shape
            center_c = float(self.latest_temp_celsius[h // 2, w // 2])
            min_c = float(self.latest_temp_celsius.min())
            max_c = float(self.latest_temp_celsius.max())
            self.get_logger().info(
                f"Received thermal frame: {message.width}x{message.height}, "
                f"encoding={message.encoding}, center={center_c:.2f}C, "
                f"min={min_c:.2f}C, max={max_c:.2f}C"
            )
        else:
            self.get_logger().info(
                f"Received thermal frame: {message.width}x{message.height}, "
                f"encoding={message.encoding}, step={message.step}, "
                f"data_bytes={len(message.data)} (non-radiometric)"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThermalCameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
