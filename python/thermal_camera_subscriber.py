#!/usr/bin/env python3

"""Display Parrot thermal and depth ROS 2 image topics in separate windows."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


THERMAL_TOPIC = "/parrot1/thermal_camera"
DEPTH_TOPIC = "/parrot1/camera/depth/image"


class CameraViewer(Node):
    def __init__(self) -> None:
        super().__init__("thermal_depth_viewer")
        self.bridge = CvBridge()
        self.create_subscription(Image, THERMAL_TOPIC, self.thermal_callback, 1)
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 1)
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
            f"Subscribed to {THERMAL_TOPIC} and {DEPTH_TOPIC}"
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