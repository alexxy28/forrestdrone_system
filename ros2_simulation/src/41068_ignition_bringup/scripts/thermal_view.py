#!/usr/bin/env python3
"""False-colour view of the thermal cameras, for the operator dashboard only.

The simulated thermal cameras publish mono16 images in 0.01 K steps. Shown
directly (rqt Image View converts 16-bit to 8-bit by dividing by 256), a
288 K ground and a 310 K person differ by only a few grey levels, so the
person is hard to see. This node maps a fixed temperature range onto an
"ironbow" palette and republishes rgb8:

    /parrot1/thermal_camera -> /parrot1/thermal_view
    /husky1/thermal_camera  -> /husky1/thermal_view

Detection (parrot_node) still reads the raw mono16 topic; nothing here feeds
the mission. Parameters: min_temp_k (default 280), max_temp_k (default 320),
cameras (default ['parrot1', 'husky1']).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# Ironbow-style palette: black -> indigo -> magenta -> orange -> yellow -> white.
_STOPS = np.array([0.0, 0.2, 0.45, 0.7, 0.88, 1.0])
_COLOURS = np.array([
    [0, 0, 0],
    [40, 0, 110],
    [180, 0, 140],
    [250, 100, 0],
    [255, 220, 0],
    [255, 255, 255],
], dtype=np.float64)
_LUT = np.stack([np.interp(np.linspace(0, 1, 256), _STOPS, _COLOURS[:, c]) for c in range(3)],
                axis=1).astype(np.uint8)


class ThermalView(Node):
    def __init__(self):
        super().__init__('thermal_view')
        self._lo = float(self.declare_parameter('min_temp_k', 280.0).value)
        self._hi = float(self.declare_parameter('max_temp_k', 320.0).value)
        cameras = self.declare_parameter('cameras', ['parrot1', 'husky1']).value
        self._pubs = {}
        for cam in cameras:
            out = self.create_publisher(Image, f'/{cam}/thermal_view', qos_profile_sensor_data)
            self._pubs[cam] = out
            self.create_subscription(
                Image, f'/{cam}/thermal_camera',
                lambda msg, pub=out: self._convert(msg, pub), qos_profile_sensor_data)
        self.get_logger().info(
            f'False-colour thermal view {self._lo:.0f}-{self._hi:.0f} K for {", ".join(cameras)}')

    def _convert(self, msg: Image, pub) -> None:
        if pub.get_subscription_count() == 0:
            return
        if msg.encoding in ('mono16', '16UC1'):
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)
            kelvin = raw[:, :msg.width].astype(np.float32) * 0.01
        elif msg.encoding in ('mono8', '8UC1'):
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            kelvin = self._lo + raw[:, :msg.width].astype(np.float32) * (self._hi - self._lo) / 255.0
        else:
            self.get_logger().warning(f'Unsupported thermal encoding {msg.encoding}',
                                      throttle_duration_sec=10.0)
            return
        index = np.clip((kelvin - self._lo) * (255.0 / (self._hi - self._lo)), 0, 255).astype(np.uint8)
        rgb = _LUT[index]

        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = 'rgb8'
        out.is_bigendian = 0
        out.step = msg.width * 3
        out.data = rgb.tobytes()
        pub.publish(out)


def main():
    rclpy.init()
    node = ThermalView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
