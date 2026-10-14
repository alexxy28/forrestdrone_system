from __future__ import annotations

import numpy as np
from python_qt_binding import QtWidgets
from python_qt_binding.QtCore import QRect, QTimer, Qt
from python_qt_binding.QtGui import QColor, QImage, QPainter, QPen
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rqt_gui_py.plugin import Plugin
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool


class TerrainMapWidget(QtWidgets.QWidget):
    # Grid parameters match TerrainMap in control_template/trace_husky.
    RESOLUTION = 0.5
    SIZE_X = 40.0
    SIZE_Y = 40.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('TerrainMapWidget')
        self.setWindowTitle('Terrain Map')
        self.resize(800, 600)
        self.half_cells_x = int(np.ceil(self.SIZE_X / (2.0 * self.RESOLUTION)))
        self.half_cells_y = int(np.ceil(self.SIZE_Y / (2.0 * self.RESOLUTION)))
        self.grid_width = 2 * self.half_cells_x + 1
        self.grid_height = 2 * self.half_cells_y + 1
        # Elevation per cell, indexed [gy, gx]; NaN means no data.
        self._elevation: np.ndarray | None = None
        self._height_min = 0.0
        self._height_max = 1.0
        self._cloud_points = 0
        self._known_cells = 0
        self._cloud_frame = ''
        self._node: Node | None = None
        self._subscription = None
        self._topic = '/parrot1/terrain_points'
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(100)

    def set_node(self, node: Node) -> None:
        self._node = node
        self._resubscribe()

    def set_topic(self, topic: str) -> None:
        clean_topic = str(topic).strip()
        if not clean_topic or clean_topic == self._topic:
            return
        self._topic = clean_topic
        self._resubscribe()

    def topic(self) -> str:
        return self._topic

    def _resubscribe(self) -> None:
        if self._node is None:
            return

        if self._subscription is not None:
            try:
                self._node.destroy_subscription(self._subscription)
            except Exception:
                pass

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscription = self._node.create_subscription(
            PointCloud2,
            self._topic,
            self._cloud_cb,
            qos,
        )

    def shutdown(self) -> None:
        self._timer.stop()
        if self._node is not None and self._subscription is not None:
            try:
                self._node.destroy_subscription(self._subscription)
            except Exception:
                pass
            self._subscription = None

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

    def _build_elevation(self, xyz: np.ndarray) -> np.ndarray:
        """Keep the highest observed elevation per map cell.

        Missing cells remain NaN so the renderer can display them as black.
        """
        elevation = np.full((self.grid_height, self.grid_width), np.nan, dtype=np.float64)

        finite = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite]
        gx = np.floor(
            xyz[:, 0] / self.RESOLUTION + self.half_cells_x + 0.5
        ).astype(np.int64)
        gy = np.floor(
            xyz[:, 1] / self.RESOLUTION + self.half_cells_y + 0.5
        ).astype(np.int64)
        valid = (gx >= 0) & (gx < self.grid_width) & (gy >= 0) & (gy < self.grid_height)
        # fmax ignores NaN, so the first point in an empty cell wins.
        np.fmax.at(elevation, (gy[valid], gx[valid]), xyz[valid, 2])

        return elevation

    def _cloud_cb(self, msg: PointCloud2) -> None:
        try:
            xyz = self._extract_xyz(msg)
            if xyz.size == 0:
                return

            finite_z = xyz[np.isfinite(xyz).all(axis=1), 2]
            self._cloud_points = int(finite_z.size)
            self._cloud_frame = msg.header.frame_id
            if finite_z.size > 0:
                self._height_min = float(np.min(finite_z))
                self._height_max = float(np.max(finite_z))

            # The builder publishes the accumulated map on each update.
            self._elevation = self._build_elevation(xyz)
            self._known_cells = int(np.isfinite(self._elevation).sum())
        except Exception as exc:
            if self._node is not None:
                self._node.get_logger().warning(
                    f'Could not decode terrain cloud on {self._topic}: {exc}',
                    throttle_duration_sec=5.0,
                )
            return

    def _render_image(self, elevation: np.ndarray) -> QImage | None:
        known = np.isfinite(elevation)
        if not known.any():
            return None

        z_min = float(np.min(elevation[known]))
        z_max = float(np.max(elevation[known]))
        z_range = z_max - z_min

        # Use the raw cloud range if the grid is flat; otherwise keep the real
        # current min/max so a higher cell can become red without the map
        # collapsing to a 0.0 m range.
        if z_range <= 1e-6 and self._height_max > self._height_min:
            z_min = self._height_min
            z_max = self._height_max
            z_range = z_max - z_min

        if z_range <= 1e-6:
            z_min -= 1e-3
            z_max += 1e-3
            z_range = z_max - z_min

        ratio = np.clip((elevation - z_min) / z_range, 0.0, 1.0)
        ratio = np.where(known, ratio, 0.0)

        # Green (low) -> red (high). Unknown cells stay black.
        rgb = np.zeros((self.grid_height, self.grid_width, 3), dtype=np.uint8)
        red = np.clip(ratio * 255.0, 0.0, 255.0).astype(np.uint8)
        green = np.clip((1.0 - ratio) * 255.0, 0.0, 255.0).astype(np.uint8)
        rgb[..., 0] = red
        rgb[..., 1] = green
        rgb[~known] = 0

        # Flip so +y is drawn upwards.
        rgb = np.ascontiguousarray(rgb[::-1])
        image = QImage(
            rgb.data, self.grid_width, self.grid_height,
            3 * self.grid_width, QImage.Format_RGB888,
        )
        self._z_range_text = f'low {z_min:.3f} m  /  high {z_max:.3f} m'
        return image.copy()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), Qt.black)

        elevation = self._elevation
        image = self._render_image(elevation) if elevation is not None else None
        if image is None:
            painter.setPen(QPen(QColor(255, 255, 255)))
            painter.drawText(self.rect(), Qt.AlignCenter, f'Waiting for {self._topic}...')
            painter.end()
            return

        scale = min(self.width() / self.grid_width, self.height() / self.grid_height)
        draw_w = max(1, int(self.grid_width * scale))
        draw_h = max(1, int(self.grid_height * scale))
        target = QRect(
            (self.width() - draw_w) // 2,
            (self.height() - draw_h) // 2,
            draw_w,
            draw_h,
        )
        painter.drawImage(target, image)

        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.drawText(
            8, 16,
            f'{self._z_range_text}  (green = low, red = high)  '
            f'cells: {self._known_cells}  points: {self._cloud_points}  '
            f'frame: {self._cloud_frame or "unknown"}',
        )
        painter.end()


class RobotMotionToggleWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('RobotMotionToggleWidget')
        self.setWindowTitle('Robot Motion Toggle')
        self.resize(260, 220)
        self._node: Node | None = None
        self._publishers = {}
        self._subscriptions = {}
        self._waypoint_topic = '/mission_control/husky1/waypoint_accept'
        self._waypoint_publisher = None
        self._robot_states = {'husky1': True, 'parrot1': True}
        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._update_labels)
        self._ui_timer.start(200)
        self._build_ui()

    def set_node(self, node: Node) -> None:
        self._node = node
        self._ensure_waypoint_publisher()
        self._publish_waypoint_accept(False)
        for robot_name in self._robot_states:
            self._ensure_publisher(robot_name)
            self._ensure_subscription(robot_name)

    def shutdown(self) -> None:
        self._ui_timer.stop()
        if self._node is None:
            return
        for publisher in self._publishers.values():
            try:
                self._node.destroy_publisher(publisher)
            except Exception:
                pass
        self._publishers.clear()

        if self._waypoint_publisher is not None:
            try:
                self._node.destroy_publisher(self._waypoint_publisher)
            except Exception:
                pass
            self._waypoint_publisher = None

        for subscription in self._subscriptions.values():
            try:
                self._node.destroy_subscription(subscription)
            except Exception:
                pass
        self._subscriptions.clear()

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        title = QtWidgets.QLabel('Robot Toggle')
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        self._waypoint_button = QtWidgets.QPushButton('Set Husky Waypoint To Latest Heat Signature')
        self._waypoint_button.clicked.connect(self._trigger_waypoint_accept)
        layout.addWidget(self._waypoint_button)

        self._husky_button = QtWidgets.QPushButton('Stop Husky')
        self._husky_button.clicked.connect(lambda: self._toggle_robot('husky1'))
        layout.addWidget(self._husky_button)

        self._parrot_button = QtWidgets.QPushButton('Stop Parrot')
        self._parrot_button.clicked.connect(lambda: self._toggle_robot('parrot1'))
        layout.addWidget(self._parrot_button)

        self._status = QtWidgets.QLabel('Idle')
        self._status.setWordWrap(True)
        layout.addWidget(self._status)
        layout.addStretch()

        self._update_labels()

    def _clean_robot_name(self, robot_name: str) -> str:
        robot_name = str(robot_name or '').strip().replace('/', '')
        return robot_name

    def _ensure_publisher(self, robot_name: str):
        clean_name = self._clean_robot_name(robot_name)
        if not clean_name or self._node is None:
            return None
        if clean_name not in self._publishers:
            self._publishers[clean_name] = self._node.create_publisher(Bool, f'mission_control/{clean_name}/estop', 10)
        return self._publishers[clean_name]

    def _ensure_subscription(self, robot_name: str):
        clean_name = self._clean_robot_name(robot_name)
        if not clean_name or self._node is None:
            return None
        if clean_name not in self._subscriptions:
            self._subscriptions[clean_name] = self._node.create_subscription(
                Bool,
                f'{clean_name}/estop',
                lambda msg, name=clean_name: self._estop_state_cb(name, msg),
                10,
            )
        return self._subscriptions[clean_name]

    def _ensure_waypoint_publisher(self):
        if self._node is None:
            return None
        if self._waypoint_publisher is None:
            self._waypoint_publisher = self._node.create_publisher(Bool, self._waypoint_topic, 10)
        return self._waypoint_publisher

    def _publish_stop_state(self, robot_name: str, stopped: bool) -> None:
        publisher = self._ensure_publisher(robot_name)
        if publisher is not None:
            publisher.publish(Bool(data=stopped))

    def _publish_waypoint_accept(self, accepted: bool) -> None:
        publisher = self._ensure_waypoint_publisher()
        if publisher is not None:
            publisher.publish(Bool(data=accepted))

    def _estop_state_cb(self, robot_name: str, msg: Bool) -> None:
        # Bool True on estop means robot is stopped.
        self._robot_states[robot_name] = not bool(msg.data)

    def _update_labels(self) -> None:
        self._husky_button.setText('Stop Husky' if self._robot_states.get('husky1', True) else 'Start Husky')
        self._parrot_button.setText('Stop Parrot' if self._robot_states.get('parrot1', True) else 'Start Parrot')

    def _trigger_waypoint_accept(self) -> None:
        if self._node is None:
            self._status.setText('ROS node is not connected yet.')
            return

        # Send a rising-edge trigger and reset to false so each click can
        # request a fresh waypoint update from mission control.
        self._publish_waypoint_accept(True)
        QTimer.singleShot(120, lambda: self._publish_waypoint_accept(False))
        self._status.setText(
            f'Waypoint trigger sent on {self._waypoint_topic} (auto-reset to False).',
        )

    def _toggle_robot(self, robot_name: str) -> None:
        clean_name = self._clean_robot_name(robot_name)
        if not clean_name:
            self._status.setText('Set a robot namespace first.')
            return
        if self._node is None:
            self._status.setText('ROS node is not connected yet.')
            return

        self._robot_states.setdefault(clean_name, True)
        self._ensure_subscription(clean_name)
        if self._robot_states[clean_name]:
            self._publish_stop_state(clean_name, True)
            self._status.setText(f'{clean_name} stopButton published as True.')
            self._robot_states[clean_name] = False
        else:
            self._publish_stop_state(clean_name, False)
            self._status.setText(f'{clean_name} stopButton published as False.')
            self._robot_states[clean_name] = True

        self._update_labels()



class TerrainMapPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('TerrainMapPlugin')

        self._widget = TerrainMapWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(f'Terrain Map ({context.serial_number()})')

        node = getattr(context, 'node', None)
        if node is not None:
            self._widget.set_node(node)

        context.add_widget(self._widget)

    def shutdown_plugin(self):
        if hasattr(self, '_widget'):
            self._widget.shutdown()
            self._widget.deleteLater()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('pointcloud_topic', self._widget.topic())

    def restore_settings(self, plugin_settings, instance_settings):
        topic = instance_settings.value('pointcloud_topic', self._widget.topic())
        if topic:
            self._widget.set_topic(topic)

    def trigger_configuration(self):
        pass


class RobotMotionTogglePlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('RobotMotionTogglePlugin')

        self._widget = RobotMotionToggleWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(f'Robot Motion Toggle ({context.serial_number()})')

        node = getattr(context, 'node', None)
        if node is not None:
            self._widget.set_node(node)

        context.add_widget(self._widget)

    def shutdown_plugin(self):
        if hasattr(self, '_widget'):
            self._widget.shutdown()
            self._widget.deleteLater()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def trigger_configuration(self):
        pass
