from __future__ import annotations

from typing import List

import numpy as np
from python_qt_binding import QtWidgets
from python_qt_binding.QtCore import QTimer, Qt
from python_qt_binding.QtGui import QColor, QPainter, QPen
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rqt_gui_py.plugin import Plugin
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class TerrainMapWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('TerrainMapWidget')
        self.setWindowTitle('Terrain Map')
        self.resize(800, 600)
        self.points: List[np.ndarray] = []
        self.max_points = 80000
        self.min_x = self.min_y = self.min_z = 0.0
        self.max_x = self.max_y = self.max_z = 1.0
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

    def _cloud_cb(self, msg: PointCloud2) -> None:
        try:
            arr = self._extract_xyz(msg)
        except Exception:
            return

        if arr.size == 0:
            return
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)

        self.points.extend(arr[:, :3].tolist())
        if len(self.points) > self.max_points:
            self.points = self.points[-self.max_points:]

        if self.points:
            merged = np.asarray(self.points, dtype=np.float64)
            self.min_x, self.max_x = float(np.min(merged[:, 0])), float(np.max(merged[:, 0]))
            self.min_y, self.max_y = float(np.min(merged[:, 1])), float(np.max(merged[:, 1]))
            self.min_z, self.max_z = float(np.min(merged[:, 2])), float(np.max(merged[:, 2]))

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), Qt.black)

        if not self.points:
            painter.setPen(QPen(QColor(255, 255, 255)))
            painter.drawText(self.rect(), Qt.AlignCenter, f'Waiting for {self._topic}...')
            return

        merged = np.asarray(self.points, dtype=np.float64)
        x_range = max(self.max_x - self.min_x, 1e-3)
        y_range = max(self.max_y - self.min_y, 1e-3)
        z_range = max(self.max_z - self.min_z, 1e-3)
        width = self.width()
        height = self.height()

        for point in merged:
            x, y, z = point
            px = int((x - self.min_x) / x_range * (width - 1))
            py = int((y - self.min_y) / y_range * (height - 1))
            intensity = (z - self.min_z) / z_range
            hue = int(120.0 * (1.0 - min(max(intensity, 0.0), 1.0)))
            color = QColor.fromHsv(hue, 220, 200)
            painter.setPen(QPen(color, 1))
            painter.drawPoint(px, height - 1 - py)

        painter.end()


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
