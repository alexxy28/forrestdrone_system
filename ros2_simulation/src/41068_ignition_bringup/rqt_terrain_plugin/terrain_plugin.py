from __future__ import annotations

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from python_qt_binding import QtWidgets
from python_qt_binding.QtCore import QRect, QTimer, Qt
from python_qt_binding.QtGui import QColor, QImage, QPainter, QPen
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rqt_gui_py.plugin import Plugin
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class TerrainMapWidget(QtWidgets.QWidget):
    # Grid parameters match TerrainMap in control_template/trace_husky.
    RESOLUTION = 0.5
    SIZE_X = 40.0
    SIZE_Y = 40.0
    # Height (m) at which each colour is pure: green, yellow, orange, red.
    HEIGHT_STOPS_M = [-0.32, 1, 2, 3.0]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('TerrainMapWidget')
        self.setWindowTitle('Terrain Map')
        self.resize(800, 600)
        self.half_cells_x = int(np.ceil(self.SIZE_X / (2.0 * self.RESOLUTION)))
        self.half_cells_y = int(np.ceil(self.SIZE_Y / (2.0 * self.RESOLUTION)))
        self.grid_width = 2 * self.half_cells_x + 1
        self.grid_height = 2 * self.half_cells_y + 1

        self._elevation: np.ndarray | None = None
        self._height_min = 0.0
        self._height_max = 1.0
        self._cloud_points = 0
        self._known_cells = 0
        self._cloud_frame = ''
        self._z_range_text = ''

        # Robot positions (x, y)
        self._husky_pos: tuple[float, float] | None = None
        self._drone_pos: tuple[float, float] | None = None

        # Waypoints and Paths
        self._husky_target_wp: tuple[float, float] | None = None
        self._drone_target_wp: tuple[float, float] | None = None
        self._husky_path: list[tuple[float, float]] = []
        self._drone_path: list[tuple[float, float]] = []

        self._node: Node | None = None
        self._subscription = None
        self._topic = '/parrot1/terrain_points'

        self._husky_odom_sub = None
        self._drone_odom_sub = None
        self._husky_plan_sub = None
        self._husky_wp_sub = None
        self._drone_plan_sub = None
        self._drone_wp_sub = None

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

        # Odometry Subscriptions (Using /odometry to match the terrain builder's frame)
        self._husky_odom_sub = self._node.create_subscription(
            Odometry, '/husky1/odometry', self._husky_odom_cb, 10)
        self._drone_odom_sub = self._node.create_subscription(
            Odometry, '/parrot1/odometry', self._drone_odom_cb, 10)

        # Nav2 Path Subscriptions
        self._husky_plan_sub = self._node.create_subscription(
            Path, '/husky1/plan', self._husky_plan_cb, 10)
        self._drone_plan_sub = self._node.create_subscription(
            Path, '/parrot1/plan', self._drone_plan_cb, 10)

        # Target Goal Pose Subscriptions
        self._husky_wp_sub = self._node.create_subscription(
            PoseStamped, '/husky1/goal_pose', self._husky_wp_cb, 10)
        self._drone_wp_sub = self._node.create_subscription(
            PoseStamped, '/parrot1/goal_pose', self._drone_wp_cb, 10)

    def _husky_odom_cb(self, msg: Odometry) -> None:
        self._husky_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _drone_odom_cb(self, msg: Odometry) -> None:
        self._drone_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _husky_plan_cb(self, msg: Path) -> None:
        self._husky_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _drone_plan_cb(self, msg: Path) -> None:
        self._drone_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _husky_wp_cb(self, msg: PoseStamped) -> None:
        self._husky_target_wp = (msg.pose.position.x, msg.pose.position.y)

    def _drone_wp_cb(self, msg: PoseStamped) -> None:
        self._drone_target_wp = (msg.pose.position.x, msg.pose.position.y)

    def shutdown(self) -> None:
        self._timer.stop()
        if self._node is not None:
            subs = [self._subscription, self._husky_odom_sub, self._drone_odom_sub, 
                    self._husky_plan_sub, self._husky_wp_sub, self._drone_plan_sub, self._drone_wp_sub]
            for sub in subs:
                if sub is not None:
                    try:
                        self._node.destroy_subscription(sub)
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

        # Colours are pinned to heights in metres, not stretched over min..max:
        # terrain_demo ground spans -0.32..10.6 m but ~95% of it is below 4 m, so a
        # min..max stretch would squash most of the ground into one colour.
        # Anything above the last stop stays red. Tune HEIGHT_STOPS_M for other worlds.
        stops = np.array([[0, 200, 0], [255, 255, 0], [255, 140, 0], [255, 0, 0]], dtype=np.float64)
        heights = np.where(known, elevation, self.HEIGHT_STOPS_M[0])
        rgb = np.stack([np.interp(heights, self.HEIGHT_STOPS_M, stops[:, c]) for c in range(3)],
                       axis=-1).astype(np.uint8)
        rgb[~known] = 0

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

        def world_to_pixel(world_x: float, world_y: float) -> tuple[int, int]:
            gx = (world_x / self.RESOLUTION) + self.half_cells_x + 0.5
            gy = (world_y / self.RESOLUTION) + self.half_cells_y + 0.5
            gy_img = self.grid_height - gy
            px = target.x() + (gx * (draw_w / self.grid_width))
            py = target.y() + (gy_img * (draw_h / self.grid_height))
            return int(px), int(py)

        def draw_trajectory(current_pos, path, goal, color):
            if current_pos is None:
                return

            pts = []
            if len(path) >= 2:
                pts = [world_to_pixel(x, y) for (x, y) in path]
            elif goal is not None:
                pts = [world_to_pixel(current_pos[0], current_pos[1]), world_to_pixel(goal[0], goal[1])]

            if len(pts) >= 2:
                painter.setPen(QPen(QColor(color), 2, Qt.DashLine))
                for i in range(len(pts) - 1):
                    painter.drawLine(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])

            dest = goal if goal is not None else (path[-1] if path else None)
            if dest is not None:
                wx, wy = world_to_pixel(dest[0], dest[1])
                painter.setBrush(QColor(color))
                painter.setPen(QPen(Qt.black, 1))
                painter.drawEllipse(wx - 4, wy - 4, 8, 8)

        def draw_robot_marker(pos, color):
            if pos is None:
                return
            px, py = world_to_pixel(pos[0], pos[1])
            painter.setBrush(QColor(color))
            painter.setPen(QPen(Qt.white, 1))
            painter.drawEllipse(px - 5, py - 5, 10, 10)

        # Draw trajectories then robots on top
        draw_trajectory(self._husky_pos, self._husky_path, self._husky_target_wp, 'cyan')
        draw_trajectory(self._drone_pos, self._drone_path, self._drone_target_wp, 'pink')
        
        draw_robot_marker(self._husky_pos, 'blue')
        draw_robot_marker(self._drone_pos, 'magenta')

        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.drawText(
            8, 16,
            f'{self._z_range_text}  (green {self.HEIGHT_STOPS_M[0]:g} m → yellow {self.HEIGHT_STOPS_M[1]:g} '
            f'→ orange {self.HEIGHT_STOPS_M[2]:g} → red {self.HEIGHT_STOPS_M[3]:g}+ m)  '
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
        self._robot_states[robot_name] = not bool(msg.data)

    def _update_labels(self) -> None:
        self._husky_button.setText('Stop Husky' if self._robot_states.get('husky1', False) else 'Start Husky')
        self._parrot_button.setText('Stop Parrot' if self._robot_states.get('parrot1', False) else 'Start Parrot')

    def _trigger_waypoint_accept(self) -> None:
        if self._node is None:
            self._status.setText('ROS node is not connected yet.')
            return

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
