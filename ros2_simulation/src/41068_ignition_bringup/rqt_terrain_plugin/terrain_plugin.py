from __future__ import annotations

import time

import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
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
from std_msgs.msg import Bool, String

# =============================================================================
# QOS (Quality of Service) PROFILES
# =============================================================================
# Mission topics are latched (reliable + transient local, depth 1) on both ends.
# This ensures that late subscribers (like this dashboard opening after the mission starts) 
# immediately get the last published state instead of waiting for a new one.
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# =============================================================================
# HELPER FUNCTIONS & CONSTANTS
# =============================================================================
def route_progress(route, husky, passed):
    """Waypoint the Husky is heading to and the distance left along the route.

    `passed` is the index of the next waypoint not yet reached; it only moves
    forward. Returns (next_index, metres_left).
    
    How it works:
    1. Checks if the Husky is within 1.0m of the current target waypoint.
    2. If so, increments the 'passed' index to target the next waypoint.
    3. Calculates the remaining straight-line Euclidean distance to the end of the route.
    """
    if not route or husky is None:
        return passed, None
    while passed < len(route) - 1 and np.hypot(route[passed][0] - husky[0],
                                                route[passed][1] - husky[1]) < 1.0:
        passed += 1
    left = float(np.hypot(route[passed][0] - husky[0], route[passed][1] - husky[1]))
    for a, b in zip(route[passed:], route[passed + 1:]):
        left += float(np.hypot(b[0] - a[0], b[1] - a[1]))
    return passed, left

# Maps internal ROS state strings to human-readable UI descriptions
MISSION_TEXT = {
    'SEARCHING': 'Drone searching for a heat signature',
    'PLANNING': 'Casualty confirmed: planning the Husky route over the terrain map',
    'HUSKY_EN_ROUTE': 'Husky driving the A* route to the casualty',
    'DONE': 'Husky has reached the casualty',
    'FAILED': 'Husky could not reach the casualty: operator decision needed',
}

# =============================================================================
# GUI WIDGET 1: TERRAIN MAP VISUALIZER
# =============================================================================
class TerrainMapWidget(QtWidgets.QWidget):
    """
    This widget renders a 2D overhead view of the environment. It converts 3D PointClouds
    into a 2D grid (Height, Slope, or Roughness), and overlays robot positions, 
    waypoints, and the mission route.
    """
    # Grid parameters match TerrainMap in control_template/trace_husky.
    RESOLUTION = 0.5 # Each pixel/cell represents 0.5 meters
    SIZE_X = 40.0    # 40 meters wide
    SIZE_Y = 40.0    # 40 meters tall
    
    # Height (m) at which each colour is pure: green, yellow, orange, red.
    HEIGHT_STOPS_M = [-0.32, 1, 2, 3.0]
    
    # View name -> (grid attribute, green/yellow/orange/red stops, unit). Slope and
    # roughness turn red at the planner's limits: MAX_SLOPE_DEG 25 and MAX_SIGMA_M 0.5
    # in control_template/SYSTEM_CONTROL/terrain_map.hpp; keep them in step.
    VIEWS = {
        'Height': ('_elevation', HEIGHT_STOPS_M, 'm'),
        'Slope': ('_slope', [0.0, 8.0, 15.0, 25.0], 'deg'),
        'Roughness': ('_sigma', [0.0, 0.1, 0.25, 0.5], 'm'),
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('TerrainMapWidget')
        self.setWindowTitle('Terrain Map')
        self.resize(800, 600)
        self.setMinimumSize(360, 300)
        
        # Calculate grid dimensions: 40m / 0.5m = 80 cells. 
        # Half cells = 40. Total grid = 81x81 cells.
        self.half_cells_x = int(np.ceil(self.SIZE_X / (2.0 * self.RESOLUTION)))
        self.half_cells_y = int(np.ceil(self.SIZE_Y / (2.0 * self.RESOLUTION)))
        self.grid_width = 2 * self.half_cells_x + 1
        self.grid_height = 2 * self.half_cells_y + 1

        # NumPy arrays storing the map data per cell
        self._elevation: np.ndarray | None = None
        self._slope: np.ndarray | None = None
        self._sigma: np.ndarray | None = None
        
        # UI string variables (for displaying map info in the UI)
        self._height_min = 0.0
        self._height_max = 1.0
        self._cloud_points = 0
        self._known_cells = 0
        self._cloud_frame = ''
        self._z_range_text = ''

        # Robot positions (x, y) extracted from Odometry
        self._husky_pos: tuple[float, float] | None = None
        self._drone_pos: tuple[float, float] | None = None

        # Waypoints and Paths for drawing trajectories
        self._husky_target_wp: tuple[float, float] | None = None
        self._drone_target_wp: tuple[float, float] | None = None
        self._husky_path: list[tuple[float, float]] = []
        self._drone_path: list[tuple[float, float]] = []

        # Second drone (/parrot2), only drawn once it publishes
        self._drone2_pos: tuple[float, float] | None = None
        self._drone2_path: list[tuple[float, float]] = []
        
        # Mission (TRACE) overlays
        self._mission_route: list[tuple[float, float]] = []
        
        # Where the Husky was when the route arrived: the route itself starts at
        # the first waypoint (system_control drops the Husky's own cell).
        self._route_start: tuple[float, float] | None = None
        
        # Points of interest for the mission
        self._casualty: tuple[float, float] | None = None
        self._candidate: tuple[float, float] | None = None
        self._hold: tuple[float, float] | None = None
        
        # State tracking
        self._route_passed = 0
        self._mission_state = 'unknown'
        self._husky_status = 'unknown'
        
        # ROS 2 Node and Subscription placeholders
        self._mission_subs = []
        self._node: Node | None = None
        self._subscription = None
        self._topic = '/parrot1/terrain_points'

        self._husky_odom_sub = None
        self._drone_odom_sub = None
        self._husky_plan_sub = None
        self._husky_wp_sub = None
        self._drone_plan_sub = None
        self._drone_wp_sub = None
        self._drone2_odom_sub = None
        self._drone2_plan_sub = None

        # Main UI refresh loop (10 FPS)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(100)

        # Button to toggle between Height, Slope, and Roughness views
        self._view = 'Height'
        self._view_button = QtWidgets.QPushButton(self)
        self._view_button.clicked.connect(self._next_view)
        self._update_view_button()

    def _next_view(self) -> None:
        """Cycles through the VIEWS dict keys."""
        names = list(self.VIEWS)
        self._view = names[(names.index(self._view) + 1) % len(names)]
        self._update_view_button()
        self.update()

    def _update_view_button(self) -> None:
        """Updates the button text and anchors it to the top right."""
        self._view_button.setText(f'View: {self._view}  (click to switch)')
        self._view_button.adjustSize()
        self._view_button.move(self.width() - self._view_button.width() - 8, 24)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._update_view_button()

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
        """Sets up all ROS 2 subscriptions required for the Map Widget."""
        if self._node is None:
            return

        if self._subscription is not None:
            try:
                self._node.destroy_subscription(self._subscription)
            except Exception:
                pass

        # Use BEST_EFFORT for high-frequency sensor data like PointClouds
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

        # Nav2 Path Subscriptions (Husky). The drone is flown by parrot_node,
        # not Nav2, so show the search path mission control gave it instead.
        self._husky_plan_sub = self._node.create_subscription(
            Path, '/husky1/plan', self._husky_plan_cb, 10)
        self._drone_plan_sub = self._node.create_subscription(
            Path, '/parrot1/mission/path', self._drone_plan_cb, LATCHED)

        # Second drone: nothing arrives on these unless it was launched (parrot2:=true).
        self._drone2_odom_sub = self._node.create_subscription(
            Odometry, '/parrot2/odometry', self._drone2_odom_cb, 10)
        self._drone2_plan_sub = self._node.create_subscription(
            Path, '/parrot2/mission/path', self._drone2_plan_cb, LATCHED)

        # Mission overlays: terrain-aware A* route, confirmed casualty, state.
        for sub in self._mission_subs:
            try:
                self._node.destroy_subscription(sub)
            except Exception:
                pass
                
        # Connect UI logic to mission control logic
        self._mission_subs = [
            self._node.create_subscription(Path, '/mission/route', self._route_cb, LATCHED),
            self._node.create_subscription(
                PointStamped, '/mission/casualty', self._casualty_cb, LATCHED),
            self._node.create_subscription(
                PointStamped, '/mission/casualty_candidate', self._candidate_cb, LATCHED),
            self._node.create_subscription(
                PointStamped, '/mission/husky_hold', self._hold_cb, LATCHED),
            self._node.create_subscription(String, '/mission/state', self._state_cb, LATCHED),
            self._node.create_subscription(
                String, '/mission/husky_status', self._husky_status_cb, LATCHED),
        ]

        # Target Goal Pose Subscriptions (where the robots are individually heading)
        self._husky_wp_sub = self._node.create_subscription(
            PoseStamped, '/husky1/goal_pose', self._husky_wp_cb, 10)
        self._drone_wp_sub = self._node.create_subscription(
            PoseStamped, '/parrot1/goal_pose', self._drone_wp_cb, 10)

    # -------------------------------------------------------------------------
    # ROS CALLBACKS (Extracting 2D x,y coordinates from incoming messages)
    # -------------------------------------------------------------------------
    def _husky_odom_cb(self, msg: Odometry) -> None:
        self._husky_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _drone_odom_cb(self, msg: Odometry) -> None:
        self._drone_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _drone2_odom_cb(self, msg: Odometry) -> None:
        self._drone2_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _drone2_plan_cb(self, msg: Path) -> None:
        self._drone2_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _husky_plan_cb(self, msg: Path) -> None:
        self._husky_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _drone_plan_cb(self, msg: Path) -> None:
        self._drone_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _route_cb(self, msg: Path) -> None:
        self._mission_route = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._route_start = self._husky_pos
        self._route_passed = 0

    def _candidate_cb(self, msg: PointStamped) -> None:
        self._candidate = (msg.point.x, msg.point.y)

    def _hold_cb(self, msg: PointStamped) -> None:
        self._hold = (msg.point.x, msg.point.y)

    def _casualty_cb(self, msg: PointStamped) -> None:
        self._casualty = (msg.point.x, msg.point.y)

    def _state_cb(self, msg: String) -> None:
        self._mission_state = msg.data

    def _husky_status_cb(self, msg: String) -> None:
        self._husky_status = msg.data

    def _husky_wp_cb(self, msg: PoseStamped) -> None:
        self._husky_target_wp = (msg.pose.position.x, msg.pose.position.y)

    def _drone_wp_cb(self, msg: PoseStamped) -> None:
        self._drone_target_wp = (msg.pose.position.x, msg.pose.position.y)

    def shutdown(self) -> None:
        self._timer.stop()
        if self._node is not None:
            subs = [self._subscription, self._husky_odom_sub, self._drone_odom_sub, 
                    self._husky_plan_sub, self._husky_wp_sub, self._drone_plan_sub, self._drone_wp_sub,
                    self._drone2_odom_sub, self._drone2_plan_sub,
                    *self._mission_subs]
            for sub in subs:
                if sub is not None:
                    try:
                        self._node.destroy_subscription(sub)
                    except Exception:
                        pass
        self._subscription = None

    # -------------------------------------------------------------------------
    # POINTCLOUD PROCESSING
    # -------------------------------------------------------------------------
    def _extract_points(self, msg: PointCloud2) -> np.ndarray:
        """(N, 4) array of x, y, z, sigma. sigma is NaN if the cloud has no sigma field.
        
        Reads the raw byte stream from the PointCloud2 message and structures it 
        into a clean NumPy array for rapid mathematical processing.
        """
        names = ['x', 'y', 'z'] + (['sigma'] if any(f.name == 'sigma' for f in msg.fields) else [])
        raw_points = point_cloud2.read_points(
            msg,
            field_names=names,
            skip_nans=True,
        )

        if isinstance(raw_points, np.ndarray):
            arr = raw_points
        else:
            arr = np.asarray(list(raw_points))

        if arr.size == 0:
            return np.empty((0, 4), dtype=np.float64)

        if getattr(arr.dtype, 'names', None):
            arr = np.column_stack([arr[n] for n in names]).astype(np.float64, copy=False)
        else:
            arr = np.asarray(arr, dtype=np.float64)
            if arr.ndim == 1:
                arr = arr.reshape(1, -1)
            if arr.shape[1] < len(names):
                return np.empty((0, 4), dtype=np.float64)
            arr = arr[:, :len(names)]

        # If no sigma data exists, pad the 4th column with NaNs
        if arr.shape[1] == 3:
            arr = np.column_stack((arr, np.full(len(arr), np.nan)))
        return arr

    def _build_grid(self, points: np.ndarray, column: int) -> np.ndarray:
        """Grid of one point column (2 = z, 3 = sigma); the builder sends one point per cell.
        
        Translates continuous real-world (X, Y) coordinates into discrete grid indices 
        (gx, gy). If multiple points map to the same cell, np.fmax.at keeps the maximum value.
        """
        grid = np.full((self.grid_height, self.grid_width), np.nan, dtype=np.float64)

        points = points[np.isfinite(points[:, :2]).all(axis=1) & np.isfinite(points[:, column])]
        gx = np.floor(
            points[:, 0] / self.RESOLUTION + self.half_cells_x + 0.5
        ).astype(np.int64)
        gy = np.floor(
            points[:, 1] / self.RESOLUTION + self.half_cells_y + 0.5
        ).astype(np.int64)
        
        # Filter out points that fall outside the 40x40m map boundaries
        valid = (gx >= 0) & (gx < self.grid_width) & (gy >= 0) & (gy < self.grid_height)
        np.fmax.at(grid, (gy[valid], gx[valid]), points[valid, column])

        return grid

    def _build_slope(self, elevation: np.ndarray) -> np.ndarray:
        """Steepest angle (deg) to the 4 neighbours, needing >= 2 known neighbours.

        Same rule as TerrainMap::calculateSlopes, but on every mapped cell: the planner
        first drops cells with < 3 returns and fills small holes, so it can differ
        slightly at the edges of scanned areas.
        """
        # Pad the elevation grid with NaNs so we don't index out of bounds on the edges
        pad = np.pad(elevation, 1, constant_values=np.nan)
        centre = pad[1:-1, 1:-1]
        
        # Create a stack of the Top, Bottom, Left, and Right neighboring cells
        neighbours = np.stack([pad[1:-1, 2:], pad[1:-1, :-2], pad[2:, 1:-1], pad[:-2, 1:-1]])
        
        # Calculate slope angle in degrees using arctan(height_diff / grid_resolution)
        angles = np.degrees(np.arctan2(np.abs(neighbours - centre), self.RESOLUTION))
        known = np.isfinite(angles)
        slope = np.where(known, angles, -np.inf).max(axis=0)
        
        # Require at least 2 valid neighbors to compute a slope, else mark as NaN
        slope[(known.sum(axis=0) < 2) | ~np.isfinite(centre)] = np.nan
        return slope

    def _cloud_cb(self, msg: PointCloud2) -> None:
        """Triggered whenever a new Terrain PointCloud arrives."""
        try:
            points = self._extract_points(msg)
            if points.size == 0:
                return

            # Keep track of global Z min/max for text display
            finite_z = points[np.isfinite(points[:, :3]).all(axis=1), 2]
            self._cloud_points = int(finite_z.size)
            self._cloud_frame = msg.header.frame_id
            if finite_z.size > 0:
                self._height_min = float(np.min(finite_z))
                self._height_max = float(np.max(finite_z))

            # Build the grids for the 3 available views
            self._elevation = self._build_grid(points, 2)
            self._sigma = self._build_grid(points, 3)
            self._slope = self._build_slope(self._elevation)
            self._known_cells = int(np.isfinite(self._elevation).sum())
        except Exception as exc:
            if self._node is not None:
                self._node.get_logger().warning(
                    f'Could not decode terrain cloud on {self._topic}: {exc}',
                    throttle_duration_sec=5.0,
                )
            return

    # -------------------------------------------------------------------------
    # QT RENDERING
    # -------------------------------------------------------------------------
    def _render_image(self, values: np.ndarray, value_stops: list[float], unit: str) -> QImage | None:
        """Converts numerical grid data (height/slope/roughness) into a colored QImage."""
        known = np.isfinite(values)
        if not known.any():
            return None

        v_min = float(np.min(values[known]))
        v_max = float(np.max(values[known]))

        # Colours are pinned to fixed values, not stretched over min..max:
        # terrain_demo ground spans -0.32..10.6 m but ~95% of it is below 4 m, so a
        # min..max stretch would squash most of the ground into one colour.
        # Anything above the last stop stays red. Tune the VIEWS stops for other worlds.
        
        # Color spectrum: Green -> Yellow -> Orange -> Red
        colours = np.array([[0, 200, 0], [255, 255, 0], [255, 140, 0], [255, 0, 0]], dtype=np.float64)
        
        # Fill missing areas with the minimum stop value (e.g. Green) to avoid NaN errors in interpolation
        filled = np.where(known, values, value_stops[0])
        
        # Interpolate the R, G, and B channels based on the values array mapping to value_stops
        rgb = np.stack([np.interp(filled, value_stops, colours[:, c]) for c in range(3)],
                       axis=-1).astype(np.uint8)
                       
        # Reset unknown cells to Black
        rgb[~known] = 0

        # Flip the array vertically. (In ROS, +Y is 'Left/Up', but in Qt UI (0,0) is top-left, so +Y goes Down).
        rgb = np.ascontiguousarray(rgb[::-1])
        image = QImage(
            rgb.data, self.grid_width, self.grid_height,
            3 * self.grid_width, QImage.Format_RGB888,
        )
        self._z_range_text = f'low {v_min:.2f} / high {v_max:.2f} {unit}'
        return image.copy()
        
    def paintEvent(self, event) -> None:
        """Main Qt draw loop. Draws background, terrain image, routes, markers, and text."""
        painter = QPainter(self)
        painter.fillRect(self.rect(), Qt.black)

        # 1. Fetch current view data (Height, Slope, or Roughness)
        attr, value_stops, unit = self.VIEWS[self._view]
        values = getattr(self, attr)
        image = self._render_image(values, value_stops, unit) if values is not None else None
        
        if image is None:
            painter.setPen(QPen(QColor(255, 255, 255)))
            waiting = (f'Waiting for {self._topic}...' if self._elevation is None
                       else f'No {self._view.lower()} data in {self._topic}')
            painter.drawText(self.rect(), Qt.AlignCenter, waiting)
            painter.end()
            return

        # 2. Calculate scaling to fit the image into the UI widget while preserving aspect ratio
        top, bottom = 76, 34
        avail_h = max(1, self.height() - top - bottom)
        scale = min(self.width() / self.grid_width, avail_h / self.grid_height)
        draw_w = max(1, int(self.grid_width * scale))
        draw_h = max(1, int(self.grid_height * scale))
        target = QRect(
            (self.width() - draw_w) // 2,
            top + (avail_h - draw_h) // 2,
            draw_w,
            draw_h,
        )
        
        # 3. Draw the terrain grid map
        painter.drawImage(target, image)

        # ---------------------------------------------------------------------
        # DRAWING HELPERS
        # ---------------------------------------------------------------------
        def world_to_pixel(world_x: float, world_y: float) -> tuple[int, int]:
            """Translates real-world (X,Y) coordinates to UI screen pixels."""
            gx = (world_x / self.RESOLUTION) + self.half_cells_x + 0.5
            gy = (world_y / self.RESOLUTION) + self.half_cells_y + 0.5
            # Invert Y to account for the rgb[::-1] vertical flip done in _render_image
            gy_img = self.grid_height - gy
            px = target.x() + (gx * (draw_w / self.grid_width))
            py = target.y() + (gy_img * (draw_h / self.grid_height))
            return int(px), int(py)

        def draw_trajectory(current_pos, path, goal, color):
            """Draws predicted paths/waypoints for individual robots."""
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
            """Draws the robot's current physical position."""
            if pos is None:
                return
            px, py = world_to_pixel(pos[0], pos[1])
            painter.setBrush(QColor(color))
            painter.setPen(QPen(Qt.white, 1))
            painter.drawEllipse(px - 5, py - 5, 10, 10)

        # ---------------------------------------------------------------------
        # EXECUTE DRAW SEQUENCE (Order matters: items drawn later appear on top)
        # ---------------------------------------------------------------------
        
        # Draw trajectories then robots on top
        # Husky trajectory = Cyan, Drone trajectory = Pink
        draw_trajectory(self._husky_pos, self._husky_path, self._husky_target_wp, 'cyan')
        draw_trajectory(self._drone_pos, self._drone_path, self._drone_target_wp, 'pink')
        draw_trajectory(self._drone2_pos, self._drone2_path, None, 'white')

        # Terrain-aware A* route (solid yellow) with its waypoints.
        if len(self._mission_route) >= 1:
            pts = [world_to_pixel(x, y) for (x, y) in self._mission_route]
            line = ([world_to_pixel(*self._route_start)] if self._route_start else []) + pts
            painter.setPen(QPen(QColor('yellow'), 3))
            for i in range(len(line) - 1):
                painter.drawLine(line[i][0], line[i][1], line[i + 1][0], line[i + 1][1])
            painter.setBrush(QColor('yellow'))
            painter.setPen(QPen(Qt.black, 1))
            for px, py in pts:
                painter.drawEllipse(px - 3, py - 3, 6, 6)

        # Detection awaiting review (orange ring), confirmed casualty (red cross).
        if self._candidate is not None and self._casualty is None:
            cx, cy = world_to_pixel(*self._candidate)
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(QColor('orange'), 3))
            painter.drawEllipse(cx - 9, cy - 9, 18, 18)
        if self._casualty is not None:
            cx, cy = world_to_pixel(*self._casualty)
            painter.setPen(QPen(QColor('red'), 3))
            # Draw an X
            painter.drawLine(cx - 8, cy - 8, cx + 8, cy + 8)
            painter.drawLine(cx - 8, cy + 8, cx + 8, cy - 8)

        # Draw robot markers (Husky = Blue, Drone = Magenta)
        draw_robot_marker(self._husky_pos, 'blue')
        draw_robot_marker(self._drone_pos, 'magenta')
        draw_robot_marker(self._drone2_pos, 'white')

        # ---------------------------------------------------------------------
        # TEXT OVERLAYS (Status, Stats, Legend)
        # ---------------------------------------------------------------------
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.drawText(
            8, 16,
            f'{self._view}: {self._z_range_text}  (green {value_stops[0]:g} → yellow {value_stops[1]:g} '
            f'→ orange {value_stops[2]:g} → red {value_stops[3]:g}+ {unit})  '
            f'cells: {self._known_cells}  points: {self._cloud_points}  '
            f'frame: {self._cloud_frame or "unknown"}',
        )
        painter.drawText(
            8, 34,
            f'Mission: {self._mission_state}   Husky: {self._husky_status}',
        )
        
        # Line 3: where the casualty is. Line 4: Husky progress (kept short so
        # a narrow dock does not cut it off).
        line3 = line4 = ''
        if self._casualty is not None:
            line3 = f'Casualty ({self._casualty[0]:.2f}, {self._casualty[1]:.2f})'
        elif self._candidate is not None:
            line3 = (f'Detection ({self._candidate[0]:.2f}, {self._candidate[1]:.2f}) '
                     'awaiting review')
                     
        # Dynamic distance string calculation
        if self._casualty is not None and self._husky_pos is not None:
            dist = np.hypot(self._husky_pos[0] - self._casualty[0],
                            self._husky_pos[1] - self._casualty[1])
            line4 = f'Husky {dist:.1f} m from casualty'
            if self._husky_status == 'ARRIVED' and self._hold is not None:
                moved = np.hypot(self._husky_pos[0] - self._hold[0], self._husky_pos[1] - self._hold[1])
                line4 += f', stopped (moved {moved:.2f} m since)'
            if self._mission_route and self._mission_state == 'HUSKY_EN_ROUTE':
                self._route_passed, left = route_progress(
                    self._mission_route, self._husky_pos, self._route_passed)
                line4 += (f', wp {self._route_passed + 1}/{len(self._mission_route)}'
                          f', {left:.1f} m left')
                          
        if line3:
            painter.drawText(8, 52, line3)
        if line4:
            painter.drawText(8, 70, line4)
            
        # Draw Legends at the bottom of the widget
        painter.drawText(8, self.height() - 26, 'yellow = A* route   orange = detection   red X = casualty')
        painter.drawText(8, self.height() - 8,
                         'blue = Husky   magenta = drone' +
                         ('   white = drone 2' if self._drone2_pos is not None else ''))
        
        # Finalize drawing
        painter.end()


# =============================================================================
# GUI WIDGET 2: ROBOT MOTION & MISSION TOGGLE
# =============================================================================
class RobotMotionToggleWidget(QtWidgets.QWidget):
    """
    Control panel widget providing mission operators with buttons to 
    stop/start robots and accept/reject thermal casualty detections.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('RobotMotionToggleWidget')
        self.setWindowTitle('Robot Motion Toggle')
        self.resize(360, 360)
        # Keep the confirm buttons and mission status readable however the
        # saved dashboard layout sizes the docks.
        self.setMinimumSize(340, 330)
        
        # Core communication attributes
        self._node: Node | None = None
        self._publishers = {}
        self._subscriptions = {}
        
        # Operator confirmation of a thermal detection (parrot_node waits on it).
        self._confirm_topic = '/CasulityConfirmed'
        self._discovered_topic = '/CasulityDiscovered'
        
        self._waypoint_publisher = None
        self._discovered_sub = None
        self._parrot_status_sub = None
        self._mission_subs = []
        
        # Local state storage
        self._casualty_pending = False
        self._last_answer: bool | None = None
        self._parrot_status = 'unknown'
        self._mission_state = 'unknown'
        self._husky_status = 'unknown'
        self._candidate = None
        self._casualty = None
        self._husky_pos = None
        self._route = []
        self._route_passed = 0
        self._odom_time = None
        self._hold = None
        
        # True = robot running (button reads "Stop"). The drone starts PAUSED in
        # the TRACE demo; _parrot_status_cb corrects this if it is already flying.
        self._robot_states = {'husky1': True, 'parrot1': False, 'parrot2': False}
        self._parrot_started = False   # operator has pressed Start Parrot at least once

        # Second drone: its button and status line only appear once /parrot2
        # reports a status, so the one-drone dashboard looks exactly as before.
        self._parrot2_status_sub = None
        self._parrot2_present = False
        self._parrot2_status = 'unknown'
        self._parrot2_started = False
        
        # Update UI text dynamically at 5 FPS
        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._update_labels)
        self._ui_timer.start(200)
        
        self._build_ui()

    def set_node(self, node: Node) -> None:
        self._node = node
        self._ensure_waypoint_publisher()
        
        # Subscribe to drone's detection pipeline
        self._discovered_sub = self._node.create_subscription(
            Bool, self._discovered_topic, self._discovered_cb, LATCHED)
        self._parrot_status_sub = self._node.create_subscription(
            String, '/parrot1/mission/status', self._parrot_status_cb, LATCHED)
        self._parrot2_status_sub = self._node.create_subscription(
            String, '/parrot2/mission/status', self._parrot2_status_cb, LATCHED)
            
        # Mission state, so the panel never shows a stale message.
        self._mission_subs = [
            self._node.create_subscription(String, '/mission/state', self._mission_state_cb, LATCHED),
            self._node.create_subscription(
                String, '/mission/husky_status', self._husky_status_cb, LATCHED),
            self._node.create_subscription(
                PointStamped, '/mission/casualty_candidate', self._candidate_cb, LATCHED),
            self._node.create_subscription(
                PointStamped, '/mission/casualty', self._casualty_cb, LATCHED),
            self._node.create_subscription(Path, '/mission/route', self._route_cb, LATCHED),
            self._node.create_subscription(Odometry, '/husky1/odometry', self._odom_cb, 10),
            self._node.create_subscription(
                PointStamped, '/mission/husky_hold', self._hold_cb, LATCHED),
        ]
        
        # Set up estop controls for both husky and parrot
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

        for subscription in [*self._subscriptions.values(), *self._mission_subs,
                             self._discovered_sub, self._parrot_status_sub,
                             self._parrot2_status_sub]:
            if subscription is None:
                continue
            try:
                self._node.destroy_subscription(subscription)
            except Exception:
                pass
        self._subscriptions.clear()

    def _build_ui(self) -> None:
        """Constructs the buttons and labels for the control panel."""
        # Scrollable, so a small rqt dock never cuts off the mission status.
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        inner = QtWidgets.QWidget()
        scroll.setWidget(inner)
        outer.addWidget(scroll)
        layout = QtWidgets.QVBoxLayout(inner)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        title = QtWidgets.QLabel('Robot Toggle')
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Casualty Alert Text
        self._casualty_label = QtWidgets.QLabel('No heat signature awaiting review')
        self._casualty_label.setWordWrap(True)
        layout.addWidget(self._casualty_label)

        # Confirm / Reject Action Buttons
        row = QtWidgets.QHBoxLayout()
        self._waypoint_button = QtWidgets.QPushButton('Confirm casualty → send Husky')
        self._waypoint_button.clicked.connect(lambda: self._answer_casualty(True))
        row.addWidget(self._waypoint_button)
        self._reject_button = QtWidgets.QPushButton('Reject (false alarm)')
        self._reject_button.clicked.connect(lambda: self._answer_casualty(False))
        row.addWidget(self._reject_button)
        layout.addLayout(row)

        # Emergency Stop / Start Buttons
        self._husky_button = QtWidgets.QPushButton('Stop Husky')
        self._husky_button.clicked.connect(lambda: self._toggle_robot('husky1'))
        layout.addWidget(self._husky_button)

        self._parrot_button = QtWidgets.QPushButton('Stop Parrot')
        self._parrot_button.clicked.connect(lambda: self._toggle_robot('parrot1'))
        layout.addWidget(self._parrot_button)

        self._parrot2_button = QtWidgets.QPushButton('Start Parrot 2')
        self._parrot2_button.clicked.connect(lambda: self._toggle_robot('parrot2'))
        self._parrot2_button.setVisible(False)   # shown once the second drone reports in
        layout.addWidget(self._parrot2_button)

        # General Mission Status Block
        self._mission_label = QtWidgets.QLabel('Mission: waiting for mission control...')
        self._mission_label.setWordWrap(True)
        self._mission_label.setStyleSheet('padding: 4px; border: 1px solid #888;')
        layout.addWidget(self._mission_label)

        # Small dev debug line at the bottom
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
            # Latched to match parrot_node's subscription; a volatile
            # publisher would be QoS-incompatible and never delivered.
            self._waypoint_publisher = self._node.create_publisher(
                Bool, self._confirm_topic, LATCHED)
        return self._waypoint_publisher

    # -------------------------------------------------------------------------
    # STATE UPDATE CALLBACKS
    # -------------------------------------------------------------------------
    def _discovered_cb(self, msg: Bool) -> None:
        if msg.data:
            self._casualty_pending = True
            self._last_answer = None

    def _parrot_status_cb(self, msg: String) -> None:
        self._parrot_status = msg.data
        # parrot_node launched with start_paused:=false (or rqt restarted mid-run):
        # the drone is already flying, so show "Stop Parrot".
        if not self._parrot_started and msg.data in ('CLIMBING', 'IDLE', 'SEARCHING', 'HOLDING'):
            self._robot_states['parrot1'] = True
            self._parrot_started = True

    def _parrot2_status_cb(self, msg: String) -> None:
        self._parrot2_status = msg.data
        self._parrot2_present = True
        if not self._parrot2_started and msg.data in ('CLIMBING', 'IDLE', 'SEARCHING', 'HOLDING'):
            self._robot_states['parrot2'] = True
            self._parrot2_started = True

    def _mission_state_cb(self, msg: String) -> None:
        if msg.data != self._mission_state and self._mission_state != 'unknown':
            self._status.setText(f'Mission state changed to {msg.data}.')
        self._mission_state = msg.data
        
        # Once mission control has the casualty, there is nothing left to review
        # (also covers a confirmation sent from the terminal, or rqt restarting
        # and receiving the old latched "discovered" message).
        if msg.data not in ('SEARCHING', 'unknown'):
            self._casualty_pending = False
        if msg.data in ('PLANNING', 'HUSKY_EN_ROUTE', 'DONE'):
            self._last_answer = True

    def _husky_status_cb(self, msg: String) -> None:
        self._husky_status = msg.data

    def _candidate_cb(self, msg: PointStamped) -> None:
        self._candidate = (msg.point.x, msg.point.y)

    def _casualty_cb(self, msg: PointStamped) -> None:
        self._casualty = (msg.point.x, msg.point.y)
        self._casualty_pending = False
        self._last_answer = True

    def _route_cb(self, msg: Path) -> None:
        self._route = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._route_passed = 0

    def _hold_cb(self, msg: PointStamped) -> None:
        self._hold = (msg.point.x, msg.point.y)

    def _odom_cb(self, msg: Odometry) -> None:
        self._husky_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._odom_time = time.monotonic()

    # -------------------------------------------------------------------------
    # ACTION EMITTERS
    # -------------------------------------------------------------------------
    def _publish_stop_state(self, robot_name: str, stopped: bool) -> None:
        publisher = self._ensure_publisher(robot_name)
        if publisher is not None:
            publisher.publish(Bool(data=stopped))

    def _answer_casualty(self, confirmed: bool) -> None:
        if self._node is None:
            self._status.setText('ROS node is not connected yet.')
            return
        
        # Validate that we actually have a pending candidate before allowing a confirm publish
        have_candidate = self._candidate is not None and self._casualty is None
        if not self._casualty_pending and not (confirmed and have_candidate):
            self._status.setText('No heat signature is waiting for review.')
            return
            
        publisher = self._ensure_waypoint_publisher()
        publisher.publish(Bool(data=confirmed))
        self._last_answer = confirmed
        
        # Any operator answer clears the urgent popup; a rejected candidate is
        # still shown on the map and can be confirmed later.
        self._casualty_pending = False
        self._status.setText(
            'Casualty CONFIRMED: drone circles it, Husky route is being planned.'
            if confirmed else 'Detection REJECTED: you can still confirm this candidate later if needed.')

    def _estop_state_cb(self, robot_name: str, msg: Bool) -> None:
        self._robot_states[robot_name] = not bool(msg.data)

    def _update_labels(self) -> None:
        """Called repeatedly by QTimer to parse mission state arrays into rich HTML text."""
        # 1. Update the prominent colored Alert Banner
        if self._casualty_pending:
            where = (f' at ({self._candidate[0]:.1f}, {self._candidate[1]:.1f})'
                     if self._candidate else '')
            self._casualty_label.setText(
                f'⚠ HEAT SIGNATURE DETECTED{where} — drone is holding. Check the '
                'thermal camera view, then confirm or reject.')
            self._casualty_label.setStyleSheet('color: white; background: #b00020; padding: 4px;')
        elif self._mission_state == 'DONE':
            self._casualty_label.setText('✔ Husky has reached the casualty.')
            self._casualty_label.setStyleSheet('color: white; background: #1b7f2a; padding: 4px;')
        elif self._mission_state == 'FAILED':
            self._casualty_label.setText('✖ Husky could not reach the casualty: operator decision needed.')
            self._casualty_label.setStyleSheet('color: black; background: #f2b600; padding: 4px;')
        elif self._casualty is not None:
            self._casualty_label.setText('Casualty confirmed. No detection awaiting review.')
            self._casualty_label.setStyleSheet('')
        else:
            self._casualty_label.setText('No detection awaiting review.')
            self._casualty_label.setStyleSheet('')

        # 2. Update the multi-line Status Box
        lines = [f'<b>Mission:</b> {self._mission_state} — '
                 f'{MISSION_TEXT.get(self._mission_state, "waiting for mission control")}']
        drone = self._parrot_status
        if self._mission_state in ('PLANNING', 'HUSKY_EN_ROUTE') and drone == 'SEARCHING':
            drone += ' (patrolling for further casualties)'
        elif self._mission_state == 'DONE':
            drone += (' (overwatch near the casualty)' if drone == 'HOLDING'
                      else ' (flying to overwatch near the casualty)')
        elif not self._parrot_started:
            drone += ' (on the ground: press Start Parrot to begin the search)'
        lines.append(f'<b>Drone{" 1" if self._parrot2_present else ""}:</b> {drone}')
        if self._parrot2_present:
            drone2 = self._parrot2_status
            if not self._parrot2_started:
                drone2 += ' (on the ground: press Start Parrot 2)'
            lines.append(f'<b>Drone 2:</b> {drone2}')
        
        husky = self._husky_status
        if self._casualty is not None and self._husky_pos is not None:
            dist = np.hypot(self._husky_pos[0] - self._casualty[0],
                            self._husky_pos[1] - self._casualty[1])
            husky += f', {dist:.1f} m from the casualty'
            if self._husky_status == 'ARRIVED' and self._hold is not None:
                moved = np.hypot(self._husky_pos[0] - self._hold[0], self._husky_pos[1] - self._hold[1])
                husky += f', stopped (moved {moved:.2f} m since)'
            if self._route and self._mission_state == 'HUSKY_EN_ROUTE':
                self._route_passed, left = route_progress(self._route, self._husky_pos,
                                                          self._route_passed)
                husky += (f', waypoint {self._route_passed + 1}/{len(self._route)}'
                          f' ({left:.1f} m of route left)')
        lines.append(f'<b>Husky:</b> {husky}')
        
        if self._casualty is not None:
            lines.append(f'<b>Casualty:</b> ({self._casualty[0]:.2f}, {self._casualty[1]:.2f})')
        elif self._candidate is not None:
            lines.append(f'<b>Detection:</b> ({self._candidate[0]:.2f}, {self._candidate[1]:.2f})')
            
        # Make a silent link visible: stale Husky odometry or no mission control.
        if self._odom_time is not None and time.monotonic() - self._odom_time > 5.0:
            lines.append(f'<span style="color:#b00020"><b>⚠ No Husky odometry for '
                         f'{time.monotonic() - self._odom_time:.0f} s</b></span>')
        if self._mission_state == 'unknown':
            lines.append('<span style="color:#b00020">⚠ No /mission/state yet: is '
                         'system_control_node running?</span>')
                         
        # Compile HTML and push to UI
        self._mission_label.setText('<br>'.join(lines))
        
        # 3. Update Button Enabled/Disabled States
        confirmed = self._casualty is not None or self._mission_state in ('PLANNING', 'HUSKY_EN_ROUTE', 'DONE')
        can_confirm = (self._casualty_pending or self._candidate is not None) and not confirmed
        
        # Once either answer is sent, keep Reject disabled to avoid repeated toggles.
        reject_locked = self._last_answer is not None
        self._waypoint_button.setEnabled(can_confirm)
        self._reject_button.setEnabled(self._casualty_pending and not reject_locked and not confirmed)
        
        # Update the Stop/Start buttons based on current estop bools
        self._husky_button.setText('Stop Husky' if self._robot_states.get('husky1', False) else 'Start Husky')
        number = ' 1' if self._parrot2_present else ''
        self._parrot_button.setText(('Stop Parrot' if self._robot_states.get('parrot1', False)
                                     else 'Start Parrot') + number)
        self._parrot2_button.setVisible(self._parrot2_present)
        self._parrot2_button.setText('Stop Parrot 2' if self._robot_states.get('parrot2', False)
                                     else 'Start Parrot 2')

    def _toggle_robot(self, robot_name: str) -> None:
        """Flips the boolean published to the specified robot's /estop topic."""
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
            if clean_name == 'parrot1':
                self._parrot_started = True
            if clean_name == 'parrot2':
                self._parrot2_started = True

        self._update_labels()

# =============================================================================
# RQT PLUGIN WRAPPERS
# =============================================================================
class TerrainMapPlugin(Plugin):
    """
    Wrapper class required by rqt to load the TerrainMapWidget into the dashboard.
    """
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
        # Save the topic input field string if the perspective is exported
        instance_settings.set_value('pointcloud_topic', self._widget.topic())

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore the topic input field string when opening from a perspective file
        topic = instance_settings.value('pointcloud_topic', self._widget.topic())
        if topic:
            self._widget.set_topic(topic)

    def trigger_configuration(self):
        pass


class RobotMotionTogglePlugin(Plugin):
    """
    Wrapper class required by rqt to load the RobotMotionToggleWidget into the dashboard.
    """
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