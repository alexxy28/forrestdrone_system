#!/usr/bin/env python3
"""TRACE test logger: records one mission and scores it against Gazebo's ground truth.

Evaluation only: it never publishes anything, and no robot uses its output.
It listens to the mission topics, and asks Gazebo (`ign topic`) for the true
positions of the person (human_heat1) and the Husky. When the mission ends
(DONE or FAILED) it keeps watching the Husky for hold_check_s of simulated
time (default 120 s), so a robot that drifts after "arrival" is caught. Then
(or when you press Ctrl+C) it prints a summary and appends a row to
~/trace_results/results.csv:

    human_pose, true human (x, y) and body centre, detected (x, y), detection
    error to the body centre (and to the feet), planned goal (last route
    waypoint), route waypoints, Husky position at arrival and after the hold
    check (Gazebo), how far it moved after arrival, stopping distance to the
    person's body (its collision box), to the detected point and to the model
    origin, whether the mission stayed DONE, navigation result, timings.

The human model's origin is its FEET: model.sdf is a standing 1.77 m figure
that the world rotates to lie down. So "distance to the model origin" can be
~1.8 m more than the distance to the person when the Husky stops near the
head; the body distance is the meaningful one.

Run it on its own (any terminal, after sourcing the workspace):
    ros2 run 41068_ignition_bringup mission_logger.py
trace_demo.launch.py starts it automatically (log_results:=true).
"""

import csv
import math
import os
import re
import subprocess
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def gazebo_poses(world, names, timeout=15.0):
    """True poses {name: ((x, y, z), (qx, qy, qz, qw))} from Gazebo, or {} if unavailable."""
    for cli in ('ign', 'gz'):
        try:
            out = subprocess.run([cli, 'topic', '-e', '-n', '1', '-t', f'/world/{world}/pose/info'],
                                 capture_output=True, text=True, timeout=timeout).stdout
        except (OSError, subprocess.TimeoutExpired):
            continue
        if out:
            break
    else:
        return {}
    found = {}
    # Protobuf text omits zero fields, so default every coordinate to 0.
    for name in names:
        m = re.search(r'name: "%s"\s*id: \d+\s*position \{([^}]*)\}\s*(orientation \{([^}]*)\})?'
                      % re.escape(name), out)
        if not m:
            continue
        p = {k: float(v) for k, v in re.findall(r'([xyz]): ([-0-9.e+]+)', m.group(1))}
        q = {k: float(v) for k, v in re.findall(r'([xyzw]): ([-0-9.e+]+)', m.group(3) or '')}
        found[name] = ((p.get('x', 0.0), p.get('y', 0.0), p.get('z', 0.0)),
                       (q.get('x', 0.0), q.get('y', 0.0), q.get('z', 0.0), q.get('w', 1.0)))
    return found


def gazebo_positions(world, names, timeout=15.0):
    return {k: v[0] for k, v in gazebo_poses(world, names, timeout).items()}


def body_footprint(pose, size=(0.76, 0.33, 1.77), centre_z=0.9):
    """2D outline (convex polygon) of the person's collision box from human/model.sdf.

    The model is a standing figure (feet at the origin, 1.77 m tall) that the
    world rotates to lie on the ground, so its origin is the feet, not the body.
    """
    (px, py, pz), (qx, qy, qz, qw) = pose
    r = [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
         [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)]]
    pts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                c = (sx * size[0] / 2, sy * size[1] / 2, centre_z + sz * size[2] / 2)
                pts.append((px + sum(r[0][i] * c[i] for i in range(3)),
                            py + sum(r[1][i] * c[i] for i in range(3))))
    pts = sorted(set(pts))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def distance_to_polygon(p, poly):
    """0 inside the polygon, else the distance to its nearest edge."""
    inside = all(((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])) >= 0
                 for a, b in zip(poly, poly[1:] + poly[:1]))
    if inside:
        return 0.0
    best = float('inf')
    for a, b in zip(poly, poly[1:] + poly[:1]):
        ax, ay = b[0] - a[0], b[1] - a[1]
        t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / (ax * ax + ay * ay or 1)))
        best = min(best, math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay))
    return best


def body_centre(pose, centre_z=0.9):
    """Middle of the person's body (x, y): the model's (0, 0, 0.9) point, rotated."""
    (px, py, pz), (qx, qy, qz, qw) = pose
    return (px + 2 * (qx * qz + qy * qw) * centre_z,
            py + 2 * (qy * qz - qx * qw) * centre_z)


def fmt(p):
    return '(%.2f, %.2f)' % (p[0], p[1]) if p else 'n/a'


class MissionLogger(Node):
    def __init__(self):
        super().__init__('mission_logger')
        self.world = self.declare_parameter('world', 'terrain_demo').value
        # "0".."5", "random" or an integer: accept any type.
        self.pose_label = str(self.declare_parameter(
            'human_pose', '', ParameterDescriptor(dynamic_typing=True)).value)
        self.results_dir = os.path.expanduser(
            self.declare_parameter('results_dir', '~/trace_results').value)
        self.human_name = self.declare_parameter('human_model', 'human_heat1').value
        self.husky_name = self.declare_parameter('husky_model', 'husky1').value

        self.t0 = time.time()
        self.events = []
        self.true_human = None
        self.candidate = None
        self.detected = None
        self.route = []
        self.route_count = 0
        self.state = None
        self.husky_status = None
        self.husky_odom = None
        self.t_detect = self.t_confirm = self.t_route = self.t_end = None
        self.estops = 0
        self.reported = False
        # After DONE: keep watching the Husky (sim seconds).
        self.hold_check_s = float(self.declare_parameter('hold_check_s', 120.0).value)
        self.arrival = None          # Gazebo Husky position just after DONE
        self.hold_start = None
        self.hold_max_drift = 0.0
        self.states_after_end = []
        self.hold_timer = None

        self.create_subscription(PointStamped, '/mission/casualty_candidate', self.on_candidate, LATCHED)
        self.create_subscription(Bool, '/CasulityConfirmed', self.on_confirm, LATCHED)
        self.create_subscription(PointStamped, '/mission/casualty', self.on_casualty, LATCHED)
        self.create_subscription(Path, '/mission/route', self.on_route, LATCHED)
        self.create_subscription(String, '/mission/state', self.on_state, LATCHED)
        self.create_subscription(String, '/mission/husky_status', self.on_husky_status, LATCHED)
        self.create_subscription(Odometry, '/husky1/odometry', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Bool, '/husky1/estop', self.on_estop, 10)
        self.create_timer(5.0, self.fetch_ground_truth_once)
        self.get_logger().info(f'Recording mission (results -> {self.results_dir}/results.csv)')

    # ---------------- events ----------------
    def log(self, text):
        line = f'[TRACE TEST +{time.time() - self.t0:6.0f}s] {text}'
        self.events.append(line)
        self.get_logger().info(line)

    def fetch_ground_truth_once(self):
        if self.true_human is not None:
            return
        pos = gazebo_positions(self.world, [self.human_name])
        if self.human_name in pos:
            self.true_human = pos[self.human_name]
            self.log(f'Ground truth (evaluation only): person {self.human_name} at {fmt(self.true_human)}')

    def on_candidate(self, msg):
        self.candidate = (msg.point.x, msg.point.y)
        if self.t_detect is None:
            self.t_detect = time.time()
        err = ''
        if self.true_human:
            err = ', %.2f m from the person' % math.dist(self.candidate, self.true_human[:2])
        self.log(f'DETECTED heat signature at {fmt(self.candidate)}{err}: awaiting operator')

    def on_confirm(self, msg):
        self.t_confirm = time.time()
        self.log('Operator ' + ('CONFIRMED' if msg.data else 'REJECTED') + ' the detection')

    def on_casualty(self, msg):
        self.detected = (msg.point.x, msg.point.y)
        self.log(f'Casualty sent to mission control at {fmt(self.detected)}')

    def on_route(self, msg):
        self.route = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.route_count += 1
        self.t_route = self.t_route or time.time()
        if self.route:
            length = sum(math.dist(a, b) for a, b in zip(self.route, self.route[1:]))
            self.log(f'A* route #{self.route_count}: {len(self.route)} waypoints, '
                     f'{length:.1f} m between waypoints, goal {fmt(self.route[-1])}')

    def on_state(self, msg):
        if msg.data == self.state:
            return
        self.state = msg.data
        self.log(f'Mission state: {msg.data}')
        if self.t_end is not None:
            self.states_after_end.append(msg.data)
        if msg.data in ('DONE', 'FAILED') and self.t_end is None:
            self.t_end = time.time()
            if msg.data == 'DONE' and self.hold_check_s > 0:
                self.hold_timer = self.create_timer(2.0, self.watch_hold)
            else:
                self.create_timer(3.0, self.report_once)

    def watch_hold(self):
        """After DONE: does the Husky stay where it stopped?"""
        if self.reported:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        pos = gazebo_positions(self.world, [self.husky_name], timeout=10.0).get(self.husky_name)
        if pos is None:
            return
        if self.arrival is None:
            self.arrival, self.hold_start = pos, now
            self.log(f'Husky at arrival (Gazebo): {fmt(pos)}; watching it for {self.hold_check_s:.0f} s '
                     '(sim) to check it stays there')
            return
        drift = math.dist(pos[:2], self.arrival[:2])
        if drift > self.hold_max_drift + 0.05:
            self.log(f'Husky has moved {drift:.2f} m since arrival, now at {fmt(pos)}')
        self.hold_max_drift = max(self.hold_max_drift, drift)
        if now - self.hold_start >= self.hold_check_s:
            self.report_once()

    def on_husky_status(self, msg):
        if msg.data != self.husky_status:
            self.husky_status = msg.data
            self.log(f'Husky status: {msg.data}')

    def on_odom(self, msg):
        self.husky_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_estop(self, msg):
        self.estops += 1
        self.log('Husky stop button: ' + ('STOP' if msg.data else 'START'))

    # ---------------- summary ----------------
    def report_once(self):
        if self.reported:
            return
        self.reported = True
        truth = gazebo_poses(self.world, [self.human_name, self.husky_name])
        human_pose = truth.get(self.human_name)
        human = human_pose[0] if human_pose else self.true_human
        husky = truth[self.husky_name][0] if self.husky_name in truth else None
        if husky and self.arrival:
            self.hold_max_drift = max(self.hold_max_drift, math.dist(husky[:2], self.arrival[:2]))
        centre = body_centre(human_pose) if human_pose else None
        husky_src = 'Gazebo ground truth'
        if husky is None and self.husky_odom:
            husky, husky_src = self.husky_odom, '/husky1/odometry'
        to_body = (round(distance_to_polygon(husky[:2], body_footprint(human_pose)), 2)
                   if (husky and human_pose) else None)

        def d(a, b):
            return math.dist(a[:2], b[:2]) if a and b else None

        goal = self.route[-1] if self.route else None
        route_len = (sum(math.dist(a, b) for a, b in zip(self.route, self.route[1:]))
                     if self.route else None)
        row = {
            'time': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'human_pose': self.pose_label,
            'true_human_x': human[0] if human else '', 'true_human_y': human[1] if human else '',
            'detected_x': self.detected[0] if self.detected else (self.candidate[0] if self.candidate else ''),
            'detected_y': self.detected[1] if self.detected else (self.candidate[1] if self.candidate else ''),
            'detection_error_m': d(self.detected or self.candidate, human),
            'body_centre_x': centre[0] if centre else '', 'body_centre_y': centre[1] if centre else '',
            'detection_error_body_m': d(self.detected or self.candidate, centre),
            'planned_goal_x': goal[0] if goal else '', 'planned_goal_y': goal[1] if goal else '',
            'goal_to_detected_m': d(goal, self.detected),
            'route_waypoints': len(self.route), 'routes_planned': self.route_count,
            'husky_arrival_x': self.arrival[0] if self.arrival else '',
            'husky_arrival_y': self.arrival[1] if self.arrival else '',
            'husky_final_x': husky[0] if husky else '', 'husky_final_y': husky[1] if husky else '',
            'hold_checked_s': (self.get_clock().now().nanoseconds * 1e-9 - self.hold_start)
                              if self.hold_start is not None else 0.0,
            'moved_after_arrival_m': self.hold_max_drift if self.arrival else '',
            'stayed_done': ('yes' if self.state == 'DONE' and not
                            [x for x in self.states_after_end if x != 'DONE'] else 'no')
                           if self.arrival else '',
            'stop_dist_to_body_m': to_body,
            'stop_dist_to_feet_origin_m': d(husky, human),
            'stop_dist_to_detected_m': d(husky, self.detected),
            'nav_result': self.husky_status or '', 'mission_state': self.state or '',
            'search_s': (self.t_detect - self.t0) if self.t_detect else '',
            'confirm_to_end_s': (self.t_end - self.t_confirm) if (self.t_end and self.t_confirm) else '',
            'husky_estops': self.estops,
        }
        row = {k: (round(v, 2) if isinstance(v, float) else v) for k, v in row.items()}

        lines = [
            '',
            '================ TRACE TEST RESULT ================',
            f' human_pose            : {self.pose_label or "?"}',
            f' Person (ground truth) : model origin (feet) {fmt(human)}',
            f' Person body centre    : {fmt(centre)}   (feet + 0.9 m along the body)',
            f' Detected casualty     : {fmt(self.detected or self.candidate)}   error '
            f'{row["detection_error_body_m"]} m to the body centre '
            f'({row["detection_error_m"]} m to the feet)',
            f' Planned goal (A*)     : {fmt(goal)}   {len(self.route)} waypoints, '
            f'{self.route_count} route(s) planned'
            + (f', {route_len:.1f} m' if route_len else ''),
            f' Husky at arrival      : {fmt(self.arrival)}   (Gazebo, just after DONE)',
            f' Husky final           : {fmt(husky)}   ({husky_src}, after '
            f'{row["hold_checked_s"]} s sim)   moved {row["moved_after_arrival_m"]} m after arrival, '
            f'mission stayed DONE: {row["stayed_done"]}',
            f' Stopping distance     : {row["stop_dist_to_body_m"]} m to the person\'s body, '
            f'{row["stop_dist_to_detected_m"]} m to the detected point, '
            f'{row["stop_dist_to_feet_origin_m"]} m to the model origin (feet)',
            '                         (Husky centre; body = collision box of human/model.sdf)',
            f' Navigation result     : {self.husky_status}   mission state {self.state}',
            f' Search time           : {row["search_s"]} s (wall clock)   '
            f'confirm -> end: {row["confirm_to_end_s"]} s',
            '===================================================',
        ]
        for line in lines:
            self.get_logger().info(line)

        try:
            os.makedirs(self.results_dir, exist_ok=True)
            path = os.path.join(self.results_dir, 'results.csv')
            new = not os.path.exists(path)
            if not new:
                # An older logger version may have written other columns:
                # start a new file rather than append misaligned rows.
                with open(path, newline='') as fh:
                    old_header = next(csv.reader(fh), [])
                if old_header != list(row.keys()):
                    os.replace(path, path.replace('.csv', datetime.now().strftime('_old_%Y%m%d_%H%M%S.csv')))
                    new = True
            with open(path, 'a', newline='') as fh:
                writer = csv.DictWriter(fh, fieldnames=list(row.keys()))
                if new:
                    writer.writeheader()
                writer.writerow(row)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            with open(os.path.join(self.results_dir, f'{stamp}_pose{self.pose_label}.log'), 'w') as fh:
                fh.write('\n'.join(self.events + lines) + '\n')
            self.get_logger().info(f'Saved to {path}')
        except OSError as exc:
            self.get_logger().error(f'Could not save results: {exc}')


def main():
    rclpy.init()
    node = MissionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.reported and node.state is not None:
            try:
                node.report_once()
            except Exception:  # shutting down; best effort
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
