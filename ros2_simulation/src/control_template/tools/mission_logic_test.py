#!/usr/bin/env python3
"""Offline test of the TRACE mission chain, no Gazebo needed.

Stands in for the simulator and Nav2 so that the real mission nodes can be
tested on any machine:

    terrain_map_node -> system_control_node -> husky_node -> (fake) Nav2

What it fakes:
  * a world: flat ground, a 5 m hill between the Husky and the casualty,
    a few trees (crown returns mixed with ground returns), the parked Husky
    body and the casualty lying on the ground;
  * the Parrot flying a lawnmower search: raw odometry, the down-lidar cloud
    in its own frame, and the base -> lidar static transform;
  * SLAM drift: husky1_map is offset from the world by (0.4, -0.3) m and 4 deg;
    odometry drift: husky1_odom is offset by (-0.5, 0.6) m and -3 deg;
  * Nav2's NavigateToPose server for the Husky, which drives the Husky toward
    each goal (optionally aborting one goal, to test the skip logic).

Run (after sourcing the workspace), with the four nodes NOT already running:

    ros2 run control_template system_control_node &
    ros2 run control_template terrain_map_node &
    ros2 run control_template husky_node &
    python3 tools/mission_logic_test.py --scenario nominal

Scenarios: nominal | estop | nav_abort | unreachable
Exit code 0 = pass. Prints a result table for the test log.
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

HILL = (-6.0, -4.0, 5.0, 2.0)          # x, y, peak height, sigma (m)
TREES = [(-10.0, -6.5), (-3.0, -8.0), (-12.0, -2.0), (4.0, 3.0)]
TREE_R = 1.2
HUSKY_START = (0.0, 0.0)
CASUALTY = (-15.0, -10.0)
MAP_OFFSET = (0.4, -0.3, math.radians(4.0))   # world -> husky1_map (fake SLAM drift)
ODOM_OFFSET = (-0.5, 0.6, math.radians(-3.0))  # world -> husky1_odom (fake odometry drift)
AGL = 7.0


def ground(x, y):
    hx, hy, h, s = HILL
    return h * math.exp(-((x - hx) ** 2 + (y - hy) ** 2) / (2 * s * s))


def slope_deg(x, y, d=0.25):
    gx = (ground(x + d, y) - ground(x - d, y)) / (2 * d)
    gy = (ground(x, y + d) - ground(x, y - d)) / (2 * d)
    return math.degrees(math.atan(math.hypot(gx, gy)))


def in_box(x, y, cx, cy, lx, ly):
    return abs(x - cx) <= lx / 2 and abs(y - cy) <= ly / 2


def world_to(offset, x, y):
    tx, ty, th = offset
    return (math.cos(th) * x - math.sin(th) * y + tx, math.sin(th) * x + math.cos(th) * y + ty)


def to_world(offset, x, y):
    tx, ty, th = offset
    x, y = x - tx, y - ty
    return (math.cos(th) * x + math.sin(th) * y, -math.sin(th) * x + math.cos(th) * y)


def world_to_map(x, y):
    return world_to(MAP_OFFSET, x, y)


def map_to_world(x, y):
    return to_world(MAP_OFFSET, x, y)


class FakeWorld(Node):
    def __init__(self, scenario):
        super().__init__('fake_world')
        self.scenario = scenario
        self.rng = np.random.default_rng(1)
        cb = ReentrantCallbackGroup()

        self.parrot_odom = self.create_publisher(Odometry, '/parrot1/odometry', 10)
        self.parrot_tf_static = self.create_publisher(TFMessage, '/parrot1/tf_static', LATCHED)
        self.parrot_cloud = self.create_publisher(PointCloud2, '/parrot1/scan/points', 10)
        self.husky_odom = self.create_publisher(Odometry, '/husky1/odometry', 10)
        self.husky_tf = self.create_publisher(TFMessage, '/husky1/tf', 10)
        self.casualty_pub = self.create_publisher(PointStamped, '/mission/casualty', LATCHED)
        self.estop_pub = self.create_publisher(Bool, '/mission_control/husky1/estop', 10)

        self.state = 'unknown'
        self.husky_status = 'unknown'
        self.states_seen = []
        self.route = []
        self.create_subscription(String, '/mission/state', self._state, LATCHED)
        self.create_subscription(String, '/mission/husky_status', self._status, LATCHED)
        self.create_subscription(Path, '/mission/route', self._route, LATCHED)

        self.husky = list(HUSKY_START)   # world x, y
        self.husky_yaw = 0.0
        self.trace = [tuple(self.husky)]
        self.goals = 0
        self.paused = False
        self.nav = ActionServer(self, NavigateToPose, '/husky1/navigate_to_pose',
                                execute_callback=self._drive, goal_callback=lambda g: GoalResponse.ACCEPT,
                                cancel_callback=lambda g: CancelResponse.ACCEPT,
                                handle_accepted_callback=self._accepted, callback_group=cb)
        self.accepted = 0
        self.create_timer(0.05, self._publish_husky, callback_group=cb)

    # ---- mission topics ----
    def _state(self, m):
        if m.data != self.state:
            self.get_logger().info(f'mission state -> {m.data}')
            self.states_seen.append(m.data)
        self.state = m.data

    def _status(self, m):
        if m.data != self.husky_status:
            self.get_logger().info(f'husky status -> {m.data}')
        self.husky_status = m.data

    def _route(self, m):
        self.route = [(p.pose.position.x, p.pose.position.y) for p in m.poses]

    # ---- Husky: odometry (world) + TF (husky1_map -> odom -> base) ----
    def _publish_husky(self):
        now = self.get_clock().now().to_msg()
        o = Odometry()
        o.header.stamp = now
        o.header.frame_id = 'husky1_odom'
        o.child_frame_id = 'husky1_base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.husky
        o.pose.pose.orientation.z = math.sin(self.husky_yaw / 2)
        o.pose.pose.orientation.w = math.cos(self.husky_yaw / 2)
        self.husky_odom.publish(o)

        # odom -> base: the Husky's pose in the drifted odometry frame.
        ox, oy = world_to(ODOM_OFFSET, *self.husky)
        oyaw = self.husky_yaw + ODOM_OFFSET[2]
        # map -> odom chosen so that map -> base is the world pose in the SLAM frame.
        th = MAP_OFFSET[2] - ODOM_OFFSET[2]
        mx, my = world_to(MAP_OFFSET, *self.husky)
        tx = mx - (math.cos(th) * ox - math.sin(th) * oy)
        ty = my - (math.sin(th) * ox + math.cos(th) * oy)
        m2o = TransformStamped()
        m2o.header.stamp = now
        m2o.header.frame_id = 'husky1_map'
        m2o.child_frame_id = 'husky1_odom'
        m2o.transform.translation.x, m2o.transform.translation.y = tx, ty
        m2o.transform.rotation.z, m2o.transform.rotation.w = math.sin(th / 2), math.cos(th / 2)
        o2b = TransformStamped()
        o2b.header.stamp = now
        o2b.header.frame_id = 'husky1_odom'
        o2b.child_frame_id = 'husky1_base_link'
        o2b.transform.translation.x, o2b.transform.translation.y = ox, oy
        o2b.transform.rotation.z, o2b.transform.rotation.w = math.sin(oyaw / 2), math.cos(oyaw / 2)
        self.husky_tf.publish(TFMessage(transforms=[m2o, o2b]))

    # lost_goal: accept the 3rd goal but never execute it (seen in Gazebo,
    # 28 Oct: a goal sent right after the previous one finished never started).
    def _accepted(self, goal_handle):
        self.accepted += 1
        if self.scenario == 'lost_goal' and self.accepted == 3:
            self.get_logger().warn('fake Nav2: accepted goal 3 but never executing it (on purpose)')
            return
        goal_handle.execute()

    # ---- fake Nav2: drive toward the goal at 2 m/s until within 0.8 m ----
    def _drive(self, handle):
        self.goals += 1
        goal = handle.request.pose
        offset = ODOM_OFFSET if goal.header.frame_id == 'husky1_odom' else MAP_OFFSET
        gx, gy = to_world(offset, goal.pose.position.x, goal.pose.position.y)
        if self.scenario == 'nav_abort' and self.goals == 2:
            self.get_logger().warn('fake Nav2: aborting goal 2 on purpose')
            handle.abort()
            return NavigateToPose.Result()
        while True:
            if handle.is_cancel_requested:
                handle.canceled()
                return NavigateToPose.Result()
            dx, dy = gx - self.husky[0], gy - self.husky[1]
            d = math.hypot(dx, dy)
            if d < 0.8:
                handle.succeed()
                return NavigateToPose.Result()
            step = min(0.1, d)
            self.husky[0] += step * dx / d
            self.husky[1] += step * dy / d
            self.husky_yaw = math.atan2(dy, dx)
            self.trace.append(tuple(self.husky))
            time.sleep(0.05)

    # ---- Parrot: fly a lawnmower and publish the down-lidar cloud ----
    def surface(self, x, y, parked_husky):
        z = ground(x, y)
        for tx, ty in TREES:
            if math.hypot(x - tx, y - ty) < TREE_R and self.rng.random() < 0.6:
                return z + self.rng.uniform(3.0, 5.0)      # crown return
        if parked_husky and in_box(x, y, HUSKY_START[0], HUSKY_START[1], 0.99, 0.67):
            return z + 0.45
        if in_box(x, y, CASUALTY[0], CASUALTY[1], 1.77, 0.6):
            return z + 0.35
        return z + self.rng.normal(0, 0.01)

    def fly_search(self):
        st = TransformStamped()
        st.header.frame_id = 'parrot1_base_link'
        st.child_frame_id = 'parrot1_base_down_scan'
        st.transform.translation.z = -0.12
        st.transform.rotation.w = 1.0
        self.parrot_tf_static.publish(TFMessage(transforms=[st]))

        elevs = np.radians(np.linspace(-90, -67, 5))
        azs = np.radians(np.arange(-180, 180, 3))
        lanes = np.arange(-12.0, 16.01, 4.0)
        for i, y in enumerate(lanes):
            xs = np.arange(-17.0, 13.01, 0.5)
            for x in (xs if i % 2 == 0 else xs[::-1]):
                z = ground(x, y) + AGL
                now = self.get_clock().now().to_msg()
                o = Odometry()
                o.header.stamp = now
                o.header.frame_id = 'parrot1_odom'
                o.child_frame_id = 'parrot1_base_link'
                o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, z
                o.pose.pose.orientation.w = 1.0
                self.parrot_odom.publish(o)
                sz = z - 0.12
                pts = []
                for e in elevs:
                    d = (AGL - 0.12) / math.tan(-e) if e > -math.pi / 2 + 1e-6 else 0.0
                    for a in azs:
                        px, py = x + d * math.cos(a), y + d * math.sin(a)
                        pts.append((px - x, py - y, self.surface(px, py, True) - sz))
                c = point_cloud2.create_cloud_xyz32(o.header, pts)
                c.header.frame_id = 'parrot1_base_down_scan'
                self.parrot_cloud.publish(c)
                time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', default='nominal',
                    choices=['nominal', 'estop', 'nav_abort', 'unreachable', 'drift', 'lost_goal'])
    ap.add_argument('--timeout', type=float, default=120.0)
    args = ap.parse_args()

    global CASUALTY
    if args.scenario == 'unreachable':
        CASUALTY = (HILL[0], HILL[1])      # on top of the 5 m hill

    rclpy.init()
    w = FakeWorld(args.scenario)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(w)
    import threading
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(2.0)

    t0 = time.time()
    w.fly_search()
    w.get_logger().info(f'search flight done in {time.time() - t0:.1f} s; publishing casualty {CASUALTY}')
    time.sleep(1.0)
    p = PointStamped()
    p.header.frame_id = 'parrot1_odom'
    p.point.x, p.point.y = CASUALTY
    w.casualty_pub.publish(p)

    estop_done = False
    stopped_ok = None
    deadline = time.time() + args.timeout
    while time.time() < deadline and w.state not in ('DONE', 'FAILED'):
        if args.scenario == 'estop' and not estop_done and w.goals >= 2:
            w.estop_pub.publish(Bool(data=True))
            time.sleep(1.0)
            before = tuple(w.husky)
            time.sleep(2.0)
            moved = math.hypot(w.husky[0] - before[0], w.husky[1] - before[1])
            stopped_ok = (w.husky_status == 'STOPPED' and moved < 0.05)
            w.get_logger().info(f'estop: status={w.husky_status}, moved {moved:.2f} m while stopped')
            w.estop_pub.publish(Bool(data=False))
            estop_done = True
        time.sleep(0.2)

    # drift: after DONE, roll the Husky 1 m back (like the slope creep seen in
    # Gazebo, 28 Oct). The mission must leave DONE, the Husky drive back, and
    # DONE come back only once it is stopped in range again.
    drift_ok = None
    if args.scenario == 'drift' and w.state == 'DONE':
        stop_point = tuple(w.husky)
        ux, uy = w.husky[0] - CASUALTY[0], w.husky[1] - CASUALTY[1]
        n = math.hypot(ux, uy)
        n_before = len(w.states_seen)
        for _ in range(20):                       # 1.0 m at 0.1 m/s
            w.husky[0] += 0.05 * ux / n
            w.husky[1] += 0.05 * uy / n
            time.sleep(0.5)
        left_done = 'HUSKY_EN_ROUTE' in w.states_seen[n_before:]
        w.get_logger().info(f'drift: rolled 1.0 m from {stop_point}; mission left DONE: {left_done}')
        deadline = time.time() + 60
        while time.time() < deadline and not (w.state == 'DONE' and w.husky_status == 'ARRIVED'
                                              and len(w.states_seen) > n_before + 1):
            time.sleep(0.2)
        back = math.hypot(w.husky[0] - CASUALTY[0], w.husky[1] - CASUALTY[1])
        drift_ok = left_done and w.state == 'DONE' and back <= 2.8
        print(f'drift: left DONE={left_done}, back to {w.state}/{w.husky_status}, {back:.2f} m from casualty')

    final = tuple(w.husky)
    dist = math.hypot(final[0] - CASUALTY[0], final[1] - CASUALTY[1])
    trace = w.trace
    length = sum(math.dist(trace[i], trace[i + 1]) for i in range(len(trace) - 1))
    max_slope = max(slope_deg(x, y) for x, y in trace)
    min_tree = min(math.hypot(x - tx, y - ty) for x, y in trace for tx, ty in TREES)
    straight = math.dist(HUSKY_START, CASUALTY)
    straight_max_slope = max(slope_deg(HUSKY_START[0] + f * (CASUALTY[0] - HUSKY_START[0]),
                                       HUSKY_START[1] + f * (CASUALTY[1] - HUSKY_START[1]))
                             for f in np.linspace(0, 1, 200))

    print('\n===== RESULT: scenario', args.scenario, '=====')
    print(f'mission state          : {w.state}')
    print(f'husky status           : {w.husky_status}')
    print(f'route waypoints        : {len(w.route)}')
    print(f'Nav2 goals sent        : {w.goals}')
    print(f'final dist to casualty : {dist:.2f} m')
    print(f'driven length          : {length:.1f} m (straight line {straight:.1f} m)')
    print(f'max slope on driven path: {max_slope:.1f} deg (straight line would hit {straight_max_slope:.1f} deg)')
    print(f'closest pass to a tree : {min_tree:.2f} m from trunk centre')
    if stopped_ok is not None:
        print(f'estop held the Husky   : {stopped_ok}')

    if args.scenario == 'unreachable':
        ok = w.state in ('PLANNING', 'FAILED') and w.goals == 0
    else:
        ok = (w.state == 'DONE' and dist < 3.0 and max_slope <= 25.0 and min_tree > TREE_R
              and (stopped_ok in (None, True)) and (drift_ok in (None, True)))
    print('PASS' if ok else 'FAIL')
    ex.shutdown()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
