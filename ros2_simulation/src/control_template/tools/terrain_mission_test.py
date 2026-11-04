#!/usr/bin/env python3
"""Offline mission test on the real terrain_demo ground, no Gazebo needed.

Like mission_logic_test.py, but the fake world is the real terrain (the
terrain_1 mesh, as in terrain_demo) and the casualty poses are the ones Gazebo
uses, including pose 6 on top of the rock. For each pose it starts the real
mission nodes (control.launch.py: terrain_map, system_control, husky), runs
one mission and scores it:

    poses 0-5: the Husky must reach the casualty (DONE, stop <= 3 m)
    pose 6:    unreachable: FAILED, the Husky never sent anywhere, the drone
               sent to keep watch over the casualty

What it fakes (one drone): the Parrot flies whatever path mission control
sends at --speed m/s, reports IDLE/SEARCHING/HOLDING, publishes its down-LiDAR
cloud sampled from the real heights (tree crowns at terrain_demo's trees,
the parked Husky and the casualty's body), and reports the casualty once it
passes within --detect m (the real thermal camera sees ~19 m). Nav2 and the
Husky come from mission_logic_test.FakeWorld.

Run (after sourcing the workspace, with no mission nodes running):

    python3 tools/terrain_mission_test.py            # all poses, 0-6
    python3 tools/terrain_mission_test.py --pose 6   # just the rock

Exit code 0 = every pose passed. Prints a result table for the test log.
"""

import argparse
import ast
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mission_logic_test as mlt  # noqa: E402

SHARE = get_package_share_directory('41068_ignition_bringup')
MESH = os.path.join(SHARE, 'models', 'terrain_1', 'media', 'terrain_1.dae')
SCALE = 22.45916            # terrain_1 mesh node scale
RES, HALF = 0.1, 20.0       # height grid (m)
AGL = 7.0
DRONE_START = (2.0, 0.0)    # parrot1 spawn


def casualty_poses():
    """Feet poses from 41068_ignition.launch.py (one list for Gazebo and this test)."""
    tree = ast.parse(open(os.path.join(SHARE, 'launch', '41068_ignition.launch.py')).read())
    lists = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and node.targets[0].id in (
                '_TERRAIN_DEMO_HUMAN_POSES', '_TERRAIN_DEMO_TEST_POSES'):
            lists[node.targets[0].id] = ast.literal_eval(node.value)
    poses = lists['_TERRAIN_DEMO_HUMAN_POSES'] + lists['_TERRAIN_DEMO_TEST_POSES']
    return [tuple(float(v) for v in p.split()) for p in poses]


def body_centre(pose):
    """The person lies along the direction the feet->head axis is rolled to (yaw 0: +y)."""
    x, y, _, roll, _, yaw = pose
    along = 0.9 * abs(math.sin(roll))
    return (x - along * math.sin(yaw), y + along * math.cos(yaw))


def height_grid():
    """Highest surface of the terrain mesh on a RES grid (cached next to /tmp)."""
    cache = f'/tmp/trace_terrain_grid_{int(os.path.getmtime(MESH))}.npy'
    if os.path.exists(cache):
        return np.load(cache)
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    g = ET.parse(MESH).find('.//c:geometry', ns)
    sid = g.find('.//c:vertices/c:input[@semantic="POSITION"]', ns).get('source')[1:]
    v = np.array(g.find('.//c:source[@id="%s"]/c:float_array' % sid, ns).text.split(),
                 float).reshape(-1, 3) * SCALE
    tris = []
    for prim in g.findall('.//c:triangles', ns) + g.findall('.//c:polylist', ns):
        inputs = prim.findall('c:input', ns)
        stride = max(int(i.get('offset')) for i in inputs) + 1
        off = int([i for i in inputs if i.get('semantic') == 'VERTEX'][0].get('offset'))
        idx = np.array(prim.find('c:p', ns).text.split(), int).reshape(-1, stride)[:, off]
        counts = (np.array(prim.find('c:vcount', ns).text.split(), int)
                  if prim.find('c:vcount', ns) is not None else np.full(len(idx) // 3, 3))
        k = 0
        for n in counts:
            for j in range(1, n - 1):
                tris.append((idx[k], idx[k + j], idx[k + j + 1]))
            k += n
    xs = np.arange(-HALF, HALF + RES / 2, RES)
    z = np.full((len(xs), len(xs)), np.nan)
    for a, b, c in v[np.array(tris)]:
        lo, hi = np.minimum(np.minimum(a, b), c), np.maximum(np.maximum(a, b), c)
        i0, i1 = max(0, int(np.ceil((lo[0] + HALF) / RES))), min(len(xs) - 1, int((hi[0] + HALF) / RES))
        j0, j1 = max(0, int(np.ceil((lo[1] + HALF) / RES))), min(len(xs) - 1, int((hi[1] + HALF) / RES))
        if i1 < i0 or j1 < j0:
            continue
        gx, gy = np.meshgrid(xs[i0:i1 + 1] - a[0], xs[j0:j1 + 1] - a[1])
        e0, e1 = c[:2] - a[:2], b[:2] - a[:2]
        d00, d01, d11 = e0 @ e0, e0 @ e1, e1 @ e1
        den = d00 * d11 - d01 * d01
        if abs(den) < 1e-12:
            continue
        d20, d21 = gx * e0[0] + gy * e0[1], gx * e1[0] + gy * e1[1]
        s, t = (d11 * d20 - d01 * d21) / den, (d00 * d21 - d01 * d20) / den
        inside = (s >= -1e-9) & (t >= -1e-9) & (s + t <= 1 + 1e-9)
        h = a[2] + s * (c[2] - a[2]) + t * (b[2] - a[2])
        cell = z[j0:j1 + 1, i0:i1 + 1]
        top = inside & (np.isnan(cell) | (h > cell))
        cell[top] = h[top]
    z = np.nan_to_num(z)
    np.save(cache, z)
    return z


def trees(ground):
    """terrain_demo's trees that stand on the ground (oak1 is buried inside the rock)."""
    out = []
    text = open(os.path.join(SHARE, 'worlds', 'terrain_demo.sdf')).read()
    for m in re.finditer(r'<model name="(?:oak|pine)\d+">.*?<pose>([^<]+)</pose>', text, re.S):
        x, y, z = (float(p) for p in m.group(1).split()[:3])
        if abs(ground(x, y) - z) < 2.0:
            out.append((x, y))
    return out


class Drone:
    """Flies the latest /parrot1/mission/path and maps the real heights below it."""

    def __init__(self, world, casualty, speed, detect):
        self.w, self.casualty, self.speed, self.detect = world, casualty, speed, detect
        self.x, self.y = DRONE_START
        self.path, self.new_path = [], None
        self.lock = threading.Lock()
        self.running = True
        self.status = None
        self.found_at = None
        self.flown_after = 0.0       # m flown after reporting the casualty
        self.paths_after = []        # paths received after the casualty report
        self.status_pub = world.create_publisher(String, '/parrot1/mission/status', mlt.LATCHED)
        world.create_subscription(Path, '/parrot1/mission/path', self._path, mlt.LATCHED)

    def _path(self, msg):
        with self.lock:
            self.new_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            if self.found_at is not None:
                self.paths_after.append(self.new_path)

    def _status(self, s):
        if s != self.status:
            self.status = s
            self.status_pub.publish(String(data=s))

    def _scan(self):
        z = mlt.ground(self.x, self.y) + AGL
        o = Odometry()
        o.header.stamp = self.w.get_clock().now().to_msg()
        o.header.frame_id, o.child_frame_id = 'parrot1_odom', 'parrot1_base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = self.x, self.y, z
        o.pose.pose.orientation.w = 1.0
        self.w.parrot_odom.publish(o)
        pts = []
        for e in np.radians(np.linspace(-90, -67, 5)):
            d = (AGL - 0.12) / math.tan(-e) if e > -math.pi / 2 + 1e-6 else 0.0
            for a in np.radians(np.arange(-180, 180, 3)):
                px, py = self.x + d * math.cos(a), self.y + d * math.sin(a)
                pts.append((px - self.x, py - self.y, self.w.surface(px, py, True) - (z - 0.12)))
        cloud = point_cloud2.create_cloud_xyz32(o.header, pts)
        cloud.header.frame_id = 'parrot1_base_down_scan'
        self.w.parrot_cloud.publish(cloud)

    def run(self):
        st = TransformStamped()
        st.header.frame_id, st.child_frame_id = 'parrot1_base_link', 'parrot1_base_down_scan'
        st.transform.translation.z = -0.12
        st.transform.rotation.w = 1.0
        self.w.parrot_tf_static.publish(TFMessage(transforms=[st]))
        self._status('IDLE')
        dt = 0.1
        while self.running and rclpy.ok():
            with self.lock:
                if self.new_path is not None:
                    self.path, self.new_path = self.new_path, None
            if self.path:
                self._status('SEARCHING')
                tx, ty = self.path[0]
                d = math.hypot(tx - self.x, ty - self.y)
                step = min(d, self.speed * dt)
                if d > 1e-9:
                    self.x += step * (tx - self.x) / d
                    self.y += step * (ty - self.y) / d
                if d <= self.speed * dt:
                    self.path.pop(0)
                if self.found_at is not None:
                    self.flown_after += step
                if not self.path:
                    self._status('HOLDING')
            self._scan()
            if self.found_at is None and math.dist((self.x, self.y), self.casualty) < self.detect:
                # Like the real drone: report, stop, then wait for a task.
                self.found_at = time.time()
                self.path = []
                p = PointStamped()
                p.header.frame_id = 'parrot1_odom'
                p.point.x, p.point.y = self.casualty
                self.w.casualty_pub.publish(p)
                time.sleep(1.0)
                self._status('HOLDING')
            time.sleep(dt)


def run_pose(n, pose, args):
    casualty = body_centre(pose)
    mlt.CASUALTY = casualty
    nodes = subprocess.Popen(
        ['ros2', 'launch', '41068_ignition_bringup', 'control.launch.py',
         'nodes:=system_control,husky,terrain_map', 'use_sim_time:=false', 'auto_start:=true'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    log = []
    threading.Thread(target=lambda: log.extend(nodes.stdout), daemon=True).start()
    time.sleep(5.0)
    w = mlt.FakeWorld('nominal')
    drone = Drone(w, casualty, args.speed, args.detect)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(w)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(1.0)
    threading.Thread(target=drone.run, daemon=True).start()
    t0 = time.time()
    while time.time() - t0 < args.timeout and w.state not in ('DONE', 'FAILED'):
        time.sleep(0.5)
    t_end = time.time()
    time.sleep(3.0)   # the overwatch path follows the final state
    surveys = sum('survey terrain between Husky and casualty' in line for line in log)
    stop = math.dist(w.husky, casualty)
    watch = drone.paths_after and math.dist(drone.paths_after[-1][-1], casualty) < 8.0
    if pose[2] > 5.0:   # on the rock: must be recognised as unreachable
        expect = 'FAILED, Husky not sent, drone keeps watch'
        ok = w.state == 'FAILED' and w.goals == 0 and bool(watch)
    else:
        expect = 'DONE, stop <= 3 m'
        ok = w.state == 'DONE' and stop <= 3.0
    row = {
        'pose': n, 'casualty': f'({casualty[0]:.1f}, {casualty[1]:.1f}, {pose[2]:.1f})',
        'expected': expect, 'state': w.state, 'nav2_goals': w.goals,
        'husky_stop_m': f'{stop:.2f}', 'surveys': surveys,
        'drone_m_after_found': f'{drone.flown_after:.0f}',
        'found_to_end_s': f'{t_end - drone.found_at:.0f}' if drone.found_at else '-',
        'result': 'PASS' if ok else 'FAIL',
    }
    drone.running = False
    time.sleep(0.3)
    ex.shutdown()
    w.destroy_node()
    os.killpg(nodes.pid, signal.SIGINT)
    try:
        nodes.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(nodes.pid, signal.SIGKILL)
    time.sleep(2.0)
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pose', default='all', help='0-6 or all')
    ap.add_argument('--speed', type=float, default=4.0, help='fake drone speed, m/s (real: 1.5)')
    ap.add_argument('--detect', type=float, default=12.0, help='fake detection range, m')
    ap.add_argument('--timeout', type=float, default=600.0, help='per pose, s')
    args = ap.parse_args()

    grid = height_grid()

    def ground(x, y):
        fx, fy = (x + HALF) / RES, (y + HALF) / RES
        i = int(np.clip(math.floor(fx), 0, grid.shape[1] - 2))
        j = int(np.clip(math.floor(fy), 0, grid.shape[0] - 2))
        tx, ty = min(max(fx - i, 0.0), 1.0), min(max(fy - j, 0.0), 1.0)
        return float((grid[j, i] * (1 - tx) + grid[j, i + 1] * tx) * (1 - ty)
                     + (grid[j + 1, i] * (1 - tx) + grid[j + 1, i + 1] * tx) * ty)

    mlt.ground = ground
    mlt.TREES = trees(ground)
    mlt.HUSKY_START = (0.0, 0.0)
    mlt.MAP_OFFSET = mlt.ODOM_OFFSET = (0.0, 0.0, 0.0)

    poses = casualty_poses()
    todo = range(len(poses)) if args.pose == 'all' else [int(args.pose)]
    rclpy.init()
    rows = []
    for n in todo:
        print(f'--- pose {n}: casualty at {body_centre(poses[n])}', flush=True)
        rows.append(run_pose(n, poses[n], args))
        print(rows[-1], flush=True)
    rclpy.shutdown()

    keys = list(rows[0].keys())
    print('\n===== RESULT: terrain_mission_test (real terrain, one drone at '
          f'{args.speed} m/s, detection {args.detect} m) =====')
    print('| ' + ' | '.join(keys) + ' |')
    print('|' + '---|' * len(keys))
    for r in rows:
        print('| ' + ' | '.join(str(r[k]) for k in keys) + ' |')
    ok = all(r['result'] == 'PASS' for r in rows)
    print('PASS' if ok else 'FAIL')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
