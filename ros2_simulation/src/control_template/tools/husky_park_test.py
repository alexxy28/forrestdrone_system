#!/usr/bin/env python3
"""Husky park test (Gazebo): does the Husky stay put on a slope with zero command?

Places the Husky on a slope facing downhill, commands zero for 60 s of sim time
and measures the ground-truth drift; then checks commanded driving on flat
ground, turning in place, and driving uphill. Start the simulation without the
mission first:

    ros2 launch 41068_ignition_bringup 41068_ignition.launch.py world:=terrain_demo \
        gui:=false rviz:=false slam:=false nav2:=false parrot:=false
    python3 tools/husky_park_test.py <label> [x y z yaw]

Default spot: (-12.39, -6.54), 8.6 deg, where the Husky rolled away in the v4
test. Steeper spots used on 28 Oct: 12.45 13.95 0.85 1.57 (22.6 deg) and
-14.5 2.5 1.25 -2.11 (24.9 deg).

Results on 28 Oct (60 s sim), wheel joint friction: none rolled 0.637 m at 8.6 deg;
10 N.m rolled 0.648 m at 22.6 deg; 20 N.m (used) 0.000 m at 22.6 and 24.9 deg.
"""
import math, re, subprocess, sys, time, rclpy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
rclpy.init(); n = rclpy.create_node('creep_test')
pub = n.create_publisher(Twist, '/husky1/cmd_vel', 10)
clk = [0.0]
n.create_subscription(Clock, '/clock', lambda m: clk.__setitem__(0, m.clock.sec + m.clock.nanosec * 1e-9), 10)
def spin(t):
    end = time.time() + t
    while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
def sim_wait(dt):
    t0 = clk[0]
    while clk[0] - t0 < dt: rclpy.spin_once(n, timeout_sec=0.05)
def pose():
    out = subprocess.run(['ign','topic','-e','-n','1','-t','/world/terrain_demo/pose/info'], capture_output=True, text=True, timeout=30).stdout
    i = out.find('name: "husky1"'); blk = out[i:i+400]
    g = lambda k: float((re.search(r'\b%s: ([-0-9.e]+)' % k, blk.split('orientation')[0]) or [0,0])[1])
    return g('x'), g('y'), g('z')
def teleport(x, y, z, yaw):
    for attempt in range(5):
        _teleport(x, y, z, yaw); spin(1.0)
        p = pose()
        if math.dist(p[:2], (x, y)) < 0.5:
            return
        print(f'teleport attempt {attempt+1} missed: at {p[0]:.2f},{p[1]:.2f}', flush=True)
    raise SystemExit('teleport failed')
def _teleport(x, y, z, yaw):
    req = 'name: "husky1", position: {x: %f, y: %f, z: %f}, orientation: {z: %f, w: %f}' % (x, y, z, math.sin(yaw/2), math.cos(yaw/2))
    subprocess.run(['ign','service','-s','/world/terrain_demo/set_pose','--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean','--timeout','5000','--req',req], capture_output=True, text=True)
def cmd(vx, w=0.0):
    t = Twist(); t.linear.x = vx; t.angular.z = w
    for _ in range(5): pub.publish(t); spin(0.05)
while clk[0] == 0.0 or pub.get_subscription_count() == 0: spin(0.5)
label = sys.argv[1] if len(sys.argv) > 1 else ''
# 1) creep on the slope (the spot from Priyanka's run, facing the casualty)
PX, PY, PZ, PYAW = (float(v) for v in sys.argv[2:6]) if len(sys.argv) > 5 else (-12.386, -6.54, 0.45, -2.37)
cmd(0.0); teleport(PX, PY, PZ, PYAW); cmd(0.0); sim_wait(5.0)
a = pose(); ta = clk[0]; sim_wait(60.0); b = pose(); tb = clk[0]
d = math.dist(a[:2], b[:2])
print(f'[{label}] PARKED at ({PX},{PY}) zero cmd: moved {d:.3f} m in {tb-ta:.0f} s sim  ({d/(tb-ta):.4f} m/s)  from {a[0]:.3f},{a[1]:.3f} to {b[0]:.3f},{b[1]:.3f}', flush=True)
# 2) commanded drive on flat ground: 0.5 m/s for 10 s sim
teleport(2.0, -3.0, 0.3, 0.0); cmd(0.0); sim_wait(3.0)
a = pose(); t0 = clk[0]
t = Twist(); t.linear.x = 0.5
while clk[0] - t0 < 10.0: pub.publish(t); spin(0.05)
cmd(0.0); b = pose(); sim_wait(2.0); c = pose()
print(f'[{label}] DRIVE 0.5 m/s x 10 s on flat: moved {math.dist(a[:2], b[:2]):.2f} m (ideal 5.0); after stop coasted {math.dist(b[:2], c[:2]):.3f} m', flush=True)
# 3) commanded turn in place 0.5 rad/s for 6 s
a = pose()
t = Twist(); t.angular.z = 0.5; t0 = clk[0]
while clk[0] - t0 < 6.0: pub.publish(t); spin(0.05)
cmd(0.0); b = pose()
print(f'[{label}] TURN in place: centre moved {math.dist(a[:2], b[:2]):.3f} m', flush=True)
# 4) climb: drive up the slope from the parked spot toward the casualty
teleport(-12.0, -6.0, 0.45, -2.37); cmd(0.0); sim_wait(3.0)
a = pose(); t0 = clk[0]; t = Twist(); t.linear.x = 0.5
while clk[0] - t0 < 6.0: pub.publish(t); spin(0.05)
cmd(0.0); b = pose()
print(f'[{label}] DRIVE uphill 0.5 m/s x 6 s: moved {math.dist(a[:2], b[:2]):.2f} m (ideal 3.0), z {a[2]:.2f} -> {b[2]:.2f}', flush=True)
