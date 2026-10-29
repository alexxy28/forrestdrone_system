#!/usr/bin/env python3
# Calibration tool used on 27 Oct 2026. With Gazebo running terrain_demo, a Parrot
# spawned and its bridge running, it teleports the drone around a target and
# measures the hot blob there exactly as ThermalCamera.cpp does.
#   python3 tools/thermal_shape_sweep.py <target_x> <target_y> <fly_z>
# Results 27 Oct: human (12, 15.8)  N=36  size median 0.97 m, elongation 2.47x
#                 demo_animal (-4, 4) N=39 size median 0.56 m, elongation 1.21x
"""Fly the Parrot (teleport) around a target at many ranges/bearings and measure
the hot blob nearest the target with the same gates and maths as ThermalCamera.cpp."""
import sys, time, math, subprocess, numpy as np, rclpy, cv2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
rclpy.init(); n = Node('sweep'); got = {}
n.create_subscription(Image, '/parrot1/thermal_camera', lambda m: got.__setitem__('th', m), qos_profile_sensor_data)
n.create_subscription(Image, '/parrot1/camera/depth/image', lambda m: got.__setitem__('dp', m), qos_profile_sensor_data)
n.create_subscription(Odometry, '/parrot1/odometry', lambda m: got.__setitem__('od', m), 10)
def setpose(x, y, z, yaw):
    req = f'name: "parrot1" position {{x: {x} y: {y} z: {z}}} orientation {{z: {math.sin(yaw/2)} w: {math.cos(yaw/2)}}}'
    subprocess.run(['ign', 'service', '-s', '/world/terrain_demo/set_pose', '--reqtype', 'ignition.msgs.Pose',
                    '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', req], capture_output=True)
def grab():
    got.clear(); t0 = time.time()
    while time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.1)
        if len(got) == 3:
            ts = lambda k: got[k].header.stamp.sec + got[k].header.stamp.nanosec * 1e-9
            if abs(ts('th') - ts('dp')) < 0.15: return True
            got.pop('th')
    return False
def measure(tx, ty):
    th, dp, od = got['th'], got['dp'], got['od']
    T = np.frombuffer(th.data, np.uint16).reshape(480, 720) * 0.01 - 273.15
    D = np.frombuffer(dp.data, np.float32).reshape(480, 720)
    k = np.ones((3, 3), np.uint8); m = ((T >= 30) * 255).astype(np.uint8)
    m = cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)
    nl, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    f = 720 / (2 * math.tan(math.radians(60))); cx, cy = 359.5, 239.5; p = math.radians(45)
    q = od.pose.pose.orientation; pp = od.pose.pose.position; yaw = 2 * math.atan2(q.z, q.w)
    best = None
    for c in range(1, nl):
        l, t, w, h, area = st[c]
        if l <= 0 or t <= 0 or l + w >= 720 or t + h >= 480: continue
        ys, xs = np.nonzero(lab == c); d = D[ys, xs]; ok = np.isfinite(d) & (d > 0.1) & (d < 39)
        if ok.sum() < 60 or ok.sum() / area < 0.25: continue
        d, xs2, ys2 = d[ok], xs[ok], ys[ok]
        X, Y, Z = d, -(xs2 - cx) * d / f, -(ys2 - cy) * d / f
        bx = math.cos(p) * X + math.sin(p) * Z + .25; bz = -math.sin(p) * X + math.cos(p) * Z + .25
        wx = pp.x + math.cos(yaw) * bx - math.sin(yaw) * Y; wy = pp.y + math.sin(yaw) * bx + math.cos(yaw) * Y; wz = pp.z + bz
        mx, my = np.median(wx), np.median(wy)
        if math.hypot(mx - tx, my - ty) > 2.5: continue
        sxx = np.sum((wx - mx) ** 2); syy = np.sum((wy - my) ** 2); sxy = np.sum((wx - mx) * (wy - my))
        ax = 0.5 * math.atan2(2 * sxy, sxx - syy)
        along = (wx - mx) * math.cos(ax) + (wy - my) * math.sin(ax); across = -(wx - mx) * math.sin(ax) + (wy - my) * math.cos(ax)
        sp = lambda u: np.percentile(u, 95) - np.percentile(u, 5)
        size = max(sp(along), sp(wz)); width = max(sp(across), 0.05)
        camz = pp.z + .25; elev = math.degrees(math.atan2(camz - np.median(wz), math.hypot(mx - pp.x, my - pp.y)))
        if elev < 20: continue
        best = (size, size / width, elev, int(ok.sum()))
    return best
tx, ty, zfly = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
rows = []
for r in [3, 5, 7, 9, 11]:
    for b in range(0, 360, 45):
        a = math.radians(b); x, y = tx - r * math.cos(a), ty - r * math.sin(a)
        setpose(x, y, zfly, a); time.sleep(1.2)
        if not grab(): continue
        mres = measure(tx, ty)
        if mres: rows.append((r, b) + mres)
for row in rows: print('range %2d bearing %3d  size %.2f  elong %.2f  elev %.0f  px %d' % row)
if rows:
    S = np.array([x[2] for x in rows]); E = np.array([x[3] for x in rows])
    print('N=%d  size median %.2f (p10 %.2f p90 %.2f)  elong median %.2f (p10 %.2f p90 %.2f)' % (len(rows), np.median(S), np.percentile(S, 10), np.percentile(S, 90), np.median(E), np.percentile(E, 10), np.percentile(E, 90)))
