#!/usr/bin/env python3
"""Ground-truth check: which casualty spawn points can a ground robot reach?

Rasterises the real terrain mesh (models/terrain_1/media/terrain_1.dae, scaled as
in the world) at the TerrainMap resolution, applies the same slope rule as
terrain_map.cpp (steepest of 4 neighbours <= 25 deg), and flood-fills from the
Husky start. Trees are ignored (they only make things harder).

    python3 tools/terrain_reachability.py <path to terrain_1.dae>

Result on 27 Oct 2026: poses 0, 1, 3, 4, 5 reachable; pose 2 (-15, 15) in a
basin enclosed by >25 deg slopes.

CORRECTION, 28 Oct: too strict. The "wall" on the way to pose 2, around
(-6.3, 8.7)-(-8, 10), is a crease only 0.1-0.3 m high that reads as 26-34 deg over
one 0.5 m cell. In Gazebo (v5) the Husky drove over it and reached pose 2. Treat
this tool's result as a conservative estimate, not ground truth.
"""
import math, sys, xml.etree.ElementTree as ET
from collections import deque
import numpy as np
from scipy.interpolate import griddata

SCALE, RES, HALF, MAX_SLOPE = 22.45916, 0.5, 40, 25.0
POSES = [(-15, -10), (-15, 0), (-15, 15), (12, 15), (12, -5), (2.3978, 12.5679)]
HUSKY = (-1.4, 0.0)

ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
t = ET.parse(sys.argv[1]); g = t.find('.//c:geometry', ns)
sid = g.find('.//c:vertices/c:input[@semantic="POSITION"]', ns).get('source')[1:]
v = np.array(g.find('.//c:source[@id="%s"]/c:float_array' % sid, ns).text.split(), float).reshape(-1, 3) * SCALE
xs = (np.arange(2 * HALF + 1) - HALF) * RES
GX, GY = np.meshgrid(xs, xs)
Z = griddata(v[:, :2], v[:, 2], (GX, GY), method='linear')
n = 2 * HALF + 1
S = np.full_like(Z, np.nan)
for iy in range(1, n - 1):
    for ix in range(1, n - 1):
        S[iy, ix] = max(math.degrees(math.atan2(abs(Z[iy + dy, ix + dx] - Z[iy, ix]), RES))
                        for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)))
ok = S <= MAX_SLOPE
cell = lambda x, y: (int(round(y / RES + HALF)), int(round(x / RES + HALF)))
seen = np.zeros_like(ok); s = cell(*HUSKY); seen[s] = True; q = deque([s])
while q:
    iy, ix = q.popleft()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            y, x = iy + dy, ix + dx
            if 0 <= y < n and 0 <= x < n and ok[y, x] and not seen[y, x]:
                seen[y, x] = True; q.append((y, x))
for i, (x, y) in enumerate(POSES):
    cy, cx = cell(x, y)
    ring = [(cy + dy, cx + dx) for dy in range(-5, 6) for dx in range(-5, 6) if 2.4 <= math.hypot(dx, dy) <= 5]
    print('pose %d (%6.1f, %6.1f): stand-off cells drivable %2d, reachable from Husky %2d -> %s'
          % (i, x, y, sum(ok[c] for c in ring), sum(seen[c] for c in ring),
             'REACHABLE' if any(seen[c] for c in ring) else 'NOT reachable by ground'))
