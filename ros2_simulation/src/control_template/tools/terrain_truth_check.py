#!/usr/bin/env python3
"""Score /parrot1/terrain_points against the terrain_demo ground truth.

Ground truth comes from the world files themselves: the terrain_1 mesh gives
the true ground height, terrain_demo.sdf gives the tree positions.

    python3 tools/terrain_truth_check.py             # sim running, after a full search
    python3 tools/terrain_truth_check.py --selftest  # offline check of the scoring itself

Exit code 0 when every check passes.
"""

import re
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

BRINGUP = Path(__file__).resolve().parents[2] / '41068_ignition_bringup'
MESH = BRINGUP / 'models/terrain_1/media/terrain_1.dae'
WORLD = BRINGUP / 'worlds/terrain_demo.sdf'

MESH_SCALE = 22.45916               # <matrix> on the Plane node in terrain_1.dae
TREE_TOP_Z = -0.3 + 1.75 + 0.75     # model pose z + crown centre + crown radius
TREE_RADIUS_M = 1.0                 # cells this close to a trunk count as tree cells
MAX_SIGMA_M = 0.5                   # keep in step with TerrainMap::MAX_SIGMA_M
MAX_SLOPE_DEG = 25.0                # keep in step with TerrainMap::MAX_SLOPE_DEG
MIN_COUNT = 3                       # keep in step with TerrainMap::MIN_COUNT
RESOLUTION = 0.5
AREA = (-17.0, 11.0, -11.0, 11.0)   # system_control_node search area: x0, x1, y0, y1
FIELDS = ('x', 'y', 'z', 'sigma', 'count')


def load_mesh():
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    arr = ET.parse(MESH).getroot().find(
        ".//c:float_array[@id='Plane-mesh-positions-array']", ns)
    return np.array(arr.text.split(), float).reshape(-1, 3) * MESH_SCALE


def load_trees():
    found = re.findall(r'<model name="(?:oak|pine)\d+">\s*<pose>(\S+) (\S+)', WORLD.read_text())
    return np.array(found, float).reshape(-1, 2)


def ground_at(mesh, xy, radius=RESOLUTION):
    """Mean height of mesh vertices within `radius` of each point (NaN if none)."""
    out = np.full(len(xy), np.nan)
    for i in range(0, len(xy), 500):
        near = ((xy[i:i + 500, None, :] - mesh[None, :, :2]) ** 2).sum(-1) < radius ** 2
        count = near.sum(1)
        heights = (near * mesh[None, :, 2]).sum(1)
        out[i:i + 500] = np.where(count > 0, heights / np.maximum(count, 1), np.nan)
    return out


def slopes(cells):
    """Steepest rise to a 4-neighbour in degrees, as TerrainMap::calculateSlopes (NaN if < 2 neighbours)."""
    key = {(round(x / RESOLUTION), round(y / RESOLUTION)): z for x, y, z in cells[:, :3]}
    out = np.full(len(cells), np.nan)
    for i, (x, y, z) in enumerate(cells[:, :3]):
        k = (round(x / RESOLUTION), round(y / RESOLUTION))
        rises = [abs(key[n] - z) for n in ((k[0] + 1, k[1]), (k[0] - 1, k[1]), (k[0], k[1] + 1), (k[0], k[1] - 1))
                 if n in key]
        if len(rises) >= 2:
            out[i] = np.degrees(np.arctan2(max(rises), RESOLUTION))
    return out


def score(cells, mesh, trees):
    """cells: N x 5 array of x, y, z, sigma, count. Prints a report, returns True if all pass."""
    cells = cells[cells[:, 4] >= MIN_COUNT]
    truth = ground_at(mesh, cells[:, :2])

    # Trees sit at absolute z = -0.3, so on high ground they are buried in the hill.
    visible = trees[TREE_TOP_Z - ground_at(mesh, trees, 1.0) > 1.0]
    if len(visible):
        to_tree = np.linalg.norm(cells[:, None, :2] - visible[None], axis=2).min(1)
    else:
        to_tree = np.full(len(cells), np.inf)
    tree = to_tree < TREE_RADIUS_M
    ground = ~tree & np.isfinite(truth)

    x0, x1, y0, y1 = AREA
    area_cells = (round((x1 - x0) / RESOLUTION) + 1) * (round((y1 - y0) / RESOLUTION) + 1)
    in_area = ((cells[:, 0] >= x0) & (cells[:, 0] <= x1) &
               (cells[:, 1] >= y0) & (cells[:, 1] <= y1)).sum()
    coverage = in_area / area_cells

    err = cells[ground, 2] - truth[ground]
    err_median = float(np.median(err)) if err.size else np.nan
    err_p95 = float(np.percentile(np.abs(err), 95)) if err.size else np.nan
    sigma_tree = float(np.median(cells[tree, 3])) if tree.any() else np.nan
    sigma_ground = float(np.median(cells[ground, 3])) if ground.any() else np.nan
    # Seen from above, a crown returns a consistent height (low sigma); its edge is what
    # blocks it, through slope. So a tree cell counts as blocked by either test.
    steep = slopes(cells) > MAX_SLOPE_DEG
    blocked_mask = (cells[:, 3] > MAX_SIGMA_M) | steep
    blocked = float(blocked_mask[tree].mean()) if tree.any() else np.nan
    # False block: passable on the true terrain and not a tree's edge neighbour, yet blocked.
    truth_cells = cells.copy()
    truth_cells[:, 2] = truth
    passable = ground & ~(slopes(truth_cells) > MAX_SLOPE_DEG) & (to_tree >= TREE_RADIUS_M + RESOLUTION)
    open_blocked = float(blocked_mask[passable].mean()) if passable.any() else np.nan

    checks = [
        (f'coverage of search area   {coverage:6.1%}  ({in_area}/{area_cells} cells)',
         coverage > 0.90, '> 90 %'),
        (f'ground error median      {err_median:+6.2f} m  ({ground.sum()} open cells)',
         abs(err_median) <= 0.15, 'within +/-0.15 m'),
        (f'ground error |p95|        {err_p95:6.2f} m', err_p95 < 0.5, '< 0.5 m'),
        (f'sigma median, open        {sigma_ground:6.2f} m', sigma_ground < 0.15, '< 0.15 m'),
        (f'tree cells blocked        {blocked:6.1%}  ({tree.sum()} cells, '
         f'{len(visible)}/{len(trees)} trees above ground; sigma median {sigma_tree:.2f} m)',
         blocked >= 0.80, '>= 80 % (sigma or slope)'),
        (f'passable cells blocked    {open_blocked:6.1%}  ({passable.sum()} truly passable cells)',
         open_blocked <= 0.10, '<= 10 %'),
    ]
    for text, ok, target in checks:
        print(f'{"PASS" if ok else "FAIL"}  {text}   target {target}')

    if err.size:
        worst = np.argsort(-np.abs(err))[:5]
        xy = cells[ground][worst, :2]
        print('worst open cells (x, y, error m):',
              ', '.join(f'({x:.1f}, {y:.1f}, {e:+.2f})' for (x, y), e in zip(xy, err[worst])))
    return all(ok for _, ok, _ in checks)


def grab_cloud(timeout_s=15.0):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    rclpy.init()
    node = rclpy.create_node('terrain_truth_check')
    got = []
    node.create_subscription(PointCloud2, '/parrot1/terrain_points', got.append,
                             qos_profile_sensor_data)
    deadline = time.time() + timeout_s
    while not got and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        sys.exit(f'No /parrot1/terrain_points within {timeout_s:.0f} s. Is terrain_map_node running?')
    pts = point_cloud2.read_points(got[0], field_names=FIELDS)
    return np.column_stack([np.asarray(pts[f], float) for f in FIELDS])


def selftest(mesh, trees):
    """A map equal to the truth must pass; the same map raised by 1 m must fail."""
    rng = np.random.default_rng(0)
    x0, x1, y0, y1 = AREA
    gx, gy = np.meshgrid(np.arange(x0, x1 + 1e-9, RESOLUTION), np.arange(y0, y1 + 1e-9, RESOLUTION))
    xy = np.column_stack([gx.ravel(), gy.ravel()])
    z = ground_at(mesh, xy) + rng.normal(0, 0.02, len(xy))
    sigma = np.full(len(xy), 0.03)
    visible = trees[TREE_TOP_Z - ground_at(mesh, trees, 1.0) > 1.0]
    near_tree = np.linalg.norm(xy[:, None] - visible[None], axis=2).min(1) < TREE_RADIUS_M
    z[near_tree] += 1.0
    sigma[near_tree] = 0.9
    cells = np.column_stack([xy, z, sigma, np.full(len(xy), 10.0)])

    print('-- truth-equal map (should pass)')
    assert score(cells, mesh, trees)
    print('-- map raised 1 m (should fail)')
    cells[:, 2] += 1.0
    assert not score(cells, mesh, trees)
    print('terrain_truth_check selftest: OK')


if __name__ == '__main__':
    mesh, trees = load_mesh(), load_trees()
    if '--selftest' in sys.argv:
        selftest(mesh, trees)
    else:
        sys.exit(0 if score(grab_cloud(), mesh, trees) else 1)
