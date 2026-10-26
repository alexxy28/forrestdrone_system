# Terrain map test results

Results of the staged test of the reworked terrain pipeline:
- `terrain_map_node`, a per-cell running mean and σ;
- one 40 m grid centred on the origin;
- σ and slope blocking;
- a σ cost in the planner.

Ground truth comes from the world files: the `terrain_1` mesh gives true ground heights, and `terrain_demo.sdf` gives the tree positions. The scoring tool is [tools/terrain_truth_check.py](tools/terrain_truth_check.py); run it with `--selftest` to check the scoring itself.

## Run: 16 October 2026

| | |
|---|---|
| Branch / base commit | `terrain-map-update` on `eedddc7`, with uncommitted changes (terrain rework plus the Parrot's new down lidar, obstacle lidar and height-above-ground hold) |
| World | `terrain_demo`, human at (−15, −10, 1) |
| Sim speed | 0.38–0.46× real time (stage 3 search: 161 s sim time in 420 s wall time) |
| Parrot | `parrot_node --ros-args -p heat_threshold_c:=1000.0` (thermal detection off, so the search never pauses for review); default `target_altitude` 5 m above ground |
| `TerrainMap` | 0.5 m cells, 40 × 40 m, `MIN_COUNT` 3, `COUNT_CAP` 50, `MAX_SIGMA_M` 0.5, `MAX_SLOPE_DEG` 25 |
| Planner | `DISTANCE_WEIGHT` 1, `SLOPE_WEIGHT` 4, `SIGMA_WEIGHT` 4 |
| Lidars | Down: `base_down_scan`, 5 rings from −90° to −67°, 0.2–30 m, 3 Hz, noise σ 0.01 m. Obstacle: `base_scan`, 360 beams horizontal, 0.35–8 m, 10 Hz |

### Stage 0: offline self-checks: PASS
- `terrain_map_check`: OK. It covers running mean and variance, the count cap, the grid round trip, the cloud round trip, and blocking of high-σ cells.
- `parrot_scan_check`: all passed. It covers seam-aware clustering, rings, separate posts, and noise or out-of-range returns.
- `terrain_truth_check.py --selftest`: OK.

### Stage 1: wiring: PASS
- **Nodes:** all four mission nodes up.
- **Rate:** `/parrot1/terrain_points` publishes one message per down-lidar scan (19 scans in, 19 messages out), with fields x, y, z, sigma and count.
- **Frames:** the terrain cloud and `/parrot1/odometry` are both in `parrot1_odom`, and odometry reads (2.0, 0.0) at spawn.
- **New lidars:** the down lidar is in frame `parrot1_base_down_scan` and the obstacle lidar in `parrot1_base_scan`.
- **Logs:** `terrain_map_node` has 0 warnings and errors, so the TF lookup for the lidar mount works. The Parrot log has no "tree 0.2 m away" phantom.

### Stage 2: hover: PASS (measured before the lidar rework)
- **Hover at 1.8 m (odometry z):** 17 cells. Every cell within ±0.02 m of the mesh, σ 0.007–0.019 m (the sensor noise).
- **Hover at 11 m (with earlier returns still in the map):** 830 cells.
  - Ground error: median +0.01 m, 95th percentile 0.06 m.
  - σ on open ground: median 0.04 m.
  - The map stayed the same over 30 s, as expected while holding position.
- **Largest open-ground "errors" are real objects:** the parked Husky at (−1.3, 0) reads +0.5 m, and the demo animal at (−4, 4) reads +0.37 m.

### Stage 2 rerun on the new lidars: PASS
The drone held in HOLDING after the stage 3 search at (10.08, 10.84), z 7.02, over ground at 1.49 m, so 5.54 m above ground. Two snapshots were taken 60 s apart (26.6 s of sim time; 0.42× real time). The drone moved 0.000 m.

| Check | Result | Target |
|---|---|---|
| Cells under the drone (radius 2.8 m) | 99, all open ground | — |
| Returns per cell | median 2103, +320 per cell in 60 s | over 50 and climbing |
| Ground error under the drone: median / 95th percentile / worst | +0.021 / 0.088 / +0.113 m | matches terrain |
| σ under the drone: median / max | 0.029 / 0.114 m | under 0.1 m median |
| Cells over 3 m above truth | 0 | 0 |
| Cells outside the footprint that changed | 0 of 3124 (no height changes, no new returns) | 0 |

The map grows only where the lidar is looking. While the drone holds still, the rest of the map is untouched (3245 cells before and after).

### Stage 3: full search, accuracy against ground truth: PASS

| Check | Result | Target |
|---|---|---|
| Coverage of search area | **99.3 %** (2547 / 2565 cells) | > 90 % |
| Ground error, median | **−0.00 m** (3099 open cells) | ±0.15 m |
| Ground error, 95th percentile | **0.09 m** | < 0.5 m |
| σ median, open ground | **0.03 m** | < 0.15 m |
| Tree cells blocked (σ or slope) | **100 %** (122 cells, 10/14 trees above ground; σ median 0.62 m) | ≥ 80 % |
| Truly passable cells blocked | **3.9 %** (2505 cells) | ≤ 10 % |

- **Worst open cells** are all on the 9.55 m hill at about (−5.1, 10.7): (−6.5, 11.0) −2.12 m, (−6.0, 10.5) −2.09 m, (−5.0, 10.0) −1.85 m. These are the steepest cells in the world, where a 0.5 m cell spans metres of height. The map and the truth sample that range at different points: the truth averages mesh vertices within 0.5 m, while the map averages wherever the lidar hit. Both versions are blocked by slope anyway.
- **Trees:** σ was a weak tree signal from directly overhead, because a crown seen from above returns a consistent height. In this run the drone flew lower and passed beside crowns, so tree σ rose to 0.62 m. Slope blocking catches the crown edges either way.
- **Buried trees:** 4 of the 14 trees are underground. The world places every tree at absolute z −0.3, so trees on high ground are buried in the hill.

### Parrot height hold (new height-above-ground logic) during stage 3
Real height above ground along the path is odometry z minus mesh ground, from 206 samples:

| Measure | Result |
|---|---|
| Median | 5.88 m |
| 5th–95th percentile | 5.09 – 7.94 m |
| Min / max | 5.03 / 11.71 m |
| Within ±0.5 m of 5 m | 29 % |
| Below 2 m | 0 samples |

- **5 m acts as a minimum clearance, not an exact height.** The drone never dropped below the target. It holds 5 m above the *highest* return in the down lidar's 23° cone, so it rides higher over trees and near the hill. That's 111 samples over 5.8 m, 47 of them within 2.5 m of a tree.
- **Highest climb:** 11.7 m above ground at (−0.7, 9.0). The hill top at (−5.1, 10.7) sits at the edge of the cone there. It's the expected behaviour for this design and the safe failure mode.
- **Plan criterion:** "median within ±0.5 m of 5" is not met; "never under 2 m" is met. If the drone should hug 5 m over open ground, base height on a low percentile or the median of the cone's returns rather than the maximum. That trades away clearance over canopy.
- **Avoidance:** 19 warnings, all real. Slopes of the hill came within 1.9–3.0 m, and trees were seen at drone height near (3.8, −1.1) early in the flight. No phantom obstacles.

### Not covered in this run
- Stages 4 (fade-out of moved objects) and 6 (node restart) are out of scope.
- Stage 5 (planner end to end) has not been run yet.
- **Thermal detection:** detection was off for this run. In earlier runs the one human at (−15, −10) was logged as three casualties, each about 4 m off, and each paused the search for review. The 4 m error is bigger than `detection_confirmation_distance_m` (2.0), so the duplicates never merge.
