# control_template

The mission software for TRACE, written in C++17 for ROS 2 Humble. It contains four nodes:

| Node | Source | Job |
|---|---|---|
| `parrot_node` | [PARROT/](PARROT/) | Flies the drone, detects casualties with the thermal camera, and asks the operator to review each detection |
| `terrain_map_node` | [SYSTEM_CONTROL/terrain_map_node.cpp](SYSTEM_CONTROL/terrain_map_node.cpp) | Builds the terrain map from the drone's down LiDAR |
| `system_control_node` | [SYSTEM_CONTROL/systemControlNode.cpp](SYSTEM_CONTROL/systemControlNode.cpp) | Runs the mission state machine, tasks the drone, and plans the Husky's route |
| `husky_node` | [HUSKY/](HUSKY/) | Drives the route through Nav2, makes the final approach, and holds position |

The simulator, robot models, Nav2 configuration, launch files and operator dashboard are in
[41068_ignition_bringup](../41068_ignition_bringup/README.md). The
[repository README](../../../README.md) covers installation, the one-command demo, and an
overview of the system.

## Contents

- [Build and run](#build-and-run)
- [Mission flow](#mission-flow)
- [parrot_node](#parrot_node)
- [terrain_map_node](#terrain_map_node)
- [system_control_node](#system_control_node)
- [husky_node](#husky_node)
- [Topics and services](#topics-and-services)
- [Frames](#frames)
- [Testing](#testing)
- [Known issues](#known-issues)
- [Conventions for contributors](#conventions-for-contributors)

## Build and run

Build from the workspace root. Build the whole workspace at least once, because the launch
files and the dashboard are in the bringup package.

```bash
cd ros2_simulation
source /opt/ros/humble/setup.bash
colcon build --symlink-install            # or: --packages-select control_template
source install/setup.bash
```

The simplest way to run the full mission is `./run_simulation.sh --trace` from the
repository root. To run the parts separately, use two terminals:

```bash
# 1. Simulator, both robots, SLAM, Husky Nav2
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py \
  world:=terrain_demo husky:=true parrot:=true slam:=true nav2:=true

# 2. The four mission nodes
ros2 launch 41068_ignition_bringup control.launch.py auto_start:=true
```

`control.launch.py` arguments:

| Argument | Default | Meaning |
|---|---|---|
| `nodes` | `all` | Comma-separated subset of `system_control`, `husky`, `parrot`, `terrain_map` |
| `auto_start` | `false` | Start the area search as soon as the drone has climbed. If false, start it with `ros2 service call /mission/start std_srvs/srv/Trigger` |
| `height_map` | empty | Saved terrain map to preload: a folder name in `pre_scanned_height_maps/` (e.g. `terrain_demo`) or a bag path |
| `use_sim_time` | `true` | |

You can also run a single node on its own, e.g. `ros2 run control_template husky_node`.
The mission topics are latched, so the nodes can start in any order.

## Mission flow

```
 system_control_node                 parrot_node                       husky_node
 ───────────────────                 ───────────                       ──────────
 SEARCHING ── search pass ─────────► fly lanes, run detector
                                     candidate found: hover (PAUSED) ◄── operator
                                     confirm: publish /mission/casualty
 PLANNING ◄─ /mission/casualty ───── fly to 5 m review point, circle,
   A* on terrain map                 resume search
   no route? ── survey corridor ───► fly the corridor
 HUSKY_EN_ROUTE ── /mission/route ────────────────────────────────────► Nav2 goal per waypoint
                                                                       final approach, settle
 DONE ◄──────────────────────────────────────── ARRIVED ────────────── hold, watch for drift
   overwatch ───────────────────────► hover 5 m from casualty
```

1. **Search.** `system_control_node` sends the drone a lawnmower path over the search area,
   by default x and y from -18 to 18 m with lanes 6 m apart. If a pass ends with nobody
   found, it sends another, up to 3 passes. Pass 2 runs north-south. Pass 3 runs
   east-west again, offset by half a lane.
2. **Detect and review.** When the thermal detector reports a person, `parrot_node` hovers
   and publishes `true` on `/CasulityDiscovered`. It then waits, with no time limit, for the
   operator's answer on `/CasulityConfirmed`.
   - **Reject:** the drone carries on with its search.
   - **Confirm:** it publishes the position on `/mission/casualty`, flies to a point 5 m from
     the person, circles once at that radius, and then carries on with the rest of its
     search.
3. **Plan.** Every second, `system_control_node` runs A\* over the terrain map, from the
   Husky to a stand-off ring around the casualty. If there's no route yet, and the drone is
   free, it sends the drone to map the box around the Husky and the casualty. Each attempt
   widens the box (up to 3 surveys). If there's still no route 20 s after the last survey,
   the mission is `FAILED`.
4. **Drive.** `husky_node` drives the route with one Nav2 goal per waypoint. At the end it
   checks its true distance to the casualty and, if needed, sends a short approach goal.
   It reports `ARRIVED` only once it has been stationary for 2 s.
5. **Done.** Mission control checks the arrival against its own odometry (within 3 m) and
   moves to `DONE`. It then sends the drone to hover 5 m from the casualty on the Husky's
   side, so the camera sees both. If the Husky later drifts away, the mission leaves `DONE`.

## parrot_node

Flies the drone along whatever path `system_control_node` sends it. It has no search
pattern of its own.

Source:
- [ParrotNode.cpp](PARROT/ParrotNode.cpp): states, casualty review, mission topics
- [parrot.cpp](PARROT/parrot.cpp): flight control and obstacle steering
- [ThermalCamera.cpp](PARROT/ThermalCamera.cpp): detection
- [main.cpp](main.cpp): the executable's `main` and a terminal interface

**Flight.** The drone holds `target_altitude` (7 m) *above the ground*, measured to the
highest return from the down LiDAR. It doesn't use odometry z, which isn't reliable on this
terrain. It turns toward the next waypoint, and only moves forward when it is roughly facing
that waypoint. Trees and slopes seen by the horizontal 360° LiDAR push it sideways:

- A cluster narrower than `max_tree_width` (2 m) is a tree and pushes within `avoid_radius`
  (4 m).
- A wider cluster is a slope and pushes within `slope_radius` (3 m).

If the drone makes no progress toward a waypoint for `stuck_timeout` (15 s), it skips that
waypoint.

**States**, published on `/parrot1/mission/status`:

| State | Meaning |
|---|---|
| `WAITING_FOR_ODOM` | No odometry yet |
| `CLIMBING` | Climbing to the target height. Happens once; after that the height is held while flying |
| `IDLE` | At altitude, no path received. `auto_start` starts the search on this state |
| `SEARCHING` | Flying a path (search, review, survey or overwatch) |
| `HOLDING` | Path finished, hovering. Mission control uses this to send the next task |
| `PAUSED` | Stop button held, or waiting for the operator to review a detection |

An empty path on `/parrot1/mission/path` means "stop and hover".

### Thermal detection

The thermal camera publishes mono16 images in units of 0.01 K. Each depth frame (5 Hz) is
paired with a thermal frame taken within 0.04 s of it, and with raw odometry taken within
0.03 s.

1. **Hot blobs.** Pixels at or above `heat_threshold_c` (30 °C), or above the image's 98th
   percentile if that is higher, are cleaned up with a 3×3 open and close, then split into
   connected blobs.
2. **Views.** A blob counts as a *view* of something only if all of these hold:
   - at least 25% of its pixels have valid depth;
   - it doesn't touch the image edge;
   - it has at least `min_classify_pixels` (60) pixels;
   - the camera sees it from at least `min_view_elevation_deg` (20°) above the horizon.

   Its pixels are then projected into world coordinates with the depth image, the camera
   mount (pitch 45°) and the drone's pose, and its size and elongation are measured.
3. **Tracks.** Views within `detection_confirmation_distance_m` (2 m) of each other join
   the same track.
4. **Decision.** Once a track has `detection_confirmation_frames` (6) views, it becomes a
   casualty if at least `min_human_view_fraction` (60%) of its views look like a person:
   median size `min_human_size_m` to `max_human_size_m` (0.75–4 m) and elongation at least
   `min_human_elongation` (1.6×). The size and elongation check rejects animals: in
   calibration a person measured 0.97 m at 2.5× elongation, and the demo animal 0.56 m at
   1.2×.

Set `show_camera_windows:=true` for OpenCV debug windows showing the thermal and depth
images.

### Terminal commands

`parrot_node` also reads commands from stdin. This only works with `ros2 run`; under
`ros2 launch` stdin isn't connected.

```
list               all detections, with position and status
confirm <index>    same as the dashboard's Confirm button
deny <index>       same as Reject
quit
```

## terrain_map_node

Builds a 2.5D grid from the drone's down LiDAR (`/parrot1/scan/points`). The grid is
40 × 40 m with 0.5 m cells, centred on the origin. The drone's raw odometry places each
return in world coordinates. The node discards returns that hit the drone itself (closer
than 0.6 m) and outliers, and skips scans whose odometry is more than 0.25 s out of step
with the scan.

Each cell keeps a running mean (height) and variance (roughness) of its returns. After 50
returns, older data starts to fade out, so moved objects clear from the map. The map is
then rebuilt:

- **Known cell:** at least 3 returns.
- **Small holes:** filled from the average of their known neighbours, in 2 passes. A hole
  needs at least 3 of its 8 neighbours known.
- **Slope:** the steepest change to any of the 4 direct neighbours.
- **Drivable:** the slope is at most 25° and the spread (sigma) is at most 0.5 m. A larger
  spread means canopy over ground. Unknown cells are never drivable.

The node publishes `/parrot1/terrain_points`, one point per known cell with fields `x, y, z,
sigma, count`, and prints statistics every 5 s.

**Saved maps.** With `height_map:=...`, `control.launch.py` plays a recorded bag in a loop
onto `/parrot1/terrain_seed`. The node loads only the first cloud and then stops listening,
so live scans aren't overwritten. To save a map after a full scan:

```bash
ros2 bag record -o ../41068_ignition_bringup/pre_scanned_height_maps/<name> /parrot1/terrain_points
```

## system_control_node

Owns the mission. Source: [systemControlNode.cpp](SYSTEM_CONTROL/systemControlNode.cpp),
[terrain_map.cpp](SYSTEM_CONTROL/terrain_map.cpp) (it keeps its own copy of the map), and
[terrain_planner.cpp](SYSTEM_CONTROL/terrain_planner.cpp).

**States**, published latched on `/mission/state`:

| State | Entered when | Leaves when |
|---|---|---|
| `SEARCHING` | Node starts | The first confirmed casualty arrives |
| `PLANNING` | Casualty confirmed, or the Husky failed and a replan is allowed | A route is found, or no route after 3 surveys → `FAILED` |
| `HUSKY_EN_ROUTE` | Route published | Husky `ARRIVED` and within `arrival_max_m` → `DONE`; or `FAILED` (see below) |
| `DONE` | Arrival confirmed | Husky moves out of range, or reports anything but `ARRIVED` → `HUSKY_EN_ROUTE` |
| `FAILED` | No route, or the Husky failed too often | Final. The operator decides what to do next |

When the Husky reports `FAILED`, the node replans from where the Husky is now, up to 2
times. If a replanned route got the Husky less than 1 m closer, it treats the casualty as
blocked and goes straight to `FAILED`.

Only the first confirmed casualty is used. Restart the nodes to run another mission.

**Route planner.** A\* over the 8-connected grid. The cost of a step to the next cell is

```
1.0 × step length + 4.0 × slope / 25° + 4.0 × sigma / 0.5 m
```

The goal is the first drivable cell the search reaches in a ring 1.6–2.5 m from the
casualty. It is never the casualty's own cell: a person lying down can look drivable in the
map, but Nav2 then refuses the goal because the person is an obstacle. Cells within 1 m of
the Husky are allowed regardless, because the drone sees the Husky's own body as steep
ground. Every 6th cell of the route is kept (`route_step`), about one waypoint every 3 m,
always including the last. The Husky's own cell is dropped.

**Parameters**

| Parameter | Default | |
|---|---|---|
| `area_x0`, `area_y0`, `area_width`, `area_height` | -18, -18, 36, 36 | Search area (m) |
| `lane_spacing` | 6.0 | Distance between lawnmower lanes (m) |
| `auto_start` | false | Start the search when the drone reports `IDLE` |
| `route_step` | 6 | Keep every Nth route cell |
| `arrival_max_m` | 3.0 | Furthest the Husky can be from the casualty for `DONE` |
| `click_box_size` | 8.0 | Size of the search box around a point clicked in RViz (m) |
| `terrain_resolution`, `terrain_size_x`, `terrain_size_y` | 0.5, 40, 40 | Must match `terrain_map_node` |
| `mission_frame` | `world` | Frame of the paths sent to the drone |

During `SEARCHING`, clicking a point in RViz with *Publish Point* (`/parrot1/clicked_point`)
sends the drone to search an 8 × 8 m box around it.

## husky_node

Receives the route and drives it. Source: [huskyNode.cpp](HUSKY/huskyNode.cpp), which
handles the frames, final approach and hold watch, and [husky.cpp](HUSKY/husky.cpp), which
sends the Nav2 goals.

- **Frames.** Route points are in world coordinates. Nav2 runs in `nav_frame`
  (`husky1_odom`). The world → `husky1_odom` offset is measured again each time a goal is
  sent, by comparing the TF pose with the raw `/husky1/odometry` pose. `husky1_map` isn't
  used, because SLAM's map frame jumped by metres on the slopes.
- **Waypoints.** One `NavigateToPose` goal per waypoint, facing the next waypoint. If Nav2
  can't reach an intermediate waypoint, the node skips it and tries the next one. The route
  fails only if the last waypoint fails, or if more than 2 waypoints in a row are skipped.
- **Final approach.** Nav2's goal tolerance is 0.5 m and odometry drifts, so the node
  measures the true distance to the casualty when the route ends. If it is more than
  `approach_max_m` (2.5 m) but within `approach_range_m` (6 m), the node sends a goal
  `approach_standoff_m` (2.0 m) from the casualty, up to `approach_attempts` (3) times.
- **Settle.** The node reports `ARRIVED` only after the Husky has been slower than
  `still_speed_mps` (0.01 m/s) for `still_time_s` (2 s). If that takes longer than
  `settle_timeout_s` (20 s), the route fails.
- **Hold watch.** After arrival, the stop position is published on `/mission/husky_hold`.
  If the Husky moves more than `hold_drift_m` (0.3 m) from it, the node reports `EN_ROUTE`
  and drives back, up to `hold_reapproaches` (2) times, and then reports `FAILED`.
- **Stop button.** `/husky1/estop` set to `true` cancels the current goal and holds.
  Setting it back to `false` resumes the route.

**Statuses**, published latched on `/mission/husky_status` whenever they change:

| Status | Meaning |
|---|---|
| `IDLE` | No route yet |
| `EN_ROUTE` | Driving, approaching, settling, or driving back after drift |
| `STOPPED` | Stop button held mid-route |
| `ARRIVED` | Stopped within the stand-off and staying there |
| `FAILED` | Route, approach or hold failed |

## Topics and services

Every topic name used here is defined once, in `namespace topics` in
[dataStructures.hpp](dataStructures.hpp). "Latched" means `QoS(1).reliable().transient_local()`.
The cost of this is that a hand-published message must use the same QoS (see
[Testing](#testing)).

| Topic | Type | From → to | QoS |
|---|---|---|---|
| `/mission/state` | `std_msgs/String` | control → all | latched |
| `/mission/casualty` | `geometry_msgs/PointStamped` | parrot → control, husky | latched |
| `/mission/casualty_candidate` | `geometry_msgs/PointStamped` | parrot → dashboard (awaiting review) | latched |
| `/mission/route` | `nav_msgs/Path` | control → husky | latched |
| `/mission/husky_status` | `std_msgs/String` | husky → control | latched |
| `/mission/husky_hold` | `geometry_msgs/PointStamped` | husky → dashboard | latched |
| `/parrot1/mission/path` | `nav_msgs/Path` | control → parrot | latched |
| `/parrot1/mission/status` | `std_msgs/String` | parrot → control | latched |
| `/CasulityDiscovered` | `std_msgs/Bool` | parrot → dashboard | latched |
| `/CasulityConfirmed` | `std_msgs/Bool` | dashboard → parrot | latched |
| `/mission_control/{husky1,parrot1}/estop` | `std_msgs/Bool` | dashboard → control | default |
| `/husky1/estop`, `/parrot1/estop` | `std_msgs/Bool` | control → robot | default |
| `/parrot1/terrain_points` | `sensor_msgs/PointCloud2` | terrain_map → control, dashboard | sensor data |
| `/parrot1/terrain_seed` | `sensor_msgs/PointCloud2` | bag → terrain_map | |
| `/parrot1/odometry`, `/husky1/odometry` | `nav_msgs/Odometry` | sim (raw, world frame) | sensor data |
| `/parrot1/scan/points` | `sensor_msgs/PointCloud2` | sim → terrain_map, parrot (down LiDAR) | sensor data |
| `/parrot1/obstacle_scan` | `sensor_msgs/LaserScan` | sim → parrot (360° LiDAR) | sensor data |
| `/parrot1/thermal_camera`, `/parrot1/camera/depth/image` | `sensor_msgs/Image` | sim → parrot | sensor data |
| `/parrot1/cmd_vel` | `geometry_msgs/Twist` | parrot → sim | default |
| `/husky1/navigate_to_pose` | `nav2_msgs/NavigateToPose` action | husky → Nav2 | |

Services on `system_control_node` (`std_srvs/Trigger`):

- `/mission/start` starts the area search.
- `/mission/stop` makes the drone stop and hover.

## Frames

- **Two odometry topics per robot.** `/<robot>/odometry` is the raw pose from Gazebo, in
  world coordinates; every node here uses it. `/<robot>/odom` is the EKF output and isn't
  used. For the Parrot it is several metres off in x and y.
- **Per-robot TF.** Each robot's transforms are on `/husky1/tf` and `/parrot1/tf`, not on
  `/tf`. `husky_node` remaps `/tf` in its own constructor. Any new node that needs
  transforms must do the same, or be started with
  `-r /tf:=/<robot>/tf -r /tf_static:=/<robot>/tf_static`.
- **Mission positions are world coordinates.** The terrain map is stamped `parrot1_odom`,
  which matches the world in this simulation (see the "ponytail" note in `tryPlan()`).
  If that ever changes, a TF lookup is needed there.

## Testing

**Mission logic without Gazebo.**
[tools/mission_logic_test.py](tools/mission_logic_test.py) fakes the world (flat ground, a
5 m hill, trees, the Husky and the casualty), the drone's search, odometry drift and Nav2.
It tests the real `terrain_map_node`, `system_control_node` and `husky_node`:

```bash
ros2 run control_template system_control_node &
ros2 run control_template terrain_map_node &
ros2 run control_template husky_node &
python3 tools/mission_logic_test.py --scenario nominal
# scenarios: nominal, estop, nav_abort, unreachable, drift, lost_goal
```

**In Gazebo:**
- [tools/thermal_shape_sweep.py](tools/thermal_shape_sweep.py) teleports the drone around a
  target and measures its blob with the same maths as the detector. It was used to set the
  size and elongation limits.
- [tools/husky_park_test.py](tools/husky_park_test.py) checks whether the Husky stays put
  on a slope with zero command.
- [tools/terrain_reachability.py](tools/terrain_reachability.py) flood-fills the real
  terrain mesh with the same slope rule. Its results are conservative: it rates pose 2
  unreachable, but the Husky reached it.
- `./run_simulation.sh --trace --pose N` runs the full mission, scored by `mission_logger.py`
  into `~/trace_results/results.csv`.

**By hand.** The mission topics are latched, so every `ros2 topic pub` and `echo` on them
needs `--qos-durability transient_local --qos-reliability reliable`. Without those options,
messages are silently never delivered.

```bash
# Watch
ros2 topic echo --qos-durability transient_local --qos-reliability reliable /mission/state
ros2 topic echo --qos-durability transient_local --qos-reliability reliable /mission/husky_status

# Fake a confirmed casualty (control goes to PLANNING)
ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable /mission/casualty \
  geometry_msgs/msg/PointStamped "{point: {x: -15.0, y: 0.0}}"

# Answer a detection review
ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable /CasulityConfirmed \
  std_msgs/msg/Bool "{data: true}"

# Send the Husky a route directly, bypassing the planner (world coordinates)
ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable /mission/route \
  nav_msgs/msg/Path "{poses: [{pose: {position: {x: 2.0, y: 0.0}}}, {pose: {position: {x: 4.0, y: 1.0}}}]}"
```

## Known issues

- **The Husky leaves on the first route found.** Planning starts as soon as any terrain is
  known. Once driving, `husky_node` ignores new routes until it finishes or fails. A better
  route found later is used only after a `FAILED`.
- **No reactive obstacle avoidance on the Husky beyond Nav2.** The Husky relies on the
  terrain route and on Nav2's costmaps from its 2D LiDAR. There is no contact sensing and
  no check for moving obstacles.
- **Camera mount values are hard-coded.** The detector assumes the thermal and depth images
  are pixel-aligned, and takes the camera mount (pitch, offsets) from parameters instead of
  TF. These must match `urdf_parrot/parrot.urdf.xacro` if the sensors move.
- **Unused aim callback.** `ThermalCamera` still takes an aim callback in its constructor,
  but the drone no longer turns to centre a heat source.
- **The executor must drain the queues.** `main.cpp` uses `spin_all`. With `spin_some` at
  10 Hz, half of the 20 Hz odometry messages were lost, and many detector views were
  dropped for lack of odometry close enough in time.
- **The frames are assumed to agree.** The terrain frame (`parrot1_odom`) is assumed to be
  the world frame (see [Frames](#frames)).
- **One casualty per mission.**

## Conventions for contributors

- **Topic names:** put them in `namespace topics` in [dataStructures.hpp](dataStructures.hpp),
  with a comment giving the message type and direction. Don't put topic strings in node
  files.
- **Shared types:** a type two nodes must agree on also goes in `dataStructures.hpp`. A
  type used by one file stays in that file.
- **Robot class vs node class:** movement and the robot's own topics go in the robot class
  (`Parrot`, `Husky`). Parameters, timers, mission topics and state go in the node class
  (`ParrotNode`, `HuskyNode`).
- **Parameters:** each node declares its own and passes them down as plain values.
- **QoS:** mission topics are latched on both ends.
- **File-private helpers:** put them in an anonymous namespace at the top of the file.
- **Executables:** each node is one executable with its own target in
  [CMakeLists.txt](CMakeLists.txt). `parrot_node`'s `main` is in [main.cpp](main.cpp); the
  others are at the bottom of their node's source file.
- **Comments:** when a constant or a check exists because of something seen in Gazebo,
  say what was seen and when, as the existing comments do.
