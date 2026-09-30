
## 1. Architecture: split into ParrotNode / HuskyNode / SystemControlNode

- [ ] Keep `Robot` / `Sensor` classes as thin domain objects (own their pub/subs
      given a node handle, use getter and setter methods to access data from the object)
- [ ] *Should the SLAM toolbox be used to create the 2D costmap for use with Nav2? or do we develop the costmap from the full terrain elevation map? Option 1 has the same sensor data being sent to two places but is specialised for this puropse, Option 2 has more data but is updated less frquently*


- [ ] Create **`ParrotNode`** (`rclcpp::Node`):
  - [ ] Owns a `Parrot` instance, drives it with a wall timer calling `tick()`. OR predetermined control frequency e.g. 100Hz
  - [ ] Owns the casualty-detection pipeline (see section 2).
  - [ ] Publishes confirmed casualty coordinates once detected.
  - [ ] *Use the SLAM toolbox for immediate object detection and avoidance using laser data (SLAM toolbox subscribes to laser and robot odometry then publishes a live updating 2D ocupancy grid [unknown, free, occupied wit uncertanty level]. nav2 uses occupancy grid and waypoints to move robot)*
  - [ ] Uses Nav2 (manage z height) OR other ways to move the robot.


- [ ] Create **`HuskyNode`** (`rclcpp::Node`):
  - [ ] Owns a `Husky` instance.
  - [ ] Receives a *planned route* (not raw terrain) from `SystemControlNode`.
  - [ ] Responsible only for executing the route via Nav2 — this is essentially `trace_husky_node.cpp`'s goal-dispatch logic (`sendGoal` / `sendNextWaypoint` / the three action callbacks) with the planning code removed.
  - [ ] *Use the SLAM toolbox for immediate object detection and avoidance using laser data (SLAM toolbox subscribes to laser and robot odometry then publishes a live updating 2D ocupancy grid [unknown, free, occupied wit uncertanty level]. nav2 uses occupancy grid and waypoints to move robot)*
  - [ ] Uses Nav2 to move the Husky


- [ ] Create **`SystemControlNode`**:
  - [ ] Owns `TerrainMap` + `TerrainPlanner` (move out of `trace_husky/` folder into a shared location, e.g. a `system_control` folder).
  - [ ] Subscribes to: terrain point cloud + casualty pose (from `ParrotNode`), Husky odometry (from `HuskyNode`).
  - [ ] Runs the planner and publishes the resulting route to `HuskyNode`.
  - [ ] Owns the mission state machine: mapping → search → detect → plan → rescue.
- [ ] **Decision made:** path planning lives in `SystemControlNode`, not on the Husky. Keeps a single source of truth for mission state, keeps `HuskyNode` a dumb/reusable executor, and matches the existing `trace_husky_node.cpp` pattern (mostly a split/rename, not a new design). Trade-off: one extra topic hop for the route — acceptable at this scale.
- [ ] *Decide wether to create a series of waypoints from the elevation map that minimises slope and distance. OR Create a 2D costmap that can be provided to nav2 and nav2 handles the path planning.*
- [ ] Investigate the usefullness for the drone to fly over and around the lowest sampled cells after its inital search pattern.

---

## 2. Casualty / human detection pipeline (currently unfinished)

`thermalCamera.hpp` / `rgbCamera.hpp` are empty stub classes, and
`thermal_camera_subscriber.py` only visualizes frames — nothing detects a
person yet.

- [ ] Implement `ThermalCamera` and `RgbCamera` as real `Sensor` subclasses (subscribe to `/parrot1/thermal_camera` and `/parrot1/camera/image`);register both  in `Sensor::create`.
- [ ] In `ThermalCamera`, decode mono16 frames to °C (reuse the math already proven in `thermal_camera_subscriber.py`) and threshold to a human-temperature band to get a binary mask.
- [ ] Cluster the mask into blobs (connected components); filter by blob size to reject noise.
- [ ] Get a 3D world position for a candidate blob: use the depth camera/point cloud at that pixel (or camera intrinsics + drone altitude) plus TF to convert to a world/odom-frame coordinate.
- [ ] Cross-check the RGB image at the same region with a simple person-shape/heuristic check (Most probably unnecessary).
- [ ] Debounce over a few frames before confirming (avoid one noisy frame triggering a false rescue).
- [ ] Wire into `ParrotNode`: on confirmed detection, call `Parrot::stop()` to end the search pattern and publish the coordinate to `SystemControlNode`.

---

## 3. Terrain map — investigate before relying on it for C3


- [ ] Confirm `terrain_map_builder.py`'s "highest LiDAR return per 0.5 m cell" approach is detailed enough. In a forested world this risks capturing canopy/branches rather than ground, which could falsely mark walkable ground as steep/blocked.
- [ ] Check `TerrainMap`'s hole-filling (2-pass neighbour averaging, requires ≥3 neighbours) against `terrain_demo.sdf` / `large_demo.sdf` — it was written for sparse gaps, not systematic canopy occlusion.
- [ ] Resolve the size/resolution mismatch: `trace_husky_node.cpp` hardcodes `TerrainMap(0.5, 20.0, 20.0)`, while `TerrainMapBuilder` (Python) defaults to 10x10 m and never actually publishes its own grid (its `self.grid` field is dead code — it only publishes the raw accumulated point cloud). Confirm whether these need to agree, since only the C++ side's grid is actually used for planning.
- [ ] Decide: full terrain-aware planning for C3, or a simplified terrain model for early testing, per your note that a simplified version is acceptable to start.



---

## 4. Build / code tasks in `control_template`

- [ ] `CMakeLists.txt` is missing executables:
  - [ ] `drone_search.cpp` is not currently built by anything.
  - [ ] Once the node split (section 1) is done, replace it with `parrot_node`, `husky_node`, and `system_control_node` executable targets, each with its own `ament_target_dependencies`.
- [ ] `Parrot` has no `.cpp` — `parrot.hpp` is declared but never defined or compiled anywhere.
- [ ] Migrate `drone_search.cpp`'s `lawnmower()` / `onOdom()` / `tick()` logic into `parrot.cpp`.
- [ ] Delete `drone_search.cpp`'s duplicate local `Waypoint` struct in favour of the shared one in `dataStructures.hpp`.
- [ ] If C++ image processing is used for detection, add `cv_bridge`, `image_transport`, and OpenCV to `package.xml` / `CMakeLists.txt` (not currently depended on).
- [ ] `SystemControl` class is a completely empty stub — needs the mission state machine and ownership of `TerrainMap` / `TerrainPlanner` moved in from `trace_husky/`.
  - [ ] *Add running min/max scan data (used for detecting trees), running varience (Welford's M2 term)(need more than 2 data points per cell to work), convert varience into a usable confidence score.* 
  - [ ] *when interpolating unknown cells use a low confidence score or identifyer so that guesses arent used in runnung average.*
- [ ] `thermal_camera_subscriber.py` currently lives outside any ROS package (`forrestdrone_system-pri-husky-terrain/thermal_camera_subscriber.py`, not under any `src/.../scripts`) — it can't be `ros2 run` as-is. Either fold its logic into the C++ `ThermalCamera` sensor, or move/ install it properly if it should stay a standalone debug tool.

---

## 5. Build / run documentation

- [ ] `README.md` currently only documents `ros2 run control_template lidar_test`. Write a proper guide covering:
  - [ ] Sourcing ROS 2 + `colcon build` from `ros2_simulation/`.
  - [ ] Launching the sim: `41068_ignition.launch.py` with `husky:=True parrot:=True nav2:=True`.
  - [ ] Separately launching terrain mapping: `41068_terrain_map.launch.py`.
  - [ ] Running/parameterizing the new nodes (e.g. `planner_mode`, `target_x` / `target_y`, `use_terrain` — once these move to `system_control_node`).

---

## 6. Simulation / UI gaps (out of primary scope, noted for awareness)

- [ ] No dashboard/RViz panel currently shows detection state or planned route — `rqt_terrain_plugin` exists but its UI content hasn't been reviewed in depth.
- [ ] `basic_autonomy_demo.py` is unit-provided template code, not
      integrated with the mission — fine to ignore unless borrowing pieces
      from it.


## General order of operations
1. Add cv_bridge, image_transport, OpenCV to package.xml/CMakeLists.txt
2. Create stubbs for node atrchetecture.
3. Decide if the elevation map is detailed enough (running min/max/average/varience, and cell conficence score).
4. Fix elevation interpolation being used as first data point in subsequent scans bug.
5. Complete RGB and IR sensor sub-calsses.
6. Complete Parrot and Husky waypoint navigation state mnachine and logic (Each given a state and sometimes waypoints, uses internal logic to move to each point while updating states). Agree on topic and data structure to send waypoints.
7. Complete image processing for IR camera and maybe RGB (either in parrot or in a seperate class,*reccomended in seperate class*), return if human detected, confidence and location.
8. Move and integrate terrainMap and terrainPlanner (impliment changes if previously agreed upon).
9. Impliment SystemControlNode, agree on a defined massage for the casualty position for Parrot to share to.
10. Make sure that only one command is sent to the simulated robot (e.g. /parrot1/cmd_vel) at a time, nav2 uses this the same as us directly changing it
11. Add parrot_node/husky_node/system_control_node executables to CMakeLists.txt
12. Full state machine
13. Documentation
14. single launch file that starts all nodes (so we only need one command)
