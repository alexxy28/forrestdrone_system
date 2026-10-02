
## 1. Architecture: open decisions and remaining node work


- [ ] *Should the SLAM toolbox be used to create the 2D costmap for use with Nav2? or do we develop the costmap from the full terrain elevation map? Option 1 has the same sensor data being sent to two places but is specialised for this puropse, Option 2 has more data but is updated less frquently*


- [ ] **`ParrotNode`**:
  - [ ] Owns the casualty-detection pipeline 
  - [ ] Publishes confirmed casualty coordinates once detected.
  - [ ] Subscribes to `/mission/state` and stops its search when the mission moves on.
  - [ ] *Use the SLAM toolbox for immediate object detection and avoidance using laser data (SLAM toolbox subscribes to laser and robot odometry then publishes a live updating 2D ocupancy grid [unknown, free, occupied wit uncertanty level]. nav2 uses occupancy grid and waypoints to move robot)*


- [ ] *Decide wether to create a series of waypoints from the elevation map that minimises slope and distance. OR Create a 2D costmap that can be provided to nav2 and nav2 handles the path planning.* (Currently waypoints from A* in `SystemControlNode`.)
- [ ] Investigate the usefullness for the drone to fly over and around the lowest sampled cells after its inital search pattern.

---

## 2. Casualty / human detection pipeline (currently unfinished)


- [ ] Create a `ThermalCamera` class; decode mono16 frames to °C (reuse the math already proven in `thermal_camera_subscriber.py`) and threshold to a human-temperature band to get a binary mask.
- [ ] Cluster the mask into blobs (connected components); filter by blob size to reject noise.
- [ ] Get a 3D world position for a candidate blob: use the depth camera/point cloud at that pixel (or camera intrinsics + drone altitude) plus TF to convert to a world/odom-frame coordinate.
- [ ] Cross-check the RGB image at the same region with a simple person-shape/heuristic check (Most probably unnecessary).
- [ ] Debounce over a few frames before confirming (avoid one noisy frame triggering a false rescue).
- [ ] Wire into `ParrotNode`: on confirmed detection, end the search pattern and publish the coordinate on `/mission/casualty`.

---

## 3. Terrain map — investigate before relying on it for C3


- [ ] Confirm `terrain_map_builder.py`'s "highest LiDAR return per 0.5 m cell" approach is detailed enough. In a forested world this risks capturing canopy/branches rather than ground, which could falsely mark walkable ground as steep/blocked.
- [ ] Check `TerrainMap`'s hole-filling (2-pass neighbour averaging, requires ≥3 neighbours) against `terrain_demo.sdf` / `large_demo.sdf` — it was written for sparse gaps, not systematic canopy occlusion.
- [ ] Resolve the size/resolution mismatch: the C++ `TerrainMap` defaults to a 20x20 m grid at 0.5 m, while `TerrainMapBuilder` (Python) defaults to 10x10 m and never actually publishes its own grid (its `self.grid` field is dead code — it only publishes the raw accumulated point cloud). Confirm whether these need to agree, since only the C++ side's grid is actually used for planning.
- [ ] Decide: full terrain-aware planning for C3, or a simplified terrain model for early testing, per your note that a simplified version is acceptable to start.
- [ ] *Add running min/max scan data (used for detecting trees), running varience (Welford's M2 term)(need more than 2 data points per cell to work), convert varience into a usable confidence score.* 
- [ ] *when interpolating unknown cells use a low confidence score or identifyer so that guesses arent used in runnung average.*

---

## 4. Build / code tasks in `control_template`

- [ ] If C++ image processing is used for detection, add `cv_bridge`, `image_transport`, and OpenCV to `package.xml` / `CMakeLists.txt` (not currently depended on).
- [ ] `thermal_camera_subscriber.py` currently lives outside any ROS package (`forrestdrone_system-pri-husky-terrain/thermal_camera_subscriber.py`, not under any `src/.../scripts`) — it can't be `ros2 run` as-is. Either fold its logic into the C++ `ThermalCamera` sensor, or move/ install it properly if it should stay a standalone debug tool.

---

## 5. Simulation / UI gaps (out of primary scope, noted for awareness)

- [ ] No dashboard/RViz panel currently shows detection state or planned route — `rqt_terrain_plugin` exists but its UI content hasn't been reviewed in depth.
- [ ] `basic_autonomy_demo.py` is unit-provided template code, not
      integrated with the mission — fine to ignore unless borrowing pieces
      from it.


## General order of operations
1. Add cv_bridge, image_transport, OpenCV to package.xml/CMakeLists.txt
2. Decide if the elevation map is detailed enough (running min/max/average/varience, and cell conficence score).
3. Fix elevation interpolation being used as first data point in subsequent scans bug.
4. Complete image processing for IR camera and maybe RGB (either in parrot or in a seperate class,*reccomended in seperate class*), return if human detected, confidence and location.
5. Make sure that only one command is sent to the simulated robot (e.g. /parrot1/cmd_vel) at a time, nav2 uses this the same as us directly changing it
6. Full state machine (Parrot reacting to the mission state)
7. single launch file that starts all nodes (so we only need one command)

---

## Tests before integration

Do these in order. Each test relies only on the ones before it. Stages 1 to 3 test one part at a
time; nothing is joined together until stage 4.

### Stage 1: no simulator

- [ ] 1. Package builds — all targets compile with no errors.
- [ ] 2. Planner finds a route through a gap 
- [ ] 3. Planner with no possible route — returns an empty route, no crash 
- [ ] 4. Planner with the goal on a steep or unseen cell — returns empty or snaps to a nearby cell; does not retry forever. 

### Stage 2: simulator, one part at a time


- [ ] 7. Husky drives one waypoint — status goes `IDLE`, `EN_ROUTE`, `ARRIVED`.
- [ ] 8. Husky drives several waypoints — visits them in order.
- [ ] 9. Husky given an unreachable waypoint — status becomes `FAILED`.
- [ ] 10. Husky started before Nav2 is up — waits, then drives the route once Nav2 is ready.
- [ ] 11. Parrot climbs — reaches and holds target altitude. 
- [ ] 12. Parrot flies the search pattern — visits every waypoint, then holds. 
- [ ] 13. Parrot stop — holds position when told. 
- [ ] 14. Cameras deliver frames — thermal values are plausible temperatures; depth is in metres. *(needs camera code)*
- [ ] 15. Detection with a casualty in view — one detection, position close to the casualty's known location. *(needs detection code)*
- [ ] 16. Detection with no casualty — no detection across the whole search pattern. *(needs detection code)*
- [ ] 17. Terrain coverage after a search flight — most cells in the search area have a known height.

### Stage 3: SystemControlNode alone, inputs published by hand

- [ ] 18. Casualty arrives — state goes `SEARCHING`, `PLANNING`, `HUSKY_EN_ROUTE` and a route is published.
- [ ] 19. Casualty arrives before terrain — control waits, then plans once terrain arrives.
- [ ] 20. Husky status faked as `ARRIVED`, then `FAILED` on a second run — state becomes `DONE`, then `FAILED`.
- [ ] 21. Second casualty published — ignored.

### Stage 4: pairs

- [ ] 22. SystemControlNode and Husky, casualty published by hand — Husky stops near the casualty; state is `DONE`. This is the first test that proves route coordinates and Husky odometry coordinates agree.
- [ ] 23. Parrot and SystemControlNode — a real detection moves control to `PLANNING`. 

### Stage 5: whole mission

- [ ] 24. Full run, nothing published by hand — reaches `DONE`.
- [ ] 25. Casualty moved, or placed behind steep ground — still reaches `DONE`, by a different route.

The commands for publishing inputs by hand are in the [README](README.md#testing-by-hand).
SystemControlNode never leaves `DONE` or `FAILED`, so restart it between runs.
