# TRACE: forest search and rescue

*41068 Robotics Studio I, UTS*

A hiker is lost in steep, wooded terrain. TRACE sends a drone to find them and a ground
robot to bring them supplies:

1. A **Parrot drone** flies a lawnmower search over the area, 7 m above the ground. It
   looks for a heat signature the size and shape of a person with its thermal and depth
   cameras, and maps the ground with a downward LiDAR as it flies.
2. When it finds a likely person, it stops and asks an **operator** to confirm or reject
   the detection on the dashboard.
3. Once the operator confirms, mission control plans a route over the drone's terrain map
   with A\*. The route prefers flat ground and avoids slopes over 25° and tree cover.
4. A **Husky ground robot** drives the route using Nav2. It stops 1.6–2.5 m from the
   person and holds there with supplies (first aid, food, emergency blanket). The drone
   circles overhead the whole time.

Everything runs in simulation, on ROS 2 Humble and Ignition Gazebo Fortress.

## Quick start

```bash
git clone <this repo> ~/git/forrestdrone_system
cd ~/git/forrestdrone_system
./run_simulation.sh --trace
```

This builds the workspace and starts the full mission:

- Gazebo with the `terrain_demo` world and both robots
- SLAM, Nav2 for the Husky, and RViz
- the four mission nodes, after a 20 s delay
- the rqt operator dashboard and the results logger

The drone takes off and starts searching by itself.

Common variations:

```bash
./run_simulation.sh --trace --pose 1               # fixed casualty position (0-5); default is random
./run_simulation.sh --trace --preload-map          # start from the saved terrain map
./run_simulation.sh --trace --no-gui --no-rviz     # lighter: no Gazebo window, no RViz
./run_simulation.sh --trace --no-build             # skip colcon build
./run_simulation.sh --help                         # all options
```

Without `--trace`, the script starts only the simulator and the robots, with Nav2 and RViz
by default. This is useful for working on one part at a time.

## Installation

You need Ubuntu 22.04 with ROS 2 Humble. Install ROS 2 Humble first by following the
[official guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
Then install the simulator and the ROS packages the project uses:

```bash
# Ignition Gazebo Fortress
sudo apt-get update && sudo apt-get install -y wget lsb-release gnupg
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install -y ignition-fortress

# ROS packages
sudo apt install -y ros-dev-tools \
  ros-humble-ros-ign ros-humble-ros-ign-interfaces \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-xacro ros-humble-cv-bridge ros-humble-rqt-image-view \
  ros-humble-teleop-twist-keyboard \
  libopencv-dev python3-numpy
```

Then build. `run_simulation.sh` builds automatically; to build by hand:

```bash
cd ros2_simulation
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

By default, `run_simulation.sh` sets `ROS_LOCALHOST_ONLY=1`. On a shared classroom network
this stops your nodes from seeing other people's robots. To turn it off, pass `--network`.

Don't use Gazebo Classic (`gazebo11`). It has been end of life since January 2025. This
project uses Ignition Fortress through `ros_ign`.

## Using the dashboard

The rqt dashboard opens with `--trace`. It shows:

- **Terrain map** that the drone has built. A button switches between the Height, Slope
  and Roughness views. The map is overlaid with the robots, the drone's search path, the
  detection, and the Husky's route with the distance remaining.
- **Camera feeds**, including false-colour thermal views from the drone and the Husky.
- **Mission controls**:
  - **Confirm casualty → send Husky** accepts the current detection. The drone then circles
    the person and the Husky is sent.
  - **Reject (false alarm)** throws the detection away, and the drone goes back to
    searching.
  - **Stop Husky** and **Stop Parrot** stop a robot. The button then reads *Start* and
    resumes the robot.
- **Mission state**: `SEARCHING`, `PLANNING`, `HUSKY_EN_ROUTE`, `DONE` or `FAILED`.

To confirm or reject from a terminal instead (`false` rejects):

```bash
ros2 topic pub --once --qos-durability transient_local /CasulityConfirmed std_msgs/msg/Bool "{data: true}"
```

## How it works

```
                  thermal + depth                 down LiDAR
   Gazebo ───────────────────────► parrot_node   ───────────► terrain_map_node
     ▲  ▲                            │   ▲                         │
     │  │ cmd_vel         /mission/  │   │ /parrot1/mission/path   │ /parrot1/terrain_points
     │  │                 casualty   ▼   │                         ▼
     │  └─────────────────────────── system_control_node ◄─────────┘
     │                                   │  ▲
     │  Nav2 goals           /mission/   │  │ /mission/husky_status
     └──────────── husky_node ◄── route ─┘  │
                       └────────────────────┘
```

| Node | Package | Job |
|---|---|---|
| `parrot_node` | `control_template` | Flies the drone at a fixed height above the ground and steers around trees using a 360° LiDAR. Runs the thermal detector and asks the operator to review candidates. |
| `terrain_map_node` | `control_template` | Builds a 40 × 40 m height map with 0.5 m cells from the down LiDAR. Each cell keeps a running mean (height) and variance (roughness, e.g. canopy over ground). |
| `system_control_node` | `control_template` | The mission state machine. Gives the drone up to three search passes in alternating lane directions, plans the Husky's route with A\*, and replans if the Husky fails. |
| `husky_node` | `control_template` | Drives the route with one Nav2 goal per waypoint. At the end it makes a close approach, waits until it has stopped, then keeps watch so it doesn't drift away. |
| `mission_logger.py` | `41068_ignition_bringup` | Evaluation only. Scores each run against Gazebo ground truth and appends the result to `~/trace_results/results.csv`. |
| rqt dashboard | `41068_ignition_bringup` | The operator interface described above. |

**Detection.** Each depth frame is paired with a thermal frame and the drone's odometry.
A hot region (≥ 30 °C) with enough valid depth becomes a *view* and is placed in world
coordinates. Views within 2 m of each other form a *track*. After 6 views, a track is
reported as a casualty if most of its views are person-sized (0.75–4 m) and elongated
(at least 1.6 times longer than wide). The shape check rejects animals and other small
heat sources.

**Route planning.** A\* runs over the terrain grid. Each step costs its length plus
penalties for slope and roughness. A cell can't be driven on if it is steeper than 25°,
too rough, or not yet seen by the drone. If there is no route yet, the drone is sent to
survey the corridor between the Husky and the casualty, and planning tries again.

The [control_template README](ros2_simulation/src/control_template/README.md) has more
detail on each node, its topics and parameters, and testing by hand.

## Repository layout

```
run_simulation.sh                     One-command build + launch
ros2_simulation/src/
  41068_ignition_bringup/             Simulation package (based on the 41068 starter)
    launch/trace_demo.launch.py         Full TRACE mission
    launch/41068_ignition.launch.py     Simulator + robots (+ SLAM, Nav2, RViz)
    launch/control.launch.py            The four mission nodes
    worlds/terrain_demo.sdf             Hilly forest with the heat-signature casualty
    pre_scanned_height_maps/            Saved terrain maps (ros2 bags) for --preload-map
    rqt_terrain_plugin/                 Operator dashboard
    scripts/                            Results logger, thermal false-colour view, demos
    urdf_husky/, urdf_parrot/           Robot models and sensors
  control_template/                   Mission software (C++17)
    PARROT/                             Drone flight + thermal detection
    HUSKY/                              Route following
    SYSTEM_CONTROL/                     Mission state machine, terrain map, A* planner
    dataStructures.hpp                  Every topic name, in one place
    tools/                              Offline and in-sim test scripts
```

## Testing

- **Full mission with scoring:** `./run_simulation.sh --trace --pose N`. After the mission
  ends, the logger watches the Husky for another 120 s, then prints a summary and adds a
  row to `~/trace_results/results.csv`. The row records the detection error, the stopping
  distance to the person, whether the Husky stayed put, and the timings.
- **Mission logic without Gazebo:** `tools/mission_logic_test.py` fakes the world, the
  drone and Nav2. It drives the real `terrain_map_node`, `system_control_node` and
  `husky_node`, and runs on any machine with ROS 2.
- **Casualty poses a ground robot can reach:** `tools/terrain_reachability.py`. Its results
  are conservative: it rates pose 2 unreachable, but the Husky reached it in Gazebo.
- **Detector calibration:** `tools/thermal_shape_sweep.py`.
- **Husky parking on slopes:** `tools/husky_park_test.py`.

`terrain_demo` has six casualty poses: (-15, -10), (-15, 0), (-15, 15), (12, 15),
(12, -5) and (2.4, 12.6). Select one with `--pose 0` to `--pose 5`.

## Saving a new terrain map

After a full scan, record the map into a folder under `pre_scanned_height_maps/` named
after the world:

```bash
ros2 bag record -o ros2_simulation/src/41068_ignition_bringup/pre_scanned_height_maps/<world> /parrot1/terrain_points
```

Rebuild, then run with `--preload-map`. The saved map is loaded once at startup, and live
scans keep refining it.

## Limitations

- **The drone is simplified.** It is a floating body with gravity turned off: no flight
  dynamics and no collisions.
- **The Husky doesn't avoid obstacles reactively.** It relies on the terrain route and on
  Nav2 costmaps from its 2D LiDAR. Nothing checks for moving obstacles or contact.
- **The Husky leaves as soon as any route exists.** Once it is driving, it won't switch to
  a better route found later.
- **One casualty per mission.** To run another, restart the mission nodes.

## Troubleshooting

- **`Could not find a package configuration file provided by "ros_ign_interfaces"`**:
  the ROS packages from [Installation](#installation) are missing.
- **`Detected jump back in time` or RViz flickering**: an old Gazebo process is still
  running. Find it with `ps aux | grep -E "ign gazebo|ros_ign" | grep -v grep` and kill
  it, or reboot.
- **Gazebo crashes with an Ogre or GL context error**: run `export QT_QPA_PLATFORM=xcb`
  before launching.
- **Very slow simulation**: use `--no-gui --no-rviz`. The dashboard still shows the map and
  the cameras.
- **A message published by hand never arrives**: the mission topics are latched. Add
  `--qos-durability transient_local` (and `--qos-reliability reliable`) to
  `ros2 topic pub` or `ros2 topic echo`.

More launch options and simulator errors are covered in the
[41068_ignition_bringup README](ros2_simulation/src/41068_ignition_bringup/README.md).

## Computer requirements

| | Minimum | Recommended |
|---|---|---|
| OS | Ubuntu 22.04 (native or dual boot) | Ubuntu 22.04 (native) |
| CPU | 4 cores (i5 / Ryzen 5) | 8 cores (i7 / Ryzen 7) |
| RAM | 8 GB | 16 GB |
| GPU | Integrated, OpenGL 3.3+ | Dedicated NVIDIA, 4 GB+ VRAM |
| Storage | 30 GB free on an SSD | 50 GB+ free on an NVMe SSD |

The full mission runs Gazebo with two robots and their cameras and LiDARs, plus SLAM, Nav2,
RViz and rqt. With 8 GB of RAM it will swap. A VM or WSL2 works only with GPU passthrough,
and is slow even then.
