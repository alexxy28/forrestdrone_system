# control_template

Mission control software for the forest search-and-rescue system. A Parrot drone flies a
search pattern over an area, a casualty position is reported, a route that avoids steep
ground is planned from the drone's terrain data, and a Husky ground robot drives that route
using Nav2.

This package holds the three ROS 2 nodes that do that. The simulator, robot models, Nav2
configuration and terrain builder live in the neighbouring
[41068_ignition_bringup](../41068_ignition_bringup/README.md) package. The project idea is
described in the [repository README](../../../README.md).

Target platform: ROS 2 Humble, Ignition Gazebo Fortress, C++17.

## Current status

- Parrot search flight: working. Climbs to altitude, visits all 10 waypoints, then holds.
- Husky route driving: working. Drives a route one waypoint at a time through Nav2 and
  reports its status.
- Mission state machine: working. Moves from searching to planning when a casualty arrives,
  and tracks the Husky.
- Terrain map and planner: partly working. The code runs, but found no route when tested.
  See [Known issues](#known-issues).
- Casualty detection: not built. Nothing publishes a casualty yet; it has to be published
  by hand.
- Parrot reacting to the mission: not built. The Parrot does not listen to the mission
  state and never stops its search early.

The whole mission has not yet run end to end without a hand-published message.

## Prerequisites and build

Install ROS 2 Humble, Ignition Fortress, Nav2, SLAM Toolbox and robot_localization by
following the installation section of the
[bringup README](../41068_ignition_bringup/README.md#installation).

Build both packages from the workspace root:

```bash
cd ros2_simulation
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build the whole workspace at least once. Building only `control_template` leaves the
bringup package's install folder out of date, and the terrain builder will be missing.

On a shared network, keep ROS traffic on your own machine:

```bash
export ROS_LOCALHOST_ONLY=1
```

## Running

Use five terminals. In each one, first run:

```bash
cd ros2_simulation
source /opt/ros/humble/setup.bash && source install/setup.bash
```

1. Simulator, both robots, Nav2 and SLAM

```bash
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py \
  world:=large_demo husky:=True parrot:=True nav2:=True
```

2. Terrain builder

```bash
ros2 run 41068_ignition_bringup terrain_map_builder.py --ros-args \
  -r __ns:=/parrot1 -r /tf:=/parrot1/tf -r /tf_static:=/parrot1/tf_static \
  -p use_sim_time:=true
```

Run the script directly instead of through `41068_terrain_map.launch.py`. The launch file
does not remap TF, so the builder would not see the Parrot's transforms.

3 to 5. The three nodes

```bash
ros2 run control_template system_control_node --ros-args -p use_sim_time:=true
ros2 run control_template husky_node          --ros-args -p use_sim_time:=true
ros2 run control_template parrot_node         --ros-args -p use_sim_time:=true
```

There is no launch file for these yet.

## Node reference

### parrot_node

Flies the drone: wait for odometry, climb, fly the search pattern, hold.
Source: [ParrotNode.cpp](PARROT/ParrotNode.cpp), [parrot.cpp](PARROT/parrot.cpp).

- Subscribes to `/parrot1/odometry` (`nav_msgs/Odometry`)
- Publishes `/parrot1/cmd_vel` (`geometry_msgs/Twist`)

The control loop runs every 100 ms. The state is worked out fresh on every cycle, so a
drone that drifts off altitude during the search goes back to climbing.

States:

- `WAITING_FOR_ODOM`: no odometry received yet. Nothing is published.
- `CLIMBING`: more than 0.5 m from the target altitude. Vertical command only.
- `SEARCHING`: at altitude, waypoints remaining. Steer toward the current waypoint.
- `HOLDING`: at altitude, all waypoints reached. Zero velocity every cycle.

The state is logged when it changes. It is not published.

### husky_node

Receives a planned route, converts it into the frame Nav2 uses, and drives it.
Source: [huskyNode.cpp](HUSKY/huskyNode.cpp), [husky.cpp](HUSKY/husky.cpp).

- Subscribes to `/mission/route` (`nav_msgs/Path`)
- Subscribes to `/husky1/tf` and `/husky1/tf_static` (`tf2_msgs/TFMessage`)
- Publishes `/mission/husky_status` (`std_msgs/String`)
- Action client for `/husky1/navigate_to_pose` (`nav2_msgs/NavigateToPose`)

It has no parameters. Every 200 ms it checks whether a route is waiting; once Nav2's
action server is up and the `husky1_odom` to `husky1_map` transform is available, it hands
the route to the `Husky` class. A route that arrives while another is being driven is
ignored.

Statuses:

- `IDLE`: no route received yet.
- `EN_ROUTE`: driving; one Nav2 goal is active.
- `ARRIVED`: the last waypoint succeeded.
- `FAILED`: Nav2 rejected a goal, or a goal ended in anything other than success.

The status is published only when it changes.

### system_control_node

Owns the mission: builds the terrain map, plans the route and tracks progress.
Source: [systemControlNode.cpp](SYSTEM_CONTROL/systemControlNode.cpp).

- Subscribes to `/parrot1/terrain_points` (`sensor_msgs/PointCloud2`)
- Subscribes to `/husky1/odometry` (`nav_msgs/Odometry`)
- Subscribes to `/mission/casualty` (`geometry_msgs/PointStamped`)
- Subscribes to `/mission/husky_status` (`std_msgs/String`)
- Publishes `/mission/route` (`nav_msgs/Path`)
- Publishes `/mission/state` (`std_msgs/String`)

States:

- `SEARCHING`: entered when the node starts. Leaves when the first casualty message
  arrives.
- `PLANNING`: entered when a casualty has arrived. Leaves when a route is found. Planning
  is retried every second.
- `HUSKY_EN_ROUTE`: entered when the route has been published. Leaves when the Husky
  reports `ARRIVED` or `FAILED`.
- `DONE`: entered when the Husky reported `ARRIVED`. Never leaves.
- `FAILED`: entered when the Husky reported `FAILED`. Never leaves.

Only the first casualty is used; later ones are ignored. `DONE` and `FAILED` are final, so
restart the node to run another mission.

## Topics

Every topic name is defined once, in `namespace topics` in
[dataStructures.hpp](dataStructures.hpp).

- `ROUTE`: `/mission/route`, `nav_msgs/Path`, control to husky, latched
- `HUSKY_STATUS`: `/mission/husky_status`, `std_msgs/String`, husky to control, latched
- `CASUALTY`: `/mission/casualty`, `geometry_msgs/PointStamped`, (not built) to control,
  latched
- `STATE`: `/mission/state`, `std_msgs/String`, control to all, latched
- `PARROT_ODOM`: `/parrot1/odometry`, `nav_msgs/Odometry`, sim to parrot, default QoS,
  depth 10
- `PARROT_CMD_VEL`: `/parrot1/cmd_vel`, `geometry_msgs/Twist`, parrot to sim, default QoS,
  depth 10
- `PARROT_TERRAIN`: `/parrot1/terrain_points`, `sensor_msgs/PointCloud2`, terrain builder
  to control, sensor data QoS
- `HUSKY_ODOM`: `/husky1/odometry`, `nav_msgs/Odometry`, sim to control, sensor data QoS
- `HUSKY_NAVIGATE`: `/husky1/navigate_to_pose`, `nav2_msgs/NavigateToPose` action, husky
  to Nav2
- `HUSKY_TF`: `/husky1/tf`, `tf2_msgs/TFMessage`, sim and Nav2 to husky, TF default QoS
- `HUSKY_TF_STATIC`: `/husky1/tf_static`, `tf2_msgs/TFMessage`, sim to husky, TF default
  QoS

"Latched" means reliable, transient local, depth 1. A node that starts late still receives
the last message, so the nodes can be started in any order. The cost is that both ends must
use exactly this QoS, including anything published by hand.

## How it works

### Parrot search

The search pattern is a lawnmower sweep, built once at start-up. It covers a 16 m by 16 m
square whose corner is at (-10, -10), in rows 4 m apart. That gives five rows and ten
waypoints, starting at (-10, -10) and ending at (6, 6).

The control law has three parts:

- Altitude: vertical speed is 0.8 times the height error, limited to 1 m/s. It is applied
  during the climb and throughout the search.
- Heading: turn rate is 1.5 times the heading error toward the waypoint, limited to
  1 rad/s.
- Speed: the drone moves forward only when it is within 0.5 rad of the right heading.
  Forward speed is half the remaining distance, capped at `max_speed`. This stops it
  swinging wide on the sharp turns at the end of each row.

### Terrain map

[terrain_map.cpp](SYSTEM_CONTROL/terrain_map.cpp) turns the terrain point cloud into a grid
of slopes. With the default parameters the grid is 40 by 40 cells, centred on the origin.
The map is rebuilt from scratch every time a cloud arrives:

1. Each cell takes the height of the highest point that falls inside it.
2. Small holes are filled. An empty cell with at least three known neighbours, out of
   eight, takes their average. This runs twice.
3. Each cell's slope is the steepest height change to any of its four direct neighbours,
   as an angle. A cell needs at least two known neighbours to get a slope.
4. A cell can be driven on if its slope is known and no more than 25 degrees.

Cells on the outer edge of the grid never get a slope, so they are never drivable.

### Route planner

[terrain_planner.cpp](SYSTEM_CONTROL/terrain_planner.cpp) runs A* over the grid, moving to
any of the eight neighbouring cells that can be driven on. The cost of a step is

```
cost = 1.0 x step length + 4.0 x (slope of the next cell / 25)
```

where the step length is 1 for a straight move and 1.41 for a diagonal. Flat ground is
therefore preferred even when it is longer. The planner returns the centres of the cells
along the route, or nothing if the start or goal is outside the grid or no route exists.

### From plan to driving

1. `system_control_node` plans from the Husky's current position to the casualty.
2. It keeps every second point (`route_step`), always keeping the last, so Nav2 gets fewer
   and longer legs.
3. It drops the first point, which is where the Husky already is, and publishes the rest
   on `/mission/route`.
4. `husky_node` converts each point from the `husky1_odom` frame to the `husky1_map` frame.
5. `Husky` sends one Nav2 goal per waypoint. Each goal faces the next waypoint; the last
   one keeps the heading the robot arrived with.

## Frames and odometry

Each robot has two odometry topics, and they are not interchangeable.

- `/<robot>/odometry` is raw, straight from Gazebo. It is in world coordinates for both
  the Parrot and the Husky.
- `/<robot>/odom` is the EKF output from robot_localization. For the Parrot, x and y are
  several metres off. For the Husky, it agrees with the raw topic at start.

Transforms are published per robot, on `/husky1/tf` and `/parrot1/tf`, not on the global
`/tf`. A TF listener always subscribes to `/tf`, so `husky_node` remaps it in its own
constructor. Any new node that needs transforms must do the same, or be started with
`-r /tf:=/<robot>/tf -r /tf_static:=/<robot>/tf_static`.

The route on `/mission/route` is stamped with the terrain cloud's frame, `parrot1_odom`.
`husky_node` treats the points as `husky1_odom` coordinates. This relies on both frames
matching world coordinates.

## Testing by hand

The mission topics are latched, so `--qos-durability transient_local` is required on every
hand-published message. Without it the message is silently never delivered.

Watch the mission:

```bash
ros2 topic echo --qos-durability transient_local --qos-reliability reliable /mission/state
ros2 topic echo --qos-durability transient_local --qos-reliability reliable /mission/husky_status
```

Publish a casualty. `system_control_node` should move to `PLANNING`, then to
`HUSKY_EN_ROUTE` once it finds a route:

```bash
ros2 topic pub --once --qos-durability transient_local /mission/casualty \
  geometry_msgs/msg/PointStamped "{point: {x: 5.0, y: -5.0}}"
```

Send a route straight to the Husky, bypassing the planner. The status should go
`EN_ROUTE`, then `ARRIVED`:

```bash
ros2 topic pub --once --qos-durability transient_local /mission/route \
  nav_msgs/msg/Path "{poses: [{pose: {position: {x: 2.0, y: 0.0}}}, {pose: {position: {x: 4.0, y: 1.0}}}]}"
```

Fake a Husky status. This only has an effect while the mission is in `HUSKY_EN_ROUTE`:

```bash
ros2 topic pub --once --qos-durability transient_local /mission/husky_status \
  std_msgs/msg/String "{data: ARRIVED}"
```

The full ordered test list is in [ToDo.md](ToDo.md).

## Known issues

- No casualty publisher. Detection from the thermal camera is not written. Until it is,
  the mission stays in `SEARCHING` unless a casualty is published by hand.
- The Parrot ignores the mission. `/mission/state` has no subscriber. The drone flies its
  whole pattern and holds, whatever the mission state is.
- The planner finds no route. The Parrot's lidar scans a flat horizontal plane, and the
  terrain map keeps the highest point per cell. In `large_demo` on 2 October 2026 most
  cells came out at about 20 m, the drone's own altitude, and the planner reported
  "No traversable route yet" indefinitely. The other worlds have not been tried; the same
  lidar geometry applies to them.
- The terrain map is offset from the world. The terrain builder places points using the
  Parrot's EKF transform, which carries the x and y error described above. The Husky's
  start position comes from raw odometry, so the two do not line up.
- The Husky stops short. Nav2's goal tolerance is 1.0 m. In testing the Husky reported
  arrival 0.95 m from the final waypoint.
- Nav2 also runs on the Parrot. `nav2:=True` starts Nav2 for every robot, and the Parrot's
  Nav2 publishes on the same `/parrot1/cmd_vel` as `parrot_node`. The launch file has no
  option for Nav2 on the Husky only.
- The Parrot odometry change is not yet flight-tested. It builds, and the node subscribes
  to the raw topic, but the search has not been re-flown in the simulator since.

## Conventions for contributors

- Topic names go in `namespace topics` in [dataStructures.hpp](dataStructures.hpp), with a
  comment giving the message type and direction. No topic strings in node files.
- Shared types (anything two nodes must agree on) also go in `dataStructures.hpp`. A type
  used by one file stays in that file.
- Robot class versus node class. Movement and the robot's own topics go in the robot
  class (`Parrot`, `Husky`). Parameters, timers, mission topics and state go in the node
  class (`ParrotNode`, `HuskyNode`).
- Parameters are declared by the node, in its constructor's initialiser list, and passed
  down as plain values.
- Mission topics are latched. Use `rclcpp::QoS(1).reliable().transient_local()` on both
  ends.
- Helpers private to one file go in an anonymous namespace at the top of that file.
- One executable per node, each with its own `main()` at the bottom of the node's source
  file and its own target in [CMakeLists.txt](CMakeLists.txt).

## Further reading

- [ToDo.md](ToDo.md): open design decisions, remaining work and the staged test plan.
- [41068_ignition_bringup README](../41068_ignition_bringup/README.md): installation,
  launch arguments, worlds and the demo scripts.
- [Repository README](../../../README.md): the project concept.
