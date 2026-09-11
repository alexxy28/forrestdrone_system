# Search_and_Rescue 
Robotics studio 

Main Idea:
- 
  Imagine you are walking in the forest, hiking up a large mountain - you can see the rain coming in but you decide to keep going. The darkness is coming in and you get lost on the path. This is where we come in - our superior innovation of saving lost hikers and animals. 

How it works:
- 

  (1) The drone A is monitoring constantly, and it patrols the area searches for heat signatures that match a human. Once it's confirmed to be a human, will have check if the human is on or off trails to determine if they are lost or not.
  
  (2) Once we can see a person is in danger Drone A sends a signal to Drone B, and drone B will be sent closer to the location to determine if the human requires rescue or not.
  
  (3) If required then a waypoint will be sent from Drone A & B to a husky robot to determine safest route. Drone B talks to the humans while sending of a signal to husky. It also lets them know help is on the way 

  (4) Drone B gets closer and decided what is needed on the site for Husky Robot to grab. Drone B has a nice look to not scare the humans. 
  
  (5) Uses LIDAR to create a topographical map and will use the path way with lowest gradient for husky.
  
  
  (6) Husky comes in on the site with items such as food, first Aid, rope, and emergency blanket. 


  (7) Then the humans will take what they have 


**Drone A**
- range and has a lot of sensors
- distance covered


**Drone B**
- drone A&B are coming together 



**Drone C**



Environmental impact (noise, scale) 
- 
in theory in the future we would probably need a swarm. 
The robot will be starting from one area, the husky will do a one way and come in and then the search rescue will take them up .

Uncertainty 
- 
weather, human interaction (multi lingual) 


Bare Minimum 
- 
mapping, planning , perception, user interface 


## Dependencies & Versions

This project targets **ROS2 Humble** (the LTS release supported until 2027).

| Component | Version / Package | Notes |
|---|---|---|
| ROS2 distro | Humble Hawksbill (LTS) | Runs on Ubuntu 22.04 |
| Ubuntu | 22.04 LTS | Required base OS for Humble |
| RViz | `rviz2` (`ros-humble-rviz2`) | Version locked by the distro |
| Gazebo | Gazebo Fortress via `ros_gz` (`ros-humble-ros-gz`) | Modern Gazebo (Ignition) |

Avoid **Gazebo Classic** (`gazebo11`) — EOL since Jan 2025. Use modern Gazebo (Fortress) with the `ros_gz` bridge on Humble.

### How to get installed versions
Run these on your machine to confirm the exact versions installed:

```bash
echo $ROS_DISTRO                                    # should print: humble
ros2 --version
apt list --installed 2>/dev/null | grep -Ei 'ros-|rviz|gazebo|gz-|ignition'
ign gazebo --version                                # Gazebo Fortress (Ignition)
dpkg -s ros-humble-rviz2 | grep Version             # exact rviz version
dpkg -s ros-humble-ros-gz | grep Version            # exact ros_gz version
```
### How to Run Project
```bash
colcon build --symlink-install
source ~git/forrestdrone_system/ros2_simulation/install/setup.bash (or provided directory)
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py husky:=true parrot:=true rviz:=true nav2:=true
```
### Likely stack for this project
- **Core:** `rclpy`/`rclcpp`, `ros2launch`, `tf2`
- **Simulation:** `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`
- **Visualization:** `rviz2`, `rviz_default_plugins`
- **Mapping/Nav:** `slam_toolbox`, `nav2_bringup` (Navigation2)
- **Sensors:** `pointcloud_to_laserscan`, camera/LIDAR drivers
- **Husky:** `husky_desktop` / `clearpath_*` packages (distro-dependent)

## Minimum Computer Specs

Running ROS2 Humble with Gazebo Fortress, RViz2, SLAM, and Nav2 simultaneously is demanding — Gazebo physics + 3D rendering and RViz point cloud visualization are the heaviest loads.

| Component | Minimum | Recommended |
|---|---|---|
| OS | Ubuntu 22.04 LTS (64-bit) | Ubuntu 22.04 LTS (64-bit) |
| CPU | Quad-core x86_64 (Intel i5 / Ryzen 5) | 8-core x86_64 (Intel i7 / Ryzen 7) or better |
| RAM | 8 GB | 16 GB+ |
| GPU | Integrated GPU (OpenGL 3.3+) | Dedicated GPU (NVIDIA GTX 1650 / 4 GB+ VRAM) |
| Storage | 30 GB free SSD | 50 GB+ free SSD (NVMe) |
| Graphics | OpenGL 3.3+ support | Recent NVIDIA driver for hardware acceleration |

### Notes
- **Gazebo Fortress** needs a GPU with proper OpenGL support. Integrated graphics work but simulations will run slowly, especially with LIDAR/camera sensors and complex terrain.
- **A dedicated NVIDIA GPU is strongly recommended** for real-time simulation and for any perception/ML (heat-signature/human detection) work.
- **16 GB RAM** is the practical minimum once Gazebo + RViz + Nav2 + SLAM run together; 8 GB will swap heavily.
- **SSD required** — HDDs make build times (`colcon build`) and asset loading painfully slow.
- **Multi-robot simulation** (Drone A/B/C + Husky at once) roughly multiplies CPU/GPU load; lean toward the recommended specs or higher.
- **VMs work but are not ideal** — GPU passthrough is required for acceptable Gazebo performance. Native Ubuntu or dual-boot is preferred over WSL2/VM for simulation.
