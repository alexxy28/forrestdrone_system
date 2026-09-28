Thermal camera ROS 2 subscriber

This node subscribes to the thermal and depth camera topics and opens separate OpenCV windows named `Thermal Camera` and `Depth Camera` when run in a graphical desktop session.

Run

```bash
pip install -r requirements.txt

# run the ROS2 node (requires your ROS2 environment to be sourced)
python3 thermal_camera_subscriber.py
```

Notes

- ROS2 (`rclpy`) must be available from your environment (install via your ROS2 distro).
