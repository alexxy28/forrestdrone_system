cmake_minimum_required(VERSION 3.8)

project(control_template)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


# ---------------------------------------------------------
# Existing LiDAR test
# ---------------------------------------------------------

add_executable(lidar_test
    main.cpp
    lidar.cpp
    sensor.cpp
    robot.cpp
    husky.cpp
)

ament_target_dependencies(lidar_test
    rclcpp
    sensor_msgs
)

target_include_directories(lidar_test PUBLIC
    .
)


# ---------------------------------------------------------
# TRACE Husky terrain-aware navigation
# ---------------------------------------------------------

add_executable(trace_husky_node
    trace_husky/trace_husky_node.cpp
    trace_husky/terrain_map.cpp
    trace_husky/terrain_planner.cpp
)

ament_target_dependencies(trace_husky_node
    rclcpp
    rclcpp_action
    sensor_msgs
    geometry_msgs
    nav_msgs
    nav2_msgs
    action_msgs
    tf2
    tf2_ros
)

target_include_directories(trace_husky_node PUBLIC
    .
    trace_husky
)


# ---------------------------------------------------------
# TRACE thermal casualty detector
# ---------------------------------------------------------

add_executable(thermal_casualty_detector
    trace_husky/thermal_casualty_detector.cpp
)

ament_target_dependencies(thermal_casualty_detector
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
)


# ---------------------------------------------------------
# Install ROS2 executables
# ---------------------------------------------------------

install(TARGETS
    lidar_test
    trace_husky_node
    thermal_casualty_detector
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
