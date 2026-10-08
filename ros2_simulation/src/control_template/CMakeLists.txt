cmake_minimum_required(VERSION 3.8)

project(control_template)

find_package(ament_cmake REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(parrot_node
    PARROT/ParrotNode.cpp
    PARROT/parrot.cpp
)
target_include_directories(parrot_node PRIVATE .)
ament_target_dependencies(parrot_node rclcpp geometry_msgs nav_msgs
    sensor_msgs
    visualization_msgs
)

add_executable(husky_node
    HUSKY/huskyNode.cpp
    HUSKY/husky.cpp
)
target_include_directories(husky_node PRIVATE .)
ament_target_dependencies(husky_node rclcpp rclcpp_action nav_msgs nav2_msgs std_msgs tf2 tf2_ros)

add_library(terrain_lib STATIC
    SYSTEM_CONTROL/terrain_map.cpp
    SYSTEM_CONTROL/terrain_planner.cpp
)
target_include_directories(terrain_lib PUBLIC SYSTEM_CONTROL)
ament_target_dependencies(terrain_lib sensor_msgs)

add_executable(system_control_node SYSTEM_CONTROL/systemControlNode.cpp)
target_include_directories(system_control_node PRIVATE .)
target_link_libraries(system_control_node terrain_lib)
ament_target_dependencies(system_control_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)

install(TARGETS
    parrot_node
    husky_node
    system_control_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
