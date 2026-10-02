cmake_minimum_required(VERSION 3.8)

project(control_template)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


add_library(husky_lib STATIC
    HUSKY/husky.cpp
    SENSORS/lidar.cpp
)

ament_target_dependencies(husky_lib
    rclcpp
    rclcpp_action
    sensor_msgs
    nav2_msgs
)

target_include_directories(husky_lib PUBLIC
    .
    SENSORS
    HUSKY
)

add_executable(lidar_test main.cpp)
target_link_libraries(lidar_test husky_lib)

install(TARGETS
    lidar_test
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(camera_test
    camera_test.cpp
    SENSORS/thermalCamera.cpp
    SENSORS/rgbCamera.cpp
)

ament_target_dependencies(camera_test
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
)

target_include_directories(camera_test PUBLIC
    SENSORS
)

install(TARGETS
    camera_test
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(parrot_node PARROT/ParrotNode.cpp)
ament_target_dependencies(parrot_node rclcpp)

add_executable(husky_node HUSKY/huskyNode.cpp)
target_link_libraries(husky_node husky_lib)
ament_target_dependencies(husky_node rclcpp nav_msgs std_msgs tf2 tf2_ros)

add_library(terrain_lib STATIC
    SYSTEM_CONTROL/terrain_map.cpp
    SYSTEM_CONTROL/terrain_planner.cpp
)
ament_target_dependencies(terrain_lib sensor_msgs)
target_include_directories(terrain_lib PUBLIC SYSTEM_CONTROL)

add_executable(system_control_node SYSTEM_CONTROL/SystemControlNode.cpp)
target_link_libraries(system_control_node terrain_lib)
ament_target_dependencies(system_control_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)

add_executable(planner_test SYSTEM_CONTROL/planner_test.cpp)
target_link_libraries(planner_test terrain_lib)
if(BUILD_TESTING)
    add_test(NAME planner_test COMMAND planner_test)
endif()

install(TARGETS
    parrot_node
    husky_node
    system_control_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
