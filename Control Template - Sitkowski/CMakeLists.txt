cmake_minimum_required(VERSION 3.8)

project(control_template)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(lidar_test
    src/main.cpp
    src/lidar.cpp
    src/sensor.cpp
)

ament_target_dependencies(lidar_test
    rclcpp
    sensor_msgs
)

target_include_directories(lidar_test PUBLIC
    include
)

install(TARGETS
    lidar_test
    DESTINATION lib/${PROJECT_NAME}
)