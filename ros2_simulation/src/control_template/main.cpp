#include "husky.hpp"

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <memory>

int main(int argc, char* argv[])
{
    // Start ROS 2
    rclcpp::init(argc, argv);

    // Create ROS node
    auto node = std::make_shared<rclcpp::Node>("lidar_test");

    Husky husky(node);
    Sensor& lidar = husky.getSensor();

    std::cout << "Waiting for LiDAR scan..." << std::endl;


    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    for (int i = 0; i < 20; i++){
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        const auto& data = lidar.getData();
        if (data.empty()) {
            continue;
        }
        double distance = data.front();

        std::cout  <<  distance << std::endl;
    }

    rclcpp::shutdown();

    return 0;
}