#include "lidar.hpp"

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <memory>

int main(int argc, char* argv[])
{
    // Start ROS 2
    rclcpp::init(argc, argv);

    // Create ROS node
    auto node = std::make_shared<rclcpp::Node>("lidar_test");

    // Create ONE LiDAR
    Lidar lidar(
        node,
        "/husky1/scan"
    );

    std::cout << "Waiting for LiDAR scan..." << std::endl;

    // Wait until a LiDAR message has been received
    while (rclcpp::ok() && !lidar.hasData())
    {
        rclcpp::spin_some(node);
    }

    if (lidar.hasData())
    {
        const LidarData& data = lidar.getData();

        std::cout << "\nLiDAR scan received!" << std::endl;

        std::cout << "Number of measurements: "
                  << data.ranges.size()
                  << std::endl;

        std::cout << "Angle minimum: "
                  << data.angle_min
                  << std::endl;

        std::cout << "Angle maximum: "
                  << data.angle_max
                  << std::endl;

        std::cout << "Angle increment: "
                  << data.angle_increment
                  << std::endl;

        std::cout << "\nRanges:" << std::endl;

        for (size_t i = 0; i < data.ranges.size(); ++i)
        {
            std::cout << i
                      << ": "
                      << data.ranges[i]
                      << " m"
                      << std::endl;
        }
    }

    rclcpp::shutdown();

    return 0;
}