#ifndef LIDAR_HPP
#define LIDAR_HPP

#include "sensor.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>
#include <vector>
#include <string>

class Lidar : public Sensor{
    public:
    Lidar(std::shared_ptr<rclcpp::Node> node, const std::string& topic);

    bool hasData() const override;

    const lidarData& getData() const;


    protected:

    private:
     void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    LidarData data_;

    bool hasData_;

};

#endif
