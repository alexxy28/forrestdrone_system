#ifndef LIDAR_HPP
#define LIDAR_HPP

#include "sensor.hpp"
#include "dataStructures.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>
#include <vector>
#include <string>

class Lidar : public Sensor{
    public:
    Lidar(std::shared_ptr<rclcpp::Node> node, const std::string& topic);

    bool hasData() override;

    const lidarData& getData() const;


    protected:

    private:
     void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    lidarData data_;

    bool hasData_;

};

#endif
