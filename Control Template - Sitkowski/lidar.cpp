#include "lidar.hpp"

#include <functional>

Lidar::Lidar(std::shared_ptr<rclcpp::Node> node, const std::string& topic)
    : Sensor(node), has_data_(false)
{
    subscription_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        topic,
        rclcpp::SensorDataQoS(),
        std::bind(&Lidar::scanCallback, this, std::placeholders::_1));
}

void Lidar::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    data_.ranges = msg->ranges;

    data_.angle_min = msg->angle_min;
    data_.angle_max = msg->angle_max;
    data_.angle_increment = msg->angle_increment;

    data_.range_min = msg->range_min;
    data_.range_max = msg->range_max;

    has_data_ = true;
}

bool Lidar::hasData() const
{
    return has_data_;
}

const LidarData& Lidar::getData() const
{
    return data_;
}