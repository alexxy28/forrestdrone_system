#include "lidar.hpp"

#include <functional>

Lidar::Lidar(std::shared_ptr<rclcpp::Node> node, const std::string& topic)
    : Sensor(node), hasData_(false)
{
    subscription_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        topic,
        rclcpp::SensorDataQoS(),
        std::bind(&Lidar::scanCallback, this, std::placeholders::_1));
}

void Lidar::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    data_.ranges.assign(msg->ranges.begin(), msg->ranges.end());

    data_.angleMin = msg->angle_min;
    data_.angleMax = msg->angle_max;
    data_.angleIncrement = msg->angle_increment;

    data_.rangeMin = msg->range_min;
    data_.rangeMax = msg->range_max;

    hasData_ = true;
}

bool Lidar::hasData()
{
    return hasData_;
}

const lidarData& Lidar::getData() const
{
    return data_;
}