#include "lidar.hpp"

#include <functional>

Lidar::Lidar(rclcpp::Node* node, const std::string& topic)
    : Sensor(node, topic), hasData_(false), hasInformation_(false)
{
    subscription_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        topic,
        rclcpp::SensorDataQoS(),
        std::bind(&Lidar::scanCallback, this, std::placeholders::_1));
}

void Lidar::scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    retrieveNewData(msg);
}

void Lidar::retrieveNewData(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!hasInformation_)
    {
        information_.angleMin = msg->angle_min;
        information_.angleMax = msg->angle_max;
        information_.angleIncrement = msg->angle_increment;

        information_.rangeMin = msg->range_min;
        information_.rangeMax = msg->range_max;
        hasInformation_ = true;
    }

    ranges_.assign(msg->ranges.begin(), msg->ranges.end());
    hasData_ = true;
}

bool Lidar::hasSensorInformation()
{
    return hasInformation_;
}

bool Lidar::hasData()
{
    return hasData_;
}

const lidarInformation& Lidar::getSensorInformation() const
{
    return information_;
}

const std::vector<double>& Lidar::getData() const
{
    return ranges_;
}