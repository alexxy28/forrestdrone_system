#include "thermalCamera.hpp"

#include <cv_bridge/cv_bridge.h>
#include <functional>

ThermalCamera::ThermalCamera(rclcpp::Node* node, const std::string& topic)
    : Sensor(node, topic)
{
    subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
        topic,
        rclcpp::SensorDataQoS(),
        std::bind(&ThermalCamera::imageCallback, this, std::placeholders::_1));
}

void ThermalCamera::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat frame = cv_bridge::toCvShare(msg, "mono16")->image;

    // The sim publishes mono16 in hundredths of a kelvin.
    frame.convertTo(temperatures_, CV_32F, 0.01, -273.15);
    stamp_ = rclcpp::Time(msg->header.stamp).seconds();
}

bool ThermalCamera::hasData()
{
    return !temperatures_.empty();
}

const cv::Mat& ThermalCamera::getTemperatures() const
{
    return temperatures_;
}

double ThermalCamera::getStamp() const
{
    return stamp_;
}
