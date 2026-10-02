#include "rgbCamera.hpp"

#include <cv_bridge/cv_bridge.h>
#include <functional>

RgbCamera::RgbCamera(rclcpp::Node* node, const std::string& imageTopic, const std::string& depthTopic)
    : Sensor(node, imageTopic)
{
    imageSubscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
        imageTopic,
        rclcpp::SensorDataQoS(),
        std::bind(&RgbCamera::imageCallback, this, std::placeholders::_1));

    depthSubscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
        depthTopic,
        rclcpp::SensorDataQoS(),
        std::bind(&RgbCamera::depthCallback, this, std::placeholders::_1));
}

void RgbCamera::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    imageStamp_ = rclcpp::Time(msg->header.stamp).seconds();
}

void RgbCamera::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    depth_ = cv_bridge::toCvCopy(msg, "32FC1")->image;
    depthStamp_ = rclcpp::Time(msg->header.stamp).seconds();
}

bool RgbCamera::hasData()
{
    return !image_.empty() && !depth_.empty();
}

const cv::Mat& RgbCamera::getImage() const
{
    return image_;
}

const cv::Mat& RgbCamera::getDepth() const
{
    return depth_;
}

double RgbCamera::getImageStamp() const
{
    return imageStamp_;
}

double RgbCamera::getDepthStamp() const
{
    return depthStamp_;
}
