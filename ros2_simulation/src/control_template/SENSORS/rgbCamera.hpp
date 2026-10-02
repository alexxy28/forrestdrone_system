#ifndef RGB_CAMERA_HPP
#define RGB_CAMERA_HPP

#include "sensor.hpp"
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>

/// The sim camera is RGB-D, so this keeps the latest colour and depth frames.
class RgbCamera : public Sensor{
    public:
    RgbCamera(rclcpp::Node* node, const std::string& imageTopic, const std::string& depthTopic);

    bool hasData() override;

    /// Latest colour frame (CV_8UC3, BGR), empty until one arrives.
    const cv::Mat& getImage() const;
    /// Latest depth frame in metres (CV_32FC1), empty until one arrives.
    const cv::Mat& getDepth() const;
    /// Header stamps of the latest frames, in seconds.
    double getImageStamp() const;
    double getDepthStamp() const;

    private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSubscription_;
    cv::Mat image_;
    cv::Mat depth_;
    double imageStamp_{0.0};
    double depthStamp_{0.0};
};

#endif
