#ifndef THERMAL_CAMERA_HPP
#define THERMAL_CAMERA_HPP

#include "sensor.hpp"
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>

class ThermalCamera : public Sensor{
    public:
    ThermalCamera(rclcpp::Node* node, const std::string& topic);

    bool hasData() override;

    /// Latest frame in degrees C (CV_32FC1), empty until the first frame arrives.
    const cv::Mat& getTemperatures() const;
    /// Header stamp of the latest frame, in seconds.
    double getStamp() const;

    private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::Mat temperatures_;
    double stamp_{0.0};
};

#endif
