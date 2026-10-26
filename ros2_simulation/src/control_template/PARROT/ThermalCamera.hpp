#ifndef THERMAL_CAMERA_HPP
#define THERMAL_CAMERA_HPP

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/core/mat.hpp>

#include <memory>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

enum class CasualtyStatus { UNREVIEWED, CONFIRMED, DENIED };

struct Casualty
{
    std::size_t index;
    geometry_msgs::msg::PointStamped position;
    int gridX;
    int gridY;
    CasualtyStatus status{CasualtyStatus::UNREVIEWED};
};

/// Detects hot objects in the Parrot thermal image and stores confirmed
/// world-space casualty positions.
class ThermalCamera
{
public:
    ThermalCamera(rclcpp::Node* node, std::function<bool(double)> aimAtBearing);
    ~ThermalCamera();

    std::vector<Casualty> getCasualties() const;
    bool hasDetectedCasualty() const;
    bool confirmCasualty(std::size_t index);
    bool denyCasualty(std::size_t index);

private:
    void processThermal(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void onThermal(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void onDepth(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    std::optional<geometry_msgs::msg::PointStamped> locateCasualty(
        const cv::Mat& temperatures, const sensor_msgs::msg::Image& thermalMsg);
    bool confirmCandidate(const geometry_msgs::msg::PointStamped& candidate);
    void showThermal(const cv::Mat& temperatures);
    void showDepth(const cv::Mat& depth);

    rclcpp::Node* node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermalSub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub_;

    mutable std::mutex dataMutex_;
    cv::Mat latestDepth_;
    nav_msgs::msg::Odometry::ConstSharedPtr latestOdometry_;
    rclcpp::Time latestDepthStamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time latestOdometryStamp_{0, 0, RCL_ROS_TIME};
    std::optional<std::pair<double, double>> lastHeatPixel_;
    std::optional<geometry_msgs::msg::PointStamped> pendingCandidate_;

    double heatThresholdC_;
    double horizontalFovDeg_;
    double cameraHeightM_;
    double cameraForwardM_;
    double cameraPitchDeg_;
    double syncToleranceS_;
    double confirmationDistanceM_;
    int minimumBlobArea_;
    int morphologyKernelSize_;
    int confirmationFrames_;
    int matchingFrames_{0};
    bool showWindows_;
    std::vector<Casualty> casualties_;

    std::mutex thermalQueueMutex_;
    std::condition_variable thermalQueueCondition_;
    sensor_msgs::msg::Image::ConstSharedPtr pendingThermal_;
    bool shuttingDown_{false};
    std::thread thermalThread_;
    std::atomic<bool> casualtyDetected_{false};
    std::function<bool(double)> aimAtBearing_;
};

#endif
