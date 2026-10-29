#ifndef THERMAL_CAMERA_HPP
#define THERMAL_CAMERA_HPP

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/core/mat.hpp>

#include <memory>
#include <deque>
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
    /// One thermal frame paired with the depth frame (and odometry) from the same instant.
    void processPair(const sensor_msgs::msg::Image::ConstSharedPtr& thermalMsg, const cv::Mat& depth,
                     const nav_msgs::msg::Odometry::ConstSharedPtr& odometry);
    void processReadyPairs();
    void onThermal(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void onDepth(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    std::optional<geometry_msgs::msg::PointStamped> locateCasualty(
        const cv::Mat& temperatures, const sensor_msgs::msg::Image& thermalMsg,
        const cv::Mat& depth, const nav_msgs::msg::Odometry& odometry);
    void addObservation(const cv::Point3d& centre, double size, double elongation, double stamp);

    /// Every hot spot seen with usable depth: its recent shape measurements.
    struct Track
    {
        std::vector<double> xs, ys, zs, sizes, elongations;
        double lastSeen{0.0};
        bool reported{false};
        int loggedAt{-1};
    };
    std::vector<Track> tracks_;
    double lastTrackSummary_{0.0};
    void showThermal(const cv::Mat& temperatures);
    void showDepth(const cv::Mat& depth);

    rclcpp::Node* node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermalSub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub_;

    mutable std::mutex dataMutex_;
    // Recent sensor data, so each thermal frame can be paired with the depth
    // frame and odometry from the SAME instant (a moving, turning drone smeared
    // shapes when they were up to 0.15 s apart). Guarded by dataMutex_.
    std::deque<std::pair<rclcpp::Time, cv::Mat>> depthFrames_;
    std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_;
    rclcpp::Time lastPairedDepth_{0, 0, RCL_ROS_TIME};
    double odometryToleranceS_;
    double minHumanViewFraction_;

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
    bool showWindows_;
    double minHumanSizeM_;
    double maxHumanSizeM_;
    double minHumanElongation_;
    int minClassifyPixels_;
    double maxDepthM_;
    double minViewElevationDeg_;
    std::vector<Casualty> casualties_;

    std::mutex thermalQueueMutex_;
    std::condition_variable thermalQueueCondition_;
    std::deque<sensor_msgs::msg::Image::ConstSharedPtr> thermalFrames_;   // guarded by thermalQueueMutex_
    bool newFrame_{false};                                                 // guarded by thermalQueueMutex_
    bool shuttingDown_{false};
    std::thread thermalThread_;
    std::atomic<bool> casualtyDetected_{false};
    std::function<bool(double)> aimAtBearing_;   // kept for the ParrotNode API; aiming is no longer
                                                 // needed (3D projection works anywhere in the image)
};

#endif
