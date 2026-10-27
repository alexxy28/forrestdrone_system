#include "ThermalCamera.hpp"

#include "dataStructures.hpp"

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>
#include <string>
#include <vector>

namespace {

constexpr double casualtyGridResolutionM = 3.0;

double median(std::vector<double> values)
{
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    const double upper = *middle;
    if (values.size() % 2 != 0) {
        return upper;
    }
    return (upper + *std::max_element(values.begin(), middle)) / 2.0;
}

double percentile98(const cv::Mat& values)
{
    std::vector<float> sorted;
    sorted.reserve(values.total());
    for (int row = 0; row < values.rows; ++row) {
        const float* input = values.ptr<float>(row);
        for (int column = 0; column < values.cols; ++column) {
            if (std::isfinite(input[column])) {
                sorted.push_back(input[column]);
            }
        }
    }
    if (sorted.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const auto index = static_cast<std::size_t>(
        std::floor(0.98 * static_cast<double>(sorted.size() - 1)));
    std::nth_element(sorted.begin(), sorted.begin() + index, sorted.end());
    return sorted[index];
}

cv::Mat colourMap(const cv::Mat& values, int map)
{
    cv::Mat validMask = values == values;
    double minimum = 0.0;
    double maximum = 0.0;
    cv::minMaxLoc(values, &minimum, &maximum, nullptr, nullptr, validMask);

    cv::Mat normalized = cv::Mat::zeros(values.size(), CV_8UC1);
    if (maximum > minimum) {
        values.convertTo(normalized, CV_8UC1, 255.0 / (maximum - minimum),
                         -minimum * 255.0 / (maximum - minimum));
        normalized.setTo(0, ~validMask);
    }
    cv::Mat output;
    cv::applyColorMap(normalized, output, map);
    return output;
}

}  // namespace

ThermalCamera::ThermalCamera(rclcpp::Node* node, std::function<bool(double)> aimAtBearing)
    : node_(node),
      aimAtBearing_(std::move(aimAtBearing)),
      heatThresholdC_(node_->declare_parameter<double>("heat_threshold_c", 30.0)),
      horizontalFovDeg_(node_->declare_parameter<double>("thermal_horizontal_fov_deg", 120.0)),
      cameraHeightM_(node_->declare_parameter<double>("camera_height_m", 0.25)),
      cameraForwardM_(node_->declare_parameter<double>("camera_forward_m", 0.25)),
      cameraPitchDeg_(node_->declare_parameter<double>("camera_pitch_deg", 45.0)),  // matches camera_joint rpy in parrot.urdf.xacro
      syncToleranceS_(node_->declare_parameter<double>("camera_sync_tolerance_s", 0.5)),
      confirmationDistanceM_(node_->declare_parameter<double>("detection_confirmation_distance_m", 2.0)),
      minimumBlobArea_(node_->declare_parameter<int>("minimum_heat_blob_area", 1)),
      morphologyKernelSize_(node_->declare_parameter<int>("thermal_morphology_kernel_size", 3)),
      confirmationFrames_(node_->declare_parameter<int>("detection_confirmation_frames", 3)),
      showWindows_(node_->declare_parameter<bool>("show_camera_windows", false))
{
    const auto thermalTopic = node_->declare_parameter<std::string>(
        "thermal_topic", topics::PARROT_THERMAL);
    const auto depthTopic = node_->declare_parameter<std::string>(
        "depth_topic", topics::PARROT_DEPTH);
    const auto odometryTopic = node_->declare_parameter<std::string>(
        "detection_odometry_topic", topics::PARROT_ODOM);
    thermalSub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        thermalTopic, rclcpp::SensorDataQoS(),
        std::bind(&ThermalCamera::onThermal, this, std::placeholders::_1));
    depthSub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        depthTopic, rclcpp::SensorDataQoS(),
        std::bind(&ThermalCamera::onDepth, this, std::placeholders::_1));
    odometrySub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odometryTopic, rclcpp::SensorDataQoS(),
        std::bind(&ThermalCamera::onOdometry, this, std::placeholders::_1));

    if (showWindows_) {
        cv::namedWindow("Thermal Camera", cv::WINDOW_NORMAL);
        cv::namedWindow("Depth Camera", cv::WINDOW_NORMAL);
    }
    thermalThread_ = std::thread([this] {
        while (true) {
            sensor_msgs::msg::Image::ConstSharedPtr msg;
            {
                std::unique_lock<std::mutex> lock(thermalQueueMutex_);
                thermalQueueCondition_.wait(lock, [this] {
                    return shuttingDown_ || pendingThermal_;
                });
                if (shuttingDown_ && !pendingThermal_) {
                    return;
                }
                msg = std::move(pendingThermal_);
            }
            processThermal(msg);
        }
    });
    RCLCPP_INFO(node_->get_logger(),
                "Thermal detection listening on %s and %s (confirmation: %d frames)",
                thermalTopic.c_str(), depthTopic.c_str(), confirmationFrames_);
}

ThermalCamera::~ThermalCamera()
{
    {
        std::lock_guard<std::mutex> lock(thermalQueueMutex_);
        shuttingDown_ = true;
    }
    thermalQueueCondition_.notify_one();
    if (thermalThread_.joinable()) {
        thermalThread_.join();
    }
    if (showWindows_) {
        cv::destroyWindow("Thermal Camera");
        cv::destroyWindow("Depth Camera");
    }
}

void ThermalCamera::onDepth(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    try {
        const auto image = cv_bridge::toCvShare(msg, msg->encoding);
        if (image->image.type() == CV_32FC1) {
            std::lock_guard<std::mutex> lock(dataMutex_);
            latestDepth_ = image->image.clone();
        } else if (image->image.type() == CV_16UC1) {
            std::lock_guard<std::mutex> lock(dataMutex_);
            image->image.convertTo(latestDepth_, CV_32FC1, 0.001);
        } else {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "Unsupported depth encoding: %s", msg->encoding.c_str());
            return;
        }
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            latestDepthStamp_ = rclcpp::Time(msg->header.stamp);
        }
        if (showWindows_) {
            showDepth(image->image);
        }
    } catch (const cv_bridge::Exception& error) {
        RCLCPP_ERROR(node_->get_logger(), "Could not decode depth frame: %s", error.what());
    }
}

void ThermalCamera::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    latestOdometry_ = msg;
    latestOdometryStamp_ = rclcpp::Time(msg->header.stamp);
}

void ThermalCamera::onThermal(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    {
        std::lock_guard<std::mutex> lock(thermalQueueMutex_);
        pendingThermal_ = msg;
    }
    thermalQueueCondition_.notify_one();
}

void ThermalCamera::processThermal(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    try {
        const auto image = cv_bridge::toCvShare(msg, msg->encoding);
        if (image->image.type() != CV_16UC1) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "Thermal detection requires 16-bit mono data, received %s",
                                 msg->encoding.c_str());
            return;
        }

        cv::Mat temperatures;
        image->image.convertTo(temperatures, CV_32FC1, 0.01, -273.15);
        if (showWindows_) {
            showThermal(temperatures);
        }
        const auto candidate = locateCasualty(temperatures, *msg);
        if (!candidate) {
            std::lock_guard<std::mutex> lock(dataMutex_);
            matchingFrames_ = 0;
            pendingCandidate_.reset();
            return;
        }
        if (!confirmCandidate(*candidate)) {
            return;
        }

        std::lock_guard<std::mutex> lock(dataMutex_);
        const auto alreadyKnown = std::find_if(
            casualties_.begin(), casualties_.end(), [&](const auto& casualty) {
                return std::hypot(candidate->point.x - casualty.position.point.x,
                                  candidate->point.y - casualty.position.point.y) <= confirmationDistanceM_;
            });
        if (alreadyKnown == casualties_.end()) {
            const std::size_t index = casualties_.size() + 1;
            casualties_.push_back({
                index,
                *candidate,
                static_cast<int>(std::floor(
                    (candidate->point.x + casualtyGridResolutionM / 2.0) /
                    casualtyGridResolutionM)),
                static_cast<int>(std::floor(
                    (candidate->point.y + casualtyGridResolutionM / 2.0) /
                    casualtyGridResolutionM))});
            RCLCPP_INFO(node_->get_logger(),
                        "Detected casualty %zu in 3m cell (%d, %d) at (%.2f, %.2f, %.2f) in %s",
                        index, casualties_.back().gridX, casualties_.back().gridY,
                        candidate->point.x, candidate->point.y, candidate->point.z,
                        candidate->header.frame_id.c_str());
            casualtyDetected_.store(true, std::memory_order_release);
        }
    } catch (const cv_bridge::Exception& error) {
        RCLCPP_ERROR(node_->get_logger(), "Could not decode thermal frame: %s", error.what());
    }
}

std::vector<Casualty> ThermalCamera::getCasualties() const
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    return casualties_;
}

bool ThermalCamera::hasDetectedCasualty() const
{
    return casualtyDetected_.load(std::memory_order_acquire);
}

bool ThermalCamera::confirmCasualty(std::size_t index)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    if (index == 0 || index > casualties_.size()) {
        return false;
    }
    casualties_[index - 1].status = CasualtyStatus::CONFIRMED;
    return true;
}

bool ThermalCamera::denyCasualty(std::size_t index)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    if (index == 0 || index > casualties_.size()) {
        return false;
    }
    casualties_[index - 1].status = CasualtyStatus::DENIED;
    return true;
}

std::optional<geometry_msgs::msg::PointStamped> ThermalCamera::locateCasualty(
    const cv::Mat& temperatures, const sensor_msgs::msg::Image& thermalMsg)
{
    cv::Mat latestDepth;
    nav_msgs::msg::Odometry::ConstSharedPtr latestOdometry;
    rclcpp::Time latestDepthStamp;
    rclcpp::Time latestOdometryStamp;
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        latestDepth = latestDepth_.clone();
        latestOdometry = latestOdometry_;
        latestDepthStamp = latestDepthStamp_;
        latestOdometryStamp = latestOdometryStamp_;
    }
    if (latestDepth.empty() || !latestOdometry) {
        return std::nullopt;
    }
    const rclcpp::Time thermalStamp(thermalMsg.header.stamp);
    if (std::abs((thermalStamp - latestDepthStamp).seconds()) > syncToleranceS_ ||
        std::abs((thermalStamp - latestOdometryStamp).seconds()) > syncToleranceS_) {
        return std::nullopt;
    }

    const double adaptiveThreshold = std::max(heatThresholdC_, percentile98(temperatures));
    if (!std::isfinite(adaptiveThreshold)) {
        return std::nullopt;
    }
    cv::Mat hotMask;
    cv::compare(temperatures, adaptiveThreshold, hotMask, cv::CMP_GE);
    const int kernelSize = std::max(1, morphologyKernelSize_ | 1);
    const cv::Mat kernel = cv::Mat::ones(kernelSize, kernelSize, CV_8UC1);
    cv::morphologyEx(hotMask, hotMask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(hotMask, hotMask, cv::MORPH_CLOSE, kernel);

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int componentCount = cv::connectedComponentsWithStats(
        hotMask, labels, stats, centroids, 8, CV_32S);
    if (componentCount <= 1) {
        return std::nullopt;
    }

    cv::Mat depth;
    if (latestDepth.size() == temperatures.size()) {
        depth = latestDepth;
    } else {
        cv::resize(latestDepth, depth, temperatures.size(), 0.0, 0.0, cv::INTER_NEAREST);
    }

    int bestComponent = -1;
    double bestScore = -std::numeric_limits<double>::infinity();
    for (int component = 1; component < componentCount; ++component) {
        const int area = stats.at<int>(component, cv::CC_STAT_AREA);
        if (area < minimumBlobArea_) {
            continue;
        }
        int validDepthCount = 0;
        double maximumTemperature = -std::numeric_limits<double>::infinity();
        double temperatureSum = 0.0;
        std::vector<double> xs;
        std::vector<double> ys;
        for (int row = 0; row < labels.rows; ++row) {
            const int* labelRow = labels.ptr<int>(row);
            const float* temperatureRow = temperatures.ptr<float>(row);
            const float* depthRow = depth.ptr<float>(row);
            for (int column = 0; column < labels.cols; ++column) {
                if (labelRow[column] != component) {
                    continue;
                }
                maximumTemperature = std::max(maximumTemperature,
                                               static_cast<double>(temperatureRow[column]));
                temperatureSum += temperatureRow[column];
                if (std::isfinite(depthRow[column]) && depthRow[column] > 0.1F) {
                    ++validDepthCount;
                    xs.push_back(column);
                    ys.push_back(row);
                }
            }
        }
        const double validRatio = static_cast<double>(validDepthCount) / area;
        if (validRatio < 0.25) {
            continue;
        }
        const double pixelX = median(xs);
        const double pixelY = median(ys);
        double score = 2.0 * maximumTemperature + temperatureSum / area +
                       8.0 * std::log1p(area) + 20.0 * validRatio;
        if (lastHeatPixel_) {
            score -= std::min(std::hypot(pixelX - lastHeatPixel_->first,
                                         pixelY - lastHeatPixel_->second) / 10.0,
                              20.0);
        }
        if (score > bestScore) {
            bestScore = score;
            bestComponent = component;
        }
    }
    if (bestComponent < 0) {
        return std::nullopt;
    }

    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> depths;
    for (int row = 0; row < labels.rows; ++row) {
        const int* labelRow = labels.ptr<int>(row);
        const float* depthRow = depth.ptr<float>(row);
        for (int column = 0; column < labels.cols; ++column) {
            if (labelRow[column] == bestComponent && std::isfinite(depthRow[column]) &&
                depthRow[column] > 0.1F) {
                xs.push_back(column);
                ys.push_back(row);
                depths.push_back(depthRow[column]);
            }
        }
    }
    const double pixelX = median(xs);
    const double pixelY = median(ys);
    const double distance = median(depths);

    const double pi = std::acos(-1.0);
    const double focal = temperatures.cols /
        (2.0 * std::tan(horizontalFovDeg_ * pi / 360.0));
    const double bearing = std::atan2(-(pixelX - (temperatures.cols - 1) / 2.0),
                                      focal);
    if (aimAtBearing_ && !aimAtBearing_(bearing)) {
        return std::nullopt;
    }
    lastHeatPixel_ = std::make_pair(pixelX, pixelY);
    const double centerX = (temperatures.cols - 1) / 2.0;
    const double centerY = (temperatures.rows - 1) / 2.0;
    const double cameraX = distance;
    const double cameraY = -(pixelX - centerX) * distance / focal;
    const double cameraZ = -(pixelY - centerY) * distance / focal;
    const double cameraPitch = cameraPitchDeg_ * pi / 180.0;
    const double baseX = std::cos(cameraPitch) * cameraX +
                         std::sin(cameraPitch) * cameraZ + cameraForwardM_;
    const double baseY = cameraY;
    const double baseZ = -std::sin(cameraPitch) * cameraX +
                         std::cos(cameraPitch) * cameraZ + cameraHeightM_;

    const auto& pose = latestOdometry->pose.pose;
    const auto& q = pose.orientation;
    const double r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double r01 = 2.0 * (q.x * q.y - q.z * q.w);
    const double r02 = 2.0 * (q.x * q.z + q.y * q.w);
    const double r10 = 2.0 * (q.x * q.y + q.z * q.w);
    const double r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
    const double r12 = 2.0 * (q.y * q.z - q.x * q.w);
    const double r20 = 2.0 * (q.x * q.z - q.y * q.w);
    const double r21 = 2.0 * (q.y * q.z + q.x * q.w);
    const double r22 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);

    geometry_msgs::msg::PointStamped result;
    result.header.stamp = thermalMsg.header.stamp;
    result.header.frame_id = latestOdometry->header.frame_id;
    result.point.x = pose.position.x + r00 * baseX + r01 * baseY + r02 * baseZ;
    result.point.y = pose.position.y + r10 * baseX + r11 * baseY + r12 * baseZ;
    result.point.z = pose.position.z + r20 * baseX + r21 * baseY + r22 * baseZ;
    return result;
}

bool ThermalCamera::confirmCandidate(const geometry_msgs::msg::PointStamped& candidate)
{
    if (!pendingCandidate_) {
        pendingCandidate_ = candidate;
        matchingFrames_ = 1;
    } else {
        const double distance = std::hypot(candidate.point.x - pendingCandidate_->point.x,
                                           candidate.point.y - pendingCandidate_->point.y);
        if (distance > confirmationDistanceM_) {
            pendingCandidate_ = candidate;
            matchingFrames_ = 1;
        } else {
            pendingCandidate_ = candidate;
            ++matchingFrames_;
        }
    }
    return matchingFrames_ >= std::max(1, confirmationFrames_);
}

void ThermalCamera::showThermal(const cv::Mat& temperatures)
{
    cv::imshow("Thermal Camera", colourMap(temperatures, cv::COLORMAP_INFERNO));
    cv::waitKey(1);
}

void ThermalCamera::showDepth(const cv::Mat& depth)
{
    cv::Mat display = depth.clone();
    for (int row = 0; row < display.rows; ++row) {
        float* values = display.ptr<float>(row);
        for (int column = 0; column < display.cols; ++column) {
            if (!std::isfinite(values[column]) || values[column] <= 0.1F) {
                values[column] = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    cv::imshow("Depth Camera", colourMap(display, cv::COLORMAP_TURBO));
    cv::waitKey(1);
}
