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
      // Must match camera_joint pitch in urdf_parrot/parrot.urdf.xacro (0.785398 rad).
      cameraPitchDeg_(node_->declare_parameter<double>("camera_pitch_deg", 45.0)),
      // Thermal renders every 0.1 s and depth every 0.2 s (5 Hz) on the sim
      // clock, so each depth frame has a thermal frame from the same instant.
      // The 0.04 s allowance covers render/stamp jitter (measured in Gazebo);
      // the old 0.15 s let a turning drone pair frames ~9 deg apart.
      syncToleranceS_(node_->declare_parameter<double>("camera_sync_tolerance_s", 0.04)),
      confirmationDistanceM_(node_->declare_parameter<double>("detection_confirmation_distance_m", 2.0)),
      minimumBlobArea_(node_->declare_parameter<int>("minimum_heat_blob_area", 1)),
      morphologyKernelSize_(node_->declare_parameter<int>("thermal_morphology_kernel_size", 3)),
      // Views of the same spot needed before its median shape is trusted.
      // Views of one spot needed before deciding (4 let the animal through once
      // in Gazebo when its first views happened to be large).
      confirmationFrames_(node_->declare_parameter<int>("detection_confirmation_frames", 6)),
      showWindows_(node_->declare_parameter<bool>("show_camera_windows", false)),
      // Human vs other heat sources (e.g. the 390 K demo_animal): real size of
      // the hot blob from depth, longest of horizontal length and height.
      // Thresholds on the MEDIAN over >= detection_confirmation_frames views.
      // Calibrated in Gazebo (tools/thermal_shape_sweep.py, 27 Oct, 36-39 views
      // each): human median 0.97 m (p10 0.81), 2.47x (p10 1.61); demo_animal
      // median 0.56 m (max single view 0.67), 1.21x (p90 1.52).
      minHumanSizeM_(node_->declare_parameter<double>("min_human_size_m", 0.75)),
      // Generous upper bound: at grazing angles depth noise stretches a 1.7 m
      // person to ~3 m (seen in Gazebo while the drone was still climbing).
      maxHumanSizeM_(node_->declare_parameter<double>("max_human_size_m", 4.0)),
      minHumanElongation_(node_->declare_parameter<double>("min_human_elongation", 1.6)),
      minClassifyPixels_(node_->declare_parameter<int>("min_classify_pixels", 60)),
      maxDepthM_(node_->declare_parameter<double>("max_detection_depth_m", 39.0)),
      minViewElevationDeg_(node_->declare_parameter<double>("min_view_elevation_deg", 20.0)),
      odometryToleranceS_(node_->declare_parameter<double>("odometry_sync_tolerance_s", 0.03)),
      // Besides the medians, this share of the individual views must look human.
      minHumanViewFraction_(node_->declare_parameter<double>("min_human_view_fraction", 0.6))
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
            {
                std::unique_lock<std::mutex> lock(thermalQueueMutex_);
                thermalQueueCondition_.wait(lock, [this] { return shuttingDown_ || newFrame_; });
                if (shuttingDown_) {
                    return;
                }
                newFrame_ = false;
            }
            processReadyPairs();
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
        cv::Mat depth;
        if (image->image.type() == CV_32FC1) {
            depth = image->image.clone();
        } else if (image->image.type() == CV_16UC1) {
            image->image.convertTo(depth, CV_32FC1, 0.001);
        } else {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "Unsupported depth encoding: %s", msg->encoding.c_str());
            return;
        }
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            depthFrames_.emplace_back(rclcpp::Time(msg->header.stamp), depth);
            while (depthFrames_.size() > 6) {
                depthFrames_.pop_front();
            }
        }
        {
            std::lock_guard<std::mutex> lock(thermalQueueMutex_);
            newFrame_ = true;
        }
        thermalQueueCondition_.notify_one();
        if (showWindows_) {
            showDepth(depth);
        }
    } catch (const cv_bridge::Exception& error) {
        RCLCPP_ERROR(node_->get_logger(), "Could not decode depth frame: %s", error.what());
    }
}

void ThermalCamera::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    odometry_.push_back(msg);
    while (odometry_.size() > 300) {
        odometry_.pop_front();
    }
}

void ThermalCamera::onThermal(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    {
        std::lock_guard<std::mutex> lock(thermalQueueMutex_);
        thermalFrames_.push_back(msg);
        while (thermalFrames_.size() > 20) {
            thermalFrames_.pop_front();
        }
        newFrame_ = true;
    }
    thermalQueueCondition_.notify_one();
}

// Pair every new depth frame with the thermal frame from the same instant (and
// the closest odometry), then process that pair. Depth frames that can never be
// paired (their thermal frame is gone) are skipped.
void ThermalCamera::processReadyPairs()
{
    std::vector<std::pair<rclcpp::Time, cv::Mat>> depths;
    std::vector<nav_msgs::msg::Odometry::ConstSharedPtr> odoms;
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        for (const auto& d : depthFrames_) {
            if (d.first > lastPairedDepth_) {
                depths.push_back(d);
            }
        }
        odoms.assign(odometry_.begin(), odometry_.end());
    }
    std::vector<sensor_msgs::msg::Image::ConstSharedPtr> thermals;
    {
        std::lock_guard<std::mutex> lock(thermalQueueMutex_);
        thermals.assign(thermalFrames_.begin(), thermalFrames_.end());
    }
    if (thermals.empty()) {
        return;
    }
    const rclcpp::Time newestThermal(thermals.back()->header.stamp);

    for (const auto& [depthStamp, depth] : depths) {
        sensor_msgs::msg::Image::ConstSharedPtr thermal;
        for (const auto& t : thermals) {
            if (std::abs((rclcpp::Time(t->header.stamp) - depthStamp).seconds()) <= syncToleranceS_) {
                thermal = t;
                break;
            }
        }
        if (!thermal) {
            if ((newestThermal - depthStamp).seconds() > 0.5) {
                std::lock_guard<std::mutex> lock(dataMutex_);
                lastPairedDepth_ = depthStamp;   // its thermal frame will never come
            }
            continue;   // otherwise wait: the thermal frame may still be on its way
        }
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            lastPairedDepth_ = depthStamp;
        }
        nav_msgs::msg::Odometry::ConstSharedPtr odom;
        double best = std::numeric_limits<double>::infinity();
        for (const auto& o : odoms) {
            const double dt = std::abs((rclcpp::Time(o->header.stamp) - depthStamp).seconds());
            if (dt < best) {
                best = dt;
                odom = o;
            }
        }
        if (!odom || best > odometryToleranceS_) {
            continue;
        }
        processPair(thermal, depth, odom);
    }
}

void ThermalCamera::processPair(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const cv::Mat& depthFrame,
                                const nav_msgs::msg::Odometry::ConstSharedPtr& odometry)
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
        // A candidate is a heat source whose track has enough observations and is
        // human-shaped on the median of all of them (see locateCasualty).
        const auto candidate = locateCasualty(temperatures, *msg, depthFrame, *odometry);
        if (!candidate) {
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
    const cv::Mat& temperatures, const sensor_msgs::msg::Image& thermalMsg,
    const cv::Mat& latestDepth, const nav_msgs::msg::Odometry& odometry)
{
    const rclcpp::Time thermalStamp(thermalMsg.header.stamp);
    const nav_msgs::msg::Odometry* latestOdometry = &odometry;

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

    // ---- camera model: pixel + depth -> world point --------------------------
    const double pi = std::acos(-1.0);
    const double focal = temperatures.cols /
        (2.0 * std::tan(horizontalFovDeg_ * pi / 360.0));
    const double centerX = (temperatures.cols - 1) / 2.0;
    const double centerY = (temperatures.rows - 1) / 2.0;
    const double cameraPitch = cameraPitchDeg_ * pi / 180.0;
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
    // Depth is measured along the optical axis (camera x).
    auto toWorld = [&](double column, double row, double d) {
        const double cameraX = d;
        const double cameraY = -(column - centerX) * d / focal;
        const double cameraZ = -(row - centerY) * d / focal;
        const double baseX = std::cos(cameraPitch) * cameraX +
                             std::sin(cameraPitch) * cameraZ + cameraForwardM_;
        const double baseY = cameraY;
        const double baseZ = -std::sin(cameraPitch) * cameraX +
                             std::cos(cameraPitch) * cameraZ + cameraHeightM_;
        return cv::Point3d(pose.position.x + r00 * baseX + r01 * baseY + r02 * baseZ,
                           pose.position.y + r10 * baseX + r11 * baseY + r12 * baseZ,
                           pose.position.z + r20 * baseX + r21 * baseY + r22 * baseZ);
    };

    // ---- group the hot pixels that have a valid depth, per blob --------------
    struct Pixel { int column, row; float depth; };
    std::vector<std::vector<Pixel>> blobPixels(componentCount);
    for (int row = 0; row < labels.rows; ++row) {
        const int* labelRow = labels.ptr<int>(row);
        const float* depthRow = depth.ptr<float>(row);
        for (int column = 0; column < labels.cols; ++column) {
            const int component = labelRow[column];
            const float d = depthRow[column];
            if (component != 0 && std::isfinite(d) && d > 0.1F && d < maxDepthM_) {
                blobPixels[component].push_back({column, row, d});
            }
        }
    }
    // Pixels on a blob's edge can see past the object to the ground behind it,
    // which stretches the shape along the line of sight. Keep only depths close
    // to the blob's median depth, then convert to world points.
    std::vector<std::vector<cv::Point3d>> blobPoints(componentCount);
    for (int component = 1; component < componentCount; ++component) {
        auto& pixels = blobPixels[component];
        if (pixels.empty()) {
            continue;
        }
        std::vector<double> depths;
        for (const auto& px : pixels) {
            depths.push_back(px.depth);
        }
        const double medianDepth = median(depths);
        const double band = std::max(0.75, 0.08 * medianDepth);
        for (const auto& px : pixels) {
            if (std::abs(px.depth - medianDepth) <= band) {
                blobPoints[component].push_back(toWorld(px.column, px.row, px.depth));
            }
        }
    }

    // ---- measure each blob's real shape and add it to that spot's track ------
    for (int component = 1; component < componentCount; ++component) {
        const int area = stats.at<int>(component, cv::CC_STAT_AREA);
        if (area < minimumBlobArea_) {
            continue;
        }
        const auto& points = blobPoints[component];
        const double validRatio = static_cast<double>(points.size()) / area;
        if (points.size() < 3 || validRatio < 0.25) {
            continue;   // too far away for the depth camera: can't size or place it yet
        }
        // A blob cut off by the image edge looks smaller than it is: wait for a full view.
        const int left = stats.at<int>(component, cv::CC_STAT_LEFT);
        const int top = stats.at<int>(component, cv::CC_STAT_TOP);
        const int boxWidth = stats.at<int>(component, cv::CC_STAT_WIDTH);
        const int boxHeight = stats.at<int>(component, cv::CC_STAT_HEIGHT);
        if (left <= 0 || top <= 0 || left + boxWidth >= labels.cols || top + boxHeight >= labels.rows) {
            continue;
        }

        // Too few depth pixels (far away): the shape can't be judged yet. Leave
        // it undecided; the search pattern brings the drone closer later.
        if (static_cast<int>(points.size()) < minClassifyPixels_) {
            continue;
        }

        // Physical shape: horizontal spread along/across the principal axis, and height.
        std::vector<double> xs, ys, zs;
        for (const auto& p : points) {
            xs.push_back(p.x);
            ys.push_back(p.y);
            zs.push_back(p.z);
        }
        const double mx = median(xs);
        const double my = median(ys);
        double sxx = 0.0, syy = 0.0, sxy = 0.0;
        for (const auto& p : points) {
            sxx += (p.x - mx) * (p.x - mx);
            syy += (p.y - my) * (p.y - my);
            sxy += (p.x - mx) * (p.y - my);
        }
        const double axis = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
        std::vector<double> along;
        for (const auto& p : points) {
            along.push_back((p.x - mx) * std::cos(axis) + (p.y - my) * std::sin(axis));
        }
        auto spread = [](std::vector<double> v) {   // 5th-95th percentile range
            std::sort(v.begin(), v.end());
            const auto at = [&](double f) { return v[static_cast<std::size_t>(f * (v.size() - 1))]; };
            return at(0.95) - at(0.05);
        };
        std::vector<double> across;
        for (const auto& p : points) {
            across.push_back(-(p.x - mx) * std::sin(axis) + (p.y - my) * std::cos(axis));
        }
        const double length = spread(along);
        const double width = std::max(spread(across), 0.05);
        const double tall = spread(zs);
        // A lying person is long and narrow; a standing one is tall and narrow.
        // An animal (or a hot rock) is roughly round. Measured in Gazebo on
        // 27 Oct: human 0.9-1.5 m long, 2.0-4.4x longer than wide (from <9 m);
        // demo_animal 0.6 m, 1.1-1.2x.
        const double size = std::max(length, tall);
        const double elongation = size / width;
        const cv::Point3d centre(mx, my, median(zs));

        // Seen at a shallow angle, depth noise stretches anything along the line
        // of sight (a 1.7 m person measured 3.2 m; the round animal could pass as
        // elongated). Don't judge shape from such views: leave it undecided.
        const double cameraWorldZ = pose.position.z + r20 * cameraForwardM_ + r22 * cameraHeightM_;
        const double horizontal = std::hypot(centre.x - pose.position.x, centre.y - pose.position.y);
        const double elevationDeg = std::atan2(cameraWorldZ - centre.z, horizontal) * 180.0 / pi;
        if (elevationDeg < minViewElevationDeg_) {
            continue;
        }

        // One observation of this heat source. Single frames are noisy (an odd
        // view of the round animal measured 0.89 m / 1.5x once), so the decision
        // is made on the median over many observations of the same spot.
        addObservation(centre, size, elongation, thermalStamp.seconds());
    }

    // Periodic summary so a missed detection can be diagnosed from the log.
    if (thermalStamp.seconds() - lastTrackSummary_ > 10.0 && !tracks_.empty()) {
        lastTrackSummary_ = thermalStamp.seconds();
        std::string summary;
        for (const auto& t : tracks_) {
            char buf[120];
            std::snprintf(buf, sizeof(buf), " (%.1f, %.1f): %zu views, median %.2f m %.1fx%s;",
                          median(t.xs), median(t.ys), t.sizes.size(), median(t.sizes),
                          median(t.elongations), t.reported ? " REPORTED" : "");
            summary += buf;
        }
        RCLCPP_INFO(node_->get_logger(), "Heat-source tracks:%s", summary.c_str());
    }

    // ---- decide: any heat-source track that is now confidently human? ----------
    for (auto& track : tracks_) {
        const int n = static_cast<int>(track.sizes.size());
        if (track.reported || n < std::max(1, confirmationFrames_)) {
            continue;
        }
        const double medianSize = median(track.sizes);
        const double medianElongation = median(track.elongations);
        int humanViews = 0;
        for (int k = 0; k < n; ++k) {
            humanViews += track.sizes[k] >= minHumanSizeM_ && track.sizes[k] <= maxHumanSizeM_ &&
                          track.elongations[k] >= minHumanElongation_;
        }
        const double humanFraction = static_cast<double>(humanViews) / n;
        const bool human = medianSize >= minHumanSizeM_ && medianSize <= maxHumanSizeM_ &&
                           medianElongation >= minHumanElongation_ &&
                           humanFraction >= minHumanViewFraction_;
        if (!human) {
            if (n != track.loggedAt) {
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                     "Ignoring heat source at (%.1f, %.1f): median %.2f m, %.1fx, %.0f%% "
                                     "human-like over %d views (needs %.2f-%.1f m, >= %.1fx, >= %.0f%%)",
                                     median(track.xs), median(track.ys), medianSize, medianElongation,
                                     100.0 * humanFraction, n, minHumanSizeM_, maxHumanSizeM_,
                                     minHumanElongation_, 100.0 * minHumanViewFraction_);
                track.loggedAt = n;
            }
            continue;
        }
        track.reported = true;
        RCLCPP_INFO(node_->get_logger(),
                    "Human-shaped heat source: median %.2f m, %.1fx, %.0f%% human-like over %d views",
                    medianSize, medianElongation, 100.0 * humanFraction, n);
        geometry_msgs::msg::PointStamped result;
        result.header.stamp = thermalMsg.header.stamp;
        result.header.frame_id = latestOdometry->header.frame_id;
        result.point.x = median(track.xs);
        result.point.y = median(track.ys);
        result.point.z = median(track.zs);
        return result;
    }
    return std::nullopt;
}

void ThermalCamera::addObservation(const cv::Point3d& centre, double size, double elongation, double stamp)
{
    Track* track = nullptr;
    for (auto& t : tracks_) {
        if (std::hypot(centre.x - median(t.xs), centre.y - median(t.ys)) <= confirmationDistanceM_) {
            track = &t;
            break;
        }
    }
    if (!track) {
        tracks_.emplace_back();
        track = &tracks_.back();
    }
    track->xs.push_back(centre.x);
    track->ys.push_back(centre.y);
    track->zs.push_back(centre.z);
    track->sizes.push_back(size);
    track->elongations.push_back(elongation);
    track->lastSeen = stamp;
    // Keep the most recent views only.
    constexpr std::size_t keep = 40;
    for (auto* v : {&track->xs, &track->ys, &track->zs, &track->sizes, &track->elongations}) {
        if (v->size() > keep) {
            v->erase(v->begin());
        }
    }
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
