// Accumulates Parrot LiDAR returns into the TerrainMap grid (running mean and
// variance per cell) and publishes it for system_control_node and rqt.
// C++ port of the former 41068_ignition_bringup/scripts/terrain_map_builder.py.

#include "dataStructures.hpp"
#include "terrain_map.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <vector>

using sensor_msgs::msg::PointCloud2;

class TerrainMapNode : public rclcpp::Node
{
public:
    TerrainMapNode()
        : Node("terrain_map_node", rclcpp::NodeOptions().arguments({
              "--ros-args",
              "-r", std::string("/tf:=") + topics::PARROT_TF,
              "-r", std::string("/tf_static:=") + topics::PARROT_TF_STATIC})),
          mapFrame_(declare_parameter<std::string>("map_frame", "parrot1_odom")),
          map_(declare_parameter<double>("terrain_resolution", 0.5),
               declare_parameter<double>("terrain_size_x", 40.0),
               declare_parameter<double>("terrain_size_y", 40.0)),
          tfBuffer_(get_clock()),
          tfListener_(tfBuffer_, this, false)
    {
        odomSub_ = create_subscription<nav_msgs::msg::Odometry>(
            topics::PARROT_ODOM, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_ = msg; });

        scanSub_ = create_subscription<PointCloud2>(
            topics::PARROT_SCAN, rclcpp::SensorDataQoS(),
            std::bind(&TerrainMapNode::scanCallback, this, std::placeholders::_1));

        terrainPub_ = create_publisher<PointCloud2>(
            topics::PARROT_TERRAIN, rclcpp::SensorDataQoS());

        // Saved map from a previous scan (ros2 bag play ... --remap
        // /parrot1/terrain_points:=/parrot1/terrain_seed). Only the first cloud
        // is used; its cell counts act as prior weight, so live scans keep
        // refining it and gradually replace it (COUNT_CAP in TerrainMap).
        seedSub_ = create_subscription<PointCloud2>(
            topics::PARROT_TERRAIN_SEED, rclcpp::SensorDataQoS(),
            std::bind(&TerrainMapNode::seedCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Terrain accumulation started: %s + %s -> %s in frame %s",
                    topics::PARROT_SCAN, topics::PARROT_ODOM, topics::PARROT_TERRAIN,
                    mapFrame_.c_str());
    }

private:
    // Returns this close to the sensor are the airframe itself.
    static constexpr double SELF_RETURN_MIN_RANGE_M = 0.6;
    // Scan-level height gate: median +/- max(floor, 6 robust sigmas). The low
    // side rejects stray ground-plane returns past the terrain edge; the high
    // side is loose so tree canopy stays in (TerrainMap turns it into sigma).
    static constexpr double LOW_OUTLIER_MIN_DEVIATION_M = 1.0;
    static constexpr double HIGH_OUTLIER_MIN_DEVIATION_M = 4.0;
    static constexpr double OUTLIER_ROBUST_SIGMAS = 6.0;
    static constexpr double MAX_ODOM_SCAN_SKEW_S = 0.25;
    static constexpr double STATS_PERIOD_S = 5.0;

    static double median(std::vector<double> values)
    {
        auto mid = values.begin() + values.size() / 2;
        std::nth_element(values.begin(), mid, values.end());
        return *mid;
    }

    static size_t rejectHeightOutliers(std::vector<tf2::Vector3> &points)
    {
        if (points.size() < 12)
            return 0;

        std::vector<double> heights;
        for (const auto &p : points)
            heights.push_back(p.z());

        const double med = median(heights);
        for (auto &h : heights)
            h = std::abs(h - med);

        const double limit = OUTLIER_ROBUST_SIGMAS * 1.4826 * median(heights);
        const double lower = med - std::max(LOW_OUTLIER_MIN_DEVIATION_M, limit);
        const double upper = med + std::max(HIGH_OUTLIER_MIN_DEVIATION_M, limit);

        const size_t before = points.size();
        points.erase(std::remove_if(points.begin(), points.end(),
                                    [&](const tf2::Vector3 &p) {
                                        return p.z() < lower || p.z() > upper;
                                    }),
                     points.end());
        return before - points.size();
    }

    void seedCallback(const PointCloud2::SharedPtr msg)
    {
        if (seedSub_ == nullptr)
            return;   // already seeded
        if (msg->header.frame_id != mapFrame_) {
            RCLCPP_ERROR(get_logger(), "Ignoring seed map: frame '%s' is not map frame '%s'",
                         msg->header.frame_id.c_str(), mapFrame_.c_str());
            return;
        }
        // Replaces the few seconds of live returns gathered before the seed arrived.
        map_.updateFromPointCloud(*msg);
        terrainPub_->publish(map_.toCloud(mapFrame_, msg->header.stamp));
        RCLCPP_INFO(get_logger(), "Seeded terrain map with %u saved cells", msg->width * msg->height);
        map_.printStatistics();
        seedSub_.reset();   // stop listening; a looping bag can't overwrite live data
    }

    void scanCallback(const PointCloud2::SharedPtr msg)
    {
        if (!odom_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Dropping LiDAR cloud: waiting for raw Parrot odometry");
            return;
        }
        if (odom_->header.frame_id != mapFrame_) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "Dropping LiDAR cloud: odometry frame '%s' is not map frame '%s'",
                                  odom_->header.frame_id.c_str(), mapFrame_.c_str());
            return;
        }
        if (std::abs((rclcpp::Time(odom_->header.stamp) - rclcpp::Time(msg->header.stamp)).seconds())
            > MAX_ODOM_SCAN_SKEW_S) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Dropping LiDAR cloud: odometry is more than %.2f s from the scan",
                                 MAX_ODOM_SCAN_SKEW_S);
            return;
        }

        // Only the static base -> LiDAR mount comes from TF. The robot pose
        // comes from raw Gazebo odometry: the EKF TF can be tilted/offset,
        // which would add altitude and attitude error to every cell height.
        geometry_msgs::msg::TransformStamped mount;
        try {
            mount = tfBuffer_.lookupTransform(odom_->child_frame_id, msg->header.frame_id,
                                              tf2::TimePointZero);
        } catch (const tf2::TransformException &e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Dropping LiDAR cloud: static transform %s -> %s unavailable: %s",
                                 msg->header.frame_id.c_str(), odom_->child_frame_id.c_str(), e.what());
            return;
        }

        const auto &p = odom_->pose.pose;
        const auto &m = mount.transform;
        const tf2::Transform mapFromBase(
            tf2::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
            tf2::Vector3(p.position.x, p.position.y, p.position.z));
        const tf2::Transform baseFromSensor(
            tf2::Quaternion(m.rotation.x, m.rotation.y, m.rotation.z, m.rotation.w),
            tf2::Vector3(m.translation.x, m.translation.y, m.translation.z));
        const tf2::Transform mapFromSensor = mapFromBase * baseFromSensor;

        std::vector<tf2::Vector3> points;
        sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
        for (; x != x.end(); ++x, ++y, ++z) {
            const tf2::Vector3 v(*x, *y, *z);
            if (std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z()) &&
                v.length() >= SELF_RETURN_MIN_RANGE_M) {
                points.push_back(mapFromSensor * v);
            }
        }

        rejected_ += rejectHeightOutliers(points);
        for (const auto &v : points)
            map_.addPoint(v.x(), v.y(), v.z());
        accepted_ += points.size();

        terrainPub_->publish(map_.toCloud(mapFrame_, msg->header.stamp));

        if ((now() - lastStats_).seconds() >= STATS_PERIOD_S) {
            RCLCPP_INFO(get_logger(), "Accumulated %zu LiDAR returns, rejected %zu height outliers",
                        accepted_, rejected_);
            map_.rebuild();
            map_.printStatistics();
            lastStats_ = now();
        }
    }

    std::string mapFrame_;
    TerrainMap map_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::Subscription<PointCloud2>::SharedPtr scanSub_;
    rclcpp::Subscription<PointCloud2>::SharedPtr seedSub_;
    rclcpp::Publisher<PointCloud2>::SharedPtr terrainPub_;

    nav_msgs::msg::Odometry::SharedPtr odom_;
    rclcpp::Time lastStats_{0, 0, RCL_ROS_TIME};
    size_t accepted_{0};
    size_t rejected_{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TerrainMapNode>());
    rclcpp::shutdown();
    return 0;
}
