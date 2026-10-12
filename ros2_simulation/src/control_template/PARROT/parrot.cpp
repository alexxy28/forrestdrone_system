// parrot.cpp — the drone: climb, zig-zag area search, lidar avoidance of trees and slopes.
// Run by ParrotNode (state machine): WAITING_FOR_ODOM -> CLIMBING -> SEARCHING -> HOLDING
//
// Terminal 1 (sim):
//   source ~/rs1_ws/install/setup.bash
//   ros2 launch 41068_ignition_bringup 41068_ignition.launch.py world:=terrain_demo husky:=true parrot:=true rviz:=true
//
// Terminal 2 (build + run, rebuild after every edit):
//   cd ~/rs1_ws
//   colcon build --symlink-install --packages-select control_template
//   source install/setup.bash
//   ros2 run control_template parrot_node
//
// Default search: x -11..11, y -11..11, lanes 4 m apart. Change without rebuilding, e.g.
//   ros2 run control_template parrot_node --ros-args -p area_x0:=-17.0 -p area_width:=28.0
// Single point test:  --ros-args -p goto_x:=-15.0 -p goto_y:=-10.0
//
// RViz: /parrot1/scan (LaserScan), /parrot1/obstacle_markers (MarkerArray)
//   green cylinders = trees, orange spheres = nearest point of a slope/hill

// parrot.cpp — the drone: climb, then fly the waypoints mission control sends,
// steering around trees and slopes seen by the lidar.
// Run by ParrotNode: WAITING_FOR_ODOM -> CLIMBING -> IDLE -> SEARCHING -> HOLDING
//
// RViz: /parrot1/scan (LaserScan), /parrot1/obstacle_markers (MarkerArray)
//   green cylinders = trees, orange spheres = nearest point of a slope/hill

#include "parrot.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <utility>

namespace {

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
}

}  // namespace

Parrot::Parrot(rclcpp::Node* node, double maxSpeed, double tolerance, double targetAlt)
    : node_(node), max_speed_(maxSpeed), tolerance_(tolerance), target_alt_(targetAlt)
{
    // max_speed, goal_tolerance and target_altitude are declared by ParrotNode.
    // Everything else the drone needs is declared here.

    // --- topics ---
    std::string odom_topic   = node_->declare_parameter<std::string>("odom_topic", "/parrot1/odometry");
    std::string cmd_topic    = node_->declare_parameter<std::string>("cmd_topic", "/parrot1/cmd_vel");
    std::string scan_topic   = node_->declare_parameter<std::string>("scan_topic", "/parrot1/scan");
    std::string marker_topic = node_->declare_parameter<std::string>("marker_topic", "/parrot1/obstacle_markers");

    // --- height: timed climb (set takeoff_time to 0 to use z / target_altitude instead) ---
    takeoff_speed_ = node_->declare_parameter<double>("takeoff_speed", 0.5);   // m/s up
    takeoff_time_  = node_->declare_parameter<double>("takeoff_time", 4.0);    // s

    // --- lidar ---
    cluster_gap_    = node_->declare_parameter<double>("cluster_gap", 0.5);     // gap that separates two objects
    min_points_     = node_->declare_parameter<int>("min_cluster_points", 3);   // smaller = noise
    max_tree_width_ = node_->declare_parameter<double>("max_tree_width", 2.0);  // wider = slope/hill

    // --- avoidance ---
    avoid_radius_  = node_->declare_parameter<double>("avoid_radius", 4.0);    // trees closer than this push
    slope_radius_  = node_->declare_parameter<double>("slope_radius", 3.0);    // slopes closer than this push
    avoid_gain_    = node_->declare_parameter<double>("avoid_gain", 2.0);      // push strength (m/s at contact)
    stuck_timeout_ = node_->declare_parameter<double>("stuck_timeout", 15.0);  // s without progress -> skip

    pub_        = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);

    sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        std::bind(&Parrot::onOdom, this, std::placeholders::_1));
    // Sensor data is best-effort; a reliable subscriber would get nothing.
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, rclcpp::SensorDataQoS(),
        std::bind(&Parrot::onScan, this, std::placeholders::_1));

    if (timedClimb()) {
        RCLCPP_INFO(node_->get_logger(), "Height: timed climb, %.1f s at %.1f m/s (z not used)",
                    takeoff_time_, takeoff_speed_);
    } else {
        RCLCPP_INFO(node_->get_logger(), "Height: hold z = %.1f m", target_alt_);
    }
}

// ================= status =================
bool Parrot::ready() const
{
    return have_pose_;
}

bool Parrot::atAltitude() const
{
    if (timedClimb()) {
        return climbed_;
    }
    return std::fabs(target_alt_ - z_) < 0.5;
}

bool Parrot::searchComplete() const
{
    return index_ >= waypoints_.size();
}

std::vector<std::pair<double, double>> Parrot::remainingWaypoints() const
{
    std::vector<std::pair<double, double>> remaining;
    remaining.reserve(waypoints_.size() - std::min(index_, waypoints_.size()));
    for (size_t i = index_; i < waypoints_.size(); ++i) {
        remaining.emplace_back(waypoints_[i].x, waypoints_[i].y);
    }
    return remaining;
}

// ================= waypoints from mission control =================
void Parrot::setWaypoints(const std::vector<std::pair<double, double>>& points)
{
    waypoints_.clear();
    for (const auto& p : points) {
        waypoints_.push_back({p.first, p.second});
    }
    index_ = 0;
    progress_index_ = std::numeric_limits<size_t>::max();
    RCLCPP_INFO(node_->get_logger(), "New mission: %zu waypoints", waypoints_.size());
    for (size_t i = 0; i < waypoints_.size(); i++) {
        RCLCPP_INFO(node_->get_logger(), "  wp %2zu: (%6.1f, %6.1f)",
                    i + 1, waypoints_[i].x, waypoints_[i].y);
    }
}

// ================= odometry =================
void Parrot::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    x_   = msg->pose.pose.position.x;
    y_   = msg->pose.pose.position.y;
    z_   = msg->pose.pose.position.z;
    yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
    have_pose_ = true;
}

// ================= lidar: trees + slopes =================
Parrot::Obstacle Parrot::makeObstacle(double bx, double by, double width) const
{
    Obstacle o;
    o.bx = bx;
    o.by = by;
    o.width = width;
    o.range = std::hypot(bx, by);
    o.wx = x_ + bx * std::cos(yaw_) - by * std::sin(yaw_);
    o.wy = y_ + bx * std::sin(yaw_) + by * std::cos(yaw_);
    return o;
}

// Consecutive scan points closer than cluster_gap belong to one object.
// Narrow object -> tree (its centre). Wide object -> slope (its closest point).
void Parrot::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!got_first_scan_) {
        got_first_scan_ = true;
        RCLCPP_INFO(node_->get_logger(),
                    "Lidar OK: frame '%s', %zu beams, %.0f to %.0f deg, range %.2f-%.1f m",
                    msg->header.frame_id.c_str(), msg->ranges.size(),
                    msg->angle_min * 180.0 / M_PI, msg->angle_max * 180.0 / M_PI,
                    msg->range_min, msg->range_max);
    }

    std::vector<Obstacle> trees;
    std::vector<Obstacle> slopes;
    std::vector<std::pair<double, double>> cluster;
    double prev_x = 0.0, prev_y = 0.0;
    bool have_prev = false;

    auto flush = [&]() {
        if (static_cast<int>(cluster.size()) >= min_points_) {
            double w = std::hypot(cluster.front().first - cluster.back().first,
                                  cluster.front().second - cluster.back().second);
            if (w > max_tree_width_) {
                auto nearest = *std::min_element(cluster.begin(), cluster.end(),
                    [](const std::pair<double, double> & a, const std::pair<double, double> & b) {
                        return std::hypot(a.first, a.second) < std::hypot(b.first, b.second);
                    });
                slopes.push_back(makeObstacle(nearest.first, nearest.second, w));
            } else {
                double sx = 0.0, sy = 0.0;
                for (const auto & p : cluster) { sx += p.first; sy += p.second; }
                trees.push_back(makeObstacle(sx / cluster.size(), sy / cluster.size(), w));
            }
        }
        cluster.clear();
    };

    for (size_t i = 0; i < msg->ranges.size(); i++) {
        double r = msg->ranges[i];
        if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
            flush();
            have_prev = false;
            continue;
        }
        double a  = msg->angle_min + i * msg->angle_increment;
        double px = r * std::cos(a);
        double py = r * std::sin(a);

        if (have_prev && std::hypot(px - prev_x, py - prev_y) > cluster_gap_) {
            flush();
        }
        cluster.push_back({px, py});
        prev_x = px;
        prev_y = py;
        have_prev = true;
    }
    flush();

    trees_ = trees;
    slopes_ = slopes;
    publishMarkers(msg->header);

    auto byRange = [](const Obstacle & a, const Obstacle & b) { return a.range < b.range; };
    if (trees.empty()) {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                             "Lidar: no trees in view | %zu slopes", slopes.size());
    } else {
        auto c = *std::min_element(trees.begin(), trees.end(), byRange);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                             "Lidar: %zu trees, closest %.1f m at world (%.1f, %.1f) | %zu slopes",
                             trees.size(), c.range, c.wx, c.wy, slopes.size());
    }
}

// Green cylinders = trees, orange spheres = closest point of each slope.
void Parrot::publishMarkers(const std_msgs::msg::Header & header)
{
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    int id = 0;
    for (const auto & t : trees_) {
        visualization_msgs::msg::Marker m;
        m.header = header;
        m.ns = "trees";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = t.bx;
        m.pose.position.y = t.by;
        m.pose.orientation.w = 1.0;
        double d = std::max(t.width, 0.3);
        m.scale.x = d;
        m.scale.y = d;
        m.scale.z = 1.0;
        m.color.r = 0.1f;
        m.color.g = 0.9f;
        m.color.b = 0.2f;
        m.color.a = 0.8f;
        arr.markers.push_back(m);
    }
    for (const auto & s : slopes_) {
        visualization_msgs::msg::Marker m;
        m.header = header;
        m.ns = "slopes";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = s.bx;
        m.pose.position.y = s.by;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.5;
        m.scale.y = 0.5;
        m.scale.z = 0.5;
        m.color.r = 1.0f;
        m.color.g = 0.5f;
        m.color.b = 0.0f;
        m.color.a = 0.9f;
        arr.markers.push_back(m);
    }
    marker_pub_->publish(arr);
}

// ================= commands =================
void Parrot::publish(double vx, double vy, double vz, double wz)
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.linear.z = vz;
    cmd.angular.z = wz;
    pub_->publish(cmd);
}

void Parrot::stop()
{
    publish(0.0, 0.0, 0.0, 0.0);
}

/// Vertical speed that holds the target altitude (z mode only; 0 in timed mode).
double Parrot::altitudeCommand() const
{
    if (timedClimb()) {
        return 0.0;
    }
    return std::clamp(0.8 * (target_alt_ - z_), -1.0, 1.0);
}

void Parrot::climb()
{
    if (!timedClimb()) {
        publish(0.0, 0.0, altitudeCommand(), 0.0);
        return;
    }
    double now_s = node_->now().seconds();
    if (climb_start_s_ < 0.0) {
        climb_start_s_ = now_s;
        RCLCPP_INFO(node_->get_logger(), "Taking off for %.1f s at %.1f m/s",
                    takeoff_time_, takeoff_speed_);
    }
    if (now_s - climb_start_s_ >= takeoff_time_) {
        climbed_ = true;
        stop();
        return;
    }
    publish(0.0, 0.0, takeoff_speed_, 0.0);
}

// Adds a "push away + slide around" velocity for one obstacle (body frame).
void Parrot::addRepulsion(const Obstacle & o, double radius, double & vx, double & vy) const
{
    double strength = avoid_gain_ * (radius - o.range) / radius;
    double ux = o.bx / o.range;   // unit vector towards the obstacle
    double uy = o.by / o.range;

    vx -= strength * ux;          // straight away from it
    vy -= strength * uy;

    if (ux > 0.0) {               // in front: slide sideways around it
        double side = (o.by >= 0.0) ? -1.0 : 1.0;   // on the left -> go right
        vx += strength * side * (-uy);
        vy += strength * side * ux;
    }
}

void Parrot::flyToWaypoint()
{
    if (searchComplete()) {
        stop();
        return;
    }
    if (!got_first_scan_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "No lidar data yet, flying without avoidance");
    }

    double dx = waypoints_[index_].x - x_;
    double dy = waypoints_[index_].y - y_;
    double distance = std::hypot(dx, dy);
    double now_s = node_->now().seconds();

    // ---- reached ----
    if (distance < tolerance_) {
        RCLCPP_INFO(node_->get_logger(), "Reached waypoint %zu/%zu (%.1f, %.1f)",
                    index_ + 1, waypoints_.size(), waypoints_[index_].x, waypoints_[index_].y);
        ++index_;
        publish(0.0, 0.0, altitudeCommand(), 0.0);
        return;
    }

    // ---- stuck: skip a waypoint we can't get closer to ----
    if (index_ != progress_index_) {
        progress_index_ = index_;
        best_distance_ = distance;
        last_progress_s_ = now_s;
    } else if (distance < best_distance_ - 0.5) {
        best_distance_ = distance;
        last_progress_s_ = now_s;
    } else if (now_s - last_progress_s_ > stuck_timeout_) {
        RCLCPP_WARN(node_->get_logger(), "Waypoint %zu/%zu (%.1f, %.1f) blocked, skipping",
                    index_ + 1, waypoints_.size(), waypoints_[index_].x, waypoints_[index_].y);
        ++index_;
        publish(0.0, 0.0, altitudeCommand(), 0.0);
        return;
    }

    // ---- pull towards the waypoint (body frame) ----
    double gx =  std::cos(yaw_) * dx + std::sin(yaw_) * dy;
    double gy = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;
    double speed = std::min(max_speed_, 0.5 * distance);
    double vx = gx / distance * speed;
    double vy = gy / distance * speed;

    // ---- push away from trees and slopes ----
    double closest_tree = std::numeric_limits<double>::infinity();
    for (const auto & t : trees_) {
        if (t.range > 1e-3 && t.range < avoid_radius_) {
            closest_tree = std::min(closest_tree, t.range);
            addRepulsion(t, avoid_radius_, vx, vy);
        }
    }
    double closest_slope = std::numeric_limits<double>::infinity();
    for (const auto & s : slopes_) {
        if (s.range > 1e-3 && s.range < slope_radius_) {
            closest_slope = std::min(closest_slope, s.range);
            addRepulsion(s, slope_radius_, vx, vy);
        }
    }
    if (std::isfinite(closest_tree)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Avoiding tree %.1f m away", closest_tree);
    }
    if (std::isfinite(closest_slope)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Avoiding slope %.1f m away", closest_slope);
    }

    // ---- cap speed ----
    double v = std::hypot(vx, vy);
    if (v > max_speed_) {
        vx *= max_speed_ / v;
        vy *= max_speed_ / v;
    }

    // ---- face the waypoint ----
    double heading_error = std::atan2(dy, dx) - yaw_;
    heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));

    publish(vx, vy, altitudeCommand(), std::clamp(1.5 * heading_error, -1.0, 1.0));
}
