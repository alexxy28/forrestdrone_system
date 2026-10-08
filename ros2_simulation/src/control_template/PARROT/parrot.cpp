// parrot.cpp — Parrot drone: zig-zag area search + lidar avoidance of trees and slopes
//
// Terminal 1 (sim):
//   source ~/rs1_ws/install/setup.bash
//   ros2 launch 41068_ignition_bringup 41068_ignition.launch.py world:=terrain_demo husky:=true parrot:=true rviz:=true
//
// Terminal 2 (heat camera):
//   source ~/rs1_ws/install/setup.bash
//   python3 ~/rs1_ws/src/forrestdrone_system/thermal_camera_subscriber.py
//
// Terminal 3 (build + run, rebuild after every edit):
//   cd ~/rs1_ws
//   colcon build --symlink-install --packages-select control_template
//   source install/setup.bash
//   ros2 run control_template drone_search
//
// Default: zig-zag over x -11..11, y -11..11, lanes 4 m apart.
// Change without rebuilding, e.g.:
//   ros2 run control_template drone_search --ros-args -p area_x0:=-16.0 -p area_width:=27.0 -p lane_spacing:=3.0
// Single point test:  --ros-args -p goto_x:=-15.0 -p goto_y:=-10.0
//
// RViz: /parrot1/scan (LaserScan), /parrot1/obstacle_markers (MarkerArray)
//   green cylinders = trees, orange spheres = nearest point of a slope/hill
//
// Height: TIMED takeoff (no z feedback), climbs takeoff_speed for takeoff_time, then linear.z = 0.
// Avoidance: trees (narrow lidar clusters) within avoid_radius and slopes (wide clusters)
// within slope_radius push the drone away and slide it around them.
// Stuck: no progress towards a waypoint for stuck_timeout s -> skip it.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

struct Waypoint { double x, y; };

// Something the lidar sees: a tree trunk (narrow) or the closest point of a slope (wide).
struct Obstacle
{
    double bx, by;       // position in lidar/body frame (m)
    double wx, wy;       // position in world frame (m)
    double range;        // distance from drone (m)
    double bearing;      // angle from drone's nose (rad, +ve = left)
    double width;        // cluster width (m)
};

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
}

class DroneSearch : public rclcpp::Node
{
    public:
        DroneSearch() : Node("drone_search")
        {
            // --- motion ---
            odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/parrot1/odometry");
            cmd_topic_  = this->declare_parameter<std::string>("cmd_topic", "/parrot1/cmd_vel");
            max_speed_  = this->declare_parameter<double>("max_speed", 1.5);        // slow: lidar is only ~2.5 Hz
            tolerance_  = this->declare_parameter<double>("goal_tolerance", 1.0);   // must be > 0
            stuck_timeout_ = this->declare_parameter<double>("stuck_timeout", 15.0); // s without progress -> skip

            // --- timed takeoff (no z feedback) ---
            takeoff_speed_ = this->declare_parameter<double>("takeoff_speed", 0.5);  // m/s up
            takeoff_time_  = this->declare_parameter<double>("takeoff_time", 4.0);   // s

            // --- zig-zag search area (world frame, m): bottom-left corner + size ---
            double x0      = this->declare_parameter<double>("area_x0", -11.0);
            double y0      = this->declare_parameter<double>("area_y0", -11.0);
            double width   = this->declare_parameter<double>("area_width", 22.0);
            double height  = this->declare_parameter<double>("area_height", 22.0);
            double spacing = this->declare_parameter<double>("lane_spacing", 4.0);

            // --- optional single point instead of the zig-zag ---
            double goto_x = this->declare_parameter<double>("goto_x", std::nan(""));
            double goto_y = this->declare_parameter<double>("goto_y", std::nan(""));

            // --- lidar ---
            scan_topic_     = this->declare_parameter<std::string>("scan_topic", "/parrot1/scan");
            marker_topic_   = this->declare_parameter<std::string>("marker_topic", "/parrot1/obstacle_markers");
            cluster_gap_    = this->declare_parameter<double>("cluster_gap", 0.5);     // gap that separates two objects
            min_points_     = this->declare_parameter<int>("min_cluster_points", 3);   // smaller = noise
            max_tree_width_ = this->declare_parameter<double>("max_tree_width", 2.0);  // wider = slope/hill

            // --- avoidance ---
            avoid_radius_ = this->declare_parameter<double>("avoid_radius", 4.0);   // trees closer than this push
            slope_radius_ = this->declare_parameter<double>("slope_radius", 3.0);   // slopes closer than this push
            avoid_gain_   = this->declare_parameter<double>("avoid_gain", 2.0);     // push strength (m/s at contact)

            pub_        = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic_, 10,
                std::bind(&DroneSearch::onOdom, this, std::placeholders::_1));
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                scan_topic_, rclcpp::SensorDataQoS(),
                std::bind(&DroneSearch::onScan, this, std::placeholders::_1));

            if (std::isfinite(goto_x) && std::isfinite(goto_y)) {
                waypoints_ = {{goto_x, goto_y}};
                RCLCPP_INFO(this->get_logger(), "GOTO mode: flying to (%.1f, %.1f)", goto_x, goto_y);
            } else {
                waypoints_ = zigzag(x0, y0, width, height, spacing);
                RCLCPP_INFO(this->get_logger(),
                            "Zig-zag search: x %.1f..%.1f, y %.1f..%.1f, lanes %.1f m apart, %zu waypoints",
                            x0, x0 + width, y0, y0 + height, spacing, waypoints_.size());
                for (size_t i = 0; i < waypoints_.size(); i++) {
                    RCLCPP_INFO(this->get_logger(), "  wp %2zu: (%6.1f, %6.1f)",
                                i + 1, waypoints_[i].x, waypoints_[i].y);
                }
            }

            timer_ = this->create_wall_timer(100ms, std::bind(&DroneSearch::tick, this));
        }

    private:
        // ================= zig-zag pattern =================
        // Lanes run along x; each lane steps up in y. The last lane sits exactly
        // on the top edge so the whole area is covered.
        std::vector<Waypoint> zigzag(double x0, double y0, double width,
                                     double height, double spacing)
        {
            std::vector<Waypoint> pts;
            int lanes = static_cast<int>(std::ceil(height / spacing)) + 1;
            for (int i = 0; i < lanes; i++) {
                double y = std::min(y0 + i * spacing, y0 + height);
                if (i % 2 == 0) {
                    pts.push_back({x0, y});
                    pts.push_back({x0 + width, y});
                } else {
                    pts.push_back({x0 + width, y});
                    pts.push_back({x0, y});
                }
                if (y >= y0 + height) {
                    break;
                }
            }
            return pts;
        }

        // ================= odometry =================
        void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            x_   = msg->pose.pose.position.x;
            y_   = msg->pose.pose.position.y;
            yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
            have_pose_ = true;
        }

        Obstacle makeObstacle(double bx, double by, double width) const
        {
            Obstacle o;
            o.bx = bx;
            o.by = by;
            o.width = width;
            o.range = std::hypot(bx, by);
            o.bearing = std::atan2(by, bx);
            o.wx = x_ + bx * std::cos(yaw_) - by * std::sin(yaw_);
            o.wy = y_ + bx * std::sin(yaw_) + by * std::cos(yaw_);
            return o;
        }

        // ================= lidar: trees + slopes =================
        // Consecutive scan points closer than cluster_gap belong to one object.
        // Narrow object -> tree (centre). Wide object -> slope (its closest point).
        void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            if (!got_first_scan_) {
                got_first_scan_ = true;
                RCLCPP_INFO(this->get_logger(),
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
                        // Slope / hill: avoid its closest point, not its centre
                        auto near = *std::min_element(cluster.begin(), cluster.end(),
                            [](const std::pair<double, double> & a, const std::pair<double, double> & b) {
                                return std::hypot(a.first, a.second) < std::hypot(b.first, b.second);
                            });
                        slopes.push_back(makeObstacle(near.first, near.second, w));
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
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                     "Lidar: no trees in view | %zu slopes", slopes.size());
            } else {
                auto c = *std::min_element(trees.begin(), trees.end(), byRange);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                     "Lidar: %zu trees, closest %.1f m at world (%.1f, %.1f) | %zu slopes",
                                     trees.size(), c.range, c.wx, c.wy, slopes.size());
            }
        }

        // Green cylinders = trees, orange spheres = closest point of each slope.
        void publishMarkers(const std_msgs::msg::Header & header)
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

        // Adds a "push away + slide around" velocity for one obstacle.
        void addRepulsion(const Obstacle & o, double radius, double & vx, double & vy) const
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

        // ================= control loop =================
        void tick()
        {
            geometry_msgs::msg::Twist cmd;   // all zero; linear.z only non-zero during takeoff

            if (!have_pose_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "No odometry yet on %s", odom_topic_.c_str());
                pub_->publish(cmd);
                return;
            }
            if (!got_first_scan_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "No lidar data yet on %s", scan_topic_.c_str());
            }

            // ---- timed takeoff ----
            double now_s = this->now().seconds();
            if (takeoff_start_s_ < 0.0) {
                takeoff_start_s_ = now_s;
                RCLCPP_INFO(this->get_logger(), "Taking off for %.1f s at %.1f m/s",
                            takeoff_time_, takeoff_speed_);
            }
            if (now_s - takeoff_start_s_ < takeoff_time_) {
                cmd.linear.z = takeoff_speed_;
                pub_->publish(cmd);
                return;
            }
            if (!airborne_) {
                airborne_ = true;
                RCLCPP_INFO(this->get_logger(), "Takeoff done, starting search");
            }

            // ---- finished ----
            if (index_ >= waypoints_.size()) {
                if (!done_) {
                    RCLCPP_INFO(this->get_logger(), "Search complete, hovering");
                    done_ = true;
                }
                pub_->publish(cmd);
                return;
            }

            double dx = waypoints_[index_].x - x_;
            double dy = waypoints_[index_].y - y_;
            double distance = std::hypot(dx, dy);

            // ---- reached ----
            if (distance < tolerance_) {
                RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu/%zu (%.1f, %.1f)",
                            index_ + 1, waypoints_.size(),
                            waypoints_[index_].x, waypoints_[index_].y);
                ++index_;
                return;
            }

            // ---- stuck check: skip a waypoint we can't get closer to ----
            if (index_ != progress_index_) {
                progress_index_ = index_;
                best_distance_ = distance;
                last_progress_s_ = now_s;
            } else if (distance < best_distance_ - 0.5) {
                best_distance_ = distance;
                last_progress_s_ = now_s;
            } else if (now_s - last_progress_s_ > stuck_timeout_) {
                RCLCPP_WARN(this->get_logger(),
                            "Waypoint %zu/%zu (%.1f, %.1f) blocked, skipping",
                            index_ + 1, waypoints_.size(),
                            waypoints_[index_].x, waypoints_[index_].y);
                ++index_;
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
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Avoiding tree %.1f m away", closest_tree);
            }
            if (std::isfinite(closest_slope)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
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

            cmd.linear.x = vx;
            cmd.linear.y = vy;
            cmd.angular.z = std::clamp(1.5 * heading_error, -1.0, 1.0);
            pub_->publish(cmd);
        }

        // ---- ROS handles ----
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        // ---- params ----
        std::string odom_topic_, cmd_topic_, scan_topic_, marker_topic_;
        double max_speed_, tolerance_, stuck_timeout_;
        double takeoff_speed_, takeoff_time_;
        double cluster_gap_, max_tree_width_;
        double avoid_radius_, slope_radius_, avoid_gain_;
        int min_points_;

        // ---- state ----
        std::vector<Waypoint> waypoints_;
        size_t index_{0};
        std::vector<Obstacle> trees_;
        std::vector<Obstacle> slopes_;
        bool have_pose_{false};
        bool got_first_scan_{false};
        bool done_{false};
        bool airborne_{false};
        double takeoff_start_s_{-1.0};
        size_t progress_index_{std::numeric_limits<size_t>::max()};
        double best_distance_{0.0};
        double last_progress_s_{0.0};
        double x_{0.0}, y_{0.0}, yaw_{0.0};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneSearch>());
    rclcpp::shutdown();
    return 0;
}