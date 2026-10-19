#ifndef PARROT_HPP
#define PARROT_HPP

#include <cstddef>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "dataStructures.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

/// The drone: climbs, then flies the waypoints mission control gives it while
/// steering around trees and slopes seen by its lidar. It has no search
/// pattern of its own; mission control decides where it goes.
///
/// Height: by default a TIMED climb (takeoff_speed for takeoff_time seconds),
/// because the z reading isn't reliable on this terrain. Run with
/// -p takeoff_time:=0.0 to use the z-based target altitude instead.
class Parrot{
public:
Parrot(rclcpp::Node* node, double maxSpeed, double tolerance, double targetAlt);

    /// True once the first odometry message has arrived.
bool ready() const;

    /// Timed climb: true once the climb has finished.
    /// z mode: true while within 0.5 m of the target altitude.
bool atAltitude() const;

    /// True once every waypoint has been reached or skipped (also true when there are none).
    bool searchComplete() const;
    std::vector<std::pair<double, double>> remainingWaypoints() const;

    /// Current drone pose in world frame.
double x() const { return x_; }
double y() const { return y_; }

    /// Replace the waypoints to fly, starting from the first one.
void setWaypoints(const std::vector<std::pair<double, double>>& points);

    /// Publish a vertical-only climb command.
void climb();

    /// Step toward the current waypoint, avoiding trees and slopes — call from the control loop.
void flyToWaypoint();

    /// Publish a zero Twist so the drone holds position.
void stop();

    /// Rotate in place until a relative bearing is centered in the thermal camera.
bool aimAtRelativeBearing(double bearing);

void hold();

private:
    struct Waypoint { double x, y; };

    /// Something the lidar sees: a tree trunk (narrow) or the nearest point of a slope (wide).
    struct Obstacle
    {
        double bx, by;    // position in lidar/body frame (m)
        double wx, wy;    // position in world frame (m)
        double range;     // distance from the drone (m)
        double width;     // width of the lidar cluster (m)
    };

void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
double altitudeCommand() const;
Obstacle makeObstacle(double bx, double by, double width) const;
void addRepulsion(const Obstacle & o, double radius, double & vx, double & vy) const;
void publishMarkers(const std_msgs::msg::Header & header);
void publish(double vx, double vy, double vz, double wz);
void catchUpAfterHold(double now_s);
bool timedClimb() const { return takeoff_time_ > 0.0; }

rclcpp::Node* node_;

    // ---- from ParrotNode ----
double max_speed_;
double tolerance_;
double target_alt_;

    // ---- ROS I/O ----
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // ---- extra parameters (declared by Parrot) ----
double takeoff_speed_{0.5};
double takeoff_time_{4.0};
double stuck_timeout_{15.0};
double cluster_gap_{0.5};
double max_tree_width_{2.0};
double avoid_radius_{4.0};
double slope_radius_{3.0};
double avoid_gain_{2.0};
int min_points_{3};

    // ---- waypoint state ----
std::vector<Waypoint> waypoints_;
size_t index_{0};
double climb_start_s_{-1.0};
bool climbed_{false};
size_t progress_index_{std::numeric_limits<size_t>::max()};
double best_distance_{0.0};
double last_progress_s_{0.0};
double hold_start_s_{-1.0};   // when the current hold began, -1 = not holding

    // ---- sensor state ----
std::vector<Obstacle> trees_;
std::vector<Obstacle> slopes_;
bool have_pose_{false};
bool got_first_scan_{false};
double x_{0.0}, y_{0.0}, z_{0.0}, yaw_{0.0};
};

#endif
