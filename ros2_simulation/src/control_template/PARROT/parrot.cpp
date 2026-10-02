#include "parrot.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

namespace {

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
}

}  // namespace

Parrot::Parrot(rclcpp::Node* node, double maxSpeed, double tolerance, double targetAlt)
    : node_(node),
      max_speed_(maxSpeed),
      tolerance_(tolerance),
      target_alt_(targetAlt){
    pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(topics::PARROT_CMD_VEL, 10);
    sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        topics::PARROT_ODOM, 10,
        std::bind(&Parrot::onOdom, this, std::placeholders::_1));

    waypoints_ = lawnmower(-10.0, -10.0, 16.0, 4.0, 16.0);

    RCLCPP_INFO(node_->get_logger(),
                "Search pattern: %zu waypoints, listening on %s",
                waypoints_.size(), topics::PARROT_ODOM);
}

bool Parrot::ready() const{
    return have_pose_;
}

bool Parrot::atAltitude() const{
    return std::fabs(target_alt_ - z_) <= 0.5;
}

bool Parrot::searchComplete() const{
    return index_ >= waypoints_.size();
}

void Parrot::climb(){
    geometry_msgs::msg::Twist cmd;
    cmd.linear.z = altitudeCommand();
    pub_->publish(cmd);
}

void Parrot::flyToWaypoint(){
    double dx = waypoints_[index_].x - x_;
    double dy = waypoints_[index_].y - y_;
    double distance = std::hypot(dx, dy);

    if (distance < tolerance_) {
        ++index_;
        RCLCPP_INFO(node_->get_logger(), "Reached waypoint %zu/%zu",
                    index_, waypoints_.size());
        return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.z = altitudeCommand();

    // Turn toward the goal, then drive at a speed that falls off as
    // heading error grows, so it doesn't arc wide on sharp turns.
    double heading_error = std::atan2(dy, dx) - yaw_;
    heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));

    cmd.angular.z = std::clamp(1.5 * heading_error, -1.0, 1.0);
    if (std::fabs(heading_error) < 0.5) {
        cmd.linear.x = std::min(max_speed_, 0.5 * distance);
    }
    pub_->publish(cmd);
}

void Parrot::stop(){
    pub_->publish(geometry_msgs::msg::Twist());
}

std::vector<Waypoint> Parrot::lawnmower(double x0, double y0, double width,
                                        double spacing, double height){
    std::vector<Waypoint> pts;
    int rows = static_cast<int>(height / spacing) + 1;
    for (int i = 0; i < rows; i++) {
        double y = y0 + i * spacing;
        if (i % 2 == 0) {
            pts.push_back({x0, y});
            pts.push_back({x0 + width, y});
        } else {
            pts.push_back({x0 + width, y});
            pts.push_back({x0, y});
        }
    }
    return pts;
}

void Parrot::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg){
    x_   = msg->pose.pose.position.x;
    y_   = msg->pose.pose.position.y;
    z_   = msg->pose.pose.position.z;
    yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
    have_pose_ = true;
}

// Hold altitude: proportional on the height error, capped at 1 m/s.
double Parrot::altitudeCommand() const{
    return std::clamp(0.8 * (target_alt_ - z_), -1.0, 1.0);
}
