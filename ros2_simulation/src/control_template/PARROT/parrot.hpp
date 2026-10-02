#ifndef PARROT_HPP
#define PARROT_HPP

#include <memory>
#include <vector>

#include "dataStructures.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

/// The drone: holds an altitude and flies a lawnmower search pattern.
class Parrot{
    public:
    Parrot(rclcpp::Node* node, double maxSpeed, double tolerance, double targetAlt);

    /// True once the first odometry message has arrived.
    bool ready() const;

    /// True while within 0.5 m of the target altitude.
    bool atAltitude() const;

    /// True once every waypoint of the search pattern has been reached.
    bool searchComplete() const;

    /// Publish a vertical-only command toward the target altitude.
    void climb();

    /// Step toward the current waypoint — call from the control loop.
    void flyToWaypoint();

    /// Publish a zero Twist so the drone holds position.
    void stop();

    protected:
    private:
    std::vector<Waypoint> lawnmower(double x0, double y0, double width,
                                    double spacing, double height);
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    double altitudeCommand() const;

    rclcpp::Node* node_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

    std::vector<Waypoint> waypoints_;
    size_t index_{0};

    double max_speed_;
    double tolerance_;
    double target_alt_;

    bool have_pose_{false};
    double x_{0.0}, y_{0.0}, z_{0.0}, yaw_{0.0};
};

#endif
