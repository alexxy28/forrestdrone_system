#ifndef PARROT_HPP
#define PARROT_HPP

#include <memory>
#include <vector>

#include "robot.hpp"
#include "sensor.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class Parrot : public Robot{
    public:
    explicit Parrot(std::shared_ptr<rclcpp::Node> node);
    Sensor& getSensor();

    /// Step the search one cycle — call from the control loop.
    void tick();

    /// Publish a zero Twist so the drone holds position.
    void stop();

    protected:
    private:
    std::vector<Waypoint> lawnmower(double x0, double y0, double width,
                                    double height, double spacing);
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<Sensor> sensor_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

    std::vector<Waypoint> waypoints_;
    size_t index_{0};

    double max_speed_{2.0};
    double tolerance_{1.0};
    double target_alt_{5.0};

    bool have_pose_{false};
    double x_{0.0}, y_{0.0}, z_{0.0}, yaw_{0.0};
};

#endif