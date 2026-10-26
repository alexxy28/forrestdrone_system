#ifndef HUSKY_NODE_HPP
#define HUSKY_NODE_HPP

#include "husky.hpp"

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/bool.hpp>

#include <optional>
#include <vector>

/// Receives a planned route, converts it for Nav2, and reports progress.
class HuskyNode : public rclcpp::Node{
    public:
    HuskyNode();

    private:
    void routeCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void onStop(const std_msgs::msg::Bool::SharedPtr msg);
    void tick();
    bool transformRoute(const nav_msgs::msg::Path& path, std::vector<Waypoint>& out);

    Husky husky_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr routeSub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stopSub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::optional<nav_msgs::msg::Path> pendingRoute_;
    std::optional<HuskyStatus> lastPublished_;
};

#endif
