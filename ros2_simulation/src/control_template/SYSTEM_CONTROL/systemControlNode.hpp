#ifndef SYSTEM_CONTROL_NODE_HPP
#define SYSTEM_CONTROL_NODE_HPP

#include "terrain_map.hpp"
#include "terrain_planner.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

/// Owns the mission: terrain map, route planning, and the mission state.
class SystemControlNode : public rclcpp::Node{
    public:
    SystemControlNode();

    private:
    enum class State { SEARCHING, PLANNING, HUSKY_EN_ROUTE, DONE, FAILED };

    void terrainCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void huskyOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void casualtyCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void huskyStatusCallback(const std_msgs::msg::String::SharedPtr msg);

    void tryPlan();
    void setState(State state);

    TerrainMap terrainMap_;
    TerrainPlanner terrainPlanner_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrainSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr huskyOdomSub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr casualtySub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr huskyStatusSub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr routePub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statePub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_{State::SEARCHING};
    int routeStep_;

    bool terrainReceived_{false};
    bool huskyOdomReceived_{false};
    std::string terrainFrame_;
    double huskyX_{0.0}, huskyY_{0.0};
    double casualtyX_{0.0}, casualtyY_{0.0};
};

#endif
