#ifndef HUSKY_HPP
#define HUSKY_HPP

#include <memory>
#include <vector>

#include "dataStructures.hpp"
#include "lidar.hpp"

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

enum class HuskyStatus { IDLE, EN_ROUTE, ARRIVED, FAILED };

/// The ground robot: drives a route through Nav2, one waypoint at a time.
class Husky{
    public:
    explicit Husky(rclcpp::Node* node);
    Lidar& getLidar();

    /// True once Nav2's NavigateToPose action server is up.
    bool ready() const;

    /// Drive the waypoints (already in the husky1_map frame) in order.
    /// Ignored while a route is already being driven.
    void followRoute(const std::vector<Waypoint>& route);

    HuskyStatus status() const;

    protected:
    private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    void sendNextWaypoint();
    void goalResponseCallback(GoalHandle::SharedPtr handle);
    void resultCallback(const GoalHandle::WrappedResult& result);

    rclcpp::Node* node_;
    std::unique_ptr<Lidar> lidar_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr navClient_;

    std::vector<Waypoint> route_;
    size_t index_{0};
    HuskyStatus status_{HuskyStatus::IDLE};
};

#endif
