#ifndef HUSKY_HPP
#define HUSKY_HPP

#include <memory>
#include <vector>

#include "dataStructures.hpp"

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

/// The ground robot: drives a route through Nav2, one waypoint at a time.
class Husky{
public:
    explicit Husky(rclcpp::Node* node);

    /// True once Nav2's NavigateToPose action server is up.
    bool ready() const;

    /// Drive the waypoints (already in the husky1_map frame) in order.
    /// Ignored while a route is already being driven.
    void followRoute(const std::vector<Waypoint>& route);

    /// Stop button: pause() cancels the Nav2 goal and holds. The route and the
    /// current waypoint are kept, so resume() carries on from the same point.
    void pause();
    void resume();
    bool paused() const;

    HuskyStatus status() const;

protected:
private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    void sendNextWaypoint();
    void cancel(GoalHandle::SharedPtr handle);
    void goalResponseCallback(GoalHandle::SharedPtr handle, int seq);
    void resultCallback(const GoalHandle::WrappedResult& result, int seq);

    rclcpp::Node* node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr navClient_;

    std::vector<Waypoint> route_;
    size_t index_{0};
    HuskyStatus status_{HuskyStatus::IDLE};

    GoalHandle::SharedPtr goalHandle_;  // goal Nav2 is driving right now
    bool paused_{false};
    int goalSeq_{0};                    // bumped on every send and every STOP; old replies get ignored
};

#endif