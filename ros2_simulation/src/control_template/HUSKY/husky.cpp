#include "husky.hpp"

#include <cmath>
#include <functional>

Husky::Husky(rclcpp::Node* node)
    : node_(node),
      navClient_(rclcpp_action::create_client<NavigateToPose>(node, "/husky1/navigate_to_pose")){
}



bool Husky::ready() const{
    return navClient_->action_server_is_ready();
}

HuskyStatus Husky::status() const{
    return status_;
}

void Husky::followRoute(const std::vector<Waypoint>& route){
    if (status_ == HuskyStatus::EN_ROUTE) {
        return;
    }
    route_ = route;
    index_ = 0;
    status_ = HuskyStatus::EN_ROUTE;
    sendNextWaypoint();
}

void Husky::sendNextWaypoint(){
    if (index_ >= route_.size()) {
        status_ = HuskyStatus::ARRIVED;
        RCLCPP_INFO(node_->get_logger(), "Route complete");
        return;
    }

    const Waypoint& wp = route_[index_];

    // Face the next waypoint; the last one keeps the heading it arrived with.
    const Waypoint& from = (index_ + 1 < route_.size()) ? wp : (index_ > 0 ? route_[index_ - 1] : wp);
    const Waypoint& to = (index_ + 1 < route_.size()) ? route_[index_ + 1] : wp;
    const double yaw = std::atan2(to.y - from.y, to.x - from.x);

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "husky1_map";
    goal.pose.header.stamp = node_->now();
    goal.pose.pose.position.x = wp.x;
    goal.pose.pose.position.y = wp.y;
    goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

    RCLCPP_INFO(node_->get_logger(), "Waypoint %zu/%zu: (%.2f, %.2f)", index_ + 1, route_.size(), wp.x, wp.y);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = std::bind(&Husky::goalResponseCallback, this, std::placeholders::_1);
    options.result_callback = std::bind(&Husky::resultCallback, this, std::placeholders::_1);
    navClient_->async_send_goal(goal, options);
}

void Husky::goalResponseCallback(GoalHandle::SharedPtr handle){
    if (!handle) {
        RCLCPP_ERROR(node_->get_logger(), "Nav2 rejected the goal");
        status_ = HuskyStatus::FAILED;
    }
}

void Husky::resultCallback(const GoalHandle::WrappedResult& result){
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        ++index_;
        sendNextWaypoint();
        return;
    }
    RCLCPP_ERROR(node_->get_logger(), "Navigation failed at waypoint %zu/%zu", index_ + 1, route_.size());
    status_ = HuskyStatus::FAILED;
}
