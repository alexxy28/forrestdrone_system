#include "husky.hpp"

#include <cmath>
#include <exception>
#include <functional>

Husky::Husky(rclcpp::Node* node)
    : node_(node),
      navClient_(rclcpp_action::create_client<NavigateToPose>(node, topics::HUSKY_NAVIGATE))
{
}

bool Husky::ready() const
{
    return navClient_->action_server_is_ready();
}

HuskyStatus Husky::status() const
{
    return status_;
}

bool Husky::paused() const
{
    return paused_;
}

void Husky::followRoute(const std::vector<Waypoint>& route)
{
    if (status_ == HuskyStatus::EN_ROUTE) {
        return;
    }
    route_ = route;
    index_ = 0;
    status_ = HuskyStatus::EN_ROUTE;

    // Stopped: keep the route, but don't drive until START.
    if (paused_) {
        RCLCPP_WARN(node_->get_logger(), "Route received while stopped: waiting for START");
        return;
    }
    sendNextWaypoint();
}

void Husky::pause()
{
    if (paused_) {
        return;   // already stopped, don't spam the log
    }
    paused_ = true;
    ++goalSeq_;   // any reply to the goal in flight is now out of date

    if (goalHandle_) {
        cancel(goalHandle_);
        goalHandle_.reset();
    }
    RCLCPP_WARN(node_->get_logger(), "STOP pressed: holding position");
}

void Husky::resume()
{
    if (!paused_) {
        return;
    }
    paused_ = false;
    RCLCPP_INFO(node_->get_logger(), "STOP released: resuming");

    // index_ wasn't touched, so this re-sends the waypoint it was driving to.
    if (status_ == HuskyStatus::EN_ROUTE) {
        sendNextWaypoint();
    }
}

void Husky::cancel(GoalHandle::SharedPtr handle)
{
    try {
        navClient_->async_cancel_goal(handle);
    } catch (const std::exception& e) {
        // Goal already finished, nothing to cancel.
        RCLCPP_DEBUG(node_->get_logger(), "Cancel skipped: %s", e.what());
    }
}

void Husky::sendNextWaypoint()
{
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

    const int seq = ++goalSeq_;
    goalHandle_.reset();

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = std::bind(&Husky::goalResponseCallback, this, std::placeholders::_1, seq);
    options.result_callback = std::bind(&Husky::resultCallback, this, std::placeholders::_1, seq);
    navClient_->async_send_goal(goal, options);
}

void Husky::goalResponseCallback(GoalHandle::SharedPtr handle, int seq)
{
    // STOP was pressed before Nav2 accepted this goal: cancel it now.
    if (seq != goalSeq_) {
        if (handle) {
            cancel(handle);
        }
        return;
    }
    if (!handle) {
        RCLCPP_ERROR(node_->get_logger(), "Nav2 rejected the goal");
        status_ = HuskyStatus::FAILED;
        return;
    }
    goalHandle_ = handle;
}

void Husky::resultCallback(const GoalHandle::WrappedResult& result, int seq)
{
    // Reply to a goal that STOP already cancelled: ignore it.
    if (seq != goalSeq_) {
        return;
    }
    goalHandle_.reset();

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        ++index_;
        sendNextWaypoint();
        return;
    }
    RCLCPP_ERROR(node_->get_logger(), "Navigation failed at waypoint %zu/%zu", index_ + 1, route_.size());
    status_ = HuskyStatus::FAILED;
}