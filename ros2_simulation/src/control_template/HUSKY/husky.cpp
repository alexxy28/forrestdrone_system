#include "husky.hpp"

#include <action_msgs/msg/goal_status.hpp>
#include <cmath>
#include <exception>
#include <functional>

Husky::Husky(rclcpp::Node* node)
    : node_(node),
      navClient_(rclcpp_action::create_client<NavigateToPose>(node, topics::HUSKY_NAVIGATE))
{
}

void Husky::setFrameConverter(FrameConverter converter, const std::string& frame)
{
    toMap_ = std::move(converter);
    goalFrame_ = frame;
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

void Husky::followRoute(const std::vector<Waypoint>& route, const Waypoint* start)
{
    if (status_ == HuskyStatus::EN_ROUTE) {
        return;
    }
    if (route.empty()) {
        status_ = HuskyStatus::ARRIVED;
        return;
    }
    route_ = route;
    haveStart_ = start != nullptr;
    if (start) {
        start_ = *start;
    }
    index_ = 0;
    consecutiveSkips_ = 0;
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

void Husky::update()
{
    const rclcpp::Time t = steady_.now();
    if (sendNextAt_ && t >= *sendNextAt_) {
        sendNextAt_.reset();
        if (status_ == HuskyStatus::EN_ROUTE && !paused_) {
            sendNextWaypoint();
        }
        return;
    }
    if (status_ != HuskyStatus::EN_ROUTE || paused_ || sendNextAt_) {
        return;
    }

    if (goalHandle_ && !goalStarted_) {
        const int8_t st = goalHandle_->get_status();
        if (st == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
            goalStarted_ = true;
            resends_ = 0;
        }
    }
    // Nav2 reports the goal finished but the result never arrived: use the status.
    if (goalHandle_) {
        const int8_t st = goalHandle_->get_status();
        const bool terminal = st == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
                              st == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
                              st == action_msgs::msg::GoalStatus::STATUS_CANCELED;
        if (!terminal) {
            terminalSeenAt_.reset();
        } else if (!terminalSeenAt_) {
            terminalSeenAt_ = t;
        } else if ((t - *terminalSeenAt_).seconds() > GOAL_START_TIMEOUT_S) {
            RCLCPP_WARN(node_->get_logger(), "Nav2 finished waypoint %zu/%zu but its result never arrived: "
                        "continuing from its status", index_ + 1, route_.size());
            terminalSeenAt_.reset();
            ++goalSeq_;   // a late result for this goal is now ignored
            goalHandle_.reset();
            handleResult(st == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED
                             ? rclcpp_action::ResultCode::SUCCEEDED
                             : (st == action_msgs::msg::GoalStatus::STATUS_CANCELED
                                    ? rclcpp_action::ResultCode::CANCELED
                                    : rclcpp_action::ResultCode::ABORTED));
            return;
        }
    }
    // The goal was never answered, or accepted but never started: send it again.
    if (!goalStarted_ && (t - goalSentAt_).seconds() > GOAL_START_TIMEOUT_S) {
        if (resends_ >= MAX_RESENDS) {
            waypointFailed("Nav2 did not start it");
            return;
        }
        ++resends_;
        RCLCPP_WARN(node_->get_logger(), "Nav2 has not started waypoint %zu/%zu after %.0f s (%s): sending "
                    "it again (%d/%d)", index_ + 1, route_.size(), GOAL_START_TIMEOUT_S,
                    goalHandle_ ? "accepted, not executing" : "no reply", resends_, MAX_RESENDS);
        if (goalHandle_) {
            cancel(goalHandle_);
            goalHandle_.reset();
        }
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

    // Face the next waypoint; the last one keeps the heading it arrived with
    // (for a one-waypoint route: the heading from where the Husky started).
    const Waypoint& from = (index_ + 1 < route_.size())
        ? wp
        : (index_ > 0 ? route_[index_ - 1] : (haveStart_ ? start_ : wp));
    const Waypoint& to = (index_ + 1 < route_.size()) ? route_[index_ + 1] : wp;

    // World -> Nav2 frame with the offset measured now.
    Waypoint mwp = wp, mfrom = from, mto = to;
    if (toMap_ && !(toMap_(wp, mwp) && toMap_(from, mfrom) && toMap_(to, mto))) {
        RCLCPP_ERROR(node_->get_logger(), "No world -> %s transform: cannot send waypoint", goalFrame_.c_str());
        status_ = HuskyStatus::FAILED;
        return;
    }
    const double yaw = std::atan2(mto.y - mfrom.y, mto.x - mfrom.x);

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = goalFrame_;
    goal.pose.header.stamp = node_->now();
    goal.pose.pose.position.x = mwp.x;
    goal.pose.pose.position.y = mwp.y;
    goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

    RCLCPP_INFO(node_->get_logger(), "Waypoint %zu/%zu: world (%.2f, %.2f) = %s (%.2f, %.2f)",
                index_ + 1, route_.size(), wp.x, wp.y, goalFrame_.c_str(), mwp.x, mwp.y);

    const int seq = ++goalSeq_;
    goalHandle_.reset();
    goalSentAt_ = steady_.now();
    goalStarted_ = false;
    terminalSeenAt_.reset();

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = std::bind(&Husky::goalResponseCallback, this, std::placeholders::_1, seq);
    options.result_callback = std::bind(&Husky::resultCallback, this, std::placeholders::_1, seq);
    // Nav2 streams feedback while it drives a goal: that is the reliable
    // "started" signal (in Gazebo the client-side goal status stayed ACCEPTED).
    options.feedback_callback = [this, seq](GoalHandle::SharedPtr,
                                            const std::shared_ptr<const NavigateToPose::Feedback>) {
        if (seq == goalSeq_ && !goalStarted_) {
            goalStarted_ = true;
            resends_ = 0;
        }
    };
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
        waypointFailed("rejected by Nav2");
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
    handleResult(result.code);
}

void Husky::handleResult(rclcpp_action::ResultCode code)
{
    if (code == rclcpp_action::ResultCode::SUCCEEDED) {
        consecutiveSkips_ = 0;
        resends_ = 0;
        ++index_;
        // Send the next goal a moment later, not from inside Nav2's result
        // callback (see the lost-goal note in husky.hpp).
        sendNextAt_ = steady_.now() + rclcpp::Duration::from_seconds(NEXT_GOAL_DELAY_S);
        return;
    }

    waypointFailed(code == rclcpp_action::ResultCode::CANCELED ? "cancelled" : "aborted by Nav2");
}

// An intermediate waypoint Nav2 can't reach (e.g. it landed in a costmap
// obstacle): try the next one rather than abandoning the casualty. A failed
// final waypoint ends the route as FAILED; husky_node then decides whether
// the Husky is already close enough or mission control should replan.
void Husky::waypointFailed(const char* why)
{
    const bool last = index_ + 1 >= route_.size();
    if (!last && consecutiveSkips_ < MAX_CONSECUTIVE_SKIPS) {
        ++consecutiveSkips_;
        RCLCPP_WARN(node_->get_logger(), "Waypoint %zu/%zu %s, skipping to the next one",
                    index_ + 1, route_.size(), why);
        ++index_;
        sendNextAt_ = steady_.now() + rclcpp::Duration::from_seconds(NEXT_GOAL_DELAY_S);
        return;
    }
    RCLCPP_ERROR(node_->get_logger(), "Navigation failed at waypoint %zu/%zu (%s)",
                 index_ + 1, route_.size(), why);
    status_ = HuskyStatus::FAILED;
}
