#include "huskyNode.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <tf2/exceptions.h>

namespace {

// Heading (yaw) of a quaternion. Exact on slopes too, where x and y are not 0.
template <typename Q>
double yawOf(const Q& q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

// The simulator publishes each robot's transforms under its own namespace,
// while the TF listener always subscribes to /tf and /tf_static.
HuskyNode::HuskyNode()
    : Node("husky_node", rclcpp::NodeOptions().arguments({
          "--ros-args",
          "-r", std::string("/tf:=") + topics::HUSKY_TF,
          "-r", std::string("/tf_static:=") + topics::HUSKY_TF_STATIC})),
      husky_(this),
      tfBuffer_(get_clock()),
      tfListener_(tfBuffer_, this, false)
{
    navFrame_ = declare_parameter<std::string>("nav_frame", "husky1_odom");
    husky_.setFrameConverter([this](const Waypoint& world, Waypoint& map) {
        return worldToMap(world, map);
    }, navFrame_);

    // Latched so a route or status published before this node started is not lost.
    const auto latched = rclcpp::QoS(1).reliable().transient_local();

    routeSub_ = create_subscription<nav_msgs::msg::Path>(
        topics::ROUTE, latched,
        std::bind(&HuskyNode::routeCallback, this, std::placeholders::_1));

    // Raw Gazebo pose (world frame), used to convert the route into nav_frame.
    odomSub_ = create_subscription<nav_msgs::msg::Odometry>(
        topics::HUSKY_ODOM, rclcpp::SensorDataQoS(),
        std::bind(&HuskyNode::odomCallback, this, std::placeholders::_1));

    statusPub_ = create_publisher<std_msgs::msg::String>(topics::HUSKY_STATUS, latched);

    // Confirmed casualty (world frame), for the final-approach check.
    casualtySub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        topics::CASUALTY, latched,
        std::bind(&HuskyNode::casualtyCallback, this, std::placeholders::_1));
    // Distances are to the detected body centre. Aim for 2.0 m: with Nav2's
    // 0.5 m goal tolerance the Husky stops 1.5-2.5 m away. The body reaches
    // ~0.9 m from its centre along its length and the Husky's bumper ~0.5 m
    // from the Husky's centre, so even approaching along the body (from the
    // head or feet) the bumper stays clear of the person.
    approachMax_ = declare_parameter<double>("approach_max_m", 2.5);
    approachStandoff_ = declare_parameter<double>("approach_standoff_m", 2.0);
    approachRange_ = declare_parameter<double>("approach_range_m", 6.0);
    maxApproachAttempts_ = declare_parameter<int>("approach_attempts", 3);

    // Arrival is only reported once the Husky has really stopped, and it is
    // watched afterwards: in Gazebo (27-28 Oct) it rolled backwards down a
    // slope after "ARRIVED" and ended 5.7 m from the person while the
    // dashboard still said ARRIVED.
    stillSpeed_ = declare_parameter<double>("still_speed_mps", 0.01);
    stillTime_ = declare_parameter<double>("still_time_s", 2.0);
    settleTimeout_ = declare_parameter<double>("settle_timeout_s", 20.0);
    holdDrift_ = declare_parameter<double>("hold_drift_m", 0.3);
    maxHoldReapproaches_ = declare_parameter<int>("hold_reapproaches", 2);
    holdPub_ = create_publisher<geometry_msgs::msg::PointStamped>(topics::HUSKY_HOLD, latched);

    // Stop button: true = cancel the Nav2 goal and hold, false = carry on.
    // rqt -> /mission_control/husky1/estop -> system_control_node -> here.
    const std::string stopTopic = declare_parameter<std::string>("stop_topic", topics::HUSKY_ESTOP);
    stopSub_ = create_subscription<std_msgs::msg::Bool>(
        stopTopic, 10,
        std::bind(&HuskyNode::onStop, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&HuskyNode::tick, this));

    RCLCPP_INFO(get_logger(), "husky_node started");
}

void HuskyNode::routeCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    pendingRoute_ = *msg;
}

void HuskyNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    worldOdom_ = msg;

    // Ground speed from position change over >= 0.5 s of sim time. More
    // honest than the twist field for a slow creep, and needs no velocity
    // estimate from the simulator.
    const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
    const Waypoint p{msg->pose.pose.position.x, msg->pose.pose.position.y};
    if (!speedRef_) {
        speedRef_ = p;
        speedRefTime_ = stamp;
        return;
    }
    const double dt = (stamp - speedRefTime_).seconds();
    if (dt < 0.0) {             // sim time went backwards (Gazebo reset)
        speedRef_ = p;
        speedRefTime_ = stamp;
    } else if (dt >= 0.5) {
        speed_ = std::hypot(p.x - speedRef_->x, p.y - speedRef_->y) / dt;
        speedRef_ = p;
        speedRefTime_ = stamp;
    }
}

void HuskyNode::casualtyCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    casualty_ = Waypoint{msg->point.x, msg->point.y};
}

double HuskyNode::distanceToCasualty() const
{
    if (!casualty_ || !worldOdom_) {
        return -1.0;
    }
    return std::hypot(worldOdom_->pose.pose.position.x - casualty_->x,
                      worldOdom_->pose.pose.position.y - casualty_->y);
}

void HuskyNode::onStop(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        husky_.pause();
    } else {
        husky_.resume();
    }
}

void HuskyNode::tick()
{
    husky_.update();

    // Hand a waiting route to the Husky once Nav2 and the TF it needs are up.
    if (pendingRoute_ && !husky_.ready()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Route received but Nav2 (%s) is not active yet: waiting",
                             topics::HUSKY_NAVIGATE);
    }
    if (pendingRoute_ && husky_.ready() && husky_.status() != HuskyStatus::EN_ROUTE && updateOffset()) {
        std::vector<Waypoint> route;
        for (const auto& pose : pendingRoute_->poses) {
            route.push_back({pose.pose.position.x, pose.pose.position.y});
        }
        const Waypoint here{worldOdom_->pose.pose.position.x, worldOdom_->pose.pose.position.y};
        husky_.followRoute(route, &here);
        pendingRoute_.reset();
        phase_ = Phase::ROUTE;
        approachAttempts_ = 0;
        holdReapproaches_ = 0;
        holdPoint_.reset();
    }

    // The route (or an approach leg) just ended: check how close we really are.
    const HuskyStatus nav = husky_.status();
    if ((phase_ == Phase::ROUTE || phase_ == Phase::APPROACH) &&
        (nav == HuskyStatus::ARRIVED || nav == HuskyStatus::FAILED)) {
        routeEnded(nav);
    }
    if (phase_ == Phase::SETTLING) {
        settle();
    } else if (phase_ == Phase::HOLDING) {
        watchHold();
    }

    // Publish the status only when it changes. While driving, approaching or
    // settling it is EN_ROUTE (STOPPED if the stop button is held). ARRIVED
    // only while the Husky is verified to be holding its stop position.
    HuskyStatus status = HuskyStatus::IDLE;
    if (phase_ == Phase::ROUTE || phase_ == Phase::APPROACH || phase_ == Phase::SETTLING) {
        status = husky_.paused() ? HuskyStatus::STOPPED : HuskyStatus::EN_ROUTE;
    } else if (phase_ == Phase::HOLDING) {
        status = HuskyStatus::ARRIVED;
    } else if (phase_ == Phase::FINISHED) {
        status = finalStatus_;
    }
    if (lastPublished_ != status) {
        std_msgs::msg::String msg;
        msg.data = toString(status);
        statusPub_->publish(msg);
        lastPublished_ = status;
    }
}

void HuskyNode::routeEnded(HuskyStatus result)
{
    const double d = distanceToCasualty();
    const char* what = result == HuskyStatus::ARRIVED ? "reached" : "failed at";
    const double hx = worldOdom_ ? worldOdom_->pose.pose.position.x : 0.0;
    const double hy = worldOdom_ ? worldOdom_->pose.pose.position.y : 0.0;

    auto finish = [&](HuskyStatus verdict) {
        finalStatus_ = verdict;
        phase_ = Phase::FINISHED;
        if (d >= 0.0) {
            RCLCPP_INFO(get_logger(), "Final Husky position (world): (%.2f, %.2f), %.2f m from the casualty "
                        "at (%.2f, %.2f): %s", hx, hy, d, casualty_->x, casualty_->y, toString(verdict));
        }
    };

    if (d < 0.0) {
        // No casualty position or odometry: trust Nav2's verdict.
        finish(result);
        return;
    }
    RCLCPP_INFO(get_logger(), "%s %s last goal: Husky at (%.2f, %.2f), %.2f m from the casualty",
                phase_ == Phase::APPROACH ? "Approach" : "Route", what, hx, hy, d);

    if (d <= approachMax_) {
        if (d < 1.2) {
            RCLCPP_WARN(get_logger(), "Husky stopped only %.2f m from the casualty's centre", d);
        }
        startSettling();
        return;
    }
    if (d <= approachRange_ && approachAttempts_ < maxApproachAttempts_ && startApproach(d, hx, hy)) {
        return;
    }
    // Close enough to help even if not at the ideal stand-off; otherwise
    // report FAILED so mission control can replan.
    if (d <= approachRange_) {
        RCLCPP_WARN(get_logger(), "Could not get closer than %.2f m to the casualty: "
                    "stopping at the closest reachable point", d);
        startSettling();
    } else {
        finish(HuskyStatus::FAILED);
    }
}

// Goal approach_standoff_m from the casualty, on the side the Husky is on.
bool HuskyNode::startApproach(double d, double hx, double hy)
{
    if (!casualty_ || d <= 1e-6 || !updateOffset()) {
        return false;
    }
    const double ux = (hx - casualty_->x) / d;
    const double uy = (hy - casualty_->y) / d;
    // At least 0.8 m ahead of the Husky: a goal within Nav2's 0.5 m tolerance
    // counts as reached at once and the Husky doesn't move (seen in Gazebo,
    // 28 Oct: three "approaches" from 2.52 m, none moved). Never nearer than
    // 1.6 m to the casualty.
    const double standoff = std::max(1.6, std::min(approachStandoff_, d - 0.8));
    std::vector<Waypoint> goal{{casualty_->x + ux * standoff,
                                casualty_->y + uy * standoff}};
    ++approachAttempts_;
    RCLCPP_INFO(get_logger(), "Final approach %d: %.2f m away, driving to (%.2f, %.2f), "
                "%.1f m from the casualty", approachAttempts_, d,
                goal[0].x, goal[0].y, standoff);
    const Waypoint here{hx, hy};
    husky_.followRoute(goal, &here);
    phase_ = Phase::APPROACH;
    return true;
}

// Nav2 has finished close enough: wait until the Husky has really stopped.
void HuskyNode::startSettling()
{
    phase_ = Phase::SETTLING;
    settleStart_ = now();
    stillSince_.reset();
    RCLCPP_INFO(get_logger(), "Nav2 finished %.2f m from the casualty: checking the Husky has stopped",
                distanceToCasualty());
}

void HuskyNode::settle()
{
    const rclcpp::Time t = now();
    if (speed_ <= stillSpeed_) {
        if (!stillSince_) {
            stillSince_ = t;
        }
    } else {
        stillSince_.reset();
    }
    const bool still = stillSince_ && (t - *stillSince_).seconds() >= stillTime_;
    const bool timedOut = (t - settleStart_).seconds() >= settleTimeout_;
    if (!still && !timedOut) {
        return;
    }
    const double hx = worldOdom_->pose.pose.position.x;
    const double hy = worldOdom_->pose.pose.position.y;
    const double d = distanceToCasualty();
    if (!still) {
        RCLCPP_WARN(get_logger(), "Husky still moving at %.3f m/s %.0f s after Nav2 finished",
                    speed_, settleTimeout_);
    }
    if (d > approachMax_ + holdDrift_) {
        // It moved away while settling: drive back.
        RCLCPP_WARN(get_logger(), "Husky is %.2f m from the casualty after stopping: re-approaching", d);
        if (approachAttempts_ < maxApproachAttempts_ + maxHoldReapproaches_ && startApproach(d, hx, hy)) {
            return;
        }
        finalStatus_ = HuskyStatus::FAILED;
        phase_ = Phase::FINISHED;
        RCLCPP_ERROR(get_logger(), "Husky cannot stay within %.1f m of the casualty: FAILED", approachMax_);
        return;
    }
    holdPoint_ = Waypoint{hx, hy};
    phase_ = Phase::HOLDING;
    geometry_msgs::msg::PointStamped hold;
    hold.header.frame_id = "world";
    hold.header.stamp = t;
    hold.point.x = hx;
    hold.point.y = hy;
    holdPub_->publish(hold);
    RCLCPP_INFO(get_logger(), "Final Husky position (world): (%.2f, %.2f), %.2f m from the casualty "
                "at (%.2f, %.2f), stationary (%.3f m/s): ARRIVED", hx, hy, d, casualty_->x, casualty_->y,
                speed_);
}

// After ARRIVED: the Husky must stay where it stopped.
void HuskyNode::watchHold()
{
    const double hx = worldOdom_->pose.pose.position.x;
    const double hy = worldOdom_->pose.pose.position.y;
    const double drift = std::hypot(hx - holdPoint_->x, hy - holdPoint_->y);
    const double d = distanceToCasualty();
    if (drift <= holdDrift_ && d <= approachMax_ + holdDrift_) {
        return;
    }
    RCLCPP_WARN(get_logger(), "Husky moved %.2f m from where it stopped and is now %.2f m from the "
                "casualty: no longer ARRIVED", drift, d);
    if (holdReapproaches_ < maxHoldReapproaches_) {
        ++holdReapproaches_;
        approachAttempts_ = 0;
        if (startApproach(d, hx, hy)) {
            return;
        }
    }
    finalStatus_ = HuskyStatus::FAILED;
    phase_ = Phase::FINISHED;
    RCLCPP_ERROR(get_logger(), "Husky cannot hold its position near the casualty: FAILED");
}

// world -> nav_frame: T_nav_world = T_nav_base * inverse(T_world_base).
// Re-measured before every waypoint, so drift while driving is removed.
bool HuskyNode::updateOffset()
{
    if (!worldOdom_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Waiting for %s to convert the route into %s", topics::HUSKY_ODOM,
                             navFrame_.c_str());
        return haveOffset_;
    }

    geometry_msgs::msg::TransformStamped mapFromBase;
    try {
        mapFromBase = tfBuffer_.lookupTransform(navFrame_, "husky1_base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Waiting for %s -> husky1_base_link TF: %s%s", navFrame_.c_str(), ex.what(),
                             haveOffset_ ? " (using the last offset)" : "");
        return haveOffset_;
    }

    // Husky pose in the world (raw Gazebo odometry) and in nav_frame (TF).
    const auto& wp = worldOdom_->pose.pose;
    const double wx = wp.position.x;
    const double wy = wp.position.y;
    const double wyaw = yawOf(wp.orientation);

    const auto& mt = mapFromBase.transform;
    const double mx = mt.translation.x;
    const double my = mt.translation.y;
    const double myaw = yawOf(mt.rotation);

    // Rotation and translation that take a world point into nav_frame.
    const double dyaw = myaw - wyaw;
    offC_ = std::cos(dyaw);
    offS_ = std::sin(dyaw);
    offTx_ = mx - (offC_ * wx - offS_ * wy);
    offTy_ = my - (offS_ * wx + offC_ * wy);
    haveOffset_ = true;

    if (std::hypot(offTx_ - loggedTx_, offTy_ - loggedTy_) > 0.3) {
        RCLCPP_INFO(get_logger(), "world -> %s offset: (%.2f, %.2f) m, %.1f deg",
                    navFrame_.c_str(), offTx_, offTy_, dyaw * 180.0 / M_PI);
        loggedTx_ = offTx_;
        loggedTy_ = offTy_;
    }
    return true;
}

bool HuskyNode::worldToMap(const Waypoint& world, Waypoint& map)
{
    if (!updateOffset()) {
        return false;
    }
    map.x = offC_ * world.x - offS_ * world.y + offTx_;
    map.y = offS_ * world.x + offC_ * world.y + offTy_;
    return true;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HuskyNode>());
    rclcpp::shutdown();
    return 0;
}
