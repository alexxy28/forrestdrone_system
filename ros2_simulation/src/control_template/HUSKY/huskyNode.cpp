#include "huskyNode.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <tf2/exceptions.h>

HuskyNode::HuskyNode()
    : Node("husky_node"),
      husky_(this),
      tfBuffer_(get_clock()),
      tfListener_(tfBuffer_, this, false)
{
    // Latched so a route or status published before this node started is not lost.
    const auto latched = rclcpp::QoS(1).reliable().transient_local();

    routeSub_ = create_subscription<nav_msgs::msg::Path>(
        topics::ROUTE, latched,
        std::bind(&HuskyNode::routeCallback, this, std::placeholders::_1));

    statusPub_ = create_publisher<std_msgs::msg::String>(topics::HUSKY_STATUS, latched);

    timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&HuskyNode::tick, this));

    RCLCPP_INFO(get_logger(), "husky_node started");
}

void HuskyNode::routeCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    pendingRoute_ = *msg;
}

void HuskyNode::tick()
{
    // Hand a waiting route to the Husky once Nav2 and the TF it needs are up.
    if (pendingRoute_ && husky_.ready()) {
        std::vector<Waypoint> route;
        if (transformRoute(*pendingRoute_, route)) {
            husky_.followRoute(route);
            pendingRoute_.reset();
        }
    }

    // Publish the status only when it changes.
    const HuskyStatus status = husky_.status();
    if (lastPublished_ != status) {
        std_msgs::msg::String msg;
        msg.data = toString(status);
        statusPub_->publish(msg);
        lastPublished_ = status;
    }
}


bool HuskyNode::transformRoute(const nav_msgs::msg::Path& path, std::vector<Waypoint>& out)
{
    try {
        const auto tf = tfBuffer_.lookupTransform("husky1_map", "husky1_odom", tf2::TimePointZero);
        const double yaw = 2.0 * std::atan2(tf.transform.rotation.z, tf.transform.rotation.w);
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);

        out.clear();
        for (const auto& pose : path.poses) {
            const double x = pose.pose.position.x;
            const double y = pose.pose.position.y;
            out.push_back({c * x - s * y + tf.transform.translation.x,
                           s * x + c * y + tf.transform.translation.y});
        }
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Waiting for husky1_odom -> husky1_map TF: %s", ex.what());
        return false;
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HuskyNode>());
    rclcpp::shutdown();
    return 0;
}
