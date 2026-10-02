#include "systemControlNode.hpp"

#include "dataStructures.hpp"

#include <chrono>
#include <functional>
#include <utility>
#include <vector>

namespace {

const char* toString(int state)
{
    static const char* names[] = {"SEARCHING", "PLANNING", "HUSKY_EN_ROUTE", "DONE", "FAILED"};
    return names[state];
}

// Keep every `step`-th point plus the last, so Nav2 gets fewer, longer legs.
std::vector<std::pair<double, double>> simplifyRoute(
    const std::vector<std::pair<double, double>>& input, std::size_t step)
{
    if (input.size() <= 2 || step <= 1) {
        return input;
    }

    std::vector<std::pair<double, double>> output;
    output.push_back(input.front());
    for (std::size_t i = step; i + 1 < input.size(); i += step) {
        output.push_back(input[i]);
    }
    if (output.back() != input.back()) {
        output.push_back(input.back());
    }
    return output;
}

}  // namespace

SystemControlNode::SystemControlNode()
    : Node("system_control_node"),
      terrainMap_(declare_parameter<double>("terrain_resolution", 0.5),
                  declare_parameter<double>("terrain_size_x", 20.0),
                  declare_parameter<double>("terrain_size_y", 20.0)),
      terrainPlanner_(terrainMap_),
      routeStep_(declare_parameter<int>("route_step", 2))
{
    // Latched so a node that starts late still sees the casualty, route and status.
    const auto latched = rclcpp::QoS(1).reliable().transient_local();

    terrainSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topics::PARROT_TERRAIN, rclcpp::SensorDataQoS(),
        std::bind(&SystemControlNode::terrainCallback, this, std::placeholders::_1));

    huskyOdomSub_ = create_subscription<nav_msgs::msg::Odometry>(
        topics::HUSKY_ODOM, rclcpp::SensorDataQoS(),
        std::bind(&SystemControlNode::huskyOdomCallback, this, std::placeholders::_1));

    casualtySub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        topics::CASUALTY, latched,
        std::bind(&SystemControlNode::casualtyCallback, this, std::placeholders::_1));

    huskyStatusSub_ = create_subscription<std_msgs::msg::String>(
        topics::HUSKY_STATUS, latched,
        std::bind(&SystemControlNode::huskyStatusCallback, this, std::placeholders::_1));

    routePub_ = create_publisher<nav_msgs::msg::Path>(topics::ROUTE, latched);
    statePub_ = create_publisher<std_msgs::msg::String>(topics::STATE, latched);

    // While PLANNING, retry once a second: the map may have grown since the last attempt.
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&SystemControlNode::tryPlan, this));

    setState(State::SEARCHING);
}

void SystemControlNode::terrainCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    terrainFrame_ = msg->header.frame_id;
    terrainMap_.updateFromPointCloud(*msg);
    terrainReceived_ = true;
}

void SystemControlNode::huskyOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    huskyX_ = msg->pose.pose.position.x;
    huskyY_ = msg->pose.pose.position.y;
    huskyOdomReceived_ = true;
}

void SystemControlNode::casualtyCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    if (state_ != State::SEARCHING) {
        return;  // first confirmed casualty wins
    }
    casualtyX_ = msg->point.x;
    casualtyY_ = msg->point.y;
    RCLCPP_INFO(get_logger(), "Casualty at (%.2f, %.2f)", casualtyX_, casualtyY_);
    setState(State::PLANNING);
}

void SystemControlNode::huskyStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (state_ != State::HUSKY_EN_ROUTE) {
        return;
    }
    if (msg->data == toString(HuskyStatus::ARRIVED)) {
        setState(State::DONE);
    } else if (msg->data == toString(HuskyStatus::FAILED)) {
        setState(State::FAILED);
    }
}

void SystemControlNode::tryPlan()
{
    if (state_ != State::PLANNING) {
        return;
    }
    if (!terrainReceived_ || !terrainMap_.hasTerrain() || !huskyOdomReceived_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "Planning: waiting for terrain and Husky odometry");
        return;
    }

    // ponytail: assumes the terrain frame and the Husky odometry share world
    // coordinates (true in this sim); add a TF lookup if they ever differ.
    auto route = terrainPlanner_.plan(huskyX_, huskyY_, casualtyX_, casualtyY_);
    if (route.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "No traversable route yet, retrying");
        return;
    }

    route = simplifyRoute(route, routeStep_);

    nav_msgs::msg::Path path;
    path.header.frame_id = terrainFrame_;
    path.header.stamp = now();
    // Skip index 0: that is where the Husky already is.
    for (std::size_t i = 1; i < route.size(); ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = route[i].first;
        pose.pose.position.y = route[i].second;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }

    RCLCPP_INFO(get_logger(), "Route ready: %zu waypoints", path.poses.size());
    routePub_->publish(path);
    setState(State::HUSKY_EN_ROUTE);
}

void SystemControlNode::setState(State state)
{
    state_ = state;
    std_msgs::msg::String msg;
    msg.data = toString(static_cast<int>(state));
    statePub_->publish(msg);
    RCLCPP_INFO(get_logger(), "State: %s", msg.data.c_str());
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SystemControlNode>());
    rclcpp::shutdown();
    return 0;
}
