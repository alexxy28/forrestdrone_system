#include "systemControlNode.hpp"

#include "dataStructures.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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
                  declare_parameter<double>("terrain_size_x", 40.0),
                  declare_parameter<double>("terrain_size_y", 40.0)),
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

    // ---- drone tasking ----
    areaX0_       = declare_parameter<double>("area_x0", -17.0);
    areaY0_       = declare_parameter<double>("area_y0", -11.0);
    areaWidth_    = declare_parameter<double>("area_width", 28.0);
    areaHeight_   = declare_parameter<double>("area_height", 22.0);
    laneSpacing_  = declare_parameter<double>("lane_spacing", 4.0);
    clickBoxSize_ = declare_parameter<double>("click_box_size", 8.0);
    missionFrame_ = declare_parameter<std::string>("mission_frame", "world");

    const std::string dronePathTopic   = declare_parameter<std::string>("drone_path_topic", "/parrot1/mission/path");
    const std::string droneStatusTopic = declare_parameter<std::string>("drone_status_topic", "/parrot1/mission/status");
    const std::string clickedTopic     = declare_parameter<std::string>("clicked_point_topic", "/parrot1/clicked_point");

    dronePathPub_ = create_publisher<nav_msgs::msg::Path>(dronePathTopic, latched);

    droneStatusSub_ = create_subscription<std_msgs::msg::String>(
        droneStatusTopic, latched,
        std::bind(&SystemControlNode::droneStatusCallback, this, std::placeholders::_1));

    clickedSub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        clickedTopic, 10,
        std::bind(&SystemControlNode::clickedPointCallback, this, std::placeholders::_1));

    startSrv_ = create_service<std_srvs::srv::Trigger>(
        "/mission/start",
        std::bind(&SystemControlNode::startSearch, this, std::placeholders::_1, std::placeholders::_2));

    stopSrv_ = create_service<std_srvs::srv::Trigger>(
        "/mission/stop",
        std::bind(&SystemControlNode::stopDrone, this, std::placeholders::_1, std::placeholders::_2));

    // While PLANNING, retry once a second: the map may have grown since the last attempt.
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&SystemControlNode::tryPlan, this));

    setState(State::SEARCHING);
    RCLCPP_INFO(get_logger(),
                "Mission control ready. Search area x %.1f..%.1f, y %.1f..%.1f. "
                "Start with: ros2 service call /mission/start std_srvs/srv/Trigger",
                areaX0_, areaX0_ + areaWidth_, areaY0_, areaY0_ + areaHeight_);
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

// ======================= drone tasking =======================

// Lanes run along x and step up in y. The last lane sits exactly on the
// top edge so the whole area is covered.
std::vector<std::pair<double, double>> SystemControlNode::zigzag(double x0, double y0, double width,
                                                                 double height, double spacing) const
{
    std::vector<std::pair<double, double>> pts;
    int lanes = static_cast<int>(std::ceil(height / spacing)) + 1;
    for (int i = 0; i < lanes; i++) {
        double y = std::min(y0 + i * spacing, y0 + height);
        if (i % 2 == 0) {
            pts.emplace_back(x0, y);
            pts.emplace_back(x0 + width, y);
        } else {
            pts.emplace_back(x0 + width, y);
            pts.emplace_back(x0, y);
        }
        if (y >= y0 + height) {
            break;
        }
    }
    return pts;
}

void SystemControlNode::sendDronePath(const std::vector<std::pair<double, double>>& points,
                                      const std::string& reason)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = missionFrame_;
    path.header.stamp = now();
    for (const auto& p : points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = p.first;
        pose.pose.position.y = p.second;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    dronePathPub_->publish(path);
    RCLCPP_INFO(get_logger(), "Drone tasked: %s (%zu waypoints)", reason.c_str(), points.size());
}

void SystemControlNode::startSearch(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (state_ != State::SEARCHING) {
        response->success = false;
        response->message = std::string("Not searching any more (state ") +
                            toString(static_cast<int>(state_)) + ")";
        return;
    }
    auto points = zigzag(areaX0_, areaY0_, areaWidth_, areaHeight_, laneSpacing_);
    sendDronePath(points, "area search");
    response->success = true;
    response->message = "Search started: " + std::to_string(points.size()) + " waypoints";
}

void SystemControlNode::stopDrone(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    sendDronePath({}, "stop and hover");
    response->success = true;
    response->message = "Drone stopping";
}

void SystemControlNode::clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    if (state_ != State::SEARCHING) {
        RCLCPP_WARN(get_logger(), "Ignoring click: casualty already found");
        return;
    }
    if (!msg->header.frame_id.empty()) {
        missionFrame_ = msg->header.frame_id;   // match RViz's fixed frame so the path draws
    }
    double half = clickBoxSize_ / 2.0;
    auto points = zigzag(msg->point.x - half, msg->point.y - half,
                         clickBoxSize_, clickBoxSize_, laneSpacing_);
    RCLCPP_INFO(get_logger(), "Click at (%.1f, %.1f): searching %.0f x %.0f m around it",
                msg->point.x, msg->point.y, clickBoxSize_, clickBoxSize_);
    sendDronePath(points, "search around click");
}

void SystemControlNode::droneStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == droneStatus_) {
        return;
    }
    droneStatus_ = msg->data;
    RCLCPP_INFO(get_logger(), "Drone: %s", droneStatus_.c_str());
    if (droneStatus_ == "HOLDING" && state_ == State::SEARCHING) {
        RCLCPP_INFO(get_logger(), "Drone finished its waypoints. Click in RViz or call /mission/start to search again");
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SystemControlNode>());
    rclcpp::shutdown();
    return 0;
}
