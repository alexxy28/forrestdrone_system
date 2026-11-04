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
      // 0.5 m cells: keep every 6th = a waypoint about every 3 m. With Nav2's
      // 1 m goal tolerance, 1 m spacing made the Husky stop-start at every point.
      routeStep_(declare_parameter<int>("route_step", 6))
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
    // ±18 m covers the ±20 m terrain_demo mesh once the LiDAR's ~4 m side reach is added.
    areaX0_       = declare_parameter<double>("area_x0", -18.0);
    areaY0_       = declare_parameter<double>("area_y0", -18.0);
    areaWidth_    = declare_parameter<double>("area_width", 36.0);
    areaHeight_   = declare_parameter<double>("area_height", 36.0);
    autoStart_    = declare_parameter<bool>("auto_start", false);
    laneSpacing_  = declare_parameter<double>("lane_spacing", 6.0);
    clickBoxSize_ = declare_parameter<double>("click_box_size", 8.0);
    missionFrame_ = declare_parameter<std::string>("mission_frame", "world");
    // husky_node stops 1.6-2.5 m from the casualty; DONE needs <= this.
    arrivalMax_   = declare_parameter<double>("arrival_max_m", 3.0);

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

    // ---- UI stop button: Ben's UI -> mission control -> drone ----
    parrotEstopPub_ = create_publisher<std_msgs::msg::Bool>(topics::PARROT_ESTOP, 10);
    uiParrotEstopSub_ = create_subscription<std_msgs::msg::Bool>(
        topics::UI_PARROT_ESTOP, 10,
        std::bind(&SystemControlNode::uiParrotEstopCallback, this, std::placeholders::_1));

    // ---- UI stop button: Ben's UI -> mission control -> Husky ----
    huskyEstopPub_ = create_publisher<std_msgs::msg::Bool>(topics::HUSKY_ESTOP, 10);
    uiHuskyEstopSub_ = create_subscription<std_msgs::msg::Bool>(
        topics::UI_HUSKY_ESTOP, 10,
        std::bind(&SystemControlNode::uiHuskyEstopCallback, this, std::placeholders::_1));

    // ---- second drone (parrot2): its own path, status and stop topics ----
    secondDrone_ = declare_parameter<bool>("second_drone", false);
    if (secondDrone_) {
        drone2PathPub_ = create_publisher<nav_msgs::msg::Path>(topics::PARROT2_MISSION_PATH, latched);
        drone2StatusSub_ = create_subscription<std_msgs::msg::String>(
            topics::PARROT2_MISSION_STATUS, latched,
            std::bind(&SystemControlNode::drone2StatusCallback, this, std::placeholders::_1));
        parrot2EstopPub_ = create_publisher<std_msgs::msg::Bool>(topics::PARROT2_ESTOP, 10);
        uiParrot2EstopSub_ = create_subscription<std_msgs::msg::Bool>(
            topics::UI_PARROT2_ESTOP, 10,
            std::bind(&SystemControlNode::uiParrot2EstopCallback, this, std::placeholders::_1));
    }

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
                "Terrain map %.1f x %.1f m. "
                "Start with: ros2 service call /mission/start std_srvs/srv/Trigger",
                areaX0_, areaX0_ + areaWidth_, areaY0_, areaY0_ + areaHeight_,
                get_parameter("terrain_size_x").as_double(),
                get_parameter("terrain_size_y").as_double());
    if (secondDrone_) {
        RCLCPP_INFO(get_logger(),
                    "Second drone ON: parrot1 searches x %.1f..%.1f, parrot2 searches x %.1f..%.1f",
                    areaX0_, areaX0_ + areaWidth_ / 2.0, areaX0_ + areaWidth_ / 2.0, areaX0_ + areaWidth_);
    }
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
    lastHuskyOdom_ = now();
}

void SystemControlNode::casualtyCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    if (state_ != State::SEARCHING) {
        return;  // first confirmed casualty wins
    }
    casualtyX_ = msg->point.x;
    casualtyY_ = msg->point.y;
    RCLCPP_INFO(get_logger(), "Casualty at (%.2f, %.2f)", casualtyX_, casualtyY_);
    if (secondDrone_) {
        // The casualty's frame is the odometry frame of the drone that saw it.
        finderDrone_ = msg->header.frame_id.rfind("parrot2", 0) == 0 ? 2 : 1;
        RCLCPP_INFO(get_logger(), "Drone %d found the casualty and stays over it; drone %d maps the "
                    "Husky route", finderDrone_, helperDrone());
    }

    setState(State::PLANNING);
}

void SystemControlNode::huskyStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
    lastHuskyStatus_ = msg->data;
    // DONE is not final: if the Husky leaves its stop position, husky_node
    // reports EN_ROUTE (driving back) or FAILED, and the mission follows.
    if (state_ == State::DONE && msg->data != toString(HuskyStatus::ARRIVED)) {
        RCLCPP_WARN(get_logger(), "Husky reports %s after arrival (now %.2f m from the casualty): "
                    "mission no longer DONE", msg->data.c_str(),
                    std::hypot(huskyX_ - casualtyX_, huskyY_ - casualtyY_));
        setState(State::HUSKY_EN_ROUTE);
        huskyAcked_ = true;
    }
    if (state_ != State::HUSKY_EN_ROUTE) {
        return;
    }
    if (msg->data != toString(HuskyStatus::IDLE)) {
        huskyAcked_ = true;
    }
    if (msg->data == toString(HuskyStatus::ARRIVED)) {
        checkArrival();
    } else if (msg->data == toString(HuskyStatus::FAILED)) {
        // Recovery: replan from where the Husky is now. The terrain map has
        // usually grown since the first plan, and the new route starts at the
        // Husky's real position rather than where it was stuck. A replanned
        // route that got the Husky no closer means it is blocked in the same
        // place (seen in Gazebo with the casualty at (-15, 15)): stop there
        // rather than drive into the same spot again.
        const double distNow = std::hypot(huskyX_ - casualtyX_, huskyY_ - casualtyY_);
        const double progress = routeStartDist_ - distNow;
        if (routeReplans_ > 0 && progress < MIN_REPLAN_PROGRESS_M) {
            RCLCPP_ERROR(get_logger(), "Replanned route got the Husky only %.1f m closer (still %.1f m "
                         "from the casualty): blocked. Mission FAILED, operator decision needed",
                         progress, distNow);
            setState(State::FAILED);
            return;
        }
        if (routeReplans_ < MAX_ROUTE_REPLANS) {
            ++routeReplans_;
            RCLCPP_WARN(get_logger(), "Husky could not finish the route at (%.2f, %.2f): replanning "
                        "from there (attempt %d/%d)", huskyX_, huskyY_, routeReplans_, MAX_ROUTE_REPLANS);
            setState(State::PLANNING);
            return;
        }
        RCLCPP_ERROR(get_logger(), "Husky failed %d routes to the casualty: operator decision needed",
                     routeReplans_ + 1);
        setState(State::FAILED);
    }
}

// Mission control's own check of "ARRIVED", from its own Husky odometry:
// DONE only if the Husky really is within arrival_max_m of the casualty.
void SystemControlNode::checkArrival()
{
    const double d = std::hypot(huskyX_ - casualtyX_, huskyY_ - casualtyY_);
    if (d > arrivalMax_) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                              "Husky reports ARRIVED but is %.2f m from the casualty (limit %.1f m): "
                              "not marking the mission DONE", d, arrivalMax_);
        return;
    }
    RCLCPP_INFO(get_logger(), "Husky arrived and stopped at (%.2f, %.2f), %.2f m from the casualty at "
                "(%.2f, %.2f): mission DONE", huskyX_, huskyY_, d, casualtyX_, casualtyY_);
    setState(State::DONE);
    sendOverwatch();
}

// Mission complete: the drone stops patrolling and holds near the casualty,
// camera on the scene, until the operator gives it a new task.
void SystemControlNode::sendOverwatch()
{
    if (overwatchSent_) {
        return;
    }
    double ux = huskyX_ - casualtyX_;
    double uy = huskyY_ - casualtyY_;
    const double n = std::hypot(ux, uy);
    if (n < 1e-3) {
        ux = 1.0;
        uy = 0.0;
    } else {
        ux /= n;
        uy /= n;
    }
    // On the Husky's side, like the review point. The camera looks forward
    // and down and the drone faces the way it flies, so it comes in along the
    // line towards the casualty and stops facing it: the camera then sees
    // both the casualty and the Husky.
    // Two drones: the one already guarding the casualty does the overwatch.
    sendDronePath({{casualtyX_ + ux * (OVERWATCH_OFFSET_M + 4.0), casualtyY_ + uy * (OVERWATCH_OFFSET_M + 4.0)},
                   {casualtyX_ + ux * OVERWATCH_OFFSET_M, casualtyY_ + uy * OVERWATCH_OFFSET_M}},
                  "overwatch near the casualty (mission DONE)", secondDrone_ ? finderDrone_ : 1);
    overwatchSent_ = true;
}

// Warn the operator when a link goes quiet, instead of waiting silently.
void SystemControlNode::watchdog()
{
    const auto t = now();
    // DONE is checked continuously against odometry, not only when the Husky
    // reports: if it ends up out of range, the mission is not DONE any more.
    if (state_ == State::DONE && huskyOdomReceived_ &&
        std::hypot(huskyX_ - casualtyX_, huskyY_ - casualtyY_) > arrivalMax_) {
        RCLCPP_ERROR(get_logger(), "Husky is now %.2f m from the casualty (limit %.1f m): mission no "
                     "longer DONE", std::hypot(huskyX_ - casualtyX_, huskyY_ - casualtyY_), arrivalMax_);
        setState(State::HUSKY_EN_ROUTE);
    }
    // ...and if husky_node confirms arrival again once back in range.
    if (state_ == State::HUSKY_EN_ROUTE && lastHuskyStatus_ == toString(HuskyStatus::ARRIVED)) {
        checkArrival();
    }
    if (huskyOdomReceived_ && (t - lastHuskyOdom_).seconds() > 5.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "No Husky odometry (%s) for %.0f s: is the Gazebo bridge running?",
                             topics::HUSKY_ODOM, (t - lastHuskyOdom_).seconds());
    }
    if (state_ == State::HUSKY_EN_ROUTE && !huskyAcked_ && (t - routeSentAt_).seconds() > 30.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 15000,
                             "Route sent %.0f s ago but husky_node has not reported EN_ROUTE: is it "
                             "running, and is Nav2 active? (the route is latched, so a restarted "
                             "husky_node picks it up)", (t - routeSentAt_).seconds());
    }
}

void SystemControlNode::tryPlan()
{
    watchdog();
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
        // The map may simply not cover the ground in between yet (e.g. the
        // casualty was spotted before the area search). Send the drone to map it.
        surveyForRoute();
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
    routeSentAt_ = now();
    routeStartDist_ = std::hypot(huskyX_ - casualtyX_, huskyY_ - casualtyY_);
    huskyAcked_ = false;
    setState(State::HUSKY_EN_ROUTE);
}

// Decision: the planner needs terrain the map doesn't have yet, so task the
// drone (when it is free) with a lawnmower over the box spanning the Husky
// and the casualty. Each further attempt widens the box by 4 m per side.
void SystemControlNode::surveyForRoute()
{
    if (state_ != State::PLANNING || !huskyOdomReceived_) {
        return;
    }
    // Two drones: the helper (not the finder, which is guarding the casualty)
    // surveys. It is taken off its search straight away: the casualty is
    // found, and the Husky needs the route. It is not free while it is already
    // surveying, paused (stop button, or waiting on its own detection) or not
    // flying yet.
    const int surveyDrone = secondDrone_ ? helperDrone() : 1;
    if (secondDrone_) {
        const std::string& status = surveyDrone == 2 ? drone2Status_ : droneStatus_;
        if (surveyInProgress_ ||
            (status != "IDLE" && status != "HOLDING" && status != "SEARCHING")) {
            return;
        }
    } else if (droneStatus_ != "IDLE" && droneStatus_ != "HOLDING") {
        return;   // still busy (reviewing the casualty or surveying)
    }
    if (routeSurveys_ >= MAX_ROUTE_SURVEYS) {
        // The drone is back from its last survey. Give the terrain map a
        // moment to take in the last strip, then stop and tell the operator,
        // rather than sitting in PLANNING forever.
        if (surveysExhaustedAt_.nanoseconds() == 0) {
            surveysExhaustedAt_ = now();
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "Surveyed %d times and still no drivable route: the casualty may be "
                             "unreachable by ground (operator decision needed)", routeSurveys_);
        if ((now() - surveysExhaustedAt_).seconds() > 20.0) {
            RCLCPP_ERROR(get_logger(), "No drivable route to the casualty at (%.2f, %.2f) after %d "
                         "terrain surveys: unreachable by ground. Mission FAILED, operator decision "
                         "needed (e.g. send a team on foot)", casualtyX_, casualtyY_, routeSurveys_);
            setState(State::FAILED);
        }
        return;
    }
    const double pad = 3.0 + 4.0 * routeSurveys_;
    const double limitX = get_parameter("terrain_size_x").as_double() / 2.0 - 1.0;
    const double limitY = get_parameter("terrain_size_y").as_double() / 2.0 - 1.0;
    const double x0 = std::max(-limitX, std::min(huskyX_, casualtyX_) - pad);
    const double x1 = std::min(limitX, std::max(huskyX_, casualtyX_) + pad);
    const double y0 = std::max(-limitY, std::min(huskyY_, casualtyY_) - pad);
    const double y1 = std::min(limitY, std::max(huskyY_, casualtyY_) + pad);
    ++routeSurveys_;
    sendDronePath(zigzag(x0, y0, x1 - x0, y1 - y0, laneSpacing_),
                  "survey terrain between Husky and casualty (attempt " +
                      std::to_string(routeSurveys_) + ")", surveyDrone);
    if (secondDrone_) {
        surveyInProgress_ = true;   // cleared when the helper reports HOLDING (survey flown)
    } else {
        droneStatus_ = "TASKED";    // don't re-task before the drone reports back
    }
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

// Same lawnmower as zigzag(), but lanes run along y and step in x.
std::vector<std::pair<double, double>> SystemControlNode::zigzagAlongY(double x0, double y0, double width,
                                                                       double height, double spacing) const
{
    std::vector<std::pair<double, double>> pts;
    for (const auto& p : zigzag(y0, x0, height, width, spacing)) {
        pts.emplace_back(p.second, p.first);
    }
    return pts;
}

void SystemControlNode::droneArea(int drone, double& x0, double& y0, double& width, double& height) const
{
    x0 = areaX0_;
    y0 = areaY0_;
    width = areaWidth_;
    height = areaHeight_;
    if (secondDrone_) {
        width = areaWidth_ / 2.0;          // parrot1: west half
        if (drone == 2) {
            x0 = areaX0_ + width;          // parrot2: east half
        }
    }
}

// Pass 1: east-west lanes. Pass 2: north-south lanes. Pass 3: east-west, offset
// by half a lane. (Same passes as before, now over each drone's own area.)
// With two drones, parrot2 flies its lawnmower backwards: it starts in the far
// (north-east) corner and works south while parrot1 works north from the
// south-west, so they spread out instead of flying side by side.
std::vector<std::pair<double, double>> SystemControlNode::searchPath(int drone, int pass) const
{
    double x0, y0, width, height;
    droneArea(drone, x0, y0, width, height);
    std::vector<std::pair<double, double>> points;
    if (pass <= 1) {
        points = zigzag(x0, y0, width, height, laneSpacing_);
    } else {
        const bool alongY = pass % 2 == 0;
        const double shift = (pass >= 3) ? laneSpacing_ / 2.0 : 0.0;
        points = alongY
            ? zigzagAlongY(x0 + shift, y0, width - shift, height, laneSpacing_)
            : zigzag(x0, y0 + shift, width, height - shift, laneSpacing_);
    }
    if (secondDrone_ && drone == 2) {
        std::reverse(points.begin(), points.end());
    }
    return points;
}

void SystemControlNode::sendDronePath(const std::vector<std::pair<double, double>>& points,
                                      const std::string& reason, int drone)
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
    if (drone == 2) {
        if (!drone2PathPub_) {
            return;   // second drone not enabled
        }
        drone2PathPub_->publish(path);
        RCLCPP_INFO(get_logger(), "Drone 2 tasked: %s (%zu waypoints)", reason.c_str(), points.size());
        return;
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
    auto points = searchPath(1, 1);
    sendDronePath(points, "area search");
    searchStarted_ = true;
    searchPasses_ = 1;
    response->success = true;
    response->message = "Search started: " + std::to_string(points.size()) + " waypoints";
    if (secondDrone_) {
        const auto points2 = searchPath(2, 1);
        sendDronePath(points2, "area search (east half)", 2);
        search2Started_ = true;
        search2Passes_ = 1;
        response->message += " (drone 1), " + std::to_string(points2.size()) + " waypoints (drone 2)";
    }
}

void SystemControlNode::stopDrone(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    sendDronePath({}, "stop and hover");
    sendDronePath({}, "stop and hover", 2);   // does nothing without a second drone
    response->success = true;
    response->message = secondDrone_ ? "Both drones stopping" : "Drone stopping";
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

    // A pass finished with nobody found: search again from another direction.
    // The drone's camera looks forward, so on east-west lanes a person near a
    // corner or edge is only glimpsed at the image edge (seen in Gazebo with
    // the casualty at (-15, 15): never detected on one pass).
    if (droneStatus_ == "HOLDING" && state_ == State::SEARCHING && searchStarted_ &&
        searchPasses_ < MAX_SEARCH_PASSES) {
        ++searchPasses_;
        const bool alongY = searchPasses_ % 2 == 0;
        auto points = searchPath(1, searchPasses_);
        sendDronePath(points, "search pass " + std::to_string(searchPasses_) +
                                  (alongY ? " (north-south lanes)" : " (east-west lanes, offset)"));
        droneStatus_ = "TASKED";
        return;
    }

    // Two drones: drone 1 has finished flying the route survey it was given.
    if (secondDrone_ && surveyInProgress_ && helperDrone() == 1 && droneStatus_ == "HOLDING") {
        surveyInProgress_ = false;
    }

    // A free drone while no route exists: map the ground between Husky and casualty.
    if (droneStatus_ == "IDLE" || droneStatus_ == "HOLDING") {
        surveyForRoute();
    }

    // auto_start: begin the area search as soon as the drone has climbed and
    // is waiting for a mission, so the demo needs no manual service call.
    if (autoStart_ && !searchStarted_ && droneStatus_ == "IDLE" && state_ == State::SEARCHING) {
        sendDronePath(searchPath(1, 1), "area search (auto_start)");
        searchStarted_ = true;
        searchPasses_ = 1;
    }
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

void SystemControlNode::uiParrotEstopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    std_msgs::msg::Bool out;
    out.data = msg->data;
    parrotEstopPub_->publish(out);
    RCLCPP_INFO(get_logger(), "UI: parrot %s", msg->data ? "STOP" : "START");
}

// The dashboard's Stop/Start Parrot 2 button (or from a terminal:
//   ros2 topic pub --once /mission_control/parrot2/estop std_msgs/msg/Bool "{data: true}")
void SystemControlNode::uiParrot2EstopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    std_msgs::msg::Bool out;
    out.data = msg->data;
    parrot2EstopPub_->publish(out);
    RCLCPP_INFO(get_logger(), "UI: parrot2 %s", msg->data ? "STOP" : "START");
}

// Same search logic as droneStatusCallback, for parrot2 over the east half.
// After a confirmed casualty it either guards it (it found it) or surveys the
// Husky route (parrot1 found it).
void SystemControlNode::drone2StatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == drone2Status_) {
        return;
    }
    drone2Status_ = msg->data;
    RCLCPP_INFO(get_logger(), "Drone 2: %s", drone2Status_.c_str());

    // Finished a pass with nobody found: search its half again from another direction.
    if (drone2Status_ == "HOLDING" && state_ == State::SEARCHING && search2Started_ &&
        search2Passes_ < MAX_SEARCH_PASSES) {
        ++search2Passes_;
        const bool alongY = search2Passes_ % 2 == 0;
        sendDronePath(searchPath(2, search2Passes_),
                      "search pass " + std::to_string(search2Passes_) +
                          (alongY ? " (north-south lanes)" : " (east-west lanes, offset)"),
                      2);
        drone2Status_ = "TASKED";
        return;
    }

    // Drone 2 has finished flying the route survey it was given.
    if (surveyInProgress_ && helperDrone() == 2 && drone2Status_ == "HOLDING") {
        surveyInProgress_ = false;
    }
    if (drone2Status_ == "IDLE" || drone2Status_ == "HOLDING") {
        surveyForRoute();
    }

    if (autoStart_ && !search2Started_ && drone2Status_ == "IDLE" && state_ == State::SEARCHING) {
        sendDronePath(searchPath(2, 1), "area search, east half (auto_start)", 2);
        search2Started_ = true;
        search2Passes_ = 1;
    }
}

void SystemControlNode::uiHuskyEstopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    std_msgs::msg::Bool out;
    out.data = msg->data;
    huskyEstopPub_->publish(out);
    RCLCPP_INFO(get_logger(), "UI: husky %s", msg->data ? "STOP" : "START");
}
