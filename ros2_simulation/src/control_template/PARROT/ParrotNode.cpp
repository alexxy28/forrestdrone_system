// ParrotNode.cpp — the drone's node. Flies only what mission control tells it to.
//
// Run:  ros2 run control_template parrot_node
// It climbs, then hovers in IDLE until system_control_node sends a path.

#include "ParrotNode.hpp"

#include <chrono>
#include <functional>
#include <utility>
#include <vector>

namespace {

const char* toString(int state)
{
static const char* names[] = {"WAITING_FOR_ODOM", "CLIMBING", "IDLE", "SEARCHING", "HOLDING"};
return names[state];
}

}  // namespace

namespace {

std::optional<Casualty> chooseNearestCasualty(const std::vector<Casualty>& casualties,
                                             double x, double y)
{
    std::optional<Casualty> best;
    double bestDistance = std::numeric_limits<double>::infinity();
    for (const auto& casualty : casualties) {
        if (casualty.status != CasualtyStatus::UNREVIEWED) {
            continue;
        }
        const double distance = std::hypot(casualty.position.point.x - x,
                                           casualty.position.point.y - y);
        if (distance < bestDistance) {
            best = casualty;
            bestDistance = distance;
        }
    }
    return best;
}

}  // namespace

ParrotNode::ParrotNode()
    : Node("parrot_node"),
parrot_(this,
declare_parameter<double>("max_speed", 1.5),
declare_parameter<double>("goal_tolerance", 1.0),
declare_parameter<double>("target_altitude", 20.0)),
thermalCamera_(this)
{
    // Latched both ways, so it doesn't matter which node starts first.
const auto latched = rclcpp::QoS(1).reliable().transient_local();

const std::string pathTopic   = declare_parameter<std::string>("mission_path_topic", "/parrot1/mission/path");
const std::string statusTopic = declare_parameter<std::string>("mission_status_topic", "/parrot1/mission/status");

pathSub_ = create_subscription<nav_msgs::msg::Path>(
    pathTopic, latched,
    std::bind(&ParrotNode::onMissionPath, this, std::placeholders::_1));
statusPub_ = create_publisher<std_msgs::msg::String>(statusTopic, latched);
casualtyPub_ = create_publisher<geometry_msgs::msg::PointStamped>(topics::CASUALTY, latched);

timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&ParrotNode::tick, this));

RCLCPP_INFO(get_logger(), "parrot_node started, waiting for missions on %s", pathTopic.c_str());
}

std::vector<Casualty> ParrotNode::getCasualties() const
{
    return thermalCamera_.getCasualties();
}

bool ParrotNode::confirmCasualty(std::size_t index)
{
    const auto casualties = thermalCamera_.getCasualties();
    if (index == 0 || index > casualties.size() ||
        casualties[index - 1].status != CasualtyStatus::UNREVIEWED) {
        return false;
    }
    if (!thermalCamera_.confirmCasualty(index)) {
        return false;
    }
    casualtyPub_->publish(casualties[index - 1].position);
    RCLCPP_INFO(get_logger(), "Confirmed casualty %zu sent to mission control for Husky routing", index);
    return true;
}

bool ParrotNode::denyCasualty(std::size_t index)
{
    return thermalCamera_.denyCasualty(index);
}

void ParrotNode::onMissionPath(const nav_msgs::msg::Path::SharedPtr msg)
{
std::vector<std::pair<double, double>> points;
for (const auto& pose : msg->poses) {
    points.emplace_back(pose.pose.position.x, pose.pose.position.y);
}

if (points.empty()) {
    RCLCPP_INFO(get_logger(), "Mission control: stop, hovering");
    hasMission_ = false;
    patrolMissionActive_ = false;
    patrolWaypoints_.clear();
    parrot_.setWaypoints(points);
    return;
}
patrolWaypoints_ = points;
patrolMissionActive_ = true;
parrot_.setWaypoints(points);
hasMission_ = true;
}

std::optional<Casualty> ParrotNode::selectCasualtyForVerification() const
{
    return chooseNearestCasualty(thermalCamera_.getCasualties(), parrot_.x(), parrot_.y());
}

void ParrotNode::tick()
{
    // Worked out fresh each cycle: a drone that drifts off altitude goes back to climbing.
if (!parrot_.ready()) {
setState(State::WAITING_FOR_ODOM);
return;
    }
if (!parrot_.atAltitude()) {
setState(State::CLIMBING);
parrot_.climb();
return;
    }

if (const auto casualty = selectCasualtyForVerification()) {
    const std::pair<double, double> target{casualty->position.point.x, casualty->position.point.y};
    if (!verifyTarget_ || std::hypot(target.first - verifyTarget_->first,
                                      target.second - verifyTarget_->second) > 0.25) {
        if (!verifyTarget_ && patrolMissionActive_) {
            patrolWaypoints_ = parrot_.remainingWaypoints();
        }
        verifyTarget_ = target;
        // Keep the mission-control route so it can be restored after review.
        RCLCPP_WARN(get_logger(),
                    "Patrol paused: casualty detected at (%.2f, %.2f); confirming by flying there",
                    target.first, target.second);
        parrot_.setWaypoints({target});
        hasMission_ = true;
    }
    setState(State::SEARCHING);
    parrot_.flyToWaypoint();
    return;
}

if (verifyTarget_) {
    verifyTarget_.reset();
    if (patrolMissionActive_) {
        RCLCPP_INFO(get_logger(), "Casualty reviewed; resuming patrol (%zu waypoints)",
                    patrolWaypoints_.size());
        parrot_.setWaypoints(patrolWaypoints_);
        hasMission_ = true;
    }
}

if (!hasMission_) {
setState(State::IDLE);
parrot_.stop();
return;
    }
if (parrot_.searchComplete()) {
setState(State::HOLDING);
parrot_.stop();
return;
    }
setState(State::SEARCHING);
parrot_.flyToWaypoint();
}

void ParrotNode::setState(State state)
{
if (state_ == state) {
return;
    }
state_ = state;
std_msgs::msg::String msg;
msg.data = toString(static_cast<int>(state));
statusPub_->publish(msg);
RCLCPP_INFO(get_logger(), "State: %s", msg.data.c_str());
}
