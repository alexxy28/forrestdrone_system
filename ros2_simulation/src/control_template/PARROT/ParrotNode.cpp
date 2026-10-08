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

ParrotNode::ParrotNode()
    : Node("parrot_node"),
parrot_(this,
declare_parameter<double>("max_speed", 1.5),
declare_parameter<double>("goal_tolerance", 1.0),
declare_parameter<double>("target_altitude", 20.0))
{
    // Latched both ways, so it doesn't matter which node starts first.
const auto latched = rclcpp::QoS(1).reliable().transient_local();

const std::string pathTopic   = declare_parameter<std::string>("mission_path_topic", "/parrot1/mission/path");
const std::string statusTopic = declare_parameter<std::string>("mission_status_topic", "/parrot1/mission/status");

pathSub_ = create_subscription<nav_msgs::msg::Path>(
    pathTopic, latched,
    std::bind(&ParrotNode::onMissionPath, this, std::placeholders::_1));
statusPub_ = create_publisher<std_msgs::msg::String>(statusTopic, latched);

timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&ParrotNode::tick, this));

RCLCPP_INFO(get_logger(), "parrot_node started, waiting for missions on %s", pathTopic.c_str());
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
    parrot_.setWaypoints(points);
    return;
}
parrot_.setWaypoints(points);
hasMission_ = true;
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

int main(int argc, char* argv[])
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<ParrotNode>());
rclcpp::shutdown();
return 0;
}