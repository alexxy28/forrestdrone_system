#include "ParrotNode.hpp"

#include <chrono>
#include <functional>

namespace {

const char* toString(int state)
{
    static const char* names[] = {"WAITING_FOR_ODOM", "CLIMBING", "SEARCHING", "HOLDING"};
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
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&ParrotNode::tick, this));

    RCLCPP_INFO(get_logger(), "parrot_node started");
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
    RCLCPP_INFO(get_logger(), "State: %s", toString(static_cast<int>(state)));
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParrotNode>());
    rclcpp::shutdown();
    return 0;
}
