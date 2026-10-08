#ifndef PARROT_NODE_HPP
#define PARROT_NODE_HPP

#include "parrot.hpp"

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <optional>
#include <string>

/// Runs the drone: wait for odometry, climb, then fly whatever path mission control sends.
///   listens on  mission_path_topic   (nav_msgs/Path)    empty path = stop and hover
///   reports on  mission_status_topic (std_msgs/String)  WAITING_FOR_ODOM / CLIMBING / IDLE / SEARCHING / HOLDING
class ParrotNode : public rclcpp::Node{
public:
ParrotNode();

private:
enum class State { WAITING_FOR_ODOM, CLIMBING, IDLE, SEARCHING, HOLDING };

void tick();
void setState(State state);
void onMissionPath(const nav_msgs::msg::Path::SharedPtr msg);

Parrot parrot_;
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSub_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPub_;

std::optional<State> state_;
bool hasMission_{false};
};

#endif