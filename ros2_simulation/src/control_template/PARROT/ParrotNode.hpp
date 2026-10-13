#ifndef PARROT_NODE_HPP
#define PARROT_NODE_HPP

#include "ThermalCamera.hpp"
#include "parrot.hpp"
#include "dataStructures.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp" 

#include <optional>
#include <string>
#include <vector>

/// Runs the drone: wait for odometry, climb, then fly whatever path mission control sends.
///   listens on  mission_path_topic   (nav_msgs/Path)    empty path = stop and hover
///   reports on  mission_status_topic (std_msgs/String)  WAITING_FOR_ODOM / CLIMBING / IDLE / SEARCHING / HOLDING
class ParrotNode : public rclcpp::Node{
public:
ParrotNode();

std::vector<Casualty> getCasualties() const;
bool confirmCasualty(std::size_t index);
bool denyCasualty(std::size_t index);

private:
enum class State { WAITING_FOR_ODOM, CLIMBING, IDLE, SEARCHING, HOLDING, PAUSED };

std::optional<Casualty> selectCasualtyForVerification() const;
void tick();
void setState(State state);
void onMissionPath(const nav_msgs::msg::Path::SharedPtr msg);
void onStop(const std_msgs::msg::Bool::SharedPtr msg);

Parrot parrot_;
ThermalCamera thermalCamera_;
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSub_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPub_;
rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr casualtyPub_;

rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stopSub_;
bool stopped_{false};   // true = stop button pressed, drone holds in place

std::optional<State> state_;
bool hasMission_{false};
std::optional<std::pair<double, double>> verifyTarget_;
std::vector<std::pair<double, double>> patrolWaypoints_;
bool patrolMissionActive_{false};
};

#endif
