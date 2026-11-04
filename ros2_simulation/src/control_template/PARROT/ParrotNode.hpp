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
#include <cmath>
#include <string>
#include <vector>

/// Runs the drone: wait for odometry, climb, then fly whatever path mission control sends.
///   listens on  mission_path_topic   (nav_msgs/Path)    empty path = stop and hover
///   reports on  mission_status_topic (std_msgs/String)  WAITING_FOR_ODOM / CLIMBING / IDLE / SEARCHING / HOLDING
class ParrotNode : public rclcpp::Node{
public:
ParrotNode();

std::vector<Casualty> getCasualties() const;
bool hasDetectedCasualty() const;
bool confirmCasualty(std::size_t index);
bool denyCasualty(std::size_t index);

private:
enum class State { WAITING_FOR_ODOM, CLIMBING, IDLE, SEARCHING, HOLDING, PAUSED };

std::optional<Casualty> selectCasualtyForVerification() const;
void tick();
void setState(State state);
void onMissionPath(const nav_msgs::msg::Path::SharedPtr msg);
void onStop(const std_msgs::msg::Bool::SharedPtr msg);
void onCasualtyConfirmed(const std_msgs::msg::Bool::SharedPtr msg);
void startCasualtyReview(const Casualty& casualty);
void trimPatrolWaypointsAlreadyPassed();
void onCandidate(const geometry_msgs::msg::PointStamped::SharedPtr msg);
void publishCandidate(const Casualty& casualty);
void dismissResightings();
void guardCasualty(double casualtyX, double casualtyY);

Parrot parrot_;
ThermalCamera thermalCamera_;
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSub_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPub_;
rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr casualtyPub_;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr casualtyDiscoveredPub_;
rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr casualtyCandidatePub_;  // where the detection is, for the operator

rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stopSub_;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr casualtyConfirmedSub_;
bool stopped_{false};   // true = stop button pressed, drone holds in place

std::optional<State> state_;
bool hasMission_{false};
std::optional<std::pair<double, double>> verifyTarget_;
std::optional<std::size_t> verifyCasualtyIndex_;
std::optional<std::size_t> pendingCasualtyIndex_;
std::optional<std::size_t> lastReviewedCasualtyIndex_;
bool awaitingCasualtyConfirmation_{false};
bool casualtyConfirmationReceived_{false};
bool casualtyConfirmed_{false};
bool casualtyOrbitActive_{false};
std::vector<std::pair<double, double>> patrolWaypoints_;
bool patrolMissionActive_{false};

// ---- more than one drone (multi_drone:=true) ----
// All drones share one operator review (the dashboard shows one detection at
// a time and its Confirm/Reject answers on one topic). So only one drone asks
// at a time, and a drone only acts on an answer to its own detection.
bool multiDrone_{false};
bool otherReviewPending_{false};     // another drone's detection is on the dashboard, unanswered
bool lastCandidateWasMine_{false};   // the detection last shown to the operator is ours
std::optional<std::pair<double, double>> myLastCandidate_;
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr candidateSub_;

// guard_casualty: after circling a casualty it found, the drone stays over it
// instead of going back to its search (the other drone carries on searching).
bool guardCasualty_{false};
// same_casualty_radius_m: a new detection this close to one the operator
// rejected, or to a casualty any drone confirmed, is the same thing seen again
// from another angle: don't stop and ask about it. 0 = off.
double sameCasualtyRadiusM_{0.0};
std::vector<std::pair<double, double>> confirmedByAnyDrone_;
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr confirmedCasualtySub_;
};

#endif
