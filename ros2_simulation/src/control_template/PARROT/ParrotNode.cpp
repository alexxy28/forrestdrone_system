// ParrotNode.cpp — the drone's node. Flies only what mission control tells it to.
//
// Run:  ros2 run control_template parrot_node
// It climbs, then hovers in IDLE until system_control_node sends a path.
// With start_paused:=true it first waits on the ground until the stop button is released.

#include "ParrotNode.hpp"

#include <chrono>
#include <functional>
#include <utility>
#include <vector>

namespace {

constexpr double casualtyStandOffDistanceM = 5.0;

const char* toString(int state) 
{
static const char* names[] = {"WAITING_FOR_ODOM", "CLIMBING", "IDLE", "SEARCHING", "HOLDING", "PAUSED"};
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
declare_parameter<double>("target_altitude", 7.0)),   // m above ground
thermalCamera_(this, [this](double bearing) {
    return parrot_.aimAtRelativeBearing(bearing);
})
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
casualtyDiscoveredPub_ = create_publisher<std_msgs::msg::Bool>(topics::CASUALTY_DISCOVERED, latched);
casualtyCandidatePub_ = create_publisher<geometry_msgs::msg::PointStamped>(topics::CASUALTY_CANDIDATE, latched);
casualtyConfirmedSub_ = create_subscription<std_msgs::msg::Bool>(
    topics::CASUALTY_CONFIRMED, latched,
    std::bind(&ParrotNode::onCasualtyConfirmed, this, std::placeholders::_1));

// Stop button: true = freeze in place, false = carry on.
// start_paused: wait on the ground in PAUSED until the operator presses Start.
stopped_ = declare_parameter<bool>("start_paused", false);
const std::string stopTopic = declare_parameter<std::string>("stop_topic", "/parrot1/estop");
stopSub_ = create_subscription<std_msgs::msg::Bool>(
    stopTopic, 10,
    std::bind(&ParrotNode::onStop, this, std::placeholders::_1));

// More than one drone: listen to the shared detection topic, so this drone
// knows when another drone is waiting for the operator to confirm or reject.
multiDrone_ = declare_parameter<bool>("multi_drone", false);
if (multiDrone_) {
    candidateSub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        topics::CASUALTY_CANDIDATE, latched,
        std::bind(&ParrotNode::onCandidate, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "multi_drone: sharing the operator review with the other drone(s)");
}

timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&ParrotNode::tick, this));

RCLCPP_INFO(get_logger(), "parrot_node started, waiting for missions on %s", pathTopic.c_str());
if (stopped_) {
    RCLCPP_WARN(get_logger(), "Starting PAUSED: press Start Parrot on the dashboard to take off");
}
}

std::vector<Casualty> ParrotNode::getCasualties() const
{
    return thermalCamera_.getCasualties();
}

bool ParrotNode::hasDetectedCasualty() const
{
    return thermalCamera_.hasDetectedCasualty();
}

bool ParrotNode::confirmCasualty(std::size_t index)
{
    const auto casualties = thermalCamera_.getCasualties();
    // A DENIED casualty may still be confirmed later (operator changed their mind).
    if (index == 0 || index > casualties.size() ||
        casualties[index - 1].status == CasualtyStatus::CONFIRMED) {
        return false;
    }
    if (!thermalCamera_.confirmCasualty(index)) {
        return false;
    }
    casualtyPub_->publish(casualties[index - 1].position);
    RCLCPP_INFO(get_logger(), "Confirmed casualty %zu sent to mission control for Husky routing", index);
    if (pendingCasualtyIndex_ && *pendingCasualtyIndex_ == index &&
        awaitingCasualtyConfirmation_) {
        casualtyConfirmationReceived_ = true;
        casualtyConfirmed_ = true;
    }
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

void ParrotNode::onStop(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data == stopped_) {
        return;   // no change, don't spam the log
    }
    stopped_ = msg->data;
    if (stopped_) {
        RCLCPP_WARN(get_logger(), "STOP pressed: holding position");
    } else {
        RCLCPP_INFO(get_logger(), "STOP released: resuming");
    }
}

void ParrotNode::onCasualtyConfirmed(const std_msgs::msg::Bool::SharedPtr msg)
/*
 * Handles the reception of a casualty confirmation message.
 * If the system is not currently awaiting confirmation, it checks if the message
 * corresponds to the last reviewed casualty and processes it accordingly.
 * Otherwise, it updates the internal state with the received confirmation.
 */
{
    if (multiDrone_) {
        // The answer is for whichever detection the dashboard was showing.
        // If that was another drone's, this one carries on as before. (If two
        // drones asked at the same instant, both are awaiting and both take
        // the answer: in practice they have seen the same person.)
        const bool forAnotherDrone = otherReviewPending_ || !lastCandidateWasMine_;
        otherReviewPending_ = false;
        if (forAnotherDrone && !awaitingCasualtyConfirmation_) {
            RCLCPP_INFO(get_logger(), "Operator answered another drone's detection: ignoring it here");
            return;
        }
    }

    if (!awaitingCasualtyConfirmation_) {
        if (!msg->data || !lastReviewedCasualtyIndex_) {
            return; // No confirmation or no last reviewed casualty index, nothing to do
        }

        const auto casualties = thermalCamera_.getCasualties(); // Get the list of casualties from the thermal camera
        const auto casualty = std::find_if(casualties.begin(), casualties.end(), [&](const auto& item) {
            return item.index == *lastReviewedCasualtyIndex_; // Match the casualty with the last reviewed index
        });
        if (casualty == casualties.end()) {
            return;
        }

        if (confirmCasualty(casualty->index)) { // If the casualty is confirmed, start the review process
            startCasualtyReview(*casualty);
            setState(State::SEARCHING); // Update the system state to SEARCHING after accepting a late casualty confirmation
            RCLCPP_WARN(get_logger(),
                        "Late casualty confirmation accepted for %zu: restarting review/orbit workflow", // Log the acceptance of a late casualty confirmation
                        casualty->index);
        }
        return;
    }
    casualtyConfirmationReceived_ = true;
    casualtyConfirmed_ = msg->data;
    RCLCPP_INFO(get_logger(), "Casualty confirmation received: %s",
                msg->data ? "true" : "false");
}

void ParrotNode::startCasualtyReview(const Casualty& casualty)
{
    const double casualtyX = casualty.position.point.x;
    const double casualtyY = casualty.position.point.y;
    const double fromCasualtyX = parrot_.x() - casualtyX;
    const double fromCasualtyY = parrot_.y() - casualtyY;
    const double distanceFromCasualty = std::hypot(fromCasualtyX, fromCasualtyY);
    const double directionX = distanceFromCasualty > 1e-6 ? fromCasualtyX / distanceFromCasualty : 1.0;
    const double directionY = distanceFromCasualty > 1e-6 ? fromCasualtyY / distanceFromCasualty : 0.0;
    const std::pair<double, double> target{
        casualtyX + directionX * casualtyStandOffDistanceM,
        casualtyY + directionY * casualtyStandOffDistanceM};

    if (!verifyTarget_ && patrolMissionActive_) {
        patrolWaypoints_ = parrot_.remainingWaypoints();
    }
    verifyTarget_ = target;
    verifyCasualtyIndex_ = casualty.index;
    RCLCPP_WARN(get_logger(),
                "Patrol paused: casualty detected at (%.2f, %.2f); flying to review point "
                "%.2f m away at (%.2f, %.2f)",
                casualtyX, casualtyY, casualtyStandOffDistanceM,
                target.first, target.second);
    parrot_.setWaypoints({target});
    hasMission_ = true;
}

void ParrotNode::trimPatrolWaypointsAlreadyPassed()
{
    while (patrolWaypoints_.size() > 1) {
        const auto& first = patrolWaypoints_[0];
        const auto& next = patrolWaypoints_[1];
        const double firstDistance = std::hypot(first.first - parrot_.x(),
                                                first.second - parrot_.y());
        const double nextDistance = std::hypot(next.first - parrot_.x(),
                                               next.second - parrot_.y());
        if (firstDistance <= 1.0 || nextDistance < firstDistance) {
            patrolWaypoints_.erase(patrolWaypoints_.begin());
            continue;
        }
        break;
    }
}

void ParrotNode::tick()
{
    // Worked out fresh each cycle. CLIMBING only happens once: after that the
    // drone holds its height above ground while it flies.
if (!parrot_.ready()) {
setState(State::WAITING_FOR_ODOM);
return;
    }

    // Stop button beats everything. Nothing gets reset, so on release the checks
    // below pick up where the drone was: climb, casualty check or search.
    if (stopped_) {
        setState(State::PAUSED);
        parrot_.hold();
        return;
    }

if (!parrot_.atAltitude()) {
setState(State::CLIMBING);
parrot_.climb();
return;
    }

if (awaitingCasualtyConfirmation_) {
if (!casualtyConfirmationReceived_) {
    setState(State::PAUSED);
    parrot_.hold();
    return;
}

awaitingCasualtyConfirmation_ = false;
casualtyConfirmationReceived_ = false;
if (!casualtyConfirmed_) {
    if (pendingCasualtyIndex_) {
        denyCasualty(*pendingCasualtyIndex_);
        lastReviewedCasualtyIndex_ = pendingCasualtyIndex_;
    }
    pendingCasualtyIndex_.reset();
    casualtyConfirmed_ = false;
} else {
    const auto casualties = thermalCamera_.getCasualties();
    const auto casualty = pendingCasualtyIndex_
        ? std::find_if(casualties.begin(), casualties.end(), [&](const auto& item) {
              return item.index == *pendingCasualtyIndex_;
          })
        : casualties.end();
    if (casualty != casualties.end()) {
        confirmCasualty(casualty->index);
        lastReviewedCasualtyIndex_ = casualty->index;
        startCasualtyReview(*casualty);
        setState(State::SEARCHING);
        parrot_.flyToWaypoint();
        return;
    }
    pendingCasualtyIndex_.reset();
    casualtyConfirmed_ = false;
}
}

// multi_drone: while another drone's detection is on the dashboard, keep
// searching and ask about our own detections once the operator has answered.
if (!verifyTarget_ && !(multiDrone_ && otherReviewPending_)) {
if (const auto casualty = selectCasualtyForVerification()) {
pendingCasualtyIndex_ = casualty->index;
awaitingCasualtyConfirmation_ = true;
casualtyConfirmationReceived_ = false;
casualtyConfirmed_ = false;
publishCandidate(*casualty);   // shown on the dashboard before review
std_msgs::msg::Bool discovered;
discovered.data = true;
casualtyDiscoveredPub_->publish(discovered);
RCLCPP_WARN(get_logger(),
            "Casualty %zu discovered; published true on /CasulityDiscovered and "
            "waiting indefinitely for /CasulityConfirmed",
            casualty->index);
setState(State::PAUSED);
parrot_.hold();
return;
}
}

if (verifyTarget_) {
    const auto casualties = thermalCamera_.getCasualties();
    const auto casualty = verifyCasualtyIndex_
        ? std::find_if(casualties.begin(), casualties.end(), [&](const auto& item) {
              return item.index == *verifyCasualtyIndex_;
          })
        : casualties.end();

    if (casualty != casualties.end() &&
        casualty->status == CasualtyStatus::CONFIRMED &&
        !casualtyOrbitActive_) {
        constexpr int orbitPointCount = 8;
        constexpr double orbitRadiusM = casualtyStandOffDistanceM;
        const double pi = std::acos(-1.0);
        std::vector<std::pair<double, double>> orbitWaypoints;
        orbitWaypoints.reserve(orbitPointCount + 1);
        for (int point = 0; point <= orbitPointCount; ++point) {
            const double angle = 2.0 * pi * point / orbitPointCount;
            orbitWaypoints.emplace_back(
                casualty->position.point.x + orbitRadiusM * std::cos(angle),
                casualty->position.point.y + orbitRadiusM * std::sin(angle));
        }
        casualtyOrbitActive_ = true;
        RCLCPP_INFO(get_logger(),
                    "Casualty %zu confirmed; circling at %.2f m before resuming patrol",
                    *verifyCasualtyIndex_, orbitRadiusM);
        parrot_.setWaypoints(orbitWaypoints);
        hasMission_ = true;
        setState(State::SEARCHING);
        parrot_.flyToWaypoint();
        return;
    }

    if (casualtyOrbitActive_) {
        if (!parrot_.searchComplete()) {
            setState(State::SEARCHING);
            parrot_.flyToWaypoint();
            return;
        }
        casualtyOrbitActive_ = false;
    }

    verifyTarget_.reset();
    verifyCasualtyIndex_.reset();
    if (patrolMissionActive_) {
        trimPatrolWaypointsAlreadyPassed();
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

void ParrotNode::publishCandidate(const Casualty& casualty)
{
    myLastCandidate_ = std::make_pair(casualty.position.point.x, casualty.position.point.y);
    lastCandidateWasMine_ = true;
    casualtyCandidatePub_->publish(casualty.position);
}

// multi_drone only: a detection was put up for review on the shared topic.
void ParrotNode::onCandidate(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    // Our own detection comes back to us as well: same position, nothing to do.
    if (myLastCandidate_ &&
        std::hypot(msg->point.x - myLastCandidate_->first,
                   msg->point.y - myLastCandidate_->second) < 1e-3) {
        return;
    }
    lastCandidateWasMine_ = false;
    otherReviewPending_ = true;
    RCLCPP_INFO(get_logger(),
                "Another drone is waiting for review at (%.2f, %.2f): holding back my own "
                "detections until the operator answers", msg->point.x, msg->point.y);
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