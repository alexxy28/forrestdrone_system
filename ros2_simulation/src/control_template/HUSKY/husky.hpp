#ifndef HUSKY_HPP
#define HUSKY_HPP

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "dataStructures.hpp"

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

/// The ground robot: drives a route through Nav2, one waypoint at a time.
///
/// An intermediate waypoint that Nav2 cannot reach is skipped (the next one is
/// tried instead); only a failure on the final waypoint, or more than
/// MAX_CONSECUTIVE_SKIPS in a row, ends the route as FAILED.
class Husky{
public:
    explicit Husky(rclcpp::Node* node);

    /// True once Nav2's NavigateToPose action server is up.
    bool ready() const;

    /// Converts a world point into Nav2's frame using the latest offset.
    using FrameConverter = std::function<bool(const Waypoint& world, Waypoint& map)>;

    /// Waypoints are given in the world frame and converted one at a time,
    /// just before each is sent to Nav2, so the drift of Nav2's frame against
    /// the world while driving never accumulates over the route.
    void setFrameConverter(FrameConverter converter, const std::string& frame);

    /// Drive the waypoints (world frame) in order.
    /// Ignored while a route is already being driven. `start` (optional, world)
    /// is where the Husky is now; it sets the heading of a one-waypoint route.
    void followRoute(const std::vector<Waypoint>& route, const Waypoint* start = nullptr);

    /// Stop button: pause() cancels the Nav2 goal and holds. The route and the
    /// current waypoint are kept, so resume() carries on from the same point.
    void pause();
    void resume();
    bool paused() const;

    HuskyStatus status() const;

    /// Call regularly (husky_node does, every 0.2 s). Sends the next waypoint
    /// after a short pause, and resends a goal Nav2 never started.
    void update();

protected:
private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    void sendNextWaypoint();
    void cancel(GoalHandle::SharedPtr handle);
    void goalResponseCallback(GoalHandle::SharedPtr handle, int seq);
    void resultCallback(const GoalHandle::WrappedResult& result, int seq);
    void waypointFailed(const char* why);
    void handleResult(rclcpp_action::ResultCode code);

    rclcpp::Node* node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr navClient_;

    FrameConverter toMap_;
    std::string goalFrame_{"husky1_odom"};
    std::vector<Waypoint> route_;   // world frame
    Waypoint start_{0.0, 0.0};
    bool haveStart_{false};
    size_t index_{0};
    HuskyStatus status_{HuskyStatus::IDLE};

    GoalHandle::SharedPtr goalHandle_;  // goal Nav2 is driving right now
    bool paused_{false};
    int goalSeq_{0};                    // bumped on every send and every STOP; old replies get ignored
    int consecutiveSkips_{0};

    static constexpr int MAX_CONSECUTIVE_SKIPS = 2;

    // Lost-goal watchdog (wall clock). Seen in Gazebo, 28 Oct: a waypoint sent
    // the instant Nav2 finished the previous one was never started and never
    // answered, and the Husky waited forever.
    rclcpp::Clock steady_{RCL_STEADY_TIME};
    rclcpp::Time goalSentAt_{0, 0, RCL_STEADY_TIME};
    bool goalStarted_{false};           // Nav2 is executing the current goal
    int resends_{0};
    std::optional<rclcpp::Time> terminalSeenAt_;   // Nav2 says done, result not received yet
    std::optional<rclcpp::Time> sendNextAt_;       // next waypoint goes out at this time
    static constexpr double GOAL_START_TIMEOUT_S = 10.0;
    static constexpr int MAX_RESENDS = 3;
    static constexpr double NEXT_GOAL_DELAY_S = 0.5;
};

#endif
