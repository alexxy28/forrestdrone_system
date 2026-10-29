#ifndef HUSKY_NODE_HPP
#define HUSKY_NODE_HPP

#include "husky.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <optional>
#include <vector>

/// Receives a planned route, converts it for Nav2, and reports progress.
///
/// The route arrives in world coordinates (the terrain map is built from raw
/// Gazebo odometry). Nav2 works in nav_frame (default husky1_odom, the EKF
/// odometry frame). The two differ by however far odometry has drifted from
/// the world, so the offset is measured every time a waypoint is sent: the
/// Husky's pose in nav_frame (TF) is compared with its pose in the world
/// (raw /husky1/odometry).
///
/// Why not husky1_map: SLAM's map frame jumped against the world while the
/// Husky drove over the slopes (6.8 m over one 20 m route, and 1.6 m within a
/// single 3 m leg, Gazebo 27 Oct, v4 tests), so Nav2 "reached" goals metres from where
/// they were planned. The EKF odometry frame moves smoothly.
///
/// Final approach: Nav2 reports "reached" anywhere within its goal tolerance
/// of the last waypoint, and SLAM drifts while driving, so the Husky could
/// stop 3-4 m from the person (seen in Gazebo, 27 Oct). When the route ends,
/// the Husky's true distance to the casualty (world frame) is checked. If it
/// is further than approach_max_m, a short approach goal approach_standoff_m
/// from the casualty is sent, with a freshly measured world -> map offset.
/// ARRIVED is only reported once that check passes.
///
/// Stopped and staying stopped: after Nav2 finishes, ARRIVED waits until the
/// Husky has been still (< still_speed_mps) for still_time_s, and its stop
/// position is published on /mission/husky_hold. While ARRIVED, if it moves
/// more than hold_drift_m from there (or out of the stand-off), the status
/// goes back to EN_ROUTE and it drives back (hold_reapproaches times), else
/// FAILED. Seen in Gazebo 27-28 Oct: the free-rolling wheels let it roll
/// backwards down a slope after ARRIVED (now fixed with wheel friction in
/// urdf_husky/wheel.urdf.xacro; this check makes sure it is noticed anyway).
class HuskyNode : public rclcpp::Node{
    public:
    HuskyNode();

    private:
    void routeCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onStop(const std_msgs::msg::Bool::SharedPtr msg);
    void casualtyCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void tick();
    void routeEnded(HuskyStatus result);
    bool startApproach(double d, double hx, double hy);
    void startSettling();
    void settle();
    void watchHold();
    bool updateOffset();
    bool worldToMap(const Waypoint& world, Waypoint& map);
    double distanceToCasualty() const;

    // ROUTE: driving the planned route. APPROACH: closing the last metres.
    // SETTLING: Nav2 is done, waiting until the Husky is really stationary.
    // HOLDING: ARRIVED, watching that it stays put. FINISHED: FAILED reported.
    enum class Phase { IDLE, ROUTE, APPROACH, SETTLING, HOLDING, FINISHED };

    Husky husky_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr routeSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stopSub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr casualtySub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr holdPub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::optional<nav_msgs::msg::Path> pendingRoute_;
    std::optional<HuskyStatus> lastPublished_;
    nav_msgs::msg::Odometry::SharedPtr worldOdom_;
    std::optional<Waypoint> casualty_;   // world frame

    std::string navFrame_;
    // Latest world -> nav_frame transform (rotation dyaw, translation tx, ty).
    bool haveOffset_{false};
    double offC_{1.0}, offS_{0.0}, offTx_{0.0}, offTy_{0.0};
    double loggedTx_{1e9}, loggedTy_{1e9};

    Phase phase_{Phase::IDLE};
    HuskyStatus finalStatus_{HuskyStatus::IDLE};
    int approachAttempts_{0};
    double approachMax_;        // accept the stop if this close to the casualty (m)
    double approachStandoff_;   // approach goal distance from the casualty (m)
    double approachRange_;      // only approach if the route ended this close (m)
    int maxApproachAttempts_;

    // Stationary check and hold watch (world frame, sim time).
    double speed_{1e9};                 // ground speed, m/s (unknown until measured)
    std::optional<Waypoint> speedRef_;
    rclcpp::Time speedRefTime_{0, 0, RCL_ROS_TIME};
    rclcpp::Time settleStart_{0, 0, RCL_ROS_TIME};
    std::optional<rclcpp::Time> stillSince_;
    std::optional<Waypoint> holdPoint_;
    int holdReapproaches_{0};
    double stillSpeed_;      // below this the Husky counts as stopped (m/s)
    double stillTime_;       // ... for this long (s)
    double settleTimeout_;   // give up waiting for it to stop (s)
    double holdDrift_;       // allowed movement after ARRIVED (m)
    int maxHoldReapproaches_;
};

#endif
