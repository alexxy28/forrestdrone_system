
// source ~/rs1_ws/install/setup.bash
//ros2 service call /mission/start std_srvs/srv/Trigger
//what algorithm is used, what is the input and output, 
// look at the techincal in canvas and understand it 
// user needs to gain control of the system 
// go into depth on the technical rules and pick one that is relevent and explain it 
// desiciion making, rover compentition 

#ifndef SYSTEM_CONTROL_NODE_HPP
#define SYSTEM_CONTROL_NODE_HPP

#include "terrain_map.hpp"
#include "terrain_planner.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

/// Owns the mission: tells the drone where to search, then plans the Husky's
/// route to the casualty over the terrain map.
///
/// Drone control:
///   ros2 service call /mission/start std_srvs/srv/Trigger   -> zig-zag over the search area
///   ros2 service call /mission/stop  std_srvs/srv/Trigger   -> drone stops and hovers
///   RViz "Publish Point" click (parrot1 window)              -> search a small box around that spot
class SystemControlNode : public rclcpp::Node{
    public:
    SystemControlNode();

    private:
    enum class State { SEARCHING, PLANNING, HUSKY_EN_ROUTE, DONE, FAILED };

    void terrainCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void huskyOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void casualtyCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void huskyStatusCallback(const std_msgs::msg::String::SharedPtr msg);

    void tryPlan();
    void watchdog();
    void checkArrival();
    void sendOverwatch();
    void failMission();
    std::vector<std::pair<double, double>> zigzagAlongY(double x0, double y0, double width,
                                                        double height, double spacing) const;
    void surveyForRoute();
    void setState(State state);

    // ---- drone tasking ----
    void startSearch(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void stopDrone(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void droneStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    /// drone 1 = parrot1, drone 2 = parrot2 (second_drone:=true)
    void sendDronePath(const std::vector<std::pair<double, double>>& points, const std::string& reason,
                       int drone = 1, bool log = true);
    std::vector<std::pair<double, double>> zigzag(double x0, double y0, double width,
                                                  double height, double spacing) const;
    /// The part of the search area a drone covers: all of it with one drone or
    /// in a race, the west half (parrot1) or east half (parrot2) in a split.
    void droneArea(int drone, double& x0, double& y0, double& width, double& height) const;
    /// Lawnmower for search pass `pass` (1, 2, 3) over that drone's area.
    std::vector<std::pair<double, double>> searchPath(int drone, int pass) const;
    /// 1 east-west, 2 north-south, 3 east-west offset, 4 north-south offset.
    int searchPattern(int drone, int pass) const;
    std::string searchPassName(int drone, int pass) const;

    // ---- second drone (parrot2) ----
    void drone2StatusCallback(const std_msgs::msg::String::SharedPtr msg);
    void uiParrot2EstopCallback(const std_msgs::msg::Bool::SharedPtr msg);
    /// HUSKY_EN_ROUTE with two drones: the helper flies behind the Husky.
    void followHusky();

    TerrainMap terrainMap_;
    TerrainPlanner terrainPlanner_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrainSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr huskyOdomSub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr casualtySub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr huskyStatusSub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr routePub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statePub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ---- drone tasking ----
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr dronePathPub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr droneStatusSub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clickedSub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startSrv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopSrv_;

    // UI stop button -> drone
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr uiParrotEstopSub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr parrotEstopPub_;
    void uiParrotEstopCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // UI stop button -> Husky
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr uiHuskyEstopSub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr huskyEstopPub_;
    void uiHuskyEstopCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // ---- second drone (parrot2) ----
    bool secondDrone_{false};
    // drone_search: "race" = both drones search the whole area on different
    // paths (first to find the casualty wins), "split" = each searches half.
    bool raceSearch_{true};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr drone2PathPub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drone2StatusSub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr parrot2EstopPub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr uiParrot2EstopSub_;
    std::string drone2Status_{"UNKNOWN"};
    bool search2Started_{false};
    int search2Passes_{0};
    // Who does what once a casualty is confirmed: the drone that found it stays
    // over it (and does the overwatch), the other one maps the Husky's route.
    int finderDrone_{1};
    bool surveyInProgress_{false};
    int helperDrone() const { return finderDrone_ == 1 ? 2 : 1; }
    // Last point sent to the helper while it follows the Husky.
    bool escortSent_{false};
    double escortX_{0.0};
    double escortY_{0.0};
    static constexpr double ESCORT_BEHIND_M = 4.0;     // behind the Husky, camera on it
    static constexpr double ESCORT_UPDATE_M = 1.5;     // re-task once the Husky has moved this far

    State state_{State::SEARCHING};
    int routeStep_;

    bool terrainReceived_{false};
    bool huskyOdomReceived_{false};
    std::string terrainFrame_;
    double huskyX_{0.0}, huskyY_{0.0};
    double casualtyX_{0.0}, casualtyY_{0.0};

    // ---- drone tasking ----
    double areaX0_{-17.0}, areaY0_{-12.0}, areaWidth_{30.0}, areaHeight_{28.0};
    double laneSpacing_{4.0};
    double clickBoxSize_{8.0};
    std::string missionFrame_{"world"};
    std::string droneStatus_{"UNKNOWN"};
    bool autoStart_{false};
    bool searchStarted_{false};
    int searchPasses_{0};
    static constexpr int MAX_SEARCH_PASSES = 3;
    int routeSurveys_{0};
    static constexpr int MAX_ROUTE_SURVEYS = 3;
    int routeReplans_{0};
    // Communication watchdog (warnings only).
    rclcpp::Time routeSentAt_{0, 0, RCL_ROS_TIME};
    rclcpp::Time lastHuskyOdom_{0, 0, RCL_ROS_TIME};
    bool huskyAcked_{false};
    rclcpp::Time surveysExhaustedAt_{0, 0, RCL_ROS_TIME};
    static constexpr int MAX_ROUTE_REPLANS = 2;
    double routeStartDist_{0.0};   // Husky -> casualty when the current route was sent (m)
    double arrivalMax_{3.0};       // DONE only within this of the casualty (m)
    std::string lastHuskyStatus_;
    bool overwatchSent_{false};
    static constexpr double OVERWATCH_OFFSET_M = 5.0;
    static constexpr double MIN_REPLAN_PROGRESS_M = 1.0;
};

#endif