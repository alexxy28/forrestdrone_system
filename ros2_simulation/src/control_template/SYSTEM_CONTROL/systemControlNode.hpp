
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
    void setState(State state);

    // ---- drone tasking ----
    void startSearch(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void stopDrone(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void droneStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    void sendDronePath(const std::vector<std::pair<double, double>>& points, const std::string& reason);
    std::vector<std::pair<double, double>> zigzag(double x0, double y0, double width,
                                                  double height, double spacing) const;

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

    State state_{State::SEARCHING};
    int routeStep_;

    bool terrainReceived_{false};
    bool huskyOdomReceived_{false};
    std::string terrainFrame_;
    double huskyX_{0.0}, huskyY_{0.0};
    double casualtyX_{0.0}, casualtyY_{0.0};

    // ---- drone tasking ----
    double areaX0_{-17.0}, areaY0_{-11.0}, areaWidth_{28.0}, areaHeight_{22.0};
    double laneSpacing_{4.0};
    double clickBoxSize_{8.0};
    std::string missionFrame_{"world"};
    std::string droneStatus_{"UNKNOWN"};
};

#endif