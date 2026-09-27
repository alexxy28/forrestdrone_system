#include "terrain_map.hpp"
#include "terrain_planner.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class TraceHuskyNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate =
        rclcpp_action::ClientGoalHandle<NavigateToPose>;

    TraceHuskyNode()
        : Node("trace_husky_node"),
          terrain_map_(0.5, 20.0, 20.0),
          terrain_planner_(terrain_map_),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        declare_parameter<double>("target_x", 2.0);
        declare_parameter<double>("target_y", 0.0);
        declare_parameter<int>("planner_mode", 1);
        declare_parameter<bool>("use_terrain", false);

        target_x_ = get_parameter("target_x").as_double();
        target_y_ = get_parameter("target_y").as_double();
        planner_mode_ = get_parameter("planner_mode").as_int();
        use_terrain_ = get_parameter("use_terrain").as_bool();

        nav_client_ =
            rclcpp_action::create_client<NavigateToPose>(
                this,
                "/husky1/navigate_to_pose");

        terrain_sub_ =
            create_subscription<sensor_msgs::msg::PointCloud2>(
                "/parrot1/terrain_points",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &TraceHuskyNode::terrainCallback,
                    this,
                    std::placeholders::_1));

        husky_odom_sub_ =
            create_subscription<nav_msgs::msg::Odometry>(
                "/husky1/odometry",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &TraceHuskyNode::huskyOdomCallback,
                    this,
                    std::placeholders::_1));

        path_pub_ =
            create_publisher<nav_msgs::msg::Path>(
                "/trace/safe_path", 10);

        timer_ =
            create_wall_timer(
                std::chrono::seconds(1),
                std::bind(
                    &TraceHuskyNode::startMission,
                    this));

        RCLCPP_INFO(get_logger(),
                    "TRACE Husky node started.");

        RCLCPP_INFO(
            get_logger(),
            "Casualty target: (%.2f, %.2f)",
            target_x_, target_y_);

        RCLCPP_INFO(
            get_logger(),
            "Planner mode: %d",
            planner_mode_);

        RCLCPP_INFO(
            get_logger(),
            "Terrain planning: %s",
            use_terrain_ ? "ENABLED" : "DISABLED");
    }

private:
    void terrainCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        terrain_frame_ = msg->header.frame_id;
        terrain_map_.updateFromPointCloud(*msg);
        terrain_received_ = true;

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Terrain received in frame %s.",
            terrain_frame_.c_str());
    }

    void huskyOdomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        husky_x_ = msg->pose.pose.position.x;
        husky_y_ = msg->pose.pose.position.y;
        husky_odom_frame_ = msg->header.frame_id;
        husky_odom_received_ = true;
    }

    bool transformOdomPointToMap(
        double odom_x,
        double odom_y,
        double &map_x,
        double &map_y)
    {
        try
        {
            const auto transform =
                tf_buffer_.lookupTransform(
                    "husky1_map",
                    "husky1_odom",
                    tf2::TimePointZero);

            const double tx =
                transform.transform.translation.x;

            const double ty =
                transform.transform.translation.y;

            const double qz =
                transform.transform.rotation.z;

            const double qw =
                transform.transform.rotation.w;

            const double yaw =
                2.0 * std::atan2(qz, qw);

            const double c = std::cos(yaw);
            const double s = std::sin(yaw);

            map_x =
                c * odom_x -
                s * odom_y +
                tx;

            map_y =
                s * odom_x +
                c * odom_y +
                ty;

            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(
                get_logger(),
                "Waiting for husky1_odom -> husky1_map TF: %s",
                ex.what());

            return false;
        }
    }

    std::vector<std::pair<double, double>>
    transformRouteToHuskyMap(
        const std::vector<std::pair<double, double>>
            &input)
    {
        std::vector<std::pair<double, double>>
            output;

        output.reserve(input.size());

        for (const auto &point : input)
        {
            double x;
            double y;

            if (!transformOdomPointToMap(
                    point.first,
                    point.second,
                    x,
                    y))
            {
                return {};
            }

            output.emplace_back(x, y);
        }

        return output;
    }

    void startMission()
    {
        if (mission_started_)
            return;

        if (!nav_client_->wait_for_action_server(
                std::chrono::milliseconds(200)))
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "Waiting for Husky Nav2...");
            return;
        }

        // =====================================================
        // DEMO 1
        // =====================================================
        if (planner_mode_ == 1)
        {
            mission_started_ = true;

            RCLCPP_INFO(
                get_logger(),
                "DEMO 1: Direct navigation to casualty.");

            sendGoal(target_x_, target_y_, 0.0);
            return;
        }

        // =====================================================
        // DEMO 2
        // =====================================================
        if (planner_mode_ == 2)
        {
            mission_started_ = true;

            const double start_x = 0.0;
            const double start_y = 0.0;

            RCLCPP_INFO(
                get_logger(),
                "DEMO 2: A* planning from "
                "(%.2f, %.2f) to casualty (%.2f, %.2f)",
                start_x,
                start_y,
                target_x_,
                target_y_);

            auto full_route =
                terrain_planner_.planDemo2(
                    start_x,
                    start_y,
                    target_x_,
                    target_y_);

            if (full_route.empty())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "A* could not find a route.");

                mission_started_ = false;
                return;
            }

            RCLCPP_INFO(
                get_logger(),
                "A* generated %zu grid points.",
                full_route.size());

            path_frame_ = "husky1_map";

            publishPath(
                full_route,
                path_frame_);

            route_ =
                simplifyRoute(
                    full_route,
                    2);

            RCLCPP_INFO(
                get_logger(),
                "Executing simplified route with "
                "%zu navigation waypoints.",
                route_.size() > 0
                    ? route_.size() - 1
                    : 0);

            waypoint_index_ = 1;
            sendNextWaypoint();
            return;
        }

        // =====================================================
        // DEMO 3
        // PARROT TERRAIN -> SLOPE -> A* -> HUSKY
        // =====================================================
        if (planner_mode_ == 3)
        {
            if (!terrain_received_ ||
                !terrain_map_.hasTerrain())
            {
                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    3000,
                    "DEMO 3: Waiting for Parrot terrain map...");
                return;
            }

            if (!husky_odom_received_)
            {
                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    3000,
                    "DEMO 3: Waiting for Husky odometry...");
                return;
            }

            if (terrain_frame_ != "parrot1_odom")
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    3000,
                    "DEMO 3: Terrain frame is %s; "
                    "expected parrot1_odom.",
                    terrain_frame_.c_str());
                return;
            }

            /*
             * In this simulation the Gazebo odometry topics
             * contain world-referenced robot positions.
             *
             * The terrain mapper has already transformed the
             * Parrot cloud into parrot1_odom, and Husky
             * odometry supplies the live planning start.
             */
            const double start_x = husky_x_;
            const double start_y = husky_y_;

            RCLCPP_INFO(
                get_logger(),
                "DEMO 3: Terrain-aware A*.");

            RCLCPP_INFO(
                get_logger(),
                "Terrain frame: %s",
                terrain_frame_.c_str());

            RCLCPP_INFO(
                get_logger(),
                "Live Husky start: (%.2f, %.2f)",
                start_x,
                start_y);

            RCLCPP_INFO(
                get_logger(),
                "Casualty target: (%.2f, %.2f)",
                target_x_,
                target_y_);

            terrain_map_.printStatistics();

            auto terrain_route =
                terrain_planner_.plan(
                    start_x,
                    start_y,
                    target_x_,
                    target_y_);

            if (terrain_route.empty())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Terrain-aware A* could not find "
                    "a traversable route.");

                /*
                 * Keep mission false so a later terrain cloud
                 * can cause another planning attempt.
                 */
                return;
            }

            RCLCPP_INFO(
                get_logger(),
                "Terrain-aware A* generated "
                "%zu grid points.",
                terrain_route.size());

            /*
             * Convert coordinates into the frame Nav2 accepts.
             */
            auto map_route =
                transformRouteToHuskyMap(
                    terrain_route);

            if (map_route.empty())
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Could not transform terrain route "
                    "into husky1_map yet.");
                return;
            }

            mission_started_ = true;

            path_frame_ = "husky1_map";

            publishPath(
                map_route,
                path_frame_);

            route_ =
                simplifyRoute(
                    map_route,
                    2);

            if (route_.size() < 2)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Terrain route contains no executable "
                    "navigation waypoint.");
                mission_started_ = false;
                return;
            }

            RCLCPP_INFO(
                get_logger(),
                "Terrain-safe route ready: "
                "%zu navigation waypoints.",
                route_.size() - 1);

            waypoint_index_ = 1;

            RCLCPP_INFO(
                get_logger(),
                "DEMO 3: Dispatching Husky on "
                "terrain-aware route.");

            sendNextWaypoint();
            return;
        }

        RCLCPP_ERROR(
            get_logger(),
            "Unknown planner mode: %d",
            planner_mode_);
    }

    std::vector<std::pair<double, double>>
    simplifyRoute(
        const std::vector<
            std::pair<double, double>> &input,
        std::size_t step)
    {
        if (input.size() <= 2 ||
            step <= 1)
        {
            return input;
        }

        std::vector<
            std::pair<double, double>>
            output;

        output.push_back(input.front());

        for (std::size_t i = step;
             i + 1 < input.size();
             i += step)
        {
            output.push_back(input[i]);
        }

        if (output.back() != input.back())
        {
            output.push_back(input.back());
        }

        return output;
    }

    void sendNextWaypoint()
    {
        if (waypoint_index_ >= route_.size())
        {
            RCLCPP_INFO(
                get_logger(),
                "TRACE SAFE ROUTE COMPLETE.");

            if (planner_mode_ == 2)
            {
                RCLCPP_INFO(
                    get_logger(),
                    "DEMO 2 COMPLETE: "
                    "A* route executed by Husky.");
            }
            else if (planner_mode_ == 3)
            {
                RCLCPP_INFO(
                    get_logger(),
                    "DEMO 3 COMPLETE: "
                    "Terrain-aware safe route "
                    "executed by Husky.");
            }

            return;
        }

        const auto &waypoint =
            route_[waypoint_index_];

        double yaw = 0.0;

        if (waypoint_index_ + 1 <
            route_.size())
        {
            const auto &next =
                route_[waypoint_index_ + 1];

            yaw =
                std::atan2(
                    next.second -
                        waypoint.second,
                    next.first -
                        waypoint.first);
        }
        else if (waypoint_index_ > 0)
        {
            const auto &previous =
                route_[waypoint_index_ - 1];

            yaw =
                std::atan2(
                    waypoint.second -
                        previous.second,
                    waypoint.first -
                        previous.first);
        }

        constexpr double RAD_TO_DEG =
            180.0 / 3.14159265358979323846;

        RCLCPP_INFO(
            get_logger(),
            "Executing waypoint %zu/%zu: "
            "(%.2f, %.2f), yaw %.1f deg",
            waypoint_index_,
            route_.size() - 1,
            waypoint.first,
            waypoint.second,
            yaw * RAD_TO_DEG);

        sendGoal(
            waypoint.first,
            waypoint.second,
            yaw);
    }

    void sendGoal(
        double x,
        double y,
        double yaw = 0.0)
    {
        NavigateToPose::Goal goal;

        goal.pose.header.frame_id =
            "husky1_map";

        goal.pose.header.stamp =
            now();

        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.position.z = 0.0;

        goal.pose.pose.orientation.x = 0.0;
        goal.pose.pose.orientation.y = 0.0;

        goal.pose.pose.orientation.z =
            std::sin(yaw / 2.0);

        goal.pose.pose.orientation.w =
            std::cos(yaw / 2.0);

        RCLCPP_INFO(
            get_logger(),
            "Sending Husky goal: "
            "(%.2f, %.2f), yaw %.1f deg",
            x,
            y,
            yaw *
                180.0 /
                3.14159265358979323846);

        typename rclcpp_action::Client<
            NavigateToPose>::SendGoalOptions
            options;

        options.goal_response_callback =
            std::bind(
                &TraceHuskyNode::
                    goalResponseCallback,
                this,
                std::placeholders::_1);

        options.feedback_callback =
            std::bind(
                &TraceHuskyNode::
                    feedbackCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2);

        options.result_callback =
            std::bind(
                &TraceHuskyNode::
                    resultCallback,
                this,
                std::placeholders::_1);

        nav_client_->async_send_goal(
            goal,
            options);
    }

    void goalResponseCallback(
        GoalHandleNavigate::SharedPtr handle)
    {
        if (!handle)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Nav2 rejected goal.");
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Goal accepted. Husky moving.");
    }

    void feedbackCallback(
        GoalHandleNavigate::SharedPtr,
        const std::shared_ptr<
            const NavigateToPose::Feedback>
            feedback)
    {
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            3000,
            "Distance remaining: %.2f m",
            feedback->distance_remaining);
    }

    void resultCallback(
        const GoalHandleNavigate::
            WrappedResult &result)
    {
        if (result.code ==
            rclcpp_action::
                ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(
                get_logger(),
                "Navigation goal reached.");

            if (planner_mode_ == 2 ||
                planner_mode_ == 3)
            {
                ++waypoint_index_;
                sendNextWaypoint();
                return;
            }

            if (planner_mode_ == 1)
            {
                RCLCPP_INFO(
                    get_logger(),
                    "DEMO 1 COMPLETE: "
                    "Husky reached casualty.");
            }

            return;
        }

        if (result.code ==
            rclcpp_action::
                ResultCode::ABORTED)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Navigation goal aborted.");
            return;
        }

        if (result.code ==
            rclcpp_action::
                ResultCode::CANCELED)
        {
            RCLCPP_WARN(
                get_logger(),
                "Navigation goal canceled.");
            return;
        }

        RCLCPP_ERROR(
            get_logger(),
            "Navigation failed.");
    }

    void publishPath(
        const std::vector<
            std::pair<double, double>>
            &route,
        const std::string &frame)
    {
        nav_msgs::msg::Path path;

        path.header.frame_id = frame;
        path.header.stamp = now();

        for (std::size_t i = 0;
             i < route.size();
             ++i)
        {
            geometry_msgs::msg::PoseStamped pose;

            pose.header = path.header;

            pose.pose.position.x =
                route[i].first;

            pose.pose.position.y =
                route[i].second;

            pose.pose.position.z = 0.0;

            double yaw = 0.0;

            if (i + 1 < route.size())
            {
                yaw =
                    std::atan2(
                        route[i + 1].second -
                            route[i].second,
                        route[i + 1].first -
                            route[i].first);
            }

            pose.pose.orientation.z =
                std::sin(yaw / 2.0);

            pose.pose.orientation.w =
                std::cos(yaw / 2.0);

            path.poses.push_back(pose);
        }

        path_pub_->publish(path);

        RCLCPP_INFO(
            get_logger(),
            "Published TRACE path with "
            "%zu poses in frame %s.",
            path.poses.size(),
            frame.c_str());
    }

    TerrainMap terrain_map_;
    TerrainPlanner terrain_planner_;

    rclcpp_action::Client<
        NavigateToPose>::SharedPtr
        nav_client_;

    rclcpp::Subscription<
        sensor_msgs::msg::PointCloud2>::
        SharedPtr terrain_sub_;

    rclcpp::Subscription<
        nav_msgs::msg::Odometry>::
        SharedPtr husky_odom_sub_;

    rclcpp::Publisher<
        nav_msgs::msg::Path>::
        SharedPtr path_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    double target_x_ = 0.0;
    double target_y_ = 0.0;

    double husky_x_ = 0.0;
    double husky_y_ = 0.0;

    int planner_mode_ = 1;

    bool use_terrain_ = false;
    bool mission_started_ = false;
    bool terrain_received_ = false;
    bool husky_odom_received_ = false;

    std::string terrain_frame_;
    std::string husky_odom_frame_;
    std::string path_frame_;

    std::vector<
        std::pair<double, double>>
        route_;

    std::size_t waypoint_index_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<TraceHuskyNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
