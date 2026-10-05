#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

struct Waypoint { double x, y; };

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
}

class DroneSearch : public rclcpp::Node
{
    public:
        DroneSearch() : Node("drone_search")
        {
            odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/parrot1/odom");
            cmd_topic_  = this->declare_parameter<std::string>("cmd_topic", "/parrot1/cmd_vel");
            max_speed_  = this->declare_parameter<double>("max_speed", 5.0);
            tolerance_  = this->declare_parameter<double>("goal_tolerance", 1.0);
            target_alt_ = this->declare_parameter<double>("target_altitude", 5.0);

            pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
            sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic_, 10,
                std::bind(&DroneSearch::onOdom, this, std::placeholders::_1));

            // terrain_demo mesh spans about 20.6 m x 21.0 m; cover it with
            // a small edge margin and 2 m lane spacing for denser LiDAR scans.
            waypoints_ = lawnmower(-11.0, -11.0, 22.0, 2.0, 22.0);

            timer_ = this->create_wall_timer(100ms, std::bind(&DroneSearch::tick, this));

            RCLCPP_INFO(this->get_logger(),
                        "Search pattern: %zu waypoints, listening on %s",
                        waypoints_.size(), odom_topic_.c_str());
        }

    private:
        std::vector<Waypoint> lawnmower(double x0, double y0, double width,
                                        double spacing, double height)
        {
            std::vector<Waypoint> pts;
            int rows = static_cast<int>(height / spacing) + 1;
            for (int i = 0; i < rows; i++) {
                double y = y0 + i * spacing;
                if (i % 2 == 0) {
                    pts.push_back({x0, y});
                    pts.push_back({x0 + width, y});
                } else {
                    pts.push_back({x0 + width, y});
                    pts.push_back({x0, y});
                }
            }
            return pts;
        }

        void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            x_   = msg->pose.pose.position.x;
            y_   = msg->pose.pose.position.y;
            z_   = msg->pose.pose.position.z;
            yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
            have_pose_ = true;
        }

        void tick()
        {
            if (!have_pose_) {
                return;  // no odometry yet
            }

            geometry_msgs::msg::Twist cmd;

            // Hold altitude before doing anything lateral.
            double alt_error = target_alt_ - z_;
            cmd.linear.z = std::clamp(0.8 * alt_error, -1.0, 1.0);

            if (std::fabs(alt_error) > 0.5) {
                pub_->publish(cmd);  // climb first
                return;
            }

            if (index_ >= waypoints_.size()) {
                pub_->publish(geometry_msgs::msg::Twist());  // all stop
                return;
            }

            double dx = waypoints_[index_].x - x_;
            double dy = waypoints_[index_].y - y_;
            double distance = std::hypot(dx, dy);

            if (distance < tolerance_) {
                ++index_;
                RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu/%zu",
                            index_, waypoints_.size());
                return;
            }

            // Turn toward the goal, then drive at a speed that falls off as
            // heading error grows, so it doesn't arc wide on sharp turns.
            double heading_error = std::atan2(dy, dx) - yaw_;
            heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));

            cmd.angular.z = std::clamp(1.5 * heading_error, -1.0, 1.0);
            if (std::fabs(heading_error) < 0.5) {
                cmd.linear.x = std::min(max_speed_, 0.5 * distance);
            }
            pub_->publish(cmd);
        }

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::string odom_topic_, cmd_topic_;
        double max_speed_, tolerance_, target_alt_;

        std::vector<Waypoint> waypoints_;
        size_t index_{0};

        bool have_pose_{false};
        double x_{0.0}, y_{0.0}, z_{0.0}, yaw_{0.0};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneSearch>());
    rclcpp::shutdown();
    return 0;
}