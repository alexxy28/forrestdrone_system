#ifndef LIDAR_HPP
#define LIDAR_HPP

#include "sensor.hpp"
#include "dataStructures.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>
#include <vector>
#include <string>

class Lidar : public Sensor{
    public:
    Lidar(std::shared_ptr<rclcpp::Node> node, const std::string& topic);

    bool hasSensorInformation() override;
    bool hasData() override;

    const lidarInformation& getSensorInformation() const;
    const std::vector<double>& getData() override;


    protected:

    private:

    void retrieveNewData(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    lidarInformation information_{};
    std::vector<double> ranges_;

    bool hasData_;
    bool hasInformation_;

};

#endif
