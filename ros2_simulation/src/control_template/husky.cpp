#include "husky.hpp"

Husky::Husky(std::shared_ptr<rclcpp::Node> node) : Robot(RobotType::HUSKY){
    sensor_ = Sensor::create(
        node,
        SensorType::LIDAR,
        "/husky1/scan");
}

Sensor& Husky::getSensor(){
    return *sensor_;
}