#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"

class Sensor{
    public:
    Sensor(rclcpp::Node* node, const std::string& topic) : node_(node), topicName_(topic){}

    virtual ~Sensor() = default;

    virtual bool hasData() = 0;

    protected:
    rclcpp::Node* node_;
    std::string topicName_;
};

#endif
