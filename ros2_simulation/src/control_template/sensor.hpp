#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <memory>
#include "dataStructures.hpp"
#include "rclcpp/rclcpp.hpp"

class Sensor{
    public:
    Sensor(std::shared_ptr<rclcpp::Node> node);

    virtual ~Sensor() = default;

    virtual bool hasData() = 0;

    protected:
    std::shared_ptr<rclcpp::Node> node_;
    private:
};

#endif