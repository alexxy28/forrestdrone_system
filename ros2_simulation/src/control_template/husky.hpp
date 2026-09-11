#ifndef HUSKY_HPP
#define HUSKY_HPP

#include "robot.hpp"
#include "sensor.hpp"

class Husky : public Robot{
    public:
    explicit Husky(std::shared_ptr<rclcpp::Node> node);
    Sensor& getSensor();
    protected:
    private:
    std::unique_ptr<Sensor> sensor_;
};

#endif