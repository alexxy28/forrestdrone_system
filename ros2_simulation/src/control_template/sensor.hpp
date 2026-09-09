#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <memory>
#include "dataStructures.hpp"
#include "rclcpp/rclcpp.hpp"

class Sensor{
    public:
    Sensor(std::shared_ptr<rclcpp::Node> node, SensorType sensorType);

    virtual ~Sensor() = default;

    virtual bool hasSensorInformation() = 0;
    virtual bool hasData() = 0;
    virtual const lidarInformation& getSensorInformation() const = 0;
    virtual const std::vector<double>& getData() = 0;

    static std::unique_ptr<Sensor> create(
        std::shared_ptr<rclcpp::Node> node,
        SensorType sensorType,
        const std::string& topic);

    protected:
    std::shared_ptr<rclcpp::Node> node_;
    std::string topicName_;
    SensorType type_;
    private:

};

#endif