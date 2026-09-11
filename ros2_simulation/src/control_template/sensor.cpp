#include "sensor.hpp"
#include "lidar.hpp"

#include <stdexcept>

Sensor::Sensor(std::shared_ptr<rclcpp::Node> node, SensorType sensorType) : node_(node), type_(sensorType){ 

}

std::unique_ptr<Sensor> Sensor::create(
	std::shared_ptr<rclcpp::Node> node,
	SensorType sensorType,
	const std::string& topic)
{
	switch (sensorType)
	{
		case SensorType::LIDAR:
			return std::make_unique<Lidar>(node, topic);
		default:
			throw std::invalid_argument("Unsupported sensor type");
	}
}
