#include "robot.hpp"

Robot::Robot(RobotType type) : type_(type) {
    switch (type) {
        case RobotType::HUSKY:
            // Initialize Husky-specific components
            break;
        case RobotType::PARROT:
            // Initialize Parrot-specific components
            break;
        default:
            break;
    }
}