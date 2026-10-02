#ifndef PARROT_NODE_HPP
#define PARROT_NODE_HPP

#include "parrot.hpp"

#include "rclcpp/rclcpp.hpp"

#include <optional>

/// Runs the drone's search: wait for odometry, climb, fly the pattern, hold.
class ParrotNode : public rclcpp::Node{
    public:
    ParrotNode();

    private:
    enum class State { WAITING_FOR_ODOM, CLIMBING, SEARCHING, HOLDING };

    void tick();
    void setState(State state);

    Parrot parrot_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::optional<State> state_;
};

#endif
