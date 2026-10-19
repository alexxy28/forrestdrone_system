#include "PARROT/ParrotNode.hpp"

#include <iostream>
#include <memory>
#include <poll.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParrotNode>();
    rclcpp::WallRate rate(10.0);
    std::cout << "Commands: quit\n";

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        pollfd input{STDIN_FILENO, POLLIN, 0};
        if (poll(&input, 1, 0) > 0) {
            std::string command;
            std::getline(std::cin, command);
            if (command == "quit") {
                break;
            }
            std::cout << "Unknown command. Type quit.\n";
        }
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
