

#include "PARROT/ParrotNode.hpp"

#include <iostream>
#include <poll.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <sstream>
#include <string>

namespace {

const char* statusName(CasualtyStatus status)
{
    switch (status) {
        case CasualtyStatus::UNREVIEWED: return "UNREVIEWED";
        case CasualtyStatus::CONFIRMED: return "CONFIRMED";
        case CasualtyStatus::DENIED: return "DENIED";
    }
    return "UNKNOWN";
}

void printCasualties(const std::vector<Casualty>& casualties)
{
    for (const auto& casualty : casualties) {
        std::cout << casualty.index << ": cell ("
                  << casualty.gridX << ", " << casualty.gridY << ") at ("
                  << casualty.position.point.x << ", "
                  << casualty.position.point.y << ", "
                  << casualty.position.point.z << ") "
                  << statusName(casualty.status) << '\n';
    }
}

void processCommand(const std::string& input, const std::shared_ptr<ParrotNode>& node)
{
    std::istringstream command(input);
    std::string action;
    std::size_t index = 0;
    command >> action >> index;

    if (action == "list") {
        printCasualties(node->getCasualties());
    } else if (action == "confirm" && node->confirmCasualty(index)) {
        std::cout << "Casualty " << index << " confirmed\n";
    } else if (action == "deny" && node->denyCasualty(index)) {
        std::cout << "Casualty " << index << " denied\n";
    } else if (action == "help") {
        std::cout << "Commands: list, confirm <index>, deny <index>, quit\n";
    } else {
        std::cout << "Unknown command or casualty index. Type help.\n";
    }
}

}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParrotNode>();
    rclcpp::WallRate rate(10.0);
    std::size_t reportedCasualties = 0;
    std::cout << "Commands: list, confirm <index>, deny <index>, quit\n";

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        const auto casualties = node->getCasualties();
        if (casualties.size() > reportedCasualties) {
            for (std::size_t index = reportedCasualties; index < casualties.size(); ++index) {
                const auto& casualty = casualties[index];
                RCLCPP_INFO(node->get_logger(),
                            "Known casualty %zu: cell (%d, %d), (%.2f, %.2f, %.2f)",
                            casualty.index, casualty.gridX, casualty.gridY,
                            casualty.position.point.x, casualty.position.point.y,
                            casualty.position.point.z);
            }
            reportedCasualties = casualties.size();
        }

        pollfd input{STDIN_FILENO, POLLIN, 0};
        if (poll(&input, 1, 0) > 0) {
            std::string command;
            std::getline(std::cin, command);
            if (command == "quit") {
                break;
            }
            processCommand(command, node);
        }
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}