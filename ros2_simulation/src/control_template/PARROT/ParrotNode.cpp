#include "ParrotNode.hpp"

ParrotNode::ParrotNode() : Node("parrot_node"){
    RCLCPP_INFO(get_logger(), "parrot_node started");
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParrotNode>());
    rclcpp::shutdown();
    return 0;
}
