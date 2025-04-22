// Copyright TUM-FTM
#include <rclcpp/rclcpp.hpp>
#include "tod_command_creation/command_creator.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tod_command_creation::CommandCreator>());
    rclcpp::shutdown();
    return 0;
}
