/**
 * @file generic_sender_node.cpp
 * @author Nils Gehrke
 * @brief Main entry point for the GenericTODSender node that initializes and starts the sender.
 * @version 1.0
 * 
 * @copyright TUMFTM 2024
 * 
 */
#include "rclcpp/rclcpp.hpp"
#include "tod_data_interface/generic_tod_sender.hpp"

using namespace tod_data_interface;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GenericTODSender>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
