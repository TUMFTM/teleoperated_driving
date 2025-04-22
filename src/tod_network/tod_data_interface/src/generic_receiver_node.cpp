/**
 * @file generic_receiver_node.cpp
 * @author Nils Gehrke
 * @brief Main entry point for the GenericTODReceiver node that initializes and starts the receiver.
 * @version 1.0
 * 
 * @copyright TUMFTM 2024
 * 
 */

#include "rclcpp/rclcpp.hpp"
#include "tod_data_interface/generic_tod_receiver.hpp"

using namespace tod_data_interface;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GenericTODReceiver>();
    node->receive();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
