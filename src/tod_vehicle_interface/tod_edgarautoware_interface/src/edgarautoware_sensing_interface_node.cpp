/**
 * @file edgarautoware_sensing_interface_node.cpp
 * @brief Sensing interface for the research vehicle EDGAR using Autoware as AV stack.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_edgarautoware_interface
 */

#include <rclcpp/rclcpp.hpp>

#include "tod_edgarautoware_interface/edgarautoware_sensing_interface.hpp"

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto sensing_interface_node = std::make_shared<tod_edgarautoware_interface::SensingInterface>();
    sensing_interface_node->run();
    rclcpp::spin(sensing_interface_node);
    rclcpp::shutdown();
    return 0;
}
