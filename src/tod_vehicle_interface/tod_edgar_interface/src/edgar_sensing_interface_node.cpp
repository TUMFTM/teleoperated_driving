/**
 * @file edgar_sensing_interface_node.cpp
 * @brief Sensing interface for the research vehicle EDGAR without AV capabilities.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_edgar_interface
 */

#include <rclcpp/rclcpp.hpp>

#include "tod_edgar_interface/edgar_sensing_interface.hpp"

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto sensing_interface_node = std::make_shared<tod_edgar_interface::SensingInterface>();
    sensing_interface_node->run();
    rclcpp::spin(sensing_interface_node);
    rclcpp::shutdown();
    return 0;
}