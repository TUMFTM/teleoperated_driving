/**
 * @file edgar_actuation_interface_node.cpp
 * @brief Actuation interface for the research vehicle EDGAR without AV capabilities.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_edgar_interface
 */

#include <rclcpp/rclcpp.hpp>

#include "tod_edgar_interface/edgar_actuation_interface.hpp"

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto actuation_interface_node = std::make_shared<tod_edgar_interface::ActuationInterface>();
    actuation_interface_node->run();
    rclcpp::spin(actuation_interface_node);
    rclcpp::shutdown();
    return 0;
}