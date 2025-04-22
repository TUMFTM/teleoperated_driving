/**
 * @file edgarautoware_automation_interface_node.cpp
 * @brief Automation interface for the research vehicle EDGAR using Autoware as AV stack.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_edgarautoware_interface
 */

#include <rclcpp/rclcpp.hpp>

#include "tod_edgarautoware_interface/edgarautoware_automation_interface.hpp"

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto automation_interface_node = std::make_shared<tod_edgarautoware_interface::AutomationInterface>();
    automation_interface_node->run();
    rclcpp::spin(automation_interface_node);
    rclcpp::shutdown();
    return 0;
}