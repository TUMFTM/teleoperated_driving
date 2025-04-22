/**
 * @file rc-car_sensing_interface_node.cpp
 * @brief RC-Car sensing interface.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_rc-car_interface
 */

#include <rclcpp/rclcpp.hpp>

#include "tod_rc-car_interface/rc-car_sensing_interface.hpp"

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto sensing_interface_node = std::make_shared<tod_rccar_interface::SensingInterface>();
    sensing_interface_node->run();
    rclcpp::spin(sensing_interface_node);
    rclcpp::shutdown();
    return 0;
}