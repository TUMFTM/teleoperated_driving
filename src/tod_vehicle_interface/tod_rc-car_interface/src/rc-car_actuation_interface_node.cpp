/**
 * @file rc-car_actuation_interface_node.cpp
 * @brief RC-Car actuation interface.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_rc-car_interface
 */

#include <rclcpp/rclcpp.hpp>

#include "tod_rc-car_interface/rc-car_actuation_interface.hpp"

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto actuation_interface_node = std::make_shared<tod_rccar_interface::ActuationInterface>();
    actuation_interface_node->run();
    rclcpp::spin(actuation_interface_node);
    rclcpp::shutdown();
    return 0;
}