/**
 * @file manager_node.cpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM
 **/

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "manager.hpp"
/**
 * @file manager_node.cpp
 * @brief Entry point for the Manager application.
 *
 * This file contains the main function, which initializes the ROS2 node, creates an instance
 * of the OperatorManager class, and starts the application's execution loop.
 */

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto manager = std::make_unique<tod_gl::Manager>(argc, argv, "Manager");

    RCLCPP_INFO(rclcpp::get_logger("manager"), "Manager initialized.");
    manager->initialize();
    manager->run();

    RCLCPP_INFO(rclcpp::get_logger("manager"), "Manager exiting.");
    return 0;
}