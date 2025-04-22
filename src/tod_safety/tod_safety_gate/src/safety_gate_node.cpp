/**
 * @file safety_gate_node.cpp
 * @brief ROS2 node for a safety gate
 * @copyright 2025 TUM-FTM
 */

#include "tod_safety_gate/safety_gate.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tod_safety_gate::SafetyGateNode>());
    rclcpp::shutdown();
    return 0;
}