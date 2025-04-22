
/**
 * @file PathTrackingControlNode.cp
 * @brief Node Loop for the \ref PathTrackingControl
 * @copyright 2024 TUMFTM
 * @ingroup tod_trajectory_guidance
 */

#include "tod_pure_pursuit/path_simulator.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tod_pure_pursuit::PathSimulator>());
    rclcpp::shutdown();
    return 0;
}
