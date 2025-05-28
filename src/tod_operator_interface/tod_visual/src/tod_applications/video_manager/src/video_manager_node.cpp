/**
 * @file main.cpp
 * @brief Main entry point for the Video Manager application.
 * @copyright 2021 TUMFTM
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "video_manager.hpp"

/**
 * @brief Main function for the Video Manager application.
 * 
 * Initializes the ROS 2 environment, creates the Video Manager, and runs the application.
 * 
 * @param argc The number of command-line arguments.
 * @param argv The array of command-line arguments.
 * @return int Exit status code.
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv); ///< Initialize ROS 2.
    auto manager = std::make_unique<tod_gl::VideoManager>(argc, argv, "VideoManager"); ///< Create a unique pointer to the Video Manager.
    manager->initialize(); ///< Initialize the Video Manager.
    manager->run(); ///< Run the Video Manager.
    return 0; ///< Exit with status code 0.
}
