/**
 * @file rtsp_clients.hpp
 * @author Nils Gehrke
 * @brief ROS node for the RTSP server.
 * @version 1.0
 *
 * @copyright TUMFTM 2025
 */

#include <rclcpp/rclcpp.hpp>
#include "rtsp_server.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto server = std::make_shared<tod_rtsp::RtspServer>();
    server->run();
    rclcpp::spin(server);
    rclcpp::shutdown();
    return 0;
}
