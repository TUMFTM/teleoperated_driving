/**
 * @file rtsp_clients_nodes.cpp
 * @author Nils Gehrke
 * @brief ROS Node for RTSP clients
 * @version 1.0
 *
 * @copyright TUMFTM 2024
  */
#include <rclcpp/rclcpp.hpp>
#include "rtsp_clients.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tod_rtsp::RtspClients>();
    node->run();
    rclcpp::shutdown();
    return 0;
}


