/**
 * @file packet_capture_service_forwarder.cpp
 * @author Xiyan Su (xiyan.su@tum.de)
 * @brief This file runs a ROS node to forward the NetworkMonitorService from operator to vehicle
 * @version 0.1
 * @date 2024-11-28
 * 
 * @copyright TUMFTM 2024
 * 
 */

#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tod_network_monitoring_msgs/srv/packet_capture_service.hpp>
#include <tod_network/tcp_sender_boost.hpp>
#include <tod_network/tcp_receiver_boost.hpp>
#include <tod_network_protocols/udp_sender.hpp>
#include <tod_network_protocols/udp_receiver.hpp>
#include <tod_network/tod_service_forwarder.hpp>

/**
 * @brief Print out help for using this file
 * 
 */
void print_help() {
    std::cout << "Usage: ros2 run tod_communication_interface NetworkMonitorServiceForwarder <service> <protocol>\n"
              << "  <service>: Service name (string)\n"
              << "  <protocol>: Protocol to use (string) [TCP, UDP]\n"
              << "  <forwarder_port>: Port to be used by forwarder (string)\n"
              << "  <listener_port>:  Port to be used by listener (string)\n";
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Parse arguments
    if (argc < 5) {
        std::cerr << "Error: Not enough arguments provided.\n";
        print_help();
        return 1;
    }

    std::string service_name = argv[1];  // The name of the ROS service
    std::string protocol = argv[2];  // Which protocol to use, TCP or UDP
    int forwarder_port = std::stoi(argv[3]);
    int listener_port = std::stoi(argv[4]);


    // Ignore extra arguments
    if (argc > 5) {
        RCLCPP_DEBUG(rclcpp::get_logger("PacketCaptureServiceForwarder"), "Warning: Extra arguments ignored.\n");
    }

    std::shared_ptr<tod_network::ServiceForwarder<tod_network_monitoring_msgs::srv::PacketCaptureService>> packet_capture_service_forwarder;

    boost::asio::io_context io;

    if (protocol == "UDP") {  // UDP
        packet_capture_service_forwarder = std::make_shared<tod_network::ServiceForwarder<tod_network_monitoring_msgs::srv::PacketCaptureService>>(
            "PacketCaptureServiceForwarder",
            service_name,
            false,
            std::make_unique<tod_network_protocols::UdpSender>(forwarder_port),
            std::make_unique<tod_network_protocols::UdpReceiver>(listener_port)
        );
    } else {  // TCP
        if (protocol != "TCP") {
            // Default use TCP if the protocol is not given correctly
            std::cerr << "Unrecognized Protocol: " << protocol << ". Falling back to TCP.\n";
        }
        packet_capture_service_forwarder = std::make_shared<tod_network::ServiceForwarder<tod_network_monitoring_msgs::srv::PacketCaptureService>>(
            "PacketCaptureServiceForwarder",
            service_name,
            false,
            std::make_unique<tod_network::TcpSenderBoost>(io, forwarder_port),
            std::make_unique<tod_network::TcpReceiverBoost>(io, listener_port)
        );
    }

    rclcpp::spin(packet_capture_service_forwarder);
    rclcpp::shutdown();

    return 0;
}
