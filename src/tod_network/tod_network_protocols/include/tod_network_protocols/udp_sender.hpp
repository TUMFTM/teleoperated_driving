/**
 * @file udp_sender.hpp
 * @author Johannes Feiler
 * @brief This file defines the `UdpSender` class, which provides an implementation for sending UDP messages to a specified destination over the network. The `UdpSender` class allows sending data to a specific IP address and port and supports changing the destination address dynamically. It also provides functionality to print connection details and manage the socket connection.
 * 
 * The `UdpSender` class inherits from the `BaseSender` class and includes methods to send data, change the destination IP address and port, and manage network communication using UDP. It ensures that data is properly sent to the specified destination, and the connection details are accessible via the provided methods.
 *
 * @version 1.0
 *
 * @copyright TUMFTM 2024
 */
#pragma once
#include "tod_network_protocols/base_protocol.hpp"

#include <arpa/inet.h>
#include <string>
#include <vector>
#include <unistd.h>
#include <rclcpp/serialization.hpp>

namespace tod_network_protocols {

class UdpSender : public BaseSender {
public:
    UdpSender(int destPort, const std::string &dest_ip_address = "127.0.0.1");
    ~UdpSender();

    UdpSender(UdpSender &&) = default;
    UdpSender &operator=(UdpSender &&) = default;
    UdpSender(const UdpSender &) = default;
    UdpSender &operator=(const UdpSender &) = default;

    int send_data(std::vector<uint8_t> &data) override;

    void change_destination(const std::string &destIPAddress, int destPort = -1) override;

    int send(const char *msg, size_t size);

    void print_connection_specs();

    std::string get_destination_ip() override;

    void disconnect() override {}

private:
    struct sockaddr_in _servaddr;
    int _sockfd;
};

} // namespace tod_network_protocols
