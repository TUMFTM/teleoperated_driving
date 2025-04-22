/**
 * @file udp_sender.cpp
 * @author Johannes Feiler
 * @brief This file defines the `UdpSender` class, which provides an implementation for sending UDP messages to a specified destination over the network. The `UdpSender` class allows sending data to a specific IP address and port and supports changing the destination address dynamically. It also provides functionality to print connection details and manage the socket connection.
 * 
 * The `UdpSender` class inherits from the `BaseSender` class and includes methods to send data, change the destination IP address and port, and manage network communication using UDP. It ensures that data is properly sent to the specified destination, and the connection details are accessible via the provided methods.
 *
 * @version 1.0
 *
 * @copyright TUMFTM 2020
 */
#include "tod_network_protocols/udp_sender.hpp"
#include <cstdio>
#include <cstdlib>

namespace tod_network_protocols {

UdpSender::UdpSender(int destPort, const std::string &destIPAddress) {
    change_destination(destIPAddress, destPort);
    // Create sender socket file descriptor
    if ((_sockfd = socket(_servaddr.sin_family, SOCK_DGRAM, 0)) < 0) {
        perror("Cannot create socket");
        printf("%s:%i", destIPAddress.c_str(), destPort);
        exit(EXIT_FAILURE);
    }
}

UdpSender::~UdpSender() {
    close(_sockfd);
}

int UdpSender::send_data(std::vector<uint8_t> &data) {
    return sendto(_sockfd, data.data(), data.size(), 0,
                  (struct sockaddr *)&_servaddr, sizeof(_servaddr));
}

void UdpSender::change_destination(const std::string &destIPAddress, int destPort) {
    // Fill _servaddr with values
    inet_aton(destIPAddress.c_str(), &(_servaddr.sin_addr)); // IP
    if (destPort >= 0)
        _servaddr.sin_port = htons(destPort); // Port
    _servaddr.sin_family = AF_INET;
}

int UdpSender::send(const char *msg, size_t size) {
    return sendto(_sockfd, msg, size, 0, (struct sockaddr *)&_servaddr, sizeof(_servaddr));
}

void UdpSender::print_connection_specs() {
    printf("Destination IP Address: %s\n", inet_ntoa(_servaddr.sin_addr));
    auto port = ntohs(_servaddr.sin_port);
    printf("Destination Port: %i\n", port);
}

std::string UdpSender::get_destination_ip() {
    return std::string(inet_ntoa(_servaddr.sin_addr));
}

} // namespace tod_network_protocols
