/**
 * @file udp_receiver.cpp
 * @author Nils Gehrke, based on the original version of 2020 by Feiler
 * @ingroup tod_network_protocols
 * @brief Defines the UdpReceiver class for receiving UDP messages over the network on a dedicated port. 
 * This file provides the implementation for the `UdpReceiver` class, which is responsible for managing 
 * a UDP socket and receiving messages. It supports both synchronous and asynchronous operations, 
 * allowing the receiver to handle incoming messages in a non-blocking manner using callbacks.
 * 
 * The `UdpReceiver` class is designed to facilitate UDP communication by receiving data packets 
 * on a specified port. It also provides an anti-block mechanism to ensure that the receiver 
 * remains responsive and avoids being blocked due to network conditions.
 *
 * @version 1.0
 *
 * @copyright TUMFTM 2024
 */
#include <rclcpp/rclcpp.hpp>
#include "tod_network_protocols/udp_receiver.hpp"

namespace tod_network_protocols {

UdpReceiver::UdpReceiver(const int destPort)
    : _anti_block_thread{}, _anti_block_sender(destPort) {
    memset(&_servaddr, 0, sizeof(_servaddr));
    _servaddr.sin_family = AF_INET;
    _servaddr.sin_port = htons(destPort);
    _servaddr.sin_addr.s_addr = INADDR_ANY;
    // creating socket file descriptor
    if ((_sockfd = socket(_servaddr.sin_family, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        printf("port %i", destPort);
        exit(EXIT_FAILURE);
    }
    // bind the socket with the server address
    if (bind(_sockfd, (const struct sockaddr *)&_servaddr, sizeof(_servaddr)) < 0) {
        perror("bind failed");
        printf("port %i", destPort);
        exit(EXIT_FAILURE);
    }
    _anti_block_thread = std::thread(&UdpReceiver::send_data_for_anti_block, this);
}

UdpReceiver::~UdpReceiver() {
    close(_sockfd);
}

std::vector<uint8_t> UdpReceiver::receive() {
    uint8_t buffer[MAXLINE];
    int num_bytes = recv(_sockfd, &buffer, MAXLINE, MSG_WAITALL);
    return std::vector<uint8_t>(buffer, buffer + num_bytes);
}

std::future<void> UdpReceiver::async_receive(std::function<void(const std::vector<uint8_t>&)> callback) {
    return std::async(std::launch::async, [this, callback]() {
        try {
            std::vector<uint8_t> data = this->receive();
            if (!data.empty()) {
                callback(data);
            }
        } catch (std::exception &ex) {
            printf("could not receive (failed on %s)\n", ex.what());
        }
    });
}

void UdpReceiver::send_data_for_anti_block() {
    if (!rclcpp::ok()) {
        printf("\033[31m ERROR @ UdpReceiver::send_data_for_anti_block() port %i: "
               " UDP-RECEIVER NEEDS TO BE initialized AFTER ros::init() \033[0m \n", _servaddr.sin_port);
    }

    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    // free block of receiver
    int integer{0};
    for (int it = 0; it != 20; ++it)
        _anti_block_sender.send((char *)&integer, sizeof(integer));
}

int UdpReceiver::wait_for_udp_receiver_to_close() {
    _anti_block_thread.join();
    return 0;
}

} // namespace tod_network_protocols
