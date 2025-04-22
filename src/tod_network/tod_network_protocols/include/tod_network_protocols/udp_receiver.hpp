/**
 * @file udp_receiver.hpp
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
#pragma once  

#include <arpa/inet.h>
#include <thread>
#include "udp_sender.hpp"
#include <cstring> // std::memcpy
#include <iostream>
#include <vector>
#include "base_protocol.hpp"
#include <future> 

#ifndef MAXLINE
#define MAXLINE 100*1024
#endif


namespace tod_network_protocols {

/**
 * @file udp_receiver.h
 * @ingroup tod_network_protocols
 * @brief Defines the UdpReceiver class for receiving UDP messages over the network on a dedicated port.
 */

/**
 * @class UdpReceiver
 * @brief Handles receiving UDP messages, including support for asynchronous operations.
 * 
 * The UdpReceiver class encapsulates the setup and operation of a UDP socket
 * to receive messages and manage blocking scenarios. It includes functionality 
 * to handle asynchronous message reception via callbacks.
 */
class UdpReceiver : public BaseReceiver {
public:
    /**
     * @brief Constructor, initializes the UDP socket.
     * @param destPort The port number to bind the UDP socket.
     */
    explicit UdpReceiver(const int destPort);

    /**
     * @brief Destructor
     */
    ~UdpReceiver();

    /**
     * @brief triggers recieve call that checks for an incoming UDP package.
     * @return vector of received bytes.
     */
    std::vector<uint8_t> receive() override;

    /**
     * @brief Waits for the UDP receiver to close during asynchronous communication
     * @return An integer status code (0 for success). Only return value is 0, no timeout implemented
     */
    int wait_for_udp_receiver_to_close();

    /**
     * @brief Asynchronous wrapper for the receive function. 
     * @param callback A callback function to handle the received data.
     * @return A std::future representing the asynchronous operation.
     */
    std::future<void> async_receive(std::function<void(const std::vector<uint8_t>&)> callback) override;

private:
    struct sockaddr_in _servaddr;  ///< Socket address structure for server.
    int _sockfd;                   ///< Socket file descriptor.
    std::thread _anti_block_thread;///< Thread to ensure port is not blocked by sending data to the port.
    UdpSender _anti_block_sender;  ///< Sender instance for the thread.

    /**
     * @brief Sends data to unblock the receiver in case of blocking scenarios.
     */
    void send_data_for_anti_block();
};

} // namespace tod_network_protocols
