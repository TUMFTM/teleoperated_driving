/**
 * @file tcp_receiver.hpp
 * @author Simon Hoffmann
 * @brief This file contains the implementation of a TCP receiver class that handles synchronous and asynchronous data reception. It also defines a RAII-based `SocketGuard` class to ensure proper socket closure. The `TcpReceiver` class listens for incoming TCP connections, handles receiving data both synchronously and asynchronously, and provides functions for managing the client connection lifecycle.
 * 
 * The `TcpReceiver` class uses the `BaseReceiver` interface and is designed to be used in network communication systems where data is transmitted over TCP. It includes mechanisms to handle connection timeouts and asynchronous reception using `std::future`. The `SocketGuard` class is used to ensure proper socket management and avoid resource leaks.
 *
 * @version 1.0
 *
 * @copyright TUMFTM 2020
 */
#pragma once
#include <iostream>
#include <vector>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <rclcpp/serialization.hpp>
#include <cstring>
#include "base_protocol.hpp"
#include <future>
#ifndef MAXLINE
#define MAXLINE 1000*1024
#endif

namespace tod_network_protocols {

/**
 * @class SocketGuard
 * @brief RAII wrapper to ensure a socket is properly closed.
 */
class SocketGuard {
public:
    /**
     * @brief Constructs a SocketGuard for the given socket file descriptor.
     * @param fd The socket file descriptor to guard.
     */
    explicit SocketGuard(int& fd) : _socket_fd(fd) {}

    /**
     * @brief Destructor that closes the socket if it's still open.
     */
    ~SocketGuard() {
        if (_socket_fd != -1) {
            close(_socket_fd);
        }
    }

private:
    int& _socket_fd; ///< Reference to the socket file descriptor.
};

/**
 * @class TcpReceiver
 * @brief TCP receiver implementation that handles synchronous and asynchronous data reception.
 */
class TcpReceiver : public BaseReceiver {
public:
    /**
     * @brief Constructs a TcpReceiver for the given destination port.
     * @param destPort The port to listen for incoming connections.
     */
    explicit TcpReceiver(const int destPort) : _destPort(destPort) { }

    /**
     * @brief Destructor that closes the client socket.
     */
    ~TcpReceiver() { close(_client_socket); }

    /**
     * @brief Receives data synchronously from the client.
     * @return A vector containing the received data.
     */
    std::vector<uint8_t> receive() override;

    /**
     * @brief Receives data synchronously from the client.
     * @return A vector containing the received data.
     */
    std::vector<uint8_t> receive_with_timeout(int timeout_ms);

    /**
     * @brief Receives data asynchronously and executes a callback with the received data.
     * @param callback The callback function to execute with the received data.
     * @return A future object representing the asynchronous operation.
     */
    std::future<void> async_receive(std::function<void(const std::vector<uint8_t>&)> callback) override;

    /**
     * @brief Waits for a client to connect.
     * @return True if a client is connected, otherwise false.
     */
    bool waiting_for_client_connect() override;

    /**
     * @brief Disconnects the client and closes the socket.
     */
    void disconnect() override;

private:
    bool _connected{false}; ///< Indicates whether the receiver is connected to a client.
    int _destPort; ///< The destination port for incoming connections.
    int _client_socket{-1}; ///< The socket for client connections.

    /**
     * @brief Waits for a client to connect to the server.
     */
    void waiting_for_client_to_connect();

    /**
     * @brief Receives the size of the incoming message.
     * @return The size of the incoming message.
     */
    size_t receive_msg_size();

    /**
     * @brief Receives the payload based on the specified size.
     * @param sizeOfPayload The size of the payload to receive.
     * @return A vector containing the received payload.
     */
    std::vector<uint8_t> receive_payload(size_t sizeOfPayload);
};

}; //namespace tod_network_protocols