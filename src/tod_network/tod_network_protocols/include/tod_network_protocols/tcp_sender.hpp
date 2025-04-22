/**
 * @file tcp_sender.hpp
 * @author Simon Hoffmann
 * @brief This file contains the implementation of a TCP sender class that handles sending data to a specified server. The `TcpSender` class provides the necessary functionality for establishing a connection, sending data to a remote server, and managing the server address configuration. It also includes mechanisms to change the destination IP address and port.
 * 
 * The `TcpSender` class is designed for use in network communication systems where data needs to be transmitted over TCP. It allows you to send data to a specific destination, change the destination address if needed, and manage socket connections in a thread-safe manner.
 * 
 * The class also includes methods for printing connection specifications and disconnecting from the socket. It ensures that the socket is properly managed through its internal functions, such as creating and closing the socket.
 * 
 * @version 1.0
 *
 * @copyright TUMFTM 2020
 */
#pragma once
#include <iostream>
#include <vector>
#include <arpa/inet.h>
#include <thread>
#include <rclcpp/serialization.hpp>
#include <unistd.h>
#include <cstring>
#include "base_protocol.hpp"
#ifndef MAX_TRIES
#define MAX_TRIES 20
#endif

namespace tod_network_protocols {
/**
 * @class TcpSender
 * @brief TCP sender implementation that handles sending data to a specified server.
 */
class TcpSender : public BaseSender {
public:
    /**
     * @brief Constructs a TcpSender with the given destination port and IP address.
     * @param destPort The destination port for sending data.
     * @param ip The IP address of the server (default is "127.0.0.1").
     */
    TcpSender(const int destPort, const std::string& ip = "127.0.0.1");

    /**
     * @brief Destructor that closes the socket.
     */
    ~TcpSender();

    // Move and copy constructors and assignment operators
    TcpSender(TcpSender&&) = default; ///< Default move constructor
    TcpSender& operator=(TcpSender&&) = default; ///< Default move assignment operator
    TcpSender(const TcpSender&) = default; ///< Default copy constructor
    TcpSender& operator=(const TcpSender&) = default; ///< Default copy assignment operator

    /**
     * @brief Sends data to the server.
     * @param data The data to send.
     * @return The number of bytes sent.
     */
    int send_data(std::vector<uint8_t>& data) override;

    /**
     * @brief Changes the destination IP address and port.
     * @param ip The new IP address.
     * @param port The new port (default is -1, indicating no change).
     */
    void change_destination(const std::string& ip, const int port = -1) override;

    /**
     * @brief Prints the connection specifications.
     */
    void print_connection_specs();

    /**
     * @brief Gets the destination IP address.
     * @return The destination IP address as a string.
     */
    std::string get_destination_ip() override;

    /**
     * @brief Disconnect from the socket.
     * 
     */
    void disconnect() override;

private:
    /**
     * @brief Connects to the server.
     */
    void connect_to_server();

    /**
     * @brief Changes the IP address of the destination server.
     * @param ip The new IP address.
     */
    void change_ip_address(const std::string& ip);

    /**
     * @brief Creates a socket for the sender.
     */
    void create_socket();

    /**
     * @brief Closes the sender's socket.
     */
    void close_socket();

    int _sock{-1}; ///< The socket file descriptor.
    struct sockaddr_in _serv_addr; ///< The server address structure.
};
}; //namespace tod_network_protocols
