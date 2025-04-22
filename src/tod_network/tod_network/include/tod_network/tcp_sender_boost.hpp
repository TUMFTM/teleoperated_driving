/**
 * @file tcp_sender_boost.hpp
 * @author Simon Hoffmann
 * @brief This file contains the implementation of a TCP senderclass using Boost.Asio. 
 *        The class handles synchronous and asynchronous data reception over TCP and provides 
 *        mechanisms for connecting to a client, sending data, and disconnecting.
 * @version 1.0
 *
 * @copyright TUMFTM 2020
  */
#pragma once
#include <iostream>
#include <vector>
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <tod_network_protocols/base_protocol.hpp>



namespace tod_network {

using boost::asio::ip::tcp;



/**
 * @class TcpSenderBoost
 * @brief TCP sender implementation that handles sending data using Boost.boost::asio to a specified server.
 */
class TcpSenderBoost : public tod_network_protocols::BaseSender {
public:
    /**
     * @brief Constructs a TcpSenderBoost with the given destination port and IP address.
     * @param io_context The context to run the asynchronous operations.
     * @param destPort The destination port for sending data.
     * @param ip The IP address of the server (default is "127.0.0.1").
     */
    TcpSenderBoost(boost::asio::io_context& io_context, const int destPort)
        : _io_context(io_context), _socket(io_context), _resolver(io_context), _dest_port{destPort} {
        // changeDestination(ip, destPort);
        std::cout << "TcpSenderBoost port: " << destPort << std::endl;
    }

    /**
     * @brief Destructor that closes the socket.
     */
    virtual ~TcpSenderBoost() { disconnect(); }

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
    void change_destination(const std::string& ip, const int port) override;

    /**
     * @brief Gets the destination IP address.
     * @return The destination IP address as a string.
     */
    std::string get_destination_ip() override;

    /**
     * @brief Disconnects and closes the socket.
     */
    void disconnect();

private:
    /**
     * @brief Connects to the server using resolved endpoints.
     */
    void connect_to_server(tcp::resolver::iterator endpoint_iterator);
    
    /**
     * @brief Closes the TCP socket
     * 
     */
    void close_socket();
    
    /**
     * @brief The IO context used for asynchronous operations.
     * 
     */
    boost::asio::io_context& _io_context;

    /**
     * @brief The socket for the TCP connection.
     * 
     */
    tcp::socket _socket;

    /**
     * @brief The resolver used to find the remote endpoint.
     * 
     */
    tcp::resolver _resolver;
    
    /**
     * @brief The destination IP address.
     * 
     */
    std::string _dest_ip = "0.0.0.0";

    /**
     * @brief The destination port.
     * 
     */
    int _dest_port{0};
};
}; // namespace tod_network