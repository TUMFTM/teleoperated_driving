/**
 * @file tcp_receiver_boost.hpp
 * @author Simon Hoffmann
 * @brief This file contains the implementation of a TCP receiver class using Boost.Asio. 
 *        The class handles synchronous and asynchronous data reception over TCP and provides 
 *        mechanisms for connecting to a client, receiving data, and disconnecting.
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
#include <tod_network_protocols/base_protocol.hpp>
#include <future>
#ifndef MAXLINE
#define MAXLINE 1000*1024
#endif

#include <boost/asio.hpp>
#include <vector>
#include <iostream>
#include <future>

using boost::asio::ip::tcp;


namespace tod_network {

/**
 * @class TcpReceiverBoost
 * @brief TCP receiver implementation that handles synchronous and asynchronous data reception.
 */
class TcpReceiverBoost : public tod_network_protocols::BaseReceiver {
public:
    explicit TcpReceiverBoost(boost::asio::io_context& io_context, int destPort)
        : _io_context(io_context), _acceptor(io_context, tcp::endpoint(tcp::v4(), destPort)), _socket(io_context) {

        std::cout << "TcpReceiverBoost port: " << destPort << std::endl;
    }
    /**
     * @brief Destructor that closes the client socket.
     */
    ~TcpReceiverBoost() { disconnect(); }

    /**
     * @brief Receives data synchronously from the client.
     * @return A vector containing the received data.
     */
    std::vector<uint8_t> receive() override;


    /*
    This async receive spins up another thread to process the receive() function. However, within
    the thread, the receive() function is still blocking. In case there are no data to receive,
    the thread will continue to run and never reach a joinable state.
    */
    std::future<void> async_receive(std::function<void(const std::vector<uint8_t>&)> callback);

    /**
     * @brief Wait for a client (sender) to connect
     * 
     * 
     * @return true if connected
     * @return false if connection fails
     */
    bool waiting_for_client_connect() override;

    /**
     * @brief Disconnect from the client
     * 
     */
    void disconnect() override;

    /**
     * @brief Get the boost io_context
     * 
     * @return boost::asio::io_context& 
     */
    boost::asio::io_context& get_io();

private:
    /**
     * @brief Indicates whether the receiver is connected to a client.
     * 
     */
    bool _connected{false};

    /**
     * @brief boost io_context
     * 
     */
    boost::asio::io_context& _io_context;

    /**
     * @brief tcp acceptor
     * 
     */
    tcp::acceptor _acceptor;

    /**
     * @brief tcp socket
     * 
     */
    tcp::socket _socket;
};

}; //namespace tod_network
