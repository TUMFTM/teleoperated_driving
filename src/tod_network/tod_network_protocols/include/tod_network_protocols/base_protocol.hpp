/**
 * @file base_protocol.hpp
 * @brief Defines the BaseReceiver and BaseSender abstract base classes for network communication.
 * @class BaseReceiver
 * @brief Abstract base class for receiving data in a network protocol.
 * 
 * This class defines the interface for receiving data, either synchronously or asynchronously,
 * and handling client connections.
 */
#pragma once
#include <vector>
#include <stdint.h>
#include <string>
#include <functional>
#include <future>

namespace tod_network_protocols {


class BaseReceiver {
public:
    /**
     * @brief Default constructor for BaseReceiver.
     */
    BaseReceiver() = default;

    /**
     * @brief Virtual destructor for BaseReceiver.
     */
    virtual ~BaseReceiver() = default;

    /**
     * @brief Move constructor.
     */
    BaseReceiver(BaseReceiver&&) = default;

    /**
     * @brief Move assignment operator.
     * @return Reference to the assigned BaseReceiver instance.
     */
    BaseReceiver& operator=(BaseReceiver&&) = default;

    /**
     * @brief Copy constructor.
     */
    BaseReceiver(const BaseReceiver&) = default;

    /**
     * @brief Copy assignment operator.
     * @return Reference to the assigned BaseReceiver instance.
     */
    BaseReceiver& operator=(const BaseReceiver&) = default;

    /**
     * @brief Pure virtual function to receive data.
     * @return A vector containing the received bytes.
     */
    virtual std::vector<uint8_t> receive() = 0;

    /**
     * @brief Pure virtual function for asynchronous data reception.
     * @param callback A callback function to handle the received data.
     * @return A std::future representing the asynchronous operation.
     */
    virtual std::future<void> async_receive(std::function<void(const std::vector<uint8_t>&)> callback) = 0;

    /**
     * @brief Checks if the receiver is waiting for a client to connect.
     * @return `true` if waiting for client connection, `false` otherwise.
     */
    virtual bool waiting_for_client_connect() { return true; }

    /**
     * @brief Disconnects the receiver from the client.
     */
    virtual void disconnect() { }
private:
};

/**
 * @class BaseSender
 * @brief Abstract base class for sending data in a network protocol.
 * 
 * This class defines the interface for sending data and managing destination connections.
 */
class BaseSender {
public:
    /**
     * @brief Default constructor for BaseSender.
     */
    BaseSender() = default;

    /**
     * @brief Virtual destructor for BaseSender.
     */
    virtual ~BaseSender() = default;

    /**
     * @brief Move constructor.
     */
    BaseSender(BaseSender&&) = default;

    /**
     * @brief Move assignment operator.
     * @return Reference to the assigned BaseSender instance.
     */
    BaseSender& operator=(BaseSender&&) = default;

    /**
     * @brief Copy constructor.
     */
    BaseSender(const BaseSender&) = default;

    /**
     * @brief Copy assignment operator.
     * @return Reference to the assigned BaseSender instance.
     */
    BaseSender& operator=(const BaseSender&) = default;

    /**
     * @brief Pure virtual function to disconnect the sender from the client.
     */
    virtual void disconnect() = 0;

    /**
     * @brief Sends data to the destination.
     * @param data A vector containing the data to be sent.
     * @return An integer status code representing the success or failure of the operation.
     */
    virtual int send_data(std::vector<uint8_t>& data) = 0;

    /**
     * @brief Changes the destination IP address and optionally the port.
     * @param destIPAddress The new destination IP address.
     * @param destPort The new destination port (optional).
     */
    virtual void change_destination(const std::string& destIPAddress, const int destPort = -1) = 0;

    /**
     * @brief Gets the current destination IP address.
     * @return A string representing the destination IP address.
     */
    virtual std::string get_destination_ip() = 0;
private:
};

}; // namespace tod_network_protocols
