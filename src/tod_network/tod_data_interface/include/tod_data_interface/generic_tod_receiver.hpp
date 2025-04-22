/**
 * @file generic_tod_reciever.cpp
 * @author Nils Gehrke
 * @brief This file implements a ROS2 node for receiving and processing messages based on the specified protocol (UDP or TCP).
 * @version 1.0
 * 
 * @copyright TUMFTM 2024
 * 
*/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tod_status_msgs/msg/status.hpp>
#include <tod_network_protocols/udp_receiver.hpp>
#include <tod_network_protocols/tcp_receiver.hpp>
#include "tod_data_interface/generic_tod_receiver.hpp"
#include <memory>
#include <algorithm>
#include <vector>
namespace tod_data_interface {

/**
 * @brief  TODO
 */
class GenericTODReceiver : public rclcpp::Node {
public:
    /**
     * @brief Constructs a GenericTODReceiver and does not creates a default sender.
     */
    GenericTODReceiver();

    /**
     * @brief Collects running thread
     */
    ~GenericTODReceiver();

    /**
     * @brief spawns thread to continously receive data
     */
    void receive();

private:
    typename rclcpp::GenericPublisher::SharedPtr _recv_msg_publisher; /**< Generic subscription for the messages to send. */
    std::unique_ptr<tod_network_protocols::BaseReceiver> _receiver; /**< Network sender implementation. */
    std::unique_ptr<std::thread> _receive_loop;
    uint8_t _connection_status{tod_status_msgs::msg::Status::TOD_STATUS_IDLE}; /**< Connection status. */
    rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr _status_subs; /**< Subscription for status messages. */
    std::string _topic;
    std::string _topic_type;
    int _destination_port;
    std::vector<uint8_t> _sending_control_modes; /**< List of allowed control modes for sending messages. */

    /**
     * @brief Callback for handling received status messages.
     * 
     * @param msg The received status message.
     */
    void status_message_received(const tod_status_msgs::msg::Status &msg);

    /**
     * @brief Callback for handling received messages to be sent over the network.
     * 
     * @param msg The serialized message received from the topic.
     */
    void message_receive();

    /**
     * @brief Sets the protocol strategy based on a string parameter.
     * 
     * This function initializes the appropriate network sender based on the protocol name provided.
     * Supported protocols include "udp" and "tcp". If an unsupported protocol is provided,
     * the function logs an error and throws an exception.
     * 
     * @param protocol_strategy The name of the protocol strategy (e.g., "udp", "tcp").
     * 
     * @throws std::invalid_argument If the protocol strategy is not supported.
     */
    void set_protocol_strategy(const std::string &protocol_strategy);
};

} // namespace tod_data_interface
