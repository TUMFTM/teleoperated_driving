/**
 * @file tod_sender.hpp
 * @author Johannes Feiler
 * @brief This file defines a `Sender` class template that subscribes to ROS2 messages and sends them to a specified IP address and port using a chosen network protocol. The `Sender` class ensures that messages are sent only when the system is in a valid connection state, as indicated by the status messages. It allows subscription to specific topics, checks control modes for sending, and handles network communication based on the current state of the system.
 *
 * The `Sender` class abstracts network communication by subscribing to ROS2 topics, serializing the messages, and sending them over the network using a specified protocol. It includes checks for connection and control modes, ensuring that messages are only sent under the appropriate conditions. The class also allows configuring multiple protocols for sending different types of messages.
 *
 * @version 1.0
 *
 * @copyright TUMFTM 2020
 */
#pragma once

#include "udp_sender.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tod_status_msgs/msg/status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <algorithm>
#include <vector>
namespace tod_network_protocols {

/**
 * @brief A class for subscribing ros2 messages and sending them to an ip and port using a specified network protocol. Sending only happens it the status messages indicates a connection state.
 * 
 * @tparam T The type of the ROS message to be sent.
 */
template <typename T>
class Sender : public rclcpp::Node {
private:
    /**
     * @brief Structure to manage subscriber and associated sender with a protocol.
     */
    struct RosMsgSender {
        typename rclcpp::Subscription<T>::SharedPtr sendMsgSubs; /**< Subscription for the messages to send. */
        std::unique_ptr<tod_network_protocols::BaseSender> Sender{nullptr}; /**< Network sender implementation. */
        bool printedInfo{false}; /**< Indicates whether info about the topic has been printed. */
    };

public:
    /**
     * @brief Constructs a Sender node with a protocol and an identifier which IP in the status he should use.
     * 
     * @param protocol_strategy The network protocol for sending messages.
     * @param senderInVehicle Indicates whether the sender is located in the vehicle.
     */
    Sender(std::unique_ptr<BaseSender>&& protocol_strategy, bool senderInVehicle);

    /**
     * @brief Constructs a Sender and does not create a default sender.
     * 
     * @param senderInVehicle Indicates whether the sender is located in the vehicle.
     */
    Sender(bool senderInVehicle);

    /**
     * @brief Adds a processor to subscribe and send a specific topic with a given protocol.
     * 
     * @param topic The topic to process.
     * @param protocol_strategy The protocol strategy to process the topic.
     */
    void add_processer(const std::string &topic, std::unique_ptr<BaseSender>&& protocol_strategy);

    /**
     * @brief Sets the control modes in which messages will be sent (e.g. DirectControl, WaypointGuidance...).
     * 
     * @param mode The control mode to add to the allowed sending modes.
     */
    void send_in_control_mode(const uint8_t mode);

private:
    std::vector<std::shared_ptr<RosMsgSender>> _ros_msg_senders; /**< List of message senders. */
    uint8_t _connection_stat{tod_status_msgs::msg::Status::TOD_STATUS_IDLE}; /**< Connection status. */
    uint8_t _control_mode; /**< Current control mode. */
    rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr _status_subs; /**< Subscription for status messages. */
    bool _sender_in_vehicle; /**< Indicates whether the sender is in the vehicle. */
    std::vector<uint8_t> _sending_control_modes; /**< List of allowed control modes for sending messages. */

    /**
     * @brief Checks whether the sender is connected.
     * 
     * @return True if connected, false otherwise.
     */
    bool connected();

    /**
     * @brief Serializes a ROS message into a byte vector using ROS2 serializer.
     * 
     * @param msg The ROS message to serialize.
     * @return A vector of bytes representing the serialized message.
     */
    std::vector<uint8_t> serialize(T& msg);

    /**
     * @brief Callback for handling received status messages.
     * 
     * @param msg The received status message.
     */
    void status_message_received(const tod_status_msgs::msg::Status &msg);

    /**
     * @brief Callback for handling received messages to be sent over the network.
     * 
     * @param event The message received from the topic.
     * @param rosMsgSender The associated message sender.
     */
    void send_message_received(const std::shared_ptr<T> event, std::shared_ptr<RosMsgSender> ros_msg_sender);

    /**
     * @brief Checks if the current control mode is allowed for sending messages.
     * 
     * @return True if the control mode allows sending, false otherwise.
     */
    bool in_sending_control_mode();
};

} // namespace tod_network_protocols