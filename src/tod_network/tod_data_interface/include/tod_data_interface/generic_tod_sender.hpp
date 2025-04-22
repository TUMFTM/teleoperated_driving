/**
 * @file generic_tod_sender.hpp
 * @author Nils Gehrke
 * @brief This file implements a ROS2 node for sending messages based on the specified protocol (UDP or TCP) to a given destination IP and port.
 * @version 1.0
 * 
 * @copyright TUMFTM 2024
 * 
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tod_status_msgs/msg/status.hpp>
#include <tod_network_protocols/udp_sender.hpp>
#include <tod_network_protocols/tcp_sender.hpp>
#include <memory>
#include <algorithm>
#include <vector>
namespace tod_data_interface {

/**
 * @brief A class for subscribing ros2 messages and sending them to an IP and port using a specified network protocol. Sending only happens if the status messages indicate a connection state.
 *          Different from the rod_sender, the deserialization of the message is not necessary.
 */
class GenericTODSender : public rclcpp::Node {
public:
    /**
     * @brief Constructs a GenericTODSender and does not create a default sender.
     */
    GenericTODSender();

    /**
     * @brief Sets the control modes in which messages will be sent (e.g. DirectControl, WaypointGuidance...).
     * 
     * @param mode The control mode to add to the allowed sending modes.
     */
    void send_in_control_mode(const uint8_t mode);

private:
    typename rclcpp::GenericSubscription::SharedPtr _send_msg_subs; /**< Generic subscription for the messages to send. */
    std::unique_ptr<tod_network_protocols::BaseSender> _sender; /**< Network sender implementation. */
    uint8_t _connection_status{tod_status_msgs::msg::Status::TOD_STATUS_IDLE}; /**< Connection status. */
    uint8_t _control_mode; /**< Current control mode. */
    rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr _status_subs; /**< Subscription for status messages. */
    bool _sender_in_vehicle; /**< Indicates whether the sender is in the vehicle. */
    bool _send_always; /**< flag wether the connection status is ignored for sending */
    std::string _topic;
    std::string _topic_type;
    std::string _destination_ip;
    int _destination_port;

    std::vector<uint8_t> _sending_control_modes; /**< List of allowed control modes for sending messages. */

    /**
     * @brief Checks whether the sender is connected.
     * 
     * @return True if connected, false otherwise.
     */
    bool connected();

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
    void send_message_received(const std::shared_ptr<rclcpp::SerializedMessage> &msg);

    /**
     * @brief Checks if the current control mode is allowed for sending messages.
     * 
     * @return True if the control mode allows sending, false otherwise.
     */
    bool in_sending_control_mode();


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
