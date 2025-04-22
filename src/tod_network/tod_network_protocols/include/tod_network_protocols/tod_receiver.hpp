/**
 * @file tod_receiver.hpp
 * @author Johannes Feiler
 * @brief This file defines a `Receiver` class template that receives network messages using a specific protocol and publishes them as ROS2 messages. The class supports receiving and processing messages asynchronously from multiple protocols, deserializing them into ROS2 messages, and publishing them on configured topics. It allows optional message restamping with the current time.
 *
 * The `Receiver` class is designed to facilitate the handling of incoming network data by abstracting the protocol details. It uses a customizable protocol strategy (such as UDP or other transport mechanisms) for receiving messages. The class also supports publishing additional information like packet metadata using a `PaketInfo` message, which is published alongside the received message.
 *
 * @version 1.0
 *
 * @copyright TUMFTM 2020
 */

#pragma once
#include "udp_receiver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tod_status_msgs/msg/status.hpp>
#include <tod_network_msgs/msg/paket_info.hpp>
#include <memory>
#include <thread>
#include <vector>
#include <cstring>
namespace tod_network_protocols {

/**
 * @brief A class for receiving network messages using a specific protocol and publishing them as ROS2 messages.
 * 
 * @tparam T The type of the ROS2 message to be published.
 */
template <typename T>
class Receiver : public rclcpp::Node {
private:
    /**
     * @brief Structure to manage publishers and associated receiver with a protocol.
     */
    struct RosMsgReceiver {
        typename rclcpp::Publisher<T>::SharedPtr recv_msg_pubs; /**< Publisher for the received messages. */
        rclcpp::Publisher<tod_network_msgs::msg::PaketInfo>::SharedPtr recv_msg_paket_info_pubs; /**< Publisher for PaketInfo messages. */
        std::unique_ptr<tod_network_protocols::BaseReceiver> receiver{nullptr}; /**< Network receiver implementation. */
        bool printed_info{false}; /**< Indicates whether info about the topic has been printed. */
        bool restamp{false}; /**< Indicates whether to restamp messages with the current time. */
        std::unique_ptr<std::thread> thread{nullptr}; /**< Thread for asynchronous message receiving. */
        T msg; /**< The ROS message to be published. */
        tod_network_msgs::msg::PaketInfo paket_info_msg; /**< The PaketInfo message to be published. */
    };

public:
    /**
     * @brief Constructs a Receiver node with a protocol and optional restamping.
     * 
     * @param protocol_strategy The network protocol for receiving messages.
     * @param restamp Indicates whether to restamp received messages with the current time.
     */
    Receiver(std::unique_ptr<BaseReceiver>&& protocol_strategy, bool restamp = false);

    /**
     * @brief Constructs a Receiver without creating a default receiver.
     */
    explicit Receiver();

    /**
     * @brief Adds a processor to publish received messages from a specific topic using a given protocol.
     * 
     * @param topic The topic to publish.
     * @param protocol_strategy The protocol to receive the topic with.
     * @param restamp Indicates whether to restamp the messages with the current time.
     */
    void add_processer(const std::string &topic, std::unique_ptr<BaseReceiver>&& protocol_strategy, const bool restamp = false);

    /**
     * @brief Starts receiving messages from all configured receivers.
     */
    void receive();

private:
    std::vector<std::shared_ptr<RosMsgReceiver>> _ros_msg_receivers; /**< List of processes */

    /**
     * @brief Deserializes data received over the network into a ROS2 message using ROS Deserializer.
     * 
     * @param data The received data to deserialize.
     * @param msg The ROS message object to populate with deserialized data.
     * @return 1 if successful, 0 otherwise.
     */
    int deserialize_data(const std::vector<uint8_t>& data, T * msg);

    /**
     * @brief Handles receiving and processing messages asynchronously.
     * 
     * @param receiver The receiver to process messages from.
     */
    void receive_msgs(std::shared_ptr<RosMsgReceiver> receiver);
};

} // namespace tod_network_protocols
