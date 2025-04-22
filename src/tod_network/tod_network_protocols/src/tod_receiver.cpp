/**
 * @file tod_receiver.cpp
 * @author Johannes Feiler
 * @brief This file defines a `Receiver` class template that receives network messages using a specific protocol and publishes them as ROS2 messages. The class supports receiving and processing messages asynchronously from multiple protocols, deserializing them into ROS2 messages, and publishing them on configured topics. It allows optional message restamping with the current time.
 *
 * The `Receiver` class is designed to facilitate the handling of incoming network data by abstracting the protocol details. It uses a customizable protocol strategy (such as UDP or other transport mechanisms) for receiving messages. The class also supports publishing additional information like packet metadata using a `PaketInfo` message, which is published alongside the received message.
 *
 * @version 1.0
 *
 * @copyright TUMFTM 2020
 */

#include "tod_network_protocols/tod_receiver.hpp"

namespace tod_network_protocols {

template <typename T>
Receiver<T>::Receiver(std::unique_ptr<BaseReceiver>&& protocol_strategy, bool restamp)
    : Node("test_receiver") {
    add_processer("/received_topic", std::move(protocol_strategy), restamp);
}

template <typename T>
Receiver<T>::Receiver() : Node("test_receiver") {}

template <typename T>
void Receiver<T>::add_processer(const std::string &topic, std::unique_ptr<BaseReceiver>&& protocol_strategy, const bool restamp) {
    auto receiver = _ros_msg_receivers.emplace_back(std::make_shared<RosMsgReceiver>());
    receiver->receiver = std::move(protocol_strategy);
    receiver->recv_msg_pubs = this->create_publisher<T>(topic, 1);
    receiver->recv_msg_paket_info_pubs = this->create_publisher<tod_network_msgs::msg::PaketInfo>(topic + "_paket_info", 1);
    receiver->restamp = restamp;
}

template <typename T>
void Receiver<T>::receive() {
    for (auto receiver : _ros_msg_receivers)
        receiver->thread = std::make_unique<std::thread>(&Receiver::receive_msgs, this, receiver);
}

template <typename T>
int Receiver<T>::deserialize_data(const std::vector<uint8_t>& data, T* msg) {
    rclcpp::SerializedMessage serMsg(data.size() + 1);
    auto& rcl_handle = serMsg.get_rcl_serialized_message();

    std::memcpy(rcl_handle.buffer, data.data(), data.size());
    rcl_handle.buffer[data.size()] = '\0';
    rcl_handle.buffer_length = static_cast<size_t>(data.size());

    rclcpp::Serialization<T> serializer;
    try {
        serializer.deserialize_message(&serMsg, msg);
        return 1;
    } catch (std::exception &ex) {
        printf("could not deserialize (failed on %s)\n", ex.what());
    }
    return 0;
}

template <typename T>
void Receiver<T>::receive_msgs(std::shared_ptr<RosMsgReceiver> receiver) {
    receiver->receiver->waiting_for_client_connect();
    while (rclcpp::ok()) {
        std::vector<uint8_t> data = receiver->Receiver->receive();
        deserialize_data(data, &receiver->msg);

        receiver->paket_info_msg.header.stamp = this->now();
        receiver->paket_info_msg.size_bit = data.size() * 8;
        receiver->paket_info_msg.latency_usec = (this->now().nanoseconds() / 1000) -
                                              (1000000 * static_cast<uint64_t>(receiver->msg.header.stamp.sec) +
                                               receiver->msg.header.stamp.nanosec / 1000);
        receiver->recv_msg_paket_info_pubs->publish(receiver->paket_info_msg);

        if (receiver->restamp)
            receiver->msg.header.stamp = this->now();
        receiver->recv_msg_pubs->publish(receiver->msg);

        if (!receiver->printedInfo) {
            receiver->printedInfo = true;
            RCLCPP_INFO(this->get_logger(), "%s: receiving topic %s", this->get_name(), receiver->recv_msg_pubs->get_topic_name());
        }
    }
    receiver->receiver->disconnect();
}

} // namespace tod_network_protocols