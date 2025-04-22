/**
 * @file tod_sender.cpp
 * @author Johannes Feiler
 * @brief This file defines a `Sender` class template that subscribes to ROS2 messages and sends them to a specified IP address and port using a chosen network protocol. The `Sender` class ensures that messages are sent only when the system is in a valid connection state, as indicated by the status messages. It allows subscription to specific topics, checks control modes for sending, and handles network communication based on the current state of the system.
 *
 * The `Sender` class abstracts network communication by subscribing to ROS2 topics, serializing the messages, and sending them over the network using a specified protocol. It includes checks for connection and control modes, ensuring that messages are only sent under the appropriate conditions. The class also allows configuring multiple protocols for sending different types of messages.
 *
 * @version 1.0
 *
 * @copyright TUMFTM 2020
 */
#include "tod_network_protocols/tod_sender.hpp"

namespace tod_network_protocols {

template <typename T>
Sender<T>::Sender(std::unique_ptr<BaseSender>&& protocol_strategy, bool senderInVehicle)
    : Node("test_sender"), _sender_in_vehicle(senderInVehicle) {
    std::string statusTopic = (_sender_in_vehicle) ? "/Vehicle/Manager/status_msg" : "/Operator/Manager/status_msg";
    _status_subs = this->create_subscription<tod_status_msgs::msg::Status>(statusTopic, 1, std::bind(&Sender::status_message_received, this, std::placeholders::_1));
    add_processer("/topic_to_send", std::move(protocol_strategy));
}

template <typename T>
Sender<T>::Sender(bool senderInVehicle)
    : Node("test_sender"), _sender_in_vehicle(senderInVehicle) {
    std::string statusTopic = (_sender_in_vehicle) ? "/Vehicle/Manager/status_msg" : "/Operator/Manager/status_msg";
    _status_subs = this->create_subscription<tod_status_msgs::msg::Status>(statusTopic, 1, std::bind(&Sender::status_message_received, this, std::placeholders::_1));
}

template <typename T>
void Sender<T>::add_processer(const std::string &topic, std::unique_ptr<BaseSender>&& protocol_strategy) {
    std::shared_ptr<RosMsgSender> sender = _ros_msg_senders.emplace_back(std::make_shared<RosMsgSender>());
    sender->Sender = std::move(protocol_strategy);
    std::function<void(const std::shared_ptr<T>)> fnc =
        std::bind(&Sender::send_message_received, this, std::placeholders::_1, sender);
    sender->send_msg_subs = this->create_subscription<T>(topic, 10, fnc);
}

template <typename T>
void Sender<T>::send_in_control_mode(const uint8_t mode) {
    _sending_control_modes.push_back(mode);
}

template <typename T>
bool Sender<T>::connected() {
    return _connection_stat != tod_status_msgs::msg::Status::TOD_STATUS_IDLE;
}

template <typename T>
std::vector<uint8_t> Sender<T>::serialize(T& msg) {
    rclcpp::SerializedMessage serMsg;
    rclcpp::Serialization<T> serializer;
    try {
        serializer.serialize_message(&msg, &serMsg);
        uint8_t* data = serMsg.get_rcl_serialized_message().buffer;
        int size = serMsg.get_rcl_serialized_message().buffer_length;
        return std::vector<uint8_t>(data, data + size);
    } catch (std::exception &ex) {
        printf("could not serialize (failed on %s)\n", ex.what());
    }
    return {};
}

template <typename T>
void Sender<T>::status_message_received(const tod_status_msgs::msg::Status &msg) {
    _connection_stat = msg.tod_status;
    _control_mode = _sender_in_vehicle ? msg.vehicle_control_mode : msg.operator_control_mode;
    std::string receiverIp;
    if (connected() && in_sending_control_mode()) {
        receiverIp = _sender_in_vehicle ? msg.operator_ip_address : msg.vehicle_ip_address;
        for (auto& ros_msg_sender : _ros_msg_senders) {
            if (ros_msg_sender->Sender->get_destination_ip() != receiverIp) {
                RCLCPP_INFO(this->get_logger(), "%s: sending to ip address %s",
                            this->get_name(), receiverIp.c_str());
                ros_msg_sender->Sender->change_destination(receiverIp);
            }
        }
    }
}

template <typename T>
void Sender<T>::send_message_received(const std::shared_ptr<T> event, std::shared_ptr<RosMsgSender> rosMsgSender) {
    if (connected() && in_sending_control_mode()) {
        std::vector<uint8_t> vec = serialize(*event);
        int nofBytesSent = rosMsgSender->Sender->send_data(vec);
        if (!rosMsgSender->printedInfo) {
            RCLCPP_INFO(this->get_logger(), "%s: sending topic %s - msg size %d", this->get_name(),
                        rosMsgSender->send_msg_subs->get_topic_name().c_str(), nofBytesSent);
            rosMsgSender->printedInfo = true;
        }
    }
}

template <typename T>
bool Sender<T>::in_sending_control_mode() {
    if (_sending_control_modes.empty()) return true;
    return std::any_of(_sending_control_modes.begin(), _sending_control_modes.end(),
                       [this](uint8_t sendingMode) { return _control_mode == sendingMode; });
}

} // namespace tod_network