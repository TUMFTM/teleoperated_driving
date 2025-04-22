/**
 * @file generic_tod_sender.cpp
 * @author Nils Gehrke
 * @brief This file implements a ROS2 node for sending messages based on the specified protocol (UDP or TCP) to a given destination IP and port.
 * @version 1.0
 * 
 * @copyright TUMFTM 2024
 * 
 */

#include "tod_data_interface/generic_tod_sender.hpp"

using namespace tod_data_interface;

GenericTODSender::GenericTODSender() : rclcpp::Node("generic_tod_sender"), _sender_in_vehicle(false), _control_mode(0) {

    declare_parameter<std::string>("protocol_strategy", "udp");
    declare_parameter<std::string>("topic", "/message_to_send");
    declare_parameter<std::string>("topic_type", "std_msgs/msg/String");
    declare_parameter<std::string>("ip", "127.0.0.1");
    declare_parameter<int>("port", 5000);
    declare_parameter<bool>("in_vehicle", true);
    declare_parameter<bool>("send_always", false);
    declare_parameter<std::vector<int>>("sending_control_modes", std::vector<int>{}); //integer because of better yaml readability
    declare_parameter<std::string>("status_topic", "input/operator_status");

    std::string protocol_strategy = get_parameter("protocol_strategy").as_string();
    this->_topic = get_parameter("topic").as_string();
    this->_topic_type = get_parameter("topic_type").as_string();
    this->_destination_ip = get_parameter("ip").as_string();
    this->_destination_port = get_parameter("port").as_int();
    this->_sender_in_vehicle = get_parameter("in_vehicle").as_bool();
    this->_send_always = get_parameter("send_always").as_bool();
    std::string status_topic = get_parameter("status_topic").as_string();
    auto _sending_control_modes_parameter = get_parameter("sending_control_modes").as_integer_array();
    // convert integer values to bytes
    std::for_each(_sending_control_modes_parameter.begin(), _sending_control_modes_parameter.end(),
                  [this](int value) {
                      this->send_in_control_mode(static_cast<uint8_t>(value));
                  });



    set_protocol_strategy(protocol_strategy);

    // Create generic subscription using the topic namespace
    this->_send_msg_subs = this->create_generic_subscription(
        this->_topic,
        this->_topic_type,
        1,
        [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            send_message_received(msg);
        });
    
    this->_status_subs = this->create_subscription<tod_status_msgs::msg::Status>(
      status_topic, 
      1, 
      std::bind(&GenericTODSender::status_message_received, this, std::placeholders::_1)
    );
}

void GenericTODSender::send_in_control_mode(const uint8_t mode) {
    _sending_control_modes.push_back(mode);
}


void GenericTODSender::set_protocol_strategy(const std::string &protocol_strategy) {
    if (protocol_strategy == "udp") {
        _sender = std::make_unique<tod_network_protocols::UdpSender>(this->_destination_port, this->_destination_ip);
    } else if (protocol_strategy == "tcp") {
        // Example: Replace with actual TCP sender initialization if available
        _sender = std::make_unique<tod_network_protocols::TcpSender>(this->_destination_port, this->_destination_ip);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported protocol strategy: %s", protocol_strategy.c_str());
        throw std::invalid_argument("Unsupported protocol strategy");
    }
}

bool GenericTODSender::connected() {
    return (_connection_status == tod_status_msgs::msg::Status::TOD_STATUS_UPLINK_ONLY) ||
        (_connection_status == tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION);
}

void GenericTODSender::status_message_received(const tod_status_msgs::msg::Status &msg) {
    _connection_status = msg.tod_status;
    _control_mode = msg.operator_control_mode;
    _destination_ip = _sender_in_vehicle ? msg.operator_ip_address : msg.vehicle_ip_address;
}

void GenericTODSender::send_message_received(const std::shared_ptr<rclcpp::SerializedMessage> &msg) {
    if ( _sender && (this->_send_always || (connected() && in_sending_control_mode()))) {
        auto data = std::vector<uint8_t>(msg->get_rcl_serialized_message().buffer, 
                                         msg->get_rcl_serialized_message().buffer + msg->get_rcl_serialized_message().buffer_length);
        if (this->_sender->get_destination_ip() != this->_destination_ip) {
                RCLCPP_INFO(this->get_logger(), "%s: sending to ip address %s",
                            this->get_name(), _destination_ip.c_str());
                this->_sender->change_destination(_destination_ip);
            }
        _sender->send_data(data);
    }
}

bool GenericTODSender::in_sending_control_mode() {
    return std::find(_sending_control_modes.begin(), _sending_control_modes.end(), _control_mode) != _sending_control_modes.end();
}
