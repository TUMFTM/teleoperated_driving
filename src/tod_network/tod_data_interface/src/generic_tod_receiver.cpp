/**
 * @file generic_tod_reciever.cpp
 * @author Nils Gehrke
 * @brief This file implements a ROS2 node for receiving and processing messages based on the specified protocol (UDP or TCP).
 * @version 1.0
 * 
 * @copyright TUMFTM 2024
 * 
*/

#include "tod_data_interface/generic_tod_receiver.hpp"

using namespace tod_data_interface;

GenericTODReceiver::GenericTODReceiver(): Node("generic_tod_receiver"){
    // Declare parameters and initialize subscriptions here if needed
    declare_parameter<std::string>("protocol_strategy", "udp");
    declare_parameter<std::string>("topic", "/message_received");
    declare_parameter<std::string>("topic_type", "std_msgs/msg/String");
    declare_parameter<std::string>("status_topic", "input/operator_status");
    declare_parameter<int>("port", 5000);

    auto protocol_strategy = get_parameter("protocol_strategy").as_string();
    this->_topic = get_parameter("topic").as_string();
    this->_topic_type = get_parameter("topic_type").as_string();
    this->_destination_port = get_parameter("port").as_int();
    std::string status_topic = get_parameter("status_topic").as_string();

    set_protocol_strategy(protocol_strategy);

    // Create generic subscription using the topic namespace
    this->_recv_msg_publisher = this->create_generic_publisher(
        this->_topic,
        this->_topic_type,
        1);
    
    this->_status_subs = this->create_subscription<tod_status_msgs::msg::Status>(
      status_topic, 
      1, 
      std::bind(&GenericTODReceiver::status_message_received, this, std::placeholders::_1)
    );
}

GenericTODReceiver::~GenericTODReceiver(){
    if (_receive_loop && _receive_loop->joinable()) {
        _receive_loop->join();
    }
}

void GenericTODReceiver::receive(){
    this->_receive_loop = std::make_unique<std::thread>(
        std::bind(&GenericTODReceiver::message_receive, this)
    );
}

void GenericTODReceiver::status_message_received(const tod_status_msgs::msg::Status &msg) {
    _connection_status = msg.tod_status;
}

void GenericTODReceiver::message_receive(){       
    this->_receiver->waiting_for_client_connect();
    while (rclcpp::ok()) {
        std::vector<uint8_t> data = this->_receiver->receive();
        // create serialized message and copy buffer into message
        rclcpp::SerializedMessage serialized_msg(data.size());
        std::copy(data.begin(), data.end(), serialized_msg.get_rcl_serialized_message().buffer);
        serialized_msg.get_rcl_serialized_message().buffer_length = data.size();
        this->_recv_msg_publisher->publish(serialized_msg);
    }
    this->_receiver->disconnect();
}

void GenericTODReceiver::set_protocol_strategy(const std::string &protocol_strategy) {
    if (protocol_strategy == "udp") {
        this->_receiver = std::make_unique<tod_network_protocols::UdpReceiver>(this->_destination_port);
    } else if (protocol_strategy == "tcp") {
        // Example: Replace with actual TCP sender initialization if available
        this->_receiver = std::make_unique<tod_network_protocols::TcpReceiver>(this->_destination_port);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported protocol strategy: %s", protocol_strategy.c_str());
        throw std::invalid_argument("Unsupported protocol strategy");
    }
}