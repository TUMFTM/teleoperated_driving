/**
 * @file tod_service_utils.hpp
 * @author Simon Hoffmann
 * @brief This file contains utility functions for serializing and deserializing ROS messages. These functions handle the conversion of ROS service request and response messages into byte vectors and vice versa. This is essential for enabling communication between ROS nodes over TCP in a format that can be transmitted over the network.
 * @version 1.0
 *
 * @copyright TUMFTM 2020
 */
 #pragma once 
#include <vector>
#include <iostream>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>


namespace tod_network{

 
/// @brief Services in ros can be serialized by being separated into request and response msg 
/// @tparam MessageType any ROS message MessageType any ROS message i.e. service response or request
/// @param msg ROS MSG
/// @return vector<uint8_t*> serialized message 
template<typename MessageType>
std::vector<uint8_t> serialize(MessageType& msg) {
    rclcpp::SerializedMessage serMsg;
    rclcpp::Serialization<MessageType> serializer;
    try {
        serializer.serialize_message(&msg, &serMsg);
        uint8_t* data = serMsg.get_rcl_serialized_message().buffer;
        int size =  serMsg.get_rcl_serialized_message().buffer_length;
        std::vector<uint8_t> serialized(data, data+size);
        return serialized;

    } catch (std::exception &ex) {
        // sometimes buffer is incomplete - ignore
        printf("could not serialize (failed on %s)\n", ex.what());
    }
    return std::vector<uint8_t>();
}


/// @brief Services in ros can be serialized by being separated into request and response msg 
/// @tparam MessageType any ROS message i.e. service response or request
/// @param msg ROS MSG Pointer that gets populated with data
/// @return int 1 if successful, 0 if not
template<typename MessageType>
int deserialize(const std::vector<uint8_t>& data, MessageType * msg) {
    // write data from buffer into serialized message
    rclcpp::SerializedMessage serMsg(data.size()+1);
    auto& rcl_handle = serMsg.get_rcl_serialized_message();

    std::memcpy(rcl_handle.buffer, data.data(), data.size());
    rcl_handle.buffer[data.size()] = '\0';
    rcl_handle.buffer_length = static_cast<size_t>(data.size());

    // deserialize
    rclcpp::Serialization<MessageType> serializer;
    try {
        serializer.deserialize_message(&serMsg, msg);
        return 1;
    } catch (std::exception &ex) {
        // sometimes buffer is incomplete - ignore
        std::cerr << "could not deserialize (failed on "<< ex.what() << std::endl;
    }
    return 0;
}

} //namespace tod_network