// Copyright 2021 Hoffmann
#ifndef TOD_HELPER__ROS__HELPERS_H_
#define TOD_HELPER__ROS__HELPERS_H_
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tod_helper::ROS
{
inline auto createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

// Callback for node parameter handling: Only sets variable if parameter value is between given min and max.
template<typename T> void cb_param_range_check(const rclcpp::Parameter & p, T *param_var_ptr, T min, T max, rclcpp::Node *node_ptr)
{
    if (min < p.get_value<T>() && p.get_value<T>() < max)
    {
        *param_var_ptr = p.get_value<T>();
        RCLCPP_DEBUG_STREAM(node_ptr->get_logger(), "Parameter " << p.get_name() << " set to " << *param_var_ptr); 
    }
    else
    {
        RCLCPP_WARN_STREAM(node_ptr->get_logger(), "Parameter " << p.get_name() << " not set. Should be between " << min << " and " << max << " !");
    }
}

// Callback for node parameter handling: Passes parameter value directly to variable.
template<typename T> void cb_param_set(const rclcpp::Parameter & p, T *param_var_ptr, rclcpp::Node *node_ptr)
{
    *param_var_ptr = p.get_value<T>();
    RCLCPP_DEBUG_STREAM(node_ptr->get_logger(), "Parameter " << p.get_name() << " set to " << *param_var_ptr);

}



inline std::vector<std::string> get_topic_list(const std::string& msg_type, const rclcpp::Node::SharedPtr& node) {
    std::vector<std::string> topicList;

    // Get the list of topics from the ROS2 graph
    auto topics = node->get_topic_names_and_types();

    // Iterate over the topics and check their types
    for (const auto& topic : topics) {
        auto const& topic_name = topic.first;
        auto const& topic_types = topic.second;

        // If one of the types of the topic matches the requested type, add to the list
        if (std::find(topic_types.begin(), topic_types.end(), msg_type) != topic_types.end()) {
            topicList.push_back(topic_name);
        }
    }

    return topicList;
}

} ; // namespace tod_helper



#endif  // TOD_HELPER__ROS__HELPERS_H_
