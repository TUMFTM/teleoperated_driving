/**
 * @file dynamic_transform_publisher_node.cpp
 * @author Feiler
 * @copyright TUMFTM 2020
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("DynamicTransformPublisher");
    tf2_ros::TransformBroadcaster broadcaster(node);
    auto subscriber = node->create_subscription<nav_msgs::msg::Odometry>(
                "input/odom", 1, [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO_ONCE(node->get_logger(), "%s: Broadcasting odometry transform from %s to %s",
                         node->get_name(), msg->header.frame_id.c_str(), msg->child_frame_id.c_str());

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = node->now();
        tf.header.frame_id = msg->header.frame_id;
        tf.child_frame_id = msg->child_frame_id;
        tf.transform.translation.x = msg->pose.pose.position.x;
        tf.transform.translation.y = msg->pose.pose.position.y;
        tf.transform.translation.z = msg->pose.pose.position.z;
        tf.transform.rotation = msg->pose.pose.orientation;
        broadcaster.sendTransform(tf);
    });
    rclcpp::spin(node);
    return 0;
}
