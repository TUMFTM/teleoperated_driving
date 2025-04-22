/**
 * @file topic_monitoring_node.cpp
 * @brief main file for topic_monitoring_node
 * @copyright 2024 TUMFTM
 */

#include <tod_topic_monitoring/topic_monitor.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tod_topic_monitoring::TopicMonitoringNode>());
    rclcpp::shutdown();
    return 0;
}