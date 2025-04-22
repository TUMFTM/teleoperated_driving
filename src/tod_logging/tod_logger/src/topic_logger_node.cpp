/**
 * @file topic_logger_node.cpp
 * @brief this file depicts the main file of the ros node
 * @copyright TUM-FTM
 */

#include "tod_logger/topic_logger.hpp"

int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tod_logger::TopicLoggerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}