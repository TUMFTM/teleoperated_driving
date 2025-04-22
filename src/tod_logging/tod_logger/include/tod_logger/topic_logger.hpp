/**
 * @file topic_logger.hpp
 * @brief this file depicts header file for the logging mechanism
 * @copyright TUM-FTM
 */

#pragma once

#include <memory>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include "yaml-cpp/yaml.h"


namespace tod_logger {
/**
 * @ingroup tod_logger
 * @brief components for log and debug infos 
 */

class TopicLoggerNode : public rclcpp::Node
{
public:
    TopicLoggerNode();

private:
    // Methods
    void create_subscription_for_topic();
    void topic_logger(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg); 
    void yamlmsg_logger(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg, YAML::Node &yaml_msg, std::ofstream &log_file);
    void parameter_initializer();
    bool is_valid_mode(const std::string &mode);
    bool is_valid_topic_for_mode(const std::string &topic_name);

    // Subscriptions
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr subLogging_;
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    
    // variables and parameters
    std::ofstream log_file_;
    std::ostringstream filepath_;
    YAML::Node yaml_msg_;
    std::string log_mode_;
    std::string logfile_suffix_;
    std::string log_path_;
    std::time_t t_;
    std::tm tm_;
    const std::set<std::string> allowed_modes_ {"both", "vehicle", "operator"};
};

} // namespace tod_logger