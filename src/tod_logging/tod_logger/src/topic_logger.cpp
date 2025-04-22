/**
 * @file topic_logger.cpp
 * @brief this file depicts the logic of the logging mechanism
 * @copyright TUM-FTM
 */

#include "tod_logger/topic_logger.hpp"

using std::placeholders::_1;
using namespace tod_logger;


TopicLoggerNode::TopicLoggerNode() : Node("topic_logger_node")
{
    // delay to fully discover all available topics with the method below
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto t_ = std::time(nullptr);
    auto tm_ = *std::localtime(&t_);

    parameter_initializer();

    filepath_ << log_path_<< std::put_time(&tm_, "%Y-%m-%d_%H-%M-%S") << logfile_suffix_ << ".yaml";

    create_subscription_for_topic();
}


/**
 * @brief this method filters topics with a given namespace within all available topics in the ROS domain and provides all relevant topics in a unordered map.
 * @param None
 * @return None
 */
void TopicLoggerNode::create_subscription_for_topic()
{
    auto available_topics_ = this->get_topic_names_and_types();

    for (const auto& topic_i : available_topics_)
    {
        const auto& topic_name = topic_i.first;

        if (is_valid_topic_for_mode(topic_name))
        {
            auto subscription = this->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
                topic_name,
                1,
                std::bind(&TopicLoggerNode::topic_logger, this, std::placeholders::_1));

            subscriptions_[topic_name] = subscription;
            RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", topic_name.c_str());
        }
    }

    for (const auto& subscription : subscriptions_) {
        RCLCPP_INFO(this->get_logger(), "Subscribed 2 topic: %s", subscription.first.c_str());
    }

   
}

/**
 * @brief Checks if a given topic name is valid for the current logging mode.
 * @param topic_name The name of the topic to be validated.
 * @return `true` if the topic is valid for the current logging mode, `false` otherwise.
 */
bool TopicLoggerNode::is_valid_topic_for_mode(const std::string &topic_name)
{
    if (log_mode_ == "operator")
    {
        return topic_name.find("/operator/debug") == 0 || topic_name.find("/operator/logging") == 0;
    }
    else if (log_mode_ == "vehicle")
    {
        return topic_name.find("/vehicle/debug") == 0 || topic_name.find("/vehicle/logging") == 0;
    }
    else if (log_mode_ == "both")
    {
        return topic_name.find("/vehicle/debug") == 0 || topic_name.find("/vehicle/logging") == 0 ||
               topic_name.find("/operator/debug") == 0 || topic_name.find("/operator/logging") == 0;
    }
    return false;
}

/**
 * @brief Logs the contents of a ROS2 DiagnosticStatus message to a YAML file.
 * @param[in] msg Shared pointer to the DiagnosticStatus message to be logged.
 */
void TopicLoggerNode::topic_logger(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg)
{
    log_file_.open(filepath_.str(), std::ios::out | std::ios::app);

    if (!log_file_.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open log file");
        return;
    }

    yamlmsg_logger(msg, yaml_msg_, log_file_);
    log_file_.close();

}

/**
 * @brief This method fills diagnostics data to a YAML-Node
 * @param[in] msg DiagnosticStatus-message from topic subscription of type diagnostic_msgs::msg
 * @param[in] yaml_msg_ reference of declared YAML::Node 
 * @param[in] log_file_ reference of declared std::ofstream instance 
 */
void TopicLoggerNode::yamlmsg_logger(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg, YAML::Node &yaml_msg_, std::ofstream &log_file_)
{
    YAML::Node values_node;
    yaml_msg_["level"] = msg->level;
    yaml_msg_["name"] = msg->name;
    yaml_msg_["message"] = msg->message;
    yaml_msg_["hardware_id"] = msg->hardware_id;

    for (const auto &kv : msg->values)
    {
        values_node[kv.key] = kv.value;
    }
    yaml_msg_["values"] = values_node;
    log_file_ << YAML::Dump(yaml_msg_) << "\n";
}

/**
 * @brief This method declares and initializes launch parameters
 * @param None This function does not take parameters
 * @return void This function does not return a value
 */
void TopicLoggerNode::parameter_initializer()
{
    this->declare_parameter("logfile_suffix", "_tod_log");
    this->declare_parameter("logger_namespace", "both");
    this->declare_parameter("log_path", "./");

    this->get_parameter("logger_namespace", log_mode_);
    this->get_parameter("logfile_suffix", logfile_suffix_);
    this->get_parameter("log_path", log_path_);
    log_mode_ = is_valid_mode(log_mode_) ? log_mode_ : "both";
    
}

/**
 * @brief This checks the allowed parameter space for the logging mode
 * @param None This function does not take parameters
 * @return bool This function returns true for valid parameters given, false otherwise
 */
bool TopicLoggerNode::is_valid_mode(const std::string &mode)
{
    if (allowed_modes_.find(mode) == allowed_modes_.end())
    {
        RCLCPP_WARN(this->get_logger(), "Invalid logging mode '%s'. Falling back to default mode 'both'. Allowed modes: 'vehicle', 'operator', 'both'.", mode.c_str());
        return 0;
    }
    return 1;
}

