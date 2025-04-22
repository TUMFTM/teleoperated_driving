/**
 * @file safety_gate.hpp
 * @brief ROS2 node for a safety gate
 * @copyright 2025 TUM-FTM
 */

#include "rclcpp/rclcpp.hpp"

#include <tod_status_msgs/msg/status.hpp>
#include <tod_topic_monitoring_msgs/msg/topic_state.hpp>
#include <tod_vehicle_msgs/msg/primary_control_cmd.hpp>
#include <tod_vehicle_msgs/msg/secondary_control_cmd.hpp>

namespace tod_safety_gate {
/**
 * @brief Safety Gate Node
 *
 * This class provides a safety gate that pipes through or blocks the control commands going to the vehicle actuation
 * depending on the status of the system monitored by the tod_monitoring moduls.
 * 
 * @ingroup tod_safety_gate
 */
class SafetyGateNode : public rclcpp::Node {
  public:
    /**
     * @brief Construct a new Safety Gate Node object
     */
    SafetyGateNode();
    ~SafetyGateNode();

  private:
    rclcpp::Subscription<tod_vehicle_msgs::msg::PrimaryControlCmd>::SharedPtr primary_control_cmd_subscriber_;
    rclcpp::Subscription<tod_vehicle_msgs::msg::SecondaryControlCmd>::SharedPtr secondary_control_cmd_subscriber_;
    rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr status_subscriber_;
    rclcpp::Subscription<tod_topic_monitoring_msgs::msg::TopicState>::SharedPtr topic_state_subscriber_;
    rclcpp::Publisher<tod_vehicle_msgs::msg::PrimaryControlCmd>::SharedPtr primary_control_cmd_publisher_;
    rclcpp::Publisher<tod_vehicle_msgs::msg::SecondaryControlCmd>::SharedPtr secondary_control_cmd_publisher_;

    float warning_velocity_{2.7778};

    uint8_t tod_status_{tod_status_msgs::msg::Status::TOD_STATUS_IDLE};
    uint8_t topic_state_{tod_topic_monitoring_msgs::msg::TopicState::STATE_NOT_RECEIVED};

    void primary_control_command_callback(const tod_vehicle_msgs::msg::PrimaryControlCmd::SharedPtr msg);
    void secondary_control_command_callback(const tod_vehicle_msgs::msg::SecondaryControlCmd::SharedPtr msg);
    void status_callback(const tod_status_msgs::msg::Status::SharedPtr msg);
    void topic_state_callback(const tod_topic_monitoring_msgs::msg::TopicState::SharedPtr msg);
};

}  // namespace tod_safety_gate
