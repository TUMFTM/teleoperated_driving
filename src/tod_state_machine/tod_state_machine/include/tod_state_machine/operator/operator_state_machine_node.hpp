/**
 * @file state_machine_node.hpp
 * @brief this file declares methods and variables for the ROS2-Node
 * @copyright TUM-FTM
 */
#include <rclcpp/rclcpp.hpp>

#include "tod_status_msgs/msg/status.hpp"
#include "tod_status_msgs/msg/manager_button_status.hpp"

#include "tod_state_machine/operator/operator_state_machine.hpp"

namespace tod_operator_state_machine {
/**
 * @ingroup tod_state_machine
 * @brief State Machine on operator sider 
 */

class OperatorStateMachineNode : public rclcpp::Node
{
    public:
        OperatorStateMachineNode();

    private:
        std::mutex mutex_;

        // Subscriptions
        rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr sub_operator_status_;
        rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr sub_vehicle_status_;
        rclcpp::Subscription<tod_status_msgs::msg::ManagerButtonStatus>::SharedPtr sub_visual_status_;
        rclcpp::Publisher<tod_status_msgs::msg::Status>::SharedPtr status_publisher_;

        /**
         * @brief Callback function to handle button status updates from the OperatorManager 
         * and manage the operator state machine
         * @param msg Shared pointer to the ManagerButtonStatus message
         * @return void This function does not return a value
         */
        void callback_visual_state(const tod_status_msgs::msg::ManagerButtonStatus::SharedPtr msg);
        
        /**
         * @brief Callback function to fill vehicle's part of the status message
         * @param msg Shared pointer to the Status message
         * @return void This function does not return a value
         */
        void callback_vehicle_state(const tod_status_msgs::msg::Status::SharedPtr msg);
        
        /**
         * @brief function to initialize operator's part of the status message
         * @param None This function does not take parameter
         * @return void This function does not return a value
         */
        void set_operator_init_status();

        StateMachine sm_;
        tod_status_msgs::msg::Status tod_operator_status_;
        rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace tod_operator_state_machine