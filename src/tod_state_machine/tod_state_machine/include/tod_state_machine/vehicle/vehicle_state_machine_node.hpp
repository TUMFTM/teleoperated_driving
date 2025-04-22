/**
 * @file vehicle_state_machine_node.hpp
 * @brief this file declares methods and variables for the ROS2-Node
 * @copyright TUM-FTM
 */
#include <rclcpp/rclcpp.hpp>

#include "tod_status_msgs/msg/status.hpp"
#include "tod_vehicle_msgs/msg/safety_driver_status.hpp"

#include "tod_state_machine/vehicle/vehicle_state_machine.hpp"

namespace tod_vehicle_state_machine {

class VehicleStateMachineNode : public rclcpp::Node
{

    public:
        VehicleStateMachineNode();

    private:
        std::mutex mutex_;

        rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr sub_operator_status_;
        rclcpp::Subscription<tod_vehicle_msgs::msg::SafetyDriverStatus>::SharedPtr sub_safety_driver_status_;
        rclcpp::Publisher<tod_status_msgs::msg::Status>::SharedPtr pub_vehicle_status_;

        /**
         * @brief Callback function to fill operator's part of the status message
         * @param msg Shared pointer to the Status message
         * @return void This function does not return a value
         */
        void callback_operator_status(const tod_status_msgs::msg::Status::SharedPtr msg);
        
        /**
         * @brief Callback function to status message with safety driver status
         * @param msg Shared pointer to the SafetyDriverStatus message
         * @return void This function does not return a value
         */
        void callback_safety_driver_status(const tod_vehicle_msgs::msg::SafetyDriverStatus::SharedPtr msg);
        
        /**
         * @brief function to initialize vehicle's part of the status message
         * @param None This function does not take parameter
         * @return void This function does not return a value
         */
        void set_vehicle_init_status();

        VehicleStateMachine vsm_;
        tod_status_msgs::msg::Status tod_vehicle_status_;
        rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace tod_vehicle_state_machine