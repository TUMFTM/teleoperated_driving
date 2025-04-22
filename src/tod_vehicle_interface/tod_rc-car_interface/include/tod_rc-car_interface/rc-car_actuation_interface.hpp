/**
 * @file rc-car_actuation_interface.hpp
 * @brief RC-Car actuation interface.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_rc-car_interface
 */

#pragma once

#include "tod_generic_interface/actuation_interface.hpp"

#include "tod_core/param_set/VehicleParameters.hpp"
#include "tod_vehicle_msgs/VehicleEnums.h"

#include "rclcpp/rclcpp.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace tod_rccar_interface {
/**
 * @defgroup tod_rccar_interface
 * @ingroup tod_rccar_interface
 * @brief Interfaces for the F1TENTH RC-Cars.
 */

/**
 * @brief Actuation interface for the F1TENTH RC-Cars.
 */
class ActuationInterface : public rclcpp::Node
{
    public:
        ActuationInterface(); //ros2 parameter: steering2wheel
        void run(); // initialize Gear in Drive (because there is no input for this data!)
    private:
        void steering_wheel_handler(const std_msgs::msg::Float64 &msg);
        void engine_speed_handler(const std_msgs::msg::Float64& msg);
        void acceleration_handler(const sensor_msgs::msg::Imu &msg);
        ackermann_msgs::msg::AckermannDriveStamped ackermann_msg_builder();

        // rc car feedback -delta_max = +0.1500
        //                       0.00 = +0.4325
        //                 +delta_max = +0.7385
        // desired         -delta_max = -0.3400
        //                       0.00 = +0.0000
        //                 +delta_max = +0.3400
        float servo_min_ = 0.15;
        float servo_max_ = 0.4325;
        float servo_zero_ = 0.7385;
        std::shared_ptr<tod_generic_interface::ActuationInterface> generic_actuation_interface_;
        std::shared_ptr<tod_core::param_set::Vehicle> vehicle_params_;
};

} // namespace tod_rccar_interface