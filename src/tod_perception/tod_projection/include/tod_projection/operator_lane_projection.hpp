/*** 
 * @file operator_lane_projection.hpp
 * @brief Subscribes to vehicle data containing the current steering_wheel_angle and publishes a set of path messages that show the future path of the vehicle if steering is kept constant.
 * @copyright 2025 TUM-FTM
 */

#pragma once

#include "VehicleEnums.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tod_core/param_set/VehicleParameters.hpp"
#include "tod_helper/vehicle/Model.h"
#include "tod_vehicle_msgs/msg/primary_vehicle_state.hpp"
#include "tod_vehicle_msgs/msg/secondary_vehicle_state.hpp"
#include <memory.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace tod_projection {

/**
 * @class OperatorLaneProjection
 * @brief Node that publishes the future lane of the vehicle based on the steering angle
 */
class OperatorLaneProjection : public rclcpp::Node {
public:
  /**
   * @brief Constructor that prepares the ROS node to process incoming vehicle feedback
   * as well as publishing the lanes. It also sets up the vehicle parameters needed for 
   * computing the lanes (i.e. vehicle width) 
   */
  OperatorLaneProjection();

private:
  /**
   * @brief Subscriber to PrimaryVehicleState message.
   *
   * This subscriber is used to determine the vehicles steering angle and calls the 
   * publishers for the vehicle lanes 
   */
  rclcpp::Subscription<tod_vehicle_msgs::msg::PrimaryVehicleState>::SharedPtr
      _subscriber_primary_vehicle_state;          
  
  /**
   * @brief Subscriber to SecondaryVehicleState message.
   * 
   * This subscriber is used to determine the forward or backward driving
   * direction based on the vehicle's gear position.
   */
  rclcpp::Subscription<tod_vehicle_msgs::msg::SecondaryVehicleState>::SharedPtr
      _subscriber_secondary_vehicle_state;        

  /// Publisher for front left vehicle lane projection
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _publisher_vehicle_lane_fl;
  /// Publisher for front right vehicle lane projection
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _publisher_vehicle_lane_fr;
  /// Publisher for rear left vehicle lane projection
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _publisher_vehicle_lane_rl;
  /// Publisher for rear right vehicle lane projection
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _publisher_vehicle_lane_rr;

  /**
   * @brief Pointer to the vehicle parameters object.
   * 
   * This object loads and manages vehicle-specific parameters from a configuration
   * file located in the "tod_vehicle_config" package. The parameters are initialized
   * using the desired path constructed during node setup.
   */
  std::unique_ptr<tod_core::param_set::Vehicle> _vehicle_params;
  /// Default parameter for the vehicleID 
  std::string _vehicle_id{"edgar"};
  /// Variable to hold the current gear position
  std::int8_t _gear_position;

  /**
   * @brief Callback to calculate and publish vehicle lane projections based on primary vehicle state.
   * 
   * @param msg The primary vehicle state message.
   */
  void callback_primary_vehicle_state(const tod_vehicle_msgs::msg::PrimaryVehicleState &msg);
  
  /**
  * @brief Callback to update the attribute gear_position based on the secondary vehicle state.
  * 
  * @param msg The secondary vehicle state message.
  */
  void callback_secondary_vehicle_state(const tod_vehicle_msgs::msg::SecondaryVehicleState &msg);
};

} // namespace tod_projection