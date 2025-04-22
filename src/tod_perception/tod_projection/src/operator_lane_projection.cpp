/*** 
 * @file operator_lane_projection.cpp
 * @brief Subscribes to vehicle data containing the current steering_wheel_angle and publishes a set of path messages that show the future path of the vehicle if steering is kept constant.
 * @copyright 2025 TUM-FTM
 */

#include "tod_projection/operator_lane_projection.hpp"

namespace tod_projection {

OperatorLaneProjection::OperatorLaneProjection()
    : Node("OperatorLaneProjection") {

  _subscriber_primary_vehicle_state = this->create_subscription<tod_vehicle_msgs::msg::PrimaryVehicleState>(
      "input/primary_vehicle_state", 5,
      std::bind(&OperatorLaneProjection::callback_primary_vehicle_state, this,
                std::placeholders::_1));
  _subscriber_secondary_vehicle_state = this->create_subscription<tod_vehicle_msgs::msg::SecondaryVehicleState>(
      "input/secondary_vehicle_state", 5,
      std::bind(&OperatorLaneProjection::callback_secondary_vehicle_state, this,
                std::placeholders::_1));
  
  _publisher_vehicle_lane_fl = 
      this->create_publisher<nav_msgs::msg::Path>("output/vehicle_lane_front_left", 5);
  _publisher_vehicle_lane_fr = 
      this->create_publisher<nav_msgs::msg::Path>("output/vehicle_lane_front_right", 5);
  _publisher_vehicle_lane_rl =
      this->create_publisher<nav_msgs::msg::Path>("output/vehicle_lane_rear_left", 5);
  _publisher_vehicle_lane_rr =
      this->create_publisher<nav_msgs::msg::Path>("output/vehicle_lane_rear_right", 5);

  this->declare_parameter<std::string>("config_path", "");
  std::string config_path;
  
  if(! this->get_parameter("config_path", config_path)) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Failed to retrieve 'config_path' parameter. Ensure it is set in the launch file.");
  }
  
  _vehicle_params =
      std::make_unique<tod_core::param_set::Vehicle>(this, config_path + "/vehicle_config/");

  _vehicle_params->load_parameters();

}

void OperatorLaneProjection::callback_primary_vehicle_state(
    const tod_vehicle_msgs::msg::PrimaryVehicleState &msg) {
  // calc and publish vehicle lane as path
  int direction{1};
  nav_msgs::msg::Path laneFrontLeft, laneFrontRight, laneRearLeft,
      laneRearRight;
  laneFrontLeft.header.stamp = laneFrontRight.header.stamp =
      laneRearLeft.header.stamp = laneRearRight.header.stamp =
          this->get_clock()->now();
  laneFrontLeft.header.frame_id = laneFrontRight.header.frame_id =
      laneRearLeft.header.frame_id = laneRearRight.header.frame_id =
          "base_footprint";

  // calc velocity to advance bicycle model in dependence of vehicle width
  const double dtStep_s = 0.050;
  const int nofSteps = 40;
  const double lengthPred_m = 5.0 * _vehicle_params->get_width();
  const double lengthStep_m = lengthPred_m / nofSteps;
  const double vel_mps = lengthStep_m / dtStep_s;

  if (this->_gear_position == eGearPosition::GEARPOSITION_REVERSE) {
    direction = -1;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.z = 0.0;
  pose.header = laneFrontLeft.header;
  pose.pose.orientation.w = 1.0;

  const float rwa = tod_helper::Vehicle::Model::swa2rwa(
      msg.steering_wheel_angle, _vehicle_params->get_max_swa_rad(),
      _vehicle_params->get_max_rwa_rad());
  const double distRear = _vehicle_params->get_distance_rear_bumper();
  const double distFront = _vehicle_params->get_distance_front_bumper();
  const double beta =
      std::atan(distRear * std::tan(rwa) / (distFront + distRear));
  double xCoM{0.0}, yCoM{0.0}, yawCoM{0.0};
  for (int i = 0; i < nofSteps; ++i) {
    // advance CoM position
    xCoM += direction * dtStep_s * vel_mps * std::cos(yawCoM + beta);
    yCoM += direction * dtStep_s * vel_mps * std::sin(yawCoM + beta);
    yawCoM += direction * dtStep_s * vel_mps * std::sin(beta) / distRear;

    double xfl, yfl, xfr, yfr, xrl, yrl, xrr, yrr;
    tod_helper::Vehicle::Model::calc_vehicle_front_edges(
        xCoM, yCoM, yawCoM, distFront, _vehicle_params->get_width(), xfl, yfl,
        xfr, yfr);
    tod_helper::Vehicle::Model::calc_vehicle_rear_edges(
        xCoM, yCoM, yawCoM, distRear, _vehicle_params->get_width(), xrl, yrl,
        xrr, yrr);

    pose.pose.position.x = xfl;
    pose.pose.position.y = yfl;
    laneFrontLeft.poses.push_back(pose);

    pose.pose.position.x = xfr;
    pose.pose.position.y = yfr;
    laneFrontRight.poses.push_back(pose);

    pose.pose.position.x = xrl;
    pose.pose.position.y = yrl;
    laneRearLeft.poses.push_back(pose);

    pose.pose.position.x = xrr;
    pose.pose.position.y = yrr;
    laneRearRight.poses.push_back(pose);
  }

  _publisher_vehicle_lane_fl->publish(laneFrontLeft);
  _publisher_vehicle_lane_fr->publish(laneFrontRight);
  _publisher_vehicle_lane_rl->publish(laneRearLeft);
  _publisher_vehicle_lane_rr->publish(laneRearRight);
  RCLCPP_INFO_ONCE(this->get_logger(),
                   "%s: Published first set of vehicle lanes to ROS!",
                   this->get_name());
}

void OperatorLaneProjection::callback_secondary_vehicle_state(
    const tod_vehicle_msgs::msg::SecondaryVehicleState &msg) {
        this->_gear_position = msg.gear_position;
}

} // namespace tod_projection

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tod_projection::OperatorLaneProjection>());
  rclcpp::shutdown();
  return 0;
}