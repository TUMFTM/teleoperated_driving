// Copyright 2021 Hoffmann
#include "tod_core/param_set/VehicleParameters.hpp"
namespace tod_core
{
namespace param_set
{
Vehicle::Vehicle(rclcpp::Node * node_ptr, const std::string & config_path)
: ParameterHandler(node_ptr, "vehicleID"), _desiredPath(config_path)
{
}
bool Vehicle::load_parameters()
{
  YamlLoader loader;
  if (!loader.load_from_path(
        get_path_to_config_files() + get_current_id() + "/" + get_yaml_file())) {
    return false;
  }
  _mass = loader.get_param<float>("mass");
  _yaw_inertia = loader.get_param<float>("yaw_inertia");
  _distance_front_axle = loader.get_param<float>("distance_front_axle");
  _distance_rear_axle = loader.get_param<float>("distance_rear_axle");
  _width_edge_to_edge = loader.get_param<float>("width_edge_to_edge");
  _track_width = loader.get_param<float>("track_width");
  _cornering_force_front = loader.get_param<float>("cornering_force_front");
  _cornering_force_rear = loader.get_param<float>("cornering_force_rear");
  _maximum_road_wheel_angle = loader.get_param<float>("maximum_road_wheel_angle");
  _maximum_steering_wheel_angle = loader.get_param<float>("maximum_steering_wheel_angle");
  _height = loader.get_param<float>("height");
  _distance_front_bumper = loader.get_param<float>("distance_front_bumper");
  _distance_rear_bumper = loader.get_param<float>("distance_rear_bumper");

  return true;
}
float Vehicle::get_mass() const { return _mass; }
float Vehicle::get_wheel_base() const { return _distance_front_axle + _distance_rear_axle; }
float Vehicle::get_yaw_inertia() const { return _yaw_inertia; }
float Vehicle::get_distance_front_axle() const { return _distance_front_axle; }
float Vehicle::get_distance_rear_axle() const { return _distance_rear_axle; }
float Vehicle::get_distance_front_bumper() const { return _distance_front_bumper; }
float Vehicle::get_distance_rear_bumper() const { return _distance_rear_bumper; }
float Vehicle::get_width() const { return _width_edge_to_edge; }
float Vehicle::get_height() const { return _height; }
float Vehicle::get_track_width() const { return _track_width; }
float Vehicle::get_cornering_force_front() const { return _cornering_force_front; }
float Vehicle::get_cornering_force_rear() const { return _cornering_force_rear; }
float Vehicle::get_max_rwa_deg() const { return _maximum_road_wheel_angle * 180.0 / M_PI; }
float Vehicle::get_max_swa_deg() const { return _maximum_steering_wheel_angle * 180 / M_PI; }
float Vehicle::get_max_rwa_rad() const { return _maximum_road_wheel_angle; }
float Vehicle::get_max_swa_rad() const { return _maximum_steering_wheel_angle; }
// TODO(Simon): compute on client side?!Vehicle .. to remove vehicle dependency from core
/* float Vehicle::compute_swa_from(const float rwa) const {
    return tod_helper::Vehicle::Model::rwa2swa(rwa, _maximum_steering_wheel_angle,
        _maximum_road_wheel_angle);
}

float Vehicle::compute_rwa_from(const float swa) const {
    return tod_helper::Vehicle::Model::swa2rwa(swa, _maximum_steering_wheel_angle,
        _maximum_road_wheel_angle);
}  */

}  // namespace param_set
}  // namespace tod_core
