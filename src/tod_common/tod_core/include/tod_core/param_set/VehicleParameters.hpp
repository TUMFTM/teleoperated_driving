// Copyright 2021 Hoffmann
#ifndef TOD_CORE__PARAM_SET__VEHICLEPARAMETERS_HPP_
#define TOD_CORE__PARAM_SET__VEHICLEPARAMETERS_HPP_

#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tod_core/YamlLoader.hpp"
#include "tod_core/param_set/ParameterHandler.hpp"
namespace tod_core
{
namespace param_set
{
class Vehicle : public ParameterHandler
{
public:
  Vehicle(rclcpp::Node * node_ptr, const std::string& config_path);
  bool load_parameters() override;

  float get_mass() const;
  float get_yaw_inertia() const;
  float get_distance_front_axle() const;
  float get_distance_rear_axle() const;
  float get_distance_front_bumper() const;
  float get_distance_rear_bumper() const;
  float get_width() const;
  float get_height() const;
  float get_track_width() const;
  float get_cornering_force_front() const;
  float get_cornering_force_rear() const;
  float get_max_rwa_deg() const;
  float get_max_swa_deg() const;
  float get_max_rwa_rad() const;
  float get_max_swa_rad() const;
  float get_wheel_base() const;
  std::string get_yaml_file() override { return "vehicle-params.yaml"; }
  std::string get_path_to_config_files() override { return _desiredPath; }

private:
  std::string _desiredPath{""};
  float _mass{0};                   // Vehicle mass in kg
  float _yaw_inertia{0};            // Yaw inertia of the vehicle in kilogram*meter^2
  float _distance_front_axle{0};    // Distance between CoM and front axle in meters
  float _distance_rear_axle{0};     // Distance between CoM and front axle in meters
  float _width_edge_to_edge{0};     // Width from tips of side-mirrors in meters
  float _cornering_force_front{0};  // Front cornering force in Newton
  float _cornering_force_rear{0};   // Rear cornering force in Newton
  float _maximum_road_wheel_angle{0};
  float _maximum_steering_wheel_angle{0};
  float _height{0};
  float _track_width{0};
  float _distance_front_bumper{0};
  float _distance_rear_bumper{0};
};
}  // namespace param_set
}  // namespace tod_core
#endif  // TOD_CORE__PARAM_SET__VEHICLEPARAMETERS_HPP_
