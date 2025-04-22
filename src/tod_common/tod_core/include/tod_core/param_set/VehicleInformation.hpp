// Copyright 2021 Hoffmann
#ifndef TOD_CORE__PARAM_SET__VEHICLEINFORMATION_HPP_
#define TOD_CORE__PARAM_SET__VEHICLEINFORMATION_HPP_
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tod_core/YamlLoader.hpp"
#include "tod_core/param_set/ParameterHandler.hpp"
namespace tod_core
{
namespace param_set
{
class VehicleInformation : public ParameterHandler
{
public:
  VehicleInformation(rclcpp::Node * node_ptr, const std::string& config_path);
  bool load_parameters() override;

  std::string get_manufacturer() const;
  std::string get_type() const;
  float get_id() const;
  std::string get_yaml_file() override { return "vehicle-info.yaml"; }
  std::string get_path_to_config_files() override { return _desiredPath; }

private:
  std::string _desiredPath{""};
  std::string _manufacturer;
  std::string _type;
  int _id;
};
}  // namespace param_set
}  // namespace tod_core
#endif  // TOD_CORE__PARAM_SET__VEHICLEINFORMATION_HPP_
