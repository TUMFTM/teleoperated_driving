// Copyright 2021 Hoffmann
#include "tod_core/param_set/VehicleInformation.hpp"

#include <string>
namespace tod_core
{
namespace param_set
{
VehicleInformation::VehicleInformation(rclcpp::Node * node_ptr, const std::string & config_path)
: ParameterHandler(node_ptr, "vehicleID"), _desiredPath(config_path)
{
}
bool VehicleInformation::load_parameters()
{
  YamlLoader loader;
  if (!loader.load_from_path(
        get_path_to_config_files() + get_current_id() + "/" + get_yaml_file())) {
    return false;
  }
  _manufacturer = loader.get_param<std::string>("manufacturer");
  _type = loader.get_param<std::string>("type");
  _id = loader.get_param<int>("id");

  return true;
}
std::string VehicleInformation::get_manufacturer() const { return _manufacturer; }
std::string VehicleInformation::get_type() const { return _type; }
float VehicleInformation::get_id() const { return _id; }
}  // namespace param_set
}  // namespace tod_core
