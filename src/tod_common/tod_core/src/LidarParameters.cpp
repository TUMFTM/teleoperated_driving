// Copyright 2021 Schimpe
#include "tod_core/param_set/LidarParameters.hpp"

#include <string>

namespace tod_core::param_set {

Lidar::Lidar(rclcpp::Node * node_ptr, const std::string & config_path)
: ParameterHandler(node_ptr, "vehicleID"), _desiredPath(config_path){}

bool Lidar::load_parameters()
{
  YamlLoader loader;
  if (!loader.load_from_path(
        get_path_to_config_files() + get_current_id() + "/" + get_yaml_file())) {
    return false;
  }

  _lidar_topics_namespace = loader.get_param<std::string>("lidar_topics_namespace");
  _pointcloud_name = loader.get_param<std::string>("pointcloud_name");

  _sensors.clear();
  for (int i = 0; i < 20; ++i) {
    std::string ns = std::string("lidar" + std::to_string(i));
    if (!loader.has_node(ns)) {
      continue;
    }
    std::string name = loader.get_param<std::string>(ns, "name");
    _sensors.emplace_back(name);
  }

  return true;
}

} // namespace tod_core::param_set