// Copyright 2021 Schimpe
#include "tod_core/param_set/CameraParameters.hpp"

#include <string>
#include <vector>
namespace tod_core
{
namespace param_set
{
Camera::Camera(rclcpp::Node * node_ptr, const std::string & config_path)
: ParameterHandler(node_ptr, "vehicleID"), _desiredPath(config_path)
{
}
bool Camera::load_parameters()
{
  YamlLoader loader;
  if (!loader.load_from_path(
        get_path_to_config_files() + get_current_id() + "/" + get_yaml_file())) {
    return false;
  }

  if (loader.has_node("camera_topics_namespace"))
    _camera_topics_namespace = loader.get_param<std::string>("camera_topics_namespace");

  if (loader.has_node("camera_image_name"))
    _camera_image_name = loader.get_param<std::string>("camera_image_name");

  _sensors.clear();
  for (int i = 0; i < 20; ++i) {
    std::string ns = std::string("camera" + std::to_string(i));
    if (!loader.has_node(ns)) continue;

    std::string name = loader.get_param<std::string>(ns, "name");

    CameraSensor & sensor = _sensors.emplace_back(name);
    if (loader.has_node(ns, "is_front_facing"))
      sensor.is_front_facing = loader.get_param<bool>(ns, "is_front_facing");
    if (loader.has_node(ns, "is_fisheye"))
      sensor.is_fisheye = loader.get_param<bool>(ns, "is_fisheye");
    if (loader.has_node(ns, "stream_on_connect"))
      sensor.stream_on_connect = loader.get_param<bool>(ns, "stream_on_connect");
    if (loader.has_node(ns, "project_on"))
      sensor.project_on = loader.get_param<bool>(ns, "project_on");
    if (loader.has_node(ns, "is_jpeg")) sensor.is_jpeg = loader.get_param<bool>(ns, "is_jpeg");
    if (loader.has_node(ns, "scalings"))
      sensor.scalings = loader.get_param<std::vector<std::string>>(ns, "scalings");
    if (loader.has_node(ns, "transition_bitrates"))
      sensor.transition_bitrates = loader.get_param<std::vector<int>>(ns, "transition_bitrates");
    if (sensor.scalings.size() != sensor.transition_bitrates.size()) {
      std::cout << sensor.name << ": has different number of scalings (" << sensor.scalings.size()
                << ") and transition bitrates (" << sensor.transition_bitrates.size() << ")"
                << std::endl;
    }
  }

  return true;
}
}  // namespace param_set
}  // namespace tod_core
