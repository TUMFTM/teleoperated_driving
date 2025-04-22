// Copyright 2021 Schimpe
#ifndef TOD_CORE__PARAM_SET__CAMERAPARAMETERS_HPP_
#define TOD_CORE__PARAM_SET__CAMERAPARAMETERS_HPP_

#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tod_core/YamlLoader.hpp"
#include "tod_core/param_set/ParameterHandler.hpp"
namespace tod_core
{
namespace param_set
{
class Camera : public ParameterHandler
{
public:
  struct CameraSensor
  {
    std::string name{""};
    bool is_front_facing{false};
    bool is_fisheye{false};
    bool project_on{false};
    bool stream_on_connect{true};
    bool is_jpeg{false};
    std::vector<std::string> scalings{"1p000"};
    std::vector<int> transition_bitrates{10000};
    CameraSensor(const std::string& camName): name{camName} { };
  };

  Camera(rclcpp::Node * node_ptr, const std::string& config_path);
  bool load_parameters() override;
  std::string get_camera_topics_namespace() const { return _camera_topics_namespace; }
  std::string get_camera_image_name() const { return _camera_image_name; }
  const std::vector<CameraSensor> & get_sensors() const { return _sensors; }
  std::string get_yaml_file() override { return "sensors-camera.yaml"; }
  std::string get_path_to_config_files() override { return _desiredPath; }

private:
  std::string _desiredPath{""};
  std::string _camera_topics_namespace{"/Vehicle/Video"};
  std::string _camera_image_name{"/image_raw"};
  std::vector<CameraSensor> _sensors;
};
}  // namespace param_set
}  // namespace tod_core
#endif  // TOD_CORE__PARAM_SET__CAMERAPARAMETERS_HPP_
