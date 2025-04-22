// Copyright 2021 Hoffmann
#ifndef TOD_CORE__PARAM_SET__TRANSFORMPARAMETERS_HPP_
#define TOD_CORE__PARAM_SET__TRANSFORMPARAMETERS_HPP_

#include <iostream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tod_core/YamlLoader.hpp"
#include "tod_core/param_set/ParameterHandler.hpp"
namespace tod_core
{
namespace param_set
{
class Transform : public ParameterHandler
{
public:
  Transform(rclcpp::Node * node_ptr, const std::string& config_path);
  bool load_parameters() override;
  std::vector<geometry_msgs::msg::TransformStamped> get_transforms();
  void updateStamp();
  std::string get_yaml_file() override { return "vehicle-transforms.yaml"; }
  std::string get_path_to_config_files() override { return _desiredPath; }

private:
  std::string _desiredPath{""};
  std::vector<geometry_msgs::msg::TransformStamped> _transforms;
};
}  // namespace param_set
}  // namespace tod_core
#endif  // TOD_CORE__PARAM_SET__TRANSFORMPARAMETERS_HPP_
