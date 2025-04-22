// Copyright 2021 Hoffmann
#include "tod_core/param_set/TransformParameters.hpp"

#include <string>
#include <vector>
namespace tod_core
{
namespace param_set
{
Transform::Transform(rclcpp::Node * node_ptr, const std::string & config_path)
: ParameterHandler(node_ptr, "vehicleID"), _desiredPath(config_path)
{
}
bool Transform::load_parameters()
{
  YamlLoader loader;
  if (!loader.load_from_path(
        get_path_to_config_files() + get_current_id() + "/" + get_yaml_file())) {
    return false;
  }

  _transforms.clear();
  int i = 1;
  while (loader.has_node(std::string("Transform" + std::to_string(i)))) {
    std::string ns = std::string("Transform" + std::to_string(i));
    geometry_msgs::msg::TransformStamped transformStamped;
    // stamp and frame_id
    transformStamped.header.stamp = get_parent()->now();
    transformStamped.header.frame_id = loader.get_param<std::string>(ns, "from");
    transformStamped.child_frame_id = loader.get_param<std::string>(ns, "to");
    // translation
    std::vector<double> translation_x_y_z;
    translation_x_y_z = loader.get_param<std::vector<double>>(ns, "translation_x_y_z");
    transformStamped.transform.translation.x = translation_x_y_z.at(0);
    transformStamped.transform.translation.y = translation_x_y_z.at(1);
    transformStamped.transform.translation.z = translation_x_y_z.at(2);
    // rotation
    std::vector<double> rot_ypr;
    rot_ypr = loader.get_param<std::vector<double>>(ns, "rotation_yaw_pitch_roll");
    tf2::Quaternion quat = tf2::Quaternion();
    quat.setRPY(rot_ypr.at(2), rot_ypr.at(1), rot_ypr.at(0));
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    // pushback
    _transforms.push_back(transformStamped);
    i++;
  }

  return true;
}
std::vector<geometry_msgs::msg::TransformStamped> Transform::get_transforms()
{
  return _transforms;
}
void Transform::updateStamp()
{
  for (auto & transform : _transforms) {
    transform.header.stamp = get_parent()->now() + rclcpp::Duration::from_seconds(1.0);
  }
}
}  // namespace param_set
}  // namespace tod_core
