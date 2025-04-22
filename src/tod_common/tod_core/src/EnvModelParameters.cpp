// Copyright 2024 Kerbl Tobias
#include "tod_core/param_set/EnvModelParameters.hpp"
//#include <stdio.h>
namespace tod_core{
namespace param_set{

EnvModel::EnvModel(rclcpp::Node * node_ptr, const std::string & config_path)
: ParameterHandler(node_ptr, "vehicleID"), _desiredPath(config_path){ }

bool EnvModel::load_parameters()
{
  YamlLoader loader;
  if (!loader.load_from_path(
        get_path_to_config_files() + get_current_id() + "/" + get_yaml_file())) 
  {
    return false;
  }

  _env_model.clear();
  for (int i = 0; i < 20; ++i) 
  {
    std::string ns = std::string("env_model_component" + std::to_string(i));
    //std::cout << "\n Name Space:" + ns + "\n";
    //std::cout << "\n Has Node:" + std::to_string(!loader.has_node(ns)) + "\n";
    
    if (!loader.has_node(ns)) { continue; }

    std::string name = loader.get_param<std::string>(ns, "name");
    EnvModelComponent & env_model_component = _env_model.emplace_back(name);

    env_model_component.input_topic_name = loader.get_param<std::string>(ns, "input_topic_name");
    env_model_component.output_topic_name = loader.get_param<std::string>(ns, "output_topic_name");
  }

  return true;
}

}  // namespace param_set
}  // namespace tod_core
