// Copyright 2024 Kerbl Tobias
#ifndef TOD_CORE__PARAM_SET__ENVMODELPARAMETERS_HPP_
#define TOD_CORE__PARAM_SET__ENVMODELPARAMETERS_HPP_

#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tod_core/YamlLoader.hpp"
#include "tod_core/param_set/ParameterHandler.hpp"

namespace tod_core{
namespace param_set{

class EnvModel : public ParameterHandler
{
  public:
    struct EnvModelComponent
    {
      std::string name{""};
      std::string input_topic_name{""};
      std::string output_topic_name{""};
      explicit EnvModelComponent(const std::string & myName) : name{myName} {}
    };

    EnvModel(rclcpp::Node * node_ptr, const std::string& config_path);

    // Overrides of virtual base class functions
    bool load_parameters() override;
    std::string get_yaml_file() override { return "automation-env_model.yaml"; }
    std::string get_path_to_config_files() override { return _desiredPath; }

    // Environmental model getter functions
    const std::vector<EnvModelComponent> & get_env_model() const { return _env_model; }
    size_t get_number_of_env_model_components() const { return _env_model.size(); }
  
  private:
    std::string _desiredPath{""};
    std::vector<EnvModelComponent> _env_model;
};

}  // namespace param_set
}  // namespace tod_core
#endif  // TOD_CORE__PARAM_SET__ENVMODELPARAMETERS_HPP_
