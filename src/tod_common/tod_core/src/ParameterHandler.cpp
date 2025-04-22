// Copyright 2021 Hoffmann
#include "tod_core/param_set/ParameterHandler.hpp"

#include <memory>
#include <string>

using namespace std::chrono_literals;
ParameterHandler::ParameterHandler(rclcpp::Node * node_ptr, const std::string & trigger_param)
: _parentNode(node_ptr), _triggerParam(trigger_param)
{
  // Declare trigger_param if not already done
  if (!_parentNode->has_parameter(trigger_param)) {
    _parentNode->declare_parameter<std::string>(trigger_param, "");
  }

  // Check if default value of trigger_param exists 
  _parentNode->get_parameter(_triggerParam, _id_temp);
  _id = _id_temp.as_string();

  // Check if value of trigger param changes during runtime
  _timer = _parentNode->create_wall_timer(3000ms, std::bind(&ParameterHandler::check_loop, this));
  _param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(_parentNode);
  _cb_handle = _param_subscriber->add_parameter_callback(
    trigger_param, std::bind(&ParameterHandler::cb_change_param, this, std::placeholders::_1));
}

void ParameterHandler::set_id_changed_cb(std::function<void(const std::string&)> callback){
    _cb_id_changed = callback;
}

void ParameterHandler::cb_change_param(const rclcpp::Parameter & p)
{
  RCLCPP_INFO_STREAM(
    _parentNode->get_logger(), "Parameter " << p.get_name() << " has changed to " << p.as_string());
  _id = p.as_string();
  _id_set_correctly = load_parameters();
  if (_cb_id_changed){
    _cb_id_changed(_id);
  }
}

void ParameterHandler::check_loop()
{ 
  
  if(!_id.empty() && !_id_set_correctly)
  {
    _id_set_correctly = load_parameters();
  }
  
  if (!_id_set_correctly) {

    RCLCPP_ERROR_STREAM(
      _parentNode->get_logger(), "Parameter set "
                                   << get_yaml_file() << " not loaded correctly. Check param "
                                   << _triggerParam << " or Path: " 
                                   << get_path_to_config_files() << _id);
  }
}