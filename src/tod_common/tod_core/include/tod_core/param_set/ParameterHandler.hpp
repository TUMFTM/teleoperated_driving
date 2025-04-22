// Copyright 2022 Hoffmann
#ifndef TOD_CORE__PARAM_SET__PARAMETERHANDLER_HPP_
#define TOD_CORE__PARAM_SET__PARAMETERHANDLER_HPP_

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
class ParameterHandler
{
public:
  ParameterHandler(rclcpp::Node * node_ptr, const std::string & trigger_param);
  virtual ~ParameterHandler() { delete _parentNode; }
  ParameterHandler(ParameterHandler && source) noexcept
  : _parentNode(std::exchange(source._parentNode, nullptr)),
    _id(std::move(source._id)),
    _triggerParam(std::move(source._triggerParam)),
    _id_set_correctly(std::move(source._id_set_correctly)),
    _param_subscriber(std::move(source._param_subscriber)),
    _cb_handle(std::move(source._cb_handle)),
    _timer(std::move(source._timer))
  {
  }  // Move Ctor required due to user-defined dtor
  ParameterHandler & operator=(ParameterHandler && source) noexcept
  {
    delete _parentNode;
    _parentNode = std::exchange(source._parentNode, nullptr);  // move and set to nullptr
    _id = std::move(source._id);
    _triggerParam = std::move(source._triggerParam);
    _id_set_correctly = std::move(source._id_set_correctly);
    _param_subscriber = std::move(source._param_subscriber);
    _cb_handle = std::move(source._cb_handle);
    _timer = std::move(source._timer);
    return *this;
  }
  ParameterHandler(const ParameterHandler & source)
  : _parentNode(source._parentNode),  // no real copy here.. should point to same node
    _id(source._id),
    _triggerParam(source._triggerParam),
    _id_set_correctly(source._id_set_correctly),
    _param_subscriber(source._param_subscriber),
    _cb_handle(source._cb_handle),
    _timer(source._timer)
  {
  }  // Copy Ctor required due to user-defined move ctor
  ParameterHandler & operator=(ParameterHandler & source)
  {
    if (this == &source) {
      return source;
    }
    _parentNode = source._parentNode;  // no real copy here.. should point to same node
    _id = source._id;
    _triggerParam = source._triggerParam;
    _id_set_correctly = source._id_set_correctly;
    _param_subscriber = source._param_subscriber;
    _cb_handle = source._cb_handle;
    _timer = source._timer;
    return *this;
  }
  virtual bool load_parameters() = 0;
  virtual std::string get_yaml_file() = 0;
  virtual std::string get_path_to_config_files() = 0;

  void cb_change_param(const rclcpp::Parameter & p);
  void set_id_changed_cb(std::function<void(const std::string&)> callback);
  std::string get_current_id() { return _id; }

protected:
  rclcpp::Node * get_parent() { return _parentNode; }

private:
  rclcpp::Node * _parentNode;
  std::string _id{""};
  rclcpp::Parameter _id_temp;
  std::string _triggerParam{""};
  bool _id_set_correctly{false};
  std::shared_ptr<rclcpp::ParameterEventHandler> _param_subscriber;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> _cb_handle;
  rclcpp::TimerBase::SharedPtr _timer;
  std::function<void(const std::string&)> _cb_id_changed{nullptr};

  void check_loop();
};
#endif  // TOD_CORE__PARAM_SET__PARAMETERHANDLER_HPP_
