// Copyright 2021 Schimpe
#ifndef TOD_CORE__PARAM_SET__LIDARPARAMETERS_HPP_
#define TOD_CORE__PARAM_SET__LIDARPARAMETERS_HPP_
#pragma once

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tod_core/YamlLoader.hpp"
#include "tod_core/param_set/ParameterHandler.hpp"

namespace tod_core::param_set {

class Lidar : public ParameterHandler
{
  public:
    struct LidarSensor
    {
      std::string name{""};
      explicit LidarSensor(std::string  myName) : name{std::move(myName)} {}
    };

    Lidar(rclcpp::Node * node_ptr, const std::string& config_path);
    bool load_parameters() override;
    std::string get_lidar_topics_namespace() const { return _lidar_topics_namespace; }
    std::string get_pointcloud_name() const { return _pointcloud_name; }
    int get_number_of_sensors() const { return _sensors.size(); }
    const std::vector<LidarSensor> & get_sensors() const { return _sensors; }
    std::string get_yaml_file() override { return "sensors-lidar.yaml"; }
    std::string get_path_to_config_files() override { return _desiredPath; }

  private:
    std::string _desiredPath{""};
    std::string _lidar_topics_namespace{""};
    std::string _pointcloud_name{""};
    std::vector<LidarSensor> _sensors;
};

} // namespace tod_core::param_set

#endif  // TOD_CORE__PARAM_SET__LIDARPARAMETERS_HPP_