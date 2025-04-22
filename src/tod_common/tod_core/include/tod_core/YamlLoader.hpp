// Copyright 2021 Hoffmann
#ifndef TOD_CORE__YAMLLOADER_HPP_
#define TOD_CORE__YAMLLOADER_HPP_

#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <optional>


#include "yaml-cpp/yaml.h"
class YamlLoader
{
private:
  YAML::Node _node;
  template <typename... Args>
  static bool node_has_node(const YAML::Node & node, Args &&... args)
  {
    YAML::Node node_new = node_get_node(node, std::forward<Args>(args)...);
    return node_new ? true : false;
  }
  // recursive get_node
  template <typename First, typename... Args>
  static YAML::Node node_get_node(const YAML::Node & node, First && first, Args &&... args)
  {
    YAML::Node node_new = node_get_node(node, std::forward<First>(first));
    if (!node_new) {
      return node_new;
    }  // return zombienode if not exists
    return node_get_node(node_new, std::forward<Args>(args)...);
  }
  template <typename Last>
  static YAML::Node node_get_node(const YAML::Node & node, Last && last)
  {
    return node[std::string(std::forward<Last>(last))];
  }

public:
  template <typename T>
  bool load_from_path(T && path)
  {
    try {
      _node = YAML::LoadFile(std::forward<T>(path));
      return true;
    } catch (...) {
      std::cout << ": Vehicle Specific Parameters could not be loaded."
                << "Check the following possible Causes: \n"
                << "- vehicleID was not set correctly \n"
                << "- yaml file does not follow conventions \n"
                << "- <vehicleID>/vehicle-params.yaml does not exit \n"
                << "- for file " + path + "\n"
                << std::endl;
      return false;
    }
  }
  template <typename T, typename... Args>
  T get_param(Args &&... args)
  {
    YAML::Node node = node_get_node(_node, std::forward<Args>(args)...);
    if (node) {
      return node.as<T>();
    } else {
      std::stringstream ss;
      ((ss << "/" << args), ...);
      std::cout << ": Could not find param " << ss.str() << std::endl;
      return T();
    }
  }
  template <typename T, typename... Args>
  T get_opt_param(Args &&... args)
  {
    YAML::Node node = node_get_node(_node, std::forward<Args>(args)...);
    if (node) {
      return node.as<T>();
    } else {
      std::stringstream ss;
      ((ss << "/" << args), ...);
      std::cout << ": Could not find param " << ss.str() << std::endl;
      return T();
    }
  }
  template <typename... Args>
  bool has_node(Args &&... args)
  {
    return node_has_node(_node, std::forward<Args>(args)...);
  }
  template<typename ValueType>
  std::optional<std::map<std::string, ValueType>> get_settings_for_key_value(
        const std::string& list_name,
        const std::string& key,
        const std::string& expected_value)
    {
        std::optional<std::map<std::string, ValueType>> result_settings;

        if(!_node[list_name])
        {
            std::cout << list_name << " node not found in YAML." << std::endl;
            return result_settings; // Return empty optional if the node doesn't exist
        }

        for(const auto& item : _node[list_name])
        {
            if(item[key] && item[key].as<std::string>() == expected_value)
            {
                std::map<std::string, ValueType> settings;
                for(const auto& setting : item)
                {
                    std::string setting_key = setting.first.as<std::string>();
                    if(setting_key != key)
                    {
                        try
                        {
                            settings[setting_key] = setting.second.as<ValueType>(); 
                        }
                        catch (const YAML::BadConversion& e)
                        {
                            std::cerr << "Warning: Skipping setting due to type conversion error for key " << setting_key << std::endl;
                        }
                    }
                }
                result_settings = settings;
                break; 
            }
        }
        return result_settings;
    }

};
#endif  // TOD_CORE__YAMLLOADER_HPP_
