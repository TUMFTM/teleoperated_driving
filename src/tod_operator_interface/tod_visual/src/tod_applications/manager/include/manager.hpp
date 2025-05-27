/**
 * @file operator_manager.hpp
 * @brief Defines the main OperatorManager class for the operator interface application.
 *
 * This file contains the declaration of the OperatorManager class, which serves as the 
 * main entry point for managing the operator interface. The class is responsible for 
 * initializing and managing various layers of the application.
 * 
 * @copyright 2024 TUMFTM
 */
#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "tod_gl/core/application.hpp"
#include "tod_gl/core/window.hpp"
#include "tod_gl/layers/io_layer.hpp"

#include "tod_gl/layers/imgui_layer.hpp"
#include "tod_gl/layers/docking_layer.hpp"
#include "imgui/imgui.h"

#include "operator_state_layer.hpp"
#include "vehicle_state_layer.hpp"
#include "vehicle_interfaces_layer.hpp"
#include "operator_manager_docking_layer.hpp"



namespace tod_gl {

/**
 * @class OperatorManager
 * @brief Main application manager for operator interface.
 *
 * The OperatorManager class initializes and manages various layers of the operator interface application.
 * It inherits from the Application base class and sets up the required layers for the operator interface.
 */
 
class Manager : public Application {
  public:

    Manager(int argc, char** argv, const std::string& name);
    ~Manager() = default;


    static rclcpp::Logger get_logger() {
      static auto logger = rclcpp::get_logger("Manager");
      return logger;
    }

  private:
      ImGuiLayer* _imGui_layer;
};

}  // namespace tod_gl
