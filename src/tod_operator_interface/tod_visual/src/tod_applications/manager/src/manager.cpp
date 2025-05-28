/**
 * @file operator_manager.cpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM
 **/

#include "manager.hpp"

namespace tod_gl {

Manager::Manager(int argc, char** argv, const std::string& name)
    : Application(argc, argv, name) {
    RCLCPP_INFO(get_logger(), "Manager initialized.");
            push_overlay(new OperatorManagerDockingLayer(_ros,ImGuiDir_None));
            push_overlay(new OperatorStateLayer(_ros,ImGuiDir_Left));
            push_overlay(new VehicleStateLayer(_ros,ImGuiDir_Up));
            push_overlay(new VehicleInterfacesLayer(_ros,ImGuiDir_Down));
    }


} 