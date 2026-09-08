/**
 * @file state_layer.cpp
 * @brief State Layer that registers all the layers and entities and defines in which control concept they are supposed to be rendered
 * @copyright 2024 TUMFTM
**/

#include "state_layer.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace tod_visual {
    
StateLayer::StateLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene)
    : tod_gl::SceneLayer(ros, scene, "StateLayer") {
        std::string vehicle_config_path = ros->get_config_path() + "/vehicle_config/";
        _cam_params = std::make_unique<tod_core::param_set::Camera>(_ros.get(), vehicle_config_path);
        _cam_params->load_parameters();
    }

void StateLayer::on_attach() {
    auto& stateManager = tod_gl::StateManager::get_instance();
    stateManager.register_entity("TrajectoryRenderer", {tod_status_msgs::msg::Status::CONTROL_MODE_NONE,tod_status_msgs::msg::Status::CONTROL_MODE_PATH_GUIDANCE});
    stateManager.register_entity("PathRenderer", {tod_status_msgs::msg::Status::CONTROL_MODE_NONE,tod_status_msgs::msg::Status::CONTROL_MODE_PATH_GUIDANCE});
    stateManager.register_entity("ValidationPathRenderer", {tod_status_msgs::msg::Status::CONTROL_MODE_NONE,tod_status_msgs::msg::Status::CONTROL_MODE_PATH_GUIDANCE});

    stateManager.register_entity("PathControlPointsRenderer", {tod_status_msgs::msg::Status::CONTROL_MODE_NONE,tod_status_msgs::msg::Status::CONTROL_MODE_PATH_GUIDANCE});
    stateManager.register_entity("PointCloudRenderer", {tod_status_msgs::msg::Status::CONTROL_MODE_NONE,tod_status_msgs::msg::Status::CONTROL_MODE_DIRECT,tod_status_msgs::msg::Status::CONTROL_MODE_PATH_GUIDANCE});
    stateManager.register_entity("ObjectRenderer", {tod_status_msgs::msg::Status::CONTROL_MODE_NONE,tod_status_msgs::msg::Status::CONTROL_MODE_PATH_GUIDANCE});
    stateManager.register_entity("Lanelet", {tod_status_msgs::msg::Status::CONTROL_MODE_NONE,tod_status_msgs::msg::Status::CONTROL_MODE_PATH_GUIDANCE});

    for (const auto& cam : _cam_params->get_sensors()) {
        std::string videoName = cam.name;
        std::cout << " Registered " << videoName << std::endl;
        videoName.erase(videoName.begin());
        stateManager.register_entity(videoName, {tod_status_msgs::msg::Status::CONTROL_MODE_NONE,tod_status_msgs::msg::Status::CONTROL_MODE_DIRECT});
    }

    stateManager.register_layer("Left", {tod_status_msgs::msg::Status::CONTROL_MODE_NONE,tod_status_msgs::msg::Status::CONTROL_MODE_DIRECT,tod_status_msgs::msg::Status::CONTROL_MODE_SHARED,tod_status_msgs::msg::Status::CONTROL_MODE_PATH_GUIDANCE});
    stateManager.register_layer("Center", {tod_status_msgs::msg::Status::CONTROL_MODE_NONE,tod_status_msgs::msg::Status::CONTROL_MODE_DIRECT,tod_status_msgs::msg::Status::CONTROL_MODE_SHARED,tod_status_msgs::msg::Status::CONTROL_MODE_PATH_GUIDANCE});
    stateManager.register_layer("Right", {tod_status_msgs::msg::Status::CONTROL_MODE_NONE,tod_status_msgs::msg::Status::CONTROL_MODE_DIRECT,tod_status_msgs::msg::Status::CONTROL_MODE_SHARED,tod_status_msgs::msg::Status::CONTROL_MODE_PATH_GUIDANCE});
    stateManager.register_layer("TrajectoryGuidanceStateLayer", {tod_status_msgs::msg::Status::CONTROL_MODE_NONE,tod_status_msgs::msg::Status::CONTROL_MODE_PATH_GUIDANCE});

    // Load toggle settings (can be set in the params and be adjusted during runtime by using the settings layer)
    std::unordered_map<std::string, bool> toggle_settings;
    _ros->get_param<bool>("enable_driving_lane", toggle_settings["enable_driving_lane"]);
    _ros->get_param<bool>("enable_lanelet_map", toggle_settings["enable_lanelet_map"]);
    _ros->get_param<bool>("enable_object_list", toggle_settings["enable_object_list"]);
    _ros->get_param<bool>("enable_point_cloud", toggle_settings["enable_point_cloud"]);
    _ros->get_param<bool>("enable_trajectory", toggle_settings["enable_trajectory"]);
    
    for (const auto& [key, value] : toggle_settings) {
        stateManager.set_toggle_setting(key, value); 
    }
}

}  // namespace tod_visual
