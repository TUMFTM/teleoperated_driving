/**
 * @file visual_node.cpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM
**/

#include "drive_info_layer.hpp"
#include "scene_layer.hpp"
#include "settings_layer.hpp"
//#include "speed_layer.hpp"
#include "state_layer.hpp"
//#include "switch_camera_layer.hpp"
//#include "traffic_sign_layer.hpp"
#include "trajectory_guidance_state_layer.hpp"
#include "ui_layer.hpp"
#include "video_layer.hpp"
#include "view_port_layer.hpp"
#include "visual_io_layer.hpp"

#include "tod_gl/core/scene_application.hpp"
#include "tod_gl/layers/debug_layer.hpp"

#include "tod_gl/ros_interface/subscribing_components/image_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/joy_stick_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/odometry_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/network_metrics_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/primary_control_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/secondary_control_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/tod_status_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/trajectory_guidance_state_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/primary_vehicle_state_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/secondary_vehicle_state_component.hpp"

#include "rclcpp/rclcpp.hpp"

namespace tod_visual {

class VisualApplicationNode : public tod_gl::SceneApplication {
  public:
    VisualApplicationNode(int argc, char** argv, const std::string& name) : tod_gl::SceneApplication(argc, argv, name) {
        const tod_gl::Window& window = get_window();
        push_layer(new VisualIOLayer(_ros, _active_scene));
        push_layer(new StateLayer(_ros, _active_scene));
        push_layer(new VisualLayer(_ros, _active_scene));

        push_overlay(new tod_gl::DockingSceneLayer(_ros, _active_scene, ImGuiDir_None));
        auto* view_port_layer = new ViewPortLayer(_ros, _active_scene, ImGuiDir_None);
        push_overlay(view_port_layer);
        push_overlay(new DriveInfoLayer<tod_gl::PrimaryVehicleStateComponent, 
                                       tod_gl::SecondaryVehicleStateComponent,
                                       tod_gl::TodStatusComponent, 
                                       tod_gl::AutomationStatusComponent,
                                       tod_gl::NetworkMetricsComponent, 
                                       tod_gl::JoyStickComponent, 
                                       tod_gl::PrimaryControlCommandComponent,
                                       tod_gl::SecondaryControlCommandComponent>(_ros, _active_scene, view_port_layer));

        push_overlay(new VideoLayer<tod_gl::ImageComponentFrontLeft>(_ros, _active_scene, ImGuiDir_Up, "Left"));
        push_overlay(new VideoLayer<tod_gl::ImageComponentFrontCenter>(_ros, _active_scene, ImGuiDir_Up, "Center"));
        push_overlay(new VideoLayer<tod_gl::ImageComponentFrontRight>(_ros, _active_scene, ImGuiDir_Up, "Right"));

        push_overlay(new TrajectoryGuidanceStateLayer<tod_gl::TrajectoryGuidanceStateComponent>(_ros, _active_scene, view_port_layer));
        // push_overlay(new SwitchCameraLayer<tod_gl::TrajectoryGuidanceStateComponent>(_ros, _active_scene, view_port_layer));
        // push_overlay(new TrafficSignLayer<tod_gl::OdometryComponent>(_ros, _active_scene, view_port_layer)); 
        
        push_overlay(new SettingsLayer(_ros, _active_scene, ImGuiDir_Up));
        // push_overlay(new tod_gl::DebugLayer(_ros, _active_scene)); push_overlay(new UILayer(_ros, _active_scene, view_port_layer));
    }
};

}  // namespace tod_visual

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto visual = std::make_unique<tod_visual::VisualApplicationNode>(argc, argv, "Visual");
    visual->initialize();
    visual->run();
    return 0;
}
