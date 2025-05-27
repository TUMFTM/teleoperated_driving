/**
 * @file visual_io_layer.cpp
 * @brief All subscriptions of the ROS Node and the containers in @ref tod_gl_ros_interface are registered in this layer from the SubscriptionManager entity and can be accessed via Entt anywhere in the application
 * @copyright 2024 TUMFTM
**/

#include "visual_io_layer.hpp"

#include "tod_gl/ros_interface/subscribing_components/image_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/odometry_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/predicted_object_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/trajectory_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/point_cloud_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/joy_stick_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/network_metrics_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/path_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/path_control_points_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/primary_control_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/secondary_control_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/tod_status_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/automation_status_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/trajectory_guidance_state_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/primary_vehicle_state_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/secondary_vehicle_state_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/driving_lane_component.hpp"

#include "tod_gl/ros_interface/subscribing_components/trajectory_control_component.hpp"

#include "tod_trajectory_guidance_msgs/msg/trajectory.hpp"
#include "tod_trajectory_guidance_msgs/msg/path.hpp"



namespace tod_visual {
    
VisualIOLayer::VisualIOLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene)
    : tod_gl::SceneLayer(ros, scene, "VisualVisualIOLayer") {}

void VisualIOLayer::on_attach() {
    tod_gl::Entity SubscriptionManager = _active_scene->create_entity("SubscriptionManager");
    SubscriptionManager.add_component<tod_gl::OdometryComponent>(_ros);

    // TODO Refactor image Component generation based on number of cams when config is there 
    SubscriptionManager.add_component<tod_gl::ImageComponentFrontCenter>(_ros);
    SubscriptionManager.add_component<tod_gl::ImageComponentFrontRight>(_ros);
    SubscriptionManager.add_component<tod_gl::ImageComponentFrontLeft>(_ros);
    SubscriptionManager.add_component<tod_gl::ImageComponentRearCenter>(_ros);
    SubscriptionManager.add_component<tod_gl::ImageComponentRearRight>(_ros);
    SubscriptionManager.add_component<tod_gl::ImageComponentRearLeft>(_ros);

    SubscriptionManager.add_component<tod_gl::DrivingLaneComponentFrontLeft>(_ros);
    SubscriptionManager.add_component<tod_gl::DrivingLaneComponentFrontRight>(_ros);
    SubscriptionManager.add_component<tod_gl::DrivingLaneComponentRearLeft>(_ros);
    SubscriptionManager.add_component<tod_gl::DrivingLaneComponentRearRight>(_ros);

    SubscriptionManager.add_component<tod_gl::TrajectoryComponent>(_ros);
    SubscriptionManager.add_component<tod_gl::PathControlPointsComponent>(_ros);
    SubscriptionManager.add_component<tod_gl::PredictedObjectComponent>(_ros);
    SubscriptionManager.add_component<tod_gl::PointCloudComponent>(_ros);
    SubscriptionManager.add_component<tod_gl::NetworkMetricsComponent>(_ros);
    SubscriptionManager.add_component<tod_gl::PrimaryControlCommandComponent>(_ros);
    SubscriptionManager.add_component<tod_gl::SecondaryControlCommandComponent>(_ros);
    SubscriptionManager.add_component<tod_gl::TodStatusComponent>(_ros);
    SubscriptionManager.add_component<tod_gl::AutomationStatusComponent>(_ros);
    SubscriptionManager.add_component<tod_gl::PrimaryVehicleStateComponent>(_ros);
    SubscriptionManager.add_component<tod_gl::SecondaryVehicleStateComponent>(_ros);
    SubscriptionManager.add_component<tod_gl::JoyStickComponent>(_ros);

    //Trajectory Guidance
    SubscriptionManager.add_component<tod_gl::TrajectoryGuidanceStateComponent>(_ros);
    SubscriptionManager.add_component<tod_gl::TrajectoryControlComponent>(_ros);
    SubscriptionManager.add_component<tod_gl::PathComponent<tod_trajectory_guidance_msgs::msg::Path>>(_ros, "input/trajectory_guidance/path_visualization");
    SubscriptionManager.add_component<tod_gl::PathComponent<tod_trajectory_guidance_msgs::msg::Trajectory>>(_ros, "input/trajectory_guidance/validation_trajectory");
}

}  // namespace tod_visual