/**
 * @file trajectory_guidance_state_component.hpp
 * @brief Manages the subscription and data of the state machine information received during the trajectory guidance control concept.
 * @copyright 2024 TUMFTM
 **/

 #pragma once

 #include <rclcpp/rclcpp.hpp>
 #include "tod_gl/ros_interface/subscribing_component_base.hpp"
 #include "tod_trajectory_guidance_msgs/msg/trajectory_guidance_state.hpp"
 
 namespace tod_gl {
 
 /**
  * @class TrajectoryGuidanceStateComponent
  * @brief Handles the subscription and state information for trajectory guidance control.
  */
 class TrajectoryGuidanceStateComponent 
     : public SubscribingComponent<tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState> {
 public:
     /**
      * @brief Constructor initializing the component with a ROS2 node.
      * @param sub_node Shared pointer to the ROS2 node.
      */
     explicit TrajectoryGuidanceStateComponent(std::shared_ptr<rclcpp::Node> sub_node);
 
     /**
      * @brief Converts the current state into a human-readable string.
      * @param current_state The current trajectory guidance state.
      * @return Corresponding string representation.
      */
     std::string state_to_string(uint8_t current_state) const;
 
     /**
      * @brief Converts the last event into a human-readable string.
      * @param last_event The last event in trajectory guidance.
      * @return Corresponding string representation.
      */
     std::string event_to_string(uint8_t last_event) const;
 
     /// Current state of the trajectory guidance
     uint8_t current_state_;
 
     /// Last event that occurred in trajectory guidance
     uint8_t last_event_;
 
     /// Target velocity for the trajectory
     float target_velocity_;
 
 private:
     /**
      * @brief Callback function to process incoming messages.
      * @param msg Shared pointer to the received message.
      */
     void cb_message(const tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::SharedPtr msg) override;
 };
 
 }  // namespace tod_gl
 