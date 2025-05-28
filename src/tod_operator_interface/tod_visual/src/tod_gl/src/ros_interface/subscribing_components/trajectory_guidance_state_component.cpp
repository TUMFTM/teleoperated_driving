/**
 * @file trajectory_guidance_state_component.cpp
 * @brief Implements the TrajectoryGuidanceStateComponent class.
 * @copyright 2024 TUMFTM
 */

 #include "tod_gl/ros_interface/subscribing_components/trajectory_guidance_state_component.hpp"

 namespace tod_gl {
 
 TrajectoryGuidanceStateComponent::TrajectoryGuidanceStateComponent(std::shared_ptr<rclcpp::Node> sub_node)
     : SubscribingComponent(sub_node, "input/trajectory_guidance/trajectory_guidance_state"),
       current_state_(0),
       last_event_(0),
       target_velocity_(0.f) {}
 
 void TrajectoryGuidanceStateComponent::cb_message(
     const tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::SharedPtr msg) {
     current_state_ = static_cast<uint8_t>(msg->current_state);
     last_event_ = static_cast<uint8_t>(msg->last_event);
     target_velocity_ = msg->target_velocity;
 }
 
 std::string TrajectoryGuidanceStateComponent::state_to_string(uint8_t current_state) const {
     switch (current_state) {
         case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::WAITING_FOR_TRAJECTORY:
             return "WAITING FOR TRAJECTORY";
         case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTING_TRAJECTORY:
             return "EXECUTING TRAJECTORY";
         case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTING_STOP_TRAJECTORY:
             return "EXECUTING STOP TRAJECTORY";
         case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VALIDATING_TRAJECTORY:
             return "VALIDATING TRAJECTORY";
         default:
             return "UNKNOWN_TRAJECTORY_GUIDANCE_STATE";
     }
 }
 
 std::string TrajectoryGuidanceStateComponent::event_to_string(uint8_t last_event) const {
     switch (last_event) {
         case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EVENT_NONE:
             return "NO EVENT OCCURRED";
         case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::TRAJECTORY_RECEIVED:
             return "TRAJECTORY RECEIVED";
         case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTION_FINISHED:
             return "EXECUTION FINISHED";
         case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::RESET_TRIGGERED:
             return "RESET TRIGGERED";
         case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::START_TRAJECTORY:
             return "START TRAJECTORY";
         case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::WATCHDOG_TRIGGERED:
             return "WATCHDOG TRIGGERED";
         case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::EXECUTION_CANCELED:
             return "EXECUTION CANCELED";
         case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VELOCITY_UPDATE_RECEIVED:
             return "SPEED UPDATE RECEIVED";
         case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VELOCITY_UPDATE_REJECTED:
             return "SPEED UPDATE REJECTED";
         case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::TRAJECTORY_REJECTED:
             return "TRAJECTORY REJECTED";
         case tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::VALIDATED_TRAJECTORY_RECIEVED:
             return "VALIDATED TRAJECTORY RECEIVED";
         default:
             return "UNKNOWN_TRAJECTORY_GUIDANCE_EVENT";
     }
 }
 
 }  // namespace tod_gl
 