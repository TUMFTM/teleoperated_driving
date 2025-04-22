// Copyright 2024 TUMFTM
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_array.hpp"
#include "tod_trajectory_guidance_msgs/msg/pp_log.hpp"
#include "tod_vehicle_msgs/msg/primary_control_cmd.hpp"
#include "tod_vehicle_msgs/msg/secondary_vehicle_state.hpp"

#include <tod_trajectory_guidance_msgs/msg/trajectory_guidance_control.hpp>
#include "tod_safety_msgs/msg/safety_state.hpp"
#include "tod_status_msgs/msg/status.hpp"
#include "tod_trajectory_guidance/vehicle/trajectory_guidance.hpp"
#include "tod_trajectory_guidance_msgs/msg/path.hpp"
#include "tod_trajectory_guidance_msgs/msg/trajectory.hpp"
#include "tod_trajectory_guidance_msgs/msg/trajectory_guidance_control.hpp"
#include "tod_trajectory_guidance_msgs/msg/trajectory_guidance_state.hpp"

namespace tod_trajectory_guidance {

class TrajectoryGuidanceNode : public rclcpp::Node {
  public:
    TrajectoryGuidanceNode();
    std::string current_state{"WAITING_FOR_TRAJECTORY"};
    std::string last_event{"EVENT_NONE"};
  private:
    struct StateTransition {
        std::string state;
        std::string event;
        bool _state_changed;
    };
    std::vector<StateTransition> _state_history;

    void log_state_event(const std::string &state, const std::string &event);
    std::string state_to_string(uint8_t state);
    std::string event_to_string(uint8_t event);

    // Subscriptions
    rclcpp::Subscription<tod_trajectory_guidance_msgs::msg::Path>::SharedPtr subPath_;
    rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr subTodStatus_;
    rclcpp::Subscription<tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceControl>::SharedPtr subTrajectoryControl_;
    rclcpp::Subscription<tod_trajectory_guidance_msgs::msg::PpLog>::SharedPtr _sub_control_state;
    rclcpp::Subscription<tod_safety_msgs::msg::SafetyState>::SharedPtr _sub_watchdog;
    // Publisher
    rclcpp::Publisher<tod_trajectory_guidance_msgs::msg::Trajectory>::SharedPtr
        _pub_active_trajectory;  // local trajectory in
                                 // "tod_trajectory_guidance_msgs/msg/trajectory.hpp"
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _pub_pose_array;  // for visualisation in rviz
    rclcpp::Publisher<tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState>::SharedPtr
        _pub_trajectory_guidance_state;

    rclcpp::Publisher<tod_trajectory_guidance_msgs::msg::Trajectory>::SharedPtr _pub_trajectory_validation;

    // Callbacks
    void callback_path(const tod_trajectory_guidance_msgs::msg::Path &msg);
    // void callback_validated_trajectory(const tod_trajectory_guidance_msgs::msg::Trajectory::SharedPtr msg);

    void callback_tod_state(const tod_status_msgs::msg::Status &msg);
    void callback_trajectory_control_cmd(const tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceControl &msg);
    void callback_control_state(const tod_trajectory_guidance_msgs::msg::PpLog &msg);
    void callback_reset_state(const tod_status_msgs::msg::Status &msg);
    void callback_watchdog_state(const tod_safety_msgs::msg::SafetyState &msg);
    void publish_trajectory_guidance_state();

    // Trajectory Guidance
    void publish_trajectory();

    TrajectoryGuidance _tg;
    tod_status_msgs::msg::Status _last_tod_status;
    tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState::SharedPtr _state_msg;
    rclcpp::TimerBase::SharedPtr _timer;
    std::string _trajectory_frame_id{"map"};
    int _next_waypoint{0};
    bool _drive_status{false};
    bool _debug{true};
};
}  // namespace tod_trajectory_guidance