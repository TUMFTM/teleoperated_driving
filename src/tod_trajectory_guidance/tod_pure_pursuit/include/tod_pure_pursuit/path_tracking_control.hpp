
/**
 * @file path_tracking_control.hpp
 * @brief Management Node for the \ref PurePursuit Controller for Trajectory Guidance
 * @details PathTrackingControl implements manages the interaction with the Controller i.e. making sure that the
 * coodinate system is correct and publishes the control cmds etc.
 * @copyright 2024 TUMFTM
 * @ingroup tod_trajectory_guidance
 */

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tod_core/param_set/VehicleParameters.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tod_status_msgs/msg/status.hpp"
#include "tod_trajectory_guidance_msgs/msg/pp_log.hpp"
#include "tod_trajectory_guidance_msgs/msg/trajectory.hpp"
#include "tod_trajectory_guidance_msgs/msg/trajectory_point.hpp"
#include "tod_vehicle_msgs/msg/primary_control_cmd.hpp"

#include "tod_pure_pursuit/pure_pursuit.hpp"
#include "tod_pure_pursuit/vehicle_model.hpp"

namespace tod_pure_pursuit {
/*
 * @ingroup tod_trajectory_guidance
 */

/*
 * @brief Manager Node for the \ref PurePursuit Controller for translating the incoming trajectory to control commands
 */
class PathTrackingControl : public rclcpp::Node {
  public:
    PathTrackingControl();

  private:
    // timer
    rclcpp::TimerBase::SharedPtr _timer;

    // subscriber
    rclcpp::Subscription<tod_trajectory_guidance_msgs::msg::Trajectory>::SharedPtr _subTrajectory;
    rclcpp::Subscription<tod_trajectory_guidance_msgs::msg::Trajectory>::SharedPtr _subValidationTrajectory;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subOdometry;
    rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr _subStatus;

    std::shared_ptr<tf2_ros::TransformListener> _transformListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> _tfBuffer;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _staticTransformBroadcaster;

    // callbacks
    void callback_trajectory(const tod_trajectory_guidance_msgs::msg::Trajectory &trajectory);
    void callback_odometry(const nav_msgs::msg::Odometry &odometry);
    void callback_status_msg(const tod_status_msgs::msg::Status &msg);
    void loop();

    // publisher
    rclcpp::Publisher<tod_trajectory_guidance_msgs::msg::PpLog>::SharedPtr _pubLogging;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _poseArrayPub;
    rclcpp::Publisher<tod_vehicle_msgs::msg::PrimaryControlCmd>::SharedPtr _pubControlCmd;

    // feedback
    geometry_msgs::msg::TransformStamped get_transform(const std::string &srcFrame, const std::string &trgFrame);
    void publish_logging_output();

    // variables
    int _previousTodStatus{0};
    double _actualSpeed{0};
    uint8_t _desiredControlMode{tod_status_msgs::msg::Status::CONTROL_MODE_NONE};
    uint8_t _actualControlMode{tod_status_msgs::msg::Status::CONTROL_MODE_NONE};

    tod_vehicle_msgs::msg::PrimaryControlCmd _primaryControlCommand;
    tod_trajectory_guidance_msgs::msg::Trajectory _trajectory;
    geometry_msgs::msg::PoseStamped _poseRearAxle;
    nav_msgs::msg::Odometry _odometry;

    std::unique_ptr<PurePursuit> _purePursuit;
    std::shared_ptr<tod_core::param_set::Vehicle> _vehParams;
};
}  // namespace tod_pure_pursuit